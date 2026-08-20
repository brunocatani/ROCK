#include "physics-interaction/weapon/collision/WeaponCollision.h"

/*
 * OMOD coverage audit, the build-time OMOD evidence dump, and the OMOD self-heal
 * that re-attaches weapon-mod geometry the engine dropped.
 *
 * This file is deliberately the ONLY place in WeaponCollision that touches raw
 * Fallout4VR offsets and native entry points:
 *   - NativeConnectPointParentLayout and its static_asserts, which pin the layout
 *     this file walks by hand;
 *   - the vtbl+0x508 / +0x458 biped-slot probe;
 *   - tryAttach3DRecurse at 0x2D9140;
 *   - applyEquippedOmodModelCustomization, which validates a byte prefix before it
 *     calls into the engine.
 * Concentrating them here is the point: every unverified assumption about the VR
 * binary is auditable in one file, and every one of them fails closed - a layout
 * that does not match must degrade into a logged skip, never a fault. Do not move
 * a native read out of this file, and do not add one anywhere else.
 *
 * The audit reads cached identity keys and activeWeaponBodies(); it owns seven
 * dedicated members of its own and mutates no body state.
 */

#include "physics-interaction/weapon/collision/WeaponCollisionInternal.h"
#include "physics-interaction/weapon/collision/WeaponSceneGraphWalk.h"

#include "RockConfig.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/native/havok/HavokOffsets.h"
#include "physics-interaction/native/hooks/NativeMemory.h"
#include "physics-interaction/native/NativeNiNodeFactory.h"
#include "physics-interaction/weapon/collision/WeaponOmodAuditPolicy.h"
#include "physics-interaction/weapon/parts/WeaponPartRecordIdentityPolicy.h"
#include "physics-interaction/weapon/WeaponSemantics.h"

#include "RE/Bethesda/BGSMod.h"
#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/NetImmerse/NiNode.h"

#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#define ROCK_OMOD_DIAGNOSTIC_INFO(...)                                  \
    do {                                                                \
        if (g_rockConfig.rockDebugWeaponOmodCoverageAudit) {             \
            ROCK_LOG_INFO(Weapon, __VA_ARGS__);                          \
        }                                                               \
    } while (false)

namespace rock
{
    // Same unqualified access to the shared helpers as every other WeaponCollision
    // translation unit; see WeaponCollisionInternal.h.
    using namespace weapon_collision_detail;

    namespace
    {
        constexpr int kMandatoryOmodSelfHealIntervalFrames = 450;

        void dumpOmodWeaponTreeRecursive(
            RE::NiAVObject* node,
            int depth,
            std::size_t& visited,
            const std::unordered_map<std::uintptr_t, std::string>& evidenceMarkers)
        {
            constexpr int kMaxDumpDepth = 16;
            constexpr std::size_t kMaxDumpNodes = 512;
            if (!node || depth > kMaxDumpDepth || visited >= kMaxDumpNodes) {
                return;
            }
            ++visited;

            const auto markerIt = evidenceMarkers.find(reinterpret_cast<std::uintptr_t>(node));
            auto* niNode = node->IsNode();
            ROCK_LOG_INFO(Weapon,
                "OMOD-DUMP tree {}{} addr={:x} children={} localT=({:.2f},{:.2f},{:.2f}){}",
                std::string(static_cast<std::size_t>(depth) * 2, ' '),
                safeNodeName(node),
                reinterpret_cast<std::uintptr_t>(node),
                niNode ? niNode->children.size() : 0,
                node->local.translate.x,
                node->local.translate.y,
                node->local.translate.z,
                markerIt != evidenceMarkers.end() ? markerIt->second : "");

            if (!niNode) {
                return;
            }
            const auto& children = niNode->children;
            for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                if (auto* child = children[i].get()) {
                    dumpOmodWeaponTreeRecursive(child, depth + 1, visited, evidenceMarkers);
                }
            }
        }
    }

    /*
     * One-shot research dump (gated on bDebugWeaponOmodDump) that pairs the
     * equipped instance's installed-OMOD records with the assembled scene tree
     * and the generated evidence bindings. This exists to establish the
     * record-to-node anchoring mechanism for record-authored part identity;
     * every field read below that is not already exercised elsewhere in ROCK
     * (attachPoint index, OMOD model path) is deliberately printed raw so a
     * VR layout mismatch shows up as garbage in the log instead of a crash.
     */
    void WeaponCollision::dumpEquippedWeaponOmodEvidence(const WeaponBodyBank& bank, RE::NiAVObject* packageDriveNode)
    {
        if (!g_rockConfig.rockDebugWeaponOmodDumpEnabled) {
            return;
        }
        if (_cachedWeaponBodySetKey == 0 || _cachedWeaponBodySetKey == _lastOmodDumpGenerationKey) {
            return;
        }
        _lastOmodDumpGenerationKey = _cachedWeaponBodySetKey;

        auto* player = f4vr::getPlayer();
        auto* equipData = f4vr::getEquippedWeaponItem();
        auto* weaponForm = equipData ? equipData->item.object : nullptr;
        auto* equippedInstanceData = equipData ? equipData->item.instanceData.get() : nullptr;
        ROCK_LOG_INFO(Weapon,
            "OMOD-DUMP begin generation={:016X} weapon={:08X} '{}'",
            _cachedWeaponBodySetKey,
            weaponForm ? weaponForm->formID : 0u,
            weaponForm ? RE::TESFullName::GetFullName(*weaponForm) : std::string_view{});

        const RE::BGSObjectInstanceExtra* objectInstanceExtra =
            weaponForm ? findEquippedWeaponObjectInstanceExtra(player, weaponForm, equippedInstanceData) : nullptr;
        if (objectInstanceExtra && objectInstanceExtra->values) {
            ROCK_LOG_INFO(Weapon, "OMOD-DUMP installed mods count={}", objectInstanceExtra->GetIndexData().size());
            visitEquippedOmodIndexData(objectInstanceExtra,
                [&](const auto& modIndex, auto* omod, const RE::BGSKeyword* attachPointKeyword) {
                if (!omod) {
                    ROCK_LOG_INFO(Weapon,
                        "OMOD-DUMP mod objectID={:08X} index={} rank={} disabled={} UNRESOLVED",
                        modIndex.objectID,
                        modIndex.index,
                        modIndex.rank,
                        modIndex.disabled);
                    return;
                }

                const std::uint16_t attachPointIndex = omod->attachPoint.keywordIndex;
                ROCK_LOG_INFO(Weapon,
                    "OMOD-DUMP mod formID={:08X} formType={:02X} name='{}' index={} rank={} disabled={} "
                    "attachPointIndex={} attachPoint={:08X} '{}' model='{}'",
                    omod->formID,
                    static_cast<std::uint32_t>(omod->formType.underlying()),
                    omod->fullName.c_str() ? omod->fullName.c_str() : "",
                    modIndex.index,
                    modIndex.rank,
                    modIndex.disabled,
                    attachPointIndex,
                    attachPointKeyword ? attachPointKeyword->formID : 0u,
                    attachPointKeyword && attachPointKeyword->formEditorID.c_str() ? attachPointKeyword->formEditorID.c_str() : "",
                    omod->model.c_str() ? omod->model.c_str() : "");
            });
        } else {
            ROCK_LOG_INFO(Weapon, "OMOD-DUMP no object instance extra available");
        }

        std::unordered_map<std::uintptr_t, std::string> evidenceMarkers;
        for (const auto& instance : bank) {
            if (!instance.body.isValid()) {
                continue;
            }
            ROCK_LOG_INFO(Weapon,
                "OMOD-DUMP evidence bodyId={} source='{}' sourceRoot='{}' sourceNode={:x} partKind={} reload={} support={} socket={} action={} points={}",
                instance.body.getBodyId().value,
                instance.sourceName,
                instance.sourceRootName,
                reinterpret_cast<std::uintptr_t>(instance.sourceNode),
                static_cast<int>(instance.semantic.partKind),
                static_cast<int>(instance.semantic.reloadRole),
                static_cast<int>(instance.semantic.supportGripRole),
                static_cast<int>(instance.semantic.socketRole),
                static_cast<int>(instance.semantic.actionRole),
                instance.generatedPointCount);
            if (instance.sourceNode) {
                auto& marker = evidenceMarkers[reinterpret_cast<std::uintptr_t>(instance.sourceNode)];
                marker += fmt::format(" <== bodyId={} '{}'", instance.body.getBodyId().value, instance.sourceName);
            }
        }

        std::size_t visited = 0;
        dumpOmodWeaponTreeRecursive(packageDriveNode, 0, visited, evidenceMarkers);
        ROCK_LOG_INFO(Weapon, "OMOD-DUMP end nodesLogged={}", visited);
    }

    namespace
    {
        constexpr std::size_t OMOD_AUDIT_MAX_MATCHES_PER_TOKEN = 8;
        constexpr std::size_t OMOD_AUDIT_MAX_CONNECT_POINT_MATCHES = 64;
        constexpr std::size_t OMOD_AUDIT_MAX_LOGGED_MATCHES_PER_OMOD = 3;

        struct OmodAuditNodeMatch
        {
            RE::NiAVObject* node{ nullptr };
            const char* rootLabel{ "" };
        };

        struct OmodAuditTokenSlot
        {
            std::string lowerToken;
            /*
             * Word-set fallback: mesh authors reorder basename words
             * ('AK74_HG_Lower.nif' vs node 'AK74_Lower_HG'), which made exact
             * substring matching report false NODE_NOT_FOUND. A node matches
             * when every basename word appears somewhere in its name. False
             * NODE_NOT_FOUND must stay rare because the self-heal uses that
             * verdict as its trigger.
             */
            std::vector<std::string> lowerWords;
            std::vector<OmodAuditNodeMatch> matches;
        };

        struct OmodAuditRecord
        {
            std::uint32_t formId{ 0 };
            std::uint32_t attachPointFormId{ 0 };
            std::uint16_t attachPointIndex{ 0 };
            std::uint32_t modIndex{ 0 };
            std::uint32_t rank{ 0 };
            bool disabled{ false };
            bool resolved{ false };
            std::string name;
            std::string modelPath;
        };

        /*
         * Search token = OMOD model NIF basename without extension, lowered.
         * Node names authored from the model file usually contain this token;
         * mesh-internal names may not, which is why the connect-point census
         * below exists as the structural fallback.
         */
        std::string makeOmodAuditModelToken(const char* modelPath)
        {
            if (!modelPath || modelPath[0] == '\0') {
                return {};
            }
            const char* base = modelPath;
            for (const char* cursor = modelPath; *cursor; ++cursor) {
                if (*cursor == '\\' || *cursor == '/') {
                    base = cursor + 1;
                }
            }
            std::string token(base);
            const auto dot = token.find_last_of('.');
            if (dot != std::string::npos) {
                token.resize(dot);
            }
            for (auto& c : token) {
                c = weapon_effect_geometry_policy::foldAscii(c);
            }
            return token;
        }

        // Allocation-free case-insensitive substring test against a pre-lowered token.
        bool omodAuditNameContainsToken(const char* name, const std::string& lowerToken)
        {
            if (!name || lowerToken.empty()) {
                return false;
            }
            const std::size_t tokenLength = lowerToken.size();
            for (const char* cursor = name; *cursor; ++cursor) {
                std::size_t i = 0;
                while (i < tokenLength) {
                    const char c = cursor[i];
                    if (c == '\0' || weapon_effect_geometry_policy::foldAscii(c) != lowerToken[i]) {
                        break;
                    }
                    ++i;
                }
                if (i == tokenLength) {
                    return true;
                }
            }
            return false;
        }

        bool omodAuditNameIsConnectPoint(const char* name)
        {
            return name && (name[0] == 'P' || name[0] == 'p') && name[1] == '-';
        }

        std::vector<std::string> makeOmodAuditTokenWords(const std::string& lowerToken)
        {
            std::vector<std::string> words;
            std::string current;
            for (const char c : lowerToken) {
                if (c == '_' || c == '-' || c == ' ') {
                    if (current.size() >= 2) {
                        words.push_back(current);
                    }
                    current.clear();
                } else {
                    current += c;
                }
            }
            if (current.size() >= 2) {
                words.push_back(current);
            }
            // A single word degenerates to the substring test; two or more
            // words are required for the reordered-words fallback to add
            // signal instead of noise.
            if (words.size() < 2) {
                words.clear();
            }
            return words;
        }

        bool omodAuditNameMatchesTokenSlot(const char* name, const OmodAuditTokenSlot& slot)
        {
            if (omodAuditNameContainsToken(name, slot.lowerToken)) {
                return true;
            }
            if (slot.lowerWords.empty()) {
                return false;
            }
            for (const auto& word : slot.lowerWords) {
                if (!omodAuditNameContainsToken(name, word)) {
                    return false;
                }
            }
            return true;
        }

        bool omodAuditMatchesContainNode(const std::vector<OmodAuditNodeMatch>& matches, const RE::NiAVObject* node)
        {
            for (const auto& match : matches) {
                if (match.node == node) {
                    return true;
                }
            }
            return false;
        }

        /*
         * One walk per root evaluates every OMOD token plus the P-* connect
         * point predicate, instead of one walk per (root, token) pair. Matches
         * deduplicate across roots by node address because the weapon subtree
         * is reachable from several of the audited roots.
         */
        void scanOmodAuditTreeRecursive(
            RE::NiAVObject* node,
            std::uint32_t depth,
            std::size_t& visited,
            std::size_t maxVisited,
            const char* rootLabel,
            std::vector<OmodAuditTokenSlot>& tokenSlots,
            std::vector<OmodAuditNodeMatch>& connectPointMatches)
        {
            if (!node || visited >= maxVisited || depth > WEAPON_ANIM_NODE_DUMP_MAX_DEPTH) {
                return;
            }
            ++visited;

            const char* name = node->name.c_str();
            if (name && name[0] != '\0') {
                if (omodAuditNameIsConnectPoint(name) &&
                    connectPointMatches.size() < OMOD_AUDIT_MAX_CONNECT_POINT_MATCHES &&
                    !omodAuditMatchesContainNode(connectPointMatches, node)) {
                    connectPointMatches.push_back(OmodAuditNodeMatch{ node, rootLabel });
                }
                for (auto& slot : tokenSlots) {
                    if (slot.lowerToken.empty() || slot.matches.size() >= OMOD_AUDIT_MAX_MATCHES_PER_TOKEN) {
                        continue;
                    }
                    if (!omodAuditNameMatchesTokenSlot(name, slot)) {
                        continue;
                    }
                    if (!omodAuditMatchesContainNode(slot.matches, node)) {
                        slot.matches.push_back(OmodAuditNodeMatch{ node, rootLabel });
                    }
                }
            }

            auto* niNode = node->IsNode();
            if (!niNode) {
                return;
            }
            const auto& children = niNode->children;
            for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                if (auto* child = children[i].get()) {
                    scanOmodAuditTreeRecursive(child, depth + 1, visited, maxVisited, rootLabel, tokenSlots, connectPointMatches);
                }
            }
        }

        // Path is rebuilt from the parent chain only for matched nodes, so the
        // scan itself stays allocation-free per visited node.
        std::string buildOmodAuditNodePath(const RE::NiAVObject* node)
        {
            std::array<const char*, WEAPON_ANIM_NODE_DUMP_MAX_DEPTH + 1> names{};
            std::size_t count = 0;
            for (const RE::NiAVObject* cursor = node; cursor && count < names.size(); cursor = cursor->parent) {
                const char* name = cursor->name.c_str();
                names[count++] = name && name[0] != '\0' ? name : "(unnamed)";
            }
            std::string path;
            for (std::size_t i = count; i > 0; --i) {
                if (!path.empty()) {
                    path += "/";
                }
                path += names[i - 1];
            }
            return path;
        }

        /*
         * Node view of the walk, for the post-workbench "weapon invisible"
         * investigation, which needs the offending ancestor named explicitly.
         * Starts at the PARENT: the question is who ABOVE this node hides it, so a
         * hidden node is never reported as its own offender.
         */
        const RE::NiAVObject* findOmodAuditHiddenAncestor(const RE::NiAVObject* node)
        {
            return walkWeaponVisibilityChain(
                node ? node->parent : nullptr,
                kOmodAuditVisibilityAncestorSteps).firstHidden;
        }

        std::size_t countOmodAuditEvidenceSourcesInSubtree(
            RE::NiAVObject* node,
            const std::unordered_set<std::uintptr_t>& evidenceSourceAddresses,
            std::size_t& visited)
        {
            if (!node || visited >= WEAPON_ANIM_NODE_DUMP_MAX_SUBTREE_NODES) {
                return 0;
            }
            ++visited;
            std::size_t count = evidenceSourceAddresses.count(reinterpret_cast<std::uintptr_t>(node)) != 0 ? 1 : 0;
            auto* niNode = node->IsNode();
            if (!niNode) {
                return count;
            }
            const auto& children = niNode->children;
            for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                if (auto* child = children[i].get()) {
                    count += countOmodAuditEvidenceSourcesInSubtree(child, evidenceSourceAddresses, visited);
                }
            }
            return count;
        }

        struct OmodPhysicalTemplateSignature
        {
            std::vector<std::string> meshNames;
            std::string durableAnchorName;
            std::uint32_t durableAnchorTriangles{ 0 };
        };

        /*
         * Physical-mesh fingerprint of an OMOD template: the set of visible mesh
         * names, plus the single largest shape kept as a durable anchor. A TriShape
         * is a leaf for this scan, whether it is counted or excluded as effect-only
         * geometry.
         */
        void collectOmodPhysicalTemplateSignatureRecursive(
            RE::NiAVObject* node,
            OmodPhysicalTemplateSignature& signature,
            std::size_t& visited)
        {
            BoundedTreeWalkState state{ .visited = visited };
            auto visitor = [&](RE::NiAVObject* current, int) {
                // Once the name list is full the rest of the tree cannot make the
                // fingerprint more distinctive, only more expensive.
                if (signature.meshNames.size() >= kOmodTemplateSignatureMaxMeshNames) {
                    return TreeWalkAction::Stop;
                }
                auto* triShape = current->IsTriShape();
                if (!triShape) {
                    return TreeWalkAction::Descend;
                }
                if (classifyGeneratedWeaponEffectGeometry(triShape) != weapon_effect_geometry_policy::ExclusionReason::None) {
                    return TreeWalkAction::SkipChildren;
                }
                const char* rawName = current->name.c_str();
                if (!rawName || rawName[0] == '\0') {
                    return TreeWalkAction::SkipChildren;
                }

                const auto duplicate = std::find_if(signature.meshNames.begin(), signature.meshNames.end(), [rawName](const std::string& existing) {
                    return _stricmp(existing.c_str(), rawName) == 0;
                });
                if (duplicate == signature.meshNames.end()) {
                    signature.meshNames.emplace_back(rawName);
                }

                // Largest shape wins the anchor: it is the part a self-heal can
                // recognize again after the engine reassembles the weapon.
                std::uint32_t triangleCount = 0;
                if (native_memory::tryReadField(triShape, VROffset::numTriangles, triangleCount) &&
                    triangleCount > signature.durableAnchorTriangles) {
                    signature.durableAnchorTriangles = triangleCount;
                    signature.durableAnchorName = rawName;
                }
                return TreeWalkAction::SkipChildren;
            };
            boundedTreeWalk(node, kTemplateScanMaxDepth, kTemplateScanMaxVisitedNodes, state, visitor);
            visited = state.visited;
        }

        OmodPhysicalTemplateSignature collectOmodPhysicalTemplateSignature(RE::NiNode* root)
        {
            OmodPhysicalTemplateSignature signature{};
            signature.meshNames.reserve(32);
            std::size_t visited = 0;
            collectOmodPhysicalTemplateSignatureRecursive(root, signature, visited);
            return signature;
        }

        [[nodiscard]] bool physicalTemplateSignatureCovers(
            const OmodPhysicalTemplateSignature& candidate,
            const OmodPhysicalTemplateSignature& required)
        {
            return std::all_of(required.meshNames.begin(), required.meshNames.end(), [&candidate](const std::string& name) {
                return std::any_of(candidate.meshNames.begin(), candidate.meshNames.end(), [&name](const std::string& candidateName) {
                    return _stricmp(candidateName.c_str(), name.c_str()) == 0;
                });
            });
        }

        // Does this template already carry native Havok collision? One hit answers
        // the question for the whole tree.
        bool templateContainsNativeCollisionObjectRecursive(
            RE::NiAVObject* object,
            std::size_t& visited)
        {
            BoundedTreeWalkState state{ .visited = visited };
            bool found = false;
            auto visitor = [&](RE::NiAVObject* current, int) {
                if (auto* collisionObject = current->collisionObject.get();
                    collisionObject && niObjectRttiChainContains(collisionObject, "bhkNPCollisionObject")) {
                    found = true;
                    return TreeWalkAction::Stop;
                }
                return TreeWalkAction::Descend;
            };
            boundedTreeWalk(object, kTemplateScanMaxDepth, kTemplateScanMaxVisitedNodes, state, visitor);
            visited = state.visited;
            return found;
        }

        RE::NiNode* resolveOmodPhysicalCoverageRoot(
            RE::NiNode* weaponRoot,
            const std::uint32_t attachPointFormId)
        {
            if (!weaponRoot) {
                return nullptr;
            }
            const std::string_view connectPoint =
                weapon_part_record_identity_policy::canonicalConnectPointForAttachPoint(attachPointFormId);
            if (connectPoint.empty()) {
                return weaponRoot;
            }

            const std::string connectPointName{ connectPoint };
            const auto matches = collectWeaponAnimNodeMatches(weaponRoot, connectPointName.c_str());
            for (const auto& match : matches) {
                if (auto* node = match.node ? match.node->IsNode() : nullptr) {
                    return node;
                }
            }
            return weaponRoot;
        }

        std::size_t countPresentOmodPhysicalSignatureNames(
            RE::NiAVObject* coverageRoot,
            const OmodPhysicalTemplateSignature& signature,
            const char*& outFirstMatchedName)
        {
            outFirstMatchedName = nullptr;
            std::size_t matched = 0;
            for (const auto& meshName : signature.meshNames) {
                if (!collectWeaponAnimNodeMatches(coverageRoot, meshName.c_str()).empty()) {
                    ++matched;
                    if (!outFirstMatchedName) {
                        outFirstMatchedName = meshName.c_str();
                    }
                }
            }
            return matched;
        }

        constexpr std::string_view kRockOmodEnrichmentPrefix = "ROCK-OMOD-Enrichment-";

        [[nodiscard]] bool tryParseRockOmodEnrichmentFormId(
            const std::string_view nodeName,
            std::uint32_t& outFormId) noexcept
        {
            outFormId = 0;
            constexpr std::size_t kFormIdHexDigits = 8;
            if (!nodeName.starts_with(kRockOmodEnrichmentPrefix) ||
                nodeName.size() != kRockOmodEnrichmentPrefix.size() + kFormIdHexDigits) {
                return false;
            }

            std::uint32_t formId = 0;
            for (const char c : nodeName.substr(kRockOmodEnrichmentPrefix.size())) {
                std::uint32_t digit = 0;
                if (c >= '0' && c <= '9') {
                    digit = static_cast<std::uint32_t>(c - '0');
                } else if (c >= 'A' && c <= 'F') {
                    digit = static_cast<std::uint32_t>(c - 'A' + 10);
                } else if (c >= 'a' && c <= 'f') {
                    digit = static_cast<std::uint32_t>(c - 'a' + 10);
                } else {
                    return false;
                }
                formId = (formId << 4) | digit;
            }

            outFormId = formId;
            return formId != 0;
        }

        // Collect ROCK's own enrichment containers by their encoded name. A
        // container is a leaf for this sweep: its contents are the OMOD geometry
        // ROCK inserted, never another container.
        void collectRockOmodEnrichmentContainersRecursive(
            RE::NiAVObject* node,
            std::vector<RE::NiNode*>& outContainers,
            std::size_t& visited)
        {
            BoundedTreeWalkState state{ .visited = visited };
            auto visitor = [&](RE::NiAVObject* current, int) {
                auto* niNode = current->IsNode();
                if (!niNode) {
                    return TreeWalkAction::SkipChildren;
                }
                std::uint32_t formId = 0;
                const char* rawName = niNode->name.c_str();
                if (rawName && tryParseRockOmodEnrichmentFormId(rawName, formId)) {
                    outContainers.push_back(niNode);
                    return TreeWalkAction::SkipChildren;
                }
                return TreeWalkAction::Descend;
            };
            boundedTreeWalk(node, kEnrichmentContainerScanMaxDepth, WEAPON_ANIM_NODE_DUMP_MAX_SUBTREE_NODES, state, visitor);
            visited = state.visited;
        }

        std::size_t removeStaleRockOmodEnrichmentContainers(
            RE::NiAVObject* weaponRoot,
            const std::unordered_set<std::uint32_t>& activeOmodFormIds)
        {
            std::vector<RE::NiNode*> containers;
            containers.reserve(8);
            std::size_t visited = 0;
            collectRockOmodEnrichmentContainersRecursive(weaponRoot, containers, visited);

            std::size_t removed = 0;
            for (auto* container : containers) {
                std::uint32_t formId = 0;
                const char* rawName = container ? container->name.c_str() : nullptr;
                if (!rawName || !tryParseRockOmodEnrichmentFormId(rawName, formId) ||
                    activeOmodFormIds.contains(formId)) {
                    continue;
                }

                auto* parent = container->parent ? container->parent->IsNode() : nullptr;
                if (!parent) {
                    continue;
                }
                RE::NiPointer<RE::NiAVObject> detached;
                parent->DetachChild(container, detached);
                if (!detached) {
                    continue;
                }
                f4vr::updateTransformsDown(parent, true);
                ++removed;
                ROCK_OMOD_DIAGNOSTIC_INFO(
                    "OMOD-HEAL removed stale owned enrichment '{}' for inactive omod={:08X}",
                    rawName,
                    formId);
            }
            return removed;
        }

        /*
         * Fallout4VR.exe 1.2.72 BSConnectPoint::Parents layout. The layout is
         * independently corroborated by the local F4SEVR 0.6.21
         * NiExtraData.h definition and the Ghidra-verified
         * BSConnectPoint::DoAttach/Parent::ConnectChild chain at
         * 0x141DF1200/0x141DEF8A0. These are read-only, frame-scoped views of
         * extra data owned by a loaded OMOD template.
         */
        struct NativeConnectPointParentLayout
        {
            std::uint64_t referenceState;
            RE::BSFixedString parentNodeName;
            RE::BSFixedString connectPointName;
            RE::NiQuaternion rotation;
            RE::NiPoint3 translation;
            float scale;
        };
        static_assert(offsetof(NativeConnectPointParentLayout, parentNodeName) == 0x08);
        static_assert(offsetof(NativeConnectPointParentLayout, connectPointName) == 0x10);
        static_assert(offsetof(NativeConnectPointParentLayout, rotation) == 0x18);
        static_assert(offsetof(NativeConnectPointParentLayout, translation) == 0x28);
        static_assert(sizeof(NativeConnectPointParentLayout) == 0x38);

        struct NativeConnectPointParentArrayLayout
        {
            NativeConnectPointParentLayout** entries;
            std::uint32_t capacity;
            std::uint32_t pad0C;
            std::uint32_t count;
            std::uint32_t pad14;
        };
        static_assert(sizeof(NativeConnectPointParentArrayLayout) == 0x18);

        struct AuthoredConnectPointParentMatch
        {
            RE::NiNode* metadataOwner{ nullptr };
            const NativeConnectPointParentLayout* parent{ nullptr };
        };

        /*
         * Find the authored BSConnectPoint::Parents record that names this connect
         * point. Only NiNodes carry the extra data, so a non-node is a leaf.
         *
         * Every read of the native array is guarded: the layout is a raw VR offset
         * walk (see NativeConnectPointParentLayout and its static_asserts), and a
         * modded or damaged tree must degrade into "not found", never into a fault.
         */
        bool findAuthoredConnectPointParentRecursive(
            RE::NiAVObject* object,
            const RE::BSFixedString& cpaKey,
            const char* targetConnectPointName,
            AuthoredConnectPointParentMatch& outMatch,
            std::size_t& visited)
        {
            if (!targetConnectPointName) {
                return false;
            }
            constexpr std::uint32_t kMaxParentRecords = 64;
            BoundedTreeWalkState state{ .visited = visited };
            bool found = false;
            auto visitor = [&](RE::NiAVObject* current, int) {
                auto* node = current->IsNode();
                if (!node) {
                    return TreeWalkAction::SkipChildren;
                }

                auto* extraData = node->GetExtraData(cpaKey);
                if (extraData && niObjectRttiChainContains(extraData, "BSConnectPoint::Parents")) {
                    NativeConnectPointParentArrayLayout points{};
                    if (native_memory::guardedCopyFromMemory(
                            reinterpret_cast<const char*>(extraData) + sizeof(RE::NiExtraData),
                            &points,
                            sizeof(points)) &&
                        points.count <= points.capacity && points.count <= kMaxParentRecords &&
                        (points.count == 0 || native_memory::pointerRangeLooksReadable(
                            points.entries, sizeof(*points.entries) * points.count))) {
                        for (std::uint32_t index = 0; index < points.count; ++index) {
                            NativeConnectPointParentLayout* parent = nullptr;
                            if (!native_memory::tryReadValue(points.entries + index, parent) ||
                                !native_memory::pointerRangeLooksReadable(parent, sizeof(*parent))) {
                                continue;
                            }
                            const char* authoredName = parent->connectPointName.c_str();
                            if (authoredName && _stricmp(authoredName, targetConnectPointName) == 0) {
                                outMatch = { .metadataOwner = node, .parent = parent };
                                found = true;
                                return TreeWalkAction::Stop;
                            }
                        }
                    }
                }
                return TreeWalkAction::Descend;
            };
            boundedTreeWalk(object, kTemplateScanMaxDepth, kTemplateScanMaxVisitedNodes, state, visitor);
            visited = state.visited;
            return found;
        }

        std::string materializeOmodRankSuffix(std::string_view authoredName, std::string_view rankSuffix)
        {
            if (rankSuffix.empty()) {
                return std::string(authoredName);
            }
            const auto separator = authoredName.find_last_of('|');
            if (separator == std::string_view::npos || separator + 1 >= authoredName.size() ||
                authoredName[separator + 1] != '0') {
                return std::string(authoredName);
            }
            std::string result(authoredName.substr(0, separator + 1));
            result.append(rankSuffix);
            return result;
        }

        enum class AuthoredOmodParentPathStage : std::uint8_t
        {
            NotAttempted,
            UnsupportedAttachPoint,
            ExistingParent,
            ExistingOwnedPath,
            ExistingOwnedPathInvalid,
            ProviderMetadataMissing,
            ProviderUsesRoot,
            ProviderParentNodeMissing,
            ProviderParentNodeAmbiguous,
            LiveAncestorMissing,
            DynamicPathNode,
            InvalidPathTransform,
            ContainerCreateFailed,
            PathNodeCreateFailed,
            AttachVerificationFailed,
            Prepared,
        };

        const char* authoredOmodParentPathStageName(const AuthoredOmodParentPathStage stage)
        {
            switch (stage) {
            case AuthoredOmodParentPathStage::NotAttempted:
                return "not-attempted";
            case AuthoredOmodParentPathStage::UnsupportedAttachPoint:
                return "unsupported-attach-point";
            case AuthoredOmodParentPathStage::ExistingParent:
                return "existing-parent";
            case AuthoredOmodParentPathStage::ExistingOwnedPath:
                return "existing-owned-path";
            case AuthoredOmodParentPathStage::ExistingOwnedPathInvalid:
                return "existing-owned-path-invalid";
            case AuthoredOmodParentPathStage::ProviderMetadataMissing:
                return "provider-metadata-missing";
            case AuthoredOmodParentPathStage::ProviderUsesRoot:
                return "provider-uses-root";
            case AuthoredOmodParentPathStage::ProviderParentNodeMissing:
                return "provider-parent-node-missing";
            case AuthoredOmodParentPathStage::ProviderParentNodeAmbiguous:
                return "provider-parent-node-ambiguous";
            case AuthoredOmodParentPathStage::LiveAncestorMissing:
                return "live-ancestor-missing";
            case AuthoredOmodParentPathStage::DynamicPathNode:
                return "dynamic-path-node";
            case AuthoredOmodParentPathStage::InvalidPathTransform:
                return "invalid-path-transform";
            case AuthoredOmodParentPathStage::ContainerCreateFailed:
                return "container-create-failed";
            case AuthoredOmodParentPathStage::PathNodeCreateFailed:
                return "path-node-create-failed";
            case AuthoredOmodParentPathStage::AttachVerificationFailed:
                return "attach-verification-failed";
            case AuthoredOmodParentPathStage::Prepared:
                return "prepared";
            default:
                return "unknown";
            }
        }

        struct OmodRecoveryTemplate
        {
            const OmodAuditRecord* record{ nullptr };
            RE::NiPointer<RE::NiNode> connectionRoot;
            RE::NiPointer<RE::NiNode> rawPhysicalRoot;
            OmodPhysicalTemplateSignature physicalSignature;
            bool usesRawReceiverGeometry{ false };

            [[nodiscard]] RE::NiNode* physicalRoot() const
            {
                return usesRawReceiverGeometry ? rawPhysicalRoot.get() : connectionRoot.get();
            }
        };

        struct AuthoredOmodParentPathPreparation
        {
            AuthoredOmodParentPathStage stage{ AuthoredOmodParentPathStage::NotAttempted };
            RE::NiNode* containerParent{ nullptr };
            RE::NiPointer<RE::NiNode> container;
            std::uint32_t providerFormId{ 0 };
            std::string connectPointName;
            std::string parentNodeName;
            std::string liveAncestorName;
        };

        void rollbackAuthoredOmodParentPath(AuthoredOmodParentPathPreparation& preparation)
        {
            if (!preparation.containerParent || !preparation.container) {
                return;
            }
            RE::NiPointer<RE::NiAVObject> detached;
            preparation.containerParent->DetachChild(preparation.container.get(), detached);
            f4vr::updateTransformsDown(preparation.containerParent, true);
            preparation.containerParent = nullptr;
            preparation.container.reset();
        }

        bool prepareAuthoredOmodParentPath(
            const OmodAuditRecord& candidate,
            RE::NiNode* weaponRoot,
            std::string_view rankSuffix,
            const std::vector<OmodRecoveryTemplate>& templates,
            AuthoredOmodParentPathPreparation& outPreparation)
        {
            outPreparation = {};
            if (!weaponRoot) {
                outPreparation.stage = AuthoredOmodParentPathStage::UnsupportedAttachPoint;
                return false;
            }

            const std::string_view canonicalConnectPoint =
                weapon_part_record_identity_policy::canonicalConnectPointForAttachPoint(candidate.attachPointFormId);
            const std::uint32_t providerAttachPoint =
                weapon_part_record_identity_policy::recoveryProviderAttachPointForAttachPoint(candidate.attachPointFormId);
            if (canonicalConnectPoint.empty() || providerAttachPoint == 0) {
                outPreparation.stage = AuthoredOmodParentPathStage::UnsupportedAttachPoint;
                return false;
            }
            outPreparation.connectPointName.assign(canonicalConnectPoint);

            const RE::BSFixedString cpaKey{ "CPA" };
            AuthoredConnectPointParentMatch parentMatch{};
            const OmodRecoveryTemplate* providerTemplate = nullptr;
            for (const auto& modelTemplate : templates) {
                if (!modelTemplate.record || !modelTemplate.connectionRoot ||
                    modelTemplate.record->attachPointFormId != providerAttachPoint) {
                    continue;
                }
                std::size_t visited = 0;
                if (findAuthoredConnectPointParentRecursive(
                        modelTemplate.connectionRoot.get(), cpaKey, outPreparation.connectPointName.c_str(),
                        parentMatch, visited)) {
                    providerTemplate = &modelTemplate;
                    break;
                }
            }
            if (!providerTemplate || !parentMatch.parent) {
                outPreparation.stage = AuthoredOmodParentPathStage::ProviderMetadataMissing;
                return false;
            }
            outPreparation.providerFormId = providerTemplate->record->formId;

            const char* rawParentNodeName = parentMatch.parent->parentNodeName.c_str();
            if (!rawParentNodeName || rawParentNodeName[0] == '\0') {
                outPreparation.stage = AuthoredOmodParentPathStage::ProviderUsesRoot;
                return false;
            }
            outPreparation.parentNodeName = materializeOmodRankSuffix(rawParentNodeName, rankSuffix);

            const auto existingParentMatches =
                collectWeaponAnimNodeMatches(weaponRoot, outPreparation.parentNodeName.c_str());
            for (const auto& match : existingParentMatches) {
                if (match.node && match.node->IsNode()) {
                    outPreparation.stage = AuthoredOmodParentPathStage::ExistingParent;
                    return false;
                }
            }

            const std::string containerName = fmt::format("{}{:08X}", kRockOmodEnrichmentPrefix, candidate.formId);
            const auto existingContainers = collectWeaponAnimNodeMatches(weaponRoot, containerName.c_str());
            if (!existingContainers.empty()) {
                for (const auto& match : existingContainers) {
                    if (match.node && !collectWeaponAnimNodeMatches(
                            match.node, outPreparation.parentNodeName.c_str()).empty()) {
                        outPreparation.stage = AuthoredOmodParentPathStage::ExistingOwnedPath;
                        return false;
                    }
                }
                outPreparation.stage = AuthoredOmodParentPathStage::ExistingOwnedPathInvalid;
                return false;
            }

            const auto providerParentMatches =
                collectWeaponAnimNodeMatches(providerTemplate->connectionRoot.get(), rawParentNodeName);
            RE::NiNode* providerParentNode = nullptr;
            for (const auto& match : providerParentMatches) {
                auto* candidateNode = match.node ? match.node->IsNode() : nullptr;
                if (!candidateNode) {
                    continue;
                }
                if (providerParentNode) {
                    outPreparation.stage = AuthoredOmodParentPathStage::ProviderParentNodeAmbiguous;
                    return false;
                }
                providerParentNode = candidateNode;
            }
            if (!providerParentNode) {
                outPreparation.stage = AuthoredOmodParentPathStage::ProviderParentNodeMissing;
                return false;
            }

            std::vector<RE::NiNode*> providerPath;
            providerPath.reserve(8);
            for (auto* cursor = providerParentNode; cursor && providerPath.size() < 16;) {
                providerPath.push_back(cursor);
                if (cursor == providerTemplate->connectionRoot.get()) {
                    break;
                }
                cursor = cursor->parent ? cursor->parent->IsNode() : nullptr;
            }
            if (providerPath.empty() || providerPath.back() != providerTemplate->connectionRoot.get()) {
                outPreparation.stage = AuthoredOmodParentPathStage::ProviderParentNodeMissing;
                return false;
            }

            RE::NiNode* liveAncestor = nullptr;
            std::size_t missingPathCount = 0;
            for (std::size_t pathIndex = 1; pathIndex < providerPath.size(); ++pathIndex) {
                const char* authoredAncestorName = providerPath[pathIndex]->name.c_str();
                if (!authoredAncestorName || authoredAncestorName[0] == '\0') {
                    continue;
                }
                const std::string liveName = materializeOmodRankSuffix(authoredAncestorName, rankSuffix);
                const auto liveMatches = collectWeaponAnimNodeMatches(weaponRoot, liveName.c_str());
                RE::NiNode* uniqueLiveNode = nullptr;
                bool ambiguous = false;
                for (const auto& match : liveMatches) {
                    auto* liveNode = match.node ? match.node->IsNode() : nullptr;
                    if (!liveNode) {
                        continue;
                    }
                    if (uniqueLiveNode) {
                        ambiguous = true;
                        break;
                    }
                    uniqueLiveNode = liveNode;
                }
                if (!ambiguous && uniqueLiveNode) {
                    liveAncestor = uniqueLiveNode;
                    missingPathCount = pathIndex;
                    outPreparation.liveAncestorName = liveName;
                    break;
                }
            }
            if (!liveAncestor || missingPathCount == 0) {
                outPreparation.stage = AuthoredOmodParentPathStage::LiveAncestorMissing;
                return false;
            }

            for (std::size_t pathIndex = 0; pathIndex < missingPathCount; ++pathIndex) {
                const auto* sourceNode = providerPath[pathIndex];
                if (sourceNode->controllers || sourceNode->extra || sourceNode->collisionObject || sourceNode->userData != 0) {
                    outPreparation.stage = AuthoredOmodParentPathStage::DynamicPathNode;
                    return false;
                }
                if (!weaponTransformFinite(sourceNode->local)) {
                    outPreparation.stage = AuthoredOmodParentPathStage::InvalidPathTransform;
                    return false;
                }
            }

            auto container = native_scene::createEngineNiNode(1);
            if (!container) {
                outPreparation.stage = AuthoredOmodParentPathStage::ContainerCreateFailed;
                return false;
            }
            container->name = containerName.c_str();
            container->local = transform_math::makeIdentityTransform<RE::NiTransform>();

            std::vector<RE::NiPointer<RE::NiNode>> pathNodes;
            pathNodes.reserve(missingPathCount);
            for (std::size_t pathIndex = missingPathCount; pathIndex-- > 0;) {
                const auto* sourceNode = providerPath[pathIndex];
                auto pathNode = native_scene::createEngineNiNode(1);
                if (!pathNode) {
                    outPreparation.stage = AuthoredOmodParentPathStage::PathNodeCreateFailed;
                    return false;
                }
                const char* sourceName = sourceNode->name.c_str();
                const std::string authoredName = materializeOmodRankSuffix(
                    sourceName ? std::string_view(sourceName) : std::string_view{}, rankSuffix);
                pathNode->name = authoredName.c_str();
                pathNode->local = sourceNode->local;
                pathNode->flags.flags = sourceNode->flags.flags;
                pathNodes.push_back(std::move(pathNode));
            }

            RE::NiNode* pathParent = container.get();
            for (auto& pathNode : pathNodes) {
                pathParent->AttachChild(pathNode.get(), true);
                pathParent = pathNode.get();
            }
            liveAncestor->AttachChild(container.get(), true);
            f4vr::updateTransformsDown(liveAncestor, true);
            if (collectWeaponAnimNodeMatches(container.get(), outPreparation.parentNodeName.c_str()).empty()) {
                RE::NiPointer<RE::NiAVObject> detached;
                liveAncestor->DetachChild(container.get(), detached);
                f4vr::updateTransformsDown(liveAncestor, true);
                outPreparation.stage = AuthoredOmodParentPathStage::AttachVerificationFailed;
                return false;
            }

            outPreparation.stage = AuthoredOmodParentPathStage::Prepared;
            outPreparation.containerParent = liveAncestor;
            outPreparation.container = std::move(container);
            return true;
        }

        // First physical (non-effect) TriShape with this authored name. A TriShape
        // is a leaf either way, so a name miss never descends into it.
        RE::BSTriShape* findTemplatePhysicalShapeByNameRecursive(
            RE::NiAVObject* node,
            const char* targetName,
            std::size_t& visited)
        {
            if (!targetName) {
                return nullptr;
            }
            BoundedTreeWalkState state{ .visited = visited };
            RE::BSTriShape* match = nullptr;
            auto visitor = [&](RE::NiAVObject* current, int) {
                auto* triShape = current->IsTriShape();
                if (!triShape) {
                    return TreeWalkAction::Descend;
                }
                const char* rawName = current->name.c_str();
                if (rawName && _stricmp(rawName, targetName) == 0 &&
                    classifyGeneratedWeaponEffectGeometry(triShape) == weapon_effect_geometry_policy::ExclusionReason::None) {
                    match = triShape;
                    return TreeWalkAction::Stop;
                }
                return TreeWalkAction::SkipChildren;
            };
            boundedTreeWalk(node, kTemplateScanMaxDepth, kTemplateScanMaxVisitedNodes, state, visitor);
            visited = state.visited;
            return match;
        }

        bool applyEquippedOmodModelCustomization(
            RE::BGSMod::Attachment::Mod* omod,
            RE::NiNode* clonedRoot,
            RE::TBO_InstanceData* instanceData)
        {
            if (!omod || !clonedRoot) {
                return false;
            }

            static const bool entryValidated = []() {
                constexpr std::array<std::uint8_t, 12> kExpectedPrefix{
                    0x48, 0x85, 0xC9, 0x0F, 0x84, 0x16, 0x01, 0x00, 0x00, 0x48, 0x8B, 0xC4
                };
                const auto address = REL::Offset(offsets::kFunc_ApplyOmodModelCustomization).address();
                std::array<std::uint8_t, kExpectedPrefix.size()> actual{};
                const bool valid = REL::Module::IsVR() &&
                    REL::Module::get().version() == F4SE::RUNTIME_VR_1_2_72 &&
                    native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address), actual.data(), actual.size()) &&
                    actual == kExpectedPrefix;
                if (!valid) {
                    ROCK_LOG_ERROR(Weapon, "OMOD physical enrichment disabled: model-customization entry validation failed at 0x{:X}", address);
                }
                return valid;
            }();
            if (!entryValidated) {
                return false;
            }

            using ApplyModelCustomization = void (*)(RE::NiAVObject*, RE::BGSModelMaterialSwap*, void*, RE::TBO_InstanceData*, void*);
            const auto applyCustomization = reinterpret_cast<ApplyModelCustomization>(
                REL::Offset(offsets::kFunc_ApplyOmodModelCustomization).address());
            applyCustomization(clonedRoot, static_cast<RE::BGSModelMaterialSwap*>(omod), nullptr, instanceData, nullptr);
            return true;
        }

        enum class OmodPhysicalEnrichmentStage : std::uint8_t
        {
            NotAttempted,
            InvalidInput,
            ExistingContainerVerified,
            ExistingContainerMissingAnchor,
            CloneFailed,
            CustomizationFailed,
            AnchorLookupFailed,
            AnchorParentMissing,
            AnchorSkinUnreadable,
            AnchorSkinned,
            AnimatedParentUnnamed,
            AnimatedParentMissing,
            AnchorDetachFailed,
            ContainerCreateFailed,
            AttachVerificationFailed,
            Restored,
        };

        const char* omodPhysicalEnrichmentStageName(const OmodPhysicalEnrichmentStage stage)
        {
            switch (stage) {
            case OmodPhysicalEnrichmentStage::NotAttempted:
                return "not-attempted";
            case OmodPhysicalEnrichmentStage::InvalidInput:
                return "invalid-input";
            case OmodPhysicalEnrichmentStage::ExistingContainerVerified:
                return "existing-container-verified";
            case OmodPhysicalEnrichmentStage::ExistingContainerMissingAnchor:
                return "existing-container-missing-anchor";
            case OmodPhysicalEnrichmentStage::CloneFailed:
                return "clone-failed";
            case OmodPhysicalEnrichmentStage::CustomizationFailed:
                return "customization-failed";
            case OmodPhysicalEnrichmentStage::AnchorLookupFailed:
                return "anchor-lookup-failed";
            case OmodPhysicalEnrichmentStage::AnchorParentMissing:
                return "anchor-parent-missing";
            case OmodPhysicalEnrichmentStage::AnchorSkinUnreadable:
                return "anchor-skin-unreadable";
            case OmodPhysicalEnrichmentStage::AnchorSkinned:
                return "anchor-skinned";
            case OmodPhysicalEnrichmentStage::AnimatedParentUnnamed:
                return "animated-parent-unnamed";
            case OmodPhysicalEnrichmentStage::AnimatedParentMissing:
                return "animated-parent-missing";
            case OmodPhysicalEnrichmentStage::AnchorDetachFailed:
                return "anchor-detach-failed";
            case OmodPhysicalEnrichmentStage::ContainerCreateFailed:
                return "container-create-failed";
            case OmodPhysicalEnrichmentStage::AttachVerificationFailed:
                return "attach-verification-failed";
            case OmodPhysicalEnrichmentStage::Restored:
                return "restored";
            default:
                return "unknown";
            }
        }

        bool enrichMissingOmodPhysicalAnchor(
            RE::BGSMod::Attachment::Mod* omod,
            RE::NiNode* templateRoot,
            const OmodPhysicalTemplateSignature& signature,
            RE::NiNode* coverageRoot,
            RE::TBO_InstanceData* instanceData,
            std::string& outTargetParentName,
            OmodPhysicalEnrichmentStage& outStage)
        {
            outTargetParentName.clear();
            outStage = OmodPhysicalEnrichmentStage::InvalidInput;
            if (!omod || !templateRoot || !coverageRoot || signature.durableAnchorName.empty()) {
                return false;
            }

            const std::string containerName = fmt::format("{}{:08X}", kRockOmodEnrichmentPrefix, omod->formID);
            if (!collectWeaponAnimNodeMatches(coverageRoot, containerName.c_str()).empty()) {
                outTargetParentName = containerName;
                const bool anchorPresent =
                    !collectWeaponAnimNodeMatches(coverageRoot, signature.durableAnchorName.c_str()).empty();
                outStage = anchorPresent ?
                    OmodPhysicalEnrichmentStage::ExistingContainerVerified :
                    OmodPhysicalEnrichmentStage::ExistingContainerMissingAnchor;
                return anchorPresent;
            }

            f4vr::NiCloneProcess cloneProcess{};
            cloneProcess.unk18 = reinterpret_cast<std::uint64_t*>(f4vr::cloneAddr1.address());
            cloneProcess.unk48 = reinterpret_cast<std::uint64_t*>(f4vr::cloneAddr2.address());
            RE::NiPointer<RE::NiNode> clonedRoot;
            clonedRoot.reset(f4vr::cloneNode(templateRoot, &cloneProcess));
            if (!clonedRoot) {
                outStage = OmodPhysicalEnrichmentStage::CloneFailed;
                return false;
            }
            if (!applyEquippedOmodModelCustomization(omod, clonedRoot.get(), instanceData)) {
                outStage = OmodPhysicalEnrichmentStage::CustomizationFailed;
                return false;
            }

            std::size_t anchorVisited = 0;
            auto* clonedAnchor = findTemplatePhysicalShapeByNameRecursive(
                clonedRoot.get(), signature.durableAnchorName.c_str(), anchorVisited);
            if (!clonedAnchor) {
                outStage = OmodPhysicalEnrichmentStage::AnchorLookupFailed;
                return false;
            }
            auto* clonedParent = clonedAnchor->parent ? clonedAnchor->parent->IsNode() : nullptr;
            if (!clonedParent) {
                outStage = OmodPhysicalEnrichmentStage::AnchorParentMissing;
                return false;
            }

            void* skinInstance = nullptr;
            if (!native_memory::tryReadField(clonedAnchor, VROffset::skinInstance, skinInstance)) {
                outStage = OmodPhysicalEnrichmentStage::AnchorSkinUnreadable;
                ROCK_LOG_WARN(Weapon,
                    "OMOD physical enrichment rejected unreadable durable anchor '{}' omod={:08X}",
                    signature.durableAnchorName,
                    omod->formID);
                return false;
            }
            if (skinInstance) {
                outStage = OmodPhysicalEnrichmentStage::AnchorSkinned;
                ROCK_LOG_WARN(Weapon,
                    "OMOD physical enrichment rejected skinned durable anchor '{}' omod={:08X}",
                    signature.durableAnchorName,
                    omod->formID);
                return false;
            }

            RE::NiNode* targetParent = nullptr;
            const bool anchorIsDirectRootChild = clonedParent == clonedRoot.get();
            if (!anchorIsDirectRootChild) {
                const char* parentName = clonedParent->name.c_str();
                if (!parentName || parentName[0] == '\0') {
                    outStage = OmodPhysicalEnrichmentStage::AnimatedParentUnnamed;
                    return false;
                }
                const auto parentMatches = collectWeaponAnimNodeMatches(coverageRoot, parentName);
                for (const auto& match : parentMatches) {
                    if (auto* candidate = match.node ? match.node->IsNode() : nullptr) {
                        targetParent = candidate;
                        break;
                    }
                }
                if (!targetParent) {
                    outStage = OmodPhysicalEnrichmentStage::AnimatedParentMissing;
                    ROCK_LOG_WARN(Weapon,
                        "OMOD physical enrichment could not map animated parent '{}' for anchor '{}' omod={:08X}",
                        parentName,
                        signature.durableAnchorName,
                        omod->formID);
                    return false;
                }
            }

            RE::NiPointer<RE::NiAVObject> recoveredAnchor;
            clonedParent->DetachChild(clonedAnchor, recoveredAnchor);
            if (!recoveredAnchor) {
                outStage = OmodPhysicalEnrichmentStage::AnchorDetachFailed;
                return false;
            }

            auto enrichmentContainer = native_scene::createEngineNiNode(1);
            if (!enrichmentContainer) {
                outStage = OmodPhysicalEnrichmentStage::ContainerCreateFailed;
                return false;
            }
            enrichmentContainer->name = containerName.c_str();
            enrichmentContainer->local = transform_math::makeIdentityTransform<RE::NiTransform>();
            if (!targetParent) {
                recoveredAnchor->local = transform_math::composeTransforms(clonedRoot->local, recoveredAnchor->local);
            }

            enrichmentContainer->AttachChild(recoveredAnchor.get(), true);
            auto* containerParent = targetParent ? targetParent : coverageRoot;
            containerParent->AttachChild(enrichmentContainer.get(), true);
            f4vr::updateTransformsDown(containerParent, true);
            const bool anchorAttached =
                !collectWeaponAnimNodeMatches(enrichmentContainer.get(), signature.durableAnchorName.c_str()).empty();
            if (!anchorAttached) {
                outStage = OmodPhysicalEnrichmentStage::AttachVerificationFailed;
                containerParent->DetachChild(enrichmentContainer.get());
                f4vr::updateTransformsDown(containerParent, true);
                return false;
            }

            outTargetParentName = fmt::format("{}/{}", safeNodeName(containerParent), containerName);
            outStage = OmodPhysicalEnrichmentStage::Restored;
            return true;
        }
    }

    namespace
    {
        /*
         * Phase 2 of the coverage audit: decide which installed OMODs are not
         * represented in the assembled weapon, and record why for each one.
         *
         * An OMOD counts as covered when a node matching its model token exists,
         * is visible, and has ROCK collider evidence under it. Missing any of the
         * three makes it a self-heal candidate. This phase is pure - it only reads
         * the scan results - so the same weapon state always classifies the same
         * way, and the verdict can be trusted from the log alone.
         */
        std::vector<std::size_t> classifyOmodAuditSelfHealCandidates(
            const std::vector<OmodAuditRecord>& records,
            const std::vector<OmodAuditTokenSlot>& tokenSlots,
            const std::unordered_map<std::uint32_t, std::uint32_t>& bodiesByAttachPointFormId,
            const std::unordered_set<std::uintptr_t>& evidenceSourceAddresses)
        {
            std::vector<std::size_t> selfHealCandidates;
        for (std::size_t i = 0; i < records.size(); ++i) {
            const auto& record = records[i];
            const auto& slot = tokenSlots[i];

            std::uint32_t pairedBodies = 0;
            if (record.attachPointFormId != 0) {
                const auto pairedIt = bodiesByAttachPointFormId.find(record.attachPointFormId);
                if (pairedIt != bodiesByAttachPointFormId.end()) {
                    pairedBodies = pairedIt->second;
                }
            }

            std::size_t evidenceUnderMatches = 0;
            bool anyMatchVisible = false;
            std::size_t loggedMatches = 0;
            for (const auto& match : slot.matches) {
                const auto stats = summarizeWeaponAnimNodeSubtree(match.node);
                std::size_t evidenceVisited = 0;
                const std::size_t evidenceSources =
                    countOmodAuditEvidenceSourcesInSubtree(match.node, evidenceSourceAddresses, evidenceVisited);
                evidenceUnderMatches += evidenceSources;
                const bool matchVisible = weaponVisualNodeVisible(match.node);
                anyMatchVisible = anyMatchVisible || matchVisible || stats.visibleTriShapeCount > 0;
                if (loggedMatches < OMOD_AUDIT_MAX_LOGGED_MATCHES_PER_OMOD) {
                    ++loggedMatches;
                    ROCK_OMOD_DIAGNOSTIC_INFO(
                        "OMOD-AUDIT match omod={:08X} root='{}' path='{}' addr={:x} visible={} flags=0x{:X} appCulled={} subtreeNodes={} triShapes={} visibleTriShapes={} hiddenFlags={} appCulledNodes={} evidenceSources={}",
                        record.formId,
                        match.rootLabel,
                        buildOmodAuditNodePath(match.node),
                        reinterpret_cast<std::uintptr_t>(match.node),
                        matchVisible ? "yes" : "no",
                        static_cast<std::uint32_t>(match.node->flags.flags),
                        match.node->GetAppCulled() ? "yes" : "no",
                        stats.nodeCount,
                        stats.triShapeCount,
                        stats.visibleTriShapeCount,
                        stats.hiddenFlagCount,
                        stats.appCulledCount,
                        evidenceSources);
                }
            }

            const auto coverageDecision = weapon_omod_audit_policy::decideCoverage(
                weapon_omod_audit_policy::CoverageInput{
                    .disabled = record.disabled,
                    .resolved = record.resolved,
                    .hasModelToken = !slot.lowerToken.empty(),
                    .hasEvidenceUnderMatch = evidenceUnderMatches > 0,
                    .hasPairedBody = pairedBodies > 0,
                    .hasNodeMatch = !slot.matches.empty(),
                    .anyNodeMatchVisible = anyMatchVisible,
                });
            if (coverageDecision.selfHealCandidate) {
                selfHealCandidates.push_back(i);
            }
            const char* verdict = weapon_omod_audit_policy::coverageVerdictName(coverageDecision.verdict);

            ROCK_OMOD_DIAGNOSTIC_INFO(
                "OMOD-AUDIT omod={:08X} '{}' index={} rank={} disabled={} attachPoint={:08X} attachPointIndex={} model='{}' token='{}' pairedBodies={} nodeMatches={} evidenceUnderMatches={} verdict={}",
                record.formId,
                record.name,
                record.modIndex,
                record.rank,
                record.disabled,
                record.attachPointFormId,
                record.attachPointIndex,
                record.modelPath,
                slot.lowerToken,
                pairedBodies,
                slot.matches.size(),
                evidenceUnderMatches,
                verdict);
        }
            return selfHealCandidates;
        }

        // What one self-heal pass did. sceneEnriched means the assembled weapon was
        // modified, so the caller owes a collider rebuild.
        struct OmodSelfHealOutcome
        {
            std::size_t attempted{ 0 };
            std::size_t healed{ 0 };
            bool sceneEnriched{ false };
        };

        /*
         * Phase 3 of the coverage audit: clone the missing OMOD geometry out of its
         * model template and parent it under the weapon instance ROCK harvests
         * colliders from.
         *
         * This is the ONLY phase that mutates the scene and the only one that calls
         * into the engine, so every step fails closed: a template that will not
         * load, a node that will not resolve, or a native call whose byte prefix
         * does not validate all end as a logged skip.
         *
         * selfHealAttempted is the caller's per-instance-address memory of what has
         * already been healed. It is threaded in explicitly rather than reached
         * through the object so the one piece of state that survives a run is
         * visible in the signature: a name-unmatchable heal must not stack
         * duplicate geometry across audits, while an engine reassembly produces a
         * new instance address and legitimately re-opens healing.
         */
        OmodSelfHealOutcome runOmodAuditSelfHeal(
            const std::uint32_t runIndex,
            const RE::TESForm* weaponForm,
            const std::vector<OmodAuditRecord>& records,
            const std::vector<OmodAuditTokenSlot>& tokenSlots,
            const std::vector<std::size_t>& selfHealCandidates,
            // Passed through to the native attach: the engine needs the equipped
            // instance's own data to resolve the OMOD's model variant.
            RE::TBO_InstanceData* equippedInstanceData,
            std::unordered_set<std::uint64_t>& selfHealAttempted)
        {
            OmodSelfHealOutcome outcome{};
            if (selfHealCandidates.empty()) {
                return outcome;
            }
            RE::NiNode* healTargetNode = nullptr;
            const char* healTargetRootLabel = "";
            if (weaponForm && tokenSlots.size() > records.size()) {
                for (const auto& match : tokenSlots.back().matches) {
                    auto* candidateNode = match.node ? match.node->IsNode() : nullptr;
                    if (!candidateNode) {
                        continue;
                    }
                    // Prefer the instance ROCK harvests colliders from.
                    if (!healTargetNode || std::strcmp(match.rootLabel, "updateWeaponNode") == 0) {
                        healTargetNode = candidateNode;
                        healTargetRootLabel = match.rootLabel;
                    }
                    if (std::strcmp(match.rootLabel, "updateWeaponNode") == 0) {
                        break;
                    }
                }
            }

            if (!healTargetNode) {
                ROCK_LOG_WARN(Weapon,
                    "OMOD-HEAL run={} skipped: no weapon instance node found for {} candidate(s)",
                    runIndex,
                    selfHealCandidates.size());
            } else {
                using TryAttach3DRecurseFn = bool (*)(RE::BGSMod::Attachment::Mod*, RE::NiNode*, const char*, RE::TBO_InstanceData*);
                static REL::Relocation<TryAttach3DRecurseFn> tryAttach3DRecurse{ REL::Offset(0x2D9140) };
                constexpr std::size_t OMOD_SELF_HEAL_MAX_PER_AUDIT = 4;

                std::vector<std::size_t> orderedSelfHealCandidates = selfHealCandidates;
                std::stable_sort(
                    orderedSelfHealCandidates.begin(),
                    orderedSelfHealCandidates.end(),
                    [&records](const std::size_t lhs, const std::size_t rhs) {
                        return weapon_part_record_identity_policy::recoveryDependencyRank(records[lhs].attachPointFormId) <
                               weapon_part_record_identity_policy::recoveryDependencyRank(records[rhs].attachPointFormId);
                    });

                /*
                 * Parent-path recovery needs the candidate model and only the
                 * installed provider slot that authors its P-* metadata
                 * (receiver for top-level parts, barrel for muzzle). Keep all
                 * loaded roots alive through the bounded recovery pass because
                 * every parsed connect-point pointer is owned by its root.
                 */
                std::unordered_set<std::uint32_t> requestedTemplateFormIds;
                std::unordered_set<std::uint32_t> requestedProviderAttachPoints;
                for (const std::size_t candidateIndex : orderedSelfHealCandidates) {
                    const auto& record = records[candidateIndex];
                    requestedTemplateFormIds.insert(record.formId);
                    const auto providerAttachPoint =
                        weapon_part_record_identity_policy::recoveryProviderAttachPointForAttachPoint(record.attachPointFormId);
                    if (providerAttachPoint != 0) {
                        requestedProviderAttachPoints.insert(providerAttachPoint);
                    }
                }

                std::vector<OmodRecoveryTemplate> recoveryTemplates;
                recoveryTemplates.reserve(requestedTemplateFormIds.size() + requestedProviderAttachPoints.size());
                std::unordered_set<std::uint32_t> loadedTemplateFormIds;
                const auto appendRecoveryTemplate = [&](const OmodAuditRecord& record) {
                    if (record.disabled || !record.resolved || record.formId == 0 || record.modelPath.empty() ||
                        !loadedTemplateFormIds.insert(record.formId).second) {
                        return;
                    }
                    auto connectionRoot = loadCompleteOmodModelTemplate(record.modelPath);
                    if (!connectionRoot) {
                        return;
                    }

                    auto completeSignature = collectOmodPhysicalTemplateSignature(connectionRoot.get());
                    OmodRecoveryTemplate modelTemplate{
                        .record = &record,
                        .connectionRoot = std::move(connectionRoot),
                        .physicalSignature = std::move(completeSignature),
                    };

                    const bool recoveryCandidate = requestedTemplateFormIds.contains(record.formId);
                    if (recoveryCandidate &&
                        record.attachPointFormId == weapon_part_record_identity_policy::kAttachPointReceiver) {
                        auto rawPhysicalRoot = loadGeometryInspectionOmodModelTemplate(record.modelPath);
                        if (rawPhysicalRoot) {
                            auto rawSignature = collectOmodPhysicalTemplateSignature(rawPhysicalRoot.get());
                            std::size_t rawCollisionVisited = 0;
                            std::size_t completeCollisionVisited = 0;
                            const bool hasNativeCollisionObject =
                                templateContainsNativeCollisionObjectRecursive(
                                    rawPhysicalRoot.get(), rawCollisionVisited) ||
                                templateContainsNativeCollisionObjectRecursive(
                                    modelTemplate.connectionRoot.get(), completeCollisionVisited);
                            const bool completeSignatureCoveredByRaw =
                                physicalTemplateSignatureCovers(rawSignature, modelTemplate.physicalSignature);
                            if (weapon_omod_audit_policy::shouldPreferRawReceiverGeometryTemplate(
                                    true,
                                    hasNativeCollisionObject,
                                    completeSignatureCoveredByRaw,
                                    modelTemplate.physicalSignature.meshNames.size(),
                                    rawSignature.meshNames.size())) {
                                ROCK_OMOD_DIAGNOSTIC_INFO(
                                    "OMOD-HEAL omod={:08X} '{}' selected raw receiver geometry template: "
                                    "normalMeshes={} rawMeshes={} nativeCollision=yes",
                                    record.formId,
                                    record.name,
                                    modelTemplate.physicalSignature.meshNames.size(),
                                    rawSignature.meshNames.size());
                                modelTemplate.rawPhysicalRoot = std::move(rawPhysicalRoot);
                                modelTemplate.physicalSignature = std::move(rawSignature);
                                modelTemplate.usesRawReceiverGeometry = true;
                            }
                        }
                    }
                    recoveryTemplates.push_back(std::move(modelTemplate));
                };
                for (const std::size_t candidateIndex : orderedSelfHealCandidates) {
                    appendRecoveryTemplate(records[candidateIndex]);
                }
                for (const auto& record : records) {
                    if (requestedProviderAttachPoints.contains(record.attachPointFormId)) {
                        appendRecoveryTemplate(record);
                    }
                }

                const auto findRecoveryTemplate = [&recoveryTemplates](const std::uint32_t formId) -> const OmodRecoveryTemplate* {
                    for (const auto& modelTemplate : recoveryTemplates) {
                        if (modelTemplate.record && modelTemplate.record->formId == formId) {
                            return &modelTemplate;
                        }
                    }
                    return nullptr;
                };

                if (selfHealAttempted.size() > 256) {
                    selfHealAttempted.clear();
                }
                for (const std::size_t candidateIndex : orderedSelfHealCandidates) {
                    if (outcome.attempted >= OMOD_SELF_HEAL_MAX_PER_AUDIT) {
                        break;
                    }
                    const auto& record = records[candidateIndex];
                    const std::uint64_t attemptKey =
                        reinterpret_cast<std::uintptr_t>(healTargetNode) ^ (static_cast<std::uint64_t>(record.formId) << 20);
                    if (selfHealAttempted.contains(attemptKey)) {
                        continue;
                    }

                    auto* omod = RE::TESForm::GetFormByID<RE::BGSMod::Attachment::Mod>(record.formId);
                    if (!omod) {
                        selfHealAttempted.insert(attemptKey);
                        ROCK_LOG_WARN(Weapon, "OMOD-HEAL run={} omod={:08X} skipped: form no longer resolves", runIndex, record.formId);
                        continue;
                    }

                    /*
                     * Filename tokens are only a candidate trigger. The truth
                     * gate loads the complete 0xED hierarchy, excludes effect
                     * geometry, and selects its largest physical mesh as the
                     * durable housing anchor. Physics-bearing receiver OMODs
                     * may instead use a strict-superset 0x20 geometry view when
                     * the normal postprocessor consumed their display shell.
                     * That anchor is authoritative for collider recovery: if it
                     * already exists, ROCK does not duplicate the model merely
                     * because cartridges, followers, glass, or other secondary
                     * pieces differ. A partial branch missing the anchor
                     * receives only the cloned authored housing.
                     */
                    const OmodRecoveryTemplate* recoveryTemplate = findRecoveryTemplate(record.formId);
                    if (!recoveryTemplate || !recoveryTemplate->physicalRoot()) {
                        selfHealAttempted.insert(attemptKey);
                        ROCK_LOG_WARN(Weapon,
                            "OMOD-HEAL run={} omod={:08X} '{}' skipped: complete 0xED model load failed model='{}'",
                            runIndex,
                            record.formId,
                            record.name,
                            record.modelPath);
                        continue;
                    }

                    RE::NiNode* signatureRoot = recoveryTemplate->physicalRoot();
                    const auto& templateSignature = recoveryTemplate->physicalSignature;
                    if (templateSignature.meshNames.empty() || templateSignature.durableAnchorName.empty()) {
                        selfHealAttempted.insert(attemptKey);
                        ROCK_LOG_WARN(Weapon,
                            "OMOD-HEAL run={} omod={:08X} '{}' skipped: template has no verifiable physical housing model='{}'",
                            runIndex,
                            record.formId,
                            record.name,
                            record.modelPath);
                        continue;
                    }

                    RE::NiNode* coverageRoot = resolveOmodPhysicalCoverageRoot(healTargetNode, record.attachPointFormId);
                    const char* firstMatchedSignatureName = nullptr;
                    const std::size_t matchedSignatureNameCount = countPresentOmodPhysicalSignatureNames(
                        coverageRoot,
                        templateSignature,
                        firstMatchedSignatureName);
                    const bool durableAnchorPresent =
                        !collectWeaponAnimNodeMatches(coverageRoot, templateSignature.durableAnchorName.c_str()).empty();
                    const std::size_t requiredSignatureNameCount =
                        weapon_omod_audit_policy::requiredTemplateSignatureMatches(templateSignature.meshNames.size());
                    const bool coherentPhysicalSignaturePresent =
                        weapon_omod_audit_policy::physicalTemplateSignatureIsPresent(
                            matchedSignatureNameCount,
                            templateSignature.meshNames.size(),
                            durableAnchorPresent);
                    if (!weapon_omod_audit_policy::requiresDurableAnchorRecovery(durableAnchorPresent)) {
                        selfHealAttempted.insert(attemptKey);
                        ROCK_OMOD_DIAGNOSTIC_INFO(
                            "OMOD-HEAL run={} omod={:08X} '{}' skipped: durable physical housing already present in slot "
                            "matches={}/{} required={} coherent={} anchor='{}' example='{}' — no duplicate recovery needed",
                            runIndex,
                            record.formId,
                            record.name,
                            matchedSignatureNameCount,
                            templateSignature.meshNames.size(),
                            requiredSignatureNameCount,
                            coherentPhysicalSignaturePresent ? "yes" : "no",
                            templateSignature.durableAnchorName,
                            firstMatchedSignatureName ? firstMatchedSignatureName : "");
                        continue;
                    }

                    ROCK_OMOD_DIAGNOSTIC_INFO(
                        "OMOD-HEAL run={} omod={:08X} '{}' confirmed incomplete: physical signature matches={}/{} required={} anchor='{}' anchorPresent={} - starting bounded recovery",
                        runIndex,
                        record.formId,
                        record.name,
                        matchedSignatureNameCount,
                        templateSignature.meshNames.size(),
                        requiredSignatureNameCount,
                        templateSignature.durableAnchorName,
                        durableAnchorPresent ? "yes" : "no");

                    char rankSuffixBuffer[8] = {};
                    const char* rankSuffix = nullptr;
                    if (record.modIndex != 0) {
                        // Same suffix rule as the engine's own attach loop:
                        // non-zero index entries get a "%u" node-name suffix.
                        std::snprintf(rankSuffixBuffer, sizeof(rankSuffixBuffer), "%u", record.modIndex);
                        rankSuffix = rankSuffixBuffer;
                    }

                    /*
                     * Never post-hide an engine attachment: the address-diff
                     * experiment hid real rendered geometry after native
                     * capture/reparenting. Whole-model attach is used when the
                     * observed names do not form a coherent template signature;
                     * coherent partial trees preserve their authored animated
                     * pieces and receive only the missing durable housing below.
                     * A raw receiver anchor is absent specifically because the
                     * native postprocessed model consumed it, so repeating that
                     * native attach cannot restore it and can only duplicate the
                     * surviving trigger/controller branch. Recover that one
                     * housing directly from the guarded raw template instead.
                     */
                    const auto beforeStats = summarizeWeaponAnimNodeSubtree(healTargetNode);
                    selfHealAttempted.insert(attemptKey);
                    ++outcome.attempted;
                    const bool nativeWholeModelEligible = weapon_omod_audit_policy::shouldAttemptWholeModelAttach(
                        matchedSignatureNameCount,
                        templateSignature.meshNames.size(),
                        durableAnchorPresent);
                    const bool nativeAttachNeeded = nativeWholeModelEligible &&
                        !recoveryTemplate->usesRawReceiverGeometry;
                    AuthoredOmodParentPathPreparation parentPathPreparation{};
                    const bool authoredParentPathPrepared = nativeAttachNeeded &&
                        prepareAuthoredOmodParentPath(
                            record,
                            healTargetNode,
                            rankSuffix ? std::string_view(rankSuffix) : std::string_view{},
                            recoveryTemplates,
                            parentPathPreparation);
                    bool attached = nativeAttachNeeded &&
                        tryAttach3DRecurse(omod, healTargetNode, rankSuffix, equippedInstanceData);
                    auto afterStats = summarizeWeaponAnimNodeSubtree(healTargetNode);
                    bool anchorPresentAfterNative =
                        !collectWeaponAnimNodeMatches(coverageRoot, templateSignature.durableAnchorName.c_str()).empty();
                    bool authoredPathCapturedAnchor = authoredParentPathPrepared && parentPathPreparation.container &&
                        !collectWeaponAnimNodeMatches(
                            parentPathPreparation.container.get(), templateSignature.durableAnchorName.c_str()).empty();
                    if (authoredParentPathPrepared && !authoredPathCapturedAnchor) {
                        /*
                         * A true native return only means a CPA record matched;
                         * FO4VR does not propagate Parent::ConnectChild failure.
                         * Retain the authored path only when the candidate's
                         * durable geometry actually landed below its owned
                         * container. Otherwise remove the entire path before
                         * the existing bounded housing enrichment runs.
                         */
                        rollbackAuthoredOmodParentPath(parentPathPreparation);
                        afterStats = summarizeWeaponAnimNodeSubtree(healTargetNode);
                        anchorPresentAfterNative =
                            !collectWeaponAnimNodeMatches(coverageRoot, templateSignature.durableAnchorName.c_str()).empty();
                    }
                    std::string enrichmentParentName;
                    OmodPhysicalEnrichmentStage enrichmentStage = OmodPhysicalEnrichmentStage::NotAttempted;
                    const bool physicalAnchorEnriched = !anchorPresentAfterNative &&
                        enrichMissingOmodPhysicalAnchor(
                            omod,
                            signatureRoot,
                            templateSignature,
                            coverageRoot,
                            equippedInstanceData,
                            enrichmentParentName,
                            enrichmentStage);
                    afterStats = summarizeWeaponAnimNodeSubtree(healTargetNode);
                    const bool geometryAdded = afterStats.triShapeCount > beforeStats.triShapeCount;
                    const bool durableAnchorRestored = anchorPresentAfterNative || physicalAnchorEnriched;
                    outcome.healed += geometryAdded ? 1 : 0;

                    ROCK_OMOD_DIAGNOSTIC_INFO(
                        "OMOD-HEAL run={} omod={:08X} '{}' model='{}' suffix='{}' target='{}'/{:x} physicalTemplate={} nativeAttachAttempted={} attached={} geometryAdded={} durableAnchor='{}' restored={} authoredPathStage={} authoredPathProvider={:08X} authoredParent='{}' authoredAncestor='{}' authoredPathCapturedAnchor={} enrichmentStage={} enrichmentParent='{}' subtreeNodes {}->{} triShapes {}->{} visibleTriShapes {}->{}",
                        runIndex,
                        record.formId,
                        record.name,
                        record.modelPath,
                        rankSuffix ? rankSuffix : "",
                        healTargetRootLabel,
                        reinterpret_cast<std::uintptr_t>(healTargetNode),
                        recoveryTemplate->usesRawReceiverGeometry ? "raw-receiver-geometry" : "complete-0xED",
                        nativeAttachNeeded ? "yes" :
                            (recoveryTemplate->usesRawReceiverGeometry ? "no-raw-receiver-anchor" : "no-coherent-partial-tree"),
                        attached ? "YES" : "no",
                        geometryAdded ? "YES" : "no",
                        templateSignature.durableAnchorName,
                        durableAnchorRestored ? "YES" : "no",
                        authoredOmodParentPathStageName(parentPathPreparation.stage),
                        parentPathPreparation.providerFormId,
                        parentPathPreparation.parentNodeName,
                        parentPathPreparation.liveAncestorName,
                        authoredPathCapturedAnchor ? "YES" : "no",
                        omodPhysicalEnrichmentStageName(enrichmentStage),
                        enrichmentParentName,
                        beforeStats.nodeCount,
                        afterStats.nodeCount,
                        beforeStats.triShapeCount,
                        afterStats.triShapeCount,
                        beforeStats.visibleTriShapeCount,
                        afterStats.visibleTriShapeCount);
                }

                if (outcome.healed > 0) {
                    ROCK_OMOD_DIAGNOSTIC_INFO(
                        "OMOD-HEAL run={} healed={} of {} attempted - requesting collider rebuild",
                        runIndex,
                        outcome.healed,
                        outcome.attempted);
                    outcome.sceneEnriched = true;
                }
            }

            return outcome;
        }
    }

    /*
     * Periodic research audit (gated on bDebugWeaponOmodCoverageAudit) that
     * re-diffs record truth against the live scene graphs while a generated
     * weapon body set is active. The build-time OMOD-DUMP can only show what
     * the tree looked like when the body set was published; this audit exists
     * to catch the missing-part failure where the engine attaches an OMOD's
     * model subtree after ROCK's build window closed and nothing ever looks
     * again. Per installed OMOD it reports whether a node matching the model
     * NIF exists under any audited root, whether it is visible, and whether
     * any generated collider evidence source lives beneath it; the P-* census
     * covers parts whose mesh names do not contain the model basename. The
     * visual-key drift line on each audit is the decisive signal: drift=YES
     * with an unchanged body set means geometry arrived or changed after the
     * build and current triggers never rescanned it.
     */
    WeaponCollision::OmodCoverageAuditResult WeaponCollision::maybeRunWeaponOmodCoverageAudit(
        RE::NiAVObject* weaponNode, std::uint64_t auditedEquippedKey, bool forceBeforeInitialBuild)
    {
        OmodCoverageAuditResult result{};
        if (!weaponNode || auditedEquippedKey == 0 ||
            (!forceBeforeInitialBuild && (!hasWeaponBody() || _cachedWeaponBodySetKey == 0))) {
            return result;
        }

        if (!forceBeforeInitialBuild && _omodCoverageAuditBodySetKey != _cachedWeaponBodySetKey) {
            _omodCoverageAuditBodySetKey = _cachedWeaponBodySetKey;
            _omodCoverageAuditFrameCounter = 0;
            _omodCoverageAuditRunIndex = 0;
        }

        const int intervalFrames = (std::min)(
            kMandatoryOmodSelfHealIntervalFrames,
            (std::max)(30, g_rockConfig.rockDebugWeaponOmodCoverageAuditIntervalFrames));
        // First audit fires ~1s after publication so late model streaming is
        // observed quickly; later audits repeat at the configured interval.
        const int dueFrames = _omodCoverageAuditRunIndex == 0 ? (std::min)(90, intervalFrames) : intervalFrames;
        if (!forceBeforeInitialBuild && ++_omodCoverageAuditFrameCounter < dueFrames) {
            return result;
        }
        _omodCoverageAuditFrameCounter = 0;
        const std::uint32_t runIndex = _omodCoverageAuditRunIndex++;
        result.ran = true;

        auto* player = f4vr::getPlayer();
        auto* equipData = f4vr::getEquippedWeaponItem();
        auto* weaponForm = equipData ? equipData->item.object : nullptr;
        auto* equippedInstanceData = equipData ? equipData->item.instanceData.get() : nullptr;

        WeaponVisualKeyStats visualStatsNow{};
        const std::uint64_t visualKeyNow = getWeaponVisualCompositionKey(weaponNode, visualStatsNow);
        const bool visualDrift = visualKeyNow != 0 && _cachedWeaponVisualKey != 0 && visualKeyNow != _cachedWeaponVisualKey;

        const RE::NiAVObject* rootHiddenAncestor = findOmodAuditHiddenAncestor(weaponNode);
        const RE::NiPoint3 cameraPosition = f4vr::getCameraPosition();
        ROCK_OMOD_DIAGNOSTIC_INFO(
            "OMOD-AUDIT begin run={} bodySetKey={:016X} weapon={:08X} '{}' bodies={} visualKeyNow={:016X} visualKeyAtBuild={:016X} drift={} visibleTriShapes={} nodes={} invisibleNodes={} rootVisible={} rootHiddenAncestor='{}' rootWorldT=({:.2f},{:.2f},{:.2f}) rootWorldScale={:.3f} cameraT=({:.2f},{:.2f},{:.2f})",
            runIndex,
            _cachedWeaponBodySetKey,
            weaponForm ? weaponForm->formID : 0u,
            weaponForm ? RE::TESFullName::GetFullName(*weaponForm) : std::string_view{},
            getWeaponBodyCount(),
            visualKeyNow,
            _cachedWeaponVisualKey,
            visualDrift ? "YES" : "no",
            visualStatsNow.visibleTriShapeCount,
            visualStatsNow.nodeCount,
            visualStatsNow.invisibleNodeCount,
            weaponVisualNodeVisible(weaponNode) ? "yes" : "no",
            rootHiddenAncestor ? safeNodeName(const_cast<RE::NiAVObject*>(rootHiddenAncestor)) : "none",
            weaponNode->world.translate.x,
            weaponNode->world.translate.y,
            weaponNode->world.translate.z,
            weaponNode->world.scale,
            cameraPosition.x,
            cameraPosition.y,
            cameraPosition.z);

        /*
         * Stored sourceNode pointers are compared by address during tree walks
         * only, never dereferenced directly, matching the discipline of the
         * build-time dump.
         */
        std::unordered_set<std::uintptr_t> evidenceSourceAddresses;
        std::unordered_map<std::uint32_t, std::uint32_t> bodiesByAttachPointFormId;
        const bool publishedBodyEvidenceCurrent = weapon_omod_audit_policy::publishedBodyEvidenceMatchesAudit(
            auditedEquippedKey,
            _cachedWeaponKey,
            hasWeaponBody() && _cachedWeaponBodySetKey != 0);
        if (publishedBodyEvidenceCurrent) {
            for (const auto& instance : activeWeaponBodies()) {
                if (!instance.body.isValid()) {
                    continue;
                }
                const bool durableAttachmentEvidence =
                    !weapon_generated_source_completeness_policy::isTransientReloadPart(instance.semantic.partKind);
                if (instance.sourceNode && durableAttachmentEvidence) {
                    evidenceSourceAddresses.insert(reinterpret_cast<std::uintptr_t>(instance.sourceNode));
                }
                if (instance.semantic.attachPointFormId != 0 && durableAttachmentEvidence) {
                    ++bodiesByAttachPointFormId[instance.semantic.attachPointFormId];
                }
            }
        } else if (hasWeaponBody()) {
            ROCK_OMOD_DIAGNOSTIC_INFO(
                "OMOD-AUDIT run={} ignoring published body evidence from a different equipped generation auditedKey={:016X} publishedKey={:016X} bodySetKey={:016X}",
                runIndex,
                auditedEquippedKey,
                _cachedWeaponKey,
                _cachedWeaponBodySetKey);
        }

        std::vector<OmodAuditRecord> records;
        const RE::BGSObjectInstanceExtra* objectInstanceExtra =
            weaponForm ? findEquippedWeaponObjectInstanceExtra(player, weaponForm, equippedInstanceData) : nullptr;
        if (objectInstanceExtra && objectInstanceExtra->values) {
            records.reserve(objectInstanceExtra->GetIndexData().size());
            visitEquippedOmodIndexData(objectInstanceExtra,
                [&](const auto& modIndex, auto* omod, const RE::BGSKeyword* attachPointKeyword) {
                OmodAuditRecord record{};
                record.modIndex = modIndex.index;
                record.rank = modIndex.rank;
                record.disabled = modIndex.disabled;
                // Unresolvable entries are still recorded, carrying the raw object
                // id: the audit reports coverage of what the engine believes is
                // installed, not only of what ROCK could look up.
                record.formId = modIndex.objectID;
                if (omod) {
                    record.resolved = true;
                    record.formId = omod->formID;
                    record.attachPointIndex = omod->attachPoint.keywordIndex;
                    record.attachPointFormId = attachPointKeyword ? attachPointKeyword->formID : 0u;
                    record.name = omod->fullName.c_str() ? omod->fullName.c_str() : "";
                    record.modelPath = omod->model.c_str() ? omod->model.c_str() : "";
                }
                records.push_back(std::move(record));
            });
        } else {
            ROCK_OMOD_DIAGNOSTIC_INFO("OMOD-AUDIT run={} no object instance extra available", runIndex);
        }

        std::unordered_set<std::uint32_t> activeOmodFormIds;
        activeOmodFormIds.reserve(records.size());
        for (const auto& record : records) {
            if (!record.disabled && record.formId != 0) {
                activeOmodFormIds.insert(record.formId);
            }
        }
        const std::size_t staleEnrichmentCount =
            removeStaleRockOmodEnrichmentContainers(weaponNode, activeOmodFormIds);
        if (staleEnrichmentCount != 0) {
            ROCK_OMOD_DIAGNOSTIC_INFO(
                "OMOD-AUDIT run={} removed {} stale ROCK-owned enrichment container(s); requesting collider rebuild",
                runIndex,
                staleEnrichmentCount);
            requestWorkbenchExitRebuild();
            result.sceneEnriched = true;
            return result;
        }

        std::vector<OmodAuditTokenSlot> tokenSlots(records.size());
        for (std::size_t i = 0; i < records.size(); ++i) {
            tokenSlots[i].lowerToken = makeOmodAuditModelToken(records[i].modelPath.c_str());
            tokenSlots[i].lowerWords = makeOmodAuditTokenWords(tokenSlots[i].lowerToken);
        }
        /*
         * Extra census slot: assembled weapon roots are named
         * 'Weapon  (<formID>)'. The 2026-07-04 session proved the game keeps
         * several parallel assembled instances (different addresses, identical
         * paths) that disagree about which OMOD subtrees exist, while the
         * renderer displays parts absent from the instances ROCK harvests.
         * Counting every instance across roots, with per-instance subtree
         * stats, identifies which copy is complete.
         */
        if (weaponForm) {
            tokenSlots.push_back(OmodAuditTokenSlot{ fmt::format("({:08x})", weaponForm->formID), {} });
        }
        std::vector<OmodAuditNodeMatch> connectPointMatches;

        struct OmodAuditRoot
        {
            const char* label;
            RE::NiAVObject* root;
            std::size_t maxVisited;
        };
        auto* playerNodes = f4vr::getPlayerNodes();
        std::vector<OmodAuditRoot> roots;
        roots.reserve(6);
        const auto addRoot = [&roots](const char* label, RE::NiAVObject* root, std::size_t maxVisited) {
            if (!root) {
                return;
            }
            for (const auto& existing : roots) {
                if (existing.root == root) {
                    return;
                }
            }
            roots.push_back(OmodAuditRoot{ label, root, maxVisited });
        };
        // Weapon-local roots stay on the shared dump budget; the skeleton and
        // full scene roots get a deep budget because the rendered weapon
        // instance may sit beyond 4096 nodes (cap saturation is logged below).
        constexpr std::size_t kOmodAuditDeepRootMaxVisited = 32768;
        // 2026-07-04 session 2 proved the renderer draws a weapon copy that is
        // in NEITHER instance reachable from the roots below (parts render
        // while absent, and the whole weapon can vanish while both instances
        // stay visible-flagged). The absolute scene root — reached by climbing
        // parents from the update weapon node to the top 'WorldRoot Node' —
        // covers everything parented into the loaded scene and gets a very
        // deep budget to find that copy.
        constexpr std::size_t kOmodAuditSceneRootMaxVisited = 262144;
        addRoot("updateWeaponNode", weaponNode, WEAPON_ANIM_NODE_DUMP_MAX_VISITED_NODES);
        addRoot("firstPersonSkeleton:Weapon", f4vr::getWeaponNode(), WEAPON_ANIM_NODE_DUMP_MAX_VISITED_NODES);
        addRoot("PlayerNodes.primaryWeapontoWeaponNode", playerNodes ? playerNodes->primaryWeapontoWeaponNode : nullptr, WEAPON_ANIM_NODE_DUMP_MAX_VISITED_NODES);
        addRoot("PlayerNodes.primaryWeaponOffsetNode", playerNodes ? playerNodes->primaryWeaponOffsetNOde : nullptr, WEAPON_ANIM_NODE_DUMP_MAX_VISITED_NODES);
        addRoot("PlayerNodes.playerworldnode", playerNodes ? playerNodes->playerworldnode : nullptr, kOmodAuditDeepRootMaxVisited);
        addRoot("PlayerNodes.roomnode", playerNodes ? playerNodes->roomnode : nullptr, WEAPON_ANIM_NODE_DUMP_MAX_VISITED_NODES);
        addRoot("firstPersonSkeleton", f4vr::getFirstPersonSkeleton(), kOmodAuditDeepRootMaxVisited);
        addRoot("playerFadeRootNode", f4vr::getWorldRootNode(), kOmodAuditDeepRootMaxVisited);
        addRoot("gameRootNode", f4vr::getRootNode(), kOmodAuditDeepRootMaxVisited);
        const auto climbToAbsoluteRoot = [](RE::NiAVObject* node) -> RE::NiAVObject* {
            if (!node) {
                return nullptr;
            }
            for (int hop = 0; hop < 64 && node->parent; ++hop) {
                node = node->parent;
            }
            return node;
        };
        RE::NiAVObject* absoluteSceneRoot = climbToAbsoluteRoot(weaponNode);
        addRoot("absoluteSceneRoot", absoluteSceneRoot, kOmodAuditSceneRootMaxVisited);
        /*
         * Scene-root topology probe (2026-07-04 session 3): the rendered
         * weapon copy is in NEITHER census instance and the WorldRoot-wide
         * scan never hits its cap, so the rendered copy must hang under a
         * sibling scene root. Climb from every player/camera anchor that can
         * live outside WorldRoot; addRoot dedup makes converging climbs free,
         * and the topology lines below prove which anchors share a graph.
         */
        addRoot("fpSkeletonAbsoluteRoot", climbToAbsoluteRoot(f4vr::getFirstPersonSkeleton()), kOmodAuditSceneRootMaxVisited);
        addRoot("playerWorldAbsoluteRoot", climbToAbsoluteRoot(playerNodes ? playerNodes->playerworldnode : nullptr), kOmodAuditSceneRootMaxVisited);
        auto* playerCamera = f4vr::getPlayerCamera();
        addRoot("cameraAbsoluteRoot", climbToAbsoluteRoot(playerCamera ? playerCamera->cameraRoot.get() : nullptr), kOmodAuditSceneRootMaxVisited);

        /*
         * Engine biped-slot ground truth (raw disasm 2026-07-04, two sources:
         * caller 0x1403f1e50 + builder 0x1401c8150): Actor vtbl+0x508 =
         * GetBiped(firstPerson), returns the ADDRESS of a refcounted
         * container member (container = *returned). Container: 44 slots,
         * stride 0x58, first slot at +0x10; per slot: item TESForm* +0x00,
         * instanceData +0x08, built weapon 3D NiPointer +0x30 — the node the
         * engine names 'Weapon %s (%08X)' (0x142c947e8), i.e. exactly what
         * the census matches. vtbl+0x458 = Get3D(firstPerson), the root the
         * builder attaches into. Read-only walk, per-hop gates fail closed
         * into logged skips; runs on the frame-update thread like the engine
         * call sites themselves.
         */
        const auto plausiblePointer = [](const void* pointer) {
            const auto value = reinterpret_cast<std::uintptr_t>(pointer);
            return value >= 0x10000 && (value & 7) == 0;
        };
        if (player && plausiblePointer(player)) {
            const auto* vtbl = *reinterpret_cast<std::uintptr_t* const*>(player);
            if (plausiblePointer(vtbl)) {
                using GetBipedFn = void** (*)(void*, bool);
                using Get3DFn = RE::NiAVObject* (*)(void*, bool);
                const auto getBiped = reinterpret_cast<GetBipedFn>(vtbl[0x508 / 8]);
                const auto get3D = reinterpret_cast<Get3DFn>(vtbl[0x458 / 8]);
                for (const bool firstPerson : { true, false }) {
                    const char* who = firstPerson ? "1st" : "3rd";
                    RE::NiAVObject* actor3D = get3D ? get3D(player, firstPerson) : nullptr;
                    RE::NiAVObject* actor3DRoot = climbToAbsoluteRoot(actor3D);
                    ROCK_OMOD_DIAGNOSTIC_INFO(
                        "OMOD-AUDIT biped probe person={} get3D={:x} name='{}' absRoot='{}'/{:x}",
                        who,
                        reinterpret_cast<std::uintptr_t>(actor3D),
                        actor3D ? safeNodeName(actor3D) : "null",
                        actor3DRoot ? safeNodeName(actor3DRoot) : "null",
                        reinterpret_cast<std::uintptr_t>(actor3DRoot));
                    addRoot(firstPerson ? "playerGet3D-1st" : "playerGet3D-3rd", actor3DRoot, kOmodAuditSceneRootMaxVisited);

                    void** bipedMember = getBiped ? getBiped(player, firstPerson) : nullptr;
                    void* container = bipedMember && plausiblePointer(bipedMember) ? *bipedMember : nullptr;
                    if (!container || !plausiblePointer(container)) {
                        ROCK_OMOD_DIAGNOSTIC_INFO("OMOD-AUDIT biped person={} container implausible member={:x} container={:x}",
                            who, reinterpret_cast<std::uintptr_t>(bipedMember), reinterpret_cast<std::uintptr_t>(container));
                        continue;
                    }
                    const int refCount = *reinterpret_cast<const int*>(container);
                    if (refCount <= 0 || refCount > 1000000) {
                        ROCK_OMOD_DIAGNOSTIC_INFO("OMOD-AUDIT biped person={} container={:x} refCount {} implausible - skipping",
                            who, reinterpret_cast<std::uintptr_t>(container), refCount);
                        continue;
                    }
                    const auto containerBase = reinterpret_cast<std::uintptr_t>(container);
                    for (std::uint32_t slot = 0; slot < 44; ++slot) {
                        const std::uintptr_t slotBase = containerBase + 0x10 + slot * 0x58;
                        auto* item = *reinterpret_cast<void* const*>(slotBase);
                        auto* instanceData = *reinterpret_cast<void* const*>(slotBase + 0x8);
                        auto* built3D = *reinterpret_cast<RE::NiAVObject* const*>(slotBase + 0x30);
                        if (!item && !built3D) {
                            continue;
                        }
                        const bool built3DPlausible = built3D && plausiblePointer(built3D) && plausiblePointer(*reinterpret_cast<void* const*>(built3D));
                        RE::NiAVObject* builtRoot = built3DPlausible ? climbToAbsoluteRoot(built3D) : nullptr;
                        ROCK_OMOD_DIAGNOSTIC_INFO(
                            "OMOD-AUDIT biped person={} container={:x} slot={} item={:x} itemIsEquippedWeapon={} instanceData={:x} built3D={:x} name='{}' absRoot='{}'/{:x}",
                            who,
                            containerBase,
                            slot,
                            reinterpret_cast<std::uintptr_t>(item),
                            item == static_cast<const void*>(weaponForm) ? "YES" : "no",
                            reinterpret_cast<std::uintptr_t>(instanceData),
                            reinterpret_cast<std::uintptr_t>(built3D),
                            built3DPlausible ? safeNodeName(built3D) : "implausible",
                            builtRoot ? safeNodeName(builtRoot) : "null",
                            reinterpret_cast<std::uintptr_t>(builtRoot));
                        if (built3DPlausible && item == static_cast<const void*>(weaponForm)) {
                            addRoot(firstPerson ? "bipedWeapon3D-1st" : "bipedWeapon3D-3rd", built3D, WEAPON_ANIM_NODE_DUMP_MAX_VISITED_NODES);
                            addRoot(firstPerson ? "bipedWeapon3DRoot-1st" : "bipedWeapon3DRoot-3rd", builtRoot, kOmodAuditSceneRootMaxVisited);
                        }
                    }
                }
            }
        }

        auto* fpWeaponNode = f4vr::getWeaponNode();
        ROCK_OMOD_DIAGNOSTIC_INFO(
            "OMOD-AUDIT topology run={} getWeaponNode={:x} fpSkeleton={:x} cameraNode={:x}",
            runIndex,
            reinterpret_cast<std::uintptr_t>(fpWeaponNode),
            reinterpret_cast<std::uintptr_t>(f4vr::getFirstPersonSkeleton()),
            reinterpret_cast<std::uintptr_t>(playerCamera ? playerCamera->cameraRoot.get() : nullptr));
        for (const auto& root : roots) {
            std::uint32_t depth = 0;
            for (const RE::NiAVObject* node = root.root; node && node->parent && depth < 64; node = node->parent) {
                ++depth;
            }
            RE::NiAVObject* absRoot = climbToAbsoluteRoot(root.root);
            ROCK_OMOD_DIAGNOSTIC_INFO(
                "OMOD-AUDIT topology root='{}' addr={:x} name='{}' depth={} absRoot='{}' absAddr={:x}",
                root.label,
                reinterpret_cast<std::uintptr_t>(root.root),
                safeNodeName(root.root),
                depth,
                safeNodeName(absRoot),
                reinterpret_cast<std::uintptr_t>(absRoot));
        }

        for (const auto& root : roots) {
            std::size_t visited = 0;
            scanOmodAuditTreeRecursive(root.root, 0, visited, root.maxVisited, root.label, tokenSlots, connectPointMatches);
            ROCK_OMOD_DIAGNOSTIC_INFO(
                "OMOD-AUDIT scan root='{}' addr={:x} visitedNodes={} capHit={}",
                root.label,
                reinterpret_cast<std::uintptr_t>(root.root),
                visited,
                visited >= root.maxVisited ? "YES" : "no");
        }

        // Phase 2 - classify: which installed OMODs look uncovered, and why.
        const std::vector<std::size_t> selfHealCandidates = classifyOmodAuditSelfHealCandidates(
            records,
            tokenSlots,
            bodiesByAttachPointFormId,
            evidenceSourceAddresses);

        for (const auto& match : connectPointMatches) {
            const auto stats = summarizeWeaponAnimNodeSubtree(match.node);
            std::size_t evidenceVisited = 0;
            const std::size_t evidenceSources =
                countOmodAuditEvidenceSourcesInSubtree(match.node, evidenceSourceAddresses, evidenceVisited);
            ROCK_OMOD_DIAGNOSTIC_INFO(
                "OMOD-AUDIT pnode name='{}' root='{}' path='{}' addr={:x} visible={} subtreeNodes={} triShapes={} visibleTriShapes={} hiddenFlags={} appCulledNodes={} evidenceSources={} childNames='{}'",
                safeNodeName(match.node),
                match.rootLabel,
                buildOmodAuditNodePath(match.node),
                reinterpret_cast<std::uintptr_t>(match.node),
                weaponVisualNodeVisible(match.node) ? "yes" : "no",
                stats.nodeCount,
                stats.triShapeCount,
                stats.visibleTriShapeCount,
                stats.hiddenFlagCount,
                stats.appCulledCount,
                evidenceSources,
                weaponAnimNodeImmediateChildNames(match.node));
        }

        std::size_t weaponInstanceCount = 0;
        if (weaponForm && tokenSlots.size() > records.size()) {
            const auto& instanceSlot = tokenSlots.back();
            weaponInstanceCount = instanceSlot.matches.size();
            for (const auto& match : instanceSlot.matches) {
                const auto stats = summarizeWeaponAnimNodeSubtree(match.node);
                std::size_t evidenceVisited = 0;
                const std::size_t evidenceSources =
                    countOmodAuditEvidenceSourcesInSubtree(match.node, evidenceSourceAddresses, evidenceVisited);
                const RE::NiAVObject* hiddenAncestor = findOmodAuditHiddenAncestor(match.node);
                ROCK_OMOD_DIAGNOSTIC_INFO(
                    "OMOD-AUDIT instance name='{}' root='{}' path='{}' addr={:x} visible={} flags=0x{:X} appCulled={} hiddenAncestor='{}' worldT=({:.2f},{:.2f},{:.2f}) worldScale={:.3f} localScale={:.3f} subtreeNodes={} triShapes={} visibleTriShapes={} hiddenFlags={} appCulledNodes={} evidenceSources={} childNames='{}'",
                    safeNodeName(match.node),
                    match.rootLabel,
                    buildOmodAuditNodePath(match.node),
                    reinterpret_cast<std::uintptr_t>(match.node),
                    weaponVisualNodeVisible(match.node) ? "yes" : "no",
                    static_cast<std::uint32_t>(match.node->flags.flags),
                    match.node->GetAppCulled() ? "yes" : "no",
                    hiddenAncestor ? safeNodeName(const_cast<RE::NiAVObject*>(hiddenAncestor)) : "none",
                    match.node->world.translate.x,
                    match.node->world.translate.y,
                    match.node->world.translate.z,
                    match.node->world.scale,
                    match.node->local.scale,
                    stats.nodeCount,
                    stats.triShapeCount,
                    stats.visibleTriShapeCount,
                    stats.hiddenFlagCount,
                    stats.appCulledCount,
                    evidenceSources,
                    weaponAnimNodeImmediateChildNames(match.node));
            }
        }

        /*
         * Flattened-bone-tree pass. Actor skeleton roots are BSFlattenedBoneTree
         * objects whose bones live in a flat transforms array, not as NiNode
         * children — a part that exists only as a flattened entry (plus skinned
         * render geometry) renders on screen while every child walk above
         * misses it. Session 3 (2026-07-04) proved the full scene graph holds
         * only the two incomplete instances, so this array is the remaining
         * candidate for where the rendered copy of a missing part lives.
         */
        std::size_t flatMatchCount = 0;
        struct OmodAuditFlatRoot
        {
            const char* label;
            f4vr::BSFlattenedBoneTree* tree;
        };
        const std::array<OmodAuditFlatRoot, 2> flatRoots{ {
            { "gameFlattenedBoneTree", f4vr::getFlattenedBoneTree() },
            { "firstPersonBoneTree", f4vr::getFirstPersonBoneTree() },
        } };
        constexpr std::size_t OMOD_AUDIT_MAX_FLAT_MATCHES = 48;
        for (const auto& flatRoot : flatRoots) {
            if (!weaponAnimFlattenedTreeValid(flatRoot.tree)) {
                continue;
            }
            for (int index = 0; index < flatRoot.tree->numTransforms && flatMatchCount < OMOD_AUDIT_MAX_FLAT_MATCHES; ++index) {
                const auto& transform = flatRoot.tree->transforms[index];
                const char* boneName = transform.name.c_str();
                auto* refNode = transform.refNode;
                const char* refNodeName = safeNodeName(refNode);
                bool matched = omodAuditNameIsConnectPoint(boneName) || omodAuditNameIsConnectPoint(refNodeName);
                if (!matched) {
                    for (const auto& slot : tokenSlots) {
                        if (slot.lowerToken.empty()) {
                            continue;
                        }
                        if (omodAuditNameContainsToken(boneName, slot.lowerToken) ||
                            omodAuditNameContainsToken(refNodeName, slot.lowerToken)) {
                            matched = true;
                            break;
                        }
                    }
                }
                if (!matched) {
                    continue;
                }
                ++flatMatchCount;
                ROCK_OMOD_DIAGNOSTIC_INFO(
                    "OMOD-AUDIT flatbone root='{}' index={} name='{}' parentIndex={} parentName='{}' refNode={:x} refNodeName='{}' refParent='{}' refVisible={} worldT=({:.2f},{:.2f},{:.2f})",
                    flatRoot.label,
                    index,
                    boneName && boneName[0] != '\0' ? boneName : "(unnamed)",
                    transform.parPos,
                    weaponAnimFlattenedParentName(flatRoot.tree, transform.parPos),
                    reinterpret_cast<std::uintptr_t>(refNode),
                    refNodeName,
                    refNode ? safeNodeName(refNode->parent) : "(null)",
                    refNode && weaponVisualNodeVisible(refNode) ? "yes" : "no",
                    transform.world.translate.x,
                    transform.world.translate.y,
                    transform.world.translate.z);
            }
        }

        /*
         * Mandatory production self-heal: reattach missing OMOD models
         * with the engine's own primitive. Ghidra-verified (raw disasm +
         * decompiler + address database, refreshed 2026-07-20):
         *   bool BGSMod::Attachment::Mod::TryAttach3DRecurse(
         *       NiNode* root, char* rankSuffix, TBO_InstanceData* instData)
         * at VR offset 0x2D9140. It demands the mod's model, deep-clones it
         * (scale from the root's REFR), applies material swaps with the
         * instance data, and attaches at the mod NIF's declared connect point.
         * Its bool reports that a BSConnectPoint::Parents record matched, not
         * that Parent::ConnectChild found the declared scene parent or that
         * geometry entered the requested subtree. Break Action Laser is the
         * concrete failure: P-Barrel names CROSSBarrelOffsetNode, which the
         * assembled receiver drops. ROCK therefore reconstructs only that
         * authored static parent path before calling the native routine and
         * verifies actual durable geometry below the owned path afterward.
         * Successful geometry requests a collider rebuild. NODE_NOT_FOUND is
         * the trigger, so the word-set
         * matcher above must keep false negatives rare - a heal on a part that
         * exists under an unmatchable name would duplicate its geometry, which
         * the once-per-instance-address guard bounds to a single attempt.
         */
        // Phase 3 - self-heal: re-attach the geometry the engine dropped.
        const OmodSelfHealOutcome healOutcome = runOmodAuditSelfHeal(
            runIndex,
            weaponForm,
            records,
            tokenSlots,
            selfHealCandidates,
            equippedInstanceData,
            _omodSelfHealAttempted);
        if (healOutcome.sceneEnriched) {
            // The assembled weapon changed under the published bodies, so the
            // collider set has to be rebuilt. The request is one atomic bit,
            // consumed by update().
            requestWorkbenchExitRebuild();
            result.sceneEnriched = true;
        }

        ROCK_OMOD_DIAGNOSTIC_INFO(
            "OMOD-AUDIT end run={} bodySetKey={:016X} installedMods={} connectPoints={} weaponInstances={} flatMatches={} healCandidates={} healAttempted={} healed={}",
            runIndex,
            _cachedWeaponBodySetKey,
            records.size(),
            connectPointMatches.size(),
            weaponInstanceCount,
            flatMatchCount,
            selfHealCandidates.size(),
            healOutcome.attempted,
            healOutcome.healed);
        return result;
    }
}

#undef ROCK_OMOD_DIAGNOSTIC_INFO
