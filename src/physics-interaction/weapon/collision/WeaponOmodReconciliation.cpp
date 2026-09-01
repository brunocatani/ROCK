#include "physics-interaction/weapon/WeaponCollisionInternal.h"

// OMOD reconciliation: detects attachment changes on the equipped weapon and reconciles generated collision against the live modded model.

namespace rock
{
    namespace
    {
        constexpr std::size_t OMOD_AUDIT_MAX_LOGGED_MATCHES_PER_OMOD = 3;

        using OmodAuditNodeMatch = weapon_omod_scene_scan::NodeMatch;
        using OmodAuditTokenSlot = weapon_omod_scene_scan::TokenSlot;

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
         * Per-node visibility checks miss renders hidden by an ANCESTOR: a
         * culled/hidden/zero-scale parent (hand bone, skeleton root) hides the
         * whole weapon while every weapon node still reports visible=yes. The
         * post-workbench "weapon invisible" investigation needs the first
         * offending ancestor named explicitly.
         */
        const RE::NiAVObject* findOmodAuditHiddenAncestor(const RE::NiAVObject* node)
        {
            for (const RE::NiAVObject* cursor = node ? node->parent : nullptr; cursor; cursor = cursor->parent) {
                if ((cursor->flags.flags & 1) != 0 || cursor->GetAppCulled() || cursor->local.scale == 0.0f) {
                    return cursor;
                }
            }
            return nullptr;
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

        void collectOmodPhysicalTemplateSignatureRecursive(
            RE::NiAVObject* node,
            OmodPhysicalTemplateSignature& signature,
            std::size_t& visited,
            const int depth = 0)
        {
            if (!node || depth > 16 || visited >= 512 || signature.meshNames.size() >= 96) {
                return;
            }
            ++visited;

            if (auto* triShape = node->IsTriShape()) {
                if (classifyGeneratedWeaponEffectGeometry(triShape) != weapon_effect_geometry_policy::ExclusionReason::None) {
                    return;
                }

                const char* rawName = node->name.c_str();
                if (!rawName || rawName[0] == '\0') {
                    return;
                }
                const auto duplicate = std::find_if(signature.meshNames.begin(), signature.meshNames.end(), [rawName](const std::string& existing) {
                    return _stricmp(existing.c_str(), rawName) == 0;
                });
                if (duplicate == signature.meshNames.end()) {
                    signature.meshNames.emplace_back(rawName);
                }

                std::uint32_t triangleCount = 0;
                if (native_memory::tryReadField(triShape, VROffset::numTriangles, triangleCount) &&
                    triangleCount > signature.durableAnchorTriangles) {
                    signature.durableAnchorTriangles = triangleCount;
                    signature.durableAnchorName = rawName;
                }
                return;
            }

            auto* niNode = node->IsNode();
            if (!niNode) {
                return;
            }
            const auto& children = niNode->children;
            for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                collectOmodPhysicalTemplateSignatureRecursive(children[i].get(), signature, visited, depth + 1);
            }
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

        bool templateContainsNativeCollisionObjectRecursive(
            RE::NiAVObject* object,
            std::size_t& visited,
            const int depth = 0)
        {
            if (!object || depth > 16 || visited >= 512) {
                return false;
            }
            ++visited;

            if (auto* collisionObject = object->collisionObject.get();
                collisionObject && niObjectRttiChainContains(collisionObject, "bhkNPCollisionObject")) {
                return true;
            }

            auto* node = object->IsNode();
            if (!node) {
                return false;
            }
            const auto& children = node->children;
            for (auto index = decltype(children.size()){ 0 }; index < children.size(); ++index) {
                if (templateContainsNativeCollisionObjectRecursive(children[index].get(), visited, depth + 1)) {
                    return true;
                }
            }
            return false;
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

        void collectRockOmodEnrichmentContainersRecursive(
            RE::NiAVObject* node,
            std::vector<RE::NiNode*>& outContainers,
            std::size_t& visited,
            const int depth = 0)
        {
            if (!node || depth > 24 || visited >= WEAPON_ANIM_NODE_DUMP_MAX_SUBTREE_NODES) {
                return;
            }
            ++visited;

            auto* niNode = node->IsNode();
            if (!niNode) {
                return;
            }
            std::uint32_t formId = 0;
            const char* rawName = niNode->name.c_str();
            if (rawName && tryParseRockOmodEnrichmentFormId(rawName, formId)) {
                outContainers.push_back(niNode);
                return;
            }

            const auto& children = niNode->children;
            for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                collectRockOmodEnrichmentContainersRecursive(children[i].get(), outContainers, visited, depth + 1);
            }
        }

        std::vector<RE::NiNode*> findStaleRockOmodEnrichmentContainers(
            RE::NiAVObject* weaponRoot,
            const std::unordered_set<std::uint32_t>& activeOmodFormIds)
        {
            std::vector<RE::NiNode*> containers;
            containers.reserve(8);
            std::size_t visited = 0;
            collectRockOmodEnrichmentContainersRecursive(weaponRoot, containers, visited);

            std::vector<RE::NiNode*> staleContainers;
            staleContainers.reserve(containers.size());
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
                staleContainers.push_back(container);
            }
            return staleContainers;
        }

        std::size_t removeStaleRockOmodEnrichmentContainers(
            const std::vector<RE::NiNode*>& staleContainers)
        {
            std::size_t removed = 0;
            for (auto* container : staleContainers) {
                std::uint32_t formId = 0;
                const char* rawName = container ? container->name.c_str() : nullptr;
                auto* parent = container && container->parent ? container->parent->IsNode() : nullptr;
                if (!rawName || !parent ||
                    !tryParseRockOmodEnrichmentFormId(rawName, formId)) {
                    continue;
                }
                RE::NiPointer<RE::NiAVObject> detached;
                parent->DetachChild(container, detached);
                if (!detached) {
                    continue;
                }
                f4vr::updateTransformsDown(parent, true);
                ++removed;
                ROCK_LOG_INFO(Weapon,
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

        bool findAuthoredConnectPointParentRecursive(
            RE::NiAVObject* object,
            const RE::BSFixedString& cpaKey,
            const char* targetConnectPointName,
            AuthoredConnectPointParentMatch& outMatch,
            std::size_t& visited,
            const int depth = 0)
        {
            constexpr std::uint32_t kMaxParentRecords = 64;
            if (!object || !targetConnectPointName || depth > 16 || visited >= 512) {
                return false;
            }
            ++visited;

            auto* node = object->IsNode();
            if (!node) {
                return false;
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
                            return true;
                        }
                    }
                }
            }

            const auto& children = node->children;
            for (auto index = decltype(children.size()){ 0 }; index < children.size(); ++index) {
                if (findAuthoredConnectPointParentRecursive(
                        children[index].get(), cpaKey, targetConnectPointName, outMatch, visited, depth + 1)) {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] bool finiteAuthoredNodeTransform(const RE::NiTransform& transform) noexcept
        {
            if (!std::isfinite(transform.translate.x) || !std::isfinite(transform.translate.y) ||
                !std::isfinite(transform.translate.z) || !std::isfinite(transform.scale) ||
                std::abs(transform.scale) <= 0.0001f) {
                return false;
            }
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(transform.rotate.entry[row][column])) {
                        return false;
                    }
                }
            }
            return true;
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
                if (!finiteAuthoredNodeTransform(sourceNode->local)) {
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

        RE::BSTriShape* findTemplatePhysicalShapeByNameRecursive(
            RE::NiAVObject* node,
            const char* targetName,
            std::size_t& visited,
            const int depth = 0)
        {
            if (!node || !targetName || depth > 16 || visited >= 512) {
                return nullptr;
            }
            ++visited;
            if (auto* triShape = node->IsTriShape()) {
                const char* rawName = node->name.c_str();
                if (rawName && _stricmp(rawName, targetName) == 0 &&
                    classifyGeneratedWeaponEffectGeometry(triShape) == weapon_effect_geometry_policy::ExclusionReason::None) {
                    return triShape;
                }
                return nullptr;
            }
            auto* niNode = node->IsNode();
            if (!niNode) {
                return nullptr;
            }
            const auto& children = niNode->children;
            for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                if (auto* match = findTemplatePhysicalShapeByNameRecursive(children[i].get(), targetName, visited, depth + 1)) {
                    return match;
                }
            }
            return nullptr;
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

    /*
     * Reconcile installed OMOD records against the assembled scene graph before
     * collider capture and once after publication. Self-heal always owns these
     * functional passes. bDebugWeaponOmodCoverageAudit independently enables
     * verbose evidence logging and repeated diagnostic passes.
     */
    WeaponCollision::OmodReconciliationResult WeaponCollision::maybeRunWeaponOmodReconciliation(
        RE::NiAVObject* weaponNode, std::uint64_t auditedEquippedKey, bool forceBeforeInitialBuild)
    {
        OmodReconciliationResult result{};
        const bool emitCoverageDiagnostics = g_rockConfig.rockDebugWeaponOmodCoverageAudit;
        if (!weaponNode || auditedEquippedKey == 0 ||
            (!forceBeforeInitialBuild && (!hasWeaponBody() || _cachedWeaponBodySetKey == 0))) {
            return result;
        }

        if (!forceBeforeInitialBuild && _omodReconciliationBodySetKey != _cachedWeaponBodySetKey) {
            _omodReconciliationBodySetKey = _cachedWeaponBodySetKey;
            _omodReconciliationFrameCounter = 0;
            _omodReconciliationRunIndex = 0;
        }

        const int diagnosticIntervalFrames =
            (std::max)(30, g_rockConfig.rockDebugWeaponOmodCoverageAuditIntervalFrames);
        constexpr int kPostBuildSelfHealCheckFrames = 90;
        // Self-heal owns one fixed post-build check. Coverage diagnostics may
        // continue at their configured interval after that functional pass.
        if (!forceBeforeInitialBuild && !emitCoverageDiagnostics &&
            _omodReconciliationRunIndex != 0) {
            return result;
        }
        const int dueFrames = _omodReconciliationRunIndex == 0 ?
            (emitCoverageDiagnostics ?
                    (std::min)(kPostBuildSelfHealCheckFrames, diagnosticIntervalFrames) :
                    kPostBuildSelfHealCheckFrames) :
            diagnosticIntervalFrames;
        if (!forceBeforeInitialBuild && ++_omodReconciliationFrameCounter < dueFrames) {
            return result;
        }
        _omodReconciliationFrameCounter = 0;
        const std::uint32_t runIndex = _omodReconciliationRunIndex++;
        result.ran = true;

        auto* player = f4vr::getPlayer();
        auto* equipData = f4vr::getEquippedWeaponItem();
        auto* weaponForm = equipData ? equipData->item.object : nullptr;
        auto* equippedInstanceData = equipData ? equipData->item.instanceData.get() : nullptr;

        if (emitCoverageDiagnostics) {
            WeaponVisualKeyStats visualStatsNow{};
            const std::uint64_t visualKeyNow =
                getWeaponVisualCompositionKey(weaponNode, visualStatsNow);
            const bool visualDrift = visualKeyNow != 0 &&
                _cachedWeaponVisualKey != 0 &&
                visualKeyNow != _cachedWeaponVisualKey;
            const RE::NiAVObject* rootHiddenAncestor =
                findOmodAuditHiddenAncestor(weaponNode);
            const RE::NiPoint3 cameraPosition = f4vr::getCameraPosition();
            ROCK_LOG_INFO(Weapon,
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
        }

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
        } else if (emitCoverageDiagnostics && hasWeaponBody()) {
            ROCK_LOG_INFO(Weapon,
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
            const auto indexData = objectInstanceExtra->GetIndexData();
            records.reserve(indexData.size());
            for (const auto& modIndex : indexData) {
                OmodAuditRecord record{};
                record.modIndex = modIndex.index;
                record.rank = modIndex.rank;
                record.disabled = modIndex.disabled;
                record.formId = modIndex.objectID;
                if (auto* omod = RE::TESForm::GetFormByID<RE::BGSMod::Attachment::Mod>(modIndex.objectID)) {
                    record.resolved = true;
                    record.formId = omod->formID;
                    record.attachPointIndex = omod->attachPoint.keywordIndex;
                    const RE::BGSKeyword* attachPointKeyword =
                        RE::BGSKeyword::GetTypedKeywordByIndex(RE::KeywordType::kAttachPoint, record.attachPointIndex);
                    record.attachPointFormId = attachPointKeyword ? attachPointKeyword->formID : 0u;
                    record.name = omod->fullName.c_str() ? omod->fullName.c_str() : "";
                    record.modelPath = omod->model.c_str() ? omod->model.c_str() : "";
                }
                records.push_back(std::move(record));
            }
        } else if (emitCoverageDiagnostics) {
            ROCK_LOG_INFO(Weapon, "OMOD-AUDIT run={} no object instance extra available", runIndex);
        }

        std::unordered_set<std::uint32_t> activeOmodFormIds;
        activeOmodFormIds.reserve(records.size());
        for (const auto& record : records) {
            if (!record.disabled && record.formId != 0) {
                activeOmodFormIds.insert(record.formId);
            }
        }
        const auto staleEnrichmentContainers =
            findStaleRockOmodEnrichmentContainers(weaponNode, activeOmodFormIds);
        if (!staleEnrichmentContainers.empty()) {
            retireActiveWeaponBodiesForSceneTransition(
                _cachedWorld,
                "omod-stale-enrichment-removal");
            const std::size_t staleEnrichmentCount =
                removeStaleRockOmodEnrichmentContainers(staleEnrichmentContainers);
            ROCK_LOG_INFO(Weapon,
                "OMOD-HEAL run={} removed {} stale ROCK-owned enrichment container(s); requesting collider rebuild",
                runIndex,
                staleEnrichmentCount);
            requestWorkbenchExitRebuild();
            result.sceneEnriched = true;
            return result;
        }

        std::vector<OmodAuditTokenSlot> tokenSlots(records.size());
        for (std::size_t i = 0; i < records.size(); ++i) {
            tokenSlots[i].lowerToken =
                weapon_omod_scene_scan::makeModelToken(
                    records[i].modelPath.c_str());
            tokenSlots[i].lowerWords =
                weapon_omod_scene_scan::makeTokenWords(
                    tokenSlots[i].lowerToken);
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
        roots.reserve(16);
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
        // Functional reconciliation uses exactly the visual roots consumed by
        // generated collider capture. The expensive scene-wide census remains
        // available only through the explicit coverage diagnostic.
        constexpr std::size_t kOmodAuditDeepRootMaxVisited = 32768;
        constexpr std::size_t kOmodAuditSceneRootMaxVisited = 262144;
        visitGeneratedWeaponMeshRootCandidates(
            weaponNode,
            [&](const WeaponMeshRootCandidate& candidate) {
                addRoot(
                    candidate.label,
                    candidate.root,
                    WEAPON_ANIM_NODE_DUMP_MAX_VISITED_NODES);
            });

        const auto climbToAbsoluteRoot = [](RE::NiAVObject* node) -> RE::NiAVObject* {
            if (!node) {
                return nullptr;
            }
            for (int hop = 0; hop < 64 && node->parent; ++hop) {
                node = node->parent;
            }
            return node;
        };
        auto* playerCamera = f4vr::getPlayerCamera();
        if (emitCoverageDiagnostics) {
            addRoot("PlayerNodes.playerworldnode", playerNodes ? playerNodes->playerworldnode : nullptr, kOmodAuditDeepRootMaxVisited);
            addRoot("PlayerNodes.roomnode", playerNodes ? playerNodes->roomnode : nullptr, WEAPON_ANIM_NODE_DUMP_MAX_VISITED_NODES);
            addRoot("firstPersonSkeleton", f4vr::getFirstPersonSkeleton(), kOmodAuditDeepRootMaxVisited);
            addRoot("playerFadeRootNode", f4vr::getWorldRootNode(), kOmodAuditDeepRootMaxVisited);
            addRoot("gameRootNode", f4vr::getRootNode(), kOmodAuditDeepRootMaxVisited);
            addRoot("absoluteSceneRoot", climbToAbsoluteRoot(weaponNode), kOmodAuditSceneRootMaxVisited);
            addRoot("fpSkeletonAbsoluteRoot", climbToAbsoluteRoot(f4vr::getFirstPersonSkeleton()), kOmodAuditSceneRootMaxVisited);
            addRoot("playerWorldAbsoluteRoot", climbToAbsoluteRoot(playerNodes ? playerNodes->playerworldnode : nullptr), kOmodAuditSceneRootMaxVisited);
            addRoot("cameraAbsoluteRoot", climbToAbsoluteRoot(playerCamera ? playerCamera->cameraRoot.get() : nullptr), kOmodAuditSceneRootMaxVisited);
        }

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
        if (emitCoverageDiagnostics && player && plausiblePointer(player)) {
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
                    if (emitCoverageDiagnostics) {
                        ROCK_LOG_INFO(Weapon,
                            "OMOD-AUDIT biped probe person={} get3D={:x} name='{}' absRoot='{}'/{:x}",
                            who,
                            reinterpret_cast<std::uintptr_t>(actor3D),
                            actor3D ? safeNodeName(actor3D) : "null",
                            actor3DRoot ? safeNodeName(actor3DRoot) : "null",
                            reinterpret_cast<std::uintptr_t>(actor3DRoot));
                    }
                    addRoot(firstPerson ? "playerGet3D-1st" : "playerGet3D-3rd", actor3DRoot, kOmodAuditSceneRootMaxVisited);

                    void** bipedMember = getBiped ? getBiped(player, firstPerson) : nullptr;
                    void* container = bipedMember && plausiblePointer(bipedMember) ? *bipedMember : nullptr;
                    if (!container || !plausiblePointer(container)) {
                        if (emitCoverageDiagnostics) {
                            ROCK_LOG_INFO(Weapon, "OMOD-AUDIT biped person={} container implausible member={:x} container={:x}",
                                who, reinterpret_cast<std::uintptr_t>(bipedMember), reinterpret_cast<std::uintptr_t>(container));
                        }
                        continue;
                    }
                    const int refCount = *reinterpret_cast<const int*>(container);
                    if (refCount <= 0 || refCount > 1000000) {
                        if (emitCoverageDiagnostics) {
                            ROCK_LOG_INFO(Weapon, "OMOD-AUDIT biped person={} container={:x} refCount {} implausible - skipping",
                                who, reinterpret_cast<std::uintptr_t>(container), refCount);
                        }
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
                        if (emitCoverageDiagnostics) {
                            ROCK_LOG_INFO(Weapon,
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
                        }
                        if (built3DPlausible && item == static_cast<const void*>(weaponForm)) {
                            addRoot(firstPerson ? "bipedWeapon3D-1st" : "bipedWeapon3D-3rd", built3D, WEAPON_ANIM_NODE_DUMP_MAX_VISITED_NODES);
                            addRoot(firstPerson ? "bipedWeapon3DRoot-1st" : "bipedWeapon3DRoot-3rd", builtRoot, kOmodAuditSceneRootMaxVisited);
                        }
                    }
                }
            }
        }

        auto* fpWeaponNode = f4vr::getWeaponNode();
        if (emitCoverageDiagnostics) {
            ROCK_LOG_INFO(Weapon,
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
                ROCK_LOG_INFO(Weapon,
                    "OMOD-AUDIT topology root='{}' addr={:x} name='{}' depth={} absRoot='{}' absAddr={:x}",
                    root.label,
                    reinterpret_cast<std::uintptr_t>(root.root),
                    safeNodeName(root.root),
                    depth,
                    safeNodeName(absRoot),
                    reinterpret_cast<std::uintptr_t>(absRoot));
            }
        }

        std::unordered_set<std::uintptr_t> visitedNodeAddresses;
        visitedNodeAddresses.reserve(8192);
        for (const auto& root : roots) {
            std::size_t visited = 0;
            weapon_omod_scene_scan::scanTree(
                root.root,
                root.maxVisited,
                WEAPON_ANIM_NODE_DUMP_MAX_DEPTH,
                root.label,
                visitedNodeAddresses,
                tokenSlots,
                connectPointMatches,
                visited);
            if (emitCoverageDiagnostics) {
                ROCK_LOG_INFO(Weapon,
                    "OMOD-AUDIT scan root='{}' addr={:x} visitedNodes={} capHit={}",
                    root.label,
                    reinterpret_cast<std::uintptr_t>(root.root),
                    visited,
                    visited >= root.maxVisited ? "YES" : "no");
            }
        }

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
                if (emitCoverageDiagnostics &&
                    loggedMatches < OMOD_AUDIT_MAX_LOGGED_MATCHES_PER_OMOD) {
                    ++loggedMatches;
                    ROCK_LOG_INFO(Weapon,
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
            if (emitCoverageDiagnostics) {
                const char* verdict =
                    weapon_omod_audit_policy::coverageVerdictName(coverageDecision.verdict);
                ROCK_LOG_INFO(Weapon,
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
        }

        if (emitCoverageDiagnostics) {
            for (const auto& match : connectPointMatches) {
                const auto stats = summarizeWeaponAnimNodeSubtree(match.node);
                std::size_t evidenceVisited = 0;
                const std::size_t evidenceSources =
                    countOmodAuditEvidenceSourcesInSubtree(match.node, evidenceSourceAddresses, evidenceVisited);
                ROCK_LOG_INFO(Weapon,
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
        }

        std::size_t weaponInstanceCount = 0;
        if (weaponForm && tokenSlots.size() > records.size()) {
            const auto& instanceSlot = tokenSlots.back();
            weaponInstanceCount = instanceSlot.matches.size();
            if (emitCoverageDiagnostics) {
                for (const auto& match : instanceSlot.matches) {
                    const auto stats = summarizeWeaponAnimNodeSubtree(match.node);
                    std::size_t evidenceVisited = 0;
                    const std::size_t evidenceSources =
                        countOmodAuditEvidenceSourcesInSubtree(match.node, evidenceSourceAddresses, evidenceVisited);
                    const RE::NiAVObject* hiddenAncestor = findOmodAuditHiddenAncestor(match.node);
                    ROCK_LOG_INFO(Weapon,
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
        if (emitCoverageDiagnostics) {
            for (const auto& flatRoot : flatRoots) {
                if (!weaponAnimFlattenedTreeValid(flatRoot.tree)) {
                    continue;
                }
                for (int index = 0; index < flatRoot.tree->numTransforms && flatMatchCount < OMOD_AUDIT_MAX_FLAT_MATCHES; ++index) {
                    const auto& transform = flatRoot.tree->transforms[index];
                    const char* boneName = transform.name.c_str();
                    auto* refNode = transform.refNode;
                    const char* refNodeName = safeNodeName(refNode);
                    bool matched =
                        weapon_omod_scene_scan::nameIsConnectPoint(boneName) ||
                        weapon_omod_scene_scan::nameIsConnectPoint(refNodeName);
                    if (!matched) {
                        for (const auto& slot : tokenSlots) {
                            if (slot.lowerToken.empty()) {
                                continue;
                            }
                            if (weapon_omod_scene_scan::nameContainsToken(
                                    boneName,
                                    slot.lowerToken) ||
                                weapon_omod_scene_scan::nameContainsToken(
                                    refNodeName,
                                    slot.lowerToken)) {
                                matched = true;
                                break;
                            }
                        }
                    }
                    if (!matched) {
                        continue;
                    }
                    ++flatMatchCount;
                    ROCK_LOG_INFO(Weapon,
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
        }

        /*
         * Functional self-heal: reattach missing OMOD models
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
        std::size_t selfHealAttemptCount = 0;
        std::size_t selfHealSuccessCount = 0;
        bool activeSceneSourceBankRetired = false;
        if (!selfHealCandidates.empty()) {
            // Functional repair targets the exact update root whose visible
            // geometry feeds collider capture. The token census remains a
            // fallback for unusual wrappers where that root is not a NiNode.
            RE::NiNode* healTargetNode = weaponNode ? weaponNode->IsNode() : nullptr;
            const char* healTargetRootLabel = healTargetNode ? "updateWeaponNode" : "";
            if (!healTargetNode && weaponForm && tokenSlots.size() > records.size()) {
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
                constexpr std::size_t OMOD_SELF_HEAL_MAX_PER_RECONCILIATION = 4;

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
                                ROCK_LOG_INFO(Weapon,
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

                if (_omodSelfHealAttempted.size() > 256) {
                    _omodSelfHealAttempted.clear();
                }
                for (const std::size_t candidateIndex : orderedSelfHealCandidates) {
                    if (selfHealAttemptCount >= OMOD_SELF_HEAL_MAX_PER_RECONCILIATION) {
                        break;
                    }
                    const auto& record = records[candidateIndex];
                    const std::uint64_t attemptKey =
                        reinterpret_cast<std::uintptr_t>(healTargetNode) ^ (static_cast<std::uint64_t>(record.formId) << 20);
                    if (_omodSelfHealAttempted.contains(attemptKey)) {
                        continue;
                    }

                    auto* omod = RE::TESForm::GetFormByID<RE::BGSMod::Attachment::Mod>(record.formId);
                    if (!omod) {
                        _omodSelfHealAttempted.insert(attemptKey);
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
                        _omodSelfHealAttempted.insert(attemptKey);
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
                        _omodSelfHealAttempted.insert(attemptKey);
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
                        _omodSelfHealAttempted.insert(attemptKey);
                        ROCK_LOG_INFO(Weapon,
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

                    ROCK_LOG_INFO(Weapon,
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
                    if (!activeSceneSourceBankRetired) {
                        retireActiveWeaponBodiesForSceneTransition(
                            _cachedWorld,
                            "omod-scene-enrichment");
                        activeSceneSourceBankRetired = true;
                    }
                    _omodSelfHealAttempted.insert(attemptKey);
                    ++selfHealAttemptCount;
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
                    selfHealSuccessCount += geometryAdded ? 1 : 0;

                    ROCK_LOG_INFO(Weapon,
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

                if (selfHealSuccessCount > 0) {
                    ROCK_LOG_INFO(Weapon,
                        "OMOD-HEAL run={} healed={} of {} attempted - requesting collider rebuild",
                        runIndex,
                        selfHealSuccessCount,
                        selfHealAttemptCount);
                    requestWorkbenchExitRebuild();
                    result.sceneEnriched = true;
                }
            }
        }

        if (emitCoverageDiagnostics) {
            ROCK_LOG_INFO(Weapon,
                "OMOD-AUDIT end run={} bodySetKey={:016X} installedMods={} connectPoints={} weaponInstances={} flatMatches={} healCandidates={} healAttempted={} healed={}",
                runIndex,
                _cachedWeaponBodySetKey,
                records.size(),
                connectPointMatches.size(),
                weaponInstanceCount,
                flatMatchCount,
                selfHealCandidates.size(),
                selfHealAttemptCount,
                selfHealSuccessCount);
        }
        return result;
    }
}
