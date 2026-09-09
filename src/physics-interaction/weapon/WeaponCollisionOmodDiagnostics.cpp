#include "physics-interaction/weapon/WeaponCollision.h"
#include "physics-interaction/weapon/WeaponCollisionInternal.h"

#include "physics-interaction/actor/ActorEquipmentGrab.h"
#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/native/HavokCompoundShapeBuilder.h"
#include "physics-interaction/native/HavokConvexShapeBuilder.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/NativeNiNodeFactory.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "RockConfig.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/ManualScopeTargetPolicy.h"
#include "physics-interaction/weapon/WeaponAccessoryPartKindPolicy.h"
#include "physics-interaction/weapon/WeaponClassificationPolicy.h"
#include "physics-interaction/weapon/WeaponEffectGeometryPolicy.h"
#include "physics-interaction/weapon/WeaponEmitterPolicy.h"
#include "physics-interaction/weapon/WeaponPartRecordIdentityPolicy.h"
#include "physics-interaction/weapon/WeaponSemantics.h"
#include "physics-interaction/weapon/WeaponTypePolicy.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/TransformMath.h"

#include <intrin.h>

#include "RE/Bethesda/BGSMod.h"
#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/MagicItems.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Havok/hkReferencedObject.h"
#include "RE/Havok/hknpCapsuleShape.h"
#include "RE/Havok/hknpMotion.h"

#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <string_view>
#include <vector>

namespace rock
{
    using namespace weapon_collision_internal;

    namespace
    {
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
        if (_identity.cachedBodySetKey == 0 || _identity.cachedBodySetKey == _diagnostics.lastOmodDumpGenerationKey) {
            return;
        }
        _diagnostics.lastOmodDumpGenerationKey = _identity.cachedBodySetKey;

        auto* player = f4vr::getPlayer();
        auto* equipData = f4vr::getEquippedWeaponItem();
        auto* weaponForm = equipData ? equipData->item.object : nullptr;
        auto* equippedInstanceData = equipData ? equipData->item.instanceData.get() : nullptr;
        ROCK_LOG_INFO(Weapon,
            "OMOD-DUMP begin generation={:016X} weapon={:08X} '{}'",
            _identity.cachedBodySetKey,
            weaponForm ? weaponForm->formID : 0u,
            weaponForm ? RE::TESFullName::GetFullName(*weaponForm) : std::string_view{});

        const RE::BGSObjectInstanceExtra* objectInstanceExtra =
            weaponForm ? findEquippedWeaponObjectInstanceExtra(player, weaponForm, equippedInstanceData) : nullptr;
        if (objectInstanceExtra && objectInstanceExtra->values) {
            const auto indexData = objectInstanceExtra->GetIndexData();
            ROCK_LOG_INFO(Weapon, "OMOD-DUMP installed mods count={}", indexData.size());
            for (const auto& modIndex : indexData) {
                auto* omod = RE::TESForm::GetFormByID<RE::BGSMod::Attachment::Mod>(modIndex.objectID);
                if (!omod) {
                    ROCK_LOG_INFO(Weapon,
                        "OMOD-DUMP mod objectID={:08X} index={} rank={} disabled={} UNRESOLVED",
                        modIndex.objectID,
                        modIndex.index,
                        modIndex.rank,
                        modIndex.disabled);
                    continue;
                }

                const std::uint16_t attachPointIndex = omod->attachPoint.keywordIndex;
                const RE::BGSKeyword* attachPointKeyword =
                    RE::BGSKeyword::GetTypedKeywordByIndex(RE::KeywordType::kAttachPoint, attachPointIndex);
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
            }
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
}
