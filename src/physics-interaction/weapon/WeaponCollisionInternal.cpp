#include "physics-interaction/weapon/WeaponCollisionInternal.h"

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/weapon/WeaponGeometry.h"

#include <intrin.h>

#include "RE/Bethesda/BGSMod.h"
#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Havok/hkReferencedObject.h"
#include "RE/NetImmerse/NiNode.h"

#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_set>
#include <utility>

namespace rock::weapon_collision_detail
{
    namespace
    {
        [[nodiscard]] bool generatedWeaponShapeHasEffectShaderProperty(const RE::BSTriShape* triShape)
        {
            if (!triShape) {
                return false;
            }
            for (const auto& property : triShape->GetRuntimeData().properties) {
                if (niObjectRttiChainContains(property.get(), "BSEffectShaderProperty")) {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] bool generatedWeaponShapeHasBillboardAncestor(const RE::NiAVObject* node)
        {
            // Parent links are frame-scoped engine references; nothing from this walk is retained.
            for (auto* ancestor = node ? node->parent : nullptr; ancestor; ancestor = ancestor->parent) {
                if (niObjectRttiChainContains(ancestor, "NiBillboardNode")) {
                    return true;
                }
            }
            return false;
        }

        RE::NiPointer<RE::NiNode> loadOmodModelTemplate(
            const std::string& modelPath,
            const std::uint8_t modelDemandFlags)
        {
            if (modelPath.empty()) {
                return nullptr;
            }

            std::string resourcePath;
            if (startsWithAsciiInsensitive(modelPath, "Data\\") || startsWithAsciiInsensitive(modelPath, "Data/")) {
                resourcePath = modelPath;
            } else if (startsWithAsciiInsensitive(modelPath, "Meshes\\") || startsWithAsciiInsensitive(modelPath, "Meshes/")) {
                resourcePath = "Data/" + modelPath;
            } else {
                resourcePath = "Data/Meshes/" + modelPath;
            }

            std::uint64_t loadFlags[2]{ 0, modelDemandFlags };
            std::uint64_t loadedRoot = 0;
            const int result = f4vr::loadNif(
                reinterpret_cast<std::uint64_t>(resourcePath.c_str()),
                reinterpret_cast<std::uint64_t>(&loadedRoot),
                reinterpret_cast<std::uint64_t>(&loadFlags));
            if (result != 0 || loadedRoot == 0) {
                return nullptr;
            }

            RE::NiPointer<RE::NiNode> root;
            root.reset(reinterpret_cast<RE::NiNode*>(loadedRoot));
            return root;
        }
    }

    const char* safeNodeName(const RE::NiAVObject* node)
    {
        if (!node) {
            return "(null)";
        }
        const char* name = node->name.c_str();
        return name ? name : "(null)";
    }

    [[nodiscard]] bool niObjectRttiChainContains(const RE::NiObject* object, const char* typeName)
    {
        const RE::NiRTTI* rtti = object && typeName ? object->GetRTTI() : nullptr;
        for (int depth = 0; rtti && depth < 16; ++depth, rtti = rtti->GetBaseRTTI()) {
            const char* rttiName = rtti->GetName();
            if (rttiName && std::strcmp(rttiName, typeName) == 0) {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] weapon_effect_geometry_policy::ExclusionReason classifyGeneratedWeaponEffectGeometry(
        const RE::BSTriShape* triShape)
    {
        return weapon_effect_geometry_policy::classify({
            .hasEffectShaderProperty = generatedWeaponShapeHasEffectShaderProperty(triShape),
            .hasBillboardAncestor = generatedWeaponShapeHasBillboardAncestor(triShape),
            .geometryName = safeNodeName(triShape),
        });
    }

    bool weaponVisualNodeVisible(const RE::NiAVObject* node)
    {
        if (!node) {
            return false;
        }
        return (node->flags.flags & 1) == 0 && !node->GetAppCulled() && node->local.scale != 0.0f;
    }

    [[nodiscard]] WeaponVisibilityWalk walkWeaponVisibilityChain(const RE::NiAVObject* node, int maxSteps)
    {
        WeaponVisibilityWalk result{};
        const RE::NiAVObject* cursor = node;
        for (int step = 0;; ++step) {
            // Budget first, so a chain that is exactly maxSteps long still
            // reports exhaustion rather than silently passing.
            if (step >= maxSteps) {
                result.boundExhausted = true;
                return result;
            }
            if (!cursor) {
                return result;
            }
            if (!weaponVisualNodeVisible(cursor)) {
                result.firstHidden = cursor;
                return result;
            }
            cursor = cursor->parent;
        }
    }

    // Every point that reaches a Havok body, a bounds test or a hash has to be
    // finite first; a single NaN poisons a whole hull or a whole key.
    [[nodiscard]] bool pointFinite(const RE::NiPoint3& point) noexcept
    {
        return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    }

    /*
     * The finiteness gate for every weapon-space transform in this file.
     *
     * Scale participates on purpose: consumers either compose the transform
     * (rotation and scale multiply through) or invert it to bring a world point
     * into node space, and a zero or near-zero scale makes the inverse explode
     * into infinities that then look like valid geometry. Rejecting it here is
     * the fail-closed choice - a caller that cannot resolve a transform skips
     * the frame, which is always safer than driving a body from garbage.
     */
    [[nodiscard]] bool weaponTransformFinite(const RE::NiTransform& transform) noexcept
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

    constexpr std::size_t kMaximumWeaponLocalHierarchyDepth = 64;

    [[nodiscard]] bool tryResolveDescendantLocalTransform(
        const RE::NiAVObject* ancestor,
        const RE::NiAVObject* descendant,
        RE::NiTransform& outDescendantLocal)
    {
        /*
         * Descendant world transforms can belong to a different scene-graph
         * propagation epoch than the current weapon root. Compose the
         * bounded parent path so callers receive one coherent local frame.
         * Shoulder draw can also leave that coherent frame in presentation
         * space; post-undraw callers remove that separately with a validated
         * source-frame correction.
         */
        outDescendantLocal = transform_math::makeIdentityTransform<RE::NiTransform>();
        if (!ancestor || !descendant) {
            return false;
        }

        std::array<const RE::NiAVObject*, kMaximumWeaponLocalHierarchyDepth> reversePath{};
        std::size_t pathLength = 0;
        auto* cursor = descendant;
        while (cursor && cursor != ancestor) {
            if (pathLength >= reversePath.size() || !weaponTransformFinite(cursor->local)) {
                outDescendantLocal = {};
                return false;
            }
            reversePath[pathLength++] = cursor;
            cursor = cursor->parent;
        }
        if (cursor != ancestor) {
            outDescendantLocal = {};
            return false;
        }

        while (pathLength != 0) {
            outDescendantLocal = transform_math::composeTransforms(
                outDescendantLocal,
                reversePath[--pathLength]->local);
        }
        if (!weaponTransformFinite(outDescendantLocal)) {
            outDescendantLocal = {};
            return false;
        }
        return true;
    }

    [[nodiscard]] bool tryResolveDescendantWorldTransform(
        const RE::NiAVObject* ancestor,
        const RE::NiTransform& ancestorWorld,
        const RE::NiAVObject* descendant,
        RE::NiTransform& outDescendantWorld)
    {
        RE::NiTransform descendantLocal{};
        if (!weaponTransformFinite(ancestorWorld) ||
            !tryResolveDescendantLocalTransform(ancestor, descendant, descendantLocal)) {
            outDescendantWorld = {};
            return false;
        }

        outDescendantWorld = transform_math::composeTransforms(ancestorWorld, descendantLocal);
        if (!weaponTransformFinite(outDescendantWorld)) {
            outDescendantWorld = {};
            return false;
        }
        return true;
    }

    QuantizedPointKey quantizePoint(const RE::NiPoint3& point, float grid)
    {
        const float safeGrid = (std::max)(grid, 0.0001f);
        return QuantizedPointKey{ static_cast<std::int64_t>(std::llround(point.x / safeGrid)), static_cast<std::int64_t>(std::llround(point.y / safeGrid)),
            static_cast<std::int64_t>(std::llround(point.z / safeGrid)) };
    }

    void mixWeaponVisualKey(std::uint64_t& key, std::uint64_t value)
    {
        weapon_visual_composition_policy::mixValue(key, value);
    }

    void mixWeaponVisualString(std::uint64_t& key, const char* value)
    {
        if (!value) {
            return;
        }
        weapon_visual_composition_policy::mixString(key, value);
    }

    std::vector<RE::NiPoint3> dedupePointCloud(const std::vector<RE::NiPoint3>& points, float grid)
    {
        std::vector<RE::NiPoint3> unique;
        unique.reserve(points.size());
        std::unordered_set<QuantizedPointKey, QuantizedPointKeyHash> seen;
        seen.reserve(points.size());

        for (const auto& point : points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                continue;
            }

            const auto key = quantizePoint(point, grid);
            if (seen.insert(key).second) {
                unique.push_back(point);
            }
        }

        return unique;
    }

    float pointCloudDiagonalSquared(const std::vector<RE::NiPoint3>& points)
    {
        if (points.empty()) {
            return 0.0f;
        }

        RE::NiPoint3 minPoint = points.front();
        RE::NiPoint3 maxPoint = points.front();
        for (const auto& point : points) {
            minPoint = weapon_collision_geometry_math::pointMin(minPoint, point);
            maxPoint = weapon_collision_geometry_math::pointMax(maxPoint, point);
        }

        const float dx = maxPoint.x - minPoint.x;
        const float dy = maxPoint.y - minPoint.y;
        const float dz = maxPoint.z - minPoint.z;
        return dx * dx + dy * dy + dz * dz;
    }

    bool pointCloudCanBuildHull(const std::vector<RE::NiPoint3>& points, float sourceScale)
    {
        return points.size() >= 4 && weapon_collision_geometry_math::scaledHullDiagonalCanBuild(
                                         pointCloudDiagonalSquared(points), sourceScale, MIN_HULL_DIAGONAL_GAME_UNITS);
    }

    PointCloudBounds pointCloudBounds(const std::vector<RE::NiPoint3>& points)
    {
        PointCloudBounds bounds{};
        if (points.empty()) {
            return bounds;
        }

        bounds.min = points.front();
        bounds.max = points.front();
        for (const auto& point : points) {
            bounds.min = weapon_collision_geometry_math::pointMin(bounds.min, point);
            bounds.max = weapon_collision_geometry_math::pointMax(bounds.max, point);
        }
        return bounds;
    }

    std::array<float, 3> pointToArray(const RE::NiPoint3& point)
    {
        return { point.x, point.y, point.z };
    }

    std::vector<RE::NiPoint3> makeCenteredHavokPointCloud(const std::vector<RE::NiPoint3>& localPointsGame, const RE::NiPoint3& localCenterGame, float sourceScale)
    {
        std::vector<RE::NiPoint3> result;
        result.reserve(localPointsGame.size());
        const float scaledHavokScale = sourceScale * gameToHavokScale();
        for (const auto& point : localPointsGame) {
            result.emplace_back((point.x - localCenterGame.x) * scaledHavokScale, (point.y - localCenterGame.y) * scaledHavokScale,
                (point.z - localCenterGame.z) * scaledHavokScale);
        }
        return result;
    }

    void shapeRemoveRef(const RE::hknpShape* shape)
    {
        if (!shape)
            return;
        auto* refCountDword = reinterpret_cast<volatile long*>(const_cast<char*>(reinterpret_cast<const char*>(shape)) + 0x08);
        for (;;) {
            long oldVal = *refCountDword;
            std::uint16_t rc = static_cast<std::uint16_t>(oldVal & 0xFFFF);
            if (rc == 0xFFFF || rc == 0)
                return;
            long newVal = (oldVal & static_cast<long>(0xFFFF0000u)) | static_cast<long>(static_cast<std::uint16_t>(rc - 1));
            if (_InterlockedCompareExchange(refCountDword, newVal, oldVal) == oldVal)
                return;
        }
    }

    const RE::BGSObjectInstanceExtra* findEquippedWeaponObjectInstanceExtra(
        const RE::PlayerCharacter* player,
        const RE::TESForm* weaponForm,
        const RE::TBO_InstanceData* instanceData)
    {
        if (!player || !weaponForm) {
            return nullptr;
        }

        auto scanBiped = [&](const RE::BipedAnim* biped) -> const RE::BGSObjectInstanceExtra* {
            if (!biped) {
                return nullptr;
            }
            for (std::uint32_t slotIndex = 0;
                 slotIndex < static_cast<std::uint32_t>(std::to_underlying(RE::BIPED_OBJECT::kTotal));
                 ++slotIndex) {
                const auto& slot = biped->object[slotIndex];
                if (slot.parent.object != weaponForm) {
                    continue;
                }
                if (instanceData && slot.parent.instanceData && slot.parent.instanceData.get() != instanceData) {
                    continue;
                }
                if (slot.modExtra) {
                    return slot.modExtra;
                }
            }
            return nullptr;
        };

        if (const auto* firstPersonExtra = scanBiped(player->firstPersonBipedAnim.get())) {
            return firstPersonExtra;
        }
        return scanBiped(player->biped.get());
    }

    // Prefix companion to weapon_effect_geometry_policy::containsAsciiInsensitive,
    // sharing its foldAscii so path and name matching cannot diverge in what
    // counts as "the same letter".
    [[nodiscard]] bool startsWithAsciiInsensitive(std::string_view value, std::string_view prefix)
    {
        return value.size() >= prefix.size() &&
            weapon_effect_geometry_policy::equalsAsciiInsensitive(value.substr(0, prefix.size()), prefix);
    }

    /*
     * BSModelDB's ordinary OMOD demand uses flag 0x2D and may return only
     * the currently selected controller branch. Loading through the same
     * native entry with 0xED preserves the complete model hierarchy. This
     * is required to see durable housings which are absent from the active
     * branch (the SR-25 magazine shell is the concrete witness).
     */
    RE::NiPointer<RE::NiNode> loadCompleteOmodModelTemplate(const std::string& modelPath)
    {
        return loadOmodModelTemplate(modelPath, 0xED);
    }

    /*
     * Fallout4VR.exe 1.2.72 uses BSModelDB flag 0x20 in the geometry-query
     * path at 0x1402824B0. Unlike an ordinary attachment demand, this path
     * does not run the 0x08 model postprocessor which can consume display
     * geometry owned by a bhkNPCollisionObject. It is used only as a
     * read/clone template after the guarded receiver-specific comparison
     * below; native attachment continues to use the engine's own 0x2D
     * path.
     */
    RE::NiPointer<RE::NiNode> loadGeometryInspectionOmodModelTemplate(const std::string& modelPath)
    {
        return loadOmodModelTemplate(modelPath, 0x20);
    }

    // Materialized form of the visitor above, for the callers that have to keep
    // the candidate list alive past the scan. The order and the identity of the
    // candidates come from the visitor - never add a root here.
    std::vector<WeaponMeshRootCandidate> makeGeneratedWeaponMeshRootCandidates(RE::NiAVObject* updateWeaponNode)
    {
        std::vector<WeaponMeshRootCandidate> candidates;
        candidates.reserve(4);
        visitGeneratedWeaponMeshRootCandidates(updateWeaponNode, [&](const WeaponMeshRootCandidate& candidate) {
            candidates.push_back(candidate);
        });
        return candidates;
    }

    std::uint64_t makeWeaponEmitterRootSetKey(RE::NiAVObject* updateWeaponNode)
    {
        std::uint64_t key = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
        visitGeneratedWeaponMeshRootCandidates(updateWeaponNode, [&](const WeaponMeshRootCandidate& candidate) {
            mixWeaponVisualKey(key, reinterpret_cast<std::uintptr_t>(candidate.root));
        });
        return key;
    }

    /*
     * sourceScale re-bakes a source NiNode's own NiTransform::scale into
     * the point cloud before Havok conversion. It must be 1.0 for points
     * already expressed in a frame with no scale divided out (e.g.
     * weapon-root-local localPointsGame under a scale=1.0 weapon root);
     * pass the captured GeneratedHullSource::sourceNodeScale for points
     * expressed in a source node's own local space
     * (sourceLocalPointsGame), since Havok never re-applies NiNode scale
     * to a built shape at runtime.
     */

    std::unordered_map<std::uint32_t, std::uint32_t> readEquippedOmodsByAttachPointFormId(
        WeaponCollision::WeaponCompositionSnapshot* outComposition)
    {
        std::unordered_map<std::uint32_t, std::uint32_t> result;
        if (outComposition) {
            *outComposition = {};
        }
        auto* player = f4vr::getPlayer();
        auto* equipData = f4vr::getEquippedWeaponItem();
        auto* weaponForm = equipData ? equipData->item.object : nullptr;
        auto* instanceData = equipData ? equipData->item.instanceData.get() : nullptr;
        const RE::BGSObjectInstanceExtra* objectInstanceExtra =
            weaponForm ? findEquippedWeaponObjectInstanceExtra(player, weaponForm, instanceData) : nullptr;
        if (!objectInstanceExtra || !objectInstanceExtra->values) {
            return result;
        }

        result.reserve(objectInstanceExtra->GetIndexData().size());
        // stableIndex is the install-order position and is what downstream
        // consumers key on, so it advances for every entry - including the
        // disabled and unresolvable ones the map itself skips.
        std::uint32_t stableIndex = 0;
        visitEquippedOmodIndexData(objectInstanceExtra,
            [&](const auto& modIndex, auto* omod, const RE::BGSKeyword* attachPointKeyword) {
                if (!modIndex.disabled && attachPointKeyword && omod) {
                    result.emplace(attachPointKeyword->formID, omod->formID);
                }
                if (outComposition &&
                    outComposition->entryCount <
                        outComposition->entries.size()) {
                    auto& entry = outComposition->entries[
                        outComposition->entryCount++];
                    // An unresolvable Mod still gets an entry, carrying the raw
                    // object id, so the snapshot shows the real install list.
                    entry.omodFormId = omod ? omod->formID : modIndex.objectID;
                    entry.attachPointFormId =
                        attachPointKeyword ? attachPointKeyword->formID : 0;
                    entry.stableIndex = stableIndex;
                    entry.flags = modIndex.disabled ? (1u << 1) : (1u << 0);
                    if (attachPointKeyword) {
                        entry.flags |= 1u << 2;
                    }
                }
                ++stableIndex;
            });
        return result;
    }
}
