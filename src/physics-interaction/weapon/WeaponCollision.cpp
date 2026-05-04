#include "physics-interaction/weapon/WeaponCollision.h"

#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/native/HavokCompoundShapeBuilder.h"
#include "physics-interaction/native/HavokConvexShapeBuilder.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "RockConfig.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponSemantics.h"
#include "physics-interaction/weapon/WeaponAuthority.h"

#include <intrin.h>

#include "RE/Havok/hkReferencedObject.h"
#include "RE/Havok/hknpCapsuleShape.h"
#include "RE/Havok/hknpMotion.h"

#include "f4vr/PlayerNodes.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <limits>
#include <unordered_set>
#include <string>
#include <string_view>
#include <vector>

namespace rock
{
    namespace
    {
        constexpr std::size_t MAX_CONVEX_HULL_POINTS = 0xFC;
        constexpr float MIN_HULL_DIAGONAL_GAME_UNITS = 0.5f;

        struct QuantizedPointKey
        {
            std::int64_t x = 0;
            std::int64_t y = 0;
            std::int64_t z = 0;

            bool operator==(const QuantizedPointKey& rhs) const noexcept { return x == rhs.x && y == rhs.y && z == rhs.z; }
        };

        struct QuantizedPointKeyHash
        {
            std::size_t operator()(const QuantizedPointKey& key) const noexcept
            {
                const auto hx = std::hash<std::int64_t>{}(key.x);
                const auto hy = std::hash<std::int64_t>{}(key.y);
                const auto hz = std::hash<std::int64_t>{}(key.z);
                return hx ^ (hy + 0x9e3779b97f4a7c15ull + (hx << 6) + (hx >> 2)) ^ (hz + 0x9e3779b97f4a7c15ull + (hy << 6) + (hy >> 2));
            }
        };

        struct WeaponMeshRootCandidate
        {
            RE::NiAVObject* root = nullptr;
            const char* label = "";
        };

        struct PointCloudBounds
        {
            RE::NiPoint3 min{};
            RE::NiPoint3 max{};
        };

        struct SemanticPartPointCloud
        {
            RE::NiAVObject* driveRoot = nullptr;
            RE::NiAVObject* semanticRoot = nullptr;
            std::string sourceName;
            WeaponPartClassification semantic{};
            std::vector<RE::NiPoint3> localPointsGame;
        };

        enum GeneratedHullCoverageClass : int
        {
            HullCoverageStock = 0,
            HullCoverageReceiver = 1,
            HullCoverageBarrel = 2,
            HullCoverageMagazine = 3,
            HullCoverageTopAccessory = 4,
            HullCoverageAction = 5,
            HullCoverageOther = 6,
            HullCoverageCosmeticAmmo = 7
        };

        struct GeneratedHullCoverageInfo
        {
            int coverageClass = HullCoverageOther;
            int priority = 50;
            bool cosmetic = false;
            const char* label = "other";
        };

        float matrixDeterminant(const RE::NiMatrix3& matrix)
        {
            return matrix.entry[0][0] * (matrix.entry[1][1] * matrix.entry[2][2] - matrix.entry[1][2] * matrix.entry[2][1]) -
                matrix.entry[0][1] * (matrix.entry[1][0] * matrix.entry[2][2] - matrix.entry[1][2] * matrix.entry[2][0]) +
                matrix.entry[0][2] * (matrix.entry[1][0] * matrix.entry[2][1] - matrix.entry[1][1] * matrix.entry[2][0]);
        }

        float bodyBasisDeterminant(const float* bodyFloats)
        {
            const float x0 = bodyFloats[0];
            const float x1 = bodyFloats[1];
            const float x2 = bodyFloats[2];
            const float y0 = bodyFloats[4];
            const float y1 = bodyFloats[5];
            const float y2 = bodyFloats[6];
            const float z0 = bodyFloats[8];
            const float z1 = bodyFloats[9];
            const float z2 = bodyFloats[10];

            return x0 * (y1 * z2 - y2 * z1) - y0 * (x1 * z2 - x2 * z1) + z0 * (x1 * y2 - x2 * y1);
        }

        RE::NiTransform makeIdentityTransform()
        {
            RE::NiTransform result{};
            result.rotate.entry[0][0] = 1.0f;
            result.rotate.entry[1][1] = 1.0f;
            result.rotate.entry[2][2] = 1.0f;
            result.scale = 1.0f;
            return result;
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

        bool pointCloudCanBuildHull(const std::vector<RE::NiPoint3>& points)
        {
            return points.size() >= 4 && pointCloudDiagonalSquared(points) >= (MIN_HULL_DIAGONAL_GAME_UNITS * MIN_HULL_DIAGONAL_GAME_UNITS);
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

        bool sourceNameContains(std::string_view haystack, std::string_view needle)
        {
            if (needle.empty() || haystack.size() < needle.size()) {
                return false;
            }

            for (std::size_t start = 0; start <= haystack.size() - needle.size(); ++start) {
                bool matched = true;
                for (std::size_t i = 0; i < needle.size(); ++i) {
                    const auto lhs = static_cast<unsigned char>(haystack[start + i]);
                    const auto rhs = static_cast<unsigned char>(needle[i]);
                    if (std::tolower(lhs) != std::tolower(rhs)) {
                        matched = false;
                        break;
                    }
                }
                if (matched) {
                    return true;
                }
            }
            return false;
        }

        GeneratedHullCoverageInfo classifyGeneratedHull(std::string_view sourceName)
        {
            /*
             * Generated firearm collision must spend its limited body budget on
             * coverage, not triangle density. The log showed dense barrel chunks
             * crowding out stock, magazine, action, and top geometry, then a single
             * overflow hull combined unrelated leftovers. HIGGS uses the weapon
             * visual tree as one coherent source; for FO4VR firearms this selector
             * keeps that same package-level intent while capping bodies.
             */
            const auto semantic = classifyWeaponPartName(sourceName);
            switch (semantic.partKind) {
            case WeaponPartKind::Stock:
            case WeaponPartKind::Grip:
                return { HullCoverageStock, semantic.priority, semantic.cosmetic, "stock/grip" };
            case WeaponPartKind::Receiver:
                return { HullCoverageReceiver, semantic.priority, semantic.cosmetic, "receiver/body" };
            case WeaponPartKind::Barrel:
            case WeaponPartKind::Handguard:
            case WeaponPartKind::Foregrip:
            case WeaponPartKind::Pump:
                return { HullCoverageBarrel, semantic.priority, semantic.cosmetic, "barrel/support" };
            case WeaponPartKind::Magazine:
            case WeaponPartKind::Magwell:
                return { HullCoverageMagazine, semantic.priority, semantic.cosmetic, "magazine/socket" };
            case WeaponPartKind::Sight:
            case WeaponPartKind::Accessory:
                return { HullCoverageTopAccessory, semantic.priority, semantic.cosmetic, "top/accessory" };
            case WeaponPartKind::Bolt:
            case WeaponPartKind::Slide:
            case WeaponPartKind::ChargingHandle:
            case WeaponPartKind::BreakAction:
            case WeaponPartKind::Cylinder:
            case WeaponPartKind::Chamber:
            case WeaponPartKind::LaserCell:
            case WeaponPartKind::Lever:
                return { HullCoverageAction, semantic.priority, semantic.cosmetic, "action/reload" };
            case WeaponPartKind::Shell:
            case WeaponPartKind::Round:
            case WeaponPartKind::CosmeticAmmo:
                return { HullCoverageCosmeticAmmo, semantic.priority, true, "cosmetic-ammo" };
            case WeaponPartKind::Other:
            default:
                return { HullCoverageOther, semantic.priority, semantic.cosmetic, "other" };
            }
        }

        weapon_collision_geometry_math::HullSelectionInput makeHullSelectionInput(const RE::NiPoint3& localCenterGame, const RE::NiPoint3& localMinGame,
            const RE::NiPoint3& localMaxGame, std::size_t pointCount, std::string_view sourceName)
        {
            const auto coverage = classifyGeneratedHull(sourceName);
            return weapon_collision_geometry_math::HullSelectionInput{
                pointToArray(localCenterGame),
                pointToArray(localMinGame),
                pointToArray(localMaxGame),
                pointCount,
                coverage.coverageClass,
                coverage.priority,
                coverage.cosmetic
            };
        }

        const char* hullAxisName(int axis)
        {
            switch (axis) {
            case 0:
                return "X";
            case 1:
                return "Y";
            default:
                return "Z";
            }
        }

        void addUniqueWeaponMeshRootCandidate(std::vector<WeaponMeshRootCandidate>& candidates, RE::NiAVObject* root, const char* label)
        {
            if (!root) {
                return;
            }
            for (const auto& candidate : candidates) {
                if (candidate.root == root) {
                    return;
                }
            }
            candidates.push_back(WeaponMeshRootCandidate{ root, label });
        }

        std::vector<WeaponMeshRootCandidate> makeGeneratedWeaponMeshRootCandidates(RE::NiAVObject* updateWeaponNode)
        {
            /*
             * Weapon mesh collision has to be rooted on the visual weapon tree, not
             * the native collision attachment tree. HIGGS extracts weapon triangles
             * from the first-person weapon node; ROCK scans several possible visual
             * roots, but every generated candidate must prove itself by
             * producing visible triangles before it is used for Havok body creation.
             */
            std::vector<WeaponMeshRootCandidate> candidates;
            candidates.reserve(6);

            addUniqueWeaponMeshRootCandidate(candidates, f4vr::getWeaponNode(), "firstPersonSkeleton:Weapon");

            if (auto* playerNodes = f4vr::getPlayerNodes()) {
                addUniqueWeaponMeshRootCandidate(candidates, playerNodes->primaryWeapontoWeaponNode, "PlayerNodes.primaryWeapontoWeaponNode");
                addUniqueWeaponMeshRootCandidate(candidates, playerNodes->primaryWeaponOffsetNOde, "PlayerNodes.primaryWeaponOffsetNode");
                addUniqueWeaponMeshRootCandidate(candidates, playerNodes->primaryMeleeWeaponOffsetNode, "PlayerNodes.primaryMeleeWeaponOffsetNode");
            }

            addUniqueWeaponMeshRootCandidate(candidates, updateWeaponNode, "updateWeaponNode");
            return candidates;
        }

        std::vector<RE::NiPoint3> makeCenteredHavokPointCloud(const std::vector<RE::NiPoint3>& localPointsGame, const RE::NiPoint3& localCenterGame)
        {
            std::vector<RE::NiPoint3> result;
            result.reserve(localPointsGame.size());
            for (const auto& point : localPointsGame) {
                result.emplace_back((point.x - localCenterGame.x) * gameToHavokScale(), (point.y - localCenterGame.y) * gameToHavokScale(),
                    (point.z - localCenterGame.z) * gameToHavokScale());
            }
            return result;
        }

        const char* safeNodeName(RE::NiAVObject* node)
        {
            if (!node) {
                return "(null)";
            }
            const char* name = node->name.c_str();
            return name ? name : "(null)";
        }

        bool weaponVisualNodeVisible(const RE::NiAVObject* node)
        {
            if (!node) {
                return false;
            }
            return (node->flags.flags & 1) == 0 && !node->GetAppCulled() && node->local.scale != 0.0f;
        }

        std::uintptr_t readRendererChildPointer(void* rendererData, std::ptrdiff_t rendererChildOffset)
        {
            if (!rendererData) {
                return 0;
            }
            auto* child = *reinterpret_cast<void**>(reinterpret_cast<char*>(rendererData) + rendererChildOffset);
            if (!child) {
                return 0;
            }
            return reinterpret_cast<std::uintptr_t>(*reinterpret_cast<void**>(reinterpret_cast<char*>(child) + 0x08));
        }

        weapon_visual_composition_policy::VisualRecord makeWeaponVisualRecord(
            RE::NiAVObject* node,
            RE::NiAVObject* parent,
            std::uint32_t childIndex,
            std::uint32_t childCount,
            std::uint32_t depth,
            bool visible)
        {
            weapon_visual_composition_policy::VisualRecord record{
                .nodeAddress = reinterpret_cast<std::uintptr_t>(node),
                .parentAddress = reinterpret_cast<std::uintptr_t>(parent ? parent : node ? node->parent : nullptr),
                .name = safeNodeName(node),
                .depth = depth,
                .childIndex = childIndex,
                .childCount = childCount,
                .visible = visible,
                .triShape = node && node->IsTriShape(),
            };

            if (auto* triShape = node ? node->IsTriShape() : nullptr) {
                auto* base = reinterpret_cast<char*>(triShape);
                auto* rendererData = *reinterpret_cast<void**>(base + VROffset::rendererData);
                record.rendererData = reinterpret_cast<std::uintptr_t>(rendererData);
                record.skinInstance = reinterpret_cast<std::uintptr_t>(*reinterpret_cast<void**>(base + VROffset::skinInstance));
                record.vertexDesc = *reinterpret_cast<std::uint64_t*>(base + VROffset::vertexDesc);
                record.numTriangles = *reinterpret_cast<std::uint32_t*>(base + VROffset::numTriangles);
                record.numVertices = *reinterpret_cast<std::uint16_t*>(base + VROffset::numVertices);
                record.geometryType = *reinterpret_cast<std::uint8_t*>(base + VROffset::geometryType);
                record.vertexBlock = readRendererChildPointer(rendererData, 0x08);
                record.triangleBlock = readRendererChildPointer(rendererData, 0x10);
            }

            return record;
        }

        void accumulateWeaponVisualKey(RE::NiAVObject* node, RE::NiAVObject* parent, std::uint32_t childIndex, int depth, std::uint64_t& key, WeaponVisualKeyStats& stats)
        {
            if (!node || depth > 15 || stats.nodeCount > 512) {
                return;
            }

            const bool visible = weaponVisualNodeVisible(node);
            ++stats.nodeCount;
            if (!visible) {
                ++stats.invisibleNodeCount;
            }

            std::uint32_t childCount = 0;
            if (auto* niNode = node->IsNode()) {
                childCount = static_cast<std::uint32_t>(niNode->GetRuntimeData().children.size());
            }
            const auto record = makeWeaponVisualRecord(node, parent, childIndex, childCount, static_cast<std::uint32_t>(depth), visible);
            weapon_visual_composition_policy::mixVisualRecord(key, record);

            if (!visible) {
                return;
            }

            if (node->IsTriShape()) {
                ++stats.triShapeCount;
                if (record.rendererData == 0 || record.vertexBlock == 0 || record.triangleBlock == 0) {
                    ++stats.missingRendererCount;
                } else if (record.numTriangles == 0 || record.numVertices == 0) {
                    ++stats.emptyGeometryCount;
                } else {
                    ++stats.visibleTriShapeCount;
                }
                return;
            }

            auto* niNode = node->IsNode();
            if (!niNode) {
                return;
            }

            auto& kids = niNode->GetRuntimeData().children;
            for (std::uint16_t i = 0; i < kids.size(); ++i) {
                auto* kid = kids[i].get();
                accumulateWeaponVisualKey(kid, node, i, depth + 1, key, stats);
            }
        }
    }

    static void shapeRemoveRef(const RE::hknpShape* shape)
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

    std::uint32_t generatedWeaponCollisionFilterInfo(bool collisionEnabled)
    {
        const std::uint32_t baseFilterInfo = (0x000B << 16) | (ROCK_WEAPON_LAYER & 0x7F);
        return collisionEnabled ? baseFilterInfo : (baseFilterInfo | collision_suppression_registry::kSuppressionNoCollideBit);
    }

    WeaponCollision::WeaponCollision() { clearAtomicBodyIds(); }

    WeaponCollision::WeaponBodyBank& WeaponCollision::activeWeaponBodies()
    {
        return _usingReplacementWeaponBodies ? _weaponReplacementBodies : _weaponBodies;
    }

    const WeaponCollision::WeaponBodyBank& WeaponCollision::activeWeaponBodies() const
    {
        return _usingReplacementWeaponBodies ? _weaponReplacementBodies : _weaponBodies;
    }

    WeaponCollision::WeaponBodyBank& WeaponCollision::inactiveWeaponBodies()
    {
        return _usingReplacementWeaponBodies ? _weaponBodies : _weaponReplacementBodies;
    }

    bool WeaponCollision::bankHasWeaponBody(const WeaponBodyBank& bank)
    {
        return std::any_of(bank.begin(), bank.end(), [](const WeaponBodyInstance& instance) {
            return instance.body.isValid();
        });
    }

    std::uint32_t WeaponCollision::bankWeaponBodyCount(const WeaponBodyBank& bank)
    {
        return static_cast<std::uint32_t>(std::count_if(bank.begin(), bank.end(), [](const WeaponBodyInstance& instance) {
            return instance.body.isValid();
        }));
    }

    bool WeaponCollision::hasWeaponBody() const
    {
        return bankHasWeaponBody(activeWeaponBodies());
    }

    std::uint32_t WeaponCollision::getWeaponBodyCount() const
    {
        return bankWeaponBodyCount(activeWeaponBodies());
    }

    RE::hknpBodyId WeaponCollision::getWeaponBodyId() const
    {
        for (const auto& instance : activeWeaponBodies()) {
            if (instance.body.isValid()) {
                return instance.body.getBodyId();
            }
        }
        return RE::hknpBodyId{ INVALID_BODY_ID };
    }

    std::uint32_t WeaponCollision::getWeaponBodyIdAtomic() const
    {
        if (_weaponBodyCountAtomic.load(std::memory_order_acquire) == 0) {
            return INVALID_BODY_ID;
        }
        return _weaponBodyIdsAtomic[0].load(std::memory_order_acquire);
    }

    std::uint32_t WeaponCollision::getWeaponBodyIdAtomic(std::size_t index) const
    {
        if (index >= MAX_WEAPON_BODIES) {
            return INVALID_BODY_ID;
        }
        if (index >= _weaponBodyCountAtomic.load(std::memory_order_acquire)) {
            return INVALID_BODY_ID;
        }
        return _weaponBodyIdsAtomic[index].load(std::memory_order_acquire);
    }

    bool WeaponCollision::isWeaponBodyIdAtomic(std::uint32_t bodyId) const
    {
        if (bodyId == INVALID_BODY_ID) {
            return false;
        }

        const std::uint32_t count = _weaponBodyCountAtomic.load(std::memory_order_acquire);
        for (std::uint32_t i = 0; i < count && i < MAX_WEAPON_BODIES; ++i) {
            if (_weaponBodyIdsAtomic[i].load(std::memory_order_acquire) == bodyId) {
                return true;
            }
        }
        return false;
    }

    bool WeaponCollision::tryGetWeaponContactAtomic(std::uint32_t bodyId, WeaponInteractionContact& outContact) const
    {
        outContact = {};
        if (bodyId == INVALID_BODY_ID) {
            return false;
        }

        const std::uint32_t count = _weaponBodyCountAtomic.load(std::memory_order_acquire);
        for (std::uint32_t i = 0; i < count && i < MAX_WEAPON_BODIES; ++i) {
            if (_weaponBodyIdsAtomic[i].load(std::memory_order_acquire) != bodyId) {
                continue;
            }

            outContact.valid = true;
            outContact.bodyId = bodyId;
            outContact.partKind = static_cast<WeaponPartKind>(_weaponBodyPartKindsAtomic[i].load(std::memory_order_acquire));
            outContact.reloadRole = static_cast<WeaponReloadRole>(_weaponBodyReloadRolesAtomic[i].load(std::memory_order_acquire));
            outContact.supportGripRole = static_cast<WeaponSupportGripRole>(_weaponBodySupportRolesAtomic[i].load(std::memory_order_acquire));
            outContact.socketRole = static_cast<WeaponSocketRole>(_weaponBodySocketRolesAtomic[i].load(std::memory_order_acquire));
            outContact.actionRole = static_cast<WeaponActionRole>(_weaponBodyActionRolesAtomic[i].load(std::memory_order_acquire));
            outContact.fallbackGripPose = static_cast<WeaponGripPoseId>(_weaponBodyGripPosesAtomic[i].load(std::memory_order_acquire));
            outContact.interactionRoot = reinterpret_cast<RE::NiAVObject*>(_weaponBodyInteractionRootsAtomic[i].load(std::memory_order_acquire));
            outContact.sourceRoot = reinterpret_cast<RE::NiAVObject*>(_weaponBodySourceRootsAtomic[i].load(std::memory_order_acquire));
            outContact.weaponGenerationKey = _weaponBodyGenerationKeysAtomic[i].load(std::memory_order_acquire);
            return true;
        }
        return false;
    }

    bool WeaponCollision::tryGetWeaponContactDebugInfo(std::uint32_t bodyId, WeaponInteractionDebugInfo& outInfo) const
    {
        outInfo = {};
        if (bodyId == INVALID_BODY_ID) {
            return false;
        }

        for (const auto& instance : activeWeaponBodies()) {
            if (!instance.body.isValid() || instance.body.getBodyId().value != bodyId) {
                continue;
            }

            outInfo.sourceName = instance.sourceName;
            outInfo.sourceRootName = instance.sourceRootName;
            return true;
        }

        return false;
    }

    std::vector<WeaponCollisionProfileEvidenceDescriptor> WeaponCollision::getProfileEvidenceDescriptors() const
    {
        std::vector<WeaponCollisionProfileEvidenceDescriptor> descriptors;
        descriptors.reserve(getWeaponBodyCount());

        auto copyLocalPoints = [](const std::vector<RE::NiPoint3>& points) {
            std::vector<WeaponEvidencePoint3> result;
            result.reserve(points.size());
            for (const auto& point : points) {
                result.push_back(makeWeaponEvidencePoint(point.x, point.y, point.z));
            }
            return result;
        };

        for (const auto& instance : activeWeaponBodies()) {
            if (!instance.body.isValid()) {
                continue;
            }

            WeaponCollisionProfileEvidenceDescriptor descriptor{};
            descriptor.valid = true;
            descriptor.bodyId = instance.body.getBodyId().value;
            descriptor.sourceRootAddress = reinterpret_cast<std::uintptr_t>(instance.sourceNode);
            descriptor.geometryRootAddress = reinterpret_cast<std::uintptr_t>(instance.driveNode);
            descriptor.sourceRootName = instance.sourceRootName;
            descriptor.geometryRootName = instance.driveNode ? safeNodeName(instance.driveNode) : "";
            descriptor.sourceName = instance.sourceName;
            descriptor.semantic = instance.semantic;
            descriptor.localBoundsGame = WeaponEvidenceBounds3{
                .min = makeWeaponEvidencePoint(instance.generatedLocalMinGame.x, instance.generatedLocalMinGame.y, instance.generatedLocalMinGame.z),
                .max = makeWeaponEvidencePoint(instance.generatedLocalMaxGame.x, instance.generatedLocalMaxGame.y, instance.generatedLocalMaxGame.z),
                .valid = true,
            };
            descriptor.localMeshPointsGame = copyLocalPoints(instance.generatedLocalPointsGame);
            descriptor.pointCount = instance.generatedPointCount;
            descriptors.push_back(std::move(descriptor));
        }

        return descriptors;
    }

    bool WeaponCollision::tryGetProfileEvidenceDescriptorForBodyId(
        std::uint32_t bodyId,
        WeaponCollisionProfileEvidenceDescriptor& outDescriptor,
        RE::NiAVObject*& outSourceNode) const
    {
        outDescriptor = {};
        outSourceNode = nullptr;
        if (bodyId == INVALID_BODY_ID) {
            return false;
        }

        auto copyLocalPoints = [](const std::vector<RE::NiPoint3>& points) {
            std::vector<WeaponEvidencePoint3> result;
            result.reserve(points.size());
            for (const auto& point : points) {
                result.push_back(makeWeaponEvidencePoint(point.x, point.y, point.z));
            }
            return result;
        };

        for (const auto& instance : activeWeaponBodies()) {
            if (!instance.body.isValid() || instance.body.getBodyId().value != bodyId) {
                continue;
            }

            outDescriptor.valid = true;
            outDescriptor.bodyId = bodyId;
            outDescriptor.sourceRootAddress = reinterpret_cast<std::uintptr_t>(instance.sourceNode);
            outDescriptor.geometryRootAddress = reinterpret_cast<std::uintptr_t>(instance.driveNode);
            outDescriptor.sourceRootName = instance.sourceRootName;
            outDescriptor.geometryRootName = instance.driveNode ? safeNodeName(instance.driveNode) : "";
            outDescriptor.sourceName = instance.sourceName;
            outDescriptor.semantic = instance.semantic;
            outDescriptor.localBoundsGame = WeaponEvidenceBounds3{
                .min = makeWeaponEvidencePoint(instance.generatedLocalMinGame.x, instance.generatedLocalMinGame.y, instance.generatedLocalMinGame.z),
                .max = makeWeaponEvidencePoint(instance.generatedLocalMaxGame.x, instance.generatedLocalMaxGame.y, instance.generatedLocalMaxGame.z),
                .valid = true,
            };
            outDescriptor.localMeshPointsGame = copyLocalPoints(instance.generatedLocalPointsGame);
            outDescriptor.pointCount = instance.generatedPointCount;
            outSourceNode = instance.sourceNode;
            return true;
        }

        return false;
    }

    bool WeaponCollision::tryFindInteractionContactNearPoint(
        const RE::NiAVObject* weaponNode,
        const RE::NiPoint3& probeWorldPoint,
        float probeRadiusGame,
        WeaponInteractionContact& outContact) const
    {
        outContact = {};
        if (!weaponNode || probeRadiusGame <= 0.0f) {
            return false;
        }

        float bestDistanceSquared = std::numeric_limits<float>::max();
        const WeaponBodyInstance* bestInstance = nullptr;

        for (const auto& instance : activeWeaponBodies()) {
            if (!instance.body.isValid()) {
                continue;
            }

            const RE::NiAVObject* driveRoot = instance.driveNode ? instance.driveNode : instance.sourceNode ? instance.sourceNode : weaponNode;
            if (!driveRoot) {
                continue;
            }

            const RE::NiPoint3 probeLocal = weapon_collision_geometry_math::worldPointToLocal(
                driveRoot->world.rotate,
                driveRoot->world.translate,
                driveRoot->world.scale,
                probeWorldPoint);

            const float distanceSquared = weapon_interaction_probe_math::pointAabbDistanceSquared(
                probeLocal,
                instance.generatedLocalMinGame,
                instance.generatedLocalMaxGame);
            if (!weapon_interaction_probe_math::isWithinProbeRadiusSquared(distanceSquared, probeRadiusGame) || distanceSquared >= bestDistanceSquared) {
                continue;
            }

            bestDistanceSquared = distanceSquared;
            bestInstance = &instance;
        }

        if (!bestInstance) {
            return false;
        }

        outContact.valid = true;
        outContact.bodyId = bestInstance->body.getBodyId().value;
        outContact.partKind = bestInstance->semantic.partKind;
        outContact.reloadRole = bestInstance->semantic.reloadRole;
        outContact.supportGripRole = bestInstance->semantic.supportGripRole;
        outContact.socketRole = bestInstance->semantic.socketRole;
        outContact.actionRole = bestInstance->semantic.actionRole;
        outContact.fallbackGripPose = bestInstance->semantic.fallbackGripPose;
        outContact.interactionRoot = bestInstance->driveNode;
        outContact.sourceRoot = bestInstance->sourceNode;
        outContact.weaponGenerationKey = _cachedWeaponKey;
        outContact.probeDistanceGame = std::sqrt(bestDistanceSquared);
        return true;
    }

    BethesdaPhysicsBody& WeaponCollision::getWeaponBody()
    {
        for (auto& instance : activeWeaponBodies()) {
            if (instance.body.isValid()) {
                return instance.body;
            }
        }
        return activeWeaponBodies()[0].body;
    }

    void WeaponCollision::init(RE::hknpWorld* world, void* bhkWorld)
    {
        // Cache the Havok context even while the feature is disabled so the INI
        // watcher can hot-enable weapon collision without requiring a physics
        // module restart.
        _cachedWorld = world;
        _cachedBhkWorld = bhkWorld;
        _cachedWeaponKey = 0;
        _pendingWeaponKey = 0;
        _visualSettleState = {};
        _weaponBodyPending = false;
        _usingReplacementWeaponBodies = false;
        _driveRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);
        _retryCounter = 0;
        _cachedConvexRadius = -1.0f;
        _cachedPointDedupGrid = -1.0f;
        _cachedGroupingMode = -1;
        clearAtomicBodyIds();

        if (!g_rockConfig.rockWeaponCollisionEnabled) {
            ROCK_LOG_INFO(Weapon, "WeaponCollision disabled via config — context cached for hot reload");
            return;
        }

        ROCK_LOG_INFO(Weapon, "WeaponCollision initialized");
    }

    void WeaponCollision::shutdown()
    {
        if (hasWeaponBody()) {
            ROCK_LOG_INFO(Weapon, "WeaponCollision shutdown destroying generated bodies from cached context");
            destroyWeaponBody(_cachedWorld);
        } else if (_dominantHandDisabled && _cachedWorld) {
            enableDominantHandCollision(_cachedWorld);
        }

        _cachedWeaponKey = 0;
        _pendingWeaponKey = 0;
        _visualSettleState = {};
        _cachedWorld = nullptr;
        _cachedBhkWorld = nullptr;
        _weaponBodyPending = false;
        _usingReplacementWeaponBodies = false;
        _driveRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);
        _retryCounter = 0;
        _cachedConvexRadius = -1.0f;
        _cachedPointDedupGrid = -1.0f;
        _cachedGroupingMode = -1;
        _dominantHandDisabled = false;
        _disabledHandBodyId.value = INVALID_BODY_ID;

        ROCK_LOG_INFO(Weapon, "WeaponCollision shutdown");
    }

    void WeaponCollision::update(RE::hknpWorld* world, RE::NiAVObject* weaponNode, RE::hknpBodyId dominantHandBodyId, float dt)
    {
        (void)dt;

        if (!g_rockConfig.rockWeaponCollisionEnabled) {
            if (hasWeaponBody() && world) {
                ROCK_LOG_INFO(Weapon, "WeaponCollision disabled via hot reload — destroying generated weapon bodies");
                destroyWeaponBody(world);
            }
            if (_dominantHandDisabled && world) {
                enableDominantHandCollision(world);
            }
            _cachedWeaponKey = 0;
            _pendingWeaponKey = 0;
            _visualSettleState = {};
            _weaponBodyPending = false;
            _driveRebuildRequested.store(false, std::memory_order_release);
            _driveFailureCount.store(0, std::memory_order_release);
            _retryCounter = 0;
            _cachedConvexRadius = -1.0f;
            _cachedPointDedupGrid = -1.0f;
            _cachedGroupingMode = -1;
            return;
        }
        if (!world)
            return;

        if (world != _cachedWorld) {
            ROCK_LOG_INFO(Weapon, "hknpWorld changed — resetting weapon collision state");
            if (hasWeaponBody()) {
                destroyWeaponBody(_cachedWorld ? _cachedWorld : world);
            } else {
                clearAtomicBodyIds();
            }
            _cachedWeaponKey = 0;
            _pendingWeaponKey = 0;
            _visualSettleState = {};
            _weaponBodyPending = false;
            _driveRebuildRequested.store(false, std::memory_order_release);
            _driveFailureCount.store(0, std::memory_order_release);
            _cachedWorld = world;
            _cachedConvexRadius = -1.0f;
            _cachedPointDedupGrid = -1.0f;
            _cachedGroupingMode = -1;
        }

        if (!weaponNode) {
            if (hasWeaponBody()) {
                ROCK_LOG_INFO(Weapon, "Weapon node gone (holstered?) — destroying weapon bodies");
                destroyWeaponBody(world);
            }
            if (_dominantHandDisabled) {
                enableDominantHandCollision(world);
            }
            _cachedWeaponKey = 0;
            _pendingWeaponKey = 0;
            _visualSettleState = {};
            _weaponBodyPending = false;
            _driveRebuildRequested.store(false, std::memory_order_release);
            _driveFailureCount.store(0, std::memory_order_release);
            _retryCounter = 0;
            _cachedConvexRadius = -1.0f;
            _cachedPointDedupGrid = -1.0f;
            _cachedGroupingMode = -1;
            return;
        }

        WeaponVisualKeyStats visualKeyStats{};
        const std::uint64_t observedKey = getEquippedWeaponKey(weaponNode, visualKeyStats);
        const auto visualSettle = weapon_visual_composition_policy::advanceVisualSettle(_visualSettleState, observedKey);
        std::uint64_t currentKey = visualSettle.settled ? observedKey : _cachedWeaponKey;

        if (_driveRebuildRequested.exchange(false, std::memory_order_acq_rel)) {
            ROCK_LOG_WARN(Weapon,
                "Generated weapon collision drive failure requested rebuild cachedKey={:016X} observedKey={:016X}",
                _cachedWeaponKey,
                observedKey);
            if (hasWeaponBody()) {
                destroyWeaponBody(world);
            }
            _cachedWeaponKey = 0;
            _pendingWeaponKey = observedKey;
            _weaponBodyPending = (observedKey != 0);
            _retryCounter = 0;
            _driveFailureCount.store(0, std::memory_order_release);
            currentKey = visualSettle.settled ? observedKey : 0;
        }

        if (!visualSettle.settled) {
            if (visualSettle.keyChanged) {
                ROCK_LOG_INFO(Weapon,
                    "Weapon visual composition changed: cached={:016X} observed={:016X} visualRoots={} visualNodes={} visualTriShapes={} visibleTriShapes={} missingGeometry={} invisibleNodes={} — waiting for stable geometry",
                    _cachedWeaponKey,
                    observedKey,
                    visualKeyStats.rootCount,
                    visualKeyStats.nodeCount,
                    visualKeyStats.triShapeCount,
                    visualKeyStats.visibleTriShapeCount,
                    visualKeyStats.missingRendererCount + visualKeyStats.emptyGeometryCount,
                    visualKeyStats.invisibleNodeCount);
            } else {
                ROCK_LOG_DEBUG(Weapon,
                    "Weapon visual composition settling: observed={:016X} stableFrames={}/{} visualRoots={} visualNodes={} visibleTriShapes={} missingGeometry={}",
                    observedKey,
                    visualSettle.stableFrames,
                    weapon_visual_composition_policy::kRequiredStableVisualFrames,
                    visualKeyStats.rootCount,
                    visualKeyStats.nodeCount,
                    visualKeyStats.visibleTriShapeCount,
                    visualKeyStats.missingRendererCount + visualKeyStats.emptyGeometryCount);
            }
        } else {
            if (visualSettle.keyChanged || observedKey != _cachedWeaponKey) {
                ROCK_LOG_DEBUG(Weapon,
                    "Weapon visual composition settled: cached={:016X} observed={:016X} stableFrames={} visualRoots={} visualNodes={} visualTriShapes={} visibleTriShapes={} missingGeometry={} invisibleNodes={}",
                    _cachedWeaponKey,
                    observedKey,
                    visualSettle.stableFrames,
                    visualKeyStats.rootCount,
                    visualKeyStats.nodeCount,
                    visualKeyStats.triShapeCount,
                    visualKeyStats.visibleTriShapeCount,
                    visualKeyStats.missingRendererCount + visualKeyStats.emptyGeometryCount,
                    visualKeyStats.invisibleNodeCount);
            }

            if (currentKey != _cachedWeaponKey) {
                if (hasWeaponBody()) {
                    if (_pendingWeaponKey != currentKey) {
                        ROCK_LOG_INFO(Weapon,
                            "Weapon visual composition changed while generated bodies are valid: cached={:016X} pending={:016X} visualRoots={} visualNodes={} visualTriShapes={} visibleTriShapes={} — keeping old bodies until replacement mesh is ready",
                            _cachedWeaponKey,
                            currentKey,
                            visualKeyStats.rootCount,
                            visualKeyStats.nodeCount,
                            visualKeyStats.triShapeCount,
                            visualKeyStats.visibleTriShapeCount);
                    }
                    _pendingWeaponKey = currentKey;
                    _weaponBodyPending = (currentKey != 0);
                } else {
                    ROCK_LOG_INFO(Weapon,
                        "Weapon changed: {:016X} -> {:016X} visualRoots={} visualNodes={} visualTriShapes={} visibleTriShapes={} missingGeometry={} invisibleNodes={}",
                        _cachedWeaponKey,
                        currentKey,
                        visualKeyStats.rootCount,
                        visualKeyStats.nodeCount,
                        visualKeyStats.triShapeCount,
                        visualKeyStats.visibleTriShapeCount,
                        visualKeyStats.missingRendererCount + visualKeyStats.emptyGeometryCount,
                        visualKeyStats.invisibleNodeCount);
                    _cachedWeaponKey = currentKey;
                    _pendingWeaponKey = currentKey;
                    _weaponBodyPending = (currentKey != 0);
                    _cachedConvexRadius = -1.0f;
                    _cachedPointDedupGrid = -1.0f;
                    _cachedGroupingMode = -1;
                }
            }

            if (hasWeaponBody() && weaponCollisionSettingsChanged()) {
                ROCK_LOG_INFO(Weapon, "Generated weapon collision settings changed — keeping current bodies while replacement is pending");
                _pendingWeaponKey = currentKey;
                _weaponBodyPending = (currentKey != 0);
                _retryCounter = 0;
            }

            if (_weaponBodyPending) {
                const bool shouldAttemptCreate = (++_retryCounter >= 90) || (_retryCounter == 1);
                if (shouldAttemptCreate) {
                    if (_retryCounter >= 90) {
                        _retryCounter = 0;
                    }

                    /*
                     * Mesh-only weapon collision:
                     * Mirroring existing FO4VR bodies was useful during bring-up, but
                     * those native shapes are incomplete for firearms and use different
                     * authoring frames from the generated visible-mesh hulls. If the
                     * visible mesh has not streamed in yet, keep the weapon pending and
                     * retry instead of creating a second, stale collision convention.
                     */
                    std::vector<GeneratedHullSource> generatedSources;
                    const std::size_t generatedCount = findGeneratedWeaponShapeSources(weaponNode, generatedSources);
                    const bool hasBuildableSource = std::any_of(generatedSources.begin(), generatedSources.end(), [](const GeneratedHullSource& source) {
                        return pointCloudCanBuildHull(source.localPointsGame);
                    });
                    const std::uint64_t replacementKey = (_pendingWeaponKey != 0) ? _pendingWeaponKey : currentKey;
                    const auto rebuildDecision = weapon_authority_lifecycle_policy::evaluateGeneratedCollisionRebuild(
                        weapon_authority_lifecycle_policy::GeneratedCollisionRebuildInput{
                            .hasExistingBodies = hasWeaponBody(),
                            .cachedKey = _cachedWeaponKey,
                            .currentKey = replacementKey,
                            .settingsChanged = weaponCollisionSettingsChanged(),
                            .generatedSourceCount = generatedCount,
                            .hasBuildableSource = hasBuildableSource,
                        });

                    bool created = false;
                    if (rebuildDecision.action == weapon_authority_lifecycle_policy::GeneratedCollisionRebuildAction::CreateInitial) {
                        _cachedWeaponKey = replacementKey;
                        created = createGeneratedWeaponBodiesInBank(world, generatedSources, activeWeaponBodies(), true) > 0;
                    } else if (rebuildDecision.action == weapon_authority_lifecycle_policy::GeneratedCollisionRebuildAction::ReplaceExisting) {
                        auto& replacementBank = inactiveWeaponBodies();
                        destroyWeaponBodyBank(replacementBank, true);
                        const auto replacementCount = createGeneratedWeaponBodiesInBank(world, generatedSources, replacementBank, false);
                        if (replacementCount > 0) {
                            ROCK_LOG_INFO(Weapon,
                                "Replacing generated weapon collision bodies cachedKey={:016X} pendingKey={:016X} sourceCount={} replacementBodies={} reason={}",
                                _cachedWeaponKey,
                                replacementKey,
                                generatedCount,
                                replacementCount,
                                rebuildDecision.reason);
                            if (_dominantHandDisabled && world) {
                                enableDominantHandCollision(world);
                            }
                            destroyWeaponBodyBank(activeWeaponBodies(), true);
                            _usingReplacementWeaponBodies = !_usingReplacementWeaponBodies;
                            _cachedWeaponKey = replacementKey;
                            setWeaponBodyBankCollisionEnabled(world, activeWeaponBodies(), true);
                            publishAtomicBodyIds(activeWeaponBodies());
                            created = true;
                        } else {
                            destroyWeaponBodyBank(replacementBank, true);
                            created = false;
                        }
                    } else {
                        created = hasWeaponBody() && rebuildDecision.keepExistingBodies;
                    }

                    if (created && !rebuildDecision.pending) {
                        _weaponBodyPending = false;
                        _pendingWeaponKey = 0;
                        _retryCounter = 0;
                        _cachedConvexRadius = g_rockConfig.rockWeaponCollisionConvexRadius;
                        _cachedPointDedupGrid = g_rockConfig.rockWeaponCollisionPointDedupGrid;
                        _cachedGroupingMode = g_rockConfig.rockWeaponCollisionGroupingMode;
                    } else {
                        ROCK_LOG_SAMPLE_WARN(Weapon,
                            g_rockConfig.rockLogSampleMilliseconds,
                            "Generated weapon mesh collision unavailable — keeping weapon pending for retry cachedKey={:016X} pendingKey={:016X} sources={} reason={}",
                            _cachedWeaponKey,
                            _pendingWeaponKey,
                            generatedCount,
                            rebuildDecision.reason);
                        _weaponBodyPending = true;
                    }
                }
            }
        }

        if (hasWeaponBody() && !_dominantHandDisabled && dominantHandBodyId.value != INVALID_BODY_ID) {
            disableDominantHandCollision(world, dominantHandBodyId);
        } else if (!hasWeaponBody() && _dominantHandDisabled) {
            enableDominantHandCollision(world);
        }

        /*
         * Per-frame body transforms are synchronized after weapon visual authority
         * settles in PhysicsInteraction. HIGGS uses one final desired weapon frame
         * for visual weapon, collision offset, offset node, and wand updates; ROCK
         * follows that ownership rule here instead of moving generated Havok bodies
         * once from FRIK/current state and again from ROCK two-handed authority.
         */
    }

    std::uint64_t WeaponCollision::getEquippedWeaponKey(RE::NiAVObject* weaponNode, WeaponVisualKeyStats& stats) const
    {
        if (!weaponNode) {
            return 0;
        }

        /*
         * The equipped weapon key must represent the visual weapon tree, not
         * only FO4VR's stable attachment parent. The parent node can be reused
         * across weapon swaps, which leaves generated mesh bodies from the
         * previous firearm alive. Hashing candidate visual roots and their
         * current TriShape subtree makes generated collision rebuild when the
         * mesh changes while still ignoring per-frame transform movement.
         */
        std::uint64_t key = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
        const auto candidates = makeGeneratedWeaponMeshRootCandidates(weaponNode);
        for (const auto& candidate : candidates) {
            if (!candidate.root) {
                continue;
            }

            ++stats.rootCount;
            mixWeaponVisualString(key, candidate.label);
            mixWeaponVisualKey(key, reinterpret_cast<std::uintptr_t>(candidate.root));
            accumulateWeaponVisualKey(candidate.root, nullptr, 0, 0, key, stats);
        }

        return key != weapon_visual_composition_policy::kWeaponVisualCompositionOffset ? key : reinterpret_cast<std::uint64_t>(weaponNode);
    }

    std::size_t WeaponCollision::findGeneratedWeaponShapeSources(RE::NiAVObject* weaponNode, std::vector<GeneratedHullSource>& outSources)
    {
        outSources.clear();
        const auto candidates = makeGeneratedWeaponMeshRootCandidates(weaponNode);
        if (candidates.empty()) {
            ROCK_LOG_DEBUG(Weapon, "Generated weapon mesh source scan: no weapon root candidates");
            return 0;
        }

        RE::NiAVObject* selectedRoot = nullptr;
        const char* selectedLabel = "";
        std::uint64_t selectedScore = 0;
        const auto groupingMode = weapon_collision_grouping_policy::sanitizeWeaponCollisionGroupingMode(g_rockConfig.rockWeaponCollisionGroupingMode);
        for (const auto& candidate : candidates) {
            std::vector<GeneratedHullSource> candidateSources;
            std::uint32_t visitedShapes = 0;
            std::uint32_t extractedTriangles = 0;
            if (groupingMode == weapon_collision_grouping_policy::WeaponCollisionGroupingMode::LegacyTriShape) {
                findGeneratedWeaponShapeSourcesRecursive(candidate.root, candidate.root, candidate.root->world, 0, candidateSources, visitedShapes, extractedTriangles);
            } else {
                /*
                 * Semantic modes keep the HIGGS-style idea of a coherent owned
                 * weapon collision package, but they respect FO4VR visible
                 * weapon authoring. Mode 1 preserves one authored semantic node
                 * per generated source. Mode 2 additionally treats helper nodes
                 * such as WeaponExtra* as structure and merges compatible part
                 * evidence before source creation so PAPER sees one reload body
                 * intent for magazines, stocks, bolts, and action pieces. The
                 * selected visual root remains the physics drive root; semantic
                 * roots are metadata and grouping boundaries, not independent
                 * keyframed weapon roots. Loaded ammo display visuals stay out
                 * of permanent weapon collision.
                 */
                const bool compoundGroupingMode =
                    groupingMode == weapon_collision_grouping_policy::WeaponCollisionGroupingMode::CompoundSemanticPart;
                std::deque<SemanticPartPointCloud> partClouds;

                auto classifyNodeSemantic = [compoundGroupingMode](std::string_view nodeName) {
                    return compoundGroupingMode ? weapon_collision_grouping_policy::classifyCompoundSemanticWeaponPartName(nodeName) :
                                                  classifyWeaponPartName(nodeName);
                };
                auto excludeNodeSemantic = [compoundGroupingMode](const WeaponPartClassification& semantic) {
                    return compoundGroupingMode ? weapon_collision_grouping_policy::excludeFromCompoundSemanticWeaponCollision(semantic) :
                                                  weapon_collision_grouping_policy::excludeFromSemanticWeaponCollision(semantic);
                };
                auto startsPartGroup = [compoundGroupingMode](const WeaponPartClassification& semantic) {
                    return compoundGroupingMode ? weapon_collision_grouping_policy::startsCompoundSemanticWeaponPartGroup(semantic) :
                                                  weapon_collision_grouping_policy::startsSemanticWeaponPartGroup(semantic);
                };
                auto childSplitsFromParent = [compoundGroupingMode](const WeaponPartClassification& parent, const WeaponPartClassification& child) {
                    return compoundGroupingMode ? weapon_collision_grouping_policy::compoundSemanticChildSplitsFromParent(parent, child) :
                                                  weapon_collision_grouping_policy::semanticChildSplitsFromParent(parent, child);
                };

                auto appendGeneratedSource = [&](RE::NiAVObject* driveRoot,
                                                 RE::NiAVObject* sourceRoot,
                                                 std::string sourceName,
                                                 const WeaponPartClassification& semantic,
                                                 std::vector<RE::NiPoint3> localPoints,
                                                 int depth) {
                    const float dedupGridGame = (std::max)(g_rockConfig.rockWeaponCollisionPointDedupGrid * havokToGameScale(), 0.01f);
                    localPoints = dedupePointCloud(localPoints, dedupGridGame);
                    if (!pointCloudCanBuildHull(localPoints)) {
                        ROCK_LOG_TRACE(Weapon, "{}generated mesh source skipped '{}': degenerate grouped point cloud points={}", std::string(depth * 2, ' '), sourceName,
                            localPoints.size());
                        return;
                    }

                    std::vector<std::vector<RE::NiPoint3>> clusters;
                    weapon_collision_geometry_math::splitOversizedCluster(localPoints, MAX_CONVEX_HULL_POINTS, clusters);
                    if (compoundGroupingMode) {
                        GeneratedHullSource source;
                        source.localCenterGame = weapon_collision_geometry_math::pointCenter(localPoints);
                        const auto sourceBounds = pointCloudBounds(localPoints);
                        source.localMinGame = sourceBounds.min;
                        source.localMaxGame = sourceBounds.max;
                        source.localPointsGame = std::move(localPoints);
                        source.driveRoot = driveRoot ? driveRoot : sourceRoot;
                        source.sourceRoot = sourceRoot;
                        source.sourceGroupId = reinterpret_cast<std::uintptr_t>(sourceRoot ? sourceRoot : source.driveRoot);
                        source.sourceName = std::move(sourceName);
                        source.semantic = semantic;

                        for (auto& clusterPoints : clusters) {
                            auto cluster = weapon_collision_geometry_math::limitPointCloud(std::move(clusterPoints), MAX_CONVEX_HULL_POINTS);
                            if (pointCloudCanBuildHull(cluster)) {
                                source.childLocalPointCloudsGame.push_back(std::move(cluster));
                            }
                        }

                        if (source.childLocalPointCloudsGame.empty()) {
                            ROCK_LOG_TRACE(Weapon,
                                "{}generated compound mesh source skipped '{}': no buildable child clusters",
                                std::string(depth * 2, ' '),
                                source.sourceName);
                            return;
                        }

                        ROCK_LOG_TRACE(Weapon,
                            "{}generated compound mesh source '{}': points={} children={} center=({:.2f},{:.2f},{:.2f}) root='{}'",
                            std::string(depth * 2, ' '),
                            source.sourceName,
                            source.localPointsGame.size(),
                            source.childLocalPointCloudsGame.size(),
                            source.localCenterGame.x,
                            source.localCenterGame.y,
                            source.localCenterGame.z,
                            safeNodeName(sourceRoot));
                        candidateSources.push_back(std::move(source));
                        return;
                    }

                    for (std::size_t clusterIndex = 0; clusterIndex < clusters.size(); ++clusterIndex) {
                        auto cluster = weapon_collision_geometry_math::limitPointCloud(std::move(clusters[clusterIndex]), MAX_CONVEX_HULL_POINTS);
                        if (!pointCloudCanBuildHull(cluster)) {
                            continue;
                        }

                        GeneratedHullSource source;
                        source.localCenterGame = weapon_collision_geometry_math::pointCenter(cluster);
                        const auto bounds = pointCloudBounds(cluster);
                        source.localMinGame = bounds.min;
                        source.localMaxGame = bounds.max;
                        source.localPointsGame = std::move(cluster);
                        source.driveRoot = driveRoot ? driveRoot : sourceRoot;
                        source.sourceRoot = sourceRoot;
                        source.sourceGroupId = reinterpret_cast<std::uintptr_t>(sourceRoot ? sourceRoot : source.driveRoot);
                        source.sourceName = sourceName;
                        if (clusters.size() > 1) {
                            source.sourceName += "#";
                            source.sourceName += std::to_string(clusterIndex);
                        }
                        source.semantic = semantic;
                        ROCK_LOG_TRACE(Weapon,
                            "{}generated grouped mesh source '{}': points={} center=({:.2f},{:.2f},{:.2f}) root='{}'",
                            std::string(depth * 2, ' '),
                            source.sourceName,
                            source.localPointsGame.size(),
                            source.localCenterGame.x,
                            source.localCenterGame.y,
                            source.localCenterGame.z,
                            safeNodeName(sourceRoot));
                        candidateSources.push_back(std::move(source));
                    }
                };

                auto appendTrianglesAsLocalPoints = [](const std::vector<TriangleData>& triangles, const RE::NiTransform& rootTransform, std::vector<RE::NiPoint3>& points) {
                    points.reserve(points.size() + triangles.size() * 3);
                    for (const auto& triangle : triangles) {
                        points.push_back(weapon_collision_geometry_math::worldPointToLocal(rootTransform.rotate, rootTransform.translate, rootTransform.scale, triangle.v0));
                        points.push_back(weapon_collision_geometry_math::worldPointToLocal(rootTransform.rotate, rootTransform.translate, rootTransform.scale, triangle.v1));
                        points.push_back(weapon_collision_geometry_math::worldPointToLocal(rootTransform.rotate, rootTransform.translate, rootTransform.scale, triangle.v2));
                    }
                };

                auto findPartCloud = [&](RE::NiAVObject* sourceRoot, const WeaponPartClassification& semantic) -> SemanticPartPointCloud* {
                    for (auto& part : partClouds) {
                        if (compoundGroupingMode &&
                            weapon_collision_grouping_policy::compoundSemanticPartsMayShareBody(part.semantic, semantic)) {
                            return &part;
                        }
                        if (!compoundGroupingMode && part.semanticRoot == sourceRoot) {
                            return &part;
                        }
                    }
                    return nullptr;
                };

                auto ensurePartCloud = [&](RE::NiAVObject* sourceRoot, const char* sourceName, const WeaponPartClassification& semantic) -> SemanticPartPointCloud* {
                    if (auto* existing = findPartCloud(sourceRoot, semantic)) {
                        return existing;
                    }
                    SemanticPartPointCloud part{};
                    part.driveRoot = candidate.root;
                    part.semanticRoot = sourceRoot;
                    part.sourceName = sourceName ? sourceName : "";
                    part.semantic = semantic;
                    partClouds.push_back(std::move(part));
                    return &partClouds.back();
                };

                auto traverseSemantic = [&](auto&& self, RE::NiAVObject* node, SemanticPartPointCloud* activePart, int depth) -> void {
                    if (!node || depth > 15) {
                        return;
                    }
                    if (!weaponVisualNodeVisible(node)) {
                        return;
                    }

                    const char* nodeName = safeNodeName(node);
                    const auto nodeSemantic = classifyNodeSemantic(nodeName);

                    if (auto* triShape = node->IsTriShape()) {
                        ++visitedShapes;
                        std::vector<TriangleData> triangles;
                        const bool skinned = isSkinned(triShape);
                        const int added = skinned ? extractTrianglesFromSkinnedTriShape(triShape, triangles) : extractTrianglesFromTriShape(triShape, triangles);
                        if (added <= 0) {
                            ROCK_LOG_TRACE(Weapon, "{}generated grouped mesh source skipped '{}': no extractable triangles", std::string(depth * 2, ' '), nodeName);
                            return;
                        }
                        extractedTriangles += static_cast<std::uint32_t>(added);

                        if (excludeNodeSemantic(nodeSemantic)) {
                            ROCK_LOG_TRACE(Weapon, "{}generated grouped mesh source skipped '{}': ammo display semantic excluded", std::string(depth * 2, ' '), nodeName);
                            return;
                        }

                        if (activePart && !childSplitsFromParent(activePart->semantic, nodeSemantic)) {
                            appendTrianglesAsLocalPoints(triangles, activePart->driveRoot->world, activePart->localPointsGame);
                            return;
                        }

                        const bool triShapeIsSemanticPart = startsPartGroup(nodeSemantic);
                        RE::NiAVObject* sourceRoot = triShapeIsSemanticPart ? node : candidate.root;
                        std::vector<RE::NiPoint3> localPoints;
                        appendTrianglesAsLocalPoints(triangles, candidate.root->world, localPoints);
                        if (compoundGroupingMode && triShapeIsSemanticPart) {
                            if (auto* part = ensurePartCloud(sourceRoot, nodeName, nodeSemantic)) {
                                part->localPointsGame.insert(part->localPointsGame.end(), localPoints.begin(), localPoints.end());
                            }
                            return;
                        }
                        appendGeneratedSource(candidate.root, sourceRoot, nodeName, triShapeIsSemanticPart ? nodeSemantic : classifyNodeSemantic(nodeName), std::move(localPoints), depth);
                        return;
                    }

                    SemanticPartPointCloud* nextPart = activePart;
                    if (node != candidate.root) {
                        if (excludeNodeSemantic(nodeSemantic)) {
                            ROCK_LOG_TRACE(Weapon, "{}generated grouped mesh subtree skipped '{}': ammo display semantic excluded", std::string(depth * 2, ' '), nodeName);
                            return;
                        }
                        if (startsPartGroup(nodeSemantic) && (!activePart || childSplitsFromParent(activePart->semantic, nodeSemantic))) {
                            nextPart = ensurePartCloud(node, nodeName, nodeSemantic);
                        }
                    }

                    auto* niNode = node->IsNode();
                    if (!niNode) {
                        return;
                    }

                    auto& kids = niNode->GetRuntimeData().children;
                    for (std::uint16_t i = 0; i < kids.size(); ++i) {
                        if (auto* kid = kids[i].get()) {
                            self(self, kid, nextPart, depth + 1);
                        }
                    }
                };

                traverseSemantic(traverseSemantic, candidate.root, nullptr, 0);

                for (auto& part : partClouds) {
                    appendGeneratedSource(part.driveRoot, part.semanticRoot, part.sourceName, part.semantic, std::move(part.localPointsGame), 0);
                }
            }

            ROCK_LOG_DEBUG(Weapon,
                "Generated weapon mesh candidate: label='{}' root='{}' addr={:x} grouping={} visitedShapes={} triangles={} hulls={}",
                candidate.label,
                safeNodeName(candidate.root),
                reinterpret_cast<std::uintptr_t>(candidate.root),
                weapon_collision_grouping_policy::weaponCollisionGroupingModeName(groupingMode),
                visitedShapes,
                extractedTriangles,
                candidateSources.size());

            if (!candidateSources.empty()) {
                const std::uint64_t candidateScore =
                    weapon_collision_grouping_policy::weaponCollisionCandidateScore(candidateSources.size(), extractedTriangles);
                if (!selectedRoot || candidateScore > selectedScore) {
                    selectedScore = candidateScore;
                    selectedRoot = candidate.root;
                    selectedLabel = candidate.label;
                    outSources = std::move(candidateSources);
                }
            }
        }

        if (!selectedRoot) {
            ROCK_LOG_DEBUG(Weapon, "Generated weapon mesh source scan: all {} candidates produced zero hulls", candidates.size());
            return 0;
        }

        if (outSources.size() > MAX_WEAPON_BODIES) {
            std::vector<WeaponSemanticHullBudgetInput> selectionInputs;
            selectionInputs.reserve(outSources.size());
            std::uint32_t cosmeticCount = 0;
            std::uint32_t gameplayCriticalCount = 0;
            for (std::size_t i = 0; i < outSources.size(); ++i) {
                const auto& source = outSources[i];
                if (source.semantic.cosmetic) {
                    ++cosmeticCount;
                }
                if (source.semantic.gameplayCritical) {
                    ++gameplayCriticalCount;
                }
                selectionInputs.push_back(WeaponSemanticHullBudgetInput{
                    .semantic = source.semantic,
                    .sourceIndex = i,
                    .sourceGroupId = source.sourceGroupId,
                    .pointCount = source.localPointsGame.size(),
                });
            }

            const auto selectedIndices = selectSemanticWeaponHullIndices(selectionInputs, MAX_WEAPON_BODIES);
            std::unordered_set<std::size_t> selectedIndexSet(selectedIndices.begin(), selectedIndices.end());
            for (std::size_t i = 0; i < outSources.size(); ++i) {
                if (selectedIndexSet.find(i) == selectedIndexSet.end()) {
                    const auto& dropped = outSources[i];
                    ROCK_LOG_DEBUG(Weapon,
                        "Generated weapon mesh hull budget dropped source='{}' root='{}' partKind={} points={} sourceGroup={:x}",
                        dropped.sourceName,
                        safeNodeName(dropped.sourceRoot),
                        static_cast<int>(dropped.semantic.partKind),
                        dropped.localPointsGame.size(),
                        dropped.sourceGroupId);
                }
            }

            std::vector<GeneratedHullSource> selectedSources;
            selectedSources.reserve(selectedIndices.size());
            for (const std::size_t selectedIndex : selectedIndices) {
                if (selectedIndex < outSources.size()) {
                    selectedSources.push_back(std::move(outSources[selectedIndex]));
                }
            }

            ROCK_LOG_DEBUG(Weapon,
                "Generated weapon mesh hull budget: extracted={} kept={} dropped={} gameplayCritical={} cosmetic={} maxBodies={} policy=semantic-priority",
                outSources.size(), selectedSources.size(), outSources.size() - selectedSources.size(), gameplayCriticalCount, cosmeticCount, MAX_WEAPON_BODIES);

            outSources = std::move(selectedSources);
        }

        for (std::size_t i = 0; i < outSources.size(); ++i) {
            const auto coverage = classifyGeneratedHull(outSources[i].sourceName);
            ROCK_LOG_TRACE(Weapon,
                "Generated weapon mesh selected[{}]: category={} source='{}' points={} center=({:.2f},{:.2f},{:.2f}) boundsMin=({:.2f},{:.2f},{:.2f}) boundsMax=({:.2f},{:.2f},{:.2f})",
                i, coverage.label, outSources[i].sourceName, outSources[i].localPointsGame.size(), outSources[i].localCenterGame.x, outSources[i].localCenterGame.y,
                outSources[i].localCenterGame.z, outSources[i].localMinGame.x, outSources[i].localMinGame.y, outSources[i].localMinGame.z, outSources[i].localMaxGame.x,
                outSources[i].localMaxGame.y, outSources[i].localMaxGame.z);
        }

        ROCK_LOG_DEBUG(Weapon, "Generated weapon mesh source selected: label='{}' root='{}' hulls={}", selectedLabel, safeNodeName(selectedRoot), outSources.size());
        return outSources.size();
    }

    void WeaponCollision::findGeneratedWeaponShapeSourcesRecursive(RE::NiAVObject* node, RE::NiAVObject* sourceRoot, const RE::NiTransform& weaponRootTransform, int depth,
        std::vector<GeneratedHullSource>& outSources, std::uint32_t& visitedShapes, std::uint32_t& extractedTriangles)
    {
        if (!node || depth > 15) {
            return;
        }
        if (!weaponVisualNodeVisible(node)) {
            return;
        }

        auto* triShape = node->IsTriShape();
        if (triShape) {
            ++visitedShapes;

            std::vector<TriangleData> triangles;
            const bool skinned = isSkinned(triShape);
            const int added = skinned ? extractTrianglesFromSkinnedTriShape(triShape, triangles) : extractTrianglesFromTriShape(triShape, triangles);
            if (added <= 0) {
                ROCK_LOG_TRACE(Weapon, "{}generated mesh source skipped '{}': no extractable triangles", std::string(depth * 2, ' '), safeNodeName(node));
                return;
            }
            extractedTriangles += static_cast<std::uint32_t>(added);

            std::vector<RE::NiPoint3> localPoints;
            localPoints.reserve(triangles.size() * 3);
            for (const auto& triangle : triangles) {
                localPoints.push_back(weapon_collision_geometry_math::worldPointToLocal(weaponRootTransform.rotate, weaponRootTransform.translate, weaponRootTransform.scale, triangle.v0));
                localPoints.push_back(weapon_collision_geometry_math::worldPointToLocal(weaponRootTransform.rotate, weaponRootTransform.translate, weaponRootTransform.scale, triangle.v1));
                localPoints.push_back(weapon_collision_geometry_math::worldPointToLocal(weaponRootTransform.rotate, weaponRootTransform.translate, weaponRootTransform.scale, triangle.v2));
            }

            const float dedupGridGame = (std::max)(g_rockConfig.rockWeaponCollisionPointDedupGrid * havokToGameScale(), 0.01f);
            localPoints = dedupePointCloud(localPoints, dedupGridGame);
            if (!pointCloudCanBuildHull(localPoints)) {
                ROCK_LOG_TRACE(Weapon, "{}generated mesh source skipped '{}': degenerate point cloud points={}", std::string(depth * 2, ' '), safeNodeName(node),
                    localPoints.size());
                return;
            }

            std::vector<std::vector<RE::NiPoint3>> clusters;
            weapon_collision_geometry_math::splitOversizedCluster(localPoints, MAX_CONVEX_HULL_POINTS, clusters);
            for (std::size_t clusterIndex = 0; clusterIndex < clusters.size(); ++clusterIndex) {
                auto cluster = weapon_collision_geometry_math::limitPointCloud(std::move(clusters[clusterIndex]), MAX_CONVEX_HULL_POINTS);
                if (!pointCloudCanBuildHull(cluster)) {
                    continue;
                }

                GeneratedHullSource source;
                source.localCenterGame = weapon_collision_geometry_math::pointCenter(cluster);
                const auto bounds = pointCloudBounds(cluster);
                source.localMinGame = bounds.min;
                source.localMaxGame = bounds.max;
                source.localPointsGame = std::move(cluster);
                source.driveRoot = sourceRoot;
                source.sourceRoot = sourceRoot;
                source.sourceGroupId = reinterpret_cast<std::uintptr_t>(node);
                source.sourceName = safeNodeName(node);
                if (clusters.size() > 1) {
                    source.sourceName += "#";
                    source.sourceName += std::to_string(clusterIndex);
                }
                source.semantic = classifyWeaponPartName(source.sourceName);
                ROCK_LOG_TRACE(Weapon, "{}generated mesh source '{}': points={} center=({:.2f},{:.2f},{:.2f})", std::string(depth * 2, ' '), source.sourceName,
                    source.localPointsGame.size(), source.localCenterGame.x, source.localCenterGame.y, source.localCenterGame.z);
                outSources.push_back(std::move(source));
            }
            return;
        }

        auto* niNode = node->IsNode();
        if (niNode) {
            auto& kids = niNode->GetRuntimeData().children;
            for (std::uint16_t i = 0; i < kids.size(); ++i) {
                if (auto* kid = kids[i].get()) {
                    findGeneratedWeaponShapeSourcesRecursive(kid, sourceRoot, weaponRootTransform, depth + 1, outSources, visitedShapes, extractedTriangles);
                }
            }
        }
    }

    RE::NiTransform WeaponCollision::makeGeneratedBodyWorldTransform(const RE::NiTransform& weaponRootTransform, const RE::NiPoint3& localCenterGame) const
    {
        RE::NiTransform result = weaponRootTransform;
        result.rotate = weaponRootTransform.rotate;
        result.translate = weapon_collision_geometry_math::localPointToWorld(weaponRootTransform.rotate, weaponRootTransform.translate, weaponRootTransform.scale, localCenterGame);
        return result;
    }

    bool WeaponCollision::weaponCollisionSettingsChanged() const
    {
        if (_cachedConvexRadius < 0.0f || _cachedPointDedupGrid < 0.0f || _cachedGroupingMode < 0) {
            return false;
        }
        return std::abs(g_rockConfig.rockWeaponCollisionConvexRadius - _cachedConvexRadius) > 0.00001f ||
               std::abs(g_rockConfig.rockWeaponCollisionPointDedupGrid - _cachedPointDedupGrid) > 0.00001f ||
               weapon_collision_grouping_policy::weaponCollisionGroupingModeChanged(_cachedGroupingMode, g_rockConfig.rockWeaponCollisionGroupingMode);
    }

    std::size_t WeaponCollision::createGeneratedWeaponBodiesInBank(RE::hknpWorld* world,
        const std::vector<GeneratedHullSource>& sources,
        WeaponBodyBank& bank,
        bool publishAfterCreate)
    {
        if (bankHasWeaponBody(bank)) {
            ROCK_LOG_WARN(Weapon, "createGeneratedWeaponBodiesInBank called but target bank already has bodies — skipping");
            return 0;
        }
        if (!world || !_cachedBhkWorld || sources.empty()) {
            return 0;
        }

        std::size_t createdCount = 0;
        const std::uint32_t filterInfo = generatedWeaponCollisionFilterInfo(publishAfterCreate);
        auto buildSourceShape = [&](const GeneratedHullSource& source) -> RE::hknpShape* {
            if (source.childLocalPointCloudsGame.size() <= 1) {
                auto centeredHavokPoints = makeCenteredHavokPointCloud(source.localPointsGame, source.localCenterGame);
                return havok_convex_shape_builder::buildConvexShapeFromLocalHavokPoints(centeredHavokPoints, g_rockConfig.rockWeaponCollisionConvexRadius);
            }

            std::vector<RE::hknpShape*> childShapes;
            std::vector<havok_compound_shape_builder::CompoundChild> children;
            childShapes.reserve(source.childLocalPointCloudsGame.size());
            children.reserve(source.childLocalPointCloudsGame.size());

            for (const auto& childLocalPointsGame : source.childLocalPointCloudsGame) {
                if (!pointCloudCanBuildHull(childLocalPointsGame)) {
                    continue;
                }

                const auto childCenterGame = weapon_collision_geometry_math::pointCenter(childLocalPointsGame);
                auto centeredChildHavokPoints = makeCenteredHavokPointCloud(childLocalPointsGame, childCenterGame);
                auto* childShape =
                    havok_convex_shape_builder::buildConvexShapeFromLocalHavokPoints(centeredChildHavokPoints, g_rockConfig.rockWeaponCollisionConvexRadius);
                if (!childShape) {
                    ROCK_LOG_WARN(Weapon, "Generated weapon compound source '{}' failed child convex build", source.sourceName);
                    continue;
                }

                childShapes.push_back(childShape);

                havok_compound_shape_builder::CompoundChild child{};
                child.shape = childShape;
                child.transform.translation.x = (childCenterGame.x - source.localCenterGame.x) * gameToHavokScale();
                child.transform.translation.y = (childCenterGame.y - source.localCenterGame.y) * gameToHavokScale();
                child.transform.translation.z = (childCenterGame.z - source.localCenterGame.z) * gameToHavokScale();
                child.transform.translation.w = 1.0f;
                children.push_back(child);
            }

            RE::hknpShape* compoundShape = nullptr;
            if (children.size() > 1) {
                compoundShape = havok_compound_shape_builder::buildStaticCompoundShape(children);
            } else if (children.size() == 1) {
                compoundShape = const_cast<RE::hknpShape*>(children.front().shape);
                childShapes.clear();
            }

            for (auto* childShape : childShapes) {
                shapeRemoveRef(childShape);
            }

            return compoundShape;
        };

        for (std::size_t sourceIndex = 0; sourceIndex < sources.size() && createdCount < MAX_WEAPON_BODIES; ++sourceIndex) {
            const auto& source = sources[sourceIndex];
            if (!pointCloudCanBuildHull(source.localPointsGame)) {
                continue;
            }

            auto* shape = buildSourceShape(source);
            if (!shape) {
                ROCK_LOG_WARN(Weapon, "Generated weapon mesh hull '{}' failed native shape build", source.sourceName);
                continue;
            }

            auto& instance = bank[createdCount];
            instance.shape = shape;
            instance.driveNode = source.driveRoot ? source.driveRoot : source.sourceRoot;
            instance.sourceNode = source.sourceRoot;
            instance.sourceName = source.sourceName;
            instance.sourceRootName = source.sourceRoot ? safeNodeName(source.sourceRoot) : "";
            instance.generatedLocalCenterGame = source.localCenterGame;
            instance.generatedLocalMinGame = source.localMinGame;
            instance.generatedLocalMaxGame = source.localMaxGame;
            instance.generatedLocalPointsGame = source.localPointsGame;
            instance.generatedPointCount = static_cast<std::uint32_t>(
                (std::min)(source.localPointsGame.size(), static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)())));
            instance.semantic = source.semantic;
            instance.ownsShapeRef = true;
            clearGeneratedKeyframedBodyDriveState(instance.driveState);

            const bool ok =
                instance.body.create(world, _cachedBhkWorld, shape, filterInfo, { 0 }, BethesdaMotionType::Keyframed, "ROCK_WeaponMeshCollision");

            if (!ok) {
                ROCK_LOG_ERROR(Weapon, "BethesdaPhysicsBody::create failed for generated weapon mesh hull '{}'", source.sourceName);
                shapeRemoveRef(shape);
                clearWeaponBodyInstance(instance, false);
                continue;
            }

            instance.body.createNiNode("ROCK_WeaponMeshCollision");
            const RE::NiTransform driveRootTransform = instance.driveNode ? instance.driveNode->world : makeIdentityTransform();
            const RE::NiTransform initialTransform = makeGeneratedBodyWorldTransform(driveRootTransform, source.localCenterGame);
            if (!placeGeneratedKeyframedBodyImmediately(instance.body, initialTransform)) {
                ROCK_LOG_ERROR(Weapon,
                    "Generated weapon mesh collision initial placement failed meshIndex={} bodyId={} source='{}' root='{}'",
                    createdCount,
                    instance.body.getBodyId().value,
                    source.sourceName,
                    safeNodeName(source.sourceRoot));
                instance.body.destroy(_cachedBhkWorld);
                shapeRemoveRef(shape);
                clearWeaponBodyInstance(instance, false);
                continue;
            }
            instance.driveState.previousTargetPosition = initialTransform.translate;
            instance.driveState.hasPreviousTarget = true;

            ROCK_LOG_DEBUG(Weapon,
                "Generated weapon mesh collision body created — meshIndex={} bodyId={} source='{}' root='{}' partKind={} supportRole={} reloadRole={} points={} children={} center=({:.2f},{:.2f},{:.2f}) layer=44",
                createdCount, instance.body.getBodyId().value, source.sourceName, safeNodeName(source.sourceRoot), static_cast<int>(source.semantic.partKind),
                static_cast<int>(source.semantic.supportGripRole), static_cast<int>(source.semantic.reloadRole), source.localPointsGame.size(),
                source.childLocalPointCloudsGame.size(), source.localCenterGame.x, source.localCenterGame.y, source.localCenterGame.z);
            ++createdCount;
        }

        if (publishAfterCreate) {
            publishAtomicBodyIds(bank);
        }
        if (createdCount > 0) {
            _driveRebuildRequested.store(false, std::memory_order_release);
            _driveFailureCount.store(0, std::memory_order_release);
        }
        ROCK_LOG_INFO(Weapon, "Generated weapon mesh collision created {}/{} hull bodies", createdCount, sources.size());
        return createdCount;
    }

    void WeaponCollision::destroyWeaponBody(RE::hknpWorld* world)
    {
        if (!bankHasWeaponBody(_weaponBodies) && !bankHasWeaponBody(_weaponReplacementBodies))
            return;

        if (_dominantHandDisabled && world) {
            enableDominantHandCollision(world);
        }

        clearAtomicBodyIds();

        const auto activeDestroyed = bankWeaponBodyCount(activeWeaponBodies());
        const auto inactiveDestroyed = bankWeaponBodyCount(inactiveWeaponBodies());
        destroyWeaponBodyBank(activeWeaponBodies(), true);
        destroyWeaponBodyBank(inactiveWeaponBodies(), true);
        _usingReplacementWeaponBodies = false;
        _driveRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);

        ROCK_LOG_INFO(Weapon, "Weapon collision bodies destroyed count={}", activeDestroyed + inactiveDestroyed);
    }

    void WeaponCollision::invalidateForScaleChange(RE::hknpWorld* world)
    {
        const auto decision = weapon_authority_lifecycle_policy::evaluateGeneratedCollisionScaleInvalidation(hasWeaponBody(), _cachedWeaponKey, _pendingWeaponKey);

        if (decision.destroyExistingBodies) {
            ROCK_LOG_INFO(Weapon, "Generated weapon collision invalidated by physics scale change — pendingKey={:016X} reason={}", decision.pendingKey, decision.reason);
            destroyWeaponBody(world);
        } else {
            clearAtomicBodyIds();
            if (_dominantHandDisabled && world) {
                enableDominantHandCollision(world);
            }
            ROCK_LOG_DEBUG(Weapon, "Generated weapon collision scale invalidation had no active bodies — pendingKey={:016X} reason={}", decision.pendingKey, decision.reason);
        }

        if (decision.resetCachedIdentity) {
            _cachedWeaponKey = 0;
            _pendingWeaponKey = decision.pendingKey;
            _visualSettleState = {};
            _weaponBodyPending = decision.pending;
        }
        if (decision.resetCachedSettings) {
            _cachedConvexRadius = -1.0f;
            _cachedPointDedupGrid = -1.0f;
            _cachedGroupingMode = -1;
        }
        _retryCounter = 0;
    }

    void WeaponCollision::destroyWeaponBodyBank(WeaponBodyBank& bank, bool releaseShapeRef)
    {
        for (auto& instance : bank) {
            if (instance.body.isValid()) {
                instance.body.destroy(_cachedBhkWorld);
            }
            clearWeaponBodyInstance(instance, releaseShapeRef);
        }
    }

    void WeaponCollision::setWeaponBodyBankCollisionEnabled(RE::hknpWorld* world, WeaponBodyBank& bank, bool enabled)
    {
        if (!world) {
            return;
        }

        const std::uint32_t filterInfo = generatedWeaponCollisionFilterInfo(enabled);
        for (auto& instance : bank) {
            if (instance.body.isValid()) {
                body_collision::setFilterInfo(world, instance.body.getBodyId(), filterInfo);
            }
        }
    }

    void WeaponCollision::clearWeaponBodyInstance(WeaponBodyInstance& instance, bool releaseShapeRef)
    {
        if (releaseShapeRef && instance.ownsShapeRef && instance.shape) {
            shapeRemoveRef(instance.shape);
        }
        instance.body.reset();
        instance.shape = nullptr;
        instance.driveNode = nullptr;
        instance.sourceNode = nullptr;
        instance.sourceName.clear();
        instance.sourceRootName.clear();
        instance.generatedLocalCenterGame = {};
        instance.generatedLocalMinGame = {};
        instance.generatedLocalMaxGame = {};
        instance.generatedLocalPointsGame.clear();
        instance.generatedPointCount = 0;
        instance.semantic = {};
        instance.ownsShapeRef = false;
        clearGeneratedKeyframedBodyDriveState(instance.driveState);
    }

    void WeaponCollision::clearAtomicBodyIds()
    {
        _weaponBodyCountAtomic.store(0, std::memory_order_release);
        for (auto& id : _weaponBodyIdsAtomic) {
            id.store(INVALID_BODY_ID, std::memory_order_release);
        }
        for (auto& value : _weaponBodyPartKindsAtomic) {
            value.store(static_cast<std::uint32_t>(WeaponPartKind::Other), std::memory_order_release);
        }
        for (auto& value : _weaponBodyReloadRolesAtomic) {
            value.store(static_cast<std::uint32_t>(WeaponReloadRole::None), std::memory_order_release);
        }
        for (auto& value : _weaponBodySupportRolesAtomic) {
            value.store(static_cast<std::uint32_t>(WeaponSupportGripRole::None), std::memory_order_release);
        }
        for (auto& value : _weaponBodySocketRolesAtomic) {
            value.store(static_cast<std::uint32_t>(WeaponSocketRole::None), std::memory_order_release);
        }
        for (auto& value : _weaponBodyActionRolesAtomic) {
            value.store(static_cast<std::uint32_t>(WeaponActionRole::None), std::memory_order_release);
        }
        for (auto& value : _weaponBodyGripPosesAtomic) {
            value.store(static_cast<std::uint32_t>(WeaponGripPoseId::None), std::memory_order_release);
        }
        for (auto& value : _weaponBodyInteractionRootsAtomic) {
            value.store(0, std::memory_order_release);
        }
        for (auto& value : _weaponBodySourceRootsAtomic) {
            value.store(0, std::memory_order_release);
        }
        for (auto& value : _weaponBodyGenerationKeysAtomic) {
            value.store(0, std::memory_order_release);
        }
    }

    void WeaponCollision::publishAtomicBodyIds(const WeaponBodyBank& bank)
    {
        std::uint32_t count = 0;
        clearAtomicBodyIds();
        for (const auto& instance : bank) {
            if (instance.body.isValid() && count < MAX_WEAPON_BODIES) {
                _weaponBodyPartKindsAtomic[count].store(static_cast<std::uint32_t>(instance.semantic.partKind), std::memory_order_release);
                _weaponBodyReloadRolesAtomic[count].store(static_cast<std::uint32_t>(instance.semantic.reloadRole), std::memory_order_release);
                _weaponBodySupportRolesAtomic[count].store(static_cast<std::uint32_t>(instance.semantic.supportGripRole), std::memory_order_release);
                _weaponBodySocketRolesAtomic[count].store(static_cast<std::uint32_t>(instance.semantic.socketRole), std::memory_order_release);
                _weaponBodyActionRolesAtomic[count].store(static_cast<std::uint32_t>(instance.semantic.actionRole), std::memory_order_release);
                _weaponBodyGripPosesAtomic[count].store(static_cast<std::uint32_t>(instance.semantic.fallbackGripPose), std::memory_order_release);
                _weaponBodyInteractionRootsAtomic[count].store(reinterpret_cast<std::uintptr_t>(instance.driveNode), std::memory_order_release);
                _weaponBodySourceRootsAtomic[count].store(reinterpret_cast<std::uintptr_t>(instance.sourceNode), std::memory_order_release);
                _weaponBodyGenerationKeysAtomic[count].store(_cachedWeaponKey, std::memory_order_release);
                _weaponBodyIdsAtomic[count].store(instance.body.getBodyId().value, std::memory_order_release);
                ++count;
            }
        }
        _weaponBodyCountAtomic.store(count, std::memory_order_release);
    }

    void WeaponCollision::updateBodiesFromCurrentSourceTransforms(RE::hknpWorld* world, RE::NiAVObject* fallbackWeaponNode)
    {
        if (!world || !hasWeaponBody()) {
            return;
        }

        auto& bank = activeWeaponBodies();
        for (std::size_t i = 0; i < bank.size(); ++i) {
            auto& instance = bank[i];
            if (!instance.body.isValid()) {
                continue;
            }

            const RE::NiTransform& driveRootTransform = instance.driveNode ? instance.driveNode->world : fallbackWeaponNode ? fallbackWeaponNode->world : makeIdentityTransform();
            const RE::NiTransform generatedTransform = makeGeneratedBodyWorldTransform(driveRootTransform, instance.generatedLocalCenterGame);
            queueBodyTarget(instance, generatedTransform);
        }
    }

    void WeaponCollision::queueBodyTarget(WeaponBodyInstance& instance, const RE::NiTransform& weaponTransform)
    {
        if (!instance.body.isValid()) {
            return;
        }

        queueGeneratedKeyframedBodyTarget(instance.driveState, weaponTransform, 1000.0f);
    }

    void WeaponCollision::flushPendingPhysicsDrive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!world || !hasWeaponBody()) {
            return;
        }

        auto& bank = activeWeaponBodies();
        for (std::size_t i = 0; i < bank.size(); ++i) {
            auto& instance = bank[i];
            if (!instance.body.isValid()) {
                continue;
            }
            const auto bodyIndex = static_cast<std::uint32_t>(i);
            handleGeneratedBodyDriveResult(
                driveGeneratedKeyframedBody(world, instance.body, instance.driveState, timing, "weapon-collision", bodyIndex),
                "weapon-collision",
                bodyIndex);
        }
    }

    void WeaponCollision::handleGeneratedBodyDriveResult(const GeneratedKeyframedBodyDriveResult& result, const char* ownerName, std::uint32_t bodyIndex)
    {
        if (!result.attempted || result.skippedStale) {
            return;
        }

        if (result.driven) {
            _driveFailureCount.store(0, std::memory_order_release);
            return;
        }

        if (!result.shouldRequestRebuild()) {
            return;
        }

        const auto failures = _driveFailureCount.fetch_add(1, std::memory_order_acq_rel) + 1;
        _driveRebuildRequested.store(true, std::memory_order_release);
        ROCK_LOG_SAMPLE_WARN(Weapon,
            g_rockConfig.rockLogSampleMilliseconds,
            "Weapon generated collider drive result requested rebuild owner={} bodyIndex={} failures={} missingBody={} placementFailed={} nativeDriveFailed={} bodyDeltaGame={:.2f}",
            ownerName ? ownerName : "unknown",
            bodyIndex,
            failures,
            result.missingBody ? "yes" : "no",
            result.placementFailed ? "yes" : "no",
            result.nativeDriveFailed ? "yes" : "no",
            result.hasLiveBodyTransform ? result.bodyDeltaGameUnits : -1.0f);
    }

    void WeaponCollision::disableDominantHandCollision(RE::hknpWorld* world, RE::hknpBodyId handBodyId)
    {
        if (!world || handBodyId.value == INVALID_BODY_ID)
            return;

        const auto result = collision_suppression_registry::globalCollisionSuppressionRegistry().acquire(
            world,
            handBodyId.value,
            collision_suppression_registry::CollisionSuppressionOwner::WeaponDominantHand,
            "weapon-dominant-hand");
        if (result.readFailed || !result.valid) {
            return;
        }

        _dominantHandDisabled = true;
        _disabledHandBodyId = handBodyId;

        ROCK_LOG_DEBUG(Weapon,
            "Dominant hand collision lease acquired bodyId={} filter=0x{:08X}->0x{:08X} wasDisabledBefore={} leases={}",
            handBodyId.value,
            result.filterBefore,
            result.filterAfter,
            result.wasNoCollideBeforeSuppression ? "yes" : "no",
            result.activeLeaseCount);
    }

    void WeaponCollision::enableDominantHandCollision(RE::hknpWorld* world)
    {
        if (!world || _disabledHandBodyId.value == INVALID_BODY_ID)
            return;

        const auto result = collision_suppression_registry::globalCollisionSuppressionRegistry().release(
            world,
            _disabledHandBodyId.value,
            collision_suppression_registry::CollisionSuppressionOwner::WeaponDominantHand,
            "weapon-dominant-hand");
        if (result.readFailed) {
            ROCK_LOG_WARN(Weapon, "Dominant hand collision restore deferred bodyId={}", _disabledHandBodyId.value);
            return;
        }

        ROCK_LOG_DEBUG(Weapon,
            "Dominant hand collision lease released bodyId={} filter=0x{:08X}->0x{:08X} restoreDisabled={} fullyReleased={}",
            _disabledHandBodyId.value,
            result.filterBefore,
            result.filterAfter,
            result.wasNoCollideBeforeSuppression ? "yes" : "no",
            result.bodyFullyReleased ? "yes" : "no");

        _dominantHandDisabled = false;
        _disabledHandBodyId.value = INVALID_BODY_ID;
    }

}
