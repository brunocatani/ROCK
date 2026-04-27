

#include "WeaponCollision.h"

#include "BodyCollisionControl.h"
#include "HavokConvexShapeBuilder.h"
#include "HavokOffsets.h"
#include "MeshGrab.h"
#include "RockConfig.h"
#include "WeaponCollisionAdjustmentMath.h"
#include "WeaponCollisionGeometryMath.h"

#include <intrin.h>

#include "RE/Havok/hkReferencedObject.h"
#include "RE/Havok/hknpCapsuleShape.h"
#include "RE/Havok/hknpMotion.h"

#include "f4vr/PlayerNodes.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <unordered_set>
#include <string>
#include <string_view>
#include <vector>

namespace frik::rock
{
    namespace
    {
        constexpr std::size_t MAX_CONVEX_HULL_POINTS = 0xFC;
        constexpr float MIN_HULL_DIAGONAL_GAME_UNITS = 0.5f;
        constexpr std::uint64_t WEAPON_VISUAL_KEY_OFFSET = 1469598103934665603ull;
        constexpr std::uint64_t WEAPON_VISUAL_KEY_PRIME = 1099511628211ull;

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
            key ^= value;
            key *= WEAPON_VISUAL_KEY_PRIME;
        }

        void mixWeaponVisualString(std::uint64_t& key, const char* value)
        {
            if (!value) {
                return;
            }
            for (const auto* cursor = reinterpret_cast<const unsigned char*>(value); *cursor; ++cursor) {
                mixWeaponVisualKey(key, static_cast<std::uint64_t>(*cursor));
            }
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
            if (sourceNameContains(sourceName, "cartridge") || sourceNameContains(sourceName, "casing") || sourceNameContains(sourceName, "bullet") ||
                sourceNameContains(sourceName, "shell") || sourceNameContains(sourceName, "ammo")) {
                return { HullCoverageCosmeticAmmo, 90, true, "cosmetic-ammo" };
            }
            if (sourceNameContains(sourceName, "stock") || sourceNameContains(sourceName, "butt") || sourceNameContains(sourceName, "grip")) {
                return { HullCoverageStock, 10, false, "stock/grip" };
            }
            if (sourceNameContains(sourceName, "barrel") || sourceNameContains(sourceName, "muzzle") || sourceNameContains(sourceName, "suppressor") ||
                sourceNameContains(sourceName, "silencer")) {
                return { HullCoverageBarrel, 10, false, "barrel/muzzle" };
            }
            if (sourceNameContains(sourceName, "receiver") || sourceNameContains(sourceName, "frame") || sourceNameContains(sourceName, "body") ||
                sourceNameContains(sourceName, "machinegun:") || sourceNameContains(sourceName, "rifle:") || sourceNameContains(sourceName, "pistol:") ||
                sourceNameContains(sourceName, "shotgun:")) {
                return { HullCoverageReceiver, 15, false, "receiver/body" };
            }
            if (sourceNameContains(sourceName, "magazine") || sourceNameContains(sourceName, "magmedium") || sourceNameContains(sourceName, "magshort") ||
                sourceNameContains(sourceName, "maglong") || sourceNameContains(sourceName, "magstraight") || sourceNameContains(sourceName, "magdrum") ||
                sourceNameContains(sourceName, "magwell") || sourceNameContains(sourceName, "clip")) {
                return { HullCoverageMagazine, 20, false, "magazine" };
            }
            if (sourceNameContains(sourceName, "handle") || sourceNameContains(sourceName, "sight") || sourceNameContains(sourceName, "scope") ||
                sourceNameContains(sourceName, "rail")) {
                return { HullCoverageTopAccessory, 30, false, "top/accessory" };
            }
            if (sourceNameContains(sourceName, "bolt") || sourceNameContains(sourceName, "slide") || sourceNameContains(sourceName, "charging")) {
                return { HullCoverageAction, 35, false, "action" };
            }
            return { HullCoverageOther, 50, false, "other" };
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
                result.emplace_back((point.x - localCenterGame.x) * kGameToHavokScale, (point.y - localCenterGame.y) * kGameToHavokScale,
                    (point.z - localCenterGame.z) * kGameToHavokScale);
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

        void accumulateWeaponVisualKey(RE::NiAVObject* node, int depth, std::uint64_t& key, WeaponVisualKeyStats& stats)
        {
            if (!node || depth > 15 || stats.nodeCount > 512) {
                return;
            }

            ++stats.nodeCount;
            mixWeaponVisualKey(key, reinterpret_cast<std::uintptr_t>(node));
            mixWeaponVisualKey(key, static_cast<std::uint64_t>(depth));
            mixWeaponVisualString(key, safeNodeName(node));

            if (node->IsTriShape()) {
                ++stats.triShapeCount;
                mixWeaponVisualKey(key, 0x7472697368617065ull);
                return;
            }

            auto* niNode = node->IsNode();
            if (!niNode) {
                return;
            }

            auto& kids = niNode->GetRuntimeData().children;
            mixWeaponVisualKey(key, static_cast<std::uint64_t>(kids.size()));
            for (std::uint16_t i = 0; i < kids.size(); ++i) {
                auto* kid = kids[i].get();
                mixWeaponVisualKey(key, static_cast<std::uint64_t>(i));
                mixWeaponVisualKey(key, reinterpret_cast<std::uintptr_t>(kid));
                accumulateWeaponVisualKey(kid, depth + 1, key, stats);
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

    WeaponCollision::WeaponCollision() { clearAtomicBodyIds(); }

    bool WeaponCollision::hasWeaponBody() const
    {
        for (const auto& instance : _weaponBodies) {
            if (instance.body.isValid()) {
                return true;
            }
        }
        return false;
    }

    std::uint32_t WeaponCollision::getWeaponBodyCount() const
    {
        std::uint32_t count = 0;
        for (const auto& instance : _weaponBodies) {
            if (instance.body.isValid()) {
                ++count;
            }
        }
        return count;
    }

    RE::hknpBodyId WeaponCollision::getWeaponBodyId() const
    {
        for (const auto& instance : _weaponBodies) {
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

    BethesdaPhysicsBody& WeaponCollision::getWeaponBody()
    {
        for (auto& instance : _weaponBodies) {
            if (instance.body.isValid()) {
                return instance.body;
            }
        }
        return _weaponBodies[0].body;
    }

    void WeaponCollision::init(RE::hknpWorld* world, void* bhkWorld)
    {
        // Cache the Havok context even while the feature is disabled so the INI
        // watcher can hot-enable weapon collision without requiring a physics
        // module restart.
        _cachedWorld = world;
        _cachedBhkWorld = bhkWorld;
        _cachedWeaponKey = 0;
        _weaponBodyPending = false;
        _retryCounter = 0;
        _cachedConvexRadius = -1.0f;
        _cachedPointDedupGrid = -1.0f;
        clearAtomicBodyIds();

        if (!g_rockConfig.rockWeaponCollisionEnabled) {
            ROCK_LOG_INFO(Weapon, "WeaponCollision disabled via config — context cached for hot reload");
            return;
        }

        ROCK_LOG_INFO(Weapon, "WeaponCollision initialized");
    }

    void WeaponCollision::shutdown()
    {
        resetWeaponBodiesWithoutDestroy();

        _cachedWeaponKey = 0;
        _cachedWorld = nullptr;
        _cachedBhkWorld = nullptr;
        _weaponBodyPending = false;
        _retryCounter = 0;
        _cachedConvexRadius = -1.0f;
        _cachedPointDedupGrid = -1.0f;
        _dominantHandDisabled = false;
        hand_collision_suppression_math::clear(_dominantHandSuppression);
        _dominantHandBroadPhaseSuppressed = false;
        _disabledHandBodyId.value = INVALID_BODY_ID;

        ROCK_LOG_INFO(Weapon, "WeaponCollision shutdown");
    }

    void WeaponCollision::update(RE::hknpWorld* world, RE::NiAVObject* weaponNode, RE::hknpBodyId dominantHandBodyId, float dt)
    {
        if (!g_rockConfig.rockWeaponCollisionEnabled) {
            if (hasWeaponBody() && world) {
                ROCK_LOG_INFO(Weapon, "WeaponCollision disabled via hot reload — destroying generated weapon bodies");
                destroyWeaponBody(world);
            }
            if (_dominantHandDisabled && world) {
                enableDominantHandCollision(world);
            }
            _cachedWeaponKey = 0;
            _weaponBodyPending = false;
            _retryCounter = 0;
            _cachedConvexRadius = -1.0f;
            _cachedPointDedupGrid = -1.0f;
            return;
        }
        if (!world)
            return;

        if (world != _cachedWorld) {
            ROCK_LOG_INFO(Weapon, "hknpWorld changed — resetting weapon collision state");
            resetWeaponBodiesWithoutDestroy();
            _cachedWeaponKey = 0;
            _weaponBodyPending = false;
            _cachedWorld = world;
            _cachedConvexRadius = -1.0f;
            _cachedPointDedupGrid = -1.0f;
        }

        if (!weaponNode) {
            if (hasWeaponBody()) {
                ROCK_LOG_INFO(Weapon, "Weapon node gone (holstered?) — destroying weapon bodies");
                destroyWeaponBody(world);
            }
            if (_dominantHandDisabled) {
                enableDominantHandCollision(world);
            }
            return;
        }

        WeaponVisualKeyStats visualKeyStats{};
        std::uint64_t currentKey = getEquippedWeaponKey(weaponNode, visualKeyStats);

        if (currentKey != _cachedWeaponKey) {
            ROCK_LOG_INFO(Weapon, "Weapon changed: {:016X} -> {:016X} visualRoots={} visualNodes={} visualTriShapes={}", _cachedWeaponKey, currentKey,
                visualKeyStats.rootCount, visualKeyStats.nodeCount, visualKeyStats.triShapeCount);

            if (hasWeaponBody()) {
                destroyWeaponBody(world);
            }

            _cachedWeaponKey = currentKey;
            _weaponBodyPending = (currentKey != 0);
            _cachedConvexRadius = -1.0f;
            _cachedPointDedupGrid = -1.0f;
        }

        if (hasWeaponBody() && weaponCollisionSettingsChanged()) {
            ROCK_LOG_INFO(Weapon, "Generated weapon collision settings changed — rebuilding weapon bodies");
            destroyWeaponBody(world);
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
                if (generatedCount > 0) {
                    createGeneratedWeaponBodies(world, generatedSources);
                }
                const bool created = hasWeaponBody();

                if (created) {
                    _weaponBodyPending = false;
                    _retryCounter = 0;
                    _cachedConvexRadius = g_rockConfig.rockWeaponCollisionConvexRadius;
                    _cachedPointDedupGrid = g_rockConfig.rockWeaponCollisionPointDedupGrid;
                } else {
                    ROCK_LOG_WARN(Weapon, "Generated weapon mesh collision unavailable — keeping weapon pending for retry");
                    _weaponBodyPending = true;
                }
            }
        }

        if (hasWeaponBody() && !_dominantHandDisabled && dominantHandBodyId.value != INVALID_BODY_ID) {
            disableDominantHandCollision(world, dominantHandBodyId);
        } else if (!hasWeaponBody() && _dominantHandDisabled) {
            enableDominantHandCollision(world);
        }

        if (hasWeaponBody() && weaponNode) {
            for (std::size_t i = 0; i < _weaponBodies.size(); ++i) {
                auto& instance = _weaponBodies[i];
                if (!instance.body.isValid()) {
                    continue;
                }
                const RE::NiTransform& sourceRootTransform = instance.sourceNode ? instance.sourceNode->world : weaponNode->world;
                const RE::NiTransform generatedTransform = makeGeneratedBodyWorldTransform(sourceRootTransform, instance.generatedLocalCenterGame);
                updateBodyTransform(world, instance, generatedTransform, dt, i);
            }
        }
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
        std::uint64_t key = WEAPON_VISUAL_KEY_OFFSET;
        const auto candidates = makeGeneratedWeaponMeshRootCandidates(weaponNode);
        for (const auto& candidate : candidates) {
            if (!candidate.root) {
                continue;
            }

            ++stats.rootCount;
            mixWeaponVisualString(key, candidate.label);
            mixWeaponVisualKey(key, reinterpret_cast<std::uintptr_t>(candidate.root));
            accumulateWeaponVisualKey(candidate.root, 0, key, stats);
        }

        return key != WEAPON_VISUAL_KEY_OFFSET ? key : reinterpret_cast<std::uint64_t>(weaponNode);
    }

    std::size_t WeaponCollision::findGeneratedWeaponShapeSources(RE::NiAVObject* weaponNode, std::vector<GeneratedHullSource>& outSources)
    {
        outSources.clear();
        const auto candidates = makeGeneratedWeaponMeshRootCandidates(weaponNode);
        if (candidates.empty()) {
            ROCK_LOG_INFO(Weapon, "Generated weapon mesh source scan: no weapon root candidates");
            return 0;
        }

        RE::NiAVObject* selectedRoot = nullptr;
        const char* selectedLabel = "";
        for (const auto& candidate : candidates) {
            std::vector<GeneratedHullSource> candidateSources;
            std::uint32_t visitedShapes = 0;
            std::uint32_t extractedTriangles = 0;
            findGeneratedWeaponShapeSourcesRecursive(candidate.root, candidate.root, candidate.root->world, 0, candidateSources, visitedShapes, extractedTriangles);

            ROCK_LOG_INFO(Weapon,
                "Generated weapon mesh candidate: label='{}' root='{}' addr={:x} visitedShapes={} triangles={} hulls={}",
                candidate.label, safeNodeName(candidate.root), reinterpret_cast<std::uintptr_t>(candidate.root), visitedShapes, extractedTriangles, candidateSources.size());

            if (!candidateSources.empty()) {
                selectedRoot = candidate.root;
                selectedLabel = candidate.label;
                outSources = std::move(candidateSources);
                break;
            }
        }

        if (!selectedRoot) {
            ROCK_LOG_INFO(Weapon, "Generated weapon mesh source scan: all {} candidates produced zero hulls", candidates.size());
            return 0;
        }

        if (outSources.size() > MAX_WEAPON_BODIES) {
            std::vector<weapon_collision_geometry_math::HullSelectionInput> selectionInputs;
            selectionInputs.reserve(outSources.size());
            std::uint32_t cosmeticCount = 0;
            for (const auto& source : outSources) {
                const auto input = makeHullSelectionInput(
                    source.localCenterGame, source.localMinGame, source.localMaxGame, source.localPointsGame.size(), source.sourceName);
                if (input.cosmetic) {
                    ++cosmeticCount;
                }
                selectionInputs.push_back(input);
            }

            const int lengthAxis = weapon_collision_geometry_math::longestAxisForHullSelection(selectionInputs);
            const auto selectedIndices = weapon_collision_geometry_math::selectBalancedHullIndices(selectionInputs, MAX_WEAPON_BODIES);

            std::vector<GeneratedHullSource> selectedSources;
            selectedSources.reserve(selectedIndices.size());
            for (const std::size_t selectedIndex : selectedIndices) {
                if (selectedIndex < outSources.size()) {
                    selectedSources.push_back(std::move(outSources[selectedIndex]));
                }
            }

            ROCK_LOG_INFO(Weapon,
                "Generated weapon mesh hull budget: extracted={} kept={} dropped={} cosmeticDroppedOrDeferred={} lengthAxis={} maxBodies={} policy=coverage-balanced",
                outSources.size(), selectedSources.size(), outSources.size() - selectedSources.size(), cosmeticCount, hullAxisName(lengthAxis), MAX_WEAPON_BODIES);

            outSources = std::move(selectedSources);
        }

        for (std::size_t i = 0; i < outSources.size(); ++i) {
            const auto coverage = classifyGeneratedHull(outSources[i].sourceName);
            ROCK_LOG_INFO(Weapon,
                "Generated weapon mesh selected[{}]: category={} source='{}' points={} center=({:.2f},{:.2f},{:.2f}) boundsMin=({:.2f},{:.2f},{:.2f}) boundsMax=({:.2f},{:.2f},{:.2f})",
                i, coverage.label, outSources[i].sourceName, outSources[i].localPointsGame.size(), outSources[i].localCenterGame.x, outSources[i].localCenterGame.y,
                outSources[i].localCenterGame.z, outSources[i].localMinGame.x, outSources[i].localMinGame.y, outSources[i].localMinGame.z, outSources[i].localMaxGame.x,
                outSources[i].localMaxGame.y, outSources[i].localMaxGame.z);
        }

        ROCK_LOG_INFO(Weapon, "Generated weapon mesh source selected: label='{}' root='{}' hulls={}", selectedLabel, safeNodeName(selectedRoot), outSources.size());
        return outSources.size();
    }

    void WeaponCollision::findGeneratedWeaponShapeSourcesRecursive(RE::NiAVObject* node, RE::NiAVObject* sourceRoot, const RE::NiTransform& weaponRootTransform, int depth,
        std::vector<GeneratedHullSource>& outSources, std::uint32_t& visitedShapes, std::uint32_t& extractedTriangles)
    {
        if (!node || depth > 15) {
            return;
        }
        if (node->flags.flags & 1) {
            return;
        }

        auto* triShape = node->IsTriShape();
        if (triShape) {
            ++visitedShapes;

            std::vector<TriangleData> triangles;
            const bool skinned = isSkinned(triShape);
            const int added = skinned ? extractTrianglesFromSkinnedTriShape(triShape, triangles) : extractTrianglesFromTriShape(triShape, triangles);
            if (added <= 0) {
                ROCK_LOG_INFO(Weapon, "{}generated mesh source skipped '{}': no extractable triangles", std::string(depth * 2, ' '), safeNodeName(node));
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

            const float dedupGridGame = (std::max)(g_rockConfig.rockWeaponCollisionPointDedupGrid * kHavokToGameScale, 0.01f);
            localPoints = dedupePointCloud(localPoints, dedupGridGame);
            if (!pointCloudCanBuildHull(localPoints)) {
                ROCK_LOG_INFO(Weapon, "{}generated mesh source skipped '{}': degenerate point cloud points={}", std::string(depth * 2, ' '), safeNodeName(node),
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
                source.sourceRoot = sourceRoot;
                source.sourceName = safeNodeName(node);
                if (clusters.size() > 1) {
                    source.sourceName += "#";
                    source.sourceName += std::to_string(clusterIndex);
                }
                ROCK_LOG_INFO(Weapon, "{}generated mesh source '{}': points={} center=({:.2f},{:.2f},{:.2f})", std::string(depth * 2, ' '), source.sourceName,
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
        const RE::NiMatrix3 generatedPackageRotation = weapon_collision_geometry_math::transposeRotation(weaponRootTransform.rotate);
        RE::NiPoint3 correctedLocalCenter = localCenterGame;
        if (g_rockConfig.rockWeaponCollisionRotationCorrectionEnabled) {
            const auto correction = weapon_collision_adjustment_math::makeLocalEulerCorrection<RE::NiMatrix3>(g_rockConfig.rockWeaponCollisionRotationDegrees);
            correctedLocalCenter = weapon_collision_geometry_math::rotateLocalPoint(correction, localCenterGame);
        }

        result.rotate = weapon_collision_adjustment_math::applyLocalEulerCorrection(
            generatedPackageRotation, g_rockConfig.rockWeaponCollisionRotationDegrees, g_rockConfig.rockWeaponCollisionRotationCorrectionEnabled);
        result.translate = weapon_collision_geometry_math::localPointToWorld(weaponRootTransform.rotate, weaponRootTransform.translate, weaponRootTransform.scale, correctedLocalCenter);
        return result;
    }

    bool WeaponCollision::weaponCollisionSettingsChanged() const
    {
        if (_cachedConvexRadius < 0.0f || _cachedPointDedupGrid < 0.0f) {
            return false;
        }
        return std::abs(g_rockConfig.rockWeaponCollisionConvexRadius - _cachedConvexRadius) > 0.00001f ||
               std::abs(g_rockConfig.rockWeaponCollisionPointDedupGrid - _cachedPointDedupGrid) > 0.00001f;
    }

    void WeaponCollision::createGeneratedWeaponBodies(RE::hknpWorld* world, const std::vector<GeneratedHullSource>& sources)
    {
        if (hasWeaponBody()) {
            ROCK_LOG_WARN(Weapon, "createGeneratedWeaponBodies called but bodies already exist — skipping");
            return;
        }
        if (!world || !_cachedBhkWorld || sources.empty()) {
            return;
        }

        std::size_t createdCount = 0;
        const std::uint32_t filterInfo = (0x000B << 16) | (ROCK_WEAPON_LAYER & 0x7F);
        for (std::size_t sourceIndex = 0; sourceIndex < sources.size() && createdCount < MAX_WEAPON_BODIES; ++sourceIndex) {
            const auto& source = sources[sourceIndex];
            if (!pointCloudCanBuildHull(source.localPointsGame)) {
                continue;
            }

            auto centeredHavokPoints = makeCenteredHavokPointCloud(source.localPointsGame, source.localCenterGame);
            auto* shape = havok_convex_shape_builder::buildConvexShapeFromLocalHavokPoints(centeredHavokPoints, g_rockConfig.rockWeaponCollisionConvexRadius);
            if (!shape) {
                ROCK_LOG_WARN(Weapon, "Generated weapon mesh hull '{}' failed native convex shape build", source.sourceName);
                continue;
            }

            auto& instance = _weaponBodies[createdCount];
            instance.shape = shape;
            instance.sourceNode = source.sourceRoot;
            instance.generatedLocalCenterGame = source.localCenterGame;
            instance.ownsShapeRef = true;
            instance.hasPrevTransform = false;
            instance.prevPosition = {};

            const bool ok =
                instance.body.create(world, _cachedBhkWorld, shape, filterInfo, { 0 }, BethesdaMotionType::Keyframed, "ROCK_WeaponMeshCollision");

            if (!ok) {
                ROCK_LOG_ERROR(Weapon, "BethesdaPhysicsBody::create failed for generated weapon mesh hull '{}'", source.sourceName);
                shapeRemoveRef(shape);
                clearWeaponBodyInstance(instance, false);
                continue;
            }

            instance.body.createNiNode("ROCK_WeaponMeshCollision");
            const RE::NiTransform sourceRootTransform = source.sourceRoot ? source.sourceRoot->world : makeIdentityTransform();
            const RE::NiTransform initialTransform = makeGeneratedBodyWorldTransform(sourceRootTransform, source.localCenterGame);
            updateBodyTransform(world, instance, initialTransform, 0.016f, createdCount);

            ROCK_LOG_INFO(Weapon,
                "Generated weapon mesh collision body created — meshIndex={} bodyId={} source='{}' root='{}' points={} center=({:.2f},{:.2f},{:.2f}) layer=44",
                createdCount, instance.body.getBodyId().value, source.sourceName, safeNodeName(source.sourceRoot), source.localPointsGame.size(), source.localCenterGame.x,
                source.localCenterGame.y, source.localCenterGame.z);
            ++createdCount;
        }

        publishAtomicBodyIds();
        ROCK_LOG_INFO(Weapon, "Generated weapon mesh collision created {}/{} hull bodies", createdCount, sources.size());
    }

    void WeaponCollision::destroyWeaponBody(RE::hknpWorld* world)
    {
        if (!hasWeaponBody())
            return;

        (void)world;
        clearAtomicBodyIds();

        std::uint32_t destroyedCount = 0;
        for (auto& instance : _weaponBodies) {
            if (instance.body.isValid()) {
                instance.body.destroy(_cachedBhkWorld);
                ++destroyedCount;
            }
            clearWeaponBodyInstance(instance, true);
        }

        ROCK_LOG_INFO(Weapon, "Weapon collision bodies destroyed count={}", destroyedCount);
    }

    void WeaponCollision::resetWeaponBodiesWithoutDestroy()
    {
        clearAtomicBodyIds();
        for (auto& instance : _weaponBodies) {
            instance.body.reset();
            clearWeaponBodyInstance(instance, true);
        }
    }

    void WeaponCollision::clearWeaponBodyInstance(WeaponBodyInstance& instance, bool releaseShapeRef)
    {
        if (releaseShapeRef && instance.ownsShapeRef && instance.shape) {
            shapeRemoveRef(instance.shape);
        }
        instance.body.reset();
        instance.shape = nullptr;
        instance.sourceNode = nullptr;
        instance.generatedLocalCenterGame = {};
        instance.ownsShapeRef = false;
        instance.hasPrevTransform = false;
        instance.prevPosition = {};
    }

    void WeaponCollision::clearAtomicBodyIds()
    {
        _weaponBodyCountAtomic.store(0, std::memory_order_release);
        for (auto& id : _weaponBodyIdsAtomic) {
            id.store(INVALID_BODY_ID, std::memory_order_release);
        }
    }

    void WeaponCollision::publishAtomicBodyIds()
    {
        std::uint32_t count = 0;
        for (auto& id : _weaponBodyIdsAtomic) {
            id.store(INVALID_BODY_ID, std::memory_order_release);
        }
        for (const auto& instance : _weaponBodies) {
            if (instance.body.isValid() && count < MAX_WEAPON_BODIES) {
                _weaponBodyIdsAtomic[count].store(instance.body.getBodyId().value, std::memory_order_release);
                ++count;
            }
        }
        _weaponBodyCountAtomic.store(count, std::memory_order_release);
    }

    void WeaponCollision::updateBodyTransform(RE::hknpWorld* world, WeaponBodyInstance& instance, const RE::NiTransform& weaponTransform, float dt, std::size_t bodyIndex)
    {
        if (!instance.body.isValid() || !world)
            return;

        const float targetX = weaponTransform.translate.x * kGameToHavokScale;
        const float targetY = weaponTransform.translate.y * kGameToHavokScale;
        const float targetZ = weaponTransform.translate.z * kGameToHavokScale;
        const RE::NiMatrix3& adjustedWeaponRotation = weaponTransform.rotate;
        const bool logWitness = bodyIndex == 0 && g_rockConfig.rockDebugVerboseLogging && (++_posLogCounter >= 90);
        if (logWitness) {
            _posLogCounter = 0;
        }
        auto bodyId = instance.body.getBodyId();
        auto* bodyArray = world->GetBodyArray();
        auto* bodyFloats = reinterpret_cast<float*>(&bodyArray[bodyId.value]);
        const float curX = bodyFloats[12];
        const float curY = bodyFloats[13];
        const float curZ = bodyFloats[14];

        if (logWitness) {
            ROCK_LOG_INFO(Weapon, "updateBodyTransform[{}]: niPos=({:.1f},{:.1f},{:.1f}) hkPos=({:.4f},{:.4f},{:.4f}) bodyId={} dt={:.4f}", bodyIndex,
                weaponTransform.translate.x, weaponTransform.translate.y, weaponTransform.translate.z, targetX, targetY, targetZ, instance.body.getBodyId().value, dt);
        }

        if (instance.hasPrevTransform) {
            float dx = weaponTransform.translate.x - instance.prevPosition.x;
            float dy = weaponTransform.translate.y - instance.prevPosition.y;
            float dz = weaponTransform.translate.z - instance.prevPosition.z;
            float distSq = dx * dx + dy * dy + dz * dz;
            if (distSq > 1000.0f * 1000.0f) {
                instance.prevPosition = weaponTransform.translate;
                RE::hkTransformf hkTransform;
                hkTransform.rotation = niRotToHkTransformRotation(adjustedWeaponRotation);
                hkTransform.translation = RE::NiPoint4(targetX, targetY, targetZ, 0.0f);
                instance.body.setTransform(hkTransform);
                if (logWitness) {
                    ROCK_LOG_INFO(Weapon,
                        "transform witness: bodyId={} niPos=({:.1f},{:.1f},{:.1f}) hkTarget=({:.4f},{:.4f},{:.4f}) "
                        "liveOrigin=({:.4f},{:.4f},{:.4f}) teleport=world-reset targetDet={:.4f} liveDet={:.4f}",
                        bodyId.value, weaponTransform.translate.x, weaponTransform.translate.y, weaponTransform.translate.z, targetX, targetY, targetZ, curX, curY, curZ,
                        matrixDeterminant(adjustedWeaponRotation), bodyBasisDeterminant(bodyFloats));
                }
                return;
            }
        }

        alignas(16) float linVelOut[4] = { 0, 0, 0, 0 };
        alignas(16) float angVelOut[4] = { 0, 0, 0, 0 };

        if (instance.hasPrevTransform && dt > 0.0001f) {
            alignas(16) float tgtPos[4] = { targetX, targetY, targetZ, 0.0f };
            alignas(16) float tgtQuat[4];
            niRotToHkQuat(adjustedWeaponRotation, tgtQuat);

            typedef void (*computeHKF_t)(void*, std::uint32_t, const float*, const float*, float, float*, float*);
            static REL::Relocation<computeHKF_t> computeHardKeyFrame{ REL::Offset(offsets::kFunc_ComputeHardKeyFrame) };
            computeHardKeyFrame(world, bodyId.value, tgtPos, tgtQuat, dt, linVelOut, angVelOut);

            constexpr float MAX_VEL = 50.0f;
            float speed = std::sqrt(linVelOut[0] * linVelOut[0] + linVelOut[1] * linVelOut[1] + linVelOut[2] * linVelOut[2]);
            if (speed > MAX_VEL) {
                float scale = MAX_VEL / speed;
                linVelOut[0] *= scale;
                linVelOut[1] *= scale;
                linVelOut[2] *= scale;
            }

            constexpr float MAX_ANG_VEL = 100.0f;
            float angSpeed = std::sqrt(angVelOut[0] * angVelOut[0] + angVelOut[1] * angVelOut[1] + angVelOut[2] * angVelOut[2]);
            if (angSpeed > MAX_ANG_VEL) {
                float scale = MAX_ANG_VEL / angSpeed;
                angVelOut[0] *= scale;
                angVelOut[1] *= scale;
                angVelOut[2] *= scale;
            }
        }

        {
            RE::hkTransformf hkTransform;
            hkTransform.rotation = niRotToHkTransformRotation(adjustedWeaponRotation);
            hkTransform.translation = RE::NiPoint4(targetX, targetY, targetZ, 0.0f);
            instance.body.setTransform(hkTransform);
        }
        instance.body.setVelocity(linVelOut, angVelOut);

        if (logWitness) {
            const auto targetQuat = niRotToHkQuat(adjustedWeaponRotation);
            const auto targetHkRotation = niRotToHkTransformRotation(adjustedWeaponRotation);
            const auto* targetHk = reinterpret_cast<const float*>(&targetHkRotation);
            const float targetDet = matrixDeterminant(adjustedWeaponRotation);
            const float liveDet = bodyBasisDeterminant(bodyFloats);

            ROCK_LOG_INFO(Weapon,
                "transform witness: bodyId={} niPos=({:.1f},{:.1f},{:.1f}) hkTarget=({:.4f},{:.4f},{:.4f}) "
                "liveOrigin=({:.4f},{:.4f},{:.4f}) targetDet={:.4f} liveDet={:.4f} dt={:.4f}",
                bodyId.value, weaponTransform.translate.x, weaponTransform.translate.y, weaponTransform.translate.z, targetX, targetY, targetZ, curX, curY, curZ, targetDet,
                liveDet, dt);

            ROCK_LOG_INFO(Weapon,
                "transform basis: targetNiRows=[({:.4f},{:.4f},{:.4f}) ({:.4f},{:.4f},{:.4f}) ({:.4f},{:.4f},{:.4f})] "
                "targetHkCols=[({:.4f},{:.4f},{:.4f}) ({:.4f},{:.4f},{:.4f}) ({:.4f},{:.4f},{:.4f})] "
                "liveHkCols=[({:.4f},{:.4f},{:.4f}) ({:.4f},{:.4f},{:.4f}) ({:.4f},{:.4f},{:.4f})] "
                "quat=({:.4f},{:.4f},{:.4f},{:.4f}) linVel=({:.4f},{:.4f},{:.4f}) angVel=({:.4f},{:.4f},{:.4f})",
                adjustedWeaponRotation.entry[0][0], adjustedWeaponRotation.entry[0][1], adjustedWeaponRotation.entry[0][2], adjustedWeaponRotation.entry[1][0],
                adjustedWeaponRotation.entry[1][1], adjustedWeaponRotation.entry[1][2], adjustedWeaponRotation.entry[2][0], adjustedWeaponRotation.entry[2][1],
                adjustedWeaponRotation.entry[2][2], targetHk[0], targetHk[1], targetHk[2], targetHk[4], targetHk[5], targetHk[6], targetHk[8], targetHk[9],
                targetHk[10], bodyFloats[0], bodyFloats[1], bodyFloats[2], bodyFloats[4], bodyFloats[5], bodyFloats[6], bodyFloats[8], bodyFloats[9], bodyFloats[10],
                targetQuat.x,
                targetQuat.y, targetQuat.z, targetQuat.w, linVelOut[0], linVelOut[1], linVelOut[2], angVelOut[0], angVelOut[1], angVelOut[2]);
        }

        instance.prevPosition = weaponTransform.translate;
        instance.hasPrevTransform = true;
    }

    void WeaponCollision::disableDominantHandCollision(RE::hknpWorld* world, RE::hknpBodyId handBodyId)
    {
        if (!world || handBodyId.value == INVALID_BODY_ID)
            return;

        std::uint32_t currentFilter = 0;
        if (!body_collision::tryReadFilterInfo(world, handBodyId, currentFilter)) {
            return;
        }

        auto newFilter = hand_collision_suppression_math::beginSuppression(_dominantHandSuppression, handBodyId.value, currentFilter);
        if (newFilter != currentFilter) {
            body_collision::setFilterInfo(world, handBodyId, newFilter);
        }

        const bool broadPhaseSet = body_collision::setBroadPhaseEnabled(world, handBodyId, false);
        _dominantHandBroadPhaseSuppressed = _dominantHandBroadPhaseSuppressed || broadPhaseSet;

        _dominantHandDisabled = true;
        _disabledHandBodyId = handBodyId;

        ROCK_LOG_INFO(Weapon, "Dominant hand collision DISABLED bodyId={} filter=0x{:08X}->0x{:08X} wasDisabledBefore={} broadphase={}", handBodyId.value, currentFilter,
            newFilter, _dominantHandSuppression.wasNoCollideBeforeSuppression ? "yes" : "no", broadPhaseSet ? "disabled" : "unchanged");
    }

    void WeaponCollision::enableDominantHandCollision(RE::hknpWorld* world)
    {
        if (!world || _disabledHandBodyId.value == INVALID_BODY_ID)
            return;

        std::uint32_t currentFilter = 0;
        if (!body_collision::tryReadFilterInfo(world, _disabledHandBodyId, currentFilter)) {
            hand_collision_suppression_math::clear(_dominantHandSuppression);
            _dominantHandBroadPhaseSuppressed = false;
            _dominantHandDisabled = false;
            _disabledHandBodyId.value = INVALID_BODY_ID;
            return;
        }

        auto newFilter = hand_collision_suppression_math::restoreFilter(_dominantHandSuppression, currentFilter);
        if (newFilter != currentFilter) {
            body_collision::setFilterInfo(world, _disabledHandBodyId, newFilter);
        }

        const bool broadPhaseRestored = _dominantHandBroadPhaseSuppressed && body_collision::setBroadPhaseEnabled(world, _disabledHandBodyId, true);

        ROCK_LOG_INFO(Weapon, "Dominant hand collision RE-ENABLED bodyId={} filter=0x{:08X}->0x{:08X} restoreDisabled={} broadphase={}", _disabledHandBodyId.value, currentFilter,
            newFilter, _dominantHandSuppression.wasNoCollideBeforeSuppression ? "yes" : "no", broadPhaseRestored ? "enabled" : "unchanged");

        _dominantHandDisabled = false;
        hand_collision_suppression_math::clear(_dominantHandSuppression);
        _dominantHandBroadPhaseSuppressed = false;
        _disabledHandBodyId.value = INVALID_BODY_ID;
    }

}
