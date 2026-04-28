#include "PhysicsShapeCast.h"

#include "HavokOffsets.h"
#include "PhysicsLog.h"
#include "PhysicsUtils.h"

#include "RE/Bethesda/bhkCharacterController.h"
#include "RE/Havok/hknpShape.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <unordered_map>
#include <xmmintrin.h>

namespace frik::rock::physics_shape_cast
{
    namespace
    {
        /*
         * ROCK selection needs HIGGS-style swept-sphere truth, but FO4VR exposes that
         * through hknp's direct CastShape ABI rather than Skyrim's phantom linear cast.
         * The wrapper below centralizes the verified 0x80 query layout and uses
         * process-lifetime query spheres keyed by radius. We do not mutate an existing
         * sphere's radius after construction because the constructor initializes support
         * data along with +0x14; reusing the right constructed shape keeps the query ABI
         * stable while avoiding per-frame allocations.
         */
        RE::hknpShape* getSelectionSphereShape(float radiusHavok)
        {
            using CreateSphereShape_t = RE::hknpShape* (*)(RE::hkVector4f*, float);
            static REL::Relocation<CreateSphereShape_t> createSphere{ REL::Offset(offsets::kFunc_CreateSphereShape) };

            static std::mutex cacheMutex;
            static std::unordered_map<std::uint32_t, RE::hknpShape*> cachedShapes;

            const auto radiusKey = std::bit_cast<std::uint32_t>(radiusHavok);
            std::scoped_lock lock(cacheMutex);
            if (auto it = cachedShapes.find(radiusKey); it != cachedShapes.end()) {
                return it->second;
            }

            RE::hkVector4f center{ 0.0f, 0.0f, 0.0f, 0.0f };
            auto* shape = createSphere(&center, radiusHavok);
            if (!shape) {
                ROCK_LOG_ERROR(Hand, "Selection shape cast: failed to create hknp sphere shape radiusHk={:.4f}", radiusHavok);
                return nullptr;
            }

            cachedShapes.emplace(radiusKey, shape);
            return shape;
        }

        void prepareAllHitsCollector(RE::hknpAllHitsCollector& collector)
        {
            /*
             * CommonLib's constructor correctly points hkInplaceArray storage at this
             * collector's inline buffer. Default assignment from a temporary does not:
             * it copies the temporary's _data pointer and leaves CastShape writing into
             * dead stack memory. Reset the fields in place instead.
             */
            collector.hints = 0;
            collector.earlyOutThreshold.real = _mm_set1_ps((std::numeric_limits<float>::max)());
            collector.hits._data = reinterpret_cast<RE::hknpCollisionResult*>(reinterpret_cast<std::uintptr_t>(&collector) + 0x30);
            collector.hits._size = 0;
            collector.hits._capacityAndFlags = 0x8000000A;
        }

        bool normalize(RE::NiPoint3 value, RE::NiPoint3& out)
        {
            const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
            if (lengthSquared <= 1.0e-6f) {
                out = RE::NiPoint3{};
                return false;
            }

            const float inverseLength = 1.0f / std::sqrt(lengthSquared);
            out = RE::NiPoint3(value.x * inverseLength, value.y * inverseLength, value.z * inverseLength);
            return true;
        }

        RE::hkTransformf identityTransform()
        {
            RE::hkTransformf transform{};
            transform.SetIdentity();
            return transform;
        }
    }

    bool castSelectionSphere(RE::hknpWorld* world, const SphereCastInput& input, RE::hknpAllHitsCollector& collector, SphereCastDiagnostics* diagnostics)
    {
        if (diagnostics) {
            *diagnostics = SphereCastDiagnostics{};
            diagnostics->attempted = true;
            diagnostics->distanceGame = input.distanceGame;
            diagnostics->radiusGame = input.radiusGame;
            diagnostics->collisionFilterInfo = input.collisionFilterInfo;
        }

        if (!world || input.distanceGame <= 0.001f || input.radiusGame <= 0.001f) {
            return false;
        }

        RE::NiPoint3 direction{};
        if (!normalize(input.directionGame, direction)) {
            return false;
        }

        const float radiusHavok = input.radiusGame * kGameToHavokScale;
        auto* shape = getSelectionSphereShape(radiusHavok);
        if (diagnostics) {
            diagnostics->shapeReady = shape != nullptr;
        }
        if (!shape) {
            return false;
        }

        auto* filterRef = getQueryFilterRef(world);
        if (!filterRef) {
            ROCK_LOG_WARN(Hand, "Selection shape cast skipped: missing hknp query filter");
            return false;
        }

        const auto startHavok = niPointToHkVector(input.startGame);
        const physics_shape_cast_math::ShapeCastVec4 start{
            startHavok.x,
            startHavok.y,
            startHavok.z,
            startHavok.w,
        };
        physics_shape_cast_math::ShapeCastVec4 displacement{
            direction.x * input.distanceGame * kGameToHavokScale,
            direction.y * input.distanceGame * kGameToHavokScale,
            direction.z * input.distanceGame * kGameToHavokScale,
            1.0f,
        };

        auto query = physics_shape_cast_math::buildRuntimeShapeCastQuery(filterRef, shape, input.collisionFilterInfo, start, displacement);
        auto transform = identityTransform();
        prepareAllHitsCollector(collector);
        world->CastShape(&query, &transform, &collector, &collector);

        if (diagnostics) {
            diagnostics->castRan = true;
            diagnostics->hitCount = collector.hits._size;
        }

        return true;
    }
}
