#include "physics-interaction/native/PhysicsRayCast.h"

#include "RE/Bethesda/BSHavok.h"
#include "RE/Bethesda/bhkPickData.h"

#include <algorithm>
#include <cmath>

namespace rock::physics_ray_cast
{
    bool castClosestSegment(
        RE::bhkWorld* world,
        const RE::NiPoint3& startGame,
        const RE::NiPoint3& endGame,
        const std::uint32_t collisionFilterInfo,
        ClosestSegmentResult& outResult)
    {
        outResult = {};
        if (!world) {
            return false;
        }

        RE::bhkPickData pickData;
        pickData.SetStartEnd(startGame, endGame);
        pickData.collisionFilter.filter = collisionFilterInfo;

        if (!world->PickObject(pickData) || !pickData.HasHit()) {
            return true;
        }

        outResult.hit = true;
        outResult.hitFraction = std::clamp(
            pickData.GetHitFraction(),
            0.0f,
            1.0f);

        const auto& normal = pickData.result.normal;
        const float normalLengthSquared =
            normal.x * normal.x +
            normal.y * normal.y +
            normal.z * normal.z;
        if (std::isfinite(normal.x) &&
            std::isfinite(normal.y) &&
            std::isfinite(normal.z) &&
            std::isfinite(normalLengthSquared) &&
            normalLengthSquared > 1.0e-8f) {
            const float inverseLength =
                1.0f / std::sqrt(normalLengthSquared);
            outResult.normalGame = {
                normal.x * inverseLength,
                normal.y * inverseLength,
                normal.z * inverseLength,
            };
            outResult.normalValid = true;
        }
        return true;
    }
}
