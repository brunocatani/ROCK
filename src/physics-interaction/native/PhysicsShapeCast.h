#pragma once

#include "physics-interaction/collision/PhysicsShapeCastMath.h"
#include "physics-interaction/hand/HandSelection.h"

#include "RE/Bethesda/BSHavok.h"
#include "RE/Havok/hknpAllHitsCollector.h"
#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiPoint.h"

namespace frik::rock::physics_shape_cast
{
    constexpr std::uint32_t kSelectionQueryCollisionFilterInfo = selection_query_policy::kDefaultShapeCastFilterInfo;

    struct SphereCastInput
    {
        RE::NiPoint3 startGame{};
        RE::NiPoint3 directionGame{};
        float distanceGame = 0.0f;
        float radiusGame = 0.0f;
        std::uint32_t collisionFilterInfo = kSelectionQueryCollisionFilterInfo;
    };

    struct SphereCastDiagnostics
    {
        bool attempted = false;
        bool shapeReady = false;
        bool castRan = false;
        int hitCount = 0;
        float distanceGame = 0.0f;
        float radiusGame = 0.0f;
        std::uint32_t collisionFilterInfo = 0;
    };

    bool castSelectionSphere(RE::hknpWorld* world, const SphereCastInput& input, RE::hknpAllHitsCollector& collector, SphereCastDiagnostics* diagnostics = nullptr);
}
