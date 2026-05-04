#pragma once

#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/native/HavokPhysicsTiming.h"

#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

#include <cstdint>

namespace frik::rock
{
    /*
     * Generated hand and weapon colliders are solver inputs, not visual nodes.
     * The stable FO4VR path is to sample their desired transform once from the
     * game/provider side, then let the Havok step consume that target through
     * Bethesda's keyframe-drive wrapper. Transform writes are kept explicit and
     * rare so normal motion does not fight deferred hknp interpolation.
     */
    struct GeneratedKeyframedBodyDriveState
    {
        RE::NiTransform pendingTarget{};
        RE::NiPoint3 previousTargetPosition{};
        std::uint64_t queuedSequence = 0;
        std::uint64_t consumedSequence = 0;
        bool hasPendingTarget = false;
        bool hasPreviousTarget = false;
        bool pendingTeleport = false;
    };

    struct GeneratedKeyframedBodyDriveResult
    {
        bool attempted = false;
        bool driven = false;
        bool teleported = false;
        bool skippedStale = false;
        bool missingBody = false;
        bool placementFailed = false;
        bool nativeDriveFailed = false;
        bool hasLiveBodyTransform = false;
        RE::NiPoint3 targetGamePosition{};
        RE::NiPoint3 targetHavokPosition{};
        RE::NiPoint3 liveBodyGamePosition{};
        float bodyDeltaGameUnits = 0.0f;
        std::uint32_t motionIndex = body_frame::kFreeMotionIndex;
        body_frame::BodyFrameSource liveBodyFrameSource = body_frame::BodyFrameSource::Fallback;
        float driveDeltaSeconds = havok_physics_timing::kFallbackPhysicsDeltaSeconds;

        [[nodiscard]] bool shouldRequestRebuild() const
        {
            return attempted && !driven && (missingBody || placementFailed || nativeDriveFailed);
        }
    };

    void clearGeneratedKeyframedBodyDriveState(GeneratedKeyframedBodyDriveState& state);
    void queueGeneratedKeyframedBodyTarget(
        GeneratedKeyframedBodyDriveState& state,
        const RE::NiTransform& target,
        float teleportDistanceGameUnits);
    bool placeGeneratedKeyframedBodyImmediately(BethesdaPhysicsBody& body, const RE::NiTransform& target);
    GeneratedKeyframedBodyDriveResult driveGeneratedKeyframedBody(
        RE::hknpWorld* world,
        BethesdaPhysicsBody& body,
        GeneratedKeyframedBodyDriveState& state,
        const havok_physics_timing::PhysicsTimingSample& timing,
        const char* ownerName,
        std::uint32_t bodyIndex);
}
