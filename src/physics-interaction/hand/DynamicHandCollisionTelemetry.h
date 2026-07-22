#pragma once

/*
 * Internal dynamic-hand collision telemetry contract. This is deliberately
 * separate from ROCKProviderApi.h: ROCK remains on provider API V1 and a later
 * API addition can copy the stable, engine-agnostic subset without exposing
 * runtime objects, pointers, or thread-owned state.
 *
 * The runtime publishes one fixed-capacity snapshot per main frame. All vector
 * names state their coordinate space and all distances/speeds state their
 * units. Proxy body IDs are transient diagnostics, never ownership handles.
 */

#include "physics-interaction/hand/DynamicHandTwinTargets.h"
#include "physics-interaction/hand/HandColliderTypes.h"

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace rock::dynamic_hand_collision_telemetry
{
    inline constexpr std::size_t kPalmSlot = 0;
    inline constexpr std::size_t kFirstForearmSlot = 1 + hand_collider_semantics::kHandFingerCount;
    inline constexpr std::size_t kForearmSlot = kFirstForearmSlot;
    inline constexpr std::size_t kBodiesPerHand = kFirstForearmSlot + dynamic_hand_twin::kForearmSegmentCountPerHand;
    inline constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;

    enum class TwinRole : std::uint8_t
    {
        Palm = 0,
        ThumbTip,
        IndexTip,
        MiddleTip,
        RingTip,
        PinkyTip,
        Forearm,
    };

    [[nodiscard]] constexpr TwinRole roleForBodyIndex(std::size_t bodyIndex) noexcept
    {
        return bodyIndex < kBodiesPerHand ? static_cast<TwinRole>(bodyIndex) : TwinRole::Palm;
    }

    [[nodiscard]] constexpr const char* roleCode(TwinRole role) noexcept
    {
        switch (role) {
        case TwinRole::Palm:
            return "PALM";
        case TwinRole::ThumbTip:
            return "THMB";
        case TwinRole::IndexTip:
            return "INDX";
        case TwinRole::MiddleTip:
            return "MIDL";
        case TwinRole::RingTip:
            return "RING";
        case TwinRole::PinkyTip:
            return "PNKY";
        case TwinRole::Forearm:
            return "FARM";
        }
        return "UNKN";
    }

    static_assert(static_cast<std::size_t>(TwinRole::Forearm) + 1 == kBodiesPerHand);

    struct TwinSample
    {
        TwinRole role{ TwinRole::Palm };
        std::uint32_t bodyId{ kInvalidBodyId };
        std::uint64_t physicsSampleSequence = 0;

        RE::NiTransform publishedTargetWorld{};
        RE::NiPoint3 requestedTargetWorldGame{};
        RE::NiPoint3 commandedTargetWorldGame{};
        RE::NiPoint3 liveBodyWorldGame{};
        RE::NiPoint3 solverResidualWorldGame{};
        RE::NiPoint3 requestedGapWorldGame{};
        RE::NiPoint3 contactDeviationWorldGame{};
        RE::NiPoint3 handTargetCorrectionWorldGame{};
        RE::NiPoint3 targetVelocityWorldGameUnitsPerSecond{};

        float lengthGameUnits = 0.0f;
        float radiusGameUnits = 0.0f;
        float convexRadiusGameUnits = 0.0f;
        float solverResidualGameUnits = 0.0f;
        float requestedGapGameUnits = 0.0f;
        float contactDeviationGameUnits = 0.0f;
        float handTargetCorrectionGameUnits = 0.0f;
        float handTargetResponseScale = 1.0f;
        float approachSpeedGameUnitsPerSecond = 0.0f;
        float physicsDeltaSeconds = 0.0f;

        bool bodyCreated = false;
        bool publishedTargetValid = false;
        bool physicsSampleValid = false;
        bool targetVelocityValid = false;
        bool contactActive = false;
        bool recoveryTeleport = false;
    };

    struct HandSample
    {
        std::array<TwinSample, kBodiesPerHand> twins{};
        RE::NiPoint3 combinedContactDeviationWorldGame{};
        RE::NiPoint3 appliedVisualDeviationWorldGame{};

        std::uint64_t contactEntrySequence = 0;
        std::uint32_t contactMask = 0;
        std::uint32_t contactCount = 0;
        std::uint32_t entryContactMask = 0;
        float combinedContactDeviationGameUnits = 0.0f;
        float appliedVisualDeviationGameUnits = 0.0f;
        float contactEntryApproachSpeedGameUnitsPerSecond = 0.0f;
        float teleportRecoverySecondsRemaining = 0.0f;

        bool isLeft = false;
        bool handDisabled = false;
        bool ownedByStrongerSystem = false;
        bool visualAuthorityAvailable = false;
        bool visualActive = false;
        bool anyContact = false;
    };

    struct Snapshot
    {
        std::array<HandSample, 2> hands{};
        std::uint64_t updateSequence = 0;
        bool runtimeEnabled = false;
        bool worldReady = false;
        bool menuBlocked = false;
        bool physicsWritesAllowed = false;
        bool transitionCollisionSuppressed = false;
    };

    struct HapticPulse
    {
        bool fire = false;
        bool isLeft = false;
        float intensity = 0.0f;
        float approachSpeedGameUnitsPerSecond = 0.0f;
        std::uint64_t contactEntrySequence = 0;
        std::uint32_t contactMask = 0;
    };

    struct HapticEvents
    {
        std::array<HapticPulse, 2> hands{};
    };
}
