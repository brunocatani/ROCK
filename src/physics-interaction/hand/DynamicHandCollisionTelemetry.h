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
    inline constexpr std::size_t kFirstFingerSlot = kPalmSlot + 1;
    inline constexpr std::size_t kFingerSlotCount =
        hand_collider_semantics::kHandFingerCount *
        hand_collider_semantics::kHandFingerSegmentCount;
    inline constexpr std::size_t kFirstForearmSlot =
        kFirstFingerSlot + kFingerSlotCount;
    inline constexpr std::size_t kForearmSlot = kFirstForearmSlot;
    inline constexpr std::size_t kBodiesPerHand = kFirstForearmSlot + dynamic_hand_twin::kForearmSegmentCountPerHand;
    inline constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;

    enum class TwinRole : std::uint8_t
    {
        Palm = 0,
        ThumbBase,
        ThumbMiddle,
        ThumbTip,
        IndexBase,
        IndexMiddle,
        IndexTip,
        MiddleBase,
        MiddleMiddle,
        MiddleTip,
        RingBase,
        RingMiddle,
        RingTip,
        PinkyBase,
        PinkyMiddle,
        PinkyTip,
        Forearm,
    };

    [[nodiscard]] constexpr bool isFingerSlot(
        const std::size_t bodyIndex) noexcept
    {
        return bodyIndex >= kFirstFingerSlot && bodyIndex < kFirstForearmSlot;
    }

    [[nodiscard]] constexpr std::size_t fingerIndexForBodyIndex(
        const std::size_t bodyIndex) noexcept
    {
        return isFingerSlot(bodyIndex) ?
            (bodyIndex - kFirstFingerSlot) /
                hand_collider_semantics::kHandFingerSegmentCount :
            hand_collider_semantics::kHandFingerCount;
    }

    [[nodiscard]] constexpr std::size_t fingerSegmentIndexForBodyIndex(
        const std::size_t bodyIndex) noexcept
    {
        return isFingerSlot(bodyIndex) ?
            (bodyIndex - kFirstFingerSlot) %
                hand_collider_semantics::kHandFingerSegmentCount :
            hand_collider_semantics::kHandFingerSegmentCount;
    }

    [[nodiscard]] constexpr std::size_t bodyIndexForFingerSegment(
        const std::size_t fingerIndex,
        const std::size_t segmentIndex) noexcept
    {
        return kFirstFingerSlot +
               fingerIndex * hand_collider_semantics::kHandFingerSegmentCount +
               segmentIndex;
    }

    [[nodiscard]] constexpr bool isFingerTipSlot(
        const std::size_t bodyIndex) noexcept
    {
        return isFingerSlot(bodyIndex) &&
               fingerSegmentIndexForBodyIndex(bodyIndex) ==
                   static_cast<std::size_t>(
                       hand_collider_semantics::HandFingerSegment::Tip);
    }

    [[nodiscard]] constexpr bool isSurfaceGrabSourceSlot(
        const std::size_t bodyIndex) noexcept
    {
        return bodyIndex == kPalmSlot || isFingerTipSlot(bodyIndex);
    }

    [[nodiscard]] constexpr TwinRole roleForBodyIndex(std::size_t bodyIndex) noexcept
    {
        return bodyIndex < kBodiesPerHand ? static_cast<TwinRole>(bodyIndex) : TwinRole::Palm;
    }

    [[nodiscard]] constexpr const char* roleCode(TwinRole role) noexcept
    {
        switch (role) {
        case TwinRole::Palm:
            return "PALM";
        case TwinRole::ThumbBase:
            return "THB0";
        case TwinRole::ThumbMiddle:
            return "THB1";
        case TwinRole::ThumbTip:
            return "THB2";
        case TwinRole::IndexBase:
            return "IDX0";
        case TwinRole::IndexMiddle:
            return "IDX1";
        case TwinRole::IndexTip:
            return "IDX2";
        case TwinRole::MiddleBase:
            return "MID0";
        case TwinRole::MiddleMiddle:
            return "MID1";
        case TwinRole::MiddleTip:
            return "MID2";
        case TwinRole::RingBase:
            return "RNG0";
        case TwinRole::RingMiddle:
            return "RNG1";
        case TwinRole::RingTip:
            return "RNG2";
        case TwinRole::PinkyBase:
            return "PNK0";
        case TwinRole::PinkyMiddle:
            return "PNK1";
        case TwinRole::PinkyTip:
            return "PNK2";
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
        std::uint64_t sourceGameFrameIndex = 0;
        std::uint64_t sourceQueueSequence = 0;
        std::uint64_t solveSequence = 0;

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
        float physicsRawDeltaSeconds = 0.0f;
        float physicsRemainderDeltaSeconds = 0.0f;
        float physicsAccumulatedDeltaSeconds = 0.0f;
        float physicsSubstepProgress = 0.0f;
        std::uint32_t physicsSubstepCount = 0;
        std::uint32_t physicsSubstepIndex = 0;

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
        std::array<float, hand_collider_semantics::kHandFingerCount>
            surfaceFingerOpenValues{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
        std::array<std::int8_t, hand_collider_semantics::kHandFingerCount>
            surfaceFingerDirections{};
        RE::NiPoint3 combinedContactDeviationWorldGame{};
        RE::NiPoint3 appliedVisualDeviationWorldGame{};

        std::uint64_t contactEntrySequence = 0;
        std::uint32_t contactMask = 0;
        std::uint32_t contactCount = 0;
        std::uint32_t entryContactMask = 0;
        std::uint32_t surfaceFingerHelpfulSlotMask = 0;
        std::uint32_t otherHandContactMask = 0;
        std::uint32_t weaponContactMask = 0;
        std::uint32_t suppressedWeaponPairCount = 0;
        std::uint32_t dynamicInteractionLayer = 0;
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
        bool surfaceFingerResponseActive = false;
        bool dynamicInteractionsEnabled = false;
        bool pairFilterReady = false;
        bool weaponPairSuppressed = false;
    };

    struct Snapshot
    {
        std::array<HandSample, 2> hands{};
        std::uint64_t updateSequence = 0;
        std::uint64_t surfaceImpulsePairSequence = 0;
        std::uint64_t surfaceProcessedPairSequence = 0;
        std::uint64_t surfaceEligiblePairSequence = 0;
        std::uint64_t surfaceContactPublishSequence = 0;
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
