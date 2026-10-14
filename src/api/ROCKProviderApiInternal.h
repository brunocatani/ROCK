#pragma once

#include "api/ProviderRuntimeTypes.h"
#include "api/TouchGrabRegistry.h"
#include "physics-interaction/timing/GameFrameTimingPolicy.h"
#include "physics-interaction/weapon/WeaponPartRuntime.h"

namespace rock
{
    class PhysicsInteraction;
}

namespace rock::provider
{
    bool isPowerArmorGrabOwnerRegisteredV1(std::uint64_t ownerToken);
    struct RockProviderWeaponPartTargetQueryV1
    {
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uintptr_t sourceRoot{ 0 };
        char sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};
    };

    struct RockProviderWeaponPartTargetResolutionV1
    {
        std::uint32_t whitelistActive{ 0 };
        std::uint32_t matched{ 0 };
        api::weaponparts::GrabMode grabMode{
            api::weaponparts::GrabMode::None
        };
        std::uint32_t groupId{ 0 };
        std::uint32_t priority{ 0 };
        std::uint32_t reserved{ 0 };
        std::uint64_t ownerToken{ 0 };
    };

    // FRIK 2.3 FrameBegin precedes provider initialization and BeforeRock.
    // Advance event timestamps here without retiring publication-bound leases.
    void beginGameFrame(std::uint64_t frameIndex) noexcept;
    void setPhysicsInteractionInstance(rock::PhysicsInteraction* pi);
    void dispatchFrameCallbacks(rock::PhysicsInteraction& pi);
    // Invalidates live reads and releases every lease after provider loss.
    void clearProviderStateForProviderLoss();
    // The reservation on hand, or Normal when that hand is not reserved.
    api::grab::HandReservationKind currentHandReservation(api::Hand hand);
    void setEquippedWeaponFiringHandIsLeft(bool isLeft);
    bool getHandlingAuthorityRequest(
        api::weapon::HandlingRequest& outRequest);
    bool ownsHandlingAuthority(
        std::uint64_t ownerToken,
        std::uint32_t requiredFlags);
    std::uint32_t currentHandInputSuppressionFlagsV1(api::Hand hand);
    std::uint32_t currentNativeAnimationAuthorityFlagsV1();
    void refreshNativeAnimationAuthorityLeasesV1();
    /*
     * Dispatches one animation phase with the frame's shared timing identity.
     * The published context deltaSeconds is truthful: an unmeasurable frame
     * publishes zero, never a fabricated nominal-rate value. Every workspace
     * consumer guards for non-positive deltas before integrating.
     */
    void dispatchAnimationPhaseCallbacksV1(
        ::rock::api::animation::AnimationPhase phase,
        const game_frame_timing_policy::GameFrameTiming& timing);
    bool resolveWeaponPartTargetV1(
        const RockProviderWeaponPartTargetQueryV1& query,
        RockProviderWeaponPartTargetResolutionV1& outResolution);
    bool hasWeaponPartTargetsForGeneration(std::uint64_t weaponGenerationKey);
    // Value snapshot for main-thread grip markers; never retains provider pointers.
    std::size_t copyWeaponPartTargets(std::span<weapon_part_runtime::Target> outTargets);
    std::uint32_t copyWeaponPartDriveTargetsV1(
        RockProviderWeaponPartDriveTargetV1* outTargets,
        std::uint32_t maxTargets,
        std::uint64_t* outOwnerTokens = nullptr,bool pruneExpired = true);
    bool resolveTouchGrabTargetV1(
        std::uint32_t bodyId,
        std::uint32_t collisionLayer,
        TouchGrabMotionClassV1 motionClass,
        api::Hand hand,
        std::uint32_t worldGeneration,
        std::uint32_t skeletonGeneration,
        std::uint32_t providerGeneration,
        TouchGrabTargetMatchV1& outMatch);
    bool currentTouchGrabTargetV1(
        std::uint64_t ownerToken,
        std::uint64_t scopeToken,
        std::uint64_t targetId,
        std::uint32_t targetGeneration,
        std::uint32_t worldGeneration,
        std::uint32_t skeletonGeneration,
        std::uint32_t providerGeneration,
        TouchGrabTargetMatchV1& outMatch);
    bool publishTouchGrabStateV1(
        std::uint64_t ownerToken,
        std::uint64_t scopeToken,
        const ::rock::api::touch::TouchGrabState& state);
    void acknowledgeTouchGrabYieldV1(
        std::uint64_t ownerToken,
        std::uint64_t scopeToken,
        std::uint64_t targetId,
        std::uint32_t targetGeneration);
    void markInteractionCommandStageV1(
        std::uint64_t ownerToken,
        std::uint64_t commandId,
        api::grab::CommandStage stage);
}
