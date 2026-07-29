#pragma once

#include "api/ROCKProviderApi.h"
#include "api/TouchGrabRegistry.h"

namespace rock
{
    class PhysicsInteraction;
}

namespace rock::provider
{
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
        RockProviderWeaponPartGrabModeV1 grabMode{
            RockProviderWeaponPartGrabModeV1::None
        };
        std::uint32_t groupId{ 0 };
        std::uint32_t priority{ 0 };
        std::uint32_t reserved{ 0 };
        std::uint64_t ownerToken{ 0 };
    };

    void setPhysicsInteractionInstance(rock::PhysicsInteraction* pi);
    void dispatchFrameCallbacks(rock::PhysicsInteraction& pi);
    void clearExternalBodiesForProviderLoss();
    bool isExternalBodyId(std::uint32_t bodyId);
    bool isExternalBodyDynamicPushSuppressed(std::uint32_t bodyId);
    bool recordExternalHandContact(
        bool isLeft,
        std::uint32_t handBodyId,
        std::uint32_t externalBodyId,
        std::uint64_t frameIndex);
    bool recordExternalContact(
        const RockProviderExternalContactV1& contact,
        std::uint32_t worldGeneration,
        std::uint32_t skeletonGeneration,
        std::uint32_t providerGeneration);
    RockProviderOffhandReservation currentOffhandReservation();
    void setEquippedWeaponFiringHandIsLeft(bool isLeft);
    bool getEquippedWeaponHandlingAuthorityV1(
        RockProviderEquippedWeaponHandlingRequestV1& outRequest);
    bool ownsEquippedWeaponHandlingAuthorityV1(
        std::uint64_t ownerToken,
        std::uint32_t requiredFlags);
    std::uint32_t currentHandInputSuppressionFlagsV1(RockProviderHand hand);
    std::uint32_t currentNativeAnimationAuthorityFlagsV1();
    void refreshNativeAnimationAuthorityLeasesV1();
    void dispatchAnimationPhaseCallbacksV1(
        RockProviderAnimationPhaseV1 phase,
        float deltaSeconds);
    bool resolveWeaponPartTargetV1(
        const RockProviderWeaponPartTargetQueryV1& query,
        RockProviderWeaponPartTargetResolutionV1& outResolution);
    std::uint32_t copyWeaponPartDriveTargetsV1(
        RockProviderWeaponPartDriveTargetV1* outTargets,
        std::uint32_t maxTargets,
        std::uint64_t* outOwnerTokens = nullptr);
    std::uint32_t currentExternalBodyCount();
    bool resolveTouchGrabTargetV1(
        std::uint32_t bodyId,
        std::uint32_t collisionLayer,
        TouchGrabMotionClassV1 motionClass,
        RockProviderHand hand,
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
        const RockProviderTouchGrabStateV1& state);
    void acknowledgeTouchGrabYieldV1(
        std::uint64_t ownerToken,
        std::uint64_t scopeToken,
        std::uint64_t targetId,
        std::uint32_t targetGeneration);
    void markInteractionCommandStageV1(
        std::uint64_t ownerToken,
        std::uint64_t commandId,
        RockProviderCommandStageV1 stage);
}
