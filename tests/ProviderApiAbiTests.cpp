#include "api/ROCKProviderApi.h"

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <type_traits>

using namespace rock::provider;

namespace
{
    template <class T>
    bool expectLayout(
        const char* name,
        const std::size_t expectedSize,
        const std::size_t expectedAlignment)
    {
        const bool ok = sizeof(T) == expectedSize &&
            alignof(T) == expectedAlignment &&
            std::is_standard_layout_v<T> &&
            std::is_trivially_copyable_v<T>;
        if (!ok) {
            std::cerr << name << ": expected size/alignment " <<
                expectedSize << '/' << expectedAlignment << ", found " <<
                sizeof(T) << '/' << alignof(T) << '\n';
        }
        return ok;
    }
}

int main()
{
    static_assert(ROCK_PROVIDER_MAX_WEAPON_BODIES == 8);
    static_assert(ROCK_PROVIDER_MAX_WEAPON_EVIDENCE_DETAILS_V1 == 100);
    static_assert(
        ROCK_PROVIDER_MAX_WEAPON_EVIDENCE_DETAILS_V1 >
        ROCK_PROVIDER_MAX_WEAPON_BODIES);

    bool ok = true;
#define ROCK_EXPECT_LAYOUT(type, size, alignment) \
    ok = expectLayout<type>(#type, size, alignment) && ok
    ROCK_EXPECT_LAYOUT(RockProviderApiDescriptorV1, 40, 8);
    ROCK_EXPECT_LAYOUT(RockProviderConsumerRegistrationV1, 104, 4);
    ROCK_EXPECT_LAYOUT(RockProviderConsumerHandleV1, 48, 8);
    ROCK_EXPECT_LAYOUT(RockProviderLimitsV1, 92, 4);
    ROCK_EXPECT_LAYOUT(RockProviderLimitsExtV1, 192, 4);
    ROCK_EXPECT_LAYOUT(RockProviderFrameSnapshot, 376, 8);
    ROCK_EXPECT_LAYOUT(RockProviderHandFrameV1, 144, 8);
    ROCK_EXPECT_LAYOUT(RockProviderHandInteractionStateV1, 160, 8);
    ROCK_EXPECT_LAYOUT(RockProviderEventV1, 96, 8);
    ROCK_EXPECT_LAYOUT(RockProviderEventStreamStateV1, 56, 8);
    ROCK_EXPECT_LAYOUT(RockProviderEquippedWeaponStateV1, 96, 8);
    ROCK_EXPECT_LAYOUT(RockProviderExternalBodyRegistration, 32, 8);
    ROCK_EXPECT_LAYOUT(RockProviderExternalContactV1, 128, 8);
    ROCK_EXPECT_LAYOUT(RockProviderExternalContactRecordV1, 152, 8);
    ROCK_EXPECT_LAYOUT(RockProviderExternalContactStreamStateV1, 56, 8);
    ROCK_EXPECT_LAYOUT(RockProviderWeaponPartResolutionQueryV1, 128, 8);
    ROCK_EXPECT_LAYOUT(RockProviderWeaponPartResolutionResultV1, 72, 8);
    ROCK_EXPECT_LAYOUT(RockProviderWeaponPartPoseV1, 232, 8);
    ROCK_EXPECT_LAYOUT(RockProviderWeaponPartDriveApplicationResultV1, 184, 8);
    ROCK_EXPECT_LAYOUT(RockProviderScopeSightStateV1, 144, 8);
    ROCK_EXPECT_LAYOUT(RockProviderWeaponCompositionStateV1, 80, 8);
    ROCK_EXPECT_LAYOUT(RockProviderWeaponCompositionEntryV1, 56, 8);
    ROCK_EXPECT_LAYOUT(RockProviderAuthoredGripPoseV1, 1744, 8);
    ROCK_EXPECT_LAYOUT(RockProviderPresentedHandPoseV1, 904, 8);
    ROCK_EXPECT_LAYOUT(RockProviderSemanticHandContactV1, 112, 8);
    ROCK_EXPECT_LAYOUT(RockProviderPlayerColliderDescriptorV1, 136, 8);
    ROCK_EXPECT_LAYOUT(RockProviderHandCollisionAvailabilityV1, 80, 8);
    ROCK_EXPECT_LAYOUT(RockProviderHandInputSuppressionStateV1, 80, 8);
    ROCK_EXPECT_LAYOUT(RockProviderOffhandReservationRequestV1, 56, 4);
    ROCK_EXPECT_LAYOUT(RockProviderOffhandReservationStateV1, 72, 8);
    ROCK_EXPECT_LAYOUT(RockProviderForceGrabRequestV1, 80, 8);
    ROCK_EXPECT_LAYOUT(RockProviderForceReleaseRequestV1, 72, 8);
    ROCK_EXPECT_LAYOUT(RockProviderThrownDropRequestV1, 96, 8);
    ROCK_EXPECT_LAYOUT(RockProviderInteractionCommandResultV1, 120, 8);
    ROCK_EXPECT_LAYOUT(RockProviderHandInputSuppressionRequestV1, 64, 4);
    ROCK_EXPECT_LAYOUT(RockProviderRawWandButtonStateV1, 32, 8);
    ROCK_EXPECT_LAYOUT(RockProviderWeaponPartTargetV1, 160, 8);
    ROCK_EXPECT_LAYOUT(RockProviderTransform, 52, 4);
    ROCK_EXPECT_LAYOUT(RockProviderWeaponPartDriveTargetV1, 192, 8);
    ROCK_EXPECT_LAYOUT(RockProviderWeaponPartGripStateV1, 248, 8);
    ROCK_EXPECT_LAYOUT(RockProviderWeaponContactQuery, 32, 4);
    ROCK_EXPECT_LAYOUT(RockProviderWeaponContactResult, 64, 8);
    ROCK_EXPECT_LAYOUT(RockProviderWeaponClassificationV1, 48, 8);
    ROCK_EXPECT_LAYOUT(RockProviderPoint3, 12, 4);
    ROCK_EXPECT_LAYOUT(RockProviderBounds3, 32, 4);
    ROCK_EXPECT_LAYOUT(RockProviderWeaponEmitterV1, 208, 8);
    ROCK_EXPECT_LAYOUT(RockProviderWorldRaycastRequestV1, 80, 4);
    ROCK_EXPECT_LAYOUT(RockProviderWorldRaycastResultV1, 96, 8);
    ROCK_EXPECT_LAYOUT(RockProviderColliderVisualizationRequestV1, 64, 8);
    ROCK_EXPECT_LAYOUT(RockProviderLogicalInputActionStateV1, 80, 8);
    ROCK_EXPECT_LAYOUT(RockProviderPlayerControllerStateV1, 104, 8);
    ROCK_EXPECT_LAYOUT(RockProviderPlayerControllerJumpRequestV1, 48, 4);
    ROCK_EXPECT_LAYOUT(RockProviderNativeAnimationAuthorityRequestV1, 64, 4);
    ROCK_EXPECT_LAYOUT(RockProviderNativeAnimationAuthorityStateV1, 64, 8);
    ROCK_EXPECT_LAYOUT(RockProviderAnimationPhaseContextV1, 72, 8);
    ROCK_EXPECT_LAYOUT(RockProviderEquippedWeaponGripStateV1, 224, 8);
    ROCK_EXPECT_LAYOUT(RockProviderEquippedWeaponHandlingRequestV1, 128, 4);
    ROCK_EXPECT_LAYOUT(RockProviderEquippedWeaponHandlingStateV1, 88, 8);
    ROCK_EXPECT_LAYOUT(RockProviderEquippedWeaponHandRequestV1, 64, 8);
    ROCK_EXPECT_LAYOUT(RockProviderHandVisualAuthorityRequestV1, 888, 4);
    ROCK_EXPECT_LAYOUT(RockProviderNativeAnimationRuntimePublicationV1, 64, 8);
    ROCK_EXPECT_LAYOUT(RockProviderDebugOverlayLineV1, 56, 4);
    ROCK_EXPECT_LAYOUT(RockProviderDebugOverlayTextV1, 200, 4);
    ROCK_EXPECT_LAYOUT(RockProviderDebugOverlayPublicationV1, 64, 8);
    ROCK_EXPECT_LAYOUT(RockProviderWeaponEvidenceDetailV1, 192, 8);
    ROCK_EXPECT_LAYOUT(RockProviderBodyContactV1, 128, 8);
    ROCK_EXPECT_LAYOUT(RockProviderTouchGrabTargetV1, 128, 8);
    ROCK_EXPECT_LAYOUT(RockProviderTouchGrabStateV1, 136, 8);
    ROCK_EXPECT_LAYOUT(RockProviderApi, 752, 8);
#undef ROCK_EXPECT_LAYOUT

    ok = ok && sizeof(RockProviderApi) == 94 * sizeof(void*);
    ok = ok && offsetof(RockProviderApi, getRawWandThumbstickV1) == 93 * sizeof(void*);
    ok = ok && alignof(RockProviderApi) == alignof(void*);
    ok = ok && offsetof(RockProviderApi, getProviderLimitsExtV1) == 54 * sizeof(void*);
    ok = ok && offsetof(RockProviderApi, clearNativeAnimationRuntimeV1) == 81 * sizeof(void*);
    ok = ok && offsetof(RockProviderApi, setTouchGrabTargetsForScopeV1) == 82 * sizeof(void*);
    ok = ok && offsetof(RockProviderApi, requestTouchGrabYieldV1) == 85 * sizeof(void*);
    ok = ok && offsetof(RockProviderApi, requestEquippedWeaponHandV1) == 86 * sizeof(void*);
    ok = ok && offsetof(RockProviderApi, queryWorldRaycastV1) == 87 * sizeof(void*);
    ok = ok && offsetof(RockProviderApi, setColliderVisualizationOverrideV1) == 88 * sizeof(void*);
    ok = ok && offsetof(RockProviderApi, clearColliderVisualizationOverrideV1) == 89 * sizeof(void*);
    ok = ok && offsetof(RockProviderApi, getLogicalInputActionStateV1) == 90 * sizeof(void*);
    ok = ok && offsetof(RockProviderApi, getPlayerControllerStateV1) == 91 * sizeof(void*);
    ok = ok && offsetof(RockProviderApi, requestPlayerControllerJumpV1) == 92 * sizeof(void*);
    ok = ok && offsetof(RockProviderWeaponPartGripStateV1, authoredSupportGrip) == 224;
    ok = ok && offsetof(RockProviderWeaponPartPoseV1, actionRole) == 44;
    ok = ok && offsetof(RockProviderEquippedWeaponGripStateV1, muzzleOriginGame) == 188;
    ok = ok && offsetof(RockProviderEquippedWeaponGripStateV1, muzzleDirectionGame) == 200;
    ok = ok &&
        offsetof(RockProviderHandInteractionStateV1, surfaceAnchorGame) == 144;
    ok = ok &&
        offsetof(RockProviderHandInteractionStateV1, surfaceGripMode) == 156;
    ok = ok &&
        offsetof(RockProviderTouchGrabStateV1, surfaceGripMode) == 52;
    ok = ok &&
        static_cast<std::uint32_t>(
            RockProviderHandInteractionFlagV1::TouchGrab) ==
            (1u << 12);
    ok = ok &&
        static_cast<std::uint32_t>(
            RockProviderHandInteractionFlagV1::FixedSurfaceLatch) ==
            (1u << 13);
    ok = ok &&
        static_cast<std::uint32_t>(
            RockProviderHandInteractionFlagV1::GlobalSurfaceLatch) ==
            (1u << 14);
    ok = ok &&
        static_cast<std::uint32_t>(
            RockProviderHandInteractionFlagV1::SurfaceAnchorValid) ==
            (1u << 15);
    ok = ok &&
        static_cast<std::uint32_t>(
            RockProviderHandInteractionFlagV1::MeshSurfaceAnchor) ==
            (1u << 16);
    ok = ok &&
        static_cast<std::uint32_t>(
            RockProviderHandInteractionFlagV1::MeshFingerPose) ==
            (1u << 17);
    ok = ok &&
        static_cast<std::uint32_t>(
            RockProviderHandInteractionFlagV1::MeshCollisionFallback) ==
            (1u << 18);
    ok = ok &&
        static_cast<std::uint32_t>(
            RockProviderTouchGrabStateFlagV1::MeshSurfaceAnchor) ==
            (1u << 6);
    ok = ok &&
        static_cast<std::uint32_t>(
            RockProviderTouchGrabStateFlagV1::MeshFingerPose) ==
            (1u << 7);
    ok = ok &&
        static_cast<std::uint32_t>(
            RockProviderTouchGrabStateFlagV1::MeshCollisionFallback) ==
            (1u << 8);
    ok = ok &&
        static_cast<std::uint32_t>(
            RockProviderSurfaceGripModeV1::CollisionAnchor) == 0;
    ok = ok &&
        static_cast<std::uint32_t>(
            RockProviderSurfaceGripModeV1::MeshAnchor) == 1;
    ok = ok &&
        static_cast<std::uint32_t>(
            RockProviderSurfaceGripModeV1::CollisionFallback) == 2;
    ok = ok && ROCK_PROVIDER_API_V1_NATIVE_ANIMATION_RUNTIME_CLEAR_TABLE_BYTES == 82 * sizeof(void*);
    ok = ok && ROCK_PROVIDER_API_V1_TOUCH_GRAB_TARGETS_TABLE_BYTES == 86 * sizeof(void*);
    ok = ok && ROCK_PROVIDER_API_V1_EQUIPPED_WEAPON_HAND_REQUEST_TABLE_BYTES == 87 * sizeof(void*);
    ok = ok && ROCK_PROVIDER_API_V1_WORLD_RAYCASTS_TABLE_BYTES == 88 * sizeof(void*);
    ok = ok && ROCK_PROVIDER_API_V1_COLLIDER_VISUALIZATION_OVERRIDE_TABLE_BYTES == 90 * sizeof(void*);
    ok = ok && ROCK_PROVIDER_API_V1_LOGICAL_INPUT_ACTION_STATE_TABLE_BYTES == 91 * sizeof(void*);
    ok = ok && ROCK_PROVIDER_API_V1_PLAYER_CONTROLLER_STATE_TABLE_BYTES == 92 * sizeof(void*);
    ok = ok && ROCK_PROVIDER_API_V1_PLAYER_CONTROLLER_JUMP_TABLE_BYTES == 93 * sizeof(void*);
    ok = ok && ROCK_PROVIDER_API_V1_RAW_WAND_THUMBSTICK_TABLE_BYTES == sizeof(RockProviderApi);
    ok = ok &&
        static_cast<std::uint32_t>(
            RockProviderHandInputSuppressionFlagV1::SuppressNativeVats) ==
            (1u << 5);
    ok = ok &&
        static_cast<std::uint32_t>(
            RockProviderHandInputSuppressionFlagV1::SuppressNativeVans) ==
            (1u << 6);
    ok = ok &&
        static_cast<std::uint32_t>(
            RockProviderFeatureBit2V1::NativeVatsVansInputSuppression) ==
            (1u << 30);
    ok = ok &&
        static_cast<std::uint32_t>(
            RockProviderFeatureBit2V1::WorldRaycasts) ==
            (1u << 31);
    ok = ok &&
        static_cast<std::uint32_t>(
            RockProviderFeatureBitV1::ColliderVisualizationOverride) ==
            (1u << 31);
    ok = ok &&
        static_cast<std::uint32_t>(
            RockProviderConsumerCapabilityV1::ColliderVisualizationOverride) ==
            (1u << 26);
    ok = ok &&
        static_cast<std::uint32_t>(
            RockProviderConsumerCapabilityV1::PlayerController) ==
            (1u << 27);
    ok = ok &&
        static_cast<std::uint32_t>(
            RockProviderEquippedWeaponGripStateFlagV1::MuzzleWorldValid) ==
            (1u << 7);
    return ok ? 0 : 1;
}
