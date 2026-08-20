#pragma once

#include "api/ROCKProviderApi.h"

namespace rock::provider::detail
{
    constexpr std::uint32_t kProviderFeatureBitsV1 =
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::FrameCallbacks) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::LifecycleFields) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::HandFrames) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::WeaponEvidence) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::BodyContacts) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::ExternalContacts) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::ConsumerRegistrationV1) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::OwnerFilteredExternalContactsV1) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::InteractionCommandQueue) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::ForceGrabCommand) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::ForceReleaseCommand) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::ThrownDropCommand) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::HandInputSuppression) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::WeaponPartInteraction) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::WeaponPartGripState) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::WeaponPartRecordIdentity) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::WeaponPartTargetNonExclusive) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::RawWandButtonState) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::PipboyInputSuppression) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::WeaponEmitters) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::NativeAnimationAuthority) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::AnimationPhases) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::EquippedWeaponGripState) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::HandVisualAuthority) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::NativeAnimationRuntimeProvider) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::EquippedWeaponHandlingAuthority) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::DebugOverlayPublication) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::PresentedHandFrames) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::EquippedWeaponHandRequest) |
        static_cast<std::uint32_t>(
            RockProviderFeatureBitV1::ColliderVisualizationOverride);
    constexpr std::uint32_t kProviderFeatureBits2V1 =
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::SafeDescriptor) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::ExtendedLimits) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::PublicStructureSizes) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::OwnerFrameCallbacks) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::HandInteractionState) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::ProviderEvents) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::EquippedWeaponState) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::ExternalBodyScopes) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::ExternalContactCursor) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::WeaponPartResolution) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::WeaponPartPoses) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::WeaponPartDriveResults) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::ScopeSightState) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::WeaponComposition) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::AuthoredGripSnapshot) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::PresentedHandPose) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::SemanticHandContacts) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::PlayerColliderDescriptors) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::HandCollisionAvailability) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::CommandCancellation) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::InputSuppressionState) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::OffhandReservationLeases) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::SnapshotEnrichment) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::NativeAnimationRuntimeLeases) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::StatefulPublicationLeases) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::CommandLifecycle) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::InputSampleMetadata) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::WeaponClassificationEnrichment) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::ExternalContactEnrichment) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::TouchGrabTargets) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::NativeVatsVansInputSuppression) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::WorldRaycasts);
}

