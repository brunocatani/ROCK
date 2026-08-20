#define ROCK_API_EXPORTS
// Build the DLL side of the public provider ABI.
#include "api/detail/ProviderApiEntryPoints.h"
#include "api/detail/ProviderApiState.h"
#include "api/detail/ProviderFeatureBits.h"

#include <algorithm>
#include <cstddef>
#include <cstring>

namespace rock::provider::detail
{
    bool ROCK_PROVIDER_CALL apiGetProviderLimitsV1(RockProviderLimitsV1* outLimits)
    {
        if (!outLimits || outLimits->size <
                offsetof(RockProviderLimitsV1, featureBits) +
                    sizeof(outLimits->featureBits)) {
            return false;
        }

        RockProviderLimitsV1 limits{};
        limits.size = sizeof(RockProviderLimitsV1);
        limits.version = ROCK_PROVIDER_API_VERSION;
        limits.featureBits = kProviderFeatureBitsV1;
        limits.maxFrameCallbacks = ROCK_PROVIDER_MAX_FRAME_CALLBACKS_V1;
        limits.maxConsumers = ROCK_PROVIDER_MAX_CONSUMERS_V1;
        limits.maxExternalBodies = ROCK_PROVIDER_MAX_EXTERNAL_BODIES_V1;
        limits.maxExternalContacts = ROCK_PROVIDER_MAX_EXTERNAL_CONTACTS_V1;
        limits.maxBodyContacts = ROCK_PROVIDER_MAX_BODY_CONTACTS_V1;
        limits.maxWeaponBodies = ROCK_PROVIDER_MAX_WEAPON_BODIES;
        limits.maxInteractionCommands = ROCK_PROVIDER_MAX_INTERACTION_COMMANDS_V1;
        limits.maxCompletedInteractionCommands = ROCK_PROVIDER_MAX_COMPLETED_INTERACTION_COMMANDS_V1;
        limits.providerApiByteSize = static_cast<std::uint32_t>(sizeof(RockProviderApi));
        limits.maxWeaponEmitters = ROCK_PROVIDER_MAX_WEAPON_EMITTERS_V1;
        limits.maxAnimationPhaseCallbacks =
            ROCK_PROVIDER_MAX_ANIMATION_PHASE_CALLBACKS_V1;
        limits.maxHandVisualAuthorityPublications =
            static_cast<std::uint32_t>(s_handVisualAuthoritySlots.size());
        limits.maxNativeAnimationRuntimeProviders = 1;
        limits.maxEquippedWeaponHandlingAuthorities = 1;
        limits.maxEquippedWeaponHandlingLeaseFrames =
            ROCK_PROVIDER_MAX_EQUIPPED_WEAPON_HANDLING_LEASE_FRAMES_V1;
        limits.maxDebugOverlayPublishers =
            ROCK_PROVIDER_MAX_DEBUG_OVERLAY_PUBLISHERS_V1;
        limits.maxDebugOverlayLinesPerPublisher =
            ROCK_PROVIDER_MAX_DEBUG_OVERLAY_LINES_PER_PUBLISHER_V1;
        limits.maxDebugOverlayTextPerPublisher =
            ROCK_PROVIDER_MAX_DEBUG_OVERLAY_TEXT_PER_PUBLISHER_V1;
        limits.maxDebugOverlayLines =
            ROCK_PROVIDER_MAX_DEBUG_OVERLAY_LINES_V1;
        limits.maxDebugOverlayText =
            ROCK_PROVIDER_MAX_DEBUG_OVERLAY_TEXT_V1;
        const auto copySize = (std::min<std::size_t>)(
            outLimits->size,
            sizeof(limits));
        std::memcpy(outLimits, &limits, copySize);
        outLimits->size = static_cast<std::uint32_t>(copySize);
        return true;
    }

    bool ROCK_PROVIDER_CALL apiGetProviderLimitsExtV1(
        RockProviderLimitsExtV1* outLimits)
    {
        if (!outLimits || outLimits->size <
                offsetof(RockProviderLimitsExtV1, featureBits2) +
                    sizeof(outLimits->featureBits2)) {
            return false;
        }
        RockProviderLimitsExtV1 limits{};
        limits.featureBits = kProviderFeatureBitsV1;
        limits.featureBits2 = kProviderFeatureBits2V1;
        limits.providerApiByteSize = sizeof(RockProviderApi);
        limits.maxConsumers = ROCK_PROVIDER_MAX_CONSUMERS_V1;
        limits.maxFrameCallbacks = ROCK_PROVIDER_MAX_FRAME_CALLBACKS_V1;
        limits.maxExternalBodies = ROCK_PROVIDER_MAX_EXTERNAL_BODIES_V1;
        limits.maxExternalScopes = ROCK_PROVIDER_MAX_EXTERNAL_SCOPES_V1;
        limits.maxExternalContacts = ROCK_PROVIDER_MAX_EXTERNAL_CONTACTS_V1;
        limits.maxBodyContacts = ROCK_PROVIDER_MAX_BODY_CONTACTS_V1;
        limits.maxWeaponBodies = ROCK_PROVIDER_MAX_WEAPON_BODIES;
        limits.maxWeaponEmitters = ROCK_PROVIDER_MAX_WEAPON_EMITTERS_V1;
        limits.maxInteractionCommands = ROCK_PROVIDER_MAX_INTERACTION_COMMANDS_V1;
        limits.maxCompletedInteractionCommands =
            ROCK_PROVIDER_MAX_COMPLETED_INTERACTION_COMMANDS_V1;
        limits.maxHandInputSuppressions =
            ROCK_PROVIDER_MAX_HAND_INPUT_SUPPRESSIONS_V1;
        limits.maxHandInputSuppressionLeaseFrames =
            ROCK_PROVIDER_MAX_HAND_INPUT_SUPPRESSION_LEASE_FRAMES_V1;
        limits.maxWeaponPartTargets = ROCK_PROVIDER_MAX_WEAPON_PART_TARGETS_V1;
        limits.maxWeaponPartDrives = ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1;
        limits.maxWeaponPartDriveLeaseFrames =
            ROCK_PROVIDER_MAX_WEAPON_PART_DRIVE_LEASE_FRAMES_V1;
        limits.maxWeaponPartPoses = ROCK_PROVIDER_MAX_WEAPON_PART_POSES_V1;
        limits.maxWeaponPartDriveResults =
            ROCK_PROVIDER_MAX_WEAPON_PART_DRIVE_RESULTS_V1;
        limits.maxNativeAnimationAuthorityLeaseFrames =
            ROCK_PROVIDER_MAX_NATIVE_ANIMATION_AUTHORITY_LEASE_FRAMES_V1;
        limits.maxAnimationPhaseCallbacks =
            ROCK_PROVIDER_MAX_ANIMATION_PHASE_CALLBACKS_V1;
        limits.maxHandVisualAuthorityPublications =
            static_cast<std::uint32_t>(s_handVisualAuthoritySlots.size());
        limits.maxNativeAnimationRuntimeProviders = 1;
        limits.maxEquippedWeaponHandlingAuthorities = 1;
        limits.maxEquippedWeaponHandlingLeaseFrames =
            ROCK_PROVIDER_MAX_EQUIPPED_WEAPON_HANDLING_LEASE_FRAMES_V1;
        limits.maxDebugOverlayPublishers =
            ROCK_PROVIDER_MAX_DEBUG_OVERLAY_PUBLISHERS_V1;
        limits.maxDebugOverlayLinesPerPublisher =
            ROCK_PROVIDER_MAX_DEBUG_OVERLAY_LINES_PER_PUBLISHER_V1;
        limits.maxDebugOverlayTextPerPublisher =
            ROCK_PROVIDER_MAX_DEBUG_OVERLAY_TEXT_PER_PUBLISHER_V1;
        limits.maxDebugOverlayLines = ROCK_PROVIDER_MAX_DEBUG_OVERLAY_LINES_V1;
        limits.maxDebugOverlayText = ROCK_PROVIDER_MAX_DEBUG_OVERLAY_TEXT_V1;
        limits.maxProviderEvents = ROCK_PROVIDER_MAX_PROVIDER_EVENTS_V1;
        limits.maxWeaponCompositionEntries =
            ROCK_PROVIDER_MAX_WEAPON_COMPOSITION_ENTRIES_V1;
        limits.maxSemanticHandContacts =
            ROCK_PROVIDER_MAX_SEMANTIC_HAND_CONTACTS_V1;
        limits.maxPlayerColliderDescriptors =
            ROCK_PROVIDER_MAX_PLAYER_COLLIDER_DESCRIPTORS_V1;
        limits.maxOffhandReservationLeaseFrames =
            ROCK_PROVIDER_MAX_OFFHAND_RESERVATION_LEASE_FRAMES_V1;
        limits.maxHandVisualAuthorityLeaseFrames =
            ROCK_PROVIDER_MAX_HAND_VISUAL_AUTHORITY_LEASE_FRAMES_V1;
        limits.maxNativeAnimationRuntimeLeaseFrames =
            ROCK_PROVIDER_MAX_NATIVE_ANIMATION_RUNTIME_LEASE_FRAMES_V1;
        limits.maxDebugOverlayPublicationLeaseFrames =
            ROCK_PROVIDER_MAX_DEBUG_OVERLAY_PUBLICATION_LEASE_FRAMES_V1;
        limits.maxNativeAnimationAuthorityOwners =
            static_cast<std::uint32_t>(s_nativeAnimationAuthoritySlots.size());
        limits.maxWeaponEvidenceDetails =
            ROCK_PROVIDER_MAX_WEAPON_EVIDENCE_DETAILS_V1;
        limits.maxWeaponEvidencePointsPerDetail =
            ROCK_PROVIDER_MAX_WEAPON_EVIDENCE_POINTS_PER_DETAIL_V1;
        limits.maxTouchGrabTargets =
            ROCK_PROVIDER_MAX_TOUCH_GRAB_TARGETS_V1;
        limits.maxTouchGrabScopes =
            ROCK_PROVIDER_MAX_TOUCH_GRAB_SCOPES_V1;
        limits.maxTouchGrabTargetLeaseFrames =
            ROCK_PROVIDER_MAX_TOUCH_GRAB_TARGET_LEASE_FRAMES_V1;
        limits.maxWorldRaycastsPerOwnerPerFrame =
            ROCK_PROVIDER_MAX_WORLD_RAYCASTS_PER_OWNER_PER_FRAME_V1;

        const auto copySize = (std::min<std::size_t>)(
            outLimits->size,
            sizeof(limits));
        std::memcpy(outLimits, &limits, copySize);
        outLimits->size = static_cast<std::uint32_t>(copySize);
        return true;
    }

}
