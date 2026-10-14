#pragma once
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <ROCK/Core.h>
#include <ROCK/Animation.h>
#include <ROCK/Collision.h>
#include <ROCK/Grab.h>
#include <ROCK/Input.h>
#include <ROCK/Touch.h>
#include <ROCK/Weapon.h>
#include <ROCK/WeaponParts.h>
#include "ProviderPublication.h"

// Private runtime representation; never installed or returned across the DLL boundary.
namespace rock::provider {
    inline constexpr std::uint32_t ROCK_PROVIDER_API_VERSION = 1;
    // Evidence describes the complete classified weapon-part catalog and is
    // independent from the compact body array embedded in frame snapshots.
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_EVIDENCE_DETAILS_V1 = api::weaponparts::kMaxEvidenceDetails;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_EVIDENCE_NAME = 64;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_CONSUMERS_V1 = api::core::kMaxOwners;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_INTERACTION_COMMANDS_V1 = api::grab::kMaxQueuedCommands;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_COMPLETED_INTERACTION_COMMANDS_V1 = api::grab::kMaxCommandResults;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_HAND_INPUT_SUPPRESSIONS_V1 = api::input::kMaxSuppressions;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_HAND_INPUT_SUPPRESSION_LEASE_FRAMES_V1 = api::input::kMaxSuppressionLeaseFrames;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_PART_TARGETS_V1 = api::weaponparts::kMaxTargets;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1 = api::weaponparts::kMaxDrives;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_PART_DRIVE_LEASE_FRAMES_V1 = api::weaponparts::kMaxDriveLeaseFrames;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_NATIVE_ANIMATION_AUTHORITY_LEASE_FRAMES_V1 = api::animation::kMaxNativeAuthorityLeaseFrames;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_ANIMATION_PHASE_CALLBACKS_V1 = api::animation::kMaxPhaseCallbacks;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_HAND_VISUAL_AUTHORITY_LEASE_FRAMES_V1 = api::animation::kMaxHandVisualLeaseFrames;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_NATIVE_ANIMATION_RUNTIME_LEASE_FRAMES_V1 = api::animation::kMaxRuntimePublicationLeaseFrames;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_HAND_HELD_BODIES_V1 = 8;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_PART_DRIVE_RESULTS_V1 = api::weaponparts::kMaxDrives;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_OFFHAND_RESERVATION_LEASE_FRAMES_V1 = api::grab::kMaxReservationLeaseFrames;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_TOUCH_GRAB_TARGETS_V1 = api::touch::kMaxTargets;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_TOUCH_GRAB_SCOPES_V1 = api::touch::kMaxScopes;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_TOUCH_GRAB_TARGET_LEASE_FRAMES_V1 = api::touch::kMaxLeaseFrames;
    inline constexpr std::uint16_t ROCK_PROVIDER_ALL_FINGER_LOCAL_TRANSFORMS_V1 = 0x7FFFu;

    /*
     * Public frameIndex/acceptedFrame/committedFrame/appliedFrame and animation
     * phase frames use the same monotonic game-frame clock, including across
     * provider recreation. Lifecycle callbacks can share a frame index; use
     * state/event sequences to distinguish changes within that frame.
     * Lease fences use the most recent provider snapshot frame and retire at
     * the next publication boundary, after ROCK consumes that update's drives.
     * Every V1 lease uses the same exclusive expiry fence. A publication made
     * at frame F with leaseFrames N is active while currentFrame < F + N and
     * expires at F + N. Zero is invalid; values above the published family
     * limit are clamped. Refresh replaces the prior expiry and generations.
     */















    /*
     * Selective Bethesda animation authority. ROCK preserves the authored
     * relationship between both arms, hands, and weapon, then rigidly anchors
     * both skeleton trees to the visible first-person weapon/controller world
     * frame while retaining the weapon's authored motion since lease acquisition.
     * Arms covers only the two
     * collarbone-to-hand chains; Hands adds the hand roots and finger/thumb
     * descendants; Weapon covers only Weapon and WeaponLeft. The character
     * root, COM, torso, head, and legs remain owned by the live VR body.
     */

















    [[nodiscard]] inline constexpr bool hasLifecycleFlag(std::uint32_t flags, api::core::LifecycleFlag flag)
    {
        return (flags & static_cast<std::uint32_t>(flag)) != 0;
    }


    [[nodiscard]] inline constexpr bool hasHandInputSuppressionFlagV1(
        std::uint32_t flags,
        ::rock::api::input::HandInputSuppressionFlag flag)
    {
        return (flags & static_cast<std::uint32_t>(flag)) != 0;
    }


    [[nodiscard]] inline constexpr bool hasTouchGrabTargetFlagV1(
        std::uint32_t flags,
        ::rock::api::touch::TouchGrabTargetFlag flag)
    {
        return (flags & static_cast<std::uint32_t>(flag)) != 0;
    }



    /*
     * Extensible limits surface. Callers set size to their local structure
     * size; ROCK prefix-copies the supported bytes and returns the copied size.
     */

    /*
     * Pointer-sized fields retained by the original V1 prefix are non-owning
     * identity witnesses, never ownership or mutation authority. They may be
     * compared only on ROCK's game-thread callback/query frame while the
     * accompanying frame and generation identities still match, and must not
     * be retained or dereferenced by a consumer. Command targetRefr inputs are
     * ABI-retained but ignored; use targetFormId and/or targetBodyId. Command
     * result targetRefr is zero. New V1 structures use value identity instead.
     */






    /*
     * Frame-local value snapshot of FO4VR's native player character controller.
     * CheckPenetration is a reserved, ignored V1 flag. PenetrationChecked and
     * Penetrating are never reported; their absence proves no clearance.
     * No engine pointer or retained runtime identity crosses the ABI.
     */
















    /*
     * Publications are copied into ROCK's bounded registry and replace one
     * owner/scope transactionally. Every target must carry the current,
     * nonzero world/skeleton/provider generations and a nonzero owner-defined
     * targetGeneration. Bump targetGeneration whenever the resolved body or
     * any behavioral geometry/policy changes; refresh the unchanged value only
     * to renew its lease.
     *
     * LimitedHinge coordinates and limits are radians. LimitedPrismatic
     * coordinates and limits are game units. Their pivot and normalized axis
     * are in current world/game space. ROCK temporarily converts a keyframed
     * mechanism body to dynamic while held, owns all constraints, and restores
     * the original motion class before publishing Latched/Yielded/Invalidated.
     *
     * FixedAnchor either names one body or uses MatchAnyBody plus a nonzero
     * allowedLayerMask. A dynamic palm/fingertip touch plus grip locks the
     * rendered hand and its dynamic proxy bodies relative to the matched body
     * until grip release. ROCK never changes, activates, constrains, or writes
     * velocity to the matched body. Held state reports the resolved bodyId,
     * active hand mask, and contact point/normal when the manifold supplies
     * them; referenceFormId/referenceNativeHandle remain provider-authored.
     * Exact body registrations are resolved before wildcard registrations.
     * One wildcard descriptor owns at most one resolved body concurrently;
     * publish disjoint right/left wildcard descriptors when a consumer needs
     * two independent surfaces at once (for example, two-hand climbing).
     */



    // Availability is independent of boolean values. Node names are descriptive,
    // not globally unique part IDs. All transforms use game units.

}
