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
    inline constexpr std::uint32_t ROCK_PROVIDER_INTERACTION_COMMAND_RESULT_V1_PREFIX_SIZE = 112;
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





    enum class RockProviderForceGrabFlagV1 : std::uint32_t
    {
        None = 0,
        UsePreferredGrabPointGame = 1u << 0,
        // targetFormId is an owned ALCH/throwable base form, not a world REFR.
        // Requires hand=None and no other flags. Transfers exactly one item to
        // a free hand, without using or equipping it. Throwables prefer ROCK's
        // configured default hand; other inventory items prefer the right.
        // Query InventoryForceGrab support first. Failure before the drop leaves
        // inventory unchanged; failed attachment attempts return the exact drop.
        FromPlayerInventory = 1u << 1,
    };


    enum class RockProviderWeaponPartTargetFlagV1 : std::uint32_t
    {
        None = 0,
        MatchBodyId = 1u << 0,
        MatchSourceRoot = 1u << 1,
        MatchSourceName = 1u << 2,
        MatchPartKind = 1u << 3,
        MatchReloadRole = 1u << 4,
        MatchSupportRole = 1u << 5,
        MatchSocketRole = 1u << 6,
        MatchActionRole = 1u << 7,
        /*
         * Non-exclusive target (feature bit WeaponPartTargetNonExclusive):
         * grants its grab mode on match without activating whitelist gating,
         * so unmatched parts keep their normal grip behavior. Omit the flag
         * for reload-session semantics where every unmatched part grip is
         * rejected while the whitelist is active.
         */
        NonExclusive = 1u << 8,
    };












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

    struct RockProviderForceGrabRequestV1
    {
        std::uint32_t size{ sizeof(RockProviderForceGrabRequestV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        api::Hand hand{ api::Hand::None };
        std::uint32_t flags{ 0 };
        std::uintptr_t targetRefr{ 0 };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t targetBodyId{ 0x7FFF'FFFF };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        float maxDistanceGame{ 0.0f };
        float preferredGrabPointGame[3]{};
        std::uint32_t reserved[5]{};
    };

    struct RockProviderForceReleaseRequestV1
    {
        std::uint32_t size{ sizeof(RockProviderForceReleaseRequestV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        api::Hand hand{ api::Hand::None };
        std::uint32_t flags{ 0 };
        std::uintptr_t targetRefr{ 0 };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t targetBodyId{ 0x7FFF'FFFF };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        float linearVelocityHavok[3]{};
        float angularVelocityRadiansPerSecond[3]{};
        std::uint32_t reserved[1]{};
    };

    struct RockProviderThrownDropRequestV1
    {
        std::uint32_t size{ sizeof(RockProviderThrownDropRequestV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        api::Hand hand{ api::Hand::None };
        std::uint32_t flags{ 0 };
        std::uintptr_t targetRefr{ 0 };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t targetBodyId{ 0x7FFF'FFFF };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved0{ 0 };
        float linearVelocityHavok[3]{};
        float angularVelocityRadiansPerSecond[3]{};
        std::uint32_t reserved[6]{};
    };

    struct RockProviderInteractionCommandResultV1
    {
        std::uint32_t size{ sizeof(RockProviderInteractionCommandResultV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t commandId{ 0 };
        api::grab::InteractionCommandKind kind{ api::grab::InteractionCommandKind::Unknown };
        api::grab::InteractionCommandState state{ api::grab::InteractionCommandState::Unknown };
        api::grab::InteractionFailure failure{ api::grab::InteractionFailure::None };
        api::Hand hand{ api::Hand::None };
        std::uintptr_t targetRefr{ 0 };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t targetBodyId{ 0x7FFF'FFFF };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        api::grab::CommandStage stage{ api::grab::CommandStage::Unknown };
        api::grab::InteractionFailure failureStage{ api::grab::InteractionFailure::None };
        std::uint64_t acceptedFrame{ 0 };
        std::uint64_t committedFrame{ 0 };
        std::uint64_t appliedFrame{ 0 };
        std::uint32_t reserved{ 0 };
    };




    struct RockProviderWeaponPartTargetV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponPartTargetV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t flags{ 0 };
        api::weaponparts::GrabMode grabMode{ api::weaponparts::GrabMode::None };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uintptr_t sourceRoot{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uint32_t groupId{ 0 };
        std::uint32_t priority{ 0 };
        char sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};
        std::uint32_t reserved[8]{};
    };

    struct RockProviderWeaponPartDriveTargetV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponPartDriveTargetV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t flags{ 0 };
        api::weaponparts::DriveSpace driveSpace{ api::weaponparts::DriveSpace::WeaponRootLocal };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uintptr_t sourceRoot{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t groupId{ 0 };
        std::uint32_t priority{ 0 };
        std::uint32_t leaseFrames{ 1 };
        api::Transform targetTransform{};
        char sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};
        std::uint32_t reserved[7]{};
    };

    /*
     * Per-hand grip report: which weapon part (if any) the hand is attached
     * to this frame. Polled; gripSequence increases on every fresh capture so
     * consumers detect re-grabs without frame callbacks. sourceRoot is a
     * non-owning engine pointer valid only while weaponGenerationKey matches
     * the frame snapshot. handPartLocal is the hand frame captured at grip
     * start in handPartLocalSpace; composing it with the part's current world
     * transform yields the glued hand target, so a consumer driving the part
     * via setWeaponPartDriveTargetsV1 can also derive controller-to-part
     * displacement from it.
     */
    struct RockProviderWeaponPartGripStateV1
    {
        // Captured ROCK grip details only. Native carry can occupy the hand
        // while active is zero; use getHandInteractionStateV1 for occupancy.
        std::uint32_t size{ sizeof(RockProviderWeaponPartGripStateV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        api::Hand hand{ api::Hand::None };
        api::weaponparts::GripKind gripKind{ api::weaponparts::GripKind::None };
        std::uint32_t active{ 0 };
        std::uint32_t attachOnly{ 0 };
        std::uint64_t gripSequence{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uintptr_t sourceRoot{ 0 };
        std::uint64_t providerOwnerToken{ 0 };
        std::uint32_t providerGroupId{ 0 };
        std::uint32_t providerGrabMode{ 0 };
        std::uint32_t hasHandPartLocal{ 0 };
        api::weaponparts::GripLocalSpace handPartLocalSpace{ api::weaponparts::GripLocalSpace::WeaponRootLocal };
        api::Transform handPartLocal{};
        char sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};
        // Record-authored identity of the gripped part; see the evidence
        // detail struct for field semantics (WeaponPartRecordIdentity bit).
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
        std::uint32_t classificationSource{ 0 };
        // Exact ROCK grip provenance. Nonzero only when the active support
        // grip was acquired from the current authored support-pose capture.
        std::uint32_t authoredSupportGrip{ 0 };
        std::uint32_t reserved[5]{};
    };


    struct RockProviderWeaponContactQuery
    {
        std::uint32_t size{ sizeof(RockProviderWeaponContactQuery) };
        float pointGame[3]{};
        float radiusGame{ 0.0f };
        std::uint32_t flags{ 0 };
        std::uint32_t reserved[2]{};
    };

    struct RockProviderWeaponContactResult
    {
        std::uint32_t size{ sizeof(RockProviderWeaponContactResult) };
        std::uint32_t valid{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uintptr_t interactionRoot{ 0 };
        std::uintptr_t sourceRoot{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        float probeDistanceGame{ 0.0f };
        std::uint32_t reserved{ 0 };
    };


    /*
     * Frame-local value snapshot of FO4VR's native player character controller.
     * CheckPenetration is a reserved, ignored V1 flag. PenetrationChecked and
     * Penetrating are never reported; their absence proves no clearance.
     * No engine pointer or retained runtime identity crosses the ABI.
     */





    struct RockProviderAnimationPhaseContextV1
    {
        std::uint32_t size{ sizeof(RockProviderAnimationPhaseContextV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        ::rock::api::animation::AnimationPhase phase{ ::rock::api::animation::AnimationPhase::BeforeRock };
        std::uint32_t flags{ 0 };
        std::uint64_t frameIndex{ 0 };
        float deltaSeconds{ 0.0f };
        std::uint32_t activeNativeAnimationAuthorityFlags{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[7]{};
    };






    /*
     * Detailed weapon evidence carries semantic body identity, local generated
     * bounds, and total point count without making the fixed function table own
     * variable-length buffers. Callers fetch the local mesh point cloud through
     * the body-id keyed copy function below.
     */
    struct RockProviderWeaponEvidenceDetailV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponEvidenceDetailV1) };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uint32_t fallbackGripPose{ 0 };
        std::uintptr_t interactionRoot{ 0 };
        std::uintptr_t sourceRoot{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        api::Bounds3 localBoundsGame{};
        std::uint32_t pointCount{ 0 };
        char sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};
        /*
         * Record-authored identity (feature bit WeaponPartRecordIdentity;
         * fields were reserved zeros before it). omodFormId is the installed
         * OMOD occupying this part's slot (0 when unpaired); attachPointFormId
         * is the vanilla attach-point keyword of that slot; classification-
         * Source is api::weaponparts::ClassificationSource.
         */
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
        std::uint32_t classificationSource{ 0 };
        std::uint32_t reserved[6]{};
    };



    struct RockProviderHandInteractionStateV1
    {
        std::uint32_t size{ sizeof(RockProviderHandInteractionStateV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t frameIndex{ 0 };
        api::Hand hand{ api::Hand::None };
        api::grab::HandInteractionPhase phase{ api::grab::HandInteractionPhase::Idle };
        api::grab::BodyContactTargetKind targetKind{ api::grab::BodyContactTargetKind::Unknown };
        std::uint32_t flags{ 0 };
        // TouchGrab: zero-extended referenceNativeHandle. Weapon: generation
        // key. Interpret only with targetKind; zero means unavailable.
        std::uint64_t reservedTargetIdentity{ 0 };
        std::uint32_t targetFormId{ 0 };
        // For TouchGrab, always identifies the target, including global world surfaces.
        std::uint32_t primaryBodyId{ 0x7FFF'FFFF };
        std::uint32_t heldBodyCount{ 0 };
        std::uint32_t heldBodyIds[ROCK_PROVIDER_MAX_HAND_HELD_BODIES_V1]{};
        std::uint32_t effectiveInputSuppressionFlags{ 0 };
        std::uint32_t collisionAvailabilityFlags{ 0 };
        std::uint64_t stateSequence{ 0 };
        std::uint64_t targetSequence{ 0 };
        std::uint64_t gripSequence{ 0 };
        std::uint64_t releaseSequence{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t collisionGeneration{ 0 };
        api::Point3 surfaceAnchorGame{};
        ::rock::api::touch::SurfaceGripMode surfaceGripMode{
            ::rock::api::touch::SurfaceGripMode::CollisionAnchor
        };
    };



    struct RockProviderWeaponPartResolutionQueryV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponPartResolutionQueryV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uintptr_t sourceRoot{ 0 };
        char sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};
        std::uint32_t reserved[4]{};
    };

    struct RockProviderWeaponPartResolutionResultV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponPartResolutionResultV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t whitelistActive{ 0 };
        std::uint32_t matched{ 0 };
        api::weaponparts::GrabMode grabMode{ api::weaponparts::GrabMode::None };
        std::uint32_t groupId{ 0 };
        std::uint32_t priority{ 0 };
        std::uint32_t reserved0{ 0 };
        std::uint64_t winningOwnerToken{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t reserved[4]{};
    };

    struct RockProviderWeaponPartPoseV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponPartPoseV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t frameIndex{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t partKind{ 0 };
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
        std::uint32_t flags{ 0 };
        // api::weaponparts::ActionRole for this semantic source.
        std::uint32_t actionRole{ 0 };
        api::Transform sourceParentLocal{};
        api::Transform weaponRootLocal{};
        char sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};
        std::uint32_t reserved[4]{};
    };

    struct RockProviderWeaponPartDriveApplicationResultV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponPartDriveApplicationResultV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t frameIndex{ 0 };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t groupId{ 0 };
        std::uint32_t priority{ 0 };
        api::weaponparts::DriveApplication result{
            api::weaponparts::DriveApplication::Unknown
        };
        api::Transform appliedSourceParentLocal{};
        char sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};
        std::uint32_t reserved[4]{};
    };



    struct RockProviderOffhandReservationRequestV1
    {
        std::uint32_t size{ sizeof(RockProviderOffhandReservationRequestV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        api::Hand hand{ api::Hand::None };
        api::grab::HandReservationKind reservation{ api::grab::HandReservationKind::Normal };
        std::uint32_t leaseFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    struct RockProviderOffhandReservationStateV1
    {
        std::uint32_t size{ sizeof(RockProviderOffhandReservationStateV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        api::Hand hand{ api::Hand::None };
        api::grab::HandReservationKind reservation{ api::grab::HandReservationKind::Normal };
        std::uint32_t active{ 0 };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
        std::uint32_t remainingFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[6]{};
    };

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
    struct RockProviderReferenceQueryV1
    {
        std::uint32_t size{ sizeof(RockProviderReferenceQueryV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t referenceFormId{ 0 };
        // Optional additional identity check; zero means unspecified.
        std::uint32_t referenceNativeHandle{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        // Native furniture use is indexed. Zero is marker 0, not "any marker".
        std::int32_t furnitureMarkerIndex{ 0 };
    };

    struct RockProviderReferenceInteractionV1
    {
        std::uint32_t size{ sizeof(RockProviderReferenceInteractionV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t flags{ 0 };
        std::uint32_t referenceFormId{ 0 };
        std::uint32_t referenceNativeHandle{ 0 };
        std::uint32_t baseFormId{ 0 };
        std::uint32_t baseFormType{ 0 };
        std::uint32_t activationBlocked{ 0 };
        api::grab::NativeOpenState openState{};
        std::uint32_t furnitureInUse{ 0 };
        std::uint32_t furnitureInUseIncludingReservations{ 0 };
        std::int32_t furnitureMarkerIndex{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint64_t frameIndex{ 0 };
    };

    struct RockProviderHandTargetDetailsV1
    {
        std::uint32_t size{ sizeof(RockProviderHandTargetDetailsV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        RockProviderHandInteractionStateV1 handState{};
        RockProviderReferenceInteractionV1 reference{};
        std::uint32_t flags{ 0 };
        std::uint32_t collisionLayer{ 0 };
        api::Point3 anchorGame{};
        api::Point3 normalGame{};
        api::PowerArmorPoint powerArmorPoint{};
        std::uint32_t sourceTriangleIndex{ 0xFFFF'FFFFu };
        char collisionNodeName[64]{};
        char meshPartName[64]{};
    };

    struct RockProviderPowerArmorPointPoseV1
    {
        api::PowerArmorPoint point{};
        // Animated armor-bone origin, not a sampled human palm pose.
        std::uint32_t valid{ 0 };
        api::Transform world{};
        api::Transform frameLocal{};
    };

    struct RockProviderPowerArmorTargetV1
    {
        std::uint32_t size{ sizeof(RockProviderPowerArmorTargetV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        RockProviderReferenceInteractionV1 touchedReference{};
        RockProviderReferenceInteractionV1 frameReference{};
        std::uint32_t flags{ 0 };
        // Nonzero only when the queried reference is a verified PA actor.
        // Furniture reservations do not imply a wearer.
        std::uint32_t actorFormId{ 0 };
        RockProviderPowerArmorPointPoseV1 points[2]{};
    };

    struct RockProviderPowerArmorGrabRequestV1
    {
        std::uint32_t size{ sizeof(RockProviderPowerArmorGrabRequestV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        RockProviderReferenceQueryV1 target{};
        api::Hand hand{ api::Hand::None };
        api::PowerArmorPoint point{};
        // Zero uses the native proximity radius. Maximum 32 game units.
        float maxDistanceGame{ 0.0f };
    };


}
