#pragma once

#include <cstdint>
#include <string_view>

namespace rock::authored_weapon_grip_capture_policy
{
    // These values intentionally match the public ROCK V1 native-animation
    // authority mask. Authored grip application must yield whenever an addon
    // owns any overlapping hand/weapon animation authority.
    inline constexpr std::uint32_t kArms = 1u << 0;
    inline constexpr std::uint32_t kHands = 1u << 1;
    inline constexpr std::uint32_t kWeapon = 1u << 2;
    inline constexpr std::uint16_t
        kCompleteAuthoredSupportFingerLocalTransformMask = 0x7FFFu;

    struct AuthoredPrimaryFiringGripEligibility
    {
        bool runtimeInitialized{ false };
        bool visualAuthorityAvailable{ false };
        bool localSkeletonReady{ false };
        bool menuBlocking{ false };
        bool compatibilityBlocking{ false };
        bool weaponDrawn{ false };
        bool weaponVisible{ false };
        bool equippedWeaponTransitionActive{ false };
        bool weaponKeyValid{ false };
        bool captureValid{ false };
        bool captureNewerThanWeaponBoundary{ false };
        bool nativeReloadAuthorityActive{ false };
        bool conflictingWeaponTransformAuthorityActive{ false };
        bool weaponVisualReturnActive{ false };
        bool primaryHandHoldingObject{ false };
        bool rockFiringHandIsLeft{ false };
    };

    enum class AuthoredPrimaryAction : std::uint8_t
    {
        Clear,
        Apply,
        RetainPoseOnly,
    };

    enum class AuthoredPrimaryDecisionReason : std::uint8_t
    {
        Apply,
        WeaponVisualReturn,
        EquipTransitionContinuity,
        RuntimeUnavailable,
        VisualAuthorityUnavailable,
        SkeletonUnavailable,
        MenuBlocking,
        CompatibilityBlocking,
        WeaponKeyInvalid,
        NativeReloadAuthority,
        PrimaryHandHoldingObject,
        PhysicalLeftFiring,
        WeaponNotDrawn,
        WeaponNotVisible,
        CaptureUnavailable,
        CaptureNotFresh,
        ConflictingWeaponAuthority,
    };

    struct AuthoredPrimaryDecision
    {
        AuthoredPrimaryAction action{ AuthoredPrimaryAction::Clear };
        AuthoredPrimaryDecisionReason reason{
            AuthoredPrimaryDecisionReason::RuntimeUnavailable
        };
    };

    struct AuthoredSupportGripCandidateInput
    {
        bool interactionAcquisitionValid{ false };
        bool activationZoneValid{ false };
        bool authoredPoseSurfaceEvidenceValid{ false };
        bool providerAuthorityActive{ false };
        bool attachOnly{ false };
        bool captureValid{ false };
        bool weaponIdentityMatches{ false };
        bool generationMatches{ false };
        bool authoredSeatWeaponSurfaceValid{ false };
        bool completeFingerPose{ false };
    };

    struct AuthoredFiringGripProbeInput
    {
        bool proximityProbeAcquisition{ false };
        bool providerAuthorityActive{ false };
        bool attachOnly{ false };
        bool authoredCanonicalAvailable{ false };
    };

    struct StableAuthoredSupportGripReuseInput
    {
        bool snapshotValid{ false };
        bool weaponNodeValid{ false };
        bool weaponNodeMatches{ false };
        std::uint64_t currentWeaponOwnershipKey{ 0 };
        std::uint64_t snapshotWeaponOwnershipKey{ 0 };
        std::uint64_t currentWeaponGenerationKey{ 0 };
        std::uint64_t snapshotWeaponGenerationKey{ 0 };
        std::uint64_t currentPrimaryGripCaptureSequence{ 0 };
        std::uint64_t snapshotPrimaryGripCaptureSequence{ 0 };
        std::uint64_t snapshotSupportGripCaptureSequence{ 0 };
        std::uint16_t snapshotFingerLocalTransformMask{ 0 };
    };

    [[nodiscard]] constexpr AuthoredPrimaryDecision
        evaluateAuthoredPrimaryFiringGrip(
            const AuthoredPrimaryFiringGripEligibility& input)
    {
        const bool commonReady =
            input.runtimeInitialized &&
            input.visualAuthorityAvailable &&
            input.localSkeletonReady &&
            !input.menuBlocking &&
            !input.compatibilityBlocking &&
            input.weaponKeyValid &&
            !input.nativeReloadAuthorityActive &&
            !input.primaryHandHoldingObject &&
            !input.rockFiringHandIsLeft;
        const bool apply =
            commonReady &&
            input.weaponDrawn &&
            (input.weaponVisible ||
                input.equippedWeaponTransitionActive) &&
            input.captureValid &&
            input.captureNewerThanWeaponBoundary &&
            !input.conflictingWeaponTransformAuthorityActive &&
            !input.weaponVisualReturnActive;
        if (apply) {
            return {
                .action = AuthoredPrimaryAction::Apply,
                .reason = AuthoredPrimaryDecisionReason::Apply,
            };
        }

        if (commonReady &&
            (input.weaponVisualReturnActive ||
                input.equippedWeaponTransitionActive)) {
            return {
                .action = AuthoredPrimaryAction::RetainPoseOnly,
                .reason = input.weaponVisualReturnActive ?
                    AuthoredPrimaryDecisionReason::WeaponVisualReturn :
                    AuthoredPrimaryDecisionReason::EquipTransitionContinuity,
            };
        }

        if (!input.runtimeInitialized) {
            return { .reason = AuthoredPrimaryDecisionReason::RuntimeUnavailable };
        }
        if (!input.visualAuthorityAvailable) {
            return { .reason = AuthoredPrimaryDecisionReason::VisualAuthorityUnavailable };
        }
        if (!input.localSkeletonReady) {
            return { .reason = AuthoredPrimaryDecisionReason::SkeletonUnavailable };
        }
        if (input.menuBlocking) {
            return { .reason = AuthoredPrimaryDecisionReason::MenuBlocking };
        }
        if (input.compatibilityBlocking) {
            return { .reason = AuthoredPrimaryDecisionReason::CompatibilityBlocking };
        }
        if (!input.weaponKeyValid) {
            return { .reason = AuthoredPrimaryDecisionReason::WeaponKeyInvalid };
        }
        if (input.nativeReloadAuthorityActive) {
            return { .reason = AuthoredPrimaryDecisionReason::NativeReloadAuthority };
        }
        if (input.primaryHandHoldingObject) {
            return { .reason = AuthoredPrimaryDecisionReason::PrimaryHandHoldingObject };
        }
        if (input.rockFiringHandIsLeft) {
            return { .reason = AuthoredPrimaryDecisionReason::PhysicalLeftFiring };
        }
        if (!input.weaponDrawn) {
            return { .reason = AuthoredPrimaryDecisionReason::WeaponNotDrawn };
        }
        if (!input.weaponVisible) {
            return { .reason = AuthoredPrimaryDecisionReason::WeaponNotVisible };
        }
        if (!input.captureValid) {
            return { .reason = AuthoredPrimaryDecisionReason::CaptureUnavailable };
        }
        if (!input.captureNewerThanWeaponBoundary) {
            return { .reason = AuthoredPrimaryDecisionReason::CaptureNotFresh };
        }
        return {
            .reason =
                AuthoredPrimaryDecisionReason::ConflictingWeaponAuthority,
        };
    }

    [[nodiscard]] constexpr const char*
        authoredPrimaryDecisionReasonName(
            const AuthoredPrimaryDecisionReason reason) noexcept
    {
        switch (reason) {
        case AuthoredPrimaryDecisionReason::Apply:
            return "apply";
        case AuthoredPrimaryDecisionReason::WeaponVisualReturn:
            return "weapon-visual-return";
        case AuthoredPrimaryDecisionReason::EquipTransitionContinuity:
            return "equip-transition-continuity";
        case AuthoredPrimaryDecisionReason::RuntimeUnavailable:
            return "runtime-unavailable";
        case AuthoredPrimaryDecisionReason::VisualAuthorityUnavailable:
            return "visual-authority-unavailable";
        case AuthoredPrimaryDecisionReason::SkeletonUnavailable:
            return "skeleton-unavailable";
        case AuthoredPrimaryDecisionReason::MenuBlocking:
            return "menu-blocking";
        case AuthoredPrimaryDecisionReason::CompatibilityBlocking:
            return "compatibility-blocking";
        case AuthoredPrimaryDecisionReason::WeaponKeyInvalid:
            return "weapon-key-invalid";
        case AuthoredPrimaryDecisionReason::NativeReloadAuthority:
            return "native-reload-authority";
        case AuthoredPrimaryDecisionReason::PrimaryHandHoldingObject:
            return "primary-hand-holding-object";
        case AuthoredPrimaryDecisionReason::PhysicalLeftFiring:
            return "physical-left-firing";
        case AuthoredPrimaryDecisionReason::WeaponNotDrawn:
            return "weapon-not-drawn";
        case AuthoredPrimaryDecisionReason::WeaponNotVisible:
            return "weapon-not-visible";
        case AuthoredPrimaryDecisionReason::CaptureUnavailable:
            return "capture-unavailable";
        case AuthoredPrimaryDecisionReason::CaptureNotFresh:
            return "capture-not-fresh";
        case AuthoredPrimaryDecisionReason::ConflictingWeaponAuthority:
            return "conflicting-weapon-authority";
        }
        return "unknown";
    }

    /*
     * A ROCK object grab owns the occupied physical hand's fingers. The
     * equipped weapon may keep its independent transform/carry authority,
     * but its persistent authored firing pose must not compete with the
     * ROCK_Grab pose for that same hand.
     */
    [[nodiscard]] constexpr bool shouldPublishAuthoredFiringFingerPose(
        const bool targetHandHoldingObject) noexcept
    {
        return !targetHandHoldingObject;
    }

    [[nodiscard]] constexpr bool shouldUseAuthoredSupportGrip(
        const AuthoredSupportGripCandidateInput& input)
    {
        return input.interactionAcquisitionValid &&
               input.activationZoneValid &&
               input.authoredPoseSurfaceEvidenceValid &&
               !input.providerAuthorityActive &&
               !input.attachOnly &&
               input.captureValid &&
               input.weaponIdentityMatches &&
               input.generationMatches &&
               input.authoredSeatWeaponSurfaceValid &&
               input.completeFingerPose;
    }

    [[nodiscard]] constexpr bool shouldUseAuthoredFiringGripProbe(
        const AuthoredFiringGripProbeInput& input)
    {
        return input.proximityProbeAcquisition &&
               !input.providerAuthorityActive &&
               !input.attachOnly &&
               input.authoredCanonicalAvailable;
    }

    /*
     * Bethesda may omit the paired support-arm pass during transient firing
     * animation frames. Reuse is safe only for the last fully validated pose
     * while every equipped-weapon and canonical identity witness still
     * matches; reload and weapon boundaries clear the snapshot separately.
     */
    [[nodiscard]] constexpr bool shouldReuseStableAuthoredSupportGrip(
        const StableAuthoredSupportGripReuseInput& input) noexcept
    {
        return input.snapshotValid &&
               input.weaponNodeValid &&
               input.weaponNodeMatches &&
               input.currentWeaponOwnershipKey != 0 &&
               input.currentWeaponOwnershipKey ==
                   input.snapshotWeaponOwnershipKey &&
               input.currentWeaponGenerationKey != 0 &&
               input.currentWeaponGenerationKey ==
                   input.snapshotWeaponGenerationKey &&
               input.currentPrimaryGripCaptureSequence != 0 &&
               input.currentPrimaryGripCaptureSequence ==
                   input.snapshotPrimaryGripCaptureSequence &&
               input.snapshotSupportGripCaptureSequence != 0 &&
               input.snapshotFingerLocalTransformMask ==
                   kCompleteAuthoredSupportFingerLocalTransformMask;
    }

    template <class Transform, class Point, class LocalPointToWorld>
    [[nodiscard]] constexpr Transform
        resolveAuthoredPrimaryWeaponWorldPositionOnly(
            const Transform& nativeWeaponWorld,
            const Point& authoredGripWeaponLocal,
            const Point& trackedPalmWorld,
            LocalPointToWorld&& localPointToWorld)
    {
        const Point currentAuthoredGripWorld = localPointToWorld(
            nativeWeaponWorld,
            authoredGripWeaponLocal);
        Transform solvedWeaponWorld = nativeWeaponWorld;
        solvedWeaponWorld.translate.x +=
            trackedPalmWorld.x - currentAuthoredGripWorld.x;
        solvedWeaponWorld.translate.y +=
            trackedPalmWorld.y - currentAuthoredGripWorld.y;
        solvedWeaponWorld.translate.z +=
            trackedPalmWorld.z - currentAuthoredGripWorld.z;
        return solvedWeaponWorld;
    }

    /*
     * The animation relation is authored in the equipped Weapon frame, while
     * a loose reference is driven by its model root. Rebase through P-Grip,
     * which is the shared rigid frame present in both graphs, so wrapper-local
     * rotation never leaks into the loose hold.
     */
    template <class Transform, class Compose, class Invert>
    [[nodiscard]] constexpr Transform rebaseRelationThroughSharedGripAnchor(
        const Transform& sourceGripAnchorRootLocal,
        const Transform& targetGripAnchorRootLocal,
        const Transform& relationSourceRootLocal,
        Compose&& compose,
        Invert&& invert)
    {
        return compose(
            targetGripAnchorRootLocal,
            compose(
                invert(sourceGripAnchorRootLocal),
                relationSourceRootLocal));
    }

    template <class Transform, class Compose, class Invert>
    [[nodiscard]] constexpr Transform resolveRootWorldFromSharedGripAnchor(
        const Transform& targetGripAnchorWorld,
        const Transform& sourceGripAnchorRootLocal,
        Compose&& compose,
        Invert&& invert)
    {
        return compose(
            targetGripAnchorWorld,
            invert(sourceGripAnchorRootLocal));
    }

    template <class Transform, class Compose, class Invert>
    [[nodiscard]] constexpr Transform resolveAuthoredSupportHandInPrimaryHand(
        const Transform& authoredPrimaryHandModel,
        const Transform& authoredSupportHandModel,
        Compose&& compose,
        Invert&& invert)
    {
        return compose(
            invert(authoredPrimaryHandModel),
            authoredSupportHandModel);
    }

    template <class Transform, class Compose>
    [[nodiscard]] constexpr Transform resolveAuthoredSupportHandInWeapon(
        const Transform& primaryHandInWeapon,
        const Transform& supportHandInPrimaryHand,
        Compose&& compose)
    {
        return compose(primaryHandInWeapon, supportHandInPrimaryHand);
    }

    [[nodiscard]] constexpr char asciiLower(const char value)
    {
        return value >= 'A' && value <= 'Z' ?
            static_cast<char>(value - 'A' + 'a') :
            value;
    }

    [[nodiscard]] constexpr bool equalsIgnoreCase(
        const std::string_view lhs,
        const std::string_view rhs)
    {
        if (lhs.size() != rhs.size()) {
            return false;
        }
        for (std::size_t index = 0; index < lhs.size(); ++index) {
            if (asciiLower(lhs[index]) != asciiLower(rhs[index])) {
                return false;
            }
        }
        return true;
    }
}
