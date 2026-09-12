#include "physics-interaction/weapon/NativeEquippedWeaponDraw.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/weapon/HeldWeaponEquipStatePolicy.h"
#include "physics-interaction/weapon/WeaponTransitionAnimationAcceleration.h"
#include "rock_support/Fo4VrRuntime.h"

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/PlayerCharacter.h"

#include <array>
#include <cmath>
#include <cstddef>

namespace rock::native_equipped_weapon_draw
{
    static_assert(
        offsetof(RE::PlayerCharacter, drawSheatheSafetyTimer) == 0x1130);

    namespace
    {
        constexpr std::array<std::uint8_t, 7> kExpectedPrepareDrawEntry{
            0x80, 0x89, 0xA3, 0x12, 0x00, 0x00, 0x80,
        };
        constexpr std::array<std::uint8_t, 10> kExpectedApplyAcceptedDrawEntry{
            0x48, 0x89, 0x5C, 0x24, 0x08,
            0x57,
            0x48, 0x83, 0xEC, 0x20,
        };
        constexpr std::array<std::uint8_t, 10> kExpectedRefreshEquipmentEntry{
            0x53,
            0x55,
            0x56,
            0x48, 0x81, 0xEC, 0xA0, 0x00, 0x00, 0x00,
        };

        struct ExactCurrent
        {
            RE::PlayerCharacter* player{ nullptr };
            SubmitResult result{ SubmitResult::MissingPlayer };
        };

        struct PartialActionWitness
        {
            Identity identity{};
            weapon_transition_animation_acceleration::ActivationEvidence
                evidenceAtReturn{};
            bool active{ false };
        };

        // Game-thread only. The animation thread publishes evidence through
        // atomics; this witness binds synchronous activation to one exact
        // rejected submission before a later frame may complete it.
        PartialActionWitness s_partialActionWitness{};

        [[nodiscard]] bool prepareDrawEntryMatchesVerifiedRuntime() noexcept
        {
            static const bool matches = []() noexcept {
                if (!REL::Module::IsVR() ||
                    REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
                    return false;
                }

                std::array<std::uint8_t, kExpectedPrepareDrawEntry.size()> actual{};
                const auto address = REL::Offset(
                    offsets::kFunc_PrepareEquippedWeaponDraw).address();
                return native_memory::guardedCopyFromMemory(
                           reinterpret_cast<const void*>(address),
                           actual.data(),
                           actual.size()) &&
                       actual == kExpectedPrepareDrawEntry;
            }();
            return matches;
        }

        template <std::size_t N>
        [[nodiscard]] bool entryMatches(
            const std::uintptr_t offset,
            const std::array<std::uint8_t, N>& expected) noexcept
        {
            std::array<std::uint8_t, N> actual{};
            const auto address = REL::Offset(offset).address();
            return native_memory::guardedCopyFromMemory(
                       reinterpret_cast<const void*>(address),
                       actual.data(),
                       actual.size()) &&
                   actual == expected;
        }

        [[nodiscard]] bool partialDrawCompletionMatchesVerifiedRuntime() noexcept
        {
            static const bool matches = []() noexcept {
                return REL::Module::IsVR() &&
                       REL::Module::get().version() ==
                           F4SE::RUNTIME_VR_1_2_72 &&
                       entryMatches(
                           offsets::kFunc_ApplyAcceptedWeaponDraw,
                           kExpectedApplyAcceptedDrawEntry) &&
                       entryMatches(
                           offsets::kFunc_RefreshActorEquipmentAfterAction,
                           kExpectedRefreshEquipmentEntry);
            }();
            return matches;
        }

        [[nodiscard]] ExactCurrent resolveExactCurrent(
            const Identity& expected) noexcept
        {
            ExactCurrent current{};
            current.player = f4vr::getPlayer();
            if (!current.player) {
                return current;
            }

            Identity observed{};
            if (!captureCurrentIdentity(observed)) {
                current.result = SubmitResult::MissingEquippedWeapon;
                return current;
            }
            if (observed.formID != expected.formID ||
                observed.instanceData != expected.instanceData ||
                observed.equipIndex != expected.equipIndex) {
                current.result = SubmitResult::IdentityChanged;
                return current;
            }

            current.result = SubmitResult::Submitted;
            return current;
        }

        void requestAnimationAcceleration(
            RE::PlayerCharacter* player,
            const Identity& identity,
            const weapon_transition_animation_acceleration_policy::Direction
                direction) noexcept
        {
            (void)weapon_transition_animation_acceleration::request(
                weapon_transition_animation_acceleration::RequestInput{
                    .player = player,
                    .identity = {
                        .formID = identity.formID,
                        .instanceData = identity.instanceData,
                        .equipIndex = identity.equipIndex,
                    },
                    .direction = direction,
                });
        }

        [[nodiscard]] weapon_transition_animation_acceleration::
            ActivationEvidence observeDrawActivationEvidence(
                RE::PlayerCharacter* player,
                const Identity& identity) noexcept
        {
            return weapon_transition_animation_acceleration::
                observeExactLeaseActivation(
                    player,
                    weapon_transition_animation_acceleration::Identity{
                        .formID = identity.formID,
                        .instanceData = identity.instanceData,
                        .equipIndex = identity.equipIndex,
                    },
                    weapon_transition_animation_acceleration_policy::
                        Direction::Draw);
        }

        void copyEvidence(
            Result& result,
            const weapon_transition_animation_acceleration::
                ActivationEvidence& evidence) noexcept
        {
            result.evidenceSequence = evidence.sequence;
            result.matchedActivations = evidence.matchedActivations;
            result.registeredUpdates = evidence.registeredUpdates;
            result.activeClips = evidence.activeClips;
            result.updatedActiveClips = evidence.updatedActiveClips;
        }

        void recordRejectedSubmissionWitness(
            const Identity& identity,
            const weapon_transition_animation_acceleration::
                ActivationEvidence& before,
            const weapon_transition_animation_acceleration::
                ActivationEvidence& after) noexcept
        {
            weapon_transition_animation_acceleration::ActivationEvidence
                submissionEvidence = after;
            submissionEvidence.activeClips = 0;
            submissionEvidence.updatedActiveClips = 0;
            submissionEvidence.activeClipIdentities.fill(0);
            submissionEvidence.updatedActiveClipIdentities.fill(0);
            for (const auto submittedClip : after.activeClipIdentities) {
                if (submittedClip == 0) {
                    continue;
                }
                bool existedBeforeSubmission = false;
                for (const auto priorClip : before.activeClipIdentities) {
                    if (submittedClip == priorClip) {
                        existedBeforeSubmission = true;
                        break;
                    }
                }
                if (!existedBeforeSubmission &&
                    submissionEvidence.activeClips <
                        submissionEvidence.activeClipIdentities.size()) {
                    submissionEvidence.activeClipIdentities
                        [submissionEvidence.activeClips++] = submittedClip;
                }
            }

            if (before.exactLease && after.exactLease &&
                before.sequence != 0 && before.sequence == after.sequence &&
                after.matchedActivations > before.matchedActivations &&
                submissionEvidence.activeClips > 0) {
                s_partialActionWitness = PartialActionWitness{
                    .identity = identity,
                    .evidenceAtReturn = submissionEvidence,
                    .active = true,
                };
                return;
            }

            if (s_partialActionWitness.identity == identity) {
                s_partialActionWitness = {};
            }
        }

        [[nodiscard]] bool partialActionWitnessProvesAcceptance(
            const Identity& identity,
            const weapon_transition_animation_acceleration::
                ActivationEvidence& evidence) noexcept
        {
            return s_partialActionWitness.active &&
                   s_partialActionWitness.identity == identity &&
                   evidence.exactLease && evidence.sequence != 0 &&
                   evidence.sequence ==
                       s_partialActionWitness.evidenceAtReturn.sequence &&
                   evidence.matchedActivations >=
                       s_partialActionWitness.evidenceAtReturn.
                           matchedActivations &&
                   evidence.hasActiveUpdatedClips() &&
                   evidence.hasUpdatedClipFrom(
                       s_partialActionWitness.evidenceAtReturn);
        }

        [[nodiscard]] Result recoverPartiallyAcceptedDraw(
            const ExactCurrent& current,
            const Identity& expected,
            const weapon_transition_animation_acceleration::
                ActivationEvidence& evidence) noexcept
        {
            Result result{};
            result.result = current.result;
            copyEvidence(result, evidence);
            if (current.result != SubmitResult::Submitted) {
                return result;
            }

            result.stateBefore = f4vr::getNativeWeaponState(current.player);
            result.stateAfter = result.stateBefore;
            if (result.stateBefore != static_cast<std::uint32_t>(
                    held_weapon_equip_state_policy::NativeWeaponState::
                        Sheathed) &&
                result.stateBefore != static_cast<std::uint32_t>(
                    held_weapon_equip_state_policy::NativeWeaponState::
                        WantToDraw)) {
                result.result = SubmitResult::RecoveryStateChanged;
                return result;
            }
            if (!partialActionWitnessProvesAcceptance(expected, evidence)) {
                result.result = SubmitResult::NativeActionRejected;
                return result;
            }
            if (!partialDrawCompletionMatchesVerifiedRuntime()) {
                result.result = SubmitResult::PartialRecoveryUnavailable;
                return result;
            }

            s_partialActionWitness = {};

            float drawSheatheSafetyTimer = 0.0f;
            if (!native_memory::tryReadValue(
                    reinterpret_cast<const float*>(
                        REL::Offset(
                            offsets::kData_DrawSheatheSafetyTimer)
                            .address()),
                    drawSheatheSafetyTimer) ||
                !std::isfinite(drawSheatheSafetyTimer)) {
                result.result = SubmitResult::PartialRecoveryUnavailable;
                return result;
            }

            const bool stateAccepted = current.player->SetWeaponState(
                RE::WEAPON_STATE::kDrawing);
            result.stateAfter = f4vr::getNativeWeaponState(current.player);
            if (!stateAccepted ||
                (result.stateAfter != static_cast<std::uint32_t>(
                     held_weapon_equip_state_policy::NativeWeaponState::
                         Drawing) &&
                    result.stateAfter != static_cast<std::uint32_t>(
                        held_weapon_equip_state_policy::NativeWeaponState::
                            Drawn))) {
                result.result = SubmitResult::PartialRecoveryRejected;
                return result;
            }
            current.player->drawSheatheSafetyTimer =
                drawSheatheSafetyTimer;

            using ApplyAcceptedWeaponDraw = void (*)(
                RE::PlayerCharacter*, bool);
            const auto apply = reinterpret_cast<ApplyAcceptedWeaponDraw>(
                REL::Offset(
                    offsets::kFunc_ApplyAcceptedWeaponDraw).address());
            apply(current.player, false);

            using RefreshActorEquipmentAfterAction = void (*)(
                RE::PlayerCharacter*,
                std::uintptr_t,
                std::uintptr_t,
                std::uintptr_t);
            const auto refresh =
                reinterpret_cast<RefreshActorEquipmentAfterAction>(
                    REL::Offset(
                        offsets::kFunc_RefreshActorEquipmentAfterAction)
                        .address());
            refresh(current.player, 0, 0, 0);

            Identity observed{};
            if (!captureCurrentIdentity(observed) ||
                observed.formID != expected.formID ||
                observed.instanceData != expected.instanceData ||
                observed.equipIndex != expected.equipIndex) {
                result.result = SubmitResult::IdentityChanged;
                return result;
            }
            result.stateAfter = f4vr::getNativeWeaponState(current.player);
            if (result.stateAfter != static_cast<std::uint32_t>(
                    held_weapon_equip_state_policy::NativeWeaponState::
                        Drawing) &&
                result.stateAfter != static_cast<std::uint32_t>(
                    held_weapon_equip_state_policy::NativeWeaponState::
                        Drawn)) {
                result.result = SubmitResult::PartialRecoveryRejected;
                return result;
            }
            result.result = SubmitResult::PartialActionRecovered;
            return result;
        }

        [[nodiscard]] Result submitResolvedDraw(
            const ExactCurrent& current,
            const Identity& expected,
            const bool prepareEquipRecovery) noexcept
        {
            Result result{};
            result.result = current.result;
            if (current.result != SubmitResult::Submitted) {
                return result;
            }

            result.stateBefore = f4vr::getNativeWeaponState(current.player);
            result.stateAfter = result.stateBefore;
            if (!held_weapon_equip_state_policy::isValidNativeWeaponState(
                    result.stateBefore)) {
                result.result = SubmitResult::InvalidWeaponState;
                return result;
            }
            if (!held_weapon_equip_state_policy::shouldSubmitDrawFollowup(
                    result.stateBefore)) {
                if (result.stateBefore == static_cast<std::uint32_t>(
                        held_weapon_equip_state_policy::NativeWeaponState::
                            Drawing)) {
                    requestAnimationAcceleration(
                        current.player,
                        expected,
                        weapon_transition_animation_acceleration_policy::
                            Direction::Draw);
                }
                result.result = SubmitResult::AlreadyDrawingOrDrawn;
                return result;
            }

            if (result.stateBefore == static_cast<std::uint32_t>(
                    held_weapon_equip_state_policy::NativeWeaponState::
                        Sheathed) ||
                result.stateBefore == static_cast<std::uint32_t>(
                    held_weapon_equip_state_policy::NativeWeaponState::
                        WantToDraw)) {
                const auto priorEvidence =
                    observeDrawActivationEvidence(current.player, expected);
                if (partialActionWitnessProvesAcceptance(
                        expected,
                        priorEvidence)) {
                    return recoverPartiallyAcceptedDraw(
                        current,
                        expected,
                        priorEvidence);
                }
            }

            if (prepareEquipRecovery) {
                if (result.stateBefore != static_cast<std::uint32_t>(
                        held_weapon_equip_state_policy::NativeWeaponState::
                            Sheathed)) {
                    result.result = SubmitResult::RecoveryStateChanged;
                    return result;
                }
                if (!prepareDrawEntryMatchesVerifiedRuntime()) {
                    result.result =
                        SubmitResult::RecoveryPreparationUnavailable;
                    return result;
                }

                using PrepareEquippedWeaponDraw = void (*)(
                    RE::PlayerCharacter*);
                const auto prepare = reinterpret_cast<PrepareEquippedWeaponDraw>(
                    REL::Offset(
                        offsets::kFunc_PrepareEquippedWeaponDraw).address());
                prepare(current.player);
            }

            requestAnimationAcceleration(
                current.player,
                expected,
                weapon_transition_animation_acceleration_policy::Direction::
                    Draw);
            const auto evidenceBefore =
                observeDrawActivationEvidence(current.player, expected);
            current.player->DrawWeaponMagicHands(true);
            result.stateAfter = f4vr::getNativeWeaponState(current.player);
            const auto evidenceAfter =
                observeDrawActivationEvidence(current.player, expected);
            copyEvidence(
                result,
                evidenceAfter);
            /*
             * FO4VR 1.2.72 at 0x140F78D10 changes the native weapon state
             * synchronously whenever ActionDraw is accepted. An unchanged
             * retryable state therefore means the action was rejected; the
             * void CommonLib boundary must not be reported as success.
             */
            result.result = result.stateAfter == result.stateBefore ?
                SubmitResult::NativeActionRejected :
                SubmitResult::Submitted;
            if (result.result == SubmitResult::NativeActionRejected) {
                recordRejectedSubmissionWitness(
                    expected,
                    evidenceBefore,
                    evidenceAfter);
            } else if (s_partialActionWitness.identity == expected) {
                s_partialActionWitness = {};
            }
            return result;
        }
    }

    bool captureCurrentIdentity(Identity& outIdentity) noexcept
    {
        outIdentity = {};
        auto* equipped = f4vr::getEquippedWeaponItem();
        auto* object = equipped ? equipped->item.object : nullptr;
        auto* instanceData = equipped ? equipped->item.instanceData.get() : nullptr;
        if (!object || object->formType != RE::ENUM_FORM_ID::kWEAP) {
            return false;
        }

        outIdentity = Identity{
            .formID = object->formID,
            .instanceData = reinterpret_cast<std::uintptr_t>(instanceData),
            .equipIndex = equipped->equipIndex.index,
        };
        return true;
    }

    Result submitExactCurrent(const Identity& expected) noexcept
    {
        const auto current = resolveExactCurrent(expected);
        return submitResolvedDraw(current, expected, false);
    }

    Result submitPreparedExactCurrent(const Identity& expected) noexcept
    {
        const auto current = resolveExactCurrent(expected);
        return submitResolvedDraw(current, expected, true);
    }

    Result finalizePartialExactCurrent(const Identity& expected) noexcept
    {
        Result result{};
        s_partialActionWitness = {};
        const auto current = resolveExactCurrent(expected);
        result.result = current.result;
        if (current.result != SubmitResult::Submitted) {
            return result;
        }

        result.stateBefore = f4vr::getNativeWeaponState(current.player);
        result.stateAfter = result.stateBefore;
        if (result.stateBefore != static_cast<std::uint32_t>(
                held_weapon_equip_state_policy::NativeWeaponState::Drawing)) {
            result.result = SubmitResult::RecoveryStateChanged;
            return result;
        }

        const bool stateAccepted = current.player->SetWeaponState(
            RE::WEAPON_STATE::kDrawn);
        result.stateAfter = f4vr::getNativeWeaponState(current.player);
        result.result = stateAccepted &&
                result.stateAfter == static_cast<std::uint32_t>(
                    held_weapon_equip_state_policy::NativeWeaponState::Drawn) ?
            SubmitResult::PartialActionFinalized :
            SubmitResult::PartialRecoveryRejected;
        if (result.result == SubmitResult::PartialRecoveryRejected) {
            (void)current.player->SetWeaponState(
                RE::WEAPON_STATE::kSheathed);
            result.stateAfter = f4vr::getNativeWeaponState(current.player);
        }
        return result;
    }

    Result submitSheatheExactCurrent(const Identity& expected) noexcept
    {
        Result result{};
        s_partialActionWitness = {};
        const auto current = resolveExactCurrent(expected);
        result.result = current.result;
        if (current.result != SubmitResult::Submitted) {
            return result;
        }

        result.stateBefore = f4vr::getNativeWeaponState(current.player);
        result.stateAfter = result.stateBefore;
        if (!held_weapon_equip_state_policy::isValidNativeWeaponState(result.stateBefore)) {
            result.result = SubmitResult::InvalidWeaponState;
            return result;
        }
        if (!held_weapon_equip_state_policy::shouldSubmitSheatheFollowup(result.stateBefore)) {
            if (result.stateBefore == static_cast<std::uint32_t>(
                    held_weapon_equip_state_policy::NativeWeaponState::
                        Sheathing)) {
                requestAnimationAcceleration(
                    current.player,
                    expected,
                    weapon_transition_animation_acceleration_policy::
                        Direction::Sheathe);
            }
            result.result = SubmitResult::AlreadySheathingOrSheathed;
            return result;
        }

        requestAnimationAcceleration(
            current.player,
            expected,
            weapon_transition_animation_acceleration_policy::
                Direction::Sheathe);
        current.player->DrawWeaponMagicHands(false);
        result.stateAfter = f4vr::getNativeWeaponState(current.player);
        result.result = result.stateAfter == result.stateBefore ?
            SubmitResult::NativeActionRejected :
            SubmitResult::Submitted;
        return result;
    }

    const char* submitResultName(const SubmitResult result) noexcept
    {
        switch (result) {
        case SubmitResult::Submitted:
            return "submitted";
        case SubmitResult::AlreadyDrawingOrDrawn:
            return "already-drawing-or-drawn";
        case SubmitResult::AlreadySheathingOrSheathed:
            return "already-sheathing-or-sheathed";
        case SubmitResult::MissingPlayer:
            return "missing-player";
        case SubmitResult::MissingEquippedWeapon:
            return "missing-equipped-weapon";
        case SubmitResult::IdentityChanged:
            return "identity-changed";
        case SubmitResult::InvalidWeaponState:
            return "invalid-weapon-state";
        case SubmitResult::RecoveryPreparationUnavailable:
            return "recovery-preparation-unavailable";
        case SubmitResult::RecoveryStateChanged:
            return "recovery-state-changed";
        case SubmitResult::NativeActionRejected:
            return "native-action-rejected";
        case SubmitResult::PartialRecoveryUnavailable:
            return "partial-recovery-unavailable";
        case SubmitResult::PartialRecoveryRejected:
            return "partial-recovery-rejected";
        case SubmitResult::PartialActionRecovered:
            return "partial-action-recovered";
        case SubmitResult::PartialActionFinalized:
            return "partial-action-finalized";
        default:
            return "unknown";
        }
    }
}
