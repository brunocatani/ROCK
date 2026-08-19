#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/core/PhysicsInteractionTransformValidation.h"

#include "api/ROCKProviderApiInternal.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string_view>

#include "physics-interaction/feedback/FeedbackHaptics.h"
#include "physics-interaction/feedback/HapticPolicy.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/stash/ShoulderStashDetector.h"
#include "physics-interaction/stash/ShoulderStashMath.h"
#include "physics-interaction/stash/ShoulderStashPolicy.h"
#include "physics-interaction/stash/ShoulderStashTransfer.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "physics-interaction/weapon/equip/EquippedWeaponDropPolicy.h"
#include "physics-interaction/weapon/equip/EquippedWeaponHandlingRuntime.h"
#include "physics-interaction/weapon/equip/HeldWeaponEquipStatePolicy.h"
#include "physics-interaction/weapon/equip/PipboyEquipRuntime.h"
#include "physics-interaction/weapon/equip/WeaponEquipTransfer.h"
#include "physics-interaction/weapon/native_anim/NativeEquippedWeaponDraw.h"

#include "RockConfig.h"
#include "RockUtils.h"

namespace rock
{
    using namespace physics_interaction_detail;

    namespace
    {
        constexpr std::uint16_t
            kEquippedWeaponHandAssignmentMaximumResolveFrames = 180;

        bool approximatelySameWeaponLocalOffset(
            const RE::NiTransform& live,
            const RE::NiTransform& expected)
        {
            constexpr float kMaximumTranslationError = 0.05f;
            constexpr float kMaximumRotationElementError = 0.001f;
            constexpr float kMaximumScaleError = 0.001f;
            if (!finiteNiTransform(live) || !finiteNiTransform(expected)) {
                return false;
            }

            const float dx = live.translate.x - expected.translate.x;
            const float dy = live.translate.y - expected.translate.y;
            const float dz = live.translate.z - expected.translate.z;
            if (dx * dx + dy * dy + dz * dz > kMaximumTranslationError * kMaximumTranslationError ||
                std::abs(live.scale - expected.scale) > kMaximumScaleError) {
                return false;
            }

            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (std::abs(live.rotate.entry[row][column] - expected.rotate.entry[row][column]) >
                        kMaximumRotationElementError) {
                        return false;
                    }
                }
            }
            return true;
        }

        template <class NativeOffsetState>
        bool sampleNativeOffsetReadiness(
            NativeOffsetState& state,
            const RE::NiTransform& liveOffset)
        {
            const bool liveOffsetFinite = finiteNiTransform(liveOffset);
            bool liveOffsetMatches =
                state.nativeOffsetSampleValid &&
                liveOffsetFinite &&
                approximatelySameWeaponLocalOffset(
                    liveOffset,
                    state.nativeOffsetSample);
            if (liveOffsetFinite && !liveOffsetMatches) {
                // Rebase to the live native authority until it stays stable.
                state.nativeOffsetSample = liveOffset;
                state.nativeOffsetSampleValid = true;
                state.matchingNativeOffsetFrames = 0;
                liveOffsetMatches = true;
            }

            return pipboy_equip_policy::advanceNativeOffsetReadiness(
                state.nativeOffsetSampleValid,
                liveOffsetMatches,
                state.matchingNativeOffsetFrames);
        }
    }

    void PhysicsInteraction::refreshEquippedWeaponHandlingSettings()
    {
        ::rock::provider::RockProviderEquippedWeaponHandlingRequestV1 request{};
        const bool externalAuthorityActive =
            ::rock::provider::getEquippedWeaponHandlingAuthorityV1(request);
        const RockEquippedWeaponHandlingBaseline rockBaseline{
            .ambidextrousHandoffEnabled =
                g_rockConfig.rockAmbidextrousFiringGripEnabled,
            .equippedWeaponShoulderStashEnabled =
                g_rockConfig.rockEquippedWeaponShoulderStashEnabled,
            .firingGripProximitySupportRadiusGameUnits =
                g_rockConfig.rockFiringGripProximitySupportRadius,
            .firingGripPromotionRadiusGameUnits =
                g_rockConfig.rockFiringGripPromotionRadius,
            .leftFiringAimYawDegrees =
                g_rockConfig.rockLeftFiringAimYawDegrees,
            .leftFiringAimPitchDegrees =
                g_rockConfig.rockLeftFiringAimPitchDegrees,
            .leftFiringAimOffsetXGameUnits =
                g_rockConfig.rockLeftFiringAimOffsetXGameUnits,
            .leftFiringAimOffsetYGameUnits =
                g_rockConfig.rockLeftFiringAimOffsetYGameUnits,
            .leftFiringAimOffsetZGameUnits =
                g_rockConfig.rockLeftFiringAimOffsetZGameUnits,
        };
        auto settings = makeEquippedWeaponHandlingSettings(
            rockBaseline,
            externalAuthorityActive ? &request : nullptr);

        const bool fixedFiringHandIsLeft = g_rockConfig.rockLeftHandedMode;
        if (fixedFiringHandIsLeft) {
            // The addon does not own ROCK's fixed-hand preference. Persistent
            // left carry needs firing-grip ownership even when an active addon
            // request explicitly disables dynamic ambidextrous handoff.
            settings.firingGripOwnershipEnabled = true;
        }

        if (_equippedWeaponHandlingModeInitialized) {
            if (requiresEquippedWeaponHandlingModeReconcile(
                    _equippedWeaponHandlingSettings,
                    settings,
                    fixedFiringHandIsLeft != _fixedFiringHandIsLeft)) {
                _equippedWeaponHandlingModeReconcilePending = true;
            }
        }

        _equippedWeaponHandlingSettings = settings;
        _fixedFiringHandIsLeft = fixedFiringHandIsLeft;
        _equippedWeaponHandlingModeInitialized = true;
        equipped_weapon_handling_runtime::publish(settings);

        const auto pipboyMode = pipboy_equip_policy::resolveEquipMode(
            settings.externalAuthorityActive &&
                settings.pipboyTriggerHandEquipEnabled,
            fixedFiringHandIsLeft);
        pipboy_equip_runtime::setEquipMode(pipboyMode);
    }

    void PhysicsInteraction::reconcileEquippedWeaponHandlingMode()
    {
        if (!_equippedWeaponHandlingModeReconcilePending) {
            return;
        }

        if (_equippedWeaponHandAssignment.pending ||
            _equippedWeaponHandAssignment.active) {
            clearEquippedWeaponHandAssignment(
                "equipped-weapon-handling-mode-changed",
                true);
        } else {
            pipboy_equip_runtime::AssignmentSnapshot persisted{};
            if (pipboy_equip_runtime::getAssignment(persisted) &&
                persisted.active) {
                pipboy_equip_runtime::clearWeaponAssignment();
            }
            _twoHandedGrip.restoreNativeRightEquippedCarry(
                "equipped-weapon-handling-mode-changed");
        }
        _pendingEquippedWeaponPrimaryOnlyGripStart = {};
        _fixedLeftCarry = {};
        _equippedWeaponHandlingModeReconcilePending = false;
    }

    bool PhysicsInteraction::submitEquippedWeaponShoulderSheath(
        const std::uint32_t observedWeaponFormID,
        const std::uintptr_t observedWeaponInstanceData,
        const equipped_weapon_drop_policy::SourceHand sourceHand,
        const shoulder_stash::Decision& stashDecision,
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey)
    {
        const bool stashHandIsLeft =
            equipped_weapon_drop_policy::isLeft(sourceHand);
        const std::size_t stashHandIndex = stashHandIsLeft ? 1u : 0u;
        native_equipped_weapon_draw::Identity sheathIdentity{};
        const bool identityCaptured =
            native_equipped_weapon_draw::captureCurrentIdentity(
                sheathIdentity);
        const bool identityMatchesObserved =
            identityCaptured &&
            sheathIdentity.formID == observedWeaponFormID &&
            sheathIdentity.instanceData == observedWeaponInstanceData;
        RE::NiTransform leftFiringHandWeaponLocal{};
        RE::NiPoint3 leftFiringGripWeaponLocal{};
        const bool hasLeftFiringGripTransfer =
            identityMatchesObserved &&
            _twoHandedGrip.tryCaptureLeftFiringGripTransfer(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                leftFiringHandWeaponLocal,
                leftFiringGripWeaponLocal);
        native_equipped_weapon_draw::Result sheathResult{};
        sheathResult.result = identityCaptured ?
            native_equipped_weapon_draw::SubmitResult::IdentityChanged :
            native_equipped_weapon_draw::SubmitResult::MissingEquippedWeapon;
        if (identityMatchesObserved) {
            sheathResult = native_equipped_weapon_draw::
                submitSheatheExactCurrent(sheathIdentity);
        }
        const bool sheathAccepted =
            (sheathResult.result ==
                    native_equipped_weapon_draw::SubmitResult::Submitted ||
                sheathResult.result ==
                    native_equipped_weapon_draw::SubmitResult::
                        AlreadySheathingOrSheathed) &&
            held_weapon_equip_state_policy::
                isShoulderStashedPresentationState(
                    sheathResult.stateAfter);
        if (sheathAccepted) {
            clearEquippedWeaponHandAssignment(
                "shoulder-weapon-sheathed",
                true);
            _pendingEquippedWeaponPrimaryOnlyGripStart = {};
            _fixedLeftCarry = {};
            _equippedWeaponShoulderSheath =
                EquippedWeaponShoulderSheathState{
                    .active = true,
                    .stashedByLeftHand = stashHandIsLeft,
                    .weaponFormID = sheathIdentity.formID,
                    .weaponInstanceData = sheathIdentity.instanceData,
                    .equipIndex = sheathIdentity.equipIndex,
                    .zone = stashDecision.zone,
                    .hasLeftFiringGripTransfer =
                        hasLeftFiringGripTransfer,
                    .leftFiringHandWeaponLocal =
                        leftFiringHandWeaponLocal,
                    .leftFiringGripWeaponLocal =
                        leftFiringGripWeaponLocal,
                };
            _equippedWeaponSheathRetrievalStates = {};
            ROCK_LOG_INFO(Weapon,
                "Equipped weapon shoulder sheathed formID={:08X} instance={:#x} equipIndex={} sourceHand={} zone={} confidence={:.2f} leftTransfer={} state={}({})->{}({}) result={}",
                sheathIdentity.formID,
                sheathIdentity.instanceData,
                sheathIdentity.equipIndex,
                equipped_weapon_drop_policy::sourceHandName(sourceHand),
                body_zone::bodyZoneName(stashDecision.zone),
                stashDecision.confidence,
                hasLeftFiringGripTransfer ? "captured" : "unavailable",
                sheathResult.stateBefore,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    sheathResult.stateBefore),
                sheathResult.stateAfter,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    sheathResult.stateAfter),
                native_equipped_weapon_draw::submitResultName(
                    sheathResult.result));
            if (g_rockConfig.rockShoulderStashHapticsEnabled) {
                (void)_feedbackHaptics.queue(
                    stashHandIsLeft ? feedback_haptics::FeedbackHand::Left :
                                      feedback_haptics::FeedbackHand::Right,
                    g_rockConfig.rockShoulderStashCommitHapticDurationSeconds,
                    g_rockConfig.rockShoulderStashCommitHapticIntensity);
            }
            if (g_rockConfig.rockShoulderStashShowCollectedNotifications) {
                f4vr::showNotification(
                    shoulder_stash_notification_policy::
                        formatStowedNotification(
                            shoulderStashItemName(
                                currentEquippedWeaponForm()),
                            sheathIdentity.formID));
            }
        } else {
            ROCK_LOG_WARN(Weapon,
                "Equipped weapon shoulder sheathe failed formID={:08X} instance={:#x} sourceHand={} identityMatch={} state={}({})->{}({}) result={} -- weapon stays equipped and physical drop is suppressed",
                sheathIdentity.formID,
                sheathIdentity.instanceData,
                equipped_weapon_drop_policy::sourceHandName(sourceHand),
                identityMatchesObserved ? "yes" : "no",
                sheathResult.stateBefore,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    sheathResult.stateBefore),
                sheathResult.stateAfter,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    sheathResult.stateAfter),
                native_equipped_weapon_draw::submitResultName(
                    sheathResult.result));
        }

        shoulder_stash::resetRuntime(
            _equippedWeaponStashStates[stashHandIndex]);
        _equippedWeaponStashCommitLeases[stashHandIndex] = {};
        return sheathAccepted;
    }

    void PhysicsInteraction::clearEquippedWeaponShoulderSheath(
        const char* reason)
    {
        if (_equippedWeaponShoulderSheath.active) {
            ROCK_LOG_INFO(
                Weapon,
                "Equipped weapon shoulder sheath cleared reason={} formID={:08X} instance={:#x} zone={}",
                reason ? reason : "unknown",
                _equippedWeaponShoulderSheath.weaponFormID,
                _equippedWeaponShoulderSheath.weaponInstanceData,
                body_zone::bodyZoneName(
                    _equippedWeaponShoulderSheath.zone));
        }
        _equippedWeaponShoulderSheath = {};
        _equippedWeaponSheathRetrievalStates = {};
    }

    void PhysicsInteraction::serviceEquippedWeaponShoulderSheathRetrieval(
        const PhysicsFrameContext& frame,
        const bool handlingEnabled,
        const bool menuInputActive,
        const std::uint32_t observedWeaponFormID,
        const std::uintptr_t observedWeaponInstanceData)
    {
        if (!_equippedWeaponShoulderSheath.active) {
            _equippedWeaponSheathRetrievalStates = {};
            return;
        }
        if (!handlingEnabled) {
            clearEquippedWeaponShoulderSheath(
                "shoulder-stash-authority-lost");
            return;
        }

        native_equipped_weapon_draw::Identity currentIdentity{};
        const bool capturedCurrentIdentity =
            native_equipped_weapon_draw::captureCurrentIdentity(
                currentIdentity);
        const bool identityMatches = capturedCurrentIdentity &&
            currentIdentity.formID ==
                _equippedWeaponShoulderSheath.weaponFormID &&
            currentIdentity.instanceData ==
                _equippedWeaponShoulderSheath.weaponInstanceData &&
            currentIdentity.equipIndex ==
                _equippedWeaponShoulderSheath.equipIndex &&
            observedWeaponFormID == currentIdentity.formID &&
            observedWeaponInstanceData == currentIdentity.instanceData;
        if (!identityMatches) {
            clearEquippedWeaponShoulderSheath(
                "equipped-weapon-identity-changed");
            return;
        }

        auto* player = f4vr::getPlayer();
        const std::uint32_t nativeWeaponState =
            f4vr::getNativeWeaponState(player);
        if (!held_weapon_equip_state_policy::isValidNativeWeaponState(
                nativeWeaponState)) {
            clearEquippedWeaponShoulderSheath(
                "invalid-native-weapon-state");
            return;
        }

        const auto commitRetrieval = [this, &currentIdentity](
                                         const equipped_weapon_drop_policy::
                                             SourceHand retrievalHand,
                                         const float confidence,
                                         const native_equipped_weapon_draw::
                                             Result& drawResult) {
            const bool retrieveWithLeftHand =
                equipped_weapon_drop_policy::isLeft(retrievalHand);
            const std::size_t handIndex =
                retrieveWithLeftHand ? 1u : 0u;
            _pendingEquippedWeaponPrimaryOnlyGripStart =
                PendingEquippedWeaponPrimaryOnlyGripStart{
                    .pending = true,
                    .isLeft = retrieveWithLeftHand,
                    .targetWeaponFormID = currentIdentity.formID,
                    .targetWeaponInstanceData = currentIdentity.instanceData,
                    .remainingSeconds = 10.0f,
                    .committedTransfer = true,
                    .hasFiringHandWeaponLocal = retrieveWithLeftHand &&
                        _equippedWeaponShoulderSheath.
                            hasLeftFiringGripTransfer,
                    .firingHandWeaponLocal =
                        _equippedWeaponShoulderSheath.
                            leftFiringHandWeaponLocal,
                    .hasFiringGripWeaponLocal = retrieveWithLeftHand &&
                        _equippedWeaponShoulderSheath.
                            hasLeftFiringGripTransfer,
                    .firingGripWeaponLocal =
                        _equippedWeaponShoulderSheath.
                            leftFiringGripWeaponLocal,
                };
            _equippedWeaponUnsheathCommittedThisFrame[handIndex] = true;
            if (g_rockConfig.rockShoulderStashHapticsEnabled) {
                (void)_feedbackHaptics.queue(
                    retrieveWithLeftHand ?
                        feedback_haptics::FeedbackHand::Left :
                        feedback_haptics::FeedbackHand::Right,
                    g_rockConfig.
                        rockShoulderStashCommitHapticDurationSeconds,
                    g_rockConfig.
                        rockShoulderStashCommitHapticIntensity);
            }
            ROCK_LOG_INFO(
                Weapon,
                "Equipped weapon shoulder unsheath acknowledged formID={:08X} hand={} zone={} confidence={:.2f} requests={} state={}({})->{}({}) result={}",
                currentIdentity.formID,
                equipped_weapon_drop_policy::sourceHandName(retrievalHand),
                body_zone::bodyZoneName(
                    _equippedWeaponShoulderSheath.zone),
                confidence,
                _equippedWeaponShoulderSheath.drawWait.requestCount,
                drawResult.stateBefore,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    drawResult.stateBefore),
                drawResult.stateAfter,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    drawResult.stateAfter),
                native_equipped_weapon_draw::submitResultName(
                    drawResult.result));
            clearEquippedWeaponShoulderSheath(
                "physical-hand-unsheath-acknowledged");
        };

        const auto restartRetrievalGesture = [this, &currentIdentity](
                                                 const char* reason) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Equipped weapon shoulder unsheath unacknowledged formID={:08X} hand={} requests={} elapsed={:.3f}s reason={}; preserving shoulder stash for another gesture",
                currentIdentity.formID,
                equipped_weapon_drop_policy::sourceHandName(
                    _equippedWeaponShoulderSheath.retrievalHand),
                _equippedWeaponShoulderSheath.drawWait.requestCount,
                _equippedWeaponShoulderSheath.drawWait.elapsedSeconds,
                reason ? reason : "unknown");
            _equippedWeaponShoulderSheath.retrievalHand =
                equipped_weapon_drop_policy::SourceHand::None;
            _equippedWeaponShoulderSheath.retrievalConfidence = 0.0f;
            _equippedWeaponShoulderSheath.drawWait = {};
            _equippedWeaponSheathRetrievalStates = {};
        };

        if (menuInputActive) {
            _equippedWeaponSheathRetrievalStates = {};
            return;
        }

        if (_equippedWeaponShoulderSheath.retrievalHand !=
            equipped_weapon_drop_policy::SourceHand::None) {
            const auto action =
                equipped_weapon_drop_policy::advanceShoulderDrawWait(
                    _equippedWeaponShoulderSheath.drawWait,
                    nativeWeaponState,
                    frame.deltaSeconds);
            if (action == equipped_weapon_drop_policy::
                              ShoulderDrawAction::CommitRetrieval) {
                native_equipped_weapon_draw::Result acknowledged{};
                acknowledged.stateBefore = nativeWeaponState;
                acknowledged.stateAfter = nativeWeaponState;
                acknowledged.result = native_equipped_weapon_draw::
                    SubmitResult::AlreadyDrawingOrDrawn;
                commitRetrieval(
                    _equippedWeaponShoulderSheath.retrievalHand,
                    _equippedWeaponShoulderSheath.retrievalConfidence,
                    acknowledged);
                return;
            }
            if (action == equipped_weapon_drop_policy::
                              ShoulderDrawAction::RestartGesture) {
                restartRetrievalGesture("native-draw-timeout");
                return;
            }
            if (action == equipped_weapon_drop_policy::
                              ShoulderDrawAction::SubmitDraw) {
                const auto drawResult =
                    native_equipped_weapon_draw::submitExactCurrent(
                        currentIdentity);
                const bool drawAccepted =
                    drawResult.result == native_equipped_weapon_draw::
                                             SubmitResult::Submitted ||
                    drawResult.result == native_equipped_weapon_draw::
                                             SubmitResult::
                                                 AlreadyDrawingOrDrawn;
                if (!drawAccepted) {
                    restartRetrievalGesture(
                        native_equipped_weapon_draw::submitResultName(
                            drawResult.result));
                    return;
                }
                if (drawResult.result == native_equipped_weapon_draw::
                                             SubmitResult::
                                                 AlreadyDrawingOrDrawn ||
                    equipped_weapon_drop_policy::
                        nativeStateAcknowledgesShoulderDraw(
                            drawResult.stateAfter)) {
                    commitRetrieval(
                        _equippedWeaponShoulderSheath.retrievalHand,
                        _equippedWeaponShoulderSheath.
                            retrievalConfidence,
                        drawResult);
                    return;
                }
                ROCK_LOG_SAMPLE_INFO(
                    Weapon,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Equipped weapon shoulder unsheath awaiting native acknowledgement formID={:08X} hand={} requests={} elapsed={:.3f}s state={}({})->{}({})",
                    currentIdentity.formID,
                    equipped_weapon_drop_policy::sourceHandName(
                        _equippedWeaponShoulderSheath.retrievalHand),
                    _equippedWeaponShoulderSheath.drawWait.requestCount,
                    _equippedWeaponShoulderSheath.drawWait.elapsedSeconds,
                    drawResult.stateBefore,
                    held_weapon_equip_state_policy::nativeWeaponStateName(
                        drawResult.stateBefore),
                    drawResult.stateAfter,
                    held_weapon_equip_state_policy::nativeWeaponStateName(
                        drawResult.stateAfter));
            }
            return;
        }

        const bool nativePresentationRetrievable =
            held_weapon_equip_state_policy::
                isShoulderStashedPresentationState(nativeWeaponState);
        if (!nativePresentationRetrievable) {
            clearEquippedWeaponShoulderSheath(
                "native-weapon-no-longer-sheathed");
            return;
        }

        std::array<shoulder_stash::Decision, 2> decisions{};
        std::array<equipped_weapon_drop_policy::ShoulderRetrievalCandidate, 2>
            candidates{};
        const auto detectorConfig =
            makeEquippedWeaponStashDetectorConfig(true);
        for (const bool isLeft : { false, true }) {
            const std::size_t handIndex = isLeft ? 1u : 0u;
            Hand& hand = isLeft ? _leftHand : _rightHand;
            const HandFrameInput& handInput = isLeft ? frame.left : frame.right;
            auto& detectorState =
                _equippedWeaponSheathRetrievalStates[handIndex];
            const bool handEmpty = !hand.isHolding() &&
                !_touchGrabRuntime.isHandActive(isLeft) &&
                !_pendingForceGrabCommits[handIndex].active &&
                !hand.hasActivePullCatchIntent() &&
                !hand.hasPendingActorEquipmentDropHandoff() &&
                !_twoHandedGrip.isHandPartGripping(isLeft) &&
                !(_twoHandedGrip.isFiringGripOccupied() &&
                    _twoHandedGrip.isFiringHandLeft() == isLeft);
            const bool handAllowedByHandlingMode =
                _equippedWeaponHandlingSettings.
                    ambidextrousHandoffEnabled ||
                isLeft == _fixedFiringHandIsLeft;
            const bool handCanOwnFiringGrip =
                handAllowedByHandlingMode &&
                TwoHandedGrip::canBeginPrimaryOnlyGripForHand(isLeft);
            if (handInput.disabled || !handEmpty ||
                !handCanOwnFiringGrip) {
                shoulder_stash::resetRuntime(detectorState);
                continue;
            }

            const auto decision = shoulder_stash::evaluate(
                shoulder_stash::DetectorInput{
                    .isLeftHand = isLeft,
                    .probe = shoulder_stash::Probe{
                        .pointGame = handInput.grabAnchorWorld,
                    },
                    .hmdProbe = makeShoulderStashHmdProbe(handInput),
                    .hasHmdProbe = true,
                    .hasHmdFrame = frame.hasHmdFrame,
                    .hmdPositionWorld = frame.hmdPositionWorld,
                    .hmdForwardWorld = frame.hmdForwardWorld,
                    .deltaSeconds = frame.deltaSeconds,
                    .config = detectorConfig,
                },
                detectorState);
            decisions[handIndex] = decision;
            const bool sameShoulderZone =
                decision.zone == _equippedWeaponShoulderSheath.zone;
            const bool gripPhysicallyHeld =
                input_remap_runtime::isRawButtonPhysicallyHeld(
                    isLeft,
                    g_rockConfig.rockGrabButtonID);
            candidates[handIndex] =
                equipped_weapon_drop_policy::ShoulderRetrievalCandidate{
                    .eligible = equipped_weapon_drop_policy::
                        canRetrieveShoulderStashedWeapon(
                            equipped_weapon_drop_policy::
                                ShoulderRetrievalInput{
                                    .stashActive = true,
                                    .handlingEnabled = handlingEnabled,
                                    .identityMatches = identityMatches,
                                    .nativePresentationRetrievable =
                                        nativePresentationRetrievable,
                                    .menuInputActive = menuInputActive,
                                    .handDisabled = handInput.disabled,
                                    .handEmpty = handEmpty,
                                    .handCanOwnFiringGrip =
                                        handCanOwnFiringGrip,
                                    .detectorConfirmed =
                                        decision.confirmedForCommit,
                                    .sameShoulderZone = sameShoulderZone,
                                    .gripPhysicallyHeld =
                                        gripPhysicallyHeld,
                                }),
                    .confidence = decision.confidence,
                };

            if (decision.candidate && sameShoulderZone &&
                g_rockConfig.rockShoulderStashHapticsEnabled &&
                shouldEmitShoulderStashCandidatePulse(
                    decision,
                    detectorState,
                    _dynamicPushElapsedSeconds)) {
                    (void)_feedbackHaptics.queue(
                        isLeft ? feedback_haptics::FeedbackHand::Left :
                                 feedback_haptics::FeedbackHand::Right,
                        g_rockConfig.
                            rockShoulderStashCandidateHapticDurationSeconds,
                        shoulder_stash_haptic_policy::
                            computeCandidatePulseIntensity(
                                decision.confidence,
                                shoulder_stash_haptic_policy::
                                    CandidatePulseConfig{
                                        .enabled = true,
                                        .baseIntensity = g_rockConfig.
                                            rockShoulderStashCandidateHapticBaseIntensity,
                                        .maxIntensity = g_rockConfig.
                                            rockShoulderStashCandidateHapticIntensity,
                                    }));
            }
        }

        const auto retrievalHand =
            equipped_weapon_drop_policy::selectShoulderRetrievalHand(
                candidates[0],
                candidates[1],
                _equippedWeaponShoulderSheath.stashedByLeftHand);
        if (retrievalHand ==
            equipped_weapon_drop_policy::SourceHand::None) {
            return;
        }

        const bool retrieveWithLeftHand =
            equipped_weapon_drop_policy::isLeft(retrievalHand);
        const std::size_t handIndex = retrieveWithLeftHand ? 1u : 0u;
        const auto drawResult =
            native_equipped_weapon_draw::submitExactCurrent(
                currentIdentity);
        const bool drawAccepted =
            drawResult.result ==
                native_equipped_weapon_draw::SubmitResult::Submitted ||
            drawResult.result ==
                native_equipped_weapon_draw::SubmitResult::
                    AlreadyDrawingOrDrawn;
        if (!drawAccepted) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Equipped weapon shoulder unsheath failed formID={:08X} hand={} zone={} state={}({})->{}({}) result={}",
                currentIdentity.formID,
                equipped_weapon_drop_policy::sourceHandName(retrievalHand),
                body_zone::bodyZoneName(
                    _equippedWeaponShoulderSheath.zone),
                drawResult.stateBefore,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    drawResult.stateBefore),
                drawResult.stateAfter,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    drawResult.stateAfter),
                native_equipped_weapon_draw::submitResultName(
                    drawResult.result));
            shoulder_stash::resetRuntime(
                _equippedWeaponSheathRetrievalStates[handIndex]);
            return;
        }

        if (drawResult.result == native_equipped_weapon_draw::SubmitResult::
                                     AlreadyDrawingOrDrawn ||
            equipped_weapon_drop_policy::nativeStateAcknowledgesShoulderDraw(
                drawResult.stateAfter)) {
            commitRetrieval(
                retrievalHand,
                decisions[handIndex].confidence,
                drawResult);
            return;
        }

        _equippedWeaponShoulderSheath.retrievalHand = retrievalHand;
        _equippedWeaponShoulderSheath.retrievalConfidence =
            decisions[handIndex].confidence;
        equipped_weapon_drop_policy::beginShoulderDrawWait(
            _equippedWeaponShoulderSheath.drawWait);
        shoulder_stash::resetRuntime(
            _equippedWeaponSheathRetrievalStates[handIndex]);
        ROCK_LOG_INFO(
            Weapon,
            "Equipped weapon shoulder unsheath submitted formID={:08X} hand={} zone={} confidence={:.2f} state={}({})->{}({}); retaining transaction until native acknowledgement",
            currentIdentity.formID,
            equipped_weapon_drop_policy::sourceHandName(retrievalHand),
            body_zone::bodyZoneName(
                _equippedWeaponShoulderSheath.zone),
            decisions[handIndex].confidence,
            drawResult.stateBefore,
            held_weapon_equip_state_policy::nativeWeaponStateName(
                drawResult.stateBefore),
            drawResult.stateAfter,
            held_weapon_equip_state_policy::nativeWeaponStateName(
                drawResult.stateAfter));
    }

    void PhysicsInteraction::serviceFixedWeaponHand(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey,
        const bool menuInputActive)
    {
        constexpr std::uint16_t kMaximumResolveFrames = 180;

        if (!_fixedFiringHandIsLeft || !weaponNode ||
            currentWeaponGenerationKey == 0 ||
            currentEquippedWeaponOwnershipKey == 0) {
            _fixedLeftCarry = {};
            return;
        }
        if (_equippedWeaponShoulderSheath.active ||
            _pendingEquippedWeaponPrimaryOnlyGripStart.pending) {
            _fixedLeftCarry = {};
            return;
        }

        // An addon-owned Pip-Boy selection is an explicit dynamic side choice.
        // Likewise, a live manual handoff is preserved regardless of whether
        // its effective ambidextrous policy comes from ROCK or the addon. The
        // fixed hand remains the fallback/default rather than fighting the
        // player's deliberate switch.
        if (_equippedWeaponHandAssignment.pending ||
            _equippedWeaponHandAssignment.active) {
            _fixedLeftCarry = {};
            return;
        }
        if (_equippedWeaponHandlingSettings.ambidextrousHandoffEnabled &&
            _twoHandedGrip.isManualOwnershipActive()) {
            _fixedLeftCarry = {};
            return;
        }

        if (_twoHandedGrip.isManualOwnershipActive()) {
            if (_twoHandedGrip.isFiringHandLeft()) {
                _fixedLeftCarry = {};
                return;
            }
            _twoHandedGrip.restoreNativeRightEquippedCarry(
                "fixed-left-hand-enforcement");
        }

        if (!TwoHandedGrip::canBeginPrimaryOnlyGripForHand(true)) {
            if (!_fixedLeftCarry.infrastructureWarningLogged) {
                _fixedLeftCarry.infrastructureWarningLogged = true;
                ROCK_LOG_WARN(
                    Weapon,
                    "Fixed left weapon hand unavailable because the required hFRIK ownership blockers are missing; retaining physical right-hand carry");
            }
            return;
        }
        if (menuInputActive || !f4vr::isNodeVisible(weaponNode)) {
            return;
        }

        auto& state = _fixedLeftCarry;
        if (state.weaponGenerationKey != currentWeaponGenerationKey ||
            state.weaponOwnershipKey != currentEquippedWeaponOwnershipKey) {
            state = FixedLeftCarryState{
                .weaponGenerationKey = currentWeaponGenerationKey,
                .weaponOwnershipKey = currentEquippedWeaponOwnershipKey,
                .remainingResolveFrames = kMaximumResolveFrames,
            };
        }

        const bool nativeOffsetReady =
            sampleNativeOffsetReadiness(state, weaponNode->local);
        if (nativeOffsetReady &&
            _twoHandedGrip.beginPersistentEquippedCarry(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey)) {
            ROCK_LOG_INFO(
                Weapon,
                "ROCK fixed left-hand equipped-weapon carry active generation={:016X} ownership={:016X}",
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey);
            state = {};
            return;
        }

        if (state.remainingResolveFrames > 0) {
            --state.remainingResolveFrames;
        }
        if (state.remainingResolveFrames == 0) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Fixed left weapon hand is waiting for a generation-bound native carry calibration generation={:016X} ownership={:016X}",
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey);
            state.remainingResolveFrames = kMaximumResolveFrames;
        }
    }

    ::rock::provider::RockProviderResultV1
    PhysicsInteraction::requestProviderEquippedWeaponHandV1(
        const std::uint64_t ownerToken,
        const ::rock::provider::RockProviderEquippedWeaponHandRequestV1& request)
    {
        using Result = ::rock::provider::RockProviderResultV1;
        using Hand = ::rock::provider::RockProviderHand;

        if (!_initialized) {
            return Result::NotReady;
        }
        if (request.hand != Hand::Right && request.hand != Hand::Left) {
            return Result::HandUnavailable;
        }

        const auto* equippedWeapon = currentEquippedWeaponForm();
        if (!equippedWeapon) {
            return Result::TargetUnavailable;
        }
        if (request.weaponFormId != 0 &&
            request.weaponFormId != equippedWeapon->formID) {
            return Result::TargetUnavailable;
        }

        const auto currentWeaponGenerationKey =
            _weaponCollision.getCurrentWeaponGenerationKey();
        if (request.weaponGenerationKey != 0 &&
            request.weaponGenerationKey != currentWeaponGenerationKey) {
            return Result::TargetUnavailable;
        }

        const bool requestedLeft = request.hand == Hand::Left;
        if (requestedLeft &&
            !TwoHandedGrip::canBeginPrimaryOnlyGripForHand(true)) {
            return Result::HandUnavailable;
        }

        auto& existing = _equippedWeaponHandAssignment;
        if (existing.source ==
                EquippedWeaponHandAssignmentSource::Provider &&
            existing.ownerToken == ownerToken &&
            existing.formId == equippedWeapon->formID &&
            existing.assignedLeft == requestedLeft &&
            existing.requestedWeaponGenerationKey ==
                request.weaponGenerationKey) {
            return existing.active &&
                           existing.effectiveLeft == requestedLeft ?
                Result::Ok :
                Result::RequestQueued;
        }

        clearEquippedWeaponHandAssignment(
            "provider-hand-request-replaced",
            true);
        _equippedWeaponHandAssignment =
            EquippedWeaponHandAssignmentState{
                .source =
                    EquippedWeaponHandAssignmentSource::Provider,
                .pending = true,
                .active = false,
                .assignedLeft = requestedLeft,
                .effectiveLeft = false,
                .remainingResolveFrames =
                    kEquippedWeaponHandAssignmentMaximumResolveFrames,
                .formId = equippedWeapon->formID,
                .ownerToken = ownerToken,
                .requestedWeaponGenerationKey =
                    request.weaponGenerationKey,
            };
        ROCK_LOG_INFO(
            Weapon,
            "Provider requested equipped weapon hand={} owner={:016X} form={:08X} generation={:016X}",
            requestedLeft ? "left" : "right",
            ownerToken,
            equippedWeapon->formID,
            request.weaponGenerationKey);
        return Result::RequestQueued;
    }

    void PhysicsInteraction::clearEquippedWeaponHandAssignment(
        const char* reason,
        const bool clearUiAssignment)
    {
        const auto& assignment = _equippedWeaponHandAssignment;
        if (assignment.pending || assignment.active) {
            ROCK_LOG_INFO(
                Weapon,
                "{} weapon hand assignment cleared reason={} handle={} stack={} form={:08X} owner={:016X}",
                assignment.source ==
                        EquippedWeaponHandAssignmentSource::Provider ?
                    "Provider" :
                    "Pip-Boy",
                reason ? reason : "unknown",
                assignment.handleId,
                assignment.stackId,
                assignment.formId,
                assignment.ownerToken);
        }
        _twoHandedGrip.restoreNativeRightEquippedCarry(reason);
        if (clearUiAssignment &&
            assignment.source ==
                EquippedWeaponHandAssignmentSource::Pipboy) {
            pipboy_equip_runtime::clearWeaponAssignment(
                assignment.handleId,
                assignment.stackId);
        }
        _equippedWeaponHandAssignment = {};
    }

    void PhysicsInteraction::serviceEquippedWeaponHandAssignment(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey,
        const bool menuInputActive,
        const EquippedWeaponHandlingSettings& handlingSettings)
    {
        if (_equippedWeaponShoulderSheath.active) {
            return;
        }
        const auto equipMode = pipboy_equip_policy::resolveEquipMode(
            handlingSettings.externalAuthorityActive &&
                handlingSettings.pipboyTriggerHandEquipEnabled,
            _fixedFiringHandIsLeft);
        const bool pipboyAssignmentManaged =
            pipboy_equip_policy::managesHandAssignment(equipMode);
        const bool leftCarryAvailable =
            TwoHandedGrip::canBeginPrimaryOnlyGripForHand(true);
        pipboy_equip_runtime::setLeftHandEquipAvailable(
            pipboyAssignmentManaged && leftCarryAvailable);

        if (!pipboyAssignmentManaged) {
            pipboy_equip_runtime::SelectionEvent discardedEvent{};
            (void)pipboy_equip_runtime::consumeSelectionEvent(
                _lastPipboyWeaponSelectionSequence,
                discardedEvent);
            if (_equippedWeaponHandAssignment.source ==
                    EquippedWeaponHandAssignmentSource::Pipboy &&
                (_equippedWeaponHandAssignment.pending ||
                    _equippedWeaponHandAssignment.active)) {
                clearEquippedWeaponHandAssignment(
                    "native-right-preference",
                    true);
            } else {
                pipboy_equip_runtime::AssignmentSnapshot persisted{};
                if (pipboy_equip_runtime::getAssignment(persisted) &&
                    persisted.active) {
                    pipboy_equip_runtime::clearWeaponAssignment();
                }
            }
        } else {
            pipboy_equip_runtime::SelectionEvent event{};
            if (pipboy_equip_runtime::consumeSelectionEvent(
                    _lastPipboyWeaponSelectionSequence,
                    event)) {
                const bool matchesCurrent =
                    _equippedWeaponHandAssignment.source ==
                        EquippedWeaponHandAssignmentSource::Pipboy &&
                    (_equippedWeaponHandAssignment.pending ||
                        _equippedWeaponHandAssignment.active) &&
                    _equippedWeaponHandAssignment.handleId ==
                        event.handleId &&
                    _equippedWeaponHandAssignment.stackId ==
                        event.stackId;
                if (!event.equipped) {
                    if (matchesCurrent) {
                        clearEquippedWeaponHandAssignment(
                            "selected-stack-unequipped",
                            false);
                    }
                } else {
                    clearEquippedWeaponHandAssignment(
                        "new-pipboy-selection",
                        false);
                    _equippedWeaponHandAssignment =
                        EquippedWeaponHandAssignmentState{
                            .source =
                                EquippedWeaponHandAssignmentSource::
                                    Pipboy,
                            .pending = true,
                            .active = false,
                            .assignedLeft =
                                event.requestedHand ==
                                pipboy_equip_policy::Hand::Left,
                            .effectiveLeft = false,
                            .remainingResolveFrames =
                                kEquippedWeaponHandAssignmentMaximumResolveFrames,
                            .handleId = event.handleId,
                            .stackId = event.stackId,
                            .formId = event.formId,
                        };
                }
            }
        }

        // PhysicsInteraction may be recreated across an hFRIK/skeleton
        // lifecycle while the inventory assignment remains valid. Rehydrate
        // from the hook-owned value snapshot instead of losing the selected
        // hand or retaining any engine pointer across the lifecycle.
        if (pipboyAssignmentManaged &&
            !_equippedWeaponHandAssignment.pending &&
            !_equippedWeaponHandAssignment.active) {
            pipboy_equip_runtime::AssignmentSnapshot persisted{};
            if (pipboy_equip_runtime::getAssignment(persisted) &&
                persisted.active) {
                const bool left = persisted.hand == pipboy_equip_policy::Hand::Left;
                _equippedWeaponHandAssignment =
                    EquippedWeaponHandAssignmentState{
                        .source =
                            EquippedWeaponHandAssignmentSource::Pipboy,
                        .pending = left,
                        .active = !left,
                        .assignedLeft = left,
                        .effectiveLeft = left,
                        .remainingResolveFrames =
                            kEquippedWeaponHandAssignmentMaximumResolveFrames,
                        .handleId = persisted.handleId,
                        .stackId = persisted.stackId,
                        .formId = persisted.formId,
                    };
            }
        }

        auto& assignment = _equippedWeaponHandAssignment;
        if (!assignment.pending && !assignment.active) {
            return;
        }

        _pendingEquippedWeaponPrimaryOnlyGripStart = {};
        if (assignment.source ==
            EquippedWeaponHandAssignmentSource::Pipboy) {
            pipboy_equip_runtime::StackSnapshot stack{};
            if (!pipboy_equip_runtime::inspectStack(
                    assignment.handleId,
                    assignment.stackId,
                    stack) ||
                !stack.resolved || !stack.weapon || !stack.equipped ||
                stack.formId != assignment.formId) {
                clearEquippedWeaponHandAssignment(
                    "selected-stack-no-longer-equipped",
                    true);
                return;
            }
        } else if (assignment.source ==
                   EquippedWeaponHandAssignmentSource::Provider) {
            std::uint32_t requiredFlags =
                static_cast<std::uint32_t>(
                    ::rock::provider::
                        RockProviderEquippedWeaponHandlingFlagV1::
                            FiringGripOwnership);
            if (assignment.assignedLeft) {
                requiredFlags |= static_cast<std::uint32_t>(
                    ::rock::provider::
                        RockProviderEquippedWeaponHandlingFlagV1::
                            AmbidextrousHandoff);
            }
            if (!::rock::provider::
                    ownsEquippedWeaponHandlingAuthorityV1(
                        assignment.ownerToken,
                        requiredFlags)) {
                clearEquippedWeaponHandAssignment(
                    "provider-hand-authority-lost",
                    false);
                return;
            }
            const auto* equippedWeapon = currentEquippedWeaponForm();
            if (!equippedWeapon ||
                equippedWeapon->formID != assignment.formId ||
                (assignment.requestedWeaponGenerationKey != 0 &&
                    currentWeaponGenerationKey != 0 &&
                    currentWeaponGenerationKey !=
                        assignment.requestedWeaponGenerationKey)) {
                clearEquippedWeaponHandAssignment(
                    "provider-hand-target-changed",
                    false);
                return;
            }
        } else {
            clearEquippedWeaponHandAssignment(
                "invalid-hand-assignment-source",
                false);
            return;
        }

        if (pipboy_equip_policy::shouldReacquirePersistentLeftCarry(
                assignment.active,
                assignment.assignedLeft,
                assignment.effectiveLeft,
                _twoHandedGrip.isPersistentEquippedCarryActive(),
                _twoHandedGrip.isManualOwnershipActive())) {
            // Menu/lifecycle gates intentionally reset TwoHandedGrip. Preserve
            // the exact hand assignment and reacquire after native right carry
            // has produced a fresh canonical frame.
            assignment.active = false;
            assignment.pending = true;
            assignment.ownershipKey = 0;
            assignment.remainingResolveFrames =
                kEquippedWeaponHandAssignmentMaximumResolveFrames;
            assignment.nativeOffsetGenerationKey = 0;
            assignment.nativeOffsetSampleValid = false;
            assignment.nativeOffsetReadinessLogged = false;
            assignment.matchingNativeOffsetFrames = 0;
        }

        if (!assignment.pending) {
            if (!assignment.effectiveLeft) {
                _twoHandedGrip.clearPersistentEquippedCarry("right-hand-assignment");
            }
            return;
        }

        const auto commitRight = [&](const char* reason) {
            _twoHandedGrip.restoreNativeRightEquippedCarry(reason);
            assignment.pending = false;
            assignment.active = true;
            assignment.effectiveLeft = false;
            assignment.ownershipKey = currentEquippedWeaponOwnershipKey;
            if (assignment.source ==
                EquippedWeaponHandAssignmentSource::Pipboy) {
                pipboy_equip_runtime::publishWeaponAssignment(
                    assignment.handleId,
                    assignment.stackId,
                    assignment.formId,
                    pipboy_equip_policy::Hand::Right);
            }
            ROCK_LOG_INFO(
                Weapon,
                "{} weapon assigned to native right hand reason={} form={:08X}",
                assignment.source ==
                        EquippedWeaponHandAssignmentSource::Provider ?
                    "Provider" :
                    "Pip-Boy",
                reason ? reason : "unknown",
                assignment.formId);
        };

        if (!assignment.assignedLeft || !leftCarryAvailable) {
            if (assignment.assignedLeft &&
                assignment.source ==
                    EquippedWeaponHandAssignmentSource::Provider) {
                clearEquippedWeaponHandAssignment(
                    "provider-left-carry-unavailable",
                    false);
                return;
            }
            commitRight(
                assignment.assignedLeft ?
                    "left-carry-unavailable" :
                    "right-hand-assignment");
            return;
        }
        if (menuInputActive) {
            return;
        }

        const auto* equippedWeapon = currentEquippedWeaponForm();
        const bool identityReady =
            weaponNode &&
            f4vr::isNodeVisible(weaponNode) &&
            currentWeaponGenerationKey != 0 &&
            currentEquippedWeaponOwnershipKey != 0 &&
            equippedWeapon &&
            equippedWeapon->formID == assignment.formId;

        bool nativeOffsetReady = false;
        if (identityReady) {
            if (assignment.nativeOffsetGenerationKey != currentWeaponGenerationKey) {
                assignment.nativeOffsetGenerationKey = currentWeaponGenerationKey;
                assignment.nativeOffsetSampleValid = false;
                assignment.nativeOffsetReadinessLogged = false;
                assignment.matchingNativeOffsetFrames = 0;
            }

            const auto previousMatchingFrames = assignment.matchingNativeOffsetFrames;
            // hFRIK owns the live native-right offset in every weapon mode.
            nativeOffsetReady = sampleNativeOffsetReadiness(
                assignment,
                weaponNode->local);
            if (!assignment.nativeOffsetReadinessLogged &&
                previousMatchingFrames == 0 && assignment.matchingNativeOffsetFrames == 1) {
                assignment.nativeOffsetReadinessLogged = true;
                ROCK_LOG_INFO(
                    Weapon,
                    "{} left-hand assignment observed visible native-right offset; reserving canonical refresh form={:08X}",
                    assignment.source ==
                            EquippedWeaponHandAssignmentSource::Provider ?
                        "Provider" :
                        "Pip-Boy",
                    assignment.formId);
            }
        } else {
            assignment.matchingNativeOffsetFrames = 0;
        }

        if (identityReady && nativeOffsetReady &&
            _twoHandedGrip.beginPersistentEquippedCarry(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey)) {
            assignment.pending = false;
            assignment.active = true;
            assignment.effectiveLeft = true;
            assignment.ownershipKey = currentEquippedWeaponOwnershipKey;
            if (assignment.source ==
                EquippedWeaponHandAssignmentSource::Pipboy) {
                pipboy_equip_runtime::publishWeaponAssignment(
                    assignment.handleId,
                    assignment.stackId,
                    assignment.formId,
                    pipboy_equip_policy::Hand::Left);
            }
            ROCK_LOG_INFO(
                Weapon,
                "{} weapon assigned to left hand form={:08X}",
                assignment.source ==
                        EquippedWeaponHandAssignmentSource::Provider ?
                    "Provider" :
                    "Pip-Boy",
                assignment.formId);
            return;
        }

        if (assignment.remainingResolveFrames > 0) {
            --assignment.remainingResolveFrames;
        }
        if (assignment.remainingResolveFrames == 0) {
            if (assignment.source ==
                EquippedWeaponHandAssignmentSource::Provider) {
                ROCK_LOG_WARN(
                    Weapon,
                    "Provider left-hand assignment timed out waiting for a generation-bound canonical carry form={:08X}",
                    assignment.formId);
                clearEquippedWeaponHandAssignment(
                    "provider-left-carry-resolve-timeout",
                    false);
            } else {
                ROCK_LOG_WARN(
                    Weapon,
                    "Pip-Boy left-hand assignment timed out waiting for a generation-bound canonical carry; falling back to right form={:08X}",
                    assignment.formId);
                commitRight("left-carry-resolve-timeout");
            }
        }
    }

    void PhysicsInteraction::reconcileEquippedWeaponHandAssignmentAfterGrip()
    {
        auto& assignment = _equippedWeaponHandAssignment;
        if (!assignment.active) {
            return;
        }
        const bool currentLeft = _twoHandedGrip.isFiringGripOccupied() && _twoHandedGrip.isFiringHandLeft();
        if (currentLeft == assignment.effectiveLeft) {
            return;
        }
        // A deliberate physical handover becomes the durable assignment.
        // Lifecycle reacquisition must restore the current side, not the side
        // originally requested by an older Pip-Boy transaction.
        assignment.assignedLeft = currentLeft;
        assignment.effectiveLeft = currentLeft;
        if (assignment.source ==
            EquippedWeaponHandAssignmentSource::Pipboy) {
            pipboy_equip_runtime::publishWeaponAssignment(
                assignment.handleId,
                assignment.stackId,
                assignment.formId,
                currentLeft ?
                    pipboy_equip_policy::Hand::Left :
                    pipboy_equip_policy::Hand::Right);
        }
        ROCK_LOG_INFO(
            Weapon,
            "{} weapon hand assignment followed firing-grip handoff hand={} form={:08X}",
            assignment.source ==
                    EquippedWeaponHandAssignmentSource::Provider ?
                "Provider" :
                "Pip-Boy",
            currentLeft ? "left" : "right",
            assignment.formId);
    }

}
