#include "physics-interaction/weapon/TwoHandedGripInternal.h"

// Grip failure diagnostics: per-frame failure records, locked-hand authority attempts, and incident dumps.

namespace rock
{
    void TwoHandedGrip::recordGripFailureFrame(
        RE::NiNode* weaponNode,
        const EquippedWeaponGripFrameInput& frameInput,
        const EquippedWeaponGripFrameInput& stableFrameInput,
        const float dt,
        const std::uint32_t currentWeaponFormID,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey)
    {
        if (!g_rockConfig.rockDebugGripFailureTelemetry) {
            return;
        }
        if (std::isfinite(dt) && dt > 0.0f) {
            _telemetry.detailedLogCooldownSeconds = (std::max)(
                0.0f,
                _telemetry.detailedLogCooldownSeconds - dt);
        }

        const auto& runtime = runtime_state::currentFrame();
        GripFailureFrameSnapshot snapshot{};
        snapshot.handFrames[0] = _scope.safeHandFrames[0].diagnostic;
        snapshot.handFrames[1] = _scope.safeHandFrames[1].diagnostic;
        snapshot.hmdPositionWorld = frameInput.hmdPositionWorld;
        snapshot.playerSpaceDeltaGameUnits =
            runtime.playerSpace.deltaGameUnits;
        snapshot.leftPhysicalGripInput =
            frameInput.leftPhysicalGripInput;
        snapshot.rightPhysicalGripInput =
            frameInput.rightPhysicalGripInput;
        snapshot.primaryLogicalInput = frameInput.primaryGripInput;
        snapshot.primaryDebouncedInput = stableFrameInput.primaryGripInput;
        snapshot.frameIndex = runtime.frameIndex;
        snapshot.weaponGenerationKey = currentWeaponGenerationKey;
        snapshot.equippedWeaponOwnershipKey =
            currentEquippedWeaponOwnershipKey;
        const WeaponPartGrip& supportGrip = supportPartGrip();
        snapshot.supportGripSequence =
            supportGrip.active ? supportGrip.gripSequence : 0;
        snapshot.weaponFormID = currentWeaponFormID;
        snapshot.primaryReleaseOpenFrames =
            _firing.primaryReleaseDebounce.consecutiveOpenFrames;
        snapshot.deltaSeconds = dt;
        snapshot.state = _session.state;
        snapshot.authorityMode = _session.authorityMode;
        snapshot.firingHandIsLeft = isFiringHandLeft();
        snapshot.leftGripHeld = stableFrameInput.leftGripHeld;
        snapshot.rightGripHeld = stableFrameInput.rightGripHeld;
        snapshot.leftHandHoldingObject =
            stableFrameInput.leftHandHoldingObject;
        snapshot.rightHandHoldingObject =
            stableFrameInput.rightHandHoldingObject;
        snapshot.toggleGrabEnabled = frameInput.toggleGrabEnabled;
        snapshot.animationBoundaryActive =
            frameInput.animationBoundaryActive;
        snapshot.scopeMenuOpen = frameInput.scopeMenuOpen;
        snapshot.manualScopeActivationRequested =
            frameInput.manualScopeActivationRequested;
        snapshot.nativeScopeRequestStateValid =
            frameInput.nativeScopeRequestStateValid;
        snapshot.nativeScopeRequestActive =
            frameInput.nativeScopeRequestActive;
        snapshot.hasHmdFrame = frameInput.hasHmdFrame;
        snapshot.visualAuthorityAvailable =
            runtime.visualAuthorityAvailable;
        if (weaponNode) {
            snapshot.weaponWorld = weaponNode->world;
            snapshot.weaponWorldValid = isFiniteTransform(weaponNode->world);
        }

        const std::size_t historyIndex = _telemetry.historyNext;
        _telemetry.history[historyIndex] = snapshot;
        _telemetry.currentHistoryIndex = historyIndex;
        _telemetry.historyNext =
            (historyIndex + 1) % kGripFailureHistoryCapacity;
        _telemetry.historyCount = (std::min)(
            kGripFailureHistoryCapacity,
            _telemetry.historyCount + 1);
    }

    void TwoHandedGrip::recordLockedHandAuthorityAttempt(
        const bool isLeft,
        const LockedHandAuthorityRole role,
        const RE::NiTransform& targetWorld,
        const RE::NiTransform* const liveWorld,
        const bool bridgeAvailable,
        const bool applied)
    {
        if (!g_rockConfig.rockDebugGripFailureTelemetry ||
            _telemetry.currentHistoryIndex >=
                kGripFailureHistoryCapacity) {
            return;
        }

        auto& snapshot =
            _telemetry.history[_telemetry.currentHistoryIndex];
        if (snapshot.frameIndex != runtime_state::currentFrame().frameIndex) {
            return;
        }

        LockedHandAuthorityAttemptDiagnostic attempt{};
        attempt.targetWorld = targetWorld;
        attempt.role = role;
        attempt.requested = true;
        attempt.bridgeAvailable = bridgeAvailable;
        attempt.targetUsable = isUsableHandAuthorityTransform(targetWorld);
        attempt.liveWorldAvailable = liveWorld != nullptr;
        if (liveWorld) {
            attempt.liveWorld = *liveWorld;
            attempt.liveWorldUsable =
                isUsableHandAuthorityTransform(*liveWorld);
        }
        attempt.applied = applied;
        snapshot.authorityAttempts[isLeft ? 0u : 1u] = attempt;
    }

    void TwoHandedGrip::logGripFailureIncident(const char* const reason)
    {
        if (!g_rockConfig.rockDebugGripFailureTelemetry) {
            // Summary line only; the ring is empty while telemetry is off.
            const auto occupancy = getGripOccupancy();
            ROCK_LOG_WARN(
                Weapon,
                "TwoHandedGrip: grip failure incident={} reason={} state={} authority={} firingHand={} generation={:016X} ownership={:016X} partGrip(L/R)={}/{}",
                ++_telemetry.incidentSequence,
                reason ? reason : "unknown",
                twoHandedStateDiagnosticName(_session.state),
                supportAuthorityDiagnosticName(_session.authorityMode),
                firingHandName(),
                _session.weaponGenerationKey,
                _session.equippedWeaponOwnershipKey,
                occupancy.left.partGripActive,
                occupancy.right.partGripActive);
            return;
        }

        GripFailureFrameSnapshot fallback{};
        GripFailureFrameSnapshot* current = &fallback;
        if (_telemetry.currentHistoryIndex <
            kGripFailureHistoryCapacity) {
            current =
                &_telemetry.history[_telemetry.currentHistoryIndex];
        } else {
            current->frameIndex = runtime_state::currentFrame().frameIndex;
        }

        current->state = _session.state;
        current->authorityMode = _session.authorityMode;
        current->firingHandIsLeft = isFiringHandLeft();
        current->handFrames[0] = _scope.safeHandFrames[0].diagnostic;
        current->handFrames[1] = _scope.safeHandFrames[1].diagnostic;
        current->weaponGenerationKey = _session.weaponGenerationKey;
        current->equippedWeaponOwnershipKey =
            _session.equippedWeaponOwnershipKey;
        const WeaponPartGrip& supportGrip = supportPartGrip();
        current->supportGripSequence =
            supportGrip.active ? supportGrip.gripSequence : 0;
        if (_session.weaponNode) {
            current->weaponWorld = _session.weaponNode->world;
            current->weaponWorldValid =
                isFiniteTransform(_session.weaponNode->world);
        }

        const auto occupancy = getGripOccupancy();
        const std::uint64_t incident =
            ++_telemetry.incidentSequence;
        const bool emitDetailedHistory =
            _telemetry.detailedLogCooldownSeconds <= 0.0f;
        if (emitDetailedHistory) {
            _telemetry.detailedLogCooldownSeconds =
                GRIP_FAILURE_DETAILED_LOG_COOLDOWN_SECONDS;
        }

        ROCK_LOG_WARN(
            Weapon,
            "TwoHandedGrip: grip failure incident={} reason={} frame={} weapon={:08X} generation={:016X} ownership={:016X} grip={} state={} authority={} firingHand={} supportHand={} inputMode={} logicalHeld(L/R)={}/{} physicalL(H/P/R)={}/{}/{} physicalR(H/P/R)={}/{}/{} primaryLogical(H/P/R)={}/{}/{} primaryDebounced(H/P/R)={}/{}/{} releaseOpenFrames={} occupancy(L/R)={}/{} holding(L/R)={}/{} animationBoundary={} scope(menu/manual/nativeValid/nativeActive)={}/{}/{}/{} hmd={} visualAuthority={} history={}",
            incident,
            reason ? reason : "unknown",
            current->frameIndex,
            current->weaponFormID,
            current->weaponGenerationKey,
            current->equippedWeaponOwnershipKey,
            current->supportGripSequence,
            twoHandedStateDiagnosticName(current->state),
            supportAuthorityDiagnosticName(current->authorityMode),
            current->firingHandIsLeft ? "left" : "right",
            current->firingHandIsLeft ? "right" : "left",
            current->toggleGrabEnabled ? "toggle" : "hold",
            current->leftGripHeld,
            current->rightGripHeld,
            current->leftPhysicalGripInput.held,
            current->leftPhysicalGripInput.pressed,
            current->leftPhysicalGripInput.released,
            current->rightPhysicalGripInput.held,
            current->rightPhysicalGripInput.pressed,
            current->rightPhysicalGripInput.released,
            current->primaryLogicalInput.held,
            current->primaryLogicalInput.pressed,
            current->primaryLogicalInput.released,
            current->primaryDebouncedInput.held,
            current->primaryDebouncedInput.pressed,
            current->primaryDebouncedInput.released,
            static_cast<unsigned>(current->primaryReleaseOpenFrames),
            occupancy.left.weaponEngaged(),
            occupancy.right.weaponEngaged(),
            current->leftHandHoldingObject,
            current->rightHandHoldingObject,
            current->animationBoundaryActive,
            current->scopeMenuOpen,
            current->manualScopeActivationRequested,
            current->nativeScopeRequestStateValid,
            current->nativeScopeRequestActive,
            current->hasHmdFrame,
            current->visualAuthorityAvailable,
            emitDetailedHistory ? "emitted" : "rate-limited");

        const auto logHandFrame = [incident](
                                      const bool isLeft,
                                      const ScopeSafeHandFrameDiagnostic& hand) {
            ROCK_LOG_WARN(
                Weapon,
                "TwoHandedGrip: grip failure hand incident={} hand={} resolution={} root(allowed/valid)={}/{} driver(node/finite/frame/usable/calibration)={}/{}/{}/{}/{} reconstructed={} last={} collisionPresentation={} scopeDriver={} miss={}->{} physicalOverride={} current={} rootT=({:.2f},{:.2f},{:.2f}) driverT=({:.2f},{:.2f},{:.2f}) reconstructedT=({:.2f},{:.2f},{:.2f}) currentT=({:.2f},{:.2f},{:.2f})",
                incident,
                isLeft ? "left" : "right",
                handFrameResolutionDiagnosticName(hand.resolutionMode),
                hand.rootSampleAllowed,
                hand.rootHandValid,
                hand.driverNodeAvailable,
                hand.driverWorldFinite,
                hand.driverFrameValid,
                hand.driverWorldUsable,
                hand.driverToHandLocalAvailable,
                hand.reconstructedHandValid,
                hand.lastHandWorldAvailable,
                hand.collisionPresentationWasLive,
                hand.scopeDriverFrameAuthorityActive,
                hand.consecutiveDriverMissFramesBefore,
                hand.consecutiveDriverMissFramesAfter,
                hand.physicalFrameOverrideApplied,
                hand.currentHandWorldValid,
                hand.rootHandWorld.translate.x,
                hand.rootHandWorld.translate.y,
                hand.rootHandWorld.translate.z,
                hand.driverWorld.translate.x,
                hand.driverWorld.translate.y,
                hand.driverWorld.translate.z,
                hand.reconstructedHandWorld.translate.x,
                hand.reconstructedHandWorld.translate.y,
                hand.reconstructedHandWorld.translate.z,
                hand.currentHandWorld.translate.x,
                hand.currentHandWorld.translate.y,
                hand.currentHandWorld.translate.z);
        };
        logHandFrame(true, current->handFrames[0]);
        logHandFrame(false, current->handFrames[1]);

        const auto authorityRoleName = [](const LockedHandAuthorityRole role) {
            switch (role) {
            case LockedHandAuthorityRole::None:
                return "none";
            case LockedHandAuthorityRole::PrimaryGrip:
                return "primary";
            case LockedHandAuthorityRole::SupportGrip:
                return "support";
            }
            return "unknown";
        };
        const auto logAuthorityAttempt = [&](
                                             const bool isLeft,
                                             const LockedHandAuthorityAttemptDiagnostic& attempt) {
            if (!attempt.requested) {
                return;
            }
            const float targetToLiveDistance =
                attempt.liveWorldAvailable ?
                diagnosticPointDistance(
                    attempt.targetWorld.translate,
                    attempt.liveWorld.translate) :
                -1.0f;
            const float targetToLiveRotation =
                attempt.liveWorldAvailable && attempt.targetUsable &&
                        attempt.liveWorldUsable ?
                    hand_visual_lerp_math::rotationDistanceDegrees(
                        attempt.targetWorld,
                        attempt.liveWorld) :
                    -1.0f;
            const float targetToHmdDistance = current->hasHmdFrame ?
                diagnosticPointDistance(
                    attempt.targetWorld.translate,
                    current->hmdPositionWorld) :
                -1.0f;
            ROCK_LOG_WARN(
                Weapon,
                "TwoHandedGrip: grip failure authority incident={} hand={} role={} bridge={} targetUsable={} live(available/usable)={}/{} applied={} targetT=({:.2f},{:.2f},{:.2f}) targetScale={:.6f} targetDet={:.6f} targetToLive=({:.2f}gu,{:.2f}deg) targetToHmd={:.2f}gu liveT=({:.2f},{:.2f},{:.2f})",
                incident,
                isLeft ? "left" : "right",
                authorityRoleName(attempt.role),
                attempt.bridgeAvailable,
                attempt.targetUsable,
                attempt.liveWorldAvailable,
                attempt.liveWorldUsable,
                attempt.applied,
                attempt.targetWorld.translate.x,
                attempt.targetWorld.translate.y,
                attempt.targetWorld.translate.z,
                attempt.targetWorld.scale,
                diagnosticRotationDeterminant(attempt.targetWorld.rotate),
                targetToLiveDistance,
                targetToLiveRotation,
                targetToHmdDistance,
                attempt.liveWorld.translate.x,
                attempt.liveWorld.translate.y,
                attempt.liveWorld.translate.z);
        };
        logAuthorityAttempt(true, current->authorityAttempts[0]);
        logAuthorityAttempt(false, current->authorityAttempts[1]);

        if (!emitDetailedHistory || _telemetry.historyCount == 0) {
            return;
        }

        const std::size_t historyStart =
            _telemetry.historyCount == kGripFailureHistoryCapacity ?
            _telemetry.historyNext :
            0;
        ROCK_LOG_INFO(
            Weapon,
            "TwoHandedGrip: grip failure history begin incident={} samples={}",
            incident,
            _telemetry.historyCount);
        for (std::size_t ordinal = 0;
             ordinal < _telemetry.historyCount;
             ++ordinal) {
            const auto& sample = _telemetry.history[
                (historyStart + ordinal) % kGripFailureHistoryCapacity];
            const auto& leftFrame = sample.handFrames[0];
            const auto& rightFrame = sample.handFrames[1];
            const auto& leftAuthority = sample.authorityAttempts[0];
            const auto& rightAuthority = sample.authorityAttempts[1];
            const std::uint64_t ageFrames =
                current->frameIndex >= sample.frameIndex ?
                current->frameIndex - sample.frameIndex :
                0;
            ROCK_LOG_INFO(
                Weapon,
                "TwoHandedGrip: grip failure history incident={} sample={}/{} frame={} ageFrames={} dt={:.5f} state={} authority={} firing={} input={} held(L/R)={}/{} physicalHeld(L/R)={}/{} primary(logical/debounced)={}/{} releaseOpen={} animationBoundary={} scope={} weaponT=({:.2f},{:.2f},{:.2f}) playerDelta=({:.2f},{:.2f},{:.2f}) handL(mode/current/driver/reconstructed/collision)={}/{}/{}/{}/{} handR(mode/current/driver/reconstructed/collision)={}/{}/{}/{}/{} authorityL(role/requested/applied/T)=({}/{}/{}/({:.2f},{:.2f},{:.2f})) authorityR(role/requested/applied/T)=({}/{}/{}/({:.2f},{:.2f},{:.2f}))",
                incident,
                ordinal + 1,
                _telemetry.historyCount,
                sample.frameIndex,
                ageFrames,
                sample.deltaSeconds,
                twoHandedStateDiagnosticName(sample.state),
                supportAuthorityDiagnosticName(sample.authorityMode),
                sample.firingHandIsLeft ? "left" : "right",
                sample.toggleGrabEnabled ? "toggle" : "hold",
                sample.leftGripHeld,
                sample.rightGripHeld,
                sample.leftPhysicalGripInput.held,
                sample.rightPhysicalGripInput.held,
                sample.primaryLogicalInput.held,
                sample.primaryDebouncedInput.held,
                static_cast<unsigned>(sample.primaryReleaseOpenFrames),
                sample.animationBoundaryActive,
                sample.scopeMenuOpen,
                sample.weaponWorld.translate.x,
                sample.weaponWorld.translate.y,
                sample.weaponWorld.translate.z,
                sample.playerSpaceDeltaGameUnits.x,
                sample.playerSpaceDeltaGameUnits.y,
                sample.playerSpaceDeltaGameUnits.z,
                handFrameResolutionDiagnosticName(leftFrame.resolutionMode),
                leftFrame.currentHandWorldValid,
                leftFrame.driverWorldUsable,
                leftFrame.reconstructedHandValid,
                leftFrame.collisionPresentationWasLive,
                handFrameResolutionDiagnosticName(rightFrame.resolutionMode),
                rightFrame.currentHandWorldValid,
                rightFrame.driverWorldUsable,
                rightFrame.reconstructedHandValid,
                rightFrame.collisionPresentationWasLive,
                authorityRoleName(leftAuthority.role),
                leftAuthority.requested,
                leftAuthority.applied,
                leftAuthority.targetWorld.translate.x,
                leftAuthority.targetWorld.translate.y,
                leftAuthority.targetWorld.translate.z,
                authorityRoleName(rightAuthority.role),
                rightAuthority.requested,
                rightAuthority.applied,
                rightAuthority.targetWorld.translate.x,
                rightAuthority.targetWorld.translate.y,
                rightAuthority.targetWorld.translate.z);
        }
        ROCK_LOG_INFO(
            Weapon,
            "TwoHandedGrip: grip failure history end incident={}",
            incident);
    }

    void TwoHandedGrip::resetGripFailureDiagnostics()
    {
        _telemetry.history = {};
        _telemetry.historyNext = 0;
        _telemetry.historyCount = 0;
        _telemetry.currentHistoryIndex =
            kGripFailureHistoryCapacity;
        _telemetry.incidentSequence = 0;
        _telemetry.detailedLogCooldownSeconds = 0.0f;
    }
}
