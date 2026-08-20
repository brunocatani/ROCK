/*
 * DIAGNOSTICS ONLY: hand transform parity sampling and the collider clock trace.
 *
 * Nothing in this file writes gameplay, Havok, or scene state. It reads and it logs.
 * If a change here starts to affect behavior, it belongs in another file.
 *
 * All logging is rate limited. These run per frame when their debug toggles are on.
 */

#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/core/PhysicsInteractionTransformValidation.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "RockConfig.h"
#include "RockUtils.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/native/havok/HavokRuntime.h"
#include "physics-interaction/native/query/PhysicsUtils.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "rock_support/Fo4VrRuntime.h"
#include "rock_support/VRControllers.h"

namespace rock
{
    using namespace physics_interaction_detail;

    namespace
    {
        constexpr float kRawParityWarnPosition = 0.10f;
        constexpr float kRawParityWarnRotationDegrees = 0.5f;
        constexpr float kRawParityFailPosition = 0.50f;
        constexpr float kRawParityFailRotationDegrees = 2.0f;
        constexpr int kRawParityWarnFrames = 2;
        constexpr int kRawParityFailFrames = 10;
        constexpr int kRawParitySummaryFrames = 300;
        constexpr int kRawParityLagFrames = 5;
        constexpr float kRawParityLagSlack = 0.05f;

        [[nodiscard]] float measureDirectionDeltaDegrees(
            const RE::NiPoint3& a,
            const RE::NiPoint3& b)
        {
            const float dot = std::clamp(
                a.x * b.x + a.y * b.y + a.z * b.z,
                -1.0f,
                1.0f);
            return std::acos(dot) * (180.0f / std::numbers::pi_v<float>);
        }
    }

    void PhysicsInteraction::sampleHandTransformParity()
    {
        if (!g_rockConfig.rockDebugHandTransformParity) {
            _parityEnabledLogged = false;
            _paritySummaryCounter = 0;
            return;
        }

        if (!frik_visual_authority::isAvailable() || !_handBoneCache.isReady()) {
            return;
        }

        if (!_parityEnabledLogged) {
            ROCK_LOG_INFO(Init, "Hand-transform parity enabled (root flattened cache vs FRIK API, pre-write sampling)");
            _parityEnabledLogged = true;
        }

        const bool playerMoving = runtime_state::currentFrame().playerSpace.moving;
        const bool emitSummary = (++_paritySummaryCounter >= kRawParitySummaryFrames);

        auto sampleHand = [&](bool isLeft) {
            auto& state = _rawHandParityStates[isLeft ? 1 : 0];
            const auto handEnum = handFromBool(isLeft);
            const auto localTransform = _handBoneCache.getWorldTransform(isLeft);
            const auto apiTransform = frik_visual_authority::getHandWorldTransform(handEnum);
            const auto delta = measureTransformDelta(localTransform, apiTransform);
            const auto localPalmPosition = computeGrabLegacyPalmPivotAWorldFromHandBasis(localTransform, isLeft);
            const auto apiPalmPosition = computeGrabLegacyPalmPivotAWorldFromHandBasis(apiTransform, isLeft);
            const auto localPalmNormal = computePalmNormalFromHandBasis(localTransform, isLeft);
            const auto apiPalmNormal = computePalmNormalFromHandBasis(apiTransform, isLeft);
            const auto localPointing = computePointingVectorFromHandBasis(localTransform, isLeft);
            const auto apiPointing = computePointingVectorFromHandBasis(apiTransform, isLeft);
            state.lastPositionDelta = delta.position;
            state.lastRotationDeltaDegrees = delta.rotationDegrees;

            const bool warnExceeded = delta.position > kRawParityWarnPosition || delta.rotationDegrees > kRawParityWarnRotationDegrees;
            const bool failExceeded = delta.position > kRawParityFailPosition || delta.rotationDegrees > kRawParityFailRotationDegrees;

            state.warnFrames = warnExceeded ? state.warnFrames + 1 : 0;
            state.failFrames = failExceeded ? state.failFrames + 1 : 0;

            const char* handLabel = isLeft ? "Left" : "Right";
            if (state.warnFrames == kRawParityWarnFrames) {
                ROCK_LOG_WARN(Hand, "{} raw hand parity warning: posDelta={:.3f} rotDelta={:.3f}deg", handLabel, delta.position, delta.rotationDegrees);
            }

            if (state.failFrames == kRawParityFailFrames) {
                ROCK_LOG_ERROR(Hand, "{} raw hand parity failure: posDelta={:.3f} rotDelta={:.3f}deg", handLabel, delta.position, delta.rotationDegrees);
            }

            if (playerMoving && state.hasPreviousApiTransform) {
                const auto prevApiDelta = measureTransformDelta(localTransform, state.previousApiTransform);
                if (prevApiDelta.position + kRawParityLagSlack < delta.position) {
                    state.lagFrames++;
                    if (state.lagFrames == kRawParityLagFrames) {
                        ROCK_LOG_WARN(Hand, "{} hand parity suggests possible one-frame lag: currentDelta={:.3f} prevApiDelta={:.3f}", handLabel, delta.position,
                            prevApiDelta.position);
                    }
                } else {
                    state.lagFrames = 0;
                }
            } else {
                state.lagFrames = 0;
            }

            state.previousApiTransform = apiTransform;
            state.hasPreviousApiTransform = true;

            if (emitSummary) {
                const char* summaryHandLabel = isLeft ? "L" : "R";
                ROCK_LOG_DEBUG(Hand, "{} parity: raw(pos={:.3f}, rot={:.3f}deg) basis(palmPos={:.3f}, palmNormal={:.3f}deg, pointing={:.3f}deg)", summaryHandLabel,
                    delta.position, delta.rotationDegrees, measurePointDelta(localPalmPosition, apiPalmPosition), measureDirectionDeltaDegrees(localPalmNormal, apiPalmNormal),
                    measureDirectionDeltaDegrees(localPointing, apiPointing));
            }
        };

        sampleHand(false);
        sampleHand(true);

        if (emitSummary) {
            _paritySummaryCounter = 0;
            const auto& right = _rawHandParityStates[0];
            const auto& left = _rawHandParityStates[1];
            ROCK_LOG_DEBUG(Hand, "Raw hand parity summary: R(pos={:.3f}, rot={:.3f}deg) L(pos={:.3f}, rot={:.3f}deg)", right.lastPositionDelta, right.lastRotationDeltaDegrees,
                left.lastPositionDelta, left.lastRotationDeltaDegrees);
        }
    }

    void PhysicsInteraction::logColliderClockTrace(
        const PhysicsFrameContext& frame,
        const RE::NiNode* weaponNode)
    {
        constexpr std::uint32_t kTraceFramesPerEpisode = 360;

        const bool colliderClockDebugActive = g_rockConfig.rockDebugColliderClockLogging;
        if (!colliderClockDebugActive) {
            _colliderClockLastLoggedFrame = 0;
            _colliderClockHasLoggedFrame = false;
            _colliderClockSession = 0;
            _colliderClockFramesRemaining = 0;
            _colliderClockPreviousPlayerMoving = false;
            _colliderClockPreviousContactActive = false;
            _colliderClockPreviousGrabActive = false;
            _grabLocomotionClockStates = {};
            return;
        }

        DynamicWeaponCollisionRuntime::DebugSnapshot weapon{};
        const bool weaponValid = _dynamicWeaponCollision.getDebugSnapshot(weapon);
        dynamic_hand_collision_telemetry::Snapshot hands{};
        const bool handsValid = _dynamicHandCollision.getTelemetrySnapshot(hands);
        const bool contactActive =
            (weaponValid && weapon.contactActive) ||
            (handsValid && (hands.hands[0].anyContact || hands.hands[1].anyContact));
        const auto& runtime = runtime_state::currentFrame();
        const bool playerMoving = runtime.playerSpace.valid && runtime.playerSpace.moving;
        const bool movementStarted =
            playerMoving && !_colliderClockPreviousPlayerMoving;
        const bool contactStarted =
            contactActive && !_colliderClockPreviousContactActive;
        const bool grabActive = _rightHand.isHolding() || _leftHand.isHolding();
        const bool grabStarted = grabActive && !_colliderClockPreviousGrabActive;
        _colliderClockPreviousPlayerMoving = playerMoving;
        _colliderClockPreviousContactActive = contactActive;
        _colliderClockPreviousGrabActive = grabActive;

        if (movementStarted || contactStarted || grabStarted) {
            ++_colliderClockSession;
            _colliderClockFramesRemaining = kTraceFramesPerEpisode;
            const char* trigger = movementStarted ? "movement" : (contactStarted ? "contact" : "grab");
            ROCK_LOG_INFO(
                Physics,
                "COLLIDER_CLOCK begin session={} trigger={} budgetFrames={} gameFrame={}",
                _colliderClockSession,
                trigger,
                kTraceFramesPerEpisode,
                frame.gameFrameIndex);
        }
        if (_colliderClockFramesRemaining == 0 ||
            (_colliderClockHasLoggedFrame &&
                _colliderClockLastLoggedFrame == frame.gameFrameIndex)) {
            return;
        }
        _colliderClockLastLoggedFrame = frame.gameFrameIndex;
        _colliderClockHasLoggedFrame = true;
        --_colliderClockFramesRemaining;

        const auto currentFrikHand = [](const bool isLeft) {
            TrackedNodeFrame sample{};
            if (!frik_visual_authority::isAvailable()) {
                return sample;
            }
            const auto world = frik_visual_authority::getHandWorldTransform(
                frik_visual_authority::handFromBool(isLeft));
            if (finiteNiTransform(world)) {
                sample.world = world;
                sample.valid = true;
            }
            return sample;
        };
        const TrackedNodeFrame rightFrik = currentFrikHand(false);
        const TrackedNodeFrame leftFrik = currentFrikHand(true);
        const bool rightExternal =
            frik_visual_authority::hasPublishedExternalHandWorldTransform(
                frik_visual_authority::Hand::Right);
        const bool leftExternal =
            frik_visual_authority::hasPublishedExternalHandWorldTransform(
                frik_visual_authority::Hand::Left);
        const auto rightController =
            vrcf::VRControllers.getPollSnapshot(vrcf::Hand::Right);
        const auto leftController =
            vrcf::VRControllers.getPollSnapshot(vrcf::Hand::Left);

        ROCK_LOG_INFO(
            Input,
            "COLLIDER_CLOCK input session={} frame={} controllerR(valid/packet/previous/changed/buttons)={}/{}/{}/{}/0x{:016X} controllerL(valid/packet/previous/changed/buttons)={}/{}/{}/{}/0x{:016X} axesR[0..4]=({:.3f},{:.3f})/({:.3f},{:.3f})/({:.3f},{:.3f})/({:.3f},{:.3f})/({:.3f},{:.3f}) axesL[0..4]=({:.3f},{:.3f})/({:.3f},{:.3f})/({:.3f},{:.3f})/({:.3f},{:.3f})/({:.3f},{:.3f})",
            _colliderClockSession,
            frame.gameFrameIndex,
            rightController.valid,
            rightController.packetNumber,
            rightController.previousPacketNumber,
            rightController.packetChanged,
            rightController.buttonsPressed,
            leftController.valid,
            leftController.packetNumber,
            leftController.previousPacketNumber,
            leftController.packetChanged,
            leftController.buttonsPressed,
            rightController.axisX[0],
            rightController.axisY[0],
            rightController.axisX[1],
            rightController.axisY[1],
            rightController.axisX[2],
            rightController.axisY[2],
            rightController.axisX[3],
            rightController.axisY[3],
            rightController.axisX[4],
            rightController.axisY[4],
            leftController.axisX[0],
            leftController.axisY[0],
            leftController.axisX[1],
            leftController.axisY[1],
            leftController.axisX[2],
            leftController.axisY[2],
            leftController.axisX[3],
            leftController.axisY[3],
            leftController.axisX[4],
            leftController.axisY[4]);

        ROCK_LOG_INFO(
            Physics,
            "COLLIDER_CLOCK frame session={} frame={} gameDt={:.6f} player(valid/moving/source)={}/{}/{} playerDelta=({:.3f},{:.3f},{:.3f}) playerWorld=({:.3f},{:.3f},{:.3f}) nodes(wandR/wandL/driverR/driverL)={}/{}/{}/{} wandR=({:.3f},{:.3f},{:.3f}) wandL=({:.3f},{:.3f},{:.3f}) driverR=({:.3f},{:.3f},{:.3f}) driverL=({:.3f},{:.3f},{:.3f})",
            _colliderClockSession,
            frame.gameFrameIndex,
            frame.deltaSeconds,
            runtime.playerSpace.valid,
            playerMoving,
            runtime.playerSpace.source ? runtime.playerSpace.source : "none",
            runtime.playerSpace.deltaGameUnits.x,
            runtime.playerSpace.deltaGameUnits.y,
            runtime.playerSpace.deltaGameUnits.z,
            runtime.playerSpace.world.translate.x,
            runtime.playerSpace.world.translate.y,
            runtime.playerSpace.world.translate.z,
            frame.rightWand.valid,
            frame.leftWand.valid,
            frame.rightWeaponDriver.valid,
            frame.leftWeaponDriver.valid,
            frame.rightWand.world.translate.x,
            frame.rightWand.world.translate.y,
            frame.rightWand.world.translate.z,
            frame.leftWand.world.translate.x,
            frame.leftWand.world.translate.y,
            frame.leftWand.world.translate.z,
            frame.rightWeaponDriver.world.translate.x,
            frame.rightWeaponDriver.world.translate.y,
            frame.rightWeaponDriver.world.translate.z,
            frame.leftWeaponDriver.world.translate.x,
            frame.leftWeaponDriver.world.translate.y,
            frame.leftWeaponDriver.world.translate.z);

        const bool weaponPresentedValid =
            weaponNode && finiteNiTransform(weaponNode->world);
        const RE::NiTransform weaponPresented =
            weaponPresentedValid ? weaponNode->world : RE::NiTransform{};
        const TransformDelta weaponRequestedToLive = weaponValid ?
            measureTransformDelta(
                weapon.requestedWeaponWorld,
                weapon.liveWeaponWorld) :
            TransformDelta{ -1.0f, -1.0f };
        const TransformDelta weaponResolvedToPresented =
            weaponValid && weaponPresentedValid ?
            measureTransformDelta(
                weapon.resolvedWeaponWorld,
                weaponPresented) :
            TransformDelta{ -1.0f, -1.0f };
        const std::uint64_t weaponSourceAgeFrames =
            weaponValid && frame.gameFrameIndex >= weapon.sourceGameFrameIndex ?
            frame.gameFrameIndex - weapon.sourceGameFrameIndex :
            0;
        ROCK_LOG_INFO(
            Weapon,
            "COLLIDER_CLOCK weapon session={} frame={} valid={} proxy={} contact={} visualCorrection={} source(frame/age/queue/solve)={}/{}/{}/{} physics(raw/sub/rem/accum/index/count/progress)=({:.6f}/{:.6f}/{:.6f}/{:.6f}/{}/{}/{:.3f}) requested=({:.3f},{:.3f},{:.3f}) live=({:.3f},{:.3f},{:.3f}) resolved=({:.3f},{:.3f},{:.3f}) presented(valid/pos)={}/({:.3f},{:.3f},{:.3f}) requestedToLive=({:.3f}gu,{:.3f}deg) resolvedToPresented=({:.3f}gu,{:.3f}deg)",
            _colliderClockSession,
            frame.gameFrameIndex,
            weaponValid,
            weapon.valid,
            weapon.contactActive,
            weapon.visualCorrectionActive,
            weapon.sourceGameFrameIndex,
            weaponSourceAgeFrames,
            weapon.sourceQueueSequence,
            weapon.solveSequence,
            weapon.physicsRawDeltaSeconds,
            weapon.physicsSubstepDeltaSeconds,
            weapon.physicsRemainderDeltaSeconds,
            weapon.physicsAccumulatedDeltaSeconds,
            weapon.physicsSubstepIndex,
            weapon.physicsSubstepCount,
            weapon.physicsSubstepProgress,
            weapon.requestedWeaponWorld.translate.x,
            weapon.requestedWeaponWorld.translate.y,
            weapon.requestedWeaponWorld.translate.z,
            weapon.liveWeaponWorld.translate.x,
            weapon.liveWeaponWorld.translate.y,
            weapon.liveWeaponWorld.translate.z,
            weapon.resolvedWeaponWorld.translate.x,
            weapon.resolvedWeaponWorld.translate.y,
            weapon.resolvedWeaponWorld.translate.z,
            weaponPresentedValid,
            weaponPresented.translate.x,
            weaponPresented.translate.y,
            weaponPresented.translate.z,
            weaponRequestedToLive.position,
            weaponRequestedToLive.rotationDegrees,
            weaponResolvedToPresented.position,
            weaponResolvedToPresented.rotationDegrees);

        const auto logHand = [&](const bool isLeft) {
            const std::size_t handIndex = isLeft ? 1u : 0u;
            const auto& handInput = isLeft ? frame.left : frame.right;
            const auto& wand = isLeft ? frame.leftWand : frame.rightWand;
            const auto& driver = isLeft ? frame.leftWeaponDriver : frame.rightWeaponDriver;
            const auto& frik = isLeft ? leftFrik : rightFrik;
            const bool external = isLeft ? leftExternal : rightExternal;
            const auto& hand = hands.hands[handIndex];
            const auto& palm = hand.twins[dynamic_hand_collision_telemetry::kPalmSlot];
            const TransformDelta rawToWand =
                !handInput.disabled && wand.valid ?
                measureTransformDelta(handInput.rawHandWorld, wand.world) :
                TransformDelta{ -1.0f, -1.0f };
            const TransformDelta wandToDriver =
                wand.valid && driver.valid ?
                measureTransformDelta(wand.world, driver.world) :
                TransformDelta{ -1.0f, -1.0f };
            const TransformDelta rawToFrik =
                !handInput.disabled && frik.valid ?
                measureTransformDelta(handInput.rawHandWorld, frik.world) :
                TransformDelta{ -1.0f, -1.0f };
            const std::uint64_t sourceAgeFrames =
                palm.physicsSampleValid &&
                    frame.gameFrameIndex >= palm.sourceGameFrameIndex ?
                frame.gameFrameIndex - palm.sourceGameFrameIndex :
                0;

            ROCK_LOG_INFO(
                Hand,
                "COLLIDER_CLOCK hand session={} frame={} side={} snapshot={} disabled={} contact={} visual(active/external)={}/{} source(frame/age/queue/solve/seqlock)={}/{}/{}/{}/{} physics(raw/sub/rem/accum/index/count/progress)=({:.6f}/{:.6f}/{:.6f}/{:.6f}/{}/{}/{:.3f}) raw=({:.3f},{:.3f},{:.3f}) palm(published/requested/commanded/live)=({:.3f},{:.3f},{:.3f})/({:.3f},{:.3f},{:.3f})/({:.3f},{:.3f},{:.3f})/({:.3f},{:.3f},{:.3f}) frik(valid/pos)={}/({:.3f},{:.3f},{:.3f}) appliedDeviation=({:.3f},{:.3f},{:.3f}) deltas(rawToWand/wandToDriver/rawToFrik/requestedToLive)=({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)/{:.3f}gu",
                _colliderClockSession,
                frame.gameFrameIndex,
                isLeft ? "L" : "R",
                handsValid,
                handInput.disabled,
                palm.contactActive,
                hand.visualActive,
                external,
                palm.sourceGameFrameIndex,
                sourceAgeFrames,
                palm.sourceQueueSequence,
                palm.solveSequence,
                palm.physicsSampleSequence,
                palm.physicsRawDeltaSeconds,
                palm.physicsDeltaSeconds,
                palm.physicsRemainderDeltaSeconds,
                palm.physicsAccumulatedDeltaSeconds,
                palm.physicsSubstepIndex,
                palm.physicsSubstepCount,
                palm.physicsSubstepProgress,
                handInput.rawHandWorld.translate.x,
                handInput.rawHandWorld.translate.y,
                handInput.rawHandWorld.translate.z,
                palm.publishedTargetWorld.translate.x,
                palm.publishedTargetWorld.translate.y,
                palm.publishedTargetWorld.translate.z,
                palm.requestedTargetWorldGame.x,
                palm.requestedTargetWorldGame.y,
                palm.requestedTargetWorldGame.z,
                palm.commandedTargetWorldGame.x,
                palm.commandedTargetWorldGame.y,
                palm.commandedTargetWorldGame.z,
                palm.liveBodyWorldGame.x,
                palm.liveBodyWorldGame.y,
                palm.liveBodyWorldGame.z,
                frik.valid,
                frik.world.translate.x,
                frik.world.translate.y,
                frik.world.translate.z,
                hand.appliedVisualDeviationWorldGame.x,
                hand.appliedVisualDeviationWorldGame.y,
                hand.appliedVisualDeviationWorldGame.z,
                rawToWand.position,
                rawToWand.rotationDegrees,
                wandToDriver.position,
                wandToDriver.rotationDegrees,
                rawToFrik.position,
                rawToFrik.rotationDegrees,
                palm.requestedGapGameUnits);
        };
        logHand(false);
        logHand(true);

        const auto logGrab = [&](const bool isLeft) {
            const std::size_t handIndex = isLeft ? 1u : 0u;
            Hand& heldHand = isLeft ? _leftHand : _rightHand;
            const auto& handInput = isLeft ? frame.left : frame.right;
            const auto& frik = isLeft ? leftFrik : rightFrik;
            auto& previous = _grabLocomotionClockStates[handIndex];
            if (!frame.hknpWorld || handInput.disabled || !heldHand.isHolding()) {
                previous = {};
                return;
            }

            GrabOverlayPointProbeSample applied{};
            const bool appliedValid =
                heldHand.tryGetGrabOverlayPointProbeSample(frame.hknpWorld, applied);
            grab_transform_telemetry::RuntimeSample sample{};
            const bool sampleValid = heldHand.getGrabTransformTelemetrySnapshot(
                frame.hknpWorld,
                handInput.rawHandWorld,
                sample);
            const bool consecutive =
                previous.active &&
                previous.session == _colliderClockSession &&
                frame.gameFrameIndex == previous.frame + 1;
            const auto stepDelta = [&](const bool currentValid,
                                       const bool previousValid,
                                       const RE::NiTransform& current,
                                       const RE::NiTransform& prior) {
                return consecutive && currentValid && previousValid ?
                    measureTransformDelta(prior, current) :
                    TransformDelta{ -1.0f, -1.0f };
            };

            const TransformDelta rawStep =
                consecutive ?
                measureTransformDelta(previous.rawHandWorld, handInput.rawHandWorld) :
                TransformDelta{ -1.0f, -1.0f };
            const TransformDelta targetStep = stepDelta(
                appliedValid,
                previous.hasAppliedProxyTarget,
                applied.appliedProxyTargetWorld,
                previous.appliedProxyTargetWorld);
            const TransformDelta proxyStep = stepDelta(
                sampleValid && sample.hasProxyReadback,
                previous.hasProxyReadback,
                sample.proxyReadbackWorld,
                previous.proxyReadbackWorld);
            const TransformDelta bodyStep = stepDelta(
                sampleValid && sample.hasHeldBodyWorld,
                previous.hasHeldBody,
                sample.heldBodyWorld,
                previous.heldBodyWorld);
            const TransformDelta nodeStep = stepDelta(
                sampleValid && sample.hasHeldNodeWorld,
                previous.hasHeldNode,
                sample.heldNodeWorld,
                previous.heldNodeWorld);
            const TransformDelta visualStep = stepDelta(
                sampleValid && sample.hasHeldRelativeHandTarget,
                previous.hasHeldRelativeHandTarget,
                sample.heldRelativeHandTargetWorld,
                previous.heldRelativeHandTargetWorld);
            const TransformDelta frikStep = stepDelta(
                frik.valid,
                previous.hasFrikHand,
                frik.world,
                previous.frikHandWorld);

            const TransformDelta rawToApplied = appliedValid ?
                measureTransformDelta(
                    handInput.rawHandWorld,
                    applied.appliedRawHandWorld) :
                TransformDelta{ -1.0f, -1.0f };
            const TransformDelta rawToQueued = appliedValid ?
                measureTransformDelta(
                    handInput.rawHandWorld,
                    applied.queuedRawHandWorld) :
                TransformDelta{ -1.0f, -1.0f };
            const TransformDelta queuedToApplied = appliedValid ?
                measureTransformDelta(
                    applied.queuedRawHandWorld,
                    applied.appliedRawHandWorld) :
                TransformDelta{ -1.0f, -1.0f };
            const TransformDelta targetToProxy =
                appliedValid && sampleValid && sample.hasProxyReadback ?
                measureTransformDelta(
                    applied.appliedProxyTargetWorld,
                    sample.proxyReadbackWorld) :
                TransformDelta{ -1.0f, -1.0f };
            const TransformDelta desiredToBody =
                sampleValid && sample.hasGrabStartFrames && sample.hasHeldBodyWorld ?
                measureTransformDelta(
                    sample.currentRawDesiredBodyWorld,
                    sample.heldBodyWorld) :
                TransformDelta{ -1.0f, -1.0f };
            const TransformDelta bodyDerivedToNode =
                sampleValid && sample.hasHeldBodyDerivedNodeWorld && sample.hasHeldNodeWorld ?
                measureTransformDelta(
                    sample.heldBodyDerivedNodeWorld,
                    sample.heldNodeWorld) :
                TransformDelta{ -1.0f, -1.0f };
            const TransformDelta visualToFrik =
                sampleValid && sample.hasHeldRelativeHandTarget && frik.valid ?
                measureTransformDelta(
                    sample.heldRelativeHandTargetWorld,
                    frik.world) :
                TransformDelta{ -1.0f, -1.0f };
            const TransformDelta rawToVisual =
                sampleValid && sample.hasHeldRelativeHandTarget ?
                measureTransformDelta(
                    handInput.rawHandWorld,
                    sample.heldRelativeHandTargetWorld) :
                TransformDelta{ -1.0f, -1.0f };
            const std::uint64_t sourceAgeFrames =
                appliedValid && applied.sourceGameFrameIndex != 0 &&
                    frame.gameFrameIndex >= applied.sourceGameFrameIndex ?
                frame.gameFrameIndex - applied.sourceGameFrameIndex :
                0;

            ROCK_LOG_INFO(
                Hand,
                "GRAB_LOCOMOTION drive session={} frame={} side={} valid(applied/sample/proxy/body)={}/{}/{}/{} form=0x{:08X} bodies(proxy/held)={}/{} source(frame/age/queue/pending/flush/after)={}/{}/{}/{}/{}/{} flushPhysics(valid/raw/sub/rem/accum/index/count/progress)={}/({:.6f}/{:.6f}/{:.6f}/{:.6f}/{}/{}/{:.3f}) afterPhysics(valid/raw/sub/rem/accum/index/count/progress)={}/({:.6f}/{:.6f}/{:.6f}/{:.6f}/{}/{}/{:.3f}) consumptionRebase(status/shift/controller/vtable/scaleRev)={}/({:.3f},{:.3f},{:.3f})/{:016X}:{:016X}/{:016X}:{:016X}/{}:{} raw=({:.3f},{:.3f},{:.3f}) queuedRaw=({:.3f},{:.3f},{:.3f}) appliedRaw=({:.3f},{:.3f},{:.3f}) target=({:.3f},{:.3f},{:.3f}) proxy=({:.3f},{:.3f},{:.3f}) desiredBody=({:.3f},{:.3f},{:.3f}) body=({:.3f},{:.3f},{:.3f}) gaps(rawToQueued/queuedToApplied/rawToApplied/targetToProxy/desiredToBody)=({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg) steps(raw/target/proxy/body)=({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)",
                _colliderClockSession,
                frame.gameFrameIndex,
                isLeft ? "L" : "R",
                appliedValid,
                sampleValid,
                sample.hasProxyReadback,
                sample.hasHeldBodyWorld,
                sample.heldFormId,
                applied.proxyBodyId.value,
                applied.objectBodyId.value,
                applied.sourceGameFrameIndex,
                sourceAgeFrames,
                applied.sourceQueueSequence,
                applied.pendingQueueSequence,
                applied.flushSequence,
                applied.afterSolveSequence,
                applied.flushTiming.valid,
                applied.flushTiming.rawDeltaSeconds,
                applied.flushTiming.substepDeltaSeconds,
                applied.flushTiming.remainderDeltaSeconds,
                applied.flushTiming.accumulatedDeltaSeconds,
                applied.flushTiming.substepIndex,
                applied.flushTiming.substepCount,
                applied.flushTiming.substepProgress,
                applied.afterSolveTiming.valid,
                applied.afterSolveTiming.rawDeltaSeconds,
                applied.afterSolveTiming.substepDeltaSeconds,
                applied.afterSolveTiming.remainderDeltaSeconds,
                applied.afterSolveTiming.accumulatedDeltaSeconds,
                applied.afterSolveTiming.substepIndex,
                applied.afterSolveTiming.substepCount,
                applied.afterSolveTiming.substepProgress,
                grab_authority_source_clock::consumptionFrameRebaseStatusName(applied.consumptionFrameRebaseStatus),
                applied.consumptionFrameShiftGame.x,
                applied.consumptionFrameShiftGame.y,
                applied.consumptionFrameShiftGame.z,
                applied.sourceControllerIdentity,
                applied.consumptionControllerIdentity,
                applied.sourceControllerVtable,
                applied.consumptionControllerVtable,
                applied.sourcePhysicsScaleRevision,
                applied.consumptionPhysicsScaleRevision,
                handInput.rawHandWorld.translate.x,
                handInput.rawHandWorld.translate.y,
                handInput.rawHandWorld.translate.z,
                applied.queuedRawHandWorld.translate.x,
                applied.queuedRawHandWorld.translate.y,
                applied.queuedRawHandWorld.translate.z,
                applied.appliedRawHandWorld.translate.x,
                applied.appliedRawHandWorld.translate.y,
                applied.appliedRawHandWorld.translate.z,
                applied.appliedProxyTargetWorld.translate.x,
                applied.appliedProxyTargetWorld.translate.y,
                applied.appliedProxyTargetWorld.translate.z,
                sample.proxyReadbackWorld.translate.x,
                sample.proxyReadbackWorld.translate.y,
                sample.proxyReadbackWorld.translate.z,
                sample.currentRawDesiredBodyWorld.translate.x,
                sample.currentRawDesiredBodyWorld.translate.y,
                sample.currentRawDesiredBodyWorld.translate.z,
                sample.heldBodyWorld.translate.x,
                sample.heldBodyWorld.translate.y,
                sample.heldBodyWorld.translate.z,
                rawToQueued.position,
                rawToQueued.rotationDegrees,
                queuedToApplied.position,
                queuedToApplied.rotationDegrees,
                rawToApplied.position,
                rawToApplied.rotationDegrees,
                targetToProxy.position,
                targetToProxy.rotationDegrees,
                desiredToBody.position,
                desiredToBody.rotationDegrees,
                rawStep.position,
                rawStep.rotationDegrees,
                targetStep.position,
                targetStep.rotationDegrees,
                proxyStep.position,
                proxyStep.rotationDegrees,
                bodyStep.position,
                bodyStep.rotationDegrees);

            ROCK_LOG_INFO(
                Hand,
                "GRAB_LOCOMOTION visual session={} frame={} side={} valid(bodyDerived/node/heldHand/frik)={}/{}/{}/{} bodyDerived=({:.3f},{:.3f},{:.3f}) node=({:.3f},{:.3f},{:.3f}) heldHand=({:.3f},{:.3f},{:.3f}) frik=({:.3f},{:.3f},{:.3f}) gaps(bodyDerivedToNode/heldHandToFrik/rawToHeldHand)=({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg) steps(node/heldHand/frik)=({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)",
                _colliderClockSession,
                frame.gameFrameIndex,
                isLeft ? "L" : "R",
                sample.hasHeldBodyDerivedNodeWorld,
                sample.hasHeldNodeWorld,
                sample.hasHeldRelativeHandTarget,
                frik.valid,
                sample.heldBodyDerivedNodeWorld.translate.x,
                sample.heldBodyDerivedNodeWorld.translate.y,
                sample.heldBodyDerivedNodeWorld.translate.z,
                sample.heldNodeWorld.translate.x,
                sample.heldNodeWorld.translate.y,
                sample.heldNodeWorld.translate.z,
                sample.heldRelativeHandTargetWorld.translate.x,
                sample.heldRelativeHandTargetWorld.translate.y,
                sample.heldRelativeHandTargetWorld.translate.z,
                frik.world.translate.x,
                frik.world.translate.y,
                frik.world.translate.z,
                bodyDerivedToNode.position,
                bodyDerivedToNode.rotationDegrees,
                visualToFrik.position,
                visualToFrik.rotationDegrees,
                rawToVisual.position,
                rawToVisual.rotationDegrees,
                nodeStep.position,
                nodeStep.rotationDegrees,
                visualStep.position,
                visualStep.rotationDegrees,
                frikStep.position,
                frikStep.rotationDegrees);

            previous.rawHandWorld = handInput.rawHandWorld;
            previous.appliedProxyTargetWorld = applied.appliedProxyTargetWorld;
            previous.proxyReadbackWorld = sample.proxyReadbackWorld;
            previous.heldBodyWorld = sample.heldBodyWorld;
            previous.heldNodeWorld = sample.heldNodeWorld;
            previous.heldRelativeHandTargetWorld = sample.heldRelativeHandTargetWorld;
            previous.frikHandWorld = frik.world;
            previous.session = _colliderClockSession;
            previous.frame = frame.gameFrameIndex;
            previous.active = true;
            previous.hasAppliedProxyTarget = appliedValid;
            previous.hasProxyReadback = sampleValid && sample.hasProxyReadback;
            previous.hasHeldBody = sampleValid && sample.hasHeldBodyWorld;
            previous.hasHeldNode = sampleValid && sample.hasHeldNodeWorld;
            previous.hasHeldRelativeHandTarget =
                sampleValid && sample.hasHeldRelativeHandTarget;
            previous.hasFrikHand = frik.valid;
        };
        logGrab(false);
        logGrab(true);

        if (_colliderClockFramesRemaining == 0) {
            ROCK_LOG_INFO(
                Physics,
                "COLLIDER_CLOCK end session={} reason=budget gameFrame={}",
                _colliderClockSession,
                frame.gameFrameIndex);
        }
    }

}
