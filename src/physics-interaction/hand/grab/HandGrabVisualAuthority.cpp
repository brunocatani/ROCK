#include "physics-interaction/hand/Hand.h"

/*
 * The VISUAL half of a grab: the hand the player sees, as distinct from the
 * physics hand body that drives the object.
 *
 * Two jobs live here. The visual RETURN state machine blends the rendered hand
 * back to its skeleton pose after a release. refreshGrabVisualAuthorityBeforeFrik
 * runs in the pre-FRIK window each frame and republishes the held pair before
 * FRIK's own solve overwrites it.
 *
 * That pre-FRIK publish and the held-update publish in HandGrabHeldUpdate.cpp
 * MUST write the scene graph the same way. Both call the single inline
 * applyHeldVisualNodeWorldTransform in HandGrabVisualDetail.h for that reason.
 */

#include "physics-interaction/hand/grab/HandGrabMath.h"
#include "physics-interaction/hand/grab/HandGrabTrace.h"
#include "physics-interaction/hand/grab/HandGrabVisualDetail.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/debug/GrabClockDebugFeed.h"
#include "physics-interaction/native/SceneWriterProbe.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/visual/PreFrikHandAuthorityPolicy.h"
#include "RockConfig.h"

#include <algorithm>
#include <cmath>

namespace rock
{
    using namespace hand_grab_detail;

    namespace
    {
        // Reject teleport and recenter deltas from same-frame pair transport.
        constexpr float kMaxPreFrikGrabPairTransportGameUnits = 64.0f;
    }

    void Hand::beginGrabVisualReturn()
    {
        if (!g_rockConfig.rockGrabHandReturnEnabled ||
            !_hasLastPublishedGrabVisualHandTransform ||
            !isUsableGrabVisualTransform(_lastPublishedGrabVisualHandTransform) ||
            !frik_visual_authority::isAvailable()) {
            clearGrabVisualReturn("release-not-eligible", false);
            return;
        }
    
        _grabVisualReturn.begin(_lastPublishedGrabVisualHandTransform);
        if (!applyGrabReturnHandWorldTransform(_isLeft, _grabVisualReturn.start)) {
            _grabVisualReturn.clear();
            clearGrabReturnHandWorldTransform(_isLeft);
            ROCK_LOG_WARN(Hand, "{} hand visual return start failed; restoring tracked authority immediately", handName());
            return;
        }
    
        ROCK_LOG_DEBUG(Hand,
            "{} hand visual return started from=({:.2f},{:.2f},{:.2f})",
            handName(),
            _grabVisualReturn.start.translate.x,
            _grabVisualReturn.start.translate.y,
            _grabVisualReturn.start.translate.z);
    }
    
    void Hand::updateGrabVisualReturn(
        const RE::NiTransform& trackedHandWorld,
        float deltaTime,
        const std::uint64_t sourceSchedulerSequence)
    {
        if (!_grabVisualReturn.active) {
            return;
        }
        if (!runtime_state::isLocalSkeletonReady() ||
            !frik_visual_authority::isAvailable() ||
            !isUsableGrabVisualTransform(trackedHandWorld)) {
            clearGrabVisualReturn("tracked-hand-unavailable", true);
            return;
        }
    
        const bool timingPending = !_grabVisualReturn.durationInitialized;
        const float initialDistance = timingPending ?
            hand_visual_lerp_math::distanceGameUnits(_grabVisualReturn.start.translate, trackedHandWorld.translate) :
            0.0f;
        const float initialAngleDegrees = timingPending ?
            hand_visual_lerp_math::rotationDistanceDegrees(_grabVisualReturn.start, trackedHandWorld) :
            0.0f;
        const auto result = hand_visual_lerp_math::advanceVisualReturn(
            _grabVisualReturn,
            trackedHandWorld,
            deltaTime,
            hand_visual_lerp_math::VisualReturnConfig{
                .minSeconds = g_rockConfig.rockGrabHandReturnTimeMin,
                .maxSeconds = g_rockConfig.rockGrabHandReturnTimeMax,
                .minDistanceGameUnits = g_rockConfig.rockGrabHandReturnMinDistance,
                .maxDistanceGameUnits = g_rockConfig.rockGrabHandReturnMaxDistance,
                .minAngleDegrees = g_rockConfig.rockGrabHandReturnMinAngleDegrees,
                .maxAngleDegrees = g_rockConfig.rockGrabHandReturnMaxAngleDegrees,
            });
        if (timingPending) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand visual return timing distance={:.2f}gu angle={:.1f}deg duration={:.3f}s",
                handName(),
                initialDistance,
                initialAngleDegrees,
                _grabVisualReturn.durationSeconds);
        }
        if (!isUsableGrabVisualTransform(result.transform) ||
            !applyGrabReturnHandWorldTransform(_isLeft, result.transform)) {
            clearGrabVisualReturn("publish-failed", true);
            return;
        }
    
        _preFrikGrabReturnAuthority.rawHandToTargetLocal =
            prefrik_hand_authority_policy::captureDriverToTargetLocal(
                trackedHandWorld,
                result.transform);
        _preFrikGrabReturnAuthority.sourceSchedulerSequence =
            sourceSchedulerSequence;
        _preFrikGrabReturnAuthority.valid =
            sourceSchedulerSequence != 0 &&
            prefrik_hand_authority_policy::isUsableTransform(
                _preFrikGrabReturnAuthority.rawHandToTargetLocal);
    
        if (!result.reachedTarget) {
            return;
        }
    
        const float completedDuration = _grabVisualReturn.durationSeconds;
        clearGrabReturnHandWorldTransform(_isLeft);
        _grabVisualReturn.clear();
        _preFrikGrabReturnAuthority.clear();
        ROCK_LOG_DEBUG(Hand, "{} hand visual return completed duration={:.3f}s", handName(), completedDuration);
    }
    
    void Hand::clearGrabVisualReturn(const char* reason, bool logCancellation)
    {
        const bool wasActive = _grabVisualReturn.active;
        clearGrabReturnHandWorldTransform(_isLeft);
        _grabVisualReturn.clear();
        _preFrikGrabReturnAuthority.clear();
        if (wasActive && logCancellation) {
            ROCK_LOG_DEBUG(Hand, "{} hand visual return cancelled reason={}", handName(), reason ? reason : "unknown");
        }
    }
    
    void Hand::refreshGrabVisualAuthorityBeforeFrik(
        const std::uint64_t schedulerSequence,
        const bool rawHandValid,
        const RE::NiTransform& rawHandWorld)
    {
        const auto physicalHand = handFromBool(_isLeft);
        const bool grabTagPublished =
            frik_visual_authority::hasPublishedExternalHandWorldTransform(
                kGrabExternalHandTag,
                physicalHand);
        if (!grabTagPublished) {
            _preFrikGrabVisualAuthority.clear();
        } else {
            const bool sourceOwned =
                _preFrikGrabVisualAuthority.valid &&
                prefrik_hand_authority_policy::isImmediateSuccessor(
                    _preFrikGrabVisualAuthority.sourceSchedulerSequence,
                    schedulerSequence) &&
                isHolding() &&
                _savedObjectState.bodyId.value ==
                    _preFrikGrabVisualAuthority.heldBodyId &&
                _activeConstraint.isValid() &&
                _activeConstraint.constraintId ==
                    _preFrikGrabVisualAuthority.constraintId &&
                _preFrikGrabVisualAuthority.heldNode != nullptr;
            const RE::NiTransform heldNodeWorld = sourceOwned ?
                _preFrikGrabVisualAuthority.heldNode->world :
                RE::NiTransform{};
            const RE::NiTransform refreshedHandWorld = sourceOwned ?
                prefrik_hand_authority_policy::reconstructTargetWorld(
                    heldNodeWorld,
                    _preFrikGrabVisualAuthority.heldNodeToHandLocal) :
                RE::NiTransform{};
    
            // ANCHOR_CLOCK probe entry state: sampled before any pre-FRIK
            // write so heldVsLastWrite stays a true engine-stomp meter
            // against the producer's write, and rawVsProducer/roomVsProducer
            // measure the engine's mid-frame root/skeleton refresh.
            const auto anchorProbeRoom = sampleAnchorClockRoom();
            const float anchorProbeHeldVsLastWrite = (sourceOwned && _hasGrabProbeLastAnchorWrite) ?
                pointDistanceGameUnits(heldNodeWorld.translate, _grabProbeLastAnchorWrite.translate) :
                -1.0f;
            const float anchorProbeRawVsProducer = (rawHandValid && _hasGrabProbeProducerSample) ?
                pointDistanceGameUnits(rawHandWorld.translate, _grabProbeProducerHandPos) :
                -1.0f;
            const float anchorProbeRoomVsProducer = (anchorProbeRoom.valid && _hasGrabProbeProducerSample) ?
                pointDistanceGameUnits(anchorProbeRoom.position, _grabProbeProducerRoomPos) :
                -1.0f;
    
            /*
             * Fresh-clock pair transport (2026-08-17 stick-locomotion buzz):
             * the engine applies stick locomotion/turn mid-frame, AFTER the
             * producer wrote the held pair from the pre-move hand (measured:
             * raw/room move >1gu between producer and pre-FRIK only under
             * stick input). Republishing the FRIK hand from the stale held
             * node anchored the rendered pair one locomotion step behind the
             * camera, with per-frame variation showing as the high-frequency
             * held-object buzz. While ROCK owns the rendered node this frame,
             * carry the rigid hand+object pair onto the freshly reconstructed
             * raw hand instead, so pair and camera render on the same clock.
             * Fails closed to the held-node republish when the fresh hand is
             * missing or the intra-frame delta is implausible (teleport /
             * recenter guard).
             */
            bool freshPairApplied = false;
            float intraFrameHandMotionGameUnits = -1.0f;
            RE::NiTransform publishedHandWorld = refreshedHandWorld;
            if (sourceOwned &&
                _preFrikGrabVisualAuthority.renderClockNodeOwned &&
                rawHandValid &&
                prefrik_hand_authority_policy::isUsableTransform(rawHandWorld)) {
                intraFrameHandMotionGameUnits =
                    prefrik_hand_authority_policy::translationDeltaGameUnits(
                        rawHandWorld,
                        _preFrikGrabVisualAuthority.sourceRawHandWorld);
                if (intraFrameHandMotionGameUnits <=
                        kMaxPreFrikGrabPairTransportGameUnits) {
                    const RE::NiTransform freshHeldWorld =
                        prefrik_hand_authority_policy::reconstructTargetWorld(
                            rawHandWorld,
                            _preFrikGrabVisualAuthority.rawHandToHeldLocal);
                    const RE::NiTransform freshHandWorld =
                        prefrik_hand_authority_policy::reconstructTargetWorld(
                            rawHandWorld,
                            _preFrikGrabVisualAuthority.rawHandToHandLocal);
                    if (prefrik_hand_authority_policy::isUsableTransform(
                            freshHeldWorld) &&
                        prefrik_hand_authority_policy::isUsableTransform(
                            freshHandWorld) &&
                        applyGrabExternalHandWorldTransform(
                            _isLeft,
                            freshHandWorld)) {
                        // Render-consumption probe counterpart: -Z here vs
                        // +Z at the producer write (node only, diagnostic).
                        RE::NiTransform preFrikNodeWritePose = freshHeldWorld;
                        const float renderProbeOffset =
                            g_rockConfig.rockGrabRenderClockProbeOffsetGameUnits;
                        if (renderProbeOffset != 0.0f) {
                            preFrikNodeWritePose.translate.z -= renderProbeOffset;
                        }
                        applyHeldVisualNodeWorldTransform(
                            _preFrikGrabVisualAuthority.heldNode.get(),
                            preFrikNodeWritePose);
                        _grabProbeLastAnchorWrite = preFrikNodeWritePose;
                        _hasGrabProbeLastAnchorWrite = true;
                        _lastPublishedGrabVisualHandTransform = freshHandWorld;
                        _hasLastPublishedGrabVisualHandTransform = true;
                        publishedHandWorld = freshHandWorld;
                        freshPairApplied = true;
                        // Fresh-clock scene-writer anchor: same reconstructed
                        // node, carried through the producer's rigid
                        // node->body relation. This is the anchor the drawn
                        // object shares with the rendered hand.
                        if (g_rockConfig.rockGrabHeldScenePoseSync &&
                            _preFrikGrabVisualAuthority.hasHeldNodeToBodyAnchorLocal) {
                            const RE::NiTransform freshBodyAnchorWorld =
                                prefrik_hand_authority_policy::reconstructTargetWorld(
                                    freshHeldWorld,
                                    _preFrikGrabVisualAuthority.heldNodeToBodyAnchorLocal);
                            if (prefrik_hand_authority_policy::isUsableTransform(freshBodyAnchorWorld)) {
                                const auto preFrikSourceRoom = sampleAnchorClockRoom();
                                scene_writer_probe::AnchorRootSample preFrikRootSample{};
                                preFrikRootSample.roomPositionGame[0] = preFrikSourceRoom.position.x;
                                preFrikRootSample.roomPositionGame[1] = preFrikSourceRoom.position.y;
                                preFrikRootSample.roomPositionGame[2] = preFrikSourceRoom.position.z;
                                preFrikRootSample.roomYawRadians = preFrikSourceRoom.yawDegrees * 0.01745329252f;
                                preFrikRootSample.roomValid = preFrikSourceRoom.valid;
                                scene_writer_probe::publishHeldAnchor(
                                    _isLeft,
                                    freshBodyAnchorWorld,
                                    scene_writer_probe::AnchorStage::PreFrik,
                                    preFrikRootSample);
                            }
                        }
                    }
                }
            }
    
            if (sourceOwned) {
                ROCK_LOG_INFO(Hand,
                    "{} ANCHOR_CLOCK stage=preFrik seq={} room=({:.2f},{:.2f},{:.2f}) roomYaw={:.3f} rawHand=({:.2f},{:.2f},{:.2f}) rawVsProducer={:.3f}gu roomVsProducer={:.3f}gu heldNode=({:.2f},{:.2f},{:.2f}) heldVsLastWrite={:.3f}gu repubHand=({:.2f},{:.2f},{:.2f}) freshPair={} intraMotion={:.3f}gu",
                    handName(),
                    schedulerSequence,
                    anchorProbeRoom.position.x,
                    anchorProbeRoom.position.y,
                    anchorProbeRoom.position.z,
                    anchorProbeRoom.yawDegrees,
                    rawHandWorld.translate.x,
                    rawHandWorld.translate.y,
                    rawHandWorld.translate.z,
                    anchorProbeRawVsProducer,
                    anchorProbeRoomVsProducer,
                    heldNodeWorld.translate.x,
                    heldNodeWorld.translate.y,
                    heldNodeWorld.translate.z,
                    anchorProbeHeldVsLastWrite,
                    publishedHandWorld.translate.x,
                    publishedHandWorld.translate.y,
                    publishedHandWorld.translate.z,
                    freshPairApplied ? "yes" : "no",
                    intraFrameHandMotionGameUnits);
    
                rock::debug::RockGrabClockStagePreFrikV1 grabClockPreFrikSample{};
                grabClockPreFrikSample.schedulerSequence = schedulerSequence;
                assignGrabClockFeedVec(grabClockPreFrikSample.roomPos, anchorProbeRoom.position);
                grabClockPreFrikSample.roomYawDegrees = anchorProbeRoom.yawDegrees;
                grabClockPreFrikSample.roomValid = anchorProbeRoom.valid ? 1u : 0u;
                assignGrabClockFeedVec(grabClockPreFrikSample.rawHandPos, rawHandWorld.translate);
                grabClockPreFrikSample.rawHandValid = rawHandValid ? 1u : 0u;
                grabClockPreFrikSample.rawVsProducerGu = anchorProbeRawVsProducer;
                grabClockPreFrikSample.roomVsProducerGu = anchorProbeRoomVsProducer;
                assignGrabClockFeedVec(grabClockPreFrikSample.heldNodePos, heldNodeWorld.translate);
                grabClockPreFrikSample.heldVsLastWriteGu = anchorProbeHeldVsLastWrite;
                assignGrabClockFeedVec(grabClockPreFrikSample.republishedHandPos, publishedHandWorld.translate);
                grabClockPreFrikSample.freshPairApplied = freshPairApplied ? 1u : 0u;
                grabClockPreFrikSample.intraFrameHandMotionGu = intraFrameHandMotionGameUnits;
                rock::debug::publishGrabClockPreFrikStage(_isLeft, grabClockPreFrikSample);
            }
            if (!freshPairApplied &&
                (!sourceOwned ||
                    !prefrik_hand_authority_policy::isUsableTransform(
                        heldNodeWorld) ||
                    !prefrik_hand_authority_policy::isUsableTransform(
                        refreshedHandWorld) ||
                    !applyGrabExternalHandWorldTransform(
                        _isLeft,
                        refreshedHandWorld))) {
                clearGrabExternalHandWorldTransform(_isLeft);
                _preFrikGrabVisualAuthority.clear();
            }
        }
    
        const bool returnTagPublished =
            frik_visual_authority::hasPublishedExternalHandWorldTransform(
                kGrabReturnHandTag,
                physicalHand);
        if (!returnTagPublished) {
            _preFrikGrabReturnAuthority.clear();
            return;
        }
    
        const bool returnSourceOwned =
            _preFrikGrabReturnAuthority.valid &&
            _grabVisualReturn.active &&
            rawHandValid &&
            prefrik_hand_authority_policy::isImmediateSuccessor(
                _preFrikGrabReturnAuthority.sourceSchedulerSequence,
                schedulerSequence) &&
            prefrik_hand_authority_policy::isUsableTransform(rawHandWorld);
        const RE::NiTransform refreshedReturnWorld = returnSourceOwned ?
            prefrik_hand_authority_policy::reconstructTargetWorld(
                rawHandWorld,
                _preFrikGrabReturnAuthority.rawHandToTargetLocal) :
            RE::NiTransform{};
        if (!returnSourceOwned ||
            !prefrik_hand_authority_policy::isUsableTransform(
                refreshedReturnWorld) ||
            !applyGrabReturnHandWorldTransform(
                _isLeft,
                refreshedReturnWorld)) {
            clearGrabReturnHandWorldTransform(_isLeft);
            _preFrikGrabReturnAuthority.clear();
        }
    }
    
    void Hand::cancelGrabVisualReturn(const char* reason)
    {
        clearGrabVisualReturn(reason, true);
    }
}
