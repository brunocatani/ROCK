#include "physics-interaction/hand/Hand.h"

#include "physics-interaction/hand/grab/HandGrabAcquisitionContext.h"

/*
 * Grab ACQUISITION: everything between "the hand asked to grab" and "the hold is
 * live". Read this file top-down and you read one acquisition in the order it
 * happens. grabSelectedObject sits at the BOTTOM and is only the table of
 * contents that calls the phases above it, in sequence.
 *
 * The phases, in run order. The last three are large enough to own their own
 * files; they are still phases of this same sequence:
 *   prepareGrabBodySet          - guards, native body-set scan, lifecycle capture
 *   resolveGrabCaptureFrames    - palm proxy frame, basis delta, capture refresh
 *   extractGrabMesh             - triangles, authored node, surface-hit fallbacks
 *   resolvePrimaryGrabBody      - contact-source policy, skinned resolution
 *   resolveGrabPivotAuthority   - candidate assessment, mesh-backed resolve
 *   evaluateGrabContactEvidence - pinch pocket, multi-finger grip, accept/reject
 *   captureCanonicalGrabFrame   - HandGrabSeatCapture.cpp: seat solve, _grabFrame
 *   commitGrabDrive             - HandGrabCommit.cpp: suppression, drive, leases
 *   publishGrabFingerPose       - HandGrabCommit.cpp: attach or mesh finger pose
 *
 * All three share GrabAcquisitionContext, defined in
 * HandGrabAcquisitionContext.h. A phase communicates with the next only through
 * that struct; none of them keeps acquisition state of its own.
 *
 * EVERY failure exit goes through abortGrabAcquisition, which runs the COMPLETE
 * teardown superset. Do not add a bare "return false": a partial unwind leaves a
 * prepared body set, a suppressed collision lease, or a stale visual-authority
 * hand offset behind. abortGrabAcquisition shares clearAllGrabRuntimeState with
 * HandGrabRelease.cpp, so acquisition teardown and release teardown cannot drift.
 */
#include "physics-interaction/hand/grab/HandGrabInternal.h"
#include "physics-interaction/hand/grab/HandGrabMath.h"
#include "physics-interaction/hand/grab/HandGrabTrace.h"
#include "physics-interaction/hand/grab/HandGrabVisualDetail.h"
#include "physics-interaction/hand/grab/HandGrabContactEvidence.h"
#include "physics-interaction/hand/grab/HandGrabFingerPose.h"
#include "physics-interaction/hand/grab/HandGrabSupportModel.h"
#include "physics-interaction/hand/grab/HandGrabOffsetSources.h"
#include "physics-interaction/hand/grab/HandGrabBodySetRuntime.h"
#include "physics-interaction/hand/grab/HandGrabPivotAuthority.h"

#include "physics-interaction/body/BodyBoneColliderSet.h"
#include "physics-interaction/native/havok/HavokOffsets.h"

#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/native/CharacterControllerRuntime.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/debug/DebugMath.h"
#include "physics-interaction/debug/GrabClockDebugFeed.h"
#include "physics-interaction/hand/HeldBodyRenderPose.h"
#include "physics-interaction/native/SceneWriterProbe.h"
#include "physics-interaction/grenade/LooseGrenadeRuntime.h"
#include "physics-interaction/grab/GrabAuthorityProxy.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/grab/GrabConstraintMath.h"
#include "physics-interaction/grab/GrabContact.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/saved/SavedGrabOffsetStore.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/GrabMassPolicy.h"
#include "physics-interaction/grab/GrabMotionController.h"
#include "physics-interaction/grab/GrabNodeInfoMath.h"
#include "physics-interaction/grab/GrabNodeNamePolicy.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/object/MechanicalConnectedBodySet.h"
#include "physics-interaction/object/CarInteractionPolicy.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/weapon/authored_grip/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/LooseWeaponGripZone.h"
#include "physics-interaction/weapon/WeaponTypePolicy.h"
#include "physics-interaction/object/SkinnedBodyResolver.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/visual/PreFrikHandAuthorityPolicy.h"
#include "physics-interaction/hand/skeleton/HandFrame.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/core/PhysicsHooks.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/native/query/PhysicsShapeCast.h"
#include "physics-interaction/native/query/PhysicsRecursiveWrappers.h"
#include "physics-interaction/native/query/PhysicsUtils.h"
#include "physics-interaction/native/query/PhysicsScale.h"
#include "physics-interaction/native/havok/HavokMaterialRegistry.h"
#include "physics-interaction/native/havok/HavokRefCount.h"
#include "RE/Havok/hkVector4.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/NetImmerse/NiUpdateData.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "physics-interaction/TransformMath.h"
#include "rock_support/Fo4VrRuntime.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <format>
#include <algorithm>
#include <array>
#include <atomic>
#include <initializer_list>
#include <limits>
#include <string>
#include <string_view>
#include <xmmintrin.h>


namespace rock
{
    using namespace hand_grab_detail;

    void Hand::prepareActiveGrabBodySet(RE::bhkWorld* bhkWorld,
        RE::hknpWorld* world,
        const SelectedObject& selection,
        RE::NiAVObject* rootNode,
        bool skipActivePrep,
        bool captureBeforePrep,
        bool trackPreparedBodies,
        active_grab_body_lifecycle::BodyLifecycleSnapshot& lifecycle,
        ActiveGrabBodySetPrep& outPrep)
    {
        outPrep = {};
        const auto scanOptions = makeActiveGrabBodyScanOptions(selection);
        outPrep.seedBodyId = scanOptions.seedBodyId;

        // Read the original body set before native wrappers change motion state.
        outPrep.beforePrepScanCacheHit =
            tryUseGrabAcquisitionBeforePrepCache(bhkWorld, world, selection, scanOptions, outPrep.beforePrepBodySet);
        if (!outPrep.beforePrepScanCacheHit) {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabAcquisitionBodyScan);
            outPrep.beforePrepBodySet =
                object_physics_body_set::scanObjectPhysicsBodySet(bhkWorld, world, selection.refr, scanOptions);
        }
        if (captureBeforePrep) {
            lifecycle.captureBeforeActivePrep(outPrep.beforePrepBodySet);
        }

        // A peer-held object is already active. Do not run the native prep twice.
        if (!skipActivePrep) {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabAcquisitionActivePrep);
            outPrep.motionConverted = physics_recursive_wrappers::setMotionRecursive(
                rootNode,
                physics_recursive_wrappers::MotionPreset::Dynamic,
                true,
                true,
                true);
            outPrep.collisionEnabled = physics_recursive_wrappers::enableCollisionRecursive(rootNode, true, true, true);
        }

        if (skipActivePrep) {
            outPrep.preparedBodySet = outPrep.beforePrepBodySet;
            outPrep.preparedScanCacheHit = true;
            outPrep.preparedBodySetPostPrepComplete = true;
        } else {
            outPrep.preparedScanCacheHit = tryBuildGrabAcquisitionPreparedBodySetFromCache(
                bhkWorld,
                world,
                selection,
                scanOptions,
                outPrep.preparedBodySet,
                outPrep.preparedBodySetPostPrepComplete);
            if (!outPrep.preparedScanCacheHit || outPrep.preparedBodySet.acceptedCount() == 0) {
                outPrep.preparedScanCacheHit = false;
                outPrep.preparedBodySetPostPrepComplete = true;
                performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabAcquisitionBodyScan);
                outPrep.preparedBodySet =
                    object_physics_body_set::scanObjectPhysicsBodySet(bhkWorld, world, selection.refr, scanOptions);
            }
        }

        if (trackPreparedBodies) {
            lifecycle.markPreparedBodies(outPrep.preparedBodySet);
            if (outPrep.preparedScanCacheHit && !outPrep.preparedBodySetPostPrepComplete) {
                lifecycle.markIncompleteNativeScan();
            }
        }
        outPrep.driveDecision = classifyHeldBodySetDrive(
            outPrep.beforePrepBodySet,
            outPrep.preparedBodySet,
            lifecycle.hasIncompleteNativeScan());
    }

    /*
     * One teardown for every failed grab attempt.
     *
     * Acquisition stages state in layers: body-set prep, then the canonical grab frame
     * and the three-phase state, then collision suppression and the constraint drive.
     * A failure can happen in any layer. This unwinds all of them in reverse order.
     * Each layer guards itself against "nothing was staged", so an exit that fails in
     * the first layer pays only for the guard checks.
     *
     * Ordering is load-bearing in one place: the inertia restore must read
     * _savedObjectState before the lifecycle restore clears it.
     */
    void Hand::abortGrabAcquisition(const GrabAcquisitionUnwind& unwind)
    {
        // Drop the drive first. The proxy body and its constraint are the only things
        // that could still move the object after this frame returns.
        destroyGrabAuthorityProxy(unwind.bhkWorld);
        // The peer hand owns the saved state when this hand only joined its hold.
        if (!unwind.joiningPeerHeldObject) {
            restoreGrabbedInertia(unwind.world, _savedObjectState);
        }

        // Give the body set its original motion back. The peer keeps ownership of the
        // lifecycle when this hand joined an object the peer already holds.
        if (!unwind.joiningPeerHeldObject && unwind.lifecycle) {
            auto& activeLifecycle = *unwind.lifecycle;
            if (unwind.consumedPullPrepLifecycle) {
                /*
                 * This attempt took the pull prep over, so the object has no other
                 * owner left. Unwind it as a physical drop, not as a plain failure.
                 */
                const auto releaseRestorePolicy =
                    active_grab_body_lifecycle::releaseRestorePolicyForTargetKind(unwind.targetKind);
                const auto releasePlan = activeLifecycle.restorePlanForRelease(
                    releaseRestorePolicy,
                    unwind.targetKind,
                    active_grab_body_lifecycle::BodyReleaseIntent::PhysicalDrop);
                restoreActiveGrabLifecycle(unwind.world,
                    activeLifecycle,
                    releasePlan,
                    unwind.objectBodyId.value,
                    handName(),
                    "failed-pull-catch-setup-physical-drop");
                if (activeLifecycle.hasIncompleteNativeScan()) {
                    // A converted loose object keeps its new motion, so a recursive root
                    // restore would undo the conversion the drop still needs.
                    if (active_grab_body_lifecycle::shouldSkipIncompleteScanRootRestore(releasePlan, unwind.originalMotionPropsId)) {
                        ROCK_LOG_DEBUG(Hand,
                            "{} hand failed pull-catch setup: skipped recursive root restore for converted loose-object physical drop root='{}' motionProps={} preservedMotion={}",
                            handName(),
                            nodeDebugName(unwind.rootNode),
                            unwind.originalMotionPropsId,
                            releasePlan.preservedConvertedMotionCount);
                    } else {
                        restoreIncompleteActivePrepRoot(unwind.rootNode, unwind.originalMotionPropsId, handName(), "failed-pull-catch-setup-incomplete-scan");
                    }
                }
                // A dropped object must fall, so wake the whole set back up.
                if (unwind.world && unwind.objectBodyId.value != INVALID_BODY_ID) {
                    const auto releaseActivation = activateHeldObjectBodySet(unwind.world, unwind.objectBodyId.value, _pulledBodyIds);
                    if (releaseActivation.failedActivationCount > 0) {
                        ROCK_LOG_WARN(Hand,
                            "{} hand failed pull-catch setup activation incomplete: primaryBody={} bodies={} activated={} failed={}",
                            handName(),
                            unwind.objectBodyId.value,
                            releaseActivation.bodyCount,
                            releaseActivation.activatedCount,
                            releaseActivation.failedActivationCount);
                    }
                }
            } else {
                restoreActiveGrabLifecycle(unwind.world,
                    activeLifecycle,
                    activeLifecycle.restorePlanForFailure(),
                    unwind.objectBodyId.value,
                    handName(),
                    "failed-setup");
                if (activeLifecycle.hasIncompleteNativeScan()) {
                    restoreIncompleteActivePrepRoot(unwind.rootNode, unwind.originalMotionPropsId, handName(), "failed-setup-incomplete-scan");
                }
            }
        }
        // Both suppression sets no-op when this attempt never armed them.
        restoreHandCollisionAfterGrab(unwind.world);
        restoreBodyCollisionAfterHeldLooseWeapon(unwind.world);

        // Clear every staged grab field after the unwind has consumed its snapshots.
        clearAllGrabRuntimeState();
    }

    bool Hand::prepareGrabBodySet(hand_grab_detail::GrabAcquisitionContext& context)
    {
        if (!context.world || !context.sharedContext) {
            return false;
        }

        auto* world = context.world;
        const auto& sharedContext = *context.sharedContext;

        if (!hasSelection() || !world)
            return false;
        if (!hasCollisionBody())
            return false;

        const auto& sel = context.selection;
        const auto selectedRef = sel.retainedRef;
        if (!selectedRef || selectedRef.get() != sel.refr || sel.bodyId.value == 0x7FFF'FFFF)
            return false;
        if (sel.refr->IsDeleted() || sel.refr->IsDisabled())
            return false;

        if (!grab_target::canUseRockActiveGrab(sel.targetKind)) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand GRAB blocked: targetKind={} formID={:08X}; actor targets are not normal ROCK physical grabs",
                handName(),
                grab_target::name(sel.targetKind),
                sel.refr ? sel.refr->GetFormID() : 0);
            clearGrabExternalHandWorldTransform(_isLeft);
            return false;
        }

        const bool joiningPeerHeldObject = sharedContextMatchesSelection(sharedContext, sel);
        const bool grabbedFromPullCatch = pullCatchIntentMatchesSelection();
        const bool looseWeaponGrab = isLooseWeaponGrabTarget(sel);
        const bool handPocketOnlyGrab = grab_target::requiresHandPocketGrab(sel.targetKind);
        ResolvedGrabOffsetSource resolvedGrabOffsetSource{};

        auto objectBodyId = sel.bodyId;
        auto* rootNode = sel.refr->Get3D();
        if (!rootNode) {
            ROCK_LOG_WARN(Hand, "{} hand GRAB failed: selected ref has no 3D root", handName());
            return false;
        }

        auto* ownerCell = sel.refr->GetParentCell();
        auto* bhkWorld = ownerCell ? ownerCell->GetbhkWorld() : nullptr;
        if (!bhkWorld) {
            ROCK_LOG_WARN(Hand, "{} hand GRAB failed: selected ref has no bhkWorld for object-tree scan", handName());
            return false;
        }

        auto* body = havok_runtime::getBody(world, objectBodyId);
        if (!body) {
            ROCK_LOG_WARN(Hand, "{} hand GRAB failed: selected body no longer readable bodyId={}", handName(), objectBodyId.value);
            return false;
        }

        auto* baseObj = sel.refr->GetObjectReference();
        const bool selectedObjectIsCar = fo4vr::isExplodableCar(baseObj);
        const auto carGrabDecision = car_interaction_policy::evaluateGrab(car_interaction_policy::GrabPolicyInput{
            .targetIsCar = selectedObjectIsCar,
            .playerInPowerArmor = selectedObjectIsCar && fo4vr::isInPowerArmor(),
        });
        if (!carGrabDecision.allowed) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand GRAB blocked: formID={:08X} reason={}",
                handName(),
                sel.refr->GetFormID(),
                carGrabDecision.reason);
            clearGrabExternalHandWorldTransform(_isLeft);
            return false;
        }
        std::string objName = "(unnamed)";
        if (baseObj) {
            auto nameView = RE::TESFullName::GetFullName(*baseObj, false);
            if (!nameView.empty())
                objName = std::string(nameView);
        }

        const char* motionTypeStr = "UNKNOWN";
        std::uint16_t selectedOriginalMotionPropsId = 1;
        {
            auto* objMotion = havok_runtime::getBodyMotion(world, objectBodyId);
            if (objMotion) {
                std::uint32_t bodyFlags = body->flags;
                std::uint8_t bodyMotionPropsId = static_cast<std::uint8_t>(body->motionPropertiesId);
                std::uint16_t motionPropsId = selectedOriginalMotionPropsId;
                if (havok_runtime::tryReadMotionPropertiesId(objMotion, motionPropsId)) {
                    selectedOriginalMotionPropsId = motionPropsId;
                }

                havok_runtime::MotionVelocityCaps velocityCaps{};
                const bool hasVelocityCaps = havok_runtime::tryReadMotionVelocityCaps(objMotion, velocityCaps);
                const float maxLinVel = hasVelocityCaps ? velocityCaps.maxLinearVelocity : 0.0f;
                const float maxAngVel = hasVelocityCaps ? velocityCaps.maxAngularVelocity : 0.0f;

                bool isDynamicInit = (bodyFlags & 0x2) != 0;
                bool isKeyframed = (bodyFlags & 0x4) != 0;

                switch (motionPropsId & 0xFF) {
                case 0:
                    motionTypeStr = "STATIC";
                    break;
                case 1:
                    motionTypeStr = "DYNAMIC";
                    break;
                case 2:
                    motionTypeStr = "KEYFRAMED";
                    break;
                default:
                    motionTypeStr = "OTHER";
                    break;
                }

                ROCK_LOG_DEBUG(Hand,
                    "{} hand GRAB MOTION: body={} motionPropsId={} ({}) "
                    "bodyFlags=0x{:08X} dynInit={} keyfr={} bodyPropsId={} "
                    "maxLinVel={:.1f} maxAngVel={:.1f}",
                    handName(), objectBodyId.value, motionPropsId, motionTypeStr, bodyFlags, isDynamicInit, isKeyframed, bodyMotionPropsId, maxLinVel, maxAngVel);

                {
                    static bool libraryDumped = false;
                    if (!libraryDumped) {
                        libraryDumped = true;
                        havok_runtime::MotionPropertiesLibrarySnapshot motionProperties{};
                        if (havok_runtime::snapshotMotionPropertiesLibrary(world, motionProperties)) {
                            ROCK_LOG_TRACE(Hand, "Motion properties library: {} entries, stride=0x40", motionProperties.count);
                            for (std::uint32_t i = 0; i < motionProperties.copiedCount; i++) {
                                const auto& record = motionProperties.records[i];
                                const auto* f = record.values;
                                const auto* u = record.words;
                                ROCK_LOG_TRACE(Hand,
                                    "  [{}] +00: {:.4f} {:.4f} {:.4f} {:.4f} | "
                                    "+10: {:.4f} {:.4f} {:.4f} {:.4f} | "
                                    "+20: {:.4f} {:.4f} {:.4f} {:.4f} | "
                                    "+30: {:.4f} {:.4f} {:.4f} {:.4f}",
                                    i, f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8], f[9], f[10], f[11], f[12], f[13], f[14], f[15]);
                                ROCK_LOG_TRACE(Hand,
                                    "  [{}] hex: {:08X} {:08X} {:08X} {:08X} | "
                                    "{:08X} {:08X} {:08X} {:08X}",
                                    i, u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7]);
                            }
                        }
                    }
                }
            }
        }

        const std::uint64_t grabTraceId = nextGrabTimelineTraceId();
        if (grabTimelineTraceEnabled()) {
            ROCK_LOG_INFO(Hand,
                "{} GRAB_TRACE stage=attempt trace={} hand={} formID={:08X} name='{}' selectedBody={} targetKind={} motion={} originalMotionProps={} root='{}'",
                handName(),
                grabTraceId,
                _isLeft ? "left" : "right",
                sel.refr ? sel.refr->GetFormID() : 0,
                objName,
                objectBodyId.value,
                grab_target::name(sel.targetKind),
                motionTypeStr,
                selectedOriginalMotionPropsId,
                nodeDebugName(rootNode));
        }

        active_grab_body_lifecycle::BodyLifecycleSnapshot activeLifecycle;
        const bool consumedPullPrepLifecycle =
            !joiningPeerHeldObject && grabbedFromPullCatch && consumePullPrepLifecycleForActiveGrab(sel.refr, activeLifecycle);
        if (joiningPeerHeldObject && sharedContext.peerActiveGrabLifecycle) {
            activeLifecycle = *sharedContext.peerActiveGrabLifecycle;
        }

        ActiveGrabBodySetPrep bodySetPrep{};
        prepareActiveGrabBodySet(
            bhkWorld,
            world,
            sel,
            rootNode,
            joiningPeerHeldObject,
            !joiningPeerHeldObject && !consumedPullPrepLifecycle,
            !joiningPeerHeldObject,
            activeLifecycle,
            bodySetPrep);
        const bool beforePrepScanCacheHit = bodySetPrep.beforePrepScanCacheHit;
        const bool preparedScanCacheHit = bodySetPrep.preparedScanCacheHit;
        const bool preparedBodySetPostPrepComplete = bodySetPrep.preparedBodySetPostPrepComplete;
        const bool motionConverted = bodySetPrep.motionConverted;
        const bool collisionEnabled = bodySetPrep.collisionEnabled;

        context.bhkWorld = bhkWorld;
        context.rootNode = rootNode;
        context.activeLifecycle = std::move(activeLifecycle);
        context.objectBodyId = objectBodyId;
        context.selectedOriginalMotionPropsId = selectedOriginalMotionPropsId;
        context.joiningPeerHeldObject = joiningPeerHeldObject;
        context.grabbedFromPullCatch = grabbedFromPullCatch;
        context.consumedPullPrepLifecycle = consumedPullPrepLifecycle;
        context.looseWeaponGrab = looseWeaponGrab;
        context.handPocketOnlyGrab = handPocketOnlyGrab;
        context.objectName = std::move(objName);
        context.motionType = motionTypeStr;
        context.grabTraceId = grabTraceId;
        context.bodySetSeedBodyId = bodySetPrep.seedBodyId;
        context.beforePrepBodySet = std::move(bodySetPrep.beforePrepBodySet);
        context.preparedBodySet = std::move(bodySetPrep.preparedBodySet);
        context.beforePrepScanCacheHit = beforePrepScanCacheHit;
        context.preparedScanCacheHit = preparedScanCacheHit;
        context.preparedBodySetPostPrepComplete = preparedBodySetPostPrepComplete;
        context.motionConverted = motionConverted;
        context.collisionEnabled = collisionEnabled;
        context.resolvedGrabOffsetSource = resolvedGrabOffsetSource;
        return true;
    }

    bool Hand::abortGrabAcquisition(hand_grab_detail::GrabAcquisitionContext& context)
    {
        abortGrabAcquisition(GrabAcquisitionUnwind{
            .world = context.world,
            .bhkWorld = context.bhkWorld,
            .rootNode = context.rootNode,
            .lifecycle = &context.activeLifecycle,
            .objectBodyId = context.objectBodyId,
            .targetKind = context.selection.targetKind,
            .originalMotionPropsId = context.selectedOriginalMotionPropsId,
            .joiningPeerHeldObject = context.joiningPeerHeldObject,
            .consumedPullPrepLifecycle = context.consumedPullPrepLifecycle,
        });
        return false;
    }


    bool Hand::resolveGrabCaptureFrames(hand_grab_detail::GrabAcquisitionContext& context)
    {
        auto* world = context.world;
        const auto& sel = context.selection;
        const auto& handWorldTransform = context.handWorldTransform;
        const auto objectBodyId = context.objectBodyId;
        auto* rootNode = context.rootNode;
        const auto& objName = context.objectName;
        const auto& beforePrepBodySet = context.beforePrepBodySet;
        const auto& preparedBodySet = context.preparedBodySet;
        const bool beforePrepScanCacheHit = context.beforePrepScanCacheHit;
        const bool preparedScanCacheHit = context.preparedScanCacheHit;
        const bool preparedBodySetPostPrepComplete = context.preparedBodySetPostPrepComplete;
        const bool motionConverted = context.motionConverted;
        const bool collisionEnabled = context.collisionEnabled;
        const bool joiningPeerHeldObject = context.joiningPeerHeldObject;
        const auto grabTraceId = context.grabTraceId;
        auto& activeLifecycle = context.activeLifecycle;
        auto& handBodyWorldAtGrab = context.handBodyWorldAtGrab;
        auto& proxyFrameWorldAtGrab = context.proxyFrameWorldAtGrab;
        auto& proxyFrameSourceAtGrab = context.proxyFrameSourceAtGrab;
        auto& hasPalmProxyFrameAtGrab = context.hasPalmProxyFrameAtGrab;
        auto& grabAuthorityPivotAWorld = context.grabAuthorityPivotAWorld;
        auto& palmPocketPivotAWorld = context.palmPocketPivotAWorld;
        auto& palmPocketToProxyDeltaGameUnits = context.palmPocketToProxyDeltaGameUnits;
        auto& grabPalmBasisDelta = context.grabPalmBasisDelta;
        auto& grabPivotAForPrimaryChoice = context.grabPivotAForPrimaryChoice;
        auto& proxyAuthorityFrameWorldAtGrab = context.proxyAuthorityFrameWorldAtGrab;
        auto*& collidableNode = context.collidableNode;
        auto*& meshSourceNode = context.meshSourceNode;
        auto& objectWorldTransform = context.objectWorldTransform;
        auto& grabGripPoint = context.grabGripPoint;
        auto& selectionToMeshDistanceGameUnits = context.selectionToMeshDistanceGameUnits;
        auto& meshGrabFound = context.meshGrabFound;
        auto& meshStats = context.meshStats;
        auto& grabMeshTriangles = context.grabMeshTriangles;
        auto& grabFingerPoseMeshTriangles = context.grabFingerPoseMeshTriangles;
        auto& grabSurfaceTriangles = context.grabSurfaceTriangles;
        auto& grabLocalMeshTriangles = context.grabLocalMeshTriangles;
        auto& grabFingerPoseLocalMeshTriangles = context.grabFingerPoseLocalMeshTriangles;
        auto& grabSurfaceHit = context.grabSurfaceHit;
        auto& contactPatchRuntime = context.contactPatchRuntime;
        auto& multiFingerGripRuntime = context.multiFingerGripRuntime;
        auto& palmSeatPointWorld = context.palmSeatPointWorld;
        auto& fingerEvidencePointWorld = context.fingerEvidencePointWorld;
        auto& palmSeatSurfaceHit = context.palmSeatSurfaceHit;
        auto& fingerEvidenceSurfaceHit = context.fingerEvidenceSurfaceHit;
        auto& contactPatchEvidenceAvailable = context.contactPatchEvidenceAvailable;
        auto& multiFingerGripUsed = context.multiFingerGripUsed;
        auto& palmSeatPointValid = context.palmSeatPointValid;
        auto& fingerEvidencePointValid = context.fingerEvidencePointValid;
        auto& activeGrabPointUsesMultiFingerEvidence = context.activeGrabPointUsesMultiFingerEvidence;
        auto& contactPatchPivotAuthorityReason = context.contactPatchPivotAuthorityReason;
        auto& pivotAuthoritySource = context.pivotAuthoritySource;
        auto& pivotAuthorityNormalTrusted = context.pivotAuthorityNormalTrusted;
        auto& pivotAuthorityPositionOnly = context.pivotAuthorityPositionOnly;
        auto& pivotAuthorityPositionConfidence = context.pivotAuthorityPositionConfidence;
        auto& pivotAuthorityPocketDistanceGameUnits = context.pivotAuthorityPocketDistanceGameUnits;
        auto& pivotAuthoritySelectionDeltaGameUnits = context.pivotAuthoritySelectionDeltaGameUnits;
        auto& pivotAuthorityLongLeverGameUnits = context.pivotAuthorityLongLeverGameUnits;
        auto*& surfaceOwnerNode = context.surfaceOwnerNode;
        auto*& authoredGrabNode = context.authoredGrabNode;
        auto& meshContactOnly = context.meshContactOnly;
        auto& grabPointMode = context.grabPointMode;
        auto& grabFallbackReason = context.grabFallbackReason;
        auto& palmSeatPointMode = context.palmSeatPointMode;
        auto& palmSeatFallbackReason = context.palmSeatFallbackReason;
        auto& fingerEvidencePointMode = context.fingerEvidencePointMode;
        auto& fingerEvidenceFallbackReason = context.fingerEvidenceFallbackReason;

        auto abortGrab = [&]() {
            return abortGrabAcquisition(context);
        };

        handBodyWorldAtGrab = getLiveBodyWorldTransform(world, _handBody.getBodyId());
        proxyFrameWorldAtGrab = handBodyWorldAtGrab;
        proxyFrameSourceAtGrab = "unresolved";
        hasPalmProxyFrameAtGrab =
            resolveGrabAuthorityProxyFrame(
                world,
                handWorldTransform,
                &handBodyWorldAtGrab,
                proxyFrameWorldAtGrab,
                proxyFrameSourceAtGrab,
                GrabAuthorityProxyFramePolicy::LivePalmOnly);
        if (!hasPalmProxyFrameAtGrab) {
            ROCK_LOG_ERROR(Hand,
                "{} hand GRAB FAILED: live palm anchor frame unavailable before grab evidence capture bodyId={} handBody={} source={} formID={:08X}",
                handName(),
                objectBodyId.value,
                _handBody.isValid() ? _handBody.getBodyId().value : INVALID_BODY_ID,
                proxyFrameSourceAtGrab,
                sel.refr ? sel.refr->GetFormID() : 0);
            return abortGrab();
        }
        /*
         * Live proxy motion and close-grab pocket acquisition both resolve the
         * configured seat offset in generated/proxy local space. The generated
         * collision body path and hidden proxy seat now agree at startup.
         */
        grabAuthorityPivotAWorld = proxyFrameWorldAtGrab.translate;
        palmPocketPivotAWorld = proxyFrameWorldAtGrab.translate;
        palmPocketToProxyDeltaGameUnits = pointDistanceGameUnits(grabAuthorityPivotAWorld, palmPocketPivotAWorld);
        grabPalmBasisDelta = computeGrabPalmBasisDelta(handWorldTransform, proxyFrameWorldAtGrab);
        grabPivotAForPrimaryChoice = palmPocketPivotAWorld;
        proxyAuthorityFrameWorldAtGrab =
            makeGeneratedProxyAuthorityRelationFrame(proxyFrameWorldAtGrab);

        if (grabPalmBasisDelta.rotationDegrees > kGrabFrameMismatchRawProxyRotationWarnDegrees) {
            ROCK_LOG_SAMPLE_WARN(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} PROXY GRAB PALM BASIS MISMATCH: ref='{}' formID={:08X} source={} rawToProxy={:.1f}deg axisDeg=({:.1f},{:.1f},{:.1f}) determinant=({:.3f},{:.3f}) proxyPivot=({:.1f},{:.1f},{:.1f}) palmPocketPivot=({:.1f},{:.1f},{:.1f}) pocketProxyDelta={:.2f}gu phase={}",
                handName(),
                objName,
                sel.refr ? sel.refr->GetFormID() : 0,
                proxyFrameSourceAtGrab,
                grabPalmBasisDelta.rotationDegrees,
                grabPalmBasisDelta.xAxisDegrees,
                grabPalmBasisDelta.yAxisDegrees,
                grabPalmBasisDelta.zAxisDegrees,
                grabPalmBasisDelta.rawDeterminant,
                grabPalmBasisDelta.proxyDeterminant,
                grabAuthorityPivotAWorld.x,
                grabAuthorityPivotAWorld.y,
                grabAuthorityPivotAWorld.z,
                palmPocketPivotAWorld.x,
                palmPocketPivotAWorld.y,
                palmPocketPivotAWorld.z,
                palmPocketToProxyDeltaGameUnits,
                grab_three_phase::phaseName(_grabAcquisitionPhase));
        }

        ROCK_LOG_DEBUG(Hand,
            "{} hand object-tree prep: ref='{}' formID={:08X} beforeBodies={} afterBodies={} accepted={} rejected={} "
            "seedBody={} seeded={} scanSource={}/{} preparedComplete={} cachedBodyIds={} cacheHits={} scanFailures={} invalidSystems={} benignSkips={} foreignSkips={} unresolvedAccepted={} unresolvedSkips={} "
            "latePrepared={} incompleteScan={} collisionObjects={} visitedNodes={} setMotion={} enableCollision={} sharedPeer={} "
            "proxyPivot=({:.1f},{:.1f},{:.1f}) palmPocketPivot=({:.1f},{:.1f},{:.1f}) pocketProxyDelta={:.2f} palmBasis={:.1f}deg axisDeg=({:.1f},{:.1f},{:.1f}) determinant=({:.3f},{:.3f})",
            handName(),
            objName,
            sel.refr->GetFormID(),
            beforePrepBodySet.records.size(),
            preparedBodySet.records.size(),
            preparedBodySet.acceptedCount(),
            preparedBodySet.rejectedCount(),
            context.bodySetSeedBodyId,
            preparedBodySet.diagnostics.seedBodiesAdded,
            beforePrepScanCacheHit ? "cache" : "direct",
            preparedScanCacheHit ? "cache" : "direct",
            preparedBodySetPostPrepComplete ? "yes" : "no",
            preparedBodySet.diagnostics.cachedBodyIds,
            preparedBodySet.diagnostics.cachedScanHits,
            preparedBodySet.diagnostics.scanFailures,
            preparedBodySet.diagnostics.invalidPhysicsSystems,
            preparedBodySet.diagnostics.benignScanSkips,
            preparedBodySet.diagnostics.foreignRefBodySkips,
            preparedBodySet.diagnostics.unresolvedRefBodiesAccepted,
            preparedBodySet.diagnostics.unresolvedRefBodySkips,
            activeLifecycle.latePreparedBodyCount(),
            activeLifecycle.hasIncompleteNativeScan() ? "yes" : "no",
            preparedBodySet.diagnostics.collisionObjects,
            preparedBodySet.diagnostics.visitedNodes,
            motionConverted ? "ok" : "failed",
            collisionEnabled ? "ok" : "failed",
            joiningPeerHeldObject ? "yes" : "no",
            grabAuthorityPivotAWorld.x,
            grabAuthorityPivotAWorld.y,
            grabAuthorityPivotAWorld.z,
            palmPocketPivotAWorld.x,
            palmPocketPivotAWorld.y,
            palmPocketPivotAWorld.z,
            palmPocketToProxyDeltaGameUnits,
            grabPalmBasisDelta.rotationDegrees,
            grabPalmBasisDelta.xAxisDegrees,
            grabPalmBasisDelta.yAxisDegrees,
            grabPalmBasisDelta.zAxisDegrees,
            grabPalmBasisDelta.rawDeterminant,
            grabPalmBasisDelta.proxyDeterminant);

        collidableNode = sel.hitNode ? sel.hitNode : rootNode;
        meshSourceNode = sel.visualNode ? sel.visualNode : rootNode;
        if (!meshSourceNode) {
            meshSourceNode = collidableNode;
        }

        grabGripPoint = sel.hasHitPoint ? sel.hitPointWorld : grabPivotAForPrimaryChoice;
        selectionToMeshDistanceGameUnits = 0.0f;
        meshGrabFound = false;
        meshStats = {};
        grabMeshTriangles.clear();
        grabFingerPoseMeshTriangles.clear();
        grabSurfaceTriangles.clear();
        grabLocalMeshTriangles.clear();
        grabFingerPoseLocalMeshTriangles.clear();
        grabSurfaceHit = {};
        contactPatchRuntime = {};
        multiFingerGripRuntime = {};
        palmSeatPointWorld = {};
        fingerEvidencePointWorld = {};
        palmSeatSurfaceHit = {};
        fingerEvidenceSurfaceHit = {};
        contactPatchEvidenceAvailable = false;
        multiFingerGripUsed = false;
        palmSeatPointValid = false;
        fingerEvidencePointValid = false;
        activeGrabPointUsesMultiFingerEvidence = false;
        contactPatchPivotAuthorityReason = "notEvaluated";
        pivotAuthoritySource = "notEvaluated";
        pivotAuthorityNormalTrusted = false;
        pivotAuthorityPositionOnly = false;
        pivotAuthorityPositionConfidence = 0.0f;
        pivotAuthorityPocketDistanceGameUnits = std::numeric_limits<float>::max();
        pivotAuthoritySelectionDeltaGameUnits = std::numeric_limits<float>::max();
        pivotAuthorityLongLeverGameUnits = 0.0f;
        surfaceOwnerNode = nullptr;
        authoredGrabNode = nullptr;
        meshContactOnly = g_rockConfig.rockGrabMeshContactOnly;
        grabPointMode = sel.hasHitPoint ? "selectionHitPointFallback" : "noContactPointPending";
        grabFallbackReason = meshSourceNode ? "noTriangles" : "noMeshSourceNode";
        palmSeatPointMode = grabPointMode;
        palmSeatFallbackReason = grabFallbackReason;
        fingerEvidencePointMode = "none";
        fingerEvidenceFallbackReason = "none";
        const auto logGrabCaptureRefresh = [&](const GrabCaptureTransformRefreshResult& refresh) {
            if (!grabTimelineTraceEnabled()) {
                return;
            }
            for (std::uint32_t i = 0; i < refresh.count; ++i) {
                const auto& sample = refresh.samples[i];
                ROCK_LOG_INFO(Hand,
                    "{} GRAB_TRACE stage=capture_refresh trace={} hand={} role={} node='{}'({:p}) validBefore={} validAfter={} posDelta={:.4f}gu rotDelta={:.3f}deg",
                    handName(),
                    grabTraceId,
                    _isLeft ? "left" : "right",
                    sample.role,
                    nodeDebugName(sample.node),
                    static_cast<const void*>(sample.node),
                    sample.validBefore ? "yes" : "no",
                    sample.validAfter ? "yes" : "no",
                    sample.positionDeltaGameUnits,
                    sample.rotationDeltaDegrees);
            }
        };

        const auto captureRefresh = refreshGrabCaptureTransforms(rootNode, meshSourceNode, collidableNode);
        logGrabCaptureRefresh(captureRefresh);
        if (!captureRefresh.ok) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: capture transform refresh produced a non-finite node transform for '{}' formID={:08X}; root='{}' mesh='{}' collidable='{}'",
                handName(),
                objName,
                sel.refr ? sel.refr->GetFormID() : 0,
                nodeDebugName(rootNode),
                nodeDebugName(meshSourceNode),
                nodeDebugName(collidableNode));
            return abortGrab();
        }

        if (collidableNode) {
            objectWorldTransform = collidableNode->world;
        } else {
            objectWorldTransform = handWorldTransform;
        }
        if (!grab_three_phase::isFinite(objectWorldTransform)) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: refreshed object transform is non-finite for '{}' formID={:08X}; collidable='{}'",
                handName(),
                objName,
                sel.refr ? sel.refr->GetFormID() : 0,
                nodeDebugName(collidableNode));
            return abortGrab();
        }


        return true;
    }

    bool Hand::extractGrabMesh(hand_grab_detail::GrabAcquisitionContext& context)
    {
        auto* world = context.world;
        const auto& sel = context.selection;
        const auto objectBodyId = context.objectBodyId;
        const bool handPocketOnlyGrab = context.handPocketOnlyGrab;
        auto* rootNode = context.rootNode;
        const auto& proxyAuthorityFrameWorldAtGrab = context.proxyAuthorityFrameWorldAtGrab;
        const auto& palmPocketPivotAWorld = context.palmPocketPivotAWorld;
        auto*& collidableNode = context.collidableNode;
        auto*& meshSourceNode = context.meshSourceNode;
        auto& grabGripPoint = context.grabGripPoint;
        auto& selectionToMeshDistanceGameUnits = context.selectionToMeshDistanceGameUnits;
        auto& meshGrabFound = context.meshGrabFound;
        auto& meshStats = context.meshStats;
        auto& grabMeshTriangles = context.grabMeshTriangles;
        auto& grabSurfaceTriangles = context.grabSurfaceTriangles;
        auto& grabSurfaceHit = context.grabSurfaceHit;
        auto*& surfaceOwnerNode = context.surfaceOwnerNode;
        auto*& authoredGrabNode = context.authoredGrabNode;
        auto& meshContactOnly = context.meshContactOnly;
        auto& hasMeshSurfaceContact = context.hasMeshSurfaceContact;
        auto& grabPointMode = context.grabPointMode;
        auto& grabFallbackReason = context.grabFallbackReason;

        /*
         * hknp selection identifies the object/body. In mesh-authoritative mode
         * it is logged as collision evidence only; the grabbed point and frame
         * must come from visual geometry or an authored ROCK grab node.
         */
        if (sel.hasHitPoint && sel.hasHitNormal) {
            const auto collisionSurfaceHit = makeCollisionQueryGrabSurfaceHit(sel, collidableNode);
            if (collisionSurfaceHit.valid) {
                if (!meshContactOnly && !handPocketOnlyGrab) {
                    grabSurfaceHit = collisionSurfaceHit;
                    grabGripPoint = grabSurfaceHit.position;
                    surfaceOwnerNode = grabSurfaceHit.sourceNode;
                    meshGrabFound = true;
                    grabPointMode = "collisionSurface";
                    grabFallbackReason = "none";
                }
                ROCK_LOG_DEBUG(Hand,
                    "{} hand COLLISION SELECTION HIT: body={} activeGrabPoint={} point=({:.1f},{:.1f},{:.1f}) normal=({:.3f},{:.3f},{:.3f}) "
                    "fraction={:.4f} shapeKey=0x{:08X} filter=0x{:08X} owner='{}'",
                    handName(),
                    sel.bodyId.value,
                    meshContactOnly ? "no" : "yes",
                    collisionSurfaceHit.position.x,
                    collisionSurfaceHit.position.y,
                    collisionSurfaceHit.position.z,
                    collisionSurfaceHit.normal.x,
                    collisionSurfaceHit.normal.y,
                    collisionSurfaceHit.normal.z,
                    collisionSurfaceHit.hitFraction,
                    collisionSurfaceHit.shapeKey,
                    collisionSurfaceHit.shapeCollisionFilterInfo,
                    nodeDebugName(collisionSurfaceHit.sourceNode));
            }
        }

        if (meshSourceNode) {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabMeshExtraction);
            const int meshExtractionDepth = (std::max)(1, g_rockConfig.rockObjectPhysicsTreeMaxDepth);
            std::array<RE::NiAVObject*, 3> meshExtractionAttemptedRoots{};
            std::uint32_t meshExtractionAttemptCount = 0;
            const char* meshExtractionSource = "visual";
            meshExtractionAttemptedRoots[meshExtractionAttemptCount++] = meshSourceNode;
            const auto primaryBeforeTriangles = grabMeshTriangles.size();
            extractAllSurfaceTriangles(meshSourceNode,
                grabMeshTriangles,
                grabSurfaceTriangles,
                meshExtractionDepth,
                &meshStats,
                g_rockConfig.rockGrabNodeNameBlacklist,
                handPocketOnlyGrab);

            const bool primaryMeshExtractionFound = grabMeshTriangles.size() != primaryBeforeTriangles;
            auto tryExtractSurfaceTrianglesFromAlternateRoot = [&](RE::NiAVObject* candidateRoot, const char* sourceName) {
                if (!candidateRoot) {
                    return false;
                }
                for (std::uint32_t i = 0; i < meshExtractionAttemptCount; ++i) {
                    if (meshExtractionAttemptedRoots[i] == candidateRoot) {
                        return false;
                    }
                }
                if (meshExtractionAttemptCount < meshExtractionAttemptedRoots.size()) {
                    meshExtractionAttemptedRoots[meshExtractionAttemptCount] = candidateRoot;
                }
                ++meshExtractionAttemptCount;

                const auto beforeTriangles = grabMeshTriangles.size();
                extractAllSurfaceTriangles(candidateRoot,
                    grabMeshTriangles,
                    grabSurfaceTriangles,
                    meshExtractionDepth,
                    &meshStats,
                    g_rockConfig.rockGrabNodeNameBlacklist,
                    handPocketOnlyGrab);

                if (grabMeshTriangles.size() == beforeTriangles) {
                    return false;
                }

                if (candidateRoot != meshSourceNode) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand mesh extraction recovered from {} node: meshNode='{}' previousMeshNode='{}' ownerNode='{}' rootNode='{}' addedTris={} totalTris={}",
                        handName(),
                        sourceName,
                        nodeDebugName(candidateRoot),
                        nodeDebugName(meshSourceNode),
                        nodeDebugName(collidableNode),
                        nodeDebugName(rootNode),
                        grabMeshTriangles.size() - beforeTriangles,
                        grabMeshTriangles.size());
                }
                meshSourceNode = candidateRoot;
                meshExtractionSource = sourceName;
                return true;
            };

            if (!primaryMeshExtractionFound) {
                if (!tryExtractSurfaceTrianglesFromAlternateRoot(collidableNode, "owner")) {
                    (void)tryExtractSurfaceTrianglesFromAlternateRoot(rootNode, "root");
                }
            }

            ROCK_LOG_DEBUG(Hand,
                "{} hand mesh extraction: meshNode='{}' ownerNode='{}' rootNode='{}' shapes={} "
                "source={} attempts={} static={}/{} dynamic={}/{} skinned={}/{} dynamicSkinnedSkipped={} emptyShapes={} blacklisted={} totalTris={}",
                handName(), nodeDebugName(meshSourceNode), nodeDebugName(collidableNode), nodeDebugName(rootNode), meshStats.visitedShapes, meshExtractionSource,
                meshExtractionAttemptCount, meshStats.staticShapes,
                meshStats.staticTriangles, meshStats.dynamicShapes, meshStats.dynamicTriangles, meshStats.skinnedShapes, meshStats.skinnedTriangles,
                meshStats.dynamicSkinnedSkipped, meshStats.emptyShapes, meshStats.blacklistedShapes, meshStats.totalTriangles());
            performance_profiler::observeValue(performance_profiler::ValueMetric::GrabMeshTriangles, meshStats.totalTriangles());

            {
                RE::BSTriShape* firstTriShape = meshSourceNode->IsTriShape();
                if (!firstTriShape) {
                    auto* meshNode = meshSourceNode->IsNode();
                    if (meshNode) {
                        auto& kids = meshNode->GetRuntimeData().children;
                        const auto childCount = kids.size();
                        for (auto ci = decltype(childCount){ 0 }; ci < childCount; ci++) {
                            auto* kid = kids[ci].get();
                            if (kid && kid->IsTriShape()) {
                                firstTriShape = kid->IsTriShape();
                                break;
                            }
                        }
                    }
                }
                if (firstTriShape) {
                    auto* tsBase = reinterpret_cast<char*>(firstTriShape);
                    std::uint64_t vtxDesc = *reinterpret_cast<std::uint64_t*>(tsBase + VROffset::vertexDesc);
                    std::uint32_t stride = static_cast<std::uint32_t>(vtxDesc & 0xF) * 4;
                    std::uint32_t posOff = static_cast<std::uint32_t>((vtxDesc >> 2) & 0x3C);
                    bool fullPrec = ((vtxDesc >> 54) & 1) != 0;
                    std::uint8_t geomType = *reinterpret_cast<std::uint8_t*>(tsBase + 0x198);
                    void* skinInst = *reinterpret_cast<void**>(tsBase + VROffset::skinInstance);

                    ROCK_LOG_TRACE(MeshGrab, "VertexDiag '{}': vtxDesc=0x{:016X} stride={} posOffset={} fullPrec={} geomType={} skinned={}",
                        firstTriShape->name.c_str() ? firstTriShape->name.c_str() : "(null)", vtxDesc, stride, posOff, fullPrec ? 1 : 0, geomType, skinInst ? 1 : 0);
                }
            }

            if (!grabMeshTriangles.empty()) {
                auto& t0 = grabMeshTriangles[0];
                float cx = (t0.v0.x + t0.v1.x + t0.v2.x) / 3.0f;
                float cy = (t0.v0.y + t0.v1.y + t0.v2.y) / 3.0f;
                float cz = (t0.v0.z + t0.v1.z + t0.v2.z) / 3.0f;

                if (auto* liveBody = havok_runtime::getBody(world, objectBodyId)) {
                    auto* objFloats = reinterpret_cast<float*>(liveBody);
                    const float hkToGameScale = havokToGameScale();
                    float distToBody = std::sqrt((cx - objFloats[12] * hkToGameScale) * (cx - objFloats[12] * hkToGameScale) +
                        (cy - objFloats[13] * hkToGameScale) * (cy - objFloats[13] * hkToGameScale) +
                        (cz - objFloats[14] * hkToGameScale) * (cz - objFloats[14] * hkToGameScale));

                    ROCK_LOG_TRACE(MeshGrab, "TRI[0] centroid=({:.1f},{:.1f},{:.1f}) distToBody={:.1f}gu", cx, cy, cz, distToBody);
                }
            }

            if (!handPocketOnlyGrab && g_rockConfig.rockGrabNodeAnchorsEnabled) {
                const std::string_view primaryNodeName = _isLeft ? std::string_view(g_rockConfig.rockGrabNodeNameLeft) : std::string_view(g_rockConfig.rockGrabNodeNameRight);
                const std::string_view fallbackNodeName = grab_node_name_policy::defaultGrabNodeName(_isLeft);
                RE::NiAVObject* grabNode = findAuthoredGrabNodeRecursive(
                    meshSourceNode, primaryNodeName, _isLeft, g_rockConfig.rockGrabNodeRejectOppositeHandAnchor, g_rockConfig.rockObjectPhysicsTreeMaxDepth);
                if (!grabNode && fallbackNodeName != primaryNodeName) {
                    grabNode = findAuthoredGrabNodeRecursive(
                        meshSourceNode, fallbackNodeName, _isLeft, g_rockConfig.rockGrabNodeRejectOppositeHandAnchor, g_rockConfig.rockObjectPhysicsTreeMaxDepth);
                }
                if (grabNode) {
                    authoredGrabNode = grabNode;
                    surfaceOwnerNode = grabNode;
                    grabSurfaceHit = {};
                    grabGripPoint = grabNode->world.translate;
                    meshGrabFound = true;
                    grabPointMode = "grabNodeAnchor";
                    grabFallbackReason = "none";
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand GRAB NODE: node='{}' point=({:.1f},{:.1f},{:.1f})",
                        handName(),
                        nodeDebugName(grabNode),
                        grabGripPoint.x,
                        grabGripPoint.y,
                        grabGripPoint.z);
                }
            }

            enum class MeshSurfaceFallback : std::uint8_t
            {
                PalmPocket,
                SelectionHit,
                General
            };

            // Each fallback selects a candidate differently. All accepted candidates
            // pass through one state update so their metadata cannot drift.
            auto tryMeshSurfaceFallback = [&](MeshSurfaceFallback fallback) {
                if (grabSurfaceTriangles.empty()) {
                    return false;
                }

                const RE::NiPoint3 grabPivotAWorld = palmPocketPivotAWorld;
                const RE::NiPoint3 palmDir = computePalmNormalFromHandBasis(proxyAuthorityFrameWorldAtGrab, _isLeft);
                GrabSurfaceHit candidate{};
                RE::NiPoint3 pocketAuthorityPoint{};
                float palmPocketSnapDistance = 0.0f;
                int rejectedBehindSurface = 0;
                bool found = false;

                switch (fallback) {
                case MeshSurfaceFallback::PalmPocket: {
                    const auto closePocket = grab_three_phase::buildGrabPocketFrameWithPalmCenter(
                        proxyAuthorityFrameWorldAtGrab,
                        _isLeft,
                        grabPivotAWorld,
                        g_rockConfig.rockGrabPocketDepthGameUnits,
                        g_rockConfig.rockGrabPocketRadiusGameUnits);
                    pocketAuthorityPoint = closePocket.valid ? closePocket.palmCenterWorld : grabPivotAWorld;
                    const RE::NiPoint3 pocketAuthorityNormal = closePocket.valid ? closePocket.palmNormalWorld : palmDir;
                    palmPocketSnapDistance = (std::max)(
                        finitePositiveOr(g_rockConfig.rockGrabPocketRadiusGameUnits, 6.0f),
                        (std::max)(
                            finitePositiveOr(g_rockConfig.rockGrabAlignmentMaxSelectionToMeshDistance, 8.0f),
                            (std::max)(
                                finitePositiveOr(g_rockConfig.rockGrabTouchAcquireDistanceGameUnits, 10.0f),
                                finitePositiveOr(g_rockConfig.rockGrabContactPatchMeshSnapMaxDistanceGameUnits, 4.0f) +
                                    finitePositiveOr(g_rockConfig.rockGrabContactPatchProbeSpacingGameUnits, 3.0f))));
                    found = findClosestGrabSurfaceHitToPointPositionOnly(
                        grabSurfaceTriangles,
                        pocketAuthorityPoint,
                        pocketAuthorityNormal,
                        palmPocketSnapDistance,
                        candidate);
                    break;
                }
                case MeshSurfaceFallback::SelectionHit: {
                    const RE::NiPoint3 expectedNormal =
                        sel.hasHitNormal && lengthSquared(sel.hitNormalWorld) > 0.0f ? sel.hitNormalWorld : palmDir;
                    found = findClosestGrabSurfaceHitToPoint(
                        grabSurfaceTriangles,
                        sel.hitPointWorld,
                        expectedNormal,
                        g_rockConfig.rockGrabAlignmentMaxSelectionToMeshDistance,
                        g_rockConfig.rockGrabContactPatchMaxNormalAngleDegrees,
                        candidate);
                    break;
                }
                case MeshSurfaceFallback::General:
                    found = findClosestGrabSurfaceHit(
                        grabSurfaceTriangles,
                        grabPivotAWorld,
                        palmDir,
                        g_rockConfig.rockGrabLateralWeight,
                        g_rockConfig.rockGrabDirectionalWeight,
                        candidate,
                        g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits,
                        &rejectedBehindSurface);
                    break;
                }

                if (!found) {
                    if (fallback == MeshSurfaceFallback::General) {
                        grabFallbackReason = "noClosestSurfacePoint";
                    }
                    return false;
                }

                grabSurfaceHit = candidate;
                grabGripPoint = grabSurfaceHit.position;
                selectionToMeshDistanceGameUnits =
                    sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint) : std::numeric_limits<float>::max();
                grabSurfaceHit.hasSelectionHit = sel.hasHitPoint;
                grabSurfaceHit.selectionToMeshDistanceGameUnits = selectionToMeshDistanceGameUnits;
                surfaceOwnerNode = grabSurfaceHit.sourceNode;
                meshGrabFound = true;

                if (fallback == MeshSurfaceFallback::PalmPocket) {
                    grabSurfaceHit.pivotToSurfaceDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                    grabPointMode = "palmPocketMeshSurface";
                    grabFallbackReason = "closePalmPocketMeshAuthority";
                } else if (fallback == MeshSurfaceFallback::SelectionHit) {
                    grabSurfaceHit.signedAlongPalmDistanceGameUnits = sel.signedAlongDistance;
                    grabSurfaceHit.lateralPalmDistanceGameUnits = sel.lateralDistance;
                    grabPointMode = "selectionHitMeshSnap";
                    grabFallbackReason = "selectionHitMeshSnap";
                } else {
                    grabPointMode = "meshSurface";
                    grabFallbackReason = "none";
                }

                if (fallback != MeshSurfaceFallback::General) {
                    grabSurfaceHit.shapeKey = sel.hitShapeKey;
                    grabSurfaceHit.shapeCollisionFilterInfo = sel.hitShapeCollisionFilterInfo;
                    grabSurfaceHit.hitFraction = sel.hitFraction;
                    grabSurfaceHit.hasShapeKey = sel.hasHitShapeKey;
                }

                ROCK_LOG_DEBUG(Hand,
                    "{} hand MESH GRAB: mode={} tris={} staticTris={} dynamicTris={} skinnedTris={} closest=({:.1f},{:.1f},{:.1f}) "
                    "tri={} source={} owner='{}' shape='{}' selectionHit={} selectionDelta={:.1f}gu surfaceAlong={:.1f} surfaceLateral={:.1f} "
                    "pocket=({:.1f},{:.1f},{:.1f}) snapLimit={:.1f} rejectBehind={}",
                    handName(),
                    grabPointMode,
                    grabMeshTriangles.size(),
                    meshStats.staticTriangles,
                    meshStats.dynamicTriangles,
                    meshStats.skinnedTriangles,
                    grabSurfaceHit.position.x,
                    grabSurfaceHit.position.y,
                    grabSurfaceHit.position.z,
                    grabSurfaceHit.triangleIndex,
                    grabSurfaceSourceKindName(grabSurfaceHit.sourceKind),
                    nodeDebugName(grabSurfaceHit.sourceNode),
                    nodeDebugName(grabSurfaceHit.sourceShape),
                    sel.hasHitPoint ? "yes" : "no",
                    sel.hasHitPoint ? selectionToMeshDistanceGameUnits : -1.0f,
                    grabSurfaceHit.signedAlongPalmDistanceGameUnits,
                    grabSurfaceHit.lateralPalmDistanceGameUnits,
                    pocketAuthorityPoint.x,
                    pocketAuthorityPoint.y,
                    pocketAuthorityPoint.z,
                    palmPocketSnapDistance,
                    rejectedBehindSurface);
                return true;
            };

            const bool closeGrabNeedsPalmPocketMeshAuthority =
                !authoredGrabNode &&
                !grabSurfaceTriangles.empty() &&
                (handPocketOnlyGrab ||
                    (!sel.isFarSelection &&
                        (!meshGrabFound || grabSurfaceHit.sourceKind == GrabSurfaceSourceKind::CollisionQuery)));
            if (closeGrabNeedsPalmPocketMeshAuthority) {
                // Seat close grabs from one palm-pocket mesh point before later evidence.
                (void)tryMeshSurfaceFallback(MeshSurfaceFallback::PalmPocket);
            }
            if (!meshGrabFound && sel.hasHitPoint) {
                (void)tryMeshSurfaceFallback(MeshSurfaceFallback::SelectionHit);
            }
            if (!meshGrabFound) {
                if (grabSurfaceTriangles.empty()) {
                    grabFallbackReason = "noTriangles";
                } else {
                    (void)tryMeshSurfaceFallback(MeshSurfaceFallback::General);
                }
            }
        }
        if (!meshGrabFound) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB POINT FALLBACK: mode={} reason={} meshNode='{}' ownerNode='{}' rootNode='{}' "
                "shapes={} static={}/{} dynamic={}/{} skinned={}/{} dynamicSkinnedSkipped={} emptyShapes={} blacklisted={} "
                "fallbackPoint=({:.1f},{:.1f},{:.1f})",
                handName(), grabPointMode, grabFallbackReason, nodeDebugName(meshSourceNode), nodeDebugName(collidableNode), nodeDebugName(rootNode), meshStats.visitedShapes,
                meshStats.staticShapes, meshStats.staticTriangles, meshStats.dynamicShapes, meshStats.dynamicTriangles, meshStats.skinnedShapes, meshStats.skinnedTriangles,
                meshStats.dynamicSkinnedSkipped, meshStats.emptyShapes, meshStats.blacklistedShapes, grabGripPoint.x, grabGripPoint.y, grabGripPoint.z);
        }

        hasMeshSurfaceContact =
            meshGrabFound && grabSurfaceHit.valid && grabSurfaceHit.sourceKind != GrabSurfaceSourceKind::CollisionQuery;

        return true;
    }

    bool Hand::resolvePrimaryGrabBody(hand_grab_detail::GrabAcquisitionContext& context)
    {
        auto* world = context.world;
        const auto& sel = context.selection;
        const auto selectedRef = sel.retainedRef;
        const bool handPocketOnlyGrab = context.handPocketOnlyGrab;
        auto* rootNode = context.rootNode;
        const auto& objName = context.objectName;
        const auto selectedOriginalMotionPropsId = context.selectedOriginalMotionPropsId;
        const auto grabTraceId = context.grabTraceId;
        auto& activeLifecycle = context.activeLifecycle;
        const auto& beforePrepBodySet = context.beforePrepBodySet;
        const auto& preparedBodySet = context.preparedBodySet;
        auto& objectBodyId = context.objectBodyId;
        const auto& grabPivotAForPrimaryChoice = context.grabPivotAForPrimaryChoice;
        auto*& collidableNode = context.collidableNode;
        auto* meshSourceNode = context.meshSourceNode;
        auto& objectWorldTransform = context.objectWorldTransform;
        auto& grabGripPoint = context.grabGripPoint;
        const bool meshGrabFound = context.meshGrabFound;
        const auto& meshStats = context.meshStats;
        auto& grabMeshTriangles = context.grabMeshTriangles;
        auto& grabLocalMeshTriangles = context.grabLocalMeshTriangles;
        auto& grabSurfaceHit = context.grabSurfaceHit;
        auto* surfaceOwnerNode = context.surfaceOwnerNode;
        auto* authoredGrabNode = context.authoredGrabNode;
        const bool meshContactOnly = context.meshContactOnly;
        const bool hasMeshSurfaceContact = context.hasMeshSurfaceContact;
        const auto grabPointMode = context.grabPointMode;
        auto& grabContactQualityMode = context.grabContactQualityMode;
        auto& contactSourcePolicy = context.contactSourcePolicy;
        auto& multiFingerEvidenceEnabled = context.multiFingerEvidenceEnabled;
        auto& hybridFingerProbeEvidenceEnabled = context.hybridFingerProbeEvidenceEnabled;
        auto& primaryChoice = context.primaryChoice;
        auto& surfaceOwnerMatchesResolvedBody = context.surfaceOwnerMatchesResolvedBody;
        auto& mechanicalScope = context.mechanicalScope;
        auto& relaxedArticulatedAuthority = context.relaxedArticulatedAuthority;
        auto& visualMeshPivotAvailable = context.visualMeshPivotAvailable;
        auto& canonicalPivotAvailable = context.canonicalPivotAvailable;
        auto& canonicalPivotPointWorld = context.canonicalPivotPointWorld;
        auto& canonicalPivotNormalWorld = context.canonicalPivotNormalWorld;
        auto& canonicalPivotMode = context.canonicalPivotMode;

        auto abortGrab = [&]() {
            return abortGrabAcquisition(context);
        };
        const auto logGrabCaptureRefresh = [&](const GrabCaptureTransformRefreshResult& refresh) {
            if (!grabTimelineTraceEnabled()) {
                return;
            }
            for (std::uint32_t i = 0; i < refresh.count; ++i) {
                const auto& sample = refresh.samples[i];
                ROCK_LOG_INFO(Hand,
                    "{} GRAB_TRACE stage=capture_refresh trace={} hand={} role={} node='{}'({:p}) validBefore={} validAfter={} posDelta={:.4f}gu rotDelta={:.3f}deg",
                    handName(),
                    grabTraceId,
                    _isLeft ? "left" : "right",
                    sample.role,
                    nodeDebugName(sample.node),
                    static_cast<const void*>(sample.node),
                    sample.validBefore ? "yes" : "no",
                    sample.validAfter ? "yes" : "no",
                    sample.positionDeltaGameUnits,
                    sample.rotationDeltaDegrees);
            }
        };

        contactSourcePolicy = grab_contact_source_policy::evaluateGrabContactSourcePolicy(
            meshContactOnly,
            g_rockConfig.rockGrabRequireMeshContact,
            hasMeshSurfaceContact,
            authoredGrabNode != nullptr);
        grabContactQualityMode = grab_contact_evidence_policy::sanitizeQualityMode(g_rockConfig.rockGrabContactQualityMode);
        multiFingerEvidenceEnabled =
            g_rockConfig.rockGrabMultiFingerContactValidationEnabled &&
            grabContactQualityMode != grab_contact_evidence_policy::GrabContactQualityMode::LegacyPermissive &&
            !authoredGrabNode &&
            !handPocketOnlyGrab &&
            meshContactOnly &&
            g_rockConfig.rockGrabRequireMeshContact;
        hybridFingerProbeEvidenceEnabled =
            multiFingerEvidenceEnabled &&
            grabContactQualityMode == grab_contact_evidence_policy::GrabContactQualityMode::HybridEvidence;
        if (contactSourcePolicy.failWithoutMesh && !handPocketOnlyGrab) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: mesh contact required for '{}' formID={:08X}; collision point was not used as pivot "
                "meshNode='{}' ownerNode='{}' rootNode='{}' shapes={} totalTris={} reason={}",
                handName(),
                objName,
                sel.refr->GetFormID(),
                nodeDebugName(meshSourceNode),
                nodeDebugName(collidableNode),
                nodeDebugName(rootNode),
                meshStats.visitedShapes,
                meshStats.totalTriangles(),
                contactSourcePolicy.reason);
            return abortGrab();
        }

        const RE::NiPoint3 primaryChoiceTarget = meshGrabFound ? grabGripPoint : (sel.hasHitPoint ? sel.hitPointWorld : grabPivotAForPrimaryChoice);
        const auto nearestPrimaryChoice =
            preparedBodySet.choosePrimaryBody(object_physics_body_set::INVALID_BODY_ID, object_physics_body_set::PurePoint3{ primaryChoiceTarget });
        const auto* surfaceOwnerRecord = preparedBodySet.findAcceptedRecordByOwnerNode(surfaceOwnerNode);
        const auto skinnedBodyResolution = skinned_body_resolver::resolvePrimaryBody(skinned_body_resolver::ResolutionInput{
            .targetKind = sel.targetKind,
            .authoredBodyId = authoredGrabNode && surfaceOwnerRecord ? surfaceOwnerRecord->bodyId : object_physics_body_set::INVALID_BODY_ID,
            .surfaceOwnerBodyId = surfaceOwnerRecord ? surfaceOwnerRecord->bodyId : object_physics_body_set::INVALID_BODY_ID,
            .selectedBodyId = sel.bodyId.value,
            .nearestBodyId = nearestPrimaryChoice.bodyId,
            .authoredUsable = authoredGrabNode != nullptr && surfaceOwnerRecord != nullptr,
            .surfaceOwnerUsable = surfaceOwnerRecord != nullptr,
            .selectedUsable = preparedBodySet.containsAcceptedBody(sel.bodyId.value),
            .nearestUsable = nearestPrimaryChoice.bodyId != object_physics_body_set::INVALID_BODY_ID,
            .surfaceIsSkinned = grabSurfaceHit.valid && grabSurfaceHit.sourceKind == GrabSurfaceSourceKind::Skinned,
            .hasSkinInfluences = grabSurfaceHit.valid && grabSurfaceHit.hasSkinInfluences,
        });
        primaryChoice = object_physics_body_set::PrimaryBodyChoice{
            .bodyId = skinnedBodyResolution.bodyId,
            .reason = skinnedBodyResolution.source == skinned_body_resolver::ResolutionSource::WeightedSkinOwner ||
                    skinnedBodyResolution.source == skinned_body_resolver::ResolutionSource::TriangleOwner ||
                    skinnedBodyResolution.source == skinned_body_resolver::ResolutionSource::AuthoredNode ?
                object_physics_body_set::PrimaryBodyChoiceReason::SurfaceOwnerAccepted :
                (skinnedBodyResolution.source == skinned_body_resolver::ResolutionSource::SelectedBody ?
                        object_physics_body_set::PrimaryBodyChoiceReason::PreferredHitAccepted :
                        (skinnedBodyResolution.source == skinned_body_resolver::ResolutionSource::NearestAccepted ?
                                object_physics_body_set::PrimaryBodyChoiceReason::NearestAcceptedFallback :
                                object_physics_body_set::PrimaryBodyChoiceReason::NoAcceptedBody)),
        };
        surfaceOwnerMatchesResolvedBody = true;
        if (grabSurfaceHit.valid) {
            const bool handPocketPositionOnlySkinnedSurface =
                handPocketOnlyGrab && grabSurfaceHit.sourceKind == GrabSurfaceSourceKind::Skinned && !grabSurfaceHit.hasSkinInfluences;
            if (grabSurfaceHit.sourceKind == GrabSurfaceSourceKind::CollisionQuery) {
                surfaceOwnerMatchesResolvedBody = primaryChoice.bodyId == sel.bodyId.value;
            } else if (handPocketPositionOnlySkinnedSurface) {
                surfaceOwnerMatchesResolvedBody =
                    primaryChoice.bodyId == sel.bodyId.value && preparedBodySet.containsAcceptedBody(sel.bodyId.value);
            } else {
                surfaceOwnerMatchesResolvedBody =
                    (surfaceOwnerRecord && surfaceOwnerRecord->bodyId == primaryChoice.bodyId) ||
                    acceptsSelectedMultibodyOwnerlessVisualMesh(sel,
                        preparedBodySet,
                        primaryChoice.bodyId,
                        surfaceOwnerNode,
                        surfaceOwnerRecord);
            }
            grabSurfaceHit.resolvedOwnerMatchesBody = surfaceOwnerMatchesResolvedBody;
        }
        ROCK_LOG_DEBUG(Hand,
            "{} hand GRAB BODY RESOLUTION: selectedBody={} resolvedBody={} reason={} resolver={} resolverReason={} skin={} sourceNode='{}' sourceKind={} ownerMatch={} target=({:.1f},{:.1f},{:.1f})",
            handName(),
            sel.bodyId.value,
            primaryChoice.bodyId,
            primaryBodyChoiceReasonName(primaryChoice.reason),
            skinned_body_resolver::sourceName(skinnedBodyResolution.source),
            skinnedBodyResolution.reason,
            skinnedBodyResolution.usedSkinInfluences ? "weighted" : (grabSurfaceHit.valid && grabSurfaceHit.sourceKind == GrabSurfaceSourceKind::Skinned ? "positionOnly" : "no"),
            nodeDebugName(surfaceOwnerNode),
            grabSurfaceHit.valid ? grabSurfaceSourceKindName(grabSurfaceHit.sourceKind) : (authoredGrabNode ? "authoredNode" : "fallback"),
            surfaceOwnerMatchesResolvedBody ? "yes" : "no",
            primaryChoiceTarget.x,
            primaryChoiceTarget.y,
            primaryChoiceTarget.z);

        if (primaryChoice.bodyId == INVALID_BODY_ID) {
            const auto* rejectedBody = diagnosticRejectedBodyRecord(preparedBodySet, sel.bodyId.value);
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: no accepted dynamic body after recursive prep for '{}' formID={:08X} visitedNodes={} collisionObjects={} seeded={} scanFailures={} invalidSystems={} benignSkips={} foreignSkips={} unresolvedAccepted={} unresolvedSkips={} rejectReason={} rejectBody={} rejectLayer={} rejectMotion={} rejectFlags=0x{:08X} rejectMotionProps={} latePrepared={} incompleteScan={}",
                handName(),
                objName,
                sel.refr->GetFormID(),
                preparedBodySet.diagnostics.visitedNodes,
                preparedBodySet.diagnostics.collisionObjects,
                preparedBodySet.diagnostics.seedBodiesAdded,
                preparedBodySet.diagnostics.scanFailures,
                preparedBodySet.diagnostics.invalidPhysicsSystems,
                preparedBodySet.diagnostics.benignScanSkips,
                preparedBodySet.diagnostics.foreignRefBodySkips,
                preparedBodySet.diagnostics.unresolvedRefBodiesAccepted,
                preparedBodySet.diagnostics.unresolvedRefBodySkips,
                rejectedBody ? physics_body_classifier::rejectReasonName(rejectedBody->rejectReason) : "none",
                rejectedBody ? rejectedBody->bodyId : INVALID_BODY_ID,
                rejectedBody ? rejectedBody->collisionLayer : 0,
                rejectedBody ? bodyMotionTypeName(rejectedBody->motionType) : "none",
                rejectedBody ? rejectedBody->bodyFlags : 0,
                rejectedBody ? rejectedBody->motionPropertiesId : 0,
                activeLifecycle.latePreparedBodyCount(),
                activeLifecycle.hasIncompleteNativeScan() ? "yes" : "no");
            return abortGrab();
        }

        mechanicalScope = mechanical_connected_body_set::buildFromPreparedBodySet(
            beforePrepBodySet,
            preparedBodySet,
            primaryChoice.bodyId,
            sel.targetKind,
            activeLifecycle.hasIncompleteNativeScan());
        relaxedArticulatedAuthority =
            mechanicalScope.strictPocketAuthorityRelaxed && primaryChoice.bodyId != object_physics_body_set::INVALID_BODY_ID;
        ROCK_LOG_DEBUG(Hand,
            "{} hand MECHANICAL SCOPE: targetKind={} kind={} reason={} primaryBody={} bodies={} accepted={} motions={} fixedRejects={} incomplete={} driveMode={} linearScope={} angularScope={} massScope={}",
            handName(),
            grab_target::name(sel.targetKind),
            mechanical_connected_body_set::scopeKindName(mechanicalScope.kind),
            mechanicalScope.reason,
            mechanicalScope.primaryBodyId,
            mechanicalScope.committedBodyIds.size(),
            mechanicalScope.acceptedBodyCount,
            mechanicalScope.uniqueMotionCount,
            mechanicalScope.rejectedFixedOrNonDynamicCount,
            mechanicalScope.incompleteDiscovery ? "yes" : "no",
            held_object_drive_policy::modeName(mechanicalScope.driveDecision.mode),
            mechanicalScope.driveDecision.includeConnectedLinearVelocity ? "bodySet" : "primaryOnly",
            mechanicalScope.driveDecision.includeConnectedAngularVelocity ? "bodySet" : "primaryOnly",
            mechanicalScope.driveDecision.includeConnectedMass ? "bodySet" : "primaryOnly");

        if (grab_contact_source_policy::shouldRejectMeshOwnerMismatch(meshContactOnly,
                g_rockConfig.rockGrabRequireMeshContact,
                hasMeshSurfaceContact,
                authoredGrabNode != nullptr,
                surfaceOwnerMatchesResolvedBody) &&
            !relaxedArticulatedAuthority) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: mesh contact owner does not match resolved body for '{}' formID={:08X}; "
                "selectedBody={} resolvedBody={} sourceNode='{}' sourceKind={} target=({:.1f},{:.1f},{:.1f})",
                handName(),
                objName,
                sel.refr->GetFormID(),
                sel.bodyId.value,
                primaryChoice.bodyId,
                nodeDebugName(surfaceOwnerNode),
                grabSurfaceSourceKindName(grabSurfaceHit.sourceKind),
                primaryChoiceTarget.x,
                primaryChoiceTarget.y,
                primaryChoiceTarget.z);
            return abortGrab();
        }

        /*
         * Dynamic grab pivot authority is one coherent position source. Authored
         * nodes still win, and weak close-grab mesh starts can yield to the
         * palm-pocket mesh point that was selected before body resolution.
         * Contact patches stay validation/pose evidence; small or corner patches
         * must not replace the frozen BODY-local pivot by themselves.
         */
        const bool collisionFallbackPivotAllowed =
            !meshContactOnly && meshGrabFound && grabSurfaceHit.valid && grabSurfaceHit.sourceKind == GrabSurfaceSourceKind::CollisionQuery;
        visualMeshPivotAvailable = authoredGrabNode != nullptr || hasMeshSurfaceContact;
        canonicalPivotAvailable = visualMeshPivotAvailable || collisionFallbackPivotAllowed;
        canonicalPivotPointWorld = grabGripPoint;
        canonicalPivotNormalWorld = grabSurfaceHit.valid ? grabSurfaceHit.normal : RE::NiPoint3{};
        canonicalPivotMode = grabPointMode;

        objectBodyId = RE::hknpBodyId{ primaryChoice.bodyId };
        auto* preparedBody = havok_runtime::getBody(world, objectBodyId);
        if (!preparedBody) {
            ROCK_LOG_ERROR(Hand, "{} grabSelectedObject: prepared primary body {} is not readable after object prep", handName(), objectBodyId.value);
            return abortGrab();
        }
        _savedObjectState.bodyId = objectBodyId;
        _savedObjectState.setReference(selectedRef);
        _savedObjectState.targetKind = sel.targetKind;
        _savedObjectState.originalFilterInfo = preparedBody->collisionFilterInfo;
        _savedObjectState.originalMotionPropsId = selectedOriginalMotionPropsId;
        if (const auto* originalPrimaryRecord = beforePrepBodySet.findRecord(objectBodyId.value)) {
            _savedObjectState.originalMotionPropsId = originalPrimaryRecord->motionPropertiesId;
        }

        {
            auto* ownerCellAtResolution = sel.refr ? sel.refr->GetParentCell() : nullptr;
            auto* bhkWorldAtResolution = ownerCellAtResolution ? ownerCellAtResolution->GetbhkWorld() : nullptr;
            auto* bodyCollisionObjectAtResolution = bhkWorldAtResolution ? RE::bhkNPCollisionObject::Getbhk(bhkWorldAtResolution, objectBodyId) : nullptr;
            if (auto* resolvedOwnerNode = bodyCollisionObjectAtResolution ? bodyCollisionObjectAtResolution->sceneObject : nullptr) {
                GrabCaptureTransformRefreshResult resolvedOwnerRefresh{};
                refreshGrabCaptureNodeTransform(resolvedOwnerRefresh, "resolvedOwner", resolvedOwnerNode);
                logGrabCaptureRefresh(resolvedOwnerRefresh);
                if (!resolvedOwnerRefresh.ok) {
                    ROCK_LOG_WARN(Hand,
                        "{} hand GRAB failed: resolved owner transform refresh produced a non-finite node transform for '{}' formID={:08X}; owner='{}'",
                        handName(),
                        objName,
                        sel.refr ? sel.refr->GetFormID() : 0,
                        nodeDebugName(resolvedOwnerNode));
                    return abortGrab();
                }
                collidableNode = resolvedOwnerNode;
                objectWorldTransform = collidableNode->world;
                if (!grab_three_phase::isFinite(objectWorldTransform)) {
                    ROCK_LOG_WARN(Hand,
                        "{} hand GRAB failed: resolved owner transform is non-finite for '{}' formID={:08X}; owner='{}'",
                        handName(),
                        objName,
                        sel.refr ? sel.refr->GetFormID() : 0,
                        nodeDebugName(collidableNode));
                    return abortGrab();
                }
            }
        }

        if (!grabMeshTriangles.empty()) {
            grabLocalMeshTriangles = cacheTrianglesInLocalSpace(grabMeshTriangles, objectWorldTransform);
        }


        return true;
    }

    bool Hand::resolveGrabPivotAuthority(hand_grab_detail::GrabAcquisitionContext& context)
    {
        auto* world = context.world;
        const auto& sel = context.selection;
        const auto& objName = context.objectName;
        const bool handPocketOnlyGrab = context.handPocketOnlyGrab;
        auto* rootNode = context.rootNode;
        auto* collidableNode = context.collidableNode;
        auto* meshSourceNode = context.meshSourceNode;
        auto& objectBodyId = context.objectBodyId;
        const auto& preparedBodySet = context.preparedBodySet;
        const auto& proxyAuthorityFrameWorldAtGrab = context.proxyAuthorityFrameWorldAtGrab;
        const auto& palmPocketPivotAWorld = context.palmPocketPivotAWorld;
        auto& objectWorldTransform = context.objectWorldTransform;
        auto& grabGripPoint = context.grabGripPoint;
        auto& selectionToMeshDistanceGameUnits = context.selectionToMeshDistanceGameUnits;
        auto& meshGrabFound = context.meshGrabFound;
        auto& grabSurfaceTriangles = context.grabSurfaceTriangles;
        auto& grabLocalMeshTriangles = context.grabLocalMeshTriangles;
        auto& grabSurfaceHit = context.grabSurfaceHit;
        auto& contactPatchRuntime = context.contactPatchRuntime;
        auto& contactPatchEvidenceAvailable = context.contactPatchEvidenceAvailable;
        auto*& surfaceOwnerNode = context.surfaceOwnerNode;
        const auto authoredGrabNode = context.authoredGrabNode;
        const bool visualMeshPivotAvailable = context.visualMeshPivotAvailable;
        const auto& contactSourcePolicy = context.contactSourcePolicy;
        auto& surfaceOwnerMatchesResolvedBody = context.surfaceOwnerMatchesResolvedBody;
        auto& grabPointMode = context.grabPointMode;
        auto& grabFallbackReason = context.grabFallbackReason;
        auto& contactPatchPivotAuthorityReason = context.contactPatchPivotAuthorityReason;
        auto& pivotAuthoritySource = context.pivotAuthoritySource;
        auto& pivotAuthorityNormalTrusted = context.pivotAuthorityNormalTrusted;
        auto& pivotAuthorityPositionOnly = context.pivotAuthorityPositionOnly;
        auto& pivotAuthorityPositionConfidence = context.pivotAuthorityPositionConfidence;
        auto& pivotAuthorityPocketDistanceGameUnits = context.pivotAuthorityPocketDistanceGameUnits;
        auto& pivotAuthoritySelectionDeltaGameUnits = context.pivotAuthoritySelectionDeltaGameUnits;
        auto& pivotAuthorityLongLeverGameUnits = context.pivotAuthorityLongLeverGameUnits;
        const bool canonicalPivotAvailable = context.canonicalPivotAvailable;
        auto& canonicalPivotPointWorld = context.canonicalPivotPointWorld;
        auto& canonicalPivotNormalWorld = context.canonicalPivotNormalWorld;
        const auto canonicalPivotMode = context.canonicalPivotMode;
        auto& semanticContacts = context.semanticContacts;
        auto& acquisitionPocket = context.acquisitionPocket;
        auto& palmPocketSurfaceHit = context.palmPocketSurfaceHit;
        auto& palmPocketMeshAvailable = context.palmPocketMeshAvailable;

        auto abortGrab = [&]() {
            return abortGrabAcquisition(context);
        };

        semanticContacts = collectFreshSemanticContactsForBody(
            objectBodyId.value,
            static_cast<std::uint32_t>(g_rockConfig.rockGrabOppositionContactMaxAgeFrames));
        const RE::NiPoint3 acquisitionGrabPivotAWorld = palmPocketPivotAWorld;
        acquisitionPocket = grab_three_phase::buildGrabPocketFrameWithPalmCenter(
            proxyAuthorityFrameWorldAtGrab,
            _isLeft,
            acquisitionGrabPivotAWorld,
            g_rockConfig.rockGrabPocketDepthGameUnits,
            g_rockConfig.rockGrabPocketRadiusGameUnits);
        palmPocketSurfaceHit = {};
        palmPocketMeshAvailable = false;
        if (!authoredGrabNode && acquisitionPocket.valid && !grabSurfaceTriangles.empty()) {
            const float palmPocketSnapDistance = (std::max)(
                g_rockConfig.rockGrabPocketRadiusGameUnits,
                (std::max)(
                    g_rockConfig.rockGrabAlignmentMaxSelectionToMeshDistance,
                    g_rockConfig.rockGrabContactPatchMeshSnapMaxDistanceGameUnits + g_rockConfig.rockGrabContactPatchProbeSpacingGameUnits));
            if (findClosestGrabSurfaceHitToPointPositionOnly(grabSurfaceTriangles,
                    acquisitionPocket.palmCenterWorld,
                    acquisitionPocket.palmNormalWorld,
                    palmPocketSnapDistance,
                    palmPocketSurfaceHit)) {
                bool ownerMatches = true;
                if (palmPocketSurfaceHit.sourceNode) {
                    const auto* ownerRecord = preparedBodySet.findAcceptedRecordByOwnerNode(palmPocketSurfaceHit.sourceNode);
                    ownerMatches =
                        (ownerRecord && ownerRecord->bodyId == objectBodyId.value) ||
                        acceptsSelectedMultibodyOwnerlessVisualMesh(sel,
                            preparedBodySet,
                            objectBodyId.value,
                            palmPocketSurfaceHit.sourceNode,
                            ownerRecord);
                }
                if (ownerMatches) {
                    palmPocketSurfaceHit.hasSelectionHit = sel.hasHitPoint;
                    palmPocketSurfaceHit.selectionToMeshDistanceGameUnits =
                        sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, palmPocketSurfaceHit.position) : std::numeric_limits<float>::max();
                    palmPocketSurfaceHit.pivotToSurfaceDistanceGameUnits = pointDistanceGameUnits(acquisitionGrabPivotAWorld, palmPocketSurfaceHit.position);
                    palmPocketSurfaceHit.resolvedOwnerMatchesBody = true;
                    palmPocketMeshAvailable = true;
                }
            }
        }

        if (!handPocketOnlyGrab && contactSourcePolicy.allowContactPatchPivot && g_rockConfig.rockGrabContactPatchEnabled && !authoredGrabNode && !sel.isFarSelection) {
            const RE::NiPoint3 palmNormalWorld = acquisitionPocket.palmNormalWorld;
            const RE::NiPoint3 palmTangentWorld = acquisitionPocket.fingerForwardWorld;
            const RE::NiPoint3 palmBitangentWorld = acquisitionPocket.crossPalmWorld;
            float contactPatchObjectLeverEstimateGameUnits = 0.0f;
            if (!grabLocalMeshTriangles.empty() && grab_three_phase::isFinite(objectWorldTransform) && grab_three_phase::isFinite(canonicalPivotPointWorld)) {
                const RE::NiPoint3 canonicalPivotLocal = transform_math::worldPointToLocal(objectWorldTransform, canonicalPivotPointWorld);
                contactPatchObjectLeverEstimateGameUnits =
                    computeLocalMeshMaxDistanceFromPoint(grabLocalMeshTriangles, canonicalPivotLocal) *
                    finitePositiveOr(objectWorldTransform.scale, 1.0f);
            }
            std::vector<GrabSurfaceTriangleData> contactPatchSurfaceTriangles;
            const RE::NiPoint3 contactPatchTriangleCenter =
                acquisitionPocket.valid ? acquisitionPocket.palmCenterWorld : canonicalPivotPointWorld;
            // The patch builder works around the palm center, so cap to that.
            const auto& contactPatchTriangleSource = limitGrabSurfaceTrianglesNear(
                grabSurfaceTriangles, contactPatchTriangleCenter, contactPatchSurfaceTriangles, handName(), "contactPatch");
            contactPatchRuntime = buildRuntimeGrabContactPatch(world,
                preparedBodySet,
                objectBodyId.value,
                sel,
                acquisitionGrabPivotAWorld,
                canonicalPivotPointWorld,
                canonicalPivotAvailable,
                palmNormalWorld,
                palmTangentWorld,
                palmBitangentWorld,
                contactPatchObjectLeverEstimateGameUnits,
                contactPatchTriangleSource);

            const bool contactPatchAcceptedForPivot = grab_contact_source_policy::shouldAcceptContactPatchPivot(
                contactSourcePolicy,
                contactPatchRuntime.patch.valid,
                contactPatchRuntime.meshSnapped);
            contactPatchEvidenceAvailable = contactPatchRuntime.patch.valid && contactPatchRuntime.meshSnapped;
            const bool contactPatchProducedMeshPivot =
                contactPatchRuntime.pivotDecision.valid &&
                contactPatchRuntime.pivotDecision.source == grab_contact_patch_math::GrabContactPatchPivotSource::MeshSnap;
            const auto canonicalPivotCandidate = assessGrabPivotAuthorityCandidate(
                GrabPivotAuthorityCandidateInput{
                    .mode = canonicalPivotMode,
                    .point = canonicalPivotPointWorld,
                    .normal = canonicalPivotNormalWorld,
                    .normalTrusted = true,
                    .positionOnlyPatch = false,
                    .positionConfidence = 1.0f,
                    .pocket = &acquisitionPocket,
                    .selection = &sel,
                    .authorityPoint = canonicalPivotPointWorld,
                    .hasAuthorityPoint = false,
                    .objectWorldTransform = objectWorldTransform,
                    .localMeshTriangles = &grabLocalMeshTriangles,
                });
            const auto contactPatchPivotCandidate = assessGrabPivotAuthorityCandidate(
                GrabPivotAuthorityCandidateInput{
                    .mode = contactPatchRuntime.pointMode,
                    .point = contactPatchRuntime.pivotDecision.valid ? contactPatchRuntime.pivotDecision.point : RE::NiPoint3{},
                    .normal = contactPatchRuntime.normalTrusted ? contactPatchRuntime.patch.normal :
                        (contactPatchRuntime.meshSnapped ? contactPatchRuntime.meshSnapHit.normal : canonicalPivotNormalWorld),
                    .normalTrusted = contactPatchRuntime.normalTrusted,
                    .positionOnlyPatch = true,
                    .positionConfidence = contactPatchRuntime.meshSnapped ? 1.0f : contactPatchRuntime.patch.confidence,
                    .pocket = &acquisitionPocket,
                    .selection = &sel,
                    .authorityPoint = canonicalPivotPointWorld,
                    .hasAuthorityPoint = canonicalPivotAvailable,
                    .objectWorldTransform = objectWorldTransform,
                    .localMeshTriangles = &grabLocalMeshTriangles,
                });
            const auto palmPocketPivotCandidate = assessGrabPivotAuthorityCandidate(
                GrabPivotAuthorityCandidateInput{
                    .mode = "palmPocketMeshSurface",
                    .point = palmPocketMeshAvailable ? palmPocketSurfaceHit.position : RE::NiPoint3{},
                    .normal = palmPocketMeshAvailable ? palmPocketSurfaceHit.normal : RE::NiPoint3{},
                    .normalTrusted = palmPocketMeshAvailable,
                    .positionOnlyPatch = false,
                    .positionConfidence = palmPocketMeshAvailable ? 0.85f : 0.0f,
                    .pocket = &acquisitionPocket,
                    .selection = &sel,
                    .authorityPoint = canonicalPivotPointWorld,
                    .hasAuthorityPoint = canonicalPivotAvailable,
                    .objectWorldTransform = objectWorldTransform,
                    .localMeshTriangles = &grabLocalMeshTriangles,
                });
            const bool contactPatchComparableForAuthority =
                contactPatchAcceptedForPivot &&
                contactPatchProducedMeshPivot &&
                visualMeshPivotAvailable;
            const auto contactPatchAuthorityResolution = resolveMeshBackedGrabPivotAuthority(
                canonicalPivotCandidate,
                contactPatchPivotCandidate,
                palmPocketPivotCandidate,
                contactPatchComparableForAuthority,
                contactPatchRuntime.meshSnapped,
                palmPocketMeshAvailable,
                g_rockConfig.rockGrabContactPatchProbeSpacingGameUnits,
                g_rockConfig.rockGrabContactPatchMeshSnapMaxDistanceGameUnits,
                g_rockConfig.rockGrabAlignmentMaxSelectionToMeshDistance);
            contactPatchPivotAuthorityReason = contactPatchAuthorityResolution.reason;
            const auto& selectedPivotAuthority = contactPatchAuthorityResolution.selected.valid ?
                contactPatchAuthorityResolution.selected :
                canonicalPivotCandidate;
            pivotAuthoritySource = grabPivotAuthoritySourceName(selectedPivotAuthority.source);
            pivotAuthorityNormalTrusted = selectedPivotAuthority.normalTrusted;
            pivotAuthorityPositionOnly = selectedPivotAuthority.positionOnlyPatch;
            pivotAuthorityPositionConfidence = selectedPivotAuthority.positionConfidence;
            pivotAuthorityPocketDistanceGameUnits = selectedPivotAuthority.pivotToPocketGameUnits;
            pivotAuthoritySelectionDeltaGameUnits = selectedPivotAuthority.selectionDeltaGameUnits;
            pivotAuthorityLongLeverGameUnits = selectedPivotAuthority.longLeverGameUnits;
            if (contactPatchAuthorityResolution.usePalmPocketPivot) {
                grabGripPoint = palmPocketSurfaceHit.position;
                grabSurfaceHit = palmPocketSurfaceHit;
                surfaceOwnerNode = grabSurfaceHit.sourceNode;
                selectionToMeshDistanceGameUnits = grabSurfaceHit.selectionToMeshDistanceGameUnits;
                surfaceOwnerMatchesResolvedBody = true;
                meshGrabFound = true;
                grabPointMode = "palmPocketMeshSurface";
                grabFallbackReason = contactPatchPivotAuthorityReason;
                ROCK_LOG_DEBUG(Hand,
                    "{} hand PIVOT AUTHORITY: source={} mode={} point=({:.1f},{:.1f},{:.1f}) pocketDistance={:.2f}gu selectionDelta={:.2f}gu longLever={:.2f}gu reason={} canonical={} canonicalPocket={:.2f} candidateScore={:.2f}",
                    handName(),
                    pivotAuthoritySource,
                    grabPointMode,
                    grabGripPoint.x,
                    grabGripPoint.y,
                    grabGripPoint.z,
                    pivotAuthorityPocketDistanceGameUnits,
                    pivotAuthoritySelectionDeltaGameUnits,
                    pivotAuthorityLongLeverGameUnits,
                    grabFallbackReason,
                    canonicalPivotMode,
                    canonicalPivotCandidate.pivotToPocketGameUnits,
                    selectedPivotAuthority.score);
            } else if (contactPatchRuntime.patch.valid && g_rockConfig.rockDebugGrabFrameLogging) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand CONTACT PATCH evidence-only: body={} mode={} probe={:.2f}/{:.2f} scale={:.2f}:{} meshSnap={} requireMeshSnap={} pivotSource={} normalTrusted={} positionOnly={} authoritySource={} "
                    "clusterRaw={} clusterRejected={} clusterDepth={:.2f} clusterLat={:.2f} clusterReason={} "
                    "authorityDelta={:.2f}/{:.2f}/{:.2f} canonical={} meshPocket={:.2f} patchPocket={:.2f} "
                    "pocketImprove={:.2f} meshScore={:.2f} patchScore={:.2f} scoreImprove={:.2f} authorityReason={} fallback={}",
                    handName(),
                    objectBodyId.value,
                    contactPatchRuntime.pointMode,
                    contactPatchRuntime.probeSpacingGameUnits,
                    contactPatchRuntime.probeRadiusGameUnits,
                    contactPatchRuntime.probeScale,
                    contactPatchRuntime.probeScaleReason,
                    contactPatchRuntime.meshSnapped ? "yes" : "no",
                    contactSourcePolicy.requireContactPatchMeshSnap ? "yes" : "no",
                    grab_contact_patch_math::pivotSourceName(contactPatchRuntime.pivotDecision.source),
                    contactPatchRuntime.normalTrusted ? "yes" : "no",
                    contactPatchRuntime.positionOnly ? "yes" : "no",
                    pivotAuthoritySource,
                    contactPatchRuntime.rawAcceptedSampleCount,
                    contactPatchRuntime.clusterRejectedSampleCount,
                    contactPatchRuntime.clusterDepthSpreadGameUnits,
                    contactPatchRuntime.clusterMaxLateralGameUnits,
                    contactPatchRuntime.clusterReason,
                    contactPatchPivotCandidate.authorityDeltaGameUnits,
                    contactPatchAuthorityResolution.baseAuthorityDeltaGameUnits,
                    contactPatchAuthorityResolution.extendedAuthorityDeltaGameUnits,
                    canonicalPivotMode,
                    canonicalPivotCandidate.pivotToPocketGameUnits,
                    contactPatchPivotCandidate.pivotToPocketGameUnits,
                    contactPatchAuthorityResolution.pocketImprovementGameUnits,
                    canonicalPivotCandidate.score,
                    contactPatchPivotCandidate.score,
                    contactPatchAuthorityResolution.scoreImprovement,
                    contactPatchPivotAuthorityReason,
                    contactPatchRuntime.patch.fallbackReason ? contactPatchRuntime.patch.fallbackReason :
                        (contactPatchProducedMeshPivot ? contactPatchPivotAuthorityReason : "nonMeshPivot"));
            } else if (g_rockConfig.rockDebugGrabFrameLogging) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand CONTACT PATCH failed: body={} samples={} probe={:.2f}/{:.2f} scale={:.2f}:{} castsHits={} rejectBody={} rejectNormal={} "
                    "exactSamples={} meshRecovered={} clusterRaw={} clusterRejected={} clusterDepth={:.2f} clusterLat={:.2f} clusterReason={} rejectedBodies={} fallback={}",
                    handName(),
                    objectBodyId.value,
                    contactPatchRuntime.sampleCount,
                    contactPatchRuntime.probeSpacingGameUnits,
                    contactPatchRuntime.probeRadiusGameUnits,
                    contactPatchRuntime.probeScale,
                    contactPatchRuntime.probeScaleReason,
                    contactPatchRuntime.castHitCount,
                    contactPatchRuntime.rejectedBodyHits,
                    contactPatchRuntime.rejectedInvalidNormals,
                    contactPatchRuntime.exactBodySamples,
                    contactPatchRuntime.meshRecoveredSamples,
                    contactPatchRuntime.rawAcceptedSampleCount,
                    contactPatchRuntime.clusterRejectedSampleCount,
                    contactPatchRuntime.clusterDepthSpreadGameUnits,
                    contactPatchRuntime.clusterMaxLateralGameUnits,
                    contactPatchRuntime.clusterReason,
                    formatContactPatchRejectedBodies(contactPatchRuntime),
                    contactPatchRuntime.patch.fallbackReason ? contactPatchRuntime.patch.fallbackReason : "unknown");
            }
        }
        if (palmPocketMeshAvailable &&
            pivotAuthoritySourceCanYieldToPalmPocket(inferGrabPivotAuthoritySource(grabPointMode, pivotAuthorityPositionOnly))) {
            const float canonicalPocketDistance =
                acquisitionPocket.valid ? pointDistanceGameUnits(canonicalPivotPointWorld, acquisitionPocket.palmCenterWorld) : std::numeric_limits<float>::max();
            const float palmPocketDistance =
                acquisitionPocket.valid ? pointDistanceGameUnits(palmPocketSurfaceHit.position, acquisitionPocket.palmCenterWorld) : std::numeric_limits<float>::max();
            const float selectionDeltaLimit =
                finitePositiveOr(g_rockConfig.rockGrabAlignmentMaxSelectionToMeshDistance, 8.0f) +
                finitePositiveOr(g_rockConfig.rockGrabContactPatchProbeSpacingGameUnits, 3.0f);
            const bool selectionCoherent =
                !sel.hasHitPoint || palmPocketSurfaceHit.selectionToMeshDistanceGameUnits <= selectionDeltaLimit;
            if (selectionCoherent && palmPocketDistance + 0.5f < canonicalPocketDistance) {
                grabGripPoint = palmPocketSurfaceHit.position;
                grabSurfaceHit = palmPocketSurfaceHit;
                surfaceOwnerNode = grabSurfaceHit.sourceNode;
                selectionToMeshDistanceGameUnits = grabSurfaceHit.selectionToMeshDistanceGameUnits;
                surfaceOwnerMatchesResolvedBody = true;
                meshGrabFound = true;
                grabPointMode = "palmPocketMeshSurface";
                grabFallbackReason = "palmPocketMeshPointImprovesSeat";
                const auto palmPocketFallbackAuthority = assessGrabPivotAuthorityCandidate(
                    GrabPivotAuthorityCandidateInput{
                        .mode = grabPointMode,
                        .point = grabGripPoint,
                        .normal = grabSurfaceHit.normal,
                        .normalTrusted = true,
                        .positionOnlyPatch = false,
                        .positionConfidence = 0.85f,
                        .pocket = &acquisitionPocket,
                        .selection = &sel,
                        .authorityPoint = canonicalPivotPointWorld,
                        .hasAuthorityPoint = canonicalPivotAvailable,
                        .objectWorldTransform = objectWorldTransform,
                        .localMeshTriangles = &grabLocalMeshTriangles,
                    });
                pivotAuthoritySource = grabPivotAuthoritySourceName(palmPocketFallbackAuthority.source);
                pivotAuthorityNormalTrusted = palmPocketFallbackAuthority.normalTrusted;
                pivotAuthorityPositionOnly = palmPocketFallbackAuthority.positionOnlyPatch;
                pivotAuthorityPositionConfidence = palmPocketFallbackAuthority.positionConfidence;
                pivotAuthorityPocketDistanceGameUnits = palmPocketFallbackAuthority.pivotToPocketGameUnits;
                pivotAuthoritySelectionDeltaGameUnits = palmPocketFallbackAuthority.selectionDeltaGameUnits;
                pivotAuthorityLongLeverGameUnits = palmPocketFallbackAuthority.longLeverGameUnits;
                ROCK_LOG_DEBUG(Hand,
                    "{} hand PIVOT AUTHORITY: source={} mode={} point=({:.1f},{:.1f},{:.1f}) pocketDistance={:.2f}gu selectionDelta={:.2f}gu longLever={:.2f}gu reason={} canonical={} canonicalPocket={:.2f}gu",
                    handName(),
                    pivotAuthoritySource,
                    grabPointMode,
                    grabGripPoint.x,
                    grabGripPoint.y,
                    grabGripPoint.z,
                    pivotAuthorityPocketDistanceGameUnits,
                    pivotAuthoritySelectionDeltaGameUnits,
                    pivotAuthorityLongLeverGameUnits,
                    grabFallbackReason,
                    canonicalPivotMode,
                    canonicalPocketDistance);
            }
        }
        if (std::strcmp(pivotAuthoritySource, "notEvaluated") == 0) {
            const auto inferredSource = inferGrabPivotAuthoritySource(grabPointMode);
            pivotAuthoritySource = grabPivotAuthoritySourceName(inferredSource);
            pivotAuthorityNormalTrusted = grabSurfaceHit.valid;
            pivotAuthorityPositionOnly = false;
            pivotAuthorityPositionConfidence = grabSurfaceHit.valid ? 1.0f : 0.0f;
            pivotAuthorityPocketDistanceGameUnits =
                acquisitionPocket.valid ? pointDistanceGameUnits(grabGripPoint, acquisitionPocket.palmCenterWorld) : std::numeric_limits<float>::max();
            pivotAuthoritySelectionDeltaGameUnits =
                sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint) : std::numeric_limits<float>::max();
        }

        if (!meshGrabFound && !sel.hasHitPoint && !authoredGrabNode && !handPocketOnlyGrab) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: no object-side contact point for '{}' formID={:08X}; object origin/COM fallback is not valid dynamic grab authority reason={} meshNode='{}' ownerNode='{}' rootNode='{}'",
                handName(),
                objName,
                sel.refr->GetFormID(),
                grabFallbackReason,
                nodeDebugName(meshSourceNode),
                nodeDebugName(collidableNode),
                nodeDebugName(rootNode));
            return abortGrab();
        }


        return true;
    }

    bool Hand::evaluateGrabContactEvidence(hand_grab_detail::GrabAcquisitionContext& context)
    {
        auto* world = context.world;
        const auto& sel = context.selection;
        const auto& handWorldTransform = context.handWorldTransform;
        const bool handPocketOnlyGrab = context.handPocketOnlyGrab;
        const bool grabbedFromPullCatch = context.grabbedFromPullCatch;
        const bool looseWeaponGrab = context.looseWeaponGrab;
        const auto objectBodyId = context.objectBodyId;
        auto* rootNode = context.rootNode;
        const auto& objName = context.objectName;
        const auto& palmPocketPivotAWorld = context.palmPocketPivotAWorld;
        const auto& preparedBodySet = context.preparedBodySet;
        auto* collidableNode = context.collidableNode;
        auto* meshSourceNode = context.meshSourceNode;
        const auto& objectWorldTransform = context.objectWorldTransform;
        auto& grabGripPoint = context.grabGripPoint;
        auto& selectionToMeshDistanceGameUnits = context.selectionToMeshDistanceGameUnits;
        auto& meshGrabFound = context.meshGrabFound;
        const auto& meshStats = context.meshStats;
        const auto& grabSurfaceTriangles = context.grabSurfaceTriangles;
        const auto& grabLocalMeshTriangles = context.grabLocalMeshTriangles;
        auto& grabSurfaceHit = context.grabSurfaceHit;
        auto& contactPatchRuntime = context.contactPatchRuntime;
        auto& multiFingerGripRuntime = context.multiFingerGripRuntime;
        auto& palmSeatPointWorld = context.palmSeatPointWorld;
        auto& fingerEvidencePointWorld = context.fingerEvidencePointWorld;
        auto& palmSeatSurfaceHit = context.palmSeatSurfaceHit;
        auto& fingerEvidenceSurfaceHit = context.fingerEvidenceSurfaceHit;
        auto& contactPatchEvidenceAvailable = context.contactPatchEvidenceAvailable;
        auto& multiFingerGripUsed = context.multiFingerGripUsed;
        auto& palmSeatPointValid = context.palmSeatPointValid;
        auto& fingerEvidencePointValid = context.fingerEvidencePointValid;
        auto& activeGrabPointUsesMultiFingerEvidence = context.activeGrabPointUsesMultiFingerEvidence;
        auto& contactPatchPivotAuthorityReason = context.contactPatchPivotAuthorityReason;
        auto& pivotAuthoritySource = context.pivotAuthoritySource;
        auto& pivotAuthorityNormalTrusted = context.pivotAuthorityNormalTrusted;
        auto& pivotAuthorityPositionOnly = context.pivotAuthorityPositionOnly;
        auto& pivotAuthorityPositionConfidence = context.pivotAuthorityPositionConfidence;
        auto& pivotAuthorityPocketDistanceGameUnits = context.pivotAuthorityPocketDistanceGameUnits;
        auto& pivotAuthoritySelectionDeltaGameUnits = context.pivotAuthoritySelectionDeltaGameUnits;
        const auto authoredGrabNode = context.authoredGrabNode;
        const bool relaxedArticulatedAuthority = context.relaxedArticulatedAuthority;
        const bool canonicalPivotAvailable = context.canonicalPivotAvailable;
        const bool visualMeshPivotAvailable = context.visualMeshPivotAvailable;
        auto& grabPointMode = context.grabPointMode;
        auto& grabFallbackReason = context.grabFallbackReason;
        auto& palmSeatPointMode = context.palmSeatPointMode;
        auto& palmSeatFallbackReason = context.palmSeatFallbackReason;
        auto& fingerEvidencePointMode = context.fingerEvidencePointMode;
        auto& fingerEvidenceFallbackReason = context.fingerEvidenceFallbackReason;
        const auto grabContactQualityMode = context.grabContactQualityMode;
        const bool multiFingerEvidenceEnabled = context.multiFingerEvidenceEnabled;
        const bool hybridFingerProbeEvidenceEnabled = context.hybridFingerProbeEvidenceEnabled;
        const auto& semanticContacts = context.semanticContacts;
        auto& pinchPocketCandidate = context.pinchPocketCandidate;

        auto abortGrab = [&]() {
            return abortGrabAcquisition(context);
        };
        auto failHandPocketOnlyGrab = [&]() {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: hand-pocket-only target requires pinch or palm-pocket support authority for '{}' formID={:08X}; targetKind={} meshNode='{}' ownerNode='{}' rootNode='{}' shapes={} totalTris={} reason={}",
                handName(),
                objName,
                sel.refr->GetFormID(),
                grab_target::name(sel.targetKind),
                nodeDebugName(meshSourceNode),
                nodeDebugName(collidableNode),
                nodeDebugName(rootNode),
                meshStats.visitedShapes,
                meshStats.totalTriangles(),
                grabFallbackReason);
            return abortGrab();
        };

        pinchPocketCandidate = buildRuntimePinchPocketCandidate(
            sel,
            preparedBodySet,
            objectBodyId.value,
            objectWorldTransform,
            grabSurfaceTriangles,
            grabLocalMeshTriangles,
            grabGripPoint,
            handWorldTransform,
            _isLeft,
            !sel.isFarSelection && !grabbedFromPullCatch,
            handPocketOnlyGrab,
            authoredGrabNode != nullptr,
            looseWeaponGrab);
        if (pinchPocketCandidate.valid) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand PINCH POCKET candidate accepted: reason={} pocket=({:.1f},{:.1f},{:.1f}) point=({:.1f},{:.1f},{:.1f}) dir=({:.2f},{:.2f},{:.2f}) gap={:.2f}gu dist={:.2f}gu extents=({:.2f},{:.2f},{:.2f})",
                handName(),
                pinchPocketCandidate.decision.reason,
                pinchPocketCandidate.pinchPocketWorld.x,
                pinchPocketCandidate.pinchPocketWorld.y,
                pinchPocketCandidate.pinchPocketWorld.z,
                pinchPocketCandidate.surfaceHit.position.x,
                pinchPocketCandidate.surfaceHit.position.y,
                pinchPocketCandidate.surfaceHit.position.z,
                pinchPocketCandidate.pinchDetectionDirectionWorld.x,
                pinchPocketCandidate.pinchDetectionDirectionWorld.y,
                pinchPocketCandidate.pinchDetectionDirectionWorld.z,
                pinchPocketCandidate.thumbIndexGapGameUnits,
                pinchPocketCandidate.pocketToSurfaceDistanceGameUnits,
                pinchPocketCandidate.meshExtents.minExtentGameUnits,
                pinchPocketCandidate.meshExtents.middleExtentGameUnits,
                pinchPocketCandidate.meshExtents.maxExtentGameUnits);
        } else if (g_rockConfig.rockDebugGrabFrameLogging) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand PINCH POCKET candidate rejected: reason={} gap={:.2f}gu dist={:.2f}gu extentsValid={} extents=({:.2f},{:.2f},{:.2f}) close={} bodies={}",
                handName(),
                pinchPocketCandidate.decision.reason,
                pinchPocketCandidate.thumbIndexGapGameUnits,
                pinchPocketCandidate.pocketToSurfaceDistanceGameUnits,
                pinchPocketCandidate.meshExtents.valid ? "yes" : "no",
                pinchPocketCandidate.meshExtents.minExtentGameUnits,
                pinchPocketCandidate.meshExtents.middleExtentGameUnits,
                pinchPocketCandidate.meshExtents.maxExtentGameUnits,
                (!sel.isFarSelection && !grabbedFromPullCatch) ? "yes" : "no",
                preparedBodySet.acceptedCount());
        }
        if (sel.pinchCloseSelectionFallback && !pinchPocketCandidate.valid) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand GRAB rejected: pinch-direction close selection did not qualify for pinch pocket reason={} formID={:08X}",
                handName(),
                pinchPocketCandidate.decision.reason,
                sel.refr ? sel.refr->GetFormID() : 0);
            return abortGrab();
        }
        if (handPocketOnlyGrab &&
            !pinchPocketCandidate.valid &&
            !relaxedArticulatedAuthority &&
            (!meshGrabFound || std::strcmp(grabPointMode, "palmPocketMeshSurface") != 0)) {
            return failHandPocketOnlyGrab();
        }

        palmSeatPointWorld = grabGripPoint;
        palmSeatSurfaceHit = grabSurfaceHit;
        palmSeatPointMode = grabPointMode;
        palmSeatFallbackReason = grabFallbackReason;
        palmSeatPointValid = canonicalPivotAvailable;

        if (!pinchPocketCandidate.valid && multiFingerEvidenceEnabled) {
            std::vector<GrabSurfaceTriangleData> multiFingerSurfaceTriangles;
            // The grip builder works around the grip point, so cap to that instead.
            const auto& multiFingerTriangleSource = limitGrabSurfaceTrianglesNear(
                grabSurfaceTriangles, grabGripPoint, multiFingerSurfaceTriangles, handName(), "multiFinger");
            multiFingerGripRuntime = buildRuntimeMultiFingerGripContact(world,
                preparedBodySet,
                objectBodyId.value,
                objectWorldTransform,
                semanticContacts,
                multiFingerTriangleSource,
                this,
                hybridFingerProbeEvidenceEnabled);
            if (multiFingerGripRuntime.gripSet.valid) {
                multiFingerGripUsed = true;
                const auto& gripSet = multiFingerGripRuntime.gripSet;
                GrabSurfaceHit representativeHit{};
                for (const auto& group : gripSet.groups) {
                    if (!group.valid) {
                        continue;
                    }
                    const int index = grab_multi_finger_contact_math::fingerIndex(group.finger);
                    if (index >= 0 && static_cast<std::size_t>(index) < multiFingerGripRuntime.groupHits.size() &&
                        multiFingerGripRuntime.groupHits[static_cast<std::size_t>(index)].valid) {
                        representativeHit = multiFingerGripRuntime.groupHits[static_cast<std::size_t>(index)];
                        break;
                    }
                }

                fingerEvidencePointWorld = gripSet.contactCenterWorld;
                fingerEvidenceSurfaceHit = representativeHit;
                fingerEvidenceSurfaceHit.position = gripSet.contactCenterWorld;
                fingerEvidenceSurfaceHit.normal = gripSet.averageNormalWorld;
                fingerEvidenceSurfaceHit.distance = 0.0f;
                fingerEvidenceSurfaceHit.hasTriangle = representativeHit.valid && representativeHit.hasTriangle;
                fingerEvidenceSurfaceHit.hasSelectionHit = sel.hasHitPoint;
                fingerEvidenceSurfaceHit.selectionToMeshDistanceGameUnits =
                    sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, fingerEvidencePointWorld) : std::numeric_limits<float>::max();
                fingerEvidenceSurfaceHit.pivotToSurfaceDistanceGameUnits = pointDistanceGameUnits(palmPocketPivotAWorld, fingerEvidencePointWorld);
                fingerEvidenceSurfaceHit.resolvedOwnerMatchesBody = true;
                fingerEvidenceSurfaceHit.valid = true;
                fingerEvidencePointMode = "multiFingerContactPatch";
                fingerEvidenceFallbackReason = gripSet.reason;
                fingerEvidencePointValid = true;

                ROCK_LOG_DEBUG(Hand,
                    "{} hand MULTI-FINGER GRIP: body={} groups={} semanticGroups={} probeGroups={} candidates={} meshHits={} "
                    "semanticHits={} probeHits={} rejectOwner={} rejectDistance={} "
                    "handCenter=({:.1f},{:.1f},{:.1f}) contactCenter=({:.1f},{:.1f},{:.1f}) normal=({:.3f},{:.3f},{:.3f}) "
                    "spread={:.2f} reason={}",
                    handName(),
                    objectBodyId.value,
                    gripSet.groupCount,
                    multiFingerGripRuntime.semanticGroupCount,
                    multiFingerGripRuntime.liveProbeGroupCount,
                    multiFingerGripRuntime.candidateContactCount,
                    multiFingerGripRuntime.meshHitCount,
                    multiFingerGripRuntime.semanticMeshHitCount,
                    multiFingerGripRuntime.liveProbeMeshHitCount,
                    multiFingerGripRuntime.rejectedOwnerCount,
                    multiFingerGripRuntime.rejectedDistanceCount,
                    gripSet.handCenterWorld.x,
                    gripSet.handCenterWorld.y,
                    gripSet.handCenterWorld.z,
                    gripSet.contactCenterWorld.x,
                    gripSet.contactCenterWorld.y,
                    gripSet.contactCenterWorld.z,
                    gripSet.averageNormalWorld.x,
                    gripSet.averageNormalWorld.y,
                    gripSet.averageNormalWorld.z,
                    gripSet.spreadGameUnits,
                    gripSet.reason);
            } else if (g_rockConfig.rockDebugGrabFrameLogging) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand MULTI-FINGER GRIP rejected: body={} groups={} semanticGroups={} probeGroups={} candidates={} meshHits={} "
                    "semanticHits={} probeHits={} rejectOwner={} rejectDistance={} reason={}",
                    handName(),
                    objectBodyId.value,
                    multiFingerGripRuntime.gripSet.groupCount,
                    multiFingerGripRuntime.semanticGroupCount,
                    multiFingerGripRuntime.liveProbeGroupCount,
                    multiFingerGripRuntime.candidateContactCount,
                    multiFingerGripRuntime.meshHitCount,
                    multiFingerGripRuntime.semanticMeshHitCount,
                    multiFingerGripRuntime.liveProbeMeshHitCount,
                    multiFingerGripRuntime.rejectedOwnerCount,
                    multiFingerGripRuntime.rejectedDistanceCount,
                    multiFingerGripRuntime.reason ? multiFingerGripRuntime.reason : "unknown");
            }
        }

        grab_contact_evidence_policy::GrabContactEvidenceDecision contactEvidenceDecision{};
        if (!pinchPocketCandidate.valid && multiFingerEvidenceEnabled) {
            grab_contact_evidence_policy::GrabContactEvidenceInput evidenceInput{};
            evidenceInput.qualityMode = static_cast<int>(grabContactQualityMode);
            evidenceInput.multiFingerValidationEnabled = g_rockConfig.rockGrabMultiFingerContactValidationEnabled;
            evidenceInput.contactPatchAccepted = contactPatchEvidenceAvailable;
            evidenceInput.contactPatchMeshSnapped = contactPatchRuntime.meshSnapped;
            evidenceInput.contactPatchReliable = contactPatchRuntime.normalTrusted;
            evidenceInput.contactPatchNormalTrusted = contactPatchRuntime.normalTrusted;
            evidenceInput.contactPatchPositionOnly = contactPatchRuntime.positionOnly;
            evidenceInput.contactPatchConfidence =
                contactPatchRuntime.positionOnly ? 0.0f : contactPatchRuntime.patch.confidence;
            evidenceInput.meshSurfacePivotAccepted = visualMeshPivotAvailable;
            evidenceInput.multiFingerGripValid = multiFingerGripRuntime.gripSet.valid;
            evidenceInput.semanticFingerGroups = multiFingerGripRuntime.semanticGroupCount;
            evidenceInput.probeFingerGroups = multiFingerGripRuntime.liveProbeGroupCount;
            evidenceInput.combinedFingerGroups = multiFingerGripRuntime.gripSet.groupCount;
            evidenceInput.minimumFingerGroups =
                static_cast<std::uint32_t>((std::max)(1, g_rockConfig.rockGrabMinFingerContactGroups));
            contactEvidenceDecision = grab_contact_evidence_policy::evaluateGrabContactEvidence(evidenceInput);

            ROCK_LOG_DEBUG(Hand,
                "{} hand CONTACT EVIDENCE: mode={} accept={} level={} reason={} patch={} meshSnap={} reliable={} normalTrusted={} positionOnly={} confidence={:.2f}/{:.2f} "
                "multiFingerValid={} semanticGroups={} probeGroups={} combinedGroups={} minGroups={} useMultiFingerPivot={}",
                handName(),
                grab_contact_evidence_policy::contactQualityModeName(grabContactQualityMode),
                contactEvidenceDecision.accept ? "yes" : "no",
                grab_contact_evidence_policy::contactEvidenceLevelName(contactEvidenceDecision.level),
                contactEvidenceDecision.reason,
                contactPatchEvidenceAvailable ? "yes" : "no",
                contactPatchRuntime.meshSnapped ? "yes" : "no",
                contactPatchRuntime.patch.orientationReliable ? "yes" : "no",
                contactPatchRuntime.normalTrusted ? "yes" : "no",
                contactPatchRuntime.positionOnly ? "yes" : "no",
                contactPatchRuntime.patch.confidence,
                evidenceInput.contactPatchConfidence,
                multiFingerGripRuntime.gripSet.valid ? "yes" : "no",
                multiFingerGripRuntime.semanticGroupCount,
                multiFingerGripRuntime.liveProbeGroupCount,
                multiFingerGripRuntime.gripSet.groupCount,
                g_rockConfig.rockGrabMinFingerContactGroups,
                contactEvidenceDecision.useMultiFingerPivot ? "yes" : "no");
        }

        if (!pinchPocketCandidate.valid && multiFingerEvidenceEnabled && !contactEvidenceDecision.accept) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: contact evidence rejected '{}' formID={:08X}; "
                "mode={} level={} groups={} semanticGroups={} probeGroups={} candidates={} meshHits={} rejectOwner={} rejectDistance={} reason={} selectionFar={}",
                handName(),
                objName,
                sel.refr->GetFormID(),
                grab_contact_evidence_policy::contactQualityModeName(grabContactQualityMode),
                grab_contact_evidence_policy::contactEvidenceLevelName(contactEvidenceDecision.level),
                multiFingerGripRuntime.gripSet.groupCount,
                multiFingerGripRuntime.semanticGroupCount,
                multiFingerGripRuntime.liveProbeGroupCount,
                multiFingerGripRuntime.candidateContactCount,
                multiFingerGripRuntime.meshHitCount,
                multiFingerGripRuntime.rejectedOwnerCount,
                multiFingerGripRuntime.rejectedDistanceCount,
                contactEvidenceDecision.reason,
                sel.isFarSelection ? "yes" : "no");
            return abortGrab();
        }

        grabSurfaceHit = palmSeatSurfaceHit;
        grabGripPoint = palmSeatPointWorld;
        grabPointMode = palmSeatPointMode;
        grabFallbackReason = palmSeatFallbackReason;
        selectionToMeshDistanceGameUnits =
            palmSeatSurfaceHit.valid ? palmSeatSurfaceHit.selectionToMeshDistanceGameUnits : selectionToMeshDistanceGameUnits;
        activeGrabPointUsesMultiFingerEvidence = false;

        ROCK_LOG_DEBUG(Hand,
            "{} hand GRAB POINT EVIDENCE: handPocket=palmSeat activeMode={} activeUsesFingerEvidence={} "
            "contactPatchEvidence={} contactPatchPivot={} contactPatchReason={} "
            "pivotAuthoritySource={} positionOnlyPatch={} normalTrusted={} positionConfidence={:.2f} authorityPocket={:.2f}gu authoritySelection={:.2f}gu "
            "palmSeatValid={} palmSeatMode={} palmSeat=({:.1f},{:.1f},{:.1f}) "
            "fingerEvidenceValid={} fingerEvidenceMode={} fingerEvidence=({:.1f},{:.1f},{:.1f})",
            handName(),
            grabPointMode,
            activeGrabPointUsesMultiFingerEvidence ? "yes" : "no",
            contactPatchEvidenceAvailable ? "yes" : "no",
            "no",
            contactPatchPivotAuthorityReason,
            pivotAuthoritySource,
            pivotAuthorityPositionOnly ? "yes" : "no",
            pivotAuthorityNormalTrusted ? "yes" : "no",
            pivotAuthorityPositionConfidence,
            pivotAuthorityPocketDistanceGameUnits,
            pivotAuthoritySelectionDeltaGameUnits,
            palmSeatPointValid ? "yes" : "no",
            palmSeatPointMode,
            palmSeatPointWorld.x,
            palmSeatPointWorld.y,
            palmSeatPointWorld.z,
            fingerEvidencePointValid ? "yes" : "no",
            fingerEvidencePointMode,
            fingerEvidencePointWorld.x,
            fingerEvidencePointWorld.y,
            fingerEvidencePointWorld.z);


        return true;
    }

    bool Hand::grabSelectedObject(RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform,
        float tau,
        float damping,
        float maxForce,
        float proportionalRecovery,
        float constantRecovery,
        const BodyBoneColliderSet* bodyBoneColliders,
        const GrabSharedObjectContext& sharedContext)
    {
        hand_grab_detail::GrabAcquisitionContext context{
            .world = world,
            .selection = _currentSelection,
            .sharedContext = &sharedContext,
            .bodyBoneColliders = bodyBoneColliders,
            .handWorldTransform = handWorldTransform,
            .tau = tau,
            .damping = damping,
            .maxForce = maxForce,
            .proportionalRecovery = proportionalRecovery,
            .constantRecovery = constantRecovery,
        };
        if (!prepareGrabBodySet(context)) {
            return false;
        }

        if (!resolveGrabCaptureFrames(context)) {
            return false;
        }
        if (!extractGrabMesh(context)) {
            return false;
        }
        if (!resolvePrimaryGrabBody(context)) {
            return false;
        }
        if (!resolveGrabPivotAuthority(context)) {
            return false;
        }
        if (!evaluateGrabContactEvidence(context)) {
            return false;
        }
        if (!captureCanonicalGrabFrame(context)) {
            return false;
        }
        if (!commitGrabDrive(context)) {
            return false;
        }

        publishGrabFingerPose(context);
        applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::GrabCommitSucceeded });
        clearPullRuntimeState();
        clearPullCatchIntent(context.grabbedFromPullCatch ? "pullCatchGrabbed" : "grabbed");

        ROCK_LOG_INFO(Hand, "{} hand grab success -> HeldInit: bodyId={}", handName(), context.objectBodyId.value);
        return true;
    }
}
