#include "physics-interaction/hand/Hand.h"
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
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
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



    namespace hand_grab_detail
    {
        struct GrabAcquisitionContext
        {
            // Phase inputs stay valid for this synchronous acquisition call.
            RE::hknpWorld* world = nullptr;
            RE::bhkWorld* bhkWorld = nullptr;
            SelectedObject selection{};
            const GrabSharedObjectContext* sharedContext = nullptr;
            const BodyBoneColliderSet* bodyBoneColliders = nullptr;
            RE::NiAVObject* rootNode = nullptr;
            active_grab_body_lifecycle::BodyLifecycleSnapshot activeLifecycle{};

            // Drive tuning is fixed for the full acquisition.
            RE::NiTransform handWorldTransform{};
            float tau = 0.0f;
            float damping = 0.0f;
            float maxForce = 0.0f;
            float proportionalRecovery = 0.0f;
            float constantRecovery = 0.0f;

            // Body-set ownership controls failure unwind and final commit.
            RE::hknpBodyId objectBodyId{};
            std::uint16_t selectedOriginalMotionPropsId = 1;
            bool joiningPeerHeldObject = false;
            bool grabbedFromPullCatch = false;
            bool consumedPullPrepLifecycle = false;
            bool looseWeaponGrab = false;
            bool handPocketOnlyGrab = false;
            std::string objectName{ "(unnamed)" };
            const char* motionType = "UNKNOWN";
            std::uint64_t grabTraceId = 0;
            std::uint32_t bodySetSeedBodyId = object_physics_body_set::INVALID_BODY_ID;
            object_physics_body_set::ObjectPhysicsBodySet beforePrepBodySet;
            object_physics_body_set::ObjectPhysicsBodySet preparedBodySet;
            bool beforePrepScanCacheHit = false;
            bool preparedScanCacheHit = false;
            bool preparedBodySetPostPrepComplete = false;
            bool motionConverted = true;
            bool collisionEnabled = true;

            // Capture frames define the palm and object authority spaces.
            RE::NiTransform handBodyWorldAtGrab{};
            RE::NiTransform proxyFrameWorldAtGrab{};
            const char* proxyFrameSourceAtGrab = "unresolved";
            bool hasPalmProxyFrameAtGrab = false;
            RE::NiPoint3 grabAuthorityPivotAWorld{};
            RE::NiPoint3 palmPocketPivotAWorld{};
            float palmPocketToProxyDeltaGameUnits = 0.0f;
            GrabPalmBasisDelta grabPalmBasisDelta{};
            RE::NiPoint3 grabPivotAForPrimaryChoice{};
            RE::NiTransform proxyAuthorityFrameWorldAtGrab{};
            RE::NiAVObject* collidableNode = nullptr;
            RE::NiAVObject* meshSourceNode = nullptr;
            RE::NiTransform objectWorldTransform{};

            // Mesh capture owns geometry and the initial surface evidence.
            RE::NiPoint3 grabGripPoint{};
            float selectionToMeshDistanceGameUnits = 0.0f;
            bool meshGrabFound = false;
            MeshExtractionStats meshStats{};
            std::vector<TriangleData> grabMeshTriangles;
            std::vector<TriangleData> grabFingerPoseMeshTriangles;
            std::vector<GrabSurfaceTriangleData> grabSurfaceTriangles;
            std::vector<GrabLocalTriangle> grabLocalMeshTriangles;
            std::vector<GrabLocalTriangle> grabFingerPoseLocalMeshTriangles;
            GrabSurfaceHit grabSurfaceHit{};
            RuntimeGrabContactPatch contactPatchRuntime{};
            RuntimeMultiFingerGripContact multiFingerGripRuntime{};
            RE::NiPoint3 palmSeatPointWorld{};
            RE::NiPoint3 fingerEvidencePointWorld{};
            GrabSurfaceHit palmSeatSurfaceHit{};
            GrabSurfaceHit fingerEvidenceSurfaceHit{};
            bool contactPatchEvidenceAvailable = false;
            bool multiFingerGripUsed = false;
            bool palmSeatPointValid = false;
            bool fingerEvidencePointValid = false;
            bool activeGrabPointUsesMultiFingerEvidence = false;
            const char* contactPatchPivotAuthorityReason = "notEvaluated";
            const char* pivotAuthoritySource = "notEvaluated";
            bool pivotAuthorityNormalTrusted = false;
            bool pivotAuthorityPositionOnly = false;
            float pivotAuthorityPositionConfidence = 0.0f;
            float pivotAuthorityPocketDistanceGameUnits = std::numeric_limits<float>::max();
            float pivotAuthoritySelectionDeltaGameUnits = std::numeric_limits<float>::max();
            float pivotAuthorityLongLeverGameUnits = 0.0f;
            RE::NiAVObject* surfaceOwnerNode = nullptr;
            RE::NiAVObject* authoredGrabNode = nullptr;
            bool meshContactOnly = false;
            bool hasMeshSurfaceContact = false;
            const char* grabPointMode = "none";
            const char* grabFallbackReason = "none";
            const char* palmSeatPointMode = "none";
            const char* palmSeatFallbackReason = "none";
            const char* fingerEvidencePointMode = "none";
            const char* fingerEvidenceFallbackReason = "none";

            // Body resolution binds the selected surface to one drive scope.
            grab_contact_evidence_policy::GrabContactQualityMode grabContactQualityMode =
                grab_contact_evidence_policy::GrabContactQualityMode::LegacyPermissive;
            grab_contact_source_policy::GrabContactSourcePolicy contactSourcePolicy{};
            bool multiFingerEvidenceEnabled = false;
            bool hybridFingerProbeEvidenceEnabled = false;
            object_physics_body_set::PrimaryBodyChoice primaryChoice{};
            bool surfaceOwnerMatchesResolvedBody = true;
            mechanical_connected_body_set::MechanicalScope mechanicalScope{};
            bool relaxedArticulatedAuthority = false;
            bool visualMeshPivotAvailable = false;
            bool canonicalPivotAvailable = false;
            RE::NiPoint3 canonicalPivotPointWorld{};
            RE::NiPoint3 canonicalPivotNormalWorld{};
            const char* canonicalPivotMode = "none";

            // Contact evidence supplies the canonical capture inputs.
            hand_semantic_contact_state::SemanticContactCollection semanticContacts{};
            grab_three_phase::GrabPocketFrame acquisitionPocket{};
            GrabSurfaceHit palmPocketSurfaceHit{};
            bool palmPocketMeshAvailable = false;
            RuntimePinchPocketCandidate pinchPocketCandidate{};

            // Final capture results feed the drive and finger-pose publish.
            ResolvedGrabOffsetSource resolvedGrabOffsetSource{};
        };
    }

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

    bool Hand::captureCanonicalGrabFrame(hand_grab_detail::GrabAcquisitionContext& context)
    {
        auto* world = context.world;
        const auto& sel = context.selection;
        const auto& sharedContext = *context.sharedContext;
        const bool joiningPeerHeldObject = context.joiningPeerHeldObject;
        const bool looseWeaponGrab = context.looseWeaponGrab;
        const bool grabbedFromPullCatch = context.grabbedFromPullCatch;
        auto& objectBodyId = context.objectBodyId;
        auto* rootNode = context.rootNode;
        const auto& objName = context.objectName;
        const auto motionTypeStr = context.motionType;
        const auto grabTraceId = context.grabTraceId;
        const auto& preparedBodySet = context.preparedBodySet;
        const auto& handWorldTransform = context.handWorldTransform;
        const auto& handBodyWorldAtGrab = context.handBodyWorldAtGrab;
        const auto& proxyFrameWorldAtGrab = context.proxyFrameWorldAtGrab;
        const auto proxyFrameSourceAtGrab = context.proxyFrameSourceAtGrab;
        const bool hasPalmProxyFrameAtGrab = context.hasPalmProxyFrameAtGrab;
        const auto& proxyAuthorityFrameWorldAtGrab = context.proxyAuthorityFrameWorldAtGrab;
        const auto& palmPocketPivotAWorld = context.palmPocketPivotAWorld;
        auto*& collidableNode = context.collidableNode;
        auto* meshSourceNode = context.meshSourceNode;
        auto& objectWorldTransform = context.objectWorldTransform;
        auto& grabGripPoint = context.grabGripPoint;
        auto& selectionToMeshDistanceGameUnits = context.selectionToMeshDistanceGameUnits;
        const auto& grabMeshTriangles = context.grabMeshTriangles;
        auto& grabFingerPoseMeshTriangles = context.grabFingerPoseMeshTriangles;
        const auto& grabSurfaceTriangles = context.grabSurfaceTriangles;
        auto& grabLocalMeshTriangles = context.grabLocalMeshTriangles;
        auto& grabFingerPoseLocalMeshTriangles = context.grabFingerPoseLocalMeshTriangles;
        auto& grabSurfaceHit = context.grabSurfaceHit;
        auto& contactPatchRuntime = context.contactPatchRuntime;
        auto& multiFingerGripRuntime = context.multiFingerGripRuntime;
        const auto& fingerEvidencePointWorld = context.fingerEvidencePointWorld;
        auto& contactPatchEvidenceAvailable = context.contactPatchEvidenceAvailable;
        auto& multiFingerGripUsed = context.multiFingerGripUsed;
        const auto fingerEvidencePointValid = context.fingerEvidencePointValid;
        auto& pivotAuthoritySource = context.pivotAuthoritySource;
        auto& pivotAuthorityNormalTrusted = context.pivotAuthorityNormalTrusted;
        auto& pivotAuthorityPositionOnly = context.pivotAuthorityPositionOnly;
        auto& pivotAuthorityPositionConfidence = context.pivotAuthorityPositionConfidence;
        auto& pivotAuthorityPocketDistanceGameUnits = context.pivotAuthorityPocketDistanceGameUnits;
        auto& pivotAuthoritySelectionDeltaGameUnits = context.pivotAuthoritySelectionDeltaGameUnits;
        auto& pivotAuthorityLongLeverGameUnits = context.pivotAuthorityLongLeverGameUnits;
        const auto& mechanicalScope = context.mechanicalScope;
        const auto& primaryChoice = context.primaryChoice;
        const auto authoredGrabNode = context.authoredGrabNode;
        const auto& canonicalPivotNormalWorld = context.canonicalPivotNormalWorld;
        const auto& semanticContacts = context.semanticContacts;
        const auto& palmPocketSurfaceHit = context.palmPocketSurfaceHit;
        const bool palmPocketMeshAvailable = context.palmPocketMeshAvailable;
        const auto& pinchPocketCandidate = context.pinchPocketCandidate;
        auto& grabPointMode = context.grabPointMode;
        auto& grabFallbackReason = context.grabFallbackReason;
        auto& resolvedGrabOffsetSource = context.resolvedGrabOffsetSource;

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

        logRuntimeScaleIfChanged(_isLeft, handName(), handWorldTransform, collidableNode);

        ROCK_LOG_INFO(Hand, "{} hand GRAB: '{}' formID={:08X} bodyId={}", handName(), objName, sel.refr->GetFormID(), objectBodyId.value);

        if (g_rockConfig.rockDebugShowGrabNotifications) {
            auto msg = std::format("[ROCK] {} GRAB: {} ({})", _isLeft ? "L" : "R", objName, motionTypeStr);
            f4vr::showNotification(msg);
        }

        bool adoptedPeerHeldBodySet = false;
        _heldBodyIds = buildCommittedHeldBodyIds(objectBodyId.value, mechanicalScope.committedBodyIds, sharedContext, adoptedPeerHeldBodySet);
        if (_heldBodyIds.empty()) {
            _heldBodyIds.push_back(objectBodyId.value);
        }
        _heldDriveDecision = mechanicalScope.driveDecision;
        if (adoptedPeerHeldBodySet) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand SHARED held body set adopted: primaryBody={} peerBodies={} committedBodies={} scanAccepted={} mechanicalKind={} driveMode={} driveReason={}",
                handName(),
                objectBodyId.value,
                sharedContext.peerHeldBodyIds ? sharedContext.peerHeldBodyIds->size() : 0,
                _heldBodyIds.size(),
                preparedBodySet.acceptedCount(),
                mechanical_connected_body_set::scopeKindName(mechanicalScope.kind),
                held_object_drive_policy::modeName(_heldDriveDecision.mode),
                _heldDriveDecision.reason);
        }

        {
            auto* player = RE::PlayerCharacter::GetSingleton();
            if (player && !joiningPeerHeldObject) {
                nativeVRGrabDrop(player, 0);
                nativeVRGrabDrop(player, 1);
            }
        }

        _grabStartTime = 0.0f;
        _grabConvergeStableInsidePocketFrames = 0;
        _grabConvergePreviousGripErrorGameUnits = std::numeric_limits<float>::max();

        {
            const RE::NiPoint3 palmPos = computeGrabLegacyPalmPivotAWorldFromHandBasis(handWorldTransform, _isLeft);
            RE::NiTransform grabBodyWorldAtGrab{};
            if (!tryGetGrabAuthorityBodyWorldTransform(world, objectBodyId, grabBodyWorldAtGrab)) {
                ROCK_LOG_ERROR(Hand,
                    "{} hand GRAB FAILED: native grab BODY frame unreadable bodyId={} formID={:08X}",
                    handName(),
                    objectBodyId.value,
                    sel.refr ? sel.refr->GetFormID() : 0);
                return abortGrab();
            }
            RE::NiTransform motionBodyWorldAtGrab{};
            body_frame::BodyFrameSource motionBodySourceAtGrab = body_frame::BodyFrameSource::Fallback;
            const bool hasMotionBodyWorldAtGrab =
                tryResolveLiveBodyWorldTransform(world, objectBodyId, motionBodyWorldAtGrab, &motionBodySourceAtGrab);
            /*
             * Custom constraint body-B data is authored in the hknp BODY frame.
             * HIGGS does the same kind of thing on Skyrim's hkp side: it freezes
             * pivot B from the rigid body transform and treats COM/motion as mass
             * data, not as a local-frame owner. A runtime MOTION-frame experiment
             * made the object settle at the wrong rotation even when the proxy
             * target was stable, so the live motion transform remains diagnostic
             * evidence only.
             */
            const bool constraintUsesMotionBodyAtGrab = false;
            const RE::NiTransform constraintBodyWorldAtGrab = grabBodyWorldAtGrab;
            /*
             * The hidden proxy is body A for dynamic grab. The close-grab
             * pocket pivot is captured from the same generated/proxy-local
             * seat frame. The proxy body keeps its seat offset, and transform A
             * carries the selected pivot as an explicit local point on body A.
             */
            RE::NiPoint3 grabPivotAWorld = palmPocketPivotAWorld;
            auto* ownerCellAtGrab = sel.refr ? sel.refr->GetParentCell() : nullptr;
            auto* bhkWorldAtGrab = ownerCellAtGrab ? ownerCellAtGrab->GetbhkWorld() : nullptr;
            auto* bodyCollisionObjectAtGrab = bhkWorldAtGrab ? RE::bhkNPCollisionObject::Getbhk(bhkWorldAtGrab, objectBodyId) : nullptr;
            auto* ownerNodeAtGrab = bodyCollisionObjectAtGrab ? bodyCollisionObjectAtGrab->sceneObject : nullptr;
            if (ownerNodeAtGrab && ownerNodeAtGrab != collidableNode) {
                GrabCaptureTransformRefreshResult ownerAtGrabRefresh{};
                refreshGrabCaptureNodeTransform(ownerAtGrabRefresh, "ownerAtGrab", ownerNodeAtGrab);
                logGrabCaptureRefresh(ownerAtGrabRefresh);
                if (!ownerAtGrabRefresh.ok) {
                    ROCK_LOG_WARN(Hand,
                        "{} hand GRAB failed: owner-at-grab transform refresh produced a non-finite node transform for '{}' formID={:08X}; owner='{}'",
                        handName(),
                        objName,
                        sel.refr ? sel.refr->GetFormID() : 0,
                        nodeDebugName(ownerNodeAtGrab));
                    return abortGrab();
                }
            }
            const RE::NiTransform objectToBodyAtGrab = computeRuntimeBodyLocalTransform(objectWorldTransform, grabBodyWorldAtGrab);
            /*
             * ROCK dynamic grab has one production authority convention:
             * the generated/proxy palm frame seats the selected BODY-local grip
             * point and owns the object angular relation through a row-view of
             * its generated local axes. The custom constraint stores body-B local
             * data in the rigid BODY frame. MOTION and COM are mass/diagnostic
             * data only.
             */
            const RE::NiTransform ownerBodyLocalAtGrab =
                ownerNodeAtGrab ? computeRuntimeBodyLocalTransform(ownerNodeAtGrab->world, grabBodyWorldAtGrab) : makeIdentityTransform();
            const RE::NiTransform rootBodyLocalAtGrab =
                rootNode ? computeRuntimeBodyLocalTransform(rootNode->world, grabBodyWorldAtGrab) : makeIdentityTransform();
            RE::NiPoint3 selectedGripPointLocal = transform_math::worldPointToLocal(objectWorldTransform, grabGripPoint);
            RE::NiPoint3 selectedPivotBBodyLocalGame = transform_math::worldPointToLocal(grabBodyWorldAtGrab, grabGripPoint);
            if (!grabMeshTriangles.empty()) {
                grabFingerPoseMeshTriangles = selectNearestGrabFingerPoseTriangles(
                    grabMeshTriangles,
                    grabGripPoint,
                    kMaxGrabRuntimeFingerPoseTriangles);
                grabFingerPoseLocalMeshTriangles = cacheTrianglesInLocalSpace(grabFingerPoseMeshTriangles, objectWorldTransform);
                if (grabFingerPoseMeshTriangles.size() != grabMeshTriangles.size()) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand MESH FINGER POSE TRIANGLES: sourceTris={} localTris={} center=({:.1f},{:.1f},{:.1f})",
                        handName(),
                        grabMeshTriangles.size(),
                        grabFingerPoseMeshTriangles.size(),
                        grabGripPoint.x,
                        grabGripPoint.y,
                        grabGripPoint.z);
                }
            }
            RE::NiTransform desiredObjectWorld = objectWorldTransform;
            RE::NiTransform desiredBodyWorld = grabBodyWorldAtGrab;
            bool looseWeaponPrimaryAttachApplied = false;
            bool looseWeaponPrimaryAttachSourceVisible = false;
            const char* looseWeaponPrimaryAttachReason = "notEvaluated";
            const auto oppositionContacts = hand_semantic_contact_state::selectThumbOppositionContacts(semanticContacts);
            auto resolvedAuthorityPivotSourceForFreeze = grab_authority_frame_math::GrabAuthorityPivotSource::None;
            const char* resolvedAuthorityPivotReasonForFreeze = "notResolved";
            /*
             * Runtime grab authority is intentionally single-source. Mesh hits,
             * authored nodes, semantic contacts, contact patches, and multi-finger
             * contacts are evidence until the pivot resolver chooses one BODY-local
             * point to freeze. Final dynamic authority is strict: only a pinch
             * pocket or an authorable support group may create the frozen frame.
             */
            {
                auto pocket = grab_three_phase::buildGrabPocketFrameWithPalmCenter(
                    proxyAuthorityFrameWorldAtGrab,
                    _isLeft,
                    grabPivotAWorld,
                    g_rockConfig.rockGrabPocketDepthGameUnits,
                    g_rockConfig.rockGrabPocketRadiusGameUnits);
                auto gripArea = grab_three_phase::buildObjectGripArea(grab_three_phase::GripAreaInput{
                    .objectBodyWorld = grabBodyWorldAtGrab,
                    .contactSeedWorld = grabGripPoint,
                    .centerOfMassWorld = grabBodyWorldAtGrab.translate,
                    .interiorDirectionWorld = RE::NiPoint3{},
                    .preferredInsetGameUnits = 0.0f,
                    .maxInsetGameUnits = 0.0f,
                    .centerOfMassValid = false,
                    .interiorDirectionValid = false,
                    .source = grabPointMode,
                });
                if (pinchPocketCandidate.valid) {
                    gripArea = grab_three_phase::buildObjectGripArea(grab_three_phase::GripAreaInput{
                        .objectBodyWorld = grabBodyWorldAtGrab,
                        .contactSeedWorld = pinchPocketCandidate.surfaceHit.position,
                        .centerOfMassWorld = grabBodyWorldAtGrab.translate,
                        .interiorDirectionWorld = RE::NiPoint3{},
                        .preferredInsetGameUnits = 0.0f,
                        .maxInsetGameUnits = 0.0f,
                        .centerOfMassValid = false,
                        .interiorDirectionValid = false,
                        .source = "pinchPocket",
                    });
                }
                const RE::NiPoint3 phaseGripSeed = gripArea.valid ? gripArea.contactSeedWorld : grabGripPoint;
                const RE::NiPoint3 palmSeatAnchorWorld = pocket.valid ? pocket.palmCenterWorld : grabPivotAWorld;
                const RE::NiPoint3 gripToPocket = pocket.valid ? (phaseGripSeed - palmSeatAnchorWorld) : RE::NiPoint3{};
                const float gripToPocketDistance =
                    pocket.valid ? std::sqrt(gripToPocket.x * gripToPocket.x + gripToPocket.y * gripToPocket.y + gripToPocket.z * gripToPocket.z) :
                                   std::numeric_limits<float>::max();
                const float stableTouchEnvelope = (std::max)(g_rockConfig.rockGrabTouchAcquireDistanceGameUnits, pocket.pocketRadiusGameUnits);
                const bool hasStablePocketTouchContact =
                    oppositionContacts.valid && gripToPocketDistance <= stableTouchEnvelope;
                const bool meshSurfaceAuthorityEvidence =
                    grabSurfaceHit.valid && grabSurfaceHit.sourceKind != GrabSurfaceSourceKind::CollisionQuery;
                const bool pullArrivalTouchHeldAuthorityEvidence =
                    hasStablePocketTouchContact ||
                    meshSurfaceAuthorityEvidence ||
                    contactPatchEvidenceAvailable ||
                    multiFingerGripUsed ||
                    authoredGrabNode != nullptr;
                const auto phaseDecision = grab_three_phase::classifyAcquisitionPhase(grab_three_phase::PhaseClassificationInput{
                    .pocket = pocket,
                    .gripSeedWorld = phaseGripSeed,
                    .hasFreshTouchContact = hasStablePocketTouchContact,
                    .isFarSelection = sel.isFarSelection,
                    .programmaticArrival = sel.forcedArrival,
                    .requireEvidenceForTouchHeld = grabbedFromPullCatch,
                    .hasTouchHeldAuthorityEvidence = pullArrivalTouchHeldAuthorityEvidence,
                    .touchAcquireDistanceGameUnits = g_rockConfig.rockGrabTouchAcquireDistanceGameUnits,
                    .touchContactMaxDistanceGameUnits = stableTouchEnvelope,
                    .nearConvergeDistanceGameUnits = g_rockConfig.rockGrabNearConvergeDistanceGameUnits,
                    .behindPalmToleranceGameUnits = g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits,
                });

                const bool usingPinchPocket = pinchPocketCandidate.valid && gripArea.valid;
                const bool captureAccepted = usingPinchPocket || (pocket.valid && gripArea.valid && phaseDecision.accepted);
                if (captureAccepted) {
                    _grabObjectGripAtGrab = gripArea;
                    _grabAcquisitionPhase = usingPinchPocket ? grab_three_phase::AcquisitionPhase::TouchHeld : phaseDecision.phase;
                    if (usingPinchPocket) {
                        _grabObjectGripAtGrab.source = "pinchPocket";
                        _grabObjectGripAtGrab.fallbackReason = pinchPocketCandidate.decision.reason;
                        _grabObjectGripAtGrab.confidence = 0.95f;
                    }
                    const char* relationMode = usingPinchPocket ? "pinchPocket" : "rockPointToPalm";
                    const char* captureReason = usingPinchPocket ? pinchPocketCandidate.decision.reason : phaseDecision.reason;

                    if (usingPinchPocket) {
                        grabPivotAWorld = pinchPocketCandidate.pinchPocketWorld;
                    } else {
                        grabPivotAWorld = pocket.palmCenterWorld;
                    }
                    grabGripPoint = gripArea.contactSeedWorld;
                    grabPointMode = relationMode;
                    grabFallbackReason = captureReason;
                    if (usingPinchPocket) {
                        grabSurfaceHit = pinchPocketCandidate.surfaceHit;
                        grabSurfaceHit.hasSelectionHit = sel.hasHitPoint;
                        grabSurfaceHit.selectionToMeshDistanceGameUnits =
                            sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint) : std::numeric_limits<float>::max();
                        grabSurfaceHit.pivotToSurfaceDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                        grabSurfaceHit.resolvedOwnerMatchesBody = true;
                        selectionToMeshDistanceGameUnits = grabSurfaceHit.selectionToMeshDistanceGameUnits;
                        pivotAuthoritySource = grabPivotAuthoritySourceName(GrabPivotAuthoritySource::PinchPocketMeshPoint);
                        pivotAuthorityPositionOnly = false;
                        pivotAuthorityNormalTrusted = true;
                        pivotAuthorityPositionConfidence = 0.95f;
                        pivotAuthorityPocketDistanceGameUnits = grabSurfaceHit.pivotToSurfaceDistanceGameUnits;
                        pivotAuthoritySelectionDeltaGameUnits = grabSurfaceHit.selectionToMeshDistanceGameUnits;
                    }

                    auto firstValidNormal = [](std::initializer_list<RE::NiPoint3> candidates) {
                        for (const auto& candidate : candidates) {
                            const RE::NiPoint3 normal = normalizeOrZero(candidate);
                            if (lengthSquared(normal) > 0.000001f) {
                                return normal;
                            }
                        }
                        return RE::NiPoint3{};
                    };

                    /*
                     * Pivot B may be chosen from hand/proxy proximity, but normal
                     * authority must come from the object surface that produced the
                     * pivot. The palm/proxy normal is only a final fallback and
                     * tangent/sign guide; promoting it to face authority makes the
                     * frozen support frame follow the hand instead of the mesh.
                     */
                    RE::NiPoint3 gripNormalWorld = firstValidNormal({
                        grabSurfaceHit.valid ? grabSurfaceHit.normal : RE::NiPoint3{},
                        contactPatchRuntime.meshSnapped ? contactPatchRuntime.meshSnapHit.normal : RE::NiPoint3{},
                        contactPatchRuntime.normalTrusted ? contactPatchRuntime.patch.normal : RE::NiPoint3{},
                        palmPocketMeshAvailable ? palmPocketSurfaceHit.normal : RE::NiPoint3{},
                        canonicalPivotNormalWorld,
                        usingPinchPocket ? pinchPocketCandidate.pinchAxisWorld : RE::NiPoint3{},
                        pocket.palmNormalWorld,
                    });

                    RE::NiPoint3 gripEvidencePointWorld = grabGripPoint;
                    RE::NiPoint3 gripEvidenceNormalWorld = gripNormalWorld;
                    RuntimeGripSupportModel gripSupportRuntime{};
                    const float supportObjectScale =
                        std::isfinite(objectWorldTransform.scale) && objectWorldTransform.scale > 0.0f ? objectWorldTransform.scale : 1.0f;
                    const float supportLeverGameUnits =
                        !grabLocalMeshTriangles.empty() && grab_three_phase::isFinite(objectWorldTransform) ?
                            computeLocalMeshMaxDistanceFromPoint(
                                grabLocalMeshTriangles,
                                transform_math::worldPointToLocal(objectWorldTransform, grabGripPoint)) *
                                supportObjectScale :
                            0.0f;
                    if (!grabSurfaceTriangles.empty() && !grabLocalMeshTriangles.empty()) {
                        gripSupportRuntime = buildRuntimeGripSupportModel(sel,
                            preparedBodySet,
                            objectBodyId.value,
                            objectWorldTransform,
                            grabSurfaceTriangles,
                            grabLocalMeshTriangles,
                            contactPatchRuntime,
                            pinchPocketCandidate,
                            grabGripPoint,
                            gripNormalWorld,
                            grabPivotAWorld,
                            pocket.valid ? pocket.palmNormalWorld : gripNormalWorld,
                            pocket.valid ? pocket.fingerForwardWorld : RE::NiPoint3{},
                            pocket.valid ? pocket.crossPalmWorld : RE::NiPoint3{},
                            supportLeverGameUnits);
                    }
                    /*
                     * Support/pinch convergence is a promotion step, not a
                     * rejection gate. If rich support probing only produced weak
                     * same-surface or single-point evidence, promote the already
                     * selected object-side point into SupportGroup authority
                     * instead of aborting a valid grab.
                     */
                    if (!usingPinchPocket && !gripSupportRuntime.model.canAuthorPivot) {
                        const bool hadWeakSupportModel = gripSupportRuntime.model.valid;
                        forceRuntimeGripSupportAuthority(gripSupportRuntime,
                            grabGripPoint,
                            gripNormalWorld,
                            pocket.valid ? pocket.palmNormalWorld : gripNormalWorld,
                            pocket.valid ? pocket.crossPalmWorld : RE::NiPoint3{},
                            hadWeakSupportModel ? 0.55f : 0.35f,
                            hadWeakSupportModel ? "forcedSupportGroupFromWeakEvidence" : "forcedSupportGroupFromGrabPoint");
                    }
                    if (gripSupportRuntime.model.canAuthorPivot) {
                        grabGripPoint = gripSupportRuntime.model.pivotPoint;
                        gripArea.contactSeedWorld = grabGripPoint;
                        grabPointMode = gripSupportActivePointMode(gripSupportRuntime.model.kind);
                        relationMode = grabPointMode;
                        grabFallbackReason = gripSupportRuntime.model.reason;
                        _grabObjectGripAtGrab.contactSeedWorld = grabGripPoint;
                        _grabObjectGripAtGrab.source = grabPointMode;
                        _grabObjectGripAtGrab.fallbackReason = grabFallbackReason;
                        _grabObjectGripAtGrab.confidence = (std::max)(_grabObjectGripAtGrab.confidence, gripSupportRuntime.model.confidence);
                        pivotAuthoritySource = grabPivotAuthoritySourceName(GrabPivotAuthoritySource::GripSupportModel);
                        pivotAuthorityPositionOnly = false;
                        pivotAuthorityNormalTrusted = lengthSquared(gripSupportRuntime.model.supportNormal) > 0.000001f;
                        pivotAuthorityPositionConfidence = gripSupportRuntime.model.confidence;
                        pivotAuthorityPocketDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                        pivotAuthoritySelectionDeltaGameUnits =
                            sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint) : std::numeric_limits<float>::max();
                        pivotAuthorityLongLeverGameUnits = supportLeverGameUnits;
                        if (pivotAuthorityNormalTrusted) {
                            gripNormalWorld = gripSupportRuntime.model.supportNormal;
                        }
                    }

                    {
                        using AuthorityCandidate = grab_authority_frame_math::GrabAuthorityPivotCandidate<RE::NiPoint3>;
                        std::array<AuthorityCandidate, 2> authorityCandidates{};
                        authorityCandidates[0] = AuthorityCandidate{
                            .valid = pinchPocketCandidate.valid,
                            .source = grab_authority_frame_math::GrabAuthorityPivotSource::PinchPocket,
                            .pointWorld = pinchPocketCandidate.valid ? pinchPocketCandidate.surfaceHit.position : RE::NiPoint3{},
                            .normalWorld = pinchPocketCandidate.valid ? pinchPocketCandidate.surfaceHit.normal : RE::NiPoint3{},
                            .normalValid = pinchPocketCandidate.valid && lengthSquared(pinchPocketCandidate.surfaceHit.normal) > 0.000001f,
                            .bodyId = objectBodyId.value,
                            .sourceNode = pinchPocketCandidate.valid ? pinchPocketCandidate.surfaceHit.sourceNode : nullptr,
                            .reason = pinchPocketCandidate.valid ? pinchPocketCandidate.decision.reason : "pinchPocketUnavailable",
                        };
                        authorityCandidates[1] = AuthorityCandidate{
                            .valid = gripSupportRuntime.model.canAuthorPivot,
                            .source = grab_authority_frame_math::GrabAuthorityPivotSource::GripSupportModel,
                            .pointWorld = gripSupportRuntime.model.canAuthorPivot ? gripSupportRuntime.model.pivotPoint : RE::NiPoint3{},
                            .normalWorld = gripSupportRuntime.model.canAuthorPivot ? gripSupportRuntime.model.supportNormal : RE::NiPoint3{},
                            .normalValid = gripSupportRuntime.model.canAuthorPivot && lengthSquared(gripSupportRuntime.model.supportNormal) > 0.000001f,
                            .bodyId = objectBodyId.value,
                            .sourceNode = grabSurfaceHit.sourceNode ? grabSurfaceHit.sourceNode : collidableNode,
                            .reason = gripSupportRuntime.model.canAuthorPivot ? gripSupportRuntime.model.reason : "gripSupportUnavailable",
                        };

                        const auto resolvedAuthorityPivot = grab_authority_frame_math::resolveGrabAuthorityPivot<RE::NiPoint3>(
                            authorityCandidates);
                        resolvedAuthorityPivotSourceForFreeze = resolvedAuthorityPivot.source;
                        resolvedAuthorityPivotReasonForFreeze = resolvedAuthorityPivot.reason ? resolvedAuthorityPivot.reason : "none";
                        if (!resolvedAuthorityPivot.valid) {
                            ROCK_LOG_WARN(Hand,
                                "{} hand GRAB failed: no final pinch/support authority bodyId={} currentMode={} reason={} supportKind={} supportReason={} pinchReason={}",
                                handName(),
                                objectBodyId.value,
                                grabPointMode,
                                resolvedAuthorityPivot.reason ? resolvedAuthorityPivot.reason : "none",
                                grab_support_model_math::gripSupportKindName(gripSupportRuntime.model.kind),
                                gripSupportRuntime.model.reason ? gripSupportRuntime.model.reason : "none",
                                pinchPocketCandidate.decision.reason ? pinchPocketCandidate.decision.reason : "none");
                            return abortGrab();
                        }
                        if (g_rockConfig.rockDebugGrabFrameLogging) {
                            for (const auto& candidate : authorityCandidates) {
                                const float pivotDistance = candidate.valid ? pointDistanceGameUnits(grabPivotAWorld, candidate.pointWorld) : -1.0f;
                                const float pocketDistance =
                                    (candidate.valid && pocket.valid) ? pointDistanceGameUnits(candidate.pointWorld, pocket.palmCenterWorld) : -1.0f;
                                const float selectionDistance =
                                    (candidate.valid && sel.hasHitPoint) ? pointDistanceGameUnits(candidate.pointWorld, sel.hitPointWorld) : -1.0f;
                                const float leverDistance =
                                    candidate.valid ? pointDistanceGameUnits(candidate.pointWorld, grabBodyWorldAtGrab.translate) : -1.0f;
                                const bool selectedCandidate = candidate.valid && candidate.source == resolvedAuthorityPivot.source;
                                ROCK_LOG_INFO(Hand,
                                    "{} GRAB FREEZE CANDIDATE: formID={:08X} body={} source={} valid={} selected={} reason={} node='{}' "
                                    "point=({:.2f},{:.2f},{:.2f}) normal=({:.3f},{:.3f},{:.3f}) pivotA={:.2f}gu pocket={:.2f}gu selection={:.2f}gu lever={:.2f}gu mode={}",
                                    handName(),
                                    sel.refr ? sel.refr->GetFormID() : 0,
                                    objectBodyId.value,
                                    grab_authority_frame_math::grabAuthorityPivotSourceName(candidate.source),
                                    candidate.valid ? "yes" : "no",
                                    selectedCandidate ? "yes" : "no",
                                    candidate.reason ? candidate.reason : "none",
                                    nodeDebugName(static_cast<const RE::NiAVObject*>(candidate.sourceNode)),
                                    candidate.valid ? candidate.pointWorld.x : 0.0f,
                                    candidate.valid ? candidate.pointWorld.y : 0.0f,
                                    candidate.valid ? candidate.pointWorld.z : 0.0f,
                                    candidate.normalValid ? candidate.normalWorld.x : 0.0f,
                                    candidate.normalValid ? candidate.normalWorld.y : 0.0f,
                                    candidate.normalValid ? candidate.normalWorld.z : 0.0f,
                                    pivotDistance,
                                    pocketDistance,
                                    selectionDistance,
                                    leverDistance,
                                    grabPointMode);
                            }
                        }

                        switch (resolvedAuthorityPivot.source) {
                        case grab_authority_frame_math::GrabAuthorityPivotSource::PinchPocket:
                            grabGripPoint = pinchPocketCandidate.surfaceHit.position;
                            grabSurfaceHit = pinchPocketCandidate.surfaceHit;
                            grabPointMode = "pinchPocket";
                            grabFallbackReason = pinchPocketCandidate.decision.reason;
                            pivotAuthoritySource = grabPivotAuthoritySourceName(GrabPivotAuthoritySource::PinchPocketMeshPoint);
                            pivotAuthorityNormalTrusted = resolvedAuthorityPivot.normalValid;
                            pivotAuthorityPositionConfidence = 0.95f;
                            break;
                        case grab_authority_frame_math::GrabAuthorityPivotSource::GripSupportModel:
                            grabGripPoint = gripSupportRuntime.model.pivotPoint;
                            grabPointMode = gripSupportActivePointMode(gripSupportRuntime.model.kind);
                            grabFallbackReason = gripSupportRuntime.model.reason;
                            pivotAuthoritySource = grabPivotAuthoritySourceName(GrabPivotAuthoritySource::GripSupportModel);
                            pivotAuthorityNormalTrusted = resolvedAuthorityPivot.normalValid;
                            pivotAuthorityPositionConfidence = gripSupportRuntime.model.confidence;
                            break;
                        case grab_authority_frame_math::GrabAuthorityPivotSource::None:
                        case grab_authority_frame_math::GrabAuthorityPivotSource::PalmPocketMesh:
                        case grab_authority_frame_math::GrabAuthorityPivotSource::SelectionMeshSnap:
                        case grab_authority_frame_math::GrabAuthorityPivotSource::CollisionFallback:
                        default:
                            break;
                        }
                        pivotAuthorityPositionOnly = false;
                        pivotAuthorityPocketDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                        pivotAuthoritySelectionDeltaGameUnits =
                            sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint) : std::numeric_limits<float>::max();
                        if (resolvedAuthorityPivot.normalValid) {
                            gripNormalWorld = resolvedAuthorityPivot.normalWorld;
                        }
                        gripEvidencePointWorld = grabGripPoint;
                        gripEvidenceNormalWorld = gripNormalWorld;
                        _grabObjectGripAtGrab.contactSeedWorld = grabGripPoint;
                        _grabObjectGripAtGrab.gripCenterWorld = grabGripPoint;
                        _grabObjectGripAtGrab.source = grabPointMode;
                        _grabObjectGripAtGrab.fallbackReason = grabFallbackReason;
                    }

                    const RE::NiPoint3 finalGripToPocketVector =
                        pocket.valid ? (grabGripPoint - pocket.palmCenterWorld) : RE::NiPoint3{};
                    const float finalGripToPocketDistance =
                        pocket.valid ?
                            std::sqrt(finalGripToPocketVector.x * finalGripToPocketVector.x +
                                      finalGripToPocketVector.y * finalGripToPocketVector.y +
                                      finalGripToPocketVector.z * finalGripToPocketVector.z) :
                            std::numeric_limits<float>::max();
                    const float finalSignedPalmDistance =
                        pocket.valid ? grab_three_phase::dot(finalGripToPocketVector, pocket.palmNormalWorld) : 0.0f;
                    const auto pullCatchSeatSafety =
                        grab_three_phase::evaluatePullCatchSeatSafety(grab_three_phase::PullCatchSeatSafetyInput{
                            .grabbedFromPullCatch = grabbedFromPullCatch,
                            .usingPinchPocket = usingPinchPocket,
                            .capturePhase = _grabAcquisitionPhase,
                            .pocketValid = pocket.valid,
                            .stablePocketTouchContact = hasStablePocketTouchContact,
                            .pivotAuthorityNormalTrusted = pivotAuthorityNormalTrusted,
                            .pivotAuthorityPositionOnly = pivotAuthorityPositionOnly,
                            .gripToPocketDistanceGameUnits = finalGripToPocketDistance,
                            .signedPalmDistanceGameUnits = finalSignedPalmDistance,
                            .behindPalmToleranceGameUnits = g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits,
                            .touchAcquireDistanceGameUnits = g_rockConfig.rockGrabTouchAcquireDistanceGameUnits,
                            .pocketRadiusGameUnits = pocket.valid ? pocket.pocketRadiusGameUnits : g_rockConfig.rockGrabPocketRadiusGameUnits,
                            .palmNormalWorld = pocket.valid ? pocket.palmNormalWorld : RE::NiPoint3{},
                            .gripNormalWorld = gripNormalWorld,
                        });
                    if (grabbedFromPullCatch &&
                        !usingPinchPocket &&
                        _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld &&
                        !pullCatchSeatSafety.allowImmediateTouchHeld) {
                        _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::NearConverging;
                        captureReason = pullCatchSeatSafety.reason;
                        grabFallbackReason = captureReason;
                        _grabObjectGripAtGrab.fallbackReason = captureReason;
                    }

                    desiredBodyWorld = grab_frame_math::shiftObjectToAlignGripWithPocket(
                        grabBodyWorldAtGrab,
                        grabPivotAWorld,
                        grabGripPoint);
                    desiredObjectWorld = deriveNodeWorldFromBodyWorld(desiredBodyWorld, objectToBodyAtGrab);
                    const bool programmaticArrival = grabbedFromPullCatch || sel.forcedArrival;
                    /*
                     * Guns and melee never seat from saved offsets: the FRIK
                     * weapon-offset attach below is the only weapon authority
                     * (saved_grab_offset::participatesInSavedGrabOffsets).
                     * Leaving the source empty here also keeps a saved finger
                     * pose from overriding the FRIK weapon hand pose.
                     */
                    if (programmaticArrival &&
                        saved_grab_offset::participatesInSavedGrabOffsets(
                            looseWeaponGrab,
                            isThrowableLooseWeapon(selectedLooseWeaponForm(sel)))) {
                        resolvedGrabOffsetSource = resolveGrabOffsetSource(_isLeft, sel.refr);
                    }
                    RE::NiTransform grabProxyWorldForOffset{};
                    const bool grabProxyWorldValidForOffset =
                        programmaticArrival && tryComputeGrabProxyLocalPalmPocketFrameWorld(world, grabProxyWorldForOffset);
                    const auto grabOffsetAttachSource = resolveGrabOffsetAttachSource(
                        grabProxyWorldForOffset, grabProxyWorldValidForOffset, resolvedGrabOffsetSource);
                    const auto looseWeaponPrimaryAttachFrame = resolveLooseWeaponPrimaryAttachFrame(
                        looseWeaponGrab,
                        grabbedFromPullCatch,
                        _isLeft,
                        sel,
                        rootNode,
                        rootBodyLocalAtGrab,
                        objectToBodyAtGrab,
                        grabBodyWorldAtGrab,
                        grabPivotAWorld,
                        handWorldTransform,
                        grabOffsetAttachSource);
                    looseWeaponPrimaryAttachReason = looseWeaponPrimaryAttachFrame.reason;
                    if (looseWeaponPrimaryAttachFrame.valid) {
                        desiredObjectWorld = looseWeaponPrimaryAttachFrame.desiredObjectWorld;
                        desiredBodyWorld = looseWeaponPrimaryAttachFrame.desiredBodyWorld;
                        grabGripPoint = looseWeaponPrimaryAttachFrame.gripPointWorld;
                        gripArea.contactSeedWorld = grabGripPoint;
                        gripEvidencePointWorld = grabGripPoint;
                        gripNormalWorld = firstValidNormal({
                            pocket.valid ? pocket.palmNormalWorld : RE::NiPoint3{},
                            gripNormalWorld,
                        });
                        gripEvidenceNormalWorld = gripNormalWorld;
                        _grabObjectGripAtGrab.contactSeedWorld = grabGripPoint;
                        _grabObjectGripAtGrab.gripCenterWorld = grabGripPoint;
                        grabPointMode = "looseWeaponPrimaryAttach";
                        relationMode = grabPointMode;
                        grabFallbackReason = looseWeaponPrimaryAttachFrame.reason;
                        _grabObjectGripAtGrab.source = grabPointMode;
                        _grabObjectGripAtGrab.fallbackReason = grabFallbackReason;
                        if (_grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld) {
                            _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::NearConverging;
                            captureReason = "looseWeaponPrimaryAttachSettle";
                        }
                        grabSurfaceHit = GrabSurfaceHit{};
                        contactPatchRuntime = RuntimeGrabContactPatch{};
                        contactPatchEvidenceAvailable = false;
                        pivotAuthoritySource = grabPivotAuthoritySourceName(GrabPivotAuthoritySource::LooseWeaponPrimaryAttach);
                        pivotAuthorityPositionOnly = false;
                        pivotAuthorityNormalTrusted = true;
                        pivotAuthorityPositionConfidence = 1.0f;
                        pivotAuthorityPocketDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                        pivotAuthoritySelectionDeltaGameUnits = sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint) : std::numeric_limits<float>::max();
                        resolvedAuthorityPivotSourceForFreeze = grab_authority_frame_math::GrabAuthorityPivotSource::LooseWeaponPrimaryAttach;
                        resolvedAuthorityPivotReasonForFreeze = looseWeaponPrimaryAttachFrame.reason;
                        looseWeaponPrimaryAttachApplied = true;
                        looseWeaponPrimaryAttachSourceVisible = looseWeaponPrimaryAttachFrame.sourceVisible;
                    }
                    /*
                     * Seat-time self-alignment. Forced arrivals (grenade menu,
                     * provider API) commit without a pull flight, so the
                     * long-axis presentation servo never gets to run: rotate
                     * the SEAT pose instead - the minimal rotation taking the
                     * mesh principal axis onto the pocket's cross-palm
                     * (thumb->pinky) line, about the grip point so the pivot
                     * pair is untouched. Pull-catch arrivals already aligned
                     * their long axis in flight and keep it.
                     *
                     * Face-to-palm alignment is a separate correction: a
                     * minimal-arc swing (flight servo or seat rotation) adds
                     * zero twist, so an object that arrived tilted keeps its
                     * arrival roll and its flat face can cut diagonally into
                     * the palm - the depth stop below only translates and
                     * cannot fix a tilted face. Which rotation achieves it
                     * depends on the shape class, read from the same PCA
                     * spectrum:
                     *
                     *   ROD/PLANK (elongationRatio gate passes): the long axis
                     *   is already committed to the cross-palm line, so the
                     *   only free DOF is a TWIST about it - roll until the
                     *   thinnest-extent direction faces the palm. Round cross
                     *   sections skip; any roll is equivalent for them.
                     *
                     *   PLATE (elongation gate fails but the minor axis is
                     *   well separated - books, trays, boards): there is no
                     *   meaningful long axis to twist about, but the face
                     *   normal is exact. Swing that normal onto the palm
                     *   normal directly. Nearest-hemisphere keeps the swing
                     *   under 90 degrees and never flips the object over. The
                     *   free in-plane spin is deliberately left alone: it
                     *   cannot affect clipping, and for near-square plates the
                     *   in-plane axes are numerically interchangeable.
                     *
                     *   Otherwise (compact/isotropic): no alignment.
                     *
                     * Rotations survive the freeze (pivot alignment only
                     * rewrites translation).
                     */
                    RE::NiTransform seatBodyWorld = grabBodyWorldAtGrab;
                    RE::NiTransform seatObjectWorld = objectWorldTransform;
                    float seatAlignmentAngleDegrees = 0.0f;
                    const char* seatAlignmentReason = "inactive";
                    float seatRollAngleDegrees = 0.0f;
                    const char* seatRollReason = "inactive";
                    bool seatPoseChanged = false;
                    const bool seatSwingAlignmentWanted =
                        sel.forcedArrival && g_rockConfig.rockForceGrabSeatAlignmentEnabled;
                    const bool seatRollAlignmentWanted =
                        programmaticArrival && g_rockConfig.rockGrabSeatRollAlignmentEnabled;
                    /*
                     * The shape class also selects the penetration backstop
                     * footprint below, and that backstop runs on EVERY arrival
                     * (close grabs never rotate the object), so the PCA is
                     * resolved outside the alignment gates. One pass at
                     * capture, never per frame.
                     */
                    const bool seatShapeEvaluable =
                        !looseWeaponPrimaryAttachApplied &&
                        !usingPinchPocket &&
                        pocket.valid &&
                        !grabMeshTriangles.empty();
                    const auto seatLongAxis =
                        seatShapeEvaluable ? computeGrabMeshLongAxis(grabMeshTriangles) : GrabMeshLongAxisResult{};
                    const bool seatRodShape =
                        seatLongAxis.valid &&
                        seatLongAxis.elongationRatio >= g_rockConfig.rockPullPresentationMinElongationRatio;
                    const bool seatFlatFace =
                        seatLongAxis.valid &&
                        seatLongAxis.secondElongationRatio >= g_rockConfig.rockGrabSeatRollMinSecondElongationRatio;
                    const bool seatPlateShape = seatLongAxis.valid && !seatRodShape && seatFlatFace;
                    if ((seatSwingAlignmentWanted || seatRollAlignmentWanted) && seatShapeEvaluable) {
                        if (seatSwingAlignmentWanted) {
                            seatAlignmentReason = seatLongAxis.reason;
                        }
                        if (seatRollAlignmentWanted) {
                            seatRollReason = seatLongAxis.reason;
                        }
                        RE::NiPoint3 currentAxisWorld = normalizeOrZero(seatLongAxis.axisWorld);
                        RE::NiPoint3 currentSecondAxisWorld = normalizeOrZero(seatLongAxis.secondAxisWorld);
                        if (seatRodShape) {
                            if (seatSwingAlignmentWanted) {
                                // Same hand-signed thumb-clearance tilt as the flight servo.
                                const float gripAxisTiltRadians = gripAxisTiltRadiansForHand(_isLeft);
                                RE::NiPoint3 targetAxisWorld = normalizeOrZero(
                                    pocket.crossPalmWorld * std::cos(gripAxisTiltRadians) +
                                    pocket.fingerForwardWorld * std::sin(gripAxisTiltRadians));
                                if (lengthSquared(currentAxisWorld) > 0.000001f && lengthSquared(targetAxisWorld) > 0.000001f) {
                                    if (dotProduct(currentAxisWorld, targetAxisWorld) < 0.0f) {
                                        targetAxisWorld = RE::NiPoint3{ -targetAxisWorld.x, -targetAxisWorld.y, -targetAxisWorld.z };
                                    }
                                    const RE::NiPoint3 rotationAxisRaw = crossProduct(currentAxisWorld, targetAxisWorld);
                                    const float sinAngle = std::sqrt((std::max)(0.0f, lengthSquared(rotationAxisRaw)));
                                    const float cosAngle = std::clamp(dotProduct(currentAxisWorld, targetAxisWorld), -1.0f, 1.0f);
                                    const float angleRadians = std::atan2(sinAngle, cosAngle);
                                    if (sinAngle > 0.000001f && angleRadians > 0.01f) {
                                        const float invSin = 1.0f / sinAngle;
                                        const RE::NiPoint3 rotationAxis{
                                            rotationAxisRaw.x * invSin,
                                            rotationAxisRaw.y * invSin,
                                            rotationAxisRaw.z * invSin,
                                        };
                                        seatBodyWorld = rotateTransformWorldAboutPoint(
                                            grabBodyWorldAtGrab, rotationAxis, angleRadians, grabGripPoint);
                                        seatObjectWorld = rotateTransformWorldAboutPoint(
                                            objectWorldTransform, rotationAxis, angleRadians, grabGripPoint);
                                        // Carry the mesh axes into the swung pose so the roll below runs in seat space.
                                        currentAxisWorld = grab_finger_pose_math::rotateAroundUnitAxis(
                                            currentAxisWorld, rotationAxis, angleRadians);
                                        currentSecondAxisWorld = grab_finger_pose_math::rotateAroundUnitAxis(
                                            currentSecondAxisWorld, rotationAxis, angleRadians);
                                        seatAlignmentAngleDegrees = angleRadians * 57.29577951308232f;
                                        seatAlignmentReason = "longAxisSeatAligned";
                                        seatPoseChanged = true;
                                    } else {
                                        seatAlignmentReason = "alreadyAligned";
                                    }
                                } else {
                                    seatAlignmentReason = "degenerateAxes";
                                }
                            }
                            if (seatRollAlignmentWanted) {
                                if (seatFlatFace &&
                                    lengthSquared(currentAxisWorld) > 0.000001f &&
                                    lengthSquared(currentSecondAxisWorld) > 0.000001f) {
                                    // Thinnest-extent direction = the flat face's normal.
                                    const RE::NiPoint3 faceNormalWorld = crossProduct(currentAxisWorld, currentSecondAxisWorld);
                                    const RE::NiPoint3 rollTargetRaw =
                                        pocket.palmNormalWorld - currentAxisWorld * dotProduct(pocket.palmNormalWorld, currentAxisWorld);
                                    const RE::NiPoint3 rollCurrentRaw =
                                        faceNormalWorld - currentAxisWorld * dotProduct(faceNormalWorld, currentAxisWorld);
                                    RE::NiPoint3 rollTargetWorld = normalizeOrZero(rollTargetRaw);
                                    const RE::NiPoint3 rollCurrentWorld = normalizeOrZero(rollCurrentRaw);
                                    if (lengthSquared(rollTargetWorld) > 0.000001f && lengthSquared(rollCurrentWorld) > 0.000001f) {
                                        if (dotProduct(rollCurrentWorld, rollTargetWorld) < 0.0f) {
                                            // A face has two sides; twist toward the nearer one.
                                            rollTargetWorld = RE::NiPoint3{ -rollTargetWorld.x, -rollTargetWorld.y, -rollTargetWorld.z };
                                        }
                                        const float rollSin = dotProduct(crossProduct(rollCurrentWorld, rollTargetWorld), currentAxisWorld);
                                        const float rollCos = std::clamp(dotProduct(rollCurrentWorld, rollTargetWorld), -1.0f, 1.0f);
                                        const float rollAngleRadians = std::atan2(rollSin, rollCos);
                                        if (std::fabs(rollAngleRadians) > 0.01f) {
                                            seatBodyWorld = rotateTransformWorldAboutPoint(
                                                seatBodyWorld, currentAxisWorld, rollAngleRadians, grabGripPoint);
                                            seatObjectWorld = rotateTransformWorldAboutPoint(
                                                seatObjectWorld, currentAxisWorld, rollAngleRadians, grabGripPoint);
                                            seatRollAngleDegrees = rollAngleRadians * 57.29577951308232f;
                                            seatRollReason = "faceRollSeatAligned";
                                            seatPoseChanged = true;
                                        } else {
                                            seatRollReason = "alreadyRollAligned";
                                        }
                                    } else {
                                        seatRollReason = "degenerateRollAxes";
                                    }
                                } else {
                                    seatRollReason = "belowSecondElongationGate";
                                }
                            }
                        } else if (seatLongAxis.valid) {
                            if (seatSwingAlignmentWanted) {
                                // Cross-palm swing is a rod correction only.
                                seatAlignmentReason = "belowElongationGate";
                            }
                            if (seatRollAlignmentWanted) {
                                /*
                                 * Plates deliberately get NO face alignment.
                                 * The intuition that a flat object should lie
                                 * face-down on the palm is simply wrong for
                                 * this hand: across 31 user-verified plate
                                 * holds (2026-07-25 ground-truth captures) the
                                 * mesh face normal sits 63 degrees off the palm
                                 * normal, IQR 57..72 - plates are held by an
                                 * EDGE or CORNER, and that is the intended
                                 * grip, not an artifact. A face swing fought
                                 * that preference on every plate, so the branch
                                 * was removed rather than retuned. The plate
                                 * CLASS is still resolved above because the
                                 * penetration backstop uses it.
                                 */
                                seatRollReason = seatPlateShape ? "plateEdgeHoldNoFaceAlign" : "belowSecondElongationGate";
                            }
                        }
                    }

                    /*
                     * Seat depth stop: the freeze below re-aligns the selected grip
                     * point exactly onto pivot A, which pulls any mesh behind the
                     * grip point through the palm. Push pivot A out along the palm
                     * normal by the mesh support depth so the object's surface rests
                     * ON the palm instead of its interior. Measured against the SEAT
                     * orientation (post-alignment), not the capture pose. This
                     * supersedes the old fixed-distance pulled-grab adjust, whose
                     * desired-world translation was erased by that same pivot
                     * re-alignment (alignLocalPointInTransformToLocalTarget) on
                     * every successful grab. Weapon attach frames and pinch pockets
                     * keep their own seat authority and are excluded.
                     */
                    GrabSeatDepthStopResult seatDepthStop{};
                    float seatDepthOffsetGameUnits = 0.0f;
                    float seatPenetrationBackstopGameUnits = 0.0f;
                    const char* seatPenetrationBackstopReason = "inactive";
                    if (!looseWeaponPrimaryAttachApplied && !usingPinchPocket && pocket.valid) {
                        seatDepthStop = computeGrabSeatDepthStop(
                            grabLocalMeshTriangles,
                            seatObjectWorld,
                            grabGripPoint,
                            pocket.palmNormalWorld,
                            g_rockConfig.rockGrabSeatDepthFootprintRadiusGameUnits,
                            g_rockConfig.rockGrabSeatDepthMaxGameUnits);
                        if (seatDepthStop.valid && seatDepthStop.depthGameUnits > 0.01f) {
                            seatDepthOffsetGameUnits =
                                seatDepthStop.depthGameUnits + (std::max)(0.0f, g_rockConfig.rockGrabSeatDepthSkinGameUnits);
                            grabPivotAWorld = grabPivotAWorld + pocket.palmNormalWorld * seatDepthOffsetGameUnits;
                            seatPoseChanged = true;
                        }
                        if (seatPoseChanged) {
                            desiredBodyWorld = grab_frame_math::shiftObjectToAlignGripWithPocket(
                                seatBodyWorld,
                                grabPivotAWorld,
                                grabGripPoint);
                            desiredObjectWorld = deriveNodeWorldFromBodyWorld(desiredBodyWorld, objectToBodyAtGrab);
                        }
                        /*
                         * Fail-closed palm-plane backstop: the depth stop above
                         * measures from the GRIP POINT and rejects mesh at
                         * negative grip-relative depth, so a grip seed at or
                         * behind the surface (inset grips, concave shells) or
                         * an empty footprint leaves the correction at zero and
                         * commits the hand inside the object. Re-measure on the
                         * FINAL seat pose from the palm plane itself: any mesh
                         * still reaching past the palm plane within the same
                         * tuned footprint is penetration by definition. One
                         * rigid push restores the surface-on-palm invariant
                         * exactly (a translation reduces every support depth by
                         * the push, so no iteration is needed). Engagement is
                         * always WARN-logged: it means the primary depth stop
                         * failed, and the root cause still needs the capture
                         * line's seatDepthReason evidence.
                         *
                         * Footprint width is shape-selected. The primary stop's
                         * radius is tuned small for seating precision, which
                         * bounds how much penetration a TILTED face can even
                         * express: inside radius r a face tilted by theta shows
                         * at most r*sin(theta), so a large plate cutting deep
                         * through the fingers registers a fraction of a unit at
                         * the palm axis and commits. Plates - where the near
                         * surface is flat and one rigid push-out is therefore
                         * exactly right everywhere - get a hand-sized footprint
                         * instead. Every other shape keeps the tuned radius:
                         * widening it for irregular geometry would let lobes
                         * that merely pass BESIDE the palm start pushing seats
                         * out (mug bodies, shell rims) and reintroduce the
                         * floaty seats that radius was tuned to remove.
                         */
                        const float backstopFootprintRadiusGameUnits =
                            seatPlateShape ? g_rockConfig.rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits
                                           : g_rockConfig.rockGrabSeatDepthFootprintRadiusGameUnits;
                        const auto palmPlaneOvershoot = computeGrabSeatDepthStop(
                            grabLocalMeshTriangles,
                            desiredObjectWorld,
                            pocket.palmCenterWorld,
                            pocket.palmNormalWorld,
                            backstopFootprintRadiusGameUnits,
                            g_rockConfig.rockGrabSeatDepthMaxGameUnits);
                        seatPenetrationBackstopReason = palmPlaneOvershoot.reason;
                        constexpr float kSeatPenetrationBackstopThresholdGameUnits = 1.0f;
                        if (palmPlaneOvershoot.valid &&
                            palmPlaneOvershoot.depthGameUnits > kSeatPenetrationBackstopThresholdGameUnits) {
                            seatPenetrationBackstopGameUnits =
                                palmPlaneOvershoot.depthGameUnits + (std::max)(0.0f, g_rockConfig.rockGrabSeatDepthSkinGameUnits);
                            grabPivotAWorld = grabPivotAWorld + pocket.palmNormalWorld * seatPenetrationBackstopGameUnits;
                            desiredBodyWorld = grab_frame_math::shiftObjectToAlignGripWithPocket(
                                seatBodyWorld,
                                grabPivotAWorld,
                                grabGripPoint);
                            desiredObjectWorld = deriveNodeWorldFromBodyWorld(desiredBodyWorld, objectToBodyAtGrab);
                            seatPoseChanged = true;
                            seatPenetrationBackstopReason = "palmPlanePenetrationPushedOut";
                            ROCK_LOG_WARN(Hand,
                                "{} SEAT DEPTH BACKSTOP: final seat still reached {:.2f}gu past the palm plane -> pushed out {:.2f}gu "
                                "(shape={} footprint={:.1f} primary seatDepthReason={} samples={} depth={:.2f} offset={:.2f} overshootSamples={})",
                                handName(),
                                palmPlaneOvershoot.depthGameUnits,
                                seatPenetrationBackstopGameUnits,
                                seatPlateShape ? "plate" : (seatRodShape ? "rod" : "compact"),
                                backstopFootprintRadiusGameUnits,
                                seatDepthStop.reason,
                                seatDepthStop.footprintSampleCount,
                                seatDepthStop.depthGameUnits,
                                seatDepthOffsetGameUnits,
                                palmPlaneOvershoot.footprintSampleCount);
                        }
                    }

                    /*
                     * Pinch seat centering: the freeze puts the pinch SURFACE hit
                     * on the pocket point, which parks the object's near face at
                     * the pocket and shifts its body toward one finger pad by its
                     * full local thickness. Measure the mesh extents both ways
                     * along the pinch axis from the grip point (small footprint -
                     * only the material actually between the pads matters) and
                     * offset pivot A so the object's MID-THICKNESS sits exactly at
                     * the pocket middle. Same pivot-A mechanism as the depth stop;
                     * the correction is zero for a surface hit already centered.
                     */
                    float pinchCenterOffsetGameUnits = 0.0f;
                    if (usingPinchPocket && !looseWeaponPrimaryAttachApplied) {
                        const RE::NiPoint3 pinchAxisWorld = normalizeOrZero(pinchPocketCandidate.pinchAxisWorld);
                        if (lengthSquared(pinchAxisWorld) > 0.000001f) {
                            // Finger-pad scale; pinch objects are small by classification.
                            constexpr float kPinchCenterFootprintRadiusGameUnits = 2.5f;
                            constexpr float kPinchCenterMaxExtentGameUnits = 8.0f;
                            const auto extentTowardIndex = computeGrabSeatDepthStop(
                                grabLocalMeshTriangles,
                                objectWorldTransform,
                                grabGripPoint,
                                RE::NiPoint3{ -pinchAxisWorld.x, -pinchAxisWorld.y, -pinchAxisWorld.z },
                                kPinchCenterFootprintRadiusGameUnits,
                                kPinchCenterMaxExtentGameUnits);
                            const auto extentTowardThumb = computeGrabSeatDepthStop(
                                grabLocalMeshTriangles,
                                objectWorldTransform,
                                grabGripPoint,
                                pinchAxisWorld,
                                kPinchCenterFootprintRadiusGameUnits,
                                kPinchCenterMaxExtentGameUnits);
                            if (extentTowardIndex.valid && extentTowardThumb.valid) {
                                pinchCenterOffsetGameUnits =
                                    (extentTowardIndex.depthGameUnits - extentTowardThumb.depthGameUnits) * 0.5f;
                                if (std::fabs(pinchCenterOffsetGameUnits) > 0.05f) {
                                    grabPivotAWorld = grabPivotAWorld - pinchAxisWorld * pinchCenterOffsetGameUnits;
                                    desiredBodyWorld = grab_frame_math::shiftObjectToAlignGripWithPocket(
                                        grabBodyWorldAtGrab,
                                        grabPivotAWorld,
                                        grabGripPoint);
                                    desiredObjectWorld = deriveNodeWorldFromBodyWorld(desiredBodyWorld, objectToBodyAtGrab);
                                }
                            }
                        }
                    }

                    selectedGripPointLocal = transform_math::worldPointToLocal(objectWorldTransform, grabGripPoint);
                    selectedPivotBBodyLocalGame = transform_math::worldPointToLocal(grabBodyWorldAtGrab, grabGripPoint);
                    const bool effectivePinchPocket = usingPinchPocket && !looseWeaponPrimaryAttachApplied;
                    RE::NiPoint3 supportFrameNormalWorld = normalizeOrZero(gripNormalWorld);
                    if (lengthSquared(supportFrameNormalWorld) <= 0.000001f) {
                        supportFrameNormalWorld = firstValidNormal({
                            gripEvidenceNormalWorld,
                            gripSupportRuntime.model.supportNormal,
                            pocket.palmNormalWorld,
                        });
                    }
                    const RE::NiPoint3 supportFrameAxisWorld = resolveSupportFrameAxisWorld(supportFrameNormalWorld,
                        gripSupportRuntime.model.supportAxis,
                        usingPinchPocket ? pinchPocketCandidate.pinchAxisWorld : RE::NiPoint3{},
                        pocket.valid ? pocket.crossPalmWorld : RE::NiPoint3{},
                        pocket.valid ? pocket.fingerForwardWorld : RE::NiPoint3{});
                    const RE::NiPoint3 supportFrameBinormalWorld = normalizeOrZero(crossProduct(supportFrameNormalWorld, supportFrameAxisWorld));
                    /*
                     * Seat diagnostics: the classification and every
                     * correction outcome, kept past the log line so a saved
                     * ground-truth capture can record the seat ROCK produced
                     * for this grab next to the pose the user corrected it to.
                     */
                    _grabFrame.heldNode = collidableNode;
                    _grabFrame.localMeshTriangles = grabLocalMeshTriangles;
                    _grabFrame.fingerPoseLocalMeshTriangles = grabFingerPoseLocalMeshTriangles;
                    _grabFrame.hasMeshPoseData =
                        !grabLocalMeshTriangles.empty() || !grabFingerPoseLocalMeshTriangles.empty();
                    _grabFrame.bodyResolutionReason = primaryBodyChoiceReasonName(primaryChoice.reason);

                    _grabFrame.seatDiagnostics = GrabSeatDiagnostics{
                        .acquisitionMode = sel.forcedArrival ? "forceGrab" : (grabbedFromPullCatch ? "pullCatch" : "closeGrab"),
                        .shapeClass = seatPlateShape ? "plate" : (seatRodShape ? "rod" : (seatLongAxis.valid ? "compact" : "none")),
                        .elongationRatio = seatLongAxis.elongationRatio,
                        .secondElongationRatio = seatLongAxis.secondElongationRatio,
                        .alignmentAngleDegrees = seatAlignmentAngleDegrees,
                        .alignmentReason = seatAlignmentReason,
                        .rollAngleDegrees = seatRollAngleDegrees,
                        .rollReason = seatRollReason,
                        .depthGameUnits = seatDepthStop.depthGameUnits,
                        .depthOffsetGameUnits = seatDepthOffsetGameUnits,
                        .depthReason = seatDepthStop.reason,
                        .penetrationBackstopGameUnits = seatPenetrationBackstopGameUnits,
                        .penetrationBackstopReason = seatPenetrationBackstopReason,
                    };
                    _grabFrame.gripEvidenceLocal = selectedGripPointLocal;
                    _grabFrame.gripNormalLocal = transform_math::worldVectorToLocal(objectWorldTransform, gripNormalWorld);
                    _grabFrame.gripEvidenceTriangleIndex =
                        !looseWeaponPrimaryAttachApplied && grabSurfaceHit.valid && grabSurfaceHit.hasTriangle ?
                        static_cast<std::uint32_t>(grabSurfaceHit.triangleIndex) :
                        0xFFFF'FFFF;
                    _grabFrame.gripEvidenceShapeKey =
                        !looseWeaponPrimaryAttachApplied && grabSurfaceHit.valid ? grabSurfaceHit.shapeKey : 0xFFFF'FFFF;
                    _grabFrame.gripEvidenceShapeCollisionFilterInfo =
                        !looseWeaponPrimaryAttachApplied && grabSurfaceHit.valid ? grabSurfaceHit.shapeCollisionFilterInfo : 0;
                    _grabFrame.gripEvidenceHitFraction =
                        !looseWeaponPrimaryAttachApplied && grabSurfaceHit.valid ? grabSurfaceHit.hitFraction : 1.0f;
                    _grabFrame.hasGripEvidenceShapeKey =
                        !looseWeaponPrimaryAttachApplied && grabSurfaceHit.valid && grabSurfaceHit.hasShapeKey;
                    if (looseWeaponPrimaryAttachApplied) {
                        _grabFrame.gripSourceNode = nullptr;
                        _grabFrame.gripPointSourceNodeLocal = {};
                        _grabFrame.gripNormalSourceNodeLocal = {};
                        _grabFrame.hasGripSourceNodePoint = false;
                        _grabFrame.hasGripSourceNodeNormal = false;
                    } else {
                        storeGripSourceEvidence(_grabFrame,
                            grabSurfaceHit.sourceNode ? grabSurfaceHit.sourceNode : collidableNode,
                            objectWorldTransform,
                            gripEvidencePointWorld,
                            gripEvidenceNormalWorld,
                            lengthSquared(gripEvidenceNormalWorld) > 0.000001f);
                    }

                    _grabFrame.pocketToGripDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                    _grabFrame.selectionToGripEvidenceDistanceGameUnits =
                        sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, gripArea.contactSeedWorld) : std::numeric_limits<float>::max();
                    if (grabSurfaceHit.valid) {
                        grabSurfaceHit.pivotToSurfaceDistanceGameUnits = _grabFrame.pocketToGripDistanceGameUnits;
                    }
                    _grabFrame.grabPivotWorldAtGrab = grabPivotAWorld;
                    _grabFrame.gripPointWorldAtGrab = grabGripPoint;
                    _grabFrame.activeGrabPointMode = grabPointMode;
                    _grabFrame.seatMode =
                        !looseWeaponPrimaryAttachApplied && usingPinchPocket ? GrabSeatMode::PinchPocket : GrabSeatMode::SupportGroup;
                    _grabFrame.hasPinchPocket = effectivePinchPocket;
                    _grabFrame.pinchPocketWorldAtGrab = effectivePinchPocket ? pinchPocketCandidate.pinchPocketWorld : RE::NiPoint3{};
                    _grabFrame.pinchAxisWorldAtGrab = effectivePinchPocket ? pinchPocketCandidate.pinchAxisWorld : RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
                    _grabFrame.palmSeatPointWorldAtGrab = effectivePinchPocket ? pinchPocketCandidate.pinchPocketWorld : pocket.palmCenterWorld;
                    _grabFrame.fingerEvidencePointWorldAtGrab =
                        looseWeaponPrimaryAttachApplied || effectivePinchPocket ? grabGripPoint : fingerEvidencePointWorld;
                    _grabFrame.hasPalmSeatPoint = true;
                    _grabFrame.hasFingerEvidencePoint =
                        looseWeaponPrimaryAttachApplied || effectivePinchPocket || fingerEvidencePointValid;
                    _grabFrame.palmSeatPointMode = effectivePinchPocket ? "pinchPocket" : "threePhasePocket";
                    _grabFrame.fingerEvidencePointMode = looseWeaponPrimaryAttachApplied ?
                        "looseWeaponPrimaryAttach" :
                        (effectivePinchPocket ? "pinchThumbIndex" : "evidenceOnly");
                    _grabFrame.activeGrabPointUsesMultiFingerEvidence = false;
                    _grabFrame.pivotAuthoritySource = pivotAuthoritySource;
                    _grabFrame.pivotAuthorityPositionOnly = pivotAuthorityPositionOnly;
                    _grabFrame.pivotAuthorityNormalTrusted = pivotAuthorityNormalTrusted;
                    _grabFrame.pivotAuthorityPositionConfidence = pivotAuthorityPositionConfidence;
                    _grabFrame.syntheticLooseWeaponPrimaryAttach = looseWeaponPrimaryAttachApplied;
                    _grabFrame.hasSettledVisualHandRelation = false;
                    _grabFrame.hasGripSupportModel = looseWeaponPrimaryAttachApplied ? false : gripSupportRuntime.valid;
                    _grabFrame.supportFrameNormalBodyLocal = transform_math::worldVectorToLocal(grabBodyWorldAtGrab, supportFrameNormalWorld);
                    _grabFrame.supportFrameAxisBodyLocal = transform_math::worldVectorToLocal(grabBodyWorldAtGrab, supportFrameAxisWorld);
                    _grabFrame.supportFrameBinormalBodyLocal = transform_math::worldVectorToLocal(grabBodyWorldAtGrab, supportFrameBinormalWorld);
                    _grabFrame.hasSupportFrameNormal = lengthSquared(supportFrameNormalWorld) > 0.000001f;
                    _grabFrame.hasSupportFrameAxis = lengthSquared(supportFrameAxisWorld) > 0.000001f;
                    _grabFrame.hasSupportFrameBinormal = lengthSquared(supportFrameBinormalWorld) > 0.000001f;
                    _grabFrame.gripSupportAuthoredPivot = looseWeaponPrimaryAttachApplied ? false : gripSupportRuntime.model.canAuthorPivot;
                    _grabFrame.gripSupportKind = looseWeaponPrimaryAttachApplied ? grab_support_model_math::GripSupportKind::None : gripSupportRuntime.model.kind;
                    _grabFrame.gripSupportReason = looseWeaponPrimaryAttachApplied ? looseWeaponPrimaryAttachReason : gripSupportRuntime.model.reason;
                    _grabFrame.gripSupportConfidence = looseWeaponPrimaryAttachApplied ? 0.0f : gripSupportRuntime.model.confidence;
                    _grabFrame.gripSupportSpanGameUnits = looseWeaponPrimaryAttachApplied ? 0.0f : gripSupportRuntime.model.supportSpanGameUnits;
                    _grabFrame.gripSupportPivotShiftGameUnits = looseWeaponPrimaryAttachApplied ? 0.0f : gripSupportRuntime.model.pivotShiftGameUnits;
                    _grabFrame.requiresSettledVisualHandRelation = looseWeaponPrimaryAttachApplied ?
                        true :
                        (effectivePinchPocket ?
                                false :
                                (pullCatchSeatSafety.requireSettledVisualRelation ||
                                    pivotAuthorityRequiresSettledVisualRelation(
                                        pivotAuthoritySource,
                                        pivotAuthorityPositionOnly,
                                        _grabAcquisitionPhase)));
                    _grabFrame.fingerPoseAimValid = true;
                    _grabFrame.fingerPoseAimReason = effectivePinchPocket ? "pinchPocketThumbIndexTargets" : "rockPointToPalmEvidence";

                    // Final evidence values are known only after the seat solve.
                    const bool keepContactPatch =
                        !looseWeaponPrimaryAttachApplied && !effectivePinchPocket && contactPatchEvidenceAvailable;
                    decltype(_grabFrame.contactPatchSamples) finalContactPatchSamples{};
                    std::uint32_t finalContactPatchSampleCount = 0;
                    if (keepContactPatch) {
                        finalContactPatchSampleCount = (std::min)(
                            contactPatchRuntime.sampleCount,
                            static_cast<std::uint32_t>(finalContactPatchSamples.size()));
                        for (std::uint32_t i = 0; i < finalContactPatchSampleCount; ++i) {
                            auto sample = contactPatchRuntime.samples[i];
                            sample.point = transform_math::worldPointToLocal(grabBodyWorldAtGrab, sample.point);
                            sample.normal = transform_math::worldVectorToLocal(grabBodyWorldAtGrab, sample.normal);
                            finalContactPatchSamples[i] = sample;
                        }
                    }
                    _grabFrame.contactPatchSamples = finalContactPatchSamples;
                    _grabFrame.contactPatchSampleCount = finalContactPatchSampleCount;
                    _grabFrame.hasContactPatch = keepContactPatch;
                    _grabFrame.hasContactPatchEvidence = keepContactPatch;
                    _grabFrame.contactPatchMeshSnapDeltaGameUnits =
                        !looseWeaponPrimaryAttachApplied && !effectivePinchPocket ?
                        contactPatchRuntime.patch.meshSnapDeltaGameUnits :
                        0.0f;
                    const bool keepMultiFingerPatch =
                        !looseWeaponPrimaryAttachApplied && (effectivePinchPocket || multiFingerGripUsed);
                    _grabFrame.hasMultiFingerContactPatch = keepMultiFingerPatch;
                    _grabFrame.multiFingerContactGroupCount = effectivePinchPocket ?
                        2u :
                        (looseWeaponPrimaryAttachApplied ? 0u : multiFingerGripRuntime.gripSet.groupCount);
                    _grabFrame.multiFingerContactReason = looseWeaponPrimaryAttachApplied ?
                        "looseWeaponPrimaryAttach" :
                        (effectivePinchPocket ?
                                "pinchPocketThumbIndex" :
                                (multiFingerGripRuntime.reason ? multiFingerGripRuntime.reason : "none"));
                    _grabFrame.multiFingerContactSpreadGameUnits = effectivePinchPocket ?
                        pinchPocketCandidate.thumbIndexGapGameUnits :
                        (looseWeaponPrimaryAttachApplied ? 0.0f : multiFingerGripRuntime.gripSet.spreadGameUnits);
                    _grabFrame.multiFingerGripCenterWorldAtGrab = effectivePinchPocket ?
                        grabGripPoint :
                        (looseWeaponPrimaryAttachApplied ? RE::NiPoint3{} : multiFingerGripRuntime.gripSet.contactCenterWorld);
                    _grabFrame.multiFingerHandCenterWorldAtGrab = effectivePinchPocket ?
                        pinchPocketCandidate.pinchPocketWorld :
                        (looseWeaponPrimaryAttachApplied ? RE::NiPoint3{} : multiFingerGripRuntime.gripSet.handCenterWorld);
                    _grabFrame.multiFingerAverageNormalWorldAtGrab = effectivePinchPocket ?
                        gripNormalWorld :
                        (looseWeaponPrimaryAttachApplied ? RE::NiPoint3{} : multiFingerGripRuntime.gripSet.averageNormalWorld);
                    /*
                     * desiredObjectWorld is already the final frozen seat even
                     * while the live body is still converging. Surface aim is
                     * therefore valid at commit; waiting for live touch was the
                     * source of the visible post-grab correction.
                     */
                    const bool fullHeldAuthorityAtCapture =
                        _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld;
                    const auto captureFingerTargets = effectivePinchPocket ?
                        buildRuntimePinchFingerPoseTargets(pinchPocketCandidate) :
                        buildRuntimeFingerPoseTargets(gripEvidencePointWorld, gripEvidenceNormalWorld);
                    storeFingerPoseTargetsInGrabFrame(_grabFrame, captureFingerTargets, objectWorldTransform);

                    ROCK_LOG_DEBUG(Hand,
                        "{} THREE-PHASE GRAB CAPTURE: relation={} seat={} rotation={} phase={} reason={} touchContact={} stableTouch={} pocket=({:.1f},{:.1f},{:.1f}) "
                        "palm=({:.1f},{:.1f},{:.1f}) normal=({:.3f},{:.3f},{:.3f}) seed=({:.1f},{:.1f},{:.1f}) "
                        "grip=({:.1f},{:.1f},{:.1f}) gripLocal=({:.2f},{:.2f},{:.2f}) pivotB=({:.2f},{:.2f},{:.2f}) dist={:.1f} signedPalm={:.1f} "
                        "fullHeldAuthority={} pivotAuthoritySource={} positionOnlyPatch={} normalTrusted={} support={} supportPivot={} supportConfidence={:.2f} supportSpan={:.2f} supportShift={:.2f} supportReason={} supportSamples={} supportMeshHits={} supportRejectOwner={} supportRejectDistance={} settledVisualRequired={} pullSeatSafety={} pullSeatDot={:.3f} pullSeatSigned={:.1f} pullSeatDist={:.1f} seatDepth={:.2f} seatDepthOffset={:.2f} seatDepthSamples={} seatDepthReason={} seatAlignDeg={:.1f} seatAlignReason={} seatRollDeg={:.1f} seatRollReason={} seatShape={} seatRatio12={:.2f} seatRatio23={:.2f} seatBackstop={:.2f} seatBackstopReason={} pinchCenter={:.2f} inset={:.2f} insetSource={} looseWeaponPrimaryAttach={} attachReason={} attachVisible={}",
                        handName(),
                        relationMode,
                        grabSeatModeName(_grabFrame.seatMode),
                        looseWeaponPrimaryAttachApplied ? "primaryWeaponAttach" : "preserve",
                        grab_three_phase::phaseName(_grabAcquisitionPhase),
                        captureReason,
                        semanticContacts.count > 0 ? "yes" : "no",
                        hasStablePocketTouchContact ? "yes" : "no",
                        pocket.pocketCenterWorld.x,
                        pocket.pocketCenterWorld.y,
                        pocket.pocketCenterWorld.z,
                        pocket.palmCenterWorld.x,
                        pocket.palmCenterWorld.y,
                        pocket.palmCenterWorld.z,
                        gripNormalWorld.x,
                        gripNormalWorld.y,
                        gripNormalWorld.z,
                        gripArea.contactSeedWorld.x,
                        gripArea.contactSeedWorld.y,
                        gripArea.contactSeedWorld.z,
                        grabGripPoint.x,
                        grabGripPoint.y,
                        grabGripPoint.z,
                        selectedGripPointLocal.x,
                        selectedGripPointLocal.y,
                        selectedGripPointLocal.z,
                        selectedPivotBBodyLocalGame.x,
                        selectedPivotBBodyLocalGame.y,
                        selectedPivotBBodyLocalGame.z,
                        usingPinchPocket ? pinchPocketCandidate.pocketToSurfaceDistanceGameUnits : finalGripToPocketDistance,
                        usingPinchPocket ? 0.0f : finalSignedPalmDistance,
                        fullHeldAuthorityAtCapture ? "yes" : "no",
                        _grabFrame.pivotAuthoritySource,
                        _grabFrame.pivotAuthorityPositionOnly ? "yes" : "no",
                        _grabFrame.pivotAuthorityNormalTrusted ? "yes" : "no",
                        grab_support_model_math::gripSupportKindName(_grabFrame.gripSupportKind),
                        _grabFrame.gripSupportAuthoredPivot ? "yes" : "no",
                        _grabFrame.gripSupportConfidence,
                        _grabFrame.gripSupportSpanGameUnits,
                        _grabFrame.gripSupportPivotShiftGameUnits,
                        _grabFrame.gripSupportReason,
                        gripSupportRuntime.sampleCount,
                        gripSupportRuntime.meshProbeHitCount,
                        gripSupportRuntime.rejectedOwnerCount,
                        gripSupportRuntime.rejectedDistanceCount,
                        _grabFrame.requiresSettledVisualHandRelation ? "yes" : "no",
                        pullCatchSeatSafety.reason,
                        pullCatchSeatSafety.normalDotPalm,
                        finalSignedPalmDistance,
                        finalGripToPocketDistance,
                        seatDepthStop.depthGameUnits,
                        seatDepthOffsetGameUnits,
                        seatDepthStop.footprintSampleCount,
                        seatDepthStop.reason,
                        seatAlignmentAngleDegrees,
                        seatAlignmentReason,
                        seatRollAngleDegrees,
                        seatRollReason,
                        seatPlateShape ? "plate" : (seatRodShape ? "rod" : (seatLongAxis.valid ? "compact" : "none")),
                        seatLongAxis.elongationRatio,
                        seatLongAxis.secondElongationRatio,
                        seatPenetrationBackstopGameUnits,
                        seatPenetrationBackstopReason,
                        pinchCenterOffsetGameUnits,
                        gripArea.seedInsetGameUnits,
                        gripArea.fallbackReason,
                        looseWeaponPrimaryAttachApplied ? "yes" : "no",
                        looseWeaponPrimaryAttachReason,
                        looseWeaponPrimaryAttachSourceVisible ? "yes" : "no");
                } else {
                    ROCK_LOG_WARN(Hand,
                        "{} THREE-PHASE GRAB ABORT: pocketValid={} gripValid={} accepted={} reason={} dist={:.2f}gu stableTouch={}",
                        handName(),
                        pocket.valid ? "yes" : "no",
                        gripArea.valid ? "yes" : "no",
                        phaseDecision.accepted ? "yes" : "no",
                        phaseDecision.reason,
                        gripToPocketDistance,
                        hasStablePocketTouchContact ? "yes" : "no");
                    return abortGrab();
                }
            }

            logGrabNodeInfo(handName(),
                _isLeft,
                collidableNode,
                authoredGrabNode,
                desiredObjectWorld,
                handWorldTransform,
                grabPivotAWorld,
                grabPointMode);

            const RE::NiPoint3 frozenVisualNormalWorld = gripEvidenceNormalWorld(_grabFrame, objectWorldTransform);
            const auto frozenAuthorityFrame = grab_authority_frame_math::freezeGrabAuthorityFrame<RE::NiTransform>(
                grab_authority_frame_math::GrabAuthorityFrameFreezeInput<RE::NiTransform>{
                    .rawHandWorld = handWorldTransform,
                    .proxyWorld = proxyFrameWorldAtGrab,
                    .proxyAuthorityFrameWorld = proxyAuthorityFrameWorldAtGrab,
                    .objectWorld = objectWorldTransform,
                    .bodyWorld = grabBodyWorldAtGrab,
                    .constraintBodyWorld = constraintBodyWorldAtGrab,
                    .rootBodyLocal = rootBodyLocalAtGrab,
                    .ownerBodyLocal = ownerBodyLocalAtGrab,
                    .desiredObjectWorld = desiredObjectWorld,
                    .desiredBodyWorld = desiredBodyWorld,
                    .pivotAWorld = grabPivotAWorld,
                    .gripPointWorld = grabGripPoint,
                    .visualNormalWorld = frozenVisualNormalWorld,
                    .source = resolvedAuthorityPivotSourceForFreeze,
                    .hasDesiredObjectWorld = true,
                    .hasDesiredBodyWorld = true,
                    .visualNormalValid = lengthSquared(frozenVisualNormalWorld) > 0.000001f,
                });
            if (!frozenAuthorityFrame.valid) {
                ROCK_LOG_ERROR(Hand,
                    "{} GRAB FAILED: unable to freeze coherent authority frame point=({:.2f},{:.2f},{:.2f}) pivotA=({:.2f},{:.2f},{:.2f}) mode={} bodyId={}",
                    handName(),
                    grabGripPoint.x,
                    grabGripPoint.y,
                    grabGripPoint.z,
                    grabPivotAWorld.x,
                    grabPivotAWorld.y,
                    grabPivotAWorld.z,
                    grabPointMode,
                    objectBodyId.value);
                return abortGrab();
            }
            applyFrozenGrabAuthorityFrameToGrabFrame(_grabFrame, frozenAuthorityFrame);
            desiredObjectWorld = frozenAuthorityFrame.desiredObjectWorld;
            desiredBodyWorld = frozenAuthorityFrame.desiredBodyWorld;
            const float objectScaleForLever =
                std::isfinite(objectWorldTransform.scale) && objectWorldTransform.scale > 0.0f ? objectWorldTransform.scale : 1.0f;
            _grabFrame.longObjectLeverGameUnits =
                computeLocalMeshMaxDistanceFromPoint(_grabFrame.localMeshTriangles, _grabFrame.gripPointLocal) * objectScaleForLever;
            _grabFrame.liveHandWorldAtGrab = handWorldTransform;
            _grabFrame.handBodyWorldAtGrab = proxyFrameWorldAtGrab;
            _grabFrame.objectNodeWorldAtGrab = objectWorldTransform;
            _grabFrame.hasTelemetryCapture = true;
            _grabFrame.handScaleAtGrab = handWorldTransform.scale;
            _grabFrame.traceId = grabTraceId;
            _grabFrame.traceTargetWriteSequence = 0;
            _grabFrame.freezeCaptureTelemetry(objectBodyId.value);
            if (grabTimelineTraceEnabled()) {
                std::array<float, 12> traceTransformBRotation{};
                std::array<float, 4> traceTransformBTranslation{};
                std::array<float, 12> traceTargetBRca{};
                grab_constraint_math::writeGrabConstraintCreationAtoms(
                    traceTransformBRotation.data(),
                    traceTransformBTranslation.data(),
                    traceTargetBRca.data(),
                    frozenAuthorityFrame.proxyAuthorityBodyHandSpace,
                    frozenAuthorityFrame.pivotAHandBodyLocalGame,
                    gameToHavokScale());
                const RE::NiTransform traceDesiredBodyToHandSpace =
                    invertTransform(frozenAuthorityFrame.proxyAuthorityBodyHandSpace);
                const RE::NiMatrix3 traceTargetRows = matrixFromHkRows(traceTargetBRca.data());
                const RE::NiMatrix3 traceTransformBColumns = matrixFromHkColumns(traceTransformBRotation.data());
                const RE::NiPoint3 traceRelationPivotB =
                    grab_constraint_math::computeDynamicTransformBTranslationGame(
                        frozenAuthorityFrame.proxyAuthorityBodyHandSpace,
                        frozenAuthorityFrame.pivotAHandBodyLocalGame);
                const RE::NiPoint3 traceTransformBTranslationGame{
                    traceTransformBTranslation[0] * havokToGameScale(),
                    traceTransformBTranslation[1] * havokToGameScale(),
                    traceTransformBTranslation[2] * havokToGameScale(),
                };
                const float traceSelectedPivotRelationDeltaGameUnits =
                    pointDistanceGameUnits(frozenAuthorityFrame.pivotBConstraintLocalGame, traceRelationPivotB);
                const float tracePivotBRelationDeltaGameUnits =
                    pointDistanceGameUnits(traceTransformBTranslationGame, traceRelationPivotB);
                const float traceTargetToHiggsRelationDegrees =
                    rotationDeltaDegrees(traceTargetRows, traceDesiredBodyToHandSpace.rotate);
                const float traceTransformBFrozenDeltaDegrees =
                    rotationDeltaDegrees(traceTransformBColumns, traceDesiredBodyToHandSpace.rotate);
                const float traceRawToProxyRotDegrees =
                    rotationDeltaDegrees(handWorldTransform.rotate, proxyFrameWorldAtGrab.rotate);
                const float traceRawToProxyMaxAxisDegrees =
                    maxColumnAxisDeltaDegrees(handWorldTransform.rotate, proxyFrameWorldAtGrab.rotate);
                const float traceObjectToDesiredMaxAxisDegrees =
                    maxColumnAxisDeltaDegrees(objectWorldTransform.rotate, frozenAuthorityFrame.desiredObjectWorld.rotate);
                const float traceDesiredBodyToGrabBodyMaxAxisDegrees =
                    maxColumnAxisDeltaDegrees(frozenAuthorityFrame.desiredBodyWorld.rotate, grabBodyWorldAtGrab.rotate);
                const RE::NiPoint3 tracePivotBeforeFreeze =
                    transform_math::localPointToWorld(grabBodyWorldAtGrab, frozenAuthorityFrame.pivotBConstraintLocalGame);
                const RE::NiPoint3 tracePivotAfterFreeze =
                    transform_math::localPointToWorld(frozenAuthorityFrame.desiredBodyWorld, frozenAuthorityFrame.pivotBConstraintLocalGame);
                const RE::NiPoint3 traceFreezeShift =
                    frozenAuthorityFrame.desiredBodyWorld.translate - grabBodyWorldAtGrab.translate;
                const RE::NiPoint3 traceExpectedShift =
                    frozenAuthorityFrame.pivotAWorld - frozenAuthorityFrame.gripPointWorldAtGrab;
                const float traceFreezeShiftLength = vectorMagnitude(traceFreezeShift);
                const float traceExpectedShiftLength = vectorMagnitude(traceExpectedShift);
                const float traceFreezeShiftDot =
                    traceFreezeShiftLength > 0.0001f && traceExpectedShiftLength > 0.0001f ?
                        std::clamp(dotProduct(traceFreezeShift, traceExpectedShift) / (traceFreezeShiftLength * traceExpectedShiftLength), -1.0f, 1.0f) :
                        0.0f;
                const float tracePivotGapBeforeFreeze =
                    pointDistanceGameUnits(tracePivotBeforeFreeze, frozenAuthorityFrame.pivotAWorld);
                const float tracePivotGapAfterFreeze =
                    pointDistanceGameUnits(tracePivotAfterFreeze, frozenAuthorityFrame.pivotAWorld);
                const float traceBodyShiftDegrees =
                    rotationDeltaDegrees(frozenAuthorityFrame.desiredBodyWorld.rotate, grabBodyWorldAtGrab.rotate);
                const auto traceRawBasis = grab_transform_telemetry::makeOrientationBasis(handWorldTransform);
                const auto traceProxyBasis = grab_transform_telemetry::makeOrientationBasis(proxyFrameWorldAtGrab);
                const auto traceBodyBasis = grab_transform_telemetry::makeOrientationBasis(grabBodyWorldAtGrab);
                const auto traceDesiredBodyBasis = grab_transform_telemetry::makeOrientationBasis(frozenAuthorityFrame.desiredBodyWorld);

                ROCK_LOG_INFO(Hand,
                    "{} GRAB_TRACE stage=capture trace={} hand={} formID={:08X} name='{}' body={} mode={} pivotAuthority={} frozenSource={} resolverSource={} resolverReason={} seat={} phase={} pinch={} gripSupport={} supportPivot={} fullHeld={} settledReq={} shapeKey=0x{:08X} triangle=0x{:08X} pivotA=({:.2f},{:.2f},{:.2f}) grip=({:.2f},{:.2f},{:.2f}) pivotBBody=({:.2f},{:.2f},{:.2f}) pivotBSelected=({:.2f},{:.2f},{:.2f}) relationPivotB=({:.2f},{:.2f},{:.2f}) selectedPivotRelationDelta={:.3f}gu pivotBRelationDelta={:.3f}gu pocket={:.2f}gu selection={:.2f}gu longLever={:.2f}gu positionOnly={} normalTrusted={} confidence={:.2f}",
                    handName(),
                    _grabFrame.traceId,
                    _isLeft ? "left" : "right",
                    sel.refr ? sel.refr->GetFormID() : 0,
                    objName,
                    objectBodyId.value,
                    _grabFrame.activeGrabPointMode ? _grabFrame.activeGrabPointMode : "none",
                    _grabFrame.pivotAuthoritySource ? _grabFrame.pivotAuthoritySource : "none",
                    grab_authority_frame_math::grabAuthorityPivotSourceName(frozenAuthorityFrame.source),
                    grab_authority_frame_math::grabAuthorityPivotSourceName(resolvedAuthorityPivotSourceForFreeze),
                    resolvedAuthorityPivotReasonForFreeze ? resolvedAuthorityPivotReasonForFreeze : "none",
                    grabSeatModeName(_grabFrame.seatMode),
                    grab_three_phase::phaseName(_grabAcquisitionPhase),
                    _grabFrame.hasPinchPocket ? "yes" : "no",
                    grab_support_model_math::gripSupportKindName(_grabFrame.gripSupportKind),
                    _grabFrame.gripSupportAuthoredPivot ? "yes" : "no",
                    _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld ? "yes" : "no",
                    _grabFrame.requiresSettledVisualHandRelation ? "yes" : "no",
                    _grabFrame.gripEvidenceShapeKey,
                    _grabFrame.gripEvidenceTriangleIndex,
                    frozenAuthorityFrame.pivotAWorld.x,
                    frozenAuthorityFrame.pivotAWorld.y,
                    frozenAuthorityFrame.pivotAWorld.z,
                    frozenAuthorityFrame.gripPointWorldAtGrab.x,
                    frozenAuthorityFrame.gripPointWorldAtGrab.y,
                    frozenAuthorityFrame.gripPointWorldAtGrab.z,
                    frozenAuthorityFrame.pivotBBodyLocalGame.x,
                    frozenAuthorityFrame.pivotBBodyLocalGame.y,
                    frozenAuthorityFrame.pivotBBodyLocalGame.z,
                    frozenAuthorityFrame.pivotBConstraintLocalGame.x,
                    frozenAuthorityFrame.pivotBConstraintLocalGame.y,
                    frozenAuthorityFrame.pivotBConstraintLocalGame.z,
                    traceRelationPivotB.x,
                    traceRelationPivotB.y,
                    traceRelationPivotB.z,
                    traceSelectedPivotRelationDeltaGameUnits,
                    tracePivotBRelationDeltaGameUnits,
                    _grabFrame.pocketToGripDistanceGameUnits,
                    _grabFrame.selectionToGripEvidenceDistanceGameUnits,
                    _grabFrame.longObjectLeverGameUnits,
                    _grabFrame.pivotAuthorityPositionOnly ? "yes" : "no",
                    _grabFrame.pivotAuthorityNormalTrusted ? "yes" : "no",
                    _grabFrame.pivotAuthorityPositionConfidence);

                ROCK_LOG_INFO(Hand,
                    "{} GRAB_TRACE stage=capture_frames trace={} rawPos=({:.2f},{:.2f},{:.2f}) proxyPos=({:.2f},{:.2f},{:.2f}) objectPos=({:.2f},{:.2f},{:.2f}) bodyPos=({:.2f},{:.2f},{:.2f}) desiredBodyPos=({:.2f},{:.2f},{:.2f}) rawProxyRot={:.2f}deg rawProxyAxisMax={:.2f}deg objectDesiredAxisMax={:.2f}deg desiredBodyGrabBodyAxisMax={:.2f}deg targetToHiggsRelation={:.2f}deg transformBFrozenDelta={:.2f}deg",
                    handName(),
                    _grabFrame.traceId,
                    handWorldTransform.translate.x,
                    handWorldTransform.translate.y,
                    handWorldTransform.translate.z,
                    proxyFrameWorldAtGrab.translate.x,
                    proxyFrameWorldAtGrab.translate.y,
                    proxyFrameWorldAtGrab.translate.z,
                    objectWorldTransform.translate.x,
                    objectWorldTransform.translate.y,
                    objectWorldTransform.translate.z,
                    grabBodyWorldAtGrab.translate.x,
                    grabBodyWorldAtGrab.translate.y,
                    grabBodyWorldAtGrab.translate.z,
                    frozenAuthorityFrame.desiredBodyWorld.translate.x,
                    frozenAuthorityFrame.desiredBodyWorld.translate.y,
                    frozenAuthorityFrame.desiredBodyWorld.translate.z,
                    traceRawToProxyRotDegrees,
                    traceRawToProxyMaxAxisDegrees,
                    traceObjectToDesiredMaxAxisDegrees,
                    traceDesiredBodyToGrabBodyMaxAxisDegrees,
                    traceTargetToHiggsRelationDegrees,
                    traceTransformBFrozenDeltaDegrees);

                ROCK_LOG_INFO(Hand,
                    "{} GRAB_TRACE stage=capture_freeze_geometry trace={} body={} mode={} pivotAuthority={} "
                    "bodyShift=({:.2f},{:.2f},{:.2f}) shiftLen={:.2f}gu expectedShift=({:.2f},{:.2f},{:.2f}) expectedLen={:.2f}gu shiftDot={:.3f} "
                    "bodyRotShift={:.2f}deg pivotGapBefore={:.3f}gu pivotGapAfter={:.3f}gu pivotBefore=({:.2f},{:.2f},{:.2f}) pivotAfter=({:.2f},{:.2f},{:.2f}) pivotA=({:.2f},{:.2f},{:.2f}) grip=({:.2f},{:.2f},{:.2f})",
                    handName(),
                    _grabFrame.traceId,
                    objectBodyId.value,
                    _grabFrame.activeGrabPointMode ? _grabFrame.activeGrabPointMode : "none",
                    _grabFrame.pivotAuthoritySource ? _grabFrame.pivotAuthoritySource : "none",
                    traceFreezeShift.x,
                    traceFreezeShift.y,
                    traceFreezeShift.z,
                    traceFreezeShiftLength,
                    traceExpectedShift.x,
                    traceExpectedShift.y,
                    traceExpectedShift.z,
                    traceExpectedShiftLength,
                    traceFreezeShiftDot,
                    traceBodyShiftDegrees,
                    tracePivotGapBeforeFreeze,
                    tracePivotGapAfterFreeze,
                    tracePivotBeforeFreeze.x,
                    tracePivotBeforeFreeze.y,
                    tracePivotBeforeFreeze.z,
                    tracePivotAfterFreeze.x,
                    tracePivotAfterFreeze.y,
                    tracePivotAfterFreeze.z,
                    frozenAuthorityFrame.pivotAWorld.x,
                    frozenAuthorityFrame.pivotAWorld.y,
                    frozenAuthorityFrame.pivotAWorld.z,
                    frozenAuthorityFrame.gripPointWorldAtGrab.x,
                    frozenAuthorityFrame.gripPointWorldAtGrab.y,
                    frozenAuthorityFrame.gripPointWorldAtGrab.z);

                ROCK_LOG_INFO(Hand,
                    "{} GRAB_TRACE stage=capture_basis trace={} {} {} {} {}",
                    handName(),
                    _grabFrame.traceId,
                    grab_transform_telemetry::formatBasis("raw", traceRawBasis),
                    grab_transform_telemetry::formatBasis("proxy", traceProxyBasis),
                    grab_transform_telemetry::formatBasis("body", traceBodyBasis),
                    grab_transform_telemetry::formatBasis("desiredBody", traceDesiredBodyBasis));
                ROCK_LOG_INFO(Hand,
                    "{} GRAB_TRACE stage=capture_axismap trace={} {} {} {} {}",
                    handName(),
                    _grabFrame.traceId,
                    grab_transform_telemetry::formatBasisCrossMap("bodyToProxy", traceBodyBasis, traceProxyBasis),
                    grab_transform_telemetry::formatBasisCrossMap("desiredBodyToProxy", traceDesiredBodyBasis, traceProxyBasis),
                    grab_transform_telemetry::formatBasisCrossMap("proxyToBody", traceProxyBasis, traceBodyBasis),
                    grab_transform_telemetry::formatBasisCrossMap("bodyToDesiredBody", traceBodyBasis, traceDesiredBodyBasis));
            }
            if (g_rockConfig.rockDebugGrabFrameLogging) {
                std::array<float, 12> freezeTransformBRotation{};
                std::array<float, 4> freezeTransformBTranslation{};
                std::array<float, 12> freezeTargetBRca{};
                grab_constraint_math::writeGrabConstraintCreationAtoms(
                    freezeTransformBRotation.data(),
                    freezeTransformBTranslation.data(),
                    freezeTargetBRca.data(),
                    frozenAuthorityFrame.proxyAuthorityBodyHandSpace,
                    frozenAuthorityFrame.pivotAHandBodyLocalGame,
                    gameToHavokScale());
                const RE::NiTransform frozenDesiredBodyToHandSpace =
                    invertTransform(frozenAuthorityFrame.proxyAuthorityBodyHandSpace);
                const RE::NiMatrix3 freezeTargetRows = matrixFromHkRows(freezeTargetBRca.data());
                const RE::NiMatrix3 freezeTransformBColumns = matrixFromHkColumns(freezeTransformBRotation.data());
                const RE::NiPoint3 predictedTransformBLocal =
                    grab_constraint_math::computeDynamicTransformBTranslationGame(
                        frozenAuthorityFrame.proxyAuthorityBodyHandSpace,
                        frozenAuthorityFrame.pivotAHandBodyLocalGame);
                const RE::NiPoint3 freezeTransformBTranslationGame{
                    freezeTransformBTranslation[0] * havokToGameScale(),
                    freezeTransformBTranslation[1] * havokToGameScale(),
                    freezeTransformBTranslation[2] * havokToGameScale(),
                };
                const float freezeSelectedPivotRelationDelta =
                    pointDistanceGameUnits(frozenAuthorityFrame.pivotBConstraintLocalGame, predictedTransformBLocal);
                const float freezePivotBRelationDelta =
                    pointDistanceGameUnits(freezeTransformBTranslationGame, predictedTransformBLocal);
                const float freezeTargetToHiggsRelationDegrees =
                    rotationDeltaDegrees(freezeTargetRows, frozenDesiredBodyToHandSpace.rotate);
                const float freezeTransformBFrozenDeltaDegrees =
                    rotationDeltaDegrees(freezeTransformBColumns, frozenDesiredBodyToHandSpace.rotate);
                const float rawToProxyRotDegrees =
                    rotationDeltaDegrees(handWorldTransform.rotate, proxyFrameWorldAtGrab.rotate);
                const float rawToProxyMaxAxisDegrees =
                    maxColumnAxisDeltaDegrees(handWorldTransform.rotate, proxyFrameWorldAtGrab.rotate);
                const float objectToDesiredMaxAxisDegrees =
                    maxColumnAxisDeltaDegrees(objectWorldTransform.rotate, frozenAuthorityFrame.desiredObjectWorld.rotate);
                const float desiredBodyToGrabBodyMaxAxisDegrees =
                    maxColumnAxisDeltaDegrees(frozenAuthorityFrame.desiredBodyWorld.rotate, grabBodyWorldAtGrab.rotate);
                const float pivotBLeverGameUnits =
                    pointDistanceGameUnits(frozenAuthorityFrame.gripPointWorldAtGrab, grabBodyWorldAtGrab.translate);
                const bool fullHeldAuthorityAtFreeze =
                    _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld;

                ROCK_LOG_INFO(Hand,
                    "{} GRAB FREEZE AUTHORITY: formID={:08X} name='{}' body={} mode={} pivotAuthority={} frozenSource={} resolverSource={} "
                    "resolverReason={} seat={} phase={} pinch={} gripSupport={} supportPivot={} supportReason={} palmPocketMesh={} fullHeld={} settledReq={} "
                    "pivotA=({:.2f},{:.2f},{:.2f}) grip=({:.2f},{:.2f},{:.2f}) pivotBBody=({:.2f},{:.2f},{:.2f}) "
                    "pivotBSelected=({:.2f},{:.2f},{:.2f}) relationPivotB=({:.2f},{:.2f},{:.2f}) "
                    "lever={:.2f}gu pocket={:.2f}gu selection={:.2f}gu rawProxy={:.2f}deg rawProxyAxisMax={:.2f}deg "
                    "objectDesiredAxisMax={:.2f}deg desiredBodyGrabBodyAxisMax={:.2f}deg targetToHiggsRelation={:.2f}deg transformBFrozenDelta={:.2f}deg selectedPivotRelationDelta={:.3f}gu pivotBRelationDelta={:.3f}gu",
                    handName(),
                    sel.refr ? sel.refr->GetFormID() : 0,
                    objName,
                    objectBodyId.value,
                    _grabFrame.activeGrabPointMode ? _grabFrame.activeGrabPointMode : "none",
                    _grabFrame.pivotAuthoritySource ? _grabFrame.pivotAuthoritySource : "none",
                    grab_authority_frame_math::grabAuthorityPivotSourceName(frozenAuthorityFrame.source),
                    grab_authority_frame_math::grabAuthorityPivotSourceName(resolvedAuthorityPivotSourceForFreeze),
                    resolvedAuthorityPivotReasonForFreeze ? resolvedAuthorityPivotReasonForFreeze : "none",
                    grabSeatModeName(_grabFrame.seatMode),
                    grab_three_phase::phaseName(_grabAcquisitionPhase),
                    _grabFrame.hasPinchPocket ? "yes" : "no",
                    grab_support_model_math::gripSupportKindName(_grabFrame.gripSupportKind),
                    _grabFrame.gripSupportAuthoredPivot ? "yes" : "no",
                    _grabFrame.gripSupportReason ? _grabFrame.gripSupportReason : "none",
                    palmPocketMeshAvailable ? "yes" : "no",
                    fullHeldAuthorityAtFreeze ? "yes" : "no",
                    _grabFrame.requiresSettledVisualHandRelation ? "yes" : "no",
                    frozenAuthorityFrame.pivotAWorld.x,
                    frozenAuthorityFrame.pivotAWorld.y,
                    frozenAuthorityFrame.pivotAWorld.z,
                    frozenAuthorityFrame.gripPointWorldAtGrab.x,
                    frozenAuthorityFrame.gripPointWorldAtGrab.y,
                    frozenAuthorityFrame.gripPointWorldAtGrab.z,
                    frozenAuthorityFrame.pivotBBodyLocalGame.x,
                    frozenAuthorityFrame.pivotBBodyLocalGame.y,
                    frozenAuthorityFrame.pivotBBodyLocalGame.z,
                    frozenAuthorityFrame.pivotBConstraintLocalGame.x,
                    frozenAuthorityFrame.pivotBConstraintLocalGame.y,
                    frozenAuthorityFrame.pivotBConstraintLocalGame.z,
                    predictedTransformBLocal.x,
                    predictedTransformBLocal.y,
                    predictedTransformBLocal.z,
                    pivotBLeverGameUnits,
                    _grabFrame.pocketToGripDistanceGameUnits,
                    _grabFrame.selectionToGripEvidenceDistanceGameUnits,
                    rawToProxyRotDegrees,
                    rawToProxyMaxAxisDegrees,
                    objectToDesiredMaxAxisDegrees,
                    desiredBodyToGrabBodyMaxAxisDegrees,
                    freezeTargetToHiggsRelationDegrees,
                    freezeTransformBFrozenDeltaDegrees,
                    freezeSelectedPivotRelationDelta,
                    freezePivotBRelationDelta);
            }
            clearGrabExternalHandWorldTransform(_isLeft);
            _grabVisualHandTransform = handWorldTransform;
            _hasGrabVisualHandTransform = false;
            _lastPublishedGrabVisualHandTransform = {};
            _hasLastPublishedGrabVisualHandTransform = false;
            _grabVisualHandLerpStartTransform = handWorldTransform;
            _grabVisualHandLerpElapsedSeconds = 0.0f;
            _grabVisualHandLerpDurationSeconds = 0.0f;
            _grabVisualDeviationExceededSeconds = 0.0f;
            _grabDeviationExceededSeconds = 0.0f;
            const RE::NiPoint3 initialGrabDelta = grabPivotAWorld - grabGripPoint;
            const float initialGrabDistance =
                std::sqrt(initialGrabDelta.x * initialGrabDelta.x + initialGrabDelta.y * initialGrabDelta.y + initialGrabDelta.z * initialGrabDelta.z);
            const bool needsLargeInitialSync = initialGrabDistance >= g_rockConfig.rockGrabHandLerpMinDistance;
            _grabFrame.fadeInGrabConstraint = needsLargeInitialSync;
            _grabFrame.motorFadeReason = needsLargeInitialSync ? "largeInitialSync" : "none";
            _heldLocalLinearVelocityHistory = {};
            _heldLocalLinearVelocityHistoryCount = 0;
            _heldLocalLinearVelocityHistoryNext = 0;
            _heldLocalHandVelocityHistory = {};
            _heldHandAngularVelocityHistory = {};
            _heldHandVelocityHistoryCount = 0;
            _heldHandVelocityHistoryNext = 0;
            _lastHeldObjectLocalLinearVelocityHavok = {};
            _hasLastHeldObjectLocalLinearVelocityHavok = false;
            _previousHeldRawHandWorld = {};
            _previousHeldHandPositionHavok = {};
            _lastHeldHandPositionHavok = {};
            _hasPreviousHeldRawHandWorld = false;
            _hasLastHeldHandPositionHavok = false;
            _grabAuthorityProxyReleasePending.store(false, std::memory_order_release);

            if (g_rockConfig.rockDebugGrabFrameLogging) {
                const RE::NiPoint3 legacyPalmPivotAHandspace = computeGrabLegacyPalmPivotAHandspacePosition(_isLeft);
                auto* vrScaleSetting = f4vr::getIniSetting("fVrScale:VR");
                const float vrScale = vrScaleSetting ? vrScaleSetting->GetFloat() : -1.0f;

                const RE::NiPoint3 rawLateral = getMatrixColumn(handWorldTransform.rotate, 0);
                const RE::NiPoint3 rawFinger = getMatrixColumn(handWorldTransform.rotate, 2);
                const RE::NiPoint3 rawBack = getMatrixColumn(handWorldTransform.rotate, 1);
                const RE::NiPoint3 proxyFinger = getMatrixColumn(proxyFrameWorldAtGrab.rotate, 2);
                const RE::NiPoint3 proxyBack = getMatrixColumn(proxyFrameWorldAtGrab.rotate, 1);
                const RE::NiPoint3 proxyLateral = getMatrixColumn(proxyFrameWorldAtGrab.rotate, 0);
                const RE::NiPoint3 grabSpaceRawFinger = getMatrixColumn(_grabFrame.rawHandSpace.rotate, 0);
                const RE::NiPoint3 grabSpaceProxyFinger = getMatrixColumn(_grabFrame.proxyAuthorityHandSpace.rotate, 0);
                const RE::NiPoint3 grabPosDelta = _grabFrame.proxyAuthorityHandSpace.translate - _grabFrame.rawHandSpace.translate;
                const float rawVsProxyRot = rotationDeltaDegrees(_grabFrame.rawHandSpace.rotate, _grabFrame.proxyAuthorityHandSpace.rotate);
                const float rawVsProxyPos = std::sqrt(grabPosDelta.x * grabPosDelta.x + grabPosDelta.y * grabPosDelta.y + grabPosDelta.z * grabPosDelta.z);
                const float motionVsGrabRot = hasMotionBodyWorldAtGrab ? rotationDeltaDegrees(motionBodyWorldAtGrab.rotate, grabBodyWorldAtGrab.rotate) : -1.0f;
                const float motionVsGrabPos = hasMotionBodyWorldAtGrab ? translationDeltaGameUnits(motionBodyWorldAtGrab, grabBodyWorldAtGrab) : -1.0f;

                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB FRAME SUMMARY: vrScale={:.3f} handScale={:.3f} bodyScale={:.3f} objectScale={:.3f} "
                    "legacyPivotHS=({:.2f},{:.2f},{:.2f}) pocketW=({:.1f},{:.1f},{:.1f}) gripW=({:.1f},{:.1f},{:.1f}) "
                    "pivotBBodyLocal=({:.2f},{:.2f},{:.2f}) pivotBConstraintLocal=({:.2f},{:.2f},{:.2f}) "
                    "rawProxyDelta={:.2f}deg/{:.2f}gu motionDiagVsGrab={:.2f}deg/{:.2f}gu proxyBodyFrame={} rotRef={} proxyFrame={} rootPalm={} pivotALocal=({:.2f},{:.2f},{:.2f}) meshMode={} meshTris={} "
                    "pocketGrip={:.1f}gu selectionGripEvidence={:.1f}gu "
                    "shapeKey=0x{:08X} shapeFilter=0x{:08X} hitFraction={:.4f} contactPatchEvidence={} contactPatchPivot={} patchHits={} snapDelta={:.1f}gu "
                    "multiFinger={} mfGroups={} mfSpread={:.2f}gu mfReason={} "
                    "activePoint={} activeUsesFingerEvidence={} pivotAuthoritySource={} positionOnlyPatch={} normalTrusted={} positionConfidence={:.2f} palmSeatPoint={} fingerEvidencePoint={} "
                    "poseTargets={} motorFade={} motorFadeReason={} bodyReason={} "
                    "fingerPoseAim={} fingerPoseAimReason={}",
                    handName(), vrScale, handWorldTransform.scale, handBodyWorldAtGrab.scale, collidableNode ? collidableNode->world.scale : -1.0f,
                    legacyPalmPivotAHandspace.x, legacyPalmPivotAHandspace.y, legacyPalmPivotAHandspace.z, grabPivotAWorld.x, grabPivotAWorld.y, grabPivotAWorld.z, grabGripPoint.x,
                    grabGripPoint.y, grabGripPoint.z, _grabFrame.pivotBBodyLocalGame.x, _grabFrame.pivotBBodyLocalGame.y, _grabFrame.pivotBBodyLocalGame.z,
                    _grabFrame.pivotBConstraintLocalGame.x, _grabFrame.pivotBConstraintLocalGame.y, _grabFrame.pivotBConstraintLocalGame.z,
                    rawVsProxyRot, rawVsProxyPos, motionVsGrabRot, motionVsGrabPos,
                    constraintUsesMotionBodyAtGrab ? "MOTION" : "BODY",
                    kGrabObjectRotationReferenceName,
                    proxyFrameSourceAtGrab,
                    hasPalmProxyFrameAtGrab ? "yes" : "no", _grabFrame.pivotAHandBodyLocalGame.x,
                    _grabFrame.pivotAHandBodyLocalGame.y, _grabFrame.pivotAHandBodyLocalGame.z, grabPointMode, grabMeshTriangles.size(),
                    _grabFrame.pocketToGripDistanceGameUnits, _grabFrame.selectionToGripEvidenceDistanceGameUnits,
                    _grabFrame.gripEvidenceShapeKey, _grabFrame.gripEvidenceShapeCollisionFilterInfo, _grabFrame.gripEvidenceHitFraction,
                    _grabFrame.hasContactPatchEvidence ? "yes" : "no", "no",
                    _grabFrame.contactPatchSampleCount, _grabFrame.contactPatchMeshSnapDeltaGameUnits,
                    _grabFrame.hasMultiFingerContactPatch ? "yes" : "no", _grabFrame.multiFingerContactGroupCount,
                    _grabFrame.multiFingerContactSpreadGameUnits, _grabFrame.multiFingerContactReason,
                    _grabFrame.activeGrabPointMode, _grabFrame.activeGrabPointUsesMultiFingerEvidence ? "yes" : "no",
                    _grabFrame.pivotAuthoritySource,
                    _grabFrame.pivotAuthorityPositionOnly ? "yes" : "no",
                    _grabFrame.pivotAuthorityNormalTrusted ? "yes" : "no",
                    _grabFrame.pivotAuthorityPositionConfidence,
                    _grabFrame.palmSeatPointMode, _grabFrame.fingerEvidencePointMode,
                    _grabFrame.fingerPoseTargetCount, _grabFrame.fadeInGrabConstraint ? "yes" : "no",
                    _grabFrame.motorFadeReason, _grabFrame.bodyResolutionReason,
                    _grabFrame.fingerPoseAimValid ? "yes" : "no", _grabFrame.fingerPoseAimReason);

                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB FRAME SNAPSHOT: rawVsProxy rotDelta={:.2f}deg posDelta=({:.2f},{:.2f},{:.2f}) "
                    "rawFinger=({:.3f},{:.3f},{:.3f}) rawBack=({:.3f},{:.3f},{:.3f}) rawLat=({:.3f},{:.3f},{:.3f}) "
                    "proxyFinger=({:.3f},{:.3f},{:.3f}) proxyBack=({:.3f},{:.3f},{:.3f}) proxyLat=({:.3f},{:.3f},{:.3f})",
                    handName(), rawVsProxyRot, grabPosDelta.x, grabPosDelta.y, grabPosDelta.z, rawFinger.x,
                    rawFinger.y, rawFinger.z, rawBack.x, rawBack.y, rawBack.z, rawLateral.x, rawLateral.y, rawLateral.z, proxyFinger.x, proxyFinger.y, proxyFinger.z,
                    proxyBack.x, proxyBack.y, proxyBack.z, proxyLateral.x, proxyLateral.y, proxyLateral.z);

                const auto rawHandBasis = grab_transform_telemetry::makeOrientationBasis(handWorldTransform);
                const auto proxyPalmBasis = grab_transform_telemetry::makeOrientationBasis(proxyFrameWorldAtGrab);
                const auto objectAtGrabBasis = grab_transform_telemetry::makeOrientationBasis(objectWorldTransform);
                const auto desiredObjectBasis = grab_transform_telemetry::makeOrientationBasis(desiredObjectWorld);
                const auto desiredBodyBasis = grab_transform_telemetry::makeOrientationBasis(desiredBodyWorld);
                const auto grabBodyBasis = grab_transform_telemetry::makeOrientationBasis(grabBodyWorldAtGrab);
                const auto motionBodyBasis = grab_transform_telemetry::makeOrientationBasis(motionBodyWorldAtGrab);
                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB BASIS CAPTURE side={} phase=capture convention=niLocalVectorToWorld proxySource={} motionBody={} motionSrc={} proxyBodyFrame={} rotRef={} {} {} {} {} {} {} {}",
                    handName(),
                    _isLeft ? "left" : "right",
                    proxyFrameSourceAtGrab,
                    hasMotionBodyWorldAtGrab ? "yes" : "no",
                    body_frame::bodyFrameSourceCode(motionBodySourceAtGrab),
                    constraintUsesMotionBodyAtGrab ? "MOTION" : "BODY",
                    kGrabObjectRotationReferenceName,
                    grab_transform_telemetry::formatBasis("rawHand", rawHandBasis),
                    grab_transform_telemetry::formatBasis("proxyPalm", proxyPalmBasis),
                    grab_transform_telemetry::formatBasis("objectAtGrab", objectAtGrabBasis),
                    grab_transform_telemetry::formatBasis("desiredObject", desiredObjectBasis),
                    grab_transform_telemetry::formatBasis("desiredBody", desiredBodyBasis),
                    grab_transform_telemetry::formatBasis("grabBody", grabBodyBasis),
                    grab_transform_telemetry::formatBasis("motionBody", motionBodyBasis));

                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB BASIS CAPTURE DELTA side={} phase=capture {} {} {} {} {}",
                    handName(),
                    _isLeft ? "left" : "right",
                    grab_transform_telemetry::formatBasisDelta("rawHandToProxyPalm", rawHandBasis, proxyPalmBasis),
                    grab_transform_telemetry::formatBasisDelta("objectAtGrabToDesiredObject", objectAtGrabBasis, desiredObjectBasis),
                    grab_transform_telemetry::formatBasisDelta("desiredObjectToDesiredBody", desiredObjectBasis, desiredBodyBasis),
                    grab_transform_telemetry::formatBasisDelta("grabBodyToDesiredBody", grabBodyBasis, desiredBodyBasis),
                    grab_transform_telemetry::formatBasisDelta("motionBodyToGrabBody", motionBodyBasis, grabBodyBasis));

                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB BASIS CAPTURE AXISMAP side={} phase=capture {} {} {} {} {}",
                    handName(),
                    _isLeft ? "left" : "right",
                    grab_transform_telemetry::formatBasisCrossMap("bodyToProxy", grabBodyBasis, proxyPalmBasis),
                    grab_transform_telemetry::formatBasisCrossMap("desiredBodyToProxy", desiredBodyBasis, proxyPalmBasis),
                    grab_transform_telemetry::formatBasisCrossMap("proxyToBody", proxyPalmBasis, grabBodyBasis),
                    grab_transform_telemetry::formatBasisCrossMap("rawHandToProxy", rawHandBasis, proxyPalmBasis),
                    grab_transform_telemetry::formatBasisCrossMap("bodyToDesiredBody", grabBodyBasis, desiredBodyBasis));

                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB FRAME TARGETS: grabHSRaw.pos=({:.2f},{:.2f},{:.2f}) grabHSProxy.pos=({:.2f},{:.2f},{:.2f}) "
                    "grabHSRawFinger=({:.3f},{:.3f},{:.3f}) grabHSProxyFinger=({:.3f},{:.3f},{:.3f}) "
                    "bodyLocal.pos=({:.2f},{:.2f},{:.2f}) bodyLocalFinger=({:.3f},{:.3f},{:.3f})",
                    handName(), _grabFrame.rawHandSpace.translate.x, _grabFrame.rawHandSpace.translate.y, _grabFrame.rawHandSpace.translate.z, _grabFrame.proxyAuthorityHandSpace.translate.x,
                    _grabFrame.proxyAuthorityHandSpace.translate.y, _grabFrame.proxyAuthorityHandSpace.translate.z, grabSpaceRawFinger.x, grabSpaceRawFinger.y, grabSpaceRawFinger.z,
                    grabSpaceProxyFinger.x, grabSpaceProxyFinger.y, grabSpaceProxyFinger.z, _grabFrame.bodyLocal.translate.x, _grabFrame.bodyLocal.translate.y,
                    _grabFrame.bodyLocal.translate.z, _grabFrame.bodyLocal.rotate.entry[0][0], _grabFrame.bodyLocal.rotate.entry[1][0],
                    _grabFrame.bodyLocal.rotate.entry[2][0]);

                const RE::NiPoint3 rootBodyLocalFinger = getMatrixColumn(_grabFrame.rootBodyLocal.rotate, 0);
                const RE::NiPoint3 ownerBodyLocalFinger = getMatrixColumn(_grabFrame.ownerBodyLocal.rotate, 0);
                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB NODE FRAMES: owner='{}'({:p}) hasCol={} ownsBodyCol={} "
                    "root='{}'({:p}) held='{}'({:p}) mesh='{}'({:p}) sameOwnerHeld={} sameRootHeld={} "
                    "ownerBodyLocal.pos=({:.2f},{:.2f},{:.2f}) ownerBodyLocalFinger=({:.3f},{:.3f},{:.3f}) "
                    "rootBodyLocal.pos=({:.2f},{:.2f},{:.2f}) rootBodyLocalFinger=({:.3f},{:.3f},{:.3f})",
                    handName(), nodeDebugName(ownerNodeAtGrab), static_cast<const void*>(ownerNodeAtGrab),
                    (ownerNodeAtGrab && ownerNodeAtGrab->collisionObject.get()) ? "yes" : "no",
                    (ownerNodeAtGrab && ownerNodeAtGrab->collisionObject.get() == bodyCollisionObjectAtGrab) ? "yes" : "no", nodeDebugName(rootNode),
                    static_cast<const void*>(rootNode), nodeDebugName(collidableNode), static_cast<const void*>(collidableNode), nodeDebugName(meshSourceNode),
                    static_cast<const void*>(meshSourceNode), ownerNodeAtGrab == collidableNode ? "yes" : "no", rootNode == collidableNode ? "yes" : "no",
                    _grabFrame.ownerBodyLocal.translate.x, _grabFrame.ownerBodyLocal.translate.y, _grabFrame.ownerBodyLocal.translate.z, ownerBodyLocalFinger.x,
                    ownerBodyLocalFinger.y, ownerBodyLocalFinger.z, _grabFrame.rootBodyLocal.translate.x, _grabFrame.rootBodyLocal.translate.y,
                    _grabFrame.rootBodyLocal.translate.z, rootBodyLocalFinger.x, rootBodyLocalFinger.y, rootBodyLocalFinger.z);
            }

            ROCK_LOG_DEBUG(Hand,
                "{} GRAB HAND SPACE: pos=({:.1f},{:.1f},{:.1f}) "
                "palmPos=({:.1f},{:.1f},{:.1f}) pivotA=({:.1f},{:.1f},{:.1f}) grabPt=({:.1f},{:.1f},{:.1f})",
                handName(), _grabFrame.rawHandSpace.translate.x, _grabFrame.rawHandSpace.translate.y, _grabFrame.rawHandSpace.translate.z, palmPos.x, palmPos.y, palmPos.z, grabPivotAWorld.x,
                grabPivotAWorld.y, grabPivotAWorld.z, grabGripPoint.x, grabGripPoint.y, grabGripPoint.z);
            ROCK_LOG_DEBUG(Hand, "{} BODY LOCAL: pos=({:.2f},{:.2f},{:.2f}) scale={:.3f}", handName(), _grabFrame.bodyLocal.translate.x, _grabFrame.bodyLocal.translate.y,
                _grabFrame.bodyLocal.translate.z, _grabFrame.bodyLocal.scale);
        }

        {
            RE::NiTransform handBodyDiag{};
            RE::NiTransform objectBodyDiag{};
            const bool hasLiveDiag = tryResolveLiveBodyWorldTransform(world, _handBody.getBodyId(), handBodyDiag) &&
                                     tryResolveLiveBodyWorldTransform(world, objectBodyId, objectBodyDiag);
            if (hasLiveDiag) {
                ROCK_LOG_TRACE(Hand,
                    "{} DIAG: handBodyLive pos=({:.1f},{:.1f},{:.1f}) objBodyLive pos=({:.1f},{:.1f},{:.1f})",
                    handName(),
                    handBodyDiag.translate.x,
                    handBodyDiag.translate.y,
                    handBodyDiag.translate.z,
                    objectBodyDiag.translate.x,
                    objectBodyDiag.translate.y,
                    objectBodyDiag.translate.z);
            }
            ROCK_LOG_TRACE(Hand, "{} DIAG: handNi pos=({:.1f},{:.1f},{:.1f}) objNi pos=({:.1f},{:.1f},{:.1f})", handName(), handWorldTransform.translate.x,
                handWorldTransform.translate.y, handWorldTransform.translate.z, objectWorldTransform.translate.x, objectWorldTransform.translate.y,
                objectWorldTransform.translate.z);

            float comX, comY, comZ;
            if (getBodyCOMWorld(world, objectBodyId, comX, comY, comZ) && hasLiveDiag) {
                ROCK_LOG_TRACE(Hand,
                    "{} B8 COM LIVE: comHk=({:.3f},{:.3f},{:.3f}) objBodyLive=({:.1f},{:.1f},{:.1f})",
                    handName(),
                    comX,
                    comY,
                    comZ,
                    objectBodyDiag.translate.x,
                    objectBodyDiag.translate.y,
                    objectBodyDiag.translate.z);
            }
        }


        return true;
    }

    bool Hand::commitGrabDrive(hand_grab_detail::GrabAcquisitionContext& context)
    {
        if (!context.world || !context.bhkWorld || !context.selection.isValid() || !context.sharedContext) {
            return false;
        }

        auto* world = context.world;
        auto* bhkWorld = context.bhkWorld;
        const auto& sel = context.selection;
        const auto& sharedContext = *context.sharedContext;
        const auto* bodyBoneColliders = context.bodyBoneColliders;
        auto* rootNode = context.rootNode;
        auto& activeLifecycle = context.activeLifecycle;
        const auto& handWorldTransform = context.handWorldTransform;
        const float tau = context.tau;
        const float damping = context.damping;
        const float maxForce = context.maxForce;
        const float proportionalRecovery = context.proportionalRecovery;
        const float constantRecovery = context.constantRecovery;
        const auto objectBodyId = context.objectBodyId;
        const bool joiningPeerHeldObject = context.joiningPeerHeldObject;
        const bool looseWeaponGrab = context.looseWeaponGrab;
        const auto& proxyFrameWorldAtGrab = context.proxyFrameWorldAtGrab;
        const auto& grabGripPoint = context.grabGripPoint;
        const bool meshGrabFound = context.meshGrabFound;
        const char* grabPointMode = context.grabPointMode;
        const char* grabFallbackReason = context.grabFallbackReason;

        auto abortGrab = [&]() -> bool {
            abortGrabAcquisition(GrabAcquisitionUnwind{
                .world = world,
                .bhkWorld = bhkWorld,
                .rootNode = rootNode,
                .lifecycle = &activeLifecycle,
                .objectBodyId = objectBodyId,
                .targetKind = sel.targetKind,
                .originalMotionPropsId = context.selectedOriginalMotionPropsId,
                .joiningPeerHeldObject = joiningPeerHeldObject,
                .consumedPullPrepLifecycle = context.consumedPullPrepLifecycle,
            });
            return false;
        };

        {
            const RE::hkVector4f zeroVel{ 0.0f, 0.0f, 0.0f, 0.0f };
            havok_runtime::setBodyVelocityDeferred(world, objectBodyId.value, zeroVel, zeroVel);

            for (auto bid : _heldBodyIds) {
                if (bid != objectBodyId.value) {
                    havok_runtime::setBodyVelocityDeferred(world, bid, zeroVel, zeroVel);
                }
            }
        }

        const auto grabActivation = activateHeldObjectBodySet(world, objectBodyId.value, _heldBodyIds);
        if (grabActivation.failedActivationCount > 0) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB activation incomplete: primaryBody={} bodies={} activated={} failed={}",
                handName(),
                objectBodyId.value,
                grabActivation.bodyCount,
                grabActivation.activatedCount,
                grabActivation.failedActivationCount);
        }

        suppressHandCollisionForGrab(world, bodyBoneColliders);

        if (joiningPeerHeldObject && sharedContext.peerSavedObjectState) {
            copyPeerInertiaSnapshot(_savedObjectState, *sharedContext.peerSavedObjectState);
            ROCK_LOG_DEBUG(Hand,
                "{} hand joined peer-held object inertia snapshot: formID={:08X} peerMotions={} inertiaModified={}",
                handName(),
                sel.refr ? sel.refr->GetFormID() : 0,
                sharedContext.peerSavedObjectState->motionInertiaStates.size(),
                sharedContext.peerSavedObjectState->inertiaModified ? "yes" : "no");
        } else {
            normalizeGrabbedInertiaForBodies(world, objectBodyId, _heldBodyIds, _savedObjectState, looseWeaponGrab);
        }

        {
            RE::NiPoint3 legacyPalmPivotAWorld = computeGrabLegacyPalmPivotAWorldFromHandBasis(handWorldTransform, _isLeft);
            RE::NiPoint3 grabPivotAWorld =
                _grabFrame.hasTelemetryCapture ? _grabFrame.grabPivotWorldAtGrab : computeGrabPivotAWorld(world, handWorldTransform);
            const float gameToHkScale = gameToHavokScale();
            const RE::NiTransform initialDesiredBodyWorld = _grabFrame.desiredBodyWorldAtGrab;

            float pivotAHk[4];
            pivotAHk[0] = grabPivotAWorld.x * gameToHkScale;
            pivotAHk[1] = grabPivotAWorld.y * gameToHkScale;
            pivotAHk[2] = grabPivotAWorld.z * gameToHkScale;
            pivotAHk[3] = 0.0f;

            float gripWorldHk[4];
            gripWorldHk[0] = grabGripPoint.x * gameToHkScale;
            gripWorldHk[1] = grabGripPoint.y * gameToHkScale;
            gripWorldHk[2] = grabGripPoint.z * gameToHkScale;
            gripWorldHk[3] = 0.0f;

            {
                float pivotAToGrab = std::sqrt(
                    (pivotAHk[0] - gripWorldHk[0]) * (pivotAHk[0] - gripWorldHk[0]) + (pivotAHk[1] - gripWorldHk[1]) * (pivotAHk[1] - gripWorldHk[1]) +
                    (pivotAHk[2] - gripWorldHk[2]) * (pivotAHk[2] - gripWorldHk[2]));
                float legacyPalmPivotAToHandOrigin = std::sqrt((legacyPalmPivotAWorld.x - handWorldTransform.translate.x) * (legacyPalmPivotAWorld.x - handWorldTransform.translate.x) +
                    (legacyPalmPivotAWorld.y - handWorldTransform.translate.y) * (legacyPalmPivotAWorld.y - handWorldTransform.translate.y) +
                    (legacyPalmPivotAWorld.z - handWorldTransform.translate.z) * (legacyPalmPivotAWorld.z - handWorldTransform.translate.z));
                float pivotAToHandOrigin = std::sqrt((grabPivotAWorld.x - handWorldTransform.translate.x) * (grabPivotAWorld.x - handWorldTransform.translate.x) +
                    (grabPivotAWorld.y - handWorldTransform.translate.y) * (grabPivotAWorld.y - handWorldTransform.translate.y) +
                    (grabPivotAWorld.z - handWorldTransform.translate.z) * (grabPivotAWorld.z - handWorldTransform.translate.z));
                ROCK_LOG_DEBUG(Hand,
                    "GRAB DIAG {}: legacyPalmPivotAWorld=({:.1f},{:.1f},{:.1f}) handPos=({:.1f},{:.1f},{:.1f}) "
                    "pocket=({:.1f},{:.1f},{:.1f}) grip=({:.1f},{:.1f},{:.1f}) meshGrab={} grabPointMode={} fallbackReason={} "
                    "frozenPivotB=({:.2f},{:.2f},{:.2f}) contactPatchEvidence={} contactPatchPivot={} patchHits={} multiFinger={} mfGroups={} mfSpread={:.2f} "
                    "activePoint={} activeUsesFingerEvidence={} palmSeatPoint={} fingerEvidencePoint={} "
                    "pivotAToGrab_hk={:.4f} ({:.1f} game units) legacyPalmPivotAToHandOrigin={:.1f} pivotAToHandOrigin={:.1f} game units "
                    "selectionToGripEvidence={:.1f} fingerPoseAim={} fingerPoseAimReason={}",
                    handName(), legacyPalmPivotAWorld.x, legacyPalmPivotAWorld.y, legacyPalmPivotAWorld.z, handWorldTransform.translate.x, handWorldTransform.translate.y, handWorldTransform.translate.z, grabPivotAWorld.x,
                    grabPivotAWorld.y, grabPivotAWorld.z, grabGripPoint.x, grabGripPoint.y, grabGripPoint.z, meshGrabFound, grabPointMode, grabFallbackReason,
                    _grabFrame.pivotBBodyLocalGame.x, _grabFrame.pivotBBodyLocalGame.y, _grabFrame.pivotBBodyLocalGame.z,
                    _grabFrame.hasContactPatchEvidence ? "yes" : "no", "no",
                    _grabFrame.contactPatchSampleCount, _grabFrame.hasMultiFingerContactPatch ? "yes" : "no", _grabFrame.multiFingerContactGroupCount,
                    _grabFrame.multiFingerContactSpreadGameUnits, _grabFrame.activeGrabPointMode, _grabFrame.activeGrabPointUsesMultiFingerEvidence ? "yes" : "no",
                    _grabFrame.palmSeatPointMode, _grabFrame.fingerEvidencePointMode, pivotAToGrab,
                    pivotAToGrab * havokToGameScale(), legacyPalmPivotAToHandOrigin, pivotAToHandOrigin, _grabFrame.selectionToGripEvidenceDistanceGameUnits,
                    _grabFrame.fingerPoseAimValid ? "yes" : "no", _grabFrame.fingerPoseAimReason);
            }

            const char* driveReason = joiningPeerHeldObject ? "joining-peer-held-loose-object" : "ordinary-dynamic-loose-object";
            if (!createProxyConstraintGrabDrive(
                    bhkWorld,
                    world,
                    objectBodyId,
                    proxyFrameWorldAtGrab,
                    handWorldTransform,
                    grabPivotAWorld,
                    tau,
                    damping,
                    maxForce,
                    held_object_drive_policy::sanitizeMotorAuthorityScale(sharedGrabAuthorityForceScale(joiningPeerHeldObject)),
                    proportionalRecovery,
                    constantRecovery,
                    looseWeaponGrab,
                    driveReason)) {
                ROCK_LOG_ERROR(Hand,
                    "{} hand GRAB FAILED: proxy constraint creation failed bodyId={} targetBody=({:.2f},{:.2f},{:.2f}) pivotBConstraint=({:.2f},{:.2f},{:.2f}) joiningPeer={}",
                    handName(),
                    objectBodyId.value,
                    initialDesiredBodyWorld.translate.x,
                    initialDesiredBodyWorld.translate.y,
                    initialDesiredBodyWorld.translate.z,
                    activeProxyConstraintPivotBLocalGame().x,
                    activeProxyConstraintPivotBLocalGame().y,
                    activeProxyConstraintPivotBLocalGame().z,
                    joiningPeerHeldObject ? "yes" : "no");
            }
        }

        const bool driveCreated = _activeConstraint.isValid() && _grabAuthorityProxy.isValid();
        if (!driveCreated) {
            ROCK_LOG_ERROR(Hand, "{} hand GRAB FAILED: proxy-constraint dynamic grab creation failed", handName());
            return abortGrab();
        }

        _heldObjectIsLooseWeapon = looseWeaponGrab;
        if (_heldObjectIsLooseWeapon) {
            suppressBodyCollisionForHeldLooseWeapon(world, bodyBoneColliders);
        }
        const auto massSummaryAtGrab = readHeldBodyMassSummary(
            world,
            _savedObjectState.bodyId,
            _heldBodyIds,
            _heldDriveDecision.includeConnectedMass);
        const float effectiveMassAtGrab = effectiveGrabMotorMass(massSummaryAtGrab.motorMass());
        const float sharedLinearForce = _activeConstraint.linearMotor ?
            (std::max)(std::fabs(_activeConstraint.linearMotor->minForce), std::fabs(_activeConstraint.linearMotor->maxForce)) :
            maxForce;
        const float sharedAngularForce = _activeConstraint.angularMotor ?
            (std::max)(std::fabs(_activeConstraint.angularMotor->minForce), std::fabs(_activeConstraint.angularMotor->maxForce)) :
            0.0f;
        ROCK_LOG_DEBUG(Hand,
            "{} hand dynamic grab created: drive={} bodyDriveMode={} driveReason={} forceShare={:.2f} linearScope={} angularScope={} massScope={} looseWeapon={} constraint={} proxyBody={} handBody={} objBody={} heldBodies={} mass={:.2f} effectiveMotorMass={:.2f} primaryMass={:.2f} massBodies={} motions={} longLever={:.1f}gu linearTau={:.3f} angularTau={:.3f} linearDamping={:.2f} angularDamping={:.2f} linearForce={:.0f} angularForce={:.0f} propRecov={:.1f} constRecov={:.1f} rotRef={}",
            handName(),
            kHeldObjectDriveName,
            held_object_drive_policy::modeName(_heldDriveDecision.mode),
            _heldDriveDecision.reason,
            sharedGrabAuthorityForceScale(joiningPeerHeldObject),
            _heldDriveDecision.includeConnectedLinearVelocity ? "bodySet" : "primaryOnly",
            _heldDriveDecision.includeConnectedAngularVelocity ? "bodySet" : "primaryOnly",
            _heldDriveDecision.includeConnectedMass ? "bodySet" : "primaryOnly",
            _heldObjectIsLooseWeapon ? "yes" : "no",
            _activeConstraint.constraintId,
            _grabAuthorityProxy.isValid() ? _grabAuthorityProxy.getBodyId().value : INVALID_BODY_ID,
            _handBody.getBodyId().value,
            objectBodyId.value,
            _heldBodyIds.size(),
            massSummaryAtGrab.motorMass(),
            effectiveMassAtGrab,
            massSummaryAtGrab.primaryMass,
            massSummaryAtGrab.sampledBodies,
            massSummaryAtGrab.uniqueMotions,
            _grabFrame.longObjectLeverGameUnits,
            _activeConstraint.linearMotor ? _activeConstraint.linearMotor->tau : tau,
            _activeConstraint.angularMotor ? _activeConstraint.angularMotor->tau : g_rockConfig.rockGrabAngularTau,
            _activeConstraint.linearMotor ? _activeConstraint.linearMotor->damping : damping,
            _activeConstraint.angularMotor ? _activeConstraint.angularMotor->damping : g_rockConfig.rockGrabAngularDamping,
            sharedLinearForce,
            sharedAngularForce,
            _activeConstraint.linearMotor ? _activeConstraint.linearMotor->proportionalRecoveryVelocity : proportionalRecovery,
            _activeConstraint.linearMotor ? _activeConstraint.linearMotor->constantRecoveryVelocity : constantRecovery,
            kGrabObjectRotationReferenceName);

        const auto heldFlagLeases =
            acquireHeldObjectBodyFlagLeases(world, _savedObjectState.bodyId.value, _heldBodyIds, heldBodyFlagLeaseOwner(this));
        if (heldFlagLeases.failedLeaseCount > 0) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB held body flag lease incomplete: primaryBody={} bodies={} collision={} authority={} failed={}",
                handName(),
                _savedObjectState.bodyId.value,
                heldFlagLeases.bodyCount,
                heldFlagLeases.collisionLeaseCount,
                heldFlagLeases.authorityLeaseCount,
                heldFlagLeases.failedLeaseCount);
        } else {
            ROCK_LOG_DEBUG(Hand,
                "{} hand GRAB held body flag leases acquired: primaryBody={} bodies={} collision={} authority={}",
                handName(),
                _savedObjectState.bodyId.value,
                heldFlagLeases.bodyCount,
                heldFlagLeases.collisionLeaseCount,
                heldFlagLeases.authorityLeaseCount);
        }
        clearHeldBodyContactSnapshot();
        _activeGrabLifecycle = std::move(activeLifecycle);

        {
            int count = (std::min)(static_cast<int>(_heldBodyIds.size()), MAX_HELD_BODIES);
            for (int i = 0; i < count; i++) {
                _heldBodyIdsSnapshot[i] = _heldBodyIds[i];
            }
            _heldBodyIdsCount.store(count, std::memory_order_release);
            _isHoldingFlag.store(true, std::memory_order_release);
        }

        if (g_rockConfig.rockGrabNearbyDampingEnabled) {
            object_physics_body_set::BodySetScanOptions dampingOptions{};
            dampingOptions.mode = physics_body_classifier::InteractionMode::PassivePush;
            dampingOptions.rightHandBodyId = _isLeft ? INVALID_BODY_ID : _handBody.getBodyId().value;
            dampingOptions.leftHandBodyId = _isLeft ? _handBody.getBodyId().value : INVALID_BODY_ID;
            dampingOptions.heldBySameHand = &_heldBodyIds;
            dampingOptions.maxDepth = (std::max)(1, g_rockConfig.rockObjectPhysicsTreeMaxDepth);
            {
                performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabNearbyDampingBegin);
                _nearbyGrabDamping = nearby_grab_damping::beginNearbyGrabDamping(bhkWorld,
                    world,
                    sel.refr,
                    _heldBodyIds,
                    grabGripPoint,
                    g_rockConfig.rockGrabNearbyDampingRadius,
                    g_rockConfig.rockGrabNearbyDampingSeconds,
                    g_rockConfig.rockGrabNearbyLinearDamping,
                    g_rockConfig.rockGrabNearbyAngularDamping,
                    dampingOptions);
            }
            performance_profiler::observeValue(performance_profiler::ValueMetric::GrabNearbyDampingMotions, _nearbyGrabDamping.motions.size());
        } else {
            _nearbyGrabDamping.clear();
        }

        return true;
    }


    void Hand::publishGrabFingerPose(hand_grab_detail::GrabAcquisitionContext& context)
    {
        if (!context.world || !context.selection.isValid()) {
            return;
        }

        auto* world = context.world;
        const auto& sel = context.selection;
        const auto& handWorldTransform = context.handWorldTransform;
        const auto& objectWorldTransform = context.objectWorldTransform;
        const auto& resolvedGrabOffsetSource = context.resolvedGrabOffsetSource;
        const auto& grabFingerPoseMeshTriangles = context.grabFingerPoseMeshTriangles;

        stopSelectionHighlight();
        clearSelectedCloseFingerPose();
        _grabFingerPose = {};
        _grabFingerTriangleIndex.clear();
        const bool useLooseWeaponPrimaryAttachHandPose = _grabFrame.syntheticLooseWeaponPrimaryAttach;
        _hasGrabFingerPose = g_rockConfig.rockGrabMeshFingerPoseEnabled && !useLooseWeaponPrimaryAttachHandPose;
        _grabFingerProbeStart = {};
        _grabFingerProbeEnd = {};
        _hasGrabFingerProbeDebug = false;
        _grabFingerSweepDebugCapture = {};
        _grabFingerSweepDebugObjectWorld = {};
        _hasGrabFingerSweepDebug = false;
        _grabFingerPadProbeStart = {};
        _grabFingerPadProbeEnd = {};
        _grabFingerPadProbeHit = {};
        _grabFingerPadProbeHitValid = {};
        _hasGrabFingerPadProbeDebug = false;
        _grabFingerSurfaceTarget = {};
        _grabFingerSurfaceTargetValid = {};
        _hasGrabFingerSurfaceTargetDebug = false;
        _grabFingerPosePublished = false;
        if (useLooseWeaponPrimaryAttachHandPose) {
            const auto grabOffsetFingerPoseSource = g_rockConfig.rockGrabMeshFingerPoseEnabled ?
                resolveGrabOffsetFingerPoseSource(resolvedGrabOffsetSource) :
                GrabOffsetFingerPoseSource{};
            if (grabOffsetFingerPoseSource.valid) {
                /*
                 * Reuse the same calibrated-or-saved source that established
                 * the object attach. This synthetic attach has no mesh contact
                 * to solve fingers from, so without the resolved snapshot it
                 * would fall back to a generic canned pose. This is a one-time
                 * publish; mesh-resolve paths must not mutate it afterward.
                 */
                _grabFingerPose = grabOffsetFingerPoseSource.pose;
                applyRockGrabHandPose(_isLeft,
                    _grabFingerPose,
                    _grabFingerJointPose,
                    _hasGrabFingerJointPose,
                    _grabFingerLocalTransforms,
                    _grabFingerLocalTransformMask,
                    _hasGrabFingerLocalTransforms,
                    0.0f,
                    /*publishLocalTransforms=*/false);
                _grabFingerPosePublished = true;
                ROCK_LOG_INFO(Hand,
                    "{} hand loose weapon attach: applying {} finger pose",
                    handName(),
                    grabOffsetFingerPoseSource.reason);
            } else {
                _grabFingerPosePublished = publishLooseWeaponPrimaryAttachHandPose(_isLeft, sel.refr);
                if (!_grabFingerPosePublished) {
                    ROCK_LOG_WARN(Hand, "{} hand loose weapon attach: failed to publish FRIK weapon hand pose", handName());
                }
            }
        } else if (_hasGrabFingerPose) {
            const bool pinchFingerPose = _grabFrame.seatMode == GrabSeatMode::PinchPocket;
            const bool touchHeldAtCommit = _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld;
            if (pinchFingerPose && !touchHeldAtCommit) {
                ROCK_LOG_DEBUG(Hand, "{} THREE-PHASE GRAB POSE: pinch solve deferred until TouchHeld phase={} cachedTriangles={} poseTargets={}", handName(),
                    grab_three_phase::phaseName(_grabAcquisitionPhase),
                    _grabFrame.fingerPoseLocalMeshTriangles.empty() ? _grabFrame.localMeshTriangles.size() : _grabFrame.fingerPoseLocalMeshTriangles.size(),
                    _grabFrame.fingerPoseTargetCount);
            } else {
                /*
                 * Regular grabs solve exactly once against the already-frozen
                 * commanded seat, never against the object's transient approach
                 * pose. The endpoint therefore exists before the first approach
                 * frame and cannot visibly change at TouchHeld. Pinch keeps its
                 * established at-touch triangle path above.
                */
                const RE::NiTransform& targetObjectWorld = pinchFingerPose ? objectWorldTransform : _grabFrame.desiredObjectWorldAtGrab;
                std::vector<TriangleData> targetFingerPoseWorldTriangles = pinchFingerPose ? grabFingerPoseMeshTriangles : std::vector<TriangleData>{};
                const auto& localFingerPoseTriangles = !_grabFrame.fingerPoseLocalMeshTriangles.empty() ? _grabFrame.fingerPoseLocalMeshTriangles : _grabFrame.localMeshTriangles;

                const RE::NiTransform& targetFingerHandTransform = handWorldTransform;
                root_flattened_finger_skeleton_runtime::Snapshot liveFingerSnapshotAtGrab{};
                const RE::NiPoint3 fingerPosePivotWorld =
                    _grabFrame.hasTelemetryCapture ? _grabFrame.grabPivotWorldAtGrab : computeGrabPivotAWorld(world, targetFingerHandTransform);
                const auto targetFingerPoseTargets = rebuildFingerPoseTargetsFromGrabFrame(_grabFrame, targetObjectWorld);
                grab_finger_pose_runtime::FingerSweepDebugCapture sweepDebugCapture{};
                grab_finger_pose_runtime::SolvedGrabFingerPose fingerPose{};
                bool spatialIndexBuilt = false;
                bool commandedOpenDirectionsValid = false;
                if (pinchFingerPose) {
                    const auto* liveFingerSnapshotAtGrabPtr =
                        root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(_isLeft, liveFingerSnapshotAtGrab) ? &liveFingerSnapshotAtGrab : nullptr;
                    fingerPose = grab_finger_pose_runtime::solveGrabFingerPoseFromTriangles(targetFingerPoseWorldTriangles, targetFingerHandTransform, _isLeft, fingerPosePivotWorld,
                        targetFingerPoseTargets, g_rockConfig.rockGrabFingerMinValue, g_rockConfig.rockGrabMaxTriangleDistance, !pinchFingerPose, liveFingerSnapshotAtGrabPtr,
                        g_rockConfig.rockGrabFingerRejectBacksideHits, g_rockConfig.rockGrabFingerSurfacePlaneToleranceGameUnits, _grabFrame.fingerPoseAimValid,
                        g_rockConfig.rockGrabFingerSweepContactRadiusGameUnits, -1.0f, g_rockConfig.rockGrabThumbSweepMaxOpenValue, g_rockConfig.rockGrabFingerSweepMaxOpenValue,
                        nullptr, nullptr, nullptr, nullptr, grab_finger_pose_runtime::FingerPoseMeshRelation::AlreadyAtCommandedSeat);
                    applyPinchFingerPosePolicy(fingerPose, _grabFrame, g_rockConfig.rockGrabFingerMinValue);
                    grab_finger_pose_runtime::useThumbIndexCurveOnlyPose(fingerPose);
                    std::array<grab_finger_pose_runtime::FingerPadSurfaceEvidence, 5> padCaptureEvidence{};
                    (void)grab_finger_pose_runtime::refineGrabFingerPoseWithPadProbes(fingerPose, targetFingerPoseWorldTriangles, targetFingerPoseTargets,
                        liveFingerSnapshotAtGrab, targetObjectWorld, g_rockConfig.rockGrabMeshFingerPoseEnabled, true, padCaptureEvidence, true);
                    grab_finger_pose_runtime::captureSurfaceAimObjectLocal(fingerPose, targetObjectWorld);
                } else {
                    const auto frozenSolve = grab_finger_pose_runtime::solveFrozenMeshFingerPose(
                        localFingerPoseTriangles,
                        targetObjectWorld,
                        targetFingerHandTransform,
                        _isLeft,
                        fingerPosePivotWorld,
                        targetFingerPoseTargets,
                        _grabFingerTriangleIndex,
                        targetFingerPoseWorldTriangles,
                        grab_finger_pose_runtime::FrozenMeshFingerPoseSolveOptions{
                            .minValue = g_rockConfig.rockGrabFingerMinValue,
                            .maxTriangleDistanceSquared = g_rockConfig.rockGrabMaxTriangleDistance,
                            .rejectBacksideHits = g_rockConfig.rockGrabFingerRejectBacksideHits,
                            .surfacePlaneToleranceGameUnits = g_rockConfig.rockGrabFingerSurfacePlaneToleranceGameUnits,
                            .allowSurfaceAimTargets = _grabFrame.fingerPoseAimValid,
                            .sweepContactRadiusGameUnits = g_rockConfig.rockGrabFingerSweepContactRadiusGameUnits,
                            .thumbSweepMaxOpenValue = g_rockConfig.rockGrabThumbSweepMaxOpenValue,
                            .fingerSweepMaxOpenValue = g_rockConfig.rockGrabFingerSweepMaxOpenValue,
                            .meshFingerPoseEnabled = g_rockConfig.rockGrabMeshFingerPoseEnabled,
                            .captureSweepDebug = g_rockConfig.rockDebugShowGrabFingerSweptArc,
                        });
                    fingerPose = frozenSolve.pose;
                    sweepDebugCapture = frozenSolve.sweepDebug;
                    liveFingerSnapshotAtGrab = frozenSolve.liveFingerSnapshot;
                    spatialIndexBuilt = frozenSolve.spatialIndexBuilt;
                    commandedOpenDirectionsValid = frozenSolve.commandedOpenDirectionsValid;
                }
                _grabFingerSweepDebugCapture = sweepDebugCapture;
                _grabFingerSweepDebugObjectWorld = targetObjectWorld;
                _hasGrabFingerSweepDebug = sweepDebugCapture.valid;
                _grabFingerPose = fingerPose;
                _grabFingerProbeStart = fingerPose.probeStart;
                _grabFingerProbeEnd = fingerPose.probeEnd;
                _hasGrabFingerProbeDebug = fingerPose.candidateTriangleCount > 0;
                auto publishFingerPose = grab_finger_pose_runtime::resolveSurfaceAimObjectLocal(_grabFingerPose, targetObjectWorld);
                if (g_rockConfig.rockDebugShowGrabFingerProbes) {
                    std::array<grab_finger_pose_runtime::FingerPadSurfaceEvidence, 5> padEvidence{};
                    (void)grab_finger_pose_runtime::refineGrabFingerPoseWithPadProbes(publishFingerPose, targetFingerPoseWorldTriangles, targetFingerPoseTargets,
                        liveFingerSnapshotAtGrab, targetObjectWorld, g_rockConfig.rockGrabMeshFingerPoseEnabled, true, padEvidence, false);
                    const auto padDebug = makeFingerPadPublishDebug(publishFingerPose, padEvidence);
                    _grabFingerPadProbeStart = padDebug.padProbeStart;
                    _grabFingerPadProbeEnd = padDebug.padProbeEnd;
                    _grabFingerPadProbeHit = padDebug.padProbeHit;
                    _grabFingerPadProbeHitValid = padDebug.padProbeHitValid;
                    _hasGrabFingerPadProbeDebug = padDebug.hasPadProbeDebug;
                    _grabFingerSurfaceTarget = padDebug.surfaceTarget;
                    _grabFingerSurfaceTargetValid = padDebug.surfaceTargetValid;
                    _hasGrabFingerSurfaceTargetDebug = padDebug.hasSurfaceTargetDebug;
                } else {
                    _hasGrabFingerPadProbeDebug = false;
                    _hasGrabFingerSurfaceTargetDebug = false;
                }
                if (!touchHeldAtCommit) {
                    publishFingerPose = buildAcquisitionFingerPose(publishFingerPose, 0.0f);
                }
                applyRockGrabHandPose(_isLeft, publishFingerPose, _grabFingerJointPose, _hasGrabFingerJointPose, _grabFingerLocalTransforms, _grabFingerLocalTransformMask,
                    _hasGrabFingerLocalTransforms, 0.0f, touchHeldAtCommit, touchHeldAtCommit);
                _grabFingerPosePublished = true;
                ROCK_LOG_DEBUG(Hand, "{} THREE-PHASE GRAB POSE TARGET: phase={} targetSpace=yes solved={} hits={} triangles={} spatial={} nodes={} tests={} commandedAnchors={}",
                    handName(), grab_three_phase::phaseName(_grabAcquisitionPhase), _grabFingerPose.solved ? "yes" : "no", _grabFingerPose.hitCount,
                    _grabFingerPose.candidateTriangleCount, _grabFingerPose.usedSpatialIndex ? "yes" : "no", _grabFingerPose.spatialNodeVisitCount,
                    _grabFingerPose.spatialTriangleTestCount, commandedOpenDirectionsValid ? "yes" : "no");
            }
        }


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
