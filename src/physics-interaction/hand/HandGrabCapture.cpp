// Grab capture, first half: validates the selected grab, prepares the object's
// bodies and the authority proxy, extracts the object's mesh near the hand and
// resolves the grab surface, contact policy, grabbed body and pivot from that
// evidence (beginResolvedGrabCommit).
#include "physics-interaction/hand/HandGrabInternal.h"

namespace rock
{

    namespace
    {
        bool sharedContextMatchesSelection(const GrabSharedObjectContext& sharedContext,
            const SelectedObject& selection)
        {
            if (!sharedContext.hasPeerState() || !selection.refr ||
                sharedContext.peerSavedObjectState->refr != selection.refr)
                return false;
            if (!grab_target::requiresNativeBodyOwnership(selection.targetKind))
                return true;
            auto* cell = selection.refr->GetParentCell();
            auto* world = cell ? havok_runtime::getHknpWorldFromBhk(cell->GetbhkWorld()) : nullptr;
            const auto component = ragdoll::readComponent(world, selection.bodyId.value);
            return component.valid && component.contains(sharedContext.peerSavedObjectState->bodyId.value);
        }

        bool isLooseWeaponGrabTarget(const SelectedObject& selection)
        {
            if (!selection.refr || !grab_target::canUseRockActiveGrab(selection.targetKind)) {
                return false;
            }

            auto* selectedBase = selection.refr->GetObjectReference();
            return selectedBase && selectedBase->Is(RE::ENUM_FORM_ID::kWEAP);
        }

        std::vector<std::uint32_t> buildCommittedHeldBodyIds(std::uint32_t primaryBodyId,
            const std::vector<std::uint32_t>& mechanicalScopeBodyIds, const GrabSharedObjectContext& sharedContext,
            bool& adoptedPeerHeldBodyIds)
        {
            /*
             * The second hand has its own selected primary body for drive-frame
             * capture, but the object body set is already owned by the first hand.
             * Reusing the peer's committed body list keeps multipart activation,
             * flag leases, release velocity, and final restoration on the same
             * bodies instead of depending on a second close-selection rescan.
             */
            adoptedPeerHeldBodyIds = false;
            std::vector<std::uint32_t> sourceBodyIds;
            if (sharedContext.hasPeerState() && sharedContext.peerHeldBodyIds &&
                !sharedContext.peerHeldBodyIds->empty()) {
                sourceBodyIds = *sharedContext.peerHeldBodyIds;
                adoptedPeerHeldBodyIds = true;
            } else {
                sourceBodyIds = mechanicalScopeBodyIds;
            }

            return held_object_body_set_policy::makePrimaryFirstUniqueBodyList(primaryBodyId, sourceBodyIds);
        }

        GrabSurfaceHit makeCollisionQueryGrabSurfaceHit(const SelectedObject& selection,
            RE::NiAVObject* fallbackOwnerNode)
        {
            GrabSurfaceHit result{};
            if (!selection.hasHitPoint || !selection.hasHitNormal) {
                return result;
            }

            result.position = selection.hitPointWorld;
            result.normal = normalizeOrZero(selection.hitNormalWorld);
            result.triangleIndex = -1;
            result.distance = selection.distance;
            result.sourceNode = selection.hitNode ? selection.hitNode : fallbackOwnerNode;
            result.sourceShape = nullptr;
            result.sourceKind = GrabSurfaceSourceKind::CollisionQuery;
            result.shapeKey = selection.hitShapeKey;
            result.shapeCollisionFilterInfo = selection.hitShapeCollisionFilterInfo;
            result.hitFraction = selection.hitFraction;
            result.hasSelectionHit = true;
            result.selectionToMeshDistanceGameUnits = 0.0f;
            result.signedAlongPalmDistanceGameUnits = selection.signedAlongDistance;
            result.lateralPalmDistanceGameUnits = selection.lateralDistance;
            result.hasShapeKey = selection.hasHitShapeKey;
            result.hasTriangle = false;
            result.valid = lengthSquared(result.normal) > 0.0f;
            return result;
        }

        GrabCaptureTransformRefreshResult refreshGrabCaptureTransforms(RE::NiAVObject* rootNode,
            RE::NiAVObject* meshSourceNode, RE::NiAVObject* collidableNode, bool refreshScene = true)
        {
            GrabCaptureTransformRefreshResult result{};
            refreshGrabCaptureNodeTransform(result, "root", rootNode, refreshScene);
            refreshGrabCaptureNodeTransform(result, "mesh", meshSourceNode, refreshScene);
            refreshGrabCaptureNodeTransform(result, "collidable", collidableNode, refreshScene);
            return result;
        }

        float axisDeltaDegrees(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            const float dot = a.x * b.x + a.y * b.y + a.z * b.z;
            const float lenA = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
            const float lenB = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
            if (lenA < 0.0001f || lenB < 0.0001f) {
                return -1.0f;
            }

            float cosTheta = dot / (lenA * lenB);
            if (cosTheta < -1.0f) {
                cosTheta = -1.0f;
            } else if (cosTheta > 1.0f) {
                cosTheta = 1.0f;
            }
            return std::acos(cosTheta) * (180.0f / 3.14159265358979323846f);
        }

        float matrixDeterminant(const RE::NiMatrix3& matrix)
        {
            return matrix.entry[0][0] *
                (matrix.entry[1][1] * matrix.entry[2][2] - matrix.entry[1][2] * matrix.entry[2][1]) -
                matrix.entry[0][1] *
                (matrix.entry[1][0] * matrix.entry[2][2] - matrix.entry[1][2] * matrix.entry[2][0]) +
                matrix.entry[0][2] *
                (matrix.entry[1][0] * matrix.entry[2][1] - matrix.entry[1][1] * matrix.entry[2][0]);
        }

        RE::NiPoint3 frameAxisWorld(const RE::NiTransform& transform, const RE::NiPoint3& localAxis)
        {
            return normalizeOrZero(transform_math::localVectorToWorld(transform, localAxis));
        }

        RE::NiPoint3 generatedFrameAxisWorld(const RE::NiTransform& transform, const RE::NiPoint3& localAxis)
        {
            return normalizeOrZero(
                hand_bone_collider_geometry_math::generatedColliderLocalVectorToWorld(transform, localAxis));
        }

        GrabPalmBasisDelta computeGrabPalmBasisDelta(const RE::NiTransform& rawHandWorld,
            const RE::NiTransform& proxyWorld)
        {
            GrabPalmBasisDelta result{};
            result.rotationDegrees = rotationDeltaDegrees(rawHandWorld.rotate, proxyWorld.rotate);
            result.xAxisDegrees = axisDeltaDegrees(frameAxisWorld(rawHandWorld, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }),
                generatedFrameAxisWorld(proxyWorld, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }));
            result.yAxisDegrees = axisDeltaDegrees(frameAxisWorld(rawHandWorld, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }),
                generatedFrameAxisWorld(proxyWorld, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }));
            result.zAxisDegrees = axisDeltaDegrees(frameAxisWorld(rawHandWorld, RE::NiPoint3{ 0.0f, 0.0f, 1.0f }),
                generatedFrameAxisWorld(proxyWorld, RE::NiPoint3{ 0.0f, 0.0f, 1.0f }));
            result.rawDeterminant = matrixDeterminant(rawHandWorld.rotate);
            result.proxyDeterminant = matrixDeterminant(proxyWorld.rotate);
            return result;
        }

        RE::NiTransform getLiveBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId)
        {
            RE::NiTransform result = makeIdentityTransform();
            tryResolveLiveBodyWorldTransform(world, bodyId, result);
            return result;
        }

        void logRuntimeScaleIfChanged(bool isLeft, const char* handName, const RE::NiTransform& handWorldTransform,
            const RE::NiAVObject* collidableNode)
        {
            struct RuntimeScaleLogState
            {
                bool initialized = false;
                float handScale = 0.0f;
                float collidableScale = 0.0f;
                float vrScale = 0.0f;
            };

            static std::array<RuntimeScaleLogState, 2> s_scaleStates{};
            auto& state = s_scaleStates[isLeft ? 1 : 0];
            auto* vrScaleSetting = f4vr::vrScaleSetting();
            const float vrScale = vrScaleSetting ? vrScaleSetting->GetFloat() : -1.0f;
            const float collidableScale = collidableNode ? collidableNode->world.scale : -1.0f;
            auto changedEnough = [](float a, float b) { return std::fabs(a - b) > 0.01f; };

            if (!state.initialized || changedEnough(state.handScale, handWorldTransform.scale) ||
                changedEnough(state.collidableScale, collidableScale) || changedEnough(state.vrScale, vrScale)) {
                ROCK_LOG_DEBUG(Hand,
                    "Runtime scale {}: handScale={:.3f} collidableScale={:.3f} vrScale={:.3f} "
                    "previous=({:.3f},{:.3f},{:.3f})",
                    handName, handWorldTransform.scale, collidableScale, vrScale, state.handScale,
                    state.collidableScale, state.vrScale);
                state.initialized = true;
                state.handScale = handWorldTransform.scale;
                state.collidableScale = collidableScale;
                state.vrScale = vrScale;
            }
        }
    }

    bool Hand::validateSelectedGrab(RE::hknpWorld* world, const GrabSharedObjectContext& sharedContext,
        ValidatedGrabSelection& outSelection)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabSelectionValidation);
        outSelection = {};
        if (!hasSelection() || !world) {
            return false;
        }
        if (!hasCollisionBody()) {
            return false;
        }

        const auto& selection = _currentSelection;
        outSelection.retainedRef = selection.retainedRef;
        if (!outSelection.retainedRef || outSelection.retainedRef.get() != selection.refr ||
            selection.bodyId.value == INVALID_BODY_ID) {
            return false;
        }
        if (selection.refr->IsDeleted() || selection.refr->IsDisabled()) {
            return false;
        }

        if (!grab_target::canUseRockActiveGrab(selection.targetKind)) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand GRAB blocked: targetKind={} formID={:08X}; actor targets are not normal ROCK physical grabs",
                handName(), grab_target::name(selection.targetKind), selection.refr ? selection.refr->GetFormID() : 0);
            clearGrabExternalHandWorldTransform(_isLeft);
            return false;
        }

        outSelection.bodyId = selection.bodyId;
        outSelection.rootNode = selection.refr->Get3D();
        if (!outSelection.rootNode) {
            ROCK_LOG_WARN(Hand, "{} hand GRAB failed: selected ref has no 3D root", handName());
            return false;
        }

        auto* ownerCell = selection.refr->GetParentCell();
        outSelection.bhkWorld = ownerCell ? ownerCell->GetbhkWorld() : nullptr;
        if (!outSelection.bhkWorld) {
            ROCK_LOG_WARN(Hand, "{} hand GRAB failed: selected ref has no bhkWorld for object-tree scan", handName());
            return false;
        }

        if (!havok_runtime::getBody(world, outSelection.bodyId)) {
            ROCK_LOG_WARN(Hand, "{} hand GRAB failed: selected body no longer readable bodyId={}", handName(),
                outSelection.bodyId.value);
            return false;
        }

        auto* baseObj = selection.refr->GetObjectReference();
        outSelection.baseObject = baseObj;
        const bool selectedObjectIsCar = fo4vr::isExplodableCar(baseObj);
        const auto carGrabDecision = car_interaction_policy::evaluateGrab(car_interaction_policy::GrabPolicyInput{
            .targetIsCar = selectedObjectIsCar,
            .playerInPowerArmor = selectedObjectIsCar && fo4vr::isInPowerArmor(),
        });
        if (!carGrabDecision.allowed) {
            ROCK_LOG_DEBUG(Hand, "{} hand GRAB blocked: formID={:08X} reason={}", handName(),
                selection.refr->GetFormID(), carGrabDecision.reason);
            clearGrabExternalHandWorldTransform(_isLeft);
            return false;
        }

        if (outSelection.baseObject) {
            const auto nameView = RE::TESFullName::GetFullName(*outSelection.baseObject, false);
            if (!nameView.empty()) {
                outSelection.objectName = std::string(nameView);
            }
        }

        outSelection.joiningPeerHeldObject = sharedContextMatchesSelection(sharedContext, selection);
        outSelection.grabbedFromPullCatch = pullCatchIntentMatchesSelection();
        outSelection.looseWeaponGrab = isLooseWeaponGrabTarget(selection);
        outSelection.handPocketOnlyGrab = grab_target::requiresHandPocketGrab(selection.targetKind);
        return true;
    }

    void Hand::prepareSelectedGrabBodies(RE::hknpWorld* world, const GrabSharedObjectContext& sharedContext,
        const ValidatedGrabSelection& selection, GrabBodyPreparation& outPreparation)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabBodyPreparation);
        outPreparation = {};
        const auto& selectedObject = _currentSelection;
        outPreparation.scanOptions = makeActiveGrabBodyScanOptions(selectedObject);
        const auto& scanOptions = outPreparation.scanOptions;
        if (grab_target::requiresNativeBodyOwnership(selectedObject.targetKind)) {
            outPreparation.beforePrepBodySet = object_physics_body_set::scanObjectPhysicsBodySet(selection.bhkWorld,
                world, selectedObject.refr, scanOptions);
            outPreparation.preparedBodySet = outPreparation.beforePrepBodySet;
            outPreparation.preparedBodySetPostPrepComplete = true;
            return;
        }

        outPreparation.consumedPullPrepLifecycle = !selection.joiningPeerHeldObject && selection.grabbedFromPullCatch &&
            consumePullPrepLifecycleForActiveGrab(selectedObject.refr, outPreparation.activeLifecycle);
        outPreparation.beforePrepScanCacheHit = tryUseGrabAcquisitionBeforePrepCache(selection.bhkWorld, world,
            selectedObject, scanOptions, outPreparation.beforePrepBodySet);
        if (!outPreparation.beforePrepScanCacheHit) {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabAcquisitionBodyScan);
            outPreparation.beforePrepBodySet = object_physics_body_set::scanObjectPhysicsBodySet(selection.bhkWorld,
                world, selectedObject.refr, scanOptions);
        }
        if (!selection.joiningPeerHeldObject && !outPreparation.consumedPullPrepLifecycle) {
            outPreparation.activeLifecycle.captureBeforeActivePrep(outPreparation.beforePrepBodySet);
        }

        if (!selection.joiningPeerHeldObject) {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabAcquisitionActivePrep);
            outPreparation.motionConverted = physics_recursive_wrappers::setMotionRecursive(selection.rootNode,
                physics_recursive_wrappers::MotionPreset::Dynamic, true, true, true);
            outPreparation.collisionEnabled =
                physics_recursive_wrappers::enableCollisionRecursive(selection.rootNode, true, true, true);
        }

        outPreparation.preparedScanCacheHit = selection.joiningPeerHeldObject;
        outPreparation.preparedBodySetPostPrepComplete = selection.joiningPeerHeldObject;
        if (selection.joiningPeerHeldObject) {
            outPreparation.preparedBodySet = outPreparation.beforePrepBodySet;
        } else {
            outPreparation.preparedScanCacheHit =
                tryBuildGrabAcquisitionPreparedBodySetFromCache(selection.bhkWorld, world, selectedObject, scanOptions,
                    outPreparation.preparedBodySet, outPreparation.preparedBodySetPostPrepComplete);
            if (!outPreparation.preparedScanCacheHit || outPreparation.preparedBodySet.acceptedCount() == 0) {
                outPreparation.preparedScanCacheHit = false;
                outPreparation.preparedBodySetPostPrepComplete = true;
                performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabAcquisitionBodyScan);
                outPreparation.preparedBodySet = object_physics_body_set::scanObjectPhysicsBodySet(selection.bhkWorld,
                    world, selectedObject.refr, scanOptions);
            }
        }

        if (!selection.joiningPeerHeldObject) {
            outPreparation.activeLifecycle.markPreparedBodies(outPreparation.preparedBodySet);
            if (outPreparation.preparedScanCacheHit && !outPreparation.preparedBodySetPostPrepComplete) {
                outPreparation.activeLifecycle.markIncompleteNativeScan();
            }
        } else if (sharedContext.peerActiveGrabLifecycle) {
            outPreparation.activeLifecycle = *sharedContext.peerActiveGrabLifecycle;
        }
    }

    bool Hand::prepareGrabProxyAuthority(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform,
        GrabProxyPreparation& outPreparation)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabProxyPreparation);
        outPreparation = {};
        outPreparation.handBodyWorldAtGrab = getLiveBodyWorldTransform(world, _handBody.getBodyId());
        outPreparation.proxyFrameWorldAtGrab = outPreparation.handBodyWorldAtGrab;
        if (!resolveGrabAuthorityProxyFrame(world, outPreparation.proxyFrameWorldAtGrab,
                outPreparation.proxyFrameSourceAtGrab, GrabAuthorityProxyFramePolicy::LivePalmOnly)) {
            return false;
        }

        outPreparation.grabAuthorityPivotAWorld = outPreparation.proxyFrameWorldAtGrab.translate;
        outPreparation.palmPocketPivotAWorld = outPreparation.proxyFrameWorldAtGrab.translate;
        outPreparation.palmPocketToProxyDeltaGameUnits =
            pointDistanceGameUnits(outPreparation.grabAuthorityPivotAWorld, outPreparation.palmPocketPivotAWorld);
        outPreparation.palmBasisDelta =
            computeGrabPalmBasisDelta(handWorldTransform, outPreparation.proxyFrameWorldAtGrab);
        outPreparation.grabPivotAForPrimaryChoice = outPreparation.palmPocketPivotAWorld;
        outPreparation.proxyAuthorityFrameWorldAtGrab =
            makeGeneratedProxyAuthorityRelationFrame(outPreparation.proxyFrameWorldAtGrab);
        outPreparation.hasPalmProxyFrameAtGrab = true;
        return true;
    }

    bool Hand::prepareGrabMeshCapture(const RE::NiTransform& handWorldTransform,
        const ValidatedGrabSelection& selection, const std::string& objectName, GrabMeshCaptureSetup& outSetup)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabMeshCapturePreparation);
        outSetup = {};
        const auto& selectedObject = _currentSelection;
        outSetup.collidableNode = selectedObject.hitNode ? selectedObject.hitNode : selection.rootNode;
        outSetup.meshSourceNode = selectedObject.visualNode ? selectedObject.visualNode : selection.rootNode;
        if (!outSetup.meshSourceNode) {
            outSetup.meshSourceNode = outSetup.collidableNode;
        }

        const auto captureRefresh = refreshGrabCaptureTransforms(selection.rootNode, outSetup.meshSourceNode,
            outSetup.collidableNode, !grab_target::requiresNativeBodyOwnership(selectedObject.targetKind));
        if (!captureRefresh.ok) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: capture transform refresh produced a non-finite node transform for '{}' "
                "formID={:08X}; root='{}' mesh='{}' collidable='{}'",
                handName(), objectName, selectedObject.refr ? selectedObject.refr->GetFormID() : 0,
                nodeDebugName(selection.rootNode), nodeDebugName(outSetup.meshSourceNode),
                nodeDebugName(outSetup.collidableNode));
            return false;
        }

        outSetup.objectWorldTransform = outSetup.collidableNode ? outSetup.collidableNode->world : handWorldTransform;
        if (!grab_three_phase::isFinite(outSetup.objectWorldTransform)) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: refreshed object transform is non-finite for '{}' formID={:08X}; collidable='{}'",
                handName(), objectName, selectedObject.refr ? selectedObject.refr->GetFormID() : 0,
                nodeDebugName(outSetup.collidableNode));
            return false;
        }
        return true;
    }

    void Hand::extractGrabMeshEvidence(RE::hknpWorld* world, RE::hknpBodyId objectBodyId, RE::NiAVObject* rootNode,
        RE::NiAVObject* collidableNode, RE::NiAVObject* meshSourceNode, bool handPocketOnlyGrab,
        bool surfaceEvidenceRequired, GrabMeshExtraction& outExtraction)
    {
        outExtraction.reset();
        outExtraction.meshSourceNode = meshSourceNode;
        // Taken or not, the probe's read is never kept past this grab.
        GrabSharedExtraction shared = std::move(_grabSharedExtraction);
        _grabSharedExtraction.clear();
        if (!meshSourceNode) {
            return;
        }

        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabMeshExtraction);
        const int meshExtractionDepth = (std::max)(1, g_rockConfig.rockObjectPhysicsTreeMaxDepth);
        const bool sharedRead = shared.valid && surfaceEvidenceRequired && shared.meshNode == meshSourceNode &&
            shared.depth == meshExtractionDepth && shared.handPocketOnlyGrab == handPocketOnlyGrab &&
            shared.frameIndex == runtime_state::currentFrame().frameIndex;
        std::array<RE::NiAVObject*, 3> attemptedRoots{};
        std::uint32_t attemptCount = 0;
        const char* extractionSource = "visual";
        attemptedRoots[attemptCount++] = meshSourceNode;
        const auto extract = [&](RE::NiAVObject* source) {
            if (surfaceEvidenceRequired) {
                extractAllSurfaceTriangles(source, outExtraction.meshTriangles, outExtraction.surfaceTriangles,
                    meshExtractionDepth, &outExtraction.stats, handPocketOnlyGrab);
                return;
            }
            // Authored placement needs no per-triangle contact/skin ownership
            // records. Keep the same vertex positions for release spin limiting.
            const auto before = outExtraction.meshTriangles.size();
            extractAllTriangles(source, outExtraction.meshTriangles, meshExtractionDepth, &outExtraction.stats);
            if (source && outExtraction.meshTriangles.size() == before) {
                RE::BSVisit::TraverseScenegraphGeometries(source, [&](RE::BSGeometry* geometry) {
                    if (auto* shape = geometry ? geometry->IsTriShape() : nullptr) {
                        extractAllTriangles(shape, outExtraction.meshTriangles, 1, &outExtraction.stats);
                    }
                    return RE::BSVisit::BSVisitControl::kContinue;
                });
            }
        };
        if (sharedRead) {
            outExtraction.meshTriangles = std::move(shared.meshTriangles);
            outExtraction.surfaceTriangles = std::move(shared.surfaceTriangles);
            outExtraction.stats = shared.stats;
            extractionSource = "visual-shared";
        } else {
            extract(meshSourceNode);
        }

        auto tryAlternateRoot = [&](RE::NiAVObject* candidateRoot, const char* sourceName) {
            if (!candidateRoot) {
                return false;
            }
            for (std::uint32_t i = 0; i < attemptCount; ++i) {
                if (attemptedRoots[i] == candidateRoot) {
                    return false;
                }
            }
            if (attemptCount < attemptedRoots.size()) {
                attemptedRoots[attemptCount] = candidateRoot;
            }
            ++attemptCount;
            const auto beforeTriangles = outExtraction.meshTriangles.size();
            extract(candidateRoot);
            if (outExtraction.meshTriangles.size() == beforeTriangles) {
                return false;
            }
            ROCK_LOG_DEBUG(Hand,
                "{} hand mesh extraction recovered from {} node: meshNode='{}' previousMeshNode='{}' ownerNode='{}' "
                "rootNode='{}' addedTris={} totalTris={}",
                handName(), sourceName, nodeDebugName(candidateRoot), nodeDebugName(outExtraction.meshSourceNode),
                nodeDebugName(collidableNode), nodeDebugName(rootNode),
                outExtraction.meshTriangles.size() - beforeTriangles, outExtraction.meshTriangles.size());
            outExtraction.meshSourceNode = candidateRoot;
            extractionSource = sourceName;
            return true;
        };

        if (outExtraction.meshTriangles.empty()) {
            if (!tryAlternateRoot(collidableNode, "owner")) {
                (void)tryAlternateRoot(rootNode, "root");
            }
        }

        ROCK_LOG_DEBUG(Hand,
            "{} hand mesh extraction: meshNode='{}' ownerNode='{}' rootNode='{}' shapes={} source={} attempts={} "
            "static={}/{} dynamic={}/{} skinned={}/{} dynamicSkinnedSkipped={} emptyShapes={} totalTris={}",
            handName(), nodeDebugName(outExtraction.meshSourceNode), nodeDebugName(collidableNode),
            nodeDebugName(rootNode), outExtraction.stats.visitedShapes, extractionSource, attemptCount,
            outExtraction.stats.staticShapes, outExtraction.stats.staticTriangles, outExtraction.stats.dynamicShapes,
            outExtraction.stats.dynamicTriangles, outExtraction.stats.skinnedShapes,
            outExtraction.stats.skinnedTriangles, outExtraction.stats.dynamicSkinnedSkipped,
            outExtraction.stats.emptyShapes, outExtraction.stats.totalTriangles());
        performance_profiler::observeValue(performance_profiler::ValueMetric::GrabMeshTriangles,
            outExtraction.stats.totalTriangles());
        performance_profiler::observeValue(performance_profiler::ValueMetric::GrabMeshStaticTriangles,
            outExtraction.stats.staticTriangles);
        performance_profiler::observeValue(performance_profiler::ValueMetric::GrabMeshDynamicTriangles,
            outExtraction.stats.dynamicTriangles);
        performance_profiler::observeValue(performance_profiler::ValueMetric::GrabMeshSkinnedTriangles,
            outExtraction.stats.skinnedTriangles);
        performance_profiler::observeValue(performance_profiler::ValueMetric::GrabMeshCaptureAttempts, attemptCount);
        performance_profiler::observeValue(performance_profiler::ValueMetric::GrabMeshPayloadBytes,
            outExtraction.meshTriangles.size() * sizeof(TriangleData) +
                outExtraction.surfaceTriangles.size() * sizeof(GrabSurfaceTriangleData));

        RE::BSTriShape* firstTriShape = outExtraction.meshSourceNode->IsTriShape();
        if (!firstTriShape) {
            if (auto* meshNode = outExtraction.meshSourceNode->IsNode()) {
                auto& children = meshNode->GetRuntimeData().children;
                const auto childCount = children.size();
                for (auto index = decltype(childCount){ 0 }; index < childCount; ++index) {
                    auto* child = children[index].get();
                    if (child && child->IsTriShape()) {
                        firstTriShape = child->IsTriShape();
                        break;
                    }
                }
            }
        }
        if (firstTriShape) {
            auto* triShapeBytes = reinterpret_cast<char*>(firstTriShape);
            const std::uint64_t vertexDescriptor =
                *reinterpret_cast<std::uint64_t*>(triShapeBytes + VROffset::vertexDesc);
            const std::uint32_t stride = static_cast<std::uint32_t>(vertexDescriptor & 0xF) * 4;
            const std::uint32_t positionOffset = static_cast<std::uint32_t>((vertexDescriptor >> 2) & 0x3C);
            const bool fullPrecision = ((vertexDescriptor >> 54) & 1) != 0;
            const std::uint8_t geometryType = *reinterpret_cast<std::uint8_t*>(triShapeBytes + 0x198);
            void* skinInstance = *reinterpret_cast<void**>(triShapeBytes + VROffset::skinInstance);
            ROCK_LOG_TRACE(MeshGrab,
                "VertexDiag '{}': vtxDesc=0x{:016X} stride={} posOffset={} fullPrec={} geomType={} skinned={}",
                firstTriShape->name.c_str() ? firstTriShape->name.c_str() : "(null)", vertexDescriptor, stride,
                positionOffset, fullPrecision ? 1 : 0, geometryType, skinInstance ? 1 : 0);
        }

        if (!outExtraction.meshTriangles.empty()) {
            const auto& triangle = outExtraction.meshTriangles.front();
            const float cx = (triangle.v0.x + triangle.v1.x + triangle.v2.x) / 3.0f;
            const float cy = (triangle.v0.y + triangle.v1.y + triangle.v2.y) / 3.0f;
            const float cz = (triangle.v0.z + triangle.v1.z + triangle.v2.z) / 3.0f;
            if (auto* liveBody = havok_runtime::getBody(world, objectBodyId)) {
                auto* bodyValues = reinterpret_cast<float*>(liveBody);
                const float scale = havokToGameScale();
                const float distance = std::sqrt((cx - bodyValues[12] * scale) * (cx - bodyValues[12] * scale) +
                    (cy - bodyValues[13] * scale) * (cy - bodyValues[13] * scale) +
                    (cz - bodyValues[14] * scale) * (cz - bodyValues[14] * scale));
                ROCK_LOG_TRACE(MeshGrab, "TRI[0] centroid=({:.1f},{:.1f},{:.1f}) distToBody={:.1f}gu", cx, cy, cz,
                    distance);
            }
        }
    }

    void Hand::resolveGrabSurfaceEvidence(const ValidatedGrabSelection& selection, const GrabProxyPreparation& proxy,
        const GrabMeshCaptureSetup& capture, const GrabMeshExtraction& mesh,
        GrabSurfaceEvidence& outEvidence)
    {
        performance_profiler::ScopedTimer timer(performance_profiler::Scope::GrabSurfaceResolution);
        const auto& sel = _currentSelection;
        auto* rootNode = selection.rootNode;
        auto* collidableNode = capture.collidableNode;
        auto* meshSourceNode = mesh.meshSourceNode;
        const auto& grabMeshTriangles = mesh.meshTriangles;
        const auto& grabSurfaceTriangles = mesh.surfaceTriangles;
        const auto& meshStats = mesh.stats;
        const bool handPocketOnlyGrab = selection.handPocketOnlyGrab;
        const auto& palmPocketPivotAWorld = proxy.palmPocketPivotAWorld;
        const auto& proxyAuthorityFrameWorldAtGrab = proxy.proxyAuthorityFrameWorldAtGrab;
        auto& grabGripPoint = outEvidence.gripPoint;
        auto& selectionToMeshDistanceGameUnits = outEvidence.selectionToMeshDistanceGameUnits;
        auto& meshGrabFound = outEvidence.meshGrabFound;
        auto& grabSurfaceHit = outEvidence.surfaceHit;
        auto& surfaceOwnerNode = outEvidence.surfaceOwnerNode;
        auto& grabPointMode = outEvidence.pointMode;
        auto& grabPointAuthoritySource = outEvidence.pointAuthoritySource;
        auto& grabFallbackReason = outEvidence.fallbackReason;
        grabGripPoint = sel.hasHitPoint ? sel.hitPointWorld : proxy.grabPivotAForPrimaryChoice;
        grabPointMode = "none";
        grabPointAuthoritySource = GrabPivotAuthoritySource::None;
        grabFallbackReason = meshSourceNode ? "noTriangles" : "noMeshSourceNode";
        /*
         * hknp selection identifies the object and body only. The grabbed point
         * is always on the rendered mesh: object collision often sits well off
         * the visible surface, and holding a collision point made grabs no
         * finger touched. No mesh point, no grab (the caller refuses).
         */
        if (sel.hasHitPoint && sel.hasHitNormal) {
            const GrabSurfaceHit collisionSurfaceHit = makeCollisionQueryGrabSurfaceHit(sel, collidableNode);
            if (collisionSurfaceHit.valid) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand COLLISION SELECTION HIT: body={} point=({:.1f},{:.1f},{:.1f}) "
                    "normal=({:.3f},{:.3f},{:.3f}) fraction={:.4f} shapeKey=0x{:08X} filter=0x{:08X} owner='{}'",
                    handName(), sel.bodyId.value, collisionSurfaceHit.position.x, collisionSurfaceHit.position.y,
                    collisionSurfaceHit.position.z, collisionSurfaceHit.normal.x, collisionSurfaceHit.normal.y,
                    collisionSurfaceHit.normal.z, collisionSurfaceHit.hitFraction, collisionSurfaceHit.shapeKey,
                    collisionSurfaceHit.shapeCollisionFilterInfo, nodeDebugName(collisionSurfaceHit.sourceNode));
            }
        }

        if (meshSourceNode) {
            const bool closeGrabNeedsPalmPocketMeshAuthority =
                !grabSurfaceTriangles.empty() && (handPocketOnlyGrab || !sel.isFarSelection);
            if (closeGrabNeedsPalmPocketMeshAuthority) {
                /*
                     * One grab point, chosen in order: the mesh point nearest
                     * the palm centre (close grabs), the mesh point the palm
                     * faces at a pull catch, the mesh point at the selection hit.
                     */
                const RE::NiPoint3 grabPivotAWorld = palmPocketPivotAWorld;
                const RE::NiPoint3 palmDir = computePalmNormalFromHandBasis(proxyAuthorityFrameWorldAtGrab, _isLeft);
                const auto closePocket = grab_three_phase::buildGrabPocketFrameWithPalmCenter(
                    proxyAuthorityFrameWorldAtGrab, _isLeft, grabPivotAWorld, g_rockConfig.rockGrabPocketDepthGameUnits,
                    g_rockConfig.rockGrabPocketRadiusGameUnits);
                const RE::NiPoint3 pocketAuthorityPoint =
                    closePocket.valid ? closePocket.palmCenterWorld : grabPivotAWorld;
                const RE::NiPoint3 pocketAuthorityNormal = closePocket.valid ? closePocket.palmNormalWorld : palmDir;
                // A hand reach (grabSelectedObject) searches the whole close reach.
                const float palmPocketSnapDistance = (std::max)({ g_rockConfig.rockGrabPocketRadiusGameUnits,
                    g_rockConfig.rockGrabAlignmentMaxSelectionToMeshDistance,
                    g_rockConfig.rockGrabTouchAcquireDistanceGameUnits,
                    selection.handReach ? g_rockConfig.rockGrabNearConvergeDistanceGameUnits : 0.0f });

                if (findClosestGrabSurfaceHitToPointPositionOnly(grabSurfaceTriangles, pocketAuthorityPoint,
                        pocketAuthorityNormal, palmPocketSnapDistance, grabSurfaceHit, &mesh.queryIndex)) {
                    grabGripPoint = grabSurfaceHit.position;
                    selectionToMeshDistanceGameUnits = sel.hasHitPoint
                        ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint)
                        : std::numeric_limits<float>::max();
                    grabSurfaceHit.hasSelectionHit = sel.hasHitPoint;
                    grabSurfaceHit.selectionToMeshDistanceGameUnits = selectionToMeshDistanceGameUnits;
                    grabSurfaceHit.pivotToSurfaceDistanceGameUnits =
                        pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                    grabSurfaceHit.shapeKey = sel.hitShapeKey;
                    grabSurfaceHit.shapeCollisionFilterInfo = sel.hitShapeCollisionFilterInfo;
                    grabSurfaceHit.hitFraction = sel.hitFraction;
                    grabSurfaceHit.hasShapeKey = sel.hasHitShapeKey;
                    surfaceOwnerNode = grabSurfaceHit.sourceNode;
                    meshGrabFound = true;
                    grabPointMode = "palmPocketMeshSurface";
                    grabPointAuthoritySource = GrabPivotAuthoritySource::PalmPocketMeshPoint;
                    grabFallbackReason = "closePalmPocketMeshAuthority";
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand MESH GRAB: mode={} tris={} closest=({:.1f},{:.1f},{:.1f}) tri={} source={} owner='{}' "
                        "shape='{}' "
                        "pocket=({:.1f},{:.1f},{:.1f}) snapLimit={:.1f} selectionHit={} selectionDelta={:.1f}gu "
                        "surfaceAlong={:.1f} surfaceLateral={:.1f}",
                        handName(), grabPointMode, grabMeshTriangles.size(), grabSurfaceHit.position.x,
                        grabSurfaceHit.position.y, grabSurfaceHit.position.z, grabSurfaceHit.triangleIndex,
                        grabSurfaceSourceKindName(grabSurfaceHit.sourceKind), nodeDebugName(grabSurfaceHit.sourceNode),
                        nodeDebugName(grabSurfaceHit.sourceShape), pocketAuthorityPoint.x, pocketAuthorityPoint.y,
                        pocketAuthorityPoint.z, palmPocketSnapDistance, sel.hasHitPoint ? "yes" : "no",
                        sel.hasHitPoint ? selectionToMeshDistanceGameUnits : -1.0f,
                        grabSurfaceHit.signedAlongPalmDistanceGameUnits, grabSurfaceHit.lateralPalmDistanceGameUnits);
                }
            }

            /*
                 * A pull is caught about 27 units from the hand, beyond the palm
                 * pocket's snap range, and its stored hit point was picked by the
                 * selection ray at range: the face that looked at the player
                 * then. The seat keeps the object's rotation, so holding that
                 * point leaves its face sloping away from the palm (a visible gap)
                 * and the fingers close on air. Take the point the palm faces at
                 * the catch instead: the nearest front-facing mesh point to the
                 * palm line, on the object as it arrived.
                 */
            if (!meshGrabFound && (selection.grabbedFromPullCatch || selection.reachArrival) &&
                !grabSurfaceTriangles.empty()) {
                const RE::NiPoint3 palmDir = computePalmNormalFromHandBasis(proxyAuthorityFrameWorldAtGrab, _isLeft);
                if (findClosestGrabSurfaceHit(grabSurfaceTriangles, palmPocketPivotAWorld, palmDir,
                        g_rockConfig.rockGrabLateralWeight, g_rockConfig.rockGrabDirectionalWeight, grabSurfaceHit,
                        g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits)) {
                    grabGripPoint = grabSurfaceHit.position;
                    selectionToMeshDistanceGameUnits = sel.hasHitPoint
                        ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint)
                        : std::numeric_limits<float>::max();
                    grabSurfaceHit.hasSelectionHit = sel.hasHitPoint;
                    grabSurfaceHit.selectionToMeshDistanceGameUnits = selectionToMeshDistanceGameUnits;
                    grabSurfaceHit.pivotToSurfaceDistanceGameUnits =
                        pointDistanceGameUnits(palmPocketPivotAWorld, grabGripPoint);
                    surfaceOwnerNode = grabSurfaceHit.sourceNode;
                    meshGrabFound = true;
                    grabPointMode = "pullCatchPalmLineMeshSurface";
                    grabPointAuthoritySource = GrabPivotAuthoritySource::PalmPocketMeshPoint;
                    grabFallbackReason = "pullCatchPalmLine";
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand MESH GRAB: mode={} tris={} closest=({:.1f},{:.1f},{:.1f}) tri={} source={} owner='{}' "
                        "selectionDelta={:.1f}gu surfaceAlong={:.1f} surfaceLateral={:.1f} normalDotPalm={:.2f}",
                        handName(), grabPointMode, grabSurfaceTriangles.size(), grabSurfaceHit.position.x,
                        grabSurfaceHit.position.y, grabSurfaceHit.position.z, grabSurfaceHit.triangleIndex,
                        grabSurfaceSourceKindName(grabSurfaceHit.sourceKind), nodeDebugName(grabSurfaceHit.sourceNode),
                        sel.hasHitPoint ? selectionToMeshDistanceGameUnits : -1.0f,
                        grabSurfaceHit.signedAlongPalmDistanceGameUnits, grabSurfaceHit.lateralPalmDistanceGameUnits,
                        grabSurfaceHit.normal.x * palmDir.x + grabSurfaceHit.normal.y * palmDir.y +
                            grabSurfaceHit.normal.z * palmDir.z);
                }
            }

            if (!meshGrabFound && !grabSurfaceTriangles.empty() && sel.hasHitPoint) {
                const RE::NiPoint3 grabPivotAWorld = palmPocketPivotAWorld;
                RE::NiPoint3 palmDir = computePalmNormalFromHandBasis(proxyAuthorityFrameWorldAtGrab, _isLeft);
                const RE::NiPoint3 expectedNormal =
                    sel.hasHitNormal && lengthSquared(sel.hitNormalWorld) > 0.0f ? sel.hitNormalWorld : palmDir;

                if (findClosestGrabSurfaceHitToPoint(grabSurfaceTriangles, sel.hitPointWorld, expectedNormal,
                        g_rockConfig.rockGrabAlignmentMaxSelectionToMeshDistance,
                        g_rockConfig.rockGrabContactPatchMaxNormalAngleDegrees, grabSurfaceHit, &mesh.queryIndex)) {
                    grabGripPoint = grabSurfaceHit.position;
                    selectionToMeshDistanceGameUnits = pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint);
                    grabSurfaceHit.hasSelectionHit = true;
                    grabSurfaceHit.selectionToMeshDistanceGameUnits = selectionToMeshDistanceGameUnits;
                    grabSurfaceHit.signedAlongPalmDistanceGameUnits = sel.signedAlongDistance;
                    grabSurfaceHit.lateralPalmDistanceGameUnits = sel.lateralDistance;
                    grabSurfaceHit.shapeKey = sel.hitShapeKey;
                    grabSurfaceHit.shapeCollisionFilterInfo = sel.hitShapeCollisionFilterInfo;
                    grabSurfaceHit.hitFraction = sel.hitFraction;
                    grabSurfaceHit.hasShapeKey = sel.hasHitShapeKey;
                    surfaceOwnerNode = grabSurfaceHit.sourceNode;
                    meshGrabFound = true;
                    grabPointMode = "selectionHitMeshSnap";
                    grabPointAuthoritySource = GrabPivotAuthoritySource::SelectionHitMeshSnap;
                    grabFallbackReason = "selectionHitMeshSnap";
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand MESH GRAB: mode={} tris={} closest=({:.1f},{:.1f},{:.1f}) tri={} source={} owner='{}' "
                        "shape='{}' selectionDelta={:.1f}gu surfaceAlong={:.1f} surfaceLateral={:.1f}",
                        handName(), grabPointMode, grabMeshTriangles.size(), grabSurfaceHit.position.x,
                        grabSurfaceHit.position.y, grabSurfaceHit.position.z, grabSurfaceHit.triangleIndex,
                        grabSurfaceSourceKindName(grabSurfaceHit.sourceKind), nodeDebugName(grabSurfaceHit.sourceNode),
                        nodeDebugName(grabSurfaceHit.sourceShape), selectionToMeshDistanceGameUnits,
                        grabSurfaceHit.signedAlongPalmDistanceGameUnits, grabSurfaceHit.lateralPalmDistanceGameUnits);
                }
            }

            if (!meshGrabFound) {
                grabFallbackReason = grabSurfaceTriangles.empty() ? "noTriangles" : "noMeshNearPalmOrSelection";
            }
        }
        if (!meshGrabFound) {
            ROCK_LOG_SAMPLE_WARN(Hand, g_rockConfig.rockLogSampleMilliseconds,
                "{} hand GRAB POINT unavailable: no mesh surface near the palm reason={} meshNode='{}' ownerNode='{}' "
                "rootNode='{}' shapes={} static={}/{} dynamic={}/{} skinned={}/{} dynamicSkinnedSkipped={} "
                "emptyShapes={}",
                handName(), grabFallbackReason, nodeDebugName(meshSourceNode), nodeDebugName(collidableNode),
                nodeDebugName(rootNode), meshStats.visitedShapes, meshStats.staticShapes, meshStats.staticTriangles,
                meshStats.dynamicShapes, meshStats.dynamicTriangles, meshStats.skinnedShapes,
                meshStats.skinnedTriangles, meshStats.dynamicSkinnedSkipped, meshStats.emptyShapes);
        }
    }

    void Hand::resolveGrabBodyAndContactPolicy(const ValidatedGrabSelection& selection,
        const object_physics_body_set::ObjectPhysicsBodySet& beforePrepBodySet,
        const object_physics_body_set::ObjectPhysicsBodySet& preparedBodySet,
        const active_grab_body_lifecycle::BodyLifecycleSnapshot& activeLifecycle, const GrabProxyPreparation& proxy,
        GrabSurfaceEvidence& surface, GrabBodyResolution& outResolution)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabBodyResolution);
        outResolution = {};
        const auto& selectedObject = _currentSelection;
        outResolution.primaryChoiceTarget =
            (surface.meshGrabFound ||
                surface.pointAuthoritySource == GrabPivotAuthoritySource::LooseWeaponPrimaryAttach)
            ? surface.gripPoint
            : (selectedObject.hasHitPoint ? selectedObject.hitPointWorld : proxy.grabPivotAForPrimaryChoice);
        const auto nearestPrimaryChoice = preparedBodySet.choosePrimaryBody(object_physics_body_set::INVALID_BODY_ID,
            object_physics_body_set::PurePoint3{ outResolution.primaryChoiceTarget });
        const bool ragdollGrab = grab_target::requiresNativeBodyOwnership(selectedObject.targetKind);
        const auto* surfaceOwnerRecord = ragdollGrab
            ? preparedBodySet.findAcceptedRagdollOwner(surface.surfaceOwnerNode)
            : preparedBodySet.findAcceptedRecordByOwnerNode(surface.surfaceOwnerNode);
        const auto skinnedResolution = skinned_body_resolver::resolvePrimaryBody(skinned_body_resolver::ResolutionInput{
            .targetKind = selectedObject.targetKind,
            .surfaceOwnerBodyId =
                surfaceOwnerRecord ? surfaceOwnerRecord->bodyId : object_physics_body_set::INVALID_BODY_ID,
            .selectedBodyId = selectedObject.bodyId.value,
            .nearestBodyId = nearestPrimaryChoice.bodyId,
            .surfaceOwnerUsable = surfaceOwnerRecord != nullptr,
            .selectedUsable = preparedBodySet.containsAcceptedBody(selectedObject.bodyId.value),
            .nearestUsable = nearestPrimaryChoice.bodyId != object_physics_body_set::INVALID_BODY_ID,
            .surfaceIsSkinned =
                surface.surfaceHit.valid && surface.surfaceHit.sourceKind == GrabSurfaceSourceKind::Skinned,
            .hasSkinInfluences = surface.surfaceHit.valid && surface.surfaceHit.hasSkinInfluences,
        });
        outResolution.primaryChoice.bodyId = skinnedResolution.bodyId;
        outResolution.primaryChoice.reason =
            skinnedResolution.source == skinned_body_resolver::ResolutionSource::WeightedSkinOwner ||
                skinnedResolution.source == skinned_body_resolver::ResolutionSource::TriangleOwner
            ? object_physics_body_set::PrimaryBodyChoiceReason::SurfaceOwnerAccepted
            : (skinnedResolution.source == skinned_body_resolver::ResolutionSource::SelectedBody
                      ? object_physics_body_set::PrimaryBodyChoiceReason::PreferredHitAccepted
                      : (skinnedResolution.source == skinned_body_resolver::ResolutionSource::NearestAccepted
                                ? object_physics_body_set::PrimaryBodyChoiceReason::NearestAcceptedFallback
                                : object_physics_body_set::PrimaryBodyChoiceReason::NoAcceptedBody));

        // Count-based loose ammo is held by its visible branch body. The root
        // body is a geometry-less cube the selection ray often hits first; it
        // is carried with the branch instead (restoreCountBasedAmmoAssembly).
        bool carriedAmmoAssembly = false;
        if (!ragdollGrab && _ammoCarry.active() && preparedBodySet.containsAcceptedBody(_ammoCarry.primaryBodyId) &&
            preparedBodySet.containsAcceptedBody(_ammoCarry.anchorBodyId)) {
            carriedAmmoAssembly = true;
            if (outResolution.primaryChoice.bodyId != _ammoCarry.primaryBodyId) {
                ROCK_LOG_DEBUG(Hand, "{} hand GRAB count-based ammo: primary body {} -> visible branch body {}",
                    handName(), outResolution.primaryChoice.bodyId, _ammoCarry.primaryBodyId);
                outResolution.primaryChoice.bodyId = _ammoCarry.primaryBodyId;
                outResolution.primaryChoice.reason =
                    object_physics_body_set::PrimaryBodyChoiceReason::SurfaceOwnerAccepted;
            }
        }

        if (surface.surfaceHit.valid) {
            const bool handPocketPositionOnlySkinnedSurface = selection.handPocketOnlyGrab &&
                surface.surfaceHit.sourceKind == GrabSurfaceSourceKind::Skinned &&
                !surface.surfaceHit.hasSkinInfluences;
            if (handPocketPositionOnlySkinnedSurface) {
                outResolution.surfaceOwnerMatchesResolvedBody =
                    outResolution.primaryChoice.bodyId == selectedObject.bodyId.value &&
                    preparedBodySet.containsAcceptedBody(selectedObject.bodyId.value);
            } else {
                outResolution.surfaceOwnerMatchesResolvedBody =
                    (surfaceOwnerRecord && surfaceOwnerRecord->bodyId == outResolution.primaryChoice.bodyId) ||
                    acceptsSelectedOwnerlessVisualMesh(selectedObject, preparedBodySet,
                        outResolution.primaryChoice.bodyId, surface.surfaceOwnerNode, surfaceOwnerRecord);
                if (outResolution.surfaceOwnerMatchesResolvedBody && !surfaceOwnerRecord &&
                    preparedBodySet.records.size() == 1) {
                    performance_profiler::addCounter(performance_profiler::Counter::GrabSingleBodyVisualOwnerAccepted);
                }
            }
            surface.surfaceHit.resolvedOwnerMatchesBody = outResolution.surfaceOwnerMatchesResolvedBody;
        }

        ROCK_LOG_DEBUG(Hand,
            "{} hand GRAB BODY RESOLUTION: selectedBody={} resolvedBody={} reason={} resolver={} resolverReason={} "
            "skin={} sourceNode='{}' sourceKind={} ownerMatch={} target=({:.1f},{:.1f},{:.1f})",
            handName(), selectedObject.bodyId.value, outResolution.primaryChoice.bodyId,
            primaryBodyChoiceReasonName(outResolution.primaryChoice.reason),
            skinned_body_resolver::sourceName(skinnedResolution.source), skinnedResolution.reason,
            skinnedResolution.usedSkinInfluences
                ? "weighted"
                : (surface.surfaceHit.valid && surface.surfaceHit.sourceKind == GrabSurfaceSourceKind::Skinned
                          ? "positionOnly"
                          : "no"),
            nodeDebugName(surface.surfaceOwnerNode),
            surface.surfaceHit.valid ? grabSurfaceSourceKindName(surface.surfaceHit.sourceKind) : "fallback",
            outResolution.surfaceOwnerMatchesResolvedBody ? "yes" : "no", outResolution.primaryChoiceTarget.x,
            outResolution.primaryChoiceTarget.y, outResolution.primaryChoiceTarget.z);

        if (ragdollGrab && (!surface.meshGrabFound || !surface.surfaceHit.valid || !surfaceOwnerRecord)) {
            outResolution.primaryChoice.bodyId = INVALID_BODY_ID;
            outResolution.surfaceOwnerMatchesResolvedBody = false;
            ROCK_LOG_WARN(Hand, "{} ragdoll grab rejected: no posed surface with exact limb owner selected={}",
                handName(), selectedObject.bodyId.value);
            return;
        }
        if (outResolution.primaryChoice.bodyId != INVALID_BODY_ID) {
            if (!ragdollGrab) {
                outResolution.mechanicalScope = mechanical_connected_body_set::buildFromPreparedBodySet(
                    beforePrepBodySet, preparedBodySet, outResolution.primaryChoice.bodyId, selectedObject.targetKind,
                    activeLifecycle.hasIncompleteNativeScan(), carriedAmmoAssembly);
                outResolution.relaxedArticulatedAuthority = outResolution.mechanicalScope.strictPocketAuthorityRelaxed;
            }
            if (ragdollGrab) {
                const auto component = ragdoll::readComponent(havok_runtime::getHknpWorldFromBhk(selection.bhkWorld),
                    outResolution.primaryChoice.bodyId);
                if (!component.valid || component.count > MAX_HELD_BODIES) {
                    ROCK_LOG_WARN(Hand, "{} ragdoll grab rejected: topology stage={} body={}", handName(),
                        component.reason, outResolution.primaryChoice.bodyId);
                    outResolution.primaryChoice.bodyId = INVALID_BODY_ID;
                    return;
                }
                auto& scope = outResolution.mechanicalScope;
                scope.primaryBodyId = outResolution.primaryChoice.bodyId;
                for (std::size_t i = 0; i < component.count; ++i) {
                    scope.committedBodyIds.push_back(component.bodies[i].body.id);
                    bool uniqueMotion = true;
                    for (std::size_t j = 0; j < i; ++j) {
                        uniqueMotion =
                            uniqueMotion && component.bodies[j].body.motion != component.bodies[i].body.motion;
                    }
                    if (uniqueMotion)
                        ++scope.uniqueMotionCount;
                }
                scope.acceptedBodyCount = static_cast<std::uint32_t>(component.count);
                scope.targetPrefersMechanical = true;
                scope.strictPocketAuthorityRelaxed = true;
                scope.kind = component.fixedAttached ? mechanical_connected_body_set::ScopeKind::FixedAttached
                                                     : mechanical_connected_body_set::ScopeKind::ArticulatedMechanical;
                scope.driveDecision.mode = component.fixedAttached
                    ? held_object_drive_policy::HeldBodySetDriveMode::FixedAttached
                    : held_object_drive_policy::HeldBodySetDriveMode::ComplexArticulated;
                scope.driveDecision.reason = component.reason;
                scope.driveDecision.includeConnectedLinearVelocity = false;
                scope.driveDecision.includeConnectedAngularVelocity = false;
                // A living actor's motors hold its other limbs to the animation:
                // drive with the held limb's mass only, never drag the whole body.
                scope.driveDecision.includeConnectedMass = !component.fixedAttached &&
                    selectedObject.extensionSelection.kind != api::native_interaction::SelectionKind::DirectLiveLimb;
                scope.reason = component.reason;
                outResolution.surfaceOwnerMatchesResolvedBody = true;
                surface.surfaceHit.resolvedOwnerMatchesBody = true;
                // Corpse support may be position-only, but its posed surface
                // and native body identity above are mandatory.
                outResolution.relaxedArticulatedAuthority = true;
            }
        }
    }

    bool Hand::captureResolvedGrabBody(RE::hknpWorld* world, const GrabMeshCaptureSetup& meshCapture,
        const GrabBodyResolution& resolution, const object_physics_body_set::ObjectPhysicsBodySet& beforePrepBodySet,
        const std::vector<TriangleData>& meshTriangles, const RE::NiPointer<RE::TESObjectREFR>& selectedRef,
        std::uint16_t selectedOriginalMotionPropsId, const std::string& objectName, ResolvedGrabBodyCapture& outCapture,
        std::vector<GrabLocalTriangle>& outLocalMeshTriangles)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabResolvedBodyCapture);
        outCapture = {};
        outLocalMeshTriangles.clear();
        outCapture.bodyId = RE::hknpBodyId{ resolution.primaryChoice.bodyId };
        outCapture.collidableNode = meshCapture.collidableNode;
        outCapture.objectWorldTransform = meshCapture.objectWorldTransform;

        auto* preparedBody = havok_runtime::getBody(world, outCapture.bodyId);
        if (!preparedBody) {
            ROCK_LOG_ERROR(Hand, "{} grabSelectedObject: prepared primary body {} is not readable after object prep",
                handName(), outCapture.bodyId.value);
            return false;
        }

        const auto& selectedObject = _currentSelection;
        _savedObjectState.bodyId = outCapture.bodyId;
        _savedObjectState.setReference(selectedRef);
        _savedObjectState.targetKind = selectedObject.targetKind;
        _savedObjectState.liveLimb =
            selectedObject.extensionSelection.kind == api::native_interaction::SelectionKind::DirectLiveLimb;
        _savedObjectState.originalFilterInfo = preparedBody->collisionFilterInfo;
        _savedObjectState.originalMotionPropsId = selectedOriginalMotionPropsId;
        if (const auto* originalPrimaryRecord = beforePrepBodySet.findRecord(outCapture.bodyId.value)) {
            _savedObjectState.originalMotionPropsId = originalPrimaryRecord->motionPropertiesId;
        }

        auto* ownerCell = selectedObject.refr ? selectedObject.refr->GetParentCell() : nullptr;
        auto* ownerWorld = ownerCell ? ownerCell->GetbhkWorld() : nullptr;
        auto* bodyCollisionObject =
            ownerWorld ? RE::bhkNPCollisionObject::Getbhk(ownerWorld, outCapture.bodyId) : nullptr;
        if (auto* resolvedOwnerNode = bodyCollisionObject ? bodyCollisionObject->sceneObject : nullptr) {
            GrabCaptureTransformRefreshResult ownerRefresh{};
            refreshGrabCaptureNodeTransform(ownerRefresh, "resolvedOwner", resolvedOwnerNode,
                !grab_target::requiresNativeBodyOwnership(selectedObject.targetKind));
            if (!ownerRefresh.ok) {
                ROCK_LOG_WARN(Hand,
                    "{} hand GRAB failed: resolved owner transform refresh produced a non-finite node transform for "
                    "'{}' formID={:08X}; owner='{}'",
                    handName(), objectName, selectedObject.refr ? selectedObject.refr->GetFormID() : 0,
                    nodeDebugName(resolvedOwnerNode));
                return false;
            }
            outCapture.collidableNode = resolvedOwnerNode;
            outCapture.objectWorldTransform = resolvedOwnerNode->world;
            if (!grab_three_phase::isFinite(outCapture.objectWorldTransform)) {
                ROCK_LOG_WARN(Hand,
                    "{} hand GRAB failed: resolved owner transform is non-finite for '{}' formID={:08X}; owner='{}'",
                    handName(), objectName, selectedObject.refr ? selectedObject.refr->GetFormID() : 0,
                    nodeDebugName(resolvedOwnerNode));
                return false;
            }
        }

        if (!meshTriangles.empty()) {
            cacheTrianglesInLocalSpace(meshTriangles, outCapture.objectWorldTransform, outLocalMeshTriangles);
        }
        return true;
    }

    void Hand::resolveGrabPivotEvidence(const GrabProxyPreparation& proxy, const ResolvedGrabBodyCapture& bodyCapture,
        const GrabSurfaceEvidence& surface, GrabPivotEvidence& outEvidence)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabPivotEvidence);
        outEvidence = {};
        const auto& sel = _currentSelection;
        /*
         * The surface resolver already chose the single grab point. This stage
         * only records the hand contacts touching the object (they decide
         * whether a pulled object counts as touching on arrival) and describes
         * that point for the seat and held drive.
         */
        outEvidence.semanticContacts = collectFreshSemanticContactsForBodyWithinSeconds(bodyCapture.bodyId.value,
            (std::max)(0.0f, g_rockConfig.rockGrabOppositionContactMaxAgeSeconds));
        const auto pocket = grab_three_phase::buildGrabPocketFrameWithPalmCenter(proxy.proxyAuthorityFrameWorldAtGrab,
            _isLeft, proxy.palmPocketPivotAWorld, g_rockConfig.rockGrabPocketDepthGameUnits,
            g_rockConfig.rockGrabPocketRadiusGameUnits);
        outEvidence.authoritySource = surface.pointAuthoritySource;
        outEvidence.authorityNormalTrusted = surface.surfaceHit.valid;
        outEvidence.authorityPositionConfidence = surface.surfaceHit.valid ? 1.0f : 0.0f;
        outEvidence.authorityPocketDistanceGameUnits = pocket.valid
            ? pointDistanceGameUnits(surface.gripPoint, pocket.palmCenterWorld)
            : std::numeric_limits<float>::max();
        outEvidence.authoritySelectionDeltaGameUnits = sel.hasHitPoint
            ? pointDistanceGameUnits(sel.hitPointWorld, surface.gripPoint)
            : std::numeric_limits<float>::max();
    }

    void Hand::beginResolvedGrabCommit(const GrabCommitPreparationInput& input)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabCommitPreparation);
        const auto& handWorldTransform = *input.handWorldTransform;
        auto* collidableNode = input.collidableNode;
        auto objectBodyId = input.objectBodyId;
        const auto& objectName = *input.objectName;
        const auto& mechanicalScope = *input.mechanicalScope;
        const auto& sharedContext = *input.sharedContext;
        const auto& preparedBodySet = *input.preparedBodySet;
        const bool joiningPeerHeldObject = input.joiningPeerHeldObject;
        logRuntimeScaleIfChanged(_isLeft, handName(), handWorldTransform, collidableNode);
        ROCK_LOG_INFO(Hand, "{} hand GRAB: '{}' formID={:08X} bodyId={}", handName(), objectName,
            _currentSelection.refr->GetFormID(), objectBodyId.value);

        bool adoptedPeerHeldBodySet = false;
        _heldBodyIds = grab_target::requiresNativeBodyOwnership(_currentSelection.targetKind)
            ? held_object_body_set_policy::makePrimaryFirstUniqueBodyList(objectBodyId.value,
                  mechanicalScope.committedBodyIds)
            : buildCommittedHeldBodyIds(objectBodyId.value, mechanicalScope.committedBodyIds, sharedContext,
                  adoptedPeerHeldBodySet);
        if (_heldBodyIds.empty()) {
            _heldBodyIds.push_back(objectBodyId.value);
        }
        _heldDriveDecision = mechanicalScope.driveDecision;
        if (adoptedPeerHeldBodySet) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand SHARED held body set adopted: primaryBody={} peerBodies={} committedBodies={} scanAccepted={} "
                "mechanicalKind={} driveMode={} driveReason={}",
                handName(), objectBodyId.value,
                sharedContext.peerHeldBodyIds ? sharedContext.peerHeldBodyIds->size() : 0, _heldBodyIds.size(),
                preparedBodySet.acceptedCount(), mechanical_connected_body_set::scopeKindName(mechanicalScope.kind),
                held_object_drive_policy::modeName(_heldDriveDecision.mode), _heldDriveDecision.reason);
        }

        if (auto* player = RE::PlayerCharacter::GetSingleton(); player && !joiningPeerHeldObject) {
            nativeVRGrabDrop(player, 0);
            nativeVRGrabDrop(player, 1);
        }

        _grabStartTime = 0.0f;
        _heldObjectUpdateFrame = 0;
        _grabConvergeStableInsidePocketSeconds = 0.0f;
        _grabConvergePreviousGripErrorGameUnits = std::numeric_limits<float>::max();
        _grabTransitPlacementElapsedSeconds = 0.0f;
    }

}
