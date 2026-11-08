// Grab seat: captures the grabbed body's frame and resolves where the object
// sits in the palm (resolveGrabSeatCapture), the frame the grab constraint
// and finger pose are built from.
#include "physics-interaction/hand/HandGrabInternal.h"

namespace rock
{

    namespace
    {
        struct LooseWeaponPrimaryAttachFrame
        {
            bool valid = false;
            bool supportGrip = false;
            AuthoredWeaponGripPose pose{};
            RE::NiTransform desiredRootWorld{};
            bool sourceVisible = false;
            RE::NiTransform desiredObjectWorld{};
            RE::NiTransform desiredBodyWorld{};
            RE::NiPoint3 gripPointWorld{};
            const char* reason = "notEvaluated";
        };

        // Cache-only disk lookup; false when this object+hand has no saved
        // offset (normal - most objects never had one saved).
        bool tryLoadSavedGrabOffsetHandOffset(RE::TESObjectREFR* refr, bool isLeft, saved_grab_offset::HandOffset& out)
        {
            if (!refr) {
                return false;
            }
            auto* baseForm = refr->GetObjectReference();
            if (!baseForm) {
                return false;
            }
            const auto formRef = saved_grab_offset::formRefFromRuntimeId(baseForm->GetFormID());
            saved_grab_offset::SavedGrabOffsetFile file{};
            if (!formRef.empty() && saved_grab_offset::load(formRef, file, nullptr)) {
                const auto& handOffset = isLeft ? file.left : file.right;
                if (handOffset.present) {
                    if (handOffset.hasExactFingerRotations && fo4vr::isInPowerArmor()) {
                        ROCK_LOG_WARN(Hand,
                            "{} saved exact grab pose skipped: normal-hand rig does not match power armor",
                            isLeft ? "Left" : "Right");
                        return false;
                    }
                    out = handOffset;
                    return true;
                }
            }
            return saved_grab_offset::loadDefaultGrabPose(baseForm->GetFormID(), isLeft, out);
        }

        /*
         * proxyWorld/proxyWorldValid are resolved by the caller (a live
         * GrabAuthorityProxy read, Hand::tryComputeGrabProxyLocalPalmPocketFrameWorld)
         * since this file's helpers are free functions with no Hand access.
         */
        bool tryResolveSavedGrabOffsetAttach(const RE::NiTransform& proxyWorld, bool proxyWorldValid,
            const saved_grab_offset::HandOffset& handOffset, RE::NiTransform& desiredRootWorld)
        {
            if (!proxyWorldValid) {
                return false;
            }

            RE::NiTransform objectProxyLocal = transform_math::makeIdentityTransform<RE::NiTransform>();
            objectProxyLocal.translate = { handOffset.translateGame[0], handOffset.translateGame[1],
                handOffset.translateGame[2] };
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    objectProxyLocal.rotate.entry[row][column] = handOffset.rotate[row * 3 + column];
                }
            }

            desiredRootWorld = grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorld, objectProxyLocal);
            return isFiniteNiTransform(desiredRootWorld);
        }

        LooseWeaponPrimaryAttachFrame resolveLooseWeaponPrimaryAttachFrame(bool looseWeaponGrab,
            bool grabbedFromPullCatch, bool isLeft, const SelectedObject& selection, const RE::NiAVObject* rootNode,
            const RE::NiTransform& rootBodyLocalAtGrab, const RE::NiTransform& objectToBodyAtGrab,
            const RE::NiTransform& grabBodyWorldAtGrab, const RE::NiPoint3& grabPivotAWorld,
            bool savedGrabOffsetAttachValid, const RE::NiTransform& savedGrabOffsetRootWorld,
            const loose_weapon_grip_zone::NearGrab* nearGrip)
        {
            LooseWeaponPrimaryAttachFrame frame{};
            // The acquisition policy has already selected exact authored
            // placement or a real touching mesh grip.
            const bool nearAuthored = looseWeaponGrab && nearGrip && nearGrip->pose.valid();
            if (!nearAuthored && !savedGrabOffsetAttachValid) {
                frame.reason = "dynamicTouchHold";
                return frame;
            }

            if (savedGrabOffsetAttachValid) {
                /*
                 * A per-object saved offset is explicit hand-placement
                 * authority, so it overrides both the generic FRIK weapon
                 * offset and the throwable live-pose default.
                 */
                frame.desiredRootWorld = savedGrabOffsetRootWorld;
                frame.sourceVisible = false;
                frame.reason = "savedGrabOffset";
            } else {
                if (!looseWeaponGrab) {
                    frame.reason = "notLooseWeapon";
                    return frame;
                }
                /*
                 * Non-throwable weapons use the selected authored relation.
                 * Grenades, mines, and Molotov variants are
                 * hand-thrown objects: force-grab and pull-catch commits keep the
                 * normal mesh/body relation so the object is translated into the
                 * pocket without forcing a root rotation from FRIK or the live hand.
                 */
                const auto* looseWeapon = selectedLooseWeaponForm(selection);
                const bool throwableArrival =
                    isThrowableLooseWeapon(looseWeapon) && (grabbedFromPullCatch || selection.forcedArrival);
                if (throwableArrival) {
                    frame.reason = selection.forcedArrival ? "throwableForcedArrivalPreservePose"
                                                           : "throwablePullCatchPreservePose";
                    return frame;
                }
                if (!rootNode || !isFiniteNiTransform(rootNode->world)) {
                    frame.reason = "missingWeaponRoot";
                    return frame;
                }

                frame.pose = nearGrip->pose;
                frame.supportGrip = frame.pose.role == loose_weapon_authored_grab_policy::Role::Support;
                frame.reason = frame.supportGrip ? "authoredSupportGrip" : "authoredFiringGrip";
            }

            frame.desiredRootWorld.scale =
                rootNode && std::isfinite(rootNode->world.scale) && rootNode->world.scale > 0.0001f
                ? rootNode->world.scale
                : 1.0f;
            if (nearAuthored) {
                const auto palmInHand = palm_point::handLocal(isLeft);
                frame.desiredRootWorld =
                    nearGrip->pose.placementWorld(nearGrip->handWorld, frame.desiredRootWorld.scale, palmInHand);
                const auto proxyDelta =
                    grabPivotAWorld - transform_math::localPointToWorld(nearGrip->handWorld, palmInHand);
                ROCK_LOG_DEBUG(Hand,
                    "{} authored loose placement uses equipped palm form={:08X} proxyDelta=({:.4f},{:.4f},{:.4f})gu",
                    isLeft ? "left" : "right", nearGrip->pose.weaponFormId, proxyDelta.x, proxyDelta.y, proxyDelta.z);
            }
            if (!isFiniteNiTransform(frame.desiredRootWorld)) {
                frame.reason = "nonFiniteDesiredRoot";
                return frame;
            }

            frame.desiredBodyWorld = multiplyTransforms(frame.desiredRootWorld, rootBodyLocalAtGrab);
            frame.desiredObjectWorld = deriveNodeWorldFromBodyWorld(frame.desiredBodyWorld, objectToBodyAtGrab);
            // Preserve the generated proxy's physical pivot. Its weapon/body
            // anchor is derived AFTER placement, so it cannot offset the hand
            // and weapon from the equipped palm seat (0.86gu in the capture).
            const RE::NiPoint3 desiredPivotBodyLocal =
                transform_math::worldPointToLocal(frame.desiredBodyWorld, grabPivotAWorld);
            frame.gripPointWorld = transform_math::localPointToWorld(grabBodyWorldAtGrab, desiredPivotBodyLocal);
            if (!isFiniteNiTransform(frame.desiredBodyWorld) || !isFiniteNiTransform(frame.desiredObjectWorld) ||
                !std::isfinite(frame.gripPointWorld.x) || !std::isfinite(frame.gripPointWorld.y) ||
                !std::isfinite(frame.gripPointWorld.z)) {
                frame.reason = "nonFiniteDesiredBody";
                return frame;
            }

            frame.valid = true;
            return frame;
        }

        void storeGripSourceEvidence(CanonicalGrabFrame& frame, RE::NiAVObject* sourceNode,
            const RE::NiTransform& fallbackObjectWorld, const RE::NiPoint3& gripPointWorld,
            const RE::NiPoint3& gripNormalWorld, bool normalValid)
        {
            /*
             * Mesh triangles choose a world-space position only. A rendered
             * triangle can live under a child node whose native X/Y/Z does not
             * match the collidable node or the hknp BODY, so its local point is
             * stored only as source-node evidence. Solver authority is the
             * separate BODY-local pivot B captured from the same world point.
             */
            const RE::NiTransform& evidenceWorld = sourceNode ? sourceNode->world : fallbackObjectWorld;
            frame.gripEvidence.gripSourceNode = sourceNode;
            frame.gripEvidence.gripSourceNodeWorldAtGrab = evidenceWorld;
            frame.gripEvidence.gripPointSourceNodeLocal =
                transform_math::worldPointToLocal(evidenceWorld, gripPointWorld);
            frame.gripEvidence.hasGripSourceNodePoint = true;
            if (normalValid) {
                frame.gripEvidence.gripNormalSourceNodeLocal =
                    transform_math::worldVectorToLocal(evidenceWorld, gripNormalWorld);
                frame.gripEvidence.hasGripSourceNodeNormal = true;
            } else {
                frame.gripEvidence.gripNormalSourceNodeLocal = {};
                frame.gripEvidence.hasGripSourceNodeNormal = false;
            }
        }

        // iLooseGrabFingerPoseMode = 1 (swept arc): finger targets and skeleton rebase, as before the capsule sweep.
        grab_finger_pose_runtime::GrabFingerPoseTargetSet buildRuntimeFingerPoseTargets(
            const RE::NiPoint3& seatPointWorld, const RE::NiPoint3& seatNormalWorld)
        {
            auto targets = grab_finger_pose_runtime::makeSharedGripPoseTarget(seatPointWorld, seatNormalWorld);
            targets.useSeatPointForMissingTargets = false;
            targets.useWholeMeshForMissingTargets = true;
            return targets;
        }

        void storeFingerPoseTargetsInGrabFrame(CanonicalGrabFrame& frame,
            const grab_finger_pose_runtime::GrabFingerPoseTargetSet& targets,
            const RE::NiTransform& objectWorldTransform)
        {
            frame.fingerPoseTargetLocal = {};
            frame.fingerPoseTargetNormalLocal = {};
            frame.fingerPoseTargetValid = {};
            frame.fingerPoseTargetNormalValid = {};
            frame.fingerPoseTargetCount = 0;
            for (std::size_t finger = 0; finger < targets.targets.size(); ++finger) {
                if (!targets.targetValid[finger]) {
                    continue;
                }
                frame.fingerPoseTargetLocal[finger] =
                    transform_math::worldPointToLocal(objectWorldTransform, targets.targets[finger]);
                frame.fingerPoseTargetValid[finger] = 1;
                if (targets.targetNormalValid[finger]) {
                    frame.fingerPoseTargetNormalLocal[finger] =
                        transform_math::worldVectorToLocal(objectWorldTransform, targets.targetNormals[finger]);
                    frame.fingerPoseTargetNormalValid[finger] = 1;
                }
                ++frame.fingerPoseTargetCount;
            }
        }

        constexpr std::size_t kMaxGrabRuntimeFingerPoseTriangles = 2048;

        std::vector<TriangleData> selectNearestGrabFingerPoseTriangles(
            const std::vector<GrabSurfaceTriangleData>& sourceTriangles, const RE::NiPoint3& centerWorld,
            std::size_t maxTriangles, const GrabSurfaceQueryIndex& queryIndex)
        {
            std::vector<TriangleData> result;
            if (sourceTriangles.size() <= maxTriangles || maxTriangles == 0 ||
                !grab_three_phase::isFinite(centerWorld)) {
                result.reserve(sourceTriangles.size());
                for (const auto& triangle : sourceTriangles)
                    result.push_back(triangle.triangle);
            } else {
                const auto& selected = queryIndex.nearest(sourceTriangles, centerWorld, maxTriangles);
                result.reserve(selected.size());
                for (const auto& triangle : selected)
                    result.push_back(sourceTriangles[triangle.index].triangle);
            }
            return result;
        }

        RE::NiTransform computeRuntimeBodyLocalTransform(const RE::NiTransform& nodeWorld,
            const RE::NiTransform& bodyWorld)
        {
            return multiplyTransforms(invertTransform(nodeWorld), bodyWorld);
        }

        const char* grabPivotAuthoritySourceName(GrabPivotAuthoritySource source)
        {
            return grab_authority_frame_math::grabAuthorityPivotSourceName(source);
        }

        GrabSeatDepthStopResult computeGrabPalmSeatDepthStop(const std::vector<GrabLocalTriangle>& localTriangles,
            const RE::NiTransform& objectNodeWorld, const RE::NiPoint3& depthOriginWorld,
            const grab_three_phase::GrabPocketFrame& pocket,
            const HandBoneColliderSet::PublishedSegmentFrames& segments, float maxFootprintRadiusGameUnits,
            float maxDepthGameUnits)
        {
            GrabSeatDepthStopResult invalid{};
            invalid.reason = "missingPalmFootprint";
            if (!pocket.valid || !isFiniteNiTransform(objectNodeWorld) ||
                !grab_three_phase::isFinite(depthOriginWorld) || !std::isfinite(maxFootprintRadiusGameUnits) ||
                maxFootprintRadiusGameUnits <= 0.0f) {
                invalid.reason = "invalidPalmSeatFrame";
                return invalid;
            }
            for (const auto& segment : segments) {
                if (segment.role != hand_collider_semantics::HandColliderRole::PalmFace || !segment.valid ||
                    !isFiniteNiTransform(segment.target) || !grab_three_phase::isFinite(segment.palmHalfExtents) ||
                    segment.palmHalfExtents.x <= 0.0f || segment.palmHalfExtents.y <= 0.0f ||
                    segment.palmHalfExtents.z <= 0.0f || !std::isfinite(segment.convexRadius) ||
                    segment.convexRadius < 0.0f) {
                    continue;
                }
                // Project the created palm hull onto the pocket plane. Its
                // generated-body axes are columns; mesh axes use Ni row storage.
                const auto axisX = hand_bone_collider_geometry_math::generatedColliderLocalVectorToWorld(segment.target,
                    RE::NiPoint3{ 1, 0, 0 });
                const auto axisY = hand_bone_collider_geometry_math::generatedColliderLocalVectorToWorld(segment.target,
                    RE::NiPoint3{ 0, 1, 0 });
                const auto axisZ = hand_bone_collider_geometry_math::generatedColliderLocalVectorToWorld(segment.target,
                    RE::NiPoint3{ 0, 0, 1 });
                auto projectedHalfExtent = [&](const RE::NiPoint3& axis) {
                    return std::abs(dotProduct(axisX, axis)) * segment.palmHalfExtents.x +
                        std::abs(dotProduct(axisY, axis)) * segment.palmHalfExtents.y +
                        std::abs(dotProduct(axisZ, axis)) * segment.palmHalfExtents.z +
                        segment.convexRadius * std::abs(segment.target.scale);
                };
                float halfAlong = projectedHalfExtent(pocket.fingerForwardWorld);
                float halfAcross = projectedHalfExtent(pocket.crossPalmWorld);
                const float footprintRadius = std::hypot(halfAlong, halfAcross);
                if (footprintRadius > maxFootprintRadiusGameUnits) {
                    const float scale = maxFootprintRadiusGameUnits / footprintRadius;
                    halfAlong *= scale;
                    halfAcross *= scale;
                }
                // The mesh will translate from depthOrigin to the palm anchor.
                // Carry only that translation into the footprint so side tabs
                // are tested against the palm they would actually overlap.
                const auto footprintCenterWorld =
                    depthOriginWorld + (segment.target.translate - pocket.palmCenterWorld);
                const auto centerLocal = transform_math::worldPointToLocal(objectNodeWorld, footprintCenterWorld);
                const auto originLocal = transform_math::worldPointToLocal(objectNodeWorld, depthOriginWorld);
                const auto alongLocal = transform_math::worldVectorToLocal(objectNodeWorld, pocket.fingerForwardWorld);
                const auto acrossLocal = transform_math::worldVectorToLocal(objectNodeWorld, pocket.crossPalmWorld);
                const auto inwardLocal =
                    transform_math::worldVectorToLocal(objectNodeWorld, pocket.palmNormalWorld * -1.0f);
                const float objectScale = std::abs(objectNodeWorld.scale);
                return grab_palm_seat::computeDepth(localTriangles, halfAlong, halfAcross, maxDepthGameUnits,
                    [&](const RE::NiPoint3& vertex) {
                        const auto lateralDelta = vertex - centerLocal;
                        return grab_palm_seat::Point{
                            dotProduct(lateralDelta, alongLocal) * objectScale,
                            dotProduct(lateralDelta, acrossLocal) * objectScale,
                            dotProduct(vertex - originLocal, inwardLocal) * objectScale,
                        };
                    });
            }
            return invalid;
        }

        /*
         * World-side rigid rotation of an NiTransform about a world pivot
         * point. Stored NiMatrix3 rows are the world images of the local axes
         * (hand_frame::transformHandspaceLocalToWorld documents the engine
         * convention), so a world rotation applies vector Rodrigues to each
         * stored row and to the pivot-relative translation. The pivot point
         * itself is the fixed point: worldPointToLocal(rotated, pivotWorld)
         * equals worldPointToLocal(original, pivotWorld).
         */
        RE::NiTransform rotateTransformWorldAboutPoint(const RE::NiTransform& transform,
            const RE::NiPoint3& unitAxisWorld, float angleRadians, const RE::NiPoint3& pivotWorld)
        {
            RE::NiTransform rotated = transform;
            for (int row = 0; row < 3; ++row) {
                const RE::NiPoint3 rowWorld{
                    transform.rotate.entry[row][0],
                    transform.rotate.entry[row][1],
                    transform.rotate.entry[row][2],
                };
                const RE::NiPoint3 rotatedRow =
                    grab_finger_pose_math::rotateAroundUnitAxis(rowWorld, unitAxisWorld, angleRadians);
                rotated.rotate.entry[row][0] = rotatedRow.x;
                rotated.rotate.entry[row][1] = rotatedRow.y;
                rotated.rotate.entry[row][2] = rotatedRow.z;
            }
            const RE::NiPoint3 pivotOffset = transform.translate - pivotWorld;
            const RE::NiPoint3 rotatedOffset =
                grab_finger_pose_math::rotateAroundUnitAxis(pivotOffset, unitAxisWorld, angleRadians);
            rotated.translate = pivotWorld + rotatedOffset;
            return rotated;
        }
    }

    bool Hand::captureGrabBodyFrame(RE::hknpWorld* world, const GrabBodyFrameCaptureInput& input,
        GrabBodyFrameCapture& outCapture)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabBodyFrameCapture);
        outCapture = {};
        const auto& handWorldTransform = *input.handWorldTransform;
        const auto& proxy = *input.proxy;
        const auto& meshCapture = *input.meshCapture;
        const auto& mesh = *input.mesh;
        auto& surface = *input.surface;
        const auto& pivotEvidence = *input.pivotEvidence;
        const auto& bodyResolution = *input.bodyResolution;
        const auto& grabLocalMeshTriangles = *input.localMeshTriangles;
        const auto& sel = _currentSelection;
        auto* collidableNode = meshCapture.collidableNode;
        const auto& objectWorldTransform = meshCapture.objectWorldTransform;
        const auto& grabMeshTriangles = mesh.meshTriangles;
        auto& grabSurfaceHit = surface.surfaceHit;
        auto& grabGripPoint = surface.gripPoint;
        const auto pivotAuthoritySource = pivotEvidence.authoritySource;
        const bool pivotAuthorityPositionOnly = pivotEvidence.authorityPositionOnly;
        const bool pivotAuthorityNormalTrusted = pivotEvidence.authorityNormalTrusted;
        const float pivotAuthorityPositionConfidence = pivotEvidence.authorityPositionConfidence;
        const auto& primaryChoice = bodyResolution.primaryChoice;
        const auto& palmPocketPivotAWorld = proxy.palmPocketPivotAWorld;
        auto* rootNode = input.rootNode;
        auto objectBodyId = input.objectBodyId;
        const auto& objName = *input.objectName;
        const char* grabPointMode = surface.pointMode;
        auto& palmPos = outCapture.palmPosition;
        auto& grabBodyWorldAtGrab = outCapture.grabBodyWorld;
        auto& motionBodyWorldAtGrab = outCapture.motionBodyWorld;
        auto& motionBodySourceAtGrab = outCapture.motionBodySource;
        auto& hasMotionBodyWorldAtGrab = outCapture.hasMotionBodyWorld;
        auto& constraintUsesMotionBodyAtGrab = outCapture.constraintUsesMotionBody;
        auto& constraintBodyWorldAtGrab = outCapture.constraintBodyWorld;
        auto& grabPivotAWorld = outCapture.grabPivotAWorld;
        RE::TESObjectCELL* ownerCellAtGrab = nullptr;
        RE::bhkWorld* bhkWorldAtGrab = nullptr;
        auto*& bodyCollisionObjectAtGrab = outCapture.bodyCollisionObject;
        auto*& ownerNodeAtGrab = outCapture.ownerNode;
        auto& objectToBodyAtGrab = outCapture.objectToBody;
        auto& ownerBodyLocalAtGrab = outCapture.ownerBodyLocal;
        auto& rootBodyLocalAtGrab = outCapture.rootBodyLocal;
        auto& selectedGripPointLocal = outCapture.selectedGripPointLocal;
        auto& selectedPivotBBodyLocalGame = outCapture.selectedPivotBBodyLocalGame;
        std::vector<TriangleData> grabFingerPoseMeshTriangles;
        auto& grabFingerPoseLocalMeshTriangles = outCapture.fingerPoseLocalMeshTriangles;
        palmPos = computeGrabPalmPointWorldFromHandBasis(handWorldTransform, _isLeft);
        if (!tryGetGrabAuthorityBodyWorldTransform(world, objectBodyId, grabBodyWorldAtGrab)) {
            ROCK_LOG_ERROR(Hand, "{} hand GRAB FAILED: native grab BODY frame unreadable bodyId={} formID={:08X}",
                handName(), objectBodyId.value, sel.refr ? sel.refr->GetFormID() : 0);
            _grabFrame.clear();
            _heldBodyIds.clear();
            _heldDriveDecision = {};
            _heldBodyIdsCount.store(0, std::memory_order_release);
            return false;
        }
        hasMotionBodyWorldAtGrab =
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
        constraintUsesMotionBodyAtGrab = false;
        constraintBodyWorldAtGrab = grabBodyWorldAtGrab;
        /*
                 * The hidden proxy is body A for dynamic grab. The close-grab
                 * pocket pivot is captured from the same generated/proxy-local
                 * seat frame. The proxy body keeps its seat offset, and transform A
                 * carries the selected pivot as an explicit local point on body A.
                 */
        grabPivotAWorld = palmPocketPivotAWorld;
        ownerCellAtGrab = sel.refr ? sel.refr->GetParentCell() : nullptr;
        bhkWorldAtGrab = ownerCellAtGrab ? ownerCellAtGrab->GetbhkWorld() : nullptr;
        bodyCollisionObjectAtGrab =
            bhkWorldAtGrab ? RE::bhkNPCollisionObject::Getbhk(bhkWorldAtGrab, objectBodyId) : nullptr;
        ownerNodeAtGrab = bodyCollisionObjectAtGrab ? bodyCollisionObjectAtGrab->sceneObject : nullptr;
        if (ownerNodeAtGrab && ownerNodeAtGrab != collidableNode) {
            GrabCaptureTransformRefreshResult ownerAtGrabRefresh{};
            refreshGrabCaptureNodeTransform(ownerAtGrabRefresh, "ownerAtGrab", ownerNodeAtGrab,
                !grab_target::requiresNativeBodyOwnership(sel.targetKind));
            if (!ownerAtGrabRefresh.ok) {
                ROCK_LOG_WARN(Hand,
                    "{} hand GRAB failed: owner-at-grab transform refresh produced a non-finite node transform for "
                    "'{}' formID={:08X}; owner='{}'",
                    handName(), objName, sel.refr ? sel.refr->GetFormID() : 0, nodeDebugName(ownerNodeAtGrab));
                outCapture.clearExternalOnFailure = true;
                return false;
            }
        }
        _grabFrame.heldNode = collidableNode;
        objectToBodyAtGrab = computeRuntimeBodyLocalTransform(objectWorldTransform, grabBodyWorldAtGrab);
        /*
                 * ROCK dynamic grab has one production authority convention:
                 * the generated/proxy palm frame seats the selected BODY-local grip
                 * point and owns the object angular relation through a row-view of
                 * its generated local axes. The custom constraint stores body-B local
                 * data in the rigid BODY frame. MOTION and COM are mass/diagnostic
                 * data only.
                 */
        ownerBodyLocalAtGrab = ownerNodeAtGrab
            ? computeRuntimeBodyLocalTransform(ownerNodeAtGrab->world, grabBodyWorldAtGrab)
            : makeIdentityTransform();
        rootBodyLocalAtGrab =
            rootNode ? computeRuntimeBodyLocalTransform(rootNode->world, grabBodyWorldAtGrab) : makeIdentityTransform();
        selectedGripPointLocal = transform_math::worldPointToLocal(objectWorldTransform, grabGripPoint);
        selectedPivotBBodyLocalGame = transform_math::worldPointToLocal(grabBodyWorldAtGrab, grabGripPoint);
        if (!input.authoredSeatPrepared && !grabMeshTriangles.empty()) {
            grabFingerPoseMeshTriangles = selectNearestGrabFingerPoseTriangles(mesh.surfaceTriangles, grabGripPoint,
                kMaxGrabRuntimeFingerPoseTriangles, mesh.queryIndex);
            cacheTrianglesInLocalSpace(grabFingerPoseMeshTriangles, objectWorldTransform,
                grabFingerPoseLocalMeshTriangles);
            if (grabFingerPoseMeshTriangles.size() != grabMeshTriangles.size()) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand MESH FINGER POSE TRIANGLES: sourceTris={} localTris={} center=({:.1f},{:.1f},{:.1f})",
                    handName(), grabMeshTriangles.size(), grabFingerPoseMeshTriangles.size(), grabGripPoint.x,
                    grabGripPoint.y, grabGripPoint.z);
            }
        }
        _grabFrame.localMeshTriangles.clear();
        _grabFrame.contactMeshCache.clear();
        _grabFrame.fingerPoseLocalMeshTriangles.clear();
        _grabFrame.gripEvidence.gripEvidenceLocal = selectedGripPointLocal;
        _grabFrame.gripEvidence.gripNormalLocal = grabSurfaceHit.valid
            ? transform_math::worldVectorToLocal(objectWorldTransform, grabSurfaceHit.normal)
            : RE::NiPoint3{};
        storeGripSourceEvidence(_grabFrame, grabSurfaceHit.sourceNode ? grabSurfaceHit.sourceNode : collidableNode,
            objectWorldTransform, grabGripPoint, grabSurfaceHit.normal,
            grabSurfaceHit.valid && lengthSquared(grabSurfaceHit.normal) > 0.000001f);
        _grabFrame.gripEvidence.gripEvidenceTriangleIndex = grabSurfaceHit.valid && grabSurfaceHit.hasTriangle
            ? static_cast<std::uint32_t>(grabSurfaceHit.triangleIndex)
            : 0xFFFF'FFFF;
        _grabFrame.gripEvidence.gripEvidenceShapeKey = grabSurfaceHit.valid ? grabSurfaceHit.shapeKey : 0xFFFF'FFFF;
        _grabFrame.gripEvidence.gripEvidenceShapeCollisionFilterInfo =
            grabSurfaceHit.valid ? grabSurfaceHit.shapeCollisionFilterInfo : 0;
        _grabFrame.gripEvidence.gripEvidenceHitFraction = grabSurfaceHit.valid ? grabSurfaceHit.hitFraction : 1.0f;
        _grabFrame.gripEvidence.hasGripEvidenceShapeKey = grabSurfaceHit.valid && grabSurfaceHit.hasShapeKey;
        _grabFrame.seat.palmSeatPointWorldAtGrab = grabGripPoint;
        _grabFrame.seat.hasPalmSeatPoint = input.authoredSeatPrepared || surface.meshGrabFound;
        _grabFrame.seat.activeGrabPointMode = grabPointMode;
        _grabFrame.pivotAuthority.source = pivotAuthoritySource;
        _grabFrame.pivotAuthority.positionOnly = pivotAuthorityPositionOnly;
        _grabFrame.pivotAuthority.normalTrusted = pivotAuthorityNormalTrusted;
        _grabFrame.pivotAuthority.positionConfidence = pivotAuthorityPositionConfidence;
        _grabFrame.seat.palmSeatPointMode = grabPointMode;
        _grabFrame.bodyResolutionReason = primaryBodyChoiceReasonName(primaryChoice.reason);
        _grabFrame.pivotAuthority.pocketDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
        _grabFrame.pivotAuthority.selectionDistanceGameUnits = grabSurfaceHit.valid && grabSurfaceHit.hasSelectionHit
            ? grabSurfaceHit.selectionToMeshDistanceGameUnits
            : (sel.hasHitPoint ? 0.0f : std::numeric_limits<float>::max());
        if (grabSurfaceHit.valid) {
            grabSurfaceHit.pivotToSurfaceDistanceGameUnits = _grabFrame.pivotAuthority.pocketDistanceGameUnits;
        }
        _grabFrame.hasMeshPoseData = false;
        if (!grabLocalMeshTriangles.empty()) {
            _grabFrame.localMeshTriangles = grabLocalMeshTriangles;
            _grabFrame.contactMeshCache.clear();
        }
        if (!grabFingerPoseLocalMeshTriangles.empty()) {
            _grabFrame.fingerPoseLocalMeshTriangles = grabFingerPoseLocalMeshTriangles;
        }
        _grabFrame.hasMeshPoseData =
            !_grabFrame.localMeshTriangles.empty() || !_grabFrame.fingerPoseLocalMeshTriangles.empty();

        return true;
    }

    bool Hand::resolveGrabSeatCapture(RE::hknpWorld* world, const GrabSeatCaptureInput& input,
        GrabSeatCaptureResult& outCapture)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabSeatCapture);
        outCapture = {};
        const auto& proxy = *input.proxy;
        const auto& meshCapture = *input.meshCapture;
        const auto& mesh = *input.mesh;
        auto& surface = *input.surface;
        auto& pivotEvidence = *input.pivotEvidence;
        auto& bodyFrame = *input.bodyFrame;
        const auto& pinchPocketCandidate = *input.pinchPocketCandidate;
        auto& savedGrabOffset = *input.savedGrabOffset;
        auto& hasSavedGrabOffset = *input.hasSavedGrabOffset;
        const auto& sel = _currentSelection;
        const auto& proxyAuthorityFrameWorldAtGrab = proxy.proxyAuthorityFrameWorldAtGrab;
        const auto& objectWorldTransform = meshCapture.objectWorldTransform;
        const auto& grabMeshTriangles = mesh.meshTriangles;
        const auto& grabLocalMeshTriangles = _grabFrame.localMeshTriangles;
        auto& grabGripPoint = surface.gripPoint;
        auto& selectionToMeshDistanceGameUnits = surface.selectionToMeshDistanceGameUnits;
        auto& grabSurfaceHit = surface.surfaceHit;
        const char*& grabPointMode = surface.pointMode;
        auto& grabPointAuthoritySource = surface.pointAuthoritySource;
        const char*& grabFallbackReason = surface.fallbackReason;
        auto& pivotAuthoritySource = pivotEvidence.authoritySource;
        auto& pivotAuthorityNormalTrusted = pivotEvidence.authorityNormalTrusted;
        auto& pivotAuthorityPositionOnly = pivotEvidence.authorityPositionOnly;
        auto& pivotAuthorityPositionConfidence = pivotEvidence.authorityPositionConfidence;
        auto& pivotAuthorityPocketDistanceGameUnits = pivotEvidence.authorityPocketDistanceGameUnits;
        auto& pivotAuthoritySelectionDeltaGameUnits = pivotEvidence.authoritySelectionDeltaGameUnits;
        auto& pivotAuthorityLongLeverGameUnits = pivotEvidence.authorityLongLeverGameUnits;
        const auto& semanticContacts = pivotEvidence.semanticContacts;
        auto& grabBodyWorldAtGrab = bodyFrame.grabBodyWorld;
        auto& grabPivotAWorld = bodyFrame.grabPivotAWorld;
        auto& selectedGripPointLocal = bodyFrame.selectedGripPointLocal;
        auto& selectedPivotBBodyLocalGame = bodyFrame.selectedPivotBBodyLocalGame;
        const auto& objectToBodyAtGrab = bodyFrame.objectToBody;
        const auto& rootBodyLocalAtGrab = bodyFrame.rootBodyLocal;
        const auto objectBodyId = input.objectBodyId;
        auto* rootNode = input.rootNode;
        auto* collidableNode = input.collidableNode;
        const bool grabbedFromPullCatch = input.grabbedFromPullCatch;
        const bool looseWeaponGrab = input.looseWeaponGrab;
        loose_weapon_grip_zone::NearGrab nearGrip{};
        const auto pocketAtAcquisition =
            grab_three_phase::buildGrabPocketFrameWithPalmCenter(proxyAuthorityFrameWorldAtGrab, _isLeft,
                grabPivotAWorld, g_rockConfig.rockGrabPocketDepthGameUnits, g_rockConfig.rockGrabPocketRadiusGameUnits);
        const auto pocketDelta = grabGripPoint - pocketAtAcquisition.palmCenterWorld;
        const bool touchingPocket = pocketAtAcquisition.valid && semanticContacts.count != 0 &&
            pointDistanceGameUnits(grabGripPoint, pocketAtAcquisition.palmCenterWorld) <=
                pocketAtAcquisition.pocketRadiusGameUnits &&
            dot(pocketDelta, pocketAtAcquisition.palmNormalWorld) >=
                -g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits;
        const bool authoredWeapon = looseWeaponGrab && !isThrowableLooseWeapon(selectedLooseWeaponForm(sel));
        bool requestAuthoredPose = false;
        bool nearAuthored = false;
        if (input.preparedAuthoredGrip) {
            nearGrip = *input.preparedAuthoredGrip;
            requestAuthoredPose = true;
            const auto* weapon = selectedLooseWeaponForm(sel);
            nearAuthored = authoredWeapon && weapon && sel.refr->Get3D() == rootNode && nearGrip.handWorldValid &&
                nearGrip.pose.valid() && nearGrip.pose.weaponFormId == weapon->formID &&
                nearGrip.pose.isLeft == _isLeft && nearGrip.pose.role == nearGrip.role;
            if (!nearAuthored) {
                performance_profiler::addCounter(performance_profiler::Counter::GrabAuthoredPreparationInvalidated);
            }
        } else if (authoredWeapon) {
            namespace policy = loose_weapon_authored_grab_policy;
            (void)loose_weapon_grip_zone::tryResolveNearGrab(_isLeft, sel.refr, nearGrip, input.joiningPeerHeldObject);
            const auto role = policy::acquisitionRole(touchingPocket && !grabbedFromPullCatch && !sel.forcedArrival,
                input.transferPose != nullptr, input.transferPose ? input.transferPose->role : policy::Role::None,
                nearGrip.arrangement, input.joiningPeerHeldObject, nearGrip.role,
                policy::freeWeaponRole(
                    policy::acquisitionSource(grabbedFromPullCatch, sel.forcedArrival, input.transferPose != nullptr),
                    nearGrip.nearestRole));
            const bool dynamicOffhand = policy::dynamicSupportAcquisition(role, input.joiningPeerHeldObject,
                nearGrip.arrangement, input.transferPose != nullptr);
            ROCK_LOG_SAMPLE_INFO(Hand, 1000,
                "{} loose weapon role selection ref={:08X} peerHolding={} touchingPocket={} pull={} forced={} "
                "transfer={} arrangement={} zoneRole={} nearestRole={} selectedRole={} dynamicSupport={}",
                handName(), sel.refr ? sel.refr->GetFormID() : 0, input.joiningPeerHeldObject, touchingPocket,
                grabbedFromPullCatch, sel.forcedArrival, input.transferPose != nullptr,
                static_cast<unsigned>(nearGrip.arrangement), static_cast<unsigned>(nearGrip.role),
                static_cast<unsigned>(nearGrip.nearestRole), static_cast<unsigned>(role), dynamicOffhand);
            requestAuthoredPose = (role != policy::Role::None || input.transferPose != nullptr) && !dynamicOffhand;
            nearGrip.role = role;
            // Retain the controller-derived frame resolved by the zone runtime.
            // The incoming grab wrist can already be presented by ROCK.
            if (input.transferPose) {
                nearGrip.pose = *input.transferPose;
                const auto* weapon = selectedLooseWeaponForm(sel);
                nearAuthored = nearGrip.pose.valid() && weapon && nearGrip.pose.weaponFormId == weapon->formID &&
                    nearGrip.pose.isLeft == _isLeft;
            } else if (requestAuthoredPose) {
                nearAuthored =
                    loose_weapon_grip_zone::tryResolveAuthoredGrabPose(_isLeft, sel.refr, role, nearGrip.pose);
            }
            nearAuthored = nearAuthored && nearGrip.handWorldValid;
        }
        const RE::NiPoint3 canonicalPivotNormalWorld = input.canonicalPivotNormalWorld;
        auto& desiredObjectWorld = outCapture.desiredObjectWorld;
        auto& desiredBodyWorld = outCapture.desiredBodyWorld;
        auto& resolvedAuthorityPivotSourceForFreeze = outCapture.resolvedAuthoritySource;
        auto& resolvedAuthorityPivotReasonForFreeze = outCapture.resolvedAuthorityReason;
        auto abortCapture = [&]() {
            _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::Idle;
            _grabObjectGripAtGrab = {};
            _heldObjectIsLooseWeapon = false;
            _grabFrame.clear();
            _heldBodyIds.clear();
            _heldDriveDecision = {};
            _heldBodyIdsCount.store(0, std::memory_order_release);
            _grabFingerPosePublished = false;
            (void)frik_visual_authority::clearHandPose("ROCK_Grab", handFromBool(_isLeft));
            clearGrabExternalHandWorldTransform(_isLeft);
            input.rollback.rollback();
            return false;
        };
        desiredObjectWorld = objectWorldTransform;
        if (input.preparedAuthoredGrip && !nearAuthored) {
            // The authored pose is unavailable now; hold the weapon dynamically.
            ROCK_LOG_SAMPLE_INFO(Hand, 1000,
                "{} authored weapon pose unavailable; dynamic grab ref={:08X} role={} transfer={}", handName(),
                sel.refr ? sel.refr->GetFormID() : 0, static_cast<unsigned>(nearGrip.role),
                input.transferPose != nullptr);
        }
        desiredBodyWorld = grabBodyWorldAtGrab;
        bool looseWeaponPrimaryAttachApplied = false;
        bool looseWeaponPrimaryAttachSourceVisible = false;
        const char* looseWeaponPrimaryAttachReason = "notEvaluated";
        const auto oppositionContacts = hand_semantic_contact_state::selectThumbOppositionContacts(semanticContacts);
        resolvedAuthorityPivotSourceForFreeze = grab_authority_frame_math::GrabAuthorityPivotSource::None;
        resolvedAuthorityPivotReasonForFreeze = "notResolved";
        /*
                 * Runtime grab authority is intentionally single-source. Mesh hits,
                 * authored nodes, semantic contacts, contact patches, and multi-finger
                 * contacts are evidence until the pivot resolver chooses one BODY-local
                 * point to freeze. Final dynamic authority is strict: only a pinch
                 * pocket or an authorable support group may create the frozen frame.
                 */
        {
            auto pocket = pocketAtAcquisition;
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
            if (!nearAuthored && pinchPocketCandidate.valid) {
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
            const float gripToPocketDistance = pocket.valid
                ? std::sqrt(gripToPocket.x * gripToPocket.x + gripToPocket.y * gripToPocket.y +
                      gripToPocket.z * gripToPocket.z)
                : std::numeric_limits<float>::max();
            const float stableTouchEnvelope =
                (std::max)(g_rockConfig.rockGrabTouchAcquireDistanceGameUnits, pocket.pocketRadiusGameUnits);
            const bool hasStablePocketTouchContact =
                oppositionContacts.valid && gripToPocketDistance <= stableTouchEnvelope;
            const bool meshSurfaceAuthorityEvidence =
                grabSurfaceHit.valid && grabSurfaceHit.sourceKind != GrabSurfaceSourceKind::CollisionQuery;
            const bool programmaticArrival = grabbedFromPullCatch || sel.forcedArrival;
            const bool pullArrivalTouchHeldAuthorityEvidence =
                hasStablePocketTouchContact || meshSurfaceAuthorityEvidence;
            /*
                     * Mesh/support evidence proves that ROCK found an object-side
                     * seat; it does not prove that a forced arrival has reached the
                     * hand. Pull catch may retain its flight evidence, but a direct
                     * forced arrival may enter TouchHeld immediately only from real
                     * stable hand contact. Otherwise it must converge to the seated
                     * relation before final hand-pose publication.
                     */
            const bool programmaticTouchHeldAuthorityEvidence = grabbedFromPullCatch
                ? pullArrivalTouchHeldAuthorityEvidence
                : (!sel.forcedArrival || hasStablePocketTouchContact);
            const auto phaseDecision =
                grab_three_phase::classifyAcquisitionPhase(grab_three_phase::PhaseClassificationInput{
                    .pocket = pocket,
                    .gripSeedWorld = phaseGripSeed,
                    .hasFreshTouchContact = hasStablePocketTouchContact,
                    .isFarSelection = sel.isFarSelection,
                    .programmaticArrival = programmaticArrival,
                    .requireEvidenceForTouchHeld = programmaticArrival,
                    .hasTouchHeldAuthorityEvidence = programmaticTouchHeldAuthorityEvidence,
                    .touchAcquireDistanceGameUnits = g_rockConfig.rockGrabTouchAcquireDistanceGameUnits,
                    .touchContactMaxDistanceGameUnits = stableTouchEnvelope,
                    .nearConvergeDistanceGameUnits = g_rockConfig.rockGrabNearConvergeDistanceGameUnits,
                    .behindPalmToleranceGameUnits = g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits,
                });

            const bool usingPinchPocket = !nearAuthored && pinchPocketCandidate.valid && gripArea.valid;
            const bool captureAccepted =
                usingPinchPocket || (pocket.valid && gripArea.valid && (nearAuthored || phaseDecision.accepted));
            if (captureAccepted) {
                _grabObjectGripAtGrab = gripArea;
                _grabAcquisitionPhase = nearAuthored
                    ? grab_three_phase::AcquisitionPhase::NearConverging
                    : (usingPinchPocket ? grab_three_phase::AcquisitionPhase::TouchHeld : phaseDecision.phase);
                if (usingPinchPocket) {
                    _grabObjectGripAtGrab.source = "pinchPocket";
                    _grabObjectGripAtGrab.fallbackReason = pinchPocketCandidate.decision.reason;
                    _grabObjectGripAtGrab.confidence = 0.95f;
                }
                const char* relationMode = usingPinchPocket ? "pinchPocket" : "rockPointToPalm";
                const char* captureReason =
                    usingPinchPocket ? pinchPocketCandidate.decision.reason : phaseDecision.reason;

                grabGripPoint = gripArea.contactSeedWorld;
                /*
                 * A pinch keeps the object where the hand found it: pivot A is
                 * the mesh point itself, so the frozen relation is the current
                 * one and only the fingers move, closing onto the surface. The
                 * player places the object between thumb and finger precisely;
                 * pulling it onto a predicted fingertip midpoint undid that.
                 */
                grabPivotAWorld = usingPinchPocket ? grabGripPoint : pocket.palmCenterWorld;
                grabPointMode = relationMode;
                grabPointAuthoritySource = usingPinchPocket ? GrabPivotAuthoritySource::PinchPocketMeshPoint
                                                            : GrabPivotAuthoritySource::PalmPocketMeshPoint;
                grabFallbackReason = captureReason;
                if (usingPinchPocket) {
                    grabSurfaceHit = pinchPocketCandidate.surfaceHit;
                    grabSurfaceHit.hasSelectionHit = sel.hasHitPoint;
                    grabSurfaceHit.selectionToMeshDistanceGameUnits = sel.hasHitPoint
                        ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint)
                        : std::numeric_limits<float>::max();
                    grabSurfaceHit.pivotToSurfaceDistanceGameUnits =
                        pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                    grabSurfaceHit.resolvedOwnerMatchesBody = true;
                    selectionToMeshDistanceGameUnits = grabSurfaceHit.selectionToMeshDistanceGameUnits;
                    pivotAuthoritySource = GrabPivotAuthoritySource::PinchPocketMeshPoint;
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
                    canonicalPivotNormalWorld,
                    usingPinchPocket ? pinchPocketCandidate.pinchAxisWorld : RE::NiPoint3{},
                    pocket.palmNormalWorld,
                });

                RE::NiPoint3 gripEvidencePointWorld = grabGripPoint;
                RE::NiPoint3 gripEvidenceNormalWorld = gripNormalWorld;
                const float gripObjectScale =
                    std::isfinite(objectWorldTransform.scale) && objectWorldTransform.scale > 0.0f
                    ? objectWorldTransform.scale
                    : 1.0f;
                // Furthest mesh point from the grip: the release spin cap
                // for long objects reads this lever.
                const float gripLeverGameUnits =
                    !grabLocalMeshTriangles.empty() && grab_three_phase::isFinite(objectWorldTransform)
                    ? computeLocalMeshMaxDistanceFromPoint(grabLocalMeshTriangles,
                          transform_math::worldPointToLocal(objectWorldTransform, grabGripPoint)) *
                        gripObjectScale
                    : 0.0f;
                if (usingPinchPocket) {
                    resolvedAuthorityPivotSourceForFreeze =
                        grab_authority_frame_math::GrabAuthorityPivotSource::PinchPocketMeshPoint;
                    resolvedAuthorityPivotReasonForFreeze =
                        pinchPocketCandidate.decision.reason ? pinchPocketCandidate.decision.reason : "none";
                } else {
                    resolvedAuthorityPivotSourceForFreeze = pivotAuthoritySource;
                    resolvedAuthorityPivotReasonForFreeze = grabFallbackReason ? grabFallbackReason : "none";
                }
                pivotAuthorityPositionOnly = false;
                pivotAuthorityNormalTrusted = lengthSquared(gripNormalWorld) > 0.000001f;
                pivotAuthorityLongLeverGameUnits = gripLeverGameUnits;
                pivotAuthorityPocketDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                pivotAuthoritySelectionDeltaGameUnits = sel.hasHitPoint
                    ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint)
                    : std::numeric_limits<float>::max();
                gripEvidencePointWorld = grabGripPoint;
                gripEvidenceNormalWorld = gripNormalWorld;
                _grabObjectGripAtGrab.contactSeedWorld = grabGripPoint;
                _grabObjectGripAtGrab.gripCenterWorld = grabGripPoint;
                _grabObjectGripAtGrab.source = grabPointMode;
                _grabObjectGripAtGrab.fallbackReason = grabFallbackReason;

                const RE::NiPoint3 finalGripToPocketVector =
                    pocket.valid ? (grabGripPoint - pocket.palmCenterWorld) : RE::NiPoint3{};
                const float finalGripToPocketDistance = pocket.valid
                    ? std::sqrt(finalGripToPocketVector.x * finalGripToPocketVector.x +
                          finalGripToPocketVector.y * finalGripToPocketVector.y +
                          finalGripToPocketVector.z * finalGripToPocketVector.z)
                    : std::numeric_limits<float>::max();
                const float finalSignedPalmDistance =
                    pocket.valid ? grab_three_phase::dot(finalGripToPocketVector, pocket.palmNormalWorld) : 0.0f;
                const auto arrivalSeatSafety = grab_three_phase::evaluateProgrammaticArrivalSeatSafety(
                    grab_three_phase::ProgrammaticArrivalSeatSafetyInput{
                        .programmaticArrival = programmaticArrival,
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
                        .pocketRadiusGameUnits =
                            pocket.valid ? pocket.pocketRadiusGameUnits : g_rockConfig.rockGrabPocketRadiusGameUnits,
                        .palmNormalWorld = pocket.valid ? pocket.palmNormalWorld : RE::NiPoint3{},
                        .gripNormalWorld = gripNormalWorld,
                    });
                if (programmaticArrival && !usingPinchPocket &&
                    _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld &&
                    !arrivalSeatSafety.allowImmediateTouchHeld) {
                    _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::NearConverging;
                    captureReason = arrivalSeatSafety.reason;
                    grabFallbackReason = captureReason;
                    _grabObjectGripAtGrab.fallbackReason = captureReason;
                }

                desiredBodyWorld = grab_frame_math::shiftObjectToAlignGripWithPocket(grabBodyWorldAtGrab,
                    grabPivotAWorld, grabGripPoint);
                desiredObjectWorld = deriveNodeWorldFromBodyWorld(desiredBodyWorld, objectToBodyAtGrab);
                /*
                         * Guns and melee never seat from saved offsets: the FRIK
                         * weapon-offset attach below is the only weapon authority
                         * (saved_grab_offset::participatesInSavedGrabOffsets).
                         * Leaving the source empty here also keeps a saved finger
                         * pose from overriding the FRIK weapon hand pose.
                         */
                bool fixedAidPose = g_rockConfig.rockImmersiveAidEnabled &&
                    immersive_aid::usesStimpakPose(
                        immersive_aid::classify(sel.refr ? sel.refr->GetObjectReference() : nullptr));
                /*
                         * Owner rule (2026-09-30, behavior library "Vanilla grip templates"):
                         * saved offsets apply only when the object arrives by a far pull
                         * (or a forced arrival); every close grab, pinch pocket included,
                         * stays dynamic.
                         */
                if (fixedAidPose) {
                    savedGrabOffset = immersive_aid::stimpakPose(_isLeft);
                    hasSavedGrabOffset = true;
                } else if (programmaticArrival && !authoredWeapon &&
                    saved_grab_offset::participatesInSavedGrabOffsets(looseWeaponGrab,
                        isThrowableLooseWeapon(selectedLooseWeaponForm(sel)))) {
                    hasSavedGrabOffset = tryLoadSavedGrabOffsetHandOffset(sel.refr, _isLeft, savedGrabOffset);
                }
                RE::NiTransform grabProxyWorldForOffset{};
                const bool grabProxyWorldValidForOffset = (programmaticArrival || fixedAidPose) &&
                    tryComputeGrabProxyLocalPalmPocketFrameWorld(world, grabProxyWorldForOffset);
                RE::NiTransform savedGrabOffsetRootWorld{};
                const bool savedGrabOffsetAttachValid = hasSavedGrabOffset &&
                    tryResolveSavedGrabOffsetAttach(grabProxyWorldForOffset, grabProxyWorldValidForOffset,
                        savedGrabOffset, savedGrabOffsetRootWorld);
                if (fixedAidPose && !savedGrabOffsetAttachValid) {
                    // Without the authored proxy frame the aid is held like any loose item.
                    ROCK_LOG_SAMPLE_INFO(Hand, 1000, "{} hand immersive aid pose unavailable; dynamic grab",
                        handName());
                    fixedAidPose = false;
                    hasSavedGrabOffset = false;
                }
                const auto looseWeaponPrimaryAttachFrame =
                    resolveLooseWeaponPrimaryAttachFrame(looseWeaponGrab, grabbedFromPullCatch, _isLeft, sel, rootNode,
                        rootBodyLocalAtGrab, objectToBodyAtGrab, grabBodyWorldAtGrab, grabPivotAWorld,
                        savedGrabOffsetAttachValid, savedGrabOffsetRootWorld, nearAuthored ? &nearGrip : nullptr);
                looseWeaponPrimaryAttachReason = looseWeaponPrimaryAttachFrame.reason;
                if (nearAuthored && !looseWeaponPrimaryAttachFrame.valid) {
                    // The authored seat cannot be resolved; hold the weapon dynamically.
                    ROCK_LOG_SAMPLE_INFO(Hand, 1000,
                        "{} hand authored loose seat unavailable; dynamic grab ref={:08X} reason={}", handName(),
                        sel.refr ? sel.refr->GetFormID() : 0, looseWeaponPrimaryAttachReason);
                    nearAuthored = false;
                }
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
                    grabPointAuthoritySource = GrabPivotAuthoritySource::LooseWeaponPrimaryAttach;
                    relationMode = grabPointMode;
                    grabFallbackReason = looseWeaponPrimaryAttachFrame.reason;
                    _grabObjectGripAtGrab.source = grabPointMode;
                    _grabObjectGripAtGrab.fallbackReason = grabFallbackReason;
                    if (_grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld) {
                        _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::NearConverging;
                        captureReason = "looseWeaponPrimaryAttachSettle";
                    }
                    grabSurfaceHit = GrabSurfaceHit{};
                    pivotAuthoritySource = GrabPivotAuthoritySource::LooseWeaponPrimaryAttach;
                    pivotAuthorityPositionOnly = false;
                    pivotAuthorityNormalTrusted = true;
                    pivotAuthorityPositionConfidence = 1.0f;
                    pivotAuthorityPocketDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                    pivotAuthoritySelectionDeltaGameUnits = sel.hasHitPoint
                        ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint)
                        : std::numeric_limits<float>::max();
                    resolvedAuthorityPivotSourceForFreeze =
                        grab_authority_frame_math::GrabAuthorityPivotSource::LooseWeaponPrimaryAttach;
                    resolvedAuthorityPivotReasonForFreeze = looseWeaponPrimaryAttachFrame.reason;
                    looseWeaponPrimaryAttachApplied = true;
                    _grabFrame.authoredLooseWeaponSupportGrip = looseWeaponPrimaryAttachFrame.supportGrip;
                    _grabFrame.authoredWeaponPose = looseWeaponPrimaryAttachFrame.pose;
                    outCapture.hasAuthoredHandRelation = nearAuthored;
                    if (nearAuthored) {
                        outCapture.authoredHandWeaponLocal = nearGrip.pose.handWeaponLocal;
                    }
                    ROCK_LOG_INFO(Hand, "{} hand loose authored grab ref={:08X} role={} source={}", handName(),
                        sel.refr ? sel.refr->GetFormID() : 0,
                        looseWeaponPrimaryAttachFrame.supportGrip ? "support" : "firing",
                        looseWeaponPrimaryAttachFrame.reason);
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
                    !looseWeaponPrimaryAttachApplied && !usingPinchPocket && pocket.valid && !grabMeshTriangles.empty();
                const auto seatLongAxis =
                    seatShapeEvaluable ? computeGrabMeshLongAxis(grabMeshTriangles) : GrabMeshLongAxisResult{};
                const bool seatRodShape = seatLongAxis.valid &&
                    seatLongAxis.elongationRatio >= g_rockConfig.rockPullPresentationMinElongationRatio;
                const bool seatFlatFace = seatLongAxis.valid &&
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
                            // Same hand-neutral, +X-toward-fingertips tilt as the flight servo.
                            RE::NiPoint3 targetAxisWorld =
                                grab_three_phase::buildGripPresentationAxisTowardFingertips(pocket.crossPalmWorld,
                                    pocket.fingerForwardWorld, g_rockConfig.rockPullPresentationGripAxisTiltDegrees);
                            if (lengthSquared(currentAxisWorld) > 0.000001f &&
                                lengthSquared(targetAxisWorld) > 0.000001f) {
                                if (dotProduct(currentAxisWorld, targetAxisWorld) < 0.0f) {
                                    targetAxisWorld =
                                        RE::NiPoint3{ -targetAxisWorld.x, -targetAxisWorld.y, -targetAxisWorld.z };
                                }
                                const RE::NiPoint3 rotationAxisRaw = crossProduct(currentAxisWorld, targetAxisWorld);
                                const float sinAngle = std::sqrt((std::max)(0.0f, lengthSquared(rotationAxisRaw)));
                                const float cosAngle =
                                    std::clamp(dotProduct(currentAxisWorld, targetAxisWorld), -1.0f, 1.0f);
                                const float angleRadians = std::atan2(sinAngle, cosAngle);
                                if (sinAngle > 0.000001f && angleRadians > 0.01f) {
                                    const float invSin = 1.0f / sinAngle;
                                    const RE::NiPoint3 rotationAxis{
                                        rotationAxisRaw.x * invSin,
                                        rotationAxisRaw.y * invSin,
                                        rotationAxisRaw.z * invSin,
                                    };
                                    seatBodyWorld = rotateTransformWorldAboutPoint(grabBodyWorldAtGrab, rotationAxis,
                                        angleRadians, grabGripPoint);
                                    seatObjectWorld = rotateTransformWorldAboutPoint(objectWorldTransform, rotationAxis,
                                        angleRadians, grabGripPoint);
                                    // Carry the mesh axes into the swung pose so the roll below runs in seat space.
                                    currentAxisWorld = grab_finger_pose_math::rotateAroundUnitAxis(currentAxisWorld,
                                        rotationAxis, angleRadians);
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
                            if (seatFlatFace && lengthSquared(currentAxisWorld) > 0.000001f &&
                                lengthSquared(currentSecondAxisWorld) > 0.000001f) {
                                // Thinnest-extent direction = the flat face's normal.
                                const RE::NiPoint3 faceNormalWorld =
                                    crossProduct(currentAxisWorld, currentSecondAxisWorld);
                                const RE::NiPoint3 rollTargetRaw = pocket.palmNormalWorld -
                                    currentAxisWorld * dotProduct(pocket.palmNormalWorld, currentAxisWorld);
                                const RE::NiPoint3 rollCurrentRaw =
                                    faceNormalWorld - currentAxisWorld * dotProduct(faceNormalWorld, currentAxisWorld);
                                RE::NiPoint3 rollTargetWorld = normalizeOrZero(rollTargetRaw);
                                const RE::NiPoint3 rollCurrentWorld = normalizeOrZero(rollCurrentRaw);
                                if (lengthSquared(rollTargetWorld) > 0.000001f &&
                                    lengthSquared(rollCurrentWorld) > 0.000001f) {
                                    if (dotProduct(rollCurrentWorld, rollTargetWorld) < 0.0f) {
                                        // A face has two sides; twist toward the nearer one.
                                        rollTargetWorld =
                                            RE::NiPoint3{ -rollTargetWorld.x, -rollTargetWorld.y, -rollTargetWorld.z };
                                    }
                                    const float rollSin =
                                        dotProduct(crossProduct(rollCurrentWorld, rollTargetWorld), currentAxisWorld);
                                    const float rollCos =
                                        std::clamp(dotProduct(rollCurrentWorld, rollTargetWorld), -1.0f, 1.0f);
                                    const float rollAngleRadians = std::atan2(rollSin, rollCos);
                                    if (std::fabs(rollAngleRadians) > 0.01f) {
                                        seatBodyWorld = rotateTransformWorldAboutPoint(seatBodyWorld, currentAxisWorld,
                                            rollAngleRadians, grabGripPoint);
                                        seatObjectWorld = rotateTransformWorldAboutPoint(seatObjectWorld,
                                            currentAxisWorld, rollAngleRadians, grabGripPoint);
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
                         * Close-grab seat mode 1 (iGrabCloseSeatMode): local grasp
                         * solve. The current seat keeps the object's rotation and
                         * slides the mesh point nearest the palm onto it, so a mug
                         * handle held in front of the palm is seated like any other
                         * point and the fingers stop on the mug body behind it. The
                         * whole-mesh shape class cannot help: a mug is compact as a
                         * whole and its handle is a bar only locally.
                         *
                         * This mode classifies the mesh inside a hand-sized window in
                         * front of the palm (GrabLocalGraspSolve.h). When it finds a
                         * bar, rim or round body there, that part's own point becomes
                         * pivot B and the object turns about it onto the hand's grip
                         * line. Pivot A moves from the palm centre toward the finger
                         * bases by what the part's width allows, because closed
                         * fingers trap a handle under their first segments and never
                         * reach one lying on the palm centre. The depth stop below
                         * then rests the turned surface on the palm exactly as it
                         * does for any other seat.
                         *
                         * Plain close grabs of loose objects only. Pulled, forced,
                         * pinched, authored, weapon and shared grabs keep their own
                         * seat authority, and an unrecognised window keeps the current
                         * seat: the mode changes how an object is placed, never whether
                         * it is grabbed. One bounded pass at capture.
                         */
                grab_local_grasp::Result localGrasp{};
                const char* localGraspReason = "modeOff";
                bool localGraspApplied = false;
                // In-plane distance of pivot A from the palm centre. Zero
                // for every seat except a local grasp seat.
                RE::NiPoint3 seatPalmAnchorOffsetWorld{};
                float localGraspSeatForwardGameUnits = 0.0f;
                if (g_rockConfig.rockGrabCloseSeatMode == 1) {
                    const auto localGraspStart = std::chrono::steady_clock::now();
                    const std::size_t fingerTrianglesBefore = _grabFrame.fingerPoseLocalMeshTriangles.size();
                    grab_local_grasp::NearestPoint localGraspSeed{};
                    const bool plainCloseGrab = !programmaticArrival && !nearAuthored && !looseWeaponGrab &&
                        !input.joiningPeerHeldObject && sel.targetKind == grab_target::Kind::LooseObject;
                    if (!plainCloseGrab) {
                        localGraspReason = "notPlainCloseGrab";
                    } else if (!seatShapeEvaluable || grabLocalMeshTriangles.empty()) {
                        localGraspReason = "seatNotEvaluable";
                    } else {
                        // Pocket frame: x along the fingers, y across the palm,
                        // z out of the palm, origin at the palm centre.
                        const auto vectorToPocket = [&](const RE::NiPoint3& vectorWorld) {
                            return grab_local_grasp::Vec3{
                                dotProduct(vectorWorld, pocket.fingerForwardWorld),
                                dotProduct(vectorWorld, pocket.crossPalmWorld),
                                dotProduct(vectorWorld, pocket.palmNormalWorld),
                            };
                        };
                        const auto vectorToWorld = [&](const grab_local_grasp::Vec3& vectorPocket) {
                            return pocket.fingerForwardWorld * vectorPocket.x + pocket.crossPalmWorld * vectorPocket.y +
                                pocket.palmNormalWorld * vectorPocket.z;
                        };
                        /*
                                 * Node-local to pocket is affine, so it is sampled
                                 * once instead of composing two transforms per
                                 * vertex. The long basis keeps the float error of
                                 * far-from-origin world coordinates out of the map.
                                 */
                        constexpr float kBasisLength = 64.0f;
                        const RE::NiPoint3 nodeOriginWorld =
                            transform_math::localPointToWorld(objectWorldTransform, RE::NiPoint3{ 0.0f, 0.0f, 0.0f });
                        const auto pocketOrigin = vectorToPocket(nodeOriginWorld - pocket.palmCenterWorld);
                        const auto pocketColumn = [&](const RE::NiPoint3& axisLocal) {
                            const auto image = vectorToPocket(
                                transform_math::localPointToWorld(objectWorldTransform, axisLocal * kBasisLength) -
                                nodeOriginWorld);
                            return grab_local_grasp::Vec3{ image.x / kBasisLength, image.y / kBasisLength,
                                image.z / kBasisLength };
                        };
                        const auto pocketFromX = pocketColumn(RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
                        const auto pocketFromY = pocketColumn(RE::NiPoint3{ 0.0f, 1.0f, 0.0f });
                        const auto pocketFromZ = pocketColumn(RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
                        const auto localToPocket = [&](const RE::NiPoint3& pointLocal) {
                            return grab_local_grasp::Vec3{
                                pocketOrigin.x + pocketFromX.x * pointLocal.x + pocketFromY.x * pointLocal.y +
                                    pocketFromZ.x * pointLocal.z,
                                pocketOrigin.y + pocketFromX.y * pointLocal.x + pocketFromY.y * pointLocal.y +
                                    pocketFromZ.y * pointLocal.z,
                                pocketOrigin.z + pocketFromX.z * pointLocal.x + pocketFromY.z * pointLocal.y +
                                    pocketFromZ.z * pointLocal.z,
                            };
                        };

                        // Same hand-neutral grip line the pull presentation seats rods on.
                        const auto gripLinePocket = vectorToPocket(
                            grab_three_phase::buildGripPresentationAxisTowardFingertips(pocket.crossPalmWorld,
                                pocket.fingerForwardWorld, g_rockConfig.rockPullPresentationGripAxisTiltDegrees));
                        localGraspSeed = grab_local_grasp::nearestPointToOrigin(grabLocalMeshTriangles, localToPocket);
                        const float behindPalmTolerance =
                            (std::max)(0.0f, g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits);
                        if (!localGraspSeed.found) {
                            localGraspReason = "noFiniteMesh";
                        } else if (localGraspSeed.point.z < -behindPalmTolerance ||
                            std::hypot(localGraspSeed.point.x, localGraspSeed.point.y) > pocket.pocketRadiusGameUnits) {
                            // The window is read along the palm normal, which
                            // only describes mesh the palm actually faces.
                            localGraspReason = "nearestMeshNotInFrontOfPalm";
                        } else {
                            localGrasp = grab_local_grasp::solve(grabLocalMeshTriangles, localGraspSeed.point,
                                gripLinePocket, grab_local_grasp::Config{}, localToPocket);
                            localGraspReason = localGrasp.reason;
                        }

                        const RE::NiPoint3 localGraspPivotWorld =
                            pocket.palmCenterWorld + vectorToWorld(localGrasp.pivot);
                        if (localGrasp.valid && localGrasp.kind != grab_local_grasp::Kind::None &&
                            grab_three_phase::isFinite(localGraspPivotWorld)) {
                            grabGripPoint = localGraspPivotWorld;
                            if (localGrasp.hasAlignment) {
                                const RE::NiPoint3 fromWorld = normalizeOrZero(vectorToWorld(localGrasp.alignFrom));
                                const RE::NiPoint3 toWorld = normalizeOrZero(vectorToWorld(localGrasp.alignTo));
                                const RE::NiPoint3 rotationAxisRaw = crossProduct(fromWorld, toWorld);
                                const float sinAngle = std::sqrt((std::max)(0.0f, lengthSquared(rotationAxisRaw)));
                                const float cosAngle = std::clamp(dotProduct(fromWorld, toWorld), -1.0f, 1.0f);
                                const float angleRadians = std::atan2(sinAngle, cosAngle);
                                if (sinAngle > 0.000001f && angleRadians > 0.01f) {
                                    const float invSin = 1.0f / sinAngle;
                                    const RE::NiPoint3 rotationAxis{
                                        rotationAxisRaw.x * invSin,
                                        rotationAxisRaw.y * invSin,
                                        rotationAxisRaw.z * invSin,
                                    };
                                    // About the new pivot, so the pivot pair is untouched.
                                    seatBodyWorld = rotateTransformWorldAboutPoint(seatBodyWorld, rotationAxis,
                                        angleRadians, grabGripPoint);
                                    seatObjectWorld = rotateTransformWorldAboutPoint(seatObjectWorld, rotationAxis,
                                        angleRadians, grabGripPoint);
                                    seatAlignmentAngleDegrees = angleRadians * 57.29577951308232f;
                                }
                            }
                            seatAlignmentReason = localGrasp.reason;

                            /*
                                     * Finger base joints from the created hand
                                     * colliders. A base joint does not move when its
                                     * finger curls, so the pose at grip press does
                                     * not matter. The joint is the first segment's
                                     * end away from the second segment; segment
                                     * centres are used so no collider axis
                                     * convention is involved.
                                     */
                            {
                                using hand_collider_semantics::HandColliderRole;
                                constexpr std::array<std::pair<HandColliderRole, HandColliderRole>, 4>
                                    kFingerBaseSegments{ {
                                        { HandColliderRole::IndexBase, HandColliderRole::IndexMiddle },
                                        { HandColliderRole::MiddleBase, HandColliderRole::MiddleMiddle },
                                        { HandColliderRole::RingBase, HandColliderRole::RingMiddle },
                                        { HandColliderRole::PinkyBase, HandColliderRole::PinkyMiddle },
                                    } };
                                const auto findSegment =
                                    [&](HandColliderRole role) -> const HandBoneColliderSet::PublishedSegmentFrame* {
                                    for (const auto& segment : _boneColliders.segmentColliderFrames()) {
                                        if (segment.valid && segment.role == role &&
                                            grab_three_phase::isFinite(segment.target) &&
                                            std::isfinite(segment.length) && segment.length > 0.0f &&
                                            std::isfinite(segment.radius) && segment.radius > 0.0f) {
                                            return &segment;
                                        }
                                    }
                                    return nullptr;
                                };
                                RE::NiPoint3 baseJointSumWorld{};
                                float fingerRadiusSum = 0.0f;
                                int fingerCount = 0;
                                for (const auto& [baseRole, middleRole] : kFingerBaseSegments) {
                                    const auto* base = findSegment(baseRole);
                                    const auto* middle = findSegment(middleRole);
                                    if (!base || !middle) {
                                        continue;
                                    }
                                    const RE::NiPoint3 towardPalm =
                                        normalizeOrZero(base->target.translate - middle->target.translate);
                                    if (lengthSquared(towardPalm) <= 0.000001f) {
                                        continue;
                                    }
                                    baseJointSumWorld =
                                        baseJointSumWorld + base->target.translate + towardPalm * (base->length * 0.5f);
                                    fingerRadiusSum += base->radius;
                                    ++fingerCount;
                                }
                                if (fingerCount > 0) {
                                    const float inverseCount = 1.0f / static_cast<float>(fingerCount);
                                    localGraspSeatForwardGameUnits = grab_local_grasp::fingerBaseSeat(localGrasp,
                                        vectorToPocket(baseJointSumWorld * inverseCount - pocket.palmCenterWorld),
                                        fingerRadiusSum * inverseCount, gripLinePocket);
                                    seatPalmAnchorOffsetWorld =
                                        pocket.fingerForwardWorld * localGraspSeatForwardGameUnits;
                                    grabPivotAWorld = pocket.palmCenterWorld + seatPalmAnchorOffsetWorld;
                                }
                            }
                            gripArea.contactSeedWorld = grabGripPoint;
                            gripEvidencePointWorld = grabGripPoint;
                            _grabObjectGripAtGrab.contactSeedWorld = grabGripPoint;
                            _grabObjectGripAtGrab.gripCenterWorld = grabGripPoint;
                            pivotAuthorityPocketDistanceGameUnits =
                                pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                            pivotAuthoritySelectionDeltaGameUnits = sel.hasHitPoint
                                ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint)
                                : std::numeric_limits<float>::max();
                            desiredBodyWorld = grab_frame_math::shiftObjectToAlignGripWithPocket(seatBodyWorld,
                                grabPivotAWorld, grabGripPoint);
                            desiredObjectWorld = deriveNodeWorldFromBodyWorld(desiredBodyWorld, objectToBodyAtGrab);
                            seatPoseChanged = true;
                            localGraspApplied = true;

                            if (localGrasp.threadFingers) {
                                /*
                                         * Finger closure stops at first contact. With
                                         * the body behind a handle loop still in its
                                         * mesh, the fingers stop on that body on their
                                         * way in and never reach the handle. Drop the
                                         * triangles wholly behind the gap from closure
                                         * only; seating and collision keep the full
                                         * mesh. An emptied list would make closure fall
                                         * back to that full mesh, so keep it populated.
                                         */
                                auto& fingerTriangles = _grabFrame.fingerPoseLocalMeshTriangles;
                                const auto behindGap = [&](const GrabLocalTriangle& triangle) {
                                    return grab_local_grasp::behindThreadGap(localGrasp, localToPocket(triangle.v0),
                                        localToPocket(triangle.v1), localToPocket(triangle.v2));
                                };
                                const auto droppedCount = static_cast<std::size_t>(
                                    std::count_if(fingerTriangles.begin(), fingerTriangles.end(), behindGap));
                                constexpr std::size_t kMinFingerClosureTriangles = 8;
                                if (droppedCount > 0 &&
                                    fingerTriangles.size() - droppedCount >= kMinFingerClosureTriangles) {
                                    std::erase_if(fingerTriangles, behindGap);
                                }
                            }
                        }
                    }
                    const float localGraspMicroseconds =
                        static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now() - localGraspStart)
                                .count());
                    // Applied-mode status: one line per grab while the mode is on.
                    ROCK_LOG_INFO(Hand,
                        "{} CLOSE GRAB SEAT: mode=1 localGrasp applied={} part={} reason={} turn={:.1f}deg "
                        "towardFingers={:.2f}gu seed=({:.2f},{:.2f},{:.2f}) "
                        "pivot=({:.2f},{:.2f},{:.2f}) width={:.2f} thickness={:.2f} length={:.2f} gapBehind={:.2f} "
                        "slide={:.2f} "
                        "elongation={:.2f} thread={} fingerTris={}->{} samples={} near={} meshTris={} us={:.0f} "
                        "ref={:08X}",
                        handName(), localGraspApplied ? "yes" : "no", grab_local_grasp::kindName(localGrasp.kind),
                        localGraspReason, localGraspApplied ? seatAlignmentAngleDegrees : 0.0f,
                        localGraspSeatForwardGameUnits, localGraspSeed.point.x, localGraspSeed.point.y,
                        localGraspSeed.point.z, localGrasp.pivot.x, localGrasp.pivot.y, localGrasp.pivot.z,
                        localGrasp.widthGameUnits, localGrasp.thicknessGameUnits, localGrasp.lengthGameUnits,
                        localGrasp.gapBehindGameUnits, localGrasp.stationShiftGameUnits, localGrasp.elongation,
                        localGrasp.threadFingers ? "yes" : "no", fingerTrianglesBefore,
                        _grabFrame.fingerPoseLocalMeshTriangles.size(), localGrasp.sampleCount,
                        localGrasp.nearSampleCount, grabLocalMeshTriangles.size(), localGraspMicroseconds,
                        sel.refr ? sel.refr->GetFormID() : 0);
                }

                /*
                         * Seat depth stop. The freeze aligns the grip point onto
                         * pivot A, which can pull mesh behind that point through
                         * the palm. Measure the final seat pose from the palm
                         * plane itself: any mesh past the plane inside the palm
                         * footprint is penetration, and one push along the palm
                         * normal removes it exactly (a translation lowers every
                         * sample depth by the same amount). Plates use their
                         * wider edge footprint. Weapon attach frames and pinch
                         * pockets keep their own seat.
                         */
                GrabSeatDepthStopResult seatDepthStop{};
                float seatDepthOffsetGameUnits = 0.0f;
                if (!looseWeaponPrimaryAttachApplied && !usingPinchPocket && pocket.valid) {
                    if (seatPoseChanged) {
                        desiredBodyWorld = grab_frame_math::shiftObjectToAlignGripWithPocket(seatBodyWorld,
                            grabPivotAWorld, grabGripPoint);
                        desiredObjectWorld = deriveNodeWorldFromBodyWorld(desiredBodyWorld, objectToBodyAtGrab);
                    }
                    const float seatFootprintRadiusGameUnits = seatPlateShape
                        ? g_rockConfig.rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits
                        : g_rockConfig.rockGrabSeatDepthFootprintRadiusGameUnits;
                    seatDepthStop = computeGrabPalmSeatDepthStop(grabLocalMeshTriangles, desiredObjectWorld,
                        pocket.palmCenterWorld, pocket, _boneColliders.segmentColliderFrames(),
                        seatFootprintRadiusGameUnits, g_rockConfig.rockGrabSeatDepthMaxGameUnits);
                    if (seatDepthStop.valid && seatDepthStop.depthGameUnits > 0.01f) {
                        seatDepthOffsetGameUnits = seatDepthStop.depthGameUnits +
                            (std::max)(0.0f, g_rockConfig.rockGrabSeatDepthSkinGameUnits);
                        grabPivotAWorld = grabPivotAWorld + pocket.palmNormalWorld * seatDepthOffsetGameUnits;
                        desiredBodyWorld = grab_frame_math::shiftObjectToAlignGripWithPocket(seatBodyWorld,
                            grabPivotAWorld, grabGripPoint);
                        desiredObjectWorld = deriveNodeWorldFromBodyWorld(desiredBodyWorld, objectToBodyAtGrab);
                    }
                }

                selectedGripPointLocal = transform_math::worldPointToLocal(objectWorldTransform, grabGripPoint);
                selectedPivotBBodyLocalGame = transform_math::worldPointToLocal(grabBodyWorldAtGrab, grabGripPoint);
                const bool effectivePinchPocket = usingPinchPocket && !looseWeaponPrimaryAttachApplied;
                /*
                         * Seat diagnostics: the classification and every
                         * correction outcome, kept past the log line so a saved
                         * ground-truth capture can record the seat ROCK produced
                         * for this grab next to the pose the user corrected it to.
                         */
                _grabFrame.seat.diagnostics = GrabSeatDiagnostics{
                    .acquisitionMode =
                        sel.forcedArrival ? "forceGrab" : (grabbedFromPullCatch ? "pullCatch" : "closeGrab"),
                    .shapeClass =
                        seatPlateShape ? "plate" : (seatRodShape ? "rod" : (seatLongAxis.valid ? "compact" : "none")),
                    .elongationRatio = seatLongAxis.elongationRatio,
                    .secondElongationRatio = seatLongAxis.secondElongationRatio,
                    .alignmentAngleDegrees = seatAlignmentAngleDegrees,
                    .alignmentReason = seatAlignmentReason,
                    .rollAngleDegrees = seatRollAngleDegrees,
                    .rollReason = seatRollReason,
                    .depthGameUnits = seatDepthStop.depthGameUnits,
                    .depthOffsetGameUnits = seatDepthOffsetGameUnits,
                    .depthReason = seatDepthStop.reason,
                };
                _grabFrame.gripEvidence.gripEvidenceLocal =
                    transform_math::worldPointToLocal(objectWorldTransform, gripEvidencePointWorld);
                _grabFrame.gripEvidence.gripNormalLocal =
                    transform_math::worldVectorToLocal(objectWorldTransform, gripNormalWorld);
                storeGripSourceEvidence(_grabFrame,
                    grabSurfaceHit.sourceNode ? grabSurfaceHit.sourceNode : collidableNode, objectWorldTransform,
                    gripEvidencePointWorld, gripEvidenceNormalWorld,
                    lengthSquared(gripEvidenceNormalWorld) > 0.000001f);
                if (looseWeaponPrimaryAttachApplied) {
                    _grabFrame.gripEvidence.gripSourceNode = nullptr;
                    _grabFrame.gripEvidence.gripPointSourceNodeLocal = {};
                    _grabFrame.gripEvidence.gripNormalSourceNodeLocal = {};
                    _grabFrame.gripEvidence.hasGripSourceNodePoint = false;
                    _grabFrame.gripEvidence.hasGripSourceNodeNormal = false;
                    _grabFrame.gripEvidence.gripEvidenceTriangleIndex = 0xFFFF'FFFF;
                    _grabFrame.gripEvidence.gripEvidenceShapeKey = 0xFFFF'FFFF;
                    _grabFrame.gripEvidence.gripEvidenceShapeCollisionFilterInfo = 0;
                    _grabFrame.gripEvidence.gripEvidenceHitFraction = 1.0f;
                    _grabFrame.gripEvidence.hasGripEvidenceShapeKey = false;
                }
                _grabFrame.pivotAuthority.pocketDistanceGameUnits =
                    pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                _grabFrame.pivotAuthority.selectionDistanceGameUnits = sel.hasHitPoint
                    ? pointDistanceGameUnits(sel.hitPointWorld, gripArea.contactSeedWorld)
                    : std::numeric_limits<float>::max();
                _grabFrame.authority.grabPivotWorldAtGrab = grabPivotAWorld;
                _grabFrame.gripEvidence.gripPointWorldAtGrab = grabGripPoint;
                _grabFrame.seat.activeGrabPointMode = grabPointMode;
                _grabFrame.seat.mode = usingPinchPocket ? GrabSeatMode::PinchPocket : GrabSeatMode::SupportGroup;
                _grabFrame.seat.programmaticArrival = programmaticArrival;
                _grabFrame.seat.localGraspSeat = localGraspApplied;
                if (looseWeaponPrimaryAttachApplied) {
                    _grabFrame.seat.mode = GrabSeatMode::SupportGroup;
                }
                _grabFrame.seat.hasPinchPocket = effectivePinchPocket;
                _grabFrame.seat.pinchFingerPose = pinchPocketCandidate.closure.sample.pose;
                _grabFrame.seat.pinchPocketWorldAtGrab =
                    effectivePinchPocket ? pinchPocketCandidate.pinchPocketWorld : RE::NiPoint3{};
                _grabFrame.seat.pinchAxisWorldAtGrab =
                    effectivePinchPocket ? pinchPocketCandidate.pinchAxisWorld : RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
                _grabFrame.seat.palmSeatPointWorldAtGrab =
                    effectivePinchPocket ? grabGripPoint : pocket.palmCenterWorld;
                _grabFrame.seat.hasPalmSeatPoint = true;
                _grabFrame.seat.palmSeatPointMode = effectivePinchPocket ? "pinchPocket" : "threePhasePocket";
                _grabFrame.pivotAuthority.source = pivotAuthoritySource;
                _grabFrame.pivotAuthority.positionOnly = pivotAuthorityPositionOnly;
                _grabFrame.pivotAuthority.normalTrusted = pivotAuthorityNormalTrusted;
                _grabFrame.pivotAuthority.positionConfidence = pivotAuthorityPositionConfidence;
                _grabFrame.syntheticLooseWeaponPrimaryAttach = looseWeaponPrimaryAttachApplied;
                _grabFrame.authoredWeaponArrangement =
                    authoredWeapon ? nearGrip.arrangement : loose_weapon_authored_grab_policy::Arrangement::Pending;
                if (looseWeaponPrimaryAttachApplied) {
                    _grabFrame.seat.hasSettledVisualHandRelation = false;
                }
                _grabFrame.seat.requiresSettledVisualHandRelation = looseWeaponPrimaryAttachApplied
                    ? true
                    : (!effectivePinchPocket && arrivalSeatSafety.requireSettledVisualRelation);
                _grabFrame.fingerPoseAimValid = true;
                _grabFrame.fingerPoseAimReason =
                    effectivePinchPocket ? "pinchPocketThumbIndexTargets" : "rockPointToPalmEvidence";
                /*
                         * An arrival that must settle first solves its final
                         * fingers at TouchHeld, against the arrived pose. Other
                         * captures solve against the frozen target before the
                         * first approach frame.
                         */
                const bool fullHeldAuthorityAtCapture =
                    _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld;

                const auto captureFingerTargets =
                    buildRuntimeFingerPoseTargets(gripEvidencePointWorld, gripEvidenceNormalWorld);
                storeFingerPoseTargetsInGrabFrame(_grabFrame, captureFingerTargets, objectWorldTransform);

                ROCK_LOG_DEBUG(Hand,
                    "{} THREE-PHASE GRAB CAPTURE: relation={} seat={} rotation={} phase={} reason={} touchContact={} "
                    "stableTouch={} pocket=({:.1f},{:.1f},{:.1f}) "
                    "palm=({:.1f},{:.1f},{:.1f}) normal=({:.3f},{:.3f},{:.3f}) seed=({:.1f},{:.1f},{:.1f}) "
                    "grip=({:.1f},{:.1f},{:.1f}) gripLocal=({:.2f},{:.2f},{:.2f}) pivotB=({:.2f},{:.2f},{:.2f}) "
                    "dist={:.1f} signedPalm={:.1f} "
                    "fullHeldAuthority={} pivotAuthoritySource={} positionOnlyPatch={} normalTrusted={} "
                    "settledVisualRequired={} arrivalSeatSafety={} arrivalSeatDot={:.3f} arrivalSeatSigned={:.1f} "
                    "arrivalSeatDist={:.1f} seatDepth={:.2f} seatDepthOffset={:.2f} seatDepthSamples={} "
                    "seatDepthReason={} seatAlignDeg={:.1f} seatAlignReason={} seatRollDeg={:.1f} seatRollReason={} "
                    "seatShape={} seatRatio12={:.2f} seatRatio23={:.2f} inset={:.2f} insetSource={} "
                    "looseWeaponPrimaryAttach={} attachReason={} attachVisible={}",
                    handName(), relationMode, grabSeatModeName(_grabFrame.seat.mode),
                    looseWeaponPrimaryAttachApplied ? "primaryWeaponAttach"
                                                    : (localGraspApplied ? "localGrasp" : "preserve"),
                    grab_three_phase::phaseName(_grabAcquisitionPhase), captureReason,
                    semanticContacts.count > 0 ? "yes" : "no", hasStablePocketTouchContact ? "yes" : "no",
                    pocket.pocketCenterWorld.x, pocket.pocketCenterWorld.y, pocket.pocketCenterWorld.z,
                    pocket.palmCenterWorld.x, pocket.palmCenterWorld.y, pocket.palmCenterWorld.z, gripNormalWorld.x,
                    gripNormalWorld.y, gripNormalWorld.z, gripArea.contactSeedWorld.x, gripArea.contactSeedWorld.y,
                    gripArea.contactSeedWorld.z, grabGripPoint.x, grabGripPoint.y, grabGripPoint.z,
                    selectedGripPointLocal.x, selectedGripPointLocal.y, selectedGripPointLocal.z,
                    selectedPivotBBodyLocalGame.x, selectedPivotBBodyLocalGame.y, selectedPivotBBodyLocalGame.z,
                    usingPinchPocket ? pinchPocketCandidate.pocketToSurfaceDistanceGameUnits
                                     : finalGripToPocketDistance,
                    usingPinchPocket ? 0.0f : finalSignedPalmDistance, fullHeldAuthorityAtCapture ? "yes" : "no",
                    grabPivotAuthoritySourceName(_grabFrame.pivotAuthority.source),
                    _grabFrame.pivotAuthority.positionOnly ? "yes" : "no",
                    _grabFrame.pivotAuthority.normalTrusted ? "yes" : "no",
                    _grabFrame.seat.requiresSettledVisualHandRelation ? "yes" : "no", arrivalSeatSafety.reason,
                    arrivalSeatSafety.normalDotPalm, finalSignedPalmDistance, finalGripToPocketDistance,
                    seatDepthStop.depthGameUnits, seatDepthOffsetGameUnits, seatDepthStop.footprintSampleCount,
                    seatDepthStop.reason, seatAlignmentAngleDegrees, seatAlignmentReason, seatRollAngleDegrees,
                    seatRollReason,
                    seatPlateShape ? "plate" : (seatRodShape ? "rod" : (seatLongAxis.valid ? "compact" : "none")),
                    seatLongAxis.elongationRatio, seatLongAxis.secondElongationRatio, gripArea.seedInsetGameUnits,
                    gripArea.fallbackReason,
                    looseWeaponPrimaryAttachApplied ? "yes" : "no", looseWeaponPrimaryAttachReason,
                    looseWeaponPrimaryAttachSourceVisible ? "yes" : "no");
            } else {
                ROCK_LOG_WARN(Hand,
                    "{} THREE-PHASE GRAB ABORT: pocketValid={} gripValid={} accepted={} reason={} dist={:.2f}gu "
                    "stableTouch={}",
                    handName(), pocket.valid ? "yes" : "no", gripArea.valid ? "yes" : "no",
                    phaseDecision.accepted ? "yes" : "no", phaseDecision.reason, gripToPocketDistance,
                    hasStablePocketTouchContact ? "yes" : "no");
                return abortCapture();
            }
        }

        return true;
    }

}
