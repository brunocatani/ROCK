// The capsule finger solve run against the selected object before a grab: the
// probe contact check every close grab passes (evaluateGrabProbeContact) and
// the probe visualizer (DebugOverlay.DebugDrawGrabFingerProbes) share it, so
// the drawn probes are exactly what the check saw.
#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/hand/HandGrabInternal.h"

#include "RockConfig.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/grab/GrabFingerCapsuleSweep.h"
#include "physics-interaction/grab/GrabFingerProbePreview.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/visual/FrikHandWorldAuthority.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace rock
{
    using namespace hand_grab_internal;

    namespace
    {
        namespace preview = grab_finger_probe_preview;
        namespace sweep = grab_finger_capsule_sweep;

        // Bound on the one-time copy of a selected object's mesh.
        constexpr std::size_t kMaxPreviewTriangles = 250'000;
        // A mesh above the solve index's capacity keeps the triangles nearest
        // the hand; they are re-picked once the hand moves this far (object-local).
        constexpr float kNearestRebuildDistance = 4.0f;

        // The options solveAndPublishGrabFingerPose uses.
        sweep::Options solveOptions()
        {
            return sweep::Options{
                .minValue = g_rockConfig.rockGrabFingerMinValue,
            };
        }

        preview::FingerState stateOf(const sweep::FingerResult& result)
        {
            if (result.blocked) {
                return preview::FingerState::Blocked;
            }
            if (result.closedPastIntersection) {
                return preview::FingerState::ClosedPastIntersection;
            }
            return result.contact ? preview::FingerState::Contact : preview::FingerState::Free;
        }

        void appendFingerSpheres(preview::Snapshot& out, const sweep::FingerChainPose& pose, std::size_t finger,
            const sweep::CapsuleRadii& radii, const grab_finger_pose_runtime::FingerPoseTriangleSpatialIndex* index,
            const RE::NiTransform& objectWorld)
        {
            (void)sweep::forEachCapsuleSphere(pose, radii, [&](const RE::NiPoint3& center, float radius, std::size_t) {
                if (out.sphereCount >= out.spheres.size()) {
                    return true;
                }
                auto& sphere = out.spheres[out.sphereCount++];
                sphere.center = center;
                sphere.radius = radius;
                sphere.finger = static_cast<std::uint8_t>(finger);
                sphere.touching = index && !index->empty() &&
                    index->querySphereWorld(objectWorld, center, radius + preview::kTouchMarginGameUnits, nullptr,
                        nullptr);
                return false;
            });
        }

        /*
         * The drawn finger skeleton placed at handWorld, read as
         * logGrabPoseQuality reads it: the chain relative to its own wrist,
         * moved from the raw hand onto the hand that is drawn.
         */
        bool resolveDrawnFingers(bool isLeft, const RE::NiTransform* heldVisualHand,
            root_flattened_finger_skeleton_runtime::Snapshot& skeleton, RE::NiTransform& handWorld)
        {
            RE::NiTransform rawHand{};
            if (!root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(isLeft, skeleton) ||
                !frik_hand_world_authority::tryGetRawHandWorld(isLeft, rawHand)) {
                return false;
            }
            if (heldVisualHand) {
                handWorld = *heldVisualHand;
            } else if (!frik_hand_world_authority::tryGetPresentedHandWorld(isLeft, handWorld)) {
                handWorld = rawHand;
            }
            return sweep::rebaseSkeleton(skeleton, rawHand, handWorld);
        }

        RE::NiPoint3 labelAnchorFor(const root_flattened_finger_skeleton_runtime::Snapshot& skeleton,
            const RE::NiTransform& handWorld)
        {
            // Above the back of the hand, clear of the fingers and the object.
            constexpr float kLabelLiftGameUnits = 7.0f;
            return skeleton.palmNormalValid ? handWorld.translate - skeleton.palmNormalWorld * kLabelLiftGameUnits
                                            : handWorld.translate;
        }

        // How long a mesh copy outlives the hand's close selection on its object.
        constexpr float kUnusedKeepSeconds = 0.5f;
        constexpr std::size_t kMaxShapePlacements = 2048;

        // The shapes extractAllTriangles reads under node, in the same order,
        // each placed in the object node's space. False when the list overflows.
        template <class ShapePlacement>
        bool collectShapePlacements(RE::NiAVObject* node, const RE::NiTransform& worldToObject, int maxDepth,
            std::vector<ShapePlacement>& out)
        {
            if (!node || maxDepth <= 0 || (node->flags.flags & 1)) {
                return true;
            }
            if (auto* triShape = node->IsTriShape()) {
                if (out.size() >= kMaxShapePlacements) {
                    return false;
                }
                out.push_back(ShapePlacement{ reinterpret_cast<std::uintptr_t>(triShape),
                    transform_math::composeTransforms(worldToObject, triShape->world) });
                return true;
            }
            if (auto* niNode = node->IsNode()) {
                auto& kids = niNode->GetRuntimeData().children;
                for (auto i = decltype(kids.size()){ 0 }; i < kids.size(); i++) {
                    if (!collectShapePlacements(kids[i].get(), worldToObject, maxDepth - 1, out)) {
                        return false;
                    }
                }
            }
            return true;
        }

        // Rounding of a rigidly moving object's world transforms stays far inside
        // these; a part that moves against the object node does not.
        bool samePlacement(const RE::NiTransform& a, const RE::NiTransform& b)
        {
            constexpr float kTranslation = 0.02f;
            constexpr float kRotation = 0.001f;
            constexpr float kScale = 0.0005f;
            if (!(std::abs(a.translate.x - b.translate.x) <= kTranslation) ||
                !(std::abs(a.translate.y - b.translate.y) <= kTranslation) ||
                !(std::abs(a.translate.z - b.translate.z) <= kTranslation) ||
                !(std::abs(a.scale - b.scale) <= kScale)) {
                return false;
            }
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!(std::abs(a.rotate.entry[row][column] - b.rotate.entry[row][column]) <= kRotation)) {
                        return false;
                    }
                }
            }
            return true;
        }

        template <class ShapePlacement>
        bool samePlacements(const std::vector<ShapePlacement>& a, const std::vector<ShapePlacement>& b)
        {
            if (a.size() != b.size()) {
                return false;
            }
            for (std::size_t i = 0; i < a.size(); ++i) {
                if (a[i].shape != b[i].shape || !samePlacement(a[i].inObject, b[i].inObject)) {
                    return false;
                }
            }
            return true;
        }

        bool rebuildNearestIndex(std::vector<GrabLocalTriangle>& triangles,
            std::vector<std::pair<float, std::uint32_t>>& scratch,
            grab_finger_pose_runtime::FingerPoseTriangleSpatialIndex& index, const RE::NiPoint3& centerLocal)
        {
            constexpr std::size_t kCapacity = grab_finger_pose_runtime::kMaxFingerPoseCandidateTriangles;
            scratch.clear();
            scratch.reserve(triangles.size());
            for (std::uint32_t i = 0; i < triangles.size(); ++i) {
                const auto& t = triangles[i];
                float distanceSquared = 0.0f;
                (void)closestPointOnTriangleToPoint(centerLocal, TriangleData{ t.v0, t.v1, t.v2 }, distanceSquared);
                scratch.emplace_back(distanceSquared, i);
            }
            const std::size_t keep = (std::min)(kCapacity, scratch.size());
            std::nth_element(scratch.begin(), scratch.begin() + keep, scratch.end());
            std::vector<GrabLocalTriangle> nearest;
            nearest.reserve(keep);
            for (std::size_t i = 0; i < keep; ++i) {
                nearest.push_back(triangles[scratch[i].second]);
            }
            return index.buildFromLocalTriangles(nearest);
        }
    }

    bool Hand::buildGrabFingerProbePreview(preview::Snapshot& out)
    {
        out = {};
        const sweep::Options options = solveOptions();

        if (isHolding()) {
            // While holding: the spheres along the fingers as they are drawn.
            if (_grabFingerProbePreview.extracted) {
                _grabFingerProbePreview.clear();
            }
            if (!_grabFrame.heldNode || !_hasGrabVisualHandTransform) {
                return false;
            }
            root_flattened_finger_skeleton_runtime::Snapshot skeleton{};
            RE::NiTransform handWorld{};
            if (!resolveDrawnFingers(_isLeft, &_grabVisualHandTransform, skeleton, handWorld)) {
                return false;
            }
            const RE::NiTransform objectWorld = _grabFrame.heldNode->world;
            const auto* index = _grabFingerTriangleIndex.empty() ? nullptr : &_grabFingerTriangleIndex;
            for (std::size_t finger = 0; finger < 5; ++finger) {
                const auto& chain = skeleton.fingers[finger];
                if (!chain.valid) {
                    continue;
                }
                const RE::NiPoint3 tipEnd = chain.tipGeometryValid
                    ? chain.points[2] + (chain.tipSegmentCenterWorld - chain.points[2]) * 2.0f
                    : chain.points[2];
                const sweep::FingerChainPose pose{ { chain.points[0], chain.points[1], chain.points[2], tipEnd } };
                appendFingerSpheres(out, pose, finger, options.radii, index, objectWorld);

                // The capsule result is not kept after publication; contact and
                // curl recover its outcome (a blocked finger sits at the open
                // pose without contact, a free one at the minimum curl). A
                // finger closed past an intersection is shown as a contact.
                auto& state = out.fingers[finger];
                const bool contact = ((_grabFingerPose.contactValidMask >> finger) & 1u) != 0;
                state.curl = _grabFingerPose.values[finger];
                state.state = contact ? preview::FingerState::Contact
                                      : (state.curl >= 0.999f ? preview::FingerState::Blocked
                                                              : preview::FingerState::Free);
                state.valid = true;
            }
            out.triangleCount = index ? static_cast<std::uint32_t>(index->triangleCount()) : 0;
            out.labelAnchor = labelAnchorFor(skeleton, handWorld);
            out.holding = true;
            out.valid = out.sphereCount > 0;
            return out.valid;
        }

        if (!hasSelection() || !_currentSelection.refr || _currentSelection.isFarSelection) {
            if (_grabFingerProbePreview.extracted) {
                _grabFingerProbePreview.clear();
            }
            return false;
        }
        return solveSelectedObjectProbes(out, false);
    }

    bool Hand::solveSelectedObjectProbes(preview::Snapshot& out, bool refreshDeformingMesh, bool commitShare,
        bool handPocketOnlyGrab)
    {
        out = {};
        _grabSharedExtraction.clear();
        const sweep::Options options = solveOptions();
        auto& cache = _grabFingerProbePreview;
        const auto& selection = _currentSelection;
        if (!hasSelection() || !selection.refr || selection.isFarSelection) {
            return false;
        }
        auto* root = selection.refr->Get3D();
        auto* objectNode = selection.hitNode ? selection.hitNode : root;
        auto* meshNode = selection.visualNode ? selection.visualNode : root;
        if (!root || !objectNode || !meshNode || !nodeIsOrDescendsFrom(root, objectNode) ||
            !nodeIsOrDescendsFrom(root, meshNode) || !grab_three_phase::isFinite(objectNode->world)) {
            return false;
        }

        const std::uint32_t formId = selection.refr->GetFormID();
        const auto objectKey = reinterpret_cast<std::uintptr_t>(objectNode);
        const auto meshKey = reinterpret_cast<std::uintptr_t>(meshNode);
        const int depth = (std::max)(1, g_rockConfig.rockObjectPhysicsTreeMaxDepth);
        const std::array<RE::NiAVObject*, 3> sourceNodes{ meshNode, objectNode, root };
        cache.unusedSeconds = 0.0f;
        /*
         * The copy is reused across the retries of a held grip: reading an
         * 85,156-triangle rifle again for every retry cost 8.5 ms each. It is
         * read again when the object or its nodes change, when a skinned or
         * dynamic mesh deforms against its node (the grab check; the overlay
         * keeps its copy), and when a rigid model's shapes no longer sit
         * where they sat in the object node's space.
         */
        bool reuse = cache.extracted && cache.refFormId == formId && cache.objectNodeKey == objectKey &&
            cache.meshNodeKey == meshKey && !(refreshDeformingMesh && !cache.rigid);
        if (reuse && cache.rigid) {
            cache.placementScratch.clear();
            reuse = cache.placementsComplete &&
                collectShapePlacements(sourceNodes[cache.sourceNode],
                    transform_math::invertTransform(objectNode->world), depth, cache.placementScratch) &&
                samePlacements(cache.placementScratch, cache.shapePlacements);
        }
        if (!reuse) {
            // Same sources the grab tries: the visual node, then the collision owner, then the root.
            std::vector<TriangleData> worldTriangles;
            MeshExtractionStats stats{};
            std::uint8_t sourceNode = 0;
            if (commitShare) {
                // One read for this check and the commit that follows it.
                auto& shared = _grabSharedExtraction;
                extractAllSurfaceTrianglesRecursive(meshNode, shared.meshTriangles, shared.surfaceTriangles, depth,
                    &shared.stats, handPocketOnlyGrab, &worldTriangles);
                // The commit's own recovery paths run when either reader got nothing.
                shared.valid = !worldTriangles.empty() && !shared.meshTriangles.empty();
                if (shared.valid) {
                    shared.frameIndex = runtime_state::currentFrame().frameIndex;
                    shared.meshNode = meshNode;
                    shared.depth = depth;
                    shared.handPocketOnlyGrab = handPocketOnlyGrab;
                    stats = shared.stats;
                } else {
                    shared.clear();
                    worldTriangles.clear();
                }
            }
            if (worldTriangles.empty()) {
                stats = {};
                extractAllTriangles(meshNode, worldTriangles, depth, &stats);
            }
            if (worldTriangles.empty() && objectNode != meshNode) {
                extractAllTriangles(objectNode, worldTriangles, depth, &stats);
                sourceNode = 1;
            }
            if (worldTriangles.empty() && root != meshNode && root != objectNode) {
                extractAllTriangles(root, worldTriangles, depth, &stats);
                sourceNode = 2;
            }
            if (worldTriangles.size() > kMaxPreviewTriangles) {
                worldTriangles.resize(kMaxPreviewTriangles);
            }
            const RE::NiTransform objectWorldAtExtraction = objectNode->world;
            cache.sourceNode = sourceNode;
            cache.shapePlacements.clear();
            cache.placementsComplete = collectShapePlacements(sourceNodes[sourceNode],
                transform_math::invertTransform(objectWorldAtExtraction), depth, cache.shapePlacements);
            cache.localTriangles.clear();
            cache.localTriangles.reserve(worldTriangles.size());
            for (const auto& triangle : worldTriangles) {
                cache.localTriangles.push_back(GrabLocalTriangle{
                    transform_math::worldPointToLocal(objectWorldAtExtraction, triangle.v0),
                    transform_math::worldPointToLocal(objectWorldAtExtraction, triangle.v1),
                    transform_math::worldPointToLocal(objectWorldAtExtraction, triangle.v2),
                });
            }
            cache.index.clear();
            cache.indexCoversAll = false;
            cache.indexCenterLocal = {};
            cache.refFormId = formId;
            cache.objectNodeKey = objectKey;
            cache.meshNodeKey = meshKey;
            cache.rigid = stats.skinnedShapes == 0 && stats.dynamicShapes == 0;
            cache.extracted = true;
            ROCK_LOG_DEBUG(Hand, "{} GRAB FINGER PROBES: ref={:08X} mesh='{}' owner='{}' triangles={} shapes={}",
                handName(), formId, nodeDebugName(meshNode), nodeDebugName(objectNode), cache.localTriangles.size(),
                stats.visitedShapes);
        }
        if (cache.localTriangles.empty()) {
            return false;
        }

        root_flattened_finger_skeleton_runtime::Snapshot skeleton{};
        RE::NiTransform handWorld{};
        sweep::FrikFingerRig rig{};
        if (!resolveDrawnFingers(_isLeft, nullptr, skeleton, handWorld) || !sweep::resolveFrikFingerRig(_isLeft, rig)) {
            return false;
        }
        const RE::NiTransform objectWorld = objectNode->world;
        const RE::NiTransform handBone = sweep::resolveHandBoneWorld(handWorld, skeleton, rig);

        const bool wholeMesh =
            cache.localTriangles.size() <= grab_finger_pose_runtime::kMaxFingerPoseCandidateTriangles;
        if (wholeMesh) {
            if (!cache.indexCoversAll) {
                cache.indexCoversAll = cache.index.buildFromLocalTriangles(cache.localTriangles);
            }
        } else {
            const RE::NiPoint3 handLocal = transform_math::worldPointToLocal(objectWorld, handBone.translate);
            const RE::NiPoint3 moved = handLocal - cache.indexCenterLocal;
            if (cache.index.empty() ||
                moved.x * moved.x + moved.y * moved.y + moved.z * moved.z >
                    kNearestRebuildDistance * kNearestRebuildDistance) {
                (void)rebuildNearestIndex(cache.localTriangles, cache.nearestScratch, cache.index, handLocal);
                cache.indexCenterLocal = handLocal;
            }
        }
        if (cache.index.empty()) {
            return false;
        }

        // The grab's own solve, with the hand where it is drawn and the object where it is.
        const auto tips = sweep::tipLengths(skeleton);
        const auto result = sweep::solve(rig, handBone, tips, cache.index, objectWorld, options);
        for (std::size_t finger = 0; finger < 5; ++finger) {
            const auto pose =
                sweep::poseFinger(rig, handBone, finger, sweep::jointValuesForCurl(finger, result.values[finger]),
                    tips[finger]);
            appendFingerSpheres(out, pose, finger, options.radii, &cache.index, objectWorld);

            auto& state = out.fingers[finger];
            state.curl = result.values[finger];
            state.state = stateOf(result.fingers[finger]);
            // The tip's path over the whole sweep: open pose down to the minimum curl.
            for (std::size_t i = 0; i < preview::kTipPathPoints; ++i) {
                const float t = static_cast<float>(i) / static_cast<float>(preview::kTipPathPoints - 1);
                const float curl = 1.0f + (options.minValue - 1.0f) * t;
                state.tipPath[i] =
                    sweep::poseFinger(rig, handBone, finger, sweep::jointValuesForCurl(finger, curl), tips[finger])
                        .joints[3];
            }
            state.tipPathCount = static_cast<std::uint8_t>(preview::kTipPathPoints);
            state.valid = true;
        }
        // The pinch line: spheres of the tip capsule's radius along the straight
        // line between the live thumb and index tips, within the pinch's gap.
        const auto& thumbChain = skeleton.fingers[0];
        const auto& indexChain = skeleton.fingers[1];
        if (thumbChain.valid && indexChain.valid) {
            const auto tipEnd = [](const auto& chain) {
                return chain.tipGeometryValid ? chain.points[2] + (chain.tipSegmentCenterWorld - chain.points[2]) * 2.0f
                                              : chain.points[2];
            };
            const RE::NiPoint3 from = tipEnd(thumbChain);
            const RE::NiPoint3 span = tipEnd(indexChain) - from;
            const float gap = std::sqrt(span.x * span.x + span.y * span.y + span.z * span.z);
            const float radius = options.radii.segment[2];
            if (std::isfinite(gap) && gap > 0.0001f && gap <= g_rockConfig.rockGrabPinchMaxFingerGapGameUnits) {
                const int samples = (std::max)(2, static_cast<int>(std::ceil(gap / radius)) + 1);
                for (int i = 0; i < samples && !out.meshBetweenThumbAndIndex; ++i) {
                    const RE::NiPoint3 center = from + span * (static_cast<float>(i) / static_cast<float>(samples - 1));
                    out.meshBetweenThumbAndIndex =
                        cache.index.querySphereWorld(objectWorld, center, radius, nullptr, nullptr);
                }
            }
        }
        out.triangleCount = static_cast<std::uint32_t>(cache.index.triangleCount());
        out.meshCoversWholeObject = wholeMesh;
        out.labelAnchor = labelAnchorFor(skeleton, handWorld);
        out.valid = out.sphereCount > 0;
        return out.valid;
    }

    void Hand::trimGrabFingerProbePreview(float deltaSeconds)
    {
        auto& cache = _grabFingerProbePreview;
        if (!cache.extracted) {
            return;
        }
        if (isHolding()) {
            cache.clear();
            return;
        }
        const auto& selection = _currentSelection;
        if (hasSelection() && selection.refr && !selection.isFarSelection &&
            selection.refr->GetFormID() == cache.refFormId) {
            cache.unusedSeconds = 0.0f;
            return;
        }
        // A selection can drop for a frame between two retries of a held grip.
        cache.unusedSeconds += std::isfinite(deltaSeconds) && deltaSeconds > 0.0f ? deltaSeconds : 0.0f;
        if (cache.unusedSeconds > kUnusedKeepSeconds) {
            cache.clear();
        }
    }

    GrabProbeContact Hand::evaluateGrabProbeContact(const bool commitShare, const bool handPocketOnlyGrab)
    {
        GrabProbeContact result{};
        preview::Snapshot probes{};
        if (!solveSelectedObjectProbes(probes, true, commitShare, handPocketOnlyGrab)) {
            result.reason = _grabFingerProbePreview.extracted && _grabFingerProbePreview.localTriangles.empty()
                ? "noMesh"
                : "probesUnavailable";
            return result;
        }
        result.triangleCount = probes.triangleCount;
        for (std::size_t finger = 0; finger < probes.fingers.size(); ++finger) {
            const auto& state = probes.fingers[finger];
            result.states[finger] = state.valid ? state.state : preview::FingerState::Free;
            if (state.valid && state.state != preview::FingerState::Free) {
                result.fingerMask |= static_cast<std::uint8_t>(1u << finger);
            }
        }
        for (std::uint32_t index = 0; index < probes.sphereCount; ++index) {
            const auto& sphere = probes.spheres[index];
            if (sphere.touching && sphere.finger < 5) {
                result.fingerMask |= static_cast<std::uint8_t>(1u << sphere.finger);
            }
        }
        if (result.fingerMask != 0) {
            result.reason = "meshContact";
        } else if (probes.meshBetweenThumbAndIndex) {
            // Thumb and index hold it between them without either arc meeting it.
            result.fingerMask = 0x03;
            result.reason = "meshBetweenThumbAndIndex";
        } else {
            result.reason = "noProbeTouchesMesh";
        }
        result.touching = result.fingerMask != 0;
        return result;
    }

    /*
     * Mesh-reach lease. Object collision often sits well off the visible mesh,
     * so a hand reaching for the mesh is stopped by the collision first (the
     * keyframed hand colliders shove a loose object, the dynamic proxy stops at
     * a fixed one) and the probe check refuses the grab. A grab refused that
     * way leases the object: PhysicsInteraction::publishHeldBodyExclusion drops
     * this hand's collider pairs with its bodies and publishHeldProxyExclusion
     * lets the proxies through it, so the hand reaches into the collision and
     * the held grip retries until a finger touches the mesh where the player
     * wants it. The lease ends when the hand holds something, or once the grip
     * is released and the hand is outside the object's bound, so collision
     * never comes back with the hand inside it. Game thread only.
     */
    void Hand::beginMeshReachLease(RE::bhkWorld* bhkWorld, RE::hknpWorld* world)
    {
        const auto& selection = _currentSelection;
        if (!bhkWorld || !world || !selection.refr) {
            return;
        }
        if (_meshReachLease.count != 0 && _meshReachLease.ref.get() == selection.refr &&
            _meshReachLease.world == world) {
            return;
        }
        const auto bodySet = object_physics_body_set::scanObjectPhysicsBodySet(bhkWorld, world, selection.refr,
            makeActiveGrabBodyScanOptions(selection));
        MeshReachLease lease{};
        lease.ref.reset(selection.refr);
        lease.world = world;
        for (const auto& record : bodySet.records) {
            if (lease.count >= lease.bodies.size()) {
                break;
            }
            if (record.bodyId != INVALID_BODY_ID && (record.accepted || record.resolvedRef == selection.refr)) {
                lease.bodies[lease.count++] = record.bodyId;
            }
        }
        if (selection.bodyId.value != INVALID_BODY_ID && lease.count < lease.bodies.size() &&
            std::find(lease.bodies.begin(), lease.bodies.begin() + lease.count, selection.bodyId.value) ==
                lease.bodies.begin() + lease.count) {
            lease.bodies[lease.count++] = selection.bodyId.value;
        }
        if (lease.count == 0) {
            return;
        }
        if (_meshReachLease.count != 0) {
            endMeshReachLease("replaced");
        }
        _meshReachLease = std::move(lease);
        ROCK_LOG_DEBUG(Hand, "{} hand MESH REACH lease begun: ref={:08X} bodies={} records={}", handName(),
            selection.refr->GetFormID(), _meshReachLease.count, bodySet.records.size());
    }

    void Hand::updateMeshReachLease(RE::hknpWorld* world, bool gripHeld, const RE::NiPoint3& handWorldPoint)
    {
        if (_meshReachLease.count == 0) {
            return;
        }
        auto* ref = _meshReachLease.ref.get();
        if (!ref || ref->IsDeleted() || ref->IsDisabled() || world != _meshReachLease.world) {
            endMeshReachLease("objectOrWorldGone");
            return;
        }
        if (isHolding()) {
            // A held object's collision belongs to the grab's own exclusions.
            endMeshReachLease("holding");
            return;
        }
        if (gripHeld) {
            return;
        }
        auto* root = ref->Get3D();
        if (!root) {
            endMeshReachLease("no3D");
            return;
        }
        // The hand must be clear of the object before collision returns, or the
        // returning pairs would shove the object off the hand.
        constexpr float kClearanceGameUnits = 6.0f;
        const auto& bound = root->worldBound;
        const RE::NiPoint3 offset = handWorldPoint - bound.center;
        const float distanceSquared = offset.x * offset.x + offset.y * offset.y + offset.z * offset.z;
        const float clearRadius = (std::max)(0.0f, bound.fRadius) + kClearanceGameUnits;
        if (!std::isfinite(distanceSquared) || distanceSquared > clearRadius * clearRadius) {
            endMeshReachLease("handClear");
        }
    }

    void Hand::endMeshReachLease(const char* reason)
    {
        if (_meshReachLease.count == 0) {
            return;
        }
        auto* ref = _meshReachLease.ref.get();
        ROCK_LOG_DEBUG(Hand, "{} hand MESH REACH lease ended: ref={:08X} bodies={} reason={}", handName(),
            ref ? ref->GetFormID() : 0, _meshReachLease.count, reason ? reason : "unknown");
        _meshReachLease = {};
    }
}
