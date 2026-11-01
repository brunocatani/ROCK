/*
 * Offline replay of bench grab records (GrabPoseReplay.h) through ROCK's own
 * finger-solve code. Two parity checks decide whether offline iteration can be
 * trusted, then a breakdown says where finger penetration comes from:
 *
 *   solver parity  - replaying the recorded inputs through
 *                    solveFrozenMeshFingerPoseBase reproduces the recorded
 *                    curl values and hit kinds (a regression from captured
 *                    input; failing it fails the test).
 *   render parity  - forward kinematics of the published local transforms from
 *                    the rendered hand frame lands on the rendered bones the
 *                    game reported (reported, so a mismatch is visible
 *                    evidence, not a silent pass).
 *   breakdown      - signed centreline-to-mesh distance per finger for
 *                    (a) the rendered pose, (b) FRIK drawing the published
 *                    joint values alone, without ROCK's local-transform layer
 *                    (splay omitted). Comparing (a) and (b) separates the
 *                    solver's choice from the layer applied on top of it.
 *
 * Usage: ROCKGrabPoseReplayTests [record.json | directory]...  With no
 * arguments it replays tests/fixtures/grab_replay.
 */
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/GrabFingerCapsuleSweep.h"
#include "physics-interaction/grab/GrabPoseReplay.h"
#include "physics-interaction/TransformMath.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace rock::root_flattened_finger_skeleton_runtime
{
    // Replays always supply the captured skeleton; the scene is never queried.
    bool resolveLiveFingerSkeletonSnapshot(bool, Snapshot&, std::string*, SkeletonBoneCaptureSpace) { return false; }
}

namespace
{
    namespace replay = rock::grab_pose_replay;
    namespace runtime = rock::grab_finger_pose_runtime;
    namespace skeleton = rock::root_flattened_finger_skeleton_runtime;
    namespace capsule = rock::grab_finger_capsule_sweep;

    RE::NiPoint3 toNi(const replay::Vec3& v) { return { v[0], v[1], v[2] }; }

    RE::NiTransform toNi(const replay::Transform& t)
    {
        RE::NiTransform out{};
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) out.rotate.entry[row][column] = t.rotate[row * 3 + column];
        }
        out.translate = toNi(t.translate);
        out.scale = t.scale;
        return out;
    }

    skeleton::Snapshot toSnapshot(const replay::FingerSkeleton& s)
    {
        skeleton::Snapshot out{};
        for (std::size_t f = 0; f < 5; ++f) {
            for (std::size_t p = 0; p < 3; ++p) out.fingers[f].points[p] = toNi(s.fingers[f].points[p]);
            out.fingers[f].tipSegmentCenterWorld = toNi(s.fingers[f].tipSegmentCenter);
            out.fingers[f].tipDirectionWorld = toNi(s.fingers[f].tipDirection);
            out.fingers[f].tipGeometryValid = s.fingers[f].tipGeometryValid;
            out.fingers[f].valid = s.fingers[f].valid;
        }
        out.palmNormalWorld = toNi(s.palmNormal);
        out.palmNormalValid = s.palmNormalValid;
        out.inPowerArmor = s.inPowerArmor;
        out.valid = s.valid;
        return out;
    }

    runtime::GrabFingerPoseTargetSet toTargets(const replay::PoseTargets& p)
    {
        runtime::GrabFingerPoseTargetSet out{};
        for (std::size_t f = 0; f < 5; ++f) {
            out.targets[f] = toNi(p.targets[f]);
            out.targetNormals[f] = toNi(p.targetNormals[f]);
            out.targetValid[f] = p.targetValid[f];
            out.targetNormalValid[f] = p.targetNormalValid[f];
        }
        out.seatPointWorld = toNi(p.seatPoint);
        out.seatNormalWorld = toNi(p.seatNormal);
        out.seatPointValid = p.seatPointValid;
        out.seatNormalValid = p.seatNormalValid;
        out.useSeatPointForMissingTargets = p.useSeatPointForMissingTargets;
        out.useWholeMeshForMissingTargets = p.useWholeMeshForMissingTargets;
        out.targetCount = p.targetCount;
        return out;
    }

    struct LocalTriangle
    {
        RE::NiPoint3 v0{};
        RE::NiPoint3 v1{};
        RE::NiPoint3 v2{};
    };

    float length(const RE::NiPoint3& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

    // FRIK's forward model, shared with the prototype solver (GrabFingerCapsuleSweep.h).
    capsule::FrikFingerRig rigFor(const replay::Record& r)
    {
        std::array<RE::NiTransform, 15> closed{}, open{};
        for (std::size_t bone = 0; bone < 15; ++bone) {
            closed[bone] = toNi(r.frikClosedLocal[bone]);
            open[bone] = toNi(r.frikOpenLocal[bone]);
        }
        return capsule::makeFrikFingerRig(closed, open);
    }

    struct FingerPose
    {
        std::array<RE::NiPoint3, 4> joints{}; // three bone origins and the tip end
    };

    // Distal tip length from the rendered skeleton: twice the tip-segment centre offset.
    float tipLength(const replay::FingerChain& rendered)
    {
        if (!rendered.tipGeometryValid) return 0.0f;
        return 2.0f * length(toNi(rendered.tipSegmentCenter) - toNi(rendered.points[2]));
    }

    template <class LocalOf>
    std::array<FingerPose, 5> forwardKinematics(const replay::Record& r, const RE::NiTransform& hand, LocalOf&& localOf)
    {
        std::array<FingerPose, 5> out{};
        for (std::size_t f = 0; f < 5; ++f) {
            RE::NiTransform parent = hand;
            for (std::size_t s = 0; s < 3; ++s) {
                parent = rock::transform_math::composeTransforms(parent, localOf(f * 3 + s));
                out[f].joints[s] = parent.translate;
            }
            out[f].joints[3] = parent.translate + rock::transform_math::localVectorToWorld(parent, RE::NiPoint3{ tipLength(r.rendered.fingers[f]), 0.0f, 0.0f });
        }
        return out;
    }

    std::array<FingerPose, 5> renderedPose(const replay::Record& r)
    {
        std::array<FingerPose, 5> out{};
        for (std::size_t f = 0; f < 5; ++f) {
            const auto& c = r.rendered.fingers[f];
            for (std::size_t s = 0; s < 3; ++s) out[f].joints[s] = toNi(c.points[s]);
            out[f].joints[3] = toNi(c.points[2]) + (toNi(c.tipSegmentCenter) - toNi(c.points[2])) * 2.0f;
        }
        return out;
    }

    // Worst skin clearance per finger: signed centreline distance (negative inside, by outward winding) minus
    // that bone's capsule radius. 0 = touching, negative = visibly inside, 12 samples per finger.
    std::array<float, 5> penetration(const std::array<FingerPose, 5>& pose, const runtime::FingerPoseTriangleSpatialIndex& index, const RE::NiTransform& objectWorld)
    {
        const capsule::CapsuleRadii radii{};
        std::array<float, 5> worst{};
        for (std::size_t f = 0; f < 5; ++f) {
            worst[f] = 99.0f;
            for (std::size_t s = 0; s < 3; ++s) {
                for (int k = 0; k < 4; ++k) {
                    const RE::NiPoint3 p = pose[f].joints[s] + (pose[f].joints[s + 1] - pose[f].joints[s]) * ((k + 0.5f) / 4.0f);
                    RE::NiPoint3 nearest{}, outward{};
                    if (!index.querySphereWorld(objectWorld, p, 12.0f, &nearest, &outward, nullptr, false)) continue;
                    const RE::NiPoint3 offset = p - nearest;
                    const float d = length(offset);
                    const float centreline = offset.x * outward.x + offset.y * outward.y + offset.z * outward.z < 0.0f ? -d : d;
                    worst[f] = (std::min)(worst[f], centreline - radii.segment[s]);
                }
            }
        }
        return worst;
    }

    std::string row(const std::array<float, 5>& v)
    {
        char buffer[96];
        std::snprintf(buffer, sizeof(buffer), "%6.2f %6.2f %6.2f %6.2f %6.2f", v[0], v[1], v[2], v[3], v[4]);
        return buffer;
    }

    bool replayRecord(const std::filesystem::path& path)
    {
        std::ifstream in(path, std::ios::binary);
        std::stringstream text;
        text << in.rdbuf();
        std::string error;
        const auto parsed = replay::parse(text.str(), &error);
        if (!parsed) {
            std::printf("FAIL %s: %s\n", path.filename().string().c_str(), error.c_str());
            return false;
        }
        const auto& r = *parsed;
        std::printf("== %s  %s %s (%s, %s)\n", path.filename().string().c_str(), r.objectName.c_str(), r.isLeft ? "left" : "right", r.acquisition.c_str(),
            r.gripSource.c_str());

        std::vector<LocalTriangle> triangles;
        for (std::size_t i = 0; i + 8 < r.trianglesObjectLocal.size(); i += 9) {
            const float* t = &r.trianglesObjectLocal[i];
            triangles.push_back({ { t[0], t[1], t[2] }, { t[3], t[4], t[5] }, { t[6], t[7], t[8] } });
        }

        // Solver parity.
        runtime::FrozenMeshFingerPoseSolveOptions options{
            .minValue = r.options.minValue,
            .maxTriangleDistanceSquared = r.options.maxTriangleDistanceSquared,
            .rejectBacksideHits = r.options.rejectBacksideHits,
            .surfacePlaneToleranceGameUnits = r.options.surfacePlaneToleranceGameUnits,
            .allowSurfaceAimTargets = r.options.allowSurfaceAimTargets,
            .sweepContactRadiusGameUnits = r.options.sweepContactRadiusGameUnits,
            .thumbSweepMaxOpenValue = r.options.thumbSweepMaxOpenValue,
            .fingerSweepMaxOpenValue = r.options.fingerSweepMaxOpenValue,
            .mode = static_cast<runtime::FingerPoseMode>(r.options.mode),
        };
        const auto snapshot = toSnapshot(r.solveSkeleton);
        std::array<RE::NiPoint3, 5> openDirections{};
        for (std::size_t f = 0; f < 5; ++f) openDirections[f] = toNi(r.commandedOpenDirectionsWorld[f]);
        runtime::FingerPoseTriangleSpatialIndex solveIndex{};
        std::vector<rock::TriangleData> scratch;
        const auto solve = runtime::solveFrozenMeshFingerPoseBase(triangles, toNi(r.solveObjectWorld), toNi(r.solveHandWorld), r.isLeft, toNi(r.grabAnchorWorld),
            toTargets(r.targets), solveIndex, scratch, options, &snapshot, r.commandedOpenDirectionsValid ? &openDirections : nullptr);
        bool solverParity = true;
        std::array<float, 5> replayed{};
        for (std::size_t f = 0; f < 5; ++f) {
            replayed[f] = solve.pose.values[f];
            solverParity = solverParity && std::fabs(replayed[f] - r.solvedValues[f]) <= 1e-4f && static_cast<int>(solve.pose.hitKind[f]) == r.solvedHitKind[f];
        }
        std::printf("  solver parity: %s  recorded [%s]  replayed [%s]\n", solverParity ? "OK" : "MISMATCH", row(r.solvedValues).c_str(), row(replayed).c_str());

        // Render parity: published local transforms from the rendered hand frame.
        const auto rig = rigFor(r);
        const RE::NiTransform hand = toNi(r.renderedVisualHandWorld);
        const auto rendered = renderedPose(r);
        const auto published = forwardKinematics(r, hand, [&](std::size_t bone) {
            return (r.publishedLocalTransformMask >> bone) & 1u ? toNi(r.publishedLocalTransforms[bone]) : rig.boneLocal(bone, r.publishedJointValues[bone]);
        });
        std::array<float, 5> renderError{};
        for (std::size_t f = 0; f < 5; ++f) {
            for (std::size_t s = 0; s < 3; ++s) renderError[f] = (std::max)(renderError[f], length(published[f].joints[s] - rendered[f].joints[s]));
        }
        // A uniform error is a hand-frame offset, not a pose error: remove the mean offset and see what is left.
        RE::NiPoint3 meanOffset{};
        for (std::size_t f = 0; f < 5; ++f) {
            for (std::size_t s = 0; s < 3; ++s) meanOffset = meanOffset + (rendered[f].joints[s] - published[f].joints[s]) * (1.0f / 15.0f);
        }
        std::array<float, 5> residual{};
        for (std::size_t f = 0; f < 5; ++f) {
            for (std::size_t s = 0; s < 3; ++s) residual[f] = (std::max)(residual[f], length(published[f].joints[s] + meanOffset - rendered[f].joints[s]));
        }
        const float worstResidual = *std::max_element(residual.begin(), residual.end());
        std::printf("  render parity: %s  hand-frame offset %.2f gu, residual after removing it [%s] gu\n", worstResidual <= 0.25f ? "OK" : "MISMATCH",
            length(meanOffset), row(residual).c_str());

        runtime::FingerPoseTriangleSpatialIndex index{};
        if (!index.buildFromLocalTriangles(triangles)) {
            std::printf("  breakdown skipped: mesh index failed\n");
            return solverParity;
        }
        const RE::NiTransform objectWorld = toNi(r.renderedObjectWorld);
        // Rendered hand bone: from the measured offset, and as the prototype recovers it from finger bases.
        // Drawn hand bone = ROCK's published hand (latched AfterWorldFinal wrist: 0.00 gu). The in-frame skeleton is stale.
        const RE::NiTransform drawnHandBone = hand;
        const auto renderedSnapshot = toSnapshot(r.rendered);
        const RE::NiTransform recoveredHandBone = capsule::resolveHandBoneWorld(hand, renderedSnapshot, rig);
        const auto jointsOnly = forwardKinematics(r, drawnHandBone, [&](std::size_t bone) { return rig.boneLocal(bone, r.publishedJointValues[bone]); });

        // Prototype: capsule sweep on FRIK kinematics, solved in the same frozen relation as the current solve.
        std::array<float, 5> tipLengths{};
        for (std::size_t f = 0; f < 5; ++f) tipLengths[f] = tipLength(r.solveSkeleton.fingers[f]);
        const RE::NiTransform solveHandBone = capsule::resolveHandBoneWorld(toNi(r.solveHandWorld), snapshot, rig);
        const capsule::Options capsuleOptions{
            .minValue = r.options.minValue,
            .fingerMaxOpenValue = r.options.fingerSweepMaxOpenValue,
            .thumbMaxOpenValue = r.options.thumbSweepMaxOpenValue,
        };
        const auto capsuleSolve = capsule::solve(rig, solveHandBone, tipLengths, index, toNi(r.solveObjectWorld), capsuleOptions);
        const auto capsulePose = forwardKinematics(r, drawnHandBone, [&](std::size_t bone) { return rig.boneLocal(bone, capsuleSolve.jointValues[bone]); });

        // Cost per solve, both including their mesh index build, as at grab time.
        constexpr int kRuns = 40;
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < kRuns; ++i) {
            runtime::FingerPoseTriangleSpatialIndex timedIndex{};
            std::vector<rock::TriangleData> timedScratch;
            (void)runtime::solveFrozenMeshFingerPoseBase(triangles, toNi(r.solveObjectWorld), toNi(r.solveHandWorld), r.isLeft, toNi(r.grabAnchorWorld),
                toTargets(r.targets), timedIndex, timedScratch, options, &snapshot, r.commandedOpenDirectionsValid ? &openDirections : nullptr);
        }
        const auto t1 = std::chrono::steady_clock::now();
        for (int i = 0; i < kRuns; ++i) {
            runtime::FingerPoseTriangleSpatialIndex timedIndex{};
            (void)timedIndex.buildFromLocalTriangles(triangles);
            (void)capsule::solve(rig, solveHandBone, tipLengths, timedIndex, toNi(r.solveObjectWorld), capsuleOptions);
        }
        const auto t2 = std::chrono::steady_clock::now();
        const auto ms = [&](auto a, auto b) { return std::chrono::duration<double, std::milli>(b - a).count() / kRuns; };

        std::array<float, 5> capsuleFlags{};
        for (std::size_t f = 0; f < 5; ++f) {
            const auto& fr = capsuleSolve.fingers[f];
            capsuleFlags[f] = fr.blocked ? -1.0f : (fr.openedToClear ? 2.0f : (fr.contact ? 1.0f : 0.0f));
        }
        std::printf("  in-frame skeleton hand bone vs drawn hand bone: %.2f gu (stale read)\n", length(recoveredHandBone.translate - drawnHandBone.translate));
        if (r.presentedHandValid) {
            // The drawn hand bone (AfterWorldFinal) against ROCK's published hand and against the in-frame skeleton's hand bone.
            const RE::NiPoint3 presented = toNi(r.presentedHandWorld.translate);
            std::printf("  drawn hand bone (AfterWorldFinal): %.2f gu from ROCK's published hand, %.2f gu from the in-frame skeleton's hand bone\n",
                length(presented - hand.translate), length(presented - recoveredHandBone.translate));
        } else {
            std::printf("  drawn hand bone (AfterWorldFinal): not in this record\n");
        }
        // Did the object stay where the solve put it relative to the hand? Object in hand-bone space, solve versus render.
        {
            const RE::NiTransform atSolve = rock::transform_math::composeTransforms(rock::transform_math::invertTransform(solveHandBone), toNi(r.solveObjectWorld));
            const RE::NiTransform atRender = rock::transform_math::composeTransforms(rock::transform_math::invertTransform(recoveredHandBone), objectWorld);
            const RE::NiMatrix3 delta = atSolve.rotate.Transpose() * atRender.rotate;
            const float cosAngle = std::clamp((delta.entry[0][0] + delta.entry[1][1] + delta.entry[2][2] - 1.0f) * 0.5f, -1.0f, 1.0f);
            const RE::NiPoint3 seatHandLocal = rock::transform_math::worldPointToLocal(solveHandBone, toNi(r.grabAnchorWorld));
            std::printf("  object drift in hand space, solve -> render: %.2f gu, %.1f deg (grip anchor at hand-local %.1f %.1f %.1f)\n",
                length(atRender.translate - atSolve.translate), std::acos(cosAngle) * 57.2958f, seatHandLocal.x, seatHandLocal.y, seatHandLocal.z);
        }
        std::printf("  skin clearance (gu, 0 = touching, <0 = inside)          thumb  index middle  ring  pinky\n");
        std::printf("    current: drawn (FK of published transforms)          [%s]\n", row(penetration(published, index, objectWorld)).c_str());
        std::printf("    current: joint values only (no local layer)         [%s]\n", row(penetration(jointsOnly, index, objectWorld)).c_str());
        std::printf("    current: curl                                       [%s]\n", row(r.solvedValues).c_str());
        std::printf("    capsule: clearance                                  [%s]\n", row(penetration(capsulePose, index, objectWorld)).c_str());
        std::printf("    capsule: curl                                       [%s]\n", row(capsuleSolve.values).c_str());
        std::printf("    capsule: 1 contact, 2 opened to clear, 0 miss, -1 blocked [%s]\n", row(capsuleFlags).c_str());
        std::printf("  cost per solve: current %.3f ms, capsule %.3f ms (%u sphere queries, %u triangle tests)\n", ms(t0, t1), ms(t1, t2),
            capsuleSolve.sphereQueries, capsuleSolve.stats.triangleTests);

        return solverParity;
    }
}

int main(int argc, char** argv)
{
    std::vector<std::filesystem::path> inputs;
    for (int i = 1; i < argc; ++i) inputs.emplace_back(argv[i]);
    if (inputs.empty()) inputs.emplace_back(std::filesystem::path(ROCK_TEST_FIXTURE_DIR) / "grab_replay");

    std::vector<std::filesystem::path> records;
    for (const auto& input : inputs) {
        if (std::filesystem::is_directory(input)) {
            for (const auto& entry : std::filesystem::directory_iterator(input)) {
                if (entry.path().extension() == ".json") records.push_back(entry.path());
            }
        } else {
            records.push_back(input);
        }
    }
    std::sort(records.begin(), records.end());
    if (records.empty()) {
        std::printf("FAIL: no grab replay records found\n");
        return 1;
    }
    bool ok = true;
    for (const auto& record : records) ok = replayRecord(record) && ok;
    return ok ? 0 : 1;
}
