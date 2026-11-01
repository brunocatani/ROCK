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
#include "physics-interaction/grab/GrabPoseReplay.h"
#include "physics-interaction/TransformMath.h"

#include <algorithm>
#include <array>
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

    // FRIK HandPoseMath: rotation = slerp(closed -> open, flex) with no clamp on
    // the interpolant (flex > 1 extrapolates past open), translation = rest.
    struct Quat
    {
        float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
    };

    Quat fromMatrix(const RE::NiMatrix3& m)
    {
        const auto& e = m.entry;
        Quat q{};
        const float trace = e[0][0] + e[1][1] + e[2][2];
        if (trace > 0.0f) {
            const float s = std::sqrt(trace + 1.0f) * 2.0f;
            q = { 0.25f * s, (e[2][1] - e[1][2]) / s, (e[0][2] - e[2][0]) / s, (e[1][0] - e[0][1]) / s };
        } else if (e[0][0] > e[1][1] && e[0][0] > e[2][2]) {
            const float s = std::sqrt(1.0f + e[0][0] - e[1][1] - e[2][2]) * 2.0f;
            q = { (e[2][1] - e[1][2]) / s, 0.25f * s, (e[0][1] + e[1][0]) / s, (e[0][2] + e[2][0]) / s };
        } else if (e[1][1] > e[2][2]) {
            const float s = std::sqrt(1.0f + e[1][1] - e[0][0] - e[2][2]) * 2.0f;
            q = { (e[0][2] - e[2][0]) / s, (e[0][1] + e[1][0]) / s, 0.25f * s, (e[1][2] + e[2][1]) / s };
        } else {
            const float s = std::sqrt(1.0f + e[2][2] - e[0][0] - e[1][1]) * 2.0f;
            q = { (e[1][0] - e[0][1]) / s, (e[0][2] + e[2][0]) / s, (e[1][2] + e[2][1]) / s, 0.25f * s };
        }
        return q;
    }

    RE::NiMatrix3 toMatrix(const Quat& q)
    {
        RE::NiMatrix3 m{};
        const float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
        const float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z, wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
        m.entry[0][0] = 1 - 2 * (yy + zz); m.entry[0][1] = 2 * (xy - wz);     m.entry[0][2] = 2 * (xz + wy);
        m.entry[1][0] = 2 * (xy + wz);     m.entry[1][1] = 1 - 2 * (xx + zz); m.entry[1][2] = 2 * (yz - wx);
        m.entry[2][0] = 2 * (xz - wy);     m.entry[2][1] = 2 * (yz + wx);     m.entry[2][2] = 1 - 2 * (xx + yy);
        return m;
    }

    Quat slerp(Quat from, const Quat& to, float t)
    {
        float d = from.w * to.w + from.x * to.x + from.y * to.y + from.z * to.z;
        if (d < 0.0f) {
            from = { -from.w, -from.x, -from.y, -from.z };
            d = -d;
        }
        if (d > 0.999995f) return from;
        const float theta0 = std::acos(d);
        const float theta = theta0 * t;
        const float s0 = std::cos(theta) - d * std::sin(theta) / std::sin(theta0);
        const float s1 = std::sin(theta) / std::sin(theta0);
        return { s0 * from.w + s1 * to.w, s0 * from.x + s1 * to.x, s0 * from.y + s1 * to.y, s0 * from.z + s1 * to.z };
    }

    RE::NiTransform frikJointLocal(const replay::Record& r, std::size_t bone, float flex)
    {
        RE::NiTransform out = toNi(r.frikOpenLocal[bone]);
        out.rotate = toMatrix(slerp(fromMatrix(toNi(r.frikClosedLocal[bone]).rotate), fromMatrix(out.rotate), std::clamp(flex, -1.0f, 2.0f)));
        return out;
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

    // Worst signed centreline distance over 12 samples per finger; negative = inside (outward winding).
    std::array<float, 5> penetration(const std::array<FingerPose, 5>& pose, const runtime::FingerPoseTriangleSpatialIndex& index, const RE::NiTransform& objectWorld)
    {
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
                    worst[f] = (std::min)(worst[f], offset.x * outward.x + offset.y * outward.y + offset.z * outward.z < 0.0f ? -d : d);
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
        const RE::NiTransform hand = toNi(r.renderedVisualHandWorld);
        const auto rendered = renderedPose(r);
        const auto published = forwardKinematics(r, hand, [&](std::size_t bone) {
            return (r.publishedLocalTransformMask >> bone) & 1u ? toNi(r.publishedLocalTransforms[bone]) : frikJointLocal(r, bone, r.publishedJointValues[bone]);
        });
        std::array<float, 5> renderError{};
        for (std::size_t f = 0; f < 5; ++f) {
            for (std::size_t s = 0; s < 3; ++s) renderError[f] = (std::max)(renderError[f], length(published[f].joints[s] - rendered[f].joints[s]));
        }
        const float worstRenderError = *std::max_element(renderError.begin(), renderError.end());
        std::printf("  render parity: %s  max joint error per finger [%s] gu\n", worstRenderError <= 0.25f ? "OK" : "MISMATCH", row(renderError).c_str());
        // A uniform error is a hand-frame offset, not a pose error: remove the mean offset and see what is left.
        RE::NiPoint3 meanOffset{};
        for (std::size_t f = 0; f < 5; ++f) {
            for (std::size_t s = 0; s < 3; ++s) meanOffset = meanOffset + (rendered[f].joints[s] - published[f].joints[s]) * (1.0f / 15.0f);
        }
        std::array<float, 5> residual{};
        for (std::size_t f = 0; f < 5; ++f) {
            for (std::size_t s = 0; s < 3; ++s) residual[f] = (std::max)(residual[f], length(published[f].joints[s] + meanOffset - rendered[f].joints[s]));
        }
        const RE::NiPoint3 offsetHandLocal = rock::transform_math::worldVectorToLocal(hand, meanOffset);
        std::printf("  hand-frame offset %.2f gu, hand-local (%.2f, %.2f, %.2f); residual after removing it [%s] gu\n", length(meanOffset), offsetHandLocal.x,
            offsetHandLocal.y, offsetHandLocal.z, row(residual).c_str());

        // Breakdown: rendered versus FRIK drawing the joint values alone.
        runtime::FingerPoseTriangleSpatialIndex index{};
        if (!index.buildFromLocalTriangles(triangles)) {
            std::printf("  breakdown skipped: mesh index failed\n");
            return solverParity;
        }
        const RE::NiTransform objectWorld = toNi(r.renderedObjectWorld);
        // Rendered hand bone = ROCK's visual hand shifted by the measured offset (residual above shows rotation is negligible).
        RE::NiTransform renderedHandBone = hand;
        renderedHandBone.translate = hand.translate + meanOffset;
        const auto jointsOnly =
            forwardKinematics(r, renderedHandBone, [&](std::size_t bone) { return frikJointLocal(r, bone, r.publishedJointValues[bone]); });
        std::array<float, 5> layerShift{};
        for (std::size_t f = 0; f < 5; ++f) layerShift[f] = length(jointsOnly[f].joints[3] - (published[f].joints[3] + meanOffset));
        std::printf("  centreline-to-mesh (gu, <0.7 = visible penetration)      thumb  index middle  ring  pinky\n");
        std::printf("    rendered                                            [%s]\n", row(penetration(rendered, index, objectWorld)).c_str());
        std::printf("    joint values only (no local layer)                  [%s]\n", row(penetration(jointsOnly, index, objectWorld)).c_str());
        std::printf("    tip moved by the local layer                        [%s]\n", row(layerShift).c_str());
        std::printf("    solved curl (1 = open, >1 = bent back)              [%s]\n", row(r.solvedValues).c_str());
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
