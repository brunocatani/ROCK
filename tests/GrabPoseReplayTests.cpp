/*
 * Offline replay of bench grab records (GrabPoseReplay.h) through ROCK's own
 * capsule finger solve (GrabFingerCapsuleSweep.h).
 *
 *   solver parity  - replaying the recorded inputs through the capsule solve
 *                    reproduces the recorded curls and per-finger outcomes
 *                    (a regression from captured input; failing it fails the
 *                    test).
 *   render parity  - FRIK forward kinematics of the published joint values
 *                    from the published hand lands on the drawn bones the
 *                    record holds (reported, so a mismatch is visible).
 *   clearance      - worst skin clearance per finger of the drawn pose
 *                    against the held mesh (0 = touching, < 0 = inside).
 *
 * Usage: ROCKGrabPoseReplayTests [record.json | directory]...  With no
 * arguments it replays tests/fixtures/grab_replay.
 */
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/GrabFingerCapsuleSweep.h"
#include "physics-interaction/grab/GrabPoseReplay.h"

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
            for (int column = 0; column < 3; ++column)
                out.rotate.entry[row][column] = t.rotate[row * 3 + column];
        }
        out.translate = toNi(t.translate);
        out.scale = t.scale;
        return out;
    }

    skeleton::Snapshot toSnapshot(const replay::FingerSkeleton& s)
    {
        skeleton::Snapshot out{};
        for (std::size_t f = 0; f < 5; ++f) {
            for (std::size_t p = 0; p < 3; ++p)
                out.fingers[f].points[p] = toNi(s.fingers[f].points[p]);
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

    struct LocalTriangle
    {
        RE::NiPoint3 v0{};
        RE::NiPoint3 v1{};
        RE::NiPoint3 v2{};
    };

    float length(const RE::NiPoint3& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

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

    float tipLength(const replay::FingerChain& chain)
    {
        if (!chain.tipGeometryValid)
            return 0.0f;
        return 2.0f * length(toNi(chain.tipSegmentCenter) - toNi(chain.points[2]));
    }

    std::array<FingerPose, 5> forwardKinematics(const replay::Record& r, const capsule::FrikFingerRig& rig,
        const RE::NiTransform& hand)
    {
        std::array<FingerPose, 5> out{};
        for (std::size_t f = 0; f < 5; ++f) {
            RE::NiTransform parent = hand;
            for (std::size_t s = 0; s < 3; ++s) {
                parent = rock::transform_math::composeTransforms(parent,
                    rig.boneLocal(f * 3 + s, r.publishedJointValues[f * 3 + s]));
                out[f].joints[s] = parent.translate;
            }
            out[f].joints[3] = parent.translate +
                rock::transform_math::localVectorToWorld(parent,
                    RE::NiPoint3{ tipLength(r.rendered.fingers[f]), 0.0f, 0.0f });
        }
        return out;
    }

    std::array<FingerPose, 5> drawnPose(const replay::Record& r)
    {
        std::array<FingerPose, 5> out{};
        for (std::size_t f = 0; f < 5; ++f) {
            const auto& c = r.rendered.fingers[f];
            for (std::size_t s = 0; s < 3; ++s)
                out[f].joints[s] = toNi(c.points[s]);
            out[f].joints[3] = toNi(c.points[2]) + (toNi(c.tipSegmentCenter) - toNi(c.points[2])) * 2.0f;
        }
        return out;
    }

    // Worst skin clearance per finger: signed centreline distance (negative inside, by outward winding) minus
    // that bone's capsule radius. 0 = touching, negative = visibly inside, 12 samples per finger.
    std::array<float, 5> clearance(const std::array<FingerPose, 5>& pose,
        const runtime::FingerPoseTriangleSpatialIndex& index, const RE::NiTransform& objectWorld)
    {
        const capsule::CapsuleRadii radii{};
        std::array<float, 5> worst{};
        for (std::size_t f = 0; f < 5; ++f) {
            worst[f] = 99.0f;
            for (std::size_t s = 0; s < 3; ++s) {
                for (int k = 0; k < 4; ++k) {
                    const RE::NiPoint3 p =
                        pose[f].joints[s] + (pose[f].joints[s + 1] - pose[f].joints[s]) * ((k + 0.5f) / 4.0f);
                    RE::NiPoint3 nearest{}, outward{};
                    if (!index.querySphereWorld(objectWorld, p, 12.0f, &nearest, &outward, nullptr, false))
                        continue;
                    const RE::NiPoint3 offset = p - nearest;
                    const float d = length(offset);
                    const float centreline =
                        offset.x * outward.x + offset.y * outward.y + offset.z * outward.z < 0.0f ? -d : d;
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
        std::printf("== %s  %s %s (%s, %s)\n", path.filename().string().c_str(), r.objectName.c_str(),
            r.isLeft ? "left" : "right", r.acquisition.c_str(), r.gripSource.c_str());
        if (!r.frikRigValid) {
            std::printf("FAIL: record has no FRIK rig\n");
            return false;
        }

        std::vector<LocalTriangle> triangles;
        for (std::size_t i = 0; i + 8 < r.trianglesObjectLocal.size(); i += 9) {
            const float* t = &r.trianglesObjectLocal[i];
            triangles.push_back({ { t[0], t[1], t[2] }, { t[3], t[4], t[5] }, { t[6], t[7], t[8] } });
        }
        runtime::FingerPoseTriangleSpatialIndex index{};
        if (!index.buildFromLocalTriangles(triangles)) {
            std::printf("FAIL: mesh index build failed\n");
            return false;
        }

        // Solver parity: the shipped solve on the recorded inputs.
        const auto rig = rigFor(r);
        const auto solveSkeleton = toSnapshot(r.solveSkeleton);
        const capsule::Options options{
            .minValue = r.options.minValue,
            .fingerMaxOpenValue = r.options.fingerMaxOpenValue,
            .thumbMaxOpenValue = r.options.thumbMaxOpenValue,
            .step = r.options.step,
        };
        const auto solve =
            capsule::solveForHand(rig, toNi(r.solveHandWorld), solveSkeleton, index, toNi(r.solveObjectWorld), options);
        bool solverParity = true;
        for (std::size_t f = 0; f < 5; ++f) {
            solverParity = solverParity && std::fabs(solve.values[f] - r.solvedValues[f]) <= 1e-4f &&
                solve.fingers[f].contact == (r.solvedContact[f] != 0) &&
                solve.fingers[f].openedToClear == (r.solvedOpenedToClear[f] != 0) &&
                solve.fingers[f].blocked == (r.solvedBlocked[f] != 0);
        }
        std::printf("  solver parity: %s  recorded [%s]  replayed [%s]\n", solverParity ? "OK" : "MISMATCH",
            row(r.solvedValues).c_str(), row(solve.values).c_str());

        // Render parity: FRIK forward kinematics of the published joint values from the published hand.
        const RE::NiTransform hand = toNi(r.renderedVisualHandWorld);
        const auto drawn = drawnPose(r);
        const auto published =
            forwardKinematics(r, rig, capsule::resolveHandBoneWorld(hand, toSnapshot(r.rendered), rig));
        std::array<float, 5> renderError{};
        for (std::size_t f = 0; f < 5; ++f) {
            for (std::size_t s = 0; s < 3; ++s)
                renderError[f] = (std::max)(renderError[f], length(published[f].joints[s] - drawn[f].joints[s]));
        }
        const float worstRender = *std::max_element(renderError.begin(), renderError.end());
        std::printf("  render parity: %s  published vs drawn joints [%s] gu\n",
            worstRender <= 0.25f ? "OK" : "MISMATCH", row(renderError).c_str());
        if (r.presentedHandValid) {
            std::printf("  drawn hand bone (AfterWorldFinal) vs published hand: %.2f gu\n",
                length(toNi(r.presentedHandWorld.translate) - hand.translate));
        }

        std::array<float, 5> flags{};
        for (std::size_t f = 0; f < 5; ++f) {
            const auto& fr = solve.fingers[f];
            flags[f] = fr.blocked ? -1.0f : (fr.openedToClear ? 2.0f : (fr.contact ? 1.0f : 0.0f));
        }
        constexpr int kRuns = 40;
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < kRuns; ++i) {
            runtime::FingerPoseTriangleSpatialIndex timedIndex{};
            (void)timedIndex.buildFromLocalTriangles(triangles);
            (void)capsule::solveForHand(rig, toNi(r.solveHandWorld), solveSkeleton, timedIndex,
                toNi(r.solveObjectWorld), options);
        }
        const double ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / kRuns;

        std::printf("  skin clearance (gu, 0 = touching, <0 = inside)   thumb  index middle  ring  pinky\n");
        std::printf("    drawn pose                                    [%s]\n",
            row(clearance(drawn, index, toNi(r.renderedObjectWorld))).c_str());
        std::printf("    curl                                          [%s]\n", row(r.finalValues).c_str());
        std::printf("    1 contact, 2 opened to clear, 0 miss, -1 blocked [%s]\n", row(flags).c_str());
        std::printf("  cost per solve incl. index build: %.3f ms (%u sphere queries, %u triangle tests)\n", ms,
            solve.sphereQueries, solve.stats.triangleTests);
        return solverParity;
    }
}

int main(int argc, char** argv)
{
    std::vector<std::filesystem::path> inputs;
    for (int i = 1; i < argc; ++i)
        inputs.emplace_back(argv[i]);
    if (inputs.empty())
        inputs.emplace_back(std::filesystem::path(ROCK_TEST_FIXTURE_DIR) / "grab_replay");

    std::vector<std::filesystem::path> records;
    for (const auto& input : inputs) {
        if (std::filesystem::is_directory(input)) {
            for (const auto& entry : std::filesystem::directory_iterator(input)) {
                if (entry.path().extension() == ".json")
                    records.push_back(entry.path());
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
    for (const auto& record : records)
        ok = replayRecord(record) && ok;
    return ok ? 0 : 1;
}
