#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

/*
 * Bench replay record for one dynamic grab: the mesh finger solve's exact
 * inputs, its outputs, what ROCK published to FRIK, FRIK's own forward model,
 * and the bones as rendered once the pose settled. Written only with
 * bDebugTestbenchTelemetry on, one file per grab.
 *
 * Offline tooling replays the solve through the same ROCK headers (solver
 * parity) and rebuilds the rendered fingers from the published values with
 * FRIK's open/closed transforms (render parity). Posing changes are then
 * iterated offline and only confirmed in game. Like SavedGrabCaptureFormat it
 * holds no engine types so the harness can parse it without the game.
 *
 * Transforms use Ni conventions: rotate[row * 3 + column] = entry[row][column],
 * world = translate + rotate * (scale * local), as transform_math does.
 */
namespace rock::grab_pose_replay
{
    inline constexpr std::uint32_t kFormatVersion = 2;
    inline constexpr std::size_t kFingerCount = 5;
    inline constexpr std::size_t kFingerBoneCount = 15;

    using Vec3 = std::array<float, 3>;

    struct Transform
    {
        std::array<float, 9> rotate{ 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
        Vec3 translate{};
        float scale = 1.0f;
    };

    // root_flattened_finger_skeleton_runtime::FingerChain / Snapshot, world space.
    struct FingerChain
    {
        std::array<Vec3, 3> points{};
        Vec3 tipSegmentCenter{};
        Vec3 tipDirection{};
        bool tipGeometryValid = false;
        bool valid = false;
    };

    struct FingerSkeleton
    {
        std::array<FingerChain, kFingerCount> fingers{};
        Vec3 palmNormal{ 0.0f, 0.0f, -1.0f };
        bool palmNormalValid = false;
        bool inPowerArmor = false;
        bool valid = false;
    };

    // grab_finger_capsule_sweep::Options.
    struct CapsuleOptions
    {
        float minValue = 0.0f;
        float fingerMaxOpenValue = 0.0f;
        float thumbMaxOpenValue = 0.0f;
        float step = 0.0f;
    };

    struct Record
    {
        std::uint32_t formatVersion = kFormatVersion;
        std::string object; // "0xFF000FA3"
        std::string objectName;
        bool isLeft = false;
        std::string acquisition; // "pullCatch" | "close"
        std::string gripSource; // grab point mode that seated the object
        std::string capturedLocal;

        // Mesh the finger solve indexed, object-node local, 9 floats per triangle.
        std::vector<float> trianglesObjectLocal;

        // Solve inputs.
        Transform solveObjectWorld;
        Transform solveHandWorld;
        Transform liveHandWorld;
        Vec3 grabAnchorWorld{};
        FingerSkeleton solveSkeleton; // live skeleton rebased onto solveHandWorld
        CapsuleOptions options;

        // Capsule solve outputs, then after ROCK's pinch policy (what was published).
        std::array<float, kFingerCount> solvedValues{};
        std::array<std::uint8_t, kFingerCount> solvedContact{};
        std::array<std::uint8_t, kFingerCount> solvedOpenedToClear{};
        std::array<std::uint8_t, kFingerCount> solvedBlocked{};
        std::array<float, kFingerCount> finalValues{};
        std::array<float, kFingerBoneCount> finalJointValues{};

        // FRIK forward model: local transforms for the authored closed (0) and open (1) poses.
        std::array<Transform, kFingerBoneCount> frikClosedLocal{};
        std::array<Transform, kFingerBoneCount> frikOpenLocal{};
        bool frikRigValid = false;

        // Published state at the measurement.
        std::array<float, kFingerBoneCount> publishedJointValues{};

        // Drawn once settled: the finger chain taken relative to its own wrist and placed at the
        // published hand (the drawn hand bone); presentedHandWorld is the wrist FRIK latched at
        // AfterWorldFinal of the previous frame.
        Transform renderedObjectWorld;
        Transform renderedVisualHandWorld;
        FingerSkeleton rendered;
        Transform presentedHandWorld;
        bool presentedHandValid = false;
    };

    [[nodiscard]] std::string serialize(const Record& record);
    [[nodiscard]] std::optional<Record> parse(std::string_view text, std::string* outError = nullptr);
}
