// The grab pose library's shape functions against their reference: the authoring tool's Python
// (export_rock_grab_library.py --fixture), which also generated the exemplar table. The exemplars
// only mean something to a describeShape/transfer that agrees with the code that produced them.
#include "physics-interaction/grab/GrabPoseLibrary.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

using namespace rock::grab_pose_library;

namespace
{
    int failures = 0;

    void check(bool value, const std::string& message)
    {
        if (!value) {
            std::cerr << message << '\n';
            ++failures;
        }
    }

    struct Point
    {
        float x, y, z;
    };

    struct Triangle
    {
        Point v0, v1, v2;
    };

    bool near(double a, double b, double tolerance) { return std::abs(a - b) <= tolerance; }
}

int main()
{
    std::ifstream file(std::string(ROCK_TEST_FIXTURE_DIR) + "/grab_pose_library.json");
    check(file.good(), "fixture grab_pose_library.json not found");
    if (!file.good()) {
        return 1;
    }
    const auto fixture = nlohmann::json::parse(file);
    check(!fixture["cases"].empty(), "fixture holds no cases");

    for (const auto& item : fixture["cases"]) {
        const std::string name = item["name"].get<std::string>();
        const auto flat = item["triangles"].get<std::vector<float>>();
        std::vector<Triangle> triangles(flat.size() / 9);
        for (std::size_t i = 0; i < triangles.size(); ++i) {
            const float* p = flat.data() + i * 9;
            triangles[i] = { { p[0], p[1], p[2] }, { p[3], p[4], p[5] }, { p[6], p[7], p[8] } };
        }

        const Shape shape = describeShape(triangles, 1.0);
        check(shape.valid, name + ": shape not described");
        const auto center = item["center"].get<std::vector<double>>();
        const auto axes = item["axes"].get<std::vector<double>>();
        const auto dims = item["dims"].get<std::vector<double>>();
        const auto features = item["features"].get<std::vector<double>>();
        for (int i = 0; i < 3; ++i) {
            check(near(shape.center[i], center[i], 1e-3), name + ": centre differs");
            check(near(shape.dims[i], dims[i], 1e-3), name + ": extents differ");
            for (int j = 0; j < 3; ++j) {
                check(near(shape.axes[i][j], axes[i * 3 + j], 1e-4), name + ": shape frame differs");
            }
        }
        check(features.size() == kFeatureCount, name + ": feature count differs");
        for (std::size_t i = 0; i < kFeatureCount && i < features.size(); ++i) {
            check(near(shape.features[i], features[i], 1e-3), name + ": feature " + std::to_string(i) + " differs");
        }

        const auto samples = sampleSurface(triangles, 1.0);
        check(samples.size() == item["sampleCount"].get<std::size_t>(), name + ": sample count differs");
        if (!samples.empty()) {
            const auto first = item["sampleFirst"].get<std::vector<double>>();
            const auto last = item["sampleLast"].get<std::vector<double>>();
            for (int i = 0; i < 3; ++i) {
                check(near(samples.front()[i], first[i], 1e-3) && near(samples.back()[i], last[i], 1e-3),
                    name + ": surface samples differ");
            }
        }

        Exemplar exemplar{};
        const auto canonToPalm = item["exemplar"]["canonToPalm"].get<std::vector<float>>();
        const auto palmNorm = item["exemplar"]["palmNorm"].get<std::vector<float>>();
        std::copy(canonToPalm.begin(), canonToPalm.end(), exemplar.canonToPalm);
        std::copy(palmNorm.begin(), palmNorm.end(), exemplar.palmNorm);
        exemplar.clearance = item["exemplar"]["clearance"].get<float>();
        exemplar.hasClearance = item["exemplar"]["hasClearance"].get<bool>();
        for (const auto& expected : item["transfers"]) {
            const auto heading = expected["heading"].get<std::uint32_t>();
            const Pose pose = transfer(exemplar, shape, samples, heading);
            const auto rotate = expected["rotate"].get<std::vector<double>>();
            const auto translate = expected["translate"].get<std::vector<double>>();
            for (int i = 0; i < 9; ++i) {
                check(near(pose.rotate[i], rotate[i], 1e-4),
                    name + ": rotation differs at heading " + std::to_string(heading));
            }
            for (int i = 0; i < 3; ++i) {
                check(near(pose.translate[i], translate[i], 5e-3),
                    name + ": translation differs at heading " + std::to_string(heading));
            }
        }
    }

    // The generated table: finite, proper rotations, and reachable through candidates().
    for (const bool left : { false, true }) {
        const auto table = exemplars(left);
        check(!table.empty(), "exemplar table is empty");
        for (const auto& exemplar : table) {
            bool finite = std::isfinite(exemplar.clearance);
            for (const float value : exemplar.features) {
                finite = finite && std::isfinite(value);
            }
            const float* m = exemplar.canonToPalm;
            const double determinant = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) +
                m[2] * (m[3] * m[7] - m[4] * m[6]);
            check(finite && near(determinant, 1.0, 1e-3), std::string(exemplar.name) + ": exemplar is not a finite rotation");
        }
    }
    {
        // A mesh described from an exemplar's own features finds that exemplar first.
        const auto table = exemplars(false);
        Shape shape{};
        shape.valid = true;
        shape.dims = { 10, 5, 5 };
        shape.axes = { Vec3{ 1, 0, 0 }, Vec3{ 0, 1, 0 }, Vec3{ 0, 0, 1 } };
        for (std::size_t i = 0; i < kFeatureCount; ++i) {
            shape.features[i] = table.front().features[i];
        }
        const auto found = candidates(shape, {}, false);
        check(found.size() >= 4 && found.front().signatureDistance < 1e-6 && found.front().rank == 0 &&
                found[3].heading == 3,
            "candidates do not start with the nearest exemplar's four headings");
        for (auto& feature : shape.features) {
            feature = 1000.0;
        }
        check(candidates(shape, {}, false).empty(), "a mesh unlike every exemplar still got candidates");
    }

    if (failures) {
        std::cerr << failures << " grab pose library failure(s)\n";
        return 1;
    }
    return 0;
}
