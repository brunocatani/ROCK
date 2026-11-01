// Optimized-versus-reference equivalence: the support finger candidate selection walking each
// part's equip-time WeaponTriangleIndex must select exactly what scanning every triangle selects.
// Geometry: the recorded meshes in the grab replay fixtures, placed as weapon parts.
#include "physics-interaction/weapon/SupportGripTriangleSelection.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace
{
    namespace selection = rock::support_grip_triangle_selection;
    using rock::TriangleData;
    using rock::WeaponTriangleIndex;

    int g_failures = 0;

    void check(bool condition, const char* what, int trial)
    {
        if (!condition) {
            ++g_failures;
            std::printf("FAIL trial %d: %s\n", trial, what);
        }
    }

    std::vector<TriangleData> loadFixtureMesh(const char* name)
    {
        std::ifstream file(std::string(ROCK_TEST_FIXTURE_DIR) + "/grab_replay/" + name);
        const auto json = nlohmann::json::parse(file);
        const auto& values = json.at("trianglesObjectLocal");
        std::vector<TriangleData> mesh(values.size() / 9);
        for (std::size_t i = 0; i < mesh.size(); ++i) {
            const auto at = [&](std::size_t k) { return values[i * 9 + k].get<float>(); };
            mesh[i].v0 = { at(0), at(1), at(2) };
            mesh[i].v1 = { at(3), at(4), at(5) };
            mesh[i].v2 = { at(6), at(7), at(8) };
        }
        return mesh;
    }

    RE::NiTransform randomTransform(std::mt19937& random)
    {
        std::normal_distribution<float> normal(0.0f, 1.0f);
        float qx = normal(random), qy = normal(random), qz = normal(random), qw = normal(random);
        const float length = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        qx /= length, qy /= length, qz /= length, qw /= length;
        RE::NiTransform t{};
        t.rotate.entry[0][0] = 1 - 2 * (qy * qy + qz * qz);
        t.rotate.entry[0][1] = 2 * (qx * qy - qz * qw);
        t.rotate.entry[0][2] = 2 * (qx * qz + qy * qw);
        t.rotate.entry[1][0] = 2 * (qx * qy + qz * qw);
        t.rotate.entry[1][1] = 1 - 2 * (qx * qx + qz * qz);
        t.rotate.entry[1][2] = 2 * (qy * qz - qx * qw);
        t.rotate.entry[2][0] = 2 * (qx * qz - qy * qw);
        t.rotate.entry[2][1] = 2 * (qy * qz + qx * qw);
        t.rotate.entry[2][2] = 1 - 2 * (qx * qx + qy * qy);
        std::uniform_real_distribution<float> offset(-20.0f, 20.0f);
        t.translate = { offset(random), offset(random), offset(random) };
        t.scale = std::uniform_real_distribution<float>(0.8f, 1.3f)(random);
        return t;
    }

    bool sameTriangle(const TriangleData& a, const TriangleData& b)
    {
        return std::memcmp(&a, &b, sizeof(TriangleData)) == 0;
    }

    struct Run
    {
        std::vector<TriangleData> selected;
        selection::Stats stats;
    };

    Run select(std::vector<selection::Part> parts, bool useIndex, const selection::References& references, std::size_t laneLimit, std::size_t globalLimit)
    {
        if (!useIndex) {
            for (auto& part : parts) part.index = nullptr;
        }
        selection::Rankings rankings{};
        Run run{};
        selection::rankParts(parts, references, laneLimit, globalLimit, rankings, &run.stats);
        selection::mergeRankings(rankings, globalLimit, run.selected);
        return run;
    }
}

int main()
{
    std::vector<std::vector<TriangleData>> meshes{ loadFixtureMesh("typewriter_pull_right.json"), loadFixtureMesh("basketball_pull_right.json"),
        loadFixtureMesh("basketball_close_left.json") };
    // A non-finite triangle: the index omits it and the scan skips it, but it still consumes a scan ordinal.
    meshes[1][17].v1.x = std::numeric_limits<float>::quiet_NaN();
    std::vector<WeaponTriangleIndex> indices(meshes.size());
    std::size_t totalTriangles = 0;
    for (std::size_t i = 0; i < meshes.size(); ++i) {
        indices[i].build(meshes[i]);
        totalTriangles += meshes[i].size();
    }

    std::mt19937 random(20261101);
    const std::size_t limits[][2] = { { 410, 2048 }, { 13, 64 }, { 3, 7 }, { 1, 1 } };
    std::uint64_t indexedMeasured = 0, scanMeasured = 0;
    int trial = 0;
    for (int repeat = 0; repeat < 60; ++repeat) {
        std::vector<selection::Part> parts;
        std::uint64_t ordinal = 0;
        for (std::size_t i = 0; i < meshes.size(); ++i) {
            parts.push_back({ .sourceTriangles = meshes[i], .index = &indices[i], .sourceToWeapon = randomTransform(random), .firstOrdinal = ordinal });
            ordinal += meshes[i].size();
        }

        // Seat on or near a random weapon-frame vertex; lane points around it, some lanes empty.
        const auto& part = parts[random() % parts.size()];
        const auto& triangle = part.sourceTriangles[random() % part.sourceTriangles.size()];
        RE::NiPoint3 anchor = rock::transform_math::localPointToWorld(part.sourceToWeapon, triangle.v0);
        if (!std::isfinite(anchor.x)) anchor = part.sourceToWeapon.translate;
        std::uniform_real_distribution<float> jitter(-3.0f, 3.0f), spread(-8.0f, 8.0f);
        selection::References references{};
        references.seatPoint = { anchor.x + jitter(random), anchor.y + jitter(random), anchor.z + jitter(random) };
        for (std::size_t lane = 0; lane < selection::kLaneCount; ++lane) {
            references.laneCounts[lane] = random() % (selection::kLaneReferenceCapacity + 1);
            for (std::size_t k = 0; k < references.laneCounts[lane]; ++k) {
                references.lanePoints[lane][k] = { references.seatPoint.x + spread(random), references.seatPoint.y + spread(random),
                    references.seatPoint.z + spread(random) };
            }
        }

        for (const auto& limit : limits) {
            ++trial;
            const Run indexed = select(parts, true, references, limit[0], limit[1]);
            const Run scanned = select(parts, false, references, limit[0], limit[1]);
            check(indexed.selected.size() == scanned.selected.size(), "selection size", trial);
            bool same = indexed.selected.size() == scanned.selected.size();
            for (std::size_t i = 0; same && i < indexed.selected.size(); ++i) same = sameTriangle(indexed.selected[i], scanned.selected[i]);
            check(same, "selected triangles and order", trial);
            check(scanned.stats.trianglesMeasured == totalTriangles - 1, "scan measured every finite triangle", trial);
            indexedMeasured += indexed.stats.trianglesMeasured;
            scanMeasured += scanned.stats.trianglesMeasured;
        }
    }

    std::printf("%d trials, %zu triangles in 3 parts: indexed walk measured %llu triangles, full scan %llu (%.1f%%)\n", trial, totalTriangles,
        static_cast<unsigned long long>(indexedMeasured), static_cast<unsigned long long>(scanMeasured),
        100.0 * static_cast<double>(indexedMeasured) / static_cast<double>(scanMeasured));
    check(indexedMeasured < scanMeasured, "the index skips work", 0);
    if (g_failures != 0) {
        std::printf("%d failures\n", g_failures);
        return 1;
    }
    std::printf("selection identical in every trial\n");
    return 0;
}
