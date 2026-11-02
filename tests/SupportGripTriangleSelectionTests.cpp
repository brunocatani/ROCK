// Optimized-versus-reference equivalence: the support finger candidate selection walking each
// part's equip-time WeaponTriangleIndex must select exactly what scanning every triangle selects.
// Geometry: the recorded meshes in the grab replay fixtures, placed as weapon parts.
#include "physics-interaction/weapon/grip/SupportGripTriangleSelection.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <random>
#include <utility>
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
    // Triangles with a repeated vertex: the exact distance routine returns NaN (0/0 on the v0-v1 edge) for points on
    // the v2 side, and a NaN seat distance keeps a triangle out of the global ranking. Skipped lane distances must
    // not change that.
    std::vector<std::pair<std::size_t, std::size_t>> collinear;
    for (std::size_t mesh = 0; mesh < meshes.size(); ++mesh) {
        for (std::size_t i = 3; i < meshes[mesh].size(); i += 41) {
            meshes[mesh][i].v1 = meshes[mesh][i].v0;
            collinear.emplace_back(mesh, i);
        }
    }
    std::vector<WeaponTriangleIndex> indices(meshes.size());
    for (std::size_t i = 0; i < meshes.size(); ++i) {
        indices[i].build(meshes[i]);
    }

    std::mt19937 random(20261101);
    const std::size_t limits[][2] = { { 410, 2048 }, { 13, 64 }, { 3, 7 }, { 1, 1 } };
    int trial = 0;
    std::uint64_t nanSeatDistances = 0;
    // 3 parts: each mesh once. 12 parts: the meshes repeated at other placements, like a detailed rifle's many parts,
    // where the walk order decides how much is skipped.
    const std::pair<std::size_t, int> phases[] = { { 3, 60 }, { 12, 10 } };
    for (const auto& [partCount, repeats] : phases) {
        std::uint64_t indexedMeasured = 0, scanMeasured = 0, indexedDistances = 0, scanDistances = 0;
        std::size_t totalTriangles = 0, finiteTriangles = 0;
        for (std::size_t i = 0; i < partCount; ++i) {
            totalTriangles += meshes[i % meshes.size()].size();
            finiteTriangles += meshes[i % meshes.size()].size() - (i % meshes.size() == 1 ? 1 : 0);
        }
        for (int repeat = 0; repeat < repeats; ++repeat) {
            std::vector<selection::Part> parts;
            std::uint64_t ordinal = 0;
            for (std::size_t i = 0; i < partCount; ++i) {
                const std::size_t mesh = i % meshes.size();
                parts.push_back({ .sourceTriangles = meshes[mesh], .index = &indices[mesh], .sourceToWeapon = randomTransform(random), .firstOrdinal = ordinal });
                ordinal += meshes[mesh].size();
            }

            // Seat on or near a random weapon-frame vertex; lane points around it, some lanes empty.
            const auto& part = parts[random() % parts.size()];
            const auto& triangle = part.sourceTriangles[random() % part.sourceTriangles.size()];
            RE::NiPoint3 anchor = rock::transform_math::localPointToWorld(part.sourceToWeapon, triangle.v0);
            if (!std::isfinite(anchor.x)) anchor = part.sourceToWeapon.translate;
            std::uniform_real_distribution<float> jitter(-3.0f, 3.0f), spread(-8.0f, 8.0f);
            selection::References references{};
            references.seatPoint = { anchor.x + jitter(random), anchor.y + jitter(random), anchor.z + jitter(random) };
            std::size_t referencePoints = 1;
            for (std::size_t lane = 0; lane < selection::kLaneCount; ++lane) {
                references.laneCounts[lane] = random() % (selection::kLaneReferenceCapacity + 1);
                referencePoints += references.laneCounts[lane];
                for (std::size_t k = 0; k < references.laneCounts[lane]; ++k) {
                    references.lanePoints[lane][k] = { references.seatPoint.x + spread(random), references.seatPoint.y + spread(random),
                        references.seatPoint.z + spread(random) };
                }
            }
            for (std::size_t p = 0; p < parts.size(); ++p) {
                for (const auto& [mesh, i] : collinear) {
                    if (mesh != p % meshes.size()) continue;
                    const auto& source = meshes[mesh][i];
                    const TriangleData placed{ rock::transform_math::localPointToWorld(parts[p].sourceToWeapon, source.v0),
                        rock::transform_math::localPointToWorld(parts[p].sourceToWeapon, source.v1),
                        rock::transform_math::localPointToWorld(parts[p].sourceToWeapon, source.v2) };
                    float distanceSquared = 0.0f;
                    (void)rock::closestPointOnTriangleToPoint(references.seatPoint, placed, distanceSquared);
                    if (std::isnan(distanceSquared)) ++nanSeatDistances;
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
                check(scanned.stats.trianglesMeasured == finiteTriangles, "scan measured every finite triangle", trial);
                check(scanned.stats.exactDistances == finiteTriangles * referencePoints, "scan computed every distance", trial);
                indexedMeasured += indexed.stats.trianglesMeasured;
                scanMeasured += scanned.stats.trianglesMeasured;
                indexedDistances += indexed.stats.exactDistances;
                scanDistances += scanned.stats.exactDistances;
            }
        }

        std::printf("%d trials so far, %zu triangles in %zu parts: indexed walk measured %.1f%% of the triangles and computed %.1f%% of the distances\n",
            trial, totalTriangles, partCount, 100.0 * static_cast<double>(indexedMeasured) / static_cast<double>(scanMeasured),
            100.0 * static_cast<double>(indexedDistances) / static_cast<double>(scanDistances));
        check(indexedDistances < scanDistances, "the index skips work", 0);
    }
    std::printf("repeated-vertex triangles with a NaN seat distance: %llu\n", static_cast<unsigned long long>(nanSeatDistances));
    if (g_failures != 0) {
        std::printf("%d failures\n", g_failures);
        return 1;
    }
    std::printf("selection identical in every trial\n");
    return 0;
}
