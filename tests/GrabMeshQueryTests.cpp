#include "physics-interaction/grab/MeshGrab.h"

#include <cassert>
#include <chrono>
#include <cstdio>
#include <random>

namespace
{
    using namespace rock;

    bool same(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }

    void sameHit(const GrabSurfaceHit& a, const GrabSurfaceHit& b)
    {
        assert(a.valid == b.valid);
        if (!a.valid) return;
        assert(a.triangleIndex == b.triangleIndex && a.sourceTriangleIndex == b.sourceTriangleIndex);
        assert(a.sourceNode == b.sourceNode && a.sourceShape == b.sourceShape && a.sourceKind == b.sourceKind);
        assert(a.hasSkinInfluences == b.hasSkinInfluences && a.hasTriangle == b.hasTriangle);
        assert(a.distance == b.distance && same(a.position, b.position) && same(a.normal, b.normal));
        assert(a.signedAlongPalmDistanceGameUnits == b.signedAlongPalmDistanceGameUnits);
        assert(a.lateralPalmDistanceGameUnits == b.lateralPalmDistanceGameUnits);
    }

    std::vector<GrabSurfaceTriangleData> makeMesh(std::size_t count, float translation = 0)
    {
        std::vector<GrabSurfaceTriangleData> mesh(count);
        for (std::size_t i = 0; i < count; ++i) {
            const RE::NiPoint3 p{ translation + static_cast<float>(i / 256) * 0.4f,
                static_cast<float>(i % 256) * 0.15f, static_cast<float>(i % 7) * 0.01f };
            mesh[i].triangle = { p, p + RE::NiPoint3{0.35f, 0, 0}, p + RE::NiPoint3{0, 0.13f, 0} };
            mesh[i].triangleIndex = static_cast<std::uint32_t>(count - i);
            // Opaque metadata is carried through without dereferencing engine objects.
            mesh[i].sourceNode = reinterpret_cast<RE::NiAVObject*>((i % 5 + 1) * 4096);
        }
        return mesh;
    }

    void compareNearest(const std::vector<GrabSurfaceTriangleData>& mesh, const GrabSurfaceQueryIndex& index,
        const RE::NiPoint3& point, std::size_t count)
    {
        // Independent exhaustive ranking is the pre-optimization selection rule.
        std::vector<std::pair<float, std::size_t>> expected;
        for (std::size_t i = 0; i < mesh.size(); ++i) {
            const auto& t = mesh[i].triangle;
            const auto center = (t.v0 + t.v1 + t.v2) * (1.0f / 3.0f);
            expected.emplace_back((std::min)({ vector_math::lengthSquared(center - point),
                vector_math::lengthSquared(t.v0 - point), vector_math::lengthSquared(t.v1 - point),
                vector_math::lengthSquared(t.v2 - point) }), i);
        }
        std::sort(expected.begin(), expected.end());
        const auto& actual = index.nearest(mesh, point, count);
        assert(actual.size() == (std::min)(count, mesh.size()));
        for (std::size_t i = 0; i < actual.size(); ++i) {
            assert(actual[i].index == expected[i].second && actual[i].distanceSquared == expected[i].first);
        }
        const auto cached = actual;
        const auto& repeated = index.nearest(mesh, point, count);
        assert(repeated.size() == cached.size());
        for (std::size_t i = 0; i < repeated.size(); ++i) assert(repeated[i].index == cached[i].index);
    }

    void compareQueries(std::vector<GrabSurfaceTriangleData>& mesh, std::mt19937& random, float translation)
    {
        GrabSurfaceQueryIndex index;
        index.build(mesh);
        std::uniform_real_distribution<float> x(-2, 90), y(-2, 42), z(-3, 3);
        for (int query = 0; query < 80; ++query) {
            const auto point = query < 8 && !mesh.empty() ? mesh[query % mesh.size()].triangle.v0 :
                RE::NiPoint3{ translation + x(random), y(random), z(random) };
            const RE::NiPoint3 normal{ z(random), z(random), z(random) };
            const float radius = query % 9 == 0 ? 0.0f : static_cast<float>(query % 13);
            GrabSurfaceHit linear{}, indexed{};
            assert(findClosestGrabSurfaceHitToPointPositionOnly(mesh, point, normal, radius, linear) ==
                findClosestGrabSurfaceHitToPointPositionOnly(mesh, point, normal, radius, indexed, &index));
            sameHit(linear, indexed);
            linear = {}; indexed = {};
            assert(findClosestGrabSurfaceHitToPoint(mesh, point, normal, radius, 65.0f, linear) ==
                findClosestGrabSurfaceHitToPoint(mesh, point, normal, radius, 65.0f, indexed, &index));
            sameHit(linear, indexed);
            if (query % 10 == 0) compareNearest(mesh, index, point, 2048);
        }
        // In-place owner filtering and geometry replacement require a rebuild,
        // including when allocation identity and triangle count do not change.
        if (mesh.size() > 3000) {
            std::reverse(mesh.begin(), mesh.end());
            index.build(mesh);
            compareNearest(mesh, index, {translation + 4, 5, 0}, 2048);
            mesh.erase(mesh.begin(), mesh.begin() + 23);
            index.build(mesh);
            compareNearest(mesh, index, {translation + 4, 5, 0}, 2048);
        }
    }

    void extractionParity(bool fullPrecision)
    {
        std::array<std::array<float, 3>, 4> floats{ { {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0} } };
        std::array<std::array<std::uint16_t, 3>, 4> halves{ { {0, 0, 0}, {0x3c00, 0, 0}, {0, 0x3c00, 0}, {0x3c00, 0x3c00, 0} } };
        std::array<std::uint16_t, 15> indices{0, 1, 2, 1, 3, 2, 0, 1, 2, 0, 4, 2, 3, 2, 1};
        TriShapeRawGeometry geometry;
        geometry.numVertices = 4;
        geometry.numTriangles = 5;
        geometry.triangles = indices.data();
        const auto* bytes = reinterpret_cast<const std::uint8_t*>(fullPrecision ? static_cast<void*>(floats.data()) : static_cast<void*>(halves.data()));
        const auto stride = fullPrecision ? 12u : 6u;
        for (float scale : { 0.0f, -2.0f, 1.0f, 1.37f }) {
            RE::NiTransform transform{};
            transform.rotate.entry[0][1] = 1;
            transform.rotate.entry[1][0] = -1;
            transform.rotate.entry[2][2] = 1;
            transform.translate = { 120034.0f, -76543.0f, 28.0f };
            transform.scale = scale;
            const std::vector<std::uint8_t> visible{1, 1, 0, 1, 1};
            std::vector<TriangleData> world, local;
            std::vector<GrabSurfaceTriangleData> surface;
            assert(appendStaticMeshTriangles(geometry, bytes, stride, 0, fullPrecision, visible, transform, nullptr, world, &surface, &local) == 3);
            std::size_t emitted = 0;
            for (std::uint32_t i : {0u, 1u, 4u}) {
                TriangleData expected{ readVertexPosition(bytes + indices[i * 3] * stride, 0, fullPrecision),
                    readVertexPosition(bytes + indices[i * 3 + 1] * stride, 0, fullPrecision),
                    readVertexPosition(bytes + indices[i * 3 + 2] * stride, 0, fullPrecision) };
                assert(same(local[emitted].v0, expected.v0) && same(local[emitted].v1, expected.v1) && same(local[emitted].v2, expected.v2));
                expected.applyTransform(transform);
                assert(same(world[emitted].v0, expected.v0) && same(world[emitted].v1, expected.v1) && same(world[emitted].v2, expected.v2));
                assert(same(surface[emitted].triangle.v0, expected.v0) && surface[emitted].triangleIndex == i);
                ++emitted;
            }
            // Appending multiple shapes preserves existing output and optional outputs.
            assert(appendStaticMeshTriangles(geometry, bytes, stride, 0, fullPrecision, visible, transform, nullptr, world, nullptr, nullptr) == 3);
            assert(world.size() == 6);
        }
    }

    void boundaryAndRebuildCases()
    {
        auto mesh = makeMesh(8192);
        for (auto& surface : mesh) surface.triangle = { {500, 0, 0}, {501, 0, 0}, {500, 1, 0} };
        mesh[31].triangle = { {0, 0, 0}, {2, 0, 0}, {0, 2, 0} };
        mesh[4097].triangle = mesh[31].triangle;
        GrabSurfaceQueryIndex index;
        index.build(mesh);
        for (const auto point : { RE::NiPoint3{0, 0, 0}, RE::NiPoint3{1, 1, 0}, RE::NiPoint3{0.5f, 0.5f, 1} }) {
            for (float radius : {0.0f, std::nextafter(1.0f, 0.0f), 1.0f, std::nextafter(1.0f, 2.0f)}) {
                GrabSurfaceHit linear{}, indexed{};
                const bool expected = findClosestGrabSurfaceHitToPointPositionOnly(mesh, point, {0, 0, -1}, radius, linear);
                assert(expected == findClosestGrabSurfaceHitToPointPositionOnly(mesh, point, {0, 0, -1}, radius, indexed, &index));
                sameHit(linear, indexed);
                if (expected) assert(indexed.triangleIndex == 31);
            }
            compareNearest(mesh, index, point, 2048);
        }
        compareNearest(mesh, index, {1.0e20f, 0, 0}, 2048); // Overflowed distances retain index ties.
        // A different source must not borrow either bounds or cached rankings.
        auto other = makeMesh(8192);
        compareNearest(other, index, {0, 0, 0}, 2048);
        compareNearest(mesh, index, {0, 0, 0}, 2048);
        mesh.clear();
        index.build(mesh);
        compareNearest(mesh, index, {}, 2048);
    }

    void measureQueries()
    {
        auto mesh = makeMesh(55826);
        GrabSurfaceQueryIndex index;
        const auto start = std::chrono::steady_clock::now();
        index.build(mesh);
        const auto built = std::chrono::steady_clock::now();
        const RE::NiPoint3 point{ 42, 18, 1 }, normal{0, 0, 1};
        std::size_t broadPhaseTests = index.visit(mesh, point, [] { return 4.0f; }, [](std::size_t) {});
        assert(broadPhaseTests * 10 < mesh.size());
        int hits = 0;
        for (int i = 0; i < 100; ++i) {
            GrabSurfaceHit hit{};
            hits += findClosestGrabSurfaceHitToPointPositionOnly(mesh, point, normal, 2, hit);
        }
        const auto linear = std::chrono::steady_clock::now();
        for (int i = 0; i < 100; ++i) {
            GrabSurfaceHit hit{};
            hits += findClosestGrabSurfaceHitToPointPositionOnly(mesh, point, normal, 2, hit, &index);
        }
        const auto indexed = std::chrono::steady_clock::now();
        assert(hits == 200);
        const auto ms = [](auto duration) { return std::chrono::duration<double, std::milli>(duration).count(); };
        std::printf("55826-triangle fixture: build %.3f ms, 100 point queries linear %.3f ms / indexed %.3f ms; radius candidates %zu/55826\n",
            ms(built - start), ms(linear - built), ms(indexed - linear), broadPhaseTests);
        // Compare selection against the previous nth_element + sorted-prefix path.
        const auto rankStart = std::chrono::steady_clock::now();
        std::size_t checksum = 0;
        for (int q = 0; q < 20; ++q) {
            const auto center = point + RE::NiPoint3{ q * 0.01f, 0, 0 };
            std::vector<std::pair<float, std::size_t>> ranks;
            ranks.reserve(mesh.size());
            for (std::size_t i = 0; i < mesh.size(); ++i) {
                const auto& t = mesh[i].triangle;
                ranks.emplace_back((std::min)({vector_math::lengthSquared((t.v0 + t.v1 + t.v2) * (1.0f / 3.0f) - center),
                    vector_math::lengthSquared(t.v0 - center), vector_math::lengthSquared(t.v1 - center),
                    vector_math::lengthSquared(t.v2 - center)}), i);
            }
            std::nth_element(ranks.begin(), ranks.begin() + 2048, ranks.end());
            std::sort(ranks.begin(), ranks.begin() + 2048);
            checksum += ranks.front().second;
        }
        const auto rankLinear = std::chrono::steady_clock::now();
        const auto expectedChecksum = checksum;
        checksum = 0;
        for (int q = 0; q < 20; ++q) checksum += index.nearest(mesh, point + RE::NiPoint3{q * 0.01f, 0, 0}, 2048).front().index;
        const auto rankIndexed = std::chrono::steady_clock::now();
        assert(checksum == expectedChecksum);
        std::printf("20 nearest-2048 selections: full ranking %.3f ms / indexed %.3f ms\n",
            ms(rankLinear - rankStart), ms(rankIndexed - rankLinear));
    }
}

int main()
{
    std::mt19937 random(71339);
    for (std::size_t count : {0u, 1u, 2048u, 2049u, 55826u}) {
        auto mesh = makeMesh(count);
        compareQueries(mesh, random, 0);
    }
    auto shuffled = makeMesh(6000, 120000);
    std::shuffle(shuffled.begin(), shuffled.end(), random);
    // Coincident and degenerate faces exercise stable source-order tie selection.
    shuffled[37].triangle = shuffled[29].triangle;
    shuffled[45].triangle = {};
    compareQueries(shuffled, random, 120000);
    auto irregular = makeMesh(6000);
    std::uniform_real_distribution<float> offset(-0.1f, 0.1f);
    for (auto& surface : irregular) {
        surface.triangle.v1 += RE::NiPoint3{offset(random), offset(random), offset(random)};
        surface.triangle.v2 += RE::NiPoint3{offset(random), offset(random), offset(random)};
    }
    compareQueries(irregular, random, 0);
    boundaryAndRebuildCases();
    extractionParity(false);
    extractionParity(true);
    measureQueries();
}
