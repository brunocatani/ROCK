#include "physics-interaction/debug/DebugConvexHullMesh.h"
#include "physics-interaction/debug/DebugOverlayLineBatch.h"
#include "physics-interaction/debug/DebugOverlayPolicy.h"

#include <cmath>
#include <cstdio>
#include <vector>

namespace
{
    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }

        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, bool value)
    {
        if (!value) {
            return true;
        }

        std::printf("%s expected false\n", label);
        return false;
    }

    bool expectFloat(const char* label, float actual, float expected, float epsilon = 0.0001f)
    {
        if (std::fabs(actual - expected) <= epsilon) {
            return true;
        }

        std::printf("%s expected %.5f got %.5f\n", label, expected, actual);
        return false;
    }
}

int main()
{
    using namespace rock::debug_overlay_policy;

    bool ok = true;

    ok &= expectFloat("line vertex budget clamps negative", static_cast<float>(clampLineVertexBudget(-4)), 0.0f);
    ok &= expectFloat("line vertex budget clamps high", static_cast<float>(clampLineVertexBudget(999999)), 65536.0f);
    ok &= expectFloat("shape cache budget clamps low", static_cast<float>(clampShapeCacheBudget(0)), 16.0f);
    ok &= expectFloat("shape cache budget clamps high", static_cast<float>(clampShapeCacheBudget(999999)), 4096.0f);
    ok &= expectFloat("convex support vertex clamp hard cap", static_cast<float>(clampMaxConvexSupportVertices(999)), 32.0f);

    ok &= expectTrue("supported sphere uses detail", chooseShapeDecodeMode(2, 12, 6, true) == ShapeDecodeMode::Detailed);
    ok &= expectTrue("heavy convex uses proxy", chooseShapeDecodeMode(1, 12, 6, true) == ShapeDecodeMode::Proxy);
    ok &= expectTrue("unsupported shape uses proxy", chooseShapeDecodeMode(99, 0, 6, true) == ShapeDecodeMode::Proxy);
    ok &= expectTrue("unsupported shape can be skipped when proxy disabled", chooseShapeDecodeMode(99, 0, 6, false) == ShapeDecodeMode::Unsupported);

    const auto keyA = makeOverlaySettingsKey(true, true, true, true, true, true, true, true, true, 48, 100, 100, 6, true, 8192, 512);
    const auto keyB = makeOverlaySettingsKey(true, true, true, true, true, true, true, true, true, 48, 100, 100, 6, true, 16384, 512);
    const auto keyC = makeOverlaySettingsKey(true, true, true, true, true, true, true, true, true, 48, 100, 100, 6, true, 8192, 1024);
    ok &= expectFalse("line vertex budget changes cache key", keyA == keyB);
    ok &= expectFalse("shape cache budget changes cache key", keyA == keyC);
    ok &= expectTrue("shape decode key stable across unrelated overlay settings",
        makeShapeDecodeSettingsKey(8, true) == makeShapeDecodeSettingsKey(8, true));
    ok &= expectFalse("shape decode key changes with convex detail cap",
        makeShapeDecodeSettingsKey(8, true) == makeShapeDecodeSettingsKey(6, true));
    ok &= expectFalse("shape decode key changes with bounds fallback",
        makeShapeDecodeSettingsKey(8, true) == makeShapeDecodeSettingsKey(8, false));

    rock::debug_overlay_line_batch::LineBatch batch;
    ok &= expectTrue("empty batch accepts line", batch.addLine({ 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f, 1.0f }, 8));
    ok &= expectTrue("point marker fits exact remaining budget", batch.addPointMarker({ 0.0f, 1.0f, 0.0f }, 0.5f, { 0.0f, 1.0f, 0.0f, 1.0f }, 8));
    ok &= expectFloat("line batch counted line and point marker vertices", static_cast<float>(batch.vertexCount()), 8.0f);
    ok &= expectFloat("line batch counted four logical lines", static_cast<float>(batch.lineCount()), 4.0f);
    ok &= expectFalse("line batch rejects over-budget append", batch.addLine({ 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, 1.0f, 1.0f }, 8));
    ok &= expectFloat("line batch tracks rejected lines", static_cast<float>(batch.rejectedLineCount()), 1.0f);

    batch.clear();
    ok &= expectTrue("line batch accepts first canonical segment", batch.addLine({ 2.0f, 0.0f, 0.0f }, { 3.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 0.0f, 1.0f }, 8));
    ok &= expectFalse("line batch rejects reversed duplicate segment", batch.addLine({ 3.0f, 0.0f, 0.0f }, { 2.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 0.0f, 1.0f }, 8));
    ok &= expectFalse("line batch rejects zero length segment", batch.addLine({ 4.0f, 0.0f, 0.0f }, { 4.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 0.0f, 1.0f }, 8));
    ok &= expectFloat("line batch keeps only unique segment vertices", static_cast<float>(batch.vertexCount()), 2.0f);

    using rock::debug_convex_hull_mesh::Vec3;
    const std::vector<Vec3> cubeWithDuplicates{
        { -1.0f, -1.0f, -1.0f },
        { 1.0f, -1.0f, -1.0f },
        { 1.0f, 1.0f, -1.0f },
        { -1.0f, 1.0f, -1.0f },
        { -1.0f, -1.0f, 1.0f },
        { 1.0f, -1.0f, 1.0f },
        { 1.0f, 1.0f, 1.0f },
        { -1.0f, 1.0f, 1.0f },
        { -1.0f, -1.0f, -1.0f },
        { 1.0f, 1.0f, 1.0f },
    };
    auto uniqueCube = rock::debug_convex_hull_mesh::deduplicateVertices(cubeWithDuplicates);
    auto cubeTriangles = rock::debug_convex_hull_mesh::triangulateConvexHullFaces(uniqueCube);
    ok &= expectFloat("convex mesh deduplicates support vertices", static_cast<float>(uniqueCube.size()), 8.0f);
    ok &= expectFloat("convex mesh triangulates cube faces once", static_cast<float>(cubeTriangles.size()), 12.0f);

    return ok ? 0 : 1;
}
