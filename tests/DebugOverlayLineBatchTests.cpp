#include <cmath>
#include <iostream>
#include <limits>

#include "physics-interaction/debug/DebugOverlayLineBatch.h"

namespace
{
    bool expect(bool condition, const char* message)
    {
        if (!condition) {
            std::cerr << message << '\n';
            return false;
        }
        return true;
    }
}

int main()
{
    using namespace rock::debug_overlay_line_batch;

    bool passed = true;
    LineBatch batch;
    passed &= expect(batch.prepare(8), "Reusable line scratch preparation failed.");
    passed &= expect(batch.preparedMaxVertices() == 8, "Prepared line capacity changed.");

    const Vec3 a{ 1.0f, 2.0f, 3.0f };
    const Vec3 b{ 4.0f, 5.0f, 6.0f };
    const Vec3 c{ 7.0f, 8.0f, 9.0f };
    const Rgba red{ 1.0f, 0.0f, 0.0f, 0.8f };
    const Rgba blue{ 0.0f, 0.2f, 1.0f, 0.6f };

    passed &= expect(batch.addLine(a, b, red, 8), "First line was rejected.");
    passed &= expect(!batch.addLine(b, a, red, 8), "Reversed duplicate was not coalesced.");
    passed &= expect(batch.addLine(b, c, blue, 8), "Second ordered line was rejected.");
    passed &= expect(batch.segments().size() == 2 && batch.segments()[0].color == red && batch.segments()[1].color == blue,
        "Line insertion order or per-vertex color was not preserved.");
    passed &= expect(batch.rejectedLineCount() == 1, "Duplicate rejection was not counted.");

    batch.clear();
    passed &= expect(batch.empty() && batch.rejectedLineCount() == 0 && batch.preparedMaxVertices() == 8,
        "Clear did not retain prepared scratch while resetting frame state.");
    passed &= expect(batch.addLine(a, b, red, 8), "Generation-based dedup did not reset after clear.");

    const float nan = std::numeric_limits<float>::quiet_NaN();
    passed &= expect(!batch.addLine(Vec3{ nan, 0.0f, 0.0f }, b, red, 8), "Non-finite line input was accepted.");
    passed &= expect(!batch.addLine(a, a, red, 8), "Degenerate line was accepted.");
    passed &= expect(batch.addLine(b, c, red, 8), "Capacity setup line was rejected.");
    passed &= expect(batch.addLine(c, Vec3{ 10.0f, 11.0f, 12.0f }, red, 8), "Capacity setup line was rejected.");
    passed &= expect(batch.addLine(Vec3{ 10.0f, 11.0f, 12.0f }, Vec3{ 13.0f, 14.0f, 15.0f }, red, 8),
        "Prepared final line was rejected.");
    passed &= expect(!batch.addLine(Vec3{ 13.0f, 14.0f, 15.0f }, Vec3{ 16.0f, 17.0f, 18.0f }, red, 16),
        "Caller budget bypassed the prepared hard capacity.");

    LineBatch markerBatch;
    passed &= expect(markerBatch.prepare(4), "Marker scratch preparation failed.");
    passed &= expect(!markerBatch.addPointMarker(a, 1.0f, blue, 64),
        "A point marker bypassed the prepared hard capacity.");
    passed &= expect(markerBatch.empty() && markerBatch.rejectedLineCount() == 3,
        "A capacity-rejected marker was partially emitted or miscounted.");

    passed &= expect(markerBatch.prepare(8), "Marker scratch resize failed.");
    passed &= expect(markerBatch.addPointMarker(a, 1.0f, blue, 8), "A marker that fits the prepared capacity was rejected.");
    passed &= expect(markerBatch.lineCount() == 3 && markerBatch.vertexCount() == 6,
        "A successful marker did not emit exactly three lines.");

    markerBatch.beginFrame(2);
    passed &= expect(markerBatch.addLine(a, b, red), "Configured frame budget rejected its first line.");
    passed &= expect(!markerBatch.addLine(b, c, blue) && markerBatch.vertexCount() == 2,
        "Configured frame budget was not enforced by the allocation-free overload.");

    return passed ? 0 : 1;
}
