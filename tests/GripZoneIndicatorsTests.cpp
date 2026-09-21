#include "physics-interaction/visual/GripZoneIndicators.h"
#include "physics-interaction/visual/WorldOverlayRenderer.h"
#include <cassert>
#include <limits>

// Only the D3D transport is replaced; publication and lifecycle are production
// code, linked without the diagnostic overlay or its settings/shape worker.
namespace rock::world_overlay_renderer
{
    bool EnsureInstalled() { return true; }
    void NotifyPublication() noexcept {}
}

int main()
{
    namespace markers = rock::grip_zone_indicators;
    markers::Frame frame{};
    frame.count=2;
    frame.positions[0]={1,2,3};
    frame.positions[1]={4,5,6};
    frame.diameterGameUnits=0.7f;
    frame.gameFrameIndex=10;
    markers::Publish(frame);
    markers::EndFrame(10);
    auto rendered=markers::Snapshot();
    assert(rendered && rendered->count==2 && rendered->positions[0].x==1);

    // A loading/abandoned frame must not reuse the previous weapon's markers.
    markers::EndFrame(11);
    assert(!markers::Snapshot());
    frame.gameFrameIndex=11;
    frame.positions[0].x=7;
    markers::Publish(frame);
    assert(markers::Snapshot()->positions[0].x==7);
    assert(rendered->positions[0].x==1); // An in-flight render remains immutable.

    markers::Clear(); // Skeleton/world teardown is independent of debug cleanup.
    assert(!markers::Snapshot());
    assert(rendered->positions[0].x==1);
    rendered.reset();

    frame.gameFrameIndex=12;
    frame.positions[1].x=std::numeric_limits<float>::quiet_NaN();
    markers::Publish(frame);
    assert(markers::Snapshot()->count==1);
    frame.count=0;
    markers::Publish(frame);
    assert(!markers::Snapshot());

    // Retained compositor snapshots exhaust a bounded pool, then recover when
    // released. Exhaustion must never redisplay a stale frame or mutate one.
    frame.count=1;
    std::array<std::shared_ptr<const markers::Frame>,4> retained;
    for(std::size_t i=0;i<retained.size();++i) {
        frame.gameFrameIndex=20+i;
        markers::Publish(frame);
        retained[i]=markers::Snapshot();
        assert(retained[i]);
    }
    frame.gameFrameIndex=24;
    markers::Publish(frame);
    assert(!markers::Snapshot());
    retained={};
    frame.gameFrameIndex=25;
    markers::Publish(frame);
    assert(markers::Snapshot()->gameFrameIndex==25);
    markers::Clear();
}
