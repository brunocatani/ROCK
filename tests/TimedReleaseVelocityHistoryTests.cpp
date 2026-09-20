#include "physics-interaction/grab/TimedReleaseVelocityHistory.h"
#include <cassert>
#include <limits>

struct Point { float x{}, y{}, z{}; };
int main()
{
    using namespace rock::release_velocity;
    rock::game_frame_timing_policy::GameFrameTiming timing{};
    timing.sequence = 2;
    timing.valid = true;
    timing.deltaSeconds = 0.01f;
    assert(admitControllerSample(1, timing).deltaSeconds == 0.01f);
    assert(admitControllerSample(2, timing).duplicate);
    timing.sequence = 4;
    assert(admitControllerSample(1, timing).rebase); // missed callback cannot use one-frame dt
    timing.rawDeltaSeconds = 0.3f;
    timing.deltaSeconds = 0.1f;
    timing.discontinuity = true;
    assert(admitControllerSample(3, timing).rebase && admitControllerSample(3, timing).deltaSeconds == 0);
    timing.discontinuity = false;
    timing.menuPaused = true;
    assert(admitControllerSample(3, timing).rebase);
    timing.menuPaused = false;
    timing.valid = false;
    assert(admitControllerSample(3, timing).rebase);
    for (const auto hz : {45, 72, 90, 120, 144}) {
        History<Point> history;
        for (std::uint64_t i = 1; i <= static_cast<std::uint64_t>(hz); ++i)
            history.append(i, double(i) / hz, {2, 3, 4}, {0, 1, 0});
        const auto peak = history.peaks(1.0);
        assert(peak.count > 0 && peak.linear.x == 2 && peak.angular.y == 1);
        // The last strong sample has the same expiry at every cadence.
        assert(history.peaks(1.0 + kRetentionSeconds).count == 0);
        history.clear();
        assert(history.peaks(1.0).count == 0);
    }
    History<Point> history;
    assert(!history.needsSource(0));
    history.append(1, 0.0, {});
    history.append(2, 1.0 / 90.0, {9, 0, 0});
    history.append(3, 2.0 / 90.0, {});
    assert(std::abs(history.peaks(2.0 / 90.0).linear.x - 3.0f) < 0.0001f);
    history.clear();
    history.append(1, 1.0, {10, 0, 0});
    history.append(1, 1.01, {100, 0, 0}); // duplicate solve cannot extend its timestamp
    history.append(2, 1.01, {-10, 0, 0}); // equal peak retains earliest direction
    assert(history.peaks(1.01).count == 2 && history.peaks(1.01).linear.x == 10);
    assert(history.peaks(1.0 + kRetentionSeconds).linear.x == -10);
    history.append(3, 1.02, {std::numeric_limits<float>::quiet_NaN(), 0, 0});
    assert(history.peaks(1.02).count == 2);
    assert(history.peaks(0.9).count == 0);
    history.clear();
    assert(history.needsSource(1));
    for (std::uint64_t i = 1; i <= 1000; ++i) history.append(i, double(i) / 10000, {1, 0, 0});
    assert(history.peaks(0.1).count <= 32);
}
