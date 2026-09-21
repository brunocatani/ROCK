#include "physics-interaction/grab/GrabMotorLoadPolicy.h"

#include <cassert>
#include <limits>

int main()
{
    using namespace rock::grab_motor_load;
    std::array<float, 24> rows{};
    Axes limits{100, 200, 300, 400, 500, 600};
    for (std::size_t i = 0; i < kAxisCount; ++i) {
        rows[4 * i] = limits[i] * 0.01f * (i % 2 == 0 ? 1.0f : -0.5f);
        rows[4 * i + 1] = static_cast<float>(i) + 7.0f;
        // Unused slots can contain private/stale data. They are not forces.
        rows[4 * i + 2] = std::numeric_limits<float>::quiet_NaN();
        rows[4 * i + 3] = 9000.0f;
    }
    const auto load = decode(rows, limits, 0.01f);
    assert(load.valid && load.enabledAxes == 0x3f);
    for (std::size_t i = 0; i < kAxisCount; ++i) {
        assert(std::abs(load.netUtilization[i] - (i % 2 == 0 ? 1.0f : 0.5f)) < 0.00001f);
        assert(load.recoveryState[i] == static_cast<float>(i) + 7.0f);
        assert(std::abs(load.averageEffort[i] * 0.01f - rows[4 * i]) < 0.00001f);
    }
    // Identical physical effort over two half intervals has the same ratio.
    auto halfRows = rows;
    for (std::size_t i = 0; i < kAxisCount; ++i) halfRows[i * 4] *= 0.5f;
    const auto half = decode(halfRows, limits, 0.005f);
    assert(half.valid);
    for (std::size_t i = 0; i < kAxisCount; ++i)
        assert(std::abs(load.netUtilization[i] - half.netUtilization[i]) < 0.00001f);
    // Compare a compensated weapon with the compensated command, once.
    auto scaledRows = rows;
    auto scaledLimits = limits;
    for (std::size_t i = 0; i < kAxisCount; ++i) {
        scaledRows[4 * i] *= 4.0f;
        scaledLimits[i] *= 4.0f;
    }
    const auto scaled = decode(scaledRows, scaledLimits, 0.01f);
    assert(scaled.valid);
    for (std::size_t i = 0; i < kAxisCount; ++i)
        assert(std::abs(load.netUtilization[i] - scaled.netUtilization[i]) < 0.00001f);

    assert(!decode(rows, limits, 0.0f).valid);
    assert(!decode(rows, limits, -0.01f).valid);
    assert(!decode(rows, limits, std::numeric_limits<float>::quiet_NaN()).valid);
    auto bad = rows;
    bad[4] = std::numeric_limits<float>::infinity();
    assert(!decode(bad, limits, 0.01f).valid);
    bad = rows;
    bad[5] = std::numeric_limits<float>::quiet_NaN();
    assert(!decode(bad, limits, 0.01f).valid);
    auto badLimits = limits;
    badLimits[0] = -1.0f;
    assert(!decode(rows, badLimits, 0.01f).valid);
    badLimits[0] = 0.0f;
    assert(!decode(rows, badLimits, 0.01f).valid); // stale nonzero impulse
    auto stopped = rows;
    stopped[0] = 0.0f;
    const auto inactive = decode(stopped, badLimits, 0.01f);
    assert(inactive.valid && inactive.enabledAxes == 0x3e && inactive.netUtilization[0] == 0);
    assert(matchingSolve(0, 1));
    assert(matchingSolve(89, 90));
    assert(!matchingSolve(90, 90));
    assert(!matchingSolve(89, 91));
    assert(!matchingSolve(std::numeric_limits<std::uint64_t>::max(), 0));
}
