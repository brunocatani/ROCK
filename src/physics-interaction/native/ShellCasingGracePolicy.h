#pragma once

#include <cmath>
#include <cstdint>

namespace rock::shell_casing_grace
{
    inline constexpr float kDefaultMilliseconds = 30.0f;
    inline constexpr float kMaximumMilliseconds = 500.0f;
    inline constexpr std::uint32_t kInvalidBody = 0x7FFF'FFFFu;

    struct GraceWindow
    {
        double bornSeconds{};
        double expiresSeconds{};
        std::uint64_t bornSolve{};

        [[nodiscard]] bool expired(double seconds, std::uint64_t completedSolve) const noexcept
        {
            // A small positive setting must allow the casing to move through
            // one solve, even when it is shorter than the next physics step.
            return completedSolve > bornSolve && seconds >= expiresSeconds;
        }
    };

    [[nodiscard]] inline float sanitizeMilliseconds(float value) noexcept
    {
        return std::isfinite(value) && value >= 0.0f && value <= kMaximumMilliseconds ?
            value : kDefaultMilliseconds;
    }

    [[nodiscard]] inline GraceWindow makeWindow(double seconds, std::uint64_t solve, float milliseconds) noexcept
    {
        return { seconds, seconds + static_cast<double>(sanitizeMilliseconds(milliseconds)) / 1000.0, solve };
    }

    struct BodyIdentity
    {
        std::uint32_t id{ kInvalidBody };
        std::uint32_t motion{};
        std::uintptr_t collisionObject{};

        [[nodiscard]] bool matches(const BodyIdentity& live) const noexcept
        {
            return id != kInvalidBody && motion != 0 && collisionObject != 0 &&
                id == live.id && motion == live.motion && collisionObject == live.collisionObject;
        }
    };
}
