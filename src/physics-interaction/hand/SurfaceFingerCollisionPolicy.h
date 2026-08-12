#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace rock::surface_finger_collision_policy
{
    inline constexpr std::size_t kFingerCount = 5;
    inline constexpr std::size_t kSegmentCount = 3;

    struct SegmentContact
    {
        float blockedDepthGameUnits = 0.0f;
        // Signed displacement toward the solver-safe direction produced by
        // closing the calibrated pose by Config::probeClosureOpenUnits.
        float helpfulProbeTravelGameUnits = 0.0f;
        bool active = false;
    };

    struct Config
    {
        float probeClosureOpenUnits = 0.10f;
        float responseGain = 1.0f;
        float maximumClosureOpenUnits = 0.85f;
        float minimumHelpfulProbeTravelGameUnits = 0.01f;
    };

    struct SolveResult
    {
        std::array<float, kFingerCount> targetOpenValues{
            1.0f, 1.0f, 1.0f, 1.0f, 1.0f
        };
        std::uint16_t helpfulSegmentMask = 0;
        bool anyHelpfulContact = false;
    };

    [[nodiscard]] inline Config sanitize(Config config) noexcept
    {
        config.probeClosureOpenUnits = std::clamp(
            std::isfinite(config.probeClosureOpenUnits) ?
                config.probeClosureOpenUnits : 0.10f,
            0.01f,
            0.50f);
        config.responseGain = std::clamp(
            std::isfinite(config.responseGain) ? config.responseGain : 1.0f,
            0.0f,
            4.0f);
        config.maximumClosureOpenUnits = std::clamp(
            std::isfinite(config.maximumClosureOpenUnits) ?
                config.maximumClosureOpenUnits : 0.85f,
            0.0f,
            1.0f);
        config.minimumHelpfulProbeTravelGameUnits = std::clamp(
            std::isfinite(config.minimumHelpfulProbeTravelGameUnits) ?
                config.minimumHelpfulProbeTravelGameUnits : 0.01f,
            0.0001f,
            1.0f);
        return config;
    }

    [[nodiscard]] inline SolveResult solve(
        const std::array<float, kFingerCount>& baselineOpenValues,
        const std::array<std::array<SegmentContact, kSegmentCount>,
            kFingerCount>& contacts,
        Config config) noexcept
    {
        config = sanitize(config);
        SolveResult result{};
        for (std::size_t finger = 0; finger < kFingerCount; ++finger) {
            const float baseline = std::clamp(
                std::isfinite(baselineOpenValues[finger]) ?
                    baselineOpenValues[finger] : 1.0f,
                0.0f,
                1.0f);
            result.targetOpenValues[finger] = baseline;
            float requiredClosure = 0.0f;
            std::uint16_t helpfulFingerMask = 0;
            for (std::size_t segment = 0; segment < kSegmentCount; ++segment) {
                const auto& contact = contacts[finger][segment];
                if (!contact.active ||
                    !std::isfinite(contact.blockedDepthGameUnits) ||
                    contact.blockedDepthGameUnits <= 0.0f ||
                    !std::isfinite(contact.helpfulProbeTravelGameUnits) ||
                    contact.helpfulProbeTravelGameUnits <
                        config.minimumHelpfulProbeTravelGameUnits) {
                    continue;
                }

                const float closure =
                    contact.blockedDepthGameUnits *
                    config.probeClosureOpenUnits /
                    contact.helpfulProbeTravelGameUnits *
                    config.responseGain;
                if (!std::isfinite(closure) || closure <= 0.0f) {
                    continue;
                }
                requiredClosure = std::max(requiredClosure, closure);
                helpfulFingerMask |= static_cast<std::uint16_t>(
                    1u << (finger * kSegmentCount + segment));
            }

            requiredClosure = std::min(
                requiredClosure,
                std::min(config.maximumClosureOpenUnits, baseline));
            result.targetOpenValues[finger] = baseline - requiredClosure;
            if (requiredClosure > 0.0f) {
                result.helpfulSegmentMask |= helpfulFingerMask;
                result.anyHelpfulContact = true;
            }
        }
        return result;
    }

    [[nodiscard]] inline std::array<float, kFingerCount> advanceOpenValues(
        const std::array<float, kFingerCount>& current,
        const std::array<float, kFingerCount>& target,
        const float smoothingSpeed,
        const float deltaSeconds) noexcept
    {
        const float speed = std::isfinite(smoothingSpeed) ?
            std::max(0.0f, smoothingSpeed) : 0.0f;
        const float dt = std::clamp(
            std::isfinite(deltaSeconds) ? deltaSeconds : (1.0f / 90.0f),
            0.0f,
            0.1f);
        const float alpha = speed > 0.0f ?
            std::clamp(1.0f - std::exp(-speed * dt), 0.0f, 1.0f) : 1.0f;

        std::array<float, kFingerCount> next{};
        for (std::size_t finger = 0; finger < kFingerCount; ++finger) {
            const float from = std::clamp(
                std::isfinite(current[finger]) ? current[finger] : 1.0f,
                0.0f,
                1.0f);
            const float to = std::clamp(
                std::isfinite(target[finger]) ? target[finger] : 1.0f,
                0.0f,
                1.0f);
            next[finger] = from + (to - from) * alpha;
        }
        return next;
    }
}
