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
        // closing/opening the calibrated pose by Config::probeDeltaOpenUnits.
        float closingProbeTravelGameUnits = 0.0f;
        float openingProbeTravelGameUnits = 0.0f;
        bool active = false;
    };

    struct Config
    {
        float probeDeltaOpenUnits = 0.10f;
        float responseGain = 1.0f;
        float maximumDeflectionOpenUnits = 0.85f;
        float minimumHelpfulProbeTravelGameUnits = 0.01f;
        float directionSwitchHysteresisFraction = 0.10f;
    };

    struct SolveResult
    {
        std::array<float, kFingerCount> targetOpenValues{
            1.0f, 1.0f, 1.0f, 1.0f, 1.0f
        };
        // -1 closes, +1 opens, 0 leaves the captured pose unchanged.
        std::array<std::int8_t, kFingerCount> directions{};
        std::uint16_t helpfulSegmentMask = 0;
        bool anyHelpfulContact = false;
    };

    [[nodiscard]] inline Config sanitize(Config config) noexcept
    {
        config.probeDeltaOpenUnits = std::clamp(
            std::isfinite(config.probeDeltaOpenUnits) ?
                config.probeDeltaOpenUnits : 0.10f,
            0.01f,
            0.50f);
        config.responseGain = std::clamp(
            std::isfinite(config.responseGain) ? config.responseGain : 1.0f,
            0.0f,
            4.0f);
        config.maximumDeflectionOpenUnits = std::clamp(
            std::isfinite(config.maximumDeflectionOpenUnits) ?
                config.maximumDeflectionOpenUnits : 0.85f,
            0.0f,
            1.0f);
        config.minimumHelpfulProbeTravelGameUnits = std::clamp(
            std::isfinite(config.minimumHelpfulProbeTravelGameUnits) ?
                config.minimumHelpfulProbeTravelGameUnits : 0.01f,
            0.0001f,
            1.0f);
        config.directionSwitchHysteresisFraction = std::clamp(
            std::isfinite(config.directionSwitchHysteresisFraction) ?
                config.directionSwitchHysteresisFraction : 0.10f,
            0.0f,
            0.5f);
        return config;
    }

    [[nodiscard]] inline SolveResult solve(
        const std::array<float, kFingerCount>& baselineOpenValues,
        const std::array<std::array<SegmentContact, kSegmentCount>,
            kFingerCount>& contacts,
        const std::array<std::int8_t, kFingerCount>& previousDirections,
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

            float baselineCost = 0.0f;
            for (std::size_t segment = 0; segment < kSegmentCount; ++segment) {
                const auto& contact = contacts[finger][segment];
                if (!contact.active ||
                    !std::isfinite(contact.blockedDepthGameUnits) ||
                    contact.blockedDepthGameUnits <= 0.0f) {
                    continue;
                }
                baselineCost += contact.blockedDepthGameUnits *
                                contact.blockedDepthGameUnits;
            }

            struct DirectionCandidate
            {
                float deflection = 0.0f;
                float residualCost = 0.0f;
                std::uint16_t helpfulMask = 0;
                std::int8_t direction = 0;
                bool valid = false;
            };

            const auto evaluateDirection = [&](const std::int8_t direction) {
                DirectionCandidate candidate{};
                candidate.direction = direction;
                std::array<float, kSegmentCount> depths{};
                std::array<float, kSegmentCount> travelPerOpenUnit{};
                for (std::size_t segment = 0; segment < kSegmentCount;
                     ++segment) {
                    const auto& contact = contacts[finger][segment];
                    if (!contact.active ||
                        !std::isfinite(contact.blockedDepthGameUnits) ||
                        contact.blockedDepthGameUnits <= 0.0f) {
                        continue;
                    }
                    const float probeTravel = direction < 0 ?
                        contact.closingProbeTravelGameUnits :
                        contact.openingProbeTravelGameUnits;
                    depths[segment] = contact.blockedDepthGameUnits;
                    if (!std::isfinite(probeTravel) ||
                        probeTravel <
                            config.minimumHelpfulProbeTravelGameUnits) {
                        if (!std::isfinite(probeTravel)) {
                            travelPerOpenUnit[segment] = 0.0f;
                            continue;
                        }
                    } else {
                        candidate.helpfulMask |= static_cast<std::uint16_t>(
                            1u << (finger * kSegmentCount + segment));
                    }
                    travelPerOpenUnit[segment] =
                        probeTravel / config.probeDeltaOpenUnits;
                }

                const float anatomicalCapacity = direction < 0 ?
                    baseline :
                    1.0f - baseline;
                const float maximumDeflection = std::min(
                    config.maximumDeflectionOpenUnits,
                    anatomicalCapacity);
                if (maximumDeflection <= 0.0f ||
                    candidate.helpfulMask == 0) {
                    return candidate;
                }

                const auto residualCostAt = [&](const float deflection) {
                    float cost = 0.0f;
                    for (std::size_t segment = 0;
                         segment < kSegmentCount;
                         ++segment) {
                        if (depths[segment] <= 0.0f) {
                            continue;
                        }
                        const float residual = std::max(
                            0.0f,
                            depths[segment] -
                                travelPerOpenUnit[segment] * deflection);
                        cost += residual * residual;
                    }
                    return cost;
                };

                float bestDeflection = 0.0f;
                float bestCost = baselineCost;
                const auto consider = [&](const float rawDeflection) {
                    const float deflection = std::clamp(
                        rawDeflection,
                        0.0f,
                        maximumDeflection);
                    const float cost = residualCostAt(deflection);
                    constexpr float kCostTieEpsilon = 1.0e-6f;
                    if (cost + kCostTieEpsilon < bestCost ||
                        (std::abs(cost - bestCost) <= kCostTieEpsilon &&
                            deflection < bestDeflection)) {
                        bestCost = cost;
                        bestDeflection = deflection;
                    }
                };

                consider(maximumDeflection);
                for (std::size_t segment = 0;
                     segment < kSegmentCount;
                     ++segment) {
                    if (travelPerOpenUnit[segment] > 0.0f) {
                        consider(
                            depths[segment] /
                            travelPerOpenUnit[segment]);
                    }
                }

                /*
                 * The squared residual is convex and piecewise quadratic.
                 * With only three phalanxes, enumerating the eight possible
                 * active-positive-contact sets gives every stationary point;
                 * zero crossings and the anatomical bound are considered
                 * above. This chooses the deflection that helps the whole
                 * finger instead of letting one segment overdrive the rest.
                 */
                for (std::uint32_t activePositiveMask = 0;
                     activePositiveMask < (1u << kSegmentCount);
                     ++activePositiveMask) {
                    float numerator = 0.0f;
                    float denominator = 0.0f;
                    for (std::size_t segment = 0;
                         segment < kSegmentCount;
                         ++segment) {
                        const float travel = travelPerOpenUnit[segment];
                        const bool include = travel <= 0.0f ||
                                             (activePositiveMask &
                                                 (1u << segment)) != 0;
                        if (!include || depths[segment] <= 0.0f) {
                            continue;
                        }
                        numerator += travel * depths[segment];
                        denominator += travel * travel;
                    }
                    if (denominator > 0.000001f) {
                        consider(numerator / denominator);
                    }
                }

                candidate.deflection = std::clamp(
                    bestDeflection * config.responseGain,
                    0.0f,
                    maximumDeflection);
                candidate.residualCost =
                    residualCostAt(candidate.deflection);
                candidate.valid = candidate.residualCost + 1.0e-6f <
                                      baselineCost &&
                                  candidate.deflection > 0.0f;
                return candidate;
            };

            const auto closing = evaluateDirection(-1);
            const auto opening = evaluateDirection(1);
            const DirectionCandidate* selected = nullptr;
            if (closing.valid && opening.valid) {
                const DirectionCandidate* previous =
                    previousDirections[finger] < 0 ? &closing :
                    previousDirections[finger] > 0 ? &opening : nullptr;
                const DirectionCandidate* alternative = previous == &closing ?
                    &opening :
                    &closing;
                if (previous) {
                    const float switchMargin =
                        baselineCost *
                        config.directionSwitchHysteresisFraction;
                    selected = alternative->residualCost + switchMargin <
                                       previous->residualCost ?
                        alternative :
                        previous;
                } else {
                    constexpr float kCostTieEpsilon = 1.0e-6f;
                    if (opening.residualCost + kCostTieEpsilon <
                        closing.residualCost) {
                        selected = &opening;
                    } else if (closing.residualCost + kCostTieEpsilon <
                               opening.residualCost) {
                        selected = &closing;
                    } else {
                        selected = opening.deflection < closing.deflection ?
                            &opening :
                            &closing;
                    }
                }
            } else if (closing.valid) {
                selected = &closing;
            } else if (opening.valid) {
                selected = &opening;
            }

            if (selected) {
                result.targetOpenValues[finger] = std::clamp(
                    baseline +
                        static_cast<float>(selected->direction) *
                            selected->deflection,
                    0.0f,
                    1.0f);
                result.directions[finger] = selected->direction;
                result.helpfulSegmentMask |= selected->helpfulMask;
                result.anyHelpfulContact = true;
            }
        }
        return result;
    }

    // The solver works from its captured baseline. Add back only the relief
    // observed in the applied geometry, not the requested curl; otherwise the
    // target relaxes as soon as its own correction starts to work.
    [[nodiscard]] inline float baselineBlockedDepth(float remainingDepth, float achievedTravel) noexcept
    {
        if (!std::isfinite(remainingDepth) || remainingDepth <= 0.0f) return 0.0f;
        return std::max(0.0f, remainingDepth + (std::isfinite(achievedTravel) ? achievedTravel : 0.0f));
    }
}
