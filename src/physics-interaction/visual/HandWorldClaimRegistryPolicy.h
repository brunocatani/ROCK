#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>

#include "physics-interaction/TransformMath.h"

#include "RE/NetImmerse/NiTransform.h"

/*
 * ROCK's mirror of the hand world claims it holds in FRIK (API v2
 * setHandWorldTransform). ROCK publishes inside FRIK's AfterArmSolve phase
 * (API v2.3) and FRIK re-solves a claimed arm before the frame continues, so a
 * claim is consumed in the frame it is published and kept until it is
 * cleared. The registry answers which claim FRIK solves each hand to under
 * FRIK's tie rule, and so whether the rendered hand bone may be read as
 * controller input. It also carries the unreachable-target report derived
 * from getHandSolveResult.
 *
 * Fixed capacity, no heap, one owner: the frik_visual_authority bridge.
 */
namespace rock::hand_world_claim_registry_policy
{
    inline constexpr std::size_t kMaxClaims = 16;
    inline constexpr std::size_t kTagCapacity = 64;

    // A target whose stored rotation deviates more than this from an
    // orthonormal basis is not a pose FRIK may render (it stretches the arm).
    inline constexpr double kMaxTargetRotationError = 0.05;

    [[nodiscard]] constexpr std::size_t handIndex(const bool isLeft) noexcept
    {
        return isLeft ? 1u : 0u;
    }

    struct DriverSample
    {
        RE::NiTransform world{};
        bool valid = false;
    };

    /*
     * This frame's sample of both controller hands (FRIK's tracked weapon
     * offset targets), taken before ROCK's update; the input driver readers
     * use it. sequence is the frame sequence that took it; zero means none yet.
     */
    struct DriverFrame
    {
        std::uint64_t sequence = 0;
        std::array<DriverSample, 2> hands{};
    };

    struct Claim
    {
        std::array<char, kTagCapacity> tag{};
        std::size_t tagLength = 0;
        bool isLeft = false;
        int priority = 0;
        RE::NiTransform target{};
        // Mirrors FRIK's registration sequence: the highest wins a priority tie.
        std::uint64_t publishOrder = 0;
        // FRIK reports the target unreachable; the owner's publishes fail until it follows the claim again.
        bool fallbackReported = false;
        bool valid = false;
    };

    struct Registry
    {
        std::array<Claim, kMaxClaims> claims{};
        std::uint64_t nextPublishOrder = 1;
    };

    [[nodiscard]] inline std::string_view tagView(const Claim& claim) noexcept
    {
        return std::string_view(claim.tag.data(), claim.tagLength);
    }

    [[nodiscard]] inline bool isRegistrableTag(const std::string_view tag) noexcept
    {
        return !tag.empty() && tag.size() < kTagCapacity;
    }

    [[nodiscard]] inline bool isFiniteTransform(const RE::NiTransform& transform) noexcept
    {
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(transform.rotate.entry[row][column])) {
                    return false;
                }
            }
        }
        return std::isfinite(transform.translate.x) &&
               std::isfinite(transform.translate.y) &&
               std::isfinite(transform.translate.z) &&
               std::isfinite(transform.scale) &&
               std::fabs(transform.scale) > 0.000001f;
    }

    [[nodiscard]] inline bool isUsableTargetRotation(const RE::NiTransform& transform) noexcept
    {
        const double error = transform_math::storedRotationOrthonormalityError(transform.rotate);
        return std::isfinite(error) && error <= kMaxTargetRotationError;
    }

    [[nodiscard]] inline float translationDeltaGameUnits(const RE::NiTransform& lhs, const RE::NiTransform& rhs) noexcept
    {
        const float x = lhs.translate.x - rhs.translate.x;
        const float y = lhs.translate.y - rhs.translate.y;
        const float z = lhs.translate.z - rhs.translate.z;
        return std::sqrt(x * x + y * y + z * z);
    }

    /*
     * Angle of lhs^T * rhs from both its cosine (trace) and sine (skew part),
     * so a sub-degree difference is exact instead of acos-noise near 1.
     */
    [[nodiscard]] inline float rotationDeltaDegrees(const RE::NiTransform& lhs, const RE::NiTransform& rhs) noexcept
    {
        float relative[3][3]{};
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                float sum = 0.0f;
                for (int k = 0; k < 3; ++k) {
                    sum += lhs.rotate.entry[k][row] * rhs.rotate.entry[k][column];
                }
                relative[row][column] = sum;
            }
        }
        const float trace = relative[0][0] + relative[1][1] + relative[2][2];
        const float skewX = relative[2][1] - relative[1][2];
        const float skewY = relative[0][2] - relative[2][0];
        const float skewZ = relative[1][0] - relative[0][1];
        const float sine = 0.5f * std::sqrt(skewX * skewX + skewY * skewY + skewZ * skewZ);
        const float cosine = 0.5f * (trace - 1.0f);
        return std::atan2(sine, cosine) * 57.29577951308232f;
    }

    [[nodiscard]] inline Claim* find(Registry& registry, const std::string_view tag, const bool isLeft) noexcept
    {
        for (auto& claim : registry.claims) {
            if (claim.valid && claim.isLeft == isLeft && tagView(claim) == tag) {
                return &claim;
            }
        }
        return nullptr;
    }

    [[nodiscard]] inline const Claim* find(const Registry& registry, const std::string_view tag, const bool isLeft) noexcept
    {
        for (const auto& claim : registry.claims) {
            if (claim.valid && claim.isLeft == isLeft && tagView(claim) == tag) {
                return &claim;
            }
        }
        return nullptr;
    }

    enum class CommitResult : std::uint8_t
    {
        Inserted,
        Updated,
        InvalidTag,
        InvalidTarget,
        Full,
    };

    /*
     * Record a claim FRIK accepted. Updating an existing tag keeps the slot and
     * its publishOrder, matching FRIK's rule since API v2.3: the newest
     * registration wins a priority tie and a republish keeps its place (clear
     * and set again to claim the tie). Its unreachable report survives, since
     * owners republish every frame and the episode spans frames.
     */
    [[nodiscard]] inline CommitResult commit(
        Registry& registry,
        const std::string_view tag,
        const bool isLeft,
        const int priority,
        const RE::NiTransform& target) noexcept
    {
        if (!isRegistrableTag(tag)) {
            return CommitResult::InvalidTag;
        }
        if (!isFiniteTransform(target) || !isUsableTargetRotation(target)) {
            return CommitResult::InvalidTarget;
        }

        Claim* claim = find(registry, tag, isLeft);
        const bool inserted = claim == nullptr;
        if (!claim) {
            for (auto& candidate : registry.claims) {
                if (!candidate.valid) {
                    claim = &candidate;
                    break;
                }
            }
        }
        if (!claim) {
            return CommitResult::Full;
        }

        claim->tag.fill('\0');
        std::memcpy(claim->tag.data(), tag.data(), tag.size());
        claim->tagLength = tag.size();
        claim->isLeft = isLeft;
        claim->priority = priority;
        claim->target = target;
        if (inserted) {
            claim->publishOrder = registry.nextPublishOrder++;
            claim->fallbackReported = false;
        }
        claim->valid = true;
        return inserted ? CommitResult::Inserted : CommitResult::Updated;
    }

    [[nodiscard]] inline bool remove(Registry& registry, const std::string_view tag, const bool isLeft) noexcept
    {
        Claim* claim = find(registry, tag, isLeft);
        if (!claim) {
            return false;
        }
        *claim = {};
        return true;
    }

    [[nodiscard]] inline bool hasClaim(const Registry& registry, const bool isLeft) noexcept
    {
        for (const auto& claim : registry.claims) {
            if (claim.valid && claim.isLeft == isLeft) {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] inline std::size_t claimCount(const Registry& registry) noexcept
    {
        std::size_t count = 0;
        for (const auto& claim : registry.claims) {
            if (claim.valid) {
                ++count;
            }
        }
        return count;
    }

    /*
     * The claim FRIK solves the hand to: highest priority, then most recently
     * published (FRIK ExternalAuthority::getHandWorldTransform).
     */
    [[nodiscard]] inline const Claim* winner(const Registry& registry, const bool isLeft) noexcept
    {
        const Claim* best = nullptr;
        for (const auto& claim : registry.claims) {
            if (!claim.valid || claim.isLeft != isLeft) {
                continue;
            }
            if (!best || claim.priority > best->priority ||
                (claim.priority == best->priority && claim.publishOrder > best->publishOrder)) {
                best = &claim;
            }
        }
        return best;
    }

    inline void clearAll(Registry& registry) noexcept
    {
        for (auto& claim : registry.claims) {
            claim = {};
        }
        registry.nextPublishOrder = 1;
    }

    // ---- The target FRIK solved to, for the trace ----

    struct ConsumedTarget
    {
        RE::NiTransform target{};
        bool valid = false;
    };

    // The target FRIK is about to solve to: the winner before ROCK's frame.
    [[nodiscard]] inline ConsumedTarget snapshotConsumedTarget(const Registry& registry, const bool isLeft) noexcept
    {
        const Claim* top = winner(registry, isLeft);
        if (!top) {
            return {};
        }
        return ConsumedTarget{ .target = top->target, .valid = true };
    }
}
