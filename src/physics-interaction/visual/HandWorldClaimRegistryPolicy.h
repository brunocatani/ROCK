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
 * setHandWorldTransform). FRIK stores a claim and solves the arm to it in its
 * NEXT skeleton frame, and keeps it until it is cleared. ROCK publishes from
 * its own frame, which runs after FRIK's, so a claim computed from this frame's
 * controller sample is consumed one frame later. The registry remembers, per
 * claim, which controller-driven node the target was expressed against and the
 * driver's transform at publish time, so a pass before FRIK's next frame can
 * move the target by the driver's motion since then (the rebase). It also
 * answers "does FRIK currently solve this hand to a ROCK claim", which decides
 * whether the rendered hand bone may be read as controller input, and detects
 * the silent fallback FRIK performs for an unreachable target.
 *
 * Fixed capacity, no heap, one owner: the frik_visual_authority bridge.
 */
namespace rock::hand_world_claim_registry_policy
{
    inline constexpr std::size_t kMaxClaims = 16;
    inline constexpr std::size_t kTagCapacity = 64;

    // Below this motion the rebase does not republish (FRIK samples a log line per publish).
    inline constexpr float kRepublishTranslationEpsilonGameUnits = 0.0005f;
    inline constexpr float kRepublishRotationEpsilonDegrees = 0.005f;

    // FRIK solves the tracked hand instead of a claim it cannot reach and
    // reports nothing. A rendered wrist this far from the claim for this many
    // consecutive frames is treated as that fallback.
    inline constexpr float kFallbackTranslationGameUnits = 3.0f;
    inline constexpr float kFallbackRotationDegrees = 15.0f;
    inline constexpr std::uint32_t kFallbackConfirmFrames = 2;
    // A target whose stored rotation deviates more than this from an
    // orthonormal basis is not a pose FRIK may render (it stretches the arm).
    inline constexpr double kMaxTargetRotationError = 0.05;

    /*
     * Which physical hand's controller chain a claim follows between ROCK
     * frames. Static claims (a hand latched to a surface) are never moved.
     */
    enum class RebaseDriver : std::uint8_t
    {
        Static,
        RightHand,
        LeftHand,
    };

    [[nodiscard]] constexpr std::size_t handIndex(const bool isLeft) noexcept
    {
        return isLeft ? 1u : 0u;
    }

    [[nodiscard]] constexpr RebaseDriver driverForHand(const bool isLeft) noexcept
    {
        return isLeft ? RebaseDriver::LeftHand : RebaseDriver::RightHand;
    }

    struct DriverSample
    {
        RE::NiTransform world{};
        bool valid = false;
    };

    /*
     * One pre-FRIK sample of both driver chains. sequence is the scheduler
     * sequence of the pass that took it; zero means no pass has run yet.
     */
    struct DriverFrame
    {
        std::uint64_t sequence = 0;
        std::array<DriverSample, 2> hands{};
    };

    [[nodiscard]] inline const DriverSample* sampleForDriver(const DriverFrame& frame, const RebaseDriver driver) noexcept
    {
        switch (driver) {
        case RebaseDriver::RightHand:
            return &frame.hands[handIndex(false)];
        case RebaseDriver::LeftHand:
            return &frame.hands[handIndex(true)];
        default:
            return nullptr;
        }
    }

    struct Claim
    {
        std::array<char, kTagCapacity> tag{};
        std::size_t tagLength = 0;
        bool isLeft = false;
        int priority = 0;
        RE::NiTransform target{};
        RebaseDriver driver = RebaseDriver::Static;
        DriverSample driverAtPublish{};
        // Mirrors FRIK's publish sequence: the highest wins a priority tie.
        std::uint64_t publishOrder = 0;
        std::uint32_t fallbackFrames = 0;
        bool fallbackReported = false;
        bool valid = false;
    };

    struct Registry
    {
        std::array<Claim, kMaxClaims> claims{};
        std::uint64_t nextPublishOrder = 1;
        std::uint32_t generation = 1;
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
     * re-stamps publishOrder, matching FRIK's "most recently published wins a
     * tie" rule for hand transforms; its fallback episode state survives, since
     * owners republish every frame and the episode spans frames. The driver
     * sample is the pre-FRIK sample of the frame the target was computed in.
     */
    [[nodiscard]] inline CommitResult commit(
        Registry& registry,
        const std::string_view tag,
        const bool isLeft,
        const int priority,
        const RE::NiTransform& target,
        const RebaseDriver driver,
        const DriverSample& driverAtPublish) noexcept
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
        claim->driver = driver;
        claim->driverAtPublish = driver == RebaseDriver::Static ? DriverSample{} : driverAtPublish;
        claim->publishOrder = registry.nextPublishOrder++;
        if (inserted) {
            claim->fallbackFrames = 0;
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
        ++registry.generation;
        if (registry.generation == 0) {
            registry.generation = 1;
        }
    }

    struct RebasePlan
    {
        RE::NiTransform target{};
        bool republish = false;
    };

    /*
     * Move a claim by its driver's motion since publish:
     * target' = driverNow * inverse(driverAtPublish) * target.
     * Not republished when the driver is static, either sample is missing, the
     * result is not finite, or the motion is below the epsilon.
     */
    [[nodiscard]] inline RebasePlan planRebase(const Claim& claim, const DriverSample& driverNow) noexcept
    {
        RebasePlan plan{ .target = claim.target, .republish = false };
        if (claim.driver == RebaseDriver::Static || !claim.driverAtPublish.valid || !driverNow.valid ||
            !isFiniteTransform(claim.driverAtPublish.world) || !isFiniteTransform(driverNow.world)) {
            return plan;
        }
        // Scene driver bases carry float drift; the transpose inverse only
        // cancels for orthonormal bases, and the rebased target is rendered.
        const RE::NiTransform delta = transform_math::composeTransforms(
            transform_math::orthonormalizedTransform(driverNow.world),
            transform_math::invertTransform(transform_math::orthonormalizedTransform(claim.driverAtPublish.world)));
        const RE::NiTransform rebased = transform_math::orthonormalizedTransform(transform_math::composeTransforms(delta, claim.target));
        if (!isFiniteTransform(rebased)) {
            return plan;
        }
        plan.target = rebased;
        plan.republish =
            translationDeltaGameUnits(rebased, claim.target) > kRepublishTranslationEpsilonGameUnits ||
            rotationDeltaDegrees(rebased, claim.target) > kRepublishRotationEpsilonDegrees;
        return plan;
    }

    /*
     * Commit a rebase that FRIK accepted: the claim now describes the target
     * against the new driver sample.
     */
    inline void applyRebase(Claim& claim, const RebasePlan& plan, const DriverSample& driverNow) noexcept
    {
        claim.target = plan.target;
        claim.driverAtPublish = driverNow;
    }

    struct RebasePassEntry
    {
        std::size_t claimIndex = 0;
        RE::NiTransform target{};
        // The driver moved: publish the rebased target and advance the claim.
        bool moved = false;
        // Publish even without motion so an equal-priority tie keeps its winner.
        bool keepOrder = false;
    };

    struct RebasePassPlan
    {
        std::array<RebasePassEntry, kMaxClaims> entries{};
        std::size_t count = 0;
    };

    /*
     * One pre-FRIK pass over every claim, oldest publish first. FRIK breaks an
     * equal-priority tie by the newest publish, and a republish re-stamps that
     * order, so once a claim republishes every later same-hand same-priority
     * claim republishes too (unchanged target) and the winner is preserved.
     */
    inline void planRebasePass(const Registry& registry, const DriverFrame& driverFrame, RebasePassPlan& outPlan) noexcept
    {
        outPlan = {};
        std::array<std::size_t, kMaxClaims> order{};
        std::size_t count = 0;
        for (std::size_t index = 0; index < registry.claims.size(); ++index) {
            if (!registry.claims[index].valid) {
                continue;
            }
            std::size_t slot = count;
            while (slot > 0 && registry.claims[order[slot - 1]].publishOrder > registry.claims[index].publishOrder) {
                order[slot] = order[slot - 1];
                --slot;
            }
            order[slot] = index;
            ++count;
        }

        std::array<std::array<int, kMaxClaims>, 2> republishedPriorities{};
        std::array<std::size_t, 2> republishedCounts{};
        for (std::size_t position = 0; position < count; ++position) {
            const std::size_t index = order[position];
            const Claim& claim = registry.claims[index];
            const std::size_t hand = handIndex(claim.isLeft);

            RebasePassEntry entry{ .claimIndex = index, .target = claim.target };
            if (const DriverSample* sample = sampleForDriver(driverFrame, claim.driver)) {
                const RebasePlan plan = planRebase(claim, *sample);
                entry.target = plan.target;
                entry.moved = plan.republish;
            }
            if (!entry.moved) {
                for (std::size_t i = 0; i < republishedCounts[hand]; ++i) {
                    if (republishedPriorities[hand][i] == claim.priority) {
                        entry.keepOrder = true;
                        break;
                    }
                }
            }
            if (entry.moved || entry.keepOrder) {
                bool known = false;
                for (std::size_t i = 0; i < republishedCounts[hand]; ++i) {
                    known = known || republishedPriorities[hand][i] == claim.priority;
                }
                if (!known && republishedCounts[hand] < kMaxClaims) {
                    republishedPriorities[hand][republishedCounts[hand]++] = claim.priority;
                }
            }
            outPlan.entries[outPlan.count++] = entry;
        }
    }

    /*
     * Record that FRIK accepted a pass entry. A moved entry adopts the rebased
     * target and the driver sample it was expressed against; both kinds take a
     * fresh publish order.
     */
    inline void commitRebasePassEntry(Registry& registry, const RebasePassEntry& entry, const DriverFrame& driverFrame) noexcept
    {
        Claim& claim = registry.claims[entry.claimIndex];
        if (entry.moved) {
            claim.target = entry.target;
            if (const DriverSample* sample = sampleForDriver(driverFrame, claim.driver)) {
                claim.driverAtPublish = *sample;
            }
        }
        claim.publishOrder = registry.nextPublishOrder++;
    }

    enum class FallbackObservation : std::uint8_t
    {
        NotApplicable,
        Following,
        Suspected,
        Confirmed,
    };

    /*
     * Compare the wrist FRIK rendered with the target it was given. Skipped
     * while ROCK's recoil controller kicks the hand (FRIK composes the kick
     * onto the claim) or when the rendered wrist is unavailable. Confirmed is
     * returned once per episode; the counter resets as soon as the hand follows
     * the claim again.
     */
    [[nodiscard]] inline FallbackObservation observeFallback(
        Claim& claim,
        const RE::NiTransform& renderedHandWorld,
        const bool renderedHandValid,
        const bool recoilComposedThisFrame) noexcept
    {
        if (!claim.valid || !renderedHandValid || recoilComposedThisFrame || !isFiniteTransform(renderedHandWorld)) {
            return FallbackObservation::NotApplicable;
        }
        const bool following =
            translationDeltaGameUnits(renderedHandWorld, claim.target) <= kFallbackTranslationGameUnits &&
            rotationDeltaDegrees(renderedHandWorld, claim.target) <= kFallbackRotationDegrees;
        if (following) {
            claim.fallbackFrames = 0;
            claim.fallbackReported = false;
            return FallbackObservation::Following;
        }
        if (claim.fallbackFrames < kFallbackConfirmFrames) {
            ++claim.fallbackFrames;
        }
        if (claim.fallbackFrames >= kFallbackConfirmFrames && !claim.fallbackReported) {
            claim.fallbackReported = true;
            return FallbackObservation::Confirmed;
        }
        return FallbackObservation::Suspected;
    }

    // ---- Presentation at the end of ROCK's frame ----

    /*
     * FRIK solved this hand at its frame start to the claim it held then.
     * ROCK's frame republished the target from this frame's inputs: the
     * weapon it just posed, the object it just stepped. The rendered chain is
     * carried by that change at the end of ROCK's frame so the hand draws on
     * this frame's seat, as Experimental's immediate solve did, instead of
     * one frame behind it. The pre-FRIK rebase cannot cover this part: it
     * moves the claim by the wand, and the seat also depends on what ROCK
     * writes after FRIK (the two-handed weapon local, the held body).
     * Bounded: a larger change is a new seat FRIK must solve the arm to.
     */
    inline constexpr float kMaxPresentationTranslationGameUnits = 10.0f;
    inline constexpr float kMaxPresentationRotationDegrees = 30.0f;

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

    enum class PresentationDecision : std::uint8_t
    {
        // No claim was consumed, none is held now, or the wrist is unreadable.
        NoClaim,
        // The target did not change this frame.
        Unchanged,
        // FRIK did not render the consumed target (fallback, recoil kick).
        NotFollowing,
        // The change exceeds a rigid carry; FRIK solves it next frame.
        TooLarge,
        Present,
    };

    struct PresentationPlan
    {
        RE::NiTransform delta{};
        PresentationDecision decision = PresentationDecision::NoClaim;
        // Displacement of the target itself, not of the delta about the origin.
        float translationGameUnits = 0.0f;
        float rotationDegrees = 0.0f;
    };

    /*
     * delta = current * inverse(consumed), applied to the rendered chain. The
     * rendered wrist must be on the consumed target: a hand FRIK solved to
     * the tracked controller instead, or kicked by recoil, is left as FRIK
     * drew it.
     */
    [[nodiscard]] inline PresentationPlan planPresentation(
        const ConsumedTarget& consumed,
        const Claim* current,
        const RE::NiTransform& renderedHandNodeWorld,
        const bool renderedHandNodeValid) noexcept
    {
        PresentationPlan plan{};
        if (!consumed.valid || !current || !current->valid || !renderedHandNodeValid ||
            !isFiniteTransform(consumed.target) || !isFiniteTransform(current->target) || !isFiniteTransform(renderedHandNodeWorld)) {
            return plan;
        }
        if (translationDeltaGameUnits(renderedHandNodeWorld, consumed.target) > kFallbackTranslationGameUnits ||
            rotationDeltaDegrees(renderedHandNodeWorld, consumed.target) > kFallbackRotationDegrees) {
            plan.decision = PresentationDecision::NotFollowing;
            return plan;
        }
        plan.translationGameUnits = translationDeltaGameUnits(current->target, consumed.target);
        plan.rotationDegrees = rotationDeltaDegrees(current->target, consumed.target);
        if (plan.translationGameUnits <= kRepublishTranslationEpsilonGameUnits &&
            plan.rotationDegrees <= kRepublishRotationEpsilonDegrees) {
            plan.decision = PresentationDecision::Unchanged;
            return plan;
        }
        if (plan.translationGameUnits > kMaxPresentationTranslationGameUnits ||
            plan.rotationDegrees > kMaxPresentationRotationDegrees) {
            plan.decision = PresentationDecision::TooLarge;
            return plan;
        }
        // Rendered scene bases: orthonormalize before the transpose inverse.
        const RE::NiTransform delta = transform_math::orthonormalizedTransform(transform_math::composeTransforms(
            transform_math::orthonormalizedTransform(current->target),
            transform_math::invertTransform(transform_math::orthonormalizedTransform(consumed.target))));
        if (!isFiniteTransform(delta)) {
            return plan;
        }
        plan.delta = delta;
        plan.decision = PresentationDecision::Present;
        return plan;
    }
}
