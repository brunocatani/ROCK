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
     * The position drivers carry the chain's translation only: a seat that
     * rides with a hand but is oriented by something else follows the hand
     * and the rig without taking the wrist's turn. The aim-axis drivers add
     * the turn of the axis from the other hand's chain to this hand's chain:
     * the support seat of a two-hand hold sits on a weapon aimed from the
     * carrier toward it, so between ROCK frames its orientation turns with
     * that axis. With the translation-only driver the whole per-frame weapon
     * turn was left for the end-of-frame presentation (ROCK.log PRESENT:
     * p99 5 deg, 9 deg on contacts), and FRIK had solved the arm for last
     * frame's orientation.
     */
    enum class RebaseDriver : std::uint8_t
    {
        Static,
        RightHand,
        LeftHand,
        RightHandPosition,
        LeftHandPosition,
        RightHandAimAxis,
        LeftHandAimAxis,
        RightObjectPivot,
        LeftObjectPivot,
    };

    [[nodiscard]] constexpr bool isPositionDriver(const RebaseDriver driver) noexcept
    {
        return driver == RebaseDriver::RightHandPosition || driver == RebaseDriver::LeftHandPosition;
    }

    [[nodiscard]] constexpr bool isObjectPivotDriver(const RebaseDriver driver) noexcept
    {
        return driver == RebaseDriver::RightObjectPivot || driver == RebaseDriver::LeftObjectPivot;
    }

    [[nodiscard]] constexpr bool isAimAxisDriver(const RebaseDriver driver) noexcept
    {
        return driver == RebaseDriver::RightHandAimAxis || driver == RebaseDriver::LeftHandAimAxis || isObjectPivotDriver(driver);
    }

    // Two hand chains closer than this do not define an aim axis.
    inline constexpr float kMinAimAxisLengthGameUnits = 1.0f;

    [[nodiscard]] constexpr std::size_t handIndex(const bool isLeft) noexcept
    {
        return isLeft ? 1u : 0u;
    }

    [[nodiscard]] constexpr RebaseDriver driverForHand(const bool isLeft) noexcept
    {
        return isLeft ? RebaseDriver::LeftHand : RebaseDriver::RightHand;
    }

    [[nodiscard]] constexpr RebaseDriver aimAxisDriverForHand(const bool isLeft) noexcept
    {
        return isLeft ? RebaseDriver::LeftHandAimAxis : RebaseDriver::RightHandAimAxis;
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
        case RebaseDriver::RightHandPosition:
        case RebaseDriver::RightHandAimAxis:
        case RebaseDriver::RightObjectPivot:
            return &frame.hands[handIndex(false)];
        case RebaseDriver::LeftHand:
        case RebaseDriver::LeftHandPosition:
        case RebaseDriver::LeftHandAimAxis:
        case RebaseDriver::LeftObjectPivot:
            return &frame.hands[handIndex(true)];
        default:
            return nullptr;
        }
    }

    // The other hand's chain sample, for the drivers that turn with the axis between the hands.
    [[nodiscard]] inline const DriverSample* otherHandSampleForDriver(const DriverFrame& frame, const RebaseDriver driver) noexcept
    {
        switch (driver) {
        case RebaseDriver::RightHandAimAxis:
        case RebaseDriver::RightObjectPivot:
            return &frame.hands[handIndex(true)];
        case RebaseDriver::LeftHandAimAxis:
        case RebaseDriver::LeftObjectPivot:
            return &frame.hands[handIndex(false)];
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
        // The other hand's chain at publish; valid for aim-axis drivers only.
        DriverSample otherDriverAtPublish{};
        // Mirrors FRIK's publish sequence: the highest wins a priority tie.
        std::uint64_t publishOrder = 0;
        std::uint32_t fallbackFrames = 0;
        bool fallbackReported = false;
        // The target moved without a FRIK publish (re-anchored to the actual
        // driver): the next pass publishes it even without driver motion.
        bool needsPublish = false;
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
     * samples are the pre-FRIK samples of the frame the target was computed
     * in; the other hand's is kept for aim-axis drivers only.
     */
    [[nodiscard]] inline CommitResult commit(
        Registry& registry,
        const std::string_view tag,
        const bool isLeft,
        const int priority,
        const RE::NiTransform& target,
        const RebaseDriver driver,
        const DriverSample& driverAtPublish,
        const DriverSample& otherDriverAtPublish = DriverSample{}) noexcept
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
        claim->otherDriverAtPublish = isAimAxisDriver(driver) ? otherDriverAtPublish : DriverSample{};
        claim->needsPublish = false;
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
     * The stored form (rows are the local axes in world) of the smallest
     * world rotation that turns direction a onto direction b. Identity when
     * either is too short to define a direction or they already agree.
     */
    [[nodiscard]] inline RE::NiMatrix3 storedRotationFromTo(const RE::NiPoint3& a, const RE::NiPoint3& b) noexcept
    {
        RE::NiMatrix3 stored = transform_math::makeIdentityRotation<RE::NiMatrix3>();
        const double ax = a.x, ay = a.y, az = a.z;
        const double bx = b.x, by = b.y, bz = b.z;
        const double la = std::sqrt(ax * ax + ay * ay + az * az);
        const double lb = std::sqrt(bx * bx + by * by + bz * bz);
        if (!(la > kMinAimAxisLengthGameUnits) || !(lb > kMinAimAxisLengthGameUnits)) {
            return stored;
        }
        const double ux = ax / la, uy = ay / la, uz = az / la;
        const double vx = bx / lb, vy = by / lb, vz = bz / lb;
        double cosine = std::clamp(ux * vx + uy * vy + uz * vz, -1.0, 1.0);
        double kx = uy * vz - uz * vy;
        double ky = uz * vx - ux * vz;
        double kz = ux * vy - uy * vx;
        double sine = std::sqrt(kx * kx + ky * ky + kz * kz);
        if (sine > 1e-9) {
            kx /= sine;
            ky /= sine;
            kz /= sine;
        } else if (cosine > 0.0) {
            return stored;
        } else {
            // Antiparallel: half a turn about any axis perpendicular to a.
            const double hx = std::abs(ux) < 0.9 ? 1.0 : 0.0;
            const double hy = std::abs(ux) < 0.9 ? 0.0 : 1.0;
            kx = uy * 0.0 - uz * hy;
            ky = uz * hx - ux * 0.0;
            kz = ux * hy - uy * hx;
            const double lk = std::sqrt(kx * kx + ky * ky + kz * kz);
            kx /= lk;
            ky /= lk;
            kz /= lk;
            cosine = -1.0;
            sine = 0.0;
        }
        // Rodrigues, math form m (v' = m v); the stored form is its transpose.
        const double t = 1.0 - cosine;
        const double m[3][3] = {
            { cosine + kx * kx * t, kx * ky * t - kz * sine, kx * kz * t + ky * sine },
            { ky * kx * t + kz * sine, cosine + ky * ky * t, ky * kz * t - kx * sine },
            { kz * kx * t - ky * sine, kz * ky * t + kx * sine, cosine + kz * kz * t },
        };
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                stored.entry[row][column] = static_cast<float>(m[column][row]);
            }
        }
        return stored;
    }

    /*
     * Move a claim by its driver's motion since publish:
     * target' = driverNow * inverse(driverAtPublish) * target.
     * A position driver moves the target by the chain's translation only; an
     * aim-axis driver adds the turn of the axis from the other hand's chain
     * to this one (translation only when the other sample is missing).
     * Not republished when the driver is static, either sample is missing, the
     * result is not finite, or the motion is below the epsilon.
     */
    [[nodiscard]] inline RebasePlan planRebase(const Claim& claim, const DriverSample& driverNow, const DriverSample& otherDriverNow) noexcept
    {
        RebasePlan plan{ .target = claim.target, .republish = false };
        if (claim.driver == RebaseDriver::Static || !claim.driverAtPublish.valid || !driverNow.valid ||
            !isFiniteTransform(claim.driverAtPublish.world) || !isFiniteTransform(driverNow.world)) {
            return plan;
        }
        RE::NiTransform rebased = claim.target;
        if (isObjectPivotDriver(claim.driver)) {
            // Both hands are seats on one rigid object. Transport both by
            // the same carrier motion and two-hand aim, never by each hand's
            // independent fore/aft translation (which slides a locked seat).
            const auto carrierDelta = transform_math::composeTransforms(
                transform_math::orthonormalizedTransform(driverNow.world),
                transform_math::invertTransform(transform_math::orthonormalizedTransform(claim.driverAtPublish.world)));
            rebased = transform_math::composeTransforms(carrierDelta, claim.target);
            if (otherDriverNow.valid && claim.otherDriverAtPublish.valid &&
                isFiniteTransform(otherDriverNow.world) && isFiniteTransform(claim.otherDriverAtPublish.world)) {
                const auto oldAxis = claim.otherDriverAtPublish.world.translate - claim.driverAtPublish.world.translate;
                const auto newAxis = otherDriverNow.world.translate - driverNow.world.translate;
                auto swing = transform_math::makeIdentityTransform<RE::NiTransform>();
                swing.rotate = storedRotationFromTo(transform_math::localVectorToWorld(carrierDelta, oldAxis), newAxis);
                rebased.translate = driverNow.world.translate + transform_math::localVectorToWorld(
                    swing, rebased.translate - driverNow.world.translate);
                rebased.rotate = transform_math::orthonormalizeStoredRotation(
                    transform_math::multiplyStoredRotations(rebased.rotate, swing.rotate));
            }
        } else if (isPositionDriver(claim.driver) || isAimAxisDriver(claim.driver)) {
            // The driver's translation only: the seat keeps its offset from
            // the hand, and its orientation unless the aim axis turned.
            rebased.translate.x += driverNow.world.translate.x - claim.driverAtPublish.world.translate.x;
            rebased.translate.y += driverNow.world.translate.y - claim.driverAtPublish.world.translate.y;
            rebased.translate.z += driverNow.world.translate.z - claim.driverAtPublish.world.translate.z;
            if (isAimAxisDriver(claim.driver) && otherDriverNow.valid && claim.otherDriverAtPublish.valid &&
                isFiniteTransform(otherDriverNow.world) && isFiniteTransform(claim.otherDriverAtPublish.world)) {
                const RE::NiPoint3 axisAtPublish{
                    claim.driverAtPublish.world.translate.x - claim.otherDriverAtPublish.world.translate.x,
                    claim.driverAtPublish.world.translate.y - claim.otherDriverAtPublish.world.translate.y,
                    claim.driverAtPublish.world.translate.z - claim.otherDriverAtPublish.world.translate.z,
                };
                const RE::NiPoint3 axisNow{
                    driverNow.world.translate.x - otherDriverNow.world.translate.x,
                    driverNow.world.translate.y - otherDriverNow.world.translate.y,
                    driverNow.world.translate.z - otherDriverNow.world.translate.z,
                };
                rebased.rotate = transform_math::orthonormalizeStoredRotation(
                    transform_math::multiplyStoredRotations(claim.target.rotate, storedRotationFromTo(axisAtPublish, axisNow)));
            }
        } else {
            // Scene driver bases carry float drift; the transpose inverse only
            // cancels for orthonormal bases, and the rebased target is rendered.
            const RE::NiTransform delta = transform_math::composeTransforms(
                transform_math::orthonormalizedTransform(driverNow.world),
                transform_math::invertTransform(transform_math::orthonormalizedTransform(claim.driverAtPublish.world)));
            rebased = transform_math::orthonormalizedTransform(transform_math::composeTransforms(delta, claim.target));
        }
        if (!isFiniteTransform(rebased)) {
            return plan;
        }
        plan.target = rebased;
        plan.republish =
            translationDeltaGameUnits(rebased, claim.target) > kRepublishTranslationEpsilonGameUnits ||
            rotationDeltaDegrees(rebased, claim.target) > kRepublishRotationEpsilonDegrees;
        return plan;
    }

    [[nodiscard]] inline RebasePlan planRebase(const Claim& claim, const DriverSample& driverNow) noexcept
    {
        return planRebase(claim, driverNow, DriverSample{});
    }

    /*
     * Commit a rebase that FRIK accepted: the claim now describes the target
     * against the new driver samples.
     */
    inline void applyRebase(Claim& claim, const RebasePlan& plan, const DriverSample& driverNow, const DriverSample& otherDriverNow = DriverSample{}) noexcept
    {
        claim.target = plan.target;
        claim.driverAtPublish = driverNow;
        if (isAimAxisDriver(claim.driver)) {
            claim.otherDriverAtPublish = otherDriverNow;
        }
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
                const DriverSample* other = otherHandSampleForDriver(driverFrame, claim.driver);
                const RebasePlan plan = planRebase(claim, *sample, other ? *other : DriverSample{});
                entry.target = plan.target;
                entry.moved = plan.republish || claim.needsPublish;
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
            if (const DriverSample* other = otherHandSampleForDriver(driverFrame, claim.driver)) {
                claim.otherDriverAtPublish = *other;
            }
            claim.needsPublish = false;
        }
        claim.publishOrder = registry.nextPublishOrder++;
    }

    /*
     * The pass rebased every claim by a predicted driver sample. Once the
     * actual sample is known (after FRIK's frame) each claim is re-expressed
     * against it: the target moves by actual versus predicted and the samples
     * are replaced, so prediction error never accumulates on a claim that is
     * not republished by its owner. FRIK still holds the predicted target, so
     * a moved claim is flagged for the next pass.
     */
    inline void reanchorClaims(Registry& registry, const DriverFrame& actualFrame) noexcept
    {
        for (auto& claim : registry.claims) {
            if (!claim.valid || claim.driver == RebaseDriver::Static || !claim.driverAtPublish.valid) {
                continue;
            }
            const DriverSample* own = sampleForDriver(actualFrame, claim.driver);
            if (!own || !own->valid || !isFiniteTransform(own->world)) {
                continue;
            }
            const DriverSample* other = otherHandSampleForDriver(actualFrame, claim.driver);
            const DriverSample otherSample = other ? *other : DriverSample{};
            const RebasePlan plan = planRebase(claim, *own, otherSample);
            if (plan.republish) {
                claim.target = plan.target;
                claim.needsPublish = true;
            }
            claim.driverAtPublish = *own;
            if (isAimAxisDriver(claim.driver) && otherSample.valid) {
                claim.otherDriverAtPublish = otherSample;
            }
        }
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
