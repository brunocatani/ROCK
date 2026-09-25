#pragma once

#include <cstdint>
#include <cstddef>
#include <atomic>
#include "physics-interaction/object/PhysicsBodyClassifier.h"

namespace rock::held_placement_policy
{
    inline constexpr std::size_t kMaxBodies = 64;

    // One submission owner for one update. Invalidation advances the token so
    // an old-world request cannot become valid again after a rebuild.
    class FrameLease {
    public:
        enum class Admission { Accepted, InvalidOwner, Stale, Busy };
        std::uint64_t token() const noexcept { return _token; }
        std::uint64_t owner() const noexcept { return _owner; }
        Admission submit(std::uint64_t owner, std::uint64_t token) noexcept {
            if (!owner) return Admission::InvalidOwner;
            if (token != _token) return Admission::Stale;
            if (_owner && _owner != owner) return Admission::Busy;
            _owner = owner;
            return Admission::Accepted;
        }
        std::uint64_t consume() noexcept {
            const auto owner = _owner;
            invalidate();
            return owner;
        }
        void clear(std::uint64_t owner) noexcept { if (_owner == owner) _owner = 0; }
        void invalidate() noexcept { _owner = 0; ++_token; }
    private:
        std::uint64_t _token{1}, _owner{};
    };

    struct SurfaceContact {
        std::uint32_t heldBodyId{0x7FFF'FFFFu};
        std::uint32_t otherBodyId{0x7FFF'FFFFu};
        bool recent{};
    };

    // Physics callbacks publish one coherent pair. The game frame ages it;
    // normal grab cleanup clears it. No native pointer crosses that boundary.
    class SurfaceContactState {
    public:
        void publish(std::uint32_t held, std::uint32_t other) noexcept {
            _pair.store((std::uint64_t{held}<<32)|other,std::memory_order_relaxed);
            _age.store(0,std::memory_order_release);
        }
        SurfaceContact read() const noexcept {
            if (_age.load(std::memory_order_acquire)>=5) return {};
            const auto pair=_pair.load(std::memory_order_acquire);
            return {static_cast<std::uint32_t>(pair>>32),static_cast<std::uint32_t>(pair),true};
        }
        void tick() noexcept {
            auto age=_age.load(std::memory_order_acquire);
            if (age<5) _age.compare_exchange_strong(age,age+1,std::memory_order_release);
        }
        void clear() noexcept { _age.store(5,std::memory_order_release); }
    private:
        std::atomic<std::uint64_t> _pair{};
        std::atomic<unsigned> _age{5};
    };

    // A static part has no movable motion ID. It needs no hand drive or
    // keyframing, but must be proven to belong to the placed reference.
    [[nodiscard]] constexpr bool preserveStaticBody(physics_body_classifier::BodyRejectReason rejection,
        physics_body_classifier::BodyMotionType motion, std::uint32_t motionId, bool sameKnownReference) noexcept
    {
        return sameKnownReference && motionId == 0 &&
            motion == physics_body_classifier::BodyMotionType::Static &&
            rejection == physics_body_classifier::BodyRejectReason::InvalidMotionId;
    }

    [[nodiscard]] constexpr bool anchoredMotion(physics_body_classifier::BodyMotionType before,
        physics_body_classifier::BodyMotionType after) noexcept
    {
        using physics_body_classifier::BodyMotionType;
        return (before == BodyMotionType::Static && after == BodyMotionType::Static) ||
            (before == BodyMotionType::Dynamic && after == BodyMotionType::Keyframed);
    }

    // Validate a fresh scan at placement. The grab lifecycle's historical
    // completeness flag describes its cached prep and is not a placement veto.
    [[nodiscard]] constexpr bool completeBodyScan(bool finished, std::size_t bodies,
        std::size_t validated, std::uint32_t skippedOrFailed) noexcept
    {
        return finished && bodies > 0 && bodies <= kMaxBodies && validated == bodies && skippedOrFailed == 0;
    }

    // A two-handed hold is one reference. Two different objects are ambiguous.
    [[nodiscard]] constexpr std::uint32_t singleObject(std::uint32_t right, std::uint32_t left) noexcept
    {
        return right && left && right != left ? 0 : (right ? right : left);
    }

}
