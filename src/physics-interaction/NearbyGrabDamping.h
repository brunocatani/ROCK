#pragma once

/*
 * HIGGS temporarily calms nearby dynamic bodies at grab start so the new held
 * constraint does not kick surrounding clutter into solver jitter. FO4VR hknp
 * damping fields are not represented in ROCK's verified local structs. The
 * runtime path therefore only owns discovery/lifetime scaffolding until verified
 * motion-property damping can replace the old repeated velocity writes.
 */

#include "ObjectPhysicsBodySet.h"

#include "RE/NetImmerse/NiPoint.h"

#include <cstdint>
#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <vector>

namespace RE
{
    class TESObjectREFR;
    class bhkWorld;
    class hknpWorld;
}

namespace frik::rock::nearby_grab_damping
{
    inline constexpr std::uint32_t INVALID_BODY_ID = 0x7FFF'FFFF;

    struct PureDampingCandidate
    {
        std::uint32_t bodyId = INVALID_BODY_ID;
        std::uint32_t motionId = 0;
        bool accepted = false;
        bool heldBySameHand = false;
    };

    class PureDampingCandidateSet
    {
    public:
        void add(const PureDampingCandidate& candidate)
        {
            if (candidate.bodyId == INVALID_BODY_ID || candidate.motionId == 0 || candidate.heldBySameHand || !candidate.accepted) {
                return;
            }
            _records.push_back(candidate);
        }

        std::vector<std::uint32_t> uniqueAcceptedMotionBodyIds() const
        {
            std::vector<std::uint32_t> result;
            std::unordered_set<std::uint32_t> seenMotionIds;
            result.reserve(_records.size());
            for (const auto& record : _records) {
                if (!record.accepted || record.heldBySameHand || record.bodyId == INVALID_BODY_ID || record.motionId == 0) {
                    continue;
                }
                if (seenMotionIds.insert(record.motionId).second) {
                    result.push_back(record.bodyId);
                }
            }
            return result;
        }

        std::uint32_t duplicateMotionSkips() const
        {
            std::uint32_t skips = 0;
            std::unordered_set<std::uint32_t> seenMotionIds;
            for (const auto& record : _records) {
                if (!record.accepted || record.heldBySameHand || record.bodyId == INVALID_BODY_ID || record.motionId == 0) {
                    continue;
                }
                if (!seenMotionIds.insert(record.motionId).second) {
                    ++skips;
                }
            }
            return skips;
        }

        bool containsBodyId(std::uint32_t bodyId) const
        {
            return std::any_of(_records.begin(), _records.end(), [&](const PureDampingCandidate& record) { return record.bodyId == bodyId; });
        }

    private:
        std::vector<PureDampingCandidate> _records;
    };

    struct SavedNearbyMotionDamping
    {
        std::uint32_t representativeBodyId = INVALID_BODY_ID;
        std::uint32_t motionId = 0;
        float originalLinearDamping = 0.0f;
        float originalAngularDamping = 0.0f;
        bool active = false;
    };

    struct NearbyGrabDampingState
    {
        std::vector<SavedNearbyMotionDamping> motions;
        float remainingSeconds = 0.0f;
        float linearDamping = 0.0f;
        float angularDamping = 0.0f;
        bool active = false;

        void clear()
        {
            motions.clear();
            remainingSeconds = 0.0f;
            linearDamping = 0.0f;
            angularDamping = 0.0f;
            active = false;
        }
    };

    inline float sanitizeRadius(float radiusGame)
    {
        if (!std::isfinite(radiusGame)) {
            return 0.0f;
        }
        return (std::max)(0.0f, radiusGame);
    }

    inline float sanitizeDuration(float durationSeconds)
    {
        if (!std::isfinite(durationSeconds)) {
            return 0.0f;
        }
        return (std::max)(0.0f, durationSeconds);
    }

    inline float sanitizeDamping(float damping)
    {
        if (!std::isfinite(damping)) {
            return 0.0f;
        }
        return std::clamp(damping, 0.0f, 1.0f);
    }

    inline constexpr bool runtimeDampingWritesVerified() { return false; }

    inline bool shouldBeginRuntimeNearbyDamping(bool enabled, bool verifiedRuntimeWriter, float radiusGame, float durationSeconds)
    {
        return enabled && verifiedRuntimeWriter && sanitizeRadius(radiusGame) > 0.0f && sanitizeDuration(durationSeconds) > 0.0f;
    }

    template <class Vec3>
    Vec3 applyDampingToVelocity(const Vec3& velocity, float damping)
    {
        const float keep = 1.0f - sanitizeDamping(damping);
        Vec3 result = velocity;
        result.x *= keep;
        result.y *= keep;
        result.z *= keep;
        return result;
    }

    inline bool advanceTimer(NearbyGrabDampingState& state, float deltaTime)
    {
        if (!state.active) {
            return false;
        }

        const float safeDelta = std::isfinite(deltaTime) ? (std::max)(0.0f, deltaTime) : 0.0f;
        state.remainingSeconds = (std::max)(0.0f, state.remainingSeconds - safeDelta);
        if (state.remainingSeconds > 0.0f) {
            return false;
        }

        state.active = false;
        return true;
    }

    NearbyGrabDampingState beginNearbyGrabDamping(RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        RE::TESObjectREFR* heldRef,
        const std::vector<std::uint32_t>& heldBodyIds,
        RE::NiPoint3 centerGame,
        float radiusGame,
        float durationSeconds,
        float linearDamping,
        float angularDamping,
        const object_physics_body_set::BodySetScanOptions& baseOptions);

    void tickNearbyGrabDamping(RE::hknpWorld* world, NearbyGrabDampingState& state, float deltaTime);

    void restoreNearbyGrabDamping(RE::hknpWorld* world, NearbyGrabDampingState& state);
}
