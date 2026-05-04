#pragma once

/*
 * Held-object motion has one runtime writer for player-space velocity. The hand
 * update may sample current velocity for release history, but it must not also
 * write per-frame body velocity while the central interaction pass writes the
 * same held motions. This registry deduplicates by motion index and exposes a
 * writer mask so debug logs can prove the steady-state authority path remains
 * ConstraintTarget plus PlayerSpaceCentral.
 */

#include <algorithm>
#include <cstdint>
#include <vector>

namespace RE
{
    class hknpWorld;
    class NiPoint3;
    class NiTransform;
}

namespace rock::held_player_space_registry
{
    inline constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;

    struct PureVec3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    enum class WriterKind : std::uint8_t
    {
        ConstraintTarget,
        PlayerSpaceCentral,
        ReleaseVelocity,
        HeldLoopVelocity,
        NearbyDampingVelocity,
        CharacterControllerHook,
    };

    inline constexpr std::uint32_t writerBit(WriterKind kind) { return 1u << static_cast<std::uint32_t>(kind); }

    struct HeldBodyRegistration
    {
        std::uint32_t bodyId = kInvalidBodyId;
        std::uint32_t motionIndex = 0;
    };

    struct HeldMotionSample
    {
        std::uint32_t bodyId = kInvalidBodyId;
        std::uint32_t motionIndex = 0;
        PureVec3 linearVelocity{};
        PureVec3 angularVelocity{};
    };

    struct HeldMotionWrite
    {
        bool shouldWrite = false;
        bool duplicateMotion = false;
        std::uint32_t bodyId = kInvalidBodyId;
        std::uint32_t motionIndex = 0;
        PureVec3 linearVelocity{};
        PureVec3 angularVelocity{};
    };

    struct RuntimeHeldPlayerSpaceResult
    {
        std::uint32_t registeredBodies = 0;
        std::uint32_t motionsWritten = 0;
        std::uint32_t transformsWarped = 0;
        std::uint32_t duplicateMotionSkips = 0;
        std::uint32_t writerMask = 0;
    };

    inline PureVec3 makeVec3(float x, float y, float z) { return PureVec3{ .x = x, .y = y, .z = z }; }

    inline bool shouldCarryPreviousPlayerVelocity(bool enabled, bool warp, std::uint32_t writtenMotionCount)
    {
        return enabled && !warp && writtenMotionCount > 0;
    }

    class HeldPlayerSpaceRegistry
    {
    public:
        void clear()
        {
            _registered.clear();
            _writtenMotionIndices.clear();
            _currentPlayerVelocity = {};
            _previousPlayerVelocity = {};
            _residualVelocityKeep = 1.0f;
            _writerMask = 0;
        }

        void beginFrame(PureVec3 currentPlayerVelocity, PureVec3 previousPlayerVelocity, float residualVelocityKeep)
        {
            _registered.clear();
            _writtenMotionIndices.clear();
            _currentPlayerVelocity = currentPlayerVelocity;
            _previousPlayerVelocity = previousPlayerVelocity;
            _residualVelocityKeep = std::clamp(residualVelocityKeep, 0.0f, 1.0f);
            _writerMask = 0;
        }

        void registerBody(std::uint32_t bodyId, std::uint32_t motionIndex)
        {
            if (bodyId == kInvalidBodyId || motionIndex == 0) {
                return;
            }
            const auto duplicateBody = std::find_if(_registered.begin(), _registered.end(), [&](const HeldBodyRegistration& entry) {
                return entry.bodyId == bodyId;
            });
            if (duplicateBody == _registered.end()) {
                _registered.push_back(HeldBodyRegistration{ .bodyId = bodyId, .motionIndex = motionIndex });
            }
        }

        HeldMotionWrite solveBodyVelocity(const HeldMotionSample& sample)
        {
            HeldMotionWrite result{};
            result.bodyId = sample.bodyId;
            result.motionIndex = sample.motionIndex;
            if (sample.bodyId == kInvalidBodyId || sample.motionIndex == 0 || !isRegistered(sample.bodyId)) {
                return result;
            }
            if (motionAlreadyWritten(sample.motionIndex)) {
                result.duplicateMotion = true;
                return result;
            }

            _writtenMotionIndices.push_back(sample.motionIndex);
            const PureVec3 localLinear{
                sample.linearVelocity.x - _previousPlayerVelocity.x,
                sample.linearVelocity.y - _previousPlayerVelocity.y,
                sample.linearVelocity.z - _previousPlayerVelocity.z,
            };
            result.linearVelocity = PureVec3{
                _currentPlayerVelocity.x + localLinear.x * _residualVelocityKeep,
                _currentPlayerVelocity.y + localLinear.y * _residualVelocityKeep,
                _currentPlayerVelocity.z + localLinear.z * _residualVelocityKeep,
            };
            result.angularVelocity = PureVec3{
                sample.angularVelocity.x * _residualVelocityKeep,
                sample.angularVelocity.y * _residualVelocityKeep,
                sample.angularVelocity.z * _residualVelocityKeep,
            };
            result.shouldWrite = true;
            return result;
        }

        void recordWriter(WriterKind kind) { _writerMask |= writerBit(kind); }

        bool writerMaskIsSteadyStateExpected() const
        {
            constexpr std::uint32_t expectedMask = writerBit(WriterKind::ConstraintTarget) | writerBit(WriterKind::PlayerSpaceCentral);
            return (_writerMask & ~expectedMask) == 0 && (_writerMask & expectedMask) == expectedMask;
        }

        std::uint32_t writerMask() const { return _writerMask; }
        std::size_t registeredBodyCount() const { return _registered.size(); }
        const std::vector<HeldBodyRegistration>& registrations() const { return _registered; }

    private:
        bool isRegistered(std::uint32_t bodyId) const
        {
            return std::any_of(_registered.begin(), _registered.end(), [&](const HeldBodyRegistration& entry) {
                return entry.bodyId == bodyId;
            });
        }

        bool motionAlreadyWritten(std::uint32_t motionIndex) const
        {
            return std::find(_writtenMotionIndices.begin(), _writtenMotionIndices.end(), motionIndex) != _writtenMotionIndices.end();
        }

        std::vector<HeldBodyRegistration> _registered;
        std::vector<std::uint32_t> _writtenMotionIndices;
        PureVec3 _currentPlayerVelocity{};
        PureVec3 _previousPlayerVelocity{};
        float _residualVelocityKeep = 1.0f;
        std::uint32_t _writerMask = 0;
    };

    RuntimeHeldPlayerSpaceResult applyCentralPlayerSpaceVelocity(RE::hknpWorld* world,
        const std::vector<std::uint32_t>& bodyIds,
        const RE::NiPoint3& currentPlayerVelocityHavok,
        const RE::NiPoint3& previousPlayerVelocityHavok,
        float residualVelocityKeep,
        bool enabled,
        bool warp,
        const RE::NiTransform* previousPlayerSpaceWorld = nullptr,
        const RE::NiTransform* currentPlayerSpaceWorld = nullptr);
}
