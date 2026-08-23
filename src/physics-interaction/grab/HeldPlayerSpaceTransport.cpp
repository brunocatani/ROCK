#include "physics-interaction/grab/HeldPlayerSpaceTransport.h"

#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/native/CharacterControllerRuntime.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/PhysicsScale.h"

#include <algorithm>
#include <cmath>

namespace rock
{
    namespace
    {
        constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFFu;
        constexpr float kWarpTranslationGameUnits = 35.0f;
        constexpr float kWarpRotationDegrees = 0.5729578f;  // 0.01 radians.
        constexpr float kMaximumControllerSpeedGameUnitsPerSecond = 5000.0f;
        constexpr float kVelocityWriteEpsilonGameUnitsPerSecond = 0.001f;

        bool isFinitePoint(const RE::NiPoint3& value) noexcept
        {
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
        }

        bool isFiniteTransform(const RE::NiTransform& value) noexcept
        {
            if (!isFinitePoint(value.translate) || !std::isfinite(value.scale)) {
                return false;
            }
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(value.rotate.entry[row][column])) {
                        return false;
                    }
                }
            }
            return true;
        }

        float vectorMagnitude(const RE::NiPoint3& value) noexcept
        {
            return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
        }

        float rotationDeltaDegrees(const RE::NiMatrix3& lhs, const RE::NiMatrix3& rhs) noexcept
        {
            float trace = 0.0f;
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    trace += lhs.entry[row][column] * rhs.entry[row][column];
                }
            }
            const float cosine = std::clamp((trace - 1.0f) * 0.5f, -1.0f, 1.0f);
            return std::acos(cosine) * 57.29577951308232f;
        }

        bool isDynamicMotion(const RE::hknpMotion& motion) noexcept
        {
            return static_cast<std::int16_t>(motion.packedInverseInertia[3]) != 0;
        }

        RE::NiPoint3 subtract(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs) noexcept
        {
            return RE::NiPoint3{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
        }

        RE::NiPoint3 negate(const RE::NiPoint3& value) noexcept
        {
            return RE::NiPoint3{ -value.x, -value.y, -value.z };
        }

        RE::NiPoint3 scaleVelocityToHavok(const RE::NiPoint3& value) noexcept
        {
            const float scale = physics_scale::gameToHavok();
            return RE::NiPoint3{ value.x * scale, value.y * scale, value.z * scale };
        }
    }

    const char* HeldPlayerSpaceTransport::actionName(Action action) noexcept
    {
        switch (action) {
        case Action::Idle:
            return "idle";
        case Action::Rebase:
            return "rebase";
        case Action::Velocity:
            return "velocity";
        case Action::Warp:
            return "warp";
        case Action::Hold:
            return "hold";
        case Action::Failed:
            return "failed";
        }
        return "unknown";
    }

    void HeldPlayerSpaceTransport::queueSourceFrame(
        const RE::NiTransform& playerSpaceWorld,
        bool valid,
        std::uint64_t sourceSequence)
    {
        std::scoped_lock lock(_mutex);
        if (sourceSequence == 0 || sourceSequence == _queuedSource.sequence) {
            return;
        }

        _queuedSource.world = playerSpaceWorld;
        _queuedSource.sequence = sourceSequence;
        _queuedSource.valid = valid && isFiniteTransform(playerSpaceWorld);
    }

    void HeldPlayerSpaceTransport::clearRuntimeLocked() noexcept
    {
        _world = nullptr;
        _previousSource = {};
        _processedSourceSequence = 0;
        _velocitySuppressedSourceSequence = 0;
        _previousSourceWarped = false;
        _motionStates = {};
        _motionStateCount = 0;
        _telemetry = {};
        _failureLogCounter = 0;
    }

    void HeldPlayerSpaceTransport::reset()
    {
        std::scoped_lock lock(_mutex);
        clearRuntimeLocked();
        _queuedSource = {};
    }

    HeldPlayerSpaceTransport::Telemetry HeldPlayerSpaceTransport::telemetrySnapshot() const
    {
        std::scoped_lock lock(_mutex);
        return _telemetry;
    }

    void HeldPlayerSpaceTransport::flushPreCollide(
        RE::hknpWorld* world,
        std::span<const std::uint32_t> heldBodyIds,
        const havok_physics_timing::PhysicsTimingSample& timing)
    {
        std::scoped_lock lock(_mutex);

        if (!world) {
            clearRuntimeLocked();
            return;
        }
        if (_world != world) {
            clearRuntimeLocked();
            _world = world;
        }

        _telemetry.action = Action::Idle;
        _telemetry.physicsStepSequence = timing.stepSequence;
        _telemetry.sourceSequence = _queuedSource.sequence;
        _telemetry.requestedBodyCount = static_cast<std::uint32_t>(heldBodyIds.size());
        _telemetry.uniqueMotionCount = 0;
        _telemetry.velocityWriteCount = 0;
        _telemetry.warpWriteCount = 0;
        _telemetry.failedWriteCount = 0;
        _telemetry.sourceTranslationDeltaGameUnits = 0.0f;
        _telemetry.sourceRotationDeltaDegrees = 0.0f;
        _telemetry.largestVelocityDeltaGameUnitsPerSecond = {};
        _telemetry.sourceValid = _queuedSource.valid;

        if (heldBodyIds.empty()) {
            // Do not subtract on release. The object keeps real player motion
            // as part of its release velocity.
            _motionStates = {};
            _motionStateCount = 0;
            return;
        }

        bool sourceAdvanced = false;
        bool currentSourceWarped = false;
        bool executeWarp = false;
        RE::NiTransform deltaRoomWorld = transform_math::makeIdentityTransform<RE::NiTransform>();
        if (_queuedSource.sequence != 0 && _queuedSource.sequence != _processedSourceSequence) {
            sourceAdvanced = true;
            _processedSourceSequence = _queuedSource.sequence;

            if (_queuedSource.valid && _previousSource.valid) {
                const RE::NiPoint3 translationDelta = subtract(
                    _queuedSource.world.translate,
                    _previousSource.world.translate);
                _telemetry.sourceTranslationDeltaGameUnits = vectorMagnitude(translationDelta);
                _telemetry.sourceRotationDeltaDegrees = rotationDeltaDegrees(
                    _queuedSource.world.rotate,
                    _previousSource.world.rotate);
                currentSourceWarped =
                    _telemetry.sourceTranslationDeltaGameUnits > kWarpTranslationGameUnits ||
                    _telemetry.sourceRotationDeltaDegrees > kWarpRotationDegrees;
                executeWarp = currentSourceWarped || _previousSourceWarped;
                if (executeWarp) {
                    deltaRoomWorld = transform_math::composeTransforms(
                        _queuedSource.world,
                        transform_math::invertTransform(_previousSource.world));
                    _velocitySuppressedSourceSequence = _queuedSource.sequence;
                }
            } else {
                _telemetry.action = Action::Rebase;
            }

            _previousSource = _queuedSource;
            _previousSourceWarped = currentSourceWarped;
        }

        RE::NiPoint3 controllerVelocityGame{};
        const bool controllerVelocityRead =
            character_controller_runtime::tryGetPlayerLocomotionVelocityRawGameUnits(
                controllerVelocityGame);
        const float controllerSpeed = controllerVelocityRead && isFinitePoint(controllerVelocityGame) ?
            vectorMagnitude(controllerVelocityGame) :
            0.0f;
        const bool controllerVelocityValid =
            controllerVelocityRead &&
            isFinitePoint(controllerVelocityGame) &&
            controllerSpeed <= kMaximumControllerSpeedGameUnitsPerSecond;
        _telemetry.controllerVelocityValid = controllerVelocityValid;
        if (controllerVelocityValid) {
            _telemetry.roomVelocityGameUnitsPerSecond = controllerVelocityGame;
        } else {
            ++_telemetry.invalidVelocityCount;
        }

        std::array<MotionState, kMaxTrackedMotions> nextStates{};
        std::size_t nextStateCount = 0;

        const auto findPreviousState = [&](std::uint32_t motionIndex, std::uint32_t motionFirstBodyId) -> const MotionState* {
            for (std::size_t index = 0; index < _motionStateCount; ++index) {
                const auto& state = _motionStates[index];
                if (state.motionIndex == motionIndex &&
                    state.motionFirstBodyId == motionFirstBodyId) {
                    return &state;
                }
            }
            return nullptr;
        };

        const auto motionAlreadyQueued = [&](std::uint32_t motionIndex) {
            for (std::size_t index = 0; index < nextStateCount; ++index) {
                if (nextStates[index].motionIndex == motionIndex) {
                    return true;
                }
            }
            return false;
        };

        for (const std::uint32_t bodyId : heldBodyIds) {
            if (bodyId == kInvalidBodyId || nextStateCount >= nextStates.size()) {
                continue;
            }

            auto* body = havok_runtime::getBody(world, RE::hknpBodyId{ bodyId });
            if (!body ||
                !body_frame::hasUsableMotionIndex(body->motionIndex) ||
                motionAlreadyQueued(body->motionIndex)) {
                continue;
            }

            auto* motion = havok_runtime::getMotion(world, body->motionIndex);
            if (!motion || !isDynamicMotion(*motion)) {
                continue;
            }

            const MotionState* previous = findPreviousState(
                body->motionIndex,
                motion->firstBodyId);
            MotionState next{
                .bodyId = bodyId,
                .motionIndex = body->motionIndex,
                .motionFirstBodyId = motion->firstBodyId,
                .contributionGameUnitsPerSecond = previous ?
                    previous->contributionGameUnitsPerSecond :
                    RE::NiPoint3{},
            };

            bool writeFailed = false;
            if (executeWarp && sourceAdvanced) {
                const RE::NiPoint3 removeContribution = negate(
                    next.contributionGameUnitsPerSecond);
                if (vectorMagnitude(removeContribution) >
                    kVelocityWriteEpsilonGameUnitsPerSecond) {
                    if (havok_runtime::applyLinearVelocityDeltaDeferred(
                            world,
                            bodyId,
                            scaleVelocityToHavok(removeContribution))) {
                        next.contributionGameUnitsPerSecond = {};
                        ++_telemetry.velocityWriteCount;
                    } else {
                        writeFailed = true;
                    }
                }

                RE::NiTransform bodyWorld{};
                if (havok_runtime::tryGetBodyWorldTransform(
                        world,
                        RE::hknpBodyId{ bodyId },
                        bodyWorld)) {
                    const RE::NiTransform transported =
                        transform_math::composeTransforms(
                            deltaRoomWorld,
                            bodyWorld);
                    if (havok_runtime::setBodyTransformDeferred(
                            world,
                            bodyId,
                            transported,
                            1)) {
                        ++_telemetry.warpWriteCount;
                    } else {
                        writeFailed = true;
                    }
                } else {
                    writeFailed = true;
                }
            } else if (_queuedSource.sequence == _velocitySuppressedSourceSequence) {
                // A warp frame never also receives a velocity contribution.
            } else if (controllerVelocityValid) {
                const RE::NiPoint3 velocityDelta = subtract(
                    controllerVelocityGame,
                    next.contributionGameUnitsPerSecond);
                if (vectorMagnitude(velocityDelta) >
                    vectorMagnitude(_telemetry.largestVelocityDeltaGameUnitsPerSecond)) {
                    _telemetry.largestVelocityDeltaGameUnitsPerSecond = velocityDelta;
                }

                if (vectorMagnitude(velocityDelta) >
                    kVelocityWriteEpsilonGameUnitsPerSecond) {
                    if (havok_runtime::applyLinearVelocityDeltaDeferred(
                            world,
                            bodyId,
                            scaleVelocityToHavok(velocityDelta))) {
                        next.contributionGameUnitsPerSecond =
                            controllerVelocityGame;
                        ++_telemetry.velocityWriteCount;
                    } else {
                        writeFailed = true;
                    }
                } else {
                    next.contributionGameUnitsPerSecond = controllerVelocityGame;
                }
            }

            if (writeFailed) {
                ++_telemetry.failedWriteCount;
            }
            nextStates[nextStateCount++] = next;
        }

        _motionStates = nextStates;
        _motionStateCount = nextStateCount;
        _telemetry.uniqueMotionCount = static_cast<std::uint32_t>(nextStateCount);

        if (_telemetry.failedWriteCount != 0) {
            _telemetry.action = Action::Failed;
            if (++_failureLogCounter % 90 == 1) {
                ROCK_LOG_WARN(Hand,
                    "HELD PLAYER-SPACE TRANSPORT failed: requested={} motions={} failures={} action={} sourceSeq={} step={}",
                    _telemetry.requestedBodyCount,
                    _telemetry.uniqueMotionCount,
                    _telemetry.failedWriteCount,
                    executeWarp ? "warp" : "velocity",
                    _telemetry.sourceSequence,
                    _telemetry.physicsStepSequence);
            }
        } else if (executeWarp && sourceAdvanced) {
            _telemetry.action = Action::Warp;
            ++_telemetry.warpCount;
        } else if (_queuedSource.sequence == _velocitySuppressedSourceSequence) {
            _telemetry.action = Action::Hold;
        } else if (controllerVelocityValid) {
            _telemetry.action = Action::Velocity;
        } else if (!sourceAdvanced || _telemetry.action != Action::Rebase) {
            _telemetry.action = Action::Hold;
        }
    }
}
