#pragma once

#include "physics-interaction/grab/GrabMotorLoadPolicy.h"
#include "physics-interaction/native/HavokPhysicsTiming.h"

namespace RE { class hknpWorld; }
namespace rock { struct ActiveConstraint; }

namespace rock::grab_motor_telemetry
{
    enum class Owner : std::uint8_t { RightHand, LeftHand, Weapon };

    struct Command
    {
        std::uintptr_t world = 0;
        std::uintptr_t runtimeIdentity = 0; // comparison token only
        std::uint64_t beforeSolve = 0, source = 0, tag = 0;
        std::uint32_t bodyA = 0, bodyB = 0, motion = 0;
        Owner owner = Owner::RightHand;
        grab_motor_load::Axes limits{};
        std::array<float, 2> tau{}, damping{}, proportional{}, constant{};
        std::array<float, 4> inverseInertiaMass{};
        std::array<float, 3> beforeLinear{}, beforeAngular{};
        float mass = 0.0f, seconds = 0.0f;
        bool contact = false;
        bool pending = false;
        const char* failure = "no-command";
    };

    struct Sample
    {
        Command command{};
        grab_motor_load::Load load{};
        std::array<float, 3> afterLinear{}, afterAngular{};
        std::uint64_t solve = 0;
        float gripError = -1.0f, rotationError = -1.0f, proxyError = -1.0f;
        float solverSeconds = 0.0f;
        std::uint32_t solverSteps = 0, microSteps = 0;
        bool contact = false, teleported = false;
    };

    // Owned by one ActiveConstraint; used under its existing physics/lifetime
    // protection. Fixed storage, copied values only, no additional locks.
    struct State
    {
        Command command{};
        Sample peakGrip{};
        grab_motor_load::Axes peakUtilization{}, nearLimitSeconds{};
        float elapsed = 0.0f, peakRotation = 0.0f;
        std::uint32_t samples = 0, invalid = 0;
        const char* failure = "none";
    };

    [[nodiscard]] bool verifyLayout() noexcept;
    void capture(RE::hknpWorld* world, ActiveConstraint& constraint,
        std::uint32_t bodyA, std::uint32_t bodyB,
        const havok_physics_timing::PhysicsTimingSample& timing,
        std::uint64_t source, std::uint64_t tag, Owner owner,
        float mass, bool contact) noexcept;
    void record(RE::hknpWorld* world, ActiveConstraint& constraint,
        const havok_physics_timing::PhysicsTimingSample& timing,
        float gripError, float rotationError, float proxyError,
        bool contact, bool teleported = false) noexcept;
    void finish(ActiveConstraint& constraint) noexcept;
}
