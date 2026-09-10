#pragma once

#include "physics-interaction/weapon/DynamicWeaponCollisionPolicy.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "physics-interaction/weapon/WeaponAuthority.h"

#include <atomic>
#include <cstdint>

namespace rock::weapon_surface_support
{
    inline constexpr int kButtonId = 32; // Physical right OpenVR Axis0 click.
    inline constexpr std::uint64_t kMaximumSampleAgeMilliseconds = 100;
    inline constexpr float kReturnSeconds = 0.20f;
    inline constexpr float kMinimumAimSeparationGameUnits = 0.25f;
    inline constexpr float kMaximumAcquisitionSeparationGameUnits = 2.0f;

    struct Contact
    {
        std::uintptr_t world{ 0 };
        std::uintptr_t shape{ 0 }; // Identity only; never dereferenced.
        std::uintptr_t collisionObject{ 0 }; // Identity only.
        std::uint64_t generation{ 0 };
        std::uint64_t sampledAtMilliseconds{ 0 };
        std::uint32_t proxyBodyId{ 0x7FFF'FFFFu };
        std::uint32_t surfaceBodyId{ 0x7FFF'FFFFu };
        RE::NiPoint3 weaponPointLocal{};
        RE::NiPoint3 surfacePointLocal{};
        RE::NiTransform surfaceWorld{};
        RE::NiTransform weaponWorld{};
        bool valid{ false };
    };

    // Contact callbacks are producers; the game frame is the consumer. Both
    // make one nonblocking attempt. All payload access owns the flag; a missed
    // sample cannot publish a mixed body/point or interrupt native collision.
    class ContactChannel
    {
    public:
        void publish(const Contact& value) noexcept
        {
            if (_busy.test_and_set(std::memory_order_acquire)) return;
            _value = value;
            _busy.clear(std::memory_order_release);
        }

        bool read(Contact& value) noexcept
        {
            if (_busy.test_and_set(std::memory_order_acquire)) return false;
            value = _value;
            _busy.clear(std::memory_order_release);
            return value.valid;
        }

        // Called only with contact callbacks quiesced.
        void clear() noexcept { _value = {}; }

    private:
        std::atomic_flag _busy = ATOMIC_FLAG_INIT;
        Contact _value{};
    };

    [[nodiscard]] inline bool isFresh(const Contact& contact,
        std::uintptr_t world, std::uint64_t generation,
        std::uint32_t proxyBodyId, std::uint64_t now) noexcept
    {
        return contact.valid && contact.world == world &&
               contact.generation == generation && contact.proxyBodyId == proxyBodyId &&
               contact.shape != 0 && now >= contact.sampledAtMilliseconds &&
               now - contact.sampledAtMilliseconds <= kMaximumSampleAgeMilliseconds;
    }

    [[nodiscard]] inline bool contactStillTouches(const Contact& contact,
        const RE::NiTransform& weaponWorld, const RE::NiTransform& surfaceWorld)
    {
        using namespace dynamic_weapon_collision_policy;
        if (!isFiniteTransform(weaponWorld) || !isFiniteTransform(surfaceWorld) ||
            std::abs(weaponWorld.scale) < 0.0001f || std::abs(surfaceWorld.scale) < 0.0001f) return false;
        const auto weaponPoint = transform_math::localPointToWorld(weaponWorld, contact.weaponPointLocal);
        const auto surfacePoint = transform_math::localPointToWorld(surfaceWorld, contact.surfacePointLocal);
        return isFinitePoint(weaponPoint) && isFinitePoint(surfacePoint) &&
            weaponSolverLength(weaponSolverSub(weaponPoint, surfacePoint)) <= kMaximumAcquisitionSeparationGameUnits;
    }

    struct ToggleInput
    {
        bool available{ false };
        bool held{ false };
        bool pressed{ false };
        std::uint32_t sampleAgeMilliseconds{ 0 };
    };

    // A release is needed after an unavailable/menu sample. Held levels never
    // toggle, and a press spent in the air cannot latch on a later contact.
    struct Toggle
    {
        bool armed{ false };

        bool consume(const ToggleInput& input) noexcept
        {
            if (!input.available || input.sampleAgeMilliseconds > kMaximumSampleAgeMilliseconds) {
                armed = false;
                return false;
            }
            if (!input.held) {
                const bool click = armed && input.pressed;
                armed = true;
                return click;
            }
            if (!armed || !input.pressed) return false;
            armed = false;
            return true;
        }
    };

    enum class Phase : std::uint8_t { Free, Latched, Returning };

    struct State
    {
        Phase phase{ Phase::Free };
        Contact contact{};
        RE::NiPoint3 primaryGripLocal{};
        RE::NiPoint3 primaryTargetAtCapture{};
        RE::NiTransform captureWorld{};
        RE::NiTransform lastWorld{};
        RE::NiTransform returnFrom{};
        float returnElapsed{ 0.0f };

        [[nodiscard]] bool latched() const noexcept { return phase == Phase::Latched; }
        [[nodiscard]] bool ownsPose() const noexcept { return phase != Phase::Free; }
    };

    [[nodiscard]] inline bool capture(State& state, const Contact& contact,
        const RE::NiTransform& requestedWeaponWorld, const RE::NiPoint3& primaryGripLocal)
    {
        using namespace dynamic_weapon_collision_policy;
        if (!contact.valid || !isFiniteTransform(contact.weaponWorld) ||
            !isFiniteTransform(contact.surfaceWorld) || !isFinitePoint(contact.weaponPointLocal) ||
            !isFinitePoint(contact.surfacePointLocal) || std::abs(contact.weaponWorld.scale) < 0.0001f ||
            std::abs(contact.surfaceWorld.scale) < 0.0001f ||
            !isFiniteTransform(requestedWeaponWorld) || !isFinitePoint(primaryGripLocal)) return false;
        const auto anchor = transform_math::localPointToWorld(contact.surfaceWorld, contact.surfacePointLocal);
        const auto primary = transform_math::localPointToWorld(requestedWeaponWorld, primaryGripLocal);
        if (!isFinitePoint(anchor) || !isFinitePoint(primary) ||
            weaponSolverLength(weaponSolverSub(primary, anchor)) < kMinimumAimSeparationGameUnits) return false;
        state = {};
        state.phase = Phase::Latched;
        state.contact = contact;
        state.primaryGripLocal = primaryGripLocal;
        state.primaryTargetAtCapture = primary;
        state.captureWorld = contact.weaponWorld;
        // Preserve the measured contact without snapping rotation to the
        // unconstrained hand target on the acquisition frame.
        state.captureWorld.translate = weaponSolverAdd(state.captureWorld.translate,
            weaponSolverSub(anchor, transform_math::localPointToWorld(state.captureWorld, contact.weaponPointLocal)));
        state.lastWorld = state.captureWorld;
        return true;
    }

    [[nodiscard]] inline bool solve(State& state, const RE::NiTransform& requestedWeaponWorld,
        const RE::NiTransform& surfaceWorld, RE::NiTransform& result)
    {
        using namespace dynamic_weapon_collision_policy;
        if (!state.latched() || !isFiniteTransform(requestedWeaponWorld) || !isFiniteTransform(surfaceWorld)) return false;
        const auto anchor = transform_math::localPointToWorld(surfaceWorld, state.contact.surfacePointLocal);
        const auto captureAnchor = transform_math::localPointToWorld(state.contact.surfaceWorld, state.contact.surfacePointLocal);
        const auto primary = transform_math::localPointToWorld(requestedWeaponWorld, state.primaryGripLocal);
        const auto captureAxis = weaponSolverSub(state.primaryTargetAtCapture, captureAnchor);
        const auto targetAxis = weaponSolverSub(primary, anchor);
        if (!isFinitePoint(primary) || weaponSolverLength(targetAxis) < kMinimumAimSeparationGameUnits) return false;

        // Aim from isolated grip intent about the real surface pivot. Native
        // wrist/IK rotation must not become a lever that drags this anchor.
        const auto delta = weaponSolverRotationBetweenStored<RE::NiMatrix3, RE::NiPoint3>(captureAxis, targetAxis);
        result = state.captureWorld;
        result.rotate = weaponSolverApplyWorldRotationToStoredBasis<RE::NiMatrix3, RE::NiPoint3>(delta, state.captureWorld.rotate);
        result.translate = weaponSolverAdd(result.translate,
            weaponSolverSub(anchor, transform_math::localPointToWorld(result, state.contact.weaponPointLocal)));
        if (!isFiniteTransform(result)) return false;
        state.lastWorld = result;
        return true;
    }

    inline void release(State& state) noexcept
    {
        if (!state.ownsPose()) return;
        state.phase = Phase::Returning;
        state.returnFrom = state.lastWorld;
        state.returnElapsed = 0.0f;
    }

    [[nodiscard]] inline RE::NiTransform advanceReturn(State& state,
        const RE::NiTransform& requested, float deltaSeconds)
    {
        if (state.phase != Phase::Returning) return requested;
        state.returnElapsed += std::isfinite(deltaSeconds) ? std::clamp(deltaSeconds, 0.0f, 0.1f) : 0.0f;
        const float alpha = std::clamp(state.returnElapsed / kReturnSeconds, 0.0f, 1.0f);
        const auto result = scope_safe_hand_frame_math::interpolateRebaseTransform(
            state.returnFrom, requested, alpha * alpha * (3.0f - 2.0f * alpha));
        state.lastWorld = result;
        if (alpha >= 1.0f) state = {};
        return result;
    }
}
