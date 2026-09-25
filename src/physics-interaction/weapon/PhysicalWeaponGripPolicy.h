#pragma once

#include "physics-interaction/weapon/EquippedWeaponToggleGrabPolicy.h"
#include "physics-interaction/weapon/EquippedWeaponDropPolicy.h"
#include "physics-interaction/input/TransferredWeaponGrabPolicy.h"

namespace rock::physical_weapon_grip_policy
{
    using Button = equipped_weapon_toggle_grab_policy::ButtonState;
    using GrabMode = equipped_weapon_toggle_grab_policy::Mode;
    using DropMode = equipped_weapon_drop_policy::Mode;
    enum class Release { KeepAttached, DetachHand, RetainLoose, Drop };

    constexpr Release releaseAction(bool canDetach, bool peerCarries, DropMode mode) noexcept
    {
        if (!canDetach) return Release::KeepAttached;
        if (peerCarries) return Release::DetachHand;
        switch (mode) {
        case DropMode::ToggleDrop: return Release::RetainLoose;
        case DropMode::AutoDrop: return Release::Drop;
        default: return Release::KeepAttached;
        }
    }

    // Release ownership is per physical grip, scoped to its weapon session.
    // Both entry paths use the same configured weapon policy. A Toggle Drop
    // becomes an inactive retained object until a trigger reactivates it or a
    // complete subsequent grab click releases it.
    class Grip
    {
    public:
        bool observe(std::uint64_t grab, bool active, GrabMode mode, bool firing,
            Button physical, bool releaseAllowed, bool transferred) noexcept
        {
            const bool toggle = equipped_weapon_toggle_grab_policy::usesToggleForRole(mode, firing);
            if (_grab != grab || (active && _toggle != toggle)) {
                *this = {};
                _grab = grab;
                _toggle = toggle;
                _retainUntilFirstHold = transferred;
                // This edge acquired/changed the grip, not released it.
                physical.pressed = false;
            }
            if (!active) {
                if (!_retained) { _release = releaseAllowed && physical.released; return _release; }
                _release = transferred_weapon_grab_policy::advance(_loose,
                    physical.held, physical.pressed, physical.released, releaseAllowed);
                return _release;
            }
            if (!releaseAllowed) {
                _weapon = {};
                _waitingForHold = !toggle;
                physical.pressed = false;
                physical.released = false;
            }
            if (_waitingForHold) {
                _release = false;
                if (!physical.held) return false;
                _waitingForHold = false;
                physical.pressed = false;
            }
            _weapon.observe(mode, firing, physical, _retainUntilFirstHold);
            _release = releaseAllowed && _weapon.releaseRequested;
            return _release;
        }
        void retainLoose(std::uint64_t grab) noexcept
        {
            *this = {};
            _grab = grab;
            _retained = true;
        }
        void keepAttached() noexcept { _weapon = {}; _release = false; _waitingForHold = true; }
        void suspend() noexcept
        {
            _weapon = {};
            _release = false;
            _waitingForHold = !_toggle;
            _loose = transferred_weapon_grab_policy::State::Held;
        }
        bool releaseRequested() const noexcept { return _release; }
        std::uint64_t grab() const noexcept { return _grab; }
    private:
        std::uint64_t _grab{};
        bool _toggle{}, _retained{}, _release{}, _retainUntilFirstHold{}, _waitingForHold{};
        equipped_weapon_toggle_grab_policy::TransferReleaseState _weapon{};
        transferred_weapon_grab_policy::State _loose{transferred_weapon_grab_policy::State::AwaitInitialRelease};
    };
}
