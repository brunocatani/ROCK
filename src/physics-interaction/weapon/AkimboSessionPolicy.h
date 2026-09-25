#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

namespace rock::akimbo
{
    enum class Hand : std::uint8_t { None, Right, Left };
    enum class Grip : std::uint8_t { None, Support, Firing };
    enum class TransferCapture : std::uint8_t { NotApplicable, Unavailable, Ready };

    constexpr TransferCapture transferCapture(bool itemReadable, bool firearm, bool hasAmmoDefinition,
        bool nativeMagazineReadable, bool supportedTiming) noexcept
    {
        if (!itemReadable) return TransferCapture::Unavailable;
        if (!firearm || !hasAmmoDefinition) return TransferCapture::NotApplicable;
        return nativeMagazineReadable && supportedTiming ? TransferCapture::Ready : TransferCapture::Unavailable;
    }

    constexpr bool keepFiringSound(bool active, Grip grip, bool inputAllowed, bool triggerHeld,
        bool ammoKnown, std::uint32_t loaded, bool reloading) noexcept
    {
        return active && grip == Grip::Firing && inputAllowed && triggerHeld && ammoKnown && loaded && !reloading;
    }

    inline constexpr std::uint32_t kArchiveVersion = 2;
    constexpr std::uint32_t archiveFlags(bool reloading, bool active) noexcept
    {
        return (reloading ? 1u : 0u) | (active ? 2u : 0u);
    }
    constexpr std::optional<std::uint32_t> restoreArchiveFlags(std::uint32_t version, std::uint32_t flags) noexcept
    {
        if ((version != 1 && version != kArchiveVersion) || (flags & ~(version == 1 ? 1u : 3u))) return std::nullopt;
        return version == 1 ? flags | 2u : flags;
    }

    // FO4 inventory ammo includes loaded rounds. The other weapon's loaded
    // rounds cannot be allocated again by a reload of this magazine.
    constexpr std::uint32_t availableForMagazine(std::uint32_t inventoryTotal, std::uint32_t otherLoaded) noexcept
    {
        return inventoryTotal > otherLoaded ? inventoryTotal - otherLoaded : 0;
    }

    constexpr std::uint32_t clampReload(std::uint32_t requested, std::uint32_t inventoryTotal,
        std::uint32_t otherLoaded) noexcept
    {
        return (std::min)(requested, availableForMagazine(inventoryTotal, otherLoaded));
    }

    // One instance per weapon session. A hand is only an input binding. Loaded
    // ammunition is supplied by the native context, never decremented here.
    class OperationState
    {
    public:
        struct Ticket
        {
            std::uint64_t session{}, binding{}, operation{};
            bool operator==(const Ticket&) const = default;
        };

        void begin(std::uint64_t session) noexcept
        {
            *this = {};
            _session = session;
        }

        void bind(Hand hand, Grip grip) noexcept
        {
            if (_hand == hand && _grip == grip) return;
            _hand = hand;
            _grip = grip;
            ++_binding;
            _pending = {};
            _armed = false;
            _wasHeld = false;
            // Hand changes do not reset weapon cooldown or reload progress.
        }

        void advance(float seconds) noexcept
        {
            if (!std::isfinite(seconds) || seconds < 0.0f) return;
            _cooldown = (std::max)(0.0f, _cooldown - seconds);
            if (_reloading) _reloadRemaining = (std::max)(0.0f, _reloadRemaining - seconds);
        }

        void cancelInput() noexcept { _armed = false; _wasHeld = false; }

        [[nodiscard]] Ticket requestFire(bool inputAllowed, bool held, bool automatic,
            bool ammoKnown, std::uint32_t loaded) noexcept
        {
            if (!inputAllowed || _hand == Hand::None || _grip != Grip::Firing) {
                cancelInput();
                return {};
            }
            if (!held) _armed = true;
            const bool press = held && !_wasHeld;
            _wasHeld = held;
            if (!_session || !_armed || !held || (!automatic && !press) ||
                !ammoKnown || !loaded || _reloading || _cooldown > 0.0f || _pending.operation) return {};
            _pending = {_session, _binding, ++_operation};
            return _pending;
        }

        [[nodiscard]] bool current(Ticket ticket) const noexcept
        {
            return ticket.operation && ticket == _pending &&
                ticket.session == _session && ticket.binding == _binding &&
                _hand != Hand::None && _grip == Grip::Firing;
        }

        void completeFire(Ticket ticket, float secondsPerShot) noexcept
        {
            if (ticket != _pending) return;
            // Even a declined native dispatch cannot create a same-frame
            // retry loop. A new operation must observe a fresh input update.
            _pending = {};
            if (std::isfinite(secondsPerShot) && secondsPerShot > 0.0f)
                _cooldown = (std::max)(_cooldown, secondsPerShot);
        }

        [[nodiscard]] bool beginReload(float seconds) noexcept
        {
            if (!_session || _hand == Hand::None || _grip != Grip::Firing || _reloading ||
                _pending.operation || !std::isfinite(seconds) || seconds <= 0.0f) return false;
            _reloading = true;
            _reloadRemaining = seconds;
            cancelInput();
            return true;
        }

        [[nodiscard]] bool reloadDue() const noexcept { return _reloading && _reloadRemaining <= 0.0f; }
        void completeReload() noexcept { _reloading = false; _reloadRemaining = 0.0f; }
        [[nodiscard]] bool reloading() const noexcept { return _reloading; }
        [[nodiscard]] std::uint64_t session() const noexcept { return _session; }
        [[nodiscard]] std::uint64_t binding() const noexcept { return _binding; }
        [[nodiscard]] Hand hand() const noexcept { return _hand; }
        [[nodiscard]] float cooldown() const noexcept { return _cooldown; }
        [[nodiscard]] float reloadRemaining() const noexcept { return _reloadRemaining; }
        void restore(float cooldown, float reloadRemaining, bool reloading) noexcept
        {
            if (!std::isfinite(cooldown) || cooldown < 0.0f || !std::isfinite(reloadRemaining) || reloadRemaining < 0.0f) return;
            _cooldown = cooldown;
            _reloadRemaining = reloadRemaining;
            _reloading = reloading;
            cancelInput();
        }

    private:
        std::uint64_t _session{}, _binding{}, _operation{};
        Ticket _pending{};
        Hand _hand{Hand::None};
        Grip _grip{Grip::None};
        float _cooldown{}, _reloadRemaining{};
        bool _armed{}, _wasHeld{}, _reloading{};
    };
}
