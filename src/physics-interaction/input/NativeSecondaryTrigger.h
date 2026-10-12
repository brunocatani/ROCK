#pragma once

namespace rock::native_secondary_trigger
{
    // One physical press remains eligible until native acceptance. It cannot
    // survive release, UI/provider suppression, stale input or item retirement.
    class Intent
    {
        bool _armed{},_held{},_consumed{},_active{};
    public:
        bool observe(bool held,bool allowed) noexcept
        {
            if (!allowed) return cancel();
            _held=held;
            if (held) return false;
            const bool release=_active;
            _armed=true; _consumed=false; _active=false;
            return release;
        }
        bool wantsStart() const noexcept { return _armed && _held && !_consumed; }
        bool armed() const noexcept { return _armed; }
        bool active() const noexcept { return _active; }
        void accepted(bool attackStarted) noexcept { _consumed=true; _active=attackStarted; }
        bool cancel() noexcept
        {
            const bool release=_active;
            _armed=false; _held=false; _consumed=false; _active=false;
            return release;
        }
    };
}
