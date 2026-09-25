#pragma once
#include <memory>

namespace RE { class TESObjectREFR; }

namespace rock::native_weapon_cycle
{
    // Interaction-thread owner. Uses the exact item's first-person subgraph,
    // applying only bones below Weapon to that item's existing scene.
    class Session
    {
    public:
        Session() noexcept;
        ~Session();
        Session(const Session&) = delete;
        Session& operator=(const Session&) = delete;
        void update(RE::TESObjectREFR* reference, float deltaSeconds) noexcept;
        void fire() noexcept;
        void clear() noexcept;
        void reap() noexcept;
        void present() noexcept;
        bool ready() const noexcept;
        bool failed() const noexcept;
    private:
        struct State;
        std::unique_ptr<State> _state;
    };
}
