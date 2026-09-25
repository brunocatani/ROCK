#pragma once
#include <cstdint>
#include <memory>

namespace RE { class TESObjectREFR; class TESObjectWEAP; class TBO_InstanceData; class NiAVObject; }

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
        void updateEquipped(RE::TESObjectWEAP* weapon, RE::TBO_InstanceData* instance,
            RE::NiAVObject* root, std::uint64_t content, float deltaSeconds) noexcept;
        void fire(float shotSeconds) noexcept;
        void clear() noexcept;
        void reap() noexcept;
        bool ready() const noexcept;
        bool failed() const noexcept;
        bool playing() const noexcept;
    private:
        struct State;
        std::unique_ptr<State> _state;
    };
}
