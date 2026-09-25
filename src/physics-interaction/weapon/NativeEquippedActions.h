#pragma once
#include "physics-interaction/weapon/NativeEquippedWeapon.h"
#include "physics-interaction/weapon/NativeWeaponCycle.h"
#include "physics-interaction/weapon/AkimboSessionPolicy.h"

namespace rock::native_equipped_actions
{
    struct Input {
        akimbo::Hand hand{akimbo::Hand::None};
        akimbo::Grip grip{akimbo::Grip::None};
        float deltaSeconds{};
        bool allowed{}, triggerHeld{}, reloadPressed{};
        std::uint64_t content{};
    };
    struct Events {
        bool fired{}, reloadStarted{}, reloadFinished{};
        std::uint32_t before{}, after{};
    };

    bool install() noexcept;
    bool ready() noexcept;
    bool supports(RE::TESObjectWEAP* weapon, RE::TBO_InstanceData* instance) noexcept;
    bool applyShotOrigin(void* launchData) noexcept;
    void observeShotLaunch(const void* launchData, std::uint32_t handle) noexcept;
    void beforeSave() noexcept;
    void afterSave() noexcept;

    class Session
    {
    public:
        // Preparation never creates equipment or a private data object.
        bool prepare(const native_equipped_weapon::Snapshot& item, std::uint64_t content, float seconds);
        Events update(const native_equipped_weapon::Snapshot& item, const Input& input);
        void clear(bool nativeWorldAvailable);
        void suspend();
        bool presentationReady() const noexcept { return _cycle.ready() && _muzzle; }
        bool presentationFailed() const noexcept { return _cycle.failed(); }
        std::uint64_t sessionId() const noexcept { return _operation.session(); }
        std::uint64_t bindingId() const noexcept { return _operation.binding(); }
        void bind(akimbo::Hand hand, akimbo::Grip grip);
        bool quiescent() const noexcept { return !_attackActive && !_operation.reloading() &&
            _operation.cooldown() == 0 && !_cycle.playing(); }
    private:
        bool fire(const native_equipped_weapon::Snapshot& item);
        void stopAttack();
        native_equipped_weapon::Identity _identity{};
        RE::NiPointer<RE::NiAVObject> _muzzle{};
        RE::NiPointer<RE::NiAVObject> _previousMuzzle{};
        native_weapon_cycle::Session _cycle{};
        akimbo::OperationState _operation{};
        float _shotSeconds{}, _reloadSeconds{};
        bool _automatic{}, _attackActive{};
    };
}
