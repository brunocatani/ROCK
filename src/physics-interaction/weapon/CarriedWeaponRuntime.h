#pragma once

#include "physics-interaction/weapon/AkimboSessionPolicy.h"
#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/NetImmerse/NiSmartPointer.h"

namespace rock
{
    // Frame-thread owner: PhysicsInteraction. The world reference remains the
    // actual item and the Hand keeps its existing model, grip and collision.
    // A separate native equipped-data context owns this weapon's live ammo.
    class CarriedWeaponRuntime
    {
    public:
        struct Input
        {
            RE::TESObjectREFR* reference{};
            RE::NiAVObject* muzzle{};
            akimbo::Hand hand{akimbo::Hand::None};
            akimbo::Grip grip{akimbo::Grip::None};
            float deltaSeconds{};
            bool inputAllowed{}, triggerHeld{}, reloadPressed{};
        };

        void prepare(const Input& input);
        void update(const Input& input);
        void clear(bool nativeWorldAvailable) noexcept;
        void shutdown(bool nativeWorldAvailable) noexcept;
        [[nodiscard]] bool suspend() noexcept;
        [[nodiscard]] bool owns(const RE::TESObjectREFR* reference) const noexcept;
        // Capture before the existing exact equipped-to-world transfer.
        // Commit only with the resulting reference from that same transfer.
        void captureTransfer() noexcept;
        void commitTransfer(RE::ObjectRefHandle reference) noexcept;
        void cancelTransfer() noexcept;
        [[nodiscard]] bool retains(const RE::TESObjectREFR* reference) const noexcept;
        [[nodiscard]] bool preparePrimaryEquip(RE::TESObjectREFR* reference) noexcept;
        void finishPrimaryEquip(RE::ObjectRefHandle source, RE::TESObjectWEAP* weapon,
            std::uintptr_t instance, bool committed) noexcept;
        [[nodiscard]] bool hasSession() const noexcept { return static_cast<bool>(_reference); }
        [[nodiscard]] std::uint64_t sessionId() const noexcept { return _operation.session(); }
        [[nodiscard]] std::uint64_t bindingId() const noexcept { return _operation.binding(); }

        // Called synchronously inside the existing native origin hook. Only
        // this thread's explicit carried-weapon shot may override its origin.
        static bool applyShotOrigin(void* launchData) noexcept;
        static void observeShotLaunch(const void* launchData, std::uint32_t handle) noexcept;
        static bool install() noexcept;
        static bool ready() noexcept;
        static void noteInteractionThread() noexcept;
        static bool isInteractionThread() noexcept;
        static bool ownsNativeContext(const RE::EquippedItem& item) noexcept;
        static void beforeSave() noexcept;
        static void afterSave() noexcept;

    private:
        [[nodiscard]] bool admit(const Input& input);
        [[nodiscard]] bool contextCurrent(RE::EquippedItem& item) const noexcept;
        [[nodiscard]] bool sourceCurrent() const noexcept;
        [[nodiscard]] bool publishContext();
        void removeContext() noexcept;
        void fire();
        void reload();
        void observeAmmo() noexcept;

        RE::NiPointer<RE::TESObjectREFR> _reference{};
        RE::NiPointer<RE::NiAVObject> _muzzle{};
        RE::BGSObjectInstance _weapon{nullptr, nullptr};
        RE::NiPointer<RE::EquippedItemData> _data{};
        RE::ObjectRefHandle _referenceHandle{};
        RE::ObjectRefHandle _declinedReference{};
        akimbo::OperationState _operation{};
        const RE::BGSEquipSlot* _slot{};
        std::uint32_t _index{}, _loaded{}, _ammoForm{}, _thread{};
        std::uint64_t _nextSession{1};
        std::uint64_t _saveEpoch{};
        float _secondsPerShot{}, _reloadSeconds{};
        bool _ammoKnown{}, _automatic{}, _registered{}, _suspended{}, _faulted{};

        struct Transfer
        {
            RE::TESForm* form{};
            RE::ObjectRefHandle reference{};
            std::uint32_t loaded{}, ammo{};
            bool valid{}, promoting{};
        } _transfer{};
    };
}
