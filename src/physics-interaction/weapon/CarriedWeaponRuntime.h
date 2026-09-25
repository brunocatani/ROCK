#pragma once

#include "physics-interaction/weapon/AkimboSessionPolicy.h"
#include "physics-interaction/weapon/NativeWeaponCycle.h"
#include "physics-interaction/weapon/PhysicalWeaponPhysics.h"
#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/NetImmerse/NiSmartPointer.h"
#include <array>

namespace rock
{
    // Frame-thread owner: PhysicsInteraction. The world reference remains the
    // actual item and the Hand keeps its existing model, grip and collision.
    // A separate native equipped-data context owns this weapon's live ammo.
    class PhysicalWeaponSession
    {
    public:
        explicit PhysicalWeaponSession(std::uint32_t slot = 0) noexcept : _slotNumber(slot) { physics.dynamic.setPhysicalSessionSlot(slot); }
        PhysicalWeaponPhysics physics;
        struct Input
        {
            RE::TESObjectREFR* reference{};
            RE::NiAVObject* muzzle{};
            akimbo::Hand hand{akimbo::Hand::None};
            akimbo::Grip grip{akimbo::Grip::None};
            float deltaSeconds{};
            bool inputAllowed{}, triggerHeld{}, reloadPressed{};
        };

        [[nodiscard]] bool activate(const Input& input) { return owns(input.reference) || (!_reference && admit(input)); }
        void prepare(const Input& input);
        void update(const Input& input);
        void clear(bool nativeWorldAvailable) noexcept;
        void shutdown(bool nativeWorldAvailable) noexcept;
        [[nodiscard]] bool suspend() noexcept;
        [[nodiscard]] bool owns(const RE::TESObjectREFR* reference) const noexcept;
        // Capture before the existing exact equipped-to-world transfer.
        // Commit only with the resulting reference from that same transfer.
        [[nodiscard]] bool captureTransfer() noexcept;
        void commitTransfer(RE::ObjectRefHandle reference) noexcept;
        void cancelTransfer() noexcept;
        [[nodiscard]] bool retains(const RE::TESObjectREFR* reference) const noexcept;
        [[nodiscard]] bool preparePrimaryEquip(RE::TESObjectREFR* reference) noexcept;
        void finishPrimaryEquip(RE::ObjectRefHandle source, RE::TESObjectWEAP* weapon,
            std::uintptr_t instance, bool committed) noexcept;
        [[nodiscard]] RE::TESObjectREFR* reference() const noexcept { return _reference.get(); }
        [[nodiscard]] RE::EquippedWeaponData* data() const noexcept { return static_cast<RE::EquippedWeaponData*>(_data.get()); }
        [[nodiscard]] std::uint32_t slotNumber() const noexcept { return _slotNumber; }
        [[nodiscard]] bool presentationReady() const noexcept { return physics.ready() && _cycle.ready(); }
        [[nodiscard]] bool presentationFailed() const noexcept { return _cycle.failed(); }
        [[nodiscard]] bool admissionFailed(RE::TESObjectREFR* reference) const noexcept
        {
            return reference && _declinedReference == reference->GetHandle();
        }
        [[nodiscard]] bool canReturnToNative() const noexcept
        {
            return _ammoKnown && !_faulted && !_operation.reloading() && _operation.cooldown() <= 0.0f && !_cycle.playing();
        }
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
        native_weapon_cycle::Session _cycle{};
        const RE::BGSEquipSlot* _slot{};
        std::uint32_t _slotNumber{}, _index{}, _loaded{}, _ammoForm{}, _thread{};
        std::uint64_t _nextSession{1};
        float _secondsPerShot{}, _reloadSeconds{};
        bool _ammoKnown{}, _automatic{}, _registered{}, _faulted{};

        struct Transfer
        {
            RE::TESForm* form{};
            RE::ObjectRefHandle reference{};
            std::uint32_t loaded{}, ammo{};
            bool valid{}, promoting{};
        } _transfer{};
    };
    class CarriedWeaponRuntime
    {
    public:
        using Input = PhysicalWeaponSession::Input;
        std::array<PhysicalWeaponSession, 2> sessions{PhysicalWeaponSession{0}, PhysicalWeaponSession{1}};
        PhysicalWeaponSession* find(const RE::TESObjectREFR* ref) noexcept
        {
            for (auto& session : sessions) if (session.owns(ref) || session.retains(ref)) return &session;
            return nullptr;
        }
        bool owns(const RE::TESObjectREFR* ref) const noexcept
        {
            for (const auto& session : sessions) if (session.owns(ref)) return true;
            return false;
        }
        bool hasSession() const noexcept
        {
            return sessions[0].hasSession() || sessions[1].hasSession();
        }
        bool activate(const Input& input)
        {
            if (auto* session = find(input.reference)) return session->activate(input);
            for (auto& session : sessions) if (!session.hasSession() && &session != pendingTransfer()) return session.activate(input);
            return false;
        }
        bool captureTransfer() noexcept
        {
            _pending = -1;
            for (unsigned i = 0; i < sessions.size(); ++i) if (!sessions[i].hasSession()) {
                if (!sessions[i].captureTransfer()) return false;
                _pending = static_cast<int>(i); return true;
            }
            return false;
        }
        void commitTransfer(RE::ObjectRefHandle reference) noexcept
        {
            if (auto* session = pendingTransfer()) session->commitTransfer(reference);
        }
        void cancelTransfer() noexcept
        {
            if (auto* session = pendingTransfer()) session->cancelTransfer();
            _pending = -1;
        }
        void transferAdmitted(PhysicalWeaponSession& session) noexcept
        {
            if (&session == pendingTransfer()) _pending = -1;
        }
        bool suspend() noexcept
        {
            for (auto& session : sessions) (void)session.suspend();
            return true;
        }
        bool preparePrimaryEquip(RE::TESObjectREFR* ref) noexcept
        {
            auto* session = find(ref);
            return !session || session->preparePrimaryEquip(ref);
        }
        void finishPrimaryEquip(RE::ObjectRefHandle ref, RE::TESObjectWEAP* weapon, std::uintptr_t instance, bool committed) noexcept
        {
            for (auto& session : sessions) session.finishPrimaryEquip(ref, weapon, instance, committed);
        }
        void shutdown(bool worldAvailable) noexcept
        {
            for (auto& session : sessions) session.shutdown(worldAvailable);
            _pending = -1;
        }
        static bool applyShotOrigin(void* data) noexcept { return PhysicalWeaponSession::applyShotOrigin(data); }
        static void observeShotLaunch(const void* data, std::uint32_t handle) noexcept { PhysicalWeaponSession::observeShotLaunch(data, handle); }
        static bool install() noexcept { return PhysicalWeaponSession::install(); }
        static bool ready() noexcept { return PhysicalWeaponSession::ready(); }
        static void noteInteractionThread() noexcept { PhysicalWeaponSession::noteInteractionThread(); }
        static bool isInteractionThread() noexcept { return PhysicalWeaponSession::isInteractionThread(); }
        static void beforeSave() noexcept { PhysicalWeaponSession::beforeSave(); }
        static void afterSave() noexcept { PhysicalWeaponSession::afterSave(); }
    private:
        PhysicalWeaponSession* pendingTransfer() noexcept { return _pending >= 0 ? &sessions[_pending] : nullptr; }
        int _pending{-1};
    };
}
