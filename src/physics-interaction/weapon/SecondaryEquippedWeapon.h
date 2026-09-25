#pragma once

#include "physics-interaction/weapon/NativeEquippedWeapon.h"
#include "physics-interaction/weapon/TwoHandedGrip.h"
#include "physics-interaction/weapon/DynamicWeaponCollision.h"
#include "physics-interaction/collision/PushContact.h"
#include <utility>

namespace rock
{
    struct PhysicsFrameContext;

    // Owns only equipment index 1's interaction/presentation. The item,
    // magazine and attached assembly remain owned by native equipment.
    class SecondaryEquippedWeapon
    {
    public:
        SecondaryEquippedWeapon();
        WeaponCollision collision;
        DynamicWeaponCollisionRuntime dynamic;
        TwoHandedGrip grip{true};
        push_assist::ContactChannel push;

        struct Transfer {
            std::uint32_t form{};
            std::uintptr_t instance{};
            weapon_grip_transfer::HandGrip firing{};
            weapon_grip_transfer::Pair paired{};
            weapon_grip_transfer::Support support{}, secondSupport{};
            RE::NiTransform presentedHandInPhysical{};
            RE::NiPoint3 sourceModelTranslation{};
            bool isLeft{}, valid{};
        };

        void setPhysicsCallbackGate(PhysicsCallbackQuiescenceGate* gate);
        void adopt(const Transfer& transfer);
        void bridge(RE::NiPointer<RE::NiAVObject> model);
        [[nodiscard]] Transfer captureContinuity() const;
        bool prepare(const PhysicsFrameContext& frame, const EquippedWeaponHandlingSettings& settings);
        TwoHandedGripUpdateResult update(const PhysicsFrameContext& frame, EquippedWeaponGripFrameInput input,
            const EquippedWeaponHandlingSettings& settings);
        void finishPresentation(const PhysicsFrameContext& frame);
        void clear(bool worldAvailable);
        void interrupt(bool worldAvailable);
        [[nodiscard]] bool ready() const noexcept { return _ready; }
        [[nodiscard]] bool pending() const noexcept { return _transfer.valid; }
        [[nodiscard]] bool pendingHandIsLeft() const noexcept { return _transfer.isLeft; }
        bool consumeTransferredRelease() noexcept { return std::exchange(_transferredRelease, false); }
        [[nodiscard]] unsigned heldHandsAtomic() const noexcept { return _heldHands.load(std::memory_order_acquire); }
        [[nodiscard]] RE::NiNode* node() const noexcept { return _snapshot.node.get(); }
        const native_equipped_weapon::Snapshot& snapshot() const noexcept { return _snapshot; }
        equipped_weapon_toggle_grab_policy::RuntimeState toggle;

    private:
        void updateBridge(bool worldAvailable);
        void releaseBridge(bool worldAvailable);
        RE::NiPointer<RE::NiAVObject> _bridgeModel{};
        RE::NiPointer<RE::NiNode> _bridgeParent{};
        Transfer _bridgeGrip{};
        PhysicsCallbackQuiescenceGate* _gate{};
        RE::hknpWorld* _world{};
        RE::bhkWorld* _bhk{};
        native_equipped_weapon::Snapshot _snapshot{};
        Transfer _transfer{};
        equipped_weapon_toggle_grab_policy::TransferReleaseState _pendingRelease{};
        std::array<weapon_interaction_acquisition_policy::State, 2> _contactAcquisition{};
        std::atomic<unsigned> _heldHands{};
        bool _ready{}, _transferredRelease{};
    };
}
