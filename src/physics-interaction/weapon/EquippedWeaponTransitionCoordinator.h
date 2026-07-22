#pragma once

#include "physics-interaction/weapon/EquipVisualBridge.h"
#include "physics-interaction/weapon/EquippedWeaponTransitionPolicy.h"

#include <cstdint>

namespace rock
{
    class EquippedWeaponTransitionCoordinator
    {
    public:
        enum class Source : std::uint8_t
        {
            ObservedEquip,
            HeldTriggerEquip,
            HeldGripZoneEquip,
            MenuExit,
            WorkbenchExit,
        };

        struct FrameInput
        {
            float deltaSeconds{ 0.0f };
            bool visualAuthorityAvailable{ false };
            bool localSkeletonReady{ false };
            bool menuBlocking{ false };
            bool compatibilityBlocking{ false };
            std::uint32_t nativeWeaponState{ 0 };
            // Native reload/bolt animation owns the weapon presentation while
            // this is set. Equip recovery must neither unhide nor reattach the
            // same graph during that authority window.
            bool nativeWeaponAnimationActive{ false };
        };

        struct ExpectedIdentity
        {
            std::uint32_t formID{ 0 };
            std::uintptr_t instanceData{ 0 };
            std::uint32_t previousFormID{ 0 };
            std::uintptr_t previousInstanceData{ 0 };
            // Address-only scene witness sampled before the native request.
            // Never dereferenced; it distinguishes an old same-base visual
            // from the replacement instance Bethesda has not built yet.
            std::uintptr_t previousNativeInstanceNode{ 0 };
        };

        bool beginHeldTransition(
            const ExpectedIdentity& expected,
            Source source,
            const EquipVisualBridge::BeginInput& bridgeInput);
        void requestCurrentWeaponReconcile(Source source) noexcept;
        void update(const FrameInput& input);
        void shutdown();
        void abandonSceneGraph();

        [[nodiscard]] bool isHandPoseHandoffActive() const noexcept
        {
            return _bridge.isHandPoseHandoffActive();
        }
        [[nodiscard]] bool handPoseHandoffIsLeft() const noexcept
        {
            return _bridge.handPoseHandoffIsLeft();
        }
        [[nodiscard]] std::uint32_t bridgeWeaponBaseFormID() const noexcept
        {
            return _bridge.weaponBaseFormID();
        }
        void completeHandPoseHandoff(const char* reason)
        {
            _bridge.completeHandPoseHandoff(reason);
        }

    private:
        struct Identity
        {
            std::uint32_t formID{ 0 };
            std::uintptr_t instanceData{ 0 };
            std::uint32_t equipIndex{ 0 };

            [[nodiscard]] bool valid() const noexcept { return formID != 0; }
            [[nodiscard]] bool operator==(const Identity&) const noexcept = default;
        };

        [[nodiscard]] static Identity readCurrentIdentity() noexcept;
        [[nodiscard]] bool expectedMatches(const Identity& identity) const noexcept;
        void bindCurrentIdentity(
            const Identity& identity,
            Source source,
            const char* reason,
            Identity previousIdentity = {},
            std::uintptr_t previousNativeInstanceNode = 0);
        void finish(const char* reason, bool releaseSceneGraph);

        EquipVisualBridge _bridge;
        equipped_weapon_transition_policy::State _policyState{};
        Identity _observedIdentity{};
        Identity _boundIdentity{};
        Identity _menuEntryIdentity{};
        ExpectedIdentity _expectedIdentity{};
        std::uintptr_t _menuEntryNativeInstanceNode{ 0 };
        std::uintptr_t _supersededNativeInstanceNode{ 0 };
        Source _source{ Source::ObservedEquip };
        Source _requestedCurrentSource{ Source::MenuExit };
        float _activeSeconds{ 0.0f };
        bool _observationInitialized{ false };
        bool _active{ false };
        bool _waitingForExpectedIdentity{ false };
        bool _requestCurrentPending{ false };
        bool _wasMenuBlocking{ false };
        bool _menuEntryCaptured{ false };
        bool _lateRecoveryWindowGranted{ false };
        bool _drawExhaustionLogged{ false };
        bool _repairExhaustionLogged{ false };
    };
}
