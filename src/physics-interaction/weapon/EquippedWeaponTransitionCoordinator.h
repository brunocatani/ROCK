#pragma once

#include "physics-interaction/weapon/EquipVisualBridge.h"
#include "physics-interaction/weapon/EquippedWeaponTransitionPolicy.h"

#include "physics-interaction/weapon/HeldWeaponTransferPolicy.h"
#include "physics-interaction/weapon/grip/LeftCarryReadiness.h"
#include "physics-interaction/weapon/EquippedWeaponToggleGrabPolicy.h"
#include "physics-interaction/weapon/WeaponSupport.h"

#include <chrono>
#include <cstdint>

namespace rock
{
    class EquippedWeaponTransitionCoordinator
    {
    public:
        struct PendingGrip
        {
            weapon_grip_transfer::Pair pairedGrips{};
            weapon_grip_transfer::Support supportGrip{};
            weapon_grip_transfer::Support secondSupportGrip{};
            bool menuResume{ false };
            std::array<equipped_weapon_toggle_grab_policy::TransferReleaseState, 2> pairedRelease{};
            bool pending{ false };
            // Originating carrier; a support-only transfer leaves the opposite firing station vacant.
            bool isLeft{ false };
            // Zero means "the current weapon" (menu reconciliation). Held
            // equip requests bind these fields to the accepted target and its
            // pre-request baseline so a cloned instance may be recognized
            // without ever starting manual ownership on an old same-base gun.
            std::uint32_t targetWeaponFormID{ 0 };
            std::uintptr_t targetWeaponInstanceData{ 0 };
            std::uint32_t previousWeaponFormID{ 0 };
            std::uintptr_t previousWeaponInstanceData{ 0 };
            float remainingSeconds{ 0.0f };
            equipped_weapon_manual_ownership_policy::PrimaryOnlyStartSource source{
                equipped_weapon_manual_ownership_policy::PrimaryOnlyStartSource::GripInput
            };
            // A toggle acquisition is a committed logical grab even after the
            // physical button opens while left takeover waits for the final
            // generation-bound authored-support verdict.
            bool toggleAcquisitionCommitted{ false };
            left_carry_readiness::TakeoverWitness takeoverWitness{};
            const char* lastStartFailureReason{ nullptr }; // Static diagnostic reason; never an engine pointer.
            bool hasFiringHandWeaponLocal{ false };
            RE::NiTransform firingHandWeaponLocal{};
            bool hasFiringGripWeaponLocal{ false };
            RE::NiPoint3 firingGripWeaponLocal{};
        };

        enum class Source : std::uint8_t
        {
            ObservedEquip,
            HeldTriggerEquip,
            HeldGripZoneEquip,
            MenuExit,
            WorkbenchExit,
        };

        enum class TerminalResult : std::uint8_t
        {
            None,
            Completed,
            WeaponUnequipped,
            IdentityLost,
            ExpectedIdentityTimeout,
            NativeAnimationHandoff,
            WeaponNoLongerDrawn,
            IntentionalShoulderSheathe,
            RecoveryExhausted,
            ProviderLost,
            Shutdown,
        };

        struct PublicSnapshot
        {
            std::uint64_t transitionSequence{ 0 };
            std::uint64_t terminalSequence{ 0 };
            std::uint32_t weaponFormID{ 0 };
            Source source{ Source::ObservedEquip };
            TerminalResult terminalResult{ TerminalResult::None };
            bool active{ false };
            bool identityPending{ false };
            bool drawPending{ false };
            bool bridgePresented{ false };
            bool nativeRenderable{ false };
            bool handPoseHandoffComplete{ false };
            bool recoveryExhausted{ false };
            std::uint32_t terminalWeaponFormID{ 0 };
            Source terminalSource{ Source::ObservedEquip };
            std::uint32_t presentationWeaponFormID{ 0 };
            bool presentationKnown{ false };
        };

        struct FrameInput
        {
            float deltaSeconds{ 0.0f };
            bool visualAuthorityAvailable{ false };
            bool localSkeletonReady{ false };
            bool menuBlocking{ false };
            bool compatibilityBlocking{ false };
            std::uint32_t nativeWeaponState{ 0 };
            // Exact identity lease for ROCK's physical shoulder sheath. It
            // exempts only the instance ROCK deliberately transitioned; it is
            // never a general permission for equipped weapons to stay hidden.
            bool intentionalShoulderSheathActive{ false };
            std::uint32_t shoulderSheathFormID{ 0 };
            std::uintptr_t shoulderSheathInstanceData{ 0 };
            std::uint32_t shoulderSheathEquipIndex{ 0 };
            // Native reload/bolt animation owns the weapon presentation while
            // this is set. Equip recovery must neither unhide nor reattach the
            // same graph during that authority window.
            bool nativeWeaponAnimationActive{ false };
            // Solved LEFT-carry weapon world from the latest grip update. The
            // left-hand bridge uses it as its rotation carrier; a live weapon
            // root read at this frame phase would return the right-hand glue
            // or draw-animation orientation instead of the rendered carry.
            bool leftCarrySolvedWeaponWorldValid{ false };
            RE::NiTransform leftCarrySolvedWeaponWorld{};
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

        [[nodiscard]] PendingGrip& pendingGrip() noexcept { return _pendingGrip; }
        [[nodiscard]] const PendingGrip& pendingGrip() const noexcept { return _pendingGrip; }
        [[nodiscard]] const held_weapon_transfer::State& heldTransfer() const noexcept { return _heldTransfer; }
        bool beginHeldRequest(const held_weapon_transfer::Request& request);
        void cancelHeldRequest(const char* reason, held_weapon_transfer::Outcome outcome = held_weapon_transfer::Outcome::Failed);
        void recordOutgoingRemoval(std::uint32_t reference);
        void recordOutgoingResult(std::uint64_t sequence, bool succeeded);
        void recordInventoryCommit(std::uint32_t form, std::uintptr_t instance, bool accepted, std::uintptr_t observedInstance);
        void recordGripAcquired(std::uint32_t form, std::uintptr_t instance, bool left, held_weapon_transfer::Role role);
        void recordGripPresentation();
        void validateHeldSource(bool held, std::uint32_t reference, std::uint64_t grab,
            std::uint32_t world, std::uint32_t skeleton);
        void suspendMenuGrip(const PendingGrip& grip, std::uint32_t world, std::uint32_t skeleton);
        void resumeMenuGrip(std::uint32_t world, std::uint32_t skeleton);
        [[nodiscard]] bool observedItemMatches(std::uint32_t form, std::uintptr_t instance) const noexcept
        {
            return _observationInitialized && held_weapon_transfer::sameMenuItem(_observedIdentity.formID,
                _observedIdentity.instanceData, form, instance);
        }

        bool beginHeldTransition(
            const ExpectedIdentity& expected,
            Source source,
            const EquipVisualBridge::BeginInput& bridgeInput);
        void requestCurrentWeaponReconcile(Source source) noexcept;
        void update(const FrameInput& input);
        void traceVisualPresentation(const char* phase) const { _bridge.tracePresentation(phase); }
        void shutdown();
        void abandonSceneGraph();
        [[nodiscard]] PublicSnapshot getPublicSnapshot() const noexcept;

        [[nodiscard]] bool isHandPoseHandoffActive() const noexcept
        {
            return _bridge.isHandPoseHandoffActive();
        }
        [[nodiscard]] bool hasPairedHandPoseHandoff() const noexcept { return _bridge.hasPairedHandPoseHandoff(); }
        [[nodiscard]] bool hasSupportOnlyHandPoseHandoff() const noexcept { return _bridge.hasSupportOnlyHandPoseHandoff(); }
        [[nodiscard]] bool hasCapturedHandPoseHandoff() const noexcept { return _bridge.hasCapturedHandPoseHandoff(); }
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
        void tryCompleteHandPoseHandoff() { _bridge.tryCompleteHandPoseHandoff(); }
        void synchronizeEquippedPresentation(RE::NiNode* weaponNode, std::uint32_t formID)
        {
            if (formID == _bridge.weaponBaseFormID()) _bridge.synchronizeEquippedPresentation(weaponNode);
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
        void resetDrawRecoveryClock(bool armed) noexcept;
        [[nodiscard]] float sampleDrawRecoveryWallDelta() noexcept;
        void finish(
            TerminalResult result,
            const char* reason,
            bool releaseSceneGraph);

        void traceHeldTransfer(const char* event) const;
        void updateHeldRecovery();
        void releasePendingNativeCull(bool restore);
        held_weapon_transfer::State _heldTransfer{};
        PendingGrip _pendingGrip{};
        PendingGrip _menuGrip{};
        bool _menuGripSaved{ false };
        std::uint32_t _menuWorld{ 0 };
        std::uint32_t _menuSkeleton{ 0 };
        RE::NiPointer<RE::NiAVObject> _pendingNativeCull{};
        float _heldWaitSeconds{ 0.0f };
        bool _heldBridgeStarted{ false };
        std::uint8_t _heldRecoveryAttempts{ 0 };
        bool _heldRecoveryHolsterRequested{ false };
        const char* _heldFailureReason{ nullptr };
        EquipVisualBridge _bridge;
        equipped_weapon_visual_state::ObservationCache _visualCache;
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
        /*
         * Wall-clock recovery clock by contract: draw and presentation
         * deadlines must expire even when the game clock stalls because the
         * native work they supervise can continue independently. This is
         * deliberately NOT gameplay time.
         */
        float _drawRecoveryElapsedSeconds{ 0.0f };
        std::chrono::steady_clock::time_point _drawRecoveryLastUpdateAt{};
        bool _observationInitialized{ false };
        bool _active{ false };
        bool _drawRecoveryClockArmed{ false };
        bool _waitingForExpectedIdentity{ false };
        bool _requestCurrentPending{ false };
        bool _wasMenuBlocking{ false };
        bool _menuEntryCaptured{ false };
        bool _lateRecoveryWindowGranted{ false };
        bool _drawExhaustionLogged{ false };
        bool _repairExhaustionLogged{ false };
        std::uint64_t _transitionSequence{ 0 };
        std::uint64_t _terminalSequence{ 0 };
        std::uint32_t _lastTerminalWeaponFormID{ 0 };
        Source _lastTerminalSource{ Source::ObservedEquip };
        TerminalResult _lastTerminalResult{ TerminalResult::None };
        std::uint32_t _presentationWeaponFormID{ 0 };
        bool _presentationKnown{ false };
        bool _nativeRenderable{ false };
    };
}
