#pragma once

#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiSmartPointer.h"

#include "physics-interaction/weapon/EquippedWeaponVisualState.h"

#include <array>
#include <chrono>
#include <cstdint>

namespace RE
{
    class TESObjectWEAP;
}

namespace rock
{
    /*
     * Visual-only bridge that covers the loose->equipped weapon gap.
     *
     * Engine facts this design rests on (Ghidra map, see
     * docs/research/2026-07-04-loose-to-equipped-weapon-visual-gap.md):
     * the pickup message handler detaches the loose ref's 3D from the scene
     * graph synchronously inside ActivateRef (DetachHavok + Set3D(nullptr)),
     * but that teardown is a refcount release, not a recursive destroy. A
     * NiPointer taken before ActivateRef therefore owns the fully assembled,
     * render-ready world model. The equipped replacement only appears later,
     * when the draw animation's WeaponAttach event builds the biped slot 3D
     * (node named "Weapon %s (%08X)").
     *
     * The bridge re-attaches that orphaned model under the world root, glues
     * it to the equipping hand's wand transform and blends it toward the pose
     * the weapon will actually stabilize at. Once the exact native instance
     * is stable, model ownership is released terminally. Native attach repair
     * remains available after handoff, but it can never re-present the loose
     * model during a later sheath, unequip, drop, throw, or animation. Every
     * exit restores any native child cull and deterministically releases both
     * scene references and pose authority.
     *
     * Blend target. begin() re-runs the shared loose-grip identity resolver
     * to recover the authored firing point and hand-pose payload. The retained
     * loose model already carries the separate position-only placement hold;
     * once the exact native Weapon frame exists, update() uses that frame as
     * the rotational carrier and translates only the authored firing point.
     * Custom JSON remains first authority, followed by ROCK-authored data and
     * embedded hFRIK fallback. A first-ever weapon never borrows another
     * weapon's cached position-only hold.
     *
     * Lifetime/threading: main-thread only, driven by PhysicsInteraction's
     * per-frame update. The bridge owns exactly one NiPointer; the scene
     * graph holds a second reference while attached. No Havok, no form
     * state, nothing serialized.
     */
    class EquipVisualBridge
    {
    public:
        struct BeginInput
        {
            // Loose weapon 3D captured before ActivateRef (engine has already
            // detached it from the scene by the time begin() runs).
            RE::NiPointer<RE::NiAVObject> worldModel;
            // Base TESObjectWEAP form ID used by the equipped instance-node
            // name. This is deliberately not the temporary loose reference ID.
            std::uint32_t weaponFormID = 0;
            bool isLeftHand = false;
            // Weapon base form for the shared loose-grip authority resolver;
            // the captured worldModel supplies the matching stock variant.
            RE::TESObjectWEAP* weapon = nullptr;
            // Canonical firing-hand hold for offhand carries (hand transform
            // in weapon-root-local space, valid against the root-flattened
            // hand frame). ROCK-authored placement derives only the grip point
            // from this transform and preserves the native weapon rotation;
            // the complete relation remains necessary for hand presentation.
            // Same capture the pending primary-only grip start consumes.
            bool hasFiringHandWeaponLocal = false;
            RE::NiTransform firingHandWeaponLocal{};
            float timeoutSeconds = 1.0f;
            float blendSeconds = 0.15f;
        };

        struct UpdateInput
        {
            float deltaSeconds = 0.0f;
            bool advanceLifetime = true;
            bool presentModel = true;
            const equipped_weapon_visual_state::Snapshot* nativeVisual = nullptr;
            /*
             * Solved LEFT-carry weapon world from the latest grip update - the
             * pose the weapon node is actually rendered at. A left-hand
             * bridge must use THIS as its rotation carrier, never the live
             * weapon root: the bridge updates before ROCK's carry re-poses
             * the node each frame, so an early-frame root read returns the
             * right-hand glue or draw-animation orientation (~180 degrees
             * from the mirrored left carry).
             */
            bool leftCarrySolvedWeaponWorldValid = false;
            RE::NiTransform leftCarrySolvedWeaponWorld{};
        };

        EquipVisualBridge() = default;
        ~EquipVisualBridge();

        EquipVisualBridge(const EquipVisualBridge&) = delete;
        EquipVisualBridge& operator=(const EquipVisualBridge&) = delete;
        EquipVisualBridge(EquipVisualBridge&&) = delete;
        EquipVisualBridge& operator=(EquipVisualBridge&&) = delete;

        // Attaches the model and captures the hand-space pose. Returns false
        // (and stays inactive) when any required node is missing; failure
        // simply means the transition looks like it does today.
        bool begin(const BeginInput& input);

        // Per-frame pose glue and exact-native-instance presentation handoff.
        // presentModel=false is a terminal visual handoff: it releases the
        // loose model immediately while allowing the independent temporary
        // hand-pose payload to survive until its equipped owner acquires it.
        void update(const UpdateInput& input);

        // Advances only the hard presentation lease while the coordinator is
        // mutation-blocked by a menu, compatibility owner, or unavailable
        // visual authority. This may release the bridge but performs no pose
        // or native-visibility repair.
        void advancePresentationLease(float deltaSeconds);

        // Ends the bridge and releases any remaining visual/pose ownership.
        void release(const char* reason);

        // Detach from the (still valid) scene graph and release.
        void shutdown();

        // Scene/world already gone: release our reference without touching
        // possibly-stale parent pointers.
        void abandonSceneGraph();

        [[nodiscard]] bool isActive() const noexcept { return _active; }
        [[nodiscard]] bool hasVisualModel() const noexcept { return _model != nullptr; }
        [[nodiscard]] bool isModelPresented() const noexcept { return _modelPresented; }
        [[nodiscard]] bool ownsNativeInstanceCull(const RE::NiAVObject* node) const noexcept;
        [[nodiscard]] bool isHandPoseHandoffActive() const noexcept { return _handPoseHandoffActive; }
        [[nodiscard]] bool handPoseHandoffIsLeft() const noexcept { return _isLeftHand; }
        [[nodiscard]] std::uint32_t weaponBaseFormID() const noexcept { return _weaponFormID; }

        // Called only after the equipped exact-pose publisher has positively
        // acquired the same physical hand. The lower-priority bridge pose is
        // then removed without disturbing the equipped publisher's tag.
        void completeHandPoseHandoff(const char* reason);

    private:
        // Attach the (already orphaned) model under the world root; false when
        // the model is still parented or the world root is unavailable.
        bool tryAttachToWorldRoot();
        bool publishHandPoseHandoff();
        void synchronizeNativeInstanceCull(
            const equipped_weapon_visual_state::Snapshot* nativeVisual,
            bool bridgePresented);
        void restoreNativeInstanceCull();
        void clearModel(const char* reason, bool detachFromParent);
        void clearHandPoseHandoff(const char* reason, bool logCompletion, bool discardPayload);
        void clear(const char* reason, bool detachFromParent, bool restoreNativeCull = true);
        [[nodiscard]] bool advancePresentationLeaseImpl(
            float deltaSeconds,
            bool presentedForLogging);

        RE::NiPointer<RE::NiAVObject> _model;
        RE::NiPointer<RE::NiAVObject> _culledNativeInstance;
        // Non-owning; validated each frame against _model->parent before use.
        RE::NiNode* _parent = nullptr;
        RE::NiTransform _modelInHandLocal{};
        RE::NiTransform _physicalHandInWandLocal{};
        // Canonical loose-to-equipped hold (see BeginInput). Re-resolved at
        // begin() against the live filewatch-published hFRIK cache so the
        // bridge obeys the same priority as pull seating and grip-zone equip.
        RE::NiTransform _firingHandWeaponLocal{};
        bool _hasFiringHandWeaponLocal = false;
        bool _usesAuthoredControllerAim = false;
        bool _hasPhysicalHandInWandLocal = false;
        std::array<RE::NiTransform, 15> _handoffFingerLocalTransforms{};
        std::uint16_t _handoffFingerLocalTransformMask = 0;
        float _elapsedSeconds = 0.0f;
        float _lifetimeSeconds = 0.0f;
        float _blendSeconds = 0.15f;
        /*
         * Wall-clock safety lease by contract: the bridge presentation must
         * expire even if game-frame updates stall, so a loose bridge model
         * can never persist indefinitely. The blend itself advances on the
         * game clock; only the lease uses wall time.
         */
        float _presentationLeaseSeconds = 1.0f;
        std::chrono::steady_clock::time_point _presentationLeaseStartedAt{};
        std::uint32_t _weaponFormID = 0;
        bool _isLeftHand = false;
        bool _modelPresented = false;
        bool _culledNativeInstanceWasVisible = false;
        bool _handPosePayloadAvailable = false;
        bool _handPoseHandoffActive = false;
        bool _handPoseBlockEngaged = false;
        bool _nativeCarrierTraceLogged = false;
        bool _nativeCarrierWasUsable = false;
        bool _active = false;
    };
}
