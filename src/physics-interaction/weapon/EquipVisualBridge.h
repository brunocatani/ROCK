#pragma once

#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiSmartPointer.h"

#include "physics-interaction/weapon/EquippedWeaponVisualState.h"

#include <array>
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
     * is stable, the model is parked off-scene as a bounded standby. If a late
     * stale WeaponDetach event removes the new native graph, the coordinator
     * can re-present this model in the same ROCK frame while the exact native
     * attach is repaired. Every exit restores any native child cull and
     * deterministically releases both scene references and pose authority.
     *
     * Blend target. begin() re-runs the shared loose-grip resolver against
     * the filewatch-published hFRIK cache. Both hands therefore converge
     * on the exact relation used during pull seating: custom JSON first,
     * ROCK's learned animation pose second, embedded hFRIK data only as a
     * cold fallback. A first-ever weapon with no authoritative relation uses
     * the engine Weapon bone until its native graph produces the first ROCK
     * capture; it never borrows another weapon's cached live local.
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
            // hand frame; weapon world = hand world o inverse(hold)). Same
            // capture the pending primary-only grip start consumes.
            bool hasFiringHandWeaponLocal = false;
            RE::NiTransform firingHandWeaponLocal{};
            float timeoutSeconds = 2.0f;
            float blendSeconds = 0.15f;
        };

        struct UpdateInput
        {
            float deltaSeconds = 0.0f;
            bool advanceLifetime = true;
            bool presentModel = true;
            const equipped_weapon_visual_state::Snapshot* nativeVisual = nullptr;
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
        // The coordinator may keep the detached model as a hidden standby and
        // re-present it in the same frame that a late native detach is seen.
        void update(const UpdateInput& input);

        // Ends the bounded late-detach watchdog and releases the hidden model.
        void releaseStandbyModel(const char* reason);

        // Detach from the (still valid) scene graph and release.
        void shutdown();

        // Scene/world already gone: release our reference without touching
        // possibly-stale parent pointers.
        void abandonSceneGraph();

        [[nodiscard]] bool isActive() const noexcept { return _active; }
        [[nodiscard]] bool hasStandbyModel() const noexcept { return _model != nullptr; }
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
        void hideModelForNativeStandby(const char* reason);
        void clearModel(const char* reason, bool detachFromParent);
        void clearHandPoseHandoff(const char* reason, bool logCompletion, bool discardPayload);
        void clear(const char* reason, bool detachFromParent, bool restoreNativeCull = true);

        RE::NiPointer<RE::NiAVObject> _model;
        RE::NiPointer<RE::NiAVObject> _culledNativeInstance;
        // Non-owning; validated each frame against _model->parent before use.
        RE::NiNode* _parent = nullptr;
        RE::NiTransform _modelInHandLocal{};
        // Canonical loose-to-equipped hold (see BeginInput). Re-resolved at
        // begin() against the live filewatch-published hFRIK cache so the
        // bridge obeys the same priority as pull seating and grip-zone equip.
        RE::NiTransform _firingHandWeaponLocal{};
        bool _hasFiringHandWeaponLocal = false;
        std::array<RE::NiTransform, 15> _handoffFingerLocalTransforms{};
        std::uint16_t _handoffFingerLocalTransformMask = 0;
        float _elapsedSeconds = 0.0f;
        float _lifetimeSeconds = 0.0f;
        float _blendSeconds = 0.15f;
        float _timeoutSeconds = 2.0f;
        std::uint32_t _weaponFormID = 0;
        bool _isLeftHand = false;
        bool _modelPresented = false;
        bool _culledNativeInstanceWasVisible = false;
        bool _handPosePayloadAvailable = false;
        bool _handPoseHandoffActive = false;
        bool _handPoseBlockEngaged = false;
        bool _active = false;
    };
}
