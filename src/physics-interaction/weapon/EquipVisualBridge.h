#pragma once

#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiSmartPointer.h"

#include <cstdint>

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
     * it to the equipping hand's wand transform, blends it toward the
     * first-person Weapon bone pose, and removes it the moment the engine's
     * equipped instance node exists and is visible. Every exit is
     * deterministic: swap detection, equip failure, node loss, or timeout.
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
            std::uint32_t weaponFormID = 0;
            bool isLeftHand = false;
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

        // Per-frame: pose glue + blend, swap detection, timeout.
        void update(float deltaSeconds);

        // Detach from the (still valid) scene graph and release.
        void shutdown();

        // Scene/world already gone: release our reference without touching
        // possibly-stale parent pointers.
        void abandonSceneGraph();

        [[nodiscard]] bool isActive() const noexcept { return _active; }

    private:
        // Attach the (already orphaned) model under the world root; false when
        // the model is still parented or the world root is unavailable.
        bool tryAttachToWorldRoot();
        void clear(const char* reason, bool detachFromParent);

        RE::NiPointer<RE::NiAVObject> _model;
        // Non-owning; validated each frame against _model->parent before use.
        RE::NiNode* _parent = nullptr;
        RE::NiTransform _modelInHandLocal{};
        float _elapsedSeconds = 0.0f;
        float _blendSeconds = 0.15f;
        float _timeoutSeconds = 2.0f;
        std::uint32_t _weaponFormID = 0;
        char _instanceNameToken[16] = {};
        bool _isLeftHand = false;
        bool _active = false;
    };
}
