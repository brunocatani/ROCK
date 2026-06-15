#pragma once

#include <cstdint>

#include "physics-interaction/weapon/HeldWeaponVisualSnapshot.h"

#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiTransform.h"

namespace RE
{
    class TESObjectREFR;
}

namespace rock
{
    class HeldWeaponEquipVisualHandoff
    {
    public:
        struct BeginInput
        {
            HeldWeaponVisualSnapshot visual{};
        };

        struct FrameInput
        {
            float deltaSeconds = 0.0f;
            RE::NiAVObject* equippedWeaponRoot = nullptr;
        };

        ~HeldWeaponEquipVisualHandoff();

        [[nodiscard]] bool begin(const BeginInput& input) noexcept;
        void prepareForWeaponCollision(const FrameInput& input) noexcept;
        void updateAfterWeaponCollision(const FrameInput& input) noexcept;
        void cancel() noexcept;

        [[nodiscard]] bool active() const noexcept { return _active; }

    private:
        [[nodiscard]] bool beginImpl(const BeginInput& input);
        void prepareForWeaponCollisionImpl(const FrameInput& input);
        void updateAfterWeaponCollisionImpl(const FrameInput& input);
        void resetState() noexcept;
        void detachPhantom() noexcept;

        RE::NiPointer<RE::NiNode> _phantomRoot{};
        RE::NiPointer<RE::NiNode> _phantomParent{};
        RE::NiPointer<RE::NiAVObject> _observedEquippedWeaponRoot{};
        std::uint32_t _heldFormID = 0;
        std::uint32_t _frames = 0;
        std::uint32_t _equippedVisualFrames = 0;
        float _elapsedSeconds = 0.0f;
        bool _active = false;
        bool _isLeft = false;
    };
}
