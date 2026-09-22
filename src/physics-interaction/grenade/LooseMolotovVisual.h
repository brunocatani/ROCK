#pragma once

#include "RE/Bethesda/BSPointerHandle.h"
#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiTimeController.h"

#include <array>
#include <memory>

namespace RE { class TESObjectREFR; }

namespace rock
{
    // Game-thread owner of one armed reference's visuals. The reference's normal
    // scene update owns movement; ROCK alone advances the detached controllers.
    class LooseMolotovVisual
    {
    public:
        LooseMolotovVisual() = default;
        ~LooseMolotovVisual();
        LooseMolotovVisual(const LooseMolotovVisual&) = delete;
        LooseMolotovVisual& operator=(const LooseMolotovVisual&) = delete;

        // F4SE game/session notifications only: model I/O never runs on a frame.
        static void prepareResources() noexcept;
        [[nodiscard]] static std::unique_ptr<LooseMolotovVisual> create(RE::TESObjectREFR* reference) noexcept;
        void update(RE::TESObjectREFR* reference, float deltaSeconds);
        void release(const char* reason);

    private:
        static constexpr std::size_t kShapeCount = 5;
        struct Part
        {
            RE::NiPointer<RE::NiAVObject> shape;
            // Declare after shape so controller targets outlive the controllers.
            std::array<RE::NiPointer<RE::NiTimeController>, 2> controllers;
        };

        bool bind(RE::NiAVObject* root);
        void detach(const char* reason);
        void advanceAnimation();

        std::array<Part, kShapeCount> _parts{};
        RE::NiPointer<RE::NiAVObject> _observedRoot;
        RE::NiPointer<RE::NiNode> _parent;
        RE::ObjectRefHandle _referenceHandle;
        std::uint32_t _referenceId{ 0 };
        double _elapsedSeconds{ 0.0 };
        bool _reportedAnimation{ false };
    };
}
