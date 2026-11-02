#include "physics-interaction/hand/NativeWandVisualSuppression.h"

#include "rock_support/Fo4VrRuntime.h"

#include <array>
#include <cstdint>
#include <cstring>

namespace rock::native_wand_visual_suppression
{
    namespace
    {
        constexpr std::uint64_t kAppCulledFlag = 0x1ull;
        constexpr std::uint32_t kMaximumControllerVisualDepth = 3;

        // Frame thread only. Found fallback roots, re-culled every frame: the
        // native device-model update can clear the cull flag again, so a
        // periodic cull alone leaves the model visible between checks. A
        // reference is held only while the root stays attached. Never
        // destroyed: releasing game nodes during process exit is unsafe.
        auto& s_fallbackRoots = *new std::array<RE::NiPointer<RE::NiAVObject>, 4>{};

        void remember(RE::NiAVObject* object) noexcept
        {
            for (const auto& root : s_fallbackRoots) {
                if (root.get() == object)
                    return;
            }
            for (auto& root : s_fallbackRoots) {
                if (!root) {
                    root.reset(object);
                    return;
                }
            }
        }

        [[nodiscard]] bool isViveFallbackVisual(const RE::NiAVObject& object) noexcept
        {
            const char* const name = object.name.c_str();
            return name &&
                (std::strcmp(name, "ViveController") == 0 || std::strcmp(name, "vr_controller_vive_1_5") == 0);
        }

        void suppressViveFallbackVisuals(RE::NiAVObject* object, const std::uint32_t remainingDepth) noexcept
        {
            if (!object) {
                return;
            }

            if (isViveFallbackVisual(*object)) {
                object->flags.flags |= kAppCulledFlag;
                remember(object);
                return;
            }

            if (remainingDepth == 0) {
                return;
            }

            auto* const node = object->IsNode();
            if (!node) {
                return;
            }

            for (const auto& child : node->children) {
                suppressViveFallbackVisuals(child.get(), remainingDepth - 1);
            }
        }
    }

    void enforce() noexcept
    {
        for (auto& root : s_fallbackRoots) {
            if (root && !root->parent)
                root.reset();
            if (root)
                root->flags.flags |= kAppCulledFlag;
        }

        // The fallback model is attached only when a controller (re)connects,
        // so a half-second search finds it without walking both wands every
        // frame; the found roots above stay culled every frame.
        constexpr std::uint32_t kCheckIntervalFrames = 45;
        static std::uint32_t framesUntilCheck = 0;
        if (framesUntilCheck > 0) {
            --framesUntilCheck;
            return;
        }
        framesUntilCheck = kCheckIntervalFrames;

        auto* const playerNodes = f4vr::getPlayerNodes();
        if (!playerNodes) {
            return;
        }

        /*
         * Fallout4VR's tracked-device model loader attaches Vive_Wand.nif when
         * the OpenVR device model is unavailable. A controller that wakes after
         * startup can keep that fallback attached. Cull only the two verified
         * fallback node names: the wand roots also own weapon, UI, laser, and
         * Pip-Boy presentation that must remain visible.
         */
        suppressViveFallbackVisuals(playerNodes->primaryWandNode, kMaximumControllerVisualDepth);
        suppressViveFallbackVisuals(playerNodes->SecondaryWandNode, kMaximumControllerVisualDepth);
    }
}
