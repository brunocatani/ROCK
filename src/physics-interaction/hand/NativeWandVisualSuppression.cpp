#include "physics-interaction/hand/NativeWandVisualSuppression.h"

#include "rock_support/Fo4VrRuntime.h"

#include <cstdint>
#include <cstring>

namespace rock::native_wand_visual_suppression
{
    namespace
    {
        constexpr std::uint64_t kAppCulledFlag = 0x1ull;
        constexpr std::uint32_t kMaximumControllerVisualDepth = 3;

        [[nodiscard]] bool isViveFallbackVisual(const RE::NiAVObject& object) noexcept
        {
            const char* const name = object.name.c_str();
            return name &&
                (std::strcmp(name, "ViveController") == 0 ||
                    std::strcmp(name, "vr_controller_vive_1_5") == 0);
        }

        void suppressViveFallbackVisuals(
            RE::NiAVObject* object,
            const std::uint32_t remainingDepth) noexcept
        {
            if (!object) {
                return;
            }

            if (isViveFallbackVisual(*object)) {
                object->flags.flags |= kAppCulledFlag;
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
                suppressViveFallbackVisuals(
                    child.get(),
                    remainingDepth - 1);
            }
        }
    }

    void enforce() noexcept
    {
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
        suppressViveFallbackVisuals(
            playerNodes->primaryWandNode,
            kMaximumControllerVisualDepth);
        suppressViveFallbackVisuals(
            playerNodes->SecondaryWandNode,
            kMaximumControllerVisualDepth);
    }
}
