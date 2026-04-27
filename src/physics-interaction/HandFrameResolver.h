#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "PhysicsLog.h"
#include "RockConfig.h"

#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiTransform.h"

#include "f4vr/F4VRUtils.h"

namespace frik::rock
{
    enum class HandFrameSource : int
    {
        FistHelperThenWand = 0,
        WandNode = 1,
        HandBone = 2,
        FrikApi = 3,
    };

    struct HandFrame
    {
        RE::NiTransform transform{};
        RE::NiNode* node = nullptr;
        const char* label = "none";
        bool valid = false;
    };

    class HandFrameResolver
    {
    public:
        /*
         * The live collision overlay showed two different problems: native wand/helper
         * frames do not match the visible FRIK hands in this setup, while the first-person
         * hand node and FRIK API frames expose correct raw hand axes. Keep those hand-bone
         * frames as the active source for ROCK, and leave Bethesda wand frames selectable
         * only for diagnostics around PlayerCharacter +0x6F0/+0x768.
         */
        HandFrame resolve(bool isLeft, RE::NiNode* handBone, RE::NiNode* frikNode, const RE::NiTransform& frikTransform, bool frikValid) const
        {
            const auto source = normalizedSource();
            HandFrame frame{};

            switch (source) {
            case HandFrameSource::FistHelperThenWand:
                frame = resolveFistHelper(isLeft);
                if (frame.valid) {
                    return frame;
                }
                frame = resolveWand(isLeft);
                if (frame.valid) {
                    return frame;
                }
                break;
            case HandFrameSource::WandNode:
                frame = resolveWand(isLeft);
                if (frame.valid) {
                    return frame;
                }
                break;
            case HandFrameSource::HandBone:
                frame = fromNode(handBone, "hand-bone");
                if (frame.valid) {
                    return frame;
                }
                break;
            case HandFrameSource::FrikApi:
                if (frikValid && frikNode) {
                    return HandFrame{ frikTransform, frikNode, "frik-api", true };
                }
                if (frikValid) {
                    return HandFrame{ frikTransform, nullptr, "frik-api", true };
                }
                break;
            }

            frame = fromNode(handBone, "hand-bone-fallback");
            if (frame.valid) {
                return frame;
            }

            if (frikValid && frikNode) {
                return HandFrame{ frikTransform, frikNode, "frik-api-fallback", true };
            }
            if (frikValid) {
                return HandFrame{ frikTransform, nullptr, "frik-api-fallback", true };
            }
            return {};
        }

    private:
        static constexpr std::uintptr_t kPrimaryWandOffset = 0x6F0;
        static constexpr std::uintptr_t kSecondaryWandOffset = 0x768;

        static HandFrameSource normalizedSource()
        {
            switch (g_rockConfig.rockHandFrameSource) {
            case 0:
                return HandFrameSource::FistHelperThenWand;
            case 1:
                return HandFrameSource::WandNode;
            case 2:
                return HandFrameSource::HandBone;
            case 3:
                return HandFrameSource::FrikApi;
            default:
                return HandFrameSource::FistHelperThenWand;
            }
        }

        static RE::NiNode* getVerifiedWandNode(bool isLeft)
        {
            auto* player = RE::PlayerCharacter::GetSingleton();
            if (!player) {
                return nullptr;
            }

            auto* bytes = reinterpret_cast<std::byte*>(player);
            const bool useLeftSlot = g_rockConfig.rockHandFrameSwapWands ? !isLeft : isLeft;
            const auto offset = useLeftSlot ? kSecondaryWandOffset : kPrimaryWandOffset;
            return *reinterpret_cast<RE::NiNode**>(bytes + offset);
        }

        static HandFrame fromNode(RE::NiNode* node, const char* label)
        {
            if (!node) {
                return {};
            }
            return HandFrame{ node->world, node, label, true };
        }

        static HandFrame resolveWand(bool isLeft)
        {
            const bool useLeftSlot = g_rockConfig.rockHandFrameSwapWands ? !isLeft : isLeft;
            return fromNode(getVerifiedWandNode(isLeft), useLeftSlot ? "secondary-wand" : "primary-wand");
        }

        static HandFrame resolveFistHelper(bool isLeft)
        {
            auto* wand = getVerifiedWandNode(isLeft);
            if (!wand) {
                return {};
            }

            const auto& names = isLeft ? leftHelperNames() : rightHelperNames();
            for (const char* name : names) {
                if (auto* helper = f4vr::findNode(wand, name, 6)) {
                    return fromNode(helper, name);
                }
            }

            return {};
        }

        static constexpr std::array<const char*, 3> rightHelperNames()
        {
            return { "fist_M_Right_HELPER", "fist_F_Right_HELPER", "PA_fist_R_HELPER" };
        }

        static constexpr std::array<const char*, 3> leftHelperNames()
        {
            return { "fist_M_Left_HELPER", "fist_F_Left_HELPER", "PA_fist_L_HELPER" };
        }
    };
}
