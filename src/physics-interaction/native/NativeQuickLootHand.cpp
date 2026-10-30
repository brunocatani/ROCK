#include "physics-interaction/native/NativeQuickLootHand.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <limits>

#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/NativeNiNodeFactory.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "rock_support/Fo4VrRuntime.h"

namespace rock::native_quick_loot_hand
{
    namespace
    {
        /*
         * FO4VR 1.2.72, raw disassembly. The quick container target choice
         * 0xA96150 (closure, out handle): closure slots +0 and +8 both point
         * at the HUDQuickContainerDataModel pointer, whose +0x364/+0x368 hold
         * the right and left wand picks that its update 0xA92290 copies from
         * the wand view-caster events. It returns the right pick when
         * 0xA94EA0(model, &handle) accepts it, else the left pick. Its only
         * callers are the lazy evaluators 0xA96240 (call 0xA9625C) and
         * 0xAA2800 (call 0xAA2818). The entry has 13 position-independent
         * bytes, too few for an entry trampoline, so both calls are redirected.
         */
        constexpr std::uintptr_t kChooseTarget = 0xA96150;
        constexpr std::array<std::uintptr_t, 2> kChooseTargetCalls{ 0xA9625C, 0xAA2818 };
        constexpr std::uintptr_t kIsLootableTarget = 0xA94EA0;
        constexpr std::ptrdiff_t kRightPickOffset = 0x364;
        constexpr std::ptrdiff_t kLeftPickOffset = 0x368;
        // The engine's invalid reference handle, compared at 0xA96186 and stored by 0xA96240.
        constexpr std::uintptr_t kInvalidHandle = 0x392E884;
        /*
         * WSLootModel: its constructor 0xC85560 stores itself at 0x5ACBC98.
         * The player VR node builder 0xEF21A0 attaches it to
         * PrimaryUIAttachNode (player +0x700) through 0xC90220, which attaches
         * model +0x10; the model's placement 0xC85680 writes the fHUDLoot
         * offset, pitch and scale into that same root's local transform.
         */
        constexpr std::uintptr_t kLootModel = 0x5ACBC98;
        constexpr std::ptrdiff_t kLootModelRootOffset = 0x10;

        using ChooseTarget = std::uint32_t* (*)(void*, std::uint32_t*);
        using IsLootableTarget = bool (*)(void*, std::uint32_t*);

        enum class Hand : std::uint8_t
        {
            Right,
            Left
        };

        ChooseTarget g_original{ nullptr };
        bool g_attempted{ false };
        // Written by the HUD data model update, read by the game thread.
        std::atomic<Hand> g_panelHand{ Hand::Right };
        std::atomic<std::uint32_t> g_rightPick{ 0 };
        std::atomic<std::uint32_t> g_leftPick{ 0 };
        // Written by the game thread, read by the HUD data model update.
        std::atomic<bool> g_leftCloser{ false };
        // Game thread only. The ROCK node under the left wand that carries the
        // panel; it outlives player node rebuilds and is re-parented to the
        // current left wand when the old one is gone.
        RE::NiPointer<RE::NiNode> g_leftAttach;
        bool g_leftAttachFailed{ false };
        Hand g_placedHand{ Hand::Right };

        std::uint32_t* chooseTarget(void* closure, std::uint32_t* out)
        {
            g_original(closure, out);
            // The original already dereferenced both closure slots.
            auto* const model = **static_cast<std::byte***>(closure);
            const std::uint32_t invalid = *REL::Relocation<const std::uint32_t*>{ REL::Offset(kInvalidHandle) };
            const auto right = *reinterpret_cast<const std::uint32_t*>(model + kRightPickOffset);
            const auto left = *reinterpret_cast<const std::uint32_t*>(model + kLeftPickOffset);
            g_rightPick.store(right == invalid ? 0 : right, std::memory_order_relaxed);
            g_leftPick.store(left == invalid ? 0 : left, std::memory_order_relaxed);
            if (*out == invalid) {
                return out;
            }

            const bool leftCloser = g_leftCloser.load(std::memory_order_relaxed);
            Hand hand = Hand::Left;
            if (*out == right) {
                hand = Hand::Right;
                if (left == right) {
                    hand = leftCloser ? Hand::Left : Hand::Right;
                } else if (left != invalid && leftCloser) {
                    static REL::Relocation<IsLootableTarget> isLootable{ REL::Offset(kIsLootableTarget) };
                    std::uint32_t candidate = left;
                    if (isLootable(model, &candidate)) {
                        *out = left;
                        hand = Hand::Left;
                    }
                }
            }
            g_panelHand.store(hand, std::memory_order_release);
            return out;
        }

        float pickDistanceSquared(const std::uint32_t handleValue, const RE::NiNode* wand)
        {
            constexpr float kNone = (std::numeric_limits<float>::max)();
            if (!handleValue || !wand) {
                return kNone;
            }
            RE::ObjectRefHandle handle{};
            static_assert(sizeof(handle) == sizeof(handleValue));
            std::memcpy(&handle, &handleValue, sizeof(handle));
            const auto ref = handle.get();
            if (!ref) {
                return kNone;
            }
            // The bound centre stands for large containers better than their origin.
            const auto* root = ref->Get3D();
            const RE::NiPoint3 target = root && root->worldBound.fRadius > 0.0f ? root->worldBound.center : ref->data.location;
            const float dx = target.x - wand->world.translate.x;
            const float dy = target.y - wand->world.translate.y;
            const float dz = target.z - wand->world.translate.z;
            return dx * dx + dy * dy + dz * dz;
        }

        // Reflection across the wand's X axis; it stays a proper rotation.
        RE::NiTransform mirroredAcrossX(RE::NiTransform transform)
        {
            transform.translate.x = -transform.translate.x;
            transform.rotate.entry[0][1] = -transform.rotate.entry[0][1];
            transform.rotate.entry[0][2] = -transform.rotate.entry[0][2];
            transform.rotate.entry[1][0] = -transform.rotate.entry[1][0];
            transform.rotate.entry[2][0] = -transform.rotate.entry[2][0];
            return transform;
        }

        void move(RE::NiAVObject* root, RE::NiNode* from, RE::NiNode* to)
        {
            RE::NiPointer<RE::NiAVObject> held;
            from->DetachChild(root, held);
            if (held) {
                to->AttachChild(held.get(), true);
            }
        }

        bool attachLeftCarrier(RE::NiNode* leftWand)
        {
            if (!g_leftAttach) {
                if (g_leftAttachFailed) {
                    return false;
                }
                g_leftAttach = native_scene::createEngineNiNode(1);
                if (!g_leftAttach) {
                    g_leftAttachFailed = true;
                    ROCK_LOG_ERROR(Hand, "Quick loot: the left-hand panel node could not be created; the panel stays on the right hand");
                    return false;
                }
            }
            if (g_leftAttach->parent != leftWand) {
                if (auto* oldParent = g_leftAttach->parent) {
                    RE::NiPointer<RE::NiAVObject> detached;
                    oldParent->DetachChild(g_leftAttach.get(), detached);
                }
                leftWand->AttachChild(g_leftAttach.get(), true);
            }
            return true;
        }

        void place(const f4vr::PlayerNodes& nodes)
        {
            auto* const model = *REL::Relocation<std::byte**>{ REL::Offset(kLootModel) };
            auto* const root = model ? *reinterpret_cast<RE::NiAVObject**>(model + kLootModelRootOffset) : nullptr;
            auto* const rightAttach = nodes.primaryUIAttachNode;
            auto* const leftWand = nodes.SecondaryWandNode;
            if (!root || !rightAttach || !leftWand) {
                return;
            }
            const bool onLeft = g_leftAttach && root->parent == g_leftAttach.get();
            // Any other parent is a native rebuild or another owner; leave it.
            if (!onLeft && root->parent != rightAttach) {
                return;
            }

            const Hand hand = g_panelHand.load(std::memory_order_acquire);
            if (hand == Hand::Left) {
                if (!attachLeftCarrier(leftWand)) {
                    return;
                }
                // The panel's world pose under the left wand mirrors the one it
                // has under the right wand: carrier = mirror(attach * root) * root^-1.
                const auto rightPanel = transform_math::composeTransforms(rightAttach->local, root->local);
                g_leftAttach->local = transform_math::composeTransforms(
                    mirroredAcrossX(rightPanel), transform_math::invertTransform(root->local));
                if (!onLeft) {
                    move(root, rightAttach, g_leftAttach.get());
                }
            } else if (onLeft) {
                move(root, g_leftAttach.get(), rightAttach);
            }
            if (hand != g_placedHand) {
                g_placedHand = hand;
                ROCK_LOG_SAMPLE_DEBUG(Hand, 1000, "Quick loot panel moved to the {} hand", hand == Hand::Left ? "left" : "right");
            }
        }
    }

    bool install() noexcept
    {
        if (g_original) {
            return true;
        }
        if (g_attempted) {
            return false;
        }
        g_attempted = true;

        const auto target = REL::Offset(kChooseTarget).address();
        std::array<std::array<std::uint8_t, 5>, kChooseTargetCalls.size()> originals{};
        for (std::size_t i = 0; i < kChooseTargetCalls.size(); ++i) {
            const auto address = REL::Offset(kChooseTargetCalls[i]).address();
            auto& bytes = originals[i];
            std::int32_t displacement{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address), bytes.data(), bytes.size()) ||
                bytes[0] != 0xE8) {
                ROCK_LOG_WARN(Init, "Quick loot hand: target choice call 0x{:X} is not a call; the panel stays on the right hand",
                    kChooseTargetCalls[i]);
                return false;
            }
            std::memcpy(&displacement, bytes.data() + 1, sizeof(displacement));
            if (address + bytes.size() + displacement != target) {
                ROCK_LOG_WARN(Init, "Quick loot hand: call 0x{:X} no longer reaches the native target choice "
                    "(another mod may own it); the panel stays on the right hand", kChooseTargetCalls[i]);
                return false;
            }
        }

        auto& trampoline = F4SE::GetTrampoline();
        std::size_t written = 0;
        try {
            g_original = reinterpret_cast<ChooseTarget>(
                trampoline.write_call<5>(REL::Offset(kChooseTargetCalls[0]).address(), &chooseTarget));
            written = 1;
            (void)trampoline.write_call<5>(REL::Offset(kChooseTargetCalls[1]).address(), &chooseTarget);
        } catch (...) {
            for (std::size_t i = 0; i < written; ++i) {
                REL::safe_write(REL::Offset(kChooseTargetCalls[i]).address(), originals[i].data(), originals[i].size());
            }
            g_original = nullptr;
            ROCK_LOG_ERROR(Init, "Quick loot hand installation failed; the panel stays on the right hand");
            return false;
        }
        ROCK_LOG_INFO(Init, "Quick loot hand installed: the loot panel follows the hand closer to its container");
        return true;
    }

    void update() noexcept
    {
        if (!g_original) {
            return;
        }
        const auto* nodes = f4vr::getPlayerNodes();
        if (!nodes) {
            return;
        }
        const float right = pickDistanceSquared(g_rightPick.load(std::memory_order_relaxed), nodes->primaryWandNode);
        const float left = pickDistanceSquared(g_leftPick.load(std::memory_order_relaxed), nodes->SecondaryWandNode);
        g_leftCloser.store(left < right, std::memory_order_relaxed);
        place(*nodes);
    }
}
