#include "physics-interaction/native/NativeQuickLootHand.h"

#include <array>
#include <atomic>
#include <cmath>
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
         * FO4VR 1.2.72, raw disassembly of the HUDQuickContainerDataModel
         * update 0xA92290. It copies the right and left wand picks from the
         * wand view-caster events into model +0x364 (0xA923E4) and +0x368
         * (0xA923F7), then locks model +0x268 (lea 0xA923FD, call 0xA92417 to
         * the spin-lock acquire 0x24E10). Everything after that reads the
         * picks from the two slots: the target choice 0xA96150 returns +0x364
         * when 0xA94EA0 accepts it, else +0x368, and the data the panel
         * renders records whether +0x364 is lootable (0xA928C0..0xA928EE).
         * The panel lists a container only when that primary flag is set, so
         * a pick by the left wand alone chose the container but left the
         * panel blank.
         *
         * The lock call is redirected. Before it runs, when the left hand owns
         * the panel the two slots are swapped, so the model treats the left
         * pick as the primary one for the rest of this update and until the
         * next update copies the real picks again. 0xF0A130, which gates the
         * copy, returns true on VR.
         */
        constexpr std::uintptr_t kPicksCopiedLockCall = 0xA92417;
        constexpr std::uintptr_t kSpinLockAcquire = 0x24E10;
        constexpr std::ptrdiff_t kModelLockOffset = 0x268;
        constexpr std::ptrdiff_t kRightPickOffset = 0x364;
        constexpr std::ptrdiff_t kLeftPickOffset = 0x368;
        constexpr std::uintptr_t kIsLootableTarget = 0xA94EA0;
        // The engine's invalid reference handle, compared at 0xA96186.
        constexpr std::uintptr_t kInvalidHandle = 0x392E884;

        struct ExpectedBytes
        {
            std::uintptr_t address;
            std::array<std::uint8_t, 7> bytes;
            std::size_t size;
        };
        constexpr std::array<ExpectedBytes, 4> kExpectedPickCopy{ {
            { 0xA923E4, { 0x89, 0x86, 0x64, 0x03, 0x00, 0x00 }, 6 },        // mov [rsi+0x364], eax
            { 0xA923F7, { 0x89, 0x86, 0x68, 0x03, 0x00, 0x00 }, 6 },        // mov [rsi+0x368], eax
            { 0xA923FD, { 0x48, 0x8D, 0x9E, 0x68, 0x02, 0x00, 0x00 }, 7 },  // lea rbx, [rsi+0x268]
            { 0xA9240D, { 0x48, 0x8B, 0xCB }, 3 },                          // mov rcx, rbx
        } };

        /*
         * HUDRolloverModel vtable slot 16 (call [vtable+0x80]) says whether a
         * wand's Search prompt hosts the quick container: the primary model
         * returns true (0xADDD60, mov al,1), the secondary false (0xADDD80,
         * xor al,al). Its only callers in the rollover code (0xAC7870,
         * 0xAC7E10, 0xAC7F60, 0xAD8930) pair it with the quick container
         * state's primary-lootable flag (0xAC5C30 reads state +0xC0) to hide
         * the hosting wand's prompt while the panel lists, and with dead-actor
         * picks. Both slots are replaced so the hand that holds the panel
         * hosts it: the left prompt hides while the left hand lists, and the
         * right prompt keeps its own target.
         */
        struct RolloverHostSlot
        {
            std::uintptr_t vtable;
            std::uintptr_t native;
            std::array<std::uint8_t, 3> nativeBytes;
        };
        constexpr std::size_t kHostsQuickContainerSlot = 16;
        constexpr RolloverHostSlot kPrimaryRolloverHost{ 0x2D4ED78, 0xADDD60, { 0xB0, 0x01, 0xC3 } };
        constexpr RolloverHostSlot kSecondaryRolloverHost{ 0x2D4EF98, 0xADDD80, { 0x32, 0xC0, 0xC3 } };

        /*
         * WSLootModel: its constructor 0xC85560 stores itself at 0x5ACBC98.
         * The player VR node builder 0xEF21A0 attaches it to
         * PrimaryUIAttachNode (player +0x700) through 0xC90220, which attaches
         * model +0x10; the model's placement 0xC85680 writes the fHUDLoot
         * offset, pitch and scale into that same root's local transform.
         */
        constexpr std::uintptr_t kLootModel = 0x5ACBC98;
        constexpr std::ptrdiff_t kLootModelRootOffset = 0x10;

        /*
         * When both wands pick lootable containers (often the same one, with
         * both hands on a two-handed weapon), a hand takes the panel only when
         * it is this much closer to its container than the owning hand is to
         * its own. Without the margin the panel flipped hands every few frames
         * on near-equal distances. Game units; about 7 cm.
         */
        constexpr float kOwnerSwitchMargin = 5.0f;
        constexpr float kNoPick = (std::numeric_limits<float>::max)();

        using SpinLockAcquire = void* (*)(void*, std::uintptr_t);
        using IsLootableTarget = bool (*)(void*, std::uint32_t*);

        enum class Hand : std::uint8_t
        {
            Right,
            Left
        };

        SpinLockAcquire g_lockAcquire{ nullptr };
        bool g_attempted{ false };
        // Written by the HUD data model update, read by the game thread.
        std::atomic<Hand> g_panelHand{ Hand::Right };
        std::atomic<std::uint32_t> g_rightPick{ 0 };
        std::atomic<std::uint32_t> g_leftPick{ 0 };
        // Written by the HUD data model update, read by the rollover models.
        std::atomic<bool> g_leftHostsPanel{ false };
        // Written by the game thread, read by the HUD data model update.
        std::atomic<bool> g_preferLeft{ false };
        // Game thread only. The ROCK node under the left wand that carries the
        // panel; it outlives player node rebuilds and is re-parented to the
        // current left wand when the old one is gone.
        RE::NiPointer<RE::NiNode> g_leftAttach;
        bool g_leftAttachFailed{ false };
        Hand g_placedHand{ Hand::Right };

        // Runs once per model update, right after the real picks are copied.
        void choosePanelHand(std::byte* model)
        {
            static REL::Relocation<IsLootableTarget> isLootable{ REL::Offset(kIsLootableTarget) };
            const std::uint32_t invalid = *REL::Relocation<const std::uint32_t*>{ REL::Offset(kInvalidHandle) };
            auto* const rightSlot = reinterpret_cast<std::uint32_t*>(model + kRightPickOffset);
            auto* const leftSlot = reinterpret_cast<std::uint32_t*>(model + kLeftPickOffset);
            const std::uint32_t right = *rightSlot;
            const std::uint32_t left = *leftSlot;

            // 0xA94EA0 takes the handle by address; the native callers pass a copy.
            std::uint32_t candidate = right;
            const bool rightLootable = right != invalid && isLootable(model, &candidate);
            candidate = left;
            const bool leftLootable = left != invalid && isLootable(model, &candidate);
            g_rightPick.store(rightLootable ? right : 0, std::memory_order_relaxed);
            g_leftPick.store(leftLootable ? left : 0, std::memory_order_relaxed);
            if (!rightLootable && !leftLootable) {
                g_leftHostsPanel.store(false, std::memory_order_relaxed);
                return;
            }

            const bool leftOwns = leftLootable && (!rightLootable || g_preferLeft.load(std::memory_order_relaxed));
            if (leftOwns && left != right) {
                *rightSlot = left;
                *leftSlot = right;
            }
            g_leftHostsPanel.store(leftOwns, std::memory_order_relaxed);
            g_panelHand.store(leftOwns ? Hand::Left : Hand::Right, std::memory_order_release);
        }

        bool primaryRolloverHostsPanel(void*)
        {
            return !g_leftHostsPanel.load(std::memory_order_relaxed);
        }

        bool secondaryRolloverHostsPanel(void*)
        {
            return g_leftHostsPanel.load(std::memory_order_relaxed);
        }

        std::uintptr_t rolloverHostSlotAddress(const RolloverHostSlot& host)
        {
            return REL::Offset(host.vtable).address() + sizeof(std::uintptr_t) * kHostsQuickContainerSlot;
        }

        bool rolloverHostSlotMatches(const RolloverHostSlot& host)
        {
            std::uintptr_t current{};
            std::array<std::uint8_t, 3> bytes{};
            const auto native = REL::Offset(host.native).address();
            return native_memory::guardedCopyFromMemory(
                       reinterpret_cast<const void*>(rolloverHostSlotAddress(host)), &current, sizeof(current)) &&
                   current == native &&
                   native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(native), bytes.data(), bytes.size()) &&
                   bytes == host.nativeBytes;
        }

        // Both prompts switch together or neither does.
        void installRolloverHosts()
        {
            if (!rolloverHostSlotMatches(kPrimaryRolloverHost) || !rolloverHostSlotMatches(kSecondaryRolloverHost)) {
                ROCK_LOG_WARN(Init, "Quick loot hand: the rollover host slots do not match; "
                    "the left hand's Search prompt stays visible while it lists a container");
                return;
            }
            REL::safe_write(rolloverHostSlotAddress(kPrimaryRolloverHost),
                reinterpret_cast<std::uintptr_t>(&primaryRolloverHostsPanel));
            REL::safe_write(rolloverHostSlotAddress(kSecondaryRolloverHost),
                reinterpret_cast<std::uintptr_t>(&secondaryRolloverHostsPanel));
        }

        void* afterPicksCopied(void* lock, const std::uintptr_t argument)
        {
            // The redirected call passes model + 0x268 (lea 0xA923FD, mov 0xA9240D).
            choosePanelHand(static_cast<std::byte*>(lock) - kModelLockOffset);
            return g_lockAcquire(lock, argument);
        }

        float pickDistance(const std::uint32_t handleValue, const RE::NiNode* wand)
        {
            if (!handleValue || !wand) {
                return kNoPick;
            }
            RE::ObjectRefHandle handle{};
            static_assert(sizeof(handle) == sizeof(handleValue));
            std::memcpy(&handle, &handleValue, sizeof(handle));
            const auto ref = handle.get();
            if (!ref) {
                return kNoPick;
            }
            // The bound centre stands for large containers better than their origin.
            const auto* root = ref->Get3D();
            const RE::NiPoint3 target = root && root->worldBound.fRadius > 0.0f ? root->worldBound.center : ref->data.location;
            const float dx = target.x - wand->world.translate.x;
            const float dy = target.y - wand->world.translate.y;
            const float dz = target.z - wand->world.translate.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        // Publishes which hand the next model update gives the panel when both
        // wands pick lootable containers.
        void updatePreference(const float right, const float left)
        {
            const bool onLeft = g_panelHand.load(std::memory_order_acquire) == Hand::Left;
            if (right == kNoPick || left == kNoPick) {
                g_preferLeft.store(onLeft, std::memory_order_relaxed);
                return;
            }
            const float otherCloserBy = onLeft ? left - right : right - left;
            bool preferLeft = onLeft;
            if (otherCloserBy > kOwnerSwitchMargin) {
                preferLeft = !onLeft;
            } else if (otherCloserBy > 0.0f) {
                ROCK_LOG_SAMPLE_DEBUG(Hand, 1000,
                    "Quick loot panel held on the {} hand: the other hand is {:.1f} units closer, under the {:.1f}-unit switch margin",
                    onLeft ? "left" : "right", otherCloserBy, kOwnerSwitchMargin);
            }
            g_preferLeft.store(preferLeft, std::memory_order_relaxed);
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
        if (g_lockAcquire) {
            return true;
        }
        if (g_attempted) {
            return false;
        }
        g_attempted = true;

        for (const auto& expected : kExpectedPickCopy) {
            std::array<std::uint8_t, 7> live{};
            if (!native_memory::guardedCopyFromMemory(
                    reinterpret_cast<const void*>(REL::Offset(expected.address).address()), live.data(), expected.size) ||
                std::memcmp(live.data(), expected.bytes.data(), expected.size) != 0) {
                ROCK_LOG_WARN(Init, "Quick loot hand: the quick container pick copy at 0x{:X} does not match; "
                    "the panel stays on the right hand", expected.address);
                return false;
            }
        }
        const auto callAddress = REL::Offset(kPicksCopiedLockCall).address();
        std::array<std::uint8_t, 5> call{};
        std::int32_t displacement{};
        if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(callAddress), call.data(), call.size()) ||
            call[0] != 0xE8) {
            ROCK_LOG_WARN(Init, "Quick loot hand: 0x{:X} is not a call; the panel stays on the right hand", kPicksCopiedLockCall);
            return false;
        }
        std::memcpy(&displacement, call.data() + 1, sizeof(displacement));
        if (callAddress + call.size() + displacement != REL::Offset(kSpinLockAcquire).address()) {
            ROCK_LOG_WARN(Init, "Quick loot hand: call 0x{:X} no longer reaches the model lock "
                "(another mod may own it); the panel stays on the right hand", kPicksCopiedLockCall);
            return false;
        }

        try {
            g_lockAcquire = reinterpret_cast<SpinLockAcquire>(
                F4SE::GetTrampoline().write_call<5>(callAddress, &afterPicksCopied));
        } catch (...) {
            REL::safe_write(callAddress, call.data(), call.size());
            g_lockAcquire = nullptr;
            ROCK_LOG_ERROR(Init, "Quick loot hand installation failed; the panel stays on the right hand");
            return false;
        }
        installRolloverHosts();
        ROCK_LOG_INFO(Init, "Quick loot hand installed: the loot panel follows the hand closer to its container");
        return true;
    }

    void update() noexcept
    {
        if (!g_lockAcquire) {
            return;
        }
        const auto* nodes = f4vr::getPlayerNodes();
        if (!nodes) {
            return;
        }
        updatePreference(pickDistance(g_rightPick.load(std::memory_order_relaxed), nodes->primaryWandNode),
            pickDistance(g_leftPick.load(std::memory_order_relaxed), nodes->SecondaryWandNode));
        place(*nodes);
    }
}
