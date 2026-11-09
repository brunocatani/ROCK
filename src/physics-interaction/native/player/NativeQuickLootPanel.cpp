#include "physics-interaction/native/player/NativeQuickLootPanel.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/player/NativeWorldUiAnchor.h"
#include "rock_support/Fo4VrRuntime.h"
#include <RPS/Runtime/HookChain.h>

namespace rock::native_quick_loot_panel
{
    namespace
    {
        using native_world_ui_anchor::attachCarrier;
        using native_world_ui_anchor::moveTo;

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
         * The lock call is redirected. Before it runs, when the left wand owns
         * the listing the two slots are swapped, so the model treats its pick
         * as the primary one for the rest of this update and until the next
         * update copies the real picks again. 0xF0A130, which gates the copy,
         * returns true on VR.
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
            std::array<std::uint8_t, 14> bytes;
            std::size_t size;
        };

        bool matches(const ExpectedBytes& expected)
        {
            std::array<std::uint8_t, 14> live{};
            return native_memory::guardedCopyFromMemory(
                       reinterpret_cast<const void*>(REL::Offset(expected.address).address()), live.data(),
                       expected.size) &&
                std::memcmp(live.data(), expected.bytes.data(), expected.size) == 0;
        }

        constexpr std::array<ExpectedBytes, 4> kExpectedPickCopy{ {
            { 0xA923E4, { 0x89, 0x86, 0x64, 0x03, 0x00, 0x00 }, 6 }, // mov [rsi+0x364], eax
            { 0xA923F7, { 0x89, 0x86, 0x68, 0x03, 0x00, 0x00 }, 6 }, // mov [rsi+0x368], eax
            { 0xA923FD, { 0x48, 0x8D, 0x9E, 0x68, 0x02, 0x00, 0x00 }, 7 }, // lea rbx, [rsi+0x268]
            { 0xA9240D, { 0x48, 0x8B, 0xCB }, 3 }, // mov rcx, rbx
        } };

        /*
         * HUDRolloverModel vtable slot 16 (call [vtable+0x80]) says whether a
         * wand's Search prompt hosts the quick container: the primary model
         * returns true (0xADDD60, mov al,1), the secondary false (0xADDD80,
         * xor al,al). Its only callers in the rollover code (0xAC7870,
         * 0xAC7E10, 0xAC7F60, 0xAD8930) pair it with the quick container
         * state's primary-lootable flag (0xAC5C30 reads state +0xC0) to hide
         * the hosting wand's prompt while the panel lists, and with dead-actor
         * picks. Both slots are replaced: at the container a wand hosts the
         * panel exactly when it points at the listed container, on a hand the
         * hand that carries it hosts it, and with nothing listed or in
         * RightHand mode both return their native answers.
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
        constexpr std::ptrdiff_t kModelRootOffset = 0x10;

        /*
         * The panel's button hints are a second model. WSPrimaryWandHUDModel's
         * constructor 0xC8D140 builds it on Data\Meshes\world_primaryWand.nif
         * and stores itself at 0x5ACBC80 (0xC8D18E); the WSPrimaryWandHUD menu
         * renders the ws_primaryWand movie, whose ButtonHintBar_mc holds the
         * hints, onto it. The player VR node builder attaches it to
         * PrimaryUIAttachNode beside the panel (0xEF39DF..0xEF39F0, through
         * 0xC90220), and its placements 0xC8D280 and 0xC8D3F0 write the local
         * transform of its root, model +0x10 (mov rbx, [rcx+0x10] at 0xC8D2B5).
         */
        constexpr std::uintptr_t kHintModel = 0x5ACBC80;

        constexpr std::array<ExpectedBytes, 3> kExpectedHintModel{ {
            // mov [rip -> 0x5ACBC80], rax
            { 0xC8D18E, { 0x48, 0x89, 0x05, 0xEB, 0xEA, 0xE3, 0x04 }, 7 },
            // mov rdx, [r14+0x700]; mov rcx, [rip -> 0x5ACBC80]
            { 0xEF39DF, { 0x49, 0x8B, 0x96, 0x00, 0x07, 0x00, 0x00, 0x48, 0x8B, 0x0D, 0x93, 0x82, 0xBD, 0x04 }, 14 },
            // mov rbx, [rcx+0x10]
            { 0xC8D2B5, { 0x48, 0x8B, 0x59, 0x10 }, 4 },
        } };

        /*
         * Meshes\world_Loot.nif (Fallout4_VR - Main.ba2): the quickLoot:0 quad
         * lies in its XZ plane (Y = 0); U grows along +X and V shrinks along
         * +Z, so the text reads along +X with +Z up and faces -Y.
         */
        constexpr native_world_ui_anchor::FaceFrame kLootFace{ { 1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f },
            { 0.0f, -1.0f, 0.0f } };

        /*
         * CloserHand: when both wands pick lootable containers (often the same
         * one, with both hands on a two-handed weapon), a hand takes the panel
         * only when it is this much closer to its container than the owning
         * hand is to its own. Without the margin the panel flipped hands every
         * few frames on near-equal distances. Game units; about 7 cm.
         */
        constexpr float kOwnerSwitchMargin = 5.0f;
        constexpr float kNoPick = (std::numeric_limits<float>::max)();

        using SpinLockAcquire = void* (*)(void*, std::uintptr_t);
        using IsLootableTarget = bool (*)(void*, std::uint32_t*);

        // Which wands host the container the panel lists.
        enum ListedFlags : std::uint8_t
        {
            kListed = 1 << 0,
            kRightHostsListed = 1 << 1,
            kLeftHostsListed = 1 << 2,
            // The left wand's pick is listed (the picks were swapped).
            kLeftOwnsListed = 1 << 3,
        };

        SpinLockAcquire g_lockAcquire{ nullptr };
        bool g_attempted{ false };
        // Written by the game thread, read by the HUD data model update.
        std::atomic<Placement> g_placement{ Placement::AtContainer };
        std::atomic<bool> g_preferLeft{ false };
        // Written by the HUD data model update; read by the rollover models
        // and the game thread.
        std::atomic<std::uint8_t> g_listedFlags{ 0 };
        std::atomic<std::uint32_t> g_listedPick{ 0 };
        std::atomic<std::uint32_t> g_rightPick{ 0 };
        std::atomic<std::uint32_t> g_leftPick{ 0 };
        // Game thread only. ROCK's carrier nodes: one under the room node that
        // holds the panel and its button hints at its container, and under the
        // left wand one each for the panel and the hints on the left hand. They
        // outlive player node rebuilds and are re-parented when their old
        // parent is gone.
        // Never destroyed: an engine object released in DLL detach raises R6025 during ExitProcess.
        RE::NiPointer<RE::NiNode>& g_containerCarrier = *new RE::NiPointer<RE::NiNode>;
        RE::NiPointer<RE::NiNode>& g_leftCarrier = *new RE::NiPointer<RE::NiNode>;
        RE::NiPointer<RE::NiNode>& g_leftHintCarrier = *new RE::NiPointer<RE::NiNode>;
        bool g_containerCarrierFailed{ false };
        bool g_leftCarrierFailed{ false };
        bool g_leftHintCarrierFailed{ false };
        // The hint model matched at install; otherwise the hints stay native.
        bool g_hintsFollow{ false };
        std::optional<Placement> g_appliedPlacement;

        const char* placementName(const Placement placement)
        {
            switch (placement) {
            case Placement::RightHand:
                return "on the right hand (game default)";
            case Placement::CloserHand:
                return "on the hand closer to its container";
            case Placement::AtContainer:
                return "at its container";
            }
            return "unknown";
        }

        // Runs once per model update, right after the real picks are copied.
        void chooseListedPick(std::byte* model)
        {
            const Placement placement = g_placement.load(std::memory_order_relaxed);
            if (placement == Placement::RightHand) {
                // The native listing, prompts and picks, untouched.
                g_listedFlags.store(0, std::memory_order_relaxed);
                g_rightPick.store(0, std::memory_order_relaxed);
                g_leftPick.store(0, std::memory_order_relaxed);
                g_listedPick.store(0, std::memory_order_release);
                return;
            }

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
                g_listedFlags.store(0, std::memory_order_relaxed);
                g_listedPick.store(0, std::memory_order_release);
                return;
            }

            // At the container the native choice stands (the right pick when
            // lootable); on a hand, the closer hand when both wands pick.
            const bool leftOwns = leftLootable &&
                (!rightLootable ||
                    (placement == Placement::CloserHand && g_preferLeft.load(std::memory_order_relaxed)));
            const std::uint32_t listed = leftOwns ? left : right;
            if (leftOwns && left != right) {
                *rightSlot = left;
                *leftSlot = right;
            }
            std::uint8_t flags = kListed | (leftOwns ? kLeftOwnsListed : 0);
            if (placement == Placement::CloserHand) {
                // Only the hand that carries the panel hosts it; the other
                // wand keeps its own prompt.
                flags |= leftOwns ? kLeftHostsListed : kRightHostsListed;
            } else {
                flags |= (right == listed ? kRightHostsListed : 0) | (left == listed ? kLeftHostsListed : 0);
            }
            g_listedFlags.store(flags, std::memory_order_relaxed);
            g_listedPick.store(listed, std::memory_order_release);
        }

        void* afterPicksCopied(void* lock, const std::uintptr_t argument)
        {
            // The redirected call passes model + 0x268 (lea 0xA923FD, mov 0xA9240D).
            chooseListedPick(static_cast<std::byte*>(lock) - kModelLockOffset);
            return g_lockAcquire(lock, argument);
        }

        // Native answer (true) whenever nothing is listed.
        bool primaryRolloverHostsPanel(void*)
        {
            const auto flags = g_listedFlags.load(std::memory_order_relaxed);
            return !(flags & kListed) || (flags & kRightHostsListed);
        }

        // Native answer (false) whenever nothing is listed.
        bool secondaryRolloverHostsPanel(void*)
        {
            const auto flags = g_listedFlags.load(std::memory_order_relaxed);
            return (flags & kListed) && (flags & kLeftHostsListed);
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
            return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(rolloverHostSlotAddress(host)),
                       &current, sizeof(current)) &&
                current == native &&
                native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(native), bytes.data(),
                    bytes.size()) &&
                bytes == host.nativeBytes;
        }

        // Both prompts switch together or neither does.
        void installRolloverHosts()
        {
            if (!rolloverHostSlotMatches(kPrimaryRolloverHost) || !rolloverHostSlotMatches(kSecondaryRolloverHost)) {
                ROCK_LOG_WARN(Init,
                    "Quick loot panel: the rollover host slots do not match; "
                    "the left wand's Search prompt stays visible while the panel lists its container");
                return;
            }
            REL::safe_write(rolloverHostSlotAddress(kPrimaryRolloverHost),
                reinterpret_cast<std::uintptr_t>(&primaryRolloverHostsPanel));
            REL::safe_write(rolloverHostSlotAddress(kSecondaryRolloverHost),
                reinterpret_cast<std::uintptr_t>(&secondaryRolloverHostsPanel));
        }

        float pickDistance(const std::uint32_t handleValue, const RE::NiNode* wand)
        {
            return native_world_ui_anchor::distanceToReference(handleValue, wand).value_or(kNoPick);
        }

        // CloserHand: publishes which hand the next model update gives the
        // panel when both wands pick lootable containers.
        void updatePreference(const float right, const float left)
        {
            const bool onLeft = (g_listedFlags.load(std::memory_order_relaxed) & kLeftOwnsListed) != 0;
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
                    "Quick loot panel held on the {} hand: the other hand is {:.1f} units closer, under the {:.1f}-unit "
                    "switch margin",
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

        RE::NiAVObject* modelRoot(const std::uintptr_t modelGlobal)
        {
            auto* const model = *REL::Relocation<std::byte**>{ REL::Offset(modelGlobal) };
            return model ? *reinterpret_cast<RE::NiAVObject**>(model + kModelRootOffset) : nullptr;
        }

        // The root's pose under the left wand mirrors the one it has under the
        // right wand: carrier = mirror(attach * root) * root^-1. False when the
        // carrier cannot be created; the root is then not moved.
        bool placeMirroredOnLeft(RE::NiAVObject* root, const RE::NiNode* handAttach, RE::NiNode* leftWand,
            RE::NiPointer<RE::NiNode>& carrier, bool& carrierFailed, const char* carrierName)
        {
            if (!attachCarrier(carrier, carrierFailed, leftWand, carrierName)) {
                return false;
            }
            const auto onRight = transform_math::composeTransforms(handAttach->local, root->local);
            carrier->local =
                transform_math::composeTransforms(mirroredAcrossX(onRight), transform_math::invertTransform(root->local));
            moveTo(root, carrier.get());
            return true;
        }

        // CloserHand: the panel rides the hand that owns the listing.
        void placeOnHand(RE::NiAVObject* root, RE::NiNode* handAttach, RE::NiNode* leftWand)
        {
            const std::uint32_t listed = g_listedPick.load(std::memory_order_acquire);
            const auto flags = g_listedFlags.load(std::memory_order_relaxed);
            const bool onContainer = g_containerCarrier && root->parent == g_containerCarrier.get();
            if (!listed) {
                // Nothing lists, so the panel is hidden; it stays on the hand it was on.
                if (onContainer) {
                    moveTo(root, handAttach);
                }
                return;
            }
            if (!(flags & kLeftOwnsListed) || !leftWand ||
                !placeMirroredOnLeft(root, handAttach, leftWand, g_leftCarrier, g_leftCarrierFailed, "left-hand")) {
                moveTo(root, handAttach);
            }
        }

        // AtContainer: the panel stands at the container it lists.
        void placeAtContainer(RE::NiAVObject* root, RE::NiNode* handAttach, RE::NiNode* room, const RE::NiNode* eye)
        {
            const std::uint32_t listed = g_listedPick.load(std::memory_order_acquire);
            const bool onLeft = g_leftCarrier && root->parent == g_leftCarrier.get();
            if (!listed) {
                // Nothing lists, so the panel is hidden; it stays where it was.
                if (onLeft) {
                    moveTo(root, handAttach);
                }
                return;
            }
            const RE::NiPoint3 eyeRight{ eye->world.rotate.entry[0][0], eye->world.rotate.entry[0][1],
                eye->world.rotate.entry[0][2] };
            const auto pose = native_world_ui_anchor::uprightPoseAt(listed, eye->world.translate, eyeRight, kLootFace,
                handAttach->world.scale * root->local.scale, native_world_ui_anchor::boundCenterInRootSpace(*root));
            if (!pose) {
                moveTo(root, handAttach);
                ROCK_LOG_SAMPLE_DEBUG(Hand, 1000,
                    "Quick loot panel: the listed container has no reference; the panel is on the right hand");
                return;
            }
            if (!attachCarrier(g_containerCarrier, g_containerCarrierFailed, room, "container")) {
                moveTo(root, handAttach);
                return;
            }
            g_containerCarrier->local = native_world_ui_anchor::carrierLocalFor(*pose, root->local, room->world);
            moveTo(root, g_containerCarrier.get());
        }

        bool isUnder(const RE::NiAVObject* root, const RE::NiPointer<RE::NiNode>& carrier)
        {
            return carrier && root->parent == carrier.get();
        }

        /*
         * The button hints go wherever the panel lists, so the hints for the
         * container stay beside it. At the container they share the panel's
         * carrier, which keeps them at the offset from the panel they have on
         * the hand; on the left hand they are mirrored like the panel. While
         * nothing lists they are on the right hand, where the game shows its
         * other hints.
         */
        void placeHints(RE::NiAVObject* hints, const RE::NiAVObject* panel, RE::NiNode* handAttach,
            RE::NiNode* leftWand)
        {
            const bool listed = g_listedPick.load(std::memory_order_acquire) != 0;
            if (listed && panel && isUnder(panel, g_containerCarrier)) {
                moveTo(hints, g_containerCarrier.get());
                return;
            }
            if (listed && panel && isUnder(panel, g_leftCarrier) && leftWand &&
                placeMirroredOnLeft(hints, handAttach, leftWand, g_leftHintCarrier, g_leftHintCarrierFailed,
                    "left-hand hints")) {
                return;
            }
            moveTo(hints, handAttach);
        }

        void placePanel(RE::NiAVObject* root, const f4vr::PlayerNodes& nodes, const Placement placement)
        {
            auto* const handAttach = nodes.primaryUIAttachNode;
            switch (placement) {
            case Placement::RightHand:
                moveTo(root, handAttach);
                return;
            case Placement::CloserHand:
                placeOnHand(root, handAttach, nodes.SecondaryWandNode);
                return;
            case Placement::AtContainer:
                if (!nodes.roomnode || !nodes.HmdNode) {
                    moveTo(root, handAttach);
                    return;
                }
                placeAtContainer(root, handAttach, nodes.roomnode, nodes.HmdNode);
                return;
            }
        }

        void place(const f4vr::PlayerNodes& nodes, const Placement placement)
        {
            auto* const handAttach = nodes.primaryUIAttachNode;
            if (!handAttach) {
                return;
            }
            // A root on any other parent belongs to a native rebuild or another
            // owner; it is left alone.
            auto* const panel = modelRoot(kLootModel);
            const bool panelOwned = panel &&
                (panel->parent == handAttach || isUnder(panel, g_containerCarrier) || isUnder(panel, g_leftCarrier));
            if (panelOwned) {
                placePanel(panel, nodes, placement);
            }
            if (!g_hintsFollow) {
                return;
            }
            auto* const hints = modelRoot(kHintModel);
            if (hints &&
                (hints->parent == handAttach || isUnder(hints, g_containerCarrier) ||
                    isUnder(hints, g_leftHintCarrier))) {
                placeHints(hints, panelOwned ? panel : nullptr, handAttach, nodes.SecondaryWandNode);
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
            if (!matches(expected)) {
                ROCK_LOG_WARN(Init,
                    "Quick loot panel: the quick container pick copy at 0x{:X} does not match; "
                    "the panel stays on the right hand",
                    expected.address);
                return false;
            }
        }
        const auto callAddress = REL::Offset(kPicksCopiedLockCall).address();
        std::array<std::uint8_t, 5> call{};
        if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(callAddress), call.data(),
                call.size()) ||
            call[0] != 0xE8) {
            ROCK_LOG_WARN(Init, "Quick loot panel: 0x{:X} is not a call; the panel stays on the right hand",
                kPicksCopiedLockCall);
            return false;
        }
        // The model lock, or another plugin's detour, which the hook calls through
        // (write_call returns whichever is there).
        const auto lock = RPS::Runtime::HookChain::resolveCall(callAddress, REL::Offset(kSpinLockAcquire).address());
        if (!lock) {
            ROCK_LOG_WARN(Init,
                "Quick loot panel: call 0x{:X} reaches neither the model lock nor another plugin's detour; "
                "the panel stays on the right hand",
                kPicksCopiedLockCall);
            return false;
        }
        if (lock.foreign())
            ROCK_LOG_INFO(Init, "Quick loot model lock call already hooked by {}; ROCK calls through it", lock.owner);

        try {
            g_lockAcquire =
                reinterpret_cast<SpinLockAcquire>(F4SE::GetTrampoline().write_call<5>(callAddress, &afterPicksCopied));
        } catch (...) {
            REL::safe_write(callAddress, call.data(), call.size());
            g_lockAcquire = nullptr;
            ROCK_LOG_ERROR(Init, "Quick loot panel installation failed; the panel stays on the right hand");
            return false;
        }
        installRolloverHosts();
        g_hintsFollow = true;
        for (const auto& expected : kExpectedHintModel) {
            if (!matches(expected)) {
                g_hintsFollow = false;
                ROCK_LOG_WARN(Init,
                    "Quick loot panel: the button hint model code at 0x{:X} does not match; "
                    "the hints stay on the right hand",
                    expected.address);
                break;
            }
        }
        ROCK_LOG_INFO(Init, "Quick loot panel installed; Grabbing.QuickLootPanelPlacement chooses where it appears");
        return true;
    }

    void update(const Placement placement) noexcept
    {
        if (!g_lockAcquire) {
            return;
        }
        if (g_appliedPlacement != placement) {
            g_appliedPlacement = placement;
            ROCK_LOG_INFO(Hand, "Quick loot panel placement: {}", placementName(placement));
        }
        g_placement.store(placement, std::memory_order_relaxed);
        const auto* nodes = f4vr::getPlayerNodes();
        if (!nodes) {
            return;
        }
        if (placement == Placement::CloserHand) {
            updatePreference(pickDistance(g_rightPick.load(std::memory_order_relaxed), nodes->primaryWandNode),
                pickDistance(g_leftPick.load(std::memory_order_relaxed), nodes->SecondaryWandNode));
        }
        place(*nodes, placement);
    }
}
