#include "physics-interaction/native/player/NativeActivatePrompts.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <optional>

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/player/NativeWorldUiAnchor.h"
#include "rock_support/Fo4VrRuntime.h"

namespace rock::native_activate_prompts
{
    namespace
    {
        using native_world_ui_anchor::attachCarrier;
        using native_world_ui_anchor::moveTo;

        /*
         * FO4VR 1.2.72, raw disassembly. The WSInteractRolloverPrimaryModel
         * constructor 0xC83690 stores itself at 0x5ACBE78 (0xC836DE) and the
         * WSInteractRolloverSecondaryModel constructor 0xC83DF0 at 0x5ACBEA8
         * (0xC83E3E); both build a WSUIModel on Data\Meshes\world_Rollover.nif.
         * The player VR node builder 0xEF21A0 attaches the primary model to
         * the primary wand node, player +0x6F0 (0xEF3C5F..0xEF3C70), and the
         * secondary one to the secondary wand node, player +0x768
         * (0xEF45D1..0xEF45E2), both through 0xC90220, which attaches model
         * +0x10. The rollover cull functions show which pick each prompt
         * names: 0xEF8940 reads the primary model with the primary wand
         * caster 0x5AC72B0 (0xEF89EE), 0xEF8820 the secondary model with the
         * secondary caster 0x5AC7F10 (0xEF88D4), both through 0x9D0CE0, which
         * locks caster +0x70 and copies the handle its vtable slot 2 returns.
         */
        constexpr std::uintptr_t kPrimaryModel = 0x5ACBE78;
        constexpr std::uintptr_t kSecondaryModel = 0x5ACBEA8;
        constexpr std::ptrdiff_t kModelRootOffset = 0x10;
        constexpr std::uintptr_t kPrimaryWandCaster = 0x5AC72B0;
        constexpr std::uintptr_t kSecondaryWandCaster = 0x5AC7F10;
        constexpr std::uintptr_t kReadCasterPick = 0x9D0CE0;
        // The engine's invalid reference handle, compared at 0xA96186.
        constexpr std::uintptr_t kInvalidHandle = 0x392E884;

        struct ExpectedBytes
        {
            std::uintptr_t address;
            std::array<std::uint8_t, 15> bytes;
            std::size_t size;
        };

        constexpr std::array<ExpectedBytes, 7> kExpected{ {
            // mov [rip -> 0x5ACBE78], rax
            { 0xC836DE, { 0x48, 0x89, 0x05, 0x93, 0x87, 0xE4, 0x04 }, 7 },
            // mov [rip -> 0x5ACBEA8], rax
            { 0xC83E3E, { 0x48, 0x89, 0x05, 0x63, 0x80, 0xE4, 0x04 }, 7 },
            // mov rdx, [r14+0x6F0]; mov rcx, [rip -> 0x5ACBE78]
            { 0xEF3C5F, { 0x49, 0x8B, 0x96, 0xF0, 0x06, 0x00, 0x00, 0x48, 0x8B, 0x0D, 0x0B, 0x82, 0xBD, 0x04 }, 14 },
            // mov rdx, [r14+0x768]; mov rcx, [rip -> 0x5ACBEA8]
            { 0xEF45D1, { 0x49, 0x8B, 0x96, 0x68, 0x07, 0x00, 0x00, 0x48, 0x8B, 0x0D, 0xC9, 0x78, 0xBD, 0x04 }, 14 },
            // mov rcx, [rip -> 0x5AC7F10]
            { 0xEF88D4, { 0x48, 0x8B, 0x0D, 0x35, 0xF6, 0xBC, 0x04 }, 7 },
            // mov rcx, [rip -> 0x5AC72B0]
            { 0xEF89EE, { 0x48, 0x8B, 0x0D, 0xBB, 0xE8, 0xBC, 0x04 }, 7 },
            // 0x9D0CE0 prologue
            { 0x9D0CE0, { 0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x10, 0x57, 0x48, 0x83, 0xEC, 0x20 },
                15 },
        } };

        /*
         * When both wands point at the same object, the hand showing its
         * prompt hands it over only when the other hand is this much closer
         * to the object. Without the margin the prompt flips hands on
         * near-equal distances. Game units; about 7 cm.
         */
        constexpr float kOwnerSwitchMargin = 5.0f;

        using ReadCasterPick = std::uint32_t* (*)(void*, std::uint32_t*);

        // Index 0 is the primary (right) wand, 1 the secondary (left) one;
        // ROCK holds bLeftHandedMode:VR off.
        constexpr std::size_t kRight = 0;
        constexpr std::size_t kLeft = 1;
        constexpr std::array<std::uintptr_t, 2> kModels{ kPrimaryModel, kSecondaryModel };
        constexpr std::array<std::uintptr_t, 2> kCasters{ kPrimaryWandCaster, kSecondaryWandCaster };
        constexpr std::array<const char*, 2> kObjectCarrierNames{ "right prompt", "left prompt" };

        bool g_installed{ false };
        bool g_attempted{ false };
        // Game thread only. Never destroyed: an engine object released in DLL
        // detach raises R6025 during ExitProcess.
        RE::NiPointer<RE::NiNode>& g_hiddenCarrier = *new RE::NiPointer<RE::NiNode>;
        std::array<RE::NiPointer<RE::NiNode>, 2>& g_objectCarriers = *new std::array<RE::NiPointer<RE::NiNode>, 2>;
        bool g_hiddenCarrierFailed{ false };
        std::array<bool, 2> g_objectCarrierFailed{};
        // The wand that shows the prompt both wands share, while they share one.
        std::optional<std::size_t> g_sharedOwner;
        std::optional<Placement> g_appliedPlacement;

        const char* placementName(const Placement placement)
        {
            switch (placement) {
            case Placement::BothHands:
                return "on both hands (game default)";
            case Placement::CloserHand:
                return "on the closer hand when both point at one object";
            case Placement::AtObject:
                return "at the object";
            }
            return "unknown";
        }

        RE::NiAVObject* modelRoot(const std::uintptr_t model)
        {
            auto* const instance = *REL::Relocation<std::byte**>{ REL::Offset(model) };
            return instance ? *reinterpret_cast<RE::NiAVObject**>(instance + kModelRootOffset) : nullptr;
        }

        std::uint32_t readPick(const std::uintptr_t casterGlobal)
        {
            void* const caster = *REL::Relocation<void**>{ REL::Offset(casterGlobal) };
            if (!caster) {
                return 0;
            }
            static REL::Relocation<ReadCasterPick> read{ REL::Offset(kReadCasterPick) };
            std::uint32_t handle{};
            read(caster, &handle);
            const std::uint32_t invalid = *REL::Relocation<const std::uint32_t*>{ REL::Offset(kInvalidHandle) };
            return handle == invalid ? 0 : handle;
        }

        std::size_t chooseSharedOwner(const std::uint32_t pick, const std::array<RE::NiNode*, 2>& wands)
        {
            const auto right = native_world_ui_anchor::distanceToReference(pick, wands[kRight]);
            const auto left = native_world_ui_anchor::distanceToReference(pick, wands[kLeft]);
            if (!right || !left) {
                return g_sharedOwner.value_or(right ? kRight : kLeft);
            }
            if (!g_sharedOwner) {
                return *right <= *left ? kRight : kLeft;
            }
            const bool onLeft = *g_sharedOwner == kLeft;
            const float otherCloserBy = onLeft ? *left - *right : *right - *left;
            if (otherCloserBy > kOwnerSwitchMargin) {
                return onLeft ? kRight : kLeft;
            }
            return *g_sharedOwner;
        }

        void hide(RE::NiAVObject* root, RE::NiNode* wand, RE::NiNode* room)
        {
            if (!room || !attachCarrier(g_hiddenCarrier, g_hiddenCarrierFailed, room, "hidden prompt")) {
                moveTo(root, wand);
                return;
            }
            // App-culled: nothing under the carrier is drawn.
            g_hiddenCarrier->flags.flags |= 1;
            moveTo(root, g_hiddenCarrier.get());
        }

        void placeAtObject(const std::size_t index, RE::NiAVObject* root, RE::NiNode* wand, const std::uint32_t pick,
            RE::NiNode* room, const RE::NiNode* eye)
        {
            if (!pick) {
                // Nothing picked, so the prompt is hidden; it stays where it was.
                if (g_hiddenCarrier && root->parent == g_hiddenCarrier.get()) {
                    moveTo(root, wand);
                }
                return;
            }
            if (!room || !eye) {
                moveTo(root, wand);
                return;
            }
            const RE::NiPoint3 eyeRight{ eye->world.rotate.entry[0][0], eye->world.rotate.entry[0][1],
                eye->world.rotate.entry[0][2] };
            const auto pose = native_world_ui_anchor::poseFacingEye(
                pick, eye->world.translate, eyeRight, wand->world.scale * root->local.scale);
            auto& carrier = g_objectCarriers[index];
            if (!pose || !attachCarrier(carrier, g_objectCarrierFailed[index], room, kObjectCarrierNames[index])) {
                moveTo(root, wand);
                return;
            }
            carrier->local = native_world_ui_anchor::carrierLocalFor(*pose, root->local, room->world);
            moveTo(root, carrier.get());
        }

        // Only roots on their wand or on ROCK's carriers are ROCK's to move;
        // any other parent is a native rebuild or another owner.
        bool ownsPlacement(const std::size_t index, const RE::NiAVObject* root, const RE::NiNode* wand)
        {
            const auto* parent = root->parent;
            return parent && (parent == wand || (g_hiddenCarrier && parent == g_hiddenCarrier.get()) ||
                                 (g_objectCarriers[index] && parent == g_objectCarriers[index].get()));
        }
    }

    bool install() noexcept
    {
        if (g_installed) {
            return true;
        }
        if (g_attempted) {
            return false;
        }
        g_attempted = true;
        for (const auto& expected : kExpected) {
            std::array<std::uint8_t, 15> live{};
            if (!native_memory::guardedCopyFromMemory(
                    reinterpret_cast<const void*>(REL::Offset(expected.address).address()), live.data(),
                    expected.size) ||
                std::memcmp(live.data(), expected.bytes.data(), expected.size) != 0) {
                ROCK_LOG_WARN(Init, "Activate prompts: the native code at 0x{:X} does not match; the prompts stay native",
                    expected.address);
                return false;
            }
        }
        g_installed = true;
        ROCK_LOG_INFO(Init, "Activate prompts installed; Grabbing.ActivatePromptPlacement chooses where they appear");
        return true;
    }

    void update(const Placement placement) noexcept
    {
        if (!g_installed) {
            return;
        }
        if (g_appliedPlacement != placement) {
            g_appliedPlacement = placement;
            ROCK_LOG_INFO(Hand, "Activate prompt placement: {}", placementName(placement));
        }
        const auto* nodes = f4vr::getPlayerNodes();
        if (!nodes) {
            return;
        }
        const std::array<RE::NiNode*, 2> wands{ nodes->primaryWandNode, nodes->SecondaryWandNode };
        const std::array<std::uint32_t, 2> picks{ readPick(kCasters[kRight]), readPick(kCasters[kLeft]) };

        const bool shared = picks[kRight] && picks[kRight] == picks[kLeft];
        if (placement != Placement::BothHands && shared && wands[kRight] && wands[kLeft]) {
            g_sharedOwner = chooseSharedOwner(picks[kRight], wands);
        } else {
            g_sharedOwner.reset();
        }

        for (std::size_t index = 0; index < 2; ++index) {
            auto* const root = modelRoot(kModels[index]);
            auto* const wand = wands[index];
            if (!root || !wand || !ownsPlacement(index, root, wand)) {
                continue;
            }
            const bool duplicate = g_sharedOwner && *g_sharedOwner != index;
            switch (placement) {
            case Placement::BothHands:
                moveTo(root, wand);
                break;
            case Placement::CloserHand:
                if (duplicate) {
                    hide(root, wand, nodes->roomnode);
                } else {
                    moveTo(root, wand);
                }
                break;
            case Placement::AtObject:
                if (duplicate) {
                    hide(root, wand, nodes->roomnode);
                } else {
                    placeAtObject(index, root, wand, picks[index], nodes->roomnode, nodes->HmdNode);
                }
                break;
            }
        }
    }
}
