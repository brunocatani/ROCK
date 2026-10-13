#include "physics-interaction/weapon/NativePipboyHandMarkers.h"

#include "physics-interaction/weapon/NativeHandEquipment.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include "RE/Bethesda/BGSInventoryItem.h"
#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/TESForms.h"
#include <REL/Relocation.h>
#include <Windows.h>
#include <array>
#include <string>

namespace rock::native_pipboy_hand_markers
{
    namespace
    {
        // FO4VR 1.2.72, raw disassembly (ghidra_fo4vr):
        // 140C21450 fills one Pip-Boy inventory entry (PipboyInventoryData,
        //   sole caller 140C20E00): "text" from 1401AE5F0(item, stack->extra)
        //   through 140C28B80(object, key 145ACB0A8, BSFixedStringCS), then
        //   "equipState" from 140C1E9A0(item, stack) at 140C215BA.
        // 140C45F00, used by every member setter, first removes an existing
        //   member of the same name (140C45870), so setting "text" again
        //   replaces the entry's text.
        constexpr std::uintptr_t kPopulate = 0xC21450;
        constexpr std::uintptr_t kEquipState = 0xC1E9A0;
        constexpr std::uintptr_t kDisplayName = 0x1AE5F0;
        constexpr std::uintptr_t kSetString = 0xC28B80;
        constexpr std::uintptr_t kMakeString = 0x1BC2050;
        constexpr std::uintptr_t kFreeString = 0x1BC20F0;
        constexpr std::uintptr_t kTextKey = 0x5ACB0A8;

        using Stack = RE::BGSInventoryItem::Stack;
        using Populate = void (*)(void*, RE::BGSInventoryItem*, Stack*, void*, void*, std::uint32_t);
        using EquipState = std::uint32_t (*)(RE::BGSInventoryItem*, Stack*);
        using DisplayName = const char* (*)(RE::BGSInventoryItem*, RE::ExtraDataList*);
        using SetString = void* (*)(void*, const void*, void*);
        using MakeString = void* (*)(void*, const char*);
        using FreeString = void (*)(void*);

        Populate s_populate = nullptr;
        EquipState s_equipState = nullptr;
        // The stack whose equipped square is replaced during the populate
        // call running on this thread (nested calls restore the outer one).
        thread_local const Stack* t_marked = nullptr;

        template <std::size_t N>
        bool matches(std::uintptr_t rva, const std::array<std::uint8_t, N>& expected) noexcept
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Offset(rva).address()),
                actual.data(), actual.size()) && actual == expected;
        }

        template <class T>
        T native(std::uintptr_t rva) noexcept { return reinterpret_cast<T>(REL::Offset(rva).address()); }

        native_hand_equipment::WornHand handOf(RE::BGSInventoryItem* item, const Stack* stack) noexcept
        {
            if (!item || !stack || !stack->IsEquipped() || !item->object || !item->object->IsWeapon())
                return native_hand_equipment::WornHand::None;
            std::uintptr_t instance = 0;
            if (const auto* extra = stack->extra.get()) {
                if (const auto* data = extra->GetByType<RE::ExtraInstanceData>())
                    instance = reinterpret_cast<std::uintptr_t>(data->data.get());
            }
            return native_hand_equipment::wornWeaponHand(item->object->formID, instance);
        }

        std::uint32_t equipState(RE::BGSInventoryItem* item, Stack* stack)
        {
            return stack && stack == t_marked ? 0u : s_equipState(item, stack);
        }

        void populate(void* self, RE::BGSInventoryItem* item, Stack* stack, void* object, void* flags, std::uint32_t mode)
        {
            auto hand = native_hand_equipment::WornHand::None;
            try {
                hand = handOf(item, stack);
            } catch (...) {
                OutputDebugStringA("ROCK: Pip-Boy hand marker lookup failed\n");
            }
            const auto* previous = t_marked;
            t_marked = hand != native_hand_equipment::WornHand::None ? stack : nullptr;
            s_populate(self, item, stack, object, flags, mode);
            t_marked = previous;
            if (hand == native_hand_equipment::WornHand::None || !object) return;
            try {
                const char* name = native<DisplayName>(kDisplayName)(item, stack->extra.get());
                std::string text = hand == native_hand_equipment::WornHand::Left ? "L  " : "R  ";
                text += name ? name : "";
                alignas(8) std::array<std::byte, 8> value{};
                native<MakeString>(kMakeString)(value.data(), text.c_str());
                native<SetString>(kSetString)(object, reinterpret_cast<const void*>(REL::Offset(kTextKey).address()), value.data());
                native<FreeString>(kFreeString)(value.data());
            } catch (...) {
                OutputDebugStringA("ROCK: Pip-Boy hand marker text failed\n");
            }
        }
    }

    bool install() noexcept
    {
        // Helpers called directly, and the populate body that proves they are
        // the text path: the name call at 140C21519 and the "text" key LEA at
        // 140C2152B (rel32 resolves to 145ACB0A8).
        const bool helpers =
            matches(kDisplayName, std::to_array<std::uint8_t>({0x40,0x53,0x48,0x83,0xEC,0x20,0x48,0x8B,0x19})) &&
            matches(kSetString, std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x6C,0x24,0x10})) &&
            matches(kMakeString, std::to_array<std::uint8_t>({0x40,0x53,0x48,0x83,0xEC,0x20,0x48,0x8B,0xD9})) &&
            matches(kFreeString, std::to_array<std::uint8_t>({0xE9,0xEB,0x07,0x00,0x00})) &&
            matches(0xC21519, std::to_array<std::uint8_t>({0xE8,0xD2,0xD0,0x58,0xFF})) &&
            matches(0xC2152B, std::to_array<std::uint8_t>({0x48,0x8D,0x15,0x76,0x9B,0xEA,0x04})) &&
            matches(0xC215BA, std::to_array<std::uint8_t>({0xE8,0xE1,0xD3,0xFF,0xFF}));
        if (!helpers) {
            ROCK_LOG_WARN(Weapon, "Pip-Boy hand markers unavailable: native inventory entry layout differs");
            return false;
        }
        constexpr auto equipStatePrefix = std::to_array<std::uint8_t>({
            0x48,0x89,0x74,0x24,0x10,0x57,0x48,0x83,0xEC,0x50,0x48,0x8B,0xF2,0x48,0x8B,0xF9});
        constexpr auto populatePrefix = std::to_array<std::uint8_t>({
            0x48,0x89,0x5C,0x24,0x18,0x48,0x89,0x4C,0x24,0x08,0x55,0x56,0x57,0x41,0x54});
        void* original = nullptr;
        if (!entry_trampoline_hook::install("Pip-Boy hand marker equip state", kEquipState, equipStatePrefix.data(),
                equipStatePrefix.size(), reinterpret_cast<void*>(&equipState), original)) return false;
        s_equipState = reinterpret_cast<EquipState>(original);
        // The equip-state hook only acts inside a marked populate call, so it
        // stays inert if the populate hook below cannot be installed.
        if (!entry_trampoline_hook::install("Pip-Boy hand marker entry", kPopulate, populatePrefix.data(),
                populatePrefix.size(), reinterpret_cast<void*>(&populate), original)) return false;
        s_populate = reinterpret_cast<Populate>(original);
        ROCK_LOG_INFO(Weapon, "Pip-Boy hand markers installed");
        return true;
    }
}
