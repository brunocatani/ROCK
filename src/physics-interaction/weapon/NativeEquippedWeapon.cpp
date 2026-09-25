#include "physics-interaction/weapon/NativeEquippedWeapon.h"
#include "physics-interaction/weapon/NativeEquippedModelSlot.h"

#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "rock_support/Fo4VrRuntime.h"
#include <array>

namespace rock::native_equipped_weapon
{
    namespace
    {
        using ReadItem = bool (*)(RE::AIProcess*, std::uint32_t, RE::EquippedItem*);
        using ResolveIndex = std::uint32_t* (*)(RE::Actor*, std::uint32_t*, RE::BGSEquipSlot*);

        template <std::size_t N>
        bool entry(std::uintptr_t rva, const std::array<std::uint8_t, N>& expected) noexcept
        {
            std::array<std::uint8_t, N> bytes{};
            const bool valid = native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(REL::Offset(rva).address()),
                bytes.data(), bytes.size()) && bytes == expected;
            if (!valid) ROCK_LOG_ERROR(Init, "Native equipped contract unavailable rva={:X}", rva);
            return valid;
        }
    }

    bool ready() noexcept
    {
        static const bool valid = [] {
            // Raw VR evidence: 1C8150 and 1C7FF0 both resolve and access the
            // indexed biped record. 1CC340 independently walks its graph slot.
            // E72DA0/3E6840 establish that equipment indices are slot-derived.
            return REL::Module::IsVR() && REL::Module::get().version() == F4SE::RUNTIME_VR_1_2_72 &&
                entry(0xE803D0, std::array<std::uint8_t, 8>{0x48,0x89,0x5C,0x24,0x18,0x48,0x89,0x6C}) &&
                entry(0x3E6840, std::array<std::uint8_t, 8>{0x48,0x89,0x5C,0x24,0x18,0x56,0x48,0x83}) &&
                entry(0x146340, std::array<std::uint8_t, 10>{0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x20}) &&
                entry(0x1CAFC0, std::array<std::uint8_t, 9>{0x48,0x83,0xEC,0x28,0x48,0x85,0xD2,0x74,0x2C}) &&
                entry(0xEC4B90, std::array<std::uint8_t, 8>{0x40,0x53,0x48,0x83,0xEC,0x30,0x41,0x8B}) &&
                entry(0xDAB8F0, std::array<std::uint8_t, 8>{0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x6C});
        }();
        return valid;
    }

    bool read(std::uint32_t index, Snapshot& result) noexcept
    {
        result = {};
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!ready() || index > 1 || !player || !player->currentProcess ||
            !player->currentProcess->middleHigh || player->currentProcess->middleHigh->equippedItems.size() > 16) return false;
        unsigned matches{};
        for (const auto& record : player->currentProcess->middleHigh->equippedItems)
            if (record.equipIndex.index == index && record.item.object) ++matches;
        if (!matches) return false;
        if (matches != 1) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Native equipped read refused index={} stage=record-uniqueness records={}", index, matches);
            return false;
        }
        if (!reinterpret_cast<ReadItem>(REL::Offset(0xE803D0).address())(player->currentProcess, index, &result.item)) return false;
        auto& item = result.item;
        std::uintptr_t table{};
        if (!item.item.object || item.item.object->formType != RE::ENUM_FORM_ID::kWEAP ||
            item.equipIndex.index != index || !item.data ||
            !native_memory::tryReadField(item.data.get(), 0, table) || table != REL::Offset(0x2D7FCF8).address()) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Native equipped read refused index={} stage=weapon-data-vtable form={:08X} data={:p} vtable={:#x}",
                index, item.item.object ? item.item.object->formID : 0u, static_cast<void*>(item.data.get()), table);
            return false;
        }
        result.identity = {item.item.object->formID, index,
            reinterpret_cast<std::uintptr_t>(item.item.instanceData.get()), reinterpret_cast<std::uintptr_t>(item.data.get())};
        result.equipped = true;
        result.modelSlot = native_equipped_model_slot::resolve(player, item.item.object, index);
        result.biped = player->firstPersonBipedAnim;
        if (!result.biped || result.modelSlot >= static_cast<std::uint32_t>(std::to_underlying(RE::BIPED_OBJECT::kTotal))) return true;
        const auto& model = result.biped->object[result.modelSlot];
        if (model.parent.object != item.item.object || model.parent.instanceData.get() != item.item.instanceData.get() ||
            !model.partClone) return true;
        result.model = model.partClone;
        result.node.reset(result.model->parent);
        result.attached = static_cast<bool>(result.node);
        return true;
    }

    bool matches(const Identity& identity) noexcept
    {
        Snapshot current;
        return read(identity.index, current) && current.identity == identity;
    }

    bool slotEmpty(std::uint32_t index) noexcept
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!ready() || index > 1 || !player || !player->currentProcess || !player->currentProcess->middleHigh) return false;
        const auto& items = player->currentProcess->middleHigh->equippedItems;
        if (items.size() > 16) return false;
        for (const auto& item : items) if (item.equipIndex.index == index && item.item.object) return false;
        return true;
    }

    RE::BGSEquipSlot* handSlot(std::uint32_t index) noexcept
    {
        if (!ready() || index > 1) return nullptr;
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto* slot = RE::TESForm::GetFormByID<RE::BGSEquipSlot>(index ? 0x13F43 : 0x13F42);
        if (!player || !slot) return nullptr;
        std::uint32_t resolved = UINT32_MAX;
        reinterpret_cast<ResolveIndex>(REL::Offset(0x3E6840).address())(player, &resolved, slot);
        return resolved == index ? slot : nullptr;
    }

    bool requestAttach(const Identity& identity) noexcept
    {
        Snapshot current;
        if (!read(identity.index, current) || current.identity != identity) return false;
        void* manager{};
        if (!native_memory::tryReadValue(reinterpret_cast<void* const*>(REL::Offset(0x5B279E0).address()), manager) || !manager) return false;
        using Attach = void (*)(void*, RE::PlayerCharacter*, RE::BGSObjectInstance*, std::uint32_t);
        reinterpret_cast<Attach>(REL::Offset(0xDAB8F0).address())(manager, RE::PlayerCharacter::GetSingleton(),
            &current.item.item, identity.index);
        return true;
    }

    std::uint16_t inventorySlotMask(const RE::BGSObjectInstance& item, const RE::BGSEquipSlot* slot) noexcept
    {
        if (!ready() || !item.object || !slot) return 0;
        // The stack bit is relative to the item's authored equip slot, NOT
        // the actor's native equipment index. E0FBC3 checks it; 1AF0C6 derives
        // the same bit before the inventory write at 1AF100.
        using Resolve = std::uint32_t (*)(const RE::BGSObjectInstance*, const RE::BGSEquipSlot*);
        const auto bit = reinterpret_cast<Resolve>(REL::Offset(0x146340).address())(&item, slot);
        return bit < 3 ? static_cast<std::uint16_t>(1u << bit) : 0;
    }

    bool restoreMagazine(const Identity& identity, std::uint32_t ammo, std::uint32_t loaded) noexcept
    {
        Snapshot current;
        if (!read(identity.index, current) || current.identity != identity) return false;
        const auto* data = static_cast<const RE::EquippedWeaponData*>(current.item.data.get());
        if (!data->ammo || data->ammo->formID != ammo) return false;
        using SetCount = void (*)(RE::AIProcess*, std::uint32_t, std::uint32_t);
        reinterpret_cast<SetCount>(REL::Offset(0xEC4B90).address())(RE::PlayerCharacter::GetSingleton()->currentProcess, identity.index, loaded);
        return data->ammoCount == loaded;
    }
}
