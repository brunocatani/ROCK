#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace rock::native_weapon_qualification
{
    // Observations, not ownership or permission to equip/fire. Address values
    // survive only as log/comparison witnesses and are never reused for access.
    constexpr std::size_t kMaximumSlots = 16;
    constexpr std::size_t kMaximumEquipped = 16;
    constexpr std::size_t kMaximumParents = 8;

    enum class Stage : std::uint8_t
    {
        Complete, NativeContract, Player, VisualRace, Process, MiddleHigh,
        RaceForm, SlotArray, SlotLimit, SlotEntry, SlotForm, SlotName,
        ParentArray, ParentLimit, ParentEntry, ParentForm,
        EquippedArray, EquippedLimit, EquippedEntry, EquippedForm,
        EquippedSlot, WeaponData, AmmoForm, ChangedDuringCapture
    };

    inline const char* stageName(Stage stage) noexcept
    {
        switch (stage) {
#define ROCK_QUALIFICATION_STAGE(name) case Stage::name: return #name;
            ROCK_QUALIFICATION_STAGE(Complete)
            ROCK_QUALIFICATION_STAGE(NativeContract)
            ROCK_QUALIFICATION_STAGE(Player)
            ROCK_QUALIFICATION_STAGE(VisualRace)
            ROCK_QUALIFICATION_STAGE(Process)
            ROCK_QUALIFICATION_STAGE(MiddleHigh)
            ROCK_QUALIFICATION_STAGE(RaceForm)
            ROCK_QUALIFICATION_STAGE(SlotArray)
            ROCK_QUALIFICATION_STAGE(SlotLimit)
            ROCK_QUALIFICATION_STAGE(SlotEntry)
            ROCK_QUALIFICATION_STAGE(SlotForm)
            ROCK_QUALIFICATION_STAGE(SlotName)
            ROCK_QUALIFICATION_STAGE(ParentArray)
            ROCK_QUALIFICATION_STAGE(ParentLimit)
            ROCK_QUALIFICATION_STAGE(ParentEntry)
            ROCK_QUALIFICATION_STAGE(ParentForm)
            ROCK_QUALIFICATION_STAGE(EquippedArray)
            ROCK_QUALIFICATION_STAGE(EquippedLimit)
            ROCK_QUALIFICATION_STAGE(EquippedEntry)
            ROCK_QUALIFICATION_STAGE(EquippedForm)
            ROCK_QUALIFICATION_STAGE(EquippedSlot)
            ROCK_QUALIFICATION_STAGE(WeaponData)
            ROCK_QUALIFICATION_STAGE(AmmoForm)
            ROCK_QUALIFICATION_STAGE(ChangedDuringCapture)
#undef ROCK_QUALIFICATION_STAGE
        }
        return "Unknown";
    }

    struct Slot
    {
        std::uint32_t form{};
        std::uint32_t parentCount{};
        std::array<std::uint32_t, kMaximumParents> parents{};
        std::array<char, 64> node{};
        bool nameTruncated{};
        bool operator==(const Slot&) const = default;
    };

    struct Equipped
    {
        std::uint32_t form{}, slot{}, index{}, ammo{}, loaded{}, attackState{};
        std::uintptr_t instance{}, data{}, muzzle{};
        std::uint8_t formType{};
        bool weaponDataValid{};
        bool operator==(const Equipped&) const = default;
    };

    struct Snapshot
    {
        Stage stage{Stage::Complete};
        std::uintptr_t failedAddress{};
        std::uint32_t failedIndex{}, race{}, slotCount{}, equippedCount{};
        std::array<Slot, kMaximumSlots> slots{};
        std::array<Equipped, kMaximumEquipped> equipped{};
        bool operator==(const Snapshot&) const = default;
    };

    namespace detail
    {
        struct Array
        {
            std::uintptr_t data{};
            std::uint32_t capacity{}, padding{}, size{}, padding2{};
            bool sameStorage(const Array& other) const noexcept
            {
                return data == other.data && capacity == other.capacity && size == other.size;
            }
        };
        struct SlotEntry { std::uintptr_t slot{}, node{}; };
        struct EquippedEntry
        {
            std::uintptr_t form{}, instance{}, slot{};
            std::uint32_t index{}, padding{};
            std::uintptr_t data{};
            bool sameIdentity(const EquippedEntry& other) const noexcept
            {
                return form == other.form && instance == other.instance && slot == other.slot &&
                    index == other.index && data == other.data;
            }
        };
        static_assert(sizeof(Array) == 0x18);
        static_assert(sizeof(SlotEntry) == 0x10);
        static_assert(sizeof(EquippedEntry) == 0x28);

        constexpr bool plausible(std::uintptr_t address) noexcept
        {
            return address >= 0x10000 && address <= 0x00007FFFFFFF0000ull && (address & 7) == 0;
        }
    }

    // Reader performs a guarded copy, never throws, never acquires an engine
    // lock, and takes (address, destination, bytes). Injectable for failure tests.
    // VR witnesses: E803D0/E806B0/E886D0 equipped records; 3E6840/1F4690/
    // 5E8A40->5F7450 race slots; 54EC10/54ECD0 slot parents; EC48C0/EC4B90
    // loaded count; E51860/E51910 attack state; DE85F0 fire node. Full VAs
    // and the live-byte guard owner are recorded in the implementation note.
    template<class Reader>
    Snapshot capture(std::uintptr_t race, std::uintptr_t middleHigh,
        std::uintptr_t weaponDataVtable, Reader&& read) noexcept
    {
        Snapshot result{};
        const auto fail = [&](Stage stage, std::uintptr_t address, std::uint32_t index = 0) {
            result.stage = stage;
            result.failedAddress = address;
            result.failedIndex = index;
            return result;
        };
        const auto form = [&](std::uintptr_t address, std::uint32_t& id, std::uint8_t& type) {
            return detail::plausible(address) && read(address + 0x14, &id, sizeof(id)) &&
                read(address + 0x1A, &type, sizeof(type)) && id != 0;
        };
        const auto array = [&](std::uintptr_t address, detail::Array& out) {
            return detail::plausible(address) && read(address, &out, sizeof(out)) &&
                out.size <= out.capacity && (!out.size || detail::plausible(out.data));
        };
        std::uint8_t type{};
        if (!form(race, result.race, type) || type != 0x11) return fail(Stage::RaceForm, race);
        detail::Array slotArray{}, equippedArray{};
        if (!array(race + 0x5D8, slotArray)) return fail(Stage::SlotArray, race + 0x5D8);
        result.slotCount = slotArray.size;
        if (slotArray.size > kMaximumSlots) return fail(Stage::SlotLimit, slotArray.data);
        for (std::uint32_t i = 0; i < slotArray.size; ++i) {
            detail::SlotEntry raw{};
            const auto address = slotArray.data + i * sizeof(raw);
            if (!read(address, &raw, sizeof(raw))) return fail(Stage::SlotEntry, address, i);
            auto& slot = result.slots[i];
            if (!form(raw.slot, slot.form, type) || type != 0x7C) return fail(Stage::SlotForm, raw.slot, i);
            if (raw.node) {
                if (raw.node < 0x10000 || raw.node > 0x00007FFFFFFF0000ull) return fail(Stage::SlotName, raw.node, i);
                for (std::size_t c = 0; c < slot.node.size(); ++c) {
                    char value{};
                    if (!read(raw.node + c, &value, 1)) return fail(Stage::SlotName, raw.node + c, i);
                    if (c == slot.node.size() - 1) { slot.nameTruncated = value != '\0'; break; }
                    slot.node[c] = value;
                    if (!value) break;
                }
            }
            detail::Array parents{};
            if (!array(raw.slot + 0x20, parents)) return fail(Stage::ParentArray, raw.slot + 0x20, i);
            slot.parentCount = parents.size;
            if (parents.size > kMaximumParents) return fail(Stage::ParentLimit, parents.data, i);
            for (std::uint32_t p = 0; p < parents.size; ++p) {
                std::uintptr_t parent{};
                const auto parentAddress = parents.data + p * sizeof(parent);
                if (!read(parentAddress, &parent, sizeof(parent))) return fail(Stage::ParentEntry, parentAddress, i);
                if (!form(parent, slot.parents[p], type) || type != 0x7C) return fail(Stage::ParentForm, parent, i);
            }
            detail::Array checkParents{};
            detail::SlotEntry checkSlot{};
            if (!array(raw.slot + 0x20, checkParents) || !parents.sameStorage(checkParents) ||
                !read(address, &checkSlot, sizeof(checkSlot)) || raw.slot != checkSlot.slot || raw.node != checkSlot.node)
                return fail(Stage::ChangedDuringCapture, address, i);
        }
        if (!detail::plausible(middleHigh)) return fail(Stage::MiddleHigh, middleHigh);
        if (!array(middleHigh + 0x290, equippedArray)) return fail(Stage::EquippedArray, middleHigh + 0x290);
        result.equippedCount = equippedArray.size;
        if (equippedArray.size > kMaximumEquipped) return fail(Stage::EquippedLimit, equippedArray.data);
        for (std::uint32_t i = 0; i < equippedArray.size; ++i) {
            detail::EquippedEntry raw{};
            const auto address = equippedArray.data + i * sizeof(raw);
            if (!read(address, &raw, sizeof(raw))) return fail(Stage::EquippedEntry, address, i);
            auto& item = result.equipped[i];
            if (!form(raw.form, item.form, item.formType)) return fail(Stage::EquippedForm, raw.form, i);
            item.index = raw.index;
            item.instance = raw.instance;
            item.data = raw.data;
            if (raw.slot && (!form(raw.slot, item.slot, type) || type != 0x7C)) return fail(Stage::EquippedSlot, raw.slot, i);
            if (item.formType == 0x2B && raw.data) {
                std::uintptr_t dataVtable{};
                if (!detail::plausible(raw.data) || !read(raw.data, &dataVtable, sizeof(dataVtable))) return fail(Stage::WeaponData, raw.data, i);
                if (dataVtable != weaponDataVtable) return fail(Stage::WeaponData, raw.data, i);
                std::uintptr_t ammo{};
                if (!read(raw.data + 0x10, &ammo, sizeof(ammo)) ||
                    !read(raw.data + 0x18, &item.loaded, sizeof(item.loaded)) ||
                    !read(raw.data + 0x30, &item.muzzle, sizeof(item.muzzle)) ||
                    !read(raw.data + 0x38, &item.attackState, sizeof(item.attackState)))
                    return fail(Stage::WeaponData, raw.data, i);
                if (ammo && (!form(ammo, item.ammo, type) || type != 0x2C)) return fail(Stage::AmmoForm, ammo, i);
                item.weaponDataValid = true;
            }
            detail::EquippedEntry check{};
            if (!read(address, &check, sizeof(check)) || !raw.sameIdentity(check)) return fail(Stage::ChangedDuringCapture, address, i);
        }
        detail::Array checkSlots{}, checkEquipped{};
        if (!array(race + 0x5D8, checkSlots) || !slotArray.sameStorage(checkSlots))
            return fail(Stage::ChangedDuringCapture, race + 0x5D8);
        if (!array(middleHigh + 0x290, checkEquipped) || !equippedArray.sameStorage(checkEquipped))
            return fail(Stage::ChangedDuringCapture, middleHigh + 0x290);
        return result;
    }
}
