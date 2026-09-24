#include "physics-interaction/weapon/NativeWeaponQualification.h"

#include <array>
#include <cstring>
#include <iostream>

namespace q = rock::native_weapon_qualification;

namespace
{
    constexpr std::uintptr_t base = 0x10000;
    constexpr std::uintptr_t race = 0x10100, middle = 0x10800;
    constexpr std::uintptr_t slots = 0x12000, equipped = 0x12500;
    constexpr std::uintptr_t slotA = 0x13000, slotB = 0x13100;
    constexpr std::uintptr_t weapon = 0x15000, ammoA = 0x15800, ammoB = 0x15900;
    constexpr std::uintptr_t dataA = 0x16000, dataB = 0x16100;
    constexpr std::uintptr_t nodeA = 0x18000, nodeB = 0x18100;
    constexpr std::uintptr_t vtable = 0x142D7FCF8;

    struct Memory
    {
        std::array<std::byte, 0x10000> bytes{};
        std::uintptr_t denied{}, unstable{};
        unsigned reads{}, unstableReads{};

        template<class T> void put(std::uintptr_t address, const T& value)
        {
            std::memcpy(bytes.data() + address - base, &value, sizeof(value));
        }
        void form(std::uintptr_t address, std::uint32_t id, std::uint8_t type)
        {
            put(address + 0x14, id);
            put(address + 0x1A, type);
        }
        bool read(std::uintptr_t address, void* out, std::size_t size) noexcept
        {
            ++reads;
            if (address == denied || address < base || address - base >= bytes.size() ||
                size > bytes.size() - (address - base)) return false;
            std::memcpy(out, bytes.data() + address - base, size);
            if (address == unstable && size == sizeof(q::detail::Array) && ++unstableReads == 2)
                static_cast<q::detail::Array*>(out)->data += 0x100;
            return true;
        }
        q::Snapshot capture()
        {
            return q::capture(race, middle, vtable,
                [this](std::uintptr_t address, void* out, std::size_t size) noexcept { return read(address, out, size); });
        }
    };

    Memory fixture()
    {
        Memory m;
        m.form(race, 0x13746, 0x11);
        m.form(slotA, 0x13F42, 0x7C);
        m.form(slotB, 0x13F43, 0x7C);
        m.form(weapon, 0x4822D, 0x2B);
        m.form(ammoA, 0x1F66A, 0x2C);
        m.form(ammoB, 0x1F66B, 0x2C);
        // Synthetic indices only: this does not assert a second native player slot.
        m.put(race + 0x5D8, q::detail::Array{slots, 2, 0, 2});
        m.put(slots, q::detail::SlotEntry{slotA, nodeA});
        m.put(slots + 0x10, q::detail::SlotEntry{slotB, nodeB});
        const char nameA[] = "Weapon A", nameB[] = "Weapon B";
        m.put(nodeA, nameA);
        m.put(nodeB, nameB);
        m.put(middle + 0x290, q::detail::Array{equipped, 2, 0, 2});
        m.put(equipped, q::detail::EquippedEntry{weapon, 0x17000, slotA, 0, 0, dataA});
        m.put(equipped + 0x28, q::detail::EquippedEntry{weapon, 0x17100, slotB, 1, 0, dataB});
        m.put(dataA, vtable);
        m.put(dataB, vtable);
        m.put(dataA + 0x10, ammoA);
        m.put(dataB + 0x10, ammoB);
        m.put(dataA + 0x18, std::uint32_t{7});
        m.put(dataB + 0x18, std::uint32_t{3});
        m.put(dataA + 0x30, std::uintptr_t{0x19000});
        m.put(dataB + 0x30, std::uintptr_t{0x19100});
        m.put(dataA + 0x38, std::uint32_t{0xF});
        m.put(dataB + 0x38, std::uint32_t{0x10});
        return m;
    }

    bool expect(bool condition, const char* message)
    {
        if (!condition) std::cerr << message << '\n';
        return condition;
    }
}

int main()
{
    bool ok = true;
    auto m = fixture();
    const auto before = m.bytes;
    const auto pair = m.capture();
    ok &= expect(pair.stage == q::Stage::Complete && pair.slotCount == 2 && pair.equippedCount == 2,
        "Both records must be observed, not only equippedItems[0]");
    ok &= expect(pair.equipped[0].form == pair.equipped[1].form &&
        pair.equipped[0].instance != pair.equipped[1].instance &&
        pair.equipped[0].loaded == 7 && pair.equipped[1].loaded == 3 &&
        pair.equipped[0].ammo != pair.equipped[1].ammo && pair.equipped[0].muzzle != pair.equipped[1].muzzle,
        "Identical base forms must retain separate instance, ammo, count, and muzzle witnesses");
    ok &= expect(m.bytes == before, "Observation must not write native state");

    m.put(dataA + 0x18, std::uint32_t{0});
    const auto empty = m.capture();
    ok &= expect(empty.equipped[0].weaponDataValid && empty.equipped[0].loaded == 0 &&
        empty.equipped[1].loaded == 3, "A real empty magazine must stay distinct from unreadable data and the other gun");
    m.put(equipped + 0x20, std::uintptr_t{0});
    const auto unavailable = m.capture();
    ok &= expect(unavailable.stage == q::Stage::Complete && !unavailable.equipped[0].weaponDataValid,
        "An absent equipped data object must not be reported as a known empty magazine");

    m = fixture();
    m.put(equipped + 0x18, std::uint32_t{9});
    ok &= expect(m.capture().equipped[0].index == 9, "Observed native index must not be relabeled as an array position or hand");

    m = fixture();
    m.put(race + 0x5D8, q::detail::Array{slots, 17, 0, 17});
    const auto overSlots = m.capture();
    ok &= expect(overSlots.stage == q::Stage::SlotLimit && overSlots.slotCount == 17 && m.reads < 6,
        "An oversized race mapping must fail before walking or claiming a truncated complete result");
    m = fixture();
    m.put(middle + 0x290, q::detail::Array{equipped, 17, 0, 17});
    ok &= expect(m.capture().stage == q::Stage::EquippedLimit, "Equipped record walk must be bounded");
    m = fixture();
    m.put(race + 0x5D8, q::detail::Array{slots, 1, 0, 2});
    ok &= expect(m.capture().stage == q::Stage::SlotArray, "Size exceeding capacity must fail closed");

    m = fixture();
    m.put(slotA + 0x20, q::detail::Array{0x14000, 9, 0, 9});
    ok &= expect(m.capture().stage == q::Stage::ParentLimit, "Parent traversal must be bounded without recursion");
    m.put(slotA + 0x20, q::detail::Array{0x14000, 1, 0, 1});
    m.put(0x14000, slotB);
    const auto parent = m.capture();
    ok &= expect(parent.stage == q::Stage::Complete && parent.slots[0].parents[0] == parent.slots[1].form,
        "Parent form identity must be captured without dereferencing retained pointers later");

    m = fixture();
    m.denied = dataB + 0x18;
    const auto denied = m.capture();
    ok &= expect(denied.stage == q::Stage::WeaponData && denied.failedIndex == 1 && denied.failedAddress == dataB,
        "A failed second-gun read must identify the deepest stage, owner address, and record");
    m = fixture();
    m.put(dataB, std::uintptr_t{0x142000000});
    ok &= expect(m.capture().stage == q::Stage::WeaponData, "A foreign data vtable must be rejected before reading weapon fields");
    m = fixture();
    m.form(ammoB, 0x1F66B, 0x2B);
    ok &= expect(m.capture().stage == q::Stage::AmmoForm, "A non-ammo form must not be reported as ammunition");
    m = fixture();
    m.form(slotA, 0x13F42, 0x11);
    ok &= expect(m.capture().stage == q::Stage::SlotForm, "Invalid slot type must stop the pointer walk");
    m = fixture();
    m.unstable = race + 0x5D8;
    ok &= expect(m.capture().stage == q::Stage::ChangedDuringCapture,
        "A mapping republished during observation must not produce a complete snapshot");
    m = fixture();
    m.unstable = middle + 0x290;
    ok &= expect(m.capture().stage == q::Stage::ChangedDuringCapture,
        "An equipment array republished during observation must be rejected");
    m = fixture();
    std::array<char, 64> longName;
    longName.fill('x');
    m.put(nodeA, longName);
    const auto truncated = m.capture();
    ok &= expect(truncated.stage == q::Stage::Complete && truncated.slots[0].nameTruncated &&
        truncated.slots[0].node.back() == '\0', "A bounded name must be terminated and report truncation");
    const auto nullRace = q::capture(0, middle, vtable,
        [&m](std::uintptr_t a, void* out, std::size_t n) noexcept { return m.read(a, out, n); });
    ok &= expect(nullRace.stage == q::Stage::RaceForm, "A missing race cannot establish a weapon mapping");
    return ok ? 0 : 1;
}
