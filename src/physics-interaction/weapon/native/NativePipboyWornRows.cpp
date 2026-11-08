/*
 * The Pip-Boy shows one row per group of an item's stacks whose extra data it
 * calls equal (FO4VR 1.2.72, raw disassembly):
 * - 140C23850(data, item, stack) looks for an existing entry of the item and
 *   reuses the first whose stack 140C239E0 calls equal; with none, 140C20E00
 *   adds an entry. 140C23850 is the only caller of 140C239E0 (call 140C2399A).
 * - 140C239E0 compares the two extra lists with 1400834E0, filter 1400A2F40
 *   (health, charge, mods, name, enchantment, instance data...), never the
 *   worn bits or ExtraUniqueID.
 * So identical copies of a weapon always share a row, the worn copy included,
 * and the other hand could not reach a second copy: its press on that row
 * means the worn copy (its support grip). For hand weapons ROCK adds one rule:
 * stacks with different worn bits are different rows. The worn copy gets its
 * own row; the unworn copies keep sharing one. Every other item keeps the
 * native rows. WeaponEquipTransfer keeps the worn copy on its own stack.
 */
#include "physics-interaction/weapon/native/NativePipboyWornRows.h"

#include <REL/Relocation.h>
#include <Windows.h>
#include <array>
#include <cstring>
#include <utility>
#include "RE/Bethesda/TESBoundObjects.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/weapon/classification/WeaponTypePolicy.h"
#include <RPS/Runtime/HookChain.h>

namespace rock::native_pipboy_worn_rows
{
    namespace
    {
        using Stack = RE::BGSInventoryItem::Stack;
        using SameRow = bool (*)(void*, const Stack*, const Stack*);
        using FindEntry = void* (*)(void*, const RE::BGSInventoryItem*, const Stack*);

        constexpr std::uintptr_t kFindEntry = 0xC23850;
        constexpr std::uintptr_t kSameRow = 0xC239E0;
        constexpr std::uintptr_t kSameRowCall = 0xC2399A;

        // Load owner; read-only after install.
        SameRow s_sameRow = nullptr;
        void* s_findEntry = nullptr;
        bool s_installed = false;
        // Set for the duration of one 140C23850 call on its own thread.
        thread_local bool t_handWeapon = false;

        [[nodiscard]] std::uint16_t wornBits(const Stack& stack) noexcept
        {
            return stack.flags.underlying() & 7u;
        }

        [[nodiscard]] bool handWeapon(const RE::BGSInventoryItem* item) noexcept
        {
            const auto* object = item ? item->object : nullptr;
            if (!object || object->GetFormType() != RE::ENUM_FORM_ID::kWEAP)
                return false;
            return weapon_type_policy::occupiesHandSlot(
                static_cast<const RE::TESObjectWEAP*>(object)->weaponData.type.get());
        }

        bool sameRowHook(void* data, const Stack* a, const Stack* b) noexcept
        {
            if (!s_sameRow(data, a, b))
                return false;
            return !t_handWeapon || !a || !b || wornBits(*a) == wornBits(*b);
        }

        void* findEntryHook(void* data, const RE::BGSInventoryItem* item, const Stack* stack) noexcept
        {
            const bool previous = std::exchange(t_handWeapon, handWeapon(item));
            auto* entry = reinterpret_cast<FindEntry>(s_findEntry)(data, item, stack);
            t_handWeapon = previous;
            return entry;
        }

        [[nodiscard]] bool callRedirects(const std::uintptr_t site, const std::uintptr_t destination) noexcept
        {
            std::array<std::uint8_t, 5> call{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(site), call.data(), call.size()) ||
                call[0] != 0xE8)
                return false;
            std::int32_t displacement = 0;
            std::memcpy(&displacement, call.data() + 1, sizeof(displacement));
            const auto stub = site + 5 + displacement;
            std::array<std::uint8_t, 14> jump{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(stub), jump.data(), jump.size()))
                return false;
            std::uintptr_t target = 0;
            std::memcpy(&target, jump.data() + 6, sizeof(target));
            return jump[0] == 0xFF && jump[1] == 0x25 && target == destination;
        }
    }

    bool sameWeaponRow(const Stack& a, const Stack& b) noexcept
    {
        if (&a == &b)
            return true;
        // 140C239E0 dereferences both extra lists.
        if (!a.extra || !b.extra)
            return !a.extra && !b.extra && wornBits(a) == wornBits(b);
        if (!s_sameRow)
            return false;
        const bool previous = std::exchange(t_handWeapon, true);
        const bool same = sameRowHook(nullptr, &a, &b);
        t_handWeapon = previous;
        return same;
    }

    bool install() noexcept
    {
        if (s_installed)
            return true;
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72)
            return false;
        constexpr std::array<std::uint8_t, 16> sameRowPrologue{ 0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xEC,
            0x20, 0x49, 0x8B, 0xF8, 0x48, 0x8B, 0xF2 };
        std::array<std::uint8_t, sameRowPrologue.size()> live{};
        if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Offset(kSameRow).address()),
                live.data(), live.size()) ||
            live != sameRowPrologue) {
            ROCK_LOG_ERROR(Init, "Pip-Boy worn weapon rows: row predicate prologue mismatch");
            return false;
        }
        const auto site = REL::Offset(kSameRowCall).address();
        const auto resolved = RPS::Runtime::HookChain::resolveCall(site, REL::Offset(kSameRow).address());
        if (!resolved) {
            ROCK_LOG_ERROR(Init, "Pip-Boy worn weapon rows: row predicate call unrecognized");
            return false;
        }
        if (resolved.foreign())
            ROCK_LOG_INFO(Init, "Pip-Boy row predicate call already hooked by {}; ROCK calls through it",
                resolved.owner);
        // push rbx (REX 40 53); push rbp; push r12; push r13; sub rsp,38h;
        // mov rax,[rdx]; mov rbp,rcx. Position-independent, ends at C23861.
        constexpr std::array<std::uint8_t, 17> findEntryPrologue{ 0x40, 0x53, 0x55, 0x41, 0x54, 0x41, 0x55, 0x48, 0x83,
            0xEC, 0x38, 0x48, 0x8B, 0x02, 0x48, 0x8B, 0xE9 };
        auto& trampoline = F4SE::GetTrampoline();
        if (trampoline.free_size() < 14)
            return false;
        std::array<std::uint8_t, 5> before{};
        std::memcpy(before.data(), reinterpret_cast<const void*>(site), before.size());
        s_sameRow = reinterpret_cast<SameRow>(resolved.callee);
        (void)trampoline.write_call<5>(site, reinterpret_cast<std::uintptr_t>(&sameRowHook));
        if (!callRedirects(site, reinterpret_cast<std::uintptr_t>(&sameRowHook)) ||
            !FlushInstructionCache(GetCurrentProcess(), nullptr, 0)) {
            REL::safe_write(site, before.data(), before.size());
            FlushInstructionCache(GetCurrentProcess(), nullptr, 0);
            s_sameRow = nullptr;
            ROCK_LOG_ERROR(Init, "Pip-Boy worn weapon rows: row predicate redirect failed; native rows kept");
            return false;
        }
        if (!entry_trampoline_hook::install("Pip-Boy inventory entry lookup", kFindEntry, findEntryPrologue.data(),
                findEntryPrologue.size(), reinterpret_cast<void*>(&findEntryHook), s_findEntry)) {
            // Without the item scope the rule never applies; the call stays a pass-through.
            ROCK_LOG_ERROR(Init, "Pip-Boy worn weapon rows: entry lookup hook failed; native rows kept");
            return false;
        }
        s_installed = true;
        ROCK_LOG_INFO(Init, "Pip-Boy worn weapon rows installed: a worn hand weapon copy has its own row");
        return true;
    }
}
