#include "physics-interaction/weapon/native/NativeAmmoCounterVisibility.h"

#include <Windows.h>
#include <array>
#include <atomic>
#include <cstdint>

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/NativeMemory.h"
#include <RPS/Runtime/HookChain.h>

namespace rock::native_ammo_counter_visibility
{
    namespace
    {
        /*
         * FO4VR 1.2.72, raw disassembly: the HUDAmmoCounter constructor
         * (0xA0DB70) stores this class's vtable 0x2D48C98. HUDComponentBase's
         * update (0xA22240) calls slot 7 (+0x38), the visibility predicate
         * 0xA22520, and fades the widget out or back in from its result.
         * Replacing this class's slot alone leaves every other HUD component
         * on the shared predicate.
         */
        constexpr std::uintptr_t kVtable = 0x2D48C98;
        constexpr std::uintptr_t kVisibleSlot = kVtable + 7 * sizeof(std::uintptr_t);
        constexpr std::uintptr_t kVisible = 0xA22520;
        constexpr auto kVisiblePrefix =
            std::to_array<std::uint8_t>({ 0x48, 0x83, 0xEC, 0x28, 0x80, 0xB9, 0xF1, 0, 0, 0, 0 });
        constexpr std::uint64_t kLeaseMilliseconds = 250;

        using Visible = bool (*)(void*);
        Visible g_original{ nullptr };
        bool g_attempted{ false };
        // Written on the game thread, read on the HUD update thread.
        std::atomic<std::uint64_t> g_replacedUntil{ 0 };

        bool visible(void* widget) noexcept
        {
            return g_original(widget) && GetTickCount64() >= g_replacedUntil.load(std::memory_order_acquire);
        }
    }

    bool install() noexcept
    {
        if (g_original)
            return true;
        if (g_attempted)
            return false;
        g_attempted = true;
        // The slot holds the native visibility function or another plugin's; the
        // lease calls through whichever is there.
        const auto slot = REL::Offset(kVisibleSlot).address();
        const auto resolved = RPS::Runtime::HookChain::resolveSlot(slot, REL::Offset(kVisible).address());
        const std::uintptr_t current = resolved.callee;
        if (!resolved || (!resolved.foreign() &&
                             !RPS::Runtime::HookChain::resolveEntry(current, kVisiblePrefix))) {
            ROCK_LOG_WARN(Weapon,
                "Ammo counters: the vanilla ammo widget's visibility slot holds unrecognised code; "
                "the vanilla counter stays visible");
            return false;
        }
        if (resolved.foreign())
            ROCK_LOG_INFO(Weapon, "Ammo widget visibility slot already replaced by {}; ROCK calls through it",
                resolved.owner);
        g_original = reinterpret_cast<Visible>(current);
        REL::safe_write(slot, reinterpret_cast<std::uintptr_t>(&visible));
        ROCK_LOG_INFO(Weapon, "Ammo counters: vanilla ammo widget visibility lease installed");
        return true;
    }

    void replace(const bool active) noexcept
    {
        g_replacedUntil.store(active && g_original ? GetTickCount64() + kLeaseMilliseconds : 0,
            std::memory_order_release);
    }
}
