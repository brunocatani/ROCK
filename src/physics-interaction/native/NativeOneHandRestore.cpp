#include "physics-interaction/native/NativeOneHandRestore.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/NativeMemory.h"

#include <REL/Relocation.h>
#include <Windows.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <stdexcept>

namespace rock::native_one_hand_restore
{
    namespace
    {
        constexpr std::uintptr_t kRestoreEntry = 0xE714A0;
        // Raw FO4VR 1.2.72: each is `call 140E714A0` (E8 rel32) inside the
        // remembered-item loop, and no caller reads its result (the next
        // instructions reload or advance the loop registers).
        constexpr std::array<std::uintptr_t, 4> kRestoreCallsites{ 0xE72CE9, 0xE732AA, 0xE7453D, 0xE7486D };

        std::atomic<bool> s_disabled{ false };

        // Same register arguments as 140E714A0 (equip manager, actor, the
        // remembered entry, its slot); restores nothing.
        void skipRestore(void*, void*, void*, void*) noexcept
        {
            ROCK_LOG_SAMPLE_INFO(Weapon, 1000, "Native one-hand item restore skipped");
        }

        [[nodiscard]] bool readCall(const std::uintptr_t callsite, std::array<std::uint8_t, 5>& out) noexcept
        {
            return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Offset(callsite).address()),
                out.data(), out.size());
        }

        [[nodiscard]] bool callsRestore(const std::uintptr_t callsite) noexcept
        {
            std::array<std::uint8_t, 5> bytes{};
            if (!readCall(callsite, bytes) || bytes[0] != 0xE8)
                return false;
            std::int32_t displacement = 0;
            std::memcpy(&displacement, bytes.data() + 1, sizeof(displacement));
            return REL::Offset(callsite).address() + bytes.size() + displacement ==
                REL::Offset(kRestoreEntry).address();
        }
    }

    bool install() noexcept
    {
        if (s_disabled.load(std::memory_order_acquire))
            return true;
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72)
            return false;
        for (const auto callsite : kRestoreCallsites) {
            if (!callsRestore(callsite)) {
                ROCK_LOG_ERROR(Init,
                    "Native one-hand item restore left active: callsite {:#x} is not the verified restore call",
                    callsite);
                return false;
            }
        }
        auto& trampoline = F4SE::GetTrampoline();
        if (trampoline.free_size() < 14 * kRestoreCallsites.size()) {
            ROCK_LOG_ERROR(Init, "Native one-hand item restore left active: trampoline capacity");
            return false;
        }
        std::array<std::array<std::uint8_t, 5>, kRestoreCallsites.size()> originals{};
        std::size_t written = 0;
        try {
            for (; written < kRestoreCallsites.size(); ++written) {
                const auto callsite = kRestoreCallsites[written];
                if (!readCall(callsite, originals[written]))
                    throw std::runtime_error("callsite unreadable");
                const auto address = REL::Offset(callsite).address();
                (void)trampoline.write_call<5>(address, &skipRestore);
                if (!FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<const void*>(address), 5))
                    throw std::runtime_error("instruction cache flush failed");
            }
        } catch (...) {
            // All four or none: a partial set would still restore at some sites.
            for (std::size_t index = 0; index < written; ++index) {
                const auto address = REL::Offset(kRestoreCallsites[index]).address();
                REL::safe_write(address, originals[index].data(), originals[index].size());
                FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<const void*>(address),
                    originals[index].size());
            }
            ROCK_LOG_ERROR(Init, "Native one-hand item restore left active: callsite redirect failed");
            return false;
        }
        s_disabled.store(true, std::memory_order_release);
        ROCK_LOG_INFO(Init, "Native one-hand item restore disabled at {} sites", kRestoreCallsites.size());
        return true;
    }

    bool disabled() noexcept { return s_disabled.load(std::memory_order_acquire); }
}
