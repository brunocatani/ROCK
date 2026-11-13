#include "physics-interaction/native/player/NativeWorkshop.h"

#include "physics-interaction/PhysicsLog.h"

#include <REL/Relocation.h>

#include <array>
#include <cstring>

namespace rock::native_workshop
{
    namespace
    {
        /*
         * Fallout4VR.exe 1.2.72, raw disassembly. Both functions take a status
         * byte and return the player's workshop reference (or null):
         *
         * Query 0x1FDDA0 writes 1 (no workshop found for the player), 2 (already
         * in Workshop mode), 3 (the workshop is not available to the player) or
         * 0, and changes nothing else. MenuOpenHandler reads it on the menu
         * button's press (0x13268D5).
         *
         * Entry 0x1FDE20 calls the query and starts Workshop mode (0x219CC0)
         * only on status 0. Its callers are the StartWorkshop console command
         * (0x51A900), which prints the four outcomes, and MenuOpenHandler's
         * menu-button hold (0x1326D34).
         *
         * Each prefix runs to the code that decides the status.
         */
        constexpr std::uintptr_t kFunc_QueryWorkshop = 0x1FDDA0;
        constexpr std::array<std::uint8_t, 48> kQueryPrefix{ 0x48, 0x89, 0x5C, 0x24, 0x08, 0x57, 0x48, 0x83, 0xEC, 0x20,
            0x48, 0x8B, 0xF9, 0xC6, 0x01, 0x01, 0x48, 0x8B, 0x0D, 0x39, 0x66, 0x90, 0x05, 0xE8, 0x64, 0x71, 0x00, 0x00, 0x48,
            0x8B, 0xD8, 0x48, 0x85, 0xC0, 0x74, 0x43, 0x80, 0x3D, 0x14, 0x37, 0x74, 0x05, 0x00, 0x74, 0x0E, 0xC6, 0x07, 0x02 };
        constexpr std::uintptr_t kFunc_EnterWorkshop = 0x1FDE20;
        constexpr std::array<std::uint8_t, 61> kEnterPrefix{ 0x48, 0x89, 0x5C, 0x24, 0x08, 0x57, 0x48, 0x83, 0xEC, 0x50,
            0x48, 0x8B, 0xF9, 0xE8, 0x6E, 0xFF, 0xFF, 0xFF, 0x48, 0x8B, 0xD8, 0x48, 0x85, 0xC0, 0x74, 0x46, 0x80, 0x3F, 0x00,
            0x75, 0x41, 0x48, 0x8B, 0x15, 0xAA, 0x65, 0x90, 0x05, 0x48, 0x8D, 0x4C, 0x24, 0x20, 0xE8, 0xF0, 0xA2, 0x00, 0x00,
            0x80, 0x3D, 0x88, 0x36, 0x74, 0x05, 0x00, 0x75, 0x16, 0x48, 0x8B, 0xCB, 0xE8 };
        using WorkshopCall = void* (*)(char* status);

        WorkshopCall s_query = nullptr;
        WorkshopCall s_enter = nullptr;

        template <std::size_t N>
        [[nodiscard]] WorkshopCall verified(std::uintptr_t offset, const std::array<std::uint8_t, N>& prefix)
        {
            const auto address = REL::Offset(offset).address();
            return std::memcmp(reinterpret_cast<const void*>(address), prefix.data(), prefix.size()) == 0
                ? reinterpret_cast<WorkshopCall>(address)
                : nullptr;
        }

        [[nodiscard]] Status call(WorkshopCall function)
        {
            if (!function) {
                return Status::Unsupported;
            }
            char status = static_cast<char>(Status::OutsideBuildArea);
            (void)function(&status);
            return status >= 0 && status <= 3 ? static_cast<Status>(status) : Status::Unsupported;
        }
    }

    void install()
    {
        s_query = verified(kFunc_QueryWorkshop, kQueryPrefix);
        s_enter = verified(kFunc_EnterWorkshop, kEnterPrefix);
        if (!s_query || !s_enter) {
            s_query = s_enter = nullptr;
            ROCK_LOG_ERROR(Input, "Workshop on B hold unavailable: unexpected bytes at the game's Workshop functions");
            return;
        }
        ROCK_LOG_INFO(Input, "Workshop on B hold ready");
    }

    Status query() { return call(s_query); }

    Status enter() { return call(s_enter); }

    const char* statusName(const Status status) noexcept
    {
        switch (status) {
        case Status::Available:
            return "in a workshop's buildable area";
        case Status::OutsideBuildArea:
            return "not inside a workshop's buildable area";
        case Status::AlreadyInWorkshop:
            return "already in Workshop mode";
        case Status::WorkshopUnavailable:
            return "the nearest workshop is not available to the player";
        case Status::Unsupported:
            break;
        }
        return "Workshop functions unavailable";
    }
}
