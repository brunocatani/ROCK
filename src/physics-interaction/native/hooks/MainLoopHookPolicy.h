#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace rock::main_loop_hook_policy
{
    inline constexpr std::uint8_t kRelativeCallOpcode = 0xE8;
    inline constexpr std::uint8_t kAbsoluteJumpOpcode = 0xFF;
    inline constexpr std::uint8_t kRipIndirectJumpModRm = 0x25;
    inline constexpr std::size_t kRelativeCallSize = 5;
    inline constexpr std::size_t kCommonLibAbsoluteJumpThunkSize = 14;

    [[nodiscard]] inline bool decodeRelativeCallTarget(
        const std::uint8_t* callBytes,
        std::uintptr_t callAddress,
        std::uintptr_t& outTarget) noexcept
    {
        outTarget = 0;
        if (!callBytes || callAddress == 0 ||
            callBytes[0] != kRelativeCallOpcode) {
            return false;
        }

        std::int32_t displacement = 0;
        std::memcpy(&displacement, callBytes + 1, sizeof(displacement));
        outTarget = static_cast<std::uintptr_t>(
            static_cast<std::intptr_t>(callAddress + kRelativeCallSize) +
            static_cast<std::intptr_t>(displacement));
        return outTarget != 0;
    }

    [[nodiscard]] inline bool isCommonLibAbsoluteJumpThunk(
        const std::uint8_t* thunkBytes) noexcept
    {
        if (!thunkBytes ||
            thunkBytes[0] != kAbsoluteJumpOpcode ||
            thunkBytes[1] != kRipIndirectJumpModRm) {
            return false;
        }
        std::int32_t displacement = 0;
        std::memcpy(&displacement, thunkBytes + 2, sizeof(displacement));
        return displacement == 0;
    }

    [[nodiscard]] inline bool decodeCommonLibAbsoluteJumpTarget(
        const std::uint8_t* thunkBytes,
        std::uintptr_t& outTarget) noexcept
    {
        outTarget = 0;
        if (!isCommonLibAbsoluteJumpThunk(thunkBytes)) {
            return false;
        }
        std::uint64_t target = 0;
        std::memcpy(&target, thunkBytes + 6, sizeof(target));
        outTarget = static_cast<std::uintptr_t>(target);
        return outTarget != 0;
    }
}
