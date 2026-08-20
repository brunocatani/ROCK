#include "physics-interaction/core/FrikSkeletonProfile.h"

#include <atomic>
#include <cstdint>

#include "rock_support/Fo4VrRuntime.h"

namespace rock::frik_skeleton_profile
{
    namespace
    {
        constexpr std::uint64_t kGenerationMask = 0xFFFF'FFFFull;
        constexpr std::uint64_t kPowerArmorFlag = 1ull << 62;
        constexpr std::uint64_t kValidFlag = 1ull << 63;

        static_assert(std::atomic<std::uint64_t>::is_always_lock_free);

        std::atomic<std::uint64_t> s_packedProfile{ 0 };
    }

    void publishReady(const std::uint32_t generation, const bool inPowerArmor) noexcept
    {
        const auto packed =
            kValidFlag |
            (inPowerArmor ? kPowerArmorFlag : 0) |
            (static_cast<std::uint64_t>(generation) & kGenerationMask);
        s_packedProfile.store(packed, std::memory_order_release);
    }

    void clear() noexcept
    {
        s_packedProfile.store(0, std::memory_order_release);
    }

    Snapshot current() noexcept
    {
        const auto packed = s_packedProfile.load(std::memory_order_acquire);
        return {
            .generation = static_cast<std::uint32_t>(packed & kGenerationMask),
            .inPowerArmor = (packed & kPowerArmorFlag) != 0,
            .valid = (packed & kValidFlag) != 0,
        };
    }

    bool effectiveInPowerArmor() noexcept
    {
        const auto profile = current();
        return profile.valid ? profile.inPowerArmor : f4vr::isInPowerArmor();
    }
}
