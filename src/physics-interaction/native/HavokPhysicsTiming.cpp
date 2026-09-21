#include "physics-interaction/native/HavokPhysicsTiming.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"

#include <REL/Relocation.h>
#include <array>
#include <atomic>

namespace rock::havok_physics_timing
{
    namespace
    {
        // Initialized before installing the main-thread SetDeltaTime hook.
        // The module-owned global remains alive for the process lifetime.
        // Main publishes one scalar per world update; physics callbacks copy
        // it into their existing timing snapshot and inherit it for all solves.
        const float* s_timeMultiplier = nullptr;
        std::atomic<float> s_capturedTimeMultiplier{0.0f};

        float readFloatGlobal(std::uintptr_t offset, float fallback)
        {
            REL::Relocation<float*> value{ REL::Offset(offset) };
            return value.address() ? *value : fallback;
        }

        std::uint32_t readUintGlobal(std::uintptr_t offset, std::uint32_t fallback)
        {
            REL::Relocation<std::uint32_t*> value{ REL::Offset(offset) };
            return value.address() ? *value : fallback;
        }
    }

    bool initializeTimeMultiplierSampling()
    {
        // FO4VR raw witnesses: 141B962EC..141B9630A preserves the unscaled
        // timer delta, multiplies by this global, and stores the scaled delta.
        // 141DF7279..141DF7289 uses the same global for the Havok step size.
        // Validate both live consumers before the first new global read.
        constexpr std::array<unsigned char, 31> timerBytes{
            0xF3,0x41,0x0F,0x10,0x40,0x10,0x4D,0x89,0x48,0x18,
            0xF3,0x41,0x0F,0x11,0x40,0x14,0xF3,0x0F,0x59,0x05,
            0x2C,0xB3,0xCE,0x01,0xF3,0x41,0x0F,0x11,0x40,0x10,0xC3};
        constexpr std::array<unsigned char, 19> physicsBytes{
            0xF3,0x0F,0x10,0x05,0xAF,0xA3,0xA8,0x01,
            0xF3,0x0F,0x59,0x05,0xEF,0xCA,0x7A,0x04,0x0F,0x28,0xD0};
        std::array<unsigned char, timerBytes.size()> liveTimer{};
        std::array<unsigned char, physicsBytes.size()> livePhysics{};
        const auto timer = REL::Offset(0x1B962EC).address();
        const auto physics = REL::Offset(0x1DF7279).address();
        const auto* multiplier = reinterpret_cast<const float*>(REL::Offset(0x3881630).address());
        if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(timer), liveTimer.data(), liveTimer.size()) ||
            !native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(physics), livePhysics.data(), livePhysics.size()) ||
            liveTimer != timerBytes || livePhysics != physicsBytes ||
            !native_memory::pointerRangeLooksReadable(multiplier, sizeof(float))) {
            ROCK_LOG_ERROR(Init, "VATS_PHYSICS native time multiplier validation failed; compensation unavailable");
            return false;
        }
        s_timeMultiplier = multiplier;
        ROCK_LOG_INFO(Init, "VATS_PHYSICS native time multiplier verified; sampling at the physics timing hook");
        return true;
    }

    void captureTimeMultiplier()
    {
        s_capturedTimeMultiplier.store(s_timeMultiplier ? *s_timeMultiplier : 0.0f, std::memory_order_release);
    }

    PhysicsTimingSample sampleCurrentTiming()
    {
        /*
         * Unreadable globals report zero so makeTimingSample classifies the
         * sample as fallback; missing native timing must never look measured.
         */
        return makeTimingSample(
            readFloatGlobal(offsets::kData_BhkWorldRawDeltaSeconds, 0.0f),
            readFloatGlobal(offsets::kData_BhkWorldSubstepDeltaSeconds, 0.0f),
            readFloatGlobal(offsets::kData_BhkWorldRemainderDeltaSeconds, 0.0f),
            readFloatGlobal(offsets::kData_BhkWorldAccumulatedDeltaSeconds, 0.0f),
            readUintGlobal(offsets::kData_BhkWorldSubstepCount, 0),
            s_capturedTimeMultiplier.load(std::memory_order_acquire));
    }

}
