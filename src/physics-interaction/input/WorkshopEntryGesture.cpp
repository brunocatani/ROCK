#include "physics-interaction/input/WorkshopEntryGesture.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "rock_support/Fo4VrRuntime.h"
#include "setup-wizard/AutoOpenGate.h"

#include <REL/Relocation.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace rock::workshop_entry_gesture
{
    namespace
    {
        /*
         * The game's Workshop entry in Fallout4VR.exe 1.2.72:
         * void* (char* status). It finds the player's workshop (0x1FDDA0) and
         * enters Workshop mode (0x219CC0) only when the status it writes is 0;
         * 1 is outside every buildable area, 2 already in Workshop mode, 3 the
         * nearest workshop is not available. Raw disassembly of both callers:
         * the StartWorkshop console command (0x51A900), which prints those four
         * outcomes, and MenuOpenHandler's menu-button hold (0x1326D34). The
         * prefix runs from the entry through the status test, the player read
         * and the in-Workshop test to the call that starts it.
         */
        constexpr std::uintptr_t kFunc_TryEnterWorkshop = 0x1FDE20;
        constexpr std::array<std::uint8_t, 61> kExpectedPrefix{ 0x48, 0x89, 0x5C, 0x24, 0x08, 0x57, 0x48, 0x83, 0xEC, 0x50,
            0x48, 0x8B, 0xF9, 0xE8, 0x6E, 0xFF, 0xFF, 0xFF, 0x48, 0x8B, 0xD8, 0x48, 0x85, 0xC0, 0x74, 0x46, 0x80, 0x3F, 0x00,
            0x75, 0x41, 0x48, 0x8B, 0x15, 0xAA, 0x65, 0x90, 0x05, 0x48, 0x8D, 0x4C, 0x24, 0x20, 0xE8, 0xF0, 0xA2, 0x00, 0x00,
            0x80, 0x3D, 0x88, 0x36, 0x74, 0x05, 0x00, 0x75, 0x16, 0x48, 0x8B, 0xCB, 0xE8 };
        using TryEnterWorkshop = void* (*)(char* status);

        // Logical stick click (Axis0) after the [Controls] layout.
        constexpr int kStickClickButtonId = 32;
        constexpr std::uint32_t kMaximumSampleAgeMilliseconds = 100;

        enum class Phase : std::uint8_t
        {
            Idle,
            Holding,
            // Entered, refused or interrupted: nothing more until the click comes up.
            DoneUntilRelease,
        };

        TryEnterWorkshop s_tryEnterWorkshop = nullptr;
        RE::Setting* s_enterDelaySetting = nullptr; // Engine setting, registered for the process.
        Phase s_phase = Phase::Idle;
        std::chrono::steady_clock::time_point s_pressedAt{};

        [[nodiscard]] const char* describeStatus(const char status)
        {
            switch (status) {
            case 0:
                return "entered Workshop mode";
            case 1:
                return "not inside a workshop's buildable area";
            case 2:
                return "already in Workshop mode";
            case 3:
                return "the nearest workshop is not available to the player";
            default:
                return "unknown result";
            }
        }

        [[nodiscard]] bool leftStickClickHeld()
        {
            const auto raw = input_remap_runtime::peekRawButtonState(true, kStickClickButtonId);
            return raw.available && raw.availabilityReason == input_remap_runtime::RawButtonAvailabilityReason::Available &&
                raw.sampleAgeMilliseconds <= kMaximumSampleAgeMilliseconds && raw.held;
        }
    }

    void install()
    {
        const auto address = REL::Offset(kFunc_TryEnterWorkshop).address();
        if (std::memcmp(reinterpret_cast<const void*>(address), kExpectedPrefix.data(), kExpectedPrefix.size()) != 0) {
            ROCK_LOG_ERROR(Input, "Workshop on left stick hold unavailable: unexpected bytes at the game's Workshop entry");
            return;
        }
        s_enterDelaySetting = f4vr::getIniSetting("fEnterWorkshopDelay:Controls");
        if (!s_enterDelaySetting || s_enterDelaySetting->GetType() != RE::Setting::SETTING_TYPE::kFloat) {
            ROCK_LOG_ERROR(Input, "Workshop on left stick hold unavailable: fEnterWorkshopDelay:Controls not found");
            s_enterDelaySetting = nullptr;
            return;
        }
        s_tryEnterWorkshop = reinterpret_cast<TryEnterWorkshop>(address);
        ROCK_LOG_INFO(Input, "Workshop on left stick hold ready: hold {:.2f}s (fEnterWorkshopDelay:Controls)",
            s_enterDelaySetting->GetFloat());
    }

    void update()
    {
        if (!s_tryEnterWorkshop) {
            return;
        }
        if (!leftStickClickHeld()) {
            s_phase = Phase::Idle;
            return;
        }
        if (s_phase == Phase::DoneUntilRelease) {
            return;
        }
        // The whole hold happens in free play: a menu, scene or fade on the
        // way ends it, so a click held through one never enters by itself.
        if (!setup_wizard::auto_open_gate::read().ready()) {
            s_phase = Phase::DoneUntilRelease;
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (s_phase == Phase::Idle) {
            s_phase = Phase::Holding;
            s_pressedAt = now;
            return;
        }
        const float delay = s_enterDelaySetting->GetFloat();
        const float heldSeconds = std::chrono::duration<float>(now - s_pressedAt).count();
        if (!(heldSeconds >= delay)) {
            return;
        }
        s_phase = Phase::DoneUntilRelease;
        char status = 1;
        (void)s_tryEnterWorkshop(&status);
        ROCK_LOG_INFO(Input, "Left stick hold {:.2f}s: {}", heldSeconds, describeStatus(status));
    }
}
