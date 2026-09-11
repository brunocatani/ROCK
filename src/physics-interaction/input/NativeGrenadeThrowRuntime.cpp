#include "physics-interaction/input/NativeGrenadeThrowRuntime.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/grenade/LooseGrenadeRuntime.h"
#include "physics-interaction/PhysicsLog.h"
#include "RockConfig.h"
#include "RE/Bethesda/BGSInventoryItem.h"
#include "RE/Bethesda/BSLock.h"
#include "RE/Bethesda/InputEvent.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include <REL/Relocation.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace rock::native_grenade_throw_runtime
{
    namespace
    {
        // FO4VR 1.2.72 raw verification, 2026-09-11:
        // FC2910 constructs/registers MeleeThrow at PlayerControls+300 with
        // vtable 2D8A998; FC1C00 independently resolves +300 and clears +2A.
        // FC8AE0 initializes on press, primes at 37D5B18, launches on release
        // through EFA340, then clears +28..2A. FCE140 supplies native gates.
        // EFA170 enters VR mode 4; C64930 restores it via C63C90 (case 4),
        // which calls F0B190 to remove the preview and disarm without throwing.
        constexpr std::uintptr_t kControls = 0x5A3B8A0;
        constexpr std::uintptr_t kHandlerVtable = 0x2D8A998;
        constexpr std::uintptr_t kProcess = 0xFC8AE0;
        constexpr std::uintptr_t kShouldHandle = 0xFCE140;
        constexpr std::uintptr_t kRestoreMode = 0xC64930;
        constexpr std::uintptr_t kVrUi = 0x5AC8EB0;
        using Process = void (*)(void*, RE::InputEvent*, void*, void*);
        using ShouldHandle = bool (*)(void*, RE::InputEvent*);
        bool s_active = false;

        template <std::size_t N>
        bool matches(std::uintptr_t rva, const std::array<unsigned char, N>& expected)
        {
            std::array<unsigned char, N> live{};
            return native_memory::guardedCopyFromMemory(
                       reinterpret_cast<const void*>(REL::Offset(rva).address()), live.data(), N) && live == expected;
        }

        bool available()
        {
            static const bool verified = [] {
                const bool valid =
                    matches(kProcess, std::array<unsigned char, 13>{0x40,0x55,0x56,0x57,0x41,0x56,0x48,0x8B,0xEC,0x48,0x83,0xEC,0x78}) &&
                    matches(kShouldHandle, std::array<unsigned char, 16>{0x40,0x57,0x48,0x83,0xEC,0x20,0x48,0x8B,0x0D,0xA3,0x62,0xB3,0x04,0x48,0x8B,0xFA}) &&
                    matches(kRestoreMode, std::array<unsigned char, 16>{0x40,0x57,0x48,0x83,0xEC,0x20,0x48,0x8B,0xF9,0x48,0x8B,0x0D,0xB0,0xFA,0xE9,0x04}) &&
                    matches(0xFC1C00, std::array<unsigned char, 16>{0x48,0x8B,0x81,0x00,0x03,0x00,0x00,0x48,0x85,0xC0,0x74,0x04,0xC6,0x40,0x2A,0x00});
                if (!valid) ROCK_LOG_ERROR(Input, "Vanilla grenade fallback disabled: native code validation failed");
                return valid;
            }();
            return verified;
        }

        void* resolveHandler()
        {
            void* controls{};
            void* handler{};
            std::uintptr_t vtable{};
            const char* stage = "controls";
            if (native_memory::tryReadValue(reinterpret_cast<void**>(REL::Offset(kControls).address()), controls) && controls) {
                stage = "melee-throw-handler";
                if (native_memory::tryReadField(controls, 0x300, handler) && handler) {
                    stage = "handler-vtable";
                    if (native_memory::tryReadField(handler, 0, vtable) &&
                        vtable == REL::Offset(kHandlerVtable).address() &&
                        native_memory::pointerRangeLooksReadable(handler, 0x30)) return handler;
                }
            }
            ROCK_LOG_SAMPLE_WARN(Input, g_rockConfig.rockLogSampleMilliseconds,
                "Vanilla grenade fallback unavailable: stage={}", stage);
            return nullptr;
        }

        bool hasEquippedThrowable()
        {
            auto* player = RE::PlayerCharacter::GetSingleton();
            if (!player || !player->inventoryList) return false;
            const RE::BSAutoReadLock lock{player->inventoryList->rwLock};
            std::uint32_t entries = 0;
            for (const auto& entry : player->inventoryList->data) {
                if (++entries > 16384) return false;
                const auto* weapon = entry.object ? entry.object->As<RE::TESObjectWEAP>() : nullptr;
                if (!loose_grenade_runtime::isThrowableWeapon(weapon)) continue;
                std::uint32_t stacks = 0;
                for (auto* stack = entry.stackData.get(); stack; stack = stack->nextStack.get()) {
                    if (++stacks > 4096) return false;
                    if (stack->GetCount() && stack->IsEquipped()) return true;
                }
            }
            return false;
        }

        struct NativeGripEvent
        {
            RE::ButtonEvent& event;
            RE::BSFixedString name;
            float value, seconds;
            decltype(RE::InputEvent::handled) handled;
            explicit NativeGripEvent(RE::ButtonEvent& source) :
                event(source), name(source.strUserEvent), value(source.value),
                seconds(source.heldDownSecs), handled(source.handled)
            { event.strUserEvent = "WandGrip"; }
            ~NativeGripEvent()
            {
                event.strUserEvent = name;
                event.value = value;
                event.heldDownSecs = seconds;
                event.handled = handled;
            }
        };
    }

    float holdSeconds()
    {
        float seconds = 0.3f;
        if (available()) native_memory::tryReadValue(
            reinterpret_cast<const float*>(REL::Offset(0x37D5B18).address()), seconds);
        return std::isfinite(seconds) ? std::clamp(seconds, 0.05f, 10.0f) : 0.3f;
    }

    bool active() { return s_active; }

    bool begin(RE::ButtonEvent& source)
    {
        if (s_active || !available()) return false;
        if (!hasEquippedThrowable()) {
            ROCK_LOG_DEBUG(Input, "Vanilla grenade hold rejected: no equipped throwable");
            return false;
        }
        auto* handler = resolveHandler();
        if (!handler) return false;
        std::uint8_t enabled{}, armed{};
        if (!native_memory::tryReadField(handler, 0x8, enabled) || !enabled ||
            !native_memory::tryReadField(handler, 0x2A, armed) || armed) {
            ROCK_LOG_DEBUG(Input, "Vanilla grenade hold rejected: handler disabled or already armed");
            return false;
        }
        NativeGripEvent restore{source};
        source.value = 1.0f;
        source.heldDownSecs = 0.0f;
        source.handled = RE::InputEvent::HANDLED_RESULT::kUnhandled;
        const auto eligible = reinterpret_cast<ShouldHandle>(REL::Offset(kShouldHandle).address());
        if (!eligible(handler, &source)) {
            ROCK_LOG_DEBUG(Input, "Vanilla grenade hold rejected by native gameplay gates");
            return false;
        }
        const auto process = reinterpret_cast<Process>(REL::Offset(kProcess).address());
        process(handler, &source, nullptr, nullptr);
        source.heldDownSecs = holdSeconds();
        source.handled = RE::InputEvent::HANDLED_RESULT::kUnhandled;
        process(handler, &source, nullptr, nullptr);
        s_active = native_memory::tryReadField(handler, 0x2A, armed) && armed != 0;
        ROCK_LOG_DEBUG(Input, "Vanilla grenade hold submitted: armed={}", s_active);
        return s_active;
    }

    void cancel()
    {
        if (!s_active) return;
        void* ui{};
        int mode{};
        if (!native_memory::tryReadValue(reinterpret_cast<void**>(REL::Offset(kVrUi).address()), ui) ||
            !native_memory::tryReadField(ui, 0x36C, mode)) {
            ROCK_LOG_SAMPLE_WARN(Input, g_rockConfig.rockLogSampleMilliseconds,
                "Vanilla grenade cancellation pending: stage=VR-UI-mode");
            return;
        }
        // Another native mode has already performed its own exit cleanup.
        if (mode == 4 && RE::PlayerCharacter::GetSingleton()) {
            reinterpret_cast<void (*)(void*)>(REL::Offset(kRestoreMode).address())(ui);
            if (!native_memory::tryReadField(ui, 0x36C, mode) || mode == 4) {
                ROCK_LOG_SAMPLE_WARN(Input, g_rockConfig.rockLogSampleMilliseconds,
                    "Vanilla grenade cancellation pending: native mode exit deferred");
                return;
            }
        }
        s_active = false;
        ROCK_LOG_DEBUG(Input, "Vanilla grenade hold ended; native mode={}", mode);
    }

    void release(RE::ButtonEvent& source)
    {
        if (!s_active) return;
        auto* handler = resolveHandler();
        if (handler) {
            NativeGripEvent restore{source};
            source.value = 0.0f;
            source.heldDownSecs = (std::max)(source.heldDownSecs, holdSeconds());
            source.handled = RE::InputEvent::HANDLED_RESULT::kUnhandled;
            if (reinterpret_cast<ShouldHandle>(REL::Offset(kShouldHandle).address())(handler, &source)) {
                reinterpret_cast<Process>(REL::Offset(kProcess).address())(handler, &source, nullptr, nullptr);
                ROCK_LOG_DEBUG(Input, "Vanilla grenade release submitted to native handler");
            } else {
                ROCK_LOG_DEBUG(Input, "Vanilla grenade release rejected by native gameplay gates; cancelling");
            }
        }
        cancel();
    }
}
