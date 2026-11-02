#include "physics-interaction/native/NativeWristPipboyLook.h"

#include "api/FRIKApiV2.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/NativeMemory.h"
#include "rock_support/Fo4VrRuntime.h"

#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/Settings.h"
#include <REL/Relocation.h>
#include <Windows.h>

#include <array>
#include <cstdint>
#include <string_view>

namespace rock::native_wrist_pipboy_look
{
    namespace
    {
        // Raw FO4VR 1.2.72 at 140EF1AC8: `mov rcx, rdi; call 140EF67B0`, the
        // player in RCX; the next instruction overwrites RBX and nothing reads
        // the result. 140EF67B0 has no other caller. Its prologue includes the
        // left-wand test `cmp qword ptr [rcx+768h], 0`.
        constexpr std::uintptr_t kLookUpdateCallSite = 0xEF1AC8;
        constexpr std::uintptr_t kLookUpdateCall = kLookUpdateCallSite + 3;
        constexpr std::uintptr_t kLookUpdateEntry = 0xEF67B0;
        constexpr std::array<std::uint8_t, 8> kExpectedCallSite{ 0x48, 0x8B, 0xCF, 0xE8, 0xE0, 0x4C, 0x00, 0x00 };
        constexpr std::array<std::uint8_t, 20> kExpectedEntry{
            0x40, 0x53, 0x48, 0x81, 0xEC, 0x00, 0x01, 0x00, 0x00, 0x48,
            0x83, 0xB9, 0x68, 0x07, 0x00, 0x00, 0x00, 0x48, 0x8B, 0xD9,
        };

        // Static Setting objects (value at +8, name at +0x10). 140EF67B0 reads
        // the angle values at 1437CF078/090/0A8/0C0; the mode values at
        // 1437B4280/298 are the ones FRIK's wrist test reads. Install checks
        // every name.
        enum SettingIndex : std::size_t
        {
            kPipboyInner,
            kPipboyOuter,
            kHmdInner,
            kHmdOuter,
            kAngleCount,
            kProjected = kAngleCount,
            kAttachedToHmd,
            kSettingCount,
        };

        struct KnownSetting
        {
            std::uintptr_t rva;
            std::string_view name;
        };

        constexpr std::array<KnownSetting, kSettingCount> kSettings{ {
            { 0x37CF070, "fPipboyScaleInnerAngle:VRPipboy" },
            { 0x37CF088, "fPipboyScaleOuterAngle:VRPipboy" },
            { 0x37CF0A0, "fHMDToPipboyScaleInnerAngle:VRPipboy" },
            { 0x37CF0B8, "fHMDToPipboyScaleOuterAngle:VRPipboy" },
            { 0x37B4278, "bAlwaysUseProjectedPipboy:VR" },
            { 0x37B4290, "bAttachPipboyToHMD:VR" },
        } };

        // 140EF67B0 maps each angle to (angle - outer) / (inner - outer) and
        // clamps it to [0, 1], unordered results to 0. With inner below an
        // outer of 0, every angle maps to 0 or less: no gaze is a look.
        constexpr float kNoLookOuterDegrees = 0.0f;
        constexpr float kNoLookInnerDegrees = -1.0f;

        enum class Decision : std::uint8_t
        {
            Unset,
            NoFrik,
            NotOnWrist,
            PowerArmor,
            FrikPipboyBlocked,
            FrikPipboyOpen,
            AnglesForcedOpen,
            FrikPipboyClosed,
        };

        using LookUpdate = void (*)(RE::PlayerCharacter*);

        // Written once by install() before the call is redirected; read only
        // by updateLook() on the main thread (PlayerCharacter::Update).
        LookUpdate s_originalLookUpdate{};
        std::array<RE::Setting*, kSettingCount> s_settings{};
        bool s_installed{ false };
        Decision s_loggedDecision{ Decision::Unset };

        [[nodiscard]] const char* describe(const Decision decision) noexcept
        {
            switch (decision) {
            case Decision::NoFrik:
                return "active: FRIK API unavailable";
            case Decision::NotOnWrist:
                return "active: projected or HMD Pip-Boy";
            case Decision::PowerArmor:
                return "active: power armor";
            case Decision::FrikPipboyBlocked:
                return "active: FRIK Pip-Boy feature blocked";
            case Decision::FrikPipboyOpen:
                return "active: FRIK Pip-Boy open";
            case Decision::AnglesForcedOpen:
                return "active: look angles forced open";
            case Decision::FrikPipboyClosed:
                return "off: FRIK Pip-Boy closed";
            case Decision::Unset:
                break;
            }
            return "unset";
        }

        [[nodiscard]] Decision decide() noexcept
        {
            const auto* frik = frik::api::FRIKApiV2::inst;
            if (!frik || !frik->isWristPipboyOpen) return Decision::NoFrik;
            if (s_settings[kProjected]->GetBinary() || s_settings[kAttachedToHmd]->GetBinary()) return Decision::NotOnWrist;
            if (f4vr::isInPowerArmor()) return Decision::PowerArmor;
            if (frik->isFeatureBlocked && frik->isFeatureBlocked(frik::api::FRIKApiV2::Feature::Pipboy)) return Decision::FrikPipboyBlocked;
            if (frik->isWristPipboyOpen()) return Decision::FrikPipboyOpen;
            // Equal inner and outer angles divide by zero in 140EF67B0, which
            // counts every gaze as a look: FRIK opens with all four at 0.
            if (s_settings[kPipboyInner]->GetFloat() == s_settings[kPipboyOuter]->GetFloat() &&
                s_settings[kHmdInner]->GetFloat() == s_settings[kHmdOuter]->GetFloat()) {
                return Decision::AnglesForcedOpen;
            }
            return Decision::FrikPipboyClosed;
        }

        void logDecision(const Decision decision) noexcept
        {
            if (decision == s_loggedDecision) return;
            s_loggedDecision = decision;
            try {
                ROCK_LOG_INFO(Input, "Native wrist Pip-Boy look detection {}", describe(decision));
            } catch (...) {
                OutputDebugStringA("ROCK: wrist Pip-Boy look decision log failed\n");
            }
        }

        // Holds the four look angles at kNoLook* for one native update and
        // restores the values FRIK set before anything else reads them.
        class NoLookAngles
        {
        public:
            NoLookAngles() noexcept
            {
                for (std::size_t index = 0; index < kAngleCount; ++index) {
                    _saved[index] = s_settings[index]->GetFloat();
                    const bool inner = index == kPipboyInner || index == kHmdInner;
                    s_settings[index]->SetFloat(inner ? kNoLookInnerDegrees : kNoLookOuterDegrees);
                }
            }

            ~NoLookAngles()
            {
                for (std::size_t index = 0; index < kAngleCount; ++index) {
                    s_settings[index]->SetFloat(_saved[index]);
                }
            }

            NoLookAngles(const NoLookAngles&) = delete;
            NoLookAngles& operator=(const NoLookAngles&) = delete;

        private:
            std::array<float, kAngleCount> _saved{};
        };

        void updateLook(RE::PlayerCharacter* player) noexcept
        {
            const Decision decision = decide();
            logDecision(decision);
            if (decision != Decision::FrikPipboyClosed) {
                s_originalLookUpdate(player);
                return;
            }
            const NoLookAngles noLook;
            s_originalLookUpdate(player);
        }

        [[nodiscard]] bool resolveSetting(const KnownSetting& known, RE::Setting*& out) noexcept
        {
            auto* setting = reinterpret_cast<RE::Setting*>(REL::Offset(known.rva).address());
            const char* key = nullptr;
            std::array<char, 48> text{};
            static_assert(sizeof(RE::Setting) == 0x18);
            if (known.name.size() >= text.size() || !native_memory::tryReadField(setting, 0x10, key) || !key ||
                !native_memory::guardedCopyFromMemory(key, text.data(), known.name.size() + 1) ||
                std::string_view(text.data()) != known.name) {
                return false;
            }
            out = setting;
            return true;
        }

        template <std::size_t N>
        [[nodiscard]] bool bytesMatch(const std::uintptr_t rva, const std::array<std::uint8_t, N>& expected) noexcept
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Offset(rva).address()), actual.data(), actual.size()) &&
                   actual == expected;
        }
    }

    bool install() noexcept
    {
        if (s_installed) return true;
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) return false;
        std::array<RE::Setting*, kSettingCount> settings{};
        for (std::size_t index = 0; index < kSettingCount; ++index) {
            if (!resolveSetting(kSettings[index], settings[index])) {
                ROCK_LOG_ERROR(Init, "Native wrist Pip-Boy look gate unavailable: setting '{}' not found at {:#x}",
                    kSettings[index].name, kSettings[index].rva);
                return false;
            }
        }
        if (!bytesMatch(kLookUpdateCallSite, kExpectedCallSite) || !bytesMatch(kLookUpdateEntry, kExpectedEntry)) {
            ROCK_LOG_ERROR(Init, "Native wrist Pip-Boy look gate unavailable: look update call {:#x} or entry {:#x} changed",
                kLookUpdateCall, kLookUpdateEntry);
            return false;
        }
        auto& trampoline = F4SE::GetTrampoline();
        if (trampoline.free_size() < 14) {
            ROCK_LOG_ERROR(Init, "Native wrist Pip-Boy look gate unavailable: trampoline capacity");
            return false;
        }
        // Plugin load: PlayerCharacter::Update cannot run before these are set.
        s_settings = settings;
        s_originalLookUpdate = reinterpret_cast<LookUpdate>(REL::Offset(kLookUpdateEntry).address());
        const auto address = REL::Offset(kLookUpdateCall).address();
        try {
            (void)trampoline.write_call<5>(address, &updateLook);
        } catch (...) {
            ROCK_LOG_ERROR(Init, "Native wrist Pip-Boy look gate unavailable: call redirect failed");
            return false;
        }
        FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<const void*>(address), 5);
        s_installed = true;
        ROCK_LOG_INFO(Init, "Native wrist Pip-Boy look gate installed at {:#x}: a closed FRIK wrist Pip-Boy ignores gaze", kLookUpdateCall);
        return true;
    }
}
