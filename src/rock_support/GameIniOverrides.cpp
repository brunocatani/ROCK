#include "rock_support/GameIniOverrides.h"

#include "RockConfig.h"
#include "rock_support/Fo4VrRuntime.h"
#include "physics-interaction/PhysicsLog.h"

#include <Windows.h>
#include <array>
#include <atomic>
#include <cstring>
#include <limits>

namespace rock::game_ini_overrides
{
    namespace
    {
        struct Override
        {
            const char* name;
            RE::Setting::SETTING_TYPE type;
            // Non-owning, statically registered engine SettingT objects. Published
            // once before installing the hook; never retain a collection iterator.
            RE::Setting* setting = nullptr;
            std::atomic<int> desired{ 0 };
            std::atomic<bool> rejected{ false };
            bool rejectionLogged = false;  // Main thread only.
        };

        using Type = RE::Setting::SETTING_TYPE;
        constinit std::array<Override, 5> s_overrides{{
            { "bUseKickback:VR", Type::kBinary },
            { "bUseRecoil:VR", Type::kBinary },
            { "bIgnoreConeOfFireCalculationsForPlayer:VR", Type::kBinary },
            { "bLeftHandedMode:VR", Type::kBinary },
            { "iRotationType:VR", Type::kInt },
        }};
        static_assert(std::atomic<int>::is_always_lock_free);
        static_assert(std::atomic<bool>::is_always_lock_free);

        using Notify = void (*)(RE::Setting*);
        Notify s_originalNotify = nullptr;
        bool s_installed = false;

        std::array<int, 5> values(const RockConfigValues& config)
        {
            return { config.rockVrUseKickback, config.rockVrUseRecoil,
                config.rockVrIgnoreConeOfFireCalculationsForPlayer,
                config.rockVrLeftHandedMode, config.rockVrRotationType };
        }

        int read(const Override& entry)
        {
            return entry.type == Type::kBinary ? entry.setting->GetBinary() : entry.setting->GetInt();
        }

        void write(const Override& entry, int value)
        {
            if (entry.type == Type::kBinary) {
                entry.setting->SetBinary(value != 0);
            } else {
                entry.setting->SetInt(value);
            }
        }

        void onSettingChanged(RE::Setting* setting)
        {
            // Runs synchronously on the native writer's thread, before observers.
            // No allocation, logging, collection lookup, or cross-thread engine call.
            for (auto& entry : s_overrides) {
                if (entry.setting != setting) {
                    continue;
                }
                const int desired = entry.desired.load(std::memory_order_relaxed);
                if (read(entry) != desired) {
                    write(entry, desired);
                    entry.rejected.store(true, std::memory_order_relaxed);
                }
                break;
            }
            s_originalNotify(setting);
        }

        bool installNotifier()
        {
            // FO4VR 1.2.72: native INI reader 0x1D13230 (+0x3A6), settings
            // menu 0xBD9880 (handedness tail at +0xD6), and Papyrus SetINIBool
            // 0x14D70B0 (+0x99) / SetINIInt 0x14D7330 (+0x99) all notify here
            // after writing Setting+8. Enforce before the observer dispatch so
            // handedness aliases, wands, and UI never rebuild for a rejected value.
            constexpr std::uintptr_t kNotifyRva = 0x1D11C30;
            constexpr std::array<std::uint8_t, 10> kPrefix{
                0x48, 0x89, 0x5C, 0x24, 0x10, 0x57, 0x48, 0x83, 0xEC, 0x20
            };
            const auto target = REL::Module::get().base() + kNotifyRva;
            if (std::memcmp(reinterpret_cast<const void*>(target), kPrefix.data(), kPrefix.size()) != 0) {
                ROCK_LOG_ERROR(Init, "VR INI override notifier bytes differ at RVA 0x{:X}; enforcement not installed", kNotifyRva);
                return false;
            }

            // Relocate only MOV [RSP+10],RBX; PUSH RDI (two instructions, six
            // bytes). The later RIP-relative access stays at its original address.
            // The existing >=14-byte entry helper cannot relocate that access.
            constexpr std::size_t kStolenBytes = 6;
            constexpr std::size_t kTrampolineBytes = 28;  // prologue + absolute return jump + hook pointer
            auto& trampoline = F4SE::GetTrampoline();
            if (trampoline.free_size() < kTrampolineBytes) {
                ROCK_LOG_ERROR(Init, "VR INI override notifier has insufficient trampoline space");
                return false;
            }
            auto* code = static_cast<std::uint8_t*>(trampoline.allocate(kTrampolineBytes));
            const auto displacement = reinterpret_cast<std::intptr_t>(code + 20) -
                static_cast<std::intptr_t>(target + kStolenBytes);
            if (displacement < std::numeric_limits<std::int32_t>::min() ||
                displacement > std::numeric_limits<std::int32_t>::max()) {
                ROCK_LOG_ERROR(Init, "VR INI override notifier trampoline is out of branch range");
                return false;
            }
            std::memcpy(code, kPrefix.data(), kStolenBytes);
            constexpr std::array<std::uint8_t, 6> kAbsoluteJump{ 0xFF, 0x25, 0, 0, 0, 0 };
            std::memcpy(code + 6, kAbsoluteJump.data(), kAbsoluteJump.size());
            const auto resume = target + kStolenBytes;
            const auto hook = reinterpret_cast<std::uintptr_t>(&onSettingChanged);
            std::memcpy(code + 12, &resume, sizeof(resume));
            std::memcpy(code + 20, &hook, sizeof(hook));

            std::array<std::uint8_t, 6> patch = kAbsoluteJump;
            const auto relative = static_cast<std::int32_t>(displacement);
            std::memcpy(patch.data() + 2, &relative, sizeof(relative));
            DWORD protection = 0;
            constexpr DWORD kExecuteReadWrite = 0x40;
            if (!VirtualProtect(reinterpret_cast<void*>(target), patch.size(), kExecuteReadWrite, &protection)) {
                ROCK_LOG_ERROR(Init, "VR INI override notifier cannot change code protection: {}", GetLastError());
                return false;
            }
            s_originalNotify = reinterpret_cast<Notify>(code);
            std::memcpy(reinterpret_cast<void*>(target), patch.data(), patch.size());
            FlushInstructionCache(GetCurrentProcess(), code, kTrampolineBytes);
            FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<void*>(target), patch.size());
            DWORD ignored = 0;
            if (!VirtualProtect(reinterpret_cast<void*>(target), patch.size(), protection, &ignored)) {
                ROCK_LOG_ERROR(Init, "VR INI override notifier installed but code protection restoration failed: {}", GetLastError());
            }
            return true;
        }
    }

    bool install(const RockConfigValues& config)
    {
        if (s_installed) {
            return true;
        }
        const auto desired = values(config);
        for (std::size_t i = 0; i < s_overrides.size(); ++i) {
            auto& entry = s_overrides[i];
            entry.setting = f4vr::getIniSetting(entry.name);
            if (!entry.setting || entry.setting->GetType() != entry.type) {
                ROCK_LOG_ERROR(Init, "VR INI override '{}' is missing or has the wrong type; enforcement not installed", entry.name);
                return false;
            }
            entry.desired.store(desired[i], std::memory_order_relaxed);
        }
        if (!installNotifier()) {
            return false;
        }
        s_installed = true;
        for (auto& entry : s_overrides) {
            const int desiredValue = entry.desired.load(std::memory_order_relaxed);
            ROCK_LOG_INFO(Config, "VR INI override '{}': native={} ROCK={}", entry.name, read(entry), desiredValue);
            if (read(entry) != desiredValue) {
                write(entry, desiredValue);
                s_originalNotify(entry.setting);
            }
        }
        ROCK_LOG_INFO(Init, "ROCK owns all five VR INI overrides; native setting notifications are guarded");
        return true;
    }

    void update(const RockConfigValues& config)
    {
        if (!s_installed) {
            return;
        }
        const auto desired = values(config);
        for (std::size_t i = 0; i < s_overrides.size(); ++i) {
            auto& entry = s_overrides[i];
            const int previous = entry.desired.load(std::memory_order_relaxed);
            if (previous != desired[i]) {
                entry.desired.store(desired[i], std::memory_order_relaxed);
                ROCK_LOG_INFO(Config, "VR INI override '{}' changed in ROCK.ini: {} -> {}", entry.name, previous, desired[i]);
            }
            if (read(entry) != desired[i]) {
                if (previous == desired[i]) {
                    entry.rejected.store(true, std::memory_order_relaxed);
                }
                write(entry, desired[i]);
                s_originalNotify(entry.setting);
            }
            if (!entry.rejectionLogged && entry.rejected.load(std::memory_order_relaxed)) {
                entry.rejectionLogged = true;
                ROCK_LOG_INFO(Config, "VR INI override '{}' rejected an external change; ROCK.ini remains authoritative (reported once)", entry.name);
            }
        }
    }
}
