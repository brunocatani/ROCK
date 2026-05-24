#include "physics-interaction/native/WeaponWorkbenchGraphRefreshHook.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/WeaponEquippedModSignature.h"

#include "F4SE/F4SE.h"
#include "REL/Relocation.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>

namespace rock::weapon_workbench_graph_refresh
{
    namespace
    {
        constexpr std::uintptr_t kBuildConfirmedNativeRefreshCallSite = 0xB3AA24;
        constexpr std::uintptr_t kExamineMenuAttachModToPreviewSlotOffset = 0x120;
        constexpr std::array<std::uint8_t, 6> kExpectedBuildConfirmedRefreshCallBytes{
            0xFF, 0x90, 0x20, 0x01, 0x00, 0x00
        };

        using ExamineMenuRefresh_t = void (*)(void* examineMenu, char applySelection);

        std::atomic<bool> s_hookInstalled{ false };
        std::atomic<bool> s_hookInstallAttempted{ false };
        std::atomic<std::uint64_t> s_lastObservedEquippedSignature{ 0 };
        std::atomic<std::uint64_t> s_lastReplayedEquippedSignature{ 0 };
        std::atomic<std::uint64_t> s_refreshEpoch{ 0 };
        thread_local bool s_insideRefreshHook = false;

        [[nodiscard]] ExamineMenuRefresh_t resolveNativeRefresh(void* examineMenu) noexcept
        {
            if (!examineMenu) {
                return nullptr;
            }

            auto** vtable = *reinterpret_cast<void***>(examineMenu);
            if (!vtable) {
                return nullptr;
            }

            return reinterpret_cast<ExamineMenuRefresh_t>(vtable[kExamineMenuAttachModToPreviewSlotOffset / sizeof(void*)]);
        }

        [[nodiscard]] bool equippedWeaponSignatureChangedForBuild(const EquippedWeaponModSignature& signature) noexcept
        {
            if (!signature.hasEquippedWeapon || signature.key == 0) {
                return false;
            }

            const auto lastObserved = s_lastObservedEquippedSignature.load(std::memory_order_acquire);
            if (lastObserved == 0 || lastObserved == signature.key) {
                return false;
            }

            const auto lastReplayed = s_lastReplayedEquippedSignature.load(std::memory_order_acquire);
            return lastReplayed != signature.key;
        }

        void buildConfirmedRefreshHook(void* examineMenu, char applySelection)
        {
            const auto nativeRefresh = resolveNativeRefresh(examineMenu);
            if (!nativeRefresh) {
                ROCK_LOG_ERROR(Weapon, "Workbench equipped weapon graph refresh hook skipped: native refresh slot unavailable");
                return;
            }

            if (s_insideRefreshHook) {
                nativeRefresh(examineMenu, applySelection);
                return;
            }

            s_insideRefreshHook = true;
            nativeRefresh(examineMenu, applySelection);

            if (applySelection != 0 && g_rockConfig.rockEnabled && g_rockConfig.rockWeaponCollisionEnabled) {
                const auto signature = readEquippedWeaponModSignature();
                if (equippedWeaponSignatureChangedForBuild(signature)) {
                    nativeRefresh(examineMenu, applySelection);
                    s_lastReplayedEquippedSignature.store(signature.key, std::memory_order_release);
                    const auto epoch = s_refreshEpoch.fetch_add(1, std::memory_order_acq_rel) + 1;
                    ROCK_LOG_INFO(Weapon,
                        "Workbench equipped weapon graph refresh replayed callsite=0x{:X} formID={:08X} previousSignature={:016X} currentSignature={:016X} epoch={} activeMods={} disabledMods={}",
                        kBuildConfirmedNativeRefreshCallSite,
                        signature.formID,
                        s_lastObservedEquippedSignature.load(std::memory_order_acquire),
                        signature.key,
                        epoch,
                        signature.activeModCount,
                        signature.disabledModCount);
                } else {
                    ROCK_LOG_DEBUG(Weapon,
                        "Workbench equipped weapon graph refresh hook observed no equipped weapon mod signature transition currentSignature={:016X}",
                        signature.key);
                }
            }

            s_insideRefreshHook = false;
        }

        [[nodiscard]] bool nativeCallsiteMatchesExpected(std::uintptr_t address) noexcept
        {
            const auto* bytes = reinterpret_cast<const std::uint8_t*>(address);
            return bytes && std::memcmp(bytes, kExpectedBuildConfirmedRefreshCallBytes.data(), kExpectedBuildConfirmedRefreshCallBytes.size()) == 0;
        }
    }

    bool installHook()
    {
        if (s_hookInstalled.load(std::memory_order_acquire)) {
            return true;
        }

        bool expectedAttempted = false;
        if (!s_hookInstallAttempted.compare_exchange_strong(expectedAttempted, true, std::memory_order_acq_rel)) {
            return s_hookInstalled.load(std::memory_order_acquire);
        }

        REL::Relocation<std::uintptr_t> callsite{ REL::Offset(kBuildConfirmedNativeRefreshCallSite) };
        if (!nativeCallsiteMatchesExpected(callsite.address())) {
            ROCK_LOG_ERROR(Init,
                "Workbench equipped weapon graph refresh hook validation failed at 0x{:X}; expected ExamineMenu::BuildConfirmed native refresh call bytes",
                callsite.address());
            return false;
        }

        auto& trampoline = F4SE::GetTrampoline();
        trampoline.write_call<6>(callsite.address(), &buildConfirmedRefreshHook);
        s_hookInstalled.store(true, std::memory_order_release);
        ROCK_LOG_INFO(Init,
            "Installed workbench equipped weapon graph refresh hook at 0x{:X}; replays ExamineMenu visual refresh only after equipped weapon mod signature transitions",
            callsite.address());
        return true;
    }

    void publishObservedEquippedWeaponSignature(std::uint64_t signatureKey) noexcept
    {
        if (signatureKey != 0) {
            s_lastObservedEquippedSignature.store(signatureKey, std::memory_order_release);
        }
    }

    std::uint64_t currentRefreshEpoch() noexcept
    {
        return s_refreshEpoch.load(std::memory_order_acquire);
    }
}
