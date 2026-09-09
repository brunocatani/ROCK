#include "physics-interaction/weapon/scope/NativeScopeData.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/weapon/ManualScopeTargetPolicy.h"

#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Scaleform/GFx/GFx_Player.h"

#include <array>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstring>

namespace rock::native_scope_data
{
    namespace
    {
        namespace policy = manual_scope_target_policy;
        using Value = RE::Scaleform::GFx::Value;
        using ConfigureHousing = void (*)(void*, std::uint32_t);
        using SetScopeVisibility = void (*)(void*, bool);
        using InvokeOverlay = bool (*)(Value::ObjectInterface*, void*, Value*, const char*, const Value*, std::size_t, bool);
        using ScopeFieldOfView = float (*)(RE::TESObjectWEAP*, float, RE::TESObjectWEAP::InstanceData*);

        ConfigureHousing s_configureHousing = nullptr;
        SetScopeVisibility s_setScopeVisibility = nullptr;
        InvokeOverlay s_invokeOverlay = nullptr;
        ScopeFieldOfView s_scopeFieldOfView = nullptr;
        bool s_installed = false;
        // Callbacks run synchronously on their native UI/render/equip threads.
        // Only diagnostic counters cross those threads; no engine pointers do.
        std::atomic<std::uint32_t> s_overlayFallbacks{ 0 };
        std::atomic<std::uint32_t> s_overlayFailures{ 0 };
        std::atomic<std::uint32_t> s_zoomFallbackFrames{ 0 };
        std::atomic<std::uint32_t> s_zoomReadFailures{ 0 };
        std::atomic<std::uint32_t> s_zoomReadFailureStage{ 0 };
        std::atomic<std::uint64_t> s_projectionSample{ 0 };
        std::atomic<bool> s_housingStateFailure{ false };
        static_assert(std::atomic<std::uint64_t>::is_always_lock_free);

        void configureHousing(void* worldScope, std::uint32_t overlay)
        {
            if (!configureManual(worldScope, overlay)) {
                // Native equip still owns this call if the optional repair
                // cannot establish the renderer's current state.
                s_configureHousing(worldScope, overlay);
            }
        }

        bool invokeOverlay(Value::ObjectInterface* objectInterface, void* object, Value* result,
            const char* method, const Value* args, std::size_t count, bool displayObject)
        {
            // This is ScopeMenu's SetOverlay call, not a global Scaleform hook.
            // Use a value owned by this callback so neither the weapon's ZOOM
            // nor native caller's argument storage needs restoration.
            const bool replace = count == 1 && args && args[0].IsUInt() &&
                !policy::isValidNativeOverlayIndex(args[0].GetUInt());
            Value fallback(policy::kDefaultOverlayIndex);
            const bool applied = s_invokeOverlay(objectInterface, object, result, method,
                replace ? &fallback : args, count, displayObject);
            if (replace) {
                s_overlayFallbacks.fetch_add(1, std::memory_order_relaxed);
            }
            if (!applied) {
                s_overlayFailures.fetch_add(1, std::memory_order_relaxed);
            }
            return applied;
        }

        float scopeFieldOfView(RE::TESObjectWEAP* weapon, float baseFov, RE::TESObjectWEAP::InstanceData* instance)
        {
            // 0x140D835AF is exclusively the native mono-scope render pass.
            // Its RCX/R8 are held by the caller for the entire render. The
            // same instance/base ZOOM fields are read by 0x140332CB0 and
            // 0x140332B80; +0x20 is magnification, not a degree-valued FOV.
            // Do not hook the general weapon getter: that would change ADS,
            // NPC cameras, and the separate world/first-person FOV paths.
            if (!weapon || !std::isfinite(baseFov) || baseFov <= 0.0f || baseFov >= 180.0f) {
                s_zoomReadFailureStage.store(1, std::memory_order_relaxed);
                s_zoomReadFailures.fetch_add(1, std::memory_order_relaxed);
                return 90.0f;
            }
            RE::BGSZoomData* zoom = nullptr;
            if (instance && !native_memory::tryReadValue(&instance->zoomData, zoom)) {
                s_zoomReadFailureStage.store(2, std::memory_order_relaxed);
                s_zoomReadFailures.fetch_add(1, std::memory_order_relaxed);
                return baseFov;
            }
            if (!zoom && !native_memory::tryReadValue(&weapon->weaponData.zoomData, zoom)) {
                s_zoomReadFailureStage.store(3, std::memory_order_relaxed);
                s_zoomReadFailures.fetch_add(1, std::memory_order_relaxed);
                return baseFov;
            }
            float magnification = 0.0f;
            if (zoom && !native_memory::tryReadValue(&zoom->zoomData.fovMult, magnification)) {
                s_zoomReadFailureStage.store(4, std::memory_order_relaxed);
                s_zoomReadFailures.fetch_add(1, std::memory_order_relaxed);
                return baseFov;
            }
            const bool authored = policy::isValidMagnification(magnification);
            const float renderedFov = authored ? s_scopeFieldOfView(weapon, baseFov, instance) :
                policy::magnifiedFieldOfView(baseFov, policy::kDefaultMagnification);
            if (!authored) {
                s_zoomFallbackFrames.fetch_add(1, std::memory_order_relaxed);
            }
            // One coherent pair, sampled later on the game thread. This
            // records the native result as well as the selected zoom so an
            // in-game report need not infer projection from an OMOD label.
            const auto zoomBits = std::bit_cast<std::uint32_t>(policy::resolveMagnification(magnification));
            s_projectionSample.store((static_cast<std::uint64_t>(zoomBits) << 32) |
                    std::bit_cast<std::uint32_t>(renderedFov), std::memory_order_relaxed);
            return renderedFov;
        }

        [[nodiscard]] bool callMatches(std::uintptr_t site, std::uintptr_t target)
        {
            std::array<std::uint8_t, 5> bytes{};
            const auto address = REL::Offset(site).address();
            std::int32_t relative = 0;
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address), bytes.data(), bytes.size()) ||
                bytes[0] != 0xE8) {
                return false;
            }
            std::memcpy(&relative, bytes.data() + 1, sizeof(relative));
            return address + 5 + relative == REL::Offset(target).address();
        }
    }

    bool install()
    {
        if (s_installed) {
            return true;
        }
        // Raw FO4VR 1.2.72 witnesses: equip 0x140F0A9F0 and ScopeMenu
        // 0x140BC7F00 independently read the ZOOM overlay at +0x24. The
        // mono renderer 0x140D831F0 passes its weapon/instance and base FOV
        // to 0x140332CB0, which implements 2*atan(tan(FOV/2)/zoom).
        constexpr std::array<std::uint8_t, 10> visibilityPrefix{
            0x48, 0x89, 0x5C, 0x24, 0x08, 0x57, 0x48, 0x83, 0xEC, 0x20
        };
        std::array<std::uint8_t, visibilityPrefix.size()> actualVisibilityPrefix{};
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72 ||
            !callMatches(0xF0AAD9, 0xC8DC60) ||
            !callMatches(0xBC8073, 0x213BB80) ||
            !callMatches(0xD835AF, 0x332CB0) ||
            !callMatches(0xEFAAF2, 0xC8E340) ||
            !native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Offset(0xC8E340).address()),
                actualVisibilityPrefix.data(), actualVisibilityPrefix.size()) || actualVisibilityPrefix != visibilityPrefix) {
            ROCK_LOG_ERROR(Init, "Native scope data hooks rejected: housing, visibility, SetOverlay, or mono FOV bytes differ");
            return false;
        }
        auto& trampoline = F4SE::GetTrampoline();
        s_setScopeVisibility = reinterpret_cast<SetScopeVisibility>(REL::Offset(0xC8E340).address());
        s_configureHousing = reinterpret_cast<ConfigureHousing>(
            trampoline.write_call<5>(REL::Offset(0xF0AAD9).address(), &configureHousing));
        s_invokeOverlay = reinterpret_cast<InvokeOverlay>(
            trampoline.write_call<5>(REL::Offset(0xBC8073).address(), &invokeOverlay));
        s_scopeFieldOfView = reinterpret_cast<ScopeFieldOfView>(
            trampoline.write_call<5>(REL::Offset(0xD835AF).address(), &scopeFieldOfView));
        s_installed = true;
        ROCK_LOG_INFO(Init, "Native scope data hooks installed: authored overlay/zoom preferred, defaults overlay=6 zoom=4x");
        return true;
    }

    bool configureManual(void* worldScope, std::uint32_t overlay)
    {
        if (!s_installed || !worldScope) {
            return false;
        }
        std::uint8_t active = 0;
        // Getter 0x141D947B0 and setter 0x141D947A0 independently witness
        // request byte +3. 0x140EFAA60 uses the same byte and calls
        // 0x140C8E340 to show/hide WSScope's current lens layers.
        if (!native_memory::tryReadValue(reinterpret_cast<const std::uint8_t*>(
                REL::Offset(0x6239343).address()), active) || active > 1) {
            s_housingStateFailure.store(true, std::memory_order_relaxed);
            return false;
        }
        s_configureHousing(worldScope, policy::resolveOverlay(overlay));
        // Configure hides both lens layers even when the renderer is already
        // active. A repeated state transition will not show them: Bethesda
        // returns early when its activation byte has not changed.
        if (active != 0) {
            s_setScopeVisibility(worldScope, true);
        }
        return true;
    }

    void reportDiagnostics()
    {
        // Game-frame observer, never called by the UI/render callbacks. Each
        // category reports once per session, avoiding render-loop log churn.
        static bool overlayReported = false;
        static bool zoomReported = false;
        static bool failureReported = false;
        static std::uint64_t lastProjection = 0;
        const auto projection = s_projectionSample.load(std::memory_order_relaxed);
        if (projection != 0 && projection != lastProjection) {
            lastProjection = projection;
            ROCK_LOG_SAMPLE_DEBUG(Weapon, 1000,
                "Native scope projection magnification={} fieldOfViewDegrees={}",
                std::bit_cast<float>(static_cast<std::uint32_t>(projection >> 32)),
                std::bit_cast<float>(static_cast<std::uint32_t>(projection)));
        }
        if (!overlayReported && s_overlayFallbacks.load(std::memory_order_relaxed) != 0) {
            overlayReported = true;
            ROCK_LOG_INFO(Weapon, "Native scope UI selected standard overlay 6 for missing/invalid authored overlay");
        }
        if (!zoomReported && s_zoomFallbackFrames.load(std::memory_order_relaxed) != 0) {
            zoomReported = true;
            ROCK_LOG_INFO(Weapon, "Native mono scope renderer selected 4x for missing/invalid authored magnification");
        }
        if (!failureReported && (s_overlayFailures.load(std::memory_order_relaxed) != 0 ||
                                   s_zoomReadFailures.load(std::memory_order_relaxed) != 0 ||
                                   s_housingStateFailure.load(std::memory_order_relaxed))) {
            failureReported = true;
            ROCK_LOG_WARN(Weapon, "Native scope presentation failed: SetOverlay failures={} zoom read failures={} lastReadStage={} (1=input 2=instance 3=base 4=ZOOM) housingStateFailure={}",
                s_overlayFailures.load(std::memory_order_relaxed), s_zoomReadFailures.load(std::memory_order_relaxed),
                s_zoomReadFailureStage.load(std::memory_order_relaxed), s_housingStateFailure.load(std::memory_order_relaxed));
        }
    }
}
