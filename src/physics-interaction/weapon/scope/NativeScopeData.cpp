#include "physics-interaction/weapon/scope/NativeScopeData.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/weapon/ManualScopeTargetPolicy.h"
#include "physics-interaction/weapon/scope/NativeScopeAdmissionStub.h"

#include "RE/Bethesda/IMenu.h"
#include "RE/Bethesda/UIMessage.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Scaleform/GFx/GFx_Player.h"

#include <array>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstring>
#include <limits>
#include <Windows.h>

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
        using ProcessScopeMessage = RE::UI_MESSAGE_RESULTS (*)(RE::IMenu*, RE::UIMessage&);

        ConfigureHousing s_configureHousing = nullptr;
        SetScopeVisibility s_setScopeVisibility = nullptr;
        InvokeOverlay s_invokeOverlay = nullptr;
        ScopeFieldOfView s_scopeFieldOfView = nullptr;
        bool s_installed = false;
        ManualScopeQuery s_manualScopeQuery = nullptr;
        ProcessScopeMessage s_processScopeMessage = nullptr;
        std::atomic<DWORD> s_gameThread{ 0 };
        std::atomic<std::uint32_t> s_geometryQueries{ 0 };
        std::atomic<std::uint32_t> s_menuQueries{ 0 };
        std::atomic<std::uint32_t> s_geometryAdmissions{ 0 };
        std::atomic<std::uint32_t> s_menuAdmissions{ 0 };
        std::atomic<std::uint32_t> s_wrongThreadQueries{ 0 };
        std::atomic<std::uint32_t> s_scopeMessageCount{ 0 };
        std::atomic<std::uint64_t> s_scopeMessageSample{ 0 };
        std::atomic<std::uint32_t> s_overlayInvocationCount{ 0 };
        std::atomic<std::uint64_t> s_overlaySample{ 0 };
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

        bool nativeHasScope(std::uint32_t nativeFlag, const void* weapon, const void* instance,
            native_scope_admission::Site site) noexcept
        {
            auto& queries = site == native_scope_admission::Site::Geometry ? s_geometryQueries : s_menuQueries;
            queries.fetch_add(1, std::memory_order_relaxed);
            if (nativeFlag != 0) {
                return true;
            }
            const auto gameThread = s_gameThread.load(std::memory_order_relaxed);
            if (gameThread == 0) {
                return false;
            }
            if (gameThread != GetCurrentThreadId()) {
                s_wrongThreadQueries.fetch_add(1, std::memory_order_relaxed);
                return false;
            }
            if (!s_manualScopeQuery || !s_manualScopeQuery(weapon, instance)) {
                return false;
            }
            auto& count = site == native_scope_admission::Site::Geometry ? s_geometryAdmissions : s_menuAdmissions;
            count.fetch_add(1, std::memory_order_relaxed);
            return true;
        }

        RE::UI_MESSAGE_RESULTS processScopeMessage(RE::IMenu* menu, RE::UIMessage& message)
        {
            // Observation only. Bethesda owns Show, its fade callback, Hide,
            // input-layer removal and the menu object's lifetime.
            const auto type = static_cast<std::uint32_t>(*message.type);
            std::uint32_t payload = 0xFFFFFFFFu;
            bool payloadValid = false;
            if (message.type == RE::UI_MESSAGE_TYPE::kShow || message.type == RE::UI_MESSAGE_TYPE::kUpdate) {
                const auto* data = message.QData();
                std::uintptr_t dataVtable = 0;
                // BSUIMessageData payload: native sender 0x1420D93F0 writes
                // vtable 0x142D56B40 and +0x28. Other Update payload classes
                // must not be interpreted as the scope begin/end enum.
                payloadValid = data && native_memory::tryReadValue(reinterpret_cast<const std::uintptr_t*>(data), dataVtable) &&
                    dataVtable == REL::Offset(0x2D56B40).address() && native_memory::tryReadField(data, 0x28, payload);
            }
            const bool movieBefore = menu->uiMovie != nullptr;
            const bool rootBefore = menu->menuObj.IsObject();
            const auto result = s_processScopeMessage(menu, message);
            const std::uint64_t sample = payload | (static_cast<std::uint64_t>(type & 0xFF) << 32) |
                (static_cast<std::uint64_t>(payloadValid) << 40) |
                (static_cast<std::uint64_t>(movieBefore) << 41) | (static_cast<std::uint64_t>(rootBefore) << 42) |
                (static_cast<std::uint64_t>(menu->uiMovie != nullptr) << 43) |
                (static_cast<std::uint64_t>(menu->menuObj.IsObject()) << 44) |
                (static_cast<std::uint64_t>(result) << 48);
            s_scopeMessageSample.store(sample, std::memory_order_relaxed);
            s_scopeMessageCount.fetch_add(1, std::memory_order_release);
            return result;
        }

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
            const auto authored = count == 1 && args && args[0].IsUInt() ? args[0].GetUInt() : 0xFFFFFFFFu;
            const auto selected = replace ? policy::kDefaultOverlayIndex : authored;
            s_overlaySample.store(authored | (static_cast<std::uint64_t>(selected & 0xFFFFu) << 32) |
                    (static_cast<std::uint64_t>(replace) << 48) | (static_cast<std::uint64_t>(applied) << 49),
                std::memory_order_relaxed);
            s_overlayInvocationCount.fetch_add(1, std::memory_order_release);
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

        template <std::size_t N>
        bool bytesMatch(std::uintptr_t site, const std::array<std::uint8_t, N>& expected)
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Offset(site).address()),
                       actual.data(), actual.size()) && actual == expected;
        }

        template <std::size_t N>
        bool makeBranch(std::uintptr_t site, const void* target, std::array<std::uint8_t, N>& patch)
        {
            const auto delta = static_cast<std::int64_t>(reinterpret_cast<std::uintptr_t>(target)) -
                static_cast<std::int64_t>(REL::Offset(site).address() + 5);
            if (delta < (std::numeric_limits<std::int32_t>::min)() || delta > (std::numeric_limits<std::int32_t>::max)()) {
                return false;
            }
            const auto relative = static_cast<std::int32_t>(delta);
            patch.fill(0x90);
            patch[0] = 0xE9;
            std::memcpy(patch.data() + 1, &relative, sizeof(relative));
            return true;
        }
    }

    bool install(ManualScopeQuery query)
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
        // Independent native readers of the HasScope bit: geometry keeps
        // the scope active; the player aiming transition admits ScopeMenu.
        constexpr std::array<std::uint8_t, 6> geometryExtract{ 0xC1, 0xEB, 0x15, 0x80, 0xE3, 0x01 };
        constexpr std::array<std::uint8_t, 5> menuExtract{ 0xC1, 0xE8, 0x15, 0x24, 0x01 };
        std::uintptr_t processMessage = 0;
        if (!query || !REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72 ||
            !bytesMatch(0xEF838E, geometryExtract) || !bytesMatch(0xF300CA, menuExtract) ||
            !native_memory::tryReadValue(reinterpret_cast<const std::uintptr_t*>(REL::Offset(0x2D5AC30).address()), processMessage) ||
            processMessage != REL::Offset(0xBC7F00).address() ||
            !callMatches(0xF0AAD9, 0xC8DC60) ||
            !callMatches(0xBC8073, 0x213BB80) ||
            !callMatches(0xD835AF, 0x332CB0) ||
            !callMatches(0xEFAAF2, 0xC8E340) ||
            !native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Offset(0xC8E340).address()),
                actualVisibilityPrefix.data(), actualVisibilityPrefix.size()) || actualVisibilityPrefix != visibilityPrefix) {
            ROCK_LOG_ERROR(Init, "Native scope data hooks rejected: admission, message, housing, visibility, SetOverlay, or mono FOV bytes differ");
            return false;
        }
        auto& trampoline = F4SE::GetTrampoline();
        native_scope_admission::Stub geometry(native_scope_admission::Site::Geometry,
            reinterpret_cast<std::uintptr_t>(&nativeHasScope), REL::Offset(0xEF8394).address());
        native_scope_admission::Stub menu(native_scope_admission::Site::Menu,
            reinterpret_cast<std::uintptr_t>(&nativeHasScope), REL::Offset(0xF300CF).address());
        if (trampoline.free_size() < geometry.getSize() + menu.getSize() + 128) {
            ROCK_LOG_ERROR(Init, "Native scope admission has insufficient trampoline space");
            return false;
        }
        const auto* geometryCode = trampoline.allocate(geometry);
        const auto* menuCode = trampoline.allocate(menu);
        std::array<std::uint8_t, 6> geometryBranch{};
        std::array<std::uint8_t, 5> menuBranch{};
        if (!makeBranch(0xEF838E, geometryCode, geometryBranch) || !makeBranch(0xF300CA, menuCode, menuBranch)) {
            ROCK_LOG_ERROR(Init, "Native scope admission trampoline is outside branch range");
            return false;
        }
        s_manualScopeQuery = query;
        s_processScopeMessage = reinterpret_cast<ProcessScopeMessage>(processMessage);
        s_setScopeVisibility = reinterpret_cast<SetScopeVisibility>(REL::Offset(0xC8E340).address());
        s_configureHousing = reinterpret_cast<ConfigureHousing>(
            trampoline.write_call<5>(REL::Offset(0xF0AAD9).address(), &configureHousing));
        s_invokeOverlay = reinterpret_cast<InvokeOverlay>(
            trampoline.write_call<5>(REL::Offset(0xBC8073).address(), &invokeOverlay));
        s_scopeFieldOfView = reinterpret_cast<ScopeFieldOfView>(
            trampoline.write_call<5>(REL::Offset(0xD835AF).address(), &scopeFieldOfView));
        REL::safe_write(REL::Offset(0x2D5AC30).address(), reinterpret_cast<std::uintptr_t>(&processScopeMessage));
        REL::safe_write(REL::Offset(0xEF838E).address(), geometryBranch.data(), geometryBranch.size());
        REL::safe_write(REL::Offset(0xF300CA).address(), menuBranch.data(), menuBranch.size());
        s_installed = true;
        ROCK_LOG_INFO(Init, "Native scope data hooks installed: native geometry/menu admission, authored overlay/zoom preferred, defaults overlay=6 zoom=4x");
        return true;
    }

    void beginGameFrame()
    {
        DWORD unset = 0;
        s_gameThread.compare_exchange_strong(unset, GetCurrentThreadId(), std::memory_order_relaxed);
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
        const auto geometryAdmissions = s_geometryAdmissions.load(std::memory_order_relaxed);
        const auto menuAdmissions = s_menuAdmissions.load(std::memory_order_relaxed);
        const auto wrongThread = s_wrongThreadQueries.load(std::memory_order_relaxed);
        const auto geometryQueries = s_geometryQueries.load(std::memory_order_relaxed);
        const auto menuQueries = s_menuQueries.load(std::memory_order_relaxed);
        static std::array<std::uint32_t, 4> lastAdmissions{};
        const std::array<std::uint32_t, 4> admissions{ geometryAdmissions, menuAdmissions, wrongThread, menuQueries };
        if (admissions != lastAdmissions) {
            lastAdmissions = admissions;
            ROCK_LOG_SAMPLE_DEBUG(Weapon, 1000, "Native scope admission geometry={} menu={} rejectedThread={} geometryQueries={} menuQueries={}",
                geometryAdmissions, menuAdmissions, wrongThread, geometryQueries, menuQueries);
        }
        static std::uint32_t lastMessages = 0;
        const auto messages = s_scopeMessageCount.load(std::memory_order_acquire);
        if (messages != lastMessages) {
            lastMessages = messages;
            const auto sample = s_scopeMessageSample.load(std::memory_order_relaxed);
            ROCK_LOG_SAMPLE_DEBUG(Weapon, 1000,
                "Native scope menu messages={} type={} payload={} payloadValid={} movieBefore={} rootBefore={} movieAfter={} rootAfter={} result={}",
                messages, (sample >> 32) & 0xFF, static_cast<std::uint32_t>(sample), (sample >> 40) & 1,
                (sample >> 41) & 1, (sample >> 42) & 1, (sample >> 43) & 1, (sample >> 44) & 1, sample >> 48);
        }
        static std::uint32_t lastOverlays = 0;
        const auto overlays = s_overlayInvocationCount.load(std::memory_order_acquire);
        if (overlays != lastOverlays) {
            lastOverlays = overlays;
            const auto sample = s_overlaySample.load(std::memory_order_relaxed);
            ROCK_LOG_SAMPLE_DEBUG(Weapon, 1000, "Native scope SetOverlay calls={} authored={} selected={} fallback={} applied={}",
                overlays, static_cast<std::uint32_t>(sample), (sample >> 32) & 0xFFFF, (sample >> 48) & 1, (sample >> 49) & 1);
        }
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
