#include "physics-interaction/input/DebugControllerRuntime.h"

#include "physics-interaction/input/DebugControllerPolicy.h"

#include "RockConfig.h"
#include "f4vr/F4VRUtils.h"
#include "physics-interaction/PhysicsLog.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <format>
#include <string>
#include <string_view>

#include <windows.h>

namespace rock::debug_controller_runtime
{
    namespace
    {
        /*
         * The debug controller is a live tuning aid, not a second gameplay input
         * stack. It polls XInput directly because SCP Toolkit exposes the PS3 pad
         * as an Xbox controller, while all game-facing VR input remains owned by
         * the existing OpenVR hook. The runtime only changes existing ROCK config
         * fields, and persistence is routed through RockConfig so the INI watcher
         * understands self-authored writes.
         */
        constexpr DWORD kMaxXInputControllers = 4;
        constexpr DWORD kXInputSuccess = 0;
        constexpr float kNotificationIntervalSeconds = 0.35f;

        struct XInputGamepad
        {
            std::uint16_t buttons;
            std::uint8_t leftTrigger;
            std::uint8_t rightTrigger;
            std::int16_t thumbLX;
            std::int16_t thumbLY;
            std::int16_t thumbRX;
            std::int16_t thumbRY;
        };

        struct XInputState
        {
            std::uint32_t packetNumber;
            XInputGamepad gamepad;
        };

        using XInputGetState_t = DWORD(WINAPI*)(DWORD, XInputState*);

        struct ControllerSample
        {
            bool connected{ false };
            std::uint16_t buttons{ 0 };
            debug_controller_policy::AnalogSample analog{};
        };

        struct TuningVisualState
        {
            bool saved{ false };
            bool showGrabPivots{ false };
            bool showPalmVectors{ false };
        };

        struct RuntimeState
        {
            bool hadPreviousButtons{ false };
            std::uint16_t previousButtons{ 0 };
            bool pivotTuningActive{ false };
            bool selectedLeft{ false };
            std::array<bool, 2> pivotDirty{};
            TuningVisualState tuningVisualState{};
            float notificationElapsedSeconds{ kNotificationIntervalSeconds };
        };

        struct XInputLoader
        {
            HMODULE module{ nullptr };
            XInputGetState_t getState{ nullptr };
            bool loadAttempted{ false };
            bool missingLogged{ false };
        };

        RuntimeState s_state;
        XInputLoader s_xinput;

        [[nodiscard]] float sanitizeDeltaSeconds(float deltaSeconds)
        {
            if (deltaSeconds > 0.0f && deltaSeconds <= 0.1f) {
                return deltaSeconds;
            }

            return 1.0f / 90.0f;
        }

        [[nodiscard]] float normalizeThumb(std::int16_t value)
        {
            if (value >= 0) {
                return static_cast<float>(value) / 32767.0f;
            }

            return static_cast<float>(value) / 32768.0f;
        }

        [[nodiscard]] bool ensureXInputLoaded()
        {
            if (s_xinput.getState) {
                return true;
            }

            if (s_xinput.loadAttempted && !s_xinput.module) {
                return false;
            }

            s_xinput.loadAttempted = true;
            constexpr std::array<std::string_view, 3> candidates{ "xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll" };
            for (const auto dllName : candidates) {
                s_xinput.module = LoadLibraryA(std::string(dllName).c_str());
                if (!s_xinput.module) {
                    continue;
                }

                s_xinput.getState = reinterpret_cast<XInputGetState_t>(GetProcAddress(s_xinput.module, "XInputGetState"));
                if (s_xinput.getState) {
                    ROCK_LOG_INFO(Input, "ROCK debug controller loaded {}", dllName);
                    return true;
                }

                FreeLibrary(s_xinput.module);
                s_xinput.module = nullptr;
            }

            if (!s_xinput.missingLogged) {
                s_xinput.missingLogged = true;
                ROCK_LOG_WARN(Input, "ROCK debug controller disabled: no XInput DLL with XInputGetState was available");
            }
            return false;
        }

        [[nodiscard]] ControllerSample pollController()
        {
            ControllerSample sample{};
            if (!ensureXInputLoaded()) {
                return sample;
            }

            for (DWORD userIndex = 0; userIndex < kMaxXInputControllers; ++userIndex) {
                XInputState state{};
                if (s_xinput.getState(userIndex, &state) != kXInputSuccess) {
                    continue;
                }

                sample.connected = true;
                sample.buttons = state.gamepad.buttons;
                sample.analog.leftX = normalizeThumb(state.gamepad.thumbLX);
                sample.analog.leftY = normalizeThumb(state.gamepad.thumbLY);
                sample.analog.rightX = normalizeThumb(state.gamepad.thumbRX);
                sample.analog.rightY = normalizeThumb(state.gamepad.thumbRY);
                return sample;
            }

            return sample;
        }

        [[nodiscard]] debug_controller_policy::PivotVector toPolicyVector(const RE::NiPoint3& value)
        {
            return debug_controller_policy::PivotVector{ .x = value.x, .y = value.y, .z = value.z };
        }

        [[nodiscard]] RE::NiPoint3 toNiPoint(const debug_controller_policy::PivotVector& value)
        {
            return RE::NiPoint3(value.x, value.y, value.z);
        }

        [[nodiscard]] RE::NiPoint3& selectedPivot()
        {
            return s_state.selectedLeft ? g_rockConfig.rockLeftGrabPivotAHandspace : g_rockConfig.rockRightGrabPivotAHandspace;
        }

        [[nodiscard]] const RE::NiPoint3& selectedPivotConst()
        {
            return s_state.selectedLeft ? g_rockConfig.rockLeftGrabPivotAHandspace : g_rockConfig.rockRightGrabPivotAHandspace;
        }

        [[nodiscard]] const char* selectedHandName()
        {
            return s_state.selectedLeft ? "left" : "right";
        }

        void notify(std::string message)
        {
            ROCK_LOG_INFO(Input, "Debug controller: {}", message);
            f4vr::showNotification(std::format("[ROCK] {}", message));
        }

        void notifyPivot()
        {
            const auto& pivot = selectedPivotConst();
            notify(std::format("{} grab pivot A X={:.2f} Y={:.2f} Z={:.2f}", selectedHandName(), pivot.x, pivot.y, pivot.z));
        }

        void persistColliderState(const char* key, bool value)
        {
            if (!g_rockConfig.persistPhysicsBool(key, value)) {
                ROCK_LOG_WARN(Input, "Debug controller failed to persist {}={}", key, value);
            }
        }

        void toggleHandColliders()
        {
            const bool visible = g_rockConfig.rockDebugShowColliders && (g_rockConfig.rockDebugDrawHandColliders || g_rockConfig.rockDebugDrawHandBoneColliders);
            const bool enabled = !visible;
            g_rockConfig.rockDebugDrawHandColliders = enabled;
            g_rockConfig.rockDebugDrawHandBoneColliders = enabled;
            if (enabled) {
                g_rockConfig.rockDebugShowColliders = true;
                persistColliderState("bDebugShowColliders", true);
            }

            persistColliderState("bDebugDrawHandColliders", enabled);
            persistColliderState("bDebugDrawHandBoneColliders", enabled);
            notify(std::format("hand collider visualizers {}", enabled ? "ON" : "OFF"));
        }

        void toggleWeaponColliders()
        {
            const bool visible = g_rockConfig.rockDebugShowColliders && g_rockConfig.rockDebugDrawWeaponColliders;
            const bool enabled = !visible;
            g_rockConfig.rockDebugDrawWeaponColliders = enabled;
            if (enabled) {
                g_rockConfig.rockDebugShowColliders = true;
                persistColliderState("bDebugShowColliders", true);
            }

            persistColliderState("bDebugDrawWeaponColliders", enabled);
            notify(std::format("weapon collider visualizers {}", enabled ? "ON" : "OFF"));
        }

        void captureTuningVisualState()
        {
            if (s_state.tuningVisualState.saved) {
                return;
            }

            s_state.tuningVisualState.saved = true;
            s_state.tuningVisualState.showGrabPivots = g_rockConfig.rockDebugShowGrabPivots;
            s_state.tuningVisualState.showPalmVectors = g_rockConfig.rockDebugShowPalmVectors;
        }

        void restoreTuningVisualState()
        {
            if (!s_state.tuningVisualState.saved) {
                return;
            }

            g_rockConfig.rockDebugShowGrabPivots = s_state.tuningVisualState.showGrabPivots;
            g_rockConfig.rockDebugShowPalmVectors = s_state.tuningVisualState.showPalmVectors;
            s_state.tuningVisualState = {};
        }

        void persistDirtyPivots()
        {
            if (s_state.pivotDirty[0]) {
                if (!g_rockConfig.persistGrabPivotAHandspace(false, g_rockConfig.rockRightGrabPivotAHandspace)) {
                    ROCK_LOG_WARN(Input, "Debug controller failed to persist right grab pivot");
                }
            }
            if (s_state.pivotDirty[1]) {
                if (!g_rockConfig.persistGrabPivotAHandspace(true, g_rockConfig.rockLeftGrabPivotAHandspace)) {
                    ROCK_LOG_WARN(Input, "Debug controller failed to persist left grab pivot");
                }
            }
            s_state.pivotDirty = {};
        }

        void togglePivotTuning()
        {
            s_state.pivotTuningActive = !s_state.pivotTuningActive;
            if (s_state.pivotTuningActive) {
                captureTuningVisualState();
                g_rockConfig.rockDebugShowGrabPivots = true;
                g_rockConfig.rockDebugShowPalmVectors = true;
                s_state.notificationElapsedSeconds = kNotificationIntervalSeconds;
                notifyPivot();
                return;
            }

            persistDirtyPivots();
            restoreTuningVisualState();
            notify(std::format("{} grab pivot tuning saved", selectedHandName()));
        }

        void selectNextPivotHand()
        {
            s_state.selectedLeft = !s_state.selectedLeft;
            notifyPivot();
        }

        void applyPivotTuningInput(const debug_controller_policy::AnalogSample& analog, float deltaSeconds)
        {
            using namespace debug_controller_policy;

            if (!s_state.pivotTuningActive) {
                return;
            }

            auto before = toPolicyVector(selectedPivotConst());
            auto after = applyPivotTuning(before, analog, deltaSeconds);
            if (!pivotChanged(before, after)) {
                return;
            }

            selectedPivot() = toNiPoint(after);
            s_state.pivotDirty[s_state.selectedLeft ? 1u : 0u] = true;
            s_state.notificationElapsedSeconds += deltaSeconds;
            if (s_state.notificationElapsedSeconds >= kNotificationIntervalSeconds) {
                s_state.notificationElapsedSeconds = 0.0f;
                notifyPivot();
            }
        }

        void processCommands(const debug_controller_policy::ButtonEdges& edges)
        {
            using debug_controller_policy::Command;

            if (debug_controller_policy::commandPressed(edges, Command::ToggleHandColliders)) {
                toggleHandColliders();
            }
            if (debug_controller_policy::commandPressed(edges, Command::ToggleWeaponColliders)) {
                toggleWeaponColliders();
            }
            if (debug_controller_policy::commandPressed(edges, Command::SelectPivotHand)) {
                selectNextPivotHand();
            }
            if (debug_controller_policy::commandPressed(edges, Command::TogglePivotTuning)) {
                togglePivotTuning();
            }
        }
    }

    void update(bool gameplayInputAllowed, float deltaSeconds)
    {
        const float dt = sanitizeDeltaSeconds(deltaSeconds);
        const auto sample = pollController();
        const auto edges = debug_controller_policy::evaluateButtonEdges(s_state.hadPreviousButtons, s_state.previousButtons, sample.connected, sample.buttons);

        if (!sample.connected) {
            s_state.hadPreviousButtons = false;
            s_state.previousButtons = 0;
            return;
        }

        s_state.hadPreviousButtons = true;
        s_state.previousButtons = sample.buttons;

        if (!gameplayInputAllowed) {
            return;
        }

        processCommands(edges);
        applyPivotTuningInput(sample.analog, dt);
    }

    bool isPivotTuningActive()
    {
        return s_state.pivotTuningActive;
    }

    bool isSelectedPivotHandLeft()
    {
        return s_state.selectedLeft;
    }
}
