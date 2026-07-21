#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include <openvr.h>

namespace rock::vr_input
{
    enum class Hand : std::uint8_t
    {
        Primary,
        Offhand,
        Right,
        Left,
    };

    // Owned by ROCK's game-thread frame loop. No method is safe to call from a
    // worker thread while update() is running.
    class VRControllersManager
    {
    public:
        void update(bool isLeftHanded) noexcept;
        void reset() noexcept;

        [[nodiscard]] bool isPressed(Hand hand, int buttonId) noexcept;
        [[nodiscard]] bool isPressHeldDown(Hand hand, int buttonId, float minimumHoldSeconds = 0.0f) const noexcept;
        [[nodiscard]] bool isReleased(Hand hand, int buttonId, float maximumHoldSeconds = 99.0f) noexcept;

        void triggerHaptic(Hand hand, float durationSeconds = 0.1f, float intensity = 0.3f) noexcept;

    private:
        static constexpr std::size_t kButtonCount = 64;

        struct ControllerState
        {
            vr::TrackedDeviceIndex_t index{ vr::k_unTrackedDeviceIndexInvalid };
            vr::VRControllerState_t current{};
            vr::VRControllerState_t previous{};
            bool valid{ false };

            std::array<float, kButtonCount> pressStartTimes{};
            std::array<float, kButtonCount> releaseStartTimes{};
            std::array<float, kButtonCount> lastPressTimes{};
            std::array<float, kButtonCount> lastReleaseTimes{};
            std::array<bool, kButtonCount> pressStartValid{};
            std::array<bool, kButtonCount> releaseStartValid{};

            float hapticEndTime{ 0.0f };
            float hapticIntensity{ 0.0f };

            void update(vr::TrackedDeviceIndex_t newIndex, float now) noexcept;
            void reset() noexcept;
            [[nodiscard]] bool buttonHeld(std::size_t buttonIndex) const noexcept;
            [[nodiscard]] bool justPressed(std::size_t buttonIndex, float now, float debounceSeconds) noexcept;
            [[nodiscard]] bool justReleased(std::size_t buttonIndex, float now, float debounceSeconds) noexcept;
            [[nodiscard]] float heldDuration(std::size_t buttonIndex, float now) const noexcept;
            [[nodiscard]] float releasedHoldDuration(std::size_t buttonIndex, float now) const noexcept;
            void startHaptic(float endTime, float intensity) noexcept;
            void emitHapticPulse() const noexcept;
        };

        [[nodiscard]] static float currentTimeSeconds() noexcept;
        [[nodiscard]] static bool validButtonId(int buttonId) noexcept;
        [[nodiscard]] vr::ETrackedControllerRole resolveHand(Hand hand) const noexcept;
        [[nodiscard]] ControllerState& stateFor(vr::ETrackedControllerRole hand) noexcept;
        [[nodiscard]] const ControllerState& stateFor(vr::ETrackedControllerRole hand) const noexcept;

        ControllerState _left;
        ControllerState _right;
        bool _leftHanded{ false };
        float _currentTime{ 0.0f };
        float _debounceSeconds{ 0.1f };
    };

    inline VRControllersManager VRControllers;
}

namespace vrcf = rock::vr_input;
