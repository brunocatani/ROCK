#include "rock_support/VRControllers.h"

#include <algorithm>
#include <chrono>

namespace rock::vr_input
{
    void VRControllersManager::update(const bool isLeftHanded) noexcept
    {
        auto* vrSystem = vr::VRSystem();
        if (!vrSystem) {
            _left.reset();
            _right.reset();
            _currentTime = currentTimeSeconds();
            return;
        }

        _leftHanded = isLeftHanded;
        const float now = currentTimeSeconds();
        _left.update(vrSystem->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_LeftHand), now);
        _right.update(vrSystem->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_RightHand), now);
        _currentTime = now;
    }

    void VRControllersManager::reset() noexcept
    {
        _left.reset();
        _right.reset();
        _currentTime = currentTimeSeconds();
    }

    bool VRControllersManager::isPressed(const Hand hand, const int buttonId) noexcept
    {
        if (!validButtonId(buttonId)) {
            return false;
        }
        return stateFor(resolveHand(hand)).justPressed(
            static_cast<std::size_t>(buttonId),
            _currentTime,
            _debounceSeconds);
    }

    bool VRControllersManager::isPressHeldDown(
        const Hand hand,
        const int buttonId,
        const float minimumHoldSeconds) const noexcept
    {
        if (!validButtonId(buttonId)) {
            return false;
        }
        const auto buttonIndex = static_cast<std::size_t>(buttonId);
        const auto& controller = stateFor(resolveHand(hand));
        return controller.buttonHeld(buttonIndex) &&
            controller.heldDuration(buttonIndex, _currentTime) >= std::max(0.0f, minimumHoldSeconds);
    }

    bool VRControllersManager::isReleased(
        const Hand hand,
        const int buttonId,
        const float maximumHoldSeconds) noexcept
    {
        if (!validButtonId(buttonId)) {
            return false;
        }
        const auto buttonIndex = static_cast<std::size_t>(buttonId);
        auto& controller = stateFor(resolveHand(hand));
        return controller.justReleased(buttonIndex, _currentTime, _debounceSeconds) &&
            controller.releasedHoldDuration(buttonIndex, _currentTime) < std::max(0.0f, maximumHoldSeconds);
    }

    void VRControllersManager::triggerHaptic(
        const Hand hand,
        const float durationSeconds,
        const float intensity) noexcept
    {
        stateFor(resolveHand(hand)).startHaptic(
            _currentTime + std::max(0.0f, durationSeconds),
            std::clamp(intensity, 0.0f, 1.0f));
    }

    void VRControllersManager::ControllerState::update(
        const vr::TrackedDeviceIndex_t newIndex,
        const float now) noexcept
    {
        previous = current;
        index = newIndex;

        auto* vrSystem = vr::VRSystem();
        if (!vrSystem || newIndex == vr::k_unTrackedDeviceIndexInvalid) {
            current = {};
            valid = false;
            return;
        }

        vr::TrackedDevicePose_t pose{};
        valid = vrSystem->GetControllerStateWithPose(
            vr::TrackingUniverseStanding,
            newIndex,
            &current,
            sizeof(current),
            &pose);
        if (!valid) {
            current = {};
            return;
        }

        for (std::size_t buttonIndex = 0; buttonIndex < kButtonCount; ++buttonIndex) {
            const std::uint64_t mask = std::uint64_t{ 1 } << buttonIndex;
            const bool pressedNow = (current.ulButtonPressed & mask) != 0;
            const bool pressedBefore = (previous.ulButtonPressed & mask) != 0;

            if (pressedNow && !pressedBefore) {
                pressStartTimes[buttonIndex] = now;
                pressStartValid[buttonIndex] = true;
                releaseStartValid[buttonIndex] = false;
            } else if (!pressedNow && pressedBefore) {
                releaseStartTimes[buttonIndex] = pressStartTimes[buttonIndex];
                releaseStartValid[buttonIndex] = pressStartValid[buttonIndex];
                pressStartValid[buttonIndex] = false;
            } else {
                releaseStartValid[buttonIndex] = false;
            }
        }

        if (now < hapticEndTime) {
            emitHapticPulse();
        }
    }

    void VRControllersManager::ControllerState::reset() noexcept
    {
        index = vr::k_unTrackedDeviceIndexInvalid;
        current = {};
        previous = {};
        valid = false;
        pressStartTimes.fill(0.0f);
        releaseStartTimes.fill(0.0f);
        lastPressTimes.fill(0.0f);
        lastReleaseTimes.fill(0.0f);
        pressStartValid.fill(false);
        releaseStartValid.fill(false);
        hapticEndTime = 0.0f;
        hapticIntensity = 0.0f;
    }

    bool VRControllersManager::ControllerState::buttonHeld(const std::size_t buttonIndex) const noexcept
    {
        return valid && (current.ulButtonPressed & (std::uint64_t{ 1 } << buttonIndex)) != 0;
    }

    bool VRControllersManager::ControllerState::justPressed(
        const std::size_t buttonIndex,
        const float now,
        const float debounceSeconds) noexcept
    {
        if (!valid) {
            return false;
        }
        const std::uint64_t mask = std::uint64_t{ 1 } << buttonIndex;
        const bool transitioned = (previous.ulButtonPressed & mask) == 0 && (current.ulButtonPressed & mask) != 0;
        if (!transitioned || now - lastPressTimes[buttonIndex] < debounceSeconds) {
            return false;
        }
        lastPressTimes[buttonIndex] = now;
        return true;
    }

    bool VRControllersManager::ControllerState::justReleased(
        const std::size_t buttonIndex,
        const float now,
        const float debounceSeconds) noexcept
    {
        if (!valid) {
            return false;
        }
        const std::uint64_t mask = std::uint64_t{ 1 } << buttonIndex;
        const bool transitioned = (previous.ulButtonPressed & mask) != 0 && (current.ulButtonPressed & mask) == 0;
        if (!transitioned || now - lastReleaseTimes[buttonIndex] < debounceSeconds) {
            return false;
        }
        lastReleaseTimes[buttonIndex] = now;
        return true;
    }

    float VRControllersManager::ControllerState::heldDuration(
        const std::size_t buttonIndex,
        const float now) const noexcept
    {
        return pressStartValid[buttonIndex] ? now - pressStartTimes[buttonIndex] : 0.0f;
    }

    float VRControllersManager::ControllerState::releasedHoldDuration(
        const std::size_t buttonIndex,
        const float now) const noexcept
    {
        return releaseStartValid[buttonIndex] ? now - releaseStartTimes[buttonIndex] : 0.0f;
    }

    void VRControllersManager::ControllerState::startHaptic(const float endTime, const float intensity) noexcept
    {
        if (!vr::VRSystem()) {
            return;
        }
        hapticEndTime = endTime;
        hapticIntensity = intensity;
        if (valid) {
            emitHapticPulse();
        }
    }

    void VRControllersManager::ControllerState::emitHapticPulse() const noexcept
    {
        auto* vrSystem = vr::VRSystem();
        if (!vrSystem || !valid || index == vr::k_unTrackedDeviceIndexInvalid) {
            return;
        }
        const auto pulse = static_cast<std::uint16_t>(
            std::clamp(static_cast<int>(hapticIntensity * 3000.0f), 0, 3000));
        vrSystem->TriggerHapticPulse(index, 0, pulse);
    }

    float VRControllersManager::currentTimeSeconds() noexcept
    {
        static const auto start = std::chrono::steady_clock::now();
        return std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
    }

    bool VRControllersManager::validButtonId(const int buttonId) noexcept
    {
        return buttonId >= 0 && buttonId < static_cast<int>(kButtonCount);
    }

    vr::ETrackedControllerRole VRControllersManager::resolveHand(const Hand hand) const noexcept
    {
        switch (hand) {
        case Hand::Primary:
            return _leftHanded ? vr::TrackedControllerRole_LeftHand : vr::TrackedControllerRole_RightHand;
        case Hand::Offhand:
            return _leftHanded ? vr::TrackedControllerRole_RightHand : vr::TrackedControllerRole_LeftHand;
        case Hand::Right:
            return vr::TrackedControllerRole_RightHand;
        case Hand::Left:
            return vr::TrackedControllerRole_LeftHand;
        }
        return vr::TrackedControllerRole_OptOut;
    }

    VRControllersManager::ControllerState& VRControllersManager::stateFor(const vr::ETrackedControllerRole hand) noexcept
    {
        return hand == vr::TrackedControllerRole_LeftHand ? _left : _right;
    }

    const VRControllersManager::ControllerState& VRControllersManager::stateFor(
        const vr::ETrackedControllerRole hand) const noexcept
    {
        return hand == vr::TrackedControllerRole_LeftHand ? _left : _right;
    }
}
