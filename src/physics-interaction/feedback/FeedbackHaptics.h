#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace rock::feedback_haptics
{
    enum class FeedbackHand : std::uint8_t
    {
        Right = 0,
        Left = 1,
    };

    struct HapticEvent
    {
        float startIntensity = 0.0f;
        float endIntensity = 0.0f;
        float durationSeconds = 0.0f;
        float elapsedSeconds = 0.0f;
        bool active = false;
        std::uint64_t sequence = 0;
    };

    struct HapticOutput
    {
        bool active = false;
        FeedbackHand hand = FeedbackHand::Right;
        float intensity = 0.0f;
        float pulseDurationSeconds = 0.0f;
    };

    class FeedbackHaptics
    {
    public:
        static constexpr std::size_t kEventsPerHand = 8;

        void reset() noexcept;
        // Supply the final held/armed reference each frame; zero ends feedback.
        // A separate sustained source cannot evict queued interaction events.
        void setArmedThrowable(FeedbackHand hand, std::uint32_t referenceId) noexcept;
        bool queue(FeedbackHand hand, float durationSeconds, float intensity) noexcept;
        bool queue(FeedbackHand hand, float durationSeconds, float startIntensity, float endIntensity) noexcept;
        [[nodiscard]] std::size_t update(float deltaSeconds, HapticOutput* outputs, std::size_t outputCapacity) noexcept;
        [[nodiscard]] std::size_t activeEventCount(FeedbackHand hand) const noexcept;

    private:
        using HandEvents = std::array<HapticEvent, kEventsPerHand>;

        struct ArmedThrowablePulse
        {
            std::uint32_t referenceId = 0;
            double phaseSeconds = 0.0;
        };

        std::array<HandEvents, 2> _events{};
        std::array<ArmedThrowablePulse, 2> _armedThrowables{};
        std::uint64_t _nextSequence = 1;
    };
}
