#include "physics-interaction/feedback/FeedbackHaptics.h"

#include <algorithm>
#include <cmath>

namespace rock::feedback_haptics
{
    namespace
    {
        [[nodiscard]] std::size_t handIndex(FeedbackHand hand) noexcept
        {
            return hand == FeedbackHand::Left ? 1u : 0u;
        }

        [[nodiscard]] float finiteOr(float value, float fallback) noexcept
        {
            return std::isfinite(value) ? value : fallback;
        }

        [[nodiscard]] float sanitizeIntensity(float value) noexcept
        {
            return std::clamp(finiteOr(value, 0.0f), 0.0f, 1.0f);
        }

        [[nodiscard]] float sanitizeDuration(float value) noexcept
        {
            return std::clamp(finiteOr(value, 0.0f), 0.0f, 0.2f);
        }

        [[nodiscard]] float currentIntensity(const HapticEvent& event) noexcept
        {
            if (event.durationSeconds <= 0.000001f) {
                return sanitizeIntensity(event.endIntensity);
            }

            const float t = std::clamp(event.elapsedSeconds / event.durationSeconds, 0.0f, 1.0f);
            return sanitizeIntensity(event.startIntensity + (event.endIntensity - event.startIntensity) * t);
        }
    }

    void FeedbackHaptics::reset() noexcept
    {
        _events = {};
        _nextSequence = 1;
    }

    bool FeedbackHaptics::queue(FeedbackHand hand, float durationSeconds, float intensity) noexcept
    {
        return queue(hand, durationSeconds, intensity, intensity);
    }

    bool FeedbackHaptics::queue(FeedbackHand hand, float durationSeconds, float startIntensity, float endIntensity) noexcept
    {
        const float duration = sanitizeDuration(durationSeconds);
        const float start = sanitizeIntensity(startIntensity);
        const float end = sanitizeIntensity(endIntensity);
        if (duration <= 0.0f || (start <= 0.0f && end <= 0.0f)) {
            return true;
        }

        auto& events = _events[handIndex(hand)];
        HapticEvent* slot = nullptr;
        for (auto& event : events) {
            if (!event.active) {
                slot = &event;
                break;
            }
        }

        if (!slot) {
            slot = &events.front();
            for (auto& event : events) {
                if (event.sequence < slot->sequence) {
                    slot = &event;
                }
            }
        }

        *slot = HapticEvent{
            .startIntensity = start,
            .endIntensity = end,
            .durationSeconds = duration,
            .elapsedSeconds = 0.0f,
            .active = true,
            .sequence = _nextSequence++,
        };
        if (_nextSequence == 0) {
            _nextSequence = 1;
        }
        return slot->sequence != 0;
    }

    std::size_t FeedbackHaptics::update(float deltaSeconds, HapticOutput* outputs, std::size_t outputCapacity) noexcept
    {
        const float delta = (std::isfinite(deltaSeconds) && deltaSeconds > 0.0f) ? deltaSeconds : 1.0f / 90.0f;
        std::size_t outputCount = 0;

        for (std::size_t hand = 0; hand < _events.size(); ++hand) {
            HapticEvent* latest = nullptr;
            for (auto& event : _events[hand]) {
                if (!event.active) {
                    continue;
                }
                if (event.elapsedSeconds >= event.durationSeconds) {
                    event.active = false;
                    continue;
                }
                if (!latest || event.sequence > latest->sequence) {
                    latest = &event;
                }
            }

            if (latest && outputs && outputCount < outputCapacity) {
                const float remaining = (std::max)(0.0f, latest->durationSeconds - latest->elapsedSeconds);
                outputs[outputCount++] = HapticOutput{
                    .active = true,
                    .hand = hand == 1u ? FeedbackHand::Left : FeedbackHand::Right,
                    .intensity = currentIntensity(*latest),
                    .pulseDurationSeconds = std::min(remaining, std::clamp(delta, 0.005f, 0.02f)),
                };
            }

            for (auto& event : _events[hand]) {
                if (!event.active) {
                    continue;
                }
                event.elapsedSeconds += delta;
                if (event.elapsedSeconds >= event.durationSeconds) {
                    event.active = false;
                }
            }
        }

        return outputCount;
    }

    std::size_t FeedbackHaptics::activeEventCount(FeedbackHand hand) const noexcept
    {
        std::size_t count = 0;
        for (const auto& event : _events[handIndex(hand)]) {
            if (event.active) {
                ++count;
            }
        }
        return count;
    }
}
