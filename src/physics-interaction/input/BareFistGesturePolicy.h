#pragma once

#include <cstdint>
#include <limits>

namespace rock::bare_fist_gesture
{
    inline constexpr std::uint64_t kButtons = (1ull << 2) | (1ull << 33);
    inline constexpr std::uint64_t kMaximumSampleAgeMilliseconds = 100;
    inline constexpr float kDefaultHoldSeconds = 0.30f;
    inline constexpr float kMinimumHoldSeconds = 0.1f;
    inline constexpr float kMaximumHoldSeconds = 10.0f;
    inline constexpr float kDrawTimeoutSeconds = 2.0f;

    struct Buttons
    {
        std::uint8_t leftBits{}, rightBits{}; // grip=1, trigger=2, input rearm/UI=4
        std::uint64_t leftAgeMilliseconds{}, rightAgeMilliseconds{};
        bool fresh{}, held{}, chordBroken{}, allReleased{}, blocked{};
    };

    [[nodiscard]] constexpr Buttons readButtons(
        std::uint64_t left, std::uint64_t right, std::uint64_t now) noexcept
    {
        const auto age = [now](std::uint64_t sample) {
            const auto tick = sample >> 3;
            return tick != 0 && now >= tick ? now - tick : (std::numeric_limits<std::uint64_t>::max)();
        };
        Buttons result{};
        result.leftBits = static_cast<std::uint8_t>(left & 7u);
        result.rightBits = static_cast<std::uint8_t>(right & 7u);
        result.leftAgeMilliseconds = age(left);
        result.rightAgeMilliseconds = age(right);
        result.fresh = result.leftAgeMilliseconds <= kMaximumSampleAgeMilliseconds &&
            result.rightAgeMilliseconds <= kMaximumSampleAgeMilliseconds;
        result.blocked = ((left | right) & 4u) != 0;
        result.held = result.fresh && result.leftBits == 3u && result.rightBits == 3u;
        // A deliberate chord break is enough to rearm. Keeping the other grip
        // or trigger held must not require relaxing all four buttons together.
        result.chordBroken = result.fresh && ((left & 3u) != 3u || (right & 3u) != 3u);
        result.allReleased = result.fresh && ((left | right) & 3u) == 0;
        return result;
    }

    // Three low bits hold capture state; upper bits identify a physical cycle.
    // A release/repress between game frames must never continue an old session.
    // All odd states retain ownership of the spent input. OR-ing bit 1 cancels
    // Holding and ReadyToRetry atomically, without losing a concurrent identity.
    enum class Capture : std::uint64_t {
        Idle = 0, Holding = 1, Rearming = 2, Draining = 3,
        ReadyToRetry = 5, CancelledRetry = 7
    };
    [[nodiscard]] constexpr Capture capture(std::uint64_t cycle) noexcept
    {
        return static_cast<Capture>(cycle & 7u);
    }
    [[nodiscard]] constexpr std::uint64_t observe(
        std::uint64_t cycle, bool eligible, const Buttons& buttons, bool interrupted) noexcept
    {
        switch (capture(cycle)) {
        case Capture::Idle:
            return eligible && buttons.held && !interrupted ? ((cycle & ~7ull) + 8u) | 1u : cycle;
        case Capture::Holding:
            // Controllers are sampled separately. A stale peer is unknown,
            // not a button-up event. Draw/damage gates still require freshness.
            if (!eligible || interrupted || buttons.blocked || (buttons.fresh && !buttons.held))
                return (cycle & ~7ull) | static_cast<std::uint64_t>(Capture::Draining);
            return cycle;
        case Capture::Rearming:
            return buttons.chordBroken && !buttons.blocked ? cycle & ~7ull : cycle;
        case Capture::Draining:
        case Capture::CancelledRetry:
            if (!buttons.fresh || buttons.blocked) return cycle;
            if (buttons.allReleased) return cycle & ~7ull;
            // Keep the remaining buttons consumed until either a fresh full
            // chord starts a new cycle or every spent button is released.
            return buttons.chordBroken ? (cycle & ~7ull) | static_cast<std::uint64_t>(Capture::ReadyToRetry) : cycle;
        case Capture::ReadyToRetry:
            if (!eligible || buttons.blocked || interrupted) return cycle | 2u;
            if (buttons.allReleased) return cycle & ~7ull;
            return eligible && buttons.held ? ((cycle & ~7ull) + 8u) | 1u : cycle;
        }
        return cycle;
    }

    enum class Phase : std::uint8_t { Idle, Qualifying, Drawing, Active };
    struct State
    {
        Phase phase{ Phase::Idle };
        std::uint64_t cycle{ 0 };
        float seconds{ 0.0f };
        float requiredHoldSeconds{ kDefaultHoldSeconds };
    };
    struct Input
    {
        std::uint64_t cycle{ 0 };
        bool eligible{ false };
        bool drawnUnarmed{ false };
        float deltaSeconds{ 0.0f };
        float holdSeconds{ kDefaultHoldSeconds };
    };
    enum class Action { None, Draw, Cancel };

    [[nodiscard]] inline Action update(State& state, const Input& input) noexcept
    {
        if (!input.eligible || capture(input.cycle) != Capture::Holding ||
            (state.phase != Phase::Idle && state.cycle != input.cycle)) {
            const bool started = state.phase != Phase::Idle;
            state = {};
            return started ? Action::Cancel : Action::None;
        }
        if (state.phase == Phase::Idle) {
            // A hot reload affects the next physical hold, never shortens
            // qualification under buttons the player is already holding.
            state = { Phase::Qualifying, input.cycle, 0.0f, input.holdSeconds };
            return Action::None;
        }
        if (state.phase == Phase::Qualifying) {
            state.seconds += input.deltaSeconds;
            if (state.seconds >= state.requiredHoldSeconds) {
                state.phase = Phase::Drawing;
                state.seconds = 0.0f;
                return Action::Draw;
            }
        } else if (state.phase == Phase::Drawing) {
            if (input.drawnUnarmed) {
                state.phase = Phase::Active;
                state.seconds = 0.0f;
            } else {
                state.seconds += input.deltaSeconds;
                if (state.seconds >= kDrawTimeoutSeconds) {
                    state = {};
                    return Action::Cancel;
                }
            }
        } else if (!input.drawnUnarmed) {
            state = {};
            return Action::Cancel;
        }
        return Action::None;
    }
}
