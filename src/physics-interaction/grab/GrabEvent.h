#pragma once

#include <cstdint>

namespace RE
{
    class TESObjectREFR;
}

namespace rock
{
    /*
     * ROCK exposes grab lifecycle events through one source so pull, grab,
     * impact, two-hand, and future stash/consume paths do not grow separate
     * notification code. The payload is POD and versioned so it can also cross
     * F4SE messaging without binding consumers to ROCK internals.
     */
    enum class GrabEventType : std::uint32_t
    {
        Unknown = 0,
        SelectionLocked = 1,
        PullStarted = 2,
        PullArrived = 3,
        PullCatchAttempt = 4,
        PullCatchSucceeded = 5,
        GrabCommitted = 6,
        HeldImpact = 7,
        Released = 8,
        // Reserved scaffold events below are ABI-visible but are only
        // dispatched once their backing gameplay/runtime state exists.
        TwoHandStarted = 9,
        TwoHandStopped = 10,
        StashCandidate = 11,
        ConsumeCandidate = 12,
        Stashed = 13,
        Consumed = 14,
        LootStarted = 15,
        LootCompleted = 16,
        SelectionUnlocked = 17,
    };

    enum class GrabEventSourceKind : std::uint32_t
    {
        Unknown = 0,
        Hand = 1,
        HeldObject = 2,
        PulledObject = 3,
        Weapon = 4,
        External = 5,
    };

    inline constexpr std::uint32_t ROCK_GRAB_EVENT_VERSION = 1;
    inline constexpr std::uint32_t ROCK_GRAB_EVENT_INVALID_BODY_ID = 0x7FFF'FFFFu;
    inline constexpr std::uint32_t ROCK_GRAB_EVENT_FLAG_HELD_IMPACT_DAMPED = 1u << 0;
    inline constexpr std::uint32_t ROCK_GRAB_EVENT_FLAG_SUPPRESS_HAPTIC = 1u << 1;
    inline constexpr std::uint32_t ROCK_GRAB_EVENT_FLAG_POSITION_VALID = 1u << 2;
    inline constexpr std::uint32_t ROCK_GRAB_EVENT_FLAG_VELOCITY_VALID = 1u << 3;
    inline constexpr std::uint32_t ROCK_GRAB_EVENT_FLAG_MASS_VALID = 1u << 4;
    inline constexpr std::uint32_t ROCK_GRAB_EVENT_FLAG_SPEED_VALID = 1u << 5;
    inline constexpr std::uint32_t ROCK_GRAB_EVENT_FLAG_INTENSITY_VALID = 1u << 6;

    struct GrabEventData
    {
        std::uint32_t size{ sizeof(GrabEventData) };
        std::uint32_t version{ ROCK_GRAB_EVENT_VERSION };
        GrabEventType type{ GrabEventType::Unknown };
        GrabEventSourceKind sourceKind{ GrabEventSourceKind::Unknown };
        bool isLeft{ false };
        std::uint8_t reservedBool[3]{};
        RE::TESObjectREFR* refr{ nullptr };
        std::uint32_t formID{ 0 };
        std::uint32_t primaryBodyId{ ROCK_GRAB_EVENT_INVALID_BODY_ID };
        std::uint32_t secondaryBodyId{ ROCK_GRAB_EVENT_INVALID_BODY_ID };
        std::uint32_t collisionLayer{ 0 };
        std::uint32_t flags{ 0 };
        std::uint64_t frameIndex{ 0 };
        float positionGame[3]{};
        float velocityGame[3]{};
        float mass{ 0.0f };
        float speedGameUnitsPerSecond{ 0.0f };
        float intensityHint{ 0.0f };
        float reservedFloat[3]{};
    };
}
