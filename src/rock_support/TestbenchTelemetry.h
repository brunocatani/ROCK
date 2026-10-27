#pragma once

#include <cstdint>

/*
 * Developer telemetry for the external FO4VR test bench (devbench `rps` tools).
 *
 * When `bDebugTestbenchTelemetry` is on, ROCK publishes one Frame record per
 * finalized interaction frame and Event records (haptics, native input gates)
 * into the shared-memory ring `Local\RPS_Telemetry_ROCK`. The bench reads it
 * to see what ROCK received from the controllers, what it handed the game and
 * why, instead of inferring that from logs. The record layout is a contract
 * with the bench's reader (`rps\mcp\rock_ring.py`); bump kSchemaVersion on any
 * change.
 *
 * Thread model: publishFrame() runs on ROCK's frame thread only and is the
 * single ring writer; it opens and closes the section when the setting
 * changes. The observe/record functions may run on any thread (OpenVR hooks,
 * input handlers): controller observations go into per-hand atomics that the
 * next frame consumes, events go into a bounded mutex-guarded queue that the
 * next frame drains. All of them return after one relaxed load while the
 * setting is off.
 */
namespace rock::testbench_telemetry
{
    // Why ROCK changed the controller state handed to the game (bit per edit).
    enum BlankReason : std::uint32_t
    {
        kBlankPhysicalReload = 1u << 0,
        kBlankSupportTrigger = 1u << 1,
        kBlankSecondaryOwnsTrigger = 1u << 2,
        kBlankProviderSuppressed = 1u << 3,
        kLeftFireRemapApplied = 1u << 4,
        kInputBlockingMenu = 1u << 5,
    };

    enum class HapticCaller : std::uint8_t
    {
        Game = 0,
        Rock = 1,
        Frik = 2,
        Other = 3,
    };

    struct HandFrameSample
    {
        float position[3]{};
        float rotation[9]{}; // row-major NiMatrix3
        float velocity[3]{}; // metres per second
        bool hasVelocity = false;
        bool disabled = false;
    };

    // A scene node's world transform, copied in the frame it was read.
    struct NodeSample
    {
        float position[3]{};
        float rotation[9]{}; // row-major NiMatrix3
        float scale = 1.0f;
        bool valid = false;
    };

    struct FrameSample
    {
        std::uint64_t frameIndex = 0;
        float deltaSeconds = 0.0f;
        bool playerAvailable = false;
        bool weaponDrawn = false;
        bool menuBlocking = false;
        bool loadingMenuOpen = false;
        bool gameStopped = false;
        bool inputMenuBlocking = false;
        bool worldReady = false;
        bool hasHmd = false;
        float hmdPosition[3]{};
        float hmdForward[3]{};
        HandFrameSample hand[2]{}; // left, right
        // Tracking-space anchors for recording a player's motion: RoomNode (tracking origin
        // in the world), HmdNode, and the wands (left = secondary, right = primary; ROCK
        // runs right-handed).
        NodeSample room{};
        NodeSample hmdNode{};
        NodeSample wand[2]{}; // left, right
    };

    [[nodiscard]] bool enabled() noexcept;

    // Any thread. One call per controller-state read through ROCK's hook. `bypassed` reads
    // (RPS UI and configurator raw reads) count separately and never update the game state.
    void observeControllerState(bool isLeft, std::uint64_t rawPressed, std::uint64_t gamePressed,
        float rawTrigger, float gameTrigger, std::uint32_t blankReasons, bool bypassed) noexcept;

    // Any thread. Controller-state reads that never reach observeControllerState: OpenVR
    // returned false, or the device is neither the left- nor the right-hand role.
    enum class UnobservedRead : std::uint8_t
    {
        Failed,
        NoHandRole,
    };
    void observeUnobservedRead(UnobservedRead kind) noexcept;

    // Any thread. `retargeted` = ROCK moved the pulse to the other hand.
    void recordHaptic(bool isLeft, std::uint16_t durationMicroseconds, HapticCaller caller, bool retargeted) noexcept;

    // Any thread. `gate` must be a string literal (stored by pointer until the next frame).
    void recordInputGate(const char* gate, bool sourceIsLeft, bool justPressed, bool suppressed, bool providerSuppressed) noexcept;

    // Frame thread. Syncs the section with `settingOn`, then writes queued events and one frame.
    // The section lives from the frame the setting turns on until it turns off or the
    // process exits; it deliberately survives save loads so one bench run is one ring.
    void publishFrame(bool settingOn, const FrameSample& frame) noexcept;
}
