/*
 * Developer telemetry for the external FO4VR test bench (devbench `rps` tools).
 *
 * When `bDebugTestbenchTelemetry` is on, ROCK publishes one Frame record per
 * finalized interaction frame and Event records (haptics, native input gates)
 * into the shared-memory ring `Local\RPS_Telemetry_ROCK`, and after the presented
 * skeleton is final, Skeleton records with every bone of the rendered root skeleton. The bench reads it
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
#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

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

    // What a hand holds at the end of ROCK's physics update, for the bench's visual checks.
    // ROCK's own grab wins over the equipped weapon's grips. Values are part of the ring
    // contract; never reuse one.
    enum class HeldKind : std::uint8_t
    {
        None = 0,
        Object = 1, // ROCK grab of anything but a weapon
        LooseWeapon = 2, // ROCK grab of a weapon reference
        EquippedWeapon = 3, // the equipped weapon's firing grip, or the akimbo second weapon
        EquippedWeaponPart = 4, // a part grip on the equipped weapon (support hand, magazine well...)
    };

    // A scene node's world transform, copied in the frame it was read.
    struct NodeSample
    {
        float position[3]{};
        float rotation[9]{}; // row-major NiMatrix3
        float scale = 1.0f;
        bool valid = false;
    };

    struct HandFrameSample
    {
        float position[3]{};
        float rotation[9]{}; // row-major NiMatrix3
        float velocity[3]{}; // metres per second
        bool hasVelocity = false;
        bool disabled = false;
        HeldKind held = HeldKind::None;
        std::uint32_t heldFormId = 0; // the grabbed reference (Object, LooseWeapon); 0 otherwise
        // The grabbed reference's root as its driven Havok body places it this frame (Object,
        // LooseWeapon). Against `position`/`rotation` it shows how far the object trails the
        // hand: the bench's slip and rubber-band measure.
        NodeSample heldRoot{};
        // The grabbed reference's 3D root as the scene holds it at FRIK's frame end, after the
        // presented skeleton and ROCK's held-scene writer: what is drawn, against the drawn hand.
        NodeSample heldNode{};
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
        // ROCK's physics update ran this frame. Frames are published every game frame,
        // menus included; without physics only the runtime state, the player's VR nodes and
        // the controller input are current, and the hand fields are empty.
        bool physicsFrame = false;
        float hmdPosition[3]{};
        float hmdForward[3]{};
        HandFrameSample hand[2]{}; // left, right
        // Tracking-space anchors for recording a player's motion: RoomNode (tracking origin
        // in the world), HmdNode, and the wands (left = secondary, right = primary; ROCK
        // runs right-handed).
        NodeSample room{};
        NodeSample hmdNode{};
        NodeSample wand[2]{}; // left, right
        // The player's heading (radians, the reference's angle z) and position. The body follows
        // the head, so the heading is not the room turn the game applies to the VR nodes.
        bool playerPoseValid = false;
        float playerHeading = 0.0f;
        float playerPosition[3]{};
        // The tracking-space poses the game received this frame (IVRCompositor::GetLastPoses):
        // head, left hand, right hand; metres, rotation rows of the device-to-absolute matrix.
        NodeSample rawPose[3]{};
    };

    [[nodiscard]] bool enabled() noexcept;

    // Raw OpenVR controller state as read through ROCK's hook (before any remap).
    struct RawControllerState
    {
        std::uint64_t pressed = 0;
        std::uint64_t touched = 0;
        float axes[10]{}; // rAxis[0..4] x, y: OpenVR axis 0 stick/trackpad, 1 trigger, 2 grip, 3-4 device-specific
    };

    // Any thread. One call per controller-state read through ROCK's hook. `bypassed` reads
    // (RPS UI and configurator raw reads) count separately and never update the game state.
    void observeControllerState(bool isLeft, const RawControllerState& raw, std::uint64_t gamePressed,
        float gameTrigger, std::uint32_t blankReasons, bool bypassed) noexcept;

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
    void recordInputGate(const char* gate, bool sourceIsLeft, bool justPressed, bool suppressed,
        bool providerSuppressed) noexcept;

    // One bone of the presented skeleton; `name` must stay valid for the call only.
    struct SkeletonBone
    {
        std::string_view name;
        float position[3]{};
        float rotation[9]{}; // row-major NiMatrix3, world
    };

    // Frame thread, after the presented skeleton is final (FRIK AfterWorldFinal). Writes the
    // bone-name table when `topologyRevision` changes and every few seconds, then the poses.
    void publishSkeleton(std::uint64_t frameIndex, std::uint64_t topologyRevision,
        std::span<const SkeletonBone> bones) noexcept;

    // The capture switch in RobCo PALM's Developer wheel (PALM.ini [Developer] bEnabled): each
    // press starts or stops a capture. While a capture runs the section is open whatever the
    // setting says, and its header flags bit 1 tells the external recorder to record.
    // Registered once after every plugin has loaded; ROCK runs without PALM.
    void registerWheelCapture() noexcept;
    // Frame thread. Applies a pending wheel press; returns the new capture state when it changed.
    [[nodiscard]] std::optional<bool> takeCaptureToggle() noexcept;
    [[nodiscard]] bool captureActive() noexcept;

    // Frame thread. Syncs the section with `settingOn`, then writes queued events and one frame.
    // The section lives from the frame the setting turns on until it turns off or the
    // process exits; it deliberately survives save loads so one bench run is one ring.
    void publishFrame(bool settingOn, const FrameSample& frame) noexcept;
}
