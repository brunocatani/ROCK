#pragma once

#include <cstdint>
#include <string>
#include <string_view>

/*
 * Pure on-disk format for per-object saved grab offsets (button-click save
 * feature): one human-editable JSON per object holding the object's saved
 * position+orientation relative to the live GrabAuthorityProxy hand frame,
 * per hand. No engine types and no I/O here so the format is policy-testable.
 * Identity is load-order independent: forms are (plugin name, plugin-local
 * id) pairs; the store layer converts to/from runtime formIDs at the disk
 * boundary ONLY.
 */
namespace rock::saved_grab_offset
{
    // v1: translate/rotate only. v2 (additive, backward compatible): optional
    // per-hand finger pose alongside the transform; a v1 file still parses
    // fine under newer formats, it simply has no "finger" section.
    // v3: optional exact parent-local finger rotations on the normal FRIK rig.
    // This private disk record does not change any published provider record.
    inline constexpr std::uint32_t kFormatVersion = 3;
    inline constexpr std::string_view kExactFingerRig = "frik-normal-authored-v1";

    // Load-order-independent form identity; empty = none.
    struct FormRef
    {
        std::string plugin;
        std::uint32_t localFormId{ 0 };
        [[nodiscard]] bool empty() const { return plugin.empty() && localFormId == 0; }
    };

    // Position + orientation relative to the live GrabAuthorityProxy frame
    // (proxy-local space, same convention as GrabCore.h's
    // objectInGeneratedProxyLocalSpace / objectFromGeneratedProxyLocalSpace),
    // plus an optional finger pose snapshot, both captured at button-save time.
    struct HandOffset
    {
        bool present{ false };
        float translateGame[3]{ 0.0f, 0.0f, 0.0f };
        // Row-major 3x3: rotate[row * 3 + column], row = object axis (X/Y/Z).
        float rotate[9]{ 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };

        // Same convention as grab_finger_pose_runtime::SolvedGrabFingerPose:
        // thumb/index/middle/ring/pinky, 0 = closed, 1 = authored open.
        // All fingers support over-open values through 2.0.
        bool hasFingerPose{ false };
        float fingerValues[5]{ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        // Optional per-joint (3 per finger: proximal/mid/distal) shaping; when
        // absent, joints are derived from fingerValues at apply time the same
        // way the live mesh solver does.
        bool hasFingerJointValues{ false };
        float fingerJointValues[15]{};
        // Exact rotations only: rest translations/scale remain FRIK-owned.
        // Ni row order, thumb through pinky, proximal through distal.
        bool hasExactFingerRotations{ false };
        float exactFingerRotations[15][9]{};
    };

    struct SavedGrabOffsetFile
    {
        std::uint32_t formatVersion{ kFormatVersion };
        FormRef object;
        // Display name, informational only (never used as identity).
        std::string objectName;
        HandOffset right;
        HandOffset left;
    };

    [[nodiscard]] std::string serialize(const SavedGrabOffsetFile& file);

    /*
     * Fail-closed parse: returns false when the document structure or format
     * version is unusable. outError (when provided) describes the failure.
     */
    [[nodiscard]] bool parse(std::string_view jsonText, SavedGrabOffsetFile& out, std::string* outError);
}
