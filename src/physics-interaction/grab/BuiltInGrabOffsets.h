#pragma once

#include "physics-interaction/grab/SavedGrabOffsetFormat.h"

#include <cstdint>
#include <span>
#include <string_view>

/*
 * ROCK's built-in grab offsets: the hand poses ROCK ships for ordinary objects,
 * compiled into the DLL. A pull or forced arrival uses them for a hand when no
 * saved offset and no provider default pose exists, instead of the dynamic seat.
 *
 * The tables live in BuiltInGrabOffsetData.cpp, generated from the grab-offset
 * authoring data (Offset Maker); regenerate it rather than editing it. Same
 * conventions as a saved offset (SavedGrabOffsetFormat.h): proxy-local
 * translation in game units, row-major rotation, thumb-to-pinky finger values.
 */
namespace rock::saved_grab_offset::built_in
{
    inline constexpr std::int32_t kNone = -1;

    enum HandPoseFlag : std::uint8_t
    {
        kFingerValues = 1,
        kFingerJointValues = 2,
    };

    struct ExactFingerRotations
    {
        // Ni rows, thumb through pinky, proximal through distal.
        float rotations[15][9];
    };

    struct HandPose
    {
        float translateGame[3];
        float rotate[9];
        float fingerValues[5];
        float fingerJointValues[15];
        std::uint8_t flags;
        // Index into exactFingerRotations(), or kNone.
        std::int32_t exactFingerRotations;
    };

    // Sorted by (plugin, localFormId, variant); plugin names are lowercase.
    struct Offset
    {
        std::string_view plugin;
        std::uint32_t localFormId;
        std::string_view variant;
        // Indices into handPoses(), or kNone.
        std::int32_t right;
        std::int32_t left;
    };

    // A form with no offset of its own, served by another form's offset (same
    // world model or family). Sorted by (plugin, localFormId); plugin lowercase.
    struct Alias
    {
        std::string_view plugin;
        std::uint32_t localFormId;
        // Index into offsets().
        std::uint32_t offset;
    };

    // A pose ROCK uses by role rather than by form ("grenade").
    struct NamedOffset
    {
        std::string_view name;
        // Index into offsets().
        std::uint32_t offset;
    };

    inline constexpr std::string_view kGrenadePose = "grenade";
    inline constexpr std::string_view kMolotovPose = "molotov";

    // Generated tables.
    [[nodiscard]] std::span<const HandPose> handPoses() noexcept;
    [[nodiscard]] std::span<const ExactFingerRotations> exactFingerRotations() noexcept;
    [[nodiscard]] std::span<const Offset> offsets() noexcept;
    [[nodiscard]] std::span<const Alias> aliases() noexcept;
    [[nodiscard]] std::span<const NamedOffset> namedOffsets() noexcept;

    // The built-in pose for one hand: the form's own offset, otherwise the
    // offset that serves it. Variants (count-based ammo) never use aliases.
    [[nodiscard]] bool find(const FormRef& object, bool left, HandOffset& out);

    // The built-in pose for one hand of a role (kGrenadePose).
    [[nodiscard]] bool findNamed(std::string_view name, bool left, HandOffset& out);
}
