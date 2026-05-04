#pragma once

#include <cstdint>

namespace RE
{
    class NiAVObject;
    class hknpWorld;
}

namespace frik::rock::physics_recursive_wrappers
{
    /*
     * ROCK uses FO4VR's recursive NiAVObject physics command wrappers for
     * active object preparation because the engine already walks subtree
     * collision owners safely. Direct bhkNPCollisionObject::SetMotionType only
     * mutates one collision object and leaves multi-child props partially
     * prepared, which is the failure mode seen on split-collision weapons.
     */
    enum class MotionPreset : std::uint32_t
    {
        Static = 0,
        Dynamic = 1,
        Keyframed = 2,
    };

    struct SetMotionCommand
    {
        std::uint32_t presetValue = 0;
        bool recursive = false;
        bool force = false;
        bool activate = false;
    };

    struct EnableCollisionCommand
    {
        bool enable = false;
        bool recursive = false;
        bool force = false;
    };

    inline constexpr std::uint32_t toNativeMotionPreset(MotionPreset preset) { return static_cast<std::uint32_t>(preset); }

    inline constexpr SetMotionCommand makeSetMotionCommand(MotionPreset preset, bool recursive, bool force, bool activate)
    {
        return SetMotionCommand{ .presetValue = toNativeMotionPreset(preset), .recursive = recursive, .force = force, .activate = activate };
    }

    inline constexpr EnableCollisionCommand makeEnableCollisionCommand(bool enable, bool recursive, bool force)
    {
        return EnableCollisionCommand{ .enable = enable, .recursive = recursive, .force = force };
    }

    bool setMotionRecursive(RE::NiAVObject* root, MotionPreset preset, bool recursive, bool force, bool activate);

    bool enableCollisionRecursive(RE::NiAVObject* root, bool enable, bool recursive, bool force);

    bool activateBody(RE::hknpWorld* world, std::uint32_t bodyId);
}
