#pragma once

#include <cstdint>

namespace RE
{
    class NiAVObject;
    class hknpWorld;
}

namespace rock::physics_recursive_wrappers
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

    inline constexpr std::uint32_t toNativeMotionPreset(MotionPreset preset) { return static_cast<std::uint32_t>(preset); }

    bool setMotionRecursive(RE::NiAVObject* root, MotionPreset preset, bool recursive, bool force, bool activate);

    bool enableCollisionRecursive(RE::NiAVObject* root, bool enable, bool recursive, bool force);

    bool activateBody(RE::hknpWorld* world, std::uint32_t bodyId);
}
