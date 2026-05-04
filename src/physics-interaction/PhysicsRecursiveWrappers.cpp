#include "PhysicsRecursiveWrappers.h"

#include "HavokRuntime.h"
#include "HavokOffsets.h"
#include "PhysicsLog.h"

#include "RE/NetImmerse/NiAVObject.h"
#include "RE/Havok/hknpWorld.h"

namespace frik::rock::physics_recursive_wrappers
{
    bool setMotionRecursive(RE::NiAVObject* root, MotionPreset preset, bool recursive, bool force, bool activate)
    {
        if (!root) {
            return false;
        }

        using SetMotion_t = std::uint8_t (*)(RE::NiAVObject*, std::uint32_t, bool, bool, bool);
        static REL::Relocation<SetMotion_t> setMotion{ REL::Offset(offsets::kFunc_BhkWorld_SetMotionRecursive) };
        const auto result = setMotion(root, toNativeMotionPreset(preset), recursive, force, activate);
        if (!result) {
            ROCK_LOG_DEBUG(Hand, "setMotionRecursive rejected root='{}' preset={} recursive={} force={} activate={}",
                root->name.c_str() ? root->name.c_str() : "(unnamed)",
                toNativeMotionPreset(preset),
                recursive,
                force,
                activate);
        }
        return result != 0;
    }

    bool enableCollisionRecursive(RE::NiAVObject* root, bool enable, bool recursive, bool force)
    {
        if (!root) {
            return false;
        }

        using EnableCollision_t = std::uint8_t (*)(RE::NiAVObject*, bool, bool, bool);
        static REL::Relocation<EnableCollision_t> enableCollision{ REL::Offset(offsets::kFunc_World_EnableCollision) };
        const auto result = enableCollision(root, enable, recursive, force);
        if (!result) {
            ROCK_LOG_DEBUG(Hand, "enableCollisionRecursive rejected root='{}' enable={} recursive={} force={}",
                root->name.c_str() ? root->name.c_str() : "(unnamed)",
                enable,
                recursive,
                force);
        }
        return result != 0;
    }

    bool activateBody(RE::hknpWorld* world, std::uint32_t bodyId)
    {
        return havok_runtime::activateBody(world, bodyId);
    }
}
