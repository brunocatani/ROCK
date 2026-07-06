#include "physics-interaction/native/PhysicsRecursiveWrappers.h"

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/PhysicsLog.h"

#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiUpdateData.h"
#include "RE/Havok/hknpWorld.h"

#include <cmath>

namespace rock::physics_recursive_wrappers
{
    namespace
    {
        bool isFiniteNiTransform(const RE::NiTransform& value)
        {
            if (!std::isfinite(value.translate.x) || !std::isfinite(value.translate.y) || !std::isfinite(value.translate.z) ||
                !std::isfinite(value.scale)) {
                return false;
            }
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(value.rotate.entry[row][column])) {
                        return false;
                    }
                }
            }
            return true;
        }
    }

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

    bool setRootNodeWorldTransform(RE::NiAVObject* rootNode, const RE::NiTransform& desiredWorld)
    {
        if (!rootNode || !isFiniteNiTransform(desiredWorld)) {
            return false;
        }

        rootNode->local = rootNode->parent ?
            transform_math::composeTransforms(transform_math::invertTransform(rootNode->parent->world), desiredWorld) :
            desiredWorld;

        RE::NiUpdateData update{};
        rootNode->UpdateTransforms(update);

        if (!isFiniteNiTransform(rootNode->world)) {
            ROCK_LOG_DEBUG(Hand, "setRootNodeWorldTransform produced a non-finite world transform for root='{}'",
                rootNode->name.c_str() ? rootNode->name.c_str() : "(unnamed)");
            return false;
        }
        return true;
    }
}
