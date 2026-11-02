#include "physics-interaction/weapon/scope/ScopeRigParentRepair.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "rock_support/Fo4VrRuntime.h"

namespace rock::scope_rig_parent_repair
{
    namespace
    {
        void reattach(RE::NiNode* node, RE::NiNode* parent, const char* name) noexcept
        {
            if (!node || node->parent || !parent)
                return;
            const RE::NiTransform world = node->world;
            parent->AttachChild(node, true);
            node->local = transform_math::composeTransforms(transform_math::invertTransform(parent->world), world);
            node->world = world;
            ROCK_LOG_WARN(Weapon,
                "Orphaned {} returned to its wand-chain parent {} after its carried weapon was torn down", name,
                parent->name.c_str());
        }
    }

    void reattachOrphanedScopeRig() noexcept
    {
        auto* nodes = f4vr::getPlayerNodes();
        if (!nodes)
            return;
        reattach(nodes->primaryWeaponScopeCamera, nodes->primaryWeaponOffsetNOde, "scope camera");
        reattach(nodes->ScopeParentNode, nodes->primaryUIAttachNode, "scope parent");
    }
}
