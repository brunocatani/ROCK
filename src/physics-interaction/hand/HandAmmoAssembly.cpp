// Count-based loose ammo: keeps the reference root body and the visible
// branch body together as one rigid object while a hand pulls or holds it.
// Rationale and engine evidence: object/CountBasedAmmo.h.
#include "physics-interaction/hand/HandGrabInternal.h"

namespace rock
{
    namespace
    {
        const object_physics_body_set::ObjectPhysicsBodyRecord* acceptedRecordOwnedBy(
            const object_physics_body_set::ObjectPhysicsBodySet& bodySet, const RE::NiAVObject* node)
        {
            if (!node) {
                return nullptr;
            }
            for (const auto& record : bodySet.records) {
                if (record.accepted && record.owningNode == node) {
                    return &record;
                }
            }
            return nullptr;
        }
    }

    bool Hand::restoreCountBasedAmmoAssembly(RE::hknpWorld* world, const count_based_ammo::Assembly& assembly,
        const object_physics_body_set::ObjectPhysicsBodySet& bodySet, const char* context)
    {
        _ammoCarry = {};
        if (!world || !assembly.valid()) {
            return false;
        }

        const auto* anchor = acceptedRecordOwnedBy(bodySet, assembly.root);
        const auto* branch = acceptedRecordOwnedBy(bodySet, assembly.variantNode);
        if (!anchor || !branch || anchor->bodyId == branch->bodyId) {
            ROCK_LOG_SAMPLE_WARN(Hand, g_rockConfig.rockLogSampleMilliseconds,
                "{} hand AMMO ASSEMBLY unavailable: context={} variant={} anchorBody={} branchBody={}", handName(),
                context ? context : "unknown", count_based_ammo::variantKey(assembly.variant),
                anchor ? anchor->bodyId : INVALID_BODY_ID, branch ? branch->bodyId : INVALID_BODY_ID);
            return false;
        }

        RE::NiTransform anchorBodyWorld{};
        RE::NiTransform branchBodyWorld{};
        if (!havok_runtime::tryGetBodyArrayWorldTransform(world, RE::hknpBodyId{ anchor->bodyId }, anchorBodyWorld) ||
            !havok_runtime::tryGetBodyArrayWorldTransform(world, RE::hknpBodyId{ branch->bodyId }, branchBodyWorld)) {
            ROCK_LOG_SAMPLE_WARN(Hand, g_rockConfig.rockLogSampleMilliseconds,
                "{} hand AMMO ASSEMBLY unavailable: context={} body transforms unreadable anchorBody={} branchBody={}",
                handName(), context ? context : "unknown", anchor->bodyId, branch->bodyId);
            return false;
        }

        auto* root = assembly.root;
        auto* branchNode = assembly.variantNode;
        // The root body drives the root node, so their measured relation is
        // the body's fixed frame inside that node.
        const RE::NiTransform anchorBodyLocal = multiplyTransforms(invertTransform(root->world), anchorBodyWorld);
        const RE::NiTransform restRootWorld = branchNode->world;
        const RE::NiTransform anchorTarget = multiplyTransforms(restRootWorld, anchorBodyLocal);
        if (!isFiniteNiTransform(anchorTarget) || !isFiniteNiTransform(branchBodyWorld)) {
            return false;
        }
        const float separationGameUnits = pointDistanceGameUnits(root->world.translate, restRootWorld.translate);

        const RE::hkVector4f zero{};
        if (!havok_runtime::setBodyTransformDeferred(world, anchor->bodyId, anchorTarget, 1) ||
            !havok_runtime::setBodyVelocityDeferred(world, anchor->bodyId, zero, zero)) {
            return false;
        }
        (void)havok_runtime::activateBody(world, anchor->bodyId);

        // The body write lands on the next physics step; the scene graph moves
        // now so every root-relative capture this frame sees one object.
        bool sceneRestored = false;
        if (branchNode->parent && branchNode->parent == root->IsNode()) {
            root->local = root->parent ? multiplyTransforms(invertTransform(root->parent->world), restRootWorld)
                                       : restRootWorld;
            branchNode->local = makeIdentityTransform();
            RE::NiUpdateData update{};
            root->UpdateTransforms(update);
            sceneRestored = isFiniteNiTransform(root->world);
        }

        _ammoCarry.anchorBodyId = anchor->bodyId;
        _ammoCarry.primaryBodyId = branch->bodyId;
        _ammoCarry.anchorFromPrimary = multiplyTransforms(invertTransform(branchBodyWorld), anchorTarget);

        ROCK_LOG_DEBUG(Hand,
            "{} hand AMMO ASSEMBLY restored: context={} variant={} anchorBody={} branchBody={} separation={:.1f}gu "
            "sceneRestored={}",
            handName(), context ? context : "unknown", count_based_ammo::variantKey(assembly.variant), anchor->bodyId,
            branch->bodyId, separationGameUnits, sceneRestored ? "yes" : "no");
        return true;
    }

    void Hand::carryCountBasedAmmoAnchor(RE::hknpWorld* world)
    {
        if (!world || !_ammoCarry.active() || _ammoCarry.primaryBodyId != _savedObjectState.bodyId.value) {
            return;
        }
        RE::NiTransform primaryBodyWorld{};
        if (!havok_runtime::tryGetBodyArrayWorldTransform(world, RE::hknpBodyId{ _ammoCarry.primaryBodyId },
                primaryBodyWorld)) {
            return;
        }
        const RE::NiTransform anchorTarget = multiplyTransforms(primaryBodyWorld, _ammoCarry.anchorFromPrimary);
        if (!isFiniteNiTransform(anchorTarget)) {
            return;
        }
        // Overwritten every held frame; the anchor has no geometry, so its own
        // velocity only matters at release, where the held body-set velocity
        // (connected-dynamic scope) gives it the branch body's motion.
        (void)havok_runtime::setBodyTransformDeferred(world, _ammoCarry.anchorBodyId, anchorTarget, 1);
    }
}
