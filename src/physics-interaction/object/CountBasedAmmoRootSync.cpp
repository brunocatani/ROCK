#include "physics-interaction/object/CountBasedAmmoRootSync.h"

#include <cmath>

#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Havok/hkVector4.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiNode.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/native/havok/HavokRuntime.h"
#include "physics-interaction/native/havok/PhysicsUtils.h"
#include "physics-interaction/object/CountBasedAmmo.h"

namespace rock::count_based_ammo
{
    namespace
    {
        // The root body may drift this far from the branch body before a
        // resting round gets a write; writes wake the bodies.
        constexpr float kSyncToleranceGameUnits = 0.25f;
        // Branch-body speed below which the round counts as resting.
        constexpr float kRestSpeedGameUnitsPerSecond = 1.5f;

        bool finiteTransform(const RE::NiTransform& value)
        {
            if (!std::isfinite(value.translate.x) || !std::isfinite(value.translate.y) ||
                !std::isfinite(value.translate.z) || !std::isfinite(value.scale)) {
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

        float distanceGameUnits(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            const float dx = a.x - b.x;
            const float dy = a.y - b.y;
            const float dz = a.z - b.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        /*
         * The root body's frame in the branch body's frame when the round is
         * whole: the root node frame equals the branch node frame
         * (CountBasedAmmo.h), so only each body's fixed frame inside its own
         * node remains. Each body drives its node, so body and node world
         * agree up to the node's one-frame lag on a moving round.
         */
        bool measureRootFromBranch(RE::hknpWorld* world, const Assembly& assembly, const AssemblyBodies& bodies,
            RE::NiTransform& outRootFromBranch)
        {
            RE::NiTransform rootBodyWorld{};
            RE::NiTransform branchBodyWorld{};
            if (!havok_runtime::tryGetBodyArrayWorldTransform(world, RE::hknpBodyId{ bodies.root }, rootBodyWorld) ||
                !havok_runtime::tryGetBodyArrayWorldTransform(world, RE::hknpBodyId{ bodies.branch },
                    branchBodyWorld)) {
                return false;
            }
            using transform_math::composeTransforms;
            using transform_math::invertTransform;
            const RE::NiTransform rootBodyLocal = composeTransforms(invertTransform(assembly.root->world), rootBodyWorld);
            const RE::NiTransform branchBodyLocal =
                composeTransforms(invertTransform(assembly.variantNode->world), branchBodyWorld);
            outRootFromBranch = composeTransforms(invertTransform(branchBodyLocal), rootBodyLocal);
            outRootFromBranch.scale = 1.0f;
            return finiteTransform(outRootFromBranch);
        }
    }

    void RootBodySync::track(RE::TESObjectREFR* refr, bool held)
    {
        if (!isCountBasedAmmo(refr)) {
            return;
        }
        const auto formId = refr->GetFormID();
        Entry* freeSlot = nullptr;
        Entry* mostRested = nullptr;
        for (auto& entry : _entries) {
            if (entry.active() && entry.formId == formId) {
                entry.held = entry.held || held;
                entry.touched = true;
                return;
            }
            if (!entry.active()) {
                freeSlot = freeSlot ? freeSlot : &entry;
            } else if (!entry.held && !entry.touched &&
                       (!mostRested || entry.restSeconds > mostRested->restSeconds)) {
                mostRested = &entry;
            }
        }
        // Full: the round resting longest gives way; its bodies are together
        // or very nearly so.
        auto* slot = freeSlot ? freeSlot : mostRested;
        if (!slot) {
            return;
        }
        *slot = Entry{};
        slot->handle = refr->GetHandle();
        slot->formId = formId;
        slot->held = held;
        slot->touched = true;
    }

    void RootBodySync::update(RE::hknpWorld* world, float deltaSeconds)
    {
        const float dt = std::isfinite(deltaSeconds) && deltaSeconds > 0.0f ? deltaSeconds : 0.0f;
        for (auto& entry : _entries) {
            if (!entry.active()) {
                continue;
            }
            const bool held = entry.held;
            const bool touched = entry.touched;
            entry.held = false;
            entry.touched = false;

            const auto refPtr = entry.handle.get();
            auto* refr = refPtr.get();
            if (!world || !refr || refr->GetFormID() != entry.formId || refr->IsDeleted() || refr->IsDisabled()) {
                entry = {};
                continue;
            }
            const auto assembly = resolve(refr);
            const auto bodies = resolveBodies(world, assembly);
            if (!bodies.valid()) {
                entry = {};
                continue;
            }
            if (!entry.measured) {
                if (!measureRootFromBranch(world, assembly, bodies, entry.rootFromBranch)) {
                    entry = {};
                    continue;
                }
                entry.measured = true;
            }
            // The holding or pulling hand carries the root body itself.
            if (held) {
                entry.restSeconds = 0.0f;
                continue;
            }

            RE::NiTransform rootBodyWorld{};
            RE::NiTransform branchBodyWorld{};
            auto* branchMotion = havok_runtime::getBodyMotion(world, RE::hknpBodyId{ bodies.branch });
            if (!branchMotion ||
                !havok_runtime::tryGetBodyArrayWorldTransform(world, RE::hknpBodyId{ bodies.root }, rootBodyWorld) ||
                !havok_runtime::tryGetBodyArrayWorldTransform(world, RE::hknpBodyId{ bodies.branch },
                    branchBodyWorld)) {
                entry = {};
                continue;
            }
            RE::NiTransform target = transform_math::composeTransforms(branchBodyWorld, entry.rootFromBranch);
            target.scale = rootBodyWorld.scale;
            const RE::hkVector4f branchVelocity = branchMotion->linearVelocity;
            const float branchSpeedGameUnits = std::sqrt(branchVelocity.x * branchVelocity.x +
                                                   branchVelocity.y * branchVelocity.y +
                                                   branchVelocity.z * branchVelocity.z) *
                havokToGameScale();
            if (!finiteTransform(target) || !std::isfinite(branchSpeedGameUnits)) {
                entry = {};
                continue;
            }

            const float separation = distanceGameUnits(rootBodyWorld.translate, target.translate);
            const bool moving = branchSpeedGameUnits > kRestSpeedGameUnitsPerSecond;
            if (moving || separation > kSyncToleranceGameUnits) {
                const RE::hkVector4f zero{};
                if (!havok_runtime::setBodyTransformDeferred(world, bodies.root, target, 1) ||
                    !havok_runtime::setBodyVelocityDeferred(world, bodies.root, branchVelocity, zero)) {
                    ROCK_LOG_SAMPLE_WARN(Hand, 2000,
                        "Count-based ammo root sync write failed: ref={:08X} rootBody={} branchBody={}", entry.formId,
                        bodies.root, bodies.branch);
                    entry = {};
                    continue;
                }
                if (separation > 2.0f) {
                    ROCK_LOG_SAMPLE_DEBUG(Hand, 1000,
                        "Count-based ammo root body rejoined its round: ref={:08X} rootBody={} branchBody={} "
                        "separation={:.1f}gu",
                        entry.formId, bodies.root, bodies.branch, separation);
                }
                entry.restSeconds = 0.0f;
                continue;
            }

            entry.restSeconds += dt;
            if (entry.restSeconds >= kRestSeconds && !touched) {
                entry = {};
            }
        }
    }
}
