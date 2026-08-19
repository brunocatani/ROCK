#pragma once

/*
 * The one entry point into the weapon emitter scan.
 *
 * Emitter discovery lives in WeaponCollisionEmitters.cpp, but the generated-source
 * scan needs a snapshot too. This header is that seam, and it is deliberately one
 * function wide: everything else about emitters - the visibility predicates, the
 * merge rules, the descriptor lifetime - stays private to the emitters file, so
 * the source scan cannot start making emitter policy decisions of its own.
 *
 * INTERNAL. Never reachable from any public include tree.
 */

#include "physics-interaction/weapon/collision/WeaponCollision.h"

#include <cstdint>
#include <unordered_map>

namespace RE
{
    class NiAVObject;
}

namespace rock::weapon_collision_detail
{
    /*
     * Walk the weapon under `weaponNode` and return every light, laser and reticle
     * emitter it carries.
     *
     * The three keys are identity, not inputs: the snapshot records the equipped
     * weapon, the published body-set generation and the set of roots that were
     * scanned, so a consumer can tell a stale snapshot from a current one without
     * re-walking the tree. `omodByAttachPointFormId` attributes each emitter to the
     * OMOD that installed it.
     *
     * Allocation is bounded by the emitter capacity in WeaponEmitterSnapshot; the
     * walk itself is bounded by depth and visited-node budgets. No engine pointer
     * outlives the returned snapshot's generation key.
     */
    [[nodiscard]] WeaponEmitterSnapshot collectWeaponEmitterSnapshot(
        RE::NiAVObject* weaponNode,
        const std::unordered_map<std::uint32_t, std::uint32_t>& omodByAttachPointFormId,
        std::uint64_t equippedWeaponKey,
        std::uint64_t weaponGenerationKey,
        std::uint64_t rootSetKey);
}
