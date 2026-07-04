#pragma once

#include <cstdint>

#include "physics-interaction/weapon/WeaponClipStrokePolicy.h"

namespace rock::weapon_clip_motion_harvest
{
    /*
     * Baked-animation stroke harvest. FO4 animates weapon parts (bolt, slide,
     * magazine) through the WEAPON's own behavior graph: every equipped item
     * carries a WeaponAnimationGraphManagerHolder whose BShkbAnimationGraph
     * runs on the weapon's rig, and that graph's hkbAnimationBindingSet holds
     * every clip binding for the weapon. This module walks that set on the
     * MAIN THREAD after the weapon's colliders finish creation — no hook, no
     * animation playback — and samples each clip with the engine's own
     * hkaAnimation sampler, reducing the tracks to authored stroke groups
     * queued for attribution to the weapon's evidence parts.
     *
     * Because the tracks target the weapon rig, the bone names ARE the
     * weapon's scene node names ('WeaponBolt' on vanilla, 'P320_Slide' on
     * mods), so every bone except the rig root is a harvest target.
     *
     * All engine access below the holder pointer goes through offsets
     * verified against the FO4VR binary — see
     * docs/research/2026-07-03-baked-animation-motion-extraction.md
     * (Addendum 2) for the evidence trail of every constant used here.
     * Every pointer hop passes a plausibility gate and fails closed.
     *
     * Thread model: main thread only (PhysicsInteraction update). The walk is
     * time-sliced (a few bindings per call) so a large clip set cannot hitch
     * a frame; no engine pointer is retained between calls — the chain is
     * re-resolved from the holder every step.
     */

    enum class StepResult : std::uint32_t
    {
        // Bindings not available yet (graph still loading) or budget spent
        // for this call; call again next frame.
        Pending = 0,
        // Every binding of the weapon graph has been processed for this
        // (formId, generationKey).
        Completed = 1,
    };

    /*
     * Advance the walk over the equipped weapon's graph bindings.
     * `weaponGraphHolder` is the biped slot's WeaponAnimationGraphManagerHolder
     * (non-owning; must be the live equipped-weapon holder this frame). A
     * change of formId/generationKey resets the cursor automatically.
     */
    StepResult stepHarvest(const void* weaponGraphHolder, std::uint32_t weaponFormId, std::uint64_t weaponGenerationKey);

    // Forget the walk cursor (weapon changed / sandbox disabled).
    void resetWalk();

    // Deepest holder-chain hop reached by the most recent resolve attempt
    // ("ok" when bindings were reachable); for give-up diagnostics.
    [[nodiscard]] const char* lastResolveStage();

    /*
     * One-shot dump of the holder→bindings chain: raw pointer of every hop,
     * each object's vtable rebased to a module offset (identifies the actual
     * runtime type in Ghidra), skeleton bone count/names, and the binding
     * set's raw data/count. Called by the walk owner when it gives up, so a
     * failing hop can be diagnosed from the log without a debugger. Reads are
     * plausibility-gated the same way as the resolve itself.
     */
    void logResolveDiagnostics(const void* weaponGraphHolder);

    // Main-thread drain of harvested stroke groups (weapon-bone local space;
    // attribution/space conversion is the caller's job). Returns the number
    // of groups written to outGroups.
    std::uint32_t drainGroups(
        weapon_clip_stroke::AuthoredStrokeGroup* outGroups,
        std::uint32_t maxGroups);

    // Drop queued groups (weapon changed; pending strokes may belong to the
    // previous weapon's graph).
    void clearPending();

    struct Stats
    {
        std::uint64_t bindingsSeen{ 0 };
        std::uint64_t bindingsHarvested{ 0 };
        std::uint64_t bindingsNoTargets{ 0 };
        std::uint64_t groupsQueued{ 0 };
        std::uint64_t groupsDropped{ 0 };
        std::uint64_t skippedNonSpline{ 0 };
        std::uint64_t walksCompleted{ 0 };
    };
    [[nodiscard]] Stats snapshotStats();
}
