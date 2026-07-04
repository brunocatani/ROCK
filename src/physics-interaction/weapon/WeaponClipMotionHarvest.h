#pragma once

#include <cstdint>

#include "physics-interaction/weapon/WeaponClipStrokePolicy.h"

namespace rock::weapon_clip_motion_harvest
{
    /*
     * Baked-animation stroke harvest. Weapon part motion (bolt, slide,
     * magazine) is authored as clips whose rig bone names match the weapon's
     * scene node names ('WeaponBolt' on vanilla, 'P320_Slide' on mods). The
     * clips' bindings live in a graph's hkbAnimationBindingSet — but which
     * graph carries them varies: the biped-slot weapon holders were verified
     * in-game (2026-07-04) to run one-bone 'x_bone01' dummy rigs with empty
     * sets, so the walk is manager-based and the caller offers every
     * candidate BSAnimationGraphManager (weapon holders and the actor's own
     * manager, where weapon subgraphs are activated on equip). This module
     * walks the chosen manager's active-graph binding set on the MAIN THREAD
     * after the weapon's colliders finish creation — no hook, no animation
     * playback — and samples each clip with the engine's own hkaAnimation
     * sampler, reducing the tracks to authored stroke groups queued for
     * attribution to the weapon's evidence parts.
     *
     * Tracks are kept only when their bone name matches one of the caller's
     * weapon scene-node names, which on an actor graph filters out every
     * body clip.
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
     * Resolve a WeaponAnimationGraphManagerHolder's BSAnimationGraphManager
     * smart pointer (+0x18, plausibility-gated); null when unavailable. The
     * walk itself is manager-based because weapon clips can live either on a
     * weapon holder's own graph or — as in-game diagnostics showed for the
     * biped-slot holders (one-bone 'x_bone01' dummy rigs, 2026-07-04) — on
     * the ACTOR's graph manager after subgraph activation.
     */
    [[nodiscard]] const void* managerFromWeaponHolder(const void* weaponGraphHolder);

    /*
     * Advance the walk over the manager's active graph bindings.
     * `graphManager` is a live BSAnimationGraphManager (non-owning; must not
     * be retained). Only clip tracks whose rig bone name matches one of
     * `allowedNodeNames` (weapon scene-node names; ':N' instancing suffix on
     * the node side is tolerated, comparison is case-insensitive) are
     * harvested — on an actor graph this filters out every body clip. A
     * change of formId/generationKey, or of the underlying binding-set data
     * (graph swap mid-walk), resets the cursor automatically.
     */
    StepResult stepHarvest(
        const void* graphManager,
        std::uint32_t weaponFormId,
        std::uint64_t weaponGenerationKey,
        const char* const* allowedNodeNames,
        std::uint32_t allowedNodeNameCount);

    // Forget the walk cursor (weapon changed / sandbox disabled).
    void resetWalk();

    /*
     * Rewind the walk cursor for another pass over the same weapon
     * generation, keeping cumulative stats and the per-generation bail-dump
     * budget. Clip spline payloads stream in only while a clip is playing,
     * so the walk owner re-runs completed walks periodically — a reload
     * performed while the weapon is held makes its clips resident and the
     * next pass harvests them.
     */
    void restartWalkPass();

    // Deepest holder-chain hop reached by the most recent resolve attempt
    // ("ok" when bindings were reachable); for give-up diagnostics.
    [[nodiscard]] const char* lastResolveStage();

    /*
     * Cheap pointer-walk probe: true when the manager's active graph
     * currently exposes a non-empty binding set. Used to pick the walk
     * target among several candidate managers — both biped-slot holder
     * copies carry a one-bone dummy rig ('x_bone01') with an empty set
     * (verified in-game 2026-07-04), so the first candidate whose set has
     * bindings wins.
     */
    [[nodiscard]] bool probeBindings(const void* graphManager);

    /*
     * FO4 streams clip spline payloads on demand: the binding-set entries
     * are permanent stubs (headers only, no sampleable data — in-game
     * confirmed: firing/reloading never fills them), and the loaded binding
     * lives on the hkbClipGenerator while its clip plays. This hook swaps
     * the hkbClipGenerator vtable's install-loaded-binding slot (an atomic
     * pointer write) and harvests from the freshly installed binding — the
     * one moment the payload is guaranteed resident. Idempotent; call from
     * the main thread once the sandbox is active.
     */
    void ensureClipActivationHookInstalled();

    /*
     * Register what the hook may harvest: the characters of the candidate
     * managers' graphs (the hook fires for every actor, so anything else is
     * ignored) and the weapon's scene-node names (copied — the hook thread
     * never touches scene-graph memory). Refresh whenever the walk steps;
     * clear when the sandbox shuts down.
     */
    void setClipActivationTargets(
        const void* const* graphManagers,
        std::uint32_t managerCount,
        const char* const* allowedNodeNames,
        std::uint32_t allowedNodeNameCount);
    void clearClipActivationTargets();

    /*
     * One-shot dump of the manager→bindings chain: raw pointer of every hop,
     * each object's vtable rebased to a module offset (identifies the actual
     * runtime type in Ghidra), skeleton bone count/names, and the binding
     * set's raw data/count. Called by the walk owner when it gives up, so a
     * failing hop can be diagnosed from the log without a debugger. Reads are
     * plausibility-gated the same way as the resolve itself. `label` names
     * the candidate in the log (e.g. "weapon-holder" / "actor").
     */
    void logResolveDiagnostics(const void* graphManager, const char* label);

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
        // Which harvestBinding gate rejected bindings (details are dumped,
        // capped per walk, as "binding bail" warnings).
        std::uint64_t bailAnimationPtr{ 0 };
        std::uint64_t bailClipParams{ 0 };
        std::uint64_t bailTrackMap{ 0 };
        std::uint64_t bailBoneCount{ 0 };
        std::uint64_t bailSampler{ 0 };
        // Spline payload not resident (engine sampler would crash on it).
        std::uint64_t bailSplineData{ 0 };
        // Clip-activation hook: total shim entries / entries that passed the
        // registered-character filter.
        std::uint64_t hookFires{ 0 };
        std::uint64_t hookActivations{ 0 };
    };
    [[nodiscard]] Stats snapshotStats();
}
