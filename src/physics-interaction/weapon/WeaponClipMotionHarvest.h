#pragma once

#include <cstdint>

#include "physics-interaction/weapon/WeaponClipStrokePolicy.h"

namespace rock::weapon_clip_motion_harvest
{
    /*
     * Baked-animation stroke harvest. A byte-validated entry hook on
     * hkbBehaviorLoadingUtils::assignAnimationBinding observes every animation
     * binding the engine links while loading a behavior graph or weapon
     * subgraph. For bindings whose tracks target Weapon* rig bones, the hook
     * samples the clip with the engine's own hkaAnimation sampler (no
     * playback) and reduces the tracks to authored stroke groups, queued as
     * plain data for main-thread attribution to the equipped weapon.
     *
     * Everything below BSAnimationGraphManager is accessed through offsets
     * verified against the FO4VR binary — see
     * docs/research/2026-07-03-baked-animation-motion-extraction.md for the
     * evidence trail of every constant used here.
     *
     * Thread model: the hook runs on the engine's loading path (any thread);
     * sampling happens entirely in-hook against objects that are alive for
     * the duration of the call, so no engine pointers ever cross threads —
     * only plain sampled data enters the mutex-guarded queue. The queue is
     * drained on the main update thread. Fails closed at every step: any
     * unexpected pointer, size, format, or duration skips that binding.
     */

    // Installed once at plugin startup, before graphs load. The hook is inert
    // (immediate passthrough) while the bolt-drive sandbox is disabled.
    [[nodiscard]] bool installHook();
    [[nodiscard]] bool hookInstalled();

    // Main-thread drain of harvested stroke groups (weapon-bone local space;
    // attribution/space conversion is the caller's job). Returns the number
    // of groups written to outGroups.
    std::uint32_t drainGroups(
        weapon_clip_stroke::AuthoredStrokeGroup* outGroups,
        std::uint32_t maxGroups);

    // Drop queued groups (weapon changed; pending strokes may belong to the
    // previous weapon's subgraph).
    void clearPending();

    /*
     * Diagnostic aid for weapons whose clips never yield Weapon* tracks
     * (modded rigs with custom bone names): grants `budget` one-shot log
     * lines that dump the bone names of small bindings with zero harvest
     * targets. Re-armed by the drain owner on every weapon change so the
     * next equip's bindings are captured instead of load-screen noise.
     */
    void armNoTargetNameDumps(std::uint32_t budget);

    struct Stats
    {
        std::uint64_t bindingsSeen{ 0 };
        std::uint64_t bindingsHarvested{ 0 };
        std::uint64_t bindingsNoTargets{ 0 };
        std::uint64_t groupsQueued{ 0 };
        std::uint64_t groupsDropped{ 0 };
        std::uint64_t skippedNonSpline{ 0 };
    };
    [[nodiscard]] Stats snapshotStats();
}
