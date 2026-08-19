#pragma once

/*
 * Liveness probe on the engine's physics-to-scene transform writer
 * (FO4VR 1.2.72 RVA 0x1E06B00). Per the 2026-08-18 physics-to-scene sync
 * dossier, this function is the first verified render-facing storage write in
 * the generic dynamic-body display path:
 *
 *   NiAVObject::UpdateWorldData -> collision vslot +0x158
 *     -> hknpWorld::predictBodyTransform (motion-record based)
 *     -> bhkNPCollisionObject(Proxy)::UpdateWorldData
 *     -> writer 0x1E06B00 (this hook) -> NiAVObject local/world
 *
 * The probe answers, for the exact held object only:
 *   1. does held clutter traverse this writer at all (main vs proxy callsite);
 *   2. what transform arrives (motion-predicted pose vs ROCK's anchor);
 *   3. does a display-only substitution at this boundary move the rendered
 *      mesh (the interception-boundary confirmation).
 *
 * Ownership/threading: registration and clearing happen on the game thread at
 * grab commit / release / world loss. The hook may run on any scene-update
 * thread, so the per-hand target slots use a seqlock-style generation; the
 * hook never blocks, never allocates, and fails closed to the original writer
 * on any doubt. The optional display offset (INI
 * fGrabSceneWriterProbeOffsetZGameUnits) modifies only a stack-local copy of
 * the writer input; engine storage and Havok state are never touched by the
 * probe itself.
 */

#include <cstddef>
#include <cstdint>

#include "RE/NetImmerse/NiTransform.h"

namespace RE
{
    class hknpWorld;
    class NiCollisionObject;
}

namespace rock::scene_writer_probe
{
    inline constexpr std::size_t kMaxTrackedCollisionObjects = 8;

    struct HeldTargetRegistration
    {
        const RE::NiCollisionObject* collisionObjects[kMaxTrackedCollisionObjects] = {};
        std::uint32_t collisionObjectCount = 0;
        RE::hknpWorld* world = nullptr;
        std::uint32_t bodyId = 0x7FFF'FFFF;
        float havokToGame = 0.0f;
        std::uint64_t traceId = 0;
    };

    // Byte-validated entry detour; idempotent, fails closed (probe disabled,
    // engine untouched) on prefix mismatch. Install once at plugin load.
    bool install();
    bool isInstalled();

    // Game thread only. Overwrites the hand's slot for the current grab.
    void registerHeldTarget(bool isLeft, const HeldTargetRegistration& registration);
    // Game thread only. Safe to call when nothing is registered. Also drops
    // the published anchor.
    void clearHeldTarget(bool isLeft);

    /*
     * Held-object render-pose sync (Contract A per the 2026-08-18 dossier and
     * the confirmed liveness probe): the hook substitutes the newest published
     * BODY-space anchor for the solver pose in the writer input, so the drawn
     * object and the rendered hand share one clock. The solver, motion record,
     * and every other object stay untouched. Divergence guard: at
     * anchor-vs-solver gaps above the full-anchor threshold the translation
     * blends back toward the solver pose, and above the solver threshold the
     * substitution is skipped entirely (object blocked by geometry must render
     * physically). Stage tells the log which clock the writer consumed.
     */
    enum class AnchorStage : std::uint8_t
    {
        None = 0,
        Producer = 1,
        PreFrik = 2,
    };
    /*
     * Controller-root sample taken when the anchor was computed. The hook
     * samples the live root again at write time and adds (current - source)
     * to the anchor translation. This removes the engine's mid-frame
     * locomotion step from the anchor (the 12:56 session proved the writer
     * consumes the producer-stage anchor, which is one room step stale).
     * Room-scale hand motion does not move the root, so it is untouched.
     */
    struct AnchorRootSample
    {
        float positionHavok[3] = { 0.0f, 0.0f, 0.0f };
        std::uintptr_t controllerIdentity = 0;
        float havokToGame = 0.0f;
        bool valid = false;
    };
    // Game thread only. Publish the held BODY anchor in game space.
    void publishHeldAnchor(
        bool isLeft,
        const RE::NiTransform& bodyAnchorWorldGame,
        AnchorStage stage,
        const AnchorRootSample& sourceRoot);
    // Game thread only. The hook falls back to the untouched solver pose.
    void invalidateHeldAnchor(bool isLeft);

    struct Status
    {
        std::uint64_t matchedCalls = 0;
        std::uint64_t mainCallsiteCalls = 0;
        std::uint64_t proxyCallsiteCalls = 0;
        std::uint64_t otherCallsiteCalls = 0;
        std::uint64_t offsetAppliedCalls = 0;
        std::uint64_t callbackFlagCalls = 0;
        std::uint64_t localFlagCalls = 0;
        std::uint64_t totalWriterCalls = 0;
        std::uint64_t syncAppliedCalls = 0;
        std::uint64_t syncProducerStageCalls = 0;
        std::uint64_t syncPreFrikStageCalls = 0;
        std::uint64_t syncDivergenceSkips = 0;
        std::uint64_t syncRebasedCalls = 0;
        std::uint64_t syncRebaseSkips = 0;
        bool installed = false;
    };
    void copyStatus(Status& out);
}
