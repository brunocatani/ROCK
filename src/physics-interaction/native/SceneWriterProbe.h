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
    // Game thread only. Safe to call when nothing is registered.
    void clearHeldTarget(bool isLeft);

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
        bool installed = false;
    };
    void copyStatus(Status& out);
}
