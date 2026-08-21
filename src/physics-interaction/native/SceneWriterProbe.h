#pragma once

/*
 * Held-object presentation boundary on the engine's physics-to-scene
 * transform writer
 * (FO4VR 1.2.72 RVA 0x1E06B00). Per the 2026-08-18 physics-to-scene sync
 * dossier, this function is the first verified render-facing storage write in
 * the generic dynamic-body display path:
 *
 *   NiAVObject::UpdateWorldData -> collision vslot +0x158
 *     -> hknpWorld::predictBodyTransform (motion-record based)
 *     -> bhkNPCollisionObject(Proxy)::UpdateWorldData
 *     -> writer 0x1E06B00 (this hook) -> NiAVObject local/world
 *
 * For an exact registered held object, the boundary can substitute ROCK's
 * published BODY-space presentation anchor for the solver pose. It does not
 * modify the solver, motion record, or any unregistered object.
 *
 * Ownership/threading: registration and clearing happen on the game thread at
 * grab commit / release / world loss. The hook may run on any scene-update
 * thread, so the per-hand target slots and immutable runtime configuration use
 * atomic seqlock-style publication. A game-thread NiPointer plus a bounded
 * reader retirement path keeps the live room node valid for every matched
 * hook reader. The hook
 * never logs, blocks, allocates, or reads mutable global configuration. The
 * optional display offset (INI
 * fGrabSceneWriterProbeOffsetZGameUnits) modifies only a stack-local copy of
 * the writer input; engine storage and Havok state are never touched directly.
 */

#include <cstddef>
#include <cstdint>

#include "RE/NetImmerse/NiTransform.h"

namespace RE
{
    class NiAVObject;
    class NiCollisionObject;
}

namespace rock::scene_writer_probe
{
    inline constexpr std::size_t kMaxTrackedCollisionObjects = 8;

    struct HeldTargetRegistration
    {
        const RE::NiCollisionObject* collisionObjects[kMaxTrackedCollisionObjects] = {};
        std::uint32_t collisionObjectCount = 0;
        // The player room node. The hook reads its live world transform to
        // measure the mid-frame locomotion step at draw time.
        RE::NiAVObject* roomNode = nullptr;
        std::uint64_t traceId = 0;
    };

    // Byte-validated entry detour; idempotent and leaves the engine untouched
    // on prefix mismatch. ROCK requires this production boundary at load.
    bool install();
    bool isInstalled();
    // Game thread only. Releases reader-retired strong owners after every
    // earlier hook reader has drained.
    void serviceGameThread();

    // Game thread only. Overwrites the hand's slot for the current grab.
    [[nodiscard]] bool registerHeldTarget(
        bool isLeft,
        const HeldTargetRegistration& registration);
    // Game thread only. Publishes hot-reloaded immutable hook configuration;
    // disabling scene sync also invalidates this hand's retained anchor.
    void refreshHeldPresentationConfig(bool isLeft);
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
     * physically). Stage records which published clock the writer consumed.
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
        /*
         * Room-node frame at anchor computation. FO4VR stick locomotion moves
         * the ROOM node mid-frame (measured 2026-08-19: roomVsProducer ~2.6gu
         * per frame while the character controller was still unmoved at draw
         * time). The hook reads the live room node and applies the rigid 2D
         * room delta - rotate about the room origin by the yaw delta, then
         * translate - so the anchor follows the same frame the camera
         * inherits. Walk is the pure-translation case; turn adds the yaw arc.
         */
        float roomPositionGame[3] = { 0.0f, 0.0f, 0.0f };
        float roomYawRadians = 0.0f;
        bool roomValid = false;
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
