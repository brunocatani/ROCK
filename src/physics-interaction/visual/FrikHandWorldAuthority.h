#pragma once

#include <cstdint>

#include "physics-interaction/hand/RenderedBoneTransportPolicy.h"
#include "physics-interaction/visual/HandWorldClaimRegistryPolicy.h"

#include "RE/NetImmerse/NiTransform.h"

/*
 * ROCK's hand world authority service for FRIK API v2.3.
 *
 * FRIK stores a setHandWorldTransform claim and solves the arm to it; the
 * claim persists until cleared. ROCK's frame runs inside FRIK's frame at the
 * AfterArmSolve phase, so a claim published there is re-solved by FRIK before
 * the frame continues, and a claim that was already held was solved by the
 * arm solve that just ran. This service owns the mirror of ROCK's claims
 * (HandWorldClaimRegistryPolicy) and the per-frame steps around them:
 *
 *   1. captureRenderedFrame (AfterWorldFinal): remember, per hand, the
 *      rendered flattened hand bone, its node and FRIK's solve result, so
 *      the next frame's readers know what was drawn and whether the claim
 *      was reachable.
 *   2. beginRockFrame (AfterArmSolve, before ROCK's update): read this
 *      frame's tracked inputs through the API (wand, weapon offset driver,
 *      first-person hand) and remember per hand whether a ROCK claim was
 *      registered while FRIK solved, so the rendered hand bone is not
 *      mistaken for controller input this frame.
 *   3. resolveRawHands (ROCK frame, after the bone cache refresh): isolate
 *      the controller hand (TrackedHandIsolationPolicy) from the API's
 *      first-person hand and the live body hand node.
 *
 * At AfterArmSolve the flattened bone array still holds the previous frame's
 * final render; the controller-space transport carries that rendered chain
 * to this frame's controller hand for every consumer that measures against
 * the controller.
 *
 * Single owner, game thread only. Explicitly reset on skeleton release and
 * session change.
 */
namespace rock::frik_hand_world_authority
{
    using RebaseDriver = hand_world_claim_registry_policy::RebaseDriver;

    struct RawHandSample
    {
        // The root flattened hand bone as rendered (refNode plus palm blend).
        RE::NiTransform flattenedHandWorld{};
        bool flattenedHandValid = false;
        // The body hand node FRIK's solver wrote (the flattened bone's refNode).
        RE::NiTransform bodyHandNodeWorld{};
        bool bodyHandNodeValid = false;
    };

    struct FrameHandSamples
    {
        RawHandSample right{};
        RawHandSample left{};
        // A native recoil kick reached FRIK's hand target this frame: skip
        // relation calibration, the composed kick is not a relation.
        bool recoilKickThisFrame = false;
        // Skeleton ready and no scope/config state that suspends FRIK's solve.
        bool fallbackObservationAllowed = false;
    };

    // ---- Scheduler (ROCKMain) ----

    /*
     * AfterWorldFinal: the frame's world transforms are final. samples carry
     * the rendered flattened hand bones and their nodes; the solve result is
     * read from FRIK here.
     */
    void captureRenderedFrame(const FrameHandSamples& samples);

    /*
     * AfterArmSolve, before ROCK's update. sequence is the never-zero frame
     * sequence of this pass. Reads this frame's tracked inputs.
     */
    void beginRockFrame(std::uint64_t sequence);

    // This frame's controller driver (FRIK's dampened weapon offset node).
    [[nodiscard]] bool tryGetInputDriverWorld(bool isLeft, RE::NiTransform& outWorld);

    /*
     * kScopeEnter / kScopeExit: the native first-person arm update rebuilds
     * its tree on the scope edge, so the controller-hand relation must not be
     * calibrated from, and the raw hand is reconstructed through, the
     * displaced first-person pose for the frames around the edge.
     */
    void noteScopeEdge();

    // ---- Publication (bridge) ----

    [[nodiscard]] bool publish(const char* tag, bool isLeft, const RE::NiTransform& worldTarget, int priority, RebaseDriver driver);
    [[nodiscard]] bool clear(const char* tag, bool isLeft);

    [[nodiscard]] bool hasActiveClaim(bool isLeft);
    [[nodiscard]] bool wasClaimConsumedThisFrame(bool isLeft);

    /*
     * The target FRIK solves this hand to (highest priority, newest publish),
     * optionally ignoring one tag.
     */
    [[nodiscard]] bool tryGetPublishedHandWorld(bool isLeft, RE::NiTransform& outWorld, const char* excludedTag = nullptr);

    // ---- Raw hand isolation (PhysicsInteraction, once per frame) ----

    void resolveRawHands(const FrameHandSamples& samples);
    [[nodiscard]] bool tryGetRawHandWorld(bool isLeft, RE::NiTransform& outWorld);
    [[nodiscard]] bool tryGetPresentedHandWorld(bool isLeft, RE::NiTransform& outWorld);
    [[nodiscard]] const char* rawHandSourceName(bool isLeft);
    [[nodiscard]] bool hasCalibratedRawHandFrame(bool isLeft);

    /*
     * The rigid delta that carries the rendered hand chain (forearm, hand,
     * fingers) of the last final frame to this frame's isolated controller
     * hand. Inactive on claim-free frames and whenever the isolation has no
     * result. Computed once per resolve so every consumer moves the chain by
     * the same delta.
     */
    using HandChainTransport = rendered_bone_transport_policy::HandTransport;
    [[nodiscard]] bool tryGetHandChainTransport(bool isLeft, HandChainTransport& outTransport);
    [[nodiscard]] RE::NiTransform transportHandChainWorld(bool isLeft, const RE::NiTransform& renderedWorld);

    // Last call of ROCK's frame: remembers this frame's targets for the next frame's trace.
    void endRockFrame();

    // ---- Lifecycle ----

    /*
     * Drop every claim ROCK still holds, clearing each one in FRIK first when
     * the API is available. Used when ROCK's physics owner is destroyed while
     * the skeleton may survive (session reset).
     */
    void clearAllClaims();

    /*
     * Forget every claim without talking to FRIK: FRIK releases its own
     * registry with the skeleton. Also resets the isolation relation.
     */
    void resetForSkeletonRelease();
}
