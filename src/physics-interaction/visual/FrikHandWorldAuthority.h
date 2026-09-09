#pragma once

#include <cstdint>

#include "physics-interaction/hand/RenderedBoneTransportPolicy.h"
#include "physics-interaction/visual/DampenedDriverPredictionPolicy.h"
#include "physics-interaction/visual/HandWorldClaimRegistryPolicy.h"

#include "RE/NetImmerse/NiTransform.h"

/*
 * ROCK's hand world authority service for FRIK API v2.
 *
 * FRIK stores a setHandWorldTransform claim and solves the arm to it in its
 * next skeleton frame; the claim persists until cleared. ROCK's frame runs
 * after FRIK's, so a claim published from controller sample N is first shown
 * with controller sample N+1. This service owns the mirror of ROCK's claims
 * (HandWorldClaimRegistryPolicy) and runs the three per-frame steps that make
 * that model correct:
 *
 *   1. runPreFrikPass (outer main-loop hook, before FRIK): sample the two
 *      controller-driven weapon offset chains and move every claim by its
 *      driver's motion since it was published, republishing to FRIK.
 *   2. beginRockFrame (inner hook, after FRIK): remember per hand whether a
 *      ROCK claim was registered while FRIK solved, so the rendered hand bone
 *      is not mistaken for controller input this frame.
 *   3. resolveRawHands (ROCK frame, after the bone cache refresh): isolate the
 *      controller hand (TrackedHandIsolationPolicy) and detect FRIK's silent
 *      fallback to the tracked hand for an unreachable claim.
 *   4. tryPlanHandPresentation (end of ROCK's frame): carry each claimed
 *      hand's rendered chain by the change ROCK made to its target since
 *      FRIK consumed it, so the hand draws on this frame's seat.
 *
 * Publication is gated on the outer hook: until ROCK has verified that its
 * pre-FRIK pass runs, publish() returns false and no claim is stored, so a
 * claim can never be shown one frame stale without the rebase that fixes it.
 *
 * Single owner, game thread only. Explicitly reset on skeleton release and
 * session change.
 */
namespace rock::frik_hand_world_authority
{
    using RebaseDriver = hand_world_claim_registry_policy::RebaseDriver;

    enum class SchedulerState : std::uint8_t
    {
        // The outer hook has not been verified yet: claims are refused.
        Unverified,
        // The pre-FRIK pass runs every frame: claims are accepted.
        Verified,
        // The outer hook was refused or given up: claims stay refused.
        Refused,
    };

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
        // fallback observation, the composed kick is not a fallback.
        bool recoilKickThisFrame = false;
        // Skeleton ready and no scope/config state that suspends FRIK's solve.
        bool fallbackObservationAllowed = false;
    };

    // ---- Scheduler (ROCKMain) ----

    void setSchedulerState(SchedulerState state);
    [[nodiscard]] SchedulerState schedulerState();

    /*
     * Outer hook, before FRIK's frame. sequence is the never-zero scheduler
     * sequence of this pass.
     */
    void runPreFrikPass(std::uint64_t sequence);

    /*
     * Inner hook, after FRIK's frame and before ROCK's. Snapshots which hands
     * FRIK solved to a ROCK claim and clears the per-frame raw hand results.
     */
    void beginRockFrame(std::uint64_t sequence);

    // Current accepted controller driver. Never read rendered offset nodes as
    // a second physical-input source after the provider pass.
    [[nodiscard]] bool tryGetInputDriverWorld(bool isLeft, RE::NiTransform& outWorld);
    [[nodiscard]] std::uint8_t scopeInputRecoveryMask() noexcept;

    struct ScopeDampenTrace
    {
        std::uint64_t driverSequence = 0;
        std::uint64_t observedSequence = 0;
        std::uint64_t runtimeFrameObserved = 0;
        bool runtimeMenuSnapshot = false;
        bool menuUsed = false;
        bool enabled = false;
        float translationFactor = 0.0f;
        float rotationFactor = 0.0f;
        RE::NiPoint3 cameraNow{}, cameraPrevious{};
        bool cameraNowValid = false, cameraPreviousValid = false;
        std::array<hand_world_claim_registry_policy::DriverSample, 2> raw{}, driver{}, history{}, presented{};
        std::array<hand_world_claim_registry_policy::DriverSample, 2> nativeDriver{}, firstPersonInput{};
        std::array<bool, 2> inputIsolated{};
        std::uint8_t recoveryMask = 0;
        std::array<hand_world_claim_registry_policy::ConsumedTarget, 2> consumed{}, claimed{};
        std::array<std::uint64_t, 2> historySequence{};
        std::array<RE::NiPoint3, 2> historyCamera{};
        std::array<bool, 2> historyCameraValid{};
        std::array<dampened_driver_prediction_policy::PredictionMode, 2> predictionMode{};
        std::array<float, 2> predictionTranslationError{}, predictionRotationError{};
        std::array<bool, 2> predictionErrorValid{};
    };
    // Value-only observation; does not advance prediction or refresh inputs.
    [[nodiscard]] ScopeDampenTrace scopeDampenTrace() noexcept;
    [[nodiscard]] ScopeDampenTrace scopeDampenTraceBeforeFrik() noexcept;

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
     * The rigid delta that carries this frame's rendered hand chain (forearm,
     * hand, fingers) to the isolated controller hand. Inactive on claim-free
     * frames and whenever the isolation has no result. Computed once per
     * resolve so every consumer moves the chain by the same delta.
     */
    using HandChainTransport = rendered_bone_transport_policy::HandTransport;
    [[nodiscard]] bool tryGetHandChainTransport(bool isLeft, HandChainTransport& outTransport);
    [[nodiscard]] RE::NiTransform transportHandChainWorld(bool isLeft, const RE::NiTransform& renderedWorld);

    // ---- Presentation (PhysicsInteraction, end of ROCK's frame) ----

    /*
     * The rigid delta that carries this hand's rendered chain from the target
     * FRIK consumed this frame to the target ROCK holds now. False when no
     * claim was consumed, the target is unchanged, FRIK did not render the
     * consumed target, or the change exceeds a rigid carry (see
     * HandWorldClaimRegistryPolicy::planPresentation).
     */
    [[nodiscard]] bool tryPlanHandPresentation(bool isLeft, RE::NiTransform& outDelta);
    /*
     * The caller moved the hand by delta and re-solved the arm behind it
     * (elbowMoveGameUnits from FRIK's elbow), or failed to: keeps the
     * presented hand, the probe counters and the debug trace honest.
     */
    void recordHandPresentation(bool isLeft, const RE::NiTransform& delta, bool applied, float elbowMoveGameUnits, float reachDeficitGameUnits);
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
