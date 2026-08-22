#pragma once

#include <array>
#include <cstdint>

#include "physics-interaction/weapon/presentation/PresentationTracePolicy.h"

#include "RE/NetImmerse/NiTransform.h"

namespace rock::presentation_trace
{
    /*
     * Game-thread collector for the equipped-weapon presentation group.
     *
     * Every call below runs on the game update thread, inside the ROCK frame
     * or the main-loop hook that brackets it. Nothing here may be called from
     * a physics callback or from the hFRIK recoil controller: those run under
     * their own thread and timing rules, and this collector neither locks nor
     * allocates.
     *
     * The invariant counters are always live. They are the release gate: a
     * non-zero counter is a presentation defect, not a diagnostic preference.
     * The per-frame record line is dev-INI gated and off by default.
     */

    using InvariantCounter = presentation_trace_policy::InvariantCounter;
    using RestoreGuardFailure = presentation_trace_policy::RestoreGuardFailure;
    using HandRecord = presentation_trace_policy::HandRecord;

    /*
     * Drops any half-built frame record. The invariant counters deliberately
     * survive: they measure the whole session, and a violation during one
     * equip cycle must still be visible after a cell load or a shutdown.
     */
    void reset();

    void beginFrame(
        std::uint64_t frameIndex,
        std::uint64_t schedulerSequence,
        std::uint64_t weaponGenerationKey);

    // The weapon generation is only known once the equipped-weapon phase has
    // refreshed it, which is after the frame record opens.
    void recordWeaponGeneration(std::uint64_t weaponGenerationKey);

    void recordIntentStability(
        const weapon_intent_stability_policy::Sample& sample);

    void recordDynamicWeaponFrame(
        bool proxyActive,
        bool contactActive,
        bool publishRequested,
        float contactRetentionSeconds,
        float translationCorrectionGameUnits,
        float rotationCorrectionDegrees,
        std::uint32_t otherBodyId,
        std::uint32_t otherLayer);

    // Only the collision fields of each HandRecord are read here. The winner
    // and publication-readiness fields are sampled at finalizeFrame, after
    // every writer of this frame has run.
    void recordCollisionGroupOutcome(
        bool weaponPublished,
        bool handGroupPublished,
        const std::array<HandRecord, 2>& hands);

    void recordRestoreGuard(RestoreGuardFailure reason);

    void recordRecoil(
        std::uint64_t acceptedSequence,
        std::uint64_t consumedSequence,
        bool applied);

    // The weapon world as the generated colliders and the muzzle saw it.
    void sampleWeaponAtColliderPublication(
        bool weaponWorldValid,
        const RE::NiTransform& weaponWorld);

    // Called after the provider Complete phase with the finally presented
    // weapon world. Evaluates the invariants and emits the frame line.
    void finalizeFrame(
        bool weaponWorldValid,
        const RE::NiTransform& weaponWorld);

    [[nodiscard]] std::uint64_t counterValue(InvariantCounter counter);
}
