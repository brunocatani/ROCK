#include "physics-interaction/weapon/presentation/PresentationTraceRuntime.h"

#include <atomic>
#include <cstddef>

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/visual/PreFrikHandAuthorityPolicy.h"
#include "RockConfig.h"

namespace rock::presentation_trace
{
    namespace
    {
        namespace policy = presentation_trace_policy;

        policy::FrameRecord g_record{};

        /*
         * Relaxed is sufficient: only the game thread increments these, and
         * the values are read for reporting. The atomics exist so a future
         * reader outside the game thread cannot observe a torn count.
         */
        std::array<
            std::atomic<std::uint64_t>,
            policy::kInvariantCounterCount>
            g_counters{};

        // For counters that measure normal behavior, not violations. The
        // count stays queryable and visible in the per-frame trace row, but
        // no "invariant violated" warning is issued for expected events.
        void countOccurrence(const policy::InvariantCounter counter)
        {
            const auto index = static_cast<std::size_t>(counter);
            if (index >= g_counters.size()) {
                return;
            }
            g_counters[index].fetch_add(1, std::memory_order_relaxed);
        }

        void bumpCounter(
            const policy::InvariantCounter counter,
            const char* const detail)
        {
            const auto index = static_cast<std::size_t>(counter);
            if (index >= g_counters.size()) {
                return;
            }
            const std::uint64_t previous =
                g_counters[index].fetch_add(1, std::memory_order_relaxed);
            if (previous == 0) {
                ROCK_LOG_WARN(
                    Weapon,
                    "Presentation invariant violated for the first time: counter={} detail={} frame={} generation={:016X}",
                    policy::counterName(counter),
                    detail,
                    g_record.frameIndex,
                    g_record.weaponGenerationKey);
                return;
            }
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                5000,
                "Presentation invariant violated: counter={} count={} detail={} frame={} generation={:016X}",
                policy::counterName(counter),
                previous + 1,
                detail,
                g_record.frameIndex,
                g_record.weaponGenerationKey);
        }

        void sampleHandAuthorityState()
        {
            for (std::size_t index = 0; index < g_record.hands.size();
                 ++index) {
                const bool isLeft = index == 0u;
                const auto hand =
                    frik_visual_authority::handFromBool(isLeft);
                auto& record = g_record.hands[index];
                record.publicationReady =
                    frik_visual_authority::
                        isExternalHandWorldPublicationReady(hand);
                frik_visual_authority::HandWorldAuthoritySnapshot winner{};
                if (frik_visual_authority::
                        tryGetPublishedExternalHandWorldWinner(hand, winner)) {
                    record.winnerValid = true;
                    record.winnerRole = winner.role;
                    record.winnerPriority = winner.priority;
                    record.winnerSequence = winner.sequence;
                }
            }
        }

        void emitFrameLine()
        {
            const auto& left = g_record.hands[0];
            const auto& right = g_record.hands[1];
            ROCK_LOG_INFO(
                Weapon,
                "PresentationTrace: frame={} seq={} generation={:016X} transaction={}/{} dwc(proxy/contact/publish/weapon/hands)={}/{}/{}/{}/{} retention={:.3f}s correction=({:.2f}gu,{:.2f}deg) other(body/layer)={}/{} intent(valid/translation/rotation/stable)={}/{:.3f}/{:.3f}/{} warmUp={} recoil(accepted/consumed/applied/rejected)={}/{}/{}/{} recoilDelta=({:.3f}gu,{:.3f}deg) recoilBound=({:.3f}gu,{:.3f}deg) restoreGuard={} lateWriter=({:.3f}gu,{:.3f}deg) left(ready/req/target/applied/live/winner/priority/seq)={}/{}/{}/{}/{}/{}/{}/{} right(ready/req/target/applied/live/winner/priority/seq)={}/{}/{}/{}/{}/{}/{}/{}",
                g_record.frameIndex,
                g_record.schedulerSequence,
                g_record.weaponGenerationKey,
                presentation_transaction_policy::stageName(
                    g_record.transactionStage),
                presentation_transaction_policy::abortReasonName(
                    g_record.transactionAbortReason),
                g_record.dwcProxyActive,
                g_record.dwcContactActive,
                g_record.dwcPublishRequested,
                g_record.dwcWeaponPublished,
                g_record.dwcHandGroupPublished,
                g_record.dwcContactRetentionSeconds,
                g_record.dwcTranslationCorrectionGameUnits,
                g_record.dwcRotationCorrectionDegrees,
                g_record.dwcOtherBodyId,
                g_record.dwcOtherLayer,
                g_record.intentStability.valid,
                g_record.intentStability.translationDeltaGameUnits,
                g_record.intentStability.rotationDeltaDegrees,
                g_record.intentStability.stableFrameCount,
                weapon_presentation_warm_up_policy::blockReasonName(
                    g_record.warmUpBlockReason),
                g_record.recoilAcceptedSequence,
                g_record.recoilConsumedSequence,
                g_record.recoilApplied,
                g_record.recoilRejected,
                g_record.recoilAppliedTranslationGameUnits,
                g_record.recoilAppliedRotationDegrees,
                g_record.recoilBoundTranslationGameUnits,
                g_record.recoilBoundRotationDegrees,
                policy::restoreGuardFailureName(g_record.restoreGuardFailure),
                g_record.lateWriterTranslationGameUnits,
                g_record.lateWriterRotationDegrees,
                left.publicationReady,
                left.collisionRequested,
                left.collisionTargetValid,
                left.collisionApplied,
                left.collisionAuthorityLive,
                policy::roleName(left.winnerRole),
                left.winnerPriority,
                left.winnerSequence,
                right.publicationReady,
                right.collisionRequested,
                right.collisionTargetValid,
                right.collisionApplied,
                right.collisionAuthorityLive,
                policy::roleName(right.winnerRole),
                right.winnerPriority,
                right.winnerSequence);
        }
    }

    void reset()
    {
        g_record = {};
    }

    void beginFrame(
        const std::uint64_t frameIndex,
        const std::uint64_t schedulerSequence,
        const std::uint64_t weaponGenerationKey)
    {
        g_record = {};
        g_record.frameIndex = frameIndex;
        g_record.schedulerSequence = schedulerSequence;
        g_record.weaponGenerationKey = weaponGenerationKey;
        g_record.valid = true;
    }

    void recordWeaponGeneration(const std::uint64_t weaponGenerationKey)
    {
        g_record.weaponGenerationKey = weaponGenerationKey;
    }

    void recordIntentStability(
        const weapon_intent_stability_policy::Sample& sample)
    {
        g_record.intentStability = sample;
    }

    void recordDynamicWeaponFrame(
        const bool proxyActive,
        const bool contactActive,
        const bool publishRequested,
        const float contactRetentionSeconds,
        const float translationCorrectionGameUnits,
        const float rotationCorrectionDegrees,
        const std::uint32_t otherBodyId,
        const std::uint32_t otherLayer)
    {
        g_record.dwcProxyActive = proxyActive;
        g_record.dwcContactActive = contactActive;
        g_record.dwcPublishRequested = publishRequested;
        g_record.dwcContactRetentionSeconds = contactRetentionSeconds;
        g_record.dwcTranslationCorrectionGameUnits =
            translationCorrectionGameUnits;
        g_record.dwcRotationCorrectionDegrees = rotationCorrectionDegrees;
        g_record.dwcOtherBodyId = otherBodyId;
        g_record.dwcOtherLayer = otherLayer;
    }

    void recordCollisionGroupOutcome(
        const bool weaponPublished,
        const bool handGroupPublished,
        const std::array<HandRecord, 2>& hands)
    {
        g_record.dwcWeaponPublished = weaponPublished;
        g_record.dwcHandGroupPublished = handGroupPublished;
        for (std::size_t index = 0; index < g_record.hands.size(); ++index) {
            auto& target = g_record.hands[index];
            const auto& source = hands[index];
            target.collisionRequested = source.collisionRequested;
            target.collisionTargetValid = source.collisionTargetValid;
            target.collisionApplied = source.collisionApplied;
            target.collisionAuthorityLive = source.collisionAuthorityLive;
        }
    }

    void recordRestoreGuard(const RestoreGuardFailure reason)
    {
        g_record.restoreGuardEvaluated = true;
        g_record.restoreGuardFailure = reason;
        if (reason != RestoreGuardFailure::None) {
            bumpCounter(
                InvariantCounter::RestoreGuardFailure,
                policy::restoreGuardFailureName(reason));
        }
    }

    void recordPresentationWarmUp(
        const bool warmedUp,
        const weapon_presentation_warm_up_policy::BlockReason blockReason)
    {
        g_record.presentationWarmedUp = warmedUp;
        g_record.warmUpBlockReason = blockReason;
    }

    void recordTransactionOutcome(
        const presentation_transaction_policy::Stage stage,
        const presentation_transaction_policy::AbortReason abortReason)
    {
        g_record.transactionStage = stage;
        g_record.transactionAbortReason = abortReason;
        /*
         * A transaction abort is fail-closed and can be legitimate (a hand
         * released mid-pipeline), but a RECURRING abort means the deferred
         * commit is dead and the weapon silently never corrects — a failure
         * the invariant counters cannot see, because the deferred path never
         * writes the weapon at all. Keep it visible in the normal log at a
         * sampled rate so it can never be silent again.
         */
        if (abortReason !=
            presentation_transaction_policy::AbortReason::None) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                5000,
                "Presentation transaction aborted: stage={} reason={} frame={} generation={:016X}",
                presentation_transaction_policy::stageName(stage),
                presentation_transaction_policy::abortReasonName(abortReason),
                g_record.frameIndex,
                g_record.weaponGenerationKey);
        }
    }

    CollisionHandMasks currentCollisionHandMasks()
    {
        CollisionHandMasks masks{};
        for (std::size_t index = 0; index < g_record.hands.size(); ++index) {
            const bool isLeft = index == 0u;
            const auto bit =
                presentation_transaction_policy::handBit(isLeft);
            const auto& hand = g_record.hands[index];
            if (!hand.collisionRequested) {
                continue;
            }
            if (hand.collisionTargetValid) {
                masks.targetsAvailable |= bit;
            }
            if (hand.collisionApplied) {
                masks.published |= bit;
            }
        }
        return masks;
    }

    void recordRecoil(
        const std::uint64_t acceptedSequence,
        const std::uint64_t consumedSequence,
        const bool applied)
    {
        g_record.recoilAcceptedSequence = acceptedSequence;
        g_record.recoilConsumedSequence = consumedSequence;
        g_record.recoilApplied = applied;
        if (applied) {
            // An applied kick is normal behavior: count it, never warn.
            countOccurrence(InvariantCounter::RecoilDeltaApplied);
        }
    }

    void recordRecoilRejected(
        const float appliedTranslationGameUnits,
        const float appliedRotationDegrees,
        const float boundTranslationGameUnits,
        const float boundRotationDegrees)
    {
        g_record.recoilApplied = false;
        g_record.recoilRejected = true;
        g_record.recoilAppliedTranslationGameUnits =
            appliedTranslationGameUnits;
        g_record.recoilAppliedRotationDegrees = appliedRotationDegrees;
        g_record.recoilBoundTranslationGameUnits = boundTranslationGameUnits;
        g_record.recoilBoundRotationDegrees = boundRotationDegrees;
        bumpCounter(
            InvariantCounter::RecoilDeltaRejected,
            "delta-exceeds-published-kick");
    }

    void sampleWeaponAtColliderPublication(
        const bool weaponWorldValid,
        const RE::NiTransform& weaponWorld)
    {
        g_record.weaponSampledAtColliderPublication =
            weaponWorldValid &&
            prefrik_hand_authority_policy::isUsableTransform(weaponWorld);
        if (g_record.weaponSampledAtColliderPublication) {
            g_record.weaponWorldAtColliderPublication = weaponWorld;
        }
    }

    void finalizeFrame(
        const bool weaponWorldValid,
        const RE::NiTransform& weaponWorld)
    {
        if (!g_record.valid) {
            return;
        }

        if (g_record.weaponSampledAtColliderPublication &&
            weaponWorldValid &&
            prefrik_hand_authority_policy::isUsableTransform(weaponWorld)) {
            g_record.lateWriterTranslationGameUnits =
                prefrik_hand_authority_policy::translationDeltaGameUnits(
                    g_record.weaponWorldAtColliderPublication,
                    weaponWorld);
            g_record.lateWriterRotationDegrees =
                prefrik_hand_authority_policy::rotationDeltaDegrees(
                    g_record.weaponWorldAtColliderPublication,
                    weaponWorld);
        } else {
            g_record.weaponSampledAtColliderPublication = false;
        }

        sampleHandAuthorityState();

        if (policy::violatesGroupCommit(g_record)) {
            bumpCounter(
                InvariantCounter::WeaponWriteWithFailedHands,
                "weapon-committed-hands-failed");
        }
        if (policy::violatesGroupAtomicity(g_record)) {
            bumpCounter(
                InvariantCounter::PartialGroupStage,
                "one-hand-of-pair-published");
        }
        if (policy::violatesPublicationReadiness(g_record)) {
            bumpCounter(
                InvariantCounter::CorrectionWhilePublicationUnready,
                "hand-publication-channel-closed");
        }
        if (policy::violatesDownstreamOrdering(g_record)) {
            bumpCounter(
                InvariantCounter::LateWeaponWriterAfterColliderPublication,
                "weapon-moved-after-collider-sample");
        }

        if (g_rockConfig.rockDebugPresentationTraceLogging) {
            emitFrameLine();
        }
        g_record.valid = false;
    }

    std::uint64_t counterValue(const InvariantCounter counter)
    {
        const auto index = static_cast<std::size_t>(counter);
        if (index >= g_counters.size()) {
            return 0;
        }
        return g_counters[index].load(std::memory_order_relaxed);
    }
}
