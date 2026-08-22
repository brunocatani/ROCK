#include "physics-interaction/weapon/presentation/EquippedWeaponPresentationCoordinator.h"
#include "physics-interaction/weapon/presentation/PresentationTransactionPolicy.h"

#include <cstdio>

namespace
{
    namespace policy = rock::presentation_transaction_policy;
    using Stage = policy::Stage;
    using AbortReason = policy::AbortReason;

    bool expectTrue(const char* label, const bool actual)
    {
        if (actual) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, const bool actual)
    {
        if (!actual) {
            return true;
        }
        std::printf("%s expected false\n", label);
        return false;
    }

    bool expectStage(const char* label, const Stage actual, const Stage expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected stage %s got %s\n", label, policy::stageName(expected), policy::stageName(actual));
        return false;
    }

    bool expectAbort(const char* label, const AbortReason actual, const AbortReason expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected abort %s got %s\n", label, policy::abortReasonName(expected), policy::abortReasonName(actual));
        return false;
    }

    policy::TransactionIdentity twoHandIdentity()
    {
        return policy::TransactionIdentity{
            .weaponGenerationKey = 0xAAAAu,
            .worldGeneration = 3u,
            .skeletonGeneration = 4u,
            .providerGeneration = 5u,
            .physicsSolveSequence = 900u,
            .attachedHandMask = static_cast<std::uint8_t>(policy::HandMask::Both),
        };
    }

    constexpr std::uint8_t kBothHands = static_cast<std::uint8_t>(policy::HandMask::Both);
    constexpr std::uint8_t kRightOnly = static_cast<std::uint8_t>(policy::HandMask::Right);

    // Drives one clean two-frame transaction end to end.
    bool testHappyPath()
    {
        bool ok = true;
        policy::Transaction transaction{};
        const auto identity = twoHandIdentity();

        ok &= expectTrue("begin", policy::begin(transaction, identity, { .frameIndex = 10, .schedulerSequence = 100 }));
        ok &= expectStage("after begin", transaction.stage, Stage::IntentReady);

        ok &= expectTrue("proposal", policy::acceptPhysicsProposal(transaction, true, identity));
        ok &= expectStage("after proposal", transaction.stage, Stage::PhysicsProposalReady);

        ok &= expectTrue("stage", policy::stageHandTargets(transaction, kBothHands, kBothHands, { .frameIndex = 10, .schedulerSequence = 100 }));
        ok &= expectStage("after stage", transaction.stage, Stage::HandTargetsStaged);

        // The next scheduling interval consumes the claims.
        ok &= expectTrue("consumed", policy::markFrikConsumed(transaction, { .frameIndex = 11, .schedulerSequence = 101 }));
        ok &= expectStage("after consume", transaction.stage, Stage::FrikConsumed);

        ok &= expectTrue("readback", policy::validateReadback(transaction, identity, kBothHands, transaction.stagedWinnerSequence));
        ok &= expectStage("after readback", transaction.stage, Stage::ReadbackValidated);

        ok &= expectTrue("commit", policy::commit(transaction, true));
        ok &= expectStage("after commit", transaction.stage, Stage::Committed);
        ok &= expectAbort("a committed transaction has no abort reason", transaction.abortReason, AbortReason::None);
        return ok;
    }

    // Every stage must refuse to be skipped. A caller that jumps ahead is a
    // programming error, and silently accepting it would commit a weapon pose
    // no hand ever validated.
    bool testStagesCannotBeSkipped()
    {
        bool ok = true;
        const auto identity = twoHandIdentity();

        {
            policy::Transaction transaction{};
            ok &= expectFalse("stage before begin", policy::stageHandTargets(transaction, kBothHands, kBothHands, {}));
            ok &= expectAbort("stage before begin aborts", transaction.abortReason, AbortReason::StageOutOfOrder);
        }
        {
            policy::Transaction transaction{};
            (void)policy::begin(transaction, identity, {});
            ok &= expectFalse("consume before stage", policy::markFrikConsumed(transaction, { .schedulerSequence = 1 }));
            ok &= expectAbort("consume before stage aborts", transaction.abortReason, AbortReason::StageOutOfOrder);
        }
        {
            policy::Transaction transaction{};
            (void)policy::begin(transaction, identity, {});
            ok &= expectFalse("commit before readback", policy::commit(transaction, true));
            ok &= expectAbort("commit before readback aborts", transaction.abortReason, AbortReason::StageOutOfOrder);
        }
        {
            policy::Transaction transaction{};
            (void)policy::begin(transaction, identity, {});
            ok &= expectFalse("readback before consume", policy::validateReadback(transaction, identity, kBothHands, {}));
            ok &= expectAbort("readback before consume aborts", transaction.abortReason, AbortReason::StageOutOfOrder);
        }
        return ok;
    }

    // The partial group. One hand takes the claim and the other does not, so
    // the whole transaction must fail rather than leave a split owner.
    bool testPartialGroupAborts()
    {
        bool ok = true;
        policy::Transaction transaction{};
        const auto identity = twoHandIdentity();
        (void)policy::begin(transaction, identity, {});
        (void)policy::acceptPhysicsProposal(transaction, true, identity);

        ok &= expectFalse("a partial publish does not stage", policy::stageHandTargets(transaction, kBothHands, kRightOnly, {}));
        ok &= expectAbort("a partial publish aborts", transaction.abortReason, AbortReason::HandPublishFailed);
        // The caller needs the published mask to know exactly what to roll back.
        ok &= expectTrue("the published mask is retained for rollback", transaction.stagedHandMask == kRightOnly);
        return ok;
    }

    bool testMissingHandTargetAborts()
    {
        bool ok = true;
        policy::Transaction transaction{};
        const auto identity = twoHandIdentity();
        (void)policy::begin(transaction, identity, {});
        (void)policy::acceptPhysicsProposal(transaction, true, identity);
        ok &= expectFalse("a missing target does not stage", policy::stageHandTargets(transaction, kRightOnly, kBothHands, {}));
        ok &= expectAbort("a missing target aborts", transaction.abortReason, AbortReason::HandTargetUnavailable);
        return ok;
    }

    bool testIdentityChangeAborts()
    {
        bool ok = true;
        const auto identity = twoHandIdentity();

        {
            // The weapon was re-equipped between the intent and the proposal.
            policy::Transaction transaction{};
            (void)policy::begin(transaction, identity, {});
            auto changed = identity;
            changed.weaponGenerationKey = 0xBBBBu;
            ok &= expectFalse("a new generation refuses the proposal", policy::acceptPhysicsProposal(transaction, true, changed));
            ok &= expectAbort("a new generation aborts", transaction.abortReason, AbortReason::IdentityChanged);
        }
        {
            // The off hand let go between staging and readback. The staged
            // pose describes a grip that no longer exists.
            policy::Transaction transaction{};
            (void)policy::begin(transaction, identity, {});
            (void)policy::acceptPhysicsProposal(transaction, true, identity);
            (void)policy::stageHandTargets(transaction, kBothHands, kBothHands, { .schedulerSequence = 100 });
            (void)policy::markFrikConsumed(transaction, { .schedulerSequence = 101 });
            auto changed = identity;
            changed.attachedHandMask = kRightOnly;
            ok &= expectFalse("a changed hand set refuses the readback", policy::validateReadback(transaction, changed, kBothHands, transaction.stagedWinnerSequence));
            ok &= expectAbort("a changed hand set aborts", transaction.abortReason, AbortReason::IdentityChanged);
        }
        return ok;
    }

    bool testSchedulerDiscontinuityAborts()
    {
        bool ok = true;
        policy::Transaction transaction{};
        const auto identity = twoHandIdentity();
        (void)policy::begin(transaction, identity, {});
        (void)policy::acceptPhysicsProposal(transaction, true, identity);
        (void)policy::stageHandTargets(transaction, kBothHands, kBothHands, { .schedulerSequence = 100 });

        // A menu or a dropped frame swallowed a scheduling interval.
        ok &= expectFalse("a skipped interval refuses the consume", policy::markFrikConsumed(transaction, { .schedulerSequence = 103 }));
        ok &= expectAbort("a skipped interval aborts", transaction.abortReason, AbortReason::SchedulerDiscontinuity);
        return ok;
    }

    /*
     * The readback is the only observation that catches the silent fallback:
     * the claim was accepted and still wins, but the wrist never arrived.
     */
    bool testReadbackCatchesSilentFallback()
    {
        bool ok = true;
        policy::Transaction transaction{};
        const auto identity = twoHandIdentity();
        (void)policy::begin(transaction, identity, {});
        (void)policy::acceptPhysicsProposal(transaction, true, identity);
        (void)policy::stageHandTargets(transaction, kBothHands, kBothHands, { .schedulerSequence = 100 });
        (void)policy::markFrikConsumed(transaction, { .schedulerSequence = 101 });

        ok &= expectFalse("one unreached wrist refuses the readback", policy::validateReadback(transaction, identity, kRightOnly, transaction.stagedWinnerSequence));
        ok &= expectAbort("one unreached wrist aborts", transaction.abortReason, AbortReason::ReadbackMismatch);
        return ok;
    }

    bool testWinnerChangeAborts()
    {
        bool ok = true;
        policy::Transaction transaction{};
        const auto identity = twoHandIdentity();
        (void)policy::begin(transaction, identity, {});
        (void)policy::acceptPhysicsProposal(transaction, true, identity);
        (void)policy::stageHandTargets(transaction, kBothHands, kBothHands, { .schedulerSequence = 100 });
        transaction.stagedWinnerSequence = { 40u, 41u };
        (void)policy::markFrikConsumed(transaction, { .schedulerSequence = 101 });

        // Someone published over the left hand after the group was staged.
        const std::array<std::uint64_t, 2> observed{ 55u, 41u };
        ok &= expectFalse("a stolen hand refuses the readback", policy::validateReadback(transaction, identity, kBothHands, observed));
        ok &= expectAbort("a stolen hand aborts", transaction.abortReason, AbortReason::WinnerChanged);
        return ok;
    }

    bool testFailedWeaponWriteAborts()
    {
        bool ok = true;
        policy::Transaction transaction{};
        const auto identity = twoHandIdentity();
        (void)policy::begin(transaction, identity, {});
        (void)policy::acceptPhysicsProposal(transaction, true, identity);
        (void)policy::stageHandTargets(transaction, kBothHands, kBothHands, { .schedulerSequence = 100 });
        (void)policy::markFrikConsumed(transaction, { .schedulerSequence = 101 });
        (void)policy::validateReadback(transaction, identity, kBothHands, transaction.stagedWinnerSequence);

        ok &= expectFalse("a refused weapon write does not commit", policy::commit(transaction, false));
        ok &= expectAbort("a refused weapon write aborts", transaction.abortReason, AbortReason::WeaponCommitFailed);
        return ok;
    }

    // A weapon with no attached hands has no group to hold together.
    bool testDetachedWeaponIsFlagged()
    {
        bool ok = true;
        policy::Transaction transaction{};
        auto identity = twoHandIdentity();
        identity.attachedHandMask = 0;
        ok &= expectTrue("begin with no hands", policy::begin(transaction, identity, {}));
        ok &= expectTrue("a handless weapon is flagged detached", transaction.weaponDetached);

        (void)policy::acceptPhysicsProposal(transaction, true, identity);
        // With nothing required, an empty publish is still a valid stage.
        ok &= expectTrue("an empty group stages", policy::stageHandTargets(transaction, 0, 0, {}));
        return ok;
    }

    bool testRejectedProposalAborts()
    {
        bool ok = true;
        policy::Transaction transaction{};
        const auto identity = twoHandIdentity();
        (void)policy::begin(transaction, identity, {});
        ok &= expectFalse("an inadmissible proposal is refused", policy::acceptPhysicsProposal(transaction, false, identity));
        ok &= expectAbort("an inadmissible proposal aborts", transaction.abortReason, AbortReason::ProposalRejected);
        return ok;
    }

    // The coordinator must recover from an abort: the next frame opens a fresh
    // transaction rather than staying stuck in the failed one.
    bool testCoordinatorRecoversAfterAbort()
    {
        bool ok = true;
        rock::EquippedWeaponPresentationCoordinator coordinator;
        const auto identity = twoHandIdentity();

        coordinator.beginFrame(identity, { .frameIndex = 1, .schedulerSequence = 10 });
        coordinator.observePhysicsProposal(true, identity);
        coordinator.observeHandGroup(kBothHands, kRightOnly, { .frameIndex = 1, .schedulerSequence = 10 });
        ok &= expectStage("a partial group aborts", coordinator.stage(), Stage::Aborted);
        ok &= expectTrue("the abort is counted", coordinator.abortCount(AbortReason::HandPublishFailed) == 1u);

        coordinator.beginFrame(identity, { .frameIndex = 2, .schedulerSequence = 11 });
        ok &= expectStage("the next frame opens cleanly", coordinator.stage(), Stage::IntentReady);
        ok &= expectTrue("recovery does not add an abort", coordinator.abortCount(AbortReason::StageOutOfOrder) == 0u);

        coordinator.observePhysicsProposal(true, identity);
        coordinator.observeHandGroup(kBothHands, kBothHands, { .frameIndex = 2, .schedulerSequence = 11 });
        coordinator.observeFrikConsumed({ .frameIndex = 3, .schedulerSequence = 12 });
        ok &= expectStage("the deferred solve is observed", coordinator.stage(), Stage::FrikConsumed);
        ok &= expectStage("the deepest stage is reported", coordinator.deepestStageReached(), Stage::FrikConsumed);

        // Shadow mode ends at the deferred solve. Retiring it is not an abort.
        coordinator.beginFrame(identity, { .frameIndex = 3, .schedulerSequence = 12 });
        ok &= expectTrue("retiring a consumed transaction is not an abort", coordinator.abortCount(AbortReason::StageOutOfOrder) == 0u);
        ok &= expectStage("the following frame opens cleanly", coordinator.stage(), Stage::IntentReady);
        return ok;
    }

    // A free-space frame offers no proposal, so the transaction simply stays
    // open at intent and records nothing.
    bool testFreeSpaceFrameIsQuiet()
    {
        bool ok = true;
        rock::EquippedWeaponPresentationCoordinator coordinator;
        const auto identity = twoHandIdentity();
        for (std::uint64_t frame = 1; frame <= 5; ++frame) {
            coordinator.beginFrame(identity, { .frameIndex = frame, .schedulerSequence = frame });
            coordinator.observeFrikConsumed({ .frameIndex = frame, .schedulerSequence = frame });
            ok &= expectStage("a quiet frame stays at intent", coordinator.stage(), Stage::IntentReady);
        }
        for (std::size_t reason = 0; reason < policy::kAbortReasonCount; ++reason) {
            ok &= expectTrue(
                "a quiet frame records no abort",
                coordinator.abortCount(static_cast<AbortReason>(reason)) == 0u);
        }
        return ok;
    }
}

int main()
{
    bool ok = true;
    ok &= testHappyPath();
    ok &= testStagesCannotBeSkipped();
    ok &= testPartialGroupAborts();
    ok &= testMissingHandTargetAborts();
    ok &= testIdentityChangeAborts();
    ok &= testSchedulerDiscontinuityAborts();
    ok &= testReadbackCatchesSilentFallback();
    ok &= testWinnerChangeAborts();
    ok &= testFailedWeaponWriteAborts();
    ok &= testDetachedWeaponIsFlagged();
    ok &= testRejectedProposalAborts();
    ok &= testCoordinatorRecoversAfterAbort();
    ok &= testFreeSpaceFrameIsQuiet();
    return ok ? 0 : 1;
}
