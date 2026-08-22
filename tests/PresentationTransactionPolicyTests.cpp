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

    using Coordinator = rock::EquippedWeaponPresentationCoordinator;

    // Runs the staging half of one frame: begin, proposal, group publish.
    void driveStagingFrame(
        Coordinator& coordinator,
        const policy::TransactionIdentity& identity,
        const std::uint64_t schedulerSequence,
        const std::uint8_t targetsAvailable,
        const std::uint8_t published)
    {
        coordinator.beginFrame(identity, { .frameIndex = schedulerSequence, .schedulerSequence = schedulerSequence });
        coordinator.observePhysicsProposal(true, identity);
        coordinator.observeHandGroup(targetsAvailable, published, { 7u, 8u }, { .frameIndex = schedulerSequence, .schedulerSequence = schedulerSequence });
    }

    // The whole point: the weapon commits one frame after its hand claims,
    // and only after the solved wrists prove the hands arrived.
    bool testPipelineCommitsOneFrameLater()
    {
        bool ok = true;
        Coordinator coordinator;
        const auto identity = twoHandIdentity();

        // Frame N stages the group. Nothing may commit yet.
        driveStagingFrame(coordinator, identity, 10, kBothHands, kBothHands);
        ok &= expectStage("the group is staged", coordinator.stagingStage(), Stage::HandTargetsStaged);
        ok &= expectFalse("nothing commits on the staging frame", coordinator.isCommitApproved());
        coordinator.rotate();
        ok &= expectStage("the staged group is now in flight", coordinator.inFlightStage(), Stage::HandTargetsStaged);

        // Frame N+1 reads back after the deferred solve.
        coordinator.observeFrikConsumed({ .frameIndex = 11, .schedulerSequence = 11 });
        coordinator.observeReadback(identity, kBothHands, { 7u, 8u });
        ok &= expectTrue("a clean readback approves the commit", coordinator.isCommitApproved());

        coordinator.beginFrame(identity, { .frameIndex = 11, .schedulerSequence = 11 });
        ok &= expectTrue("opening the next staging frame keeps the approval", coordinator.isCommitApproved());

        coordinator.observeWeaponCommit(true);
        ok &= expectStage("the weapon commits", coordinator.inFlightStage(), Stage::Committed);
        ok &= expectFalse("a committed transaction holds no claims", coordinator.hasUncommittedInFlightClaims());

        coordinator.rotate();
        ok &= expectStage("an unstaged frame leaves nothing in flight", coordinator.inFlightStage(), Stage::Idle);
        for (std::size_t reason = 0; reason < policy::kAbortReasonCount; ++reason) {
            ok &= expectTrue(
                "a clean pipeline records no abort",
                coordinator.abortCount(static_cast<AbortReason>(reason)) == 0u);
        }
        return ok;
    }

    /*
     * The silent fallback. The claim was accepted and still wins, but the
     * wrist never arrived. The commit must be refused and the caller told it
     * still holds claims to release.
     */
    bool testFallbackReadbackFailsClosed()
    {
        bool ok = true;
        Coordinator coordinator;
        const auto identity = twoHandIdentity();

        driveStagingFrame(coordinator, identity, 10, kBothHands, kBothHands);
        coordinator.rotate();
        coordinator.observeFrikConsumed({ .frameIndex = 11, .schedulerSequence = 11 });
        coordinator.observeReadback(identity, kRightOnly, { 7u, 8u });

        ok &= expectFalse("an unreached wrist refuses the commit", coordinator.isCommitApproved());
        ok &= expectTrue("the caller is told to release the claims", coordinator.hasUncommittedInFlightClaims());
        ok &= expectTrue("the mismatch is counted", coordinator.abortCount(AbortReason::ReadbackMismatch) == 1u);

        coordinator.beginFrame(identity, { .frameIndex = 11, .schedulerSequence = 11 });
        coordinator.rotate();
        ok &= expectStage("the failed transaction is retired", coordinator.inFlightStage(), Stage::Idle);
        return ok;
    }

    // A group that could not publish in full never reaches the pipeline, so
    // the following frame has nothing to commit.
    bool testPartialGroupNeverReachesTheCommit()
    {
        bool ok = true;
        Coordinator coordinator;
        const auto identity = twoHandIdentity();

        driveStagingFrame(coordinator, identity, 10, kBothHands, kRightOnly);
        ok &= expectStage("a partial group aborts", coordinator.stagingStage(), Stage::Aborted);
        coordinator.rotate();
        ok &= expectStage("a partial group is not put in flight", coordinator.inFlightStage(), Stage::Idle);

        coordinator.observeFrikConsumed({ .frameIndex = 11, .schedulerSequence = 11 });
        ok &= expectFalse("there is nothing to commit", coordinator.isCommitApproved());
        ok &= expectTrue("the partial publish is counted", coordinator.abortCount(AbortReason::HandPublishFailed) == 1u);
        return ok;
    }

    // A dropped scheduling interval means the staged pose describes something
    // the solver never saw.
    bool testSchedulerGapFailsClosedInThePipeline()
    {
        bool ok = true;
        Coordinator coordinator;
        const auto identity = twoHandIdentity();

        driveStagingFrame(coordinator, identity, 10, kBothHands, kBothHands);
        coordinator.rotate();
        coordinator.observeFrikConsumed({ .frameIndex = 13, .schedulerSequence = 13 });
        ok &= expectFalse("a dropped interval refuses the commit", coordinator.isCommitApproved());
        ok &= expectTrue("the gap is counted", coordinator.abortCount(AbortReason::SchedulerDiscontinuity) == 1u);
        ok &= expectTrue("the caller is told to release the claims", coordinator.hasUncommittedInFlightClaims());
        return ok;
    }

    // Contact ended before the deferred solve reported back. The claims are
    // still live, so the caller must be told to release them.
    bool testAbandonedInFlightIsCounted()
    {
        bool ok = true;
        Coordinator coordinator;
        const auto identity = twoHandIdentity();

        driveStagingFrame(coordinator, identity, 10, kBothHands, kBothHands);
        coordinator.rotate();
        // The next frame never observes the deferred solve at all.
        coordinator.beginFrame(identity, { .frameIndex = 11, .schedulerSequence = 11 });
        coordinator.rotate();
        ok &= expectTrue("an abandoned group is counted", coordinator.abortCount(AbortReason::SubjectLost) == 1u);
        ok &= expectStage("an abandoned group is retired", coordinator.inFlightStage(), Stage::Idle);
        return ok;
    }

    // A free-space frame offers no proposal, so nothing stages, nothing goes
    // in flight, and nothing is counted.
    bool testFreeSpaceFrameIsQuiet()
    {
        bool ok = true;
        Coordinator coordinator;
        const auto identity = twoHandIdentity();
        for (std::uint64_t frame = 1; frame <= 5; ++frame) {
            coordinator.beginFrame(identity, { .frameIndex = frame, .schedulerSequence = frame });
            coordinator.observeFrikConsumed({ .frameIndex = frame, .schedulerSequence = frame });
            ok &= expectStage("a quiet frame stays at intent", coordinator.stagingStage(), Stage::IntentReady);
            ok &= expectFalse("a quiet frame commits nothing", coordinator.isCommitApproved());
            coordinator.rotate();
        }
        for (std::size_t reason = 0; reason < policy::kAbortReasonCount; ++reason) {
            ok &= expectTrue(
                "a quiet frame records no abort",
                coordinator.abortCount(static_cast<AbortReason>(reason)) == 0u);
        }
        return ok;
    }

    // A single attached hand runs the same pipeline.
    bool testOneHandPipeline()
    {
        bool ok = true;
        Coordinator coordinator;
        auto identity = twoHandIdentity();
        identity.attachedHandMask = kRightOnly;

        driveStagingFrame(coordinator, identity, 10, kRightOnly, kRightOnly);
        coordinator.rotate();
        coordinator.observeFrikConsumed({ .frameIndex = 11, .schedulerSequence = 11 });
        // The unrequested left hand contributes nothing either way.
        coordinator.observeReadback(identity, kRightOnly, { 999u, 8u });
        ok &= expectTrue("a one-hand group commits", coordinator.isCommitApproved());
        coordinator.observeWeaponCommit(true);
        ok &= expectStage("the one-hand weapon commits", coordinator.inFlightStage(), Stage::Committed);
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
    ok &= testPipelineCommitsOneFrameLater();
    ok &= testFallbackReadbackFailsClosed();
    ok &= testPartialGroupNeverReachesTheCommit();
    ok &= testSchedulerGapFailsClosedInThePipeline();
    ok &= testAbandonedInFlightIsCounted();
    ok &= testFreeSpaceFrameIsQuiet();
    ok &= testOneHandPipeline();
    return ok ? 0 : 1;
}
