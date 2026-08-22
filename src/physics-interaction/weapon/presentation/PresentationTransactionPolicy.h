#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "physics-interaction/visual/PreFrikHandAuthorityPolicy.h"

#include "RE/NetImmerse/NiTransform.h"

namespace rock::presentation_transaction_policy
{
    /*
     * The weapon and its attached hands must move together, but they cannot
     * be written together: the weapon is written on the game thread now, and
     * the hands are only solved by the next deferred hFRIK skeleton pass. A
     * write that assumes both landed is the partial-commit defect.
     *
     * This is the state machine for treating the pair as one transaction that
     * spans two frames:
     *
     *   frame N   : Idle -> IntentReady -> PhysicsProposalReady
     *                    -> HandTargetsStaged      (claims published, weapon NOT written)
     *   frame N+1 : -> FrikConsumed                (the skeleton pass ran)
     *               -> ReadbackValidated           (the wrists actually arrived)
     *               -> Committed                   (the weapon is written once)
     *
     * Any state can abort. An abort is fail-closed: the weapon keeps its
     * collision-free intent, which errs toward visual penetration rather than
     * toward a weapon that has separated from the hands holding it.
     *
     * The machine is pure and fixed-storage. It owns no engine objects and
     * performs no writes; the caller performs every write and reports the
     * result back here.
     */

    inline constexpr float kReadbackTranslationToleranceGameUnits = 1.0f;
    inline constexpr float kReadbackRotationToleranceDegrees = 3.0f;

    enum class Stage : std::uint8_t
    {
        Idle = 0,
        IntentReady,
        PhysicsProposalReady,
        HandTargetsStaged,
        FrikConsumed,
        ReadbackValidated,
        Committed,
        Aborted,
    };

    enum class AbortReason : std::uint8_t
    {
        None = 0,
        // A caller advanced a stage that is not the current one.
        StageOutOfOrder,
        // The world, skeleton, provider, weapon generation, or attached-hand
        // set changed between staging and commit.
        IdentityChanged,
        // The physics snapshot did not describe the current body.
        ProposalRejected,
        // A required hand target could not be computed.
        HandTargetUnavailable,
        // At least one required hand claim was refused by the solver.
        HandPublishFailed,
        // The next skeleton pass did not follow the staging frame.
        SchedulerDiscontinuity,
        // Another owner took a required hand between staging and readback.
        WinnerChanged,
        // The solved wrist did not arrive at the staged target. This is the
        // silent tracked-hand fallback, which no publish result reports.
        ReadbackMismatch,
        // The staged group did not survive to the readback: the pre-hFRIK
        // rebase failed or a required claim was released in between.
        TransportFailed,
        // The final weapon write itself failed.
        WeaponCommitFailed,
        // The skeleton or the weapon went away mid-transaction.
        SubjectLost,
        Count,
    };

    inline constexpr std::size_t kAbortReasonCount =
        static_cast<std::size_t>(AbortReason::Count);

    enum class HandMask : std::uint8_t
    {
        None = 0,
        Left = 1 << 0,
        Right = 1 << 1,
        Both = Left | Right,
    };

    [[nodiscard]] inline constexpr std::uint8_t handBit(
        const bool isLeft) noexcept
    {
        return isLeft ?
            static_cast<std::uint8_t>(HandMask::Left) :
            static_cast<std::uint8_t>(HandMask::Right);
    }

    [[nodiscard]] inline constexpr bool maskContains(
        const std::uint8_t mask,
        const bool isLeft) noexcept
    {
        return (mask & handBit(isLeft)) != 0;
    }

    /*
     * Everything that must not change between staging and commit. Frame,
     * scheduler, and physics-solve counters are deliberately absent: the
     * transaction is designed to span frames, and each of those advances
     * every frame by construction, so including one makes the cross-frame
     * equality check fail on every readback. Frame continuity is enforced
     * separately through the stamps and markFrikConsumed.
     */
    struct TransactionIdentity
    {
        std::uint64_t weaponGenerationKey = 0;
        std::uint64_t worldGeneration = 0;
        std::uint64_t skeletonGeneration = 0;
        std::uint64_t providerGeneration = 0;
        std::uint8_t attachedHandMask = 0;

        [[nodiscard]] friend constexpr bool operator==(
            const TransactionIdentity&,
            const TransactionIdentity&) = default;
    };

    struct FrameStamp
    {
        std::uint64_t frameIndex = 0;
        std::uint64_t schedulerSequence = 0;
    };

    struct Transaction
    {
        Stage stage = Stage::Idle;
        AbortReason abortReason = AbortReason::None;
        TransactionIdentity identity{};
        FrameStamp openedAt{};
        FrameStamp stagedAt{};
        std::uint8_t requiredHandMask = 0;
        std::uint8_t stagedHandMask = 0;
        // The corrected weapon pose as computed in the staging frame, and the
        // driver it was captured against so the next frame can rebase it.
        RE::NiTransform stagedWeaponWorld{};
        RE::NiTransform stagedDriverWorld{};
        std::array<RE::NiTransform, 2> stagedHandWorld{};
        bool stagedDriverValid = false;
        // A weapon with no attached hands has no group to keep together, so
        // it commits immediately instead of deferring.
        bool weaponDetached = false;
    };

    [[nodiscard]] inline constexpr bool isTerminal(const Stage stage) noexcept
    {
        return stage == Stage::Committed || stage == Stage::Aborted;
    }

    [[nodiscard]] inline constexpr bool isOpen(const Stage stage) noexcept
    {
        return stage != Stage::Idle && !isTerminal(stage);
    }

    inline void reset(Transaction& transaction) noexcept
    {
        transaction = {};
    }

    inline void abort(
        Transaction& transaction,
        const AbortReason reason) noexcept
    {
        transaction.stage = Stage::Aborted;
        transaction.abortReason =
            reason == AbortReason::None ? AbortReason::StageOutOfOrder : reason;
    }

    // Opens a transaction for this frame's weapon intent. A transaction still
    // open from an earlier frame is abandoned, not silently continued.
    [[nodiscard]] inline bool begin(
        Transaction& transaction,
        const TransactionIdentity& identity,
        const FrameStamp& stamp) noexcept
    {
        if (isOpen(transaction.stage)) {
            abort(transaction, AbortReason::StageOutOfOrder);
            return false;
        }
        reset(transaction);
        transaction.stage = Stage::IntentReady;
        transaction.identity = identity;
        transaction.openedAt = stamp;
        transaction.requiredHandMask = identity.attachedHandMask;
        transaction.weaponDetached = identity.attachedHandMask == 0;
        return true;
    }

    [[nodiscard]] inline bool acceptPhysicsProposal(
        Transaction& transaction,
        const bool proposalAdmissible,
        const TransactionIdentity& observedIdentity) noexcept
    {
        if (transaction.stage != Stage::IntentReady) {
            abort(transaction, AbortReason::StageOutOfOrder);
            return false;
        }
        if (!(observedIdentity == transaction.identity)) {
            abort(transaction, AbortReason::IdentityChanged);
            return false;
        }
        if (!proposalAdmissible) {
            abort(transaction, AbortReason::ProposalRejected);
            return false;
        }
        transaction.stage = Stage::PhysicsProposalReady;
        return true;
    }

    /*
     * The group publish. The caller reports which required hands actually took
     * the claim; anything short of all of them aborts, and the caller must
     * then clear every tag it staged in this same phase. A partial group left
     * standing is the defect this machine exists to prevent.
     */
    [[nodiscard]] inline bool stageHandTargets(
        Transaction& transaction,
        const std::uint8_t targetsAvailableMask,
        const std::uint8_t publishedMask,
        const FrameStamp& stamp) noexcept
    {
        if (transaction.stage != Stage::PhysicsProposalReady) {
            abort(transaction, AbortReason::StageOutOfOrder);
            return false;
        }
        if ((targetsAvailableMask & transaction.requiredHandMask) !=
            transaction.requiredHandMask) {
            abort(transaction, AbortReason::HandTargetUnavailable);
            return false;
        }
        if ((publishedMask & transaction.requiredHandMask) !=
            transaction.requiredHandMask) {
            transaction.stagedHandMask = publishedMask;
            abort(transaction, AbortReason::HandPublishFailed);
            return false;
        }
        transaction.stagedHandMask = publishedMask;
        transaction.stagedAt = stamp;
        transaction.stage = Stage::HandTargetsStaged;
        return true;
    }

    /*
     * The deferred skeleton pass for the staging frame has now run. It must be
     * the immediately following scheduler interval: a gap means a frame was
     * dropped or a menu intervened, and the staged pose no longer describes
     * anything the solver saw.
     */
    [[nodiscard]] inline bool markFrikConsumed(
        Transaction& transaction,
        const FrameStamp& stamp) noexcept
    {
        if (transaction.stage != Stage::HandTargetsStaged) {
            abort(transaction, AbortReason::StageOutOfOrder);
            return false;
        }
        if (!prefrik_hand_authority_policy::isImmediateSuccessor(
                transaction.stagedAt.schedulerSequence,
                stamp.schedulerSequence)) {
            abort(transaction, AbortReason::SchedulerDiscontinuity);
            return false;
        }
        transaction.stage = Stage::FrikConsumed;
        return true;
    }

    [[nodiscard]] inline bool residualWithinPolicy(
        const RE::NiTransform& presentedWorld,
        const RE::NiTransform& expectedWorld,
        const float translationToleranceGameUnits =
            kReadbackTranslationToleranceGameUnits,
        const float rotationToleranceDegrees =
            kReadbackRotationToleranceDegrees) noexcept
    {
        return prefrik_hand_authority_policy::calibrationRelationsCoherent(
            presentedWorld,
            expectedWorld,
            translationToleranceGameUnits,
            rotationToleranceDegrees);
    }

    /*
     * The readback. The caller has compared each required hand's presented
     * wrist against the pose it staged, and re-read the winning owner. This is
     * the only place the silent tracked-hand fallback becomes observable.
     *
     * The winner comparison is the CALLER's, against the sequence stamped by
     * its LATEST publish of each claim, including the pre-solve rebase
     * republish. The registry stamps a fresh sequence on every publish, so a
     * transaction that held the stage-time sequence would read the pipeline's
     * own rebase as a theft and abort every commit. The policy therefore
     * receives per-hand verdicts, not raw sequences.
     */
    [[nodiscard]] inline bool validateReadback(
        Transaction& transaction,
        const TransactionIdentity& observedIdentity,
        const bool stagedGroupStillHeld,
        const std::uint8_t winnerUnchangedMask,
        const std::uint8_t residualWithinPolicyMask) noexcept
    {
        if (transaction.stage != Stage::FrikConsumed) {
            abort(transaction, AbortReason::StageOutOfOrder);
            return false;
        }
        if (!(observedIdentity == transaction.identity)) {
            abort(transaction, AbortReason::IdentityChanged);
            return false;
        }
        if (!stagedGroupStillHeld) {
            abort(transaction, AbortReason::TransportFailed);
            return false;
        }
        if ((winnerUnchangedMask & transaction.requiredHandMask) !=
            transaction.requiredHandMask) {
            abort(transaction, AbortReason::WinnerChanged);
            return false;
        }
        if ((residualWithinPolicyMask & transaction.requiredHandMask) !=
            transaction.requiredHandMask) {
            abort(transaction, AbortReason::ReadbackMismatch);
            return false;
        }
        transaction.stage = Stage::ReadbackValidated;
        return true;
    }

    [[nodiscard]] inline bool commit(
        Transaction& transaction,
        const bool weaponWritten) noexcept
    {
        if (transaction.stage != Stage::ReadbackValidated) {
            abort(transaction, AbortReason::StageOutOfOrder);
            return false;
        }
        if (!weaponWritten) {
            abort(transaction, AbortReason::WeaponCommitFailed);
            return false;
        }
        transaction.stage = Stage::Committed;
        transaction.abortReason = AbortReason::None;
        return true;
    }

    [[nodiscard]] inline constexpr const char* stageName(
        const Stage stage) noexcept
    {
        switch (stage) {
        case Stage::Idle:
            return "idle";
        case Stage::IntentReady:
            return "intent-ready";
        case Stage::PhysicsProposalReady:
            return "physics-proposal-ready";
        case Stage::HandTargetsStaged:
            return "hand-targets-staged";
        case Stage::FrikConsumed:
            return "frik-consumed";
        case Stage::ReadbackValidated:
            return "readback-validated";
        case Stage::Committed:
            return "committed";
        case Stage::Aborted:
            return "aborted";
        default:
            return "unknown";
        }
    }

    [[nodiscard]] inline constexpr const char* abortReasonName(
        const AbortReason reason) noexcept
    {
        switch (reason) {
        case AbortReason::None:
            return "none";
        case AbortReason::StageOutOfOrder:
            return "stage-out-of-order";
        case AbortReason::IdentityChanged:
            return "identity-changed";
        case AbortReason::ProposalRejected:
            return "proposal-rejected";
        case AbortReason::HandTargetUnavailable:
            return "hand-target-unavailable";
        case AbortReason::HandPublishFailed:
            return "hand-publish-failed";
        case AbortReason::SchedulerDiscontinuity:
            return "scheduler-discontinuity";
        case AbortReason::WinnerChanged:
            return "winner-changed";
        case AbortReason::ReadbackMismatch:
            return "readback-mismatch";
        case AbortReason::TransportFailed:
            return "transport-failed";
        case AbortReason::WeaponCommitFailed:
            return "weapon-commit-failed";
        case AbortReason::SubjectLost:
            return "subject-lost";
        default:
            return "unknown";
        }
    }
}
