#pragma once

#include <array>
#include <cstdint>

#include "physics-interaction/weapon/presentation/PresentationTransactionPolicy.h"

namespace rock
{
    /*
     * Owns the equipped-weapon presentation transaction across the two frames
     * it needs.
     *
     * The weapon and its attached hands are one visual object, but the hands
     * are only moved by the NEXT deferred hFRIK skeleton pass. Writing the
     * weapon in the same frame the hand claims are published assumes those
     * claims will be honoured, and the solver is free to silently fall back
     * to the tracked hand instead. That assumption is the split-presentation
     * defect.
     *
     * So the correction is pipelined. Two transactions are live at once:
     *
     *   _staging   the correction produced this frame. Its hand claims are
     *              published as a group; the weapon is NOT written.
     *   _inFlight  the correction staged last frame. The deferred solve has
     *              now run, so the presented wrists can be read back. If they
     *              arrived, the weapon commits once, composed onto THIS
     *              frame's collision-free intent.
     *
     * At the end of the frame _staging rotates into _inFlight.
     *
     * The cost is one frame of latency on the correction, about 11 ms at
     * 90 Hz, on a correction that is already one physics solve old. The gain
     * is that the weapon never moves to a pose its hands did not reach.
     *
     * Aborting is always fail-closed: no weapon write, staged claims
     * released, and the weapon keeps its collision-free intent. That errs
     * toward visual penetration and never toward a weapon that has separated
     * from the hands holding it.
     *
     * All calls run on the game update thread, in this order per frame:
     *   post-hFRIK hook : observeFrikConsumed, observeReadback   (_inFlight)
     *   phase 3         : beginFrame                             (_staging)
     *   phase 6         : isCommitApproved / observeWeaponCommit (_inFlight)
     *                     observePhysicsProposal, observeHandGroup (_staging)
     *                     rotate
     */
    class EquippedWeaponPresentationCoordinator
    {
    public:
        using Stage = presentation_transaction_policy::Stage;
        using AbortReason = presentation_transaction_policy::AbortReason;
        using TransactionIdentity =
            presentation_transaction_policy::TransactionIdentity;
        using FrameStamp = presentation_transaction_policy::FrameStamp;

        void reset()
        {
            presentation_transaction_policy::reset(_staging);
            presentation_transaction_policy::reset(_inFlight);
            _abortCounts = {};
            _deepestStage = Stage::Idle;
            _frameAbortReason = AbortReason::None;
        }

        void beginFrame(
            const TransactionIdentity& identity,
            const FrameStamp& stamp)
        {
            _deepestStage = Stage::Idle;
            _frameAbortReason = AbortReason::None;
            presentation_transaction_policy::reset(_staging);
            (void)presentation_transaction_policy::begin(
                _staging,
                identity,
                stamp);
            advanceDeepestStage(_staging.stage);
        }

        void observePhysicsProposal(
            const bool proposalAdmissible,
            const TransactionIdentity& observedIdentity)
        {
            if (!presentation_transaction_policy::acceptPhysicsProposal(
                    _staging,
                    proposalAdmissible,
                    observedIdentity)) {
                countAbort(_staging);
                return;
            }
            advanceDeepestStage(_staging.stage);
        }

        void observeHandGroup(
            const std::uint8_t targetsAvailableMask,
            const std::uint8_t publishedMask,
            const FrameStamp& stamp)
        {
            if (!presentation_transaction_policy::stageHandTargets(
                    _staging,
                    targetsAvailableMask,
                    publishedMask,
                    stamp)) {
                countAbort(_staging);
                return;
            }
            advanceDeepestStage(_staging.stage);
        }

        // The deferred solve for the in-flight transaction has now run.
        void observeFrikConsumed(const FrameStamp& stamp)
        {
            if (_inFlight.stage != Stage::HandTargetsStaged) {
                return;
            }
            if (!presentation_transaction_policy::markFrikConsumed(
                    _inFlight,
                    stamp)) {
                countAbort(_inFlight);
                return;
            }
            advanceDeepestStage(_inFlight.stage);
        }

        // The winner mask is the caller's per-hand comparison of the live
        // winning owner against the sequence its own LAST publish stamped,
        // rebase republish included. See validateReadback.
        void observeReadback(
            const TransactionIdentity& observedIdentity,
            const bool stagedGroupStillHeld,
            const std::uint8_t winnerUnchangedMask,
            const std::uint8_t residualWithinPolicyMask)
        {
            if (_inFlight.stage != Stage::FrikConsumed) {
                return;
            }
            if (!presentation_transaction_policy::validateReadback(
                    _inFlight,
                    observedIdentity,
                    stagedGroupStillHeld,
                    winnerUnchangedMask,
                    residualWithinPolicyMask)) {
                countAbort(_inFlight);
                return;
            }
            advanceDeepestStage(_inFlight.stage);
        }

        [[nodiscard]] bool isCommitApproved() const
        {
            return _inFlight.stage == Stage::ReadbackValidated;
        }

        // True while an in-flight transaction still holds staged hand claims
        // that the caller must release if it is not going to commit.
        [[nodiscard]] bool hasUncommittedInFlightClaims() const
        {
            return _inFlight.stage == Stage::HandTargetsStaged ||
                _inFlight.stage == Stage::FrikConsumed ||
                _inFlight.stage == Stage::Aborted;
        }

        void observeWeaponCommit(const bool weaponWritten)
        {
            if (!presentation_transaction_policy::commit(
                    _inFlight,
                    weaponWritten)) {
                countAbort(_inFlight);
                return;
            }
            advanceDeepestStage(_inFlight.stage);
        }

        /*
         * End of the frame's presentation work. This frame's staged group
         * becomes the next frame's in-flight transaction. Anything left open
         * in the outgoing in-flight slot never reached a commit, so it is
         * counted rather than dropped silently.
         */
        void rotate()
        {
            if (presentation_transaction_policy::isOpen(_inFlight.stage)) {
                presentation_transaction_policy::abort(
                    _inFlight,
                    AbortReason::SubjectLost);
                countAbort(_inFlight);
            }
            if (_staging.stage == Stage::HandTargetsStaged) {
                _inFlight = _staging;
            } else {
                presentation_transaction_policy::reset(_inFlight);
            }
            presentation_transaction_policy::reset(_staging);
        }

        [[nodiscard]] Stage stagingStage() const { return _staging.stage; }

        [[nodiscard]] Stage inFlightStage() const { return _inFlight.stage; }

        [[nodiscard]] Stage deepestStageReached() const
        {
            return _deepestStage;
        }

        [[nodiscard]] AbortReason abortReason() const
        {
            return _frameAbortReason;
        }

        [[nodiscard]] std::uint64_t abortCount(const AbortReason reason) const
        {
            const auto index = static_cast<std::size_t>(reason);
            return index < _abortCounts.size() ? _abortCounts[index] : 0u;
        }

    private:
        void countAbort(
            const presentation_transaction_policy::Transaction& transaction)
        {
            _frameAbortReason = transaction.abortReason;
            const auto index =
                static_cast<std::size_t>(transaction.abortReason);
            if (index < _abortCounts.size()) {
                ++_abortCounts[index];
            }
        }

        void advanceDeepestStage(const Stage stage)
        {
            if (stage != Stage::Aborted &&
                static_cast<std::uint8_t>(stage) >
                    static_cast<std::uint8_t>(_deepestStage)) {
                _deepestStage = stage;
            }
        }

        presentation_transaction_policy::Transaction _staging{};
        presentation_transaction_policy::Transaction _inFlight{};
        std::array<
            std::uint64_t,
            presentation_transaction_policy::kAbortReasonCount>
            _abortCounts{};
        Stage _deepestStage = Stage::Idle;
        AbortReason _frameAbortReason = AbortReason::None;
    };
}
