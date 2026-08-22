#pragma once

#include <array>
#include <cstdint>

#include "physics-interaction/weapon/presentation/PresentationTransactionPolicy.h"

namespace rock
{
    /*
     * Owns the equipped-weapon presentation transaction for the frame.
     *
     * Scope note. This stage of the work runs the transaction in SHADOW mode:
     * the coordinator observes the existing writers and drives the state
     * machine, but performs no writes and changes no behavior. Its output is
     * the per-frame stage and abort reason, which measure how often the
     * current immediate-write path would have had to abort once the deferred
     * commit is switched on.
     *
     * Shadow mode deliberately stops at FrikConsumed. Validating the readback
     * requires the staged hand poses, and those only reach this object when it
     * owns the staging itself, which is the next change. Reporting a readback
     * it could not perform would be worse than reporting the honest stage.
     *
     * All calls run on the game update thread.
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
            presentation_transaction_policy::reset(_transaction);
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
            /*
             * Most frames retire a transaction that never needed to commit:
             * free space offers no correction, and shadow mode has no commit
             * after the deferred solve. Neither is an abort. Staged claims
             * with no reported solve are different, and are counted: the
             * previous frame published hand claims that nothing consumed.
             */
            if (_transaction.stage == Stage::HandTargetsStaged) {
                presentation_transaction_policy::abort(
                    _transaction,
                    AbortReason::SubjectLost);
                countAbort();
            }
            presentation_transaction_policy::reset(_transaction);
            (void)presentation_transaction_policy::begin(
                _transaction,
                identity,
                stamp);
            advanceDeepestStage();
        }

        void observePhysicsProposal(
            const bool proposalAdmissible,
            const TransactionIdentity& observedIdentity)
        {
            if (!presentation_transaction_policy::acceptPhysicsProposal(
                    _transaction,
                    proposalAdmissible,
                    observedIdentity)) {
                countAbort();
                return;
            }
            advanceDeepestStage();
        }

        void observeHandGroup(
            const std::uint8_t targetsAvailableMask,
            const std::uint8_t publishedMask,
            const FrameStamp& stamp)
        {
            if (!presentation_transaction_policy::stageHandTargets(
                    _transaction,
                    targetsAvailableMask,
                    publishedMask,
                    stamp)) {
                countAbort();
                return;
            }
            advanceDeepestStage();
        }

        void observeFrikConsumed(const FrameStamp& stamp)
        {
            if (_transaction.stage != Stage::HandTargetsStaged) {
                // Nothing was staged for the deferred pass to consume. That is
                // the ordinary case whenever the weapon is not in contact.
                return;
            }
            if (!presentation_transaction_policy::markFrikConsumed(
                    _transaction,
                    stamp)) {
                countAbort();
                return;
            }
            advanceDeepestStage();
        }

        [[nodiscard]] Stage stage() const { return _transaction.stage; }

        [[nodiscard]] Stage deepestStageReached() const
        {
            return _deepestStage;
        }

        // The reason this frame's transaction stopped, if it did.
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
        void countAbort()
        {
            _frameAbortReason = _transaction.abortReason;
            const auto index =
                static_cast<std::size_t>(_transaction.abortReason);
            if (index < _abortCounts.size()) {
                ++_abortCounts[index];
            }
        }

        void advanceDeepestStage()
        {
            if (static_cast<std::uint8_t>(_transaction.stage) >
                static_cast<std::uint8_t>(_deepestStage)) {
                _deepestStage = _transaction.stage;
            }
        }

        presentation_transaction_policy::Transaction _transaction{};
        std::array<
            std::uint64_t,
            presentation_transaction_policy::kAbortReasonCount>
            _abortCounts{};
        Stage _deepestStage = Stage::Idle;
        AbortReason _frameAbortReason = AbortReason::None;
    };
}
