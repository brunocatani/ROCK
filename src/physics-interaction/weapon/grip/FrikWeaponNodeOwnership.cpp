#include "physics-interaction/weapon/TwoHandedGripInternal.h"
#include "physics-interaction/weapon/grip/WeaponNodeWriteBlockPolicy.h"

namespace rock
{
    namespace
    {
        namespace write_block_policy = weapon_node_write_block_policy;
    }

    void TwoHandedGrip::noteFrikWeaponNodeWrite()
    {
        _frikWeaponNode.writtenThisFrame = true;
        _frikWeaponNode.framesSinceWrite = 0;
        // Engage on the write, not at the end of the callback: FRIK's re-glue
        // and weapon pass must skip the node from this frame on.
        engageFrikWeaponNodeWriteBlock();
    }

    void TwoHandedGrip::noteFrikRecoilWeaponNodeWrite()
    {
        _frikWeaponNode.recoilWrittenThisFrame = true;
    }

    void TwoHandedGrip::engageFrikWeaponNodeWriteBlock()
    {
        if (_frikWeaponNode.writeBlockEngaged) {
            return;
        }
        if (blockOwnedWeaponNode(ownerTag(WEAPON_NODE_WRITE_TAG), true)) {
            _frikWeaponNode.writeBlockEngaged = true;
            ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: FRIK weapon-node writes blocked while ROCK owns the weapon transform");
        } else {
            ROCK_LOG_SAMPLE_WARN(Weapon,
                5000,
                "TwoHandedGrip: FRIK weapon-node write block unavailable; FRIK's weapon pass may overwrite ROCK's weapon transform");
        }
    }

    void TwoHandedGrip::finalizeFrikWeaponOwnershipForFrame(const std::uint64_t equippedWeaponOwnershipKey)
    {
        if (equippedWeaponOwnershipKey != _frikWeaponNode.ownershipKey) {
            /*
             * A tail bridges a missed write on the weapon it was written for.
             * Carried across an equip it keeps FRIK's weapon pass, so its
             * first offset write and its grip clearing, off the new weapon.
             */
            _frikWeaponNode.ownershipKey = equippedWeaponOwnershipKey;
            _frikWeaponNode.framesSinceWrite = write_block_policy::kNeverWritten;
            _frikWeaponNode.framesSinceRecoilWrite = write_block_policy::kNeverWritten;
        }
        _frikWeaponNode.framesSinceWrite = write_block_policy::advanceFramesSinceWrite(
            _frikWeaponNode.framesSinceWrite,
            _frikWeaponNode.writtenThisFrame);
        _frikWeaponNode.writtenThisFrame = false;
        _frikWeaponNode.framesSinceRecoilWrite = write_block_policy::advanceFramesSinceWrite(
            _frikWeaponNode.framesSinceRecoilWrite,
            _frikWeaponNode.recoilWrittenThisFrame);
        _frikWeaponNode.recoilWrittenThisFrame = false;
        // Right-firing PrimaryOnly is lifecycle/input ownership only and stays with FRIK.
        const write_block_policy::OwnershipInput input{
            .framesSinceWrite = _frikWeaponNode.framesSinceWrite,
            .framesSinceRecoilWrite = _frikWeaponNode.framesSinceRecoilWrite,
            .ownsWeaponTransform = ownsWeaponTransform(),
            .weaponReturnActive = isWeaponVisualReturnActive(),
            .leftCarryActive = usesManagedFiringCarry() && isManualOwnershipActive(),
            .oneHandRecoilActive = isOneHandRecoilEnvelopeActive(),
            .authoredPrimaryAlignmentActive = _firing.authoredHandWorldActive,
        };
        const auto reason = write_block_policy::holdReason(input);
        const bool heldByTail =
            reason == write_block_policy::HoldReason::WriteTail || reason == write_block_policy::HoldReason::RecoilTail;
        if (heldByTail && reason != _frikWeaponNode.lastHoldReason) {
            // The tail is a backstop: it says when it acts, so a hold that no
            // ownership predicate explains is visible in the log.
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: FRIK weapon-node write block held by {} tail (framesSinceWrite={} framesSinceRecoilWrite={})",
                write_block_policy::holdReasonName(reason),
                input.framesSinceWrite,
                input.framesSinceRecoilWrite);
        }
        _frikWeaponNode.lastHoldReason = reason;
        if (reason != write_block_policy::HoldReason::None) {
            engageFrikWeaponNodeWriteBlock();
        } else {
            releaseFrikWeaponNodeWriteBlock("no-weapon-authority");
        }
    }

    void TwoHandedGrip::releaseFrikWeaponNodeWriteBlock(const char* reason)
    {
        if (!_frikWeaponNode.writeBlockEngaged) {
            return;
        }
        (void)blockOwnedWeaponNode(ownerTag(WEAPON_NODE_WRITE_TAG), false);
        _frikWeaponNode.writeBlockEngaged = false;
        ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: FRIK weapon-node writes released reason={}", reason ? reason : "unknown");
    }

    void TwoHandedGrip::syncFrikOffHandGripReport()
    {
        const bool active =
            _session.state == TwoHandedState::Gripping || _session.state == TwoHandedState::PartCarry;
        const bool supportIsLeft = isSupportHandLeft();
        const std::uint64_t weaponKey = _session.equippedWeaponOwnershipKey;
        /*
         * Run after FRIK's weapon pass, which drops reports on a drawn weapon
         * change. Report on ROCK's state, support-hand and weapon-instance
         * edges only after that invalidation has finished. FRIK's
         * isOffHandGrippingWeapon is the union over every reporter and its
         * own detector, so it is not consulted here.
         */
        const bool changed = active != _frikWeaponNode.gripReported ||
            (active && (supportIsLeft != _frikWeaponNode.gripReportedSupportIsLeft ||
                           weaponKey != _frikWeaponNode.gripReportedWeaponKey));
        if (!changed) {
            return;
        }
        RE::NiTransform supportWorld{};
        const bool supportWorldValid = active && tryGetSolverHandTransform(supportIsLeft, supportWorld);
        if (!frik_visual_authority::setOffHandGripping(
                ownerTag(TWO_HANDED_GRIP_REPORT_TAG),
                active,
                frik_visual_authority::handFromBool(supportIsLeft),
                supportWorldValid ? &supportWorld : nullptr)) {
            if (active) {
                ROCK_LOG_SAMPLE_WARN(Weapon, 5000, "TwoHandedGrip: FRIK rejected the two-handed grip report");
            }
            _frikWeaponNode.gripReported = false;
            return;
        }
        _frikWeaponNode.gripReported = active;
        _frikWeaponNode.gripReportedSupportIsLeft = supportIsLeft;
        _frikWeaponNode.gripReportedWeaponKey = active ? weaponKey : 0;
        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: two-handed grip {} reported to FRIK support={}",
            active ? "engaged" : "released",
            supportIsLeft ? "left" : "right");
    }

    void TwoHandedGrip::resetFrikWeaponOwnership()
    {
        releaseFrikWeaponNodeWriteBlock("reset");
        if (_frikWeaponNode.gripReported) {
            (void)frik_visual_authority::setOffHandGripping(
                ownerTag(TWO_HANDED_GRIP_REPORT_TAG),
                false,
                frik_visual_authority::handFromBool(_frikWeaponNode.gripReportedSupportIsLeft),
                nullptr);
        }
        _frikWeaponNode = {};
    }
}
