#include "physics-interaction/weapon/TwoHandedGripInternal.h"
#include "physics-interaction/grab/FrikWeaponOffsetCache.h"
#include "physics-interaction/weapon/grip/FrikWeaponPresentationPolicy.h"
#include "physics-interaction/weapon/grip/WeaponNodeWriteBlockPolicy.h"

// FRIK API v2.3 weapon-node ownership and grip reporting: the write block ROCK
// holds while it owns the primary weapon node, the FRIK weapon offset ROCK
// presents on the node for its own frame while it does not, and the two-handed
// grip report FRIK keys its Pip-Boy guards and isOffHandGrippingWeapon on.

namespace rock
{
    namespace
    {
        namespace write_block_policy = weapon_node_write_block_policy;
        namespace presentation_policy = frik_weapon_presentation_policy;

        [[nodiscard]] presentation_policy::NodeIdentity frikWeaponIdentity(const RE::NiNode* weaponNode) noexcept
        {
            presentation_policy::NodeIdentity identity{};
            if (!weaponNode || !weaponNode->parent) {
                return identity;
            }
            identity.node = reinterpret_cast<std::uintptr_t>(weaponNode);
            identity.parent = reinterpret_cast<std::uintptr_t>(weaponNode->parent);
            if (!weaponNode->children.empty() && weaponNode->children[0]) {
                identity.modelRoot = reinterpret_cast<std::uintptr_t>(weaponNode->children[0].get());
            }
            identity.inPowerArmor = f4vr::isInPowerArmor();
            return identity;
        }
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
        if (frik_visual_authority::blockPrimaryWeaponNodeOwnership(WEAPON_NODE_WRITE_TAG, true)) {
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
            .leftCarryActive = usesLeftFiringCarry() && isManualOwnershipActive(),
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
        (void)frik_visual_authority::blockPrimaryWeaponNodeOwnership(WEAPON_NODE_WRITE_TAG, false);
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
                TWO_HANDED_GRIP_REPORT_TAG,
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

    void TwoHandedGrip::captureFrikWeaponOffsetLatch(RE::NiNode* weaponNode)
    {
        _frikWeaponPresentation.latch = presentation_policy::captureOffsetLatch(
            _frikWeaponPresentation.latch,
            presentation_policy::CaptureInput{
                .identity = frikWeaponIdentity(weaponNode),
                .local = weaponNode ? weaponNode->local : RE::NiTransform{},
                .nodeVisible = f4vr::isNodeVisible(weaponNode),
                .writeBlockHeld = _frikWeaponNode.writeBlockEngaged,
            });
    }

    bool TwoHandedGrip::ownsWeaponPoseForFrikPresentation() const
    {
        return ownsWeaponTransform() || isWeaponVisualReturnActive() ||
            (usesLeftFiringCarry() && isManualOwnershipActive());
    }

    void TwoHandedGrip::refreshSynthesizedFrikWeaponOffset(
        RE::NiNode* weaponNode,
        const RE::TESObjectWEAP* equippedWeapon,
        const presentation_policy::NodeIdentity& identity)
    {
        auto& synthesized = _frikWeaponPresentation.synthesized;
        if (!identity.valid()) {
            synthesized = {};
            return;
        }
        const std::uint64_t revision = frik_weapon_offset_cache::currentRevision();
        if (synthesized.identity == identity && synthesized.offsetTableRevision == revision) {
            return;
        }
        synthesized = presentation_policy::SynthesizedOffset{ .identity = identity, .offsetTableRevision = revision };
        const auto lookup = frik_weapon_offset_cache::findPrimaryWeaponOffset(equippedWeapon, weaponNode);
        /*
         * Only a stored offset stands in for FRIK's write. The table's
         * live-node fallback is the node's current local, which is the glue
         * (or ROCK's own pose) here; FRIK itself keeps the glue for a weapon
         * without a stored offset.
         */
        const bool stored = lookup.found &&
            (lookup.source == frik_weapon_offset_cache::OffsetSource::EmbeddedResource ||
                lookup.source == frik_weapon_offset_cache::OffsetSource::CustomFile);
        if (stored && presentation_policy::isFiniteTransform(lookup.offset)) {
            synthesized.local = lookup.offset;
            synthesized.valid = true;
        }
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: FRIK weapon offset for the equipped weapon {} (reason={} tableRevision={})",
            synthesized.valid ? "resolved from ROCK's offset table" : "not stored; the node reads at FRIK's glue until FRIK writes it",
            lookup.reason ? lookup.reason : "unknown",
            revision);
    }

    void TwoHandedGrip::presentFrikWeaponOffsetForRockFrame(RE::NiNode* weaponNode, const RE::TESObjectWEAP* equippedWeapon)
    {
        // A frame that ended without its restore must not leak into this one.
        restoreFrikWeaponOffsetAfterRockFrame();
        auto& presentation = _frikWeaponPresentation;
        // FRIK's stored offset is authored under the game's primary hand.
        bool gameLeftHanded = false;
        (void)frik_visual_authority::tryResolveHandIsLeft(frik_visual_authority::Hand::Primary, gameLeftHanded);
        RE::NiNode* const primaryHand = resolveFirstPersonHandNode(gameLeftHanded);
        const presentation_policy::PresentInput input{
            .identity = frikWeaponIdentity(weaponNode),
            .nodeVisible = f4vr::isNodeVisible(weaponNode),
            .rockOwnsPose = ownsWeaponPoseForFrikPresentation(),
            .underPrimaryHand = weaponNode && primaryHand && weaponNode->parent == primaryHand,
        };
        refreshSynthesizedFrikWeaponOffset(weaponNode, equippedWeapon, input.identity);
        const auto source = presentation_policy::selectPresentation(
            presentation.latch,
            presentation.synthesized,
            frik_weapon_offset_cache::currentRevision(),
            input);
        if (!weaponNode || source == presentation_policy::PresentSource::None) {
            /*
             * FRIK re-glued a visible weapon this frame and neither a latch
             * nor a stored offset matches it, so ROCK's frame reads the node
             * at the glue pose. Expected for a weapon without a stored
             * offset (FRIK keeps the glue for it too); a run of them on a
             * stored weapon means the identity churns or the node hangs
             * under a hand the offset is not authored for.
             */
            if (weaponNode && input.nodeVisible && input.identity.valid() && !input.rockOwnsPose) {
                if (_frikWeaponNode.glueFramesWithoutLatch != 0xFFFFFFFFu) {
                    ++_frikWeaponNode.glueFramesWithoutLatch;
                }
                if (_frikWeaponNode.glueFramesWithoutLatch > 1) {
                    ROCK_LOG_SAMPLE_DEBUG(Weapon,
                        2000,
                        "TwoHandedGrip: weapon read at glue pose: no presentable FRIK offset for {} frames (latchValid={} sameWeapon={} sameParent={} storedOffset={} underPrimaryHand={})",
                        _frikWeaponNode.glueFramesWithoutLatch,
                        presentation.latch.valid,
                        presentation.latch.valid && presentation.latch.identity.sameWeapon(input.identity),
                        presentation.latch.identity.parent == input.identity.parent,
                        presentation.synthesized.valid,
                        input.underPrimaryHand);
                }
            } else {
                _frikWeaponNode.glueFramesWithoutLatch = 0;
            }
            return;
        }
        _frikWeaponNode.glueFramesWithoutLatch = 0;
        /*
         * Not an authority write: FRIK keeps the node, the block stays
         * released, and FRIK's weapon pass writes this same local after the
         * restore. While the block is held for the authored alignment or a
         * recoil pose, this re-seeds FRIK's offset under the hand every frame
         * and those writers re-apply themselves on top, the order FRIK 0.78
         * gave them; the block then keeps the result through FRIK's pass.
         * Local and subtree worlds change together, since ROCK's readers take
         * both.
         */
        presentation.presentedNode = weaponNode;
        presentation.reglueLocal = weaponNode->local;
        weaponNode->local = source == presentation_policy::PresentSource::CapturedLatch ?
            presentation.latch.local :
            presentation.synthesized.local;
        f4vr::updateTransformsDown(weaponNode, true);
    }

    void TwoHandedGrip::restoreFrikWeaponOffsetAfterRockFrame()
    {
        auto& presentation = _frikWeaponPresentation;
        RE::NiNode* const node = presentation.presentedNode;
        presentation.presentedNode = nullptr;
        if (!presentation_policy::shouldRestore(node != nullptr, _frikWeaponNode.writeBlockEngaged)) {
            return;
        }
        // FRIK's weapon pass reads the node's local and world before rewriting it: both go back to what FRIK left.
        node->local = presentation.reglueLocal;
        f4vr::updateTransformsDown(node, true);
    }

    void TwoHandedGrip::resetFrikWeaponOwnership()
    {
        releaseFrikWeaponNodeWriteBlock("reset");
        if (_frikWeaponNode.gripReported) {
            (void)frik_visual_authority::setOffHandGripping(
                TWO_HANDED_GRIP_REPORT_TAG,
                false,
                frik_visual_authority::handFromBool(_frikWeaponNode.gripReportedSupportIsLeft),
                nullptr);
        }
        _frikWeaponNode = {};
    }
}
