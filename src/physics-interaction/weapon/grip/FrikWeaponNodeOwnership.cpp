#include "physics-interaction/weapon/TwoHandedGripInternal.h"
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

    void TwoHandedGrip::finalizeFrikWeaponOwnershipForFrame()
    {
        _frikWeaponNode.framesSinceWrite = write_block_policy::advanceFramesSinceWrite(
            _frikWeaponNode.framesSinceWrite,
            _frikWeaponNode.writtenThisFrame);
        _frikWeaponNode.writtenThisFrame = false;
        _frikWeaponNode.framesSinceRecoilWrite = write_block_policy::advanceFramesSinceWrite(
            _frikWeaponNode.framesSinceRecoilWrite,
            _frikWeaponNode.recoilWrittenThisFrame);
        _frikWeaponNode.recoilWrittenThisFrame = false;
        // Right-firing PrimaryOnly is lifecycle/input ownership only and stays with FRIK.
        const bool wantsWrites = write_block_policy::shouldHoldWriteBlock(write_block_policy::OwnershipInput{
            .framesSinceWrite = _frikWeaponNode.framesSinceWrite,
            .framesSinceRecoilWrite = _frikWeaponNode.framesSinceRecoilWrite,
            .ownsWeaponTransform = ownsWeaponTransform(),
            .weaponReturnActive = isWeaponVisualReturnActive(),
            .leftCarryActive = usesLeftFiringCarry() && isManualOwnershipActive(),
            .oneHandRecoilActive = isOneHandRecoilEnvelopeActive(),
        });
        if (wantsWrites) {
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

    void TwoHandedGrip::presentFrikWeaponOffsetForRockFrame(RE::NiNode* weaponNode)
    {
        // A frame that ended without its restore must not leak into this one.
        restoreFrikWeaponOffsetAfterRockFrame();
        auto& presentation = _frikWeaponPresentation;
        if (!weaponNode ||
            !presentation_policy::shouldPresent(presentation.latch,
                presentation_policy::PresentInput{
                    .identity = frikWeaponIdentity(weaponNode),
                    .nodeVisible = f4vr::isNodeVisible(weaponNode),
                    .rockOwnsLivePose = _frikWeaponNode.writeBlockEngaged,
                })) {
            return;
        }
        /*
         * Not an authority write: FRIK keeps the node, the block stays
         * released, and FRIK's weapon pass writes this same local after the
         * restore. Local and subtree worlds change together, since ROCK's
         * readers take both.
         */
        presentation.presentedNode = weaponNode;
        presentation.reglueLocal = weaponNode->local;
        weaponNode->local = presentation.latch.local;
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
