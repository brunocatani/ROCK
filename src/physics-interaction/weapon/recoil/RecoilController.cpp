#include "physics-interaction/weapon/TwoHandedGripInternal.h"
#include "physics-interaction/weapon/DynamicWeaponCollision.h"

namespace rock
{
    bool TwoHandedGrip::hasVisualOnlySupportRecoilAssist() const noexcept
    {
        if (!g_rockConfig.rockImmersiveRecoil ||
            _session.state != TwoHandedState::Gripping ||
            !_session.weaponNode ||
            _session.weaponGenerationKey == 0 ||
            _session.equippedWeaponOwnershipKey == 0 ||
            (usesLeftFiringCarry() &&
                (!_leftCarry.weaponNodeOwnershipBlockEngaged ||
                    !isManualOwnershipActive()))) {
            return false;
        }

        const WeaponPartGrip& supportGrip = supportPartGrip();
        return supportGrip.weaponGenerationKey ==
                   _session.weaponGenerationKey &&
               supportGrip.gripSequence != 0 &&
               weapon_support_authority_policy::
                   shouldApplyVisualOnlySupportRecoilAssist(
                       _session.authorityMode,
                       supportGrip.active,
                       supportGrip.providerPartAuthority.active,
                       supportGrip.attachOnly);
    }

    weapon_recoil_policy::SampleIdentity TwoHandedGrip::recoilSampleIdentity(
        const bool nativePrimaryIsLeft) const noexcept
    {
        const bool fullTwoHanded = _session.state == TwoHandedState::Gripping &&
            _session.authorityMode == weapon_support_authority_policy::
                WeaponSupportAuthorityMode::FullTwoHandedSolver;
        auto identity = _recoil.equippedIdentity;
        const bool surfaceLatched = g_rockConfig.rockBipodMode && _surfaceSupportRuntime &&
            _surfaceSupportRuntime->hasLatchedSurfaceSupport(identity.weaponNode, identity.weaponGeneration);
        identity.profile = weapon_recoil_policy::selectProfile(
            f4vr::isInPowerArmor(), hasVisualOnlySupportRecoilAssist(), fullTwoHanded, surfaceLatched);
        identity.firingHandIsLeft = isFiringHandLeft();
        identity.nativePrimaryIsLeft = nativePrimaryIsLeft;
        identity.fullTwoHanded = fullTwoHanded;
        identity.oneHanded = _session.state != TwoHandedState::Gripping &&
            _session.state != TwoHandedState::PartCarry;
        switch (identity.family) {
        case weapon_recoil_policy::Family::Laser: identity.familyPercent = g_rockConfig.rockLaserRecoilPercent; break;
        case weapon_recoil_policy::Family::Pistol: identity.familyPercent = weapon_recoil_policy::selectHoldPercent(identity.oneHanded,
            g_rockConfig.rockPistolOneHandRecoilPercent, g_rockConfig.rockPistolTwoHandRecoilPercent); break;
        case weapon_recoil_policy::Family::Rifle: identity.familyPercent = weapon_recoil_policy::selectHoldPercent(identity.oneHanded,
            g_rockConfig.rockRifleOneHandRecoilPercent, g_rockConfig.rockRifleTwoHandRecoilPercent); break;
        case weapon_recoil_policy::Family::Shotgun: identity.familyPercent = weapon_recoil_policy::selectHoldPercent(identity.oneHanded,
            g_rockConfig.rockShotgunOneHandRecoilPercent, g_rockConfig.rockShotgunTwoHandRecoilPercent); break;
        case weapon_recoil_policy::Family::Heavy: identity.familyPercent = weapon_recoil_policy::selectHoldPercent(identity.oneHanded,
            g_rockConfig.rockHeavyOneHandRecoilPercent, g_rockConfig.rockHeavyTwoHandRecoilPercent); break;
        default: identity.familyPercent = weapon_recoil_policy::selectHoldPercent(identity.oneHanded,
            g_rockConfig.rockDefaultOneHandRecoilPercent, g_rockConfig.rockDefaultTwoHandRecoilPercent); break;
        }
        if (identity.profile == weapon_recoil_policy::Profile::Bipod ||
            (identity.profile == weapon_recoil_policy::Profile::PowerArmor &&
                identity.family != weapon_recoil_policy::Family::Laser)) {
            identity.familyPercent = 100.0f;
        }
        return identity;
    }

    bool FRIK_CALL TwoHandedGrip::controlWeaponHandRecoil(
        const frik::api::FRIKApiV2::RecoilSample* const sample,
        frik::api::FRIKApiV2::RecoilResponse* const outResponse,
        void* const userData) noexcept
    {
        using Api = frik::api::FRIKApiV2;
        using namespace weapon_recoil_policy;
        static_assert(static_cast<std::uint32_t>(HandMask::Primary) ==
            static_cast<std::uint32_t>(Api::RecoilHandMask::Primary));
        static_assert(static_cast<std::uint32_t>(HandMask::Offhand) ==
            static_cast<std::uint32_t>(Api::RecoilHandMask::Offhand));
        auto* const self = static_cast<TwoHandedGrip*>(userData);
        if (!self) {
            return false;
        }
        auto& state = self->_recoil;
        state.ticket.invalidate();
        ++state.ticket.sequence;
        if (!sample || sample->structSize < sizeof(Api::RecoilSample) || !outResponse) {
            return false;
        }

        const RE::NiTransform identity = transform_math::makeIdentityTransform<RE::NiTransform>();
        const bool nativeKick = isFiniteTransform(sample->nativeKickLocal) &&
            (hand_world_claim_registry_policy::translationDeltaGameUnits(sample->nativeKickLocal, identity) > 0.05f ||
                hand_world_claim_registry_policy::rotationDeltaDegrees(sample->nativeKickLocal, identity) > 0.1f);
        const auto decline = [&]() noexcept {
            if (nativeKick) {
                ++state.nativeKickSequence;
            }
            return false;
        };
        if (!g_rockConfig.rockImmersiveRecoil) {
            // Hand claims are separate from recoil registration. Release the
            // prior direct target before FRIK solves this disabled frame, then
            // decline so FRIK (or another controller) owns native hand recoil.
            self->clearOneHandRecoilClaim();
            return decline();
        }
        const auto* const handedMode = f4vr::getIniSetting("bLeftHandedMode:VR");
        if (!handedMode || !f4vr::IsWeaponDrawn()) {
            return decline();
        }
        const auto context = self->recoilSampleIdentity(handedMode->GetBinary());
        const bool rightCandidate = self->canUseRightOneHandRecoil();
        const bool directRight = rightCandidate && needsOneHandPresentation(
            weapon_recoil_authority_math::hasKick(sample->nativeKickLocal), state.rightNeedsNeutralFrame);
        if (!directRight) {
            self->clearOneHandRecoilClaim();
        }
        const bool ownedCarry = directRight || (self->isManualOwnershipActive() &&
            (context.fullTwoHanded ||
                (self->_session.state == TwoHandedState::Gripping && self->_firing.transferredPrimaryGrip.valid()) ||
                (self->usesLeftFiringCarry() &&
                self->_leftCarry.weaponNodeOwnershipBlockEngaged)));
        // Like armor, the laser profile also controls FRIK's native hand path
        // when ROCK does not currently own a weapon presentation target.
        if (!ownedCarry && context.family != Family::Laser &&
            (context.profile == Profile::OneHand || context.profile == Profile::FullTwoHand)) {
            return decline();
        }

        RE::NiTransform controlled{};
        if (!weapon_recoil_authority_math::tryBuildControlledKick(
                sample->nativeKickLocal, effectiveGains(context.family, context.profile, context.familyPercent), controlled)) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Weapon recoil: invalid native sample; controlled delivery declined");
            return decline();
        }
        if (ownedCarry && !self->captureOwnedWeaponRecoil(controlled, context)) {
            // Keep a valid neutral response: ROCK owns the final weapon and
            // hand seats, so native fallback is not an independent delivery.
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Weapon recoil: owned carry frame unavailable; holding recoil neutral");
            controlled = identity;
        }

        const auto mask = deliveryHand(ownedCarry, context.firingHandIsLeft, context.nativePrimaryIsLeft);
        if (nativeKick && mask != HandMask::None) {
            ++state.nativeKickSequence;
        }
        *outResponse = {};
        outResponse->structSize = sizeof(Api::RecoilResponse);
        outResponse->handMask = static_cast<std::uint32_t>(mask);
        outResponse->delivery = Api::RecoilDelivery::Direct;
        outResponse->controlledKickLocal = controlled;
        frik_hand_world_authority::noteNativeRecoilControlled(true);

        if (nativeKick) {
            self->traceRecoilSample(context, sample->nativeKickLocal, controlled, ownedCarry, static_cast<std::uint32_t>(mask));
        }
        return true;
    }

    bool TwoHandedGrip::captureOwnedWeaponRecoil(
        const RE::NiTransform& controlledKickLocal,
        const weapon_recoil_policy::SampleIdentity& identity) noexcept
    {
        const auto* const nodes = f4vr::getPlayerNodes();
        const auto* const kick = nodes ? nodes->primaryWeaponKickbackRecoilNode : nullptr;
        const auto* const parent = kick ? kick->parent : nullptr;
        if (!parent || !isInvertibleTransform(parent->world)) {
            return false;
        }
        const bool nativeOffhand = identity.firingHandIsLeft != identity.nativePrimaryIsLeft;
        if (nativeOffhand && (!nodes->primaryWandNode || !nodes->SecondaryWandNode ||
                !isInvertibleTransform(nodes->primaryWandNode->world) ||
                !isInvertibleTransform(nodes->SecondaryWandNode->world))) {
            return false;
        }
        const auto neutral = transform_math::makeIdentityTransform<RE::NiTransform>();
        const auto delta = weapon_recoil_authority_math::resolveWorldDelta(
            controlledKickLocal, parent->world,
            nativeOffhand ? nodes->primaryWandNode->world : neutral,
            nativeOffhand ? nodes->SecondaryWandNode->world : neutral, nativeOffhand);
        if (!isFiniteTransform(delta) || !isInvertibleTransform(delta)) {
            return false;
        }
        _recoil.controlledKickActive = weapon_recoil_authority_math::hasKick(controlledKickLocal);
        _recoil.worldDelta = delta;
        _recoil.ticket.identity = identity;
        _recoil.ticket.valid = true;
        return true;
    }

    bool TwoHandedGrip::consumeOwnedWeaponRecoil(RE::NiTransform& outWorldDelta) noexcept
    {
        outWorldDelta = transform_math::makeIdentityTransform<RE::NiTransform>();
        const auto* const handedMode = f4vr::getIniSetting("bLeftHandedMode:VR");
        if (!g_rockConfig.rockImmersiveRecoil || !handedMode || !f4vr::IsWeaponDrawn()) {
            _recoil.ticket.invalidate();
            return false;
        }
        const auto context = recoilSampleIdentity(handedMode->GetBinary());
        const bool directRight = canUseRightOneHandRecoil();
        const bool managedCarry = isManualOwnershipActive() &&
            (context.fullTwoHanded ||
                (_session.state == TwoHandedState::Gripping && _firing.transferredPrimaryGrip.valid()) ||
                (usesLeftFiringCarry() && _leftCarry.weaponNodeOwnershipBlockEngaged));
        if (!directRight && !managedCarry) {
            _recoil.ticket.invalidate();
            return false;
        }
        if (!_recoil.ticket.consume(context)) {
            return false;
        }
        outWorldDelta = _recoil.worldDelta;
        return true;
    }

    bool TwoHandedGrip::canUseRightOneHandRecoil() const noexcept
    {
        return g_rockConfig.rockImmersiveRecoil &&
            frik_hand_world_authority::hasCalibratedRawHandFrame(false) &&
            !usesLeftFiringCarry() && !isGripping() &&
            !_firing.rightHandHoldingObjectForPose &&
            !isWeaponVisualReturnActive() && !isHandVisualReturnActive(false) &&
            _recoil.rightBaseValid && _recoil.equippedIdentity.weaponNode != 0 &&
            _recoil.equippedIdentity.equippedOwnership != 0;
    }

    bool TwoHandedGrip::isOneHandRecoilEnvelopeActive() const noexcept
    {
        // Gated on the path itself, so a kick flag left over from a holster, a
        // grip or a disabled setting never holds FRIK's weapon-node write block.
        return canUseRightOneHandRecoil() &&
            (_recoil.controlledKickActive || _recoil.rightNeedsNeutralFrame);
    }

    void TwoHandedGrip::clearOneHandRecoilClaim()
    {
        if (!_recoil.rightHandClaimActive) {
            return;
        }
        if (frik_visual_authority::clearHandWorld(ONE_HAND_RECOIL_TAG,
                frik_visual_authority::Hand::Right)) {
            _recoil.rightHandClaimActive = false;
        } else {
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Weapon recoil: right firing-hand claim could not be cleared");
        }
    }

    void TwoHandedGrip::applyRightOneHandRecoil(RE::NiNode* weaponNode)
    {
        traceRecoilReadiness();
        if (!weaponNode || !canUseRightOneHandRecoil() || _recoil.rightHandClaimActive ||
            _recoil.equippedIdentity.weaponNode != reinterpret_cast<std::uintptr_t>(weaponNode)) {
            return;
        }
        RE::NiTransform delta{};
        if (!consumeOwnedWeaponRecoil(delta)) {
            return;
        }
        RE::NiTransform weaponTarget{};
        RE::NiTransform handTarget{};
        weapon_recoil_authority_math::applyOneHandKick(delta,
            _recoil.rightWeaponBase, _recoil.rightHandBase, weaponTarget, handTarget);
        if (!isFiniteTransform(weaponTarget) || !isUsableHandAuthorityTransform(handTarget)) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Weapon recoil: invalid right one-hand target; publication skipped");
            return;
        }

        // One bounded claim, renewed after native/authored alignment and before
        // collision. FRIK consumes it with no additional kick. The current
        // controller-derived base never contains the preceding frame's recoil.
        if (scope_safe_hand_frame_math::shouldPublishLockedHandVisualAuthority(_scope.menuOpenThisFrame)) {
            if (!frik_visual_authority::publishHandWorld(ONE_HAND_RECOIL_TAG,
                    frik_visual_authority::Hand::Right, handTarget, GRIP_HAND_POSE_PRIORITY)) {
                ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Weapon recoil: right firing-hand publication failed");
                return;
            }
            _recoil.rightHandClaimActive = true;
        }
        if (!applyWeaponVisualAuthority(weaponNode, weaponTarget,
                _recoil.equippedIdentity.weaponGeneration, true, true, _recoil.rightBaseSource)) {
            clearOneHandRecoilClaim();
            return;
        }
        noteFrikRecoilWeaponNodeWrite();
        traceRecoilPresentation("one-hand-right");
        _recoil.rightNeedsNeutralFrame = _recoil.controlledKickActive;
        recordPublishedHandWorld(false, handTarget);
        _lastSolvedWeaponTransform = weaponTarget;
        _hasSolvedWeaponTransform = true;
    }

}
