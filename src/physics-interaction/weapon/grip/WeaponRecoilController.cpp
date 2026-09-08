#include "physics-interaction/weapon/TwoHandedGripInternal.h"

namespace rock
{
    bool TwoHandedGrip::hasVisualOnlySupportRecoilAssist() const noexcept
    {
        if (_session.state != TwoHandedState::Gripping ||
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
        return {
            .weaponNode = reinterpret_cast<std::uintptr_t>(_session.weaponNode),
            .weaponGeneration = _session.weaponGenerationKey,
            .equippedOwnership = _session.equippedWeaponOwnershipKey,
            .profile = weapon_recoil_policy::selectProfile(
                f4vr::isInPowerArmor(), hasVisualOnlySupportRecoilAssist()),
            .firingHandIsLeft = isFiringHandLeft(),
            .nativePrimaryIsLeft = nativePrimaryIsLeft,
            .fullTwoHanded = _session.state == TwoHandedState::Gripping &&
                _session.authorityMode == weapon_support_authority_policy::
                    WeaponSupportAuthorityMode::FullTwoHandedSolver,
        };
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
        const auto* const handedMode = f4vr::getIniSetting("bLeftHandedMode:VR");
        if (!handedMode || !f4vr::IsWeaponDrawn()) {
            return decline();
        }
        const auto context = self->recoilSampleIdentity(handedMode->GetBinary());
        const bool ownedCarry = self->isManualOwnershipActive() &&
            (context.fullTwoHanded || (self->usesLeftFiringCarry() &&
                self->_leftCarry.weaponNodeOwnershipBlockEngaged));
        if (!ownedCarry && context.profile == Profile::Native) {
            return decline();
        }

        RE::NiTransform controlled{};
        if (!weapon_recoil_authority_math::tryBuildControlledKick(
                sample->nativeKickLocal, gainsFor(context.profile), controlled)) {
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

        if (g_rockConfig.rockDebugGrabFrameLogging && nativeKick) {
            ROCK_LOG_SAMPLE_INFO(Weapon, 250,
                "Weapon recoil: profile={} firingHand={} nativePrimary={} delivery={} handMask={} sample={} generation={:016X} nativeT={:.4f} controlledT={:.4f} nativeR={:.4f} controlledR={:.4f}",
                name(context.profile), context.firingHandIsLeft ? "left" : "right",
                context.nativePrimaryIsLeft ? "left" : "right", ownedCarry ? "owned-weapon" : "native-hand",
                static_cast<std::uint32_t>(mask), state.ticket.sequence, context.weaponGeneration,
                hand_world_claim_registry_policy::translationDeltaGameUnits(sample->nativeKickLocal, identity),
                hand_world_claim_registry_policy::translationDeltaGameUnits(controlled, identity),
                hand_world_claim_registry_policy::rotationDeltaDegrees(sample->nativeKickLocal, identity),
                hand_world_claim_registry_policy::rotationDeltaDegrees(controlled, identity));
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
        _recoil.worldDelta = delta;
        _recoil.ticket.identity = identity;
        _recoil.ticket.valid = true;
        return true;
    }

    bool TwoHandedGrip::consumeOwnedWeaponRecoil(RE::NiTransform& outWorldDelta) noexcept
    {
        outWorldDelta = transform_math::makeIdentityTransform<RE::NiTransform>();
        const auto* const handedMode = f4vr::getIniSetting("bLeftHandedMode:VR");
        if (!handedMode || !isManualOwnershipActive()) {
            _recoil.ticket.invalidate();
            return false;
        }
        const auto context = recoilSampleIdentity(handedMode->GetBinary());
        if ((!context.fullTwoHanded && (!usesLeftFiringCarry() ||
                !_leftCarry.weaponNodeOwnershipBlockEngaged)) ||
            !_recoil.ticket.consume(context)) {
            return false;
        }
        outWorldDelta = _recoil.worldDelta;
        return true;
    }
}
