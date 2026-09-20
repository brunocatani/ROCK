#include "physics-interaction/weapon/TwoHandedGripInternal.h"

namespace rock
{
    void TwoHandedGrip::traceRecoilSample(const weapon_recoil_policy::SampleIdentity& context,
        const RE::NiTransform& nativeKick, const RE::NiTransform& controlledKick,
        const bool ownedCarry, const std::uint32_t handMask) const
    {
        if (!g_rockConfig.rockDebugGrabFrameLogging) {
            return;
        }
        const auto identity = transform_math::makeIdentityTransform<RE::NiTransform>();
        const auto gains = weapon_recoil_policy::effectiveGains(context.family, context.profile, context.familyPercent);
        ROCK_LOG_SAMPLE_INFO(Weapon, 250,
            "Weapon recoil: form={:08X} family={} resolved={} classSource={} keywords={:016X} profile={} hold={} percent={:.1f} gains=({:.3f},{:.3f}) firingHand={} nativePrimary={} delivery={} handMask={} sample={} generation={:016X} nativeT={:.4f} controlledT={:.4f} nativeR={:.4f} controlledR={:.4f}",
            context.formID, weapon_recoil_policy::name(context.family), _recoil.weaponEvidence.resolved,
            static_cast<int>(_recoil.weaponEvidence.source), _recoil.weaponEvidence.keywordFlags,
            weapon_recoil_policy::name(context.profile), context.oneHanded ? "one-hand" : "two-hand", context.familyPercent, gains.translation, gains.rotation,
            context.firingHandIsLeft ? "left" : "right", context.nativePrimaryIsLeft ? "left" : "right",
            ownedCarry ? "owned-weapon" : "native-hand", handMask, _recoil.ticket.sequence, context.weaponGeneration,
            hand_world_claim_registry_policy::translationDeltaGameUnits(nativeKick, identity),
            hand_world_claim_registry_policy::translationDeltaGameUnits(controlledKick, identity),
            hand_world_claim_registry_policy::rotationDeltaDegrees(nativeKick, identity),
            hand_world_claim_registry_policy::rotationDeltaDegrees(controlledKick, identity));
    }

    void TwoHandedGrip::traceRecoilReadiness() const
    {
        if (g_rockConfig.rockImmersiveRecoil && _recoil.rightBaseValid &&
            !frik_hand_world_authority::hasCalibratedRawHandFrame(false)) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                "Weapon recoil: right acquisition deferred; rawSource={} controller/body relation unavailable form={:08X} generation={:016X}",
                frik_hand_world_authority::rawHandSourceName(false), _recoil.equippedIdentity.formID,
                _recoil.equippedIdentity.weaponGeneration);
        }
    }

    void TwoHandedGrip::traceRecoilPresentation(const char* route) const
    {
        if (!g_rockConfig.rockDebugGrabFrameLogging || !_recoil.controlledKickActive) {
            return;
        }
        const auto& sample = _recoil.ticket.identity;
        ROCK_LOG_SAMPLE_INFO(Weapon, 250,
            "Weapon recoil published: route={} sample={} form={:08X} family={} profile={} hold={} percent={:.1f} rawSource={} generation={:016X}",
            route, _recoil.ticket.sequence, sample.formID, weapon_recoil_policy::name(sample.family),
            weapon_recoil_policy::name(sample.profile), sample.oneHanded ? "one-hand" : "two-hand", sample.familyPercent,
            frik_hand_world_authority::rawHandSourceName(sample.firingHandIsLeft), sample.weaponGeneration);
    }
}
