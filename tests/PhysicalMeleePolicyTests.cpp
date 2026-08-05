#include "physics-interaction/melee/PhysicalMeleePolicy.h"
#include "physics-interaction/melee/WeaponImpactProfile.h"
#include "physics-interaction/contact/ContactSignalPointPolicy.h"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

namespace
{
    using namespace rock;

    constexpr std::uint32_t flag(provider::RockProviderExternalContactFlagV1 value)
    {
        return static_cast<std::uint32_t>(value);
    }

    constexpr std::uint32_t anatomyFlag(provider::RockProviderTargetAnatomyFlagV1 value)
    {
        return static_cast<std::uint32_t>(value);
    }

    constexpr std::uint32_t episodeFlag(provider::RockProviderImpactEpisodeFlagV1 value)
    {
        return static_cast<std::uint32_t>(value);
    }

    provider::RockProviderExternalContactRecordV1 validContact()
    {
        provider::RockProviderExternalContactRecordV1 contact{};
        contact.impactId = 1;
        contact.sourceBodyId = 10;
        contact.targetExternalBodyId = 20;
        contact.sourceKind = provider::RockProviderExternalSourceKind::Weapon;
        contact.quality = provider::RockProviderExternalContactQuality::RawPoint;
        contact.flags =
            flag(provider::RockProviderExternalContactFlagV1::RawManifoldValid) |
            flag(provider::RockProviderExternalContactFlagV1::RelativeVelocityValid) |
            flag(provider::RockProviderExternalContactFlagV1::SourceSurfaceClassified) |
            flag(provider::RockProviderExternalContactFlagV1::TargetAnatomyValid) |
            flag(provider::RockProviderExternalContactFlagV1::CollisionAvailable);
        contact.manifoldPointCount = 1;
        contact.selectedPointIndex = 0;
        contact.sourceWeaponFormId = 0x1234;
        contact.sourceSurfaceRegion = provider::RockProviderImpactSurfaceRegionV1::Edge;
        contact.sourceSurfaceDamageCoefficient = 1.0f;
        contact.targetBodyPartIndex = 1;
        contact.targetAnatomyFlags = anatomyFlag(provider::RockProviderTargetAnatomyFlagV1::BodyPartValid);
        contact.episodeFlags = episodeFlag(provider::RockProviderImpactEpisodeFlagV1::Started);
        contact.closingSpeedHavok = 8.0f;
        contact.tangentSpeedHavok = 0.0f;
        return contact;
    }

    bool expect(bool condition, const char* name)
    {
        if (!condition) {
            std::cerr << "FAILED: " << name << '\n';
        }
        return condition;
    }

    bool accepts_exact_locational_weapon_contact()
    {
        const auto decision = physical_melee::evaluate(validContact(), {}, 0.02f);
        return expect(decision.accepted, "exact locational weapon contact accepted") &&
               expect(decision.reason == physical_melee::DecisionReason::Accepted, "accepted reason") &&
               expect(std::fabs(decision.closingSpeedGame - 400.0f) < 0.01f, "speed converted to game units") &&
               expect(std::fabs(decision.relativePointSpeedGame - 400.0f) < 0.01f, "relative point speed converted to game units") &&
               expect(std::fabs(decision.nativeDamageMultiplier - 1.0f) < 0.001f, "reference kinetic multiplier");
    }

    bool accepts_fast_tangential_edge_sweep()
    {
        auto contact = validContact();
        contact.closingSpeedHavok = 0.0f;
        contact.tangentSpeedHavok = 8.0f;

        const auto decision = physical_melee::evaluate(contact, physical_melee::Settings{}, 0.02f);
        return expect(decision.accepted, "fast tangential edge sweep accepted") &&
               expect(std::fabs(decision.relativePointSpeedGame - 400.0f) < 0.01f, "tangential point speed used") &&
               expect(std::fabs(decision.closingSpeedGame) < 0.01f, "closing component remains independently reported");
    }

    bool rejects_slow_contact()
    {
        auto contact = validContact();
        contact.closingSpeedHavok = 4.0f;
        const auto decision = physical_melee::evaluate(contact, {}, 0.02f);
        return expect(!decision.accepted, "slow contact rejected") &&
               expect(decision.reason == physical_melee::DecisionReason::SourceTooSlow, "slow reason");
    }

    bool episode_flags_do_not_preempt_candidate_arbitration()
    {
        auto contact = validContact();
        contact.episodeFlags = episodeFlag(provider::RockProviderImpactEpisodeFlagV1::Continued);
        const auto decision = physical_melee::evaluate(contact, {}, 0.02f);
        return expect(decision.accepted, "episode arbitration is outside pure eligibility");
    }

    bool rejects_missing_or_ambiguous_anatomy()
    {
        auto missing = validContact();
        missing.flags &= ~flag(provider::RockProviderExternalContactFlagV1::TargetAnatomyValid);
        const auto missingDecision = physical_melee::evaluate(missing, {}, 0.02f);

        auto ambiguous = validContact();
        ambiguous.targetAnatomyFlags |= anatomyFlag(provider::RockProviderTargetAnatomyFlagV1::BodyPartAmbiguous);
        const auto ambiguousDecision = physical_melee::evaluate(ambiguous, {}, 0.02f);
        return expect(missingDecision.reason == physical_melee::DecisionReason::MissingTargetAnatomy, "missing anatomy reason") &&
               expect(ambiguousDecision.reason == physical_melee::DecisionReason::AmbiguousTargetAnatomy, "ambiguous anatomy reason");
    }

    bool weapon_surface_controls_damage()
    {
        auto contact = validContact();
        contact.sourceSurfaceDamageCoefficient = 1.75f;
        const auto decision = physical_melee::evaluate(contact, {}, 0.02f);
        return expect(decision.accepted, "classified weapon surface accepted") &&
               expect(std::fabs(decision.nativeDamageMultiplier - 1.75f) < 0.001f, "surface coefficient applied");
    }

    bool rejects_invalid_scale()
    {
        const auto zero = physical_melee::evaluate(validContact(), {}, 0.0f);
        const auto nan = physical_melee::evaluate(
            validContact(),
            {},
            std::numeric_limits<float>::quiet_NaN());
        return expect(zero.reason == physical_melee::DecisionReason::InvalidScale, "zero scale rejected") &&
               expect(nan.reason == physical_melee::DecisionReason::InvalidScale, "nan scale rejected");
    }

    bool lifecycle_and_non_damaging_parts_fail_closed()
    {
        auto unavailable = validContact();
        unavailable.flags &= ~flag(provider::RockProviderExternalContactFlagV1::CollisionAvailable);
        auto suppressed = validContact();
        suppressed.flags |= flag(provider::RockProviderExternalContactFlagV1::TransitionSuppressed);
        auto accessory = validContact();
        accessory.sourceSurfaceDamageCoefficient = 0.0f;
        return expect(
                   physical_melee::evaluate(unavailable, {}, 0.02f).reason ==
                       physical_melee::DecisionReason::CollisionUnavailable,
                   "collision unavailable reason") &&
               expect(
                   physical_melee::evaluate(suppressed, {}, 0.02f).reason ==
                       physical_melee::DecisionReason::TransitionSuppressed,
                   "transition suppressed reason") &&
               expect(
                   physical_melee::evaluate(accessory, {}, 0.02f).reason ==
                       physical_melee::DecisionReason::NonDamagingSource,
                   "non-damaging source reason");
    }

    bool exact_weapon_witness_is_required()
    {
        const physical_melee::WeaponIdentityWitness expected{
            .bodyGenerationKey = 1,
            .identityKey = 2,
            .ownershipKey = 3,
            .instanceDataAddress = 4,
            .weaponFormId = 5,
            .collisionGeneration = 6,
            .equipIndex = 1,
        };
        auto current = expected;
        const bool exact = physical_melee::weaponWitnessMatches(expected, current);
        current.instanceDataAddress = 7;
        return expect(exact, "exact weapon witness accepted") &&
               expect(!physical_melee::weaponWitnessMatches(expected, current), "instance mismatch rejected");
    }

    bool impact_profiles_keep_accessories_non_damaging()
    {
        const auto accessory = physical_melee::classifyWeaponImpact(
            WeaponPartKind::Scope, WeaponSizeClass::Rifle, {});
        const auto bladeTip = physical_melee::classifyWeaponImpact(
            WeaponPartKind::Other,
            WeaponSizeClass::Melee,
            { .longitudinalCoordinate = 0.95f, .majorAxis = 2, .nearestAxis = 2, .closeFaceCount = 1 });
        return expect(!accessory.damaging && accessory.damageCoefficient == 0.0f,
                   "scope reports but cannot damage") &&
               expect(bladeTip.damaging && bladeTip.region == provider::RockProviderImpactSurfaceRegionV1::Tip &&
                       std::fabs(bladeTip.damageCoefficient - 1.25f) < 0.001f,
                   "melee tip profile");
    }

    bool contact_selection_matches_native_positive_impulse_semantics()
    {
        havok_runtime::ContactSignalPointSelectionInput input{};
        input.pointCount = 3;
        input.contactIndex = 1;
        input.contactNormalHavok[2] = 1.0f;
        input.contactPointsHavok[0][0] = 10.0f;
        input.contactPointsHavok[1][0] = 20.0f;
        input.contactPointsHavok[2][0] = 30.0f;
        input.contactImpulses[0] = -100.0f;
        input.contactImpulses[1] = 2.0f;
        input.contactImpulses[2] = 5.0f;
        havok_runtime::ContactSignalPointResult result{};
        const bool selected = havok_runtime::selectContactSignalPoint(input, result);
        input.contactImpulses[1] = -2.0f;
        input.contactImpulses[2] = 0.0f;
        havok_runtime::ContactSignalPointResult fallback{};
        const bool fellBack = havok_runtime::selectContactSignalPoint(input, fallback);
        return expect(selected && result.selectedPointIndex == 2, "strongest positive impulse selected") &&
               expect(std::fabs(result.contactPointWeightSum - 7.0f) < 0.001f,
                   "only positive impulses summed") &&
               expect(fellBack && fallback.selectedPointIndex == 1 &&
                       fallback.selectionSource == havok_runtime::ContactSignalPointSelectionSource::NativeContactIndex,
                   "native point used when no positive impulse exists");
    }

    bool strongest_eligible_callback_wins_deterministically()
    {
        const physical_melee::ImpactCandidateScore first{
            .nativeDamageMultiplier = 1.0f,
            .positiveImpulseSum = 8.0f,
            .surfaceConfidencePermille = 900,
            .sourceBodyId = 20,
            .descriptorIndex = 2,
        };
        auto stronger = first;
        stronger.nativeDamageMultiplier = 1.5f;
        auto tied = first;
        tied.sourceBodyId = 10;
        return expect(physical_melee::isBetterImpactCandidate(stronger, first),
                   "later stronger eligible callback wins") &&
               expect(physical_melee::isBetterImpactCandidate(tied, first),
                   "stable body id breaks exact score ties");
    }
}

int main()
{
    bool ok = true;
    ok &= accepts_exact_locational_weapon_contact();
    ok &= accepts_fast_tangential_edge_sweep();
    ok &= rejects_slow_contact();
    ok &= episode_flags_do_not_preempt_candidate_arbitration();
    ok &= rejects_missing_or_ambiguous_anatomy();
    ok &= weapon_surface_controls_damage();
    ok &= rejects_invalid_scale();
    ok &= lifecycle_and_non_damaging_parts_fail_closed();
    ok &= exact_weapon_witness_is_required();
    ok &= impact_profiles_keep_accessories_non_damaging();
    ok &= contact_selection_matches_native_positive_impulse_semantics();
    ok &= strongest_eligible_callback_wins_deterministically();
    return ok ? 0 : 1;
}
