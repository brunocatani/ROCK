#include "physics-interaction/melee/PhysicalMeleePolicy.h"

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
            flag(provider::RockProviderExternalContactFlagV1::TargetAnatomyValid);
        contact.manifoldPointCount = 1;
        contact.selectedPointIndex = 0;
        contact.sourceWeaponFormId = 0x1234;
        contact.sourceSurfaceRegion = provider::RockProviderImpactSurfaceRegionV1::Edge;
        contact.sourceSurfaceDamageCoefficient = 1.0f;
        contact.targetBodyPartIndex = 1;
        contact.targetAnatomyFlags = anatomyFlag(provider::RockProviderTargetAnatomyFlagV1::BodyPartValid);
        contact.episodeFlags = episodeFlag(provider::RockProviderImpactEpisodeFlagV1::Started);
        contact.closingSpeedHavok = 8.0f;
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
               expect(std::fabs(decision.nativeDamageMultiplier - 1.0f) < 0.001f, "reference kinetic multiplier");
    }

    bool rejects_slow_contact()
    {
        auto contact = validContact();
        contact.closingSpeedHavok = 4.0f;
        const auto decision = physical_melee::evaluate(contact, {}, 0.02f);
        return expect(!decision.accepted, "slow contact rejected") &&
               expect(decision.reason == physical_melee::DecisionReason::SourceTooSlow, "slow reason");
    }

    bool rejects_continued_episode()
    {
        auto contact = validContact();
        contact.episodeFlags = episodeFlag(provider::RockProviderImpactEpisodeFlagV1::Continued);
        const auto decision = physical_melee::evaluate(contact, {}, 0.02f);
        return expect(!decision.accepted, "continued contact rejected") &&
               expect(decision.reason == physical_melee::DecisionReason::ContinuedEpisode, "continued reason");
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
}

int main()
{
    bool ok = true;
    ok &= accepts_exact_locational_weapon_contact();
    ok &= rejects_slow_contact();
    ok &= rejects_continued_episode();
    ok &= rejects_missing_or_ambiguous_anatomy();
    ok &= weapon_surface_controls_damage();
    ok &= rejects_invalid_scale();
    return ok ? 0 : 1;
}
