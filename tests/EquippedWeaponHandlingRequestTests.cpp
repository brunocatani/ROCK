// Released provider contract: RockProviderEquippedWeaponHandlingRequestV1.
// An addon handling lease must keep the observable meaning of each flag and
// tuning field it publishes, and must not switch off ROCK-owned behavior it
// never requested (shoulder stash, toggle grab, authored-only support).
#include "physics-interaction/weapon/EquippedWeaponHandlingSettings.h"

#include <cmath>
#include <cstdint>
#include <cstdio>

namespace
{
    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, bool value)
    {
        return expectTrue(label, !value);
    }

    template <class T>
    bool expectEqual(const char* label, T actual, T expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected %llu got %llu\n", label, static_cast<unsigned long long>(expected), static_cast<unsigned long long>(actual));
        return false;
    }

    bool expectNear(const char* label, float actual, float expected, float tolerance = 0.0001f)
    {
        if (std::fabs(actual - expected) <= tolerance) {
            return true;
        }
        std::printf("%s expected %.6f got %.6f\n", label, expected, actual);
        return false;
    }
}

int main()
{
    using WeaponGrabMode = rock::equipped_weapon_toggle_grab_policy::Mode;
    bool ok = true;

    const rock::RockEquippedWeaponHandlingBaseline coreWeaponHandlingBaseline{
        .ambidextrousHandoffEnabled = true,
        .authoredOnlySupportGrabsEnabled = true,
        .weaponGrabMode = WeaponGrabMode::ToggleBoth,
        .equippedWeaponShoulderStashEnabled = true,
        .lastGripReleaseDropEnabled = false,
        .immersiveWeapon = {
            .firingGripDetachEnabled = true,
            .firingGripDetachPosePreservationEnabled = true,
            .firingGripReattachRadiusGameUnits = 4.0f,
            .firingGripHapticDurationSeconds = 0.11f,
            .firingGripAttachHapticIntensity = 0.81f,
            .firingGripDetachHapticIntensity = 0.31f,
        },
        .firingGripProximitySupportRadiusGameUnits = 7.0f,
        .leftFiringAimYawDegrees = 1.5f,
        .leftFiringAimPitchDegrees = -2.5f,
        .leftFiringAimOffsetXGameUnits = 0.5f,
        .leftFiringAimOffsetYGameUnits = -0.75f,
        .leftFiringAimOffsetZGameUnits = 1.25f,
    };
    const auto coreWeaponHandling = rock::makeEquippedWeaponHandlingSettings(
        coreWeaponHandlingBaseline,
        nullptr);

    auto stashOnlyBaseline = coreWeaponHandlingBaseline;
    stashOnlyBaseline.ambidextrousHandoffEnabled = false;
    auto fixedOnlyBaseline = stashOnlyBaseline;
    fixedOnlyBaseline.equippedWeaponShoulderStashEnabled = false;

    rock::provider::RockProviderEquippedWeaponHandlingRequestV1 externalWeaponHandling{};
    externalWeaponHandling.flags = static_cast<std::uint32_t>(
        rock::provider::RockProviderEquippedWeaponHandlingFlagV1::FiringGripOwnership);
    externalWeaponHandling.firingGripProximitySupportRadiusGameUnits = 8.0f;
    auto externalHandling = rock::makeEquippedWeaponHandlingSettings(
        coreWeaponHandlingBaseline,
        &externalWeaponHandling);
    ok &= expectTrue("an equipped-weapon request activates external authority",
        externalHandling.externalAuthorityActive);
    ok &= expectTrue("an addon lease cannot suppress ROCK shoulder stash",
        externalHandling.equippedWeaponShoulderStashEnabled);
    ok &= expectTrue("an addon lease cannot suppress ROCK toggle grab",
        externalHandling.weaponGrabMode == WeaponGrabMode::ToggleBoth);
    ok &= expectFalse("an addon lease cannot re-enable the last-grip drop",
        externalHandling.lastGripReleaseDropEnabled);
    ok &= expectTrue("an addon handling lease cannot suppress authored-only support",
        externalHandling.authoredOnlySupportGrabsEnabled);
    ok &= expectFalse("ROCK stash cannot add detach to a non-detach addon lease",
        externalHandling.primaryDetachEnabled);
    const auto integratedUnderNonDetachProvider =
        rock::resolveEquippedWeaponDetachDecision(
            externalHandling);
    ok &= expectEqual("non-detach provider cannot suppress integrated immersive detach",
        integratedUnderNonDetachProvider.authority,
        rock::immersive_weapon_policy::DetachAuthority::
            IntegratedImmersive);
    ok &= expectFalse("an active addon request may suppress ROCK ambidextrous handoff",
        externalHandling.ambidextrousHandoffEnabled);
    ok &= expectNear("external authority preserves ROCK's radius without an override",
        externalHandling.firingGripProximitySupportRadiusGameUnits,
        7.0f);
    externalWeaponHandling.flags |=
        static_cast<std::uint32_t>(
            rock::provider::RockProviderEquippedWeaponHandlingFlagV1::AmbidextrousHandoff) |
        static_cast<std::uint32_t>(
            rock::provider::RockProviderEquippedWeaponHandlingFlagV1::FiringGripProximitySupport);
    externalWeaponHandling.firingGripPromotionRadiusGameUnits = 9.0f;
    externalWeaponHandling.leftFiringAimYawDegrees = -4.0f;
    externalHandling = rock::makeEquippedWeaponHandlingSettings(
        coreWeaponHandlingBaseline,
        &externalWeaponHandling);
    ok &= expectTrue("an active addon request may enable handoff through the ROCK executor",
        externalHandling.ambidextrousHandoffEnabled);
    ok &= expectNear("an active owner supplies proximity tuning",
        externalHandling.firingGripProximitySupportRadiusGameUnits,
        8.0f);
    ok &= expectNear("legacy promotion tuning cannot change the shared station reach",
        externalHandling.firingGripReattachRadiusGameUnits,
        externalWeaponHandling.firingGripReattachRadiusGameUnits);
    ok &= expectNear("an active owner supplies left firing aim tuning",
        externalHandling.leftFiringAimYawDegrees,
        -4.0f);

    ok &= expectFalse("compatible addon activation does not tear down ROCK handoff",
        rock::requiresEquippedWeaponHandlingModeReconcile(
            coreWeaponHandling,
            externalHandling));

    rock::provider::RockProviderEquippedWeaponHandlingRequestV1 legacyStashRequest{};
    legacyStashRequest.flags =
        static_cast<std::uint32_t>(
            rock::provider::RockProviderEquippedWeaponHandlingFlagV1::FiringGripOwnership) |
        static_cast<std::uint32_t>(
            rock::provider::RockProviderEquippedWeaponHandlingFlagV1::PrimaryDetach) |
        static_cast<std::uint32_t>(
            rock::provider::RockProviderEquippedWeaponHandlingFlagV1::EquippedWeaponShoulderStash);
    legacyStashRequest.firingGripReattachRadiusGameUnits = 9.0f;
    legacyStashRequest.weaponGripHapticDurationSeconds = 0.20f;
    legacyStashRequest.firingGripAttachHapticIntensity = 0.40f;
    legacyStashRequest.firingGripDetachHapticIntensity = 0.50f;
    const auto legacyStashHandling = rock::makeEquippedWeaponHandlingSettings(
        fixedOnlyBaseline,
        &legacyStashRequest);
    ok &= expectFalse("legacy provider stash bit cannot enable ROCK shoulder stash",
        legacyStashHandling.equippedWeaponShoulderStashEnabled);
    ok &= expectTrue("legacy provider detach remains available to its other handling paths",
        legacyStashHandling.primaryDetachEnabled);
    const auto providerDetach =
        rock::resolveEquippedWeaponDetachDecision(
            legacyStashHandling);
    ok &= expectEqual("provider PrimaryDetach retains firing-role authority",
        providerDetach.authority,
        rock::immersive_weapon_policy::DetachAuthority::ExternalProvider);
    ok &= expectFalse("provider detach does not inherit integrated pose preservation",
        providerDetach.preserveWeaponPoseOnDetach);
    ok &= expectTrue("provider PrimaryDetach remains all-firing-hand capable",
        providerDetach.primaryDetachEnabled);
    ok &= expectFalse("provider PrimaryDetach does not inherit integrated pose preservation",
        providerDetach.preserveWeaponPoseOnDetach);
    ok &= expectNear("provider PrimaryDetach owns reattach tuning",
        providerDetach.reattachRadiusGameUnits,
        9.0f);
    ok &= expectNear("provider PrimaryDetach owns attach haptic tuning",
        providerDetach.gripAttachHapticIntensity,
        0.40f);
    ok &= expectNear("provider PrimaryDetach owns detach haptic tuning",
        providerDetach.gripDetachHapticIntensity,
        0.50f);

    return ok ? 0 : 1;
}
