#include "physics-interaction/stash/ShoulderStashMath.h"
#include "physics-interaction/stash/ShoulderStashPolicy.h"

#include <cmath>
#include <cstdio>
#include <cstring>

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
        if (!value) {
            return true;
        }

        std::printf("%s expected false\n", label);
        return false;
    }

    bool expectString(const char* label, const char* actual, const char* expected)
    {
        if (std::strcmp(actual, expected) == 0) {
            return true;
        }

        std::printf("%s expected %s got %s\n", label, expected, actual);
        return false;
    }

    bool expectNear(const char* label, float actual, float expected, float epsilon)
    {
        const float delta = std::fabs(actual - expected);
        if (delta <= epsilon) {
            return true;
        }

        std::printf("%s expected %.4f got %.4f\n", label, expected, actual);
        return false;
    }
}

int main()
{
    using namespace rock::shoulder_stash;
    using rock::body_zone::BodyZoneKind;
    namespace haptics = rock::shoulder_stash_haptic_policy;
    namespace notifications = rock::shoulder_stash_notification_policy;

    bool ok = true;

    ok &= expectString("HMD back source is explicit",
        evidenceSourceName(EvidenceSource::HmdBackVolume),
        "hmd-back-volume");

    ok &= expectTrue("HMD back source is recognized",
        isHmdBackVolumeEvidenceSource(EvidenceSource::HmdBackVolume));
    ok &= expectFalse("body source is not HMD back",
        isHmdBackVolumeEvidenceSource(EvidenceSource::BodyZoneCollider));
    ok &= expectTrue("HMD back gate allows a probe behind the headset",
        hmdBackBehindGateAllows(-18.0f, 4.0f));
    ok &= expectFalse("HMD back gate rejects the old forward-leaking sphere edge",
        hmdBackBehindGateAllows(1.0f, 4.0f));
    ok &= expectFalse("HMD back gate rejects probes short of the rear threshold",
        hmdBackBehindGateAllows(-3.9f, 4.0f));
    ok &= expectTrue("HMD back gate can be disabled for compatibility",
        hmdBackBehindGateAllows(1.0f, 0.0f));

    ok &= expectTrue("HMD primary to collider backup preserves dwell on same zone",
        shoulderStashDwellIdentityMatches(
            BodyZoneKind::RightShoulder,
            EvidenceSource::HmdBackVolume,
            kInvalidBodyId,
            BodyZoneKind::RightShoulder,
            EvidenceSource::BodyZoneCollider,
            42));

    ok &= expectTrue("collider backup to HMD primary preserves dwell on same zone",
        shoulderStashDwellIdentityMatches(
            BodyZoneKind::LeftShoulder,
            EvidenceSource::BodyZoneColliderAndContact,
            77,
            BodyZoneKind::LeftShoulder,
            EvidenceSource::HmdBackVolume,
            kInvalidBodyId));

    ok &= expectFalse("HMD/body transition does not bridge opposite shoulders",
        shoulderStashDwellIdentityMatches(
            BodyZoneKind::LeftShoulder,
            EvidenceSource::HmdBackVolume,
            kInvalidBodyId,
            BodyZoneKind::RightShoulder,
            EvidenceSource::BodyZoneCollider,
            42));

    ok &= expectFalse("different body colliders still reset body-only dwell",
        shoulderStashDwellIdentityMatches(
            BodyZoneKind::RightShoulder,
            EvidenceSource::BodyZoneCollider,
            42,
            BodyZoneKind::RightShoulder,
            EvidenceSource::BodyZoneCollider,
            43));

    haptics::CandidatePulseConfig haptic{};
    haptic.baseIntensity = 0.20f;
    haptic.maxIntensity = 0.42f;
    ok &= expectNear("candidate haptic starts at base intensity",
        haptics::computeCandidatePulseIntensity(0.0f, haptic),
        0.20f,
        0.001f);
    ok &= expectNear("candidate haptic scales by confidence",
        haptics::computeCandidatePulseIntensity(0.5f, haptic),
        0.31f,
        0.001f);
    ok &= expectNear("candidate haptic clamps to max",
        haptics::computeCandidatePulseIntensity(2.0f, haptic),
        0.42f,
        0.001f);

    haptic.enabled = false;
    ok &= expectNear("disabled candidate haptic suppresses pulse",
        haptics::computeCandidatePulseIntensity(1.0f, haptic),
        0.0f,
        0.001f);

    const auto namedNotification = notifications::formatCollectedNotification("Abraxo Cleaner", 3, 0x000ABCDEu);
    ok &= expectString("collected notification includes item count",
        namedNotification.c_str(),
        "[ROCK] Collected Abraxo Cleaner x3");

    const auto fallbackNotification = notifications::formatCollectedNotification({}, 1, 0x000ABCDEu);
    ok &= expectString("collected notification falls back to form id",
        fallbackNotification.c_str(),
        "[ROCK] Collected item 000ABCDE");

    return ok ? 0 : 1;
}
