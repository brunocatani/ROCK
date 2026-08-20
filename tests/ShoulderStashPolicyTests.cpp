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

    bool expectPointNear(
        const char* label,
        const RE::NiPoint3& actual,
        const RE::NiPoint3& expected,
        float epsilon)
    {
        const bool matches =
            std::fabs(actual.x - expected.x) <= epsilon &&
            std::fabs(actual.y - expected.y) <= epsilon &&
            std::fabs(actual.z - expected.z) <= epsilon;
        if (matches) {
            return true;
        }

        std::printf(
            "%s expected (%.4f, %.4f, %.4f) got (%.4f, %.4f, %.4f)\n",
            label,
            expected.x,
            expected.y,
            expected.z,
            actual.x,
            actual.y,
            actual.z);
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

    const RE::NiPoint3 hmdOrigin{ 100.0f, 200.0f, 300.0f };
    const RE::NiPoint3 expectedRightPocketLocal{ 14.0f, -18.0f, -6.85f };
    HmdBackFrame levelFrame{};
    ok &= expectTrue("level HMD produces a back frame",
        tryBuildHmdBackFrame(
            hmdOrigin,
            RE::NiPoint3{ 0.0f, 1.0f, 0.0f },
            levelFrame));
    const RE::NiPoint3 rightPocketWorld =
        hmdBackLocalPointToWorld(levelFrame, expectedRightPocketLocal);
    ok &= expectPointNear("level HMD back frame round trips pocket",
        worldPointToHmdBackLocal(levelFrame, rightPocketWorld),
        expectedRightPocketLocal,
        0.001f);

    HmdBackFrame pitchedFrame{};
    ok &= expectTrue("pitched HMD keeps a valid yaw-only back frame",
        tryBuildHmdBackFrame(
            hmdOrigin,
            RE::NiPoint3{ 0.0f, 0.70710678f, -0.70710678f },
            pitchedFrame));
    ok &= expectPointNear("HMD pitch does not move the back pocket",
        hmdBackLocalPointToWorld(pitchedFrame, expectedRightPocketLocal),
        rightPocketWorld,
        0.001f);

    HmdBackFrame walkedFrame{};
    const RE::NiPoint3 walkDelta{ 40.0f, -25.0f, 0.0f };
    ok &= expectTrue("translated HMD produces a back frame",
        tryBuildHmdBackFrame(
            add(hmdOrigin, walkDelta),
            RE::NiPoint3{ 0.0f, 1.0f, 0.0f },
            walkedFrame));
    const RE::NiPoint3 walkedHandWorld = add(rightPocketWorld, walkDelta);
    const RE::NiPoint3 walkedHandLocal =
        worldPointToHmdBackLocal(walkedFrame, walkedHandWorld);
    ok &= expectPointNear("walking preserves HMD-local pocket position",
        walkedHandLocal,
        expectedRightPocketLocal,
        0.001f);
    ok &= expectNear("walking does not add HMD-local gesture speed",
        pointMotionSpeed(
            walkedHandLocal,
            expectedRightPocketLocal,
            true,
            0.01f),
        0.0f,
        0.001f);

    HmdBackFrame crouchedFrame{};
    const RE::NiPoint3 crouchDelta{ 0.0f, 0.0f, -30.0f };
    ok &= expectTrue("crouched HMD produces a back frame",
        tryBuildHmdBackFrame(
            add(hmdOrigin, crouchDelta),
            RE::NiPoint3{ 0.0f, 1.0f, 0.0f },
            crouchedFrame));
    ok &= expectPointNear("crouching preserves HMD-local pocket position",
        worldPointToHmdBackLocal(
            crouchedFrame,
            add(rightPocketWorld, crouchDelta)),
        expectedRightPocketLocal,
        0.001f);

    HmdBackFrame turnedFrame{};
    ok &= expectTrue("yaw-turned HMD produces a back frame",
        tryBuildHmdBackFrame(
            hmdOrigin,
            RE::NiPoint3{ 1.0f, 0.0f, 0.0f },
            turnedFrame));
    ok &= expectPointNear("yaw-turned back frame round trips pocket",
        worldPointToHmdBackLocal(
            turnedFrame,
            hmdBackLocalPointToWorld(turnedFrame, expectedRightPocketLocal)),
        expectedRightPocketLocal,
        0.001f);

    HmdBackFrame invalidVerticalFrame{};
    ok &= expectFalse("vertical HMD direction fails closed without yaw",
        tryBuildHmdBackFrame(
            hmdOrigin,
            RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
            invalidVerticalFrame));
    ok &= expectNear("real HMD-local hand motion retains gesture speed",
        pointMotionSpeed(
            RE::NiPoint3{ 14.0f, -16.0f, -6.85f },
            expectedRightPocketLocal,
            true,
            0.01f),
        200.0f,
        0.001f);

    ok &= expectTrue("configured stash speed ceiling rejects fast probes",
        exceedsShoulderStashSpeedLimit(141.0f, 140.0f));
    ok &= expectFalse("stash speed ceiling accepts the boundary",
        exceedsShoulderStashSpeedLimit(140.0f, 140.0f));
    ok &= expectFalse("zero stash speed ceiling disables speed rejection",
        exceedsShoulderStashSpeedLimit(1000.0f, 0.0f));
    ok &= expectTrue("confirmed fast open release arms equipped stash lease",
        shouldArmEquippedWeaponFastReleaseCommitLease(true, true, false, true));
    ok &= expectFalse("fast held motion cannot arm equipped stash lease",
        shouldArmEquippedWeaponFastReleaseCommitLease(true, true, true, true));
    ok &= expectFalse("unconfirmed fast entry cannot arm equipped stash lease",
        shouldArmEquippedWeaponFastReleaseCommitLease(false, true, false, true));
    ok &= expectFalse("leaving the confirmed volume cannot arm equipped stash lease",
        shouldArmEquippedWeaponFastReleaseCommitLease(true, true, false, false));
    ok &= expectTrue("matching open-frame equipped stash lease is usable",
        equippedWeaponFastReleaseCommitLeaseIsUsable(true, 0xAAu, 0xAAu, 1, false, true));
    ok &= expectFalse("regripping invalidates equipped stash lease",
        equippedWeaponFastReleaseCommitLeaseIsUsable(true, 0xAAu, 0xAAu, 1, true, true));
    ok &= expectFalse("leaving the back volume invalidates equipped stash lease",
        equippedWeaponFastReleaseCommitLeaseIsUsable(true, 0xAAu, 0xAAu, 1, false, false));
    ok &= expectFalse("equipped stash lease rejects a replacement weapon instance",
        equippedWeaponFastReleaseCommitLeaseIsUsable(true, 0xAAu, 0xBBu, 1, false, true));
    ok &= expectFalse("equipped stash lease expires after its bounded open-frame window",
        equippedWeaponFastReleaseCommitLeaseIsUsable(true, 0xAAu, 0xAAu, 0, false, true));

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
        "Collected Abraxo Cleaner x3");

    const auto fallbackNotification = notifications::formatCollectedNotification({}, 1, 0x000ABCDEu);
    ok &= expectString("collected notification falls back to form id",
        fallbackNotification.c_str(),
        "Collected item 000ABCDE");

    const auto stowedNotification = notifications::formatStowedNotification("10mm Pistol", 0x000ABCDEu);
    ok &= expectString("stowed notification uses the stow verb",
        stowedNotification.c_str(),
        "Stowed 10mm Pistol");

    const auto stowedFallbackNotification = notifications::formatStowedNotification({}, 0x000ABCDEu);
    ok &= expectString("stowed notification falls back to form id",
        stowedFallbackNotification.c_str(),
        "Stowed item 000ABCDE");

    return ok ? 0 : 1;
}
