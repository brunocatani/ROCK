#include "physics-interaction/stash/ShoulderStashMath.h"

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
        if (!value) {
            return true;
        }

        std::printf("%s expected false\n", label);
        return false;
    }
}

int main()
{
    using namespace rock::shoulder_stash;
    using rock::body_zone::BodyZoneKind;

    bool ok = true;

    /*
     * HMD fallback is only a substitute when the FRIK body authority cannot
     * answer the frame. When shoulder colliders are readable, a miss from the
     * generated body zone is a real rejection and must not be widened by HMD.
     */
    ok &= expectTrue("hmd fallback allowed when body colliders are disabled",
        shouldUseHmdFallback(true, false, true));
    ok &= expectTrue("hmd fallback allowed when body authority is unavailable",
        shouldUseHmdFallback(true, true, false));
    ok &= expectFalse("hmd fallback blocked when body authority is available",
        shouldUseHmdFallback(true, true, true));
    ok &= expectFalse("hmd fallback disabled by config",
        shouldUseHmdFallback(false, true, false));

    ok &= expectTrue("dwell survives collider contact confirmation",
        shoulderStashDwellIdentityMatches(
            BodyZoneKind::RightShoulder,
            EvidenceSource::BodyZoneCollider,
            42,
            BodyZoneKind::RightShoulder,
            EvidenceSource::BodyZoneColliderAndContact,
            42));
    ok &= expectTrue("dwell survives contact returning to collider",
        shoulderStashDwellIdentityMatches(
            BodyZoneKind::LeftShoulder,
            EvidenceSource::BodyZoneColliderAndContact,
            77,
            BodyZoneKind::LeftShoulder,
            EvidenceSource::BodyZoneCollider,
            77));
    ok &= expectTrue("dwell survives sustained contact after fresh contact expires",
        shoulderStashDwellIdentityMatches(
            BodyZoneKind::RightShoulder,
            EvidenceSource::BodyZoneContact,
            42,
            BodyZoneKind::RightShoulder,
            EvidenceSource::BodyZoneSustainedContact,
            42));
    ok &= expectTrue("dwell survives sustained contact returning to collider",
        shoulderStashDwellIdentityMatches(
            BodyZoneKind::RightShoulder,
            EvidenceSource::BodyZoneSustainedContact,
            42,
            BodyZoneKind::RightShoulder,
            EvidenceSource::BodyZoneCollider,
            42));
    ok &= expectFalse("dwell resets across hmd and body evidence",
        shoulderStashDwellIdentityMatches(
            BodyZoneKind::RightShoulder,
            EvidenceSource::HmdFallback,
            kInvalidBodyId,
            BodyZoneKind::RightShoulder,
            EvidenceSource::BodyZoneCollider,
            42));
    ok &= expectFalse("dwell resets when shoulder body changes",
        shoulderStashDwellIdentityMatches(
            BodyZoneKind::RightShoulder,
            EvidenceSource::BodyZoneCollider,
            42,
            BodyZoneKind::RightShoulder,
            EvidenceSource::BodyZoneCollider,
            43));
    ok &= expectFalse("dwell resets when shoulder side changes",
        shoulderStashDwellIdentityMatches(
            BodyZoneKind::RightShoulder,
            EvidenceSource::BodyZoneCollider,
            42,
            BodyZoneKind::LeftShoulder,
            EvidenceSource::BodyZoneCollider,
            42));

    ok &= expectTrue("sustained contact tolerates configured missed frames",
        sustainedContactMissWithinTolerance(18, 18));
    ok &= expectFalse("sustained contact clears after configured missed frames",
        sustainedContactMissWithinTolerance(19, 18));
    ok &= expectFalse("negative sustained miss tolerance disables bridging",
        sustainedContactMissWithinTolerance(0, -1));

    return ok ? 0 : 1;
}
