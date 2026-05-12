#include "physics-interaction/stash/ShoulderStashMath.h"

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
}

int main()
{
    using namespace rock::shoulder_stash;
    using rock::body_zone::BodyZoneKind;

    bool ok = true;

    ok &= expectString("HMD back source is explicit",
        evidenceSourceName(EvidenceSource::HmdBackVolume),
        "hmd-back-volume");

    ok &= expectTrue("HMD back source is recognized",
        isHmdBackVolumeEvidenceSource(EvidenceSource::HmdBackVolume));
    ok &= expectFalse("body source is not HMD back",
        isHmdBackVolumeEvidenceSource(EvidenceSource::BodyZoneCollider));

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

    return ok ? 0 : 1;
}
