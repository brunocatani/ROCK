#include <cstdio>

#include "physics-interaction/contact/NativeContactEvidence.h"

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

    bool expectUint32(const char* label, std::uint32_t actual, std::uint32_t expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected %u got %u\n", label, expected, actual);
        return false;
    }
}

int main()
{
    using namespace rock::contact_evidence;

    bool ok = true;

    NativeContactEvidenceCache cache;
    NativeContactEvidenceSnapshot snapshot{};

    NativeContactEvidenceRecord invalid{};
    invalid.sourceBodyId = 100;
    invalid.targetBodyId = 100;
    cache.record(invalid);
    cache.snapshot(snapshot, 10);
    ok &= expectUint32("invalid same-body evidence is rejected", snapshot.count, 0);

    NativeContactEvidenceRecord worldContact{};
    worldContact.frame = 42;
    worldContact.sourceBodyId = 101;
    worldContact.targetBodyId = 501;
    worldContact.sourceLayer = 43;
    worldContact.targetLayer = 1;
    worldContact.sourceFilterInfo = 0x0000002B;
    worldContact.targetFilterInfo = 0x00000001;
    worldContact.sourceKind = NativeContactEndpointKind::RightHand;
    worldContact.targetKind = NativeContactEndpointKind::WorldSurface;
    worldContact.quality = NativeContactQuality::RawPoint;
    worldContact.sourceIsLeft = false;
    worldContact.contactPointGame = RE::NiPoint3{ 1.0f, 2.0f, 3.0f };
    worldContact.contactNormalGame = RE::NiPoint3{ 0.0f, 0.0f, 1.0f };
    cache.record(worldContact);

    cache.snapshot(snapshot, 43);
    ok &= expectUint32("valid native evidence is captured", snapshot.count, 1);
    ok &= expectTrue("captured evidence is marked valid", snapshot.records[0].valid);
    ok &= expectTrue("captured evidence gets sequence", snapshot.records[0].sequence > 0);
    ok &= expectUint32("captured evidence preserves source layer", snapshot.records[0].sourceLayer, 43);
    ok &= expectUint32("captured evidence preserves target layer", snapshot.records[0].targetLayer, 1);
    ok &= expectUint32("captured evidence preserves source filter info", snapshot.records[0].sourceFilterInfo, 0x0000002B);
    ok &= expectUint32("captured evidence preserves target filter info", snapshot.records[0].targetFilterInfo, 0x00000001);
    ok &= expectTrue("one-frame native evidence is fresh", isFrameFresh(snapshot.currentFrame, snapshot.records[0].frame, 2));
    ok &= expectFalse("old native evidence is stale", isFrameFresh(50, snapshot.records[0].frame, 2));

    NativeContactEvidenceRecord leftWorldContact = worldContact;
    leftWorldContact.sourceBodyId = 102;
    leftWorldContact.targetBodyId = 502;
    leftWorldContact.sourceKind = NativeContactEndpointKind::LeftHand;
    leftWorldContact.sourceIsLeft = true;
    cache.record(leftWorldContact);

    ok &= expectUint32("right-hand invalidation removes one record", cache.invalidateHand(false), 1);
    cache.snapshot(snapshot, 44);
    ok &= expectUint32("left-hand evidence survives right invalidation", snapshot.count, 1);
    ok &= expectTrue("remaining evidence belongs to left hand", snapshot.records[0].sourceKind == NativeContactEndpointKind::LeftHand);
    ok &= expectUint32("left-hand invalidation removes remaining record", cache.invalidateHand(true), 1);
    cache.snapshot(snapshot, 45);
    ok &= expectUint32("per-hand invalidation clears native hand evidence", snapshot.count, 0);

    cache.reset();
    cache.snapshot(snapshot, 51);
    ok &= expectUint32("reset clears native evidence", snapshot.count, 0);

    for (std::uint32_t i = 0; i < static_cast<std::uint32_t>(kMaxNativeContactEvidenceRecords + 4); ++i) {
        auto record = worldContact;
        record.frame = i;
        record.sourceBodyId = 1000 + i;
        record.targetBodyId = 2000 + i;
        cache.record(record);
    }

    cache.snapshot(snapshot, 200);
    ok &= expectUint32(
        "native evidence cache stays bounded",
        snapshot.count,
        static_cast<std::uint32_t>(kMaxNativeContactEvidenceRecords));

    return ok ? 0 : 1;
}
