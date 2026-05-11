#include <cstdio>

#include "physics-interaction/body/BodyContactRuntime.h"
#include "physics-interaction/debug/SkeletonBoneDebugMath.h"

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

    bool expectUInt32(const char* label, std::uint32_t actual, std::uint32_t expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected %u got %u\n", label, expected, actual);
        return false;
    }

    bool expectSize(const char* label, std::size_t actual, std::size_t expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected %zu got %zu\n", label, expected, actual);
        return false;
    }
}

int main()
{
    using namespace rock::body_contact_runtime;
    using rock::body_zone::BodyZoneKind;
    using rock::body_zone::BodyZoneSide;
    using rock::contact_pipeline_policy::ContactEndpointKind;
    using rock::skeleton_bone_debug_math::BoneColliderRole;
    using rock::skeleton_bone_debug_math::kPowerArmorBodyColliderDescriptors;
    using rock::skeleton_bone_debug_math::kStandardBodyColliderDescriptors;

    bool ok = true;

    static_assert(kStandardBodyColliderDescriptors.size() == kPowerArmorBodyColliderDescriptors.size());
    static_assert(kStandardBodyColliderDescriptors[5].zone == BodyZoneKind::LeftShoulder);
    static_assert(kStandardBodyColliderDescriptors[10].zone == BodyZoneKind::RightShoulder);
    static_assert(kPowerArmorBodyColliderDescriptors[5].zone == BodyZoneKind::LeftShoulder);
    static_assert(kPowerArmorBodyColliderDescriptors[10].zone == BodyZoneKind::RightShoulder);

    BodyContactRuntime runtime;
    BodyContactRecord invalid{};
    invalid.bodyId = 100;
    invalid.targetBodyId = 100;
    runtime.record(invalid);
    ok &= expectSize("same-body contact is rejected", runtime.recordCount(), 0);

    BodyContactRecord shoulderContact{};
    shoulderContact.frame = 42;
    shoulderContact.bodyId = 101;
    shoulderContact.targetBodyId = 501;
    shoulderContact.bodyLayer = 47;
    shoulderContact.targetLayer = 44;
    shoulderContact.role = BoneColliderRole::UpperArmSegment;
    shoulderContact.zone = BodyZoneKind::LeftShoulder;
    shoulderContact.side = BodyZoneSide::Left;
    shoulderContact.descriptorIndex = 5;
    shoulderContact.targetKind = ContactEndpointKind::Weapon;
    shoulderContact.inPowerArmor = true;
    shoulderContact.contactPointGame = RE::NiPoint3{ 1.0f, 2.0f, 3.0f };
    shoulderContact.hasContactPointGame = true;
    runtime.record(shoulderContact);

    BodyContactRecord snapshot[2]{};
    ok &= expectSize("valid body contact is recorded", runtime.snapshot(snapshot, 2), 1);
    ok &= expectUInt32("body contact preserves source id", snapshot[0].bodyId, 101);
    ok &= expectUInt32("body contact preserves target id", snapshot[0].targetBodyId, 501);
    ok &= expectTrue("body contact preserves source zone", snapshot[0].zone == BodyZoneKind::LeftShoulder);
    ok &= expectTrue("body contact preserves source side", snapshot[0].side == BodyZoneSide::Left);
    ok &= expectTrue("body contact preserves target kind", snapshot[0].targetKind == ContactEndpointKind::Weapon);
    ok &= expectTrue("body contact preserves point validity", snapshot[0].hasContactPointGame);
    ok &= expectTrue("body contact preserves power armor state", snapshot[0].inPowerArmor);

    for (std::uint32_t i = 0; i < static_cast<std::uint32_t>(kMaxBodyContactRecords + 4); ++i) {
        BodyContactRecord record = shoulderContact;
        record.frame = i;
        record.bodyId = 1000 + i;
        record.targetBodyId = 2000 + i;
        runtime.record(record);
    }

    BodyContactRecord latest[3]{};
    ok &= expectSize("body contact snapshot copies requested latest window", runtime.snapshot(latest, 3), 3);
    ok &= expectUInt32("latest window starts at third newest body", latest[0].bodyId, 1000 + static_cast<std::uint32_t>(kMaxBodyContactRecords + 1));
    ok &= expectUInt32("latest window ends at newest body", latest[2].bodyId, 1000 + static_cast<std::uint32_t>(kMaxBodyContactRecords + 3));

    runtime.reset();
    ok &= expectSize("reset clears body contacts", runtime.recordCount(), 0);

    return ok ? 0 : 1;
}
