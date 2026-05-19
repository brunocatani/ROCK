#include "physics-interaction/grab/GrabCore.h"

#include <cstdio>

namespace
{
    rock::object_physics_body_set::ObjectPhysicsBodyRecord makeRecord(
        std::uint32_t bodyId,
        rock::physics_body_classifier::BodyMotionType motionType,
        rock::physics_body_classifier::BodyRejectReason rejectReason,
        bool accepted)
    {
        rock::object_physics_body_set::ObjectPhysicsBodyRecord record{};
        record.bodyId = bodyId;
        record.motionId = bodyId + 100u;
        record.motionPropertiesId =
            motionType == rock::physics_body_classifier::BodyMotionType::Dynamic ? 1u :
            motionType == rock::physics_body_classifier::BodyMotionType::Keyframed ? 2u :
            0u;
        record.motionType = motionType;
        record.filterInfo = 0x01880004u;
        record.accepted = accepted;
        record.rejectReason = rejectReason;
        return record;
    }

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
    using namespace rock;
    using namespace rock::active_grab_body_lifecycle;
    using object_physics_body_set::ObjectPhysicsBodySet;
    using physics_body_classifier::BodyMotionType;
    using physics_body_classifier::BodyRejectReason;

    bool ok = true;

    ObjectPhysicsBodySet dynamicBefore{};
    dynamicBefore.records.push_back(makeRecord(101u, BodyMotionType::Dynamic, BodyRejectReason::None, true));
    BodyLifecycleSnapshot dynamicSnapshot{};
    dynamicSnapshot.captureBeforeActivePrep(dynamicBefore);
    dynamicSnapshot.markPreparedBodies(dynamicBefore);
    const auto dynamicRelease = dynamicSnapshot.restorePlanForRelease(BodyRestorePolicy::ProtectComplexSystemOwned);
    ok &= expectFalse("loose dynamic release should keep active collision filter", dynamicRelease.entries.front().restoreFilter);
    ok &= expectFalse("loose dynamic release should keep dynamic motion", dynamicRelease.entries.front().restoreMotion);

    const auto dynamicFailure = dynamicSnapshot.restorePlanForFailure();
    ok &= expectTrue("failed dynamic prep should restore captured filter", dynamicFailure.entries.front().restoreFilter);
    ok &= expectTrue("failed dynamic prep should restore captured motion", dynamicFailure.entries.front().restoreMotion);

    ObjectPhysicsBodySet keyframedBefore{};
    keyframedBefore.records.push_back(makeRecord(202u, BodyMotionType::Keyframed, BodyRejectReason::KeyframedPassive, false));
    ObjectPhysicsBodySet keyframedPrepared{};
    keyframedPrepared.records.push_back(makeRecord(202u, BodyMotionType::Dynamic, BodyRejectReason::None, true));
    BodyLifecycleSnapshot keyframedSnapshot{};
    keyframedSnapshot.captureBeforeActivePrep(keyframedBefore);
    keyframedSnapshot.markPreparedBodies(keyframedPrepared);
    const auto keyframedRelease = keyframedSnapshot.restorePlanForRelease(BodyRestorePolicy::ProtectComplexSystemOwned);
    ok &= expectTrue("system-owned release should restore filter", keyframedRelease.entries.front().restoreFilter);
    ok &= expectTrue("system-owned release should restore motion", keyframedRelease.entries.front().restoreMotion);

    return ok ? 0 : 1;
}
