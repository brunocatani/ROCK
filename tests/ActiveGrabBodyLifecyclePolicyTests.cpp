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
    const auto dynamicRelease = dynamicSnapshot.restorePlanForRelease(
        releaseRestorePolicyForTargetKind(grab_target::Kind::LooseObject),
        grab_target::Kind::LooseObject,
        BodyReleaseIntent::PhysicalDrop);
    ok &= expectFalse("loose dynamic release should keep active collision filter", dynamicRelease.entries.front().restoreFilter);
    ok &= expectFalse("loose dynamic release should keep dynamic motion", dynamicRelease.entries.front().restoreMotion);
    ok &= expectTrue("loose dynamic physical drop should skip incomplete root restore when no state is restored",
        shouldSkipIncompleteScanRootRestore(dynamicRelease, 5));
    ok &= expectTrue("loose object release should use protected restore policy",
        releaseRestorePolicyForTargetKind(grab_target::Kind::LooseObject) == BodyRestorePolicy::ProtectComplexSystemOwned);

    const auto dynamicFailure = dynamicSnapshot.restorePlanForFailure();
    ok &= expectTrue("failed dynamic prep should restore captured filter", dynamicFailure.entries.front().restoreFilter);
    ok &= expectTrue("failed dynamic prep should restore captured motion", dynamicFailure.entries.front().restoreMotion);

    ObjectPhysicsBodySet truncatedScan{};
    truncatedScan.diagnostics.depthLimitSkips = 1;
    BodyLifecycleSnapshot truncatedSnapshot{};
    truncatedSnapshot.captureBeforeActivePrep(truncatedScan);
    ok &= expectTrue("depth-truncated scans should force incomplete restore fallback", truncatedSnapshot.hasIncompleteNativeScan());

    ObjectPhysicsBodySet cachedPreparedReplay{};
    cachedPreparedReplay.records.push_back(makeRecord(151u, BodyMotionType::Dynamic, BodyRejectReason::None, true));
    BodyLifecycleSnapshot cachedPreparedSnapshot{};
    cachedPreparedSnapshot.captureBeforeActivePrep(cachedPreparedReplay);
    cachedPreparedSnapshot.markPreparedBodies(cachedPreparedReplay);
    ok &= expectFalse("pre-prep cached replay should not manufacture late bodies", cachedPreparedSnapshot.latePreparedBodyCount() > 0);
    cachedPreparedSnapshot.markIncompleteNativeScan();
    ok &= expectTrue("pre-prep cached replay without post-prep proof should force incomplete restore fallback", cachedPreparedSnapshot.hasIncompleteNativeScan());

    const auto deadActorRelease = dynamicSnapshot.restorePlanForRelease(
        releaseRestorePolicyForTargetKind(grab_target::Kind::DeadActorBody),
        grab_target::Kind::DeadActorBody,
        BodyReleaseIntent::PhysicalDrop);
    ok &= expectTrue("dead actor release should restore captured dynamic filter", deadActorRelease.entries.front().restoreFilter);
    ok &= expectTrue("dead actor release should restore touched dynamic motion", deadActorRelease.entries.front().restoreMotion);
    ok &= expectTrue("dead actor release should use restore-all policy",
        releaseRestorePolicyForTargetKind(grab_target::Kind::DeadActorBody) == BodyRestorePolicy::RestoreAllChanged);

    ObjectPhysicsBodySet keyframedBefore{};
    keyframedBefore.records.push_back(makeRecord(202u, BodyMotionType::Keyframed, BodyRejectReason::KeyframedPassive, false));
    ObjectPhysicsBodySet keyframedPrepared{};
    keyframedPrepared.records.push_back(makeRecord(202u, BodyMotionType::Dynamic, BodyRejectReason::None, true));
    BodyLifecycleSnapshot keyframedSnapshot{};
    keyframedSnapshot.captureBeforeActivePrep(keyframedBefore);
    keyframedSnapshot.markPreparedBodies(keyframedPrepared);
    const auto keyframedPhysicalDrop = keyframedSnapshot.restorePlanForRelease(
        releaseRestorePolicyForTargetKind(grab_target::Kind::LooseObject),
        grab_target::Kind::LooseObject,
        BodyReleaseIntent::PhysicalDrop);
    ok &= expectFalse("loose keyframed physical drop should keep active collision filter", keyframedPhysicalDrop.entries.front().restoreFilter);
    ok &= expectFalse("loose keyframed physical drop should keep converted dynamic motion", keyframedPhysicalDrop.entries.front().restoreMotion);
    ok &= expectTrue("loose keyframed physical drop should count preserved motion", keyframedPhysicalDrop.preservedConvertedMotionCount == 1);
    ok &= expectTrue("loose keyframed physical drop should skip incomplete root restore",
        shouldSkipIncompleteScanRootRestore(keyframedPhysicalDrop, 2));

    const auto keyframedTransfer = keyframedSnapshot.restorePlanForRelease(
        releaseRestorePolicyForTargetKind(grab_target::Kind::LooseObject),
        grab_target::Kind::LooseObject,
        BodyReleaseIntent::NonPhysicalTransfer);
    ok &= expectTrue("loose keyframed non-physical transfer should restore filter", keyframedTransfer.entries.front().restoreFilter);
    ok &= expectTrue("loose keyframed non-physical transfer should restore motion", keyframedTransfer.entries.front().restoreMotion);
    ok &= expectFalse("loose keyframed non-physical transfer should allow incomplete root restore skip",
        shouldSkipIncompleteScanRootRestore(keyframedTransfer, 2));

    for (const auto disposition : { GrabReleaseDisposition::PendingInventoryTransfer, GrabReleaseDisposition::PendingConsumeTransfer }) {
        const auto intent = releaseIntentFromDisposition(disposition);
        ok &= expectTrue("unconfirmed native transfer remains pending", intent == BodyReleaseIntent::PendingTransfer);
        ok &= expectTrue("pending drop activates even without a velocity snapshot", shouldActivateReleasedBodies(disposition));
        const auto pending = keyframedSnapshot.restorePlanForRelease(
            releaseRestorePolicyForTargetKind(grab_target::Kind::LooseObject), grab_target::Kind::LooseObject, intent);
        ok &= expectFalse("pending pickup cannot return converted body to keyframed motion", pending.entries.front().restoreMotion);
        ok &= expectFalse("pending pickup cannot restore an inactive collision filter", pending.entries.front().restoreFilter);
        ok &= expectTrue("pending pickup has no native keyframe restoration commands", keyframedSnapshot.makeMotionRestoreCommands(pending).empty());
        ok &= expectTrue("incomplete pending scan preserves world physics", shouldSkipIncompleteScanRootRestore(pending, 2));
        ok &= expectTrue("pending converted state equals a rejected transfer's physical drop",
            pending.entries.front().restoreMotion == keyframedPhysicalDrop.entries.front().restoreMotion &&
            pending.entries.front().restoreFilter == keyframedPhysicalDrop.entries.front().restoreFilter);
        // Preservation must not depend on throw history: a native pickup can
        // fail immediately after acquisition, before a velocity sample exists.
        const auto pendingDynamic = dynamicSnapshot.restorePlanForRelease(
            releaseRestorePolicyForTargetKind(grab_target::Kind::LooseObject), grab_target::Kind::LooseObject, intent);
        ok &= expectTrue("pending dynamic bodies avoid coarse property reset", shouldSkipIncompleteScanRootRestore(pendingDynamic, 5));
        const auto pendingActor = dynamicSnapshot.restorePlanForRelease(
            releaseRestorePolicyForTargetKind(grab_target::Kind::DeadActorBody), grab_target::Kind::DeadActorBody, intent);
        ok &= expectTrue("actor restoration is unaffected by pending transfer", pendingActor.entries.front().restoreMotion && pendingActor.entries.front().restoreFilter);
    }
    ok &= expectFalse("confirmed native ownership must not reactivate world bodies", shouldActivateReleasedBodies(GrabReleaseDisposition::TransferToInventory));
    ok &= expectFalse("ownership handoff is not a physical drop", shouldActivateReleasedBodies(GrabReleaseDisposition::OwnershipHandoff));
    ok &= expectTrue("ordinary drop activation is preserved", shouldActivateReleasedBodies(GrabReleaseDisposition::PhysicalDrop));
    ok &= expectTrue("confirmed inventory transfer keeps native restoration", releaseIntentFromDisposition(
        GrabReleaseDisposition::TransferToInventory) == BodyReleaseIntent::NonPhysicalTransfer);
    ok &= expectTrue("normal drop keeps physical intent", releaseIntentFromDisposition(
        GrabReleaseDisposition::PhysicalDrop) == BodyReleaseIntent::PhysicalDrop);
    ok &= expectTrue("ownership handoff remains separate", releaseIntentFromDisposition(
        GrabReleaseDisposition::OwnershipHandoff) == BodyReleaseIntent::OwnershipHandoff);

    const auto keyframedFailure = keyframedSnapshot.restorePlanForFailure();
    ok &= expectTrue("failed keyframed prep should restore filter", keyframedFailure.entries.front().restoreFilter);
    ok &= expectTrue("failed keyframed prep should restore motion", keyframedFailure.entries.front().restoreMotion);

    BodyLifecycleSnapshot pullConsumedSnapshot = keyframedSnapshot;
    const auto pullConsumedRelease = pullConsumedSnapshot.restorePlanForRelease(
        releaseRestorePolicyForTargetKind(grab_target::Kind::LooseObject),
        grab_target::Kind::LooseObject,
        BodyReleaseIntent::PhysicalDrop);
    ok &= expectFalse("pull-consumed loose keyframed physical drop should keep converted dynamic motion", pullConsumedRelease.entries.front().restoreMotion);

    ObjectPhysicsBodySet staticBefore{};
    staticBefore.records.push_back(makeRecord(303u, BodyMotionType::Static, BodyRejectReason::StaticMotion, false));
    ObjectPhysicsBodySet staticPrepared{};
    staticPrepared.records.push_back(makeRecord(303u, BodyMotionType::Dynamic, BodyRejectReason::None, true));
    BodyLifecycleSnapshot staticSnapshot{};
    staticSnapshot.captureBeforeActivePrep(staticBefore);
    staticSnapshot.markPreparedBodies(staticPrepared);
    const auto staticPhysicalDrop = staticSnapshot.restorePlanForRelease(
        releaseRestorePolicyForTargetKind(grab_target::Kind::LooseObject),
        grab_target::Kind::LooseObject,
        BodyReleaseIntent::PhysicalDrop);
    ok &= expectTrue("loose static physical drop should still restore motion", staticPhysicalDrop.entries.front().restoreMotion);

    const auto staticPending = staticSnapshot.restorePlanForRelease(
        releaseRestorePolicyForTargetKind(grab_target::Kind::LooseObject), grab_target::Kind::LooseObject, BodyReleaseIntent::PendingTransfer);
    ok &= expectTrue("pending transfer preserves static/system-owned restoration", staticPending.entries.front().restoreMotion);

    return ok ? 0 : 1;
}
