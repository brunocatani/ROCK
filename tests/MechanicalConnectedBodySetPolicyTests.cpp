#include "physics-interaction/object/MechanicalConnectedBodySet.h"

#include <cstdio>
#include <string_view>

namespace
{
    rock::object_physics_body_set::ObjectPhysicsBodyRecord makeAccepted(std::uint32_t bodyId, std::uint32_t motionId)
    {
        rock::object_physics_body_set::ObjectPhysicsBodyRecord record{};
        record.bodyId = bodyId;
        record.motionId = motionId;
        record.accepted = true;
        return record;
    }

    bool expectKind(
        const char* label,
        rock::mechanical_connected_body_set::ScopeKind actual,
        rock::mechanical_connected_body_set::ScopeKind expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected kind=%s got kind=%s\n",
            label,
            rock::mechanical_connected_body_set::scopeKindName(expected),
            rock::mechanical_connected_body_set::scopeKindName(actual));
        return false;
    }

    bool expectMode(
        const char* label,
        rock::held_object_drive_policy::HeldBodySetDriveMode actual,
        rock::held_object_drive_policy::HeldBodySetDriveMode expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected mode=%s got mode=%s\n",
            label,
            rock::held_object_drive_policy::modeName(expected),
            rock::held_object_drive_policy::modeName(actual));
        return false;
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

    bool expectSize(const char* label, std::size_t actual, std::size_t expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected size=%zu got size=%zu\n", label, expected, actual);
        return false;
    }

    bool expectReason(const char* label, const char* actual, std::string_view expected)
    {
        if (actual && std::string_view(actual) == expected) {
            return true;
        }
        std::printf("%s expected reason=%.*s got reason=%s\n", label, static_cast<int>(expected.size()), expected.data(), actual ? actual : "null");
        return false;
    }
}

int main()
{
    using namespace rock::mechanical_connected_body_set;
    using rock::grab_target::Kind;
    using rock::held_object_drive_policy::HeldBodySetDriveMode;
    using rock::object_physics_body_set::ObjectPhysicsBodySet;
    using rock::physics_body_classifier::BodyRejectReason;

    bool ok = true;

    ObjectPhysicsBodySet before{};

    ObjectPhysicsBodySet deadActor{};
    deadActor.records.push_back(makeAccepted(10u, 1u));
    const auto deadActorScope = buildFromPreparedBodySet(before, deadActor, 10u, Kind::DeadActorBody, false);
    ok &= expectKind("dead actor is mechanical even with one accepted body", deadActorScope.kind, ScopeKind::ArticulatedMechanical);
    ok &= expectMode("dead actor drive stays single dynamic", deadActorScope.driveDecision.mode, HeldBodySetDriveMode::SingleDynamic);
    ok &= expectTrue("dead actor relaxes strict pocket authority", deadActorScope.strictPocketAuthorityRelaxed);
    ok &= expectSize("dead actor committed body count", deadActorScope.committedBodyIds.size(), 1u);
    ok &= expectReason("dead actor scope reason", deadActorScope.reason, "dead-actor-mechanical-scope");

    ObjectPhysicsBodySet movableStatic{};
    movableStatic.records.push_back(makeAccepted(20u, 2u));
    movableStatic.records.push_back(makeAccepted(21u, 3u));
    const auto movableStaticScope = buildFromPreparedBodySet(before, movableStatic, 20u, Kind::DynamicMovableStatic, false);
    ok &= expectKind("movable static multi-motion is articulated", movableStaticScope.kind, ScopeKind::ArticulatedMechanical);
    ok &= expectMode("movable static drive is complex", movableStaticScope.driveDecision.mode, HeldBodySetDriveMode::ComplexArticulated);
    ok &= expectFalse("movable static keeps strict authority", movableStaticScope.strictPocketAuthorityRelaxed);
    ok &= expectFalse("complex movable static narrows angular scope", movableStaticScope.driveDecision.includeConnectedAngularVelocity);

    ObjectPhysicsBodySet connectedLoose{};
    connectedLoose.records.push_back(makeAccepted(30u, 4u));
    connectedLoose.records.push_back(makeAccepted(31u, 4u));
    const auto connectedLooseScope = buildFromPreparedBodySet(before, connectedLoose, 30u, Kind::LooseObject, false);
    ok &= expectKind("same-motion loose set is connected", connectedLooseScope.kind, ScopeKind::RefTreeConnectedDynamic);
    ok &= expectMode("same-motion loose drive mode", connectedLooseScope.driveDecision.mode, HeldBodySetDriveMode::ConnectedDynamic);

    ObjectPhysicsBodySet fixedAttached{};
    fixedAttached.records.push_back(makeAccepted(40u, 5u));
    fixedAttached.diagnostics.rejectCounts[static_cast<std::size_t>(BodyRejectReason::StaticMotion)] = 1u;
    const auto fixedScope = buildFromPreparedBodySet(before, fixedAttached, 40u, Kind::DynamicMovableStatic, false);
    ok &= expectKind("fixed attached scope", fixedScope.kind, ScopeKind::FixedAttached);
    ok &= expectMode("fixed attached drive mode", fixedScope.driveDecision.mode, HeldBodySetDriveMode::FixedAttached);
    ok &= expectFalse("fixed attached narrows linear", fixedScope.driveDecision.includeConnectedLinearVelocity);

    ObjectPhysicsBodySet failedScan{};
    failedScan.records.push_back(makeAccepted(50u, 6u));
    failedScan.diagnostics.scanFailures = 1u;
    const auto failedScope = buildFromPreparedBodySet(before, failedScan, 50u, Kind::DeadActorBody, false);
    ok &= expectKind("failed scan is incomplete", failedScope.kind, ScopeKind::IncompleteDiscovery);
    ok &= expectMode("failed scan drive mode", failedScope.driveDecision.mode, HeldBodySetDriveMode::IncompleteNativeScan);
    ok &= expectTrue("failed scan marks incomplete", failedScope.incompleteDiscovery);

    return ok ? 0 : 1;
}
