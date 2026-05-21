#include "physics-interaction/grab/GrabHeldObject.h"

#include <cstdio>
#include <string_view>

namespace
{
    bool expectMode(
        const char* label,
        rock::held_object_drive_policy::HeldBodySetDriveMode actual,
        rock::held_object_drive_policy::HeldBodySetDriveMode expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected %s got %s\n",
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

    bool expectNear(const char* label, float actual, float expected, float epsilon)
    {
        const float delta = actual > expected ? actual - expected : expected - actual;
        if (delta <= epsilon) {
            return true;
        }
        std::printf("%s expected %.4f got %.4f\n", label, expected, actual);
        return false;
    }

    bool expectReason(const char* label, const char* actual, std::string_view expected)
    {
        if (actual && std::string_view(actual) == expected) {
            return true;
        }
        std::printf("%s expected %.*s got %s\n", label, static_cast<int>(expected.size()), expected.data(), actual ? actual : "null");
        return false;
    }
}

int main()
{
    using namespace rock::held_object_drive_policy;

    bool ok = true;

    const auto single = evaluateHeldBodySetDrive(HeldBodySetDriveInput{
        .acceptedBodyCount = 1,
        .uniqueMotionCount = 1,
    });
    ok &= expectMode("single dynamic mode", single.mode, HeldBodySetDriveMode::SingleDynamic);
    ok &= expectNear("single dynamic full force", single.motorAuthorityScale, 1.0f, 0.001f);
    ok &= expectTrue("single dynamic linear scope harmlessly permits body set", single.includeConnectedLinearVelocity);
    ok &= expectTrue("single dynamic angular scope harmlessly permits body set", single.includeConnectedAngularVelocity);
    ok &= expectTrue("single dynamic mass scope harmlessly permits body set", single.includeConnectedMass);

    const auto connectedRigid = evaluateHeldBodySetDrive(HeldBodySetDriveInput{
        .acceptedBodyCount = 4,
        .uniqueMotionCount = 1,
    });
    ok &= expectMode("connected rigid mode", connectedRigid.mode, HeldBodySetDriveMode::ConnectedDynamic);
    ok &= expectTrue("connected rigid keeps linear velocity body set", connectedRigid.includeConnectedLinearVelocity);
    ok &= expectTrue("connected rigid keeps angular velocity body set", connectedRigid.includeConnectedAngularVelocity);
    ok &= expectTrue("connected rigid keeps aggregate mass", connectedRigid.includeConnectedMass);

    const auto complex = evaluateHeldBodySetDrive(HeldBodySetDriveInput{
        .acceptedBodyCount = 4,
        .uniqueMotionCount = 3,
    });
    ok &= expectMode("complex articulated mode", complex.mode, HeldBodySetDriveMode::ComplexArticulated);
    ok &= expectNear("complex articulated softens motor force", complex.motorAuthorityScale, 0.80f, 0.001f);
    ok &= expectTrue("complex articulated keeps linear release scope", complex.includeConnectedLinearVelocity);
    ok &= expectFalse("complex articulated narrows angular assist scope", complex.includeConnectedAngularVelocity);
    ok &= expectTrue("complex articulated keeps aggregate mass", complex.includeConnectedMass);

    const auto fixedAttached = evaluateHeldBodySetDrive(HeldBodySetDriveInput{
        .acceptedBodyCount = 2,
        .uniqueMotionCount = 2,
        .rejectedFixedOrNonDynamicCount = 1,
    });
    ok &= expectMode("fixed attached mode", fixedAttached.mode, HeldBodySetDriveMode::FixedAttached);
    ok &= expectNear("fixed attached lowers force", fixedAttached.motorAuthorityScale, 0.65f, 0.001f);
    ok &= expectFalse("fixed attached narrows linear velocity writes", fixedAttached.includeConnectedLinearVelocity);
    ok &= expectFalse("fixed attached narrows angular velocity writes", fixedAttached.includeConnectedAngularVelocity);
    ok &= expectFalse("fixed attached narrows mass budget", fixedAttached.includeConnectedMass);
    ok &= expectReason("fixed attached reason", fixedAttached.reason, "fixed-or-nondynamic-body");

    const auto incomplete = evaluateHeldBodySetDrive(HeldBodySetDriveInput{
        .acceptedBodyCount = 3,
        .uniqueMotionCount = 2,
        .scanFailureCount = 1,
    });
    ok &= expectMode("incomplete scan mode", incomplete.mode, HeldBodySetDriveMode::IncompleteNativeScan);
    ok &= expectNear("incomplete scan lowers force", incomplete.motorAuthorityScale, 0.55f, 0.001f);
    ok &= expectFalse("incomplete scan narrows linear velocity writes", incomplete.includeConnectedLinearVelocity);
    ok &= expectFalse("incomplete scan narrows angular velocity writes", incomplete.includeConnectedAngularVelocity);
    ok &= expectFalse("incomplete scan narrows mass budget", incomplete.includeConnectedMass);

    const auto noAccepted = evaluateHeldBodySetDrive(HeldBodySetDriveInput{
        .acceptedBodyCount = 0,
        .uniqueMotionCount = 0,
    });
    ok &= expectMode("no accepted body fails closed", noAccepted.mode, HeldBodySetDriveMode::IncompleteNativeScan);
    ok &= expectReason("no accepted body reason", noAccepted.reason, "no-accepted-dynamic-body");
    ok &= expectFalse("no accepted narrows linear velocity writes", noAccepted.includeConnectedLinearVelocity);

    ok &= expectNear("shared and fixed force scales compose",
        combineMotorAuthorityScale(0.5f, fixedAttached),
        0.325f,
        0.001f);
    ok &= expectNear("invalid base force scale clamps",
        combineMotorAuthorityScale(-1.0f, complex),
        0.80f,
        0.001f);

    return ok ? 0 : 1;
}
