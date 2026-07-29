#include "physics-interaction/weapon/EquippedWeaponDropMomentum.h"

#include <cmath>
#include <cstdio>

namespace
{
    struct Vec3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

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
        const float delta = std::fabs(actual - expected);
        if (delta <= epsilon) {
            return true;
        }

        std::printf("%s expected %.5f got %.5f\n", label, expected, actual);
        return false;
    }
}

int main()
{
    using namespace rock::equipped_weapon_drop_momentum;

    bool ok = true;

    HandMotionHistory<Vec3> history{};
    const ReleaseVelocitySettings settings{
        .controllerDerivedEnabled = true,
        .throwMultiplier = 2.0f,
        .maxLinearVelocityHavok = 100.0f,
        .angularVelocityScale = 0.5f,
        .maxAngularVelocityRadiansPerSecond = 10.0f,
    };

    const auto emptyRelease = composeReleaseVelocity(history, Vec3{ 1.0f, 0.0f, 0.0f }, settings);
    ok &= expectFalse("empty history has no release data", emptyRelease.hasData);

    history.push(Vec3{ 5.0f, 0.0f, 0.0f }, Vec3{ 0.0f, 2.0f, 0.0f });
    const auto singleRelease = composeReleaseVelocity(history, Vec3{ 1.0f, 0.0f, 0.0f }, settings);
    ok &= expectTrue("single sample produces release data", singleRelease.hasData);
    ok &= expectNear("player velocity plus multiplied hand velocity", singleRelease.linearVelocityHavok.x, 11.0f, 0.001f);
    ok &= expectNear("angular velocity scaled", singleRelease.angularVelocityRadiansPerSecond.y, 1.0f, 0.001f);

    history.reset();
    const auto resetRelease = composeReleaseVelocity(history, Vec3{}, settings);
    ok &= expectFalse("reset clears release data", resetRelease.hasData);

    // Ring overflow: capacity+2 pushes drop the oldest samples; an interior
    // peak of the surviving window smooths with its neighbors
    // (maxMagnitudeVelocity semantics: [1x6,3,3] peaks at index 6 -> avg of
    // 1,3,3).
    for (std::size_t i = 0; i < kHandMotionHistoryCapacity; ++i) {
        history.push(Vec3{ 1.0f, 0.0f, 0.0f }, Vec3{});
    }
    history.push(Vec3{ 3.0f, 0.0f, 0.0f }, Vec3{});
    history.push(Vec3{ 3.0f, 0.0f, 0.0f }, Vec3{});
    const ReleaseVelocitySettings plainSettings{
        .controllerDerivedEnabled = true,
        .throwMultiplier = 1.0f,
        .maxLinearVelocityHavok = 100.0f,
        .angularVelocityScale = 1.0f,
        .maxAngularVelocityRadiansPerSecond = 10.0f,
    };
    const auto smoothedRelease = composeReleaseVelocity(history, Vec3{}, plainSettings);
    ok &= expectNear("interior window peak smooths with neighbors", smoothedRelease.linearVelocityHavok.x, 7.0f / 3.0f, 0.001f);

    // A release-frame peak (newest sample) is used exactly.
    history.reset();
    history.push(Vec3{ 2.0f, 0.0f, 0.0f }, Vec3{});
    history.push(Vec3{ 4.0f, 0.0f, 0.0f }, Vec3{});
    const auto edgePeakRelease = composeReleaseVelocity(history, Vec3{}, plainSettings);
    ok &= expectNear("release-frame peak drives release exactly", edgePeakRelease.linearVelocityHavok.x, 4.0f, 0.001f);

    // Caps: hand velocity clamps to maxLinearVelocityHavok, angular clamps to
    // maxAngularVelocityRadiansPerSecond.
    history.reset();
    history.push(Vec3{ 50.0f, 0.0f, 0.0f }, Vec3{ 0.0f, 0.0f, 40.0f });
    const ReleaseVelocitySettings cappedSettings{
        .controllerDerivedEnabled = true,
        .throwMultiplier = 1.0f,
        .maxLinearVelocityHavok = 12.0f,
        .angularVelocityScale = 1.0f,
        .maxAngularVelocityRadiansPerSecond = 18.0f,
    };
    const auto cappedRelease = composeReleaseVelocity(history, Vec3{}, cappedSettings);
    ok &= expectNear("linear velocity clamps to cap", cappedRelease.linearVelocityHavok.x, 12.0f, 0.001f);
    ok &= expectNear("angular velocity clamps to cap", cappedRelease.angularVelocityRadiansPerSecond.z, 18.0f, 0.001f);

    history.reset();
    history.push(Vec3{}, Vec3{ 0.0f, 0.0f, 40.0f });
    auto longWeaponSettings = cappedSettings;
    longWeaponSettings.longObjectAngularScalingEnabled = true;
    longWeaponSettings.longObjectLeverGameUnits = 96.0f;
    longWeaponSettings.longObjectReferenceLeverGameUnits = 24.0f;
    longWeaponSettings.longObjectMinAngularScale = 0.35f;
    const auto longWeaponRelease = composeReleaseVelocity(history, Vec3{}, longWeaponSettings);
    ok &= expectNear("long weapon uses configured angular floor scale", longWeaponRelease.longObjectAngularScale, 0.35f, 0.001f);
    ok &= expectNear("long weapon angular cap follows held-object release policy", longWeaponRelease.angularVelocityCapRadiansPerSecond, 6.3f, 0.001f);
    ok &= expectNear("long weapon angular velocity is lever capped", longWeaponRelease.angularVelocityRadiansPerSecond.z, 6.3f, 0.001f);

    ok &= expectFalse("discovery sequence cannot settle itself", completedSettleStep(42, 42));
    ok &= expectTrue("later solve sequence completes settling", completedSettleStep(42, 43));
    ok &= expectFalse("publication budget does not expire before its solve bound", publicationProgressStalled(10, 189, 180));
    ok &= expectTrue("publication budget expires at its solve bound", publicationProgressStalled(10, 190, 180));
    ok &= expectFalse("paused or reset solve sequence cannot expire publication", publicationProgressStalled(10, 9, 180));
    ok &= expectFalse("zero publication budget disables expiry", publicationProgressStalled(10, 1000, 0));

    const BodyIdentityKey bodyIdentity{
        .bodyId = 11,
        .motionId = 7,
        .motionFirstBodyId = 11,
        .shapeIdentity = 0x1000,
        .owningNodeIdentity = 0x2000,
        .collisionObjectIdentity = 0x3000,
        .physicsSystemInstanceIdentity = 0x4000,
    };
    ok &= expectTrue("identical native body generation is stable", sameBodyIdentity(bodyIdentity, bodyIdentity));
    auto rebuiltIdentity = bodyIdentity;
    rebuiltIdentity.physicsSystemInstanceIdentity = 0x5000;
    ok &= expectFalse("reused body and motion IDs do not hide a native rebuild", sameBodyIdentity(bodyIdentity, rebuiltIdentity));
    rebuiltIdentity = bodyIdentity;
    rebuiltIdentity.motionId = 8;
    ok &= expectFalse("motion changes invalidate native identity", sameBodyIdentity(bodyIdentity, rebuiltIdentity));
    rebuiltIdentity = bodyIdentity;
    rebuiltIdentity.shapeIdentity = 0x1100;
    ok &= expectFalse("shape changes invalidate native identity", sameBodyIdentity(bodyIdentity, rebuiltIdentity));
    rebuiltIdentity = bodyIdentity;
    rebuiltIdentity.owningNodeIdentity = 0x2100;
    ok &= expectFalse("owner-node changes invalidate native identity", sameBodyIdentity(bodyIdentity, rebuiltIdentity));
    rebuiltIdentity = bodyIdentity;
    rebuiltIdentity.collisionObjectIdentity = 0x3100;
    ok &= expectFalse("collision-object changes invalidate native identity", sameBodyIdentity(bodyIdentity, rebuiltIdentity));

    // Disabled controller-derived throw keeps only player velocity and drops
    // angular momentum, matching the held-object release path.
    history.reset();
    history.push(Vec3{ 5.0f, 0.0f, 0.0f }, Vec3{ 0.0f, 2.0f, 0.0f });
    const ReleaseVelocitySettings disabledSettings{
        .controllerDerivedEnabled = false,
        .throwMultiplier = 2.0f,
        .maxLinearVelocityHavok = 100.0f,
        .angularVelocityScale = 1.0f,
        .maxAngularVelocityRadiansPerSecond = 10.0f,
    };
    const auto disabledRelease = composeReleaseVelocity(history, Vec3{ 1.0f, 0.0f, 0.0f }, disabledSettings);
    ok &= expectTrue("disabled controller-derived throw still reports data", disabledRelease.hasData);
    ok &= expectNear("disabled controller-derived throw keeps player velocity", disabledRelease.linearVelocityHavok.x, 1.0f, 0.001f);
    ok &= expectNear("disabled controller-derived throw drops angular momentum", disabledRelease.angularVelocityRadiansPerSecond.y, 0.0f, 0.001f);

    if (!ok) {
        std::printf("EquippedWeaponDropMomentumPolicyTests failed\n");
        return 1;
    }

    std::printf("EquippedWeaponDropMomentumPolicyTests passed\n");
    return 0;
}
