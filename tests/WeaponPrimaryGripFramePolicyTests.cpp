#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/WeaponPrimaryGripFrame.h"

#include <cmath>
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

    template <class T>
    bool expectEqual(const char* label, T actual, T expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s value mismatch\n", label);
        return false;
    }

    bool expectNear(const char* label, float actual, float expected, float tolerance = 0.001f)
    {
        if (std::isfinite(actual) && std::fabs(actual - expected) <= tolerance) {
            return true;
        }
        std::printf("%s expected %.3f got %.3f\n", label, expected, actual);
        return false;
    }

    RE::NiTransform identityTransform()
    {
        return rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    }
}

int main()
{
    namespace primary_grip = rock::weapon_primary_grip_frame_policy;

    bool ok = true;

    auto weaponOffset = identityTransform();
    weaponOffset.translate = RE::NiPoint3{ 10.0f, 2.0f, -3.0f };
    const auto frameFromOffset = primary_grip::primaryGripFrameFromWeaponOffset(
        weaponOffset,
        primary_grip::PrimaryGripFrameSource::FrikWeaponOffset,
        "offset");
    ok &= expectTrue("valid FRIK weapon offset resolves", frameFromOffset.valid);
    ok &= expectEqual("explicit offset source is preserved", frameFromOffset.source, primary_grip::PrimaryGripFrameSource::FrikWeaponOffset);
    ok &= expectNear("inverse offset x becomes weapon-local grip x", frameFromOffset.weaponLocalFrame.translate.x, -10.0f);
    ok &= expectNear("inverse offset y becomes weapon-local grip y", frameFromOffset.weaponLocalFrame.translate.y, -2.0f);
    ok &= expectNear("inverse offset z becomes weapon-local grip z", frameFromOffset.weaponLocalFrame.translate.z, 3.0f);

    auto movedWeapon = identityTransform();
    movedWeapon.translate = RE::NiPoint3{ 100.0f, 50.0f, 25.0f };
    const RE::NiPoint3 movedGripWorld = rock::transform_math::localPointToWorld(movedWeapon, frameFromOffset.weaponLocalFrame.translate);
    ok &= expectNear("weapon-relative grip follows moved weapon x", movedGripWorld.x, 90.0f);
    ok &= expectNear("weapon-relative grip follows moved weapon y", movedGripWorld.y, 48.0f);
    ok &= expectNear("weapon-relative grip follows moved weapon z", movedGripWorld.z, 28.0f);

    auto handMovedWeapon = movedWeapon;
    const RE::NiPoint3 unchangedGripWorld = rock::transform_math::localPointToWorld(handMovedWeapon, frameFromOffset.weaponLocalFrame.translate);
    ok &= expectNear("hand movement does not change weapon-relative target x", unchangedGripWorld.x, movedGripWorld.x);
    ok &= expectNear("hand movement does not change weapon-relative target y", unchangedGripWorld.y, movedGripWorld.y);
    ok &= expectNear("hand movement does not change weapon-relative target z", unchangedGripWorld.z, movedGripWorld.z);

    auto invalidOffset = identityTransform();
    invalidOffset.scale = 0.0f;
    const auto invalidFrame = primary_grip::primaryGripFrameFromWeaponOffset(
        invalidOffset,
        primary_grip::PrimaryGripFrameSource::FrikWeaponOffset,
        "offset");
    ok &= expectFalse("invalid offset fails closed", invalidFrame.valid);

    auto baselineOffset = identityTransform();
    baselineOffset.translate = RE::NiPoint3{ -4.0f, 0.0f, 1.0f };
    const auto frameFromBaseline = primary_grip::primaryGripFrameFromWeaponOffset(
        baselineOffset,
        primary_grip::PrimaryGripFrameSource::WeaponNodeLocalBaseline,
        "weapon-node-local-baseline");
    ok &= expectTrue("finite weapon-node local baseline resolves", frameFromBaseline.valid);
    ok &= expectEqual("baseline source is preserved", frameFromBaseline.source, primary_grip::PrimaryGripFrameSource::WeaponNodeLocalBaseline);
    ok &= expectNear("baseline inverse produces weapon-local grip x", frameFromBaseline.weaponLocalFrame.translate.x, 4.0f);
    ok &= expectNear("baseline inverse produces weapon-local grip z", frameFromBaseline.weaponLocalFrame.translate.z, -1.0f);

    ok &= expectTrue("configured FRIK offset reason is explicit", primary_grip::isExplicitFrikWeaponOffsetReason("offset"));
    ok &= expectFalse("default weapon-node local reason is not explicit FRIK offset", primary_grip::isExplicitFrikWeaponOffsetReason("defaultWeaponNodeLocal"));

    ok &= expectEqual("detached away from firing grip routes free hand",
        primary_grip::resolveDetachedPrimaryGripRoute(primary_grip::DetachedPrimaryReattachInput{
            .primaryDetached = true,
            .weaponGenerationCurrent = true,
            .firingGripResolved = true,
            .supportHandStillOwnsWeapon = true,
            .primaryGripHeld = true,
            .distanceToFiringGripGameUnits = 12.0f,
        }),
        primary_grip::DetachedPrimaryGripRoute::FreeHand);
    ok &= expectEqual("detached inside enter radius with grip held reattaches",
        primary_grip::resolveDetachedPrimaryGripRoute(primary_grip::DetachedPrimaryReattachInput{
            .primaryDetached = true,
            .weaponGenerationCurrent = true,
            .firingGripResolved = true,
            .supportHandStillOwnsWeapon = true,
            .primaryGripHeld = true,
            .distanceToFiringGripGameUnits = 3.5f,
        }),
        primary_grip::DetachedPrimaryGripRoute::ReattachNow);
    ok &= expectEqual("detached normal grab owner never reattaches",
        primary_grip::resolveDetachedPrimaryGripRoute(primary_grip::DetachedPrimaryReattachInput{
            .primaryDetached = true,
            .weaponGenerationCurrent = true,
            .firingGripResolved = true,
            .primaryHandHasNormalGrabOwner = true,
            .supportHandStillOwnsWeapon = true,
            .primaryGripHeld = true,
            .distanceToFiringGripGameUnits = 1.0f,
        }),
        primary_grip::DetachedPrimaryGripRoute::FreeHand);
    ok &= expectEqual("unresolved firing grip never reattaches",
        primary_grip::resolveDetachedPrimaryGripRoute(primary_grip::DetachedPrimaryReattachInput{
            .primaryDetached = true,
            .weaponGenerationCurrent = true,
            .firingGripResolved = false,
            .supportHandStillOwnsWeapon = true,
            .primaryGripHeld = true,
            .distanceToFiringGripGameUnits = 1.0f,
        }),
        primary_grip::DetachedPrimaryGripRoute::FreeHand);
    ok &= expectEqual("inside exit radius without grip is only a candidate",
        primary_grip::resolveDetachedPrimaryGripRoute(primary_grip::DetachedPrimaryReattachInput{
            .primaryDetached = true,
            .weaponGenerationCurrent = true,
            .firingGripResolved = true,
            .supportHandStillOwnsWeapon = true,
            .primaryGripHeld = false,
            .distanceToFiringGripGameUnits = 5.0f,
        }),
        primary_grip::DetachedPrimaryGripRoute::ReattachCandidate);

    return ok ? 0 : 1;
}
