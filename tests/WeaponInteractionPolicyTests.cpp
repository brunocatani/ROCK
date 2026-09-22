#include "physics-interaction/collision/ContactPipelinePolicy.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/hand/HandInteractionStateMachine.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/weapon/EquippedWeaponDropPolicy.h"
#include "physics-interaction/weapon/EquipVisualBridgePolicy.h"
#include "physics-interaction/weapon/EquippedWeaponHandlingSettings.h"
#include "physics-interaction/weapon/AuthoredSupportGrabPolicy.h"
#include "physics-interaction/weapon/EquippedWeaponToggleGrabPolicy.h"
#include "physics-interaction/weapon/FiringGripReattachZonePolicy.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponInteraction.h"
#include "physics-interaction/weapon/WeaponAccessoryPartKindPolicy.h"
#include "physics-interaction/weapon/WeaponPartGripReportPolicy.h"
#include "physics-interaction/weapon/WeaponPartRecordIdentityPolicy.h"
#include "physics-interaction/weapon/WeaponPartRuntime.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/recoil/RecoilController.h"
#include "physics-interaction/weapon/WeaponTypePolicy.h"
#include "physics-interaction/weapon/NativeScopeSightAnchorPolicy.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <limits>

namespace
{
    namespace toggle_grab =
        rock::equipped_weapon_toggle_grab_policy;

    enum class TestWeaponType
    {
        kHandToHand = 0,
        kOneHandSword = 1,
        kOneHandDagger = 2,
        kOneHandAxe = 3,
        kOneHandMace = 4,
        kTwoHandSword = 5,
        kTwoHandAxe = 6,
        kBow = 7,
        kStaff = 8,
        kGun = 9,
        kGrenade = 10,
        kMine = 11,
    };

    struct TestVector3
    {
        float x{ 0.0f };
        float y{ 0.0f };
        float z{ 0.0f };
    };

    struct TestMatrix3
    {
        float entry[3][4]{};
    };

    struct TestTransform
    {
        TestMatrix3 rotate{};
        TestVector3 translate{};
        float scale{ 1.0f };
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

    template <class T>
    bool expectEqual(const char* label, T actual, T expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected %llu got %llu\n", label, static_cast<unsigned long long>(expected), static_cast<unsigned long long>(actual));
        return false;
    }

    bool expectNear(const char* label, float actual, float expected, float tolerance = 0.0001f)
    {
        if (std::fabs(actual - expected) <= tolerance) {
            return true;
        }

        std::printf("%s expected %.6f got %.6f\n", label, expected, actual);
        return false;
    }

    bool expectTransformNear(const char* label, const TestTransform& actual, const TestTransform& expected)
    {
        bool ok = true;
        ok &= expectNear(label, actual.translate.x, expected.translate.x);
        ok &= expectNear(label, actual.translate.y, expected.translate.y);
        ok &= expectNear(label, actual.translate.z, expected.translate.z);
        ok &= expectNear(label, actual.scale, expected.scale);
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                ok &= expectNear(label, actual.rotate.entry[row][column], expected.rotate.entry[row][column]);
            }
        }
        return ok;
    }

    bool expectVectorNear(
        const char* label,
        const TestVector3& actual,
        const TestVector3& expected,
        float tolerance = 0.0001f)
    {
        bool ok = true;
        ok &= expectNear(label, actual.x, expected.x, tolerance);
        ok &= expectNear(label, actual.y, expected.y, tolerance);
        ok &= expectNear(label, actual.z, expected.z, tolerance);
        return ok;
    }

    TestMatrix3 makeAxisAngleRotation(
        const TestVector3& axis,
        float degrees)
    {
        return rock::weaponSolverAxisAngleStored<
            TestMatrix3,
            TestVector3>(
            axis,
            degrees * 0.01745329251994329577f);
    }
}

static bool testScopeRollCalibration()
{
    namespace scope = rock::native_scope_camera_follow_math;
    namespace math = rock::transform_math;
    bool ok = true;
    auto cameraBasis = math::makeIdentityTransform<TestTransform>();
    cameraBasis.rotate = {};
    cameraBasis.rotate.entry[0][1] = 1.0f;
    cameraBasis.rotate.entry[1][2] = 1.0f;
    cameraBasis.rotate.entry[2][0] = 1.0f;
    cameraBasis.scale = 0.75f;
    const TestVector3 anchor{ 1.2f, 8.0f, 11.0f };
    auto expectedLocal = cameraBasis;
    expectedLocal.translate = anchor;

    // The same optical direction can arrive with either sign of controller
    // twist. Each equip must retain the same camera-to-weapon registration.
    for (float entryYaw : { -175.0f, -40.0f, 0.0f, 95.0f }) {
        auto entryWeapon = rock::native_scope_rotation_math::makePitchYawRollLocal<TestTransform>(78.0f, entryYaw, 23.0f);
        entryWeapon.translate = { -1300.0f, 2300.0f, 800.0f };
        entryWeapon.scale = 1.2f;
        for (float entryRoll : { -170.0f, -35.0f, -7.0f, 0.0f, 12.0f, 90.0f, 175.0f }) {
            auto twist = math::makeIdentityTransform<TestTransform>();
            twist.rotate = makeAxisAngleRotation(TestVector3{ 1.0f, 0.0f, 0.0f }, entryRoll);
            const auto nativeLocal = math::composeTransforms(cameraBasis, twist);
            const auto nativeWorld = math::composeTransforms(entryWeapon, nativeLocal);
            TestTransform captured{};
            ok &= expectTrue("scope roll capture accepts either hand's entry orientation",
                scope::tryCaptureRigidAnchorFrameWeaponLocal(entryWeapon, nativeWorld, anchor, captured));
            ok &= expectTransformNear("entry pose cannot become permanent scope roll", captured, expectedLocal);
            ok &= expectNear("roll diagnostic distinguishes aligned-forward twists",
                scope::weaponLocalCameraRollDegrees(nativeLocal.rotate), entryRoll);
            ok &= expectNear("retained scope frame has zero relative roll",
                scope::weaponLocalCameraRollDegrees(captured.rotate), 0.0f);
            for (float finalCant : { -65.0f, 0.0f, 42.0f }) {
                auto finalWeapon = rock::native_scope_rotation_math::makePitchYawRollLocal<TestTransform>(-35.0f, 110.0f, finalCant);
                finalWeapon.translate = { 900.0f, -850.0f, 1100.0f };
                finalWeapon.scale = 0.8f;
                const auto camera = scope::resolveRigidAnchorFrameWorld(finalWeapon, captured);
                ok &= expectTransformNear("scope follows final weapon pose independent of entry",
                    camera, math::composeTransforms(finalWeapon, expectedLocal));
                const auto overlay = rock::native_scope_overlay_follow_math::resolveScopeModelRootWorld(
                    camera, math::invertTransform(cameraBasis), math::makeIdentityTransform<TestTransform>());
                for (int column = 0; column < 3; ++column) {
                    ok &= expectNear("intentional weapon cant also cants scope markings",
                        camera.rotate.entry[1][column], finalWeapon.rotate.entry[2][column]);
                    ok &= expectNear("optical forward still follows barrel",
                        camera.rotate.entry[0][column], finalWeapon.rotate.entry[1][column]);
                    ok &= expectNear("calibrated reticle horizontal follows weapon horizontal",
                        overlay.rotate.entry[0][column], finalWeapon.rotate.entry[0][column]);
                    ok &= expectNear("calibrated reticle vertical follows weapon vertical",
                        overlay.rotate.entry[2][column], finalWeapon.rotate.entry[2][column]);
                }
            }
        }
    }

    const auto identity = math::makeIdentityTransform<TestTransform>();
    const auto tiltedCamera = scope::applyWeaponLocalRotationOffset(cameraBasis, 3.0f, -7.0f, 28.0f);
    TestTransform captured{};
    ok &= expectTrue("slightly offset optical direction remains usable",
        scope::tryCaptureRigidAnchorFrameWeaponLocal(identity, tiltedCamera, anchor, captured));
    for (int column = 0; column < 3; ++column) {
        ok &= expectNear("roll repair preserves sampled optical direction",
            captured.rotate.entry[0][column], tiltedCamera.rotate.entry[0][column]);
    }
    ok &= expectNear("offset optical direction has no residual twist", scope::weaponLocalCameraRollDegrees(captured.rotate), 0.0f);
    ok &= expectNear("scope rotation remains orthonormal", static_cast<float>(math::storedRotationOrthonormalityError(captured.rotate)), 0.0f);

    auto invalidCamera = cameraBasis;
    invalidCamera.rotate.entry[0][1] = 0.0f;
    ok &= expectFalse("zero optical direction fails closed", scope::tryCaptureRigidAnchorFrameWeaponLocal(identity, invalidCamera, anchor, captured));
    ok &= expectNear("rejected capture cannot be used as a valid frame", captured.scale, 0.0f);
    invalidCamera.rotate.entry[0][2] = 1.0f;
    ok &= expectFalse("weapon-up parallel to forward cannot seed a roll", scope::tryCaptureRigidAnchorFrameWeaponLocal(identity, invalidCamera, anchor, captured));
    for (float scale : { 0.0f, -1.0f, std::numeric_limits<float>::infinity() }) {
        auto invalidWeapon = identity;
        invalidWeapon.scale = scale;
        ok &= expectFalse("invalid weapon scale cannot seed a scope frame", scope::tryCaptureRigidAnchorFrameWeaponLocal(invalidWeapon, cameraBasis, anchor, captured));
        invalidCamera = cameraBasis;
        invalidCamera.scale = scale;
        ok &= expectFalse("invalid camera scale cannot seed a scope frame", scope::tryCaptureRigidAnchorFrameWeaponLocal(identity, invalidCamera, anchor, captured));
    }
    invalidCamera = cameraBasis;
    invalidCamera.rotate.entry[1][0] = std::numeric_limits<float>::quiet_NaN();
    ok &= expectFalse("nonfinite native rotation fails closed", scope::tryCaptureRigidAnchorFrameWeaponLocal(identity, invalidCamera, anchor, captured));
    ok &= expectTrue("invalid diagnostic roll is unknown rather than zero", std::isnan(scope::weaponLocalCameraRollDegrees(invalidCamera.rotate)));
    auto invalidAnchor = anchor;
    invalidAnchor.z = std::numeric_limits<float>::infinity();
    ok &= expectFalse("invalid sight anchor fails closed", scope::tryCaptureRigidAnchorFrameWeaponLocal(identity, cameraBasis, invalidAnchor, captured));
    return ok;
}

static bool testRecoilProfiles()
{
    bool ok = true;
    using namespace rock::weapon_recoil_policy;
    using rock::weapon_recoil_authority_math::tryBuildControlledKick;
    ok &= expectTrue("ordinary one hand selects independent profile",
        selectProfile(false, false, false) == Profile::OneHand);
    for (const bool armor : { false, true }) {
        for (const bool twoHands : { false, true }) {
            for (const bool close : { false, true }) {
                ok &= expectTrue("latched bipod overrides all hand and armor profiles",
                    selectProfile(armor, close, twoHands, true) == Profile::Bipod);
                ok &= expectTrue("unlatched bipod restores ordinary profile selection",
                    selectProfile(armor, close, twoHands, false) == selectProfile(armor, close, twoHands));
            }
        }
    }
    for (const float percent : { 0.0f, 80.0f, 100.0f, 292.1f, 300.0f }) {
        const auto bipodGains = effectiveGains(Family::Default, Profile::Bipod, percent);
        ok &= expectTrue("bipod gains are ten percent regardless of weapon tuning",
            bipodGains.translation == 0.10f && bipodGains.rotation == 0.10f);
    }
    ok &= expectTrue("close support chooses its own profile",
        selectProfile(false, true, false) == Profile::CloseSupport);
    for (const bool supported : { false, true }) {
        ok &= expectTrue("armor profile wins independently of support",
            selectProfile(true, supported, false) == Profile::PowerArmor);
    }
    for (const bool nativeLeft : { false, true }) {
        for (const bool firingLeft : { false, true }) {
            const auto nativeMask = deliveryHand(false, firingLeft, nativeLeft);
            ok &= expectTrue("native recoil selects the physical firing hand",
                nativeMask == (firingLeft == nativeLeft ? HandMask::Primary : HandMask::Offhand));
            ok &= expectTrue("owned weapon/solver has no additional FRIK hand kick",
                deliveryHand(true, firingLeft, nativeLeft) == HandMask::None);
        }
    }
    ok &= expectTrue("full two-hand has a separately tunable profile",
        selectProfile(false, false, true) == Profile::FullTwoHand);
    ok &= expectTrue("armor overrides full two-hand profile",
        selectProfile(true, false, true) == Profile::PowerArmor);
    ok &= expectTrue("close support remains distinct from one hand",
        selectProfile(false, true, false) != Profile::OneHand);
    ok &= expectFalse("idle recoil must not acquire weapon or hand", needsOneHandPresentation(false, false));
    ok &= expectTrue("active kick acquires direct presentation", needsOneHandPresentation(true, false));
    ok &= expectTrue("settling publishes one final neutral frame", needsOneHandPresentation(false, true));
    const auto neutralKick = rock::transform_math::makeIdentityTransform<TestTransform>();
    auto distantFrame = neutralKick;
    distantFrame.translate = { -70700.0f, 80000.0f, 7450.0f };
    distantFrame.rotate = makeAxisAngleRotation(TestVector3{ 0.0f, 0.0f, 1.0f }, 27.0f);
    ok &= expectTransformNear("identity recoil stays exact at distant world coordinates",
        rock::weapon_recoil_authority_math::resolveWorldDelta(neutralKick, distantFrame, distantFrame, distantFrame, true),
        neutralKick);
    const auto flags = [](const rock::WeaponKeywordFlag flag) { return static_cast<std::uint64_t>(flag); };
    WeaponEvidence evidence{
        .formID = 0x123u, .keywordFlags = flags(rock::WeaponKeywordFlag::Pistol),
        .sizeClass = rock::WeaponSizeClass::Pistol,
        .source = rock::WeaponClassificationSource::Keyword, .resolved = true,
    };
    ok &= expectTrue("pistol evidence selects pistol multiplier", classifyFamily(evidence) == Family::Pistol);
    evidence.keywordFlags |= flags(rock::WeaponKeywordFlag::Rifle);
    evidence.sizeClass = rock::WeaponSizeClass::Rifle;
    evidence.source = rock::WeaponClassificationSource::EquipSlot;
    ok &= expectTrue("converted pistol respects resolved effective rifle slot", classifyFamily(evidence) == Family::Rifle);
    evidence.keywordFlags |= flags(rock::WeaponKeywordFlag::Shotgun);
    ok &= expectTrue("shotgun family wins over broad rifle tag", classifyFamily(evidence) == Family::Shotgun);
    evidence.sizeClass = rock::WeaponSizeClass::Heavy;
    ok &= expectTrue("heavy family wins over shotgun tag", classifyFamily(evidence) == Family::Heavy);
    evidence.resolved = false;
    ok &= expectTrue("unresolved conflicting evidence uses explicit default", classifyFamily(evidence) == Family::Default);
    evidence.resolved = true;
    evidence.sizeClass = rock::WeaponSizeClass::Rifle;
    evidence.keywordFlags = flags(rock::WeaponKeywordFlag::Rifle) | flags(rock::WeaponKeywordFlag::Laser) |
        flags(rock::WeaponKeywordFlag::Automatic);
    ok &= expectTrue("laser automatic rifle selects laser recoil", classifyFamily(evidence) == Family::Laser);
    for (const auto laser : { rock::WeaponKeywordFlag::Laser, rock::WeaponKeywordFlag::LaserMusket,
             rock::WeaponKeywordFlag::GatlingLaser }) {
        for (const auto size : { rock::WeaponSizeClass::Pistol, rock::WeaponSizeClass::Rifle, rock::WeaponSizeClass::Heavy }) {
            evidence.keywordFlags = flags(laser) | flags(rock::WeaponKeywordFlag::Shotgun);
            evidence.sizeClass = size;
            ok &= expectTrue("laser family wins over grip, shotgun, and heavy classification",
                classifyFamily(evidence) == Family::Laser);
        }
    }
    evidence.resolved = false;
    ok &= expectTrue("unresolved laser evidence remains default", classifyFamily(evidence) == Family::Default);
    evidence.resolved = true;
    evidence.sizeClass = rock::WeaponSizeClass::Melee;
    ok &= expectTrue("melee does not select laser recoil", classifyFamily(evidence) == Family::Default);
    evidence.sizeClass = rock::WeaponSizeClass::Rifle;
    for (const auto other : { rock::WeaponKeywordFlag::Plasma, rock::WeaponKeywordFlag::Ballistic }) {
        evidence.keywordFlags = flags(other) | flags(rock::WeaponKeywordFlag::Rifle);
        ok &= expectTrue("non-laser guns retain rifle recoil", classifyFamily(evidence) == Family::Rifle);
    }
    for (const auto profile : { Profile::OneHand, Profile::FullTwoHand, Profile::CloseSupport }) {
        const auto off = effectiveGains(Family::Default, profile, 0.0f);
        const auto normal = effectiveGains(Family::Default, profile, 100.0f);
        const auto doubleKick = effectiveGains(Family::Default, profile, 200.0f);
        const auto base = gainsFor(profile);
        ok &= expectTrue("zero percent suppresses both kick components", off.translation == 0.0f && off.rotation == 0.0f);
        ok &= expectTrue("100 percent preserves the hold profile", normal.translation == base.translation && normal.rotation == base.rotation);
        ok &= expectTrue("200 percent doubles both hold gains", doubleKick.translation == base.translation * 2.0f && doubleKick.rotation == base.rotation * 2.0f);
    }
    for (const auto profile : { Profile::OneHand, Profile::FullTwoHand, Profile::CloseSupport, Profile::PowerArmor }) {
        const auto laser = effectiveGains(Family::Laser, profile, 100.0f);
        const auto laserOff = effectiveGains(Family::Laser, profile, 0.0f);
        const auto laserDouble = effectiveGains(Family::Laser, profile, 200.0f);
        ok &= expectTrue("laser default matches bipod independently of hold and armor",
            laser.translation == kBipod.translation && laser.rotation == kBipod.rotation);
        ok &= expectTrue("laser strength can suppress both recoil components",
            laserOff.translation == 0.0f && laserOff.rotation == 0.0f);
        ok &= expectTrue("laser strength scales the bipod profile once",
            laserDouble.translation == 2.0f * kBipod.translation && laserDouble.rotation == 2.0f * kBipod.rotation);
    }
    for (const float percent : { 0.0f, 100.0f, 200.0f }) {
        const auto armorGains = effectiveGains(Family::Default, Profile::PowerArmor, percent);
        ok &= expectTrue("armor umbrella ignores weapon percentages",
            armorGains.translation == kPowerArmor.translation && armorGains.rotation == kPowerArmor.rotation);
        const auto laser = effectiveGains(Family::Laser, Profile::Bipod, percent);
        ok &= expectTrue("latched bipod overrides laser strength without stacking reductions",
            laser.translation == kBipod.translation && laser.rotation == kBipod.rotation);
    }
    ok &= expectTrue("one-hand percentage is a direct override, not stacked on family tuning",
        selectHoldPercent(true, 300.0f, 200.0f) == 300.0f);
    ok &= expectTrue("two-hand retains custom low tuning", selectHoldPercent(false, 300.0f, 50.0f) == 50.0f);
    ok &= expectTrue("two-hand retains custom high tuning", selectHoldPercent(false, 300.0f, 200.0f) == 200.0f);
    for (const auto profile : { Profile::FullTwoHand, Profile::CloseSupport }) {
        for (const float currentPercent : { 50.0f, 100.0f, 200.0f }) {
            const auto before = effectiveGains(Family::Default, profile, currentPercent);
            const auto after = effectiveGains(Family::Default, profile, selectHoldPercent(false, 300.0f, currentPercent));
            ok &= expectTrue("two-hand full and close profiles preserve today's response",
                before.translation == after.translation && before.rotation == after.rotation);
        }
    }
    for (const bool oneHanded : { false, true }) {
        const auto unchangedArmor = effectiveGains(Family::Default, Profile::PowerArmor, selectHoldPercent(oneHanded, 300.0f, 200.0f));
        ok &= expectTrue("power armor ignores either hold's percentage",
            unchangedArmor.translation == kPowerArmor.translation && unchangedArmor.rotation == kPowerArmor.rotation);
    }
    TestTransform shot = rock::transform_math::makeIdentityTransform<TestTransform>();
    shot.translate = { 10.0f, -4.0f, 2.0f };
    shot.rotate = makeAxisAngleRotation(TestVector3{ 0.0f, 0.0f, 1.0f }, 60.0f);
    auto modestShot = shot;
    modestShot.rotate = makeAxisAngleRotation(TestVector3{ 0.0f, 0.0f, 1.0f }, 20.0f);
    TestTransform tripleKick{};
    ok &= expectTrue("300 percent builds a valid three-times impulse",
        tryBuildControlledKick(modestShot, effectiveGains(Family::Default, Profile::OneHand, selectHoldPercent(true, 300.0f, 200.0f)), tripleKick));
    auto tripleExpected = rock::transform_math::makeIdentityTransform<TestTransform>();
    tripleExpected.translate = { 30.0f, -12.0f, 6.0f };
    tripleExpected.rotate = makeAxisAngleRotation(TestVector3{ 0.0f, 0.0f, 1.0f }, 60.0f);
    ok &= expectTransformNear("one hand gets 300 percent rather than 600 percent", tripleKick, tripleExpected);
    TestTransform amplified{};
    ok &= expectTrue("200 percent produces a rigid amplified transform",
        tryBuildControlledKick(shot, effectiveGains(Family::Default, Profile::OneHand, 200.0f), amplified));
    auto doubleExpected = rock::transform_math::makeIdentityTransform<TestTransform>();
    doubleExpected.translate = { 20.0f, -8.0f, 4.0f };
    doubleExpected.rotate = makeAxisAngleRotation(TestVector3{ 0.0f, 0.0f, 1.0f }, 120.0f);
    ok &= expectTransformNear("200 percent doubles translation and angular displacement", amplified, doubleExpected);
    ok &= expectTrue("zero percent accepts and neutralizes native recoil",
        tryBuildControlledKick(shot, effectiveGains(Family::Default, Profile::OneHand, 0.0f), amplified));
    ok &= expectTransformNear("zero percent produces identity rather than native fallback", amplified,
        rock::transform_math::makeIdentityTransform<TestTransform>());
    TestTransform armor{};
    ok &= expectTrue("armor builds a rigid reduced kick",
        tryBuildControlledKick(shot, gainsFor(Profile::PowerArmor), armor));
    auto expected = rock::transform_math::makeIdentityTransform<TestTransform>();
    expected.translate = { 4.5f, -1.8f, 0.9f };
    expected.rotate = makeAxisAngleRotation(TestVector3{ 0.0f, 0.0f, 1.0f }, 18.0f);
    ok &= expectTransformNear("armor starts with the requested attenuation", armor, expected);
    TestTransform bipod{};
    ok &= expectTrue("bipod builds a rigid reduced kick",
        tryBuildControlledKick(shot, effectiveGains(Family::Default, Profile::Bipod, 300.0f), bipod));
    auto bipodExpected = rock::transform_math::makeIdentityTransform<TestTransform>();
    bipodExpected.translate = { 1.0f, -0.4f, 0.2f };
    bipodExpected.rotate = makeAxisAngleRotation(TestVector3{ 0.0f, 0.0f, 1.0f }, 6.0f);
    ok &= expectTransformNear("bipod applies ten percent of native translation and angle", bipod, bipodExpected);
    for (const auto profile : { Profile::OneHand, Profile::FullTwoHand, Profile::CloseSupport, Profile::PowerArmor, Profile::Bipod }) {
        TestTransform laser{};
        ok &= expectTrue("laser default builds a valid controlled kick",
            tryBuildControlledKick(shot, effectiveGains(Family::Laser, profile, 100.0f), laser));
        ok &= expectTransformNear("laser kick matches bipod in every hold and armor state", laser, bipod);
    }
    auto independentlyTunedSupport = kCloseSupport;
    independentlyTunedSupport.translation = 0.2f;
    independentlyTunedSupport.rotation = 0.1f;
    TestTransform tuned{};
    ok &= expectTrue("support profile can be independently tuned",
        tryBuildControlledKick(shot, independentlyTunedSupport, tuned));
    ok &= expectTrue("support tuning produces a different impulse",
        std::abs(tuned.translate.x - armor.translate.x) > 1.0f);
    TestTransform unchangedArmor{};
    ok &= expectTrue("armor remains independently selectable",
        tryBuildControlledKick(shot, gainsFor(Profile::PowerArmor), unchangedArmor));
    ok &= expectTransformNear("support tuning leaves armor unchanged", unchangedArmor, armor);
    TestTransform native{};
    ok &= expectTrue("native profile builds an unchanged rigid sample",
        tryBuildControlledKick(shot, gainsFor(Profile::OneHand), native));
    ok &= expectTransformNear("unassisted native sample is preserved", native, shot);
    TestTransform fullTwoHand{};
    ok &= expectTrue("two-hand profile retains unscaled kick",
        tryBuildControlledKick(shot, gainsFor(Profile::FullTwoHand), fullTwoHand));
    ok &= expectTransformNear("one-hand and full two-hand start with identical gains", fullTwoHand, native);
    auto tunedOneHand = kOneHand;
    tunedOneHand.rotation = 0.5f;
    TestTransform oneHandTuned{};
    ok &= expectTrue("one-hand profile accepts independent tuning",
        tryBuildControlledKick(shot, tunedOneHand, oneHandTuned));
    ok &= expectTrue("one-hand tuning leaves full two-hand gains intact",
        gainsFor(Profile::FullTwoHand).rotation == 1.0f);

    // Both baselines (authored and reconstructed native) pass through this
    // operation. The hand must retain its weapon-local seat as recoil changes.
    auto weaponBase = rock::transform_math::makeIdentityTransform<TestTransform>();
    weaponBase.translate = { 12.0f, 28.0f, 7.0f };
    auto handLocal = rock::transform_math::makeIdentityTransform<TestTransform>();
    handLocal.translate = { 2.0f, -3.0f, 0.5f };
    handLocal.rotate = makeAxisAngleRotation(TestVector3{ 1.0f, 0.0f, 0.0f }, 15.0f);
    const auto handBase = rock::transform_math::composeTransforms(weaponBase, handLocal);
    TestTransform weaponTarget{};
    TestTransform handTarget{};
    for (const auto kick : { native, armor,
            rock::transform_math::makeIdentityTransform<TestTransform>() }) {
        rock::weapon_recoil_authority_math::applyOneHandKick(
            kick, weaponBase, handBase, weaponTarget, handTarget);
        ok &= expectTransformNear("direct recoil preserves the firing-hand seat",
            rock::transform_math::composeTransforms(
                rock::transform_math::invertTransform(weaponTarget), handTarget), handLocal);
    }
    ok &= expectTransformNear("identity sample fully removes previous recoil", weaponTarget, weaponBase);
    ok &= expectTransformNear("firing hand returns to its clean baseline", handTarget, handBase);
    auto invalid = shot;
    invalid.rotate.entry[0][0] = 0.0f;
    ok &= expectFalse("non-rigid input cannot become a plausible controlled kick",
        tryBuildControlledKick(invalid, kPowerArmor, tuned));
    ok &= expectTransformNear("invalid input outputs identity", tuned,
        rock::transform_math::makeIdentityTransform<TestTransform>());
    invalid = rock::transform_math::makeIdentityTransform<TestTransform>();
    invalid.rotate.entry[0][0] = -1.0f;
    ok &= expectFalse("reflection is not a rigid recoil rotation",
        tryBuildControlledKick(invalid, kPowerArmor, tuned));

    const SampleIdentity captured{
        .weaponNode = 0x1234, .weaponGeneration = 2, .equippedOwnership = 3,
        .profile = Profile::PowerArmor, .firingHandIsLeft = true,
        .nativePrimaryIsLeft = false, .fullTwoHanded = false,
    };
    SampleTicket ticket{ .identity = captured, .sequence = 1, .valid = true };
    ticket.beginUpdate(true);
    ok &= expectTrue("fresh callback can be consumed", ticket.consume(captured));
    ok &= expectFalse("one sample cannot kick both carry and solver", ticket.consume(captured));
    ticket.beginUpdate(true);
    ok &= expectFalse("skipped callback never replays a shot", ticket.consume(captured));
    for (int changed = 0; changed < 11; ++changed) {
        auto current = captured;
        switch (changed) {
        case 0: ++current.weaponNode; break;
        case 1: ++current.weaponGeneration; break;
        case 2: ++current.equippedOwnership; break;
        case 3: current.profile = Profile::OneHand; break;
        case 4: current.firingHandIsLeft = false; break;
        case 5: current.nativePrimaryIsLeft = true; break;
        case 6: current.fullTwoHanded = true; break;
        case 7: current.oneHanded = true; break;
        case 8: current.familyPercent = 200.0f; break;
        case 9: current.family = Family::Shotgun; break;
        case 10: current.formID = 0x234u; break;
        }
        ++ticket.sequence;
        ticket.valid = true;
        ticket.beginUpdate(true);
        ok &= expectFalse("owner/profile/role/solver changes discard the old shot", ticket.consume(current));
        ok &= expectFalse("rejected ticket cannot replay after identity returns", ticket.consume(captured));
    }
    ++ticket.sequence;
    ticket.valid = true;
    ticket.beginUpdate(true);
    ticket.invalidate();
    ok &= expectFalse("lifecycle reset discards pending recoil", ticket.consume(captured));
    ++ticket.sequence;
    ticket.valid = true;
    ticket.beginUpdate(false);
    ok &= expectFalse("disabled immersive recoil discards a captured kick", ticket.consume(captured));
    ticket.beginUpdate(true);
    ok &= expectFalse("reenabling recoil cannot replay the pre-disable sample", ticket.consume(captured));
    ++ticket.sequence;
    ticket.valid = true;
    ticket.beginUpdate(true);
    ok &= expectTrue("reenabled recoil accepts a fresh callback sample", ticket.consume(captured));
    return ok;
}

static bool testNativeGripFrames()
{
    bool ok = true;
    TestTransform rightNativeWeaponInWand =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    rightNativeWeaponInWand.rotate = makeAxisAngleRotation(
        rock::weaponSolverNormalize(
            TestVector3{ 0.23f, 0.51f, 0.83f }),
        31.0f);
    rightNativeWeaponInWand.translate = { 8.0f, -3.0f, 4.0f };
    rightNativeWeaponInWand.scale = 2.0f;

    const TestTransform leftWeaponInWand =
        rock::left_firing_position_only_math::
            mirrorRightWeaponInWandOrientation(
                rightNativeWeaponInWand);
    const TestVector3 weaponForward{ 0.0f, 1.0f, 0.0f };
    const TestTransform rightNativeWeaponOrientation =
        rock::left_firing_position_only_math::orientationOnly(
            rightNativeWeaponInWand);
    const TestVector3 rightBarrel =
        rock::transform_math::localVectorToWorld(
            rightNativeWeaponOrientation,
            weaponForward);
    const TestVector3 leftBarrel =
        rock::transform_math::localVectorToWorld(
            leftWeaponInWand,
            weaponForward);
    ok &= expectVectorNear(
        "left native weapon mirror negates only barrel lateral component",
        leftBarrel,
        TestVector3{ -rightBarrel.x, rightBarrel.y, rightBarrel.z });
    const TestVector3 rightLateral =
        rock::transform_math::localVectorToWorld(
            rightNativeWeaponOrientation,
            TestVector3{ 1.0f, 0.0f, 0.0f });
    const TestVector3 leftCorrespondingLateral =
        rock::transform_math::localVectorToWorld(
            leftWeaponInWand,
            TestVector3{ -1.0f, 0.0f, 0.0f });
    ok &= expectVectorNear(
        "left native weapon mirror maps right +X to left -X",
        leftCorrespondingLateral,
        TestVector3{
            -rightLateral.x,
            rightLateral.y,
            rightLateral.z });
    const TestVector3 rightUp =
        rock::transform_math::localVectorToWorld(
            rightNativeWeaponOrientation,
            TestVector3{ 0.0f, 0.0f, 1.0f });
    const TestVector3 leftUp =
        rock::transform_math::localVectorToWorld(
            leftWeaponInWand,
            TestVector3{ 0.0f, 0.0f, 1.0f });
    ok &= expectVectorNear(
        "left native weapon mirror maps right +Z to left +Z",
        leftUp,
        TestVector3{ -rightUp.x, rightUp.y, rightUp.z });
    ok &= expectVectorNear(
        "left native weapon orientation discards right-wand translation",
        leftWeaponInWand.translate,
        TestVector3{});
    ok &= expectNear(
        "left native weapon orientation is unit scale",
        leftWeaponInWand.scale,
        1.0f);
    const auto& mirroredRotation = leftWeaponInWand.rotate.entry;
    const float mirroredDeterminant =
        mirroredRotation[0][0] *
            (mirroredRotation[1][1] * mirroredRotation[2][2] -
                mirroredRotation[1][2] * mirroredRotation[2][1]) -
        mirroredRotation[0][1] *
            (mirroredRotation[1][0] * mirroredRotation[2][2] -
                mirroredRotation[1][2] * mirroredRotation[2][0]) +
        mirroredRotation[0][2] *
            (mirroredRotation[1][0] * mirroredRotation[2][1] -
                mirroredRotation[1][1] * mirroredRotation[2][0]);
    ok &= expectNear(
        "bilateral weapon mirror remains a proper rotation",
        mirroredDeterminant,
        1.0f);

    TestTransform rawLeftWandWorld =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    rawLeftWandWorld.rotate = makeAxisAngleRotation(
        rock::weaponSolverNormalize(
            TestVector3{ -0.4f, 0.7f, 0.2f }),
        28.0f);
    rawLeftWandWorld.translate = { 15.0f, -6.0f, 11.0f };
    TestTransform referenceHandInWand =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    referenceHandInWand.rotate = makeAxisAngleRotation(
        rock::weaponSolverNormalize(
            TestVector3{ 0.6f, 0.1f, -0.5f }),
        -19.0f);
    referenceHandInWand.translate = { 2.0f, -5.0f, 3.0f };
    const TestTransform rawReferenceHandWorld =
        rock::transform_math::composeTransforms(
            rawLeftWandWorld,
            referenceHandInWand);

    TestTransform hfrikDampingWorldDelta =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    hfrikDampingWorldDelta.rotate = makeAxisAngleRotation(
        rock::weaponSolverNormalize(
            TestVector3{ 0.2f, -0.3f, 0.8f }),
        7.0f);
    hfrikDampingWorldDelta.translate = { -4.0f, 9.0f, 1.0f };
    const TestTransform dampedPhysicalHandWorld =
        rock::transform_math::composeTransforms(
            hfrikDampingWorldDelta,
            rawReferenceHandWorld);
    const TestTransform dampedAimCarrierWorld =
        rock::left_firing_position_only_math::
            resolveDampedAimCarrierWorld(
                rawLeftWandWorld,
                referenceHandInWand,
                dampedPhysicalHandWorld);
    TestTransform expectedDampedCarrierWorld = rawLeftWandWorld;
    expectedDampedCarrierWorld.rotate =
        rock::transform_math::composeTransforms(
            rock::left_firing_position_only_math::orientationOnly(
                hfrikDampingWorldDelta),
            rock::left_firing_position_only_math::orientationOnly(
                rawLeftWandWorld))
            .rotate;
    ok &= expectTransformNear(
        "hFRIK damped follow applies only the observed hand rotation delta",
        dampedAimCarrierWorld,
        expectedDampedCarrierWorld);
    ok &= expectTransformNear(
        "zero hFRIK damping delta preserves the raw aim carrier",
        rock::left_firing_position_only_math::
            resolveDampedAimCarrierWorld(
                rawLeftWandWorld,
                referenceHandInWand,
                rawReferenceHandWorld),
        rawLeftWandWorld);

    const TestTransform weaponOnDampedCarrier =
        rock::transform_math::composeTransforms(
            dampedAimCarrierWorld,
            leftWeaponInWand);
    const TestTransform weaponBackInDampedCarrier =
        rock::transform_math::composeTransforms(
            rock::transform_math::invertTransform(
                dampedAimCarrierWorld),
            weaponOnDampedCarrier);
    ok &= expectTransformNear(
        "shared damping preserves mirrored weapon aim in the corrected carrier",
        weaponBackInDampedCarrier,
        leftWeaponInWand);

    TestTransform leftWandWorld =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    leftWandWorld.rotate = makeAxisAngleRotation(
        rock::weaponSolverNormalize(
            TestVector3{ 0.3f, 0.7f, -0.2f }),
        23.0f);
    leftWandWorld.translate = { 40.0f, -12.0f, 9.0f };

    TestTransform liveWeaponWorld =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    liveWeaponWorld.rotate = makeAxisAngleRotation(
        TestVector3{ 1.0f, 0.0f, 0.0f },
        -67.0f);
    liveWeaponWorld.translate = { -100.0f, 55.0f, 13.0f };
    liveWeaponWorld.scale = 1.25f;
    const TestVector3 firingGripWeaponLocal{ 2.0f, 6.0f, -1.0f };
    const TestVector3 physicalLeftGripTargetWorld{ 18.0f, 27.0f, 33.0f };
    const TestTransform solvedLeftWeapon =
        rock::left_firing_position_only_math::
            resolveWeaponWorldPositionOnly(
                leftWandWorld,
                leftWeaponInWand,
                liveWeaponWorld,
                firingGripWeaponLocal,
                physicalLeftGripTargetWorld);
    const TestVector3 solvedGripWorld =
        rock::transform_math::localPointToWorld(
            solvedLeftWeapon,
            firingGripWeaponLocal);
    ok &= expectVectorNear(
        "left position-only weapon solve seats authored firing point",
        solvedGripWorld,
        physicalLeftGripTargetWorld);
    ok &= expectNear(
        "left position-only weapon solve preserves live weapon scale",
        solvedLeftWeapon.scale,
        liveWeaponWorld.scale);

    TestTransform expectedLeftWeaponOrientation =
        rock::transform_math::composeTransforms(
            leftWandWorld,
            leftWeaponInWand);
    expectedLeftWeaponOrientation.translate = solvedLeftWeapon.translate;
    expectedLeftWeaponOrientation.scale = liveWeaponWorld.scale;
    ok &= expectTransformNear(
        "left weapon orientation depends only on native aim, not authored wrist",
        solvedLeftWeapon,
        expectedLeftWeaponOrientation);

    TestTransform authoredLeftWristA =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    authoredLeftWristA.translate = { 1.5f, -2.0f, 0.75f };
    TestTransform authoredLeftWristB = authoredLeftWristA;
    authoredLeftWristB.rotate = makeAxisAngleRotation(
        TestVector3{ 1.0f, 0.0f, 0.0f },
        47.0f);
    const TestTransform solvedAfterAuthoredWristChange =
        rock::left_firing_position_only_math::
            resolveWeaponWorldPositionOnly(
                leftWandWorld,
                leftWeaponInWand,
                liveWeaponWorld,
                firingGripWeaponLocal,
                physicalLeftGripTargetWorld);
    ok &= expectTransformNear(
        "authored wrist rotation cannot alter left weapon aim",
        solvedAfterAuthoredWristChange,
        solvedLeftWeapon);

    const TestTransform presentedLeftWristA =
        rock::transform_math::composeTransforms(
            solvedLeftWeapon,
            authoredLeftWristA);
    const TestTransform presentedLeftWristB =
        rock::transform_math::composeTransforms(
            solvedLeftWeapon,
            authoredLeftWristB);
    const TestVector3 presentedFingerAxisA =
        rock::transform_math::localVectorToWorld(
            presentedLeftWristA,
            TestVector3{ 0.0f, 1.0f, 0.0f });
    const TestVector3 presentedFingerAxisB =
        rock::transform_math::localVectorToWorld(
            presentedLeftWristB,
            TestVector3{ 0.0f, 1.0f, 0.0f });
    const TestVector3 presentedAxisDelta{
        presentedFingerAxisA.x - presentedFingerAxisB.x,
        presentedFingerAxisA.y - presentedFingerAxisB.y,
        presentedFingerAxisA.z - presentedFingerAxisB.z,
    };
    ok &= expectTrue(
        "authored wrist rotation remains visible on the presented hand",
        std::sqrt(
            presentedAxisDelta.x * presentedAxisDelta.x +
            presentedAxisDelta.y * presentedAxisDelta.y +
            presentedAxisDelta.z * presentedAxisDelta.z) >
            0.1f);
    return ok;
}

static bool testSupportRelease()
{
    bool ok = true;
    /*
     * Support release on the left carry: the last rendered two-hand pose
     * rides the physical firing hand and eases into the wand-aimed
     * position-only pose while the firing grip stays on the hand.
     */
    const TestVector3 firingGripWeaponLocal{ 2.0f, -4.0f, -4.0f };
    const TestVector3 gripHandLocal{ 1.0f, 4.0f, 0.5f };
    const auto seatGripOnHand = [&](TestTransform weaponInHand) {
        const TestVector3 gripWithoutTranslation =
            rock::transform_math::localPointToWorld(
                rock::left_firing_position_only_math::orientationOnly(
                    weaponInHand),
                TestVector3{
                    firingGripWeaponLocal.x * weaponInHand.scale,
                    firingGripWeaponLocal.y * weaponInHand.scale,
                    firingGripWeaponLocal.z * weaponInHand.scale });
        weaponInHand.translate = {
            gripHandLocal.x - gripWithoutTranslation.x,
            gripHandLocal.y - gripWithoutTranslation.y,
            gripHandLocal.z - gripWithoutTranslation.z,
        };
        return weaponInHand;
    };

    TestTransform twoHandWeaponInHand =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    twoHandWeaponInHand.rotate = makeAxisAngleRotation(
        TestVector3{ 0.0f, 0.0f, 1.0f },
        40.0f);
    twoHandWeaponInHand.scale = 1.25f;
    twoHandWeaponInHand = seatGripOnHand(twoHandWeaponInHand);
    TestTransform wandAimedWeaponInHand =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    wandAimedWeaponInHand.rotate = makeAxisAngleRotation(
        TestVector3{ 1.0f, 0.0f, 0.0f },
        -5.0f);
    wandAimedWeaponInHand.scale = 1.25f;
    wandAimedWeaponInHand = seatGripOnHand(wandAimedWeaponInHand);

    TestTransform handAtRelease =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    handAtRelease.rotate = makeAxisAngleRotation(
        rock::weaponSolverNormalize(TestVector3{ 0.2f, -0.6f, 0.4f }),
        35.0f);
    handAtRelease.translate = { 12.0f, -4.0f, 30.0f };
    const TestTransform renderedTwoHandWeapon =
        rock::transform_math::composeTransforms(
            handAtRelease,
            twoHandWeaponInHand);
    const TestTransform startHandLocal =
        rock::left_firing_position_only_math::
            weaponWorldToPhysicalHandLocal(
                handAtRelease,
                renderedTwoHandWeapon);
    ok &= expectTransformNear(
        "left carry return start captures the rendered pose in the hand frame",
        startHandLocal,
        twoHandWeaponInHand);

    TestTransform handLater =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    handLater.rotate = makeAxisAngleRotation(
        rock::weaponSolverNormalize(TestVector3{ -0.3f, 0.5f, 0.8f }),
        -22.0f);
    handLater.translate = { 20.0f, 3.0f, 26.0f };
    const TestTransform positionOnlyWeapon =
        rock::transform_math::composeTransforms(
            handLater,
            wandAimedWeaponInHand);
    const TestTransform targetHandLocal =
        rock::left_firing_position_only_math::
            weaponWorldToPhysicalHandLocal(
                handLater,
                positionOnlyWeapon);
    const auto blendedWeaponAt = [&](const float alpha) {
        return rock::left_firing_position_only_math::
            physicalHandLocalToWeaponWorld(
                handLater,
                rock::hand_visual_lerp_math::interpolateTransform(
                    startHandLocal,
                    targetHandLocal,
                    alpha),
                positionOnlyWeapon.scale);
    };
    ok &= expectTransformNear(
        "left carry return start rides the moved firing hand",
        blendedWeaponAt(0.0f),
        rock::transform_math::composeTransforms(
            handLater,
            twoHandWeaponInHand));
    ok &= expectTransformNear(
        "left carry return ends on the wand-aimed pose",
        blendedWeaponAt(1.0f),
        positionOnlyWeapon);

    const TestVector3 physicalGripWorld =
        rock::transform_math::localPointToWorld(
            handLater,
            gripHandLocal);
    for (const float alpha : { 0.25f, 0.5f, 0.75f }) {
        const TestVector3 blendedGripWorld =
            rock::transform_math::localPointToWorld(
                blendedWeaponAt(alpha),
                firingGripWeaponLocal);
        const TestVector3 gripDrift{
            blendedGripWorld.x - physicalGripWorld.x,
            blendedGripWorld.y - physicalGripWorld.y,
            blendedGripWorld.z - physicalGripWorld.z,
        };
        ok &= expectTrue(
            "left carry return keeps the firing grip on the hand",
            std::sqrt(
                gripDrift.x * gripDrift.x +
                gripDrift.y * gripDrift.y +
                gripDrift.z * gripDrift.z) < 1.0f);
    }
    return ok;
}

static bool testPoseHandoffResidual()
{
    bool ok = true;
    // Weapon pose handoff residual: firing-grip detach into part carry
    // and reattach into the two-hand solve ease from the rendered pose.
    const TestTransform identity =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    TestTransform renderedWeapon = identity;
    renderedWeapon.rotate = makeAxisAngleRotation(
        rock::weaponSolverNormalize(TestVector3{ 0.3f, 0.2f, -0.9f }),
        40.0f);
    renderedWeapon.translate = { 10.0f, -5.0f, 30.0f };
    TestTransform solvedWeapon = identity;
    solvedWeapon.rotate = makeAxisAngleRotation(
        rock::weaponSolverNormalize(TestVector3{ -0.5f, 0.7f, 0.4f }),
        -25.0f);
    solvedWeapon.translate = { 14.0f, -1.0f, 27.0f };

    const TestTransform residual =
        rock::hand_visual_lerp_math::captureHandoffResidualLocal(
            solvedWeapon,
            renderedWeapon);
    ok &= expectTransformNear(
        "handoff residual alpha zero reproduces the rendered pose",
        rock::hand_visual_lerp_math::applyHandoffResidual(
            solvedWeapon,
            residual,
            0.0f),
        renderedWeapon);
    ok &= expectTransformNear(
        "handoff residual alpha one lands on the solve",
        rock::hand_visual_lerp_math::applyHandoffResidual(
            solvedWeapon,
            residual,
            1.0f),
        solvedWeapon);

    TestTransform movedSolvedWeapon = identity;
    movedSolvedWeapon.rotate = makeAxisAngleRotation(
        rock::weaponSolverNormalize(TestVector3{ 0.1f, 0.9f, 0.3f }),
        15.0f);
    movedSolvedWeapon.translate = { 20.0f, 3.0f, 26.0f };
    ok &= expectTransformNear(
        "handoff residual rides the moved solve",
        rock::hand_visual_lerp_math::applyHandoffResidual(
            movedSolvedWeapon,
            residual,
            0.0f),
        rock::transform_math::composeTransforms(
            movedSolvedWeapon,
            residual));

    const float fullResidualDegrees =
        rock::hand_visual_lerp_math::rotationDistanceDegrees(
            residual,
            identity);
    ok &= expectTrue(
        "handoff residual carries a measurable rotation",
        fullResidualDegrees > 10.0f);
    const TestTransform halfBlended =
        rock::hand_visual_lerp_math::applyHandoffResidual(
            solvedWeapon,
            residual,
            0.5f);
    ok &= expectNear(
        "handoff residual half alpha slerps half the rotation",
        rock::hand_visual_lerp_math::rotationDistanceDegrees(
            halfBlended,
            solvedWeapon),
        fullResidualDegrees * 0.5f,
        0.01f);

    const float residualDuration =
        rock::hand_visual_lerp_math::computeVisualReturnDuration(
            residual,
            identity,
            rock::hand_visual_lerp_math::kEquippedWeaponReturnConfig);
    ok &= expectTrue(
        "handoff residual outside tolerance blends within the return window",
        residualDuration >= 0.12f && residualDuration <= 0.20f);
    TestTransform exactResidual = identity;
    exactResidual.translate = { 0.2f, -0.1f, 0.3f };
    exactResidual.rotate = makeAxisAngleRotation(
        rock::weaponSolverNormalize(TestVector3{ 0.0f, 0.0f, 1.0f }),
        2.0f);
    ok &= expectNear(
        "handoff residual inside tolerance needs no blend",
        rock::hand_visual_lerp_math::computeVisualReturnDuration(
            exactResidual,
            identity,
            rock::hand_visual_lerp_math::kEquippedWeaponReturnConfig),
        0.0f);
    return ok;
}

static bool testSupportInputFrames()
{
    bool ok = true;
    TestTransform supportInputAtAttach =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    supportInputAtAttach.rotate = makeAxisAngleRotation(
        rock::weaponSolverNormalize(
            TestVector3{ 0.3f, -0.4f, 0.8f }),
        37.0f);
    supportInputAtAttach.translate = { 11.0f, -4.0f, 8.0f };

    TestTransform supportGripTargetAtAttach =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    supportGripTargetAtAttach.rotate = makeAxisAngleRotation(
        rock::weaponSolverNormalize(
            TestVector3{ -0.2f, 0.9f, 0.3f }),
        -24.0f);
    supportGripTargetAtAttach.translate = { -7.0f, 13.0f, 5.0f };

    TestTransform inputToGripTargetLocal{};
    ok &= expectTrue(
        "support baseline captures damped support input to authored grip baseline",
        rock::weapon_support_acquisition_math::
            tryCaptureSupportInputBaseline(
                supportInputAtAttach,
                supportGripTargetAtAttach,
                inputToGripTargetLocal));

    TestTransform resolvedAttachTarget{};
    ok &= expectTrue(
        "support baseline resolves captured support baseline",
        rock::weapon_support_acquisition_math::
            tryResolveSupportInputTarget(
                supportInputAtAttach,
                inputToGripTargetLocal,
                resolvedAttachTarget));
    ok &= expectTransformNear(
        "support baseline unchanged support input reproduces exact authored target",
        resolvedAttachTarget,
        supportGripTargetAtAttach);

    TestTransform laterWorldDelta =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    laterWorldDelta.rotate = makeAxisAngleRotation(
        rock::weaponSolverNormalize(
            TestVector3{ 0.7f, 0.1f, -0.5f }),
        29.0f);
    laterWorldDelta.translate = { 2.0f, -3.0f, 1.5f };
    const TestTransform movedSupportInput =
        rock::transform_math::composeTransforms(
            laterWorldDelta,
            supportInputAtAttach);
    const TestTransform expectedMovedTarget =
        rock::transform_math::composeTransforms(
            laterWorldDelta,
            supportGripTargetAtAttach);
    TestTransform resolvedMovedTarget{};
    ok &= expectTrue(
        "support baseline resolves post-attach support delta",
        rock::weapon_support_acquisition_math::
            tryResolveSupportInputTarget(
                movedSupportInput,
                inputToGripTargetLocal,
                resolvedMovedTarget));
    ok &= expectTransformNear(
        "support baseline carries only the post-attach rigid support delta",
        resolvedMovedTarget,
        expectedMovedTarget);

    TestTransform degenerateInput = supportInputAtAttach;
    degenerateInput.scale = 0.0f;
    ok &= expectFalse(
        "support baseline rejects degenerate support input at capture",
        rock::weapon_support_acquisition_math::
            tryCaptureSupportInputBaseline(
                degenerateInput,
                supportGripTargetAtAttach,
                inputToGripTargetLocal));

    TestTransform nonFiniteRelation = inputToGripTargetLocal;
    nonFiniteRelation.translate.x =
        (std::numeric_limits<float>::quiet_NaN)();
    ok &= expectFalse(
        "support baseline rejects non-finite captured support baseline",
        rock::weapon_support_acquisition_math::
            tryResolveSupportInputTarget(
                supportInputAtAttach,
                nonFiniteRelation,
                resolvedMovedTarget));
    return ok;
}

static bool testPrimaryDriverFrames()
{
    bool ok = true;
    TestTransform primaryDriver =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    primaryDriver.rotate = makeAxisAngleRotation(
        TestVector3{ 0.0f, 0.0f, 1.0f },
        14.0f);
    primaryDriver.translate = { 2.0f, -5.0f, 7.0f };
    TestTransform supportDriver =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    supportDriver.rotate = makeAxisAngleRotation(
        TestVector3{ 0.0f, 1.0f, 0.0f },
        -31.0f);
    supportDriver.translate = { -4.0f, 12.0f, 3.0f };

    TestTransform primaryTarget =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    primaryTarget.rotate = makeAxisAngleRotation(
        TestVector3{ 1.0f, 0.0f, 0.0f },
        22.0f);
    primaryTarget.translate = { 8.0f, 1.0f, -2.0f };
    TestTransform supportTarget =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    supportTarget.rotate = makeAxisAngleRotation(
        rock::weaponSolverNormalize(
            TestVector3{ 0.3f, 0.7f, -0.2f }),
        -19.0f);
    supportTarget.translate = { -9.0f, 6.0f, 11.0f };

    TestTransform primaryRelation{};
    TestTransform supportRelation{};
    ok &= expectTrue(
        "dynamic support captures both controller-driver relations atomically",
        rock::weapon_support_acquisition_math::
            tryCaptureDynamicSupportDriverBaseline(
                primaryDriver,
                primaryTarget,
                supportDriver,
                supportTarget,
                primaryRelation,
                supportRelation));

    TestTransform resolvedPrimary{};
    TestTransform resolvedSupport{};
    ok &= expectTrue(
        "dynamic support resolves both controller-driver targets atomically",
        rock::weapon_support_acquisition_math::
            tryResolveDynamicSupportDriverTargets(
                primaryDriver,
                primaryRelation,
                supportDriver,
                supportRelation,
                resolvedPrimary,
                resolvedSupport));
    ok &= expectTransformNear(
        "unchanged primary driver reproduces captured primary target",
        resolvedPrimary,
        primaryTarget);
    ok &= expectTransformNear(
        "unchanged support driver reproduces captured support target",
        resolvedSupport,
        supportTarget);

    TestTransform primaryDelta =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    primaryDelta.rotate = makeAxisAngleRotation(
        TestVector3{ 0.0f, 1.0f, 0.0f },
        9.0f);
    primaryDelta.translate = { 3.0f, 0.0f, -1.0f };
    TestTransform supportDelta =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    supportDelta.rotate = makeAxisAngleRotation(
        TestVector3{ 1.0f, 0.0f, 0.0f },
        -12.0f);
    supportDelta.translate = { -2.0f, 4.0f, 0.5f };
    const TestTransform movedPrimaryDriver =
        rock::transform_math::composeTransforms(
            primaryDelta,
            primaryDriver);
    const TestTransform movedSupportDriver =
        rock::transform_math::composeTransforms(
            supportDelta,
            supportDriver);
    ok &= expectTrue(
        "dynamic support resolves independent post-capture driver deltas",
        rock::weapon_support_acquisition_math::
            tryResolveDynamicSupportDriverTargets(
                movedPrimaryDriver,
                primaryRelation,
                movedSupportDriver,
                supportRelation,
                resolvedPrimary,
                resolvedSupport));
    ok &= expectTransformNear(
        "primary target follows only primary driver delta",
        resolvedPrimary,
        rock::transform_math::composeTransforms(
            primaryDelta,
            primaryTarget));
    ok &= expectTransformNear(
        "support target follows only support driver delta",
        resolvedSupport,
        rock::transform_math::composeTransforms(
            supportDelta,
            supportTarget));

    TestTransform invalidSupportDriver = supportDriver;
    invalidSupportDriver.scale = 0.0f;
    ok &= expectFalse(
        "dynamic support fails closed when either current driver is invalid",
        rock::weapon_support_acquisition_math::
            tryResolveDynamicSupportDriverTargets(
                primaryDriver,
                primaryRelation,
                invalidSupportDriver,
                supportRelation,
                resolvedPrimary,
                resolvedSupport));
    return ok;
}

static bool testWeaponGripTransforms()
{
    bool ok = true;
    TestTransform weaponWorld =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    weaponWorld.rotate = makeAxisAngleRotation(
        TestVector3{ 0.0f, 0.0f, 1.0f },
        18.0f);
    weaponWorld.translate = { 4.0f, -6.0f, 2.0f };
    const TestVector3 primaryGripLocal{ 0.0f, 0.0f, 0.0f };
    const TestVector3 supportGripLocal{ 0.0f, 12.0f, 0.0f };
    const TestVector3 supportNormalLocal{ 1.0f, 0.0f, 0.0f };
    const TestVector3 primaryTargetWorld =
        rock::transform_math::localPointToWorld(
            weaponWorld,
            primaryGripLocal);
    const TestVector3 supportGripWorld =
        rock::transform_math::localPointToWorld(
            weaponWorld,
            supportGripLocal);

    TestTransform supportGripHandWorld = weaponWorld;
    supportGripHandWorld.translate = supportGripWorld;
    TestTransform dampedSupportInput =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    dampedSupportInput.rotate = makeAxisAngleRotation(
        TestVector3{ 0.0f, 1.0f, 0.0f },
        -41.0f);
    dampedSupportInput.translate = { 20.0f, 7.0f, -3.0f };

    TestTransform inputToGripTargetLocal{};
    TestTransform calibratedSupportTarget{};
    ok &= expectTrue(
        "support baseline solver test captures authored hand target",
        rock::weapon_support_acquisition_math::
            tryCaptureSupportInputBaseline(
                dampedSupportInput,
                supportGripHandWorld,
                inputToGripTargetLocal));
    ok &= expectTrue(
        "support baseline solver test resolves authored hand target",
        rock::weapon_support_acquisition_math::
            tryResolveSupportInputTarget(
                dampedSupportInput,
                inputToGripTargetLocal,
                calibratedSupportTarget));

    const TestVector3 lockedSupportTarget =
        rock::makeLockedSupportGripTarget(
            primaryTargetWorld,
            calibratedSupportTarget.translate,
            supportGripWorld,
            rock::weaponSolverLength(
                rock::weaponSolverSub(
                    supportGripWorld,
                    primaryTargetWorld)),
            0.001f);
    rock::WeaponTwoHandedSolverInput<
        TestTransform,
        TestVector3> solverInput{};
    solverInput.weaponWorldTransform = weaponWorld;
    solverInput.primaryGripLocal = primaryGripLocal;
    solverInput.supportGripLocal = supportGripLocal;
    solverInput.primaryTargetWorld = primaryTargetWorld;
    solverInput.supportTargetWorld = lockedSupportTarget;
    solverInput.supportNormalLocal = supportNormalLocal;
    solverInput.supportNormalTargetWorld =
        rock::transform_math::localVectorToWorld(
            calibratedSupportTarget,
            supportNormalLocal);
    solverInput.useSupportNormalTwist = true;
    solverInput.supportNormalTwistFactor = 0.5f;

    const auto attachSolve =
        rock::solveTwoHandedWeaponTransformFrikPivot(solverInput);
    ok &= expectTrue(
        "support baseline calibrated attach target solves",
        attachSolve.solved);
    ok &= expectTransformNear(
        "support baseline calibrated attach target leaves weapon unchanged",
        attachSolve.weaponWorldTransform,
        weaponWorld);
    return ok;
}

static bool testAttachRelativeFrames()
{
    bool ok = true;
    const TestTransform weaponAtAttach =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    const TestVector3 primaryGripLocal{ 0.0f, 0.0f, 0.0f };
    const TestVector3 supportGripLocal{ 0.0f, 10.0f, 0.0f };
    const TestVector3 supportNormalLocal{ 1.0f, 0.0f, 0.0f };

    TestTransform supportGripHandAtAttach = weaponAtAttach;
    supportGripHandAtAttach.translate = supportGripLocal;
    TestTransform supportInputAtAttach =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    supportInputAtAttach.rotate = makeAxisAngleRotation(
        TestVector3{ 1.0f, 0.0f, 0.0f },
        45.0f);
    supportInputAtAttach.translate = { 7.0f, 3.0f, -2.0f };

    TestTransform inputToGripTargetLocal{};
    ok &= expectTrue(
        "support baseline tandem test captures attach baseline",
        rock::weapon_support_acquisition_math::
            tryCaptureSupportInputBaseline(
                supportInputAtAttach,
                supportGripHandAtAttach,
                inputToGripTargetLocal));

    TestTransform supportDelta =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    supportDelta.rotate = makeAxisAngleRotation(
        TestVector3{ 0.0f, 0.0f, 1.0f },
        30.0f);
    const TestTransform movedSupportInput =
        rock::transform_math::composeTransforms(
            supportDelta,
            supportInputAtAttach);
    TestTransform movedCalibratedTarget{};
    ok &= expectTrue(
        "support baseline tandem test resolves moved support input",
        rock::weapon_support_acquisition_math::
            tryResolveSupportInputTarget(
                movedSupportInput,
                inputToGripTargetLocal,
                movedCalibratedTarget));

    rock::WeaponTwoHandedSolverInput<
        TestTransform,
        TestVector3> solverInput{};
    solverInput.weaponWorldTransform = weaponAtAttach;
    solverInput.primaryGripLocal = primaryGripLocal;
    solverInput.supportGripLocal = supportGripLocal;
    solverInput.primaryTargetWorld = primaryGripLocal;
    solverInput.supportTargetWorld =
        rock::makeLockedSupportGripTarget(
            primaryGripLocal,
            movedCalibratedTarget.translate,
            supportGripLocal,
            10.0f,
            0.001f);
    solverInput.supportNormalLocal = supportNormalLocal;
    solverInput.supportNormalTargetWorld =
        rock::transform_math::localVectorToWorld(
            movedCalibratedTarget,
            supportNormalLocal);
    solverInput.useSupportNormalTwist = true;
    solverInput.supportNormalTwistFactor = 0.5f;

    const auto movedSolve =
        rock::solveTwoHandedWeaponTransformFrikPivot(solverInput);
    ok &= expectTrue(
        "support baseline post-attach tandem delta solves",
        movedSolve.solved);
    ok &= expectTransformNear(
        "support baseline post-attach support delta drives existing tandem solver",
        movedSolve.weaponWorldTransform,
        supportDelta);
    ok &= expectVectorNear(
        "support baseline tandem delta keeps primary pivot fixed",
        rock::transform_math::localPointToWorld(
            movedSolve.weaponWorldTransform,
            primaryGripLocal),
        primaryGripLocal);
    return ok;
}

static bool testLiveHandDriver()
{
    bool ok = true;
    TestTransform liveHandWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
    liveHandWorld.translate = { 10.0f, 5.0f, -2.0f };
    const TestVector3 livePalmPivot{ 12.0f, 8.0f, 1.0f };
    const TestVector3 selectedGripPoint{ 18.0f, 6.0f, 5.0f };

    TestTransform partWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
    partWorld.translate = { 30.0f, 40.0f, 50.0f };
    partWorld.rotate.entry[0][0] = 0.0f;
    partWorld.rotate.entry[0][1] = 1.0f;
    partWorld.rotate.entry[1][0] = -1.0f;
    partWorld.rotate.entry[1][1] = 0.0f;
    partWorld.scale = 1.25f;

    const TestTransform seatedHandWorld = rock::weapon_two_handed_grip_math::alignHandFrameToGripPoint(
        liveHandWorld,
        livePalmPivot,
        selectedGripPoint);
    const TestTransform virtualPartWorld = rock::weapon_two_handed_grip_math::virtualizeMeshForTranslatedHandSeat(
        partWorld,
        livePalmPivot,
        selectedGripPoint);
    const TestVector3 virtualGripPoint = rock::weapon_two_handed_grip_math::virtualizeGripPointForTranslatedHandSeat(
        livePalmPivot,
        selectedGripPoint);

    ok &= expectNear("support grip seats hand translation x", seatedHandWorld.translate.x, 16.0f);
    ok &= expectNear("support grip seats hand translation y", seatedHandWorld.translate.y, 3.0f);
    ok &= expectNear("support grip seats hand translation z", seatedHandWorld.translate.z, 2.0f);
    ok &= expectNear("support grip virtual mesh applies inverse seat x", virtualPartWorld.translate.x, 24.0f);
    ok &= expectNear("support grip virtual mesh applies inverse seat y", virtualPartWorld.translate.y, 42.0f);
    ok &= expectNear("support grip virtual mesh applies inverse seat z", virtualPartWorld.translate.z, 46.0f);
    ok &= expectNear("support grip virtual seat resolves to live palm x", virtualGripPoint.x, livePalmPivot.x);
    ok &= expectNear("support grip virtual seat resolves to live palm y", virtualGripPoint.y, livePalmPivot.y);
    ok &= expectNear("support grip virtual seat resolves to live palm z", virtualGripPoint.z, livePalmPivot.z);
    ok &= expectNear("support grip virtual mesh preserves scale", virtualPartWorld.scale, partWorld.scale);
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            ok &= expectNear("support grip virtual mesh preserves rotation", virtualPartWorld.rotate.entry[row][column], partWorld.rotate.entry[row][column]);
        }
    }

    const TestVector3 finalGripFromHand{
        selectedGripPoint.x - seatedHandWorld.translate.x,
        selectedGripPoint.y - seatedHandWorld.translate.y,
        selectedGripPoint.z - seatedHandWorld.translate.z,
    };
    const TestVector3 virtualGripFromHand{
        virtualGripPoint.x - liveHandWorld.translate.x,
        virtualGripPoint.y - liveHandWorld.translate.y,
        virtualGripPoint.z - liveHandWorld.translate.z,
    };
    ok &= expectNear("support grip frozen solve preserves final hand/mesh relation x", virtualGripFromHand.x, finalGripFromHand.x);
    ok &= expectNear("support grip frozen solve preserves final hand/mesh relation y", virtualGripFromHand.y, finalGripFromHand.y);
    ok &= expectNear("support grip frozen solve preserves final hand/mesh relation z", virtualGripFromHand.z, finalGripFromHand.z);
    return ok;
}

static bool testRawHandDriver()
{
    bool ok = true;
    TestTransform rawHandWorld =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    rawHandWorld.translate = { 10.0f, -5.0f, 3.0f };
    const TestVector3 palmPivotWorld{ 12.0f, -5.0f, 3.0f };
    const TestVector3 palmNormalWorld{ 1.0f, 0.0f, 0.0f };
    const TestVector3 surfaceNormalWorld{ 0.0f, -1.0f, 0.0f };
    const TestVector3 targetGripPointWorld{ 30.0f, 9.0f, -4.0f };
    constexpr float kThirtyDegreesRadians =
        0.52359877559829887308f;

    const auto seated =
        rock::weapon_support_acquisition_math::
            alignHandFrameToGripSurface<
                TestTransform,
                TestVector3>(
                rawHandWorld,
                palmPivotWorld,
                palmNormalWorld,
                targetGripPointWorld,
                surfaceNormalWorld,
                kThirtyDegreesRadians);
    ok &= expectTrue(
        "support surface seat produces a finite hand frame",
        seated.valid);
    ok &= expectNear(
        "support surface seat clamps wrist swing",
        seated.appliedRotationRadians,
        kThirtyDegreesRadians);

    const TestVector3 rawOriginToPalm =
        rock::weaponSolverSub(
            palmPivotWorld,
            rawHandWorld.translate);
    const TestVector3 seatedPalmPoint =
        rock::weaponSolverAdd(
            seated.handWorld.translate,
            rock::weaponSolverApplyStoredWorldRotationToVector<
                TestMatrix3,
                TestVector3>(
                seated.handWorld.rotate,
                rawOriginToPalm));
    ok &= expectVectorNear(
        "support surface seat preserves exact palm contact pivot",
        seatedPalmPoint,
        targetGripPointWorld);

    const TestVector3 seatedPalmNormal =
        rock::weaponSolverNormalize(
            rock::weaponSolverApplyStoredWorldRotationToVector<
                TestMatrix3,
                TestVector3>(
                seated.handWorld.rotate,
                palmNormalWorld));
    ok &= expectNear(
        "support surface seat rotates palm toward inward normal",
        rock::weaponSolverDot(
            seatedPalmNormal,
            TestVector3{ 0.0f, 1.0f, 0.0f }),
        0.5f);

    TestTransform weaponWorld =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    weaponWorld.rotate = makeAxisAngleRotation(
        rock::weaponSolverNormalize(
            TestVector3{ 0.2f, 0.7f, -0.4f }),
        23.0f);
    weaponWorld.translate = { -40.0f, 70.0f, 18.0f };
    weaponWorld.scale = 1.3f;
    const TestVector3 meshPointLocal{ 4.0f, -3.0f, 2.0f };
    const TestVector3 originalMeshPointWorld =
        rock::transform_math::localPointToWorld(
            weaponWorld,
            meshPointLocal);
    const TestTransform virtualWeaponWorld =
        rock::weapon_two_handed_grip_math::
            virtualizeMeshForSeatedHand(
                weaponWorld,
                rawHandWorld,
                seated.handWorld);
    const TestVector3 virtualMeshPointWorld =
        rock::transform_math::localPointToWorld(
            virtualWeaponWorld,
            meshPointLocal);
    ok &= expectVectorNear(
        "full surface seat virtualization preserves hand-mesh relation",
        rock::transform_math::worldPointToLocal(
            rawHandWorld,
            virtualMeshPointWorld),
        rock::transform_math::worldPointToLocal(
            seated.handWorld,
            originalMeshPointWorld));
    return ok;
}

static bool testWeaponPresentationDelta()
{
    bool ok = true;
    TestTransform oldWeaponWorld =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    oldWeaponWorld.translate = { 8.0f, -3.0f, 4.0f };
    oldWeaponWorld.rotate = makeAxisAngleRotation(
        TestVector3{ 0.0f, 0.0f, 1.0f },
        25.0f);
    oldWeaponWorld.scale = 1.2f;

    TestTransform newWeaponWorld =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    newWeaponWorld.translate = { -6.0f, 12.0f, 2.0f };
    newWeaponWorld.rotate = makeAxisAngleRotation(
        TestVector3{ 1.0f, 0.0f, 0.0f },
        -35.0f);
    newWeaponWorld.scale = 0.85f;

    TestTransform animatedPresentationWorld =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    animatedPresentationWorld.translate = { 15.0f, 7.0f, -2.0f };
    animatedPresentationWorld.rotate = makeAxisAngleRotation(
        TestVector3{ 0.0f, 1.0f, 0.0f },
        48.0f);
    animatedPresentationWorld.scale = 0.9f;

    const TestTransform presentationWorldDelta =
        rock::weapon_visual_authority_math::makePresentationWorldDelta(
            oldWeaponWorld,
            newWeaponWorld);
    const TestTransform reframedPresentationWorld =
        rock::weapon_visual_authority_math::applyPresentationWorldDelta(
            presentationWorldDelta,
            animatedPresentationWorld);
    const TestTransform oldWeaponRelativePresentation =
        rock::transform_math::composeTransforms(
            rock::transform_math::invertTransform(oldWeaponWorld),
            animatedPresentationWorld);
    const TestTransform newWeaponRelativePresentation =
        rock::transform_math::composeTransforms(
            rock::transform_math::invertTransform(newWeaponWorld),
            reframedPresentationWorld);
    ok &= expectTransformNear(
        "weapon presentation reframe preserves evaluated root-relative world",
        newWeaponRelativePresentation,
        oldWeaponRelativePresentation);

    TestTransform staleAuthoredLocal =
        rock::transform_math::makeIdentityTransform<TestTransform>();
    staleAuthoredLocal.translate = { 100.0f, 200.0f, 300.0f };
    const TestTransform staleLocalRebuild =
        rock::transform_math::composeTransforms(
            newWeaponWorld,
            staleAuthoredLocal);
    ok &= expectTrue(
        "weapon presentation reframe ignores stale descendant local",
        std::fabs(
            reframedPresentationWorld.translate.x -
            staleLocalRebuild.translate.x) > 1.0f);
    return ok;
}

static bool testPartGripReporting()
{
    bool ok = true;
    using namespace rock::weapon_part_grip_report_policy;
    ok &= expectTrue("active non-attach part grip counts as carry", partGripCountsAsCarry(true, false));
    ok &= expectFalse("attach-only part grip never counts as carry", partGripCountsAsCarry(true, true));
    ok &= expectFalse("inactive part grip never counts as carry", partGripCountsAsCarry(false, false));

    ok &= expectTrue("provider AttachOnly grab mode resolves attach-only",
        providerGrabModeIsAttachOnly(true, static_cast<std::uint32_t>(rock::weapon_part_runtime::GrabMode::AttachOnly)));
    ok &= expectFalse("provider full-authority grab mode is not attach-only",
        providerGrabModeIsAttachOnly(true, static_cast<std::uint32_t>(rock::weapon_part_runtime::GrabMode::FullTwoHandAuthority)));
    ok &= expectFalse("attach-only requires an active provider authority",
        providerGrabModeIsAttachOnly(false, static_cast<std::uint32_t>(rock::weapon_part_runtime::GrabMode::AttachOnly)));

    ok &= expectEqual("firing hand in gripping state reports the firing grip",
        resolveHandGripKind(true, false, false, true, false, false, false),
        HandGripKind::FiringGrip);
    ok &= expectEqual("firing hand in primary-only state reports the firing grip",
        resolveHandGripKind(false, false, true, true, false, false, false),
        HandGripKind::FiringGrip);
    ok &= expectEqual("offhand full-authority support grip reports full authority",
        resolveHandGripKind(true, false, false, false, true, false, false),
        HandGripKind::SupportFullAuthority);
    ok &= expectEqual("offhand visual-only support grip reports visual-only",
        resolveHandGripKind(true, false, false, false, true, false, true),
        HandGripKind::SupportVisualOnly);
    ok &= expectEqual("attach-only grip reports attach-only in gripping state",
        resolveHandGripKind(true, false, false, false, true, true, true),
        HandGripKind::AttachOnly);
    ok &= expectEqual("carry part grip in part-carry reports part carry",
        resolveHandGripKind(false, true, false, false, true, false, false),
        HandGripKind::PartCarry);
    ok &= expectEqual("detached firing hand attach-only grip reports attach-only",
        resolveHandGripKind(false, true, false, true, true, true, false),
        HandGripKind::AttachOnly);
    ok &= expectEqual("detached firing hand carry grip reports part carry",
        resolveHandGripKind(false, true, false, true, true, false, false),
        HandGripKind::PartCarry);
    ok &= expectEqual("idle hand reports no grip",
        resolveHandGripKind(false, false, false, false, false, false, false),
        HandGripKind::None);
    ok &= expectEqual("firing hand without part grip in part-carry reports no grip",
        resolveHandGripKind(false, true, false, true, false, false, false),
        HandGripKind::None);
    const CarryGripInput authoredSeat{ .active = true, .authoredSupportSeat = true };
    ok &= expectTrue("paired authored handguards use one moving anchor",
        usesAuthoredSupportCarryPair(authoredSeat, authoredSeat));
    for (const bool changeLeft : { true, false }) {
        for (int excluded = 0; excluded < 4; ++excluded) {
            auto other = authoredSeat;
            if (excluded == 0) other.active = false;
            if (excluded == 1) other.authoredSupportSeat = false;
            if (excluded == 2) other.providerAuthorityActive = true;
            if (excluded == 3) other.attachOnly = true;
            ok &= expectTrue("only two unreserved authored support seats select visual carry",
                !usesAuthoredSupportCarryPair(changeLeft ? other : authoredSeat,
                    changeLeft ? authoredSeat : other));
        }
    }
    ok &= expectEqual("authored carry follower reports visual support",
        resolveHandGripKind(false, true, false, true, true, false, true),
        HandGripKind::SupportVisualOnly);
    ok &= expectEqual("provider glue remains attach-only during visual carry",
        resolveHandGripKind(false, true, false, false, true, true, true),
        HandGripKind::AttachOnly);
    return ok;
}

static bool testPartStructureIdentity()
{
    bool ok = true;
    using namespace rock::weapon_part_record_identity_policy;
    ok &= expectEqual("P-Mag resolves the magazine slot anchor",
        resolveStructureAnchor("P-Mag"), StructureAnchor::SlotMagazine);
    ok &= expectEqual("P-Stock resolves the rear-furniture slot anchor",
        resolveStructureAnchor("P-Stock"), StructureAnchor::SlotRearFurniture);
    ok &= expectEqual("P-Barrel resolves the barrel slot anchor",
        resolveStructureAnchor("P-Barrel"), StructureAnchor::SlotBarrel);
    ok &= expectEqual("P-Compensator resolves the muzzle slot anchor",
        resolveStructureAnchor("P-Compensator"), StructureAnchor::SlotMuzzle);
    ok &= expectEqual("P-Bipod resolves the dedicated bipod slot anchor",
        resolveStructureAnchor("P-Bipod"), StructureAnchor::SlotBipod);
    ok &= expectEqual("mod-prefixed bipod connect point resolves the bipod slot anchor",
        resolveStructureAnchor("P-SV98Bipod"), StructureAnchor::SlotBipod);
    ok &= expectEqual("WeaponBolt resolves the bolt rig anchor",
        resolveStructureAnchor("WeaponBolt"), StructureAnchor::RigBolt);
    ok &= expectEqual("WeaponMagazineChild3 resolves the magazine display rig anchor",
        resolveStructureAnchor("WeaponMagazineChild3"), StructureAnchor::RigMagazineDisplay);
    ok &= expectEqual("dedicated magazine slot outranks its internal display rig",
        chooseStructureAnchor(StructureAnchor::SlotMagazine, StructureAnchor::RigMagazineDisplay), StructureAnchor::SlotMagazine);
    ok &= expectEqual("bolt rig outranks the catch-all receiver slot",
        chooseStructureAnchor(StructureAnchor::SlotReceiver, StructureAnchor::RigBolt), StructureAnchor::RigBolt);
    ok &= expectEqual("unknown mod-added connect point resolves no anchor",
        resolveStructureAnchor("P-CustomThing"), StructureAnchor::None);
    ok &= expectEqual("plain mesh name resolves no anchor",
        resolveStructureAnchor("AK74M_Body"), StructureAnchor::None);
    const auto otherByName = rock::classifyWeaponPartKind(rock::WeaponPartKind::Other);
    const auto magFromSlot = applyStructureAnchor(otherByName, StructureAnchor::SlotMagazine);
    ok &= expectEqual("magazine slot classifies an unnamed part as magazine",
        magFromSlot.partKind, rock::WeaponPartKind::Magazine);
    ok &= expectEqual("magazine slot classification is slot-sourced",
        magFromSlot.classificationSource, rock::WeaponPartClassificationSource::SlotAnchor);
    ok &= expectEqual("magazine slot carries the vanilla attach-point form id",
        magFromSlot.attachPointFormId, kAttachPointMagazine);
    const auto cartridgeKeptInMagazineSlot = applyStructureAnchor(
        rock::classifyWeaponPartKind(rock::WeaponPartKind::Round), StructureAnchor::SlotMagazine);
    ok &= expectEqual("magazine slot preserves an explicitly named cartridge",
        cartridgeKeptInMagazineSlot.partKind, rock::WeaponPartKind::Round);
    const auto cosmeticBulletKeptInMagazineSlot = applyStructureAnchor(
        rock::classifyWeaponPartKind(rock::WeaponPartKind::CosmeticAmmo), StructureAnchor::SlotMagazine);
    ok &= expectEqual("magazine slot preserves explicitly cosmetic bullet geometry",
        cosmeticBulletKeptInMagazineSlot.partKind, rock::WeaponPartKind::CosmeticAmmo);

    const auto receiverByWeakToken = rock::classifyWeaponPartKind(rock::WeaponPartKind::Receiver);
    const auto barrelOverride = applyStructureAnchor(receiverByWeakToken, StructureAnchor::SlotBarrel);
    ok &= expectEqual("barrel slot overrides a weak receiver name match",
        barrelOverride.partKind, rock::WeaponPartKind::Barrel);
    const auto muzzleOverride = applyStructureAnchor(receiverByWeakToken, StructureAnchor::SlotMuzzle);
    ok &= expectEqual("muzzle slot classifies its physical module separately from the barrel",
        muzzleOverride.partKind, rock::WeaponPartKind::MuzzleDevice);
    ok &= expectEqual("muzzle slot carries the vanilla attach-point form id",
        muzzleOverride.attachPointFormId, kAttachPointMuzzle);

    const auto slideByName = rock::classifyWeaponPartKind(rock::WeaponPartKind::Slide);
    const auto slideKept = applyStructureAnchor(slideByName, StructureAnchor::RigBolt);
    ok &= expectEqual("action-named part keeps its name under the bolt rig",
        slideKept.partKind, rock::WeaponPartKind::Slide);
    ok &= expectEqual("kept action name stays name-sourced",
        slideKept.classificationSource, rock::WeaponPartClassificationSource::NameToken);
    const auto pumpKept = applyStructureAnchor(
        rock::classifyWeaponPartKind(rock::WeaponPartKind::Pump), StructureAnchor::SlotHandguard);
    ok &= expectEqual("pump keeps its action role inside the handguard slot",
        pumpKept.partKind, rock::WeaponPartKind::Pump);

    const auto receiverFill = applyStructureAnchor(otherByName, StructureAnchor::SlotReceiver);
    ok &= expectEqual("receiver slot fills unclassified parts",
        receiverFill.partKind, rock::WeaponPartKind::Receiver);
    const auto stockKeptOverReceiver = applyStructureAnchor(
        rock::classifyWeaponPartKind(rock::WeaponPartKind::Stock), StructureAnchor::SlotReceiver);
    ok &= expectEqual("receiver slot never overrides a critical name match",
        stockKeptOverReceiver.partKind, rock::WeaponPartKind::Stock);

    const auto roundKept = applyStructureAnchor(
        rock::classifyWeaponPartKind(rock::WeaponPartKind::Round), StructureAnchor::RigMagazineDisplay);
    ok &= expectEqual("named ammo round keeps its reload role under the magazine rig",
        roundKept.partKind, rock::WeaponPartKind::Round);
    const auto followerFill = applyStructureAnchor(otherByName, StructureAnchor::RigMagazineDisplay);
    ok &= expectEqual("unnamed magazine-rig part fills as cosmetic ammo",
        followerFill.partKind, rock::WeaponPartKind::CosmeticAmmo);

    const auto noAnchor = applyStructureAnchor(otherByName, StructureAnchor::None);
    ok &= expectEqual("no anchor keeps the name classification",
        noAnchor.partKind, rock::WeaponPartKind::Other);
    ok &= expectEqual("no anchor keeps the name source",
        noAnchor.classificationSource, rock::WeaponPartClassificationSource::NameToken);

    ok &= expectEqual("authored suppressor name classifies as a muzzle device",
        rock::classifyWeaponPartName("AK_Suppressor_Mesh").partKind,
        rock::WeaponPartKind::MuzzleDevice);
    ok &= expectEqual("barrel remains distinct from its installed muzzle device",
        rock::classifyWeaponPartName("WeaponBarrel").partKind,
        rock::WeaponPartKind::Barrel);
    ok &= expectEqual("authored bipod name classifies without deployment inference",
        rock::classifyWeaponPartName("Rifle_BiPod").partKind,
        rock::WeaponPartKind::Bipod);
    ok &= expectEqual("bipod identity outranks incidental cylinder export token",
        rock::classifyWeaponPartName("bipod_Cylinder_009_Bipod").partKind,
        rock::WeaponPartKind::Bipod);
    const auto bipodOverActionName = applyStructureAnchor(
        rock::classifyWeaponPartKind(rock::WeaponPartKind::Cylinder), StructureAnchor::SlotBipod);
    ok &= expectEqual("dedicated bipod slot overrides incidental action-name classification",
        bipodOverActionName.partKind, rock::WeaponPartKind::Bipod);
    return ok;
}

static bool testAccessoryClassification()
{
    bool ok = true;
    using namespace rock::weapon_accessory_part_kind_policy;

    auto sight = rock::classifyWeaponPartKind(rock::WeaponPartKind::Sight);
    sight.attachPointFormId = rock::weapon_part_record_identity_policy::kAttachPointSight;
    const auto unchangedSight = applyAttachmentEvidence(sight, {});
    ok &= expectEqual("reticle-only optic remains Sight",
        unchangedSight.partKind, rock::WeaponPartKind::Sight);
    ok &= expectEqual("unchanged sight retains its original classification source",
        unchangedSight.classificationSource, rock::WeaponPartClassificationSource::NameToken);

    const auto laser = applyAttachmentEvidence(sight, Evidence{ .laserEmitter = true });
    ok &= expectEqual("laser emitter refines the physical module to LaserSight",
        laser.partKind, rock::WeaponPartKind::LaserSight);
    ok &= expectEqual("laser module reports attachment-backed classification",
        laser.classificationSource, rock::WeaponPartClassificationSource::AttachmentEvidence);
    ok &= expectEqual("attachment refinement retains the owning slot FormID",
        laser.attachPointFormId, sight.attachPointFormId);

    const auto flashlight = applyAttachmentEvidence(sight, Evidence{ .flashlightEmitter = true });
    ok &= expectEqual("flashlight emitter refines the physical module to Flashlight",
        flashlight.partKind, rock::WeaponPartKind::Flashlight);

    const auto combo = applyAttachmentEvidence(sight, Evidence{ .laserEmitter = true, .flashlightEmitter = true });
    ok &= expectEqual("co-owned laser and flashlight emitters produce the combo kind",
        combo.partKind, rock::WeaponPartKind::LaserFlashlightCombo);

    const auto nativeScope = applyAttachmentEvidence(
        rock::classifyWeaponPartKind(rock::WeaponPartKind::Barrel),
        Evidence{ .nativeScopeOverlay = true, .laserEmitter = true, .flashlightEmitter = true });
    ok &= expectEqual("native overlay OMOD evidence is authoritative for Scope",
        nativeScope.partKind, rock::WeaponPartKind::Scope);

    static_assert(static_cast<std::uint32_t>(rock::WeaponPartKind::Other) == 22);
    static_assert(static_cast<std::uint32_t>(rock::WeaponPartKind::LaserSight) == 23);
    static_assert(static_cast<std::uint32_t>(rock::WeaponPartKind::Flashlight) == 24);
    static_assert(static_cast<std::uint32_t>(rock::WeaponPartKind::LaserFlashlightCombo) == 25);
    static_assert(static_cast<std::uint32_t>(rock::WeaponPartKind::Scope) == 26);
    static_assert(static_cast<std::uint32_t>(rock::WeaponPartKind::MuzzleDevice) == 27);
    static_assert(static_cast<std::uint32_t>(rock::WeaponPartKind::Bipod) == 28);
    return ok;
}

bool testSelectionCleanupKeepsHeldOwnership()
{
    using rock::HandInteractionEvent;
    using rock::HandState;
    using rock::HandTransitionEffect;
    bool ok = true;
    for (const auto state : { HandState::HeldInit, HandState::HeldBody,
             HandState::StashCandidate, HandState::ConsumeCandidate }) {
        const auto clear = rock::evaluateHandTransition({ .current = state, .event = HandInteractionEvent::ClearSelection });
        ok &= expectFalse("selection cleanup cannot cancel a physical hold", clear.accepted);
        ok &= expectEqual("held state survives selection cleanup", clear.next, state);
        ok &= expectEqual("rejected cleanup has no side effects", clear.effects, 0u);

        // Regression: the paired equip path used to invalidate the support
        // state, making releaseGrabbedObject return without releasing its pose
        // or constraint. A selection clear must leave normal release possible.
        ok &= expectTrue("physical hold remains eligible for release", rock::isHoldingState(clear.next));
        const auto release = rock::evaluateHandTransition({ .current = clear.next, .event = HandInteractionEvent::ReleaseRequested });
        ok &= expectTrue("held release remains accepted", release.accepted);
        ok &= expectEqual("held release finishes idle", release.next, HandState::Idle);
        const auto required = rock::transitionEffectMask(HandTransitionEffect::ReleaseHeld,
            HandTransitionEffect::ClearHeldRuntime, HandTransitionEffect::ClearFingerPose);
        ok &= expectEqual("held release retains native and pose cleanup", release.effects & required, required);
    }
    for (const auto state : { HandState::GrabFromOtherHand, HandState::SelectedTwoHand, HandState::HeldTwoHanded }) {
        const auto clear = rock::evaluateHandTransition({ .current = state, .event = HandInteractionEvent::ClearSelection });
        ok &= expectFalse("selection cleanup cannot discard another hand owner", clear.accepted);
        ok &= expectEqual("other hand ownership survives selection cleanup", clear.next, state);
        ok &= expectEqual("other hand cleanup has no side effects", clear.effects, 0u);
    }
    for (const auto state : { HandState::Idle, HandState::SelectedClose, HandState::SelectedFar,
             HandState::SelectionLocked, HandState::PreGrabItem, HandState::PrePullItem,
             HandState::Pulled, HandState::GrabExternal, HandState::LootOtherHand }) {
        const auto clear = rock::evaluateHandTransition({ .current = state, .event = HandInteractionEvent::ClearSelection });
        ok &= expectTrue("selection and pending acquisition remain cancellable", clear.accepted);
        ok &= expectEqual("selection cleanup finishes idle", clear.next, HandState::Idle);
        ok &= expectEqual("selection cleanup never claims to release a hold",
            clear.effects & rock::transitionEffectMask(HandTransitionEffect::ReleaseHeld), 0u);
    }
    return ok;
}

int main()
{
    using WeaponGrabMode = rock::equipped_weapon_toggle_grab_policy::Mode;
    namespace grab_modes = rock::equipped_weapon_toggle_grab_policy;
    for (const auto mode : { WeaponGrabMode::ToggleBoth, WeaponGrabMode::ToggleFiringOnly, WeaponGrabMode::HoldBoth }) {
        for (const bool firing : { false, true }) {
            grab_modes::TransferReleaseState transfer{};
            transfer.observe(mode, firing, { .held = true, .pressed = true });
            if (transfer.releaseRequested) return 96;
            transfer.observe(mode, firing, { .released = true });
            if (transfer.releaseRequested == grab_modes::usesToggleForRole(mode, firing)) return 97;
            transfer.observe(mode, firing, { .held = true, .pressed = true });
            if (transfer.releaseRequested != grab_modes::usesToggleForRole(mode, firing)) return 98;
            transfer.observe(mode, firing, { .held = true });
            if (transfer.releaseRequested != grab_modes::usesToggleForRole(mode, firing)) return 99;
        }
        for (const bool firingIsLeft : { false, true }) {
            grab_modes::GripOccupancy pair{};
            (firingIsLeft ? pair.left : pair.right).firingGripActive = true;
            (firingIsLeft ? pair.right : pair.left).partGripActive = true;
            grab_modes::RuntimeState state{};
            grab_modes::adoptTransferredGrips(state, mode, 123, pair);
            const auto held = grab_modes::prepare(state, {
                .weaponGrabMode = mode, .inputAllowed = true, .weaponOwnershipKey = 123,
                .occupancy = pair, .left = { .held = true }, .right = { .held = true },
            });
            if (!held.left.held || !held.right.held || held.left.pressed || held.right.pressed) return 94;
            const auto opened = grab_modes::prepare(state, {
                .weaponGrabMode = mode, .inputAllowed = true, .weaponOwnershipKey = 123,
                .occupancy = pair, .left = { .released = true }, .right = { .released = true },
            });
            if (opened.left.held != grab_modes::usesToggleForRole(mode, firingIsLeft) ||
                opened.right.held != grab_modes::usesToggleForRole(mode, !firingIsLeft)) return 95;
        }
        for (const bool supportIsLeft : { false, true }) {
            grab_modes::GripOccupancy supportOnly{};
            (supportIsLeft ? supportOnly.left : supportOnly.right).partGripActive = true;
            grab_modes::RuntimeState state{};
            grab_modes::adoptTransferredGrips(state, mode, 123, supportOnly);
            const auto opened = grab_modes::prepare(state, {
                .weaponGrabMode = mode, .inputAllowed = true, .weaponOwnershipKey = 123,
                .occupancy = supportOnly,
                .left = { .released = supportIsLeft }, .right = { .released = !supportIsLeft },
            });
            const auto& support = supportIsLeft ? opened.left : opened.right;
            const auto& freeHand = supportIsLeft ? opened.right : opened.left;
            if (support.held != grab_modes::usesToggleForRole(mode, false) || freeHand.held || freeHand.pressed) return 100;
            const auto pressed = grab_modes::prepare(state, {
                .weaponGrabMode = mode, .inputAllowed = true, .weaponOwnershipKey = 123,
                .occupancy = supportOnly,
                .left = { .held = supportIsLeft, .pressed = supportIsLeft },
                .right = { .held = !supportIsLeft, .pressed = !supportIsLeft },
            });
            const auto& nextSupport = supportIsLeft ? pressed.left : pressed.right;
            if (nextSupport.released != grab_modes::usesToggleForRole(mode, false)) return 101;
        }
    }
    bool ok = true;

    ok &= testSelectionCleanupKeepsHeldOwnership();

    ok &= testRecoilProfiles();
    ok &= testScopeRollCalibration();

    ok &= testNativeGripFrames();

    ok &= testSupportRelease();

    ok &= testPoseHandoffResidual();

    ok &= testSupportInputFrames();

    ok &= testPrimaryDriverFrames();

    ok &= testWeaponGripTransforms();

    ok &= testAttachRelativeFrames();

    ok &= expectTrue("one-hand sword is melee", rock::weapon_type_policy::isMelee(TestWeaponType::kOneHandSword));
    ok &= expectTrue("two-hand axe is melee", rock::weapon_type_policy::isMelee(TestWeaponType::kTwoHandAxe));
    ok &= expectFalse("gun does not bit-alias melee", rock::weapon_type_policy::isMelee(TestWeaponType::kGun));
    ok &= expectFalse("grenade is not melee", rock::weapon_type_policy::isMelee(TestWeaponType::kGrenade));
    ok &= expectTrue("equipped hand-to-hand weapon retains its combat pose", rock::weapon_type_policy::isEquippedMelee(TestWeaponType::kHandToHand));
    ok &= expectTrue("equipped sword retains its combat pose", rock::weapon_type_policy::isEquippedMelee(TestWeaponType::kOneHandSword));
    ok &= expectFalse("equipped grenade cannot preserve the bare-fist pose", rock::weapon_type_policy::isEquippedMelee(TestWeaponType::kGrenade));
    ok &= expectFalse("equipped mine cannot preserve the bare-fist pose", rock::weapon_type_policy::isEquippedMelee(TestWeaponType::kMine));
    ok &= expectFalse("equipped gun is not an inventory melee witness", rock::weapon_type_policy::isEquippedMelee(TestWeaponType::kGun));

    {
        using namespace rock::collision_suppression_registry;
        PureCollisionSuppressionRegistry registry;
        constexpr auto body = 71u;
        auto grab = registry.acquire(body, CollisionSuppressionOwner::Grab, 0x35);
        auto grenade = registry.acquire(body, CollisionSuppressionOwner::NativeGrenadeThrow, grab.filterAfter);
        ok &= expectTrue("grenade protection joins existing collision ownership", grenade.valid && grenade.activeLeaseCount == 2);
        auto restored = registry.release(body, CollisionSuppressionOwner::NativeGrenadeThrow, grenade.filterAfter);
        ok &= expectTrue("grenade expiry preserves an active grab's suppression", restored.valid && !restored.bodyFullyReleased &&
            (restored.filterAfter & kSuppressionNoCollideBit));
        restored = registry.release(body, CollisionSuppressionOwner::Grab, restored.filterAfter);
        ok &= expectTrue("last collision owner restores the original filter", restored.bodyFullyReleased && restored.filterAfter == 0x35);
        grenade = registry.acquire(body, CollisionSuppressionOwner::NativeGrenadeThrow, 0x35);
        restored = registry.release(body, CollisionSuppressionOwner::NativeGrenadeThrow, grenade.filterAfter);
        ok &= expectTrue("standalone grenade protection restores collision", restored.bodyFullyReleased && restored.filterAfter == 0x35);
        grab = registry.acquire(body, CollisionSuppressionOwner::Grab, 0x35);
        const auto missingPose = registry.acquire(body, CollisionSuppressionOwner::InvalidFinalPose, grab.filterAfter);
        restored = registry.release(body, CollisionSuppressionOwner::Grab, missingPose.filterAfter);
        ok &= expectTrue("release of a constrained palm cannot enable collision while its pose is missing",
            !restored.bodyFullyReleased && (restored.filterAfter & kSuppressionNoCollideBit));
        restored = registry.release(body, CollisionSuppressionOwner::InvalidFinalPose, restored.filterAfter);
        ok &= expectTrue("pose recovery restores collision after the other owner has released",
            restored.bodyFullyReleased && restored.filterAfter == 0x35);
        const auto missingOnly = registry.acquire(body, CollisionSuppressionOwner::InvalidFinalPose, 0x35);
        grab = registry.acquire(body, CollisionSuppressionOwner::Grab, missingOnly.filterAfter);
        restored = registry.release(body, CollisionSuppressionOwner::InvalidFinalPose, grab.filterAfter);
        ok &= expectTrue("pose recovery preserves a continuing grab's collision suppression",
            !restored.bodyFullyReleased && (restored.filterAfter & kSuppressionNoCollideBit));
        (void)registry.release(body, CollisionSuppressionOwner::Grab, restored.filterAfter);
        DelayedRestoreTimer grace;
        ok &= expectTrue("grenade release starts the collision grace period", grace.begin(body, 1, 0.5f));
        ok &= expectFalse("grenade collision stays off inside the grace period", grace.advance(true, 0.4f));
        ok &= expectTrue("another grenade renews the full grace period", grace.begin(body, 1, 0.5f));
        ok &= expectFalse("the earlier throw cannot restore collisions during the next throw", grace.advance(true, 0.4f));
        ok &= expectTrue("grenade collision grace expires after the final throw", grace.advance(true, 0.1f));
    }

    ok &= testLiveHandDriver();

    ok &= testRawHandDriver();

    {
        TestTransform oneHandWeapon =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        const TestVector3 primaryGripLocal{ 2.0f, 1.0f, -1.0f };
        const TestVector3 supportGripLocal{ 2.0f, 11.0f, -1.0f };
        const TestVector3 primaryTarget{ 100.0f, 50.0f, 20.0f };
        const TestVector3 supportTarget{ 110.0f, 50.0f, 20.0f };
        oneHandWeapon.translate = { 98.0f, 49.0f, 21.0f };

        rock::WeaponTwoHandedSolverInput<TestTransform, TestVector3>
            solverInput{};
        solverInput.weaponWorldTransform = oneHandWeapon;
        solverInput.primaryGripLocal = primaryGripLocal;
        solverInput.supportGripLocal = supportGripLocal;
        solverInput.primaryTargetWorld = primaryTarget;
        solverInput.supportTargetWorld = supportTarget;
        solverInput.supportNormalLocal = { 0.0f, 0.0f, 1.0f };
        solverInput.supportNormalTargetWorld = { 0.0f, 1.0f, 0.0f };
        solverInput.useSupportNormalTwist = true;
        solverInput.supportNormalTwistFactor = 0.5f;

        const auto fullSolve =
            rock::solveTwoHandedWeaponTransformFrikPivot(solverInput);
        auto axisOnlyInput = solverInput;
        axisOnlyInput.useSupportNormalTwist = false;
        axisOnlyInput.supportNormalTwistFactor = 0.0f;
        const auto axisOnlySolve =
            rock::solveTwoHandedWeaponTransformFrikPivot(axisOnlyInput);
        ok &= expectTrue(
            "dynamic acquisition fixture full solve succeeds",
            fullSolve.solved);
        ok &= expectTrue(
            "dynamic acquisition fixture axis solve succeeds",
            axisOnlySolve.solved);

        const float fullCorrectionRadians =
            rock::weapon_support_acquisition_math::
                rotationAngleRadians(fullSolve.rotationDelta);
        const float twistContributionRadians =
            rock::weapon_support_acquisition_math::
                rotationDistanceRadians(
                    axisOnlySolve.rotationDelta,
                    fullSolve.rotationDelta);
        ok &= expectTrue(
            "dynamic acquisition composite includes support normal twist",
            twistContributionRadians >
                1.0f * 0.01745329251994329577f);

        constexpr std::array<float, 5> kAcquisitionAlphas{
            0.0f,
            0.25f,
            0.5f,
            0.75f,
            1.0f,
        };
        for (const float alpha : kAcquisitionAlphas) {
            const auto acquired =
                rock::weapon_support_acquisition_math::
                    applyRotationAroundPrimaryPivot<
                        TestTransform,
                        TestVector3>(
                        oneHandWeapon,
                        fullSolve.rotationDelta,
                        primaryGripLocal,
                        primaryTarget,
                        alpha);
            ok &= expectTrue(
                "dynamic acquisition partial solve stays valid",
                acquired.valid);
            const TestVector3 primaryWorld =
                rock::transform_math::localPointToWorld(
                    acquired.weaponWorldTransform,
                    primaryGripLocal);
            ok &= expectNear(
                "dynamic acquisition keeps primary pivot x exact",
                primaryWorld.x,
                primaryTarget.x);
            ok &= expectNear(
                "dynamic acquisition keeps primary pivot y exact",
                primaryWorld.y,
                primaryTarget.y);
            ok &= expectNear(
                "dynamic acquisition keeps primary pivot z exact",
                primaryWorld.z,
                primaryTarget.z);
            ok &= expectNear(
                "dynamic acquisition slerps the complete correction",
                acquired.appliedRotationRadians,
                fullCorrectionRadians * alpha,
                0.001f);
            if (alpha == 0.0f) {
                ok &= expectTransformNear(
                    "dynamic acquisition alpha zero preserves one-hand frame",
                    acquired.weaponWorldTransform,
                    oneHandWeapon);
            }
            if (alpha == 1.0f) {
                ok &= expectTransformNear(
                    "dynamic acquisition alpha one matches full solver",
                    acquired.weaponWorldTransform,
                    fullSolve.weaponWorldTransform);
            }
        }

        const TestVector3 movingPrimaryTarget{
            primaryTarget.x + 7.0f,
            primaryTarget.y - 4.0f,
            primaryTarget.z + 3.0f,
        };
        const auto movingPrimaryAcquire =
            rock::weapon_support_acquisition_math::
                applyRotationAroundPrimaryPivot<
                    TestTransform,
                    TestVector3>(
                    oneHandWeapon,
                    fullSolve.rotationDelta,
                    primaryGripLocal,
                    movingPrimaryTarget,
                    0.35f);
        const TestVector3 movingPrimaryWorld =
            rock::transform_math::localPointToWorld(
                movingPrimaryAcquire.weaponWorldTransform,
                primaryGripLocal);
        ok &= expectNear(
            "dynamic acquisition follows moving primary x immediately",
            movingPrimaryWorld.x,
            movingPrimaryTarget.x);
        ok &= expectNear(
            "dynamic acquisition follows moving primary y immediately",
            movingPrimaryWorld.y,
            movingPrimaryTarget.y);
        ok &= expectNear(
            "dynamic acquisition follows moving primary z immediately",
            movingPrimaryWorld.z,
            movingPrimaryTarget.z);

        const TestMatrix3 almostFullTurn =
            makeAxisAngleRotation(
                TestVector3{ 0.0f, 0.0f, 1.0f },
                350.0f);
        const auto shortestArcAcquire =
            rock::weapon_support_acquisition_math::
                applyRotationAroundPrimaryPivot<
                    TestTransform,
                    TestVector3>(
                    oneHandWeapon,
                    almostFullTurn,
                    primaryGripLocal,
                    primaryTarget,
                    0.5f);
        ok &= expectNear(
            "dynamic acquisition uses quaternion shortest arc",
            shortestArcAcquire.appliedRotationRadians,
            5.0f * 0.01745329251994329577f,
            0.001f);

        ok &= expectNear(
            "dynamic acquisition smoothstep quarter",
            rock::weapon_support_acquisition_math::smoothStepAlpha(
                0.25f),
            0.15625f);
        ok &= expectNear(
            "dynamic acquisition zero duration reaches endpoint",
            rock::weapon_support_acquisition_math::
                timedSmoothStepAlpha(0.0f, 0.0f),
            1.0f);

        TestMatrix3 invalidRotation = fullSolve.rotationDelta;
        invalidRotation.entry[0][0] =
            (std::numeric_limits<float>::quiet_NaN)();
        const auto invalidAcquire =
            rock::weapon_support_acquisition_math::
                applyRotationAroundPrimaryPivot<
                    TestTransform,
                    TestVector3>(
                    oneHandWeapon,
                    invalidRotation,
                    primaryGripLocal,
                    primaryTarget,
                    0.5f);
        ok &= expectFalse(
            "dynamic acquisition rejects non-finite correction",
            invalidAcquire.valid);
        ok &= expectTransformNear(
            "dynamic acquisition non-finite correction fails closed",
            invalidAcquire.weaponWorldTransform,
            oneHandWeapon);
    }

    ok &= testWeaponPresentationDelta();

    {
        TestTransform nativeKickLocal =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        nativeKickLocal.rotate = makeAxisAngleRotation(
            TestVector3{ 0.0f, 0.0f, 1.0f },
            60.0f);
        nativeKickLocal.translate = { 10.0f, -4.0f, 2.0f };

        TestTransform controlledKickLocal{};
        ok &= expectTrue(
            "visual-only support builds a controlled rigid recoil sample",
            rock::weapon_recoil_authority_math::
                tryBuildControlledKick(
                    nativeKickLocal,
                    rock::weapon_recoil_policy::kCloseSupport,
                    controlledKickLocal));
        TestTransform expectedControlledKick =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        expectedControlledKick.rotate = makeAxisAngleRotation(
            TestVector3{ 0.0f, 0.0f, 1.0f },
            18.0f);
        expectedControlledKick.translate = { 4.5f, -1.8f, 0.9f };
        ok &= expectTransformNear(
            "visual-only support reduces angular recoil more than linear impulse",
            controlledKickLocal,
            expectedControlledKick);

        TestTransform invalidKick = nativeKickLocal;
        invalidKick.rotate.entry[0][0] =
            (std::numeric_limits<float>::quiet_NaN)();
        TestTransform rejectedKick = nativeKickLocal;
        ok &= expectFalse(
            "visual-only support rejects a non-finite native kick",
            rock::weapon_recoil_authority_math::
                tryBuildControlledKick(
                    invalidKick,
                    rock::weapon_recoil_policy::kCloseSupport,
                    rejectedKick));
        ok &= expectTransformNear(
            "invalid visual-only recoil fails closed to identity",
            rejectedKick,
            rock::transform_math::makeIdentityTransform<TestTransform>());

        TestTransform primaryWand =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        primaryWand.translate = { 18.0f, -4.0f, 7.0f };
        primaryWand.rotate = makeAxisAngleRotation(
            TestVector3{ 0.0f, 0.0f, 1.0f },
            12.0f);
        TestTransform offhandWand =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        offhandWand.translate = { -16.0f, -3.0f, 8.0f };
        offhandWand.rotate = makeAxisAngleRotation(
            TestVector3{ 0.0f, 0.0f, 1.0f },
            -11.0f);
        TestTransform kickParentInPrimary =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        kickParentInPrimary.translate = { 1.0f, 3.0f, -2.0f };
        kickParentInPrimary.rotate = makeAxisAngleRotation(
            TestVector3{ 1.0f, 0.0f, 0.0f },
            8.0f);
        const TestTransform kickParentWorld =
            rock::transform_math::composeTransforms(
                primaryWand,
                kickParentInPrimary);
        TestTransform primaryHandInWand =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        primaryHandInWand.translate = { 2.0f, 5.0f, 1.0f };
        primaryHandInWand.rotate = makeAxisAngleRotation(
            TestVector3{ 0.0f, 1.0f, 0.0f },
            15.0f);
        const TestTransform primaryHandWorld =
            rock::transform_math::composeTransforms(
                primaryWand,
                primaryHandInWand);
        const TestTransform mirroredHandWorld =
            rock::transform_math::composeTransforms(
                offhandWand,
                rock::weapon_recoil_authority_math::
                    mirrorLocalAcrossSagittal(primaryHandInWand));
        const TestTransform primaryWorldDelta =
            rock::weapon_recoil_authority_math::resolveWorldDelta(
                nativeKickLocal,
                kickParentWorld,
                primaryWand,
                offhandWand,
                false);
        const TestTransform mirroredWorldDelta =
            rock::weapon_recoil_authority_math::resolveWorldDelta(
                nativeKickLocal,
                kickParentWorld,
                primaryWand,
                offhandWand,
                true);
        const TestTransform recoiledPrimaryHand =
            rock::transform_math::composeTransforms(
                primaryWorldDelta,
                primaryHandWorld);
        const TestTransform expectedMirroredRecoiledHand =
            rock::transform_math::composeTransforms(
                offhandWand,
                rock::weapon_recoil_authority_math::
                    mirrorLocalAcrossSagittal(
                        rock::transform_math::composeTransforms(
                            rock::transform_math::invertTransform(
                                primaryWand),
                            recoiledPrimaryHand)));
        const TestTransform actualMirroredRecoiledHand =
            rock::transform_math::composeTransforms(
                mirroredWorldDelta,
                mirroredHandWorld);
        ok &= expectTransformNear(
            "left firing recoil is the wand-conjugated mirror of right firing recoil",
            actualMirroredRecoiledHand,
            expectedMirroredRecoiledHand);
    }

    {
        TestTransform weaponBefore = rock::transform_math::makeIdentityTransform<TestTransform>();
        weaponBefore.translate = { 10.0f, 20.0f, 30.0f };
        TestTransform scopeBefore = rock::transform_math::makeIdentityTransform<TestTransform>();
        scopeBefore.rotate = {};
        scopeBefore.rotate.entry[0][1] = 1.0f;
        scopeBefore.rotate.entry[1][2] = 1.0f;
        scopeBefore.rotate.entry[2][0] = 1.0f;
        scopeBefore.translate = { 12.0f, 24.0f, 35.0f };
        TestTransform weaponAfter = weaponBefore;
        weaponAfter.translate = { 17.0f, 16.0f, 32.0f };
        weaponAfter.rotate.entry[0][0] = 0.0f;
        weaponAfter.rotate.entry[0][1] = 1.0f;
        weaponAfter.rotate.entry[1][0] = -1.0f;
        weaponAfter.rotate.entry[1][1] = 0.0f;
        const TestTransform relativeBefore = rock::transform_math::composeTransforms(
            rock::transform_math::invertTransform(weaponBefore),
            scopeBefore);

        const TestVector3 sightBoundsMin{ -4.0f, 8.0f, 7.0f };
        const TestVector3 sightBoundsMax{ 6.0f, 38.0f, 15.0f };
        const TestVector3 sightAnchor = rock::native_scope_camera_follow_math::rearPlaneCenterFromSightBounds(sightBoundsMin, sightBoundsMax);
        ok &= expectNear("native scope sight anchor centers lateral bounds", sightAnchor.x, 1.0f);
        ok &= expectNear("native scope sight anchor uses rear forward plane", sightAnchor.y, 8.0f);
        ok &= expectNear("native scope sight anchor centers vertical bounds", sightAnchor.z, 11.0f);

        const TestVector3 firingGripWeaponLocal{ -2.0f, 3.0f, 1.0f };
        const TestVector3 fallbackOffsetWeaponLocal{ 1.5f, 12.0f, 7.0f };
        const auto generatedResolution =
            rock::native_scope_sight_anchor_policy::resolve(
                false,
                true,
                sightAnchor,
                true,
                firingGripWeaponLocal,
                fallbackOffsetWeaponLocal);
        ok &= expectTrue("valid generated sight remains the preferred scope anchor",
            generatedResolution.valid &&
                generatedResolution.source ==
                    rock::native_scope_sight_anchor_policy::AnchorSource::GeneratedSight);
        ok &= expectNear("generated scope anchor keeps sight x", generatedResolution.weaponLocal.x, sightAnchor.x);
        ok &= expectNear("generated scope anchor keeps sight y", generatedResolution.weaponLocal.y, sightAnchor.y);
        ok &= expectNear("generated scope anchor keeps sight z", generatedResolution.weaponLocal.z, sightAnchor.z);

        const auto missingGeometryResolution =
            rock::native_scope_sight_anchor_policy::resolve(
                false,
                false,
                TestVector3{},
                true,
                firingGripWeaponLocal,
                fallbackOffsetWeaponLocal);
        ok &= expectTrue("missing scope geometry selects the firing-grip fallback",
            missingGeometryResolution.valid &&
                missingGeometryResolution.source ==
                    rock::native_scope_sight_anchor_policy::AnchorSource::FiringGripFallback);
        ok &= expectNear("firing-grip fallback adds lateral offset", missingGeometryResolution.weaponLocal.x, -0.5f);
        ok &= expectNear("firing-grip fallback adds forward offset", missingGeometryResolution.weaponLocal.y, 15.0f);
        ok &= expectNear("firing-grip fallback adds vertical offset", missingGeometryResolution.weaponLocal.z, 8.0f);

        const auto forcedFallbackResolution =
            rock::native_scope_sight_anchor_policy::resolve(
                true,
                true,
                sightAnchor,
                true,
                firingGripWeaponLocal,
                fallbackOffsetWeaponLocal);
        ok &= expectTrue("forced fallback bypasses an incorrectly accepted sight collider",
            forcedFallbackResolution.valid &&
                forcedFallbackResolution.source ==
                    rock::native_scope_sight_anchor_policy::AnchorSource::FiringGripFallback);
        const auto unavailableFallbackResolution =
            rock::native_scope_sight_anchor_policy::resolve(
                true,
                true,
                sightAnchor,
                false,
                TestVector3{},
                fallbackOffsetWeaponLocal);
        ok &= expectFalse("forced fallback fails closed without a current firing grip",
            unavailableFallbackResolution.valid);
        TestVector3 invalidGeneratedSight = sightAnchor;
        invalidGeneratedSight.x =
            (std::numeric_limits<float>::quiet_NaN)();
        const auto invalidGeometryResolution =
            rock::native_scope_sight_anchor_policy::resolve(
                false,
                true,
                invalidGeneratedSight,
                true,
                firingGripWeaponLocal,
                fallbackOffsetWeaponLocal);
        ok &= expectTrue("non-finite generated sight falls back to the firing grip",
            invalidGeometryResolution.valid &&
                invalidGeometryResolution.source ==
                    rock::native_scope_sight_anchor_policy::AnchorSource::FiringGripFallback);
        TestVector3 invalidFallbackOffset = fallbackOffsetWeaponLocal;
        invalidFallbackOffset.z =
            (std::numeric_limits<float>::infinity)();
        const auto invalidFallbackResolution =
            rock::native_scope_sight_anchor_policy::resolve(
                true,
                false,
                TestVector3{},
                true,
                firingGripWeaponLocal,
                invalidFallbackOffset);
        ok &= expectFalse("non-finite firing-grip offset fails closed",
            invalidFallbackResolution.valid);

        TestTransform rigidSightFrameLocal{};
        ok &= expectTrue("native scope captures a complete weapon-relative frame",
            rock::native_scope_camera_follow_math::tryCaptureRigidAnchorFrameWeaponLocal(
                weaponBefore,
                scopeBefore,
                sightAnchor,
                rigidSightFrameLocal));
        const TestTransform anchoredScopeAfter =
            rock::native_scope_camera_follow_math::resolveRigidAnchorFrameWorld(
                weaponAfter,
                rigidSightFrameLocal);
        const TestTransform anchoredRelativeAfter = rock::transform_math::composeTransforms(rock::transform_math::invertTransform(weaponAfter), anchoredScopeAfter);
        ok &= expectNear("native scope camera replaces controller-relative lateral position", anchoredRelativeAfter.translate.x, sightAnchor.x);
        ok &= expectNear("native scope camera replaces controller-relative forward position", anchoredRelativeAfter.translate.y, sightAnchor.y);
        ok &= expectNear("native scope camera replaces controller-relative vertical position", anchoredRelativeAfter.translate.z, sightAnchor.z);
        ok &= expectNear("native scope camera preserves calibrated scale", anchoredRelativeAfter.scale, relativeBefore.scale);
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                ok &= expectNear("native scope camera preserves calibrated rotation", anchoredRelativeAfter.rotate.entry[row][column], relativeBefore.rotate.entry[row][column]);
            }
        }

        TestTransform fallbackCameraBase = rigidSightFrameLocal;
        // Isolate weapon-axis offset checks; nonidentity calibration is tested below.
        fallbackCameraBase.rotate = rock::transform_math::makeIdentityRotation<TestMatrix3>();
        fallbackCameraBase.translate =
            missingGeometryResolution.weaponLocal;
        const TestTransform zeroFallbackRotation =
            rock::native_scope_camera_follow_math::
                applyWeaponLocalRotationOffset(
                    fallbackCameraBase,
                    0.0f,
                    0.0f,
                    0.0f);
        ok &= expectTransformNear(
            "zero fallback rotation preserves native camera calibration",
            zeroFallbackRotation,
            fallbackCameraBase);

        const TestTransform pitchFallbackRotation =
            rock::native_scope_camera_follow_math::
                applyWeaponLocalRotationOffset(
                    fallbackCameraBase,
                    90.0f,
                    0.0f,
                    0.0f);
        ok &= expectNear("fallback pitch keeps firing-grip anchor x", pitchFallbackRotation.translate.x, fallbackCameraBase.translate.x);
        ok &= expectNear("fallback pitch keeps firing-grip anchor y", pitchFallbackRotation.translate.y, fallbackCameraBase.translate.y);
        ok &= expectNear("fallback pitch keeps firing-grip anchor z", pitchFallbackRotation.translate.z, fallbackCameraBase.translate.z);
        ok &= expectNear("fallback pitch preserves native camera scale", pitchFallbackRotation.scale, fallbackCameraBase.scale);
        ok &= expectNear("fallback pitch rotates local Y toward Z", pitchFallbackRotation.rotate.entry[1][2], 1.0f);
        ok &= expectNear("fallback pitch rotates local Z toward negative Y", pitchFallbackRotation.rotate.entry[2][1], -1.0f);

        const TestTransform yawFallbackRotation =
            rock::native_scope_camera_follow_math::
                applyWeaponLocalRotationOffset(
                    fallbackCameraBase,
                    0.0f,
                    90.0f,
                    0.0f);
        ok &= expectNear("fallback yaw rotates local X toward Y", yawFallbackRotation.rotate.entry[0][1], 1.0f);
        ok &= expectNear("fallback yaw rotates local Y toward negative X", yawFallbackRotation.rotate.entry[1][0], -1.0f);

        const TestTransform rollFallbackRotation =
            rock::native_scope_camera_follow_math::
                applyWeaponLocalRotationOffset(
                    fallbackCameraBase,
                    0.0f,
                    0.0f,
                    90.0f);
        ok &= expectNear("fallback roll rotates local X toward negative Z", rollFallbackRotation.rotate.entry[0][2], -1.0f);
        ok &= expectNear("fallback roll rotates local Z toward X", rollFallbackRotation.rotate.entry[2][0], 1.0f);

        TestTransform calibratedFallbackCamera = fallbackCameraBase;
        calibratedFallbackCamera.rotate =
            yawFallbackRotation.rotate;
        const TestTransform weaponAxisPitchFallback =
            rock::native_scope_camera_follow_math::
                applyWeaponLocalRotationOffset(
                    calibratedFallbackCamera,
                    90.0f,
                    0.0f,
                    0.0f);
        ok &= expectNear("fallback pitch uses weapon X after nonidentity native calibration row0 x", weaponAxisPitchFallback.rotate.entry[0][0], 0.0f);
        ok &= expectNear("fallback pitch uses weapon X after nonidentity native calibration row0 y", weaponAxisPitchFallback.rotate.entry[0][1], 0.0f);
        ok &= expectNear("fallback pitch uses weapon X after nonidentity native calibration row0 z", weaponAxisPitchFallback.rotate.entry[0][2], 1.0f);
        ok &= expectNear("fallback pitch uses weapon X after nonidentity native calibration row1 x", weaponAxisPitchFallback.rotate.entry[1][0], -1.0f);
        ok &= expectNear("fallback pitch uses weapon X after nonidentity native calibration row2 y", weaponAxisPitchFallback.rotate.entry[2][1], -1.0f);
        ok &= expectNear("weapon-axis fallback rotation never orbits the anchor x", weaponAxisPitchFallback.translate.x, fallbackCameraBase.translate.x);
        ok &= expectNear("weapon-axis fallback rotation never orbits the anchor y", weaponAxisPitchFallback.translate.y, fallbackCameraBase.translate.y);
        ok &= expectNear("weapon-axis fallback rotation never orbits the anchor z", weaponAxisPitchFallback.translate.z, fallbackCameraBase.translate.z);

        ok &= expectNear("rigid scope frame stores generated sight x", rigidSightFrameLocal.translate.x, sightAnchor.x);
        ok &= expectNear("rigid scope frame stores generated sight y", rigidSightFrameLocal.translate.y, sightAnchor.y);
        ok &= expectNear("rigid scope frame stores generated sight z", rigidSightFrameLocal.translate.z, sightAnchor.z);
        const std::array<TestTransform, 3> handModeWeaponFrames{
            weaponBefore,
            weaponAfter,
            [] {
                TestTransform leftFiring = rock::transform_math::makeIdentityTransform<TestTransform>();
                leftFiring.translate = { -18.0f, 6.0f, 42.0f };
                leftFiring.rotate.entry[0][0] = -1.0f;
                leftFiring.rotate.entry[1][1] = -1.0f;
                return leftFiring;
            }(),
        };
        for (const TestTransform& handModeWeaponFrame : handModeWeaponFrames) {
            const TestTransform rigidScopeWorld = rock::native_scope_camera_follow_math::resolveRigidAnchorFrameWorld(handModeWeaponFrame, rigidSightFrameLocal);
            const TestTransform resolvedLocal = rock::transform_math::composeTransforms(rock::transform_math::invertTransform(handModeWeaponFrame), rigidScopeWorld);
            ok &= expectTransformNear("one-hand, two-hand, and left-hand modes preserve one rigid scope frame", resolvedLocal, rigidSightFrameLocal);
        }

        TestTransform fineTunedRotatedWeapon = weaponAfter;
        fineTunedRotatedWeapon.rotate = makeAxisAngleRotation(
            rock::weaponSolverNormalize(
                TestVector3{ 0.2f, 0.9f, 0.3f }),
            -10.0f);
        const TestTransform fineTunedScopeWorld =
            rock::native_scope_camera_follow_math::
                resolveRigidAnchorFrameWorld(
                    fineTunedRotatedWeapon,
                    rigidSightFrameLocal);
        const TestTransform fineTunedScopeResolvedLocal =
            rock::transform_math::composeTransforms(
                rock::transform_math::invertTransform(
                    fineTunedRotatedWeapon),
                fineTunedScopeWorld);
        ok &= expectTransformNear(
            "fine-tuned rotated weapon preserves retained rigid scope frame",
            fineTunedScopeResolvedLocal,
            rigidSightFrameLocal);

        const TestTransform equippedSightBaseline =
            rock::native_scope_camera_follow_math::resolveRigidAnchorFrameWorld(
                weaponBefore,
                rigidSightFrameLocal);
        const TestVector3 expectedSightAnchorWorld = rock::transform_math::localPointToWorld(weaponBefore, sightAnchor);
        ok &= expectNear("equipped scope baseline uses sight world position x", equippedSightBaseline.translate.x, expectedSightAnchorWorld.x);
        ok &= expectNear("equipped scope baseline uses sight world position y", equippedSightBaseline.translate.y, expectedSightAnchorWorld.y);
        ok &= expectNear("equipped scope baseline uses sight world position z", equippedSightBaseline.translate.z, expectedSightAnchorWorld.z);
        ok &= expectNear("equipped scope baseline preserves hFRIK camera scale", equippedSightBaseline.scale, scopeBefore.scale);
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                ok &= expectNear("equipped scope baseline preserves hFRIK camera rotation", equippedSightBaseline.rotate.entry[row][column], scopeBefore.rotate.entry[row][column]);
            }
        }

        TestTransform nativeModelRootInCameraLocal = rock::transform_math::makeIdentityTransform<TestTransform>();
        nativeModelRootInCameraLocal.translate = { 6.0f, -30.0f, 2.0f };
        nativeModelRootInCameraLocal.rotate.entry[1][1] = 0.0f;
        nativeModelRootInCameraLocal.rotate.entry[1][2] = 1.0f;
        nativeModelRootInCameraLocal.rotate.entry[2][1] = -1.0f;
        nativeModelRootInCameraLocal.rotate.entry[2][2] = 0.0f;
        const TestTransform nativeScopeModelRootWorld = rock::transform_math::composeTransforms(
            scopeBefore,
            nativeModelRootInCameraLocal);

        auto bridgeBase = rock::transform_math::makeIdentityTransform<TestTransform>();
        const TestVector3 primaryGrip{ 1.0f, -2.0f, 3.0f };
        const TestVector3 supportGrip{ 1.0f, 18.0f, 3.0f };
        const auto stillBridge = rock::equip_visual_bridge_policy::solvePairedBridge(
            bridgeBase, primaryGrip, supportGrip, supportGrip);
        ok &= expectTrue("paired bridge starts with its captured loose pose", stillBridge.solved);
        ok &= expectTransformNear("unchanged controllers cannot snap the bridge", stillBridge.weaponWorldTransform, bridgeBase);
        const auto movedBridge = rock::equip_visual_bridge_policy::solvePairedBridge(
            bridgeBase, primaryGrip, supportGrip, TestVector3{ 21.0f, -2.0f, 3.0f });
        ok &= expectTrue("support-controller motion steers the bridge before equipped adoption", movedBridge.solved);
        const auto primaryAfter = rock::transform_math::localPointToWorld(movedBridge.weaponWorldTransform, primaryGrip);
        const auto supportAfter = rock::transform_math::localPointToWorld(movedBridge.weaponWorldTransform, supportGrip);
        ok &= expectNear("paired bridge keeps firing pivot x", primaryAfter.x, primaryGrip.x);
        ok &= expectNear("paired bridge keeps firing pivot y", primaryAfter.y, primaryGrip.y);
        ok &= expectNear("paired bridge follows support x", supportAfter.x, 21.0f);
        ok &= expectNear("paired bridge follows support y", supportAfter.y, -2.0f);
        const auto coincidentBridge = rock::equip_visual_bridge_policy::solvePairedBridge(
            bridgeBase, primaryGrip, primaryGrip, supportGrip);
        ok &= expectFalse("coincident seats do not invent a two-hand aim axis", coincidentBridge.solved);
        const TestVector3 sourceRegistration{ 0.0f, 2.0f, 0.0f };
        const TestVector3 equippedRegistration{ 0.0f, 10.0f, 0.0f };
        auto registeredGrip = rock::transform_math::makeIdentityTransform<TestTransform>();
        registeredGrip.translate = { 1.0f, 6.0f, 3.0f };
        auto looseGrip = registeredGrip;
        looseGrip.translate.y -= 8.0f;
        for (const auto& equippedWorld : handModeWeaponFrames) {
            const auto bridgeWorld = rock::equip_visual_bridge_policy::registeredLooseWorld(
                equippedWorld, sourceRegistration, equippedRegistration);
            ok &= expectTransformNear("loose and equipped grips coincide after full-pose registration",
                rock::transform_math::composeTransforms(bridgeWorld, looseGrip),
                rock::transform_math::composeTransforms(equippedWorld, registeredGrip));
        }
        const TestTransform modelRootCalibration =
            rock::native_scope_overlay_follow_math::captureModelRootCalibrationInCameraLocal(
                scopeBefore,
                nativeScopeModelRootWorld);
        ok &= expectNear("native scope overlay discards obsolete lateral calibration", modelRootCalibration.translate.x, 0.0f);
        ok &= expectNear("native scope overlay discards obsolete depth calibration", modelRootCalibration.translate.y, 0.0f);
        ok &= expectNear("native scope overlay discards obsolete vertical calibration", modelRootCalibration.translate.z, 0.0f);
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                ok &= expectNear(
                    "native scope overlay preserves Bethesda model orientation",
                    modelRootCalibration.rotate.entry[row][column],
                    nativeModelRootInCameraLocal.rotate.entry[row][column]);
            }
        }

        const TestTransform zeroFineTune =
            rock::native_scope_overlay_follow_math::makeModelRootFineTuneLocal<TestTransform>(
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

        // First calibration must be independent of whether the gun was
        // already carried in two hands when its native overlay appeared.
        auto cameraParent = weaponBefore;
        auto cameraLocal = rock::transform_math::makeIdentityTransform<TestTransform>();
        cameraLocal.scale = 0.75f;
        const auto unsteered = rock::native_scope_overlay_follow_math::resolveUnsteeredCameraWorld(cameraParent, cameraLocal);
        const auto nativeOverlay = rock::transform_math::composeTransforms(unsteered, nativeModelRootInCameraLocal);
        for (float angle : { 0.0f, 37.0f, -81.0f, 165.0f }) {
            cameraLocal.rotate = makeAxisAngleRotation(TestVector3{ 0.0f, 0.0f, 1.0f }, angle);
            const auto baseline = rock::native_scope_overlay_follow_math::resolveUnsteeredCameraWorld(cameraParent, cameraLocal);
            const auto calibration = rock::native_scope_overlay_follow_math::captureModelRootCalibrationInCameraLocal(baseline, nativeOverlay);
            ok &= expectTransformNear("first-equip scope calibration excludes two-hand steering", calibration, modelRootCalibration);
            const auto aimedOverlay = rock::native_scope_overlay_follow_math::resolveScopeModelRootWorld(
                anchoredScopeAfter, calibration, zeroFineTune);
            const auto expectedOverlay = rock::transform_math::composeTransforms(anchoredScopeAfter, modelRootCalibration);
            ok &= expectTransformNear("scope housing follows final aim after paired equip", aimedOverlay, expectedOverlay);
        }
        const TestTransform targetScopeModelRoot =
            rock::native_scope_overlay_follow_math::resolveScopeModelRootWorld(
                anchoredScopeAfter,
                modelRootCalibration,
                zeroFineTune);
        const TestTransform expectedScopeModelRoot = rock::transform_math::composeTransforms(
            anchoredScopeAfter,
            modelRootCalibration);
        ok &= expectTransformNear(
            "native scope overlay applies Bethesda orientation at the generated sight",
            targetScopeModelRoot,
            expectedScopeModelRoot);
        ok &= expectNear("native scope overlay keeps generated sight lateral anchor", targetScopeModelRoot.translate.x, anchoredScopeAfter.translate.x);
        ok &= expectNear("native scope overlay keeps generated sight depth anchor", targetScopeModelRoot.translate.y, anchoredScopeAfter.translate.y);
        ok &= expectNear("native scope overlay keeps generated sight vertical anchor", targetScopeModelRoot.translate.z, anchoredScopeAfter.translate.z);

        TestTransform scopeModelRootLocal = rock::transform_math::makeIdentityTransform<TestTransform>();
        scopeModelRootLocal.translate = { 0.0f, -12.0f, 0.0f };
        const TestTransform correctedScopeParent =
            rock::native_scope_overlay_follow_math::resolveScopeParentWorldForModelRoot(
                targetScopeModelRoot,
                scopeModelRootLocal);
        const TestTransform correctedScopeModelRoot = rock::transform_math::composeTransforms(
            correctedScopeParent,
            scopeModelRootLocal);
        ok &= expectTransformNear(
            "native scope overlay compensates model-root depth after calibrated orientation",
            correctedScopeModelRoot,
            targetScopeModelRoot);

        const TestTransform fineTune =
            rock::native_scope_overlay_follow_math::makeModelRootFineTuneLocal<TestTransform>(
                1.0f, 2.0f, 3.0f, 90.0f, 0.0f, 0.0f);
        ok &= expectNear("native scope overlay INI lateral offset", fineTune.translate.x, 1.0f);
        ok &= expectNear("native scope overlay INI depth offset", fineTune.translate.y, 2.0f);
        ok &= expectNear("native scope overlay INI vertical offset", fineTune.translate.z, 3.0f);
        ok &= expectNear("native scope overlay INI pitch rotates local Y toward Z", fineTune.rotate.entry[1][2], 1.0f);
        ok &= expectNear("native scope overlay INI pitch rotates local Z toward negative Y", fineTune.rotate.entry[2][1], -1.0f);

        const TestTransform yawFineTune =
            rock::native_scope_overlay_follow_math::makeModelRootFineTuneLocal<TestTransform>(
                0.0f, 0.0f, 0.0f, 0.0f, 90.0f, 0.0f);
        ok &= expectNear("native scope overlay INI yaw rotates local X toward Y", yawFineTune.rotate.entry[0][1], 1.0f);
        ok &= expectNear("native scope overlay INI yaw rotates local Y toward negative X", yawFineTune.rotate.entry[1][0], -1.0f);

        const TestTransform rollFineTune =
            rock::native_scope_overlay_follow_math::makeModelRootFineTuneLocal<TestTransform>(
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 90.0f);
        ok &= expectNear("native scope overlay INI roll rotates local X toward negative Z", rollFineTune.rotate.entry[0][2], -1.0f);
        ok &= expectNear("native scope overlay INI roll rotates local Z toward X", rollFineTune.rotate.entry[2][0], 1.0f);
    }

    {
        TestTransform driverBefore = rock::transform_math::makeIdentityTransform<TestTransform>();
        driverBefore.translate = { 10.0f, -4.0f, 7.0f };
        TestTransform handBefore = rock::transform_math::makeIdentityTransform<TestTransform>();
        handBefore.translate = { 12.0f, -1.0f, 8.5f };

        const TestTransform driverToHand = rock::scope_safe_hand_frame_math::captureDriverToHandLocal(
            driverBefore,
            handBefore);

        TestTransform driverAfter = rock::transform_math::makeIdentityTransform<TestTransform>();
        driverAfter.translate = { -3.0f, 14.0f, 11.0f };
        driverAfter.rotate.entry[0][0] = 0.0f;
        driverAfter.rotate.entry[0][1] = 1.0f;
        driverAfter.rotate.entry[1][0] = -1.0f;
        driverAfter.rotate.entry[1][1] = 0.0f;

        const TestTransform handAfter = rock::scope_safe_hand_frame_math::resolveHandWorld(
            driverAfter,
            driverToHand);
        const TestTransform resolvedDriverToHand = rock::transform_math::composeTransforms(
            rock::transform_math::invertTransform(driverAfter),
            handAfter);
        ok &= expectTransformNear("scope-safe hand preserves hFRIK driver-local calibration", resolvedDriverToHand, driverToHand);
        using rock::scope_safe_hand_frame_math::ResolutionMode;
        ok &= expectEqual("visible body uses root-flattened hand authority",
            rock::scope_safe_hand_frame_math::resolveMode(false, true, true, true, 0, 3),
            ResolutionMode::RootFlattened);
        ok &= expectEqual("native scope ignores even finite root data and reconstructs from the hFRIK driver",
            rock::scope_safe_hand_frame_math::resolveMode(true, true, true, true, 0, 3),
            ResolutionMode::DriverReconstructed);
        ok &= expectEqual("native scope briefly freezes the last valid frame across a transient driver miss",
            rock::scope_safe_hand_frame_math::resolveMode(true, false, false, true, 0, 3),
            ResolutionMode::LastKnown);
        ok &= expectEqual("native scope stops freezing after the bounded driver-miss grace",
            rock::scope_safe_hand_frame_math::resolveMode(true, false, false, true, 3, 3),
            ResolutionMode::Unavailable);
        ok &= expectEqual("native scope seeds an uncalibrated hand from the isolated controller root",
            rock::scope_safe_hand_frame_math::resolveMode(true, true, false, false, 0, 3),
            ResolutionMode::RootFlattened);
        ok &= expectEqual("native scope still fails closed when the caller offers no root sample",
            rock::scope_safe_hand_frame_math::resolveMode(true, false, false, false, 0, 3),
            ResolutionMode::Unavailable);
        ok &= expectEqual("a seeded scope hand never outranks a live reconstruction",
            rock::scope_safe_hand_frame_math::resolveMode(true, true, true, false, 0, 3),
            ResolutionMode::DriverReconstructed);
        ok &= expectTrue("an uncalibrated scoped hand may sample a calibrated controller root",
            rock::scope_safe_hand_frame_math::canSampleScopeRootHand(true, false, true));
        ok &= expectFalse("a calibrated scoped hand never re-samples the root",
            rock::scope_safe_hand_frame_math::canSampleScopeRootHand(true, true, true));
        ok &= expectFalse("a contaminated raw hand may not seed a scoped hand",
            rock::scope_safe_hand_frame_math::canSampleScopeRootHand(true, false, false));
        ok &= expectTrue("unscoped frames always sample the root",
            rock::scope_safe_hand_frame_math::canSampleScopeRootHand(false, true, false));
        ok &= expectTrue("a placed hand near its controller may seed a scoped frame",
            rock::scope_safe_hand_frame_math::isPlausibleScopeSeedHand(10.5f));
        ok &= expectFalse("an unplaced first-person arm never seeds a scoped frame",
            rock::scope_safe_hand_frame_math::isPlausibleScopeSeedHand(109950.0f));
        ok &= expectFalse("a non-finite seed distance is refused",
            rock::scope_safe_hand_frame_math::isPlausibleScopeSeedHand(
                std::numeric_limits<float>::quiet_NaN()));
        ok &= expectEqual("ordinary aiming never substitutes the scope driver for a missing canonical hand frame",
            rock::scope_safe_hand_frame_math::resolveMode(false, false, true, true, 0, 3), ResolutionMode::Unavailable);
        ok &= expectEqual("prior collision presentation reconstructs from the physical driver even while unscoped",
            rock::scope_safe_hand_frame_math::resolveCollisionIsolatedMode(true, false, true, true, true, 0, 3),
            ResolutionMode::DriverReconstructed);
        ok &= expectEqual("prior collision presentation never falls back to a contaminated root or cached output",
            rock::scope_safe_hand_frame_math::resolveCollisionIsolatedMode(true, false, true, false, true, 0, 3),
            ResolutionMode::Unavailable);
        ok &= expectEqual("ordinary frames retain the visible-root policy",
            rock::scope_safe_hand_frame_math::resolveCollisionIsolatedMode(false, false, true, true, true, 0, 3),
            ResolutionMode::RootFlattened);
        ok &= expectEqual("ordinary scoped frames retain bounded last-known continuity",
            rock::scope_safe_hand_frame_math::resolveCollisionIsolatedMode(false, true, false, false, true, 0, 3),
            ResolutionMode::LastKnown);
        ok &= expectTrue("open ScopeMenu selects driver-frame weapon authority",
            rock::scope_safe_hand_frame_math::retainDriverFrameAuthority(true, false, false, false));
        ok &= expectTrue("held scope button retains driver-frame authority across a transient ScopeMenu close",
            rock::scope_safe_hand_frame_math::retainDriverFrameAuthority(false, true, true, true));
        ok &= expectFalse("released scope button returns to the restored root before manual grip teardown",
            rock::scope_safe_hand_frame_math::retainDriverFrameAuthority(false, false, true, true));
        ok &= expectTrue("manual grip retains driver-frame authority across a later ScopeMenu reopen",
            rock::scope_safe_hand_frame_math::retainDriverFrameAuthority(true, true, true, true));
        ok &= expectFalse("driver-frame authority stops if manual ownership ends despite a held scope button",
            rock::scope_safe_hand_frame_math::retainDriverFrameAuthority(false, true, false, true));
        ok &= expectFalse("ordinary unscoped aiming does not acquire driver-frame authority",
            rock::scope_safe_hand_frame_math::retainDriverFrameAuthority(false, true, true, false));
        ok &= expectFalse("button release never rebases the visible root from a stale hidden-scope hand",
            rock::scope_safe_hand_frame_math::shouldStartRootRebase(false, true, true, true));
        ok &= expectTrue("a non-release root handoff may preserve the last scoped hand continuously",
            rock::scope_safe_hand_frame_math::shouldStartRootRebase(true, true, false, true));
        ok &= expectFalse("root rebase requires an actual driver-authority stop edge",
            rock::scope_safe_hand_frame_math::shouldStartRootRebase(true, false, true, true));
        ok &= expectFalse("root rebase requires a valid reconstructed or recent scoped hand",
            rock::scope_safe_hand_frame_math::shouldStartRootRebase(true, true, false, false));
        ok &= expectTrue("visible stable carry may refresh the right firing canonical", rock::scope_safe_hand_frame_math::canRefreshRightFiringCanonicalFrame(false, false));
        ok &= expectFalse("ScopeMenu cannot overwrite the right firing canonical", rock::scope_safe_hand_frame_math::canRefreshRightFiringCanonicalFrame(true, false));
        ok &= expectFalse("scope-exit hand rebase cannot overwrite the right firing canonical", rock::scope_safe_hand_frame_math::canRefreshRightFiringCanonicalFrame(false, true));
        ok &= expectTrue("scoped right firing grip reuses the matching pre-scope canonical",
            rock::scope_safe_hand_frame_math::shouldReuseRightFiringCanonicalGrip(true, false, true, 0x1234u, 0x1234u));
        ok &= expectFalse("scoped grip rejects a canonical from a stale weapon generation",
            rock::scope_safe_hand_frame_math::shouldReuseRightFiringCanonicalGrip(true, false, true, 0x1234u, 0x5678u));
        ok &= expectFalse("left firing grip keeps its established mirrored hold",
            rock::scope_safe_hand_frame_math::shouldReuseRightFiringCanonicalGrip(true, true, true, 0x1234u, 0x1234u));

        using rock::scope_safe_hand_frame_math::DeferredClearAction;
        using rock::scope_safe_hand_frame_math::DesiredHandAuthorityInput;
        using rock::scope_safe_hand_frame_math::HandAuthorityRole;
        const auto primaryRole = rock::scope_safe_hand_frame_math::roleMask(HandAuthorityRole::PrimaryGrip);
        const auto supportRole = rock::scope_safe_hand_frame_math::roleMask(HandAuthorityRole::SupportGrip);

        const DesiredHandAuthorityInput rightPrimaryOnly{
            .firingHandIsLeft = false,
        };
        ok &= expectEqual("right-hand-only scope owns no ROCK wrist tag",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(rightPrimaryOnly, false),
            static_cast<rock::scope_safe_hand_frame_math::HandAuthorityRoleMask>(0));
        ok &= expectEqual("right-hand-only scope leaves the left wrist native",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(rightPrimaryOnly, true),
            static_cast<rock::scope_safe_hand_frame_math::HandAuthorityRoleMask>(0));

        const DesiredHandAuthorityInput leftPrimaryOnly{
            .firingHandIsLeft = true,
        };
        ok &= expectEqual("left-hand-only scope uses mirrored carry without a ROCK wrist tag",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(leftPrimaryOnly, true),
            static_cast<rock::scope_safe_hand_frame_math::HandAuthorityRoleMask>(0));
        ok &= expectEqual("left-hand-only scope leaves the right wrist native",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(leftPrimaryOnly, false),
            static_cast<rock::scope_safe_hand_frame_math::HandAuthorityRoleMask>(0));

        const DesiredHandAuthorityInput rightFiringTwoHand{
            .gripping = true,
            .primaryHandAuthorityEnabled = true,
            .firingHandIsLeft = false,
            .leftPartGripActive = true,
        };
        ok &= expectEqual("right-fire two-hand scope retains the right primary wrist",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(rightFiringTwoHand, false), primaryRole);
        ok &= expectEqual("right-fire two-hand scope retains the left support wrist",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(rightFiringTwoHand, true), supportRole);

        const DesiredHandAuthorityInput rightFiringVisualSupport{
            .gripping = true,
            .primaryHandAuthorityEnabled = false,
            .firingHandIsLeft = false,
            .leftPartGripActive = true,
        };
        ok &= expectEqual("right-fire visual support keeps the native firing wrist unowned",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(rightFiringVisualSupport, false),
            static_cast<rock::scope_safe_hand_frame_math::HandAuthorityRoleMask>(0));
        ok &= expectEqual("right-fire visual support retains only the left support wrist",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(rightFiringVisualSupport, true), supportRole);

        const DesiredHandAuthorityInput leftFiringTwoHand{
            .gripping = true,
            .primaryHandAuthorityEnabled = true,
            .firingHandIsLeft = true,
            .rightPartGripActive = true,
        };
        ok &= expectEqual("left-fire two-hand scope retains the left primary wrist",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(leftFiringTwoHand, true), primaryRole);
        ok &= expectEqual("left-fire two-hand scope retains the right support wrist",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(leftFiringTwoHand, false), supportRole);

        const DesiredHandAuthorityInput leftFiringVisualSupport{
            .gripping = true,
            .primaryHandAuthorityEnabled = false,
            .firingHandIsLeft = true,
            .rightPartGripActive = true,
        };
        ok &= expectEqual("left-fire visual support keeps the mirrored firing wrist native",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(leftFiringVisualSupport, true),
            static_cast<rock::scope_safe_hand_frame_math::HandAuthorityRoleMask>(0));
        ok &= expectEqual("left-fire visual support retains only the right support wrist",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(leftFiringVisualSupport, false), supportRole);

        const DesiredHandAuthorityInput twoHandPartCarry{
            .leftPartGripActive = true,
            .rightPartGripActive = true,
        };
        ok &= expectEqual("part carry retains the left part-grip wrist",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(twoHandPartCarry, true), supportRole);
        ok &= expectEqual("part carry retains the right part-grip wrist",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(twoHandPartCarry, false), supportRole);

        ok &= expectEqual("scope exit retains a role still owned by the same hand",
            rock::scope_safe_hand_frame_math::resolveDeferredClearAction(
                HandAuthorityRole::PrimaryGrip, primaryRole, 0),
            DeferredClearAction::RetainLiveRole);
        ok &= expectEqual("scope exit waits before replacing primary with support on one hand",
            rock::scope_safe_hand_frame_math::resolveDeferredClearAction(
                HandAuthorityRole::PrimaryGrip, supportRole, 0),
            DeferredClearAction::WaitForReplacementPublication);
        ok &= expectEqual("scope exit clears the old primary after support publishes",
            rock::scope_safe_hand_frame_math::resolveDeferredClearAction(
                HandAuthorityRole::PrimaryGrip, supportRole, supportRole),
            DeferredClearAction::ClearStaleRole);
        ok &= expectEqual("scope exit clears released authority when no ROCK role remains",
            rock::scope_safe_hand_frame_math::resolveDeferredClearAction(
                HandAuthorityRole::SupportGrip, 0, 0),
            DeferredClearAction::ClearStaleRole);

        TestTransform rebaseStart = rock::transform_math::makeIdentityTransform<TestTransform>();
        rebaseStart.translate = { 1.5f, -2.0f, 0.75f };
        rebaseStart.rotate.entry[0][0] = 0.0f;
        rebaseStart.rotate.entry[0][1] = 1.0f;
        rebaseStart.rotate.entry[1][0] = -1.0f;
        rebaseStart.rotate.entry[1][1] = 0.0f;
        const TestTransform rebaseIdentity = rock::transform_math::makeIdentityTransform<TestTransform>();
        ok &= expectTransformNear("scope-exit rebase starts at the prior ROCK hand frame",
            rock::scope_safe_hand_frame_math::interpolateRebaseTransform(rebaseStart, rebaseIdentity, 0.0f),
            rebaseStart);
        ok &= expectTransformNear("scope-exit rebase finishes at the restored hFRIK root frame",
            rock::scope_safe_hand_frame_math::interpolateRebaseTransform(rebaseStart, rebaseIdentity, 1.0f),
            rebaseIdentity);
        ok &= expectNear("scope-exit rebase timing clamps at completion",
            rock::scope_safe_hand_frame_math::rebaseAlpha(0.10f, 0.075f),
            1.0f);
    }

    {
        TestTransform renderedTwoHandWeaponWorld =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        renderedTwoHandWeaponWorld.rotate = makeAxisAngleRotation(
            rock::weaponSolverNormalize(
                TestVector3{ 0.31f, -0.44f, 0.72f }),
            37.0f);
        renderedTwoHandWeaponWorld.translate =
            TestVector3{ 18.0f, -7.0f, 23.0f };
        renderedTwoHandWeaponWorld.scale = 1.25f;

        TestTransform authoredLeftHandWeaponLocal =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        authoredLeftHandWeaponLocal.rotate = makeAxisAngleRotation(
            rock::weaponSolverNormalize(
                TestVector3{ 0.0f, 1.0f, 0.0f }),
            90.0f);
        authoredLeftHandWeaponLocal.translate =
            TestVector3{ -4.0f, 22.0f, 1.5f };
        const TestTransform authoredLeftHandTargetAtDetach =
            rock::transform_math::composeTransforms(
                renderedTwoHandWeaponWorld,
                authoredLeftHandWeaponLocal);

        TestTransform rawLeftDriverAtDetach =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        rawLeftDriverAtDetach.translate =
            TestVector3{ -11.0f, 26.0f, 9.0f };
        rawLeftDriverAtDetach.scale = 1.25f;

        TestTransform driverToAuthoredTargetLocal{};
        TestTransform driverToWeaponLocal{};
        ok &= expectTrue(
            "integrated detach captures raw-driver to authored-target baseline",
            rock::weapon_support_acquisition_math::
                tryCaptureSupportInputBaseline(
                    rawLeftDriverAtDetach,
                    authoredLeftHandTargetAtDetach,
                    driverToAuthoredTargetLocal));
        ok &= expectTrue(
            "integrated detach captures raw-driver to weapon baseline",
            rock::weapon_support_acquisition_math::
                tryCaptureSupportInputBaseline(
                    rawLeftDriverAtDetach,
                    renderedTwoHandWeaponWorld,
                    driverToWeaponLocal));

        TestTransform firstAuthoredLeftHandTarget{};
        TestTransform firstPartCarryWeaponWorld{};
        ok &= expectTrue(
            "unchanged raw driver resolves the authored support target",
            rock::weapon_support_acquisition_math::
                tryResolveSupportInputTarget(
                    rawLeftDriverAtDetach,
                    driverToAuthoredTargetLocal,
                    firstAuthoredLeftHandTarget));
        ok &= expectTrue(
            "unchanged raw driver resolves the rendered weapon pose",
            rock::weapon_support_acquisition_math::
                tryResolveSupportInputTarget(
                    rawLeftDriverAtDetach,
                    driverToWeaponLocal,
                    firstPartCarryWeaponWorld));
        ok &= expectTransformNear(
            "integrated detach preserves the authored support target",
            firstAuthoredLeftHandTarget,
            authoredLeftHandTargetAtDetach);
        ok &= expectTransformNear(
            "integrated detach preserves the rendered two-hand pose",
            firstPartCarryWeaponWorld,
            renderedTwoHandWeaponWorld);

        TestTransform postDetachHandDelta =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        postDetachHandDelta.rotate = makeAxisAngleRotation(
            rock::weaponSolverNormalize(
                TestVector3{ 0.56f, 0.14f, -0.63f }),
            18.0f);
        postDetachHandDelta.translate =
            TestVector3{ 6.0f, -2.0f, 9.0f };
        const TestTransform movedLeftHand =
            rock::transform_math::composeTransforms(
                postDetachHandDelta,
                rawLeftDriverAtDetach);
        TestTransform movedAuthoredLeftHandTarget{};
        TestTransform movedPartCarryWeaponWorld{};
        ok &= expectTrue(
            "moved raw driver resolves the calibrated authored target",
            rock::weapon_support_acquisition_math::
                tryResolveSupportInputTarget(
                    movedLeftHand,
                    driverToAuthoredTargetLocal,
                    movedAuthoredLeftHandTarget));
        ok &= expectTrue(
            "moved raw driver resolves the calibrated weapon",
            rock::weapon_support_acquisition_math::
                tryResolveSupportInputTarget(
                    movedLeftHand,
                    driverToWeaponLocal,
                    movedPartCarryWeaponWorld));
        const TestTransform expectedMovedAuthoredTarget =
            rock::transform_math::composeTransforms(
                postDetachHandDelta,
                authoredLeftHandTargetAtDetach);
        const TestTransform expectedMovedWeaponWorld =
            rock::transform_math::composeTransforms(
                postDetachHandDelta,
                renderedTwoHandWeaponWorld);
        ok &= expectTransformNear(
            "part carry retains the controller-to-authored-hand orientation offset",
            movedAuthoredLeftHandTarget,
            expectedMovedAuthoredTarget);
        ok &= expectTransformNear(
            "part carry applies only post-detach left-hand rigid motion",
            movedPartCarryWeaponWorld,
            expectedMovedWeaponWorld);
        const TestTransform movedAuthoredHandWeaponLocal =
            rock::transform_math::composeTransforms(
                rock::transform_math::invertTransform(
                    movedPartCarryWeaponWorld),
                movedAuthoredLeftHandTarget);
        ok &= expectTransformNear(
            "part carry never rewrites the authored hand-in-weapon relation",
            movedAuthoredHandWeaponLocal,
            authoredLeftHandWeaponLocal);
    }

    // Alternating handguard carriers have very different raw wrist rotations.
    // Every transfer must start at the prior rendered pose, then follow only
    // subsequent controller motion while preserving the authored hand relation.
    for (const bool startLeft : { true, false }) {
        TestTransform weapon = rock::transform_math::makeIdentityTransform<TestTransform>();
        weapon.rotate = makeAxisAngleRotation(rock::weaponSolverNormalize(TestVector3{ 0.3f, -0.4f, 0.7f }), 43.0f);
        weapon.translate = { 20.0f, -15.0f, 31.0f };
        weapon.scale = 1.25f;
        for (int transfer = 0; transfer < 12; ++transfer) {
            const bool left = (transfer % 2 == 0) == startLeft;
            TestTransform seat = rock::transform_math::makeIdentityTransform<TestTransform>();
            seat.rotate = makeAxisAngleRotation(TestVector3{ 0.0f, 1.0f, 0.0f }, left ? 90.0f : -90.0f);
            seat.translate = { left ? -4.0f : 4.0f, 22.0f, 1.5f };
            const auto handTarget = rock::transform_math::composeTransforms(weapon, seat);
            TestTransform driver = rock::transform_math::makeIdentityTransform<TestTransform>();
            driver.rotate = makeAxisAngleRotation(rock::weaponSolverNormalize(TestVector3{ 0.2f, 0.8f, -0.3f }),
                (left ? -117.0f : 73.0f) + transfer * 3.0f);
            driver.translate = { left ? -12.0f : 16.0f, 26.0f, 9.0f };
            driver.scale = left ? 0.8f : 1.1f;
            TestTransform weaponBaseline{}, handBaseline{}, firstWeapon{}, firstHand{};
            ok &= expectTrue("carry handoff captures weapon baseline",
                rock::weapon_support_acquisition_math::tryCaptureSupportInputBaseline(driver, weapon, weaponBaseline));
            ok &= expectTrue("carry handoff captures authored wrist baseline",
                rock::weapon_support_acquisition_math::tryCaptureSupportInputBaseline(driver, handTarget, handBaseline));
            ok &= expectTrue("carry handoff resolves unchanged controller",
                rock::weapon_support_acquisition_math::tryResolveSupportInputTarget(driver, weaponBaseline, firstWeapon));
            ok &= expectTrue("carry handoff resolves unchanged authored wrist",
                rock::weapon_support_acquisition_math::tryResolveSupportInputTarget(driver, handBaseline, firstHand));
            ok &= expectTransformNear("handguard handoff preserves translation rotation and scale", firstWeapon, weapon);
            ok &= expectTransformNear("handguard handoff preserves the authored wrist", firstHand, handTarget);
            TestTransform motion = rock::transform_math::makeIdentityTransform<TestTransform>();
            motion.rotate = makeAxisAngleRotation(TestVector3{ 0.0f, 0.0f, 1.0f }, left ? 13.0f : -8.0f);
            motion.translate = { 3.0f, -2.0f, 1.0f };
            const auto movedDriver = rock::transform_math::composeTransforms(motion, driver);
            TestTransform movedWeapon{}, movedHand{};
            ok &= expectTrue("new carrier moves the calibrated weapon",
                rock::weapon_support_acquisition_math::tryResolveSupportInputTarget(movedDriver, weaponBaseline, movedWeapon));
            ok &= expectTrue("new carrier moves its authored target",
                rock::weapon_support_acquisition_math::tryResolveSupportInputTarget(movedDriver, handBaseline, movedHand));
            ok &= expectTransformNear("only post-handoff raw motion moves the weapon", movedWeapon,
                rock::transform_math::composeTransforms(motion, weapon));
            ok &= expectTransformNear("handoffs never reauthor the handguard seat",
                rock::transform_math::composeTransforms(rock::transform_math::invertTransform(movedWeapon), movedHand), seat);
            weapon = movedWeapon;
        }
    }

    using namespace rock::contact_pipeline_policy;

    const ContactEndpoint weapon{
        .bodyId = 100,
        .layer = 44,
        .kind = ContactEndpointKind::Weapon,
    };

    const auto leftWeapon = classifyContact(
        ContactEndpoint{ .bodyId = 10, .layer = 43, .kind = ContactEndpointKind::LeftHand },
        weapon);
    ok &= expectEqual("left hand weapon contact routes as hand weapon", leftWeapon.route, ContactRoute::HandWeapon);
    ok &= expectTrue("left hand weapon contact drives support evidence", leftWeapon.drivesWeaponSupportContact);
    ok &= expectEqual("left hand remains contact source", leftWeapon.source.kind, ContactEndpointKind::LeftHand);

    const auto rightWeapon = classifyContact(
        ContactEndpoint{ .bodyId = 20, .layer = 43, .kind = ContactEndpointKind::RightHand },
        weapon);
    ok &= expectEqual("right hand weapon contact routes as hand weapon", rightWeapon.route, ContactRoute::HandWeapon);
    ok &= expectTrue("right hand weapon contact drives support evidence", rightWeapon.drivesWeaponSupportContact);
    ok &= expectEqual("right hand remains contact source", rightWeapon.source.kind, ContactEndpointKind::RightHand);

    using rock::weapon_two_handed_grip_math::canProcessNormalGrabInput;
    using rock::weapon_two_handed_grip_math::resolveSupportReleaseManualAction;
    using rock::weapon_two_handed_grip_math::SupportReleaseOwnershipInput;
    using rock::weapon_two_handed_grip_math::SupportReleaseManualAction;
    ok &= expectFalse("support-hand normal grab is blocked while its part grip is active", canProcessNormalGrabInput(false, true, true, false));
    ok &= expectTrue("support-hand normal grab stays available without a part grip", canProcessNormalGrabInput(false, true, false, false));
    ok &= expectFalse("firing-hand normal grab is blocked while a weapon is equipped", canProcessNormalGrabInput(true, true, false, false));
    ok &= expectTrue("firing-hand normal grab is restored while detached and free", canProcessNormalGrabInput(true, true, false, true));
    ok &= expectTrue("firing-hand normal grab stays available without equipped weapon", canProcessNormalGrabInput(true, false, false, false));
    ok &= expectTrue("full two-handed support still owns weapon transform",
        rock::weapon_support_authority_policy::supportGripOwnsWeaponTransform(rock::weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver));
    ok &= expectTrue("full two-handed support applies primary hand authority while active",
        rock::weapon_support_authority_policy::supportGripAppliesPrimaryHandAuthority(rock::weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver));
    ok &= expectTrue("support grip continues to apply offhand visual authority",
        rock::weapon_support_authority_policy::supportGripAppliesSupportHandAuthority(rock::weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver));
    ok &= expectTrue("only a normal dynamic full-authority support grip uses synchronized acquisition",
        rock::weapon_support_authority_policy::shouldUseDynamicSupportAcquisition(
            rock::weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver,
            false,
            false,
            false));
    ok &= expectFalse("authored support bypasses synchronized dynamic acquisition",
        rock::weapon_support_authority_policy::shouldUseDynamicSupportAcquisition(
            rock::weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver,
            true,
            false,
            false));
    ok &= expectFalse("provider support bypasses synchronized dynamic acquisition",
        rock::weapon_support_authority_policy::shouldUseDynamicSupportAcquisition(
            rock::weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver,
            false,
            true,
            false));
    ok &= expectFalse("attach-only support bypasses synchronized dynamic acquisition",
        rock::weapon_support_authority_policy::shouldUseDynamicSupportAcquisition(
            rock::weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver,
            false,
            false,
            true));
    ok &= expectFalse("visual-only support never gains synchronized weapon acquisition",
        rock::weapon_support_authority_policy::shouldUseDynamicSupportAcquisition(
            rock::weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport,
            false,
            false,
            false));

    const rock::RockEquippedWeaponHandlingBaseline coreWeaponHandlingBaseline{
        .ambidextrousHandoffEnabled = true,
        .authoredOnlySupportGrabsEnabled = true,
        .weaponGrabMode = WeaponGrabMode::ToggleBoth,
        .equippedWeaponShoulderStashEnabled = true,
        .lastGripReleaseDropEnabled = false,
        .immersiveWeapon = {
            .firingGripDetachEnabled = true,
            .firingGripDetachPosePreservationEnabled = true,
            .firingGripReattachRadiusGameUnits = 4.0f,
            .firingGripHapticDurationSeconds = 0.11f,
            .firingGripAttachHapticIntensity = 0.81f,
            .firingGripDetachHapticIntensity = 0.31f,
        },
        .firingGripReattachCylinderRadiusGameUnits = 2.5f,
        .firingGripProximitySupportRadiusGameUnits = 7.0f,
        .leftFiringAimYawDegrees = 1.5f,
        .leftFiringAimPitchDegrees = -2.5f,
        .leftFiringAimOffsetXGameUnits = 0.5f,
        .leftFiringAimOffsetYGameUnits = -0.75f,
        .leftFiringAimOffsetZGameUnits = 1.25f,
    };
    const auto coreWeaponHandling = rock::makeEquippedWeaponHandlingSettings(
        coreWeaponHandlingBaseline,
        nullptr);
    ok &= expectFalse("base ROCK has no external equipped-weapon authority",
        coreWeaponHandling.externalAuthorityActive);
    ok &= expectNear("base ROCK owns the proximity support radius",
        coreWeaponHandling.firingGripProximitySupportRadiusGameUnits,
        7.0f);
    ok &= expectNear("base ROCK owns the reattach cylinder radius",
        coreWeaponHandling.firingGripReattachCylinderRadiusGameUnits,
        2.5f);
    ok &= expectTrue("base ROCK enables configured firing-grip ownership",
        coreWeaponHandling.firingGripOwnershipEnabled);
    ok &= expectTrue("base ROCK enables configured ambidextrous handoff",
        coreWeaponHandling.ambidextrousHandoffEnabled);
    ok &= expectTrue("base ROCK enables authored-only support acquisition",
        coreWeaponHandling.authoredOnlySupportGrabsEnabled);
    ok &= expectTrue("base ROCK enables configured equipped-weapon toggle grab",
        coreWeaponHandling.weaponGrabMode == WeaponGrabMode::ToggleBoth);
    ok &= expectFalse("base ROCK applies the configured last-grip drop preference",
        coreWeaponHandling.lastGripReleaseDropEnabled);
    ok &= expectFalse("base ROCK shoulder stash never enables physical detach",
        coreWeaponHandling.primaryDetachEnabled);
    const auto integratedDetach =
        rock::resolveEquippedWeaponDetachDecision(
            coreWeaponHandling);
    ok &= expectEqual("integrated immersive authority follows the firing role",
        integratedDetach.authority,
        rock::immersive_weapon_policy::DetachAuthority::
            IntegratedImmersive);
    ok &= expectTrue("integrated immersive enables firing ownership",
        integratedDetach.firingGripOwnershipEnabled);
    ok &= expectTrue("integrated immersive enables detach",
        integratedDetach.primaryDetachEnabled);
    ok &= expectTrue("integrated immersive selects detach pose preservation",
        integratedDetach.preserveWeaponPoseOnDetach);
    ok &= expectNear("integrated immersive owns reattach radius",
        integratedDetach.reattachRadiusGameUnits,
        4.0f);
    ok &= expectNear("integrated immersive owns haptic duration",
        integratedDetach.gripHapticDurationSeconds,
        0.11f);
    ok &= expectNear("integrated immersive owns attach haptic",
        integratedDetach.gripAttachHapticIntensity,
        0.81f);
    ok &= expectNear("integrated immersive owns detach haptic",
        integratedDetach.gripDetachHapticIntensity,
        0.31f);
    ok &= expectEqual(
        "authored-only integrated detach rejects an ordinary detached firing-hand part grab",
        rock::immersive_weapon_policy::
            resolveDetachedFiringHandPartGrab({
                .partCarryAuthority =
                    rock::immersive_weapon_policy::DetachAuthority::
                        IntegratedImmersive,
                .authoredOnlySupportGrabsEnabled = true,
                .exactProviderPartTargetActive = false,
            }),
        rock::immersive_weapon_policy::
            DetachedFiringHandPartGrabSelection::Reject);
    ok &= expectEqual("detached firing hand accepts its validated authored handguard seat",
        rock::immersive_weapon_policy::resolveDetachedFiringHandPartGrab({
            .partCarryAuthority = rock::immersive_weapon_policy::DetachAuthority::IntegratedImmersive,
            .authoredOnlySupportGrabsEnabled = true,
            .authoredSupportSeatAvailable = true,
        }), rock::immersive_weapon_policy::DetachedFiringHandPartGrabSelection::Standard);
    ok &= expectEqual(
        "authored-only integrated detach accepts an exact provider part target",
        rock::immersive_weapon_policy::
            resolveDetachedFiringHandPartGrab({
                .partCarryAuthority =
                    rock::immersive_weapon_policy::DetachAuthority::
                        IntegratedImmersive,
                .authoredOnlySupportGrabsEnabled = true,
                .exactProviderPartTargetActive = true,
            }),
        rock::immersive_weapon_policy::
            DetachedFiringHandPartGrabSelection::ExactProviderTarget);
    ok &= expectEqual(
        "authored-only off preserves unrestricted detached firing-hand selection",
        rock::immersive_weapon_policy::
            resolveDetachedFiringHandPartGrab({
                .partCarryAuthority =
                    rock::immersive_weapon_policy::DetachAuthority::
                        IntegratedImmersive,
                .authoredOnlySupportGrabsEnabled = false,
                .exactProviderPartTargetActive = false,
            }),
        rock::immersive_weapon_policy::
            DetachedFiringHandPartGrabSelection::Standard);
    ok &= expectEqual(
        "external detach preserves its established part selection",
        rock::immersive_weapon_policy::
            resolveDetachedFiringHandPartGrab({
                .partCarryAuthority =
                    rock::immersive_weapon_policy::DetachAuthority::
                        ExternalProvider,
                .authoredOnlySupportGrabsEnabled = true,
                .exactProviderPartTargetActive = false,
            }),
        rock::immersive_weapon_policy::
            DetachedFiringHandPartGrabSelection::Standard);
    ok &= expectTrue("base ROCK owns equipped-weapon shoulder stash",
        coreWeaponHandling.equippedWeaponShoulderStashEnabled);
    ok &= expectNear("ROCK yaw tuning adds to the calibrated left firing baseline",
        coreWeaponHandling.leftFiringAimYawDegrees,
        10.5f);
    ok &= expectNear("zero ROCK yaw tuning retains the calibrated left firing aim",
        rock::makeEquippedWeaponHandlingSettings({}, nullptr).leftFiringAimYawDegrees,
        9.0f);
    ok &= expectNear("base ROCK owns left firing aim pitch",
        coreWeaponHandling.leftFiringAimPitchDegrees,
        -2.5f);
    ok &= expectNear("base ROCK owns left firing aim offset X",
        coreWeaponHandling.leftFiringAimOffsetXGameUnits,
        0.5f);
    ok &= expectNear("base ROCK owns left firing aim offset Y",
        coreWeaponHandling.leftFiringAimOffsetYGameUnits,
        -0.75f);
    ok &= expectNear("base ROCK owns left firing aim offset Z",
        coreWeaponHandling.leftFiringAimOffsetZGameUnits,
        1.25f);

    auto stashOnlyBaseline = coreWeaponHandlingBaseline;
    stashOnlyBaseline.ambidextrousHandoffEnabled = false;
    const auto stashOnlyHandling = rock::makeEquippedWeaponHandlingSettings(
        stashOnlyBaseline,
        nullptr);
    ok &= expectFalse("ROCK stash does not create firing ownership with handoff off",
        stashOnlyHandling.firingGripOwnershipEnabled);
    ok &= expectFalse("ROCK stash does not create physical detach with handoff off",
        stashOnlyHandling.primaryDetachEnabled);
    ok &= expectFalse("disabled ROCK ambidextrous mode keeps handoff disabled",
        stashOnlyHandling.ambidextrousHandoffEnabled);
    ok &= expectTrue("authored-only support remains independent of handoff",
        stashOnlyHandling.authoredOnlySupportGrabsEnabled);

    auto fixedOnlyBaseline = stashOnlyBaseline;
    fixedOnlyBaseline.equippedWeaponShoulderStashEnabled = false;
    const auto fixedOnlyHandling = rock::makeEquippedWeaponHandlingSettings(
        fixedOnlyBaseline,
        nullptr);
    ok &= expectFalse("disabled ROCK handoff and stash release firing ownership",
        fixedOnlyHandling.firingGripOwnershipEnabled);
    ok &= expectFalse("disabled ROCK stash releases primary detach",
        fixedOnlyHandling.primaryDetachEnabled);
    ok &= expectFalse("disabled ROCK stash remains disabled",
        fixedOnlyHandling.equippedWeaponShoulderStashEnabled);

    auto leftDefaultBaseline = fixedOnlyBaseline;
    leftDefaultBaseline.leftHandedModeEnabled = true;
    leftDefaultBaseline.immersiveWeapon.firingGripDetachEnabled = false;
    const auto leftDefaultHandling = rock::makeEquippedWeaponHandlingSettings(leftDefaultBaseline, nullptr);
    ok &= expectTrue("left default independently enables firing ownership",
        leftDefaultHandling.firingGripOwnershipEnabled);
    ok &= expectFalse("left default does not enable handoff",
        leftDefaultHandling.ambidextrousHandoffEnabled);
    ok &= expectFalse("left default does not enable detach",
        rock::resolveEquippedWeaponDetachDecision(leftDefaultHandling).primaryDetachEnabled);

    rock::provider::RockProviderEquippedWeaponHandlingRequestV1 externalWeaponHandling{};
    externalWeaponHandling.flags = static_cast<std::uint32_t>(
        rock::provider::RockProviderEquippedWeaponHandlingFlagV1::FiringGripOwnership);
    externalWeaponHandling.firingGripProximitySupportRadiusGameUnits = 8.0f;
    auto externalHandling = rock::makeEquippedWeaponHandlingSettings(
        coreWeaponHandlingBaseline,
        &externalWeaponHandling);
    ok &= expectTrue("an equipped-weapon request activates external authority",
        externalHandling.externalAuthorityActive);
    ok &= expectTrue("an addon lease cannot suppress ROCK shoulder stash",
        externalHandling.equippedWeaponShoulderStashEnabled);
    ok &= expectTrue("an addon lease cannot suppress ROCK toggle grab",
        externalHandling.weaponGrabMode == WeaponGrabMode::ToggleBoth);
    ok &= expectFalse("an addon lease cannot re-enable the last-grip drop",
        externalHandling.lastGripReleaseDropEnabled);
    ok &= expectTrue("an addon handling lease cannot suppress authored-only support",
        externalHandling.authoredOnlySupportGrabsEnabled);
    ok &= expectFalse("ROCK stash cannot add detach to a non-detach addon lease",
        externalHandling.primaryDetachEnabled);
    const auto integratedUnderNonDetachProvider =
        rock::resolveEquippedWeaponDetachDecision(
            externalHandling);
    ok &= expectEqual("non-detach provider cannot suppress integrated immersive detach",
        integratedUnderNonDetachProvider.authority,
        rock::immersive_weapon_policy::DetachAuthority::
            IntegratedImmersive);
    ok &= expectFalse("an active addon request may suppress ROCK ambidextrous handoff",
        externalHandling.ambidextrousHandoffEnabled);
    ok &= expectNear("external authority preserves ROCK's radius without an override",
        externalHandling.firingGripProximitySupportRadiusGameUnits,
        7.0f);
    externalWeaponHandling.flags |=
        static_cast<std::uint32_t>(
            rock::provider::RockProviderEquippedWeaponHandlingFlagV1::AmbidextrousHandoff) |
        static_cast<std::uint32_t>(
            rock::provider::RockProviderEquippedWeaponHandlingFlagV1::FiringGripProximitySupport);
    externalWeaponHandling.firingGripPromotionRadiusGameUnits = 9.0f;
    externalWeaponHandling.leftFiringAimYawDegrees = -4.0f;
    externalHandling = rock::makeEquippedWeaponHandlingSettings(
        coreWeaponHandlingBaseline,
        &externalWeaponHandling);
    ok &= expectTrue("an active addon request may enable handoff through the ROCK executor",
        externalHandling.ambidextrousHandoffEnabled);
    ok &= expectNear("an active owner supplies proximity tuning",
        externalHandling.firingGripProximitySupportRadiusGameUnits,
        8.0f);
    ok &= expectNear("legacy promotion tuning cannot change the shared station reach",
        externalHandling.firingGripReattachRadiusGameUnits,
        externalWeaponHandling.firingGripReattachRadiusGameUnits);
    ok &= expectNear("an active owner supplies left firing aim tuning",
        externalHandling.leftFiringAimYawDegrees,
        -4.0f);

    ok &= expectFalse("compatible addon activation does not tear down ROCK handoff",
        rock::requiresEquippedWeaponHandlingModeReconcile(
            coreWeaponHandling,
            externalHandling));
    auto authoredOnlyDisabledHandling = coreWeaponHandling;
    for (const auto type : { TestWeaponType::kHandToHand, TestWeaponType::kOneHandSword,
             TestWeaponType::kOneHandDagger, TestWeaponType::kOneHandAxe, TestWeaponType::kOneHandMace,
             TestWeaponType::kTwoHandSword, TestWeaponType::kTwoHandAxe, TestWeaponType::kGun }) {
        auto baseline = coreWeaponHandlingBaseline;
        baseline.meleeWeapon = rock::weapon_type_policy::isEquippedMelee(type);
        for (const bool restricted : { false, true }) {
            baseline.authoredOnlySupportGrabsEnabled = restricted;
            for (const bool provider : { false, true }) {
                const auto handling = rock::makeEquippedWeaponHandlingSettings(
                    baseline, provider ? &externalWeaponHandling : nullptr);
                const bool shouldRestrict = restricted && type == TestWeaponType::kGun;
                ok &= expectTrue("melee always bypasses authored restrictions; firearms follow the setting",
                    handling.authoredOnlySupportGrabsEnabled == shouldRestrict);
                const auto selection = rock::authored_support_grab_policy::select({
                    .modeEnabled = handling.authoredOnlySupportGrabsEnabled,
                    .capability = rock::authored_support_grab_policy::Capability::Pending,
                });
                ok &= expectTrue("melee can dynamically acquire without waiting for an authored pose",
                    rock::authored_support_grab_policy::captured(selection.selection) == !shouldRestrict);
                const auto detachedSelection = rock::immersive_weapon_policy::resolveDetachedFiringHandPartGrab({
                    .partCarryAuthority = rock::immersive_weapon_policy::DetachAuthority::IntegratedImmersive,
                    .authoredOnlySupportGrabsEnabled = handling.authoredOnlySupportGrabsEnabled,
                });
                ok &= expectTrue("melee detached firing hand may dynamically grab another part",
                    (detachedSelection == rock::immersive_weapon_policy::DetachedFiringHandPartGrabSelection::Standard) == !shouldRestrict);
            }
        }
    }
    authoredOnlyDisabledHandling.authoredOnlySupportGrabsEnabled = false;
    ok &= expectFalse("authored-only hot reload applies to the next acquisition",
        rock::requiresEquippedWeaponHandlingModeReconcile(
            coreWeaponHandling,
            authoredOnlyDisabledHandling));
    auto detachPosePreservationDisabled = coreWeaponHandling;
    detachPosePreservationDisabled.immersiveWeapon.
        firingGripDetachPosePreservationEnabled = false;
    const auto integratedWithoutPosePreservation =
        rock::resolveEquippedWeaponDetachDecision(
            detachPosePreservationDisabled);
    ok &= expectTrue("pose preservation off keeps integrated immersive detach",
        integratedWithoutPosePreservation.primaryDetachEnabled);
    ok &= expectFalse("pose preservation off selects the legacy carry relation",
        integratedWithoutPosePreservation.preserveWeaponPoseOnDetach);
    ok &= expectFalse("pose-preservation hot reload does not tear down a legal carry",
        rock::requiresEquippedWeaponHandlingModeReconcile(
            coreWeaponHandling,
            detachPosePreservationDisabled));
    auto addonDetachHandling = fixedOnlyHandling;
    addonDetachHandling.firingGripOwnershipEnabled = true;
    addonDetachHandling.primaryDetachEnabled = true;
    ok &= expectTrue("removing provider detach capability reconciles manual weapon state",
        rock::requiresEquippedWeaponHandlingModeReconcile(
            addonDetachHandling,
            fixedOnlyHandling));
    auto integratedDetachDisabled = coreWeaponHandling;
    integratedDetachDisabled.immersiveWeapon.
        firingGripDetachEnabled = false;
    ok &= expectTrue("removing integrated immersive detach reconciles manual weapon state",
        rock::requiresEquippedWeaponHandlingModeReconcile(
            coreWeaponHandling,
            integratedDetachDisabled));
    const auto disabledIntegratedDetach =
        rock::resolveEquippedWeaponDetachDecision(
            integratedDetachDisabled);
    ok &= expectEqual("disabled integrated policy grants no detach authority",
        disabledIntegratedDetach.authority,
        rock::immersive_weapon_policy::DetachAuthority::None);
    ok &= expectFalse("disabled integrated policy cannot detach a firing hand",
        disabledIntegratedDetach.primaryDetachEnabled);
    ok &= expectFalse("disabled integrated policy cannot preserve a detach pose",
        disabledIntegratedDetach.preserveWeaponPoseOnDetach);
    ok &= expectTrue("an addon override that disables handoff reconciles the live switch",
        rock::requiresEquippedWeaponHandlingModeReconcile(
            coreWeaponHandling,
            fixedOnlyHandling));
    ok &= expectFalse("gaining ROCK handoff capability does not require teardown",
        rock::requiresEquippedWeaponHandlingModeReconcile(
            fixedOnlyHandling,
            coreWeaponHandling));

    rock::provider::RockProviderEquippedWeaponHandlingRequestV1 legacyStashRequest{};
    legacyStashRequest.flags =
        static_cast<std::uint32_t>(
            rock::provider::RockProviderEquippedWeaponHandlingFlagV1::FiringGripOwnership) |
        static_cast<std::uint32_t>(
            rock::provider::RockProviderEquippedWeaponHandlingFlagV1::PrimaryDetach) |
        static_cast<std::uint32_t>(
            rock::provider::RockProviderEquippedWeaponHandlingFlagV1::EquippedWeaponShoulderStash);
    legacyStashRequest.firingGripReattachRadiusGameUnits = 9.0f;
    legacyStashRequest.weaponGripHapticDurationSeconds = 0.20f;
    legacyStashRequest.firingGripAttachHapticIntensity = 0.40f;
    legacyStashRequest.firingGripDetachHapticIntensity = 0.50f;
    const auto legacyStashHandling = rock::makeEquippedWeaponHandlingSettings(
        fixedOnlyBaseline,
        &legacyStashRequest);
    ok &= expectFalse("legacy provider stash bit cannot enable ROCK shoulder stash",
        legacyStashHandling.equippedWeaponShoulderStashEnabled);
    ok &= expectTrue("legacy provider detach remains available to its other handling paths",
        legacyStashHandling.primaryDetachEnabled);
    const auto providerDetach =
        rock::resolveEquippedWeaponDetachDecision(
            legacyStashHandling);
    ok &= expectEqual("provider PrimaryDetach retains firing-role authority",
        providerDetach.authority,
        rock::immersive_weapon_policy::DetachAuthority::ExternalProvider);
    ok &= expectFalse("provider detach does not inherit integrated pose preservation",
        providerDetach.preserveWeaponPoseOnDetach);
    ok &= expectTrue("provider PrimaryDetach remains all-firing-hand capable",
        providerDetach.primaryDetachEnabled);
    ok &= expectFalse("provider PrimaryDetach does not inherit integrated pose preservation",
        providerDetach.preserveWeaponPoseOnDetach);
    ok &= expectNear("provider PrimaryDetach owns reattach tuning",
        providerDetach.reattachRadiusGameUnits,
        9.0f);
    ok &= expectNear("provider PrimaryDetach owns attach haptic tuning",
        providerDetach.gripAttachHapticIntensity,
        0.40f);
    ok &= expectNear("provider PrimaryDetach owns detach haptic tuning",
        providerDetach.gripDetachHapticIntensity,
        0.50f);
    auto holdToGrabHandling = coreWeaponHandling;
    holdToGrabHandling.weaponGrabMode = WeaponGrabMode::HoldBoth;
    ok &= expectTrue("changing equipped-weapon grab input mode reconciles live grips",
        rock::requiresEquippedWeaponHandlingModeReconcile(
            coreWeaponHandling,
            holdToGrabHandling));

    {
        toggle_grab::RuntimeState toggleState{};
        toggle_grab::Input toggleInput{
            .weaponGrabMode = WeaponGrabMode::ToggleBoth,
            .inputAllowed = true,
            .weaponOwnershipKey = 0x1234u,
            .occupancy = {},
            .left = {
                .held = true,
                .pressed = true,
                .released = false,
            },
            .right = {
                .held = true,
                .pressed = true,
                .released = false,
            },
        };
        auto toggleDecision = toggle_grab::prepare(toggleState, toggleInput);
        ok &= expectTrue("open left weapon grip passes its acquisition press",
            toggleDecision.left.held && toggleDecision.left.pressed);
        ok &= expectTrue("open right weapon grip passes its acquisition press",
            toggleDecision.right.held && toggleDecision.right.pressed);
        ok &= expectFalse("acquisition presses are not consumed as releases",
            toggleDecision.leftReleasePressConsumed ||
                toggleDecision.rightReleasePressConsumed);

        const auto toggleAcquisition = toggle_grab::reconcile(
            toggleState,
            WeaponGrabMode::ToggleBoth,
            toggleInput.weaponOwnershipKey,
            toggle_grab::GripOccupancy{ .left = { .partGripActive = true }, .right = { .partGripActive = true } },
            toggle_grab::GripReleaseRetention{});
        ok &= expectTrue("new left weapon occupancy owns its acquisition press",
            toggleAcquisition.leftGripAcquired);
        ok &= expectTrue("new right weapon occupancy owns its acquisition press",
            toggleAcquisition.rightGripAcquired);
        toggleInput.occupancy = { .left = { .partGripActive = true }, .right = { .partGripActive = true } };
        toggleInput.left = { .released = true };
        toggleInput.right = { .released = true };
        toggleDecision = toggle_grab::prepare(toggleState, toggleInput);
        ok &= expectTrue("left weapon grip stays latched after physical release",
            toggleDecision.left.held && !toggleDecision.left.released);
        ok &= expectTrue("right weapon grip stays latched after physical release",
            toggleDecision.right.held && !toggleDecision.right.released);

        toggleInput.left = { .held = true, .pressed = true };
        toggleInput.right = {};
        toggleDecision = toggle_grab::prepare(toggleState, toggleInput);
        ok &= expectTrue("second left press requests a logical release",
            !toggleDecision.left.held && toggleDecision.left.released);
        ok &= expectTrue("second left press is reserved for the weapon release",
            toggleDecision.leftReleasePressConsumed);
        ok &= expectTrue("right weapon grip remains independently latched",
            toggleDecision.right.held &&
                !toggleDecision.rightReleasePressConsumed);

        static_cast<void>(toggle_grab::reconcile(
            toggleState,
            WeaponGrabMode::ToggleBoth,
            toggleInput.weaponOwnershipKey,
            toggle_grab::GripOccupancy{ .left = { .partGripActive = true }, .right = { .partGripActive = true } },
            toggle_grab::GripReleaseRetention{}));
        toggleInput.occupancy = { .left = { .partGripActive = true }, .right = { .partGripActive = true } };
        toggleInput.left = { .held = true };
        toggleDecision = toggle_grab::prepare(toggleState, toggleInput);
        ok &= expectTrue("pending left release stays logically open for debounce",
            !toggleDecision.left.held && !toggleDecision.left.released);

        static_cast<void>(toggle_grab::reconcile(
            toggleState,
            WeaponGrabMode::ToggleBoth,
            toggleInput.weaponOwnershipKey,
            toggle_grab::GripOccupancy{ .left = { .partGripActive = false }, .right = { .partGripActive = true } },
            toggle_grab::GripReleaseRetention{}));
        toggleInput.occupancy = { .left = { .partGripActive = false }, .right = { .partGripActive = true } };
        toggleInput.left = { .held = true };
        toggleDecision = toggle_grab::prepare(toggleState, toggleInput);
        ok &= expectFalse("release press tail cannot re-acquire the left weapon grip",
            toggleDecision.left.held || toggleDecision.left.pressed);
        ok &= expectTrue("right latch survives the peer hand release",
            toggleDecision.right.held);

        toggleInput.left = { .released = true };
        toggleDecision = toggle_grab::prepare(toggleState, toggleInput);
        ok &= expectEqual("physical release rearms only the released hand",
            toggleState.hands[toggle_grab::handIndex(true)],
            toggle_grab::HandState::Open);
        ok &= expectEqual("peer hand remains latched after left rearm",
            toggleState.hands[toggle_grab::handIndex(false)],
            toggle_grab::HandState::Latched);

        toggleInput.weaponGrabMode = WeaponGrabMode::HoldBoth;
        toggleInput.right = { .held = false, .released = true };
        toggleDecision = toggle_grab::prepare(toggleState, toggleInput);
        ok &= expectTrue("disabled toggle mode passes physical release input",
            toggleDecision.right.released);
        ok &= expectEqual("hold-mode support clears its former latch",
            toggleState.hands[toggle_grab::handIndex(false)],
            toggle_grab::HandState::Open);
        toggleInput.weaponOwnershipKey = 0;
        static_cast<void>(toggle_grab::prepare(toggleState, toggleInput));
        ok &= expectEqual("unequip clears weapon identity",
            toggleState.weaponOwnershipKey,
            std::uint64_t{ 0 });
    }

    {
        toggle_grab::RuntimeState retainedState{};
        toggle_grab::Input retainedInput{
            .weaponGrabMode = WeaponGrabMode::ToggleBoth,
            .inputAllowed = true,
            .weaponOwnershipKey = 0x1235u,
            .occupancy = {},
            .left = {},
            .right = {
                .held = true,
                .pressed = true,
                .released = false,
            },
        };
        auto retainedDecision = toggle_grab::prepare(retainedState, retainedInput);
        static_cast<void>(toggle_grab::reconcile(
            retainedState,
            WeaponGrabMode::ToggleBoth,
            retainedInput.weaponOwnershipKey,
            toggle_grab::GripOccupancy{ .right = { .partGripActive = true } },
            toggle_grab::GripReleaseRetention{}));
        retainedInput.occupancy = { .right = { .partGripActive = true } };
        retainedInput.right = { .held = true, .pressed = true };
        retainedDecision = toggle_grab::prepare(retainedState, retainedInput);
        ok &= expectTrue("latched right grip press requests a logical release",
            !retainedDecision.right.held && retainedDecision.right.released);
        static_cast<void>(toggle_grab::reconcile(
            retainedState,
            WeaponGrabMode::ToggleBoth,
            retainedInput.weaponOwnershipKey,
            toggle_grab::GripOccupancy{ .right = { .partGripActive = true } },
            toggle_grab::GripReleaseRetention{}));
        ok &= expectEqual("an unresolved release stays pending while the grip is still occupied",
            retainedState.hands[toggle_grab::handIndex(false)],
            toggle_grab::HandState::ReleasePending);
        static_cast<void>(toggle_grab::reconcile(
            retainedState,
            WeaponGrabMode::ToggleBoth,
            retainedInput.weaponOwnershipKey,
            toggle_grab::GripOccupancy{ .right = { .partGripActive = true } },
            toggle_grab::GripReleaseRetention{ .right = true }));
        ok &= expectEqual("a refused last-carrier release re-latches the hand",
            retainedState.hands[toggle_grab::handIndex(false)],
            toggle_grab::HandState::Latched);
        retainedInput.right = { .held = true };
        retainedDecision = toggle_grab::prepare(retainedState, retainedInput);
        ok &= expectTrue("the re-latched hand reports a closed grip while the press tail is still held",
            retainedDecision.right.held && !retainedDecision.right.pressed);
        retainedInput.right = { .released = true };
        retainedDecision = toggle_grab::prepare(retainedState, retainedInput);
        ok &= expectTrue("physical release keeps the re-latched grip closed",
            retainedDecision.right.held && !retainedDecision.right.released);
        retainedInput.right = { .held = true, .pressed = true };
        retainedDecision = toggle_grab::prepare(retainedState, retainedInput);
        ok &= expectTrue("the next press is again a release request",
            retainedDecision.right.released &&
                retainedDecision.rightReleasePressConsumed);
        static_cast<void>(toggle_grab::reconcile(
            retainedState,
            WeaponGrabMode::ToggleBoth,
            retainedInput.weaponOwnershipKey,
            toggle_grab::GripOccupancy{ .right = { .partGripActive = false } },
            toggle_grab::GripReleaseRetention{ .right = true }));
        ok &= expectEqual("retention never re-latches a vacated grip",
            retainedState.hands[toggle_grab::handIndex(false)],
            toggle_grab::HandState::BlockedUntilRelease);
    }

    for (const auto mode : { WeaponGrabMode::ToggleFiringOnly, WeaponGrabMode::ToggleBoth }) {
        const bool toggleEnabled = mode == WeaponGrabMode::ToggleBoth;
        for (const bool firingIsLeft : { false, true }) {
            toggle_grab::RuntimeState state{ .weaponOwnershipKey = 0x1236u };
            toggle_grab::Input input{
                .weaponGrabMode = mode,
                .inputAllowed = true,
                .weaponOwnershipKey = state.weaponOwnershipKey,
            };
            auto& firing = firingIsLeft ? input.occupancy.left : input.occupancy.right;
            auto& support = firingIsLeft ? input.occupancy.right : input.occupancy.left;
            auto& firingButton = firingIsLeft ? input.left : input.right;
            auto& supportButton = firingIsLeft ? input.right : input.left;
            const auto firingDecision = [&](const toggle_grab::Decision& decision) {
                return firingIsLeft ? decision.left : decision.right;
            };
            const auto supportDecision = [&](const toggle_grab::Decision& decision) {
                return firingIsLeft ? decision.right : decision.left;
            };
            const auto reconcile = [&](const toggle_grab::GripReleaseRetention& retention = {}) {
                return toggle_grab::reconcile(state, mode,
                    input.weaponOwnershipKey, input.occupancy, retention);
            };

            firing = { .firingGripActive = true };
            auto decision = toggle_grab::prepare(state, input);
            ok &= expectTrue("either firing hand latches without a physical hold in both firing-toggle modes",
                firingDecision(decision).held);

            supportButton = { .held = true, .pressed = true };
            decision = toggle_grab::prepare(state, input);
            ok &= expectTrue("support acquisition cannot release an unsqueezed firing hand",
                supportDecision(decision).held && firingDecision(decision).held &&
                    !firingDecision(decision).released);
            support = { .partGripActive = true };
            const auto supportAcquired = reconcile();
            ok &= expectEqual("ordinary support acquisition latches only in configured toggle mode",
                firingIsLeft ? supportAcquired.rightGripAcquired : supportAcquired.leftGripAcquired,
                toggleEnabled);
            supportButton = { .released = true };
            decision = toggle_grab::prepare(state, input);
            ok &= expectEqual("dynamic full-authority and authored support obey configured release mode",
                supportDecision(decision).held, toggleEnabled);
            ok &= expectTrue("support release leaves firing grip latched",
                firingDecision(decision).held);

            // A provider may replace a support grip with attach-only glue.
            // Neither a previous latch nor the hand's role may retain it.
            support.partGripAttachOnly = true;
            decision = toggle_grab::prepare(state, input);
            ok &= expectTrue("attach-only conversion immediately follows physical release",
                !supportDecision(decision).held && supportDecision(decision).released);
            supportButton = { .held = true, .pressed = true };
            decision = toggle_grab::prepare(state, input);
            ok &= expectTrue("attach-only squeeze stays a physical hold instead of a toggle release",
                supportDecision(decision).held && supportDecision(decision).pressed &&
                    !decision.leftReleasePressConsumed && !decision.rightReleasePressConsumed);
            const auto attachAcquired = reconcile();
            ok &= expectFalse("attach-only reconciliation never arms a latch",
                attachAcquired.leftGripAcquired || attachAcquired.rightGripAcquired);
            supportButton = { .released = true };
            decision = toggle_grab::prepare(state, input);
            ok &= expectTrue("reload part releases on button-up in either firing-toggle mode",
                !supportDecision(decision).held && supportDecision(decision).released);

            support = {};
            firingButton = { .held = true, .pressed = true };
            decision = toggle_grab::prepare(state, input);
            ok &= expectTrue("firing squeeze explicitly requests release in either firing-toggle mode",
                !firingDecision(decision).held && firingDecision(decision).released);
            toggle_grab::GripReleaseRetention retention{};
            (firingIsLeft ? retention.left : retention.right) = true;
            static_cast<void>(reconcile(retention));
            firingButton = { .released = true };
            decision = toggle_grab::prepare(state, input);
            ok &= expectTrue("auto-drop refusal relatches firing grip in either firing-toggle mode",
                firingDecision(decision).held && !firingDecision(decision).released);

            support = { .partGripActive = true };
            firingButton = { .held = true, .pressed = true };
            static_cast<void>(toggle_grab::prepare(state, input));
            firing = {};
            static_cast<void>(reconcile());
            firingButton = { .held = true };
            decision = toggle_grab::prepare(state, input);
            ok &= expectFalse("firing release press cannot reacquire after its grip becomes empty",
                firingDecision(decision).held || firingDecision(decision).pressed);
            firingButton = { .released = true };
            static_cast<void>(toggle_grab::prepare(state, input));

            firingButton = { .held = true, .pressed = true };
            decision = toggle_grab::prepare(state, input);
            ok &= expectTrue("fresh squeeze rearms the detached firing hand",
                firingDecision(decision).held && firingDecision(decision).pressed);
            firing = { .partGripActive = true, .partGripAttachOnly = true };
            support = { .firingGripActive = true };
            static_cast<void>(reconcile());
            firingButton = { .released = true };
            supportButton = { .released = true };
            decision = toggle_grab::prepare(state, input);
            ok &= expectTrue("after handoff old firing hand releases attach-only while new firing hand latches",
                !firingDecision(decision).held && firingDecision(decision).released &&
                    supportDecision(decision).held && !supportDecision(decision).released);
        }
    }

    using rock::weapon_support_authority_policy::canApplyFiringGripProximityAuthority;
    using rock::weapon_support_authority_policy::canCarryAfterFiringGripDetach;
    using rock::weapon_support_authority_policy::canPromoteSupportGripToFiringGrip;
    using rock::weapon_support_authority_policy::resolveFiringGripProximityAuthorityMode;
    using rock::weapon_support_authority_policy::shouldApplyVisualOnlySupportRecoilAssist;
    using rock::weapon_support_authority_policy::WeaponSupportAuthorityMode;
    ok &= expectTrue("firing-grip proximity contract always applies to eligible equipped weapons",
        canApplyFiringGripProximityAuthority(false));
    ok &= expectFalse("firing-grip proximity never changes a provider-mandated grab mode",
        canApplyFiringGripProximityAuthority(true));
    ok &= expectTrue("full two-hand support can carry after firing-grip detach",
        canCarryAfterFiringGripDetach(
            WeaponSupportAuthorityMode::FullTwoHandedSolver));
    ok &= expectFalse("visual-only support cannot carry after firing-grip detach",
        canCarryAfterFiringGripDetach(
            WeaponSupportAuthorityMode::VisualOnlySupport));
    ok &= expectEqual("any weapon grab near the firing grip stays visual-only",
        resolveFiringGripProximityAuthorityMode(5.5f, 6.0f),
        WeaponSupportAuthorityMode::VisualOnlySupport);
    ok &= expectEqual("any weapon grab at the firing-grip radius stays visual-only",
        resolveFiringGripProximityAuthorityMode(6.0f, 6.0f),
        WeaponSupportAuthorityMode::VisualOnlySupport);
    ok &= expectEqual("any weapon grab away from the firing grip takes full authority",
        resolveFiringGripProximityAuthorityMode(6.5f, 6.0f),
        WeaponSupportAuthorityMode::FullTwoHandedSolver);
    ok &= expectTrue("active core visual-only support receives recoil-only authority",
        shouldApplyVisualOnlySupportRecoilAssist(
            WeaponSupportAuthorityMode::VisualOnlySupport,
            true,
            false,
            false));
    ok &= expectFalse("full two-handed support keeps its geometric recoil solve",
        shouldApplyVisualOnlySupportRecoilAssist(
            WeaponSupportAuthorityMode::FullTwoHandedSolver,
            true,
            false,
            false));
    ok &= expectFalse("inactive visual support never changes recoil",
        shouldApplyVisualOnlySupportRecoilAssist(
            WeaponSupportAuthorityMode::VisualOnlySupport,
            false,
            false,
            false));
    ok &= expectFalse("provider visual glue never inherits recoil authority",
        shouldApplyVisualOnlySupportRecoilAssist(
            WeaponSupportAuthorityMode::VisualOnlySupport,
            true,
            true,
            false));
    ok &= expectFalse("AttachOnly visual glue never inherits recoil authority",
        shouldApplyVisualOnlySupportRecoilAssist(
            WeaponSupportAuthorityMode::VisualOnlySupport,
            true,
            false,
            true));
    ok &= expectTrue("active support may attempt handoff regardless of authored or dynamic pose selection",
        canPromoteSupportGripToFiringGrip(true, false, true));
    ok &= expectFalse("inactive support cannot attempt handoff",
        canPromoteSupportGripToFiringGrip(false, false, true));
    ok &= expectFalse("AttachOnly support never inherits firing-grip ownership",
        canPromoteSupportGripToFiringGrip(true, true, true));

    using rock::weapon_support_authority_policy::DynamicHandoffGripCaptureInput;
    using rock::weapon_support_authority_policy::shouldCaptureDynamicHandoffGrip;
    {
        namespace zone = rock::firing_grip_reattach_zone_policy;
        const rock::WeaponInteractionRuntimeState normal{};
        const rock::WeaponInteractionContact noContact{};
        ok &= expectEqual("normal support still needs its existing acquisition route",
            rock::routeWeaponInteraction(noContact, normal).kind,
            rock::WeaponInteractionKind::None);
        constexpr float reach = 10.0f;
        for (const float side : { -1.0f, 1.0f }) {
            for (const float distance : { 7.76f, 10.0f }) {
                const auto inside = zone::evaluateZone({
                    .palmWorld = { side * distance, 3.0f, 0.0f },
                    .weaponLeftAxisWorld = { 1.0f, 0.0f, 0.0f },
                    .reachGameUnits = reach,
                    .radiusGameUnits = 3.0f,
                });
                const bool available = rock::canAcquireFiringGripHandoff(inside.inside, normal, false);
                ok &= expectTrue("firing-grip cylinders admit either side without a contact or authored seat",
                    available && inside.indicatorValid);
                ok &= expectFalse("entering the cylinder alone does not attach",
                    rock::weapon_two_handed_grip_math::canStartSupportGrip(available, false, false));
                ok &= expectTrue("grab commits the cylinder-admitted support hand",
                    rock::weapon_two_handed_grip_math::canStartSupportGrip(available, true, false));
            }
        }
        for (const auto palm : { zone::Vec3{ 10.01f, 0.0f, 0.0f },
                 zone::Vec3{ -10.01f, 0.0f, 0.0f }, zone::Vec3{ 0.0f, 3.01f, 0.0f } }) {
            const auto outside = zone::evaluateZone({
                .palmWorld = palm,
                .weaponLeftAxisWorld = { 1.0f, 0.0f, 0.0f },
                .reachGameUnits = reach,
                .radiusGameUnits = 3.0f,
            });
            ok &= expectFalse("outside the shared cylinders has neither handoff admission nor marker",
                rock::canAcquireFiringGripHandoff(outside.inside, normal, false) || outside.indicatorValid);
        }
        auto reserved = normal;
        reserved.supportGripAllowed = false;
        ok &= expectFalse("handoff respects support reservations",
            rock::canAcquireFiringGripHandoff(true, reserved, false));
        auto providerPart = normal;
        providerPart.providerPartAuthority.active = true;
        ok &= expectFalse("provider parts retain their own grab route",
            rock::canAcquireFiringGripHandoff(true, providerPart, false));
        ok &= expectFalse("no-contact handoff cannot bypass an exclusive part whitelist",
            rock::canAcquireFiringGripHandoff(true, normal, true));
    }
    const DynamicHandoffGripCaptureInput dynamicHandoffGrip{
        .normalSupportAcquisition = true,
        .supportPoseAbsent = true,
        .firingGripProximityAuthorityEnabled = true,
        .providerPartAuthorityActive = false,
        .authoredCaptureEligible = false,
        .supportPalmInsideHandoffZone = true,
    };
    ok &= expectTrue(
        "confirmed one-handed animation permits dynamic support at the shared station",
        shouldCaptureDynamicHandoffGrip(dynamicHandoffGrip));
    {
        namespace zone = rock::firing_grip_reattach_zone_policy;
        const auto evaluateHandoff = [](const zone::Vec3& palm) {
            return zone::evaluateZone({
                .palmWorld = palm,
                .weaponLeftAxisWorld = { 1.0f, 0.0f, 0.0f },
                .reachGameUnits = 5.0f,
                .radiusGameUnits = 2.0f,
            });
        };
        auto input = dynamicHandoffGrip;
        for (const float side : { -1.0f, 1.0f }) {
            const auto inside = evaluateHandoff({ side * 5.0f, 2.0f, 0.0f });
            input.supportPalmInsideHandoffZone = inside.inside;
            ok &= expectTrue("both cylinder rims admit handoff beyond the former sphere",
                inside.radialDistanceGameUnits > 5.0f &&
                    shouldCaptureDynamicHandoffGrip(input));
            ok &= expectTrue("an eligible cylinder rim has an indicator on its approach side",
                inside.indicatorValid && inside.indicatorWorld.x * side > 0.0f);
        }
        for (const auto palm : { zone::Vec3{ 0.0f, 2.1f, 0.0f },
                 zone::Vec3{ 5.1f, 0.0f, 0.0f } }) {
            const auto outside = evaluateHandoff(palm);
            input.supportPalmInsideHandoffZone = outside.inside;
            ok &= expectFalse("outside cylinder width or reach rejects handoff and its indicator",
                shouldCaptureDynamicHandoffGrip(input) || outside.indicatorValid);
        }
        input.supportPalmInsideHandoffZone = true;
        input.authoredCaptureEligible = true;
        ok &= expectFalse("eligible authored support wins an overlapping firing-hand station",
            shouldCaptureDynamicHandoffGrip(input));
    }
    {
        auto input = dynamicHandoffGrip;
        input.authoredCaptureEligible = true;
        ok &= expectFalse(
            "eligible authored support wins regardless of its distance from the firing seat",
            shouldCaptureDynamicHandoffGrip(input));
    }
    {
        auto input = dynamicHandoffGrip;
        input.authoredCaptureEligible = true;
        ok &= expectFalse(
            "authored firing-grip seat remains preferred for pistols",
            shouldCaptureDynamicHandoffGrip(input));
    }
    {
        auto input = dynamicHandoffGrip;
        input.supportPalmInsideHandoffZone = false;
        ok &= expectFalse(
            "dynamic handoff cannot escape the lateral cylinders",
            shouldCaptureDynamicHandoffGrip(input));
    }
    {
        auto input = dynamicHandoffGrip;
        input.providerPartAuthorityActive = true;
        ok &= expectFalse(
            "exact provider target remains ahead of dynamic handoff",
            shouldCaptureDynamicHandoffGrip(input));
    }
    {
        auto input = dynamicHandoffGrip;
        input.normalSupportAcquisition = false;
        ok &= expectFalse(
            "part-carry acquisition cannot use the handoff bypass",
            shouldCaptureDynamicHandoffGrip(input));
    }
    {
        auto input = dynamicHandoffGrip;
        input.supportPoseAbsent = false;
        ok &= expectFalse(
            "missing or invalid data cannot authorize dynamic support",
            shouldCaptureDynamicHandoffGrip(input));
    }
    {
        auto input = dynamicHandoffGrip;
        input.firingGripProximityAuthorityEnabled = false;
        ok &= expectFalse(
            "provider-owned proximity policy cannot create a local handoff station",
            shouldCaptureDynamicHandoffGrip(input));
    }

    using rock::weapon_interaction_probe_math::isBetterProbeCandidate;
    using rock::weapon_interaction_probe_math::ProbeCandidateRank;
    ok &= expectTrue("closer exact weapon surface always wins",
        isBetterProbeCandidate(
            ProbeCandidateRank{ .distanceSquaredGame = 4.0f, .aabbDiagonalSquaredGame = 1225.0f, .semanticPriority = 62 },
            ProbeCandidateRank{ .distanceSquaredGame = 9.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 95 }));
    ok &= expectFalse("farther exact weapon surface always loses",
        isBetterProbeCandidate(
            ProbeCandidateRank{ .distanceSquaredGame = 9.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 95 },
            ProbeCandidateRank{ .distanceSquaredGame = 4.0f, .aabbDiagonalSquaredGame = 1225.0f, .semanticPriority = 62 }));
    ok &= expectTrue("smaller part breaks an exact surface-distance tie",
        isBetterProbeCandidate(
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 95 },
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 1225.0f, .semanticPriority = 62 }));
    ok &= expectFalse("tiny overlapping AABB cannot beat a closer rendered surface",
        isBetterProbeCandidate(
            ProbeCandidateRank{ .distanceSquaredGame = 0.81f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 95 },
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 1225.0f, .semanticPriority = 62 }));
    ok &= expectTrue("semantic priority breaks equal-size containment ties",
        isBetterProbeCandidate(
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 95 },
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 62 }));
    ok &= expectFalse("equal candidates keep the current best",
        isBetterProbeCandidate(
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 62 },
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 62 }));
    ok &= expectEqual("releasing support from a two-hand hold retains the firing hand",
        resolveSupportReleaseManualAction(SupportReleaseOwnershipInput{ .firingGripOwnershipEnabled = true }),
        SupportReleaseManualAction::KeepPrimaryOwnership);
    ok &= expectEqual("without manual firing ownership support release ends only support",
        resolveSupportReleaseManualAction(SupportReleaseOwnershipInput{}), SupportReleaseManualAction::EndSupportOnly);
    using rock::weapon_two_handed_grip_math::canReleaseCarryGrip;
    {
        using HandGrip = rock::equipped_weapon_toggle_grab_policy::HandGripOccupancy;
        using rock::equipped_weapon_drop_policy::canStartAutoDrop;
        const HandGrip carryingPart{ .partGripActive = true };
        const HandGrip visualPart{ .partGripActive = true, .partGripAttachOnly = true };
        const HandGrip firing{ .firingGripActive = true };
        ok &= expectTrue("attach-only still occupies the hand", visualPart.weaponEngaged());
        ok &= expectFalse("attach-only does not carry the weapon", visualPart.carriesWeapon());
        for (const auto carrier : { carryingPart, firing }) {
            for (const bool carrierIsLeft : { false, true }) {
                const auto left = carrierIsLeft ? carrier : visualPart;
                const auto right = carrierIsLeft ? visualPart : carrier;
                const bool allowed = canStartAutoDrop(left.carriesWeapon(), right.carriesWeapon(), false);
                ok &= expectTrue("visual peer permits last-carrier transfer on either hand", allowed);
                ok &= expectTrue("real carrier releases with visual peer", canReleaseCarryGrip(
                    carrier.carriesWeapon(), visualPart.carriesWeapon(), allowed));
                ok &= expectFalse("disabled last-grip release still retains real carrier", canReleaseCarryGrip(
                    carrier.carriesWeapon(), visualPart.carriesWeapon(), false));
                ok &= expectFalse("refused release still requires a new hold", canReleaseCarryGrip(
                    carrier.carriesWeapon(), visualPart.carriesWeapon(), allowed, true));
                ok &= expectFalse("firing-station hover still prevents transfer", canStartAutoDrop(
                    left.carriesWeapon(), right.carriesWeapon(), true));
            }
        }
        using rock::equipped_weapon_drop_policy::simultaneousReleaseSource;
        using rock::equipped_weapon_drop_policy::SourceHand;
        ok &= expectEqual("simultaneous releases select the firing carrier once",
            simultaneousReleaseSource(true, true, true, true, true, true, false, true), SourceHand::Right);
        ok &= expectEqual("simultaneous part releases select the current pivot",
            simultaneousReleaseSource(true, true, true, true, true, false, false, true), SourceHand::Left);
        ok &= expectEqual("drop off preserves a simultaneous carrier",
            simultaneousReleaseSource(false, true, true, true, true, true, false, true), SourceHand::None);
        ok &= expectFalse("two visual grips never create a carrier", canStartAutoDrop(visualPart.carriesWeapon(), visualPart.carriesWeapon(), false));
    }
    ok &= expectTrue("a carry grip releases while its peer still carries",
        canReleaseCarryGrip(true, true, false));
    ok &= expectTrue("the last carry grip releases when the last-grip drop is enabled",
        canReleaseCarryGrip(true, false, true));
    ok &= expectFalse("the last carry grip is retained when the last-grip drop is disabled",
        canReleaseCarryGrip(true, false, false));
    ok &= expectFalse("a refused simultaneous release cannot become a delayed auto-drop",
        canReleaseCarryGrip(true, false, true, true));
    ok &= expectTrue("a fresh hold clears refusal before a later last-hand release",
        canReleaseCarryGrip(true, false, true, false));
    ok &= expectTrue("attach-only glue releases regardless of the last-grip drop",
        canReleaseCarryGrip(false, false, false));

    using rock::weapon_two_handed_grip_math::canStartFreeHandPartGrip;
    using rock::weapon_two_handed_grip_math::canAttemptFiringGripReattach;
    using rock::weapon_two_handed_grip_math::FiringGripReattachInput;
    using rock::weapon_two_handed_grip_math::shouldReattachFiringGripOnGrab;
    ok &= expectTrue("firing grip reattach is eligible during part carry",
        canAttemptFiringGripReattach(FiringGripReattachInput{
            .partCarryActive = true,
        }));
    ok &= expectFalse("firing grip reattach requires part-carry state",
        canAttemptFiringGripReattach(FiringGripReattachInput{}));
    ok &= expectFalse("firing grip reattach is blocked while a menu owns input",
        canAttemptFiringGripReattach(FiringGripReattachInput{
            .partCarryActive = true,
            .menuInputActive = true,
        }));
    ok &= expectFalse("firing grip reattach is blocked while the hand holds an object",
        canAttemptFiringGripReattach(FiringGripReattachInput{
            .partCarryActive = true,
            .handHoldingObject = true,
        }));

    ok &= expectTrue("held grab with the palm inside the reattach zone re-takes the firing grip",
        shouldReattachFiringGripOnGrab(true, true));
    ok &= expectFalse("grab reattach requires the palm inside the zone",
        shouldReattachFiringGripOnGrab(true, false));
    ok &= expectFalse("an open hand never re-takes the firing grip",
        shouldReattachFiringGripOnGrab(false, true));

    using rock::weapon_two_handed_grip_math::isFiringGripReattachHoverCandidate;
    ok &= expectTrue("open palm inside the reattach zone is a hover candidate",
        isFiringGripReattachHoverCandidate(false, true));
    ok &= expectFalse("hover candidate requires the palm inside the zone",
        isFiringGripReattachHoverCandidate(false, false));
    ok &= expectFalse("a held grab is the reattach itself, never a hover",
        isFiringGripReattachHoverCandidate(true, true));

    {
        namespace zone = rock::firing_grip_reattach_zone_policy;
        using zone::Side;
        using zone::Vec3;
        constexpr float kReach = 12.0f;
        constexpr float kCylinderRadius = 2.0f;

        const Vec3 grip{ 10.0f, -4.0f, 55.0f };
        // Unit lateral axis with a deliberately non-axis-aligned direction.
        const Vec3 weaponLeft{ 0.6f, 0.8f, 0.0f };
        const Vec3 across{ -0.8f, 0.6f, 0.0f };
        const auto palmAt = [&](const float along, const float perpendicular) {
            return Vec3{
                grip.x + weaponLeft.x * along + across.x * perpendicular,
                grip.y + weaponLeft.y * along + across.y * perpendicular,
                grip.z + weaponLeft.z * along + across.z * perpendicular,
            };
        };
        const auto evaluate = [&](const Vec3& palm,
                                  const float reach,
                                  const float radius) {
            return zone::evaluateZone(zone::ZoneInput{
                .gripWorld = grip,
                .palmWorld = palm,
                .weaponLeftAxisWorld = weaponLeft,
                .reachGameUnits = reach,
                .radiusGameUnits = radius,
            });
        };

        const auto leftMid = evaluate(palmAt(6.0f, 1.5f), kReach, kCylinderRadius);
        ok &= expectTrue("palm inside the weapon-left cylinder is inside the zone",
            leftMid.inside && leftMid.reachPass && leftMid.radiusPass);
        ok &= expectTrue("left cylinder reports the LEFT side",
            leftMid.side == Side::Left);
        ok &= expectNear("left cylinder reports the distance along the axis",
            leftMid.alongAxisGameUnits, 6.0f, 0.001f);
        ok &= expectNear("left cylinder reports the perpendicular offset",
            leftMid.perpendicularDistanceGameUnits, 1.5f, 0.001f);

        const auto rightFar = evaluate(palmAt(-11.9f, 1.9f), kReach, kCylinderRadius);
        ok &= expectTrue("palm near the end of the weapon-right cylinder is inside the zone",
            rightFar.inside);
        ok &= expectTrue("right cylinder reports the RIGHT side",
            rightFar.side == Side::Right);

        const auto tooWide = evaluate(palmAt(6.0f, 2.5f), kReach, kCylinderRadius);
        ok &= expectTrue("palm off the axis beyond the radius still passes the reach",
            tooWide.reachPass);
        ok &= expectFalse("palm off the axis beyond the radius fails the radius",
            tooWide.radiusPass);
        ok &= expectFalse("palm off the axis beyond the radius is outside the zone",
            tooWide.inside);

        const auto beyondReach = evaluate(palmAt(12.5f, 0.5f), kReach, kCylinderRadius);
        ok &= expectFalse("palm beyond the reach fails the reach",
            beyondReach.reachPass);
        ok &= expectTrue("palm beyond the reach still passes the radius",
            beyondReach.radiusPass);
        ok &= expectFalse("palm beyond the reach is outside the zone",
            beyondReach.inside);

        const auto onGrip = evaluate(palmAt(0.05f, 0.1f), kReach, kCylinderRadius);
        ok &= expectTrue("palm on the grip point is inside the zone without history",
            onGrip.inside);

        const auto above = evaluate(palmAt(0.0f, 3.0f), kReach, kCylinderRadius);
        ok &= expectFalse("palm above the grip beyond the radius is outside the zone",
            above.inside);

        const auto zeroAxis = zone::evaluateZone(zone::ZoneInput{
            .gripWorld = grip,
            .palmWorld = palmAt(2.0f, 0.0f),
            .weaponLeftAxisWorld = Vec3{},
            .reachGameUnits = kReach,
            .radiusGameUnits = kCylinderRadius,
        });
        ok &= expectFalse("zone without a lateral axis fails closed",
            zeroAxis.axisValid || zeroAxis.inside);

        const auto negativeReach = evaluate(palmAt(1.0f, 0.0f), -1.0f, kCylinderRadius);
        ok &= expectFalse("negative reach fails closed",
            negativeReach.reachPass || negativeReach.inside);
        const auto negativeRadius = evaluate(palmAt(1.0f, 0.0f), kReach, -1.0f);
        ok &= expectFalse("negative radius fails closed",
            negativeRadius.radiusPass || negativeRadius.inside);

        const float nan = std::numeric_limits<float>::quiet_NaN();
        const auto nonFinite = evaluate(Vec3{ nan, 0.0f, 0.0f }, kReach, kCylinderRadius);
        ok &= expectFalse("non-finite palm fails every zone gate",
            nonFinite.inside || nonFinite.reachPass || nonFinite.radiusPass);

        ok &= expectTrue("palm inside the left cylinder seats the indicator",
            leftMid.indicatorValid);
        ok &= expectNear("left indicator sits at the offset along weapon left (x)",
            leftMid.indicatorWorld.x,
            grip.x + weaponLeft.x * zone::kIndicatorOffsetGameUnits,
            0.001f);
        ok &= expectNear("left indicator sits at the offset along weapon left (y)",
            leftMid.indicatorWorld.y,
            grip.y + weaponLeft.y * zone::kIndicatorOffsetGameUnits,
            0.001f);
        ok &= expectNear("right indicator sits at the offset along weapon right (x)",
            rightFar.indicatorWorld.x,
            grip.x - weaponLeft.x * zone::kIndicatorOffsetGameUnits,
            0.001f);
        ok &= expectFalse("palm outside the zone seats no indicator",
            tooWide.indicatorValid);
    }

    ok &= expectTrue("free hand part grip starts on grab press over a routed support part",
        canStartFreeHandPartGrip(true, true, false, false));
    ok &= expectFalse("free hand part grip requires a routed support-grip contact",
        canStartFreeHandPartGrip(false, true, false, false));
    ok &= expectFalse("free hand part grip requires a grab press edge",
        canStartFreeHandPartGrip(true, false, false, false));
    ok &= expectFalse("free hand part grip is blocked while the hand holds an object",
        canStartFreeHandPartGrip(true, true, true, false));
    ok &= expectFalse("free hand part grip does not restart while already gripping",
        canStartFreeHandPartGrip(true, true, false, true));

    using namespace rock::equipped_weapon_manual_ownership_policy;
    ok &= expectTrue("provider detach mode enables firing-role ownership",
        firingGripOwnershipEnabled(FiringGripModeAvailability{
            .primaryDetachEnabled = true,
        }));
    ok &= expectTrue("ambidextrous handoff independently enables firing-grip ownership",
        firingGripOwnershipEnabled(FiringGripModeAvailability{
            .ambidextrousHandoffAvailable = true,
        }));
    ok &= expectTrue("integrated detach enables firing-role ownership",
        firingGripOwnershipEnabled(FiringGripModeAvailability{
            .integratedDetachEnabled = true,
        }));
    ok &= expectFalse("firing-grip ownership is disabled when all modes are off",
        firingGripOwnershipEnabled(FiringGripModeAvailability{}));
    for (const bool handIsLeft : { false, true }) {
        ok &= expectTrue("left default preserves either physical trigger-equip hand without enabling handoff",
            shouldStartHeldWeaponEquipOwnership(HeldWeaponEquipOwnershipInput{
                .modes = FiringGripModeAvailability{ .leftHandedModeEnabled = true },
                .handIsLeft = handIsLeft,
                .holdingLooseWeapon = true,
            }));
    }
    ok &= expectTrue("default equip starts without a held grab button",
        shouldStartPendingPrimaryOnlyGrip(true, false, PrimaryOnlyStartSource::DefaultEquip));
    ok &= expectTrue("default equip waits for authored readiness without a held grab button",
        shouldKeepPendingPrimaryOnlyStart(PendingPrimaryOnlyStartInput{
            .pending = true,
            .source = PrimaryOnlyStartSource::DefaultEquip,
        }));
    ok &= expectTrue("left-hand trigger equip starts ambidextrous handoff ownership",
        shouldStartHeldWeaponEquipOwnership(HeldWeaponEquipOwnershipInput{
            .modes = FiringGripModeAvailability{
                .ambidextrousHandoffAvailable = true,
            },
            .handIsLeft = true,
            .holdingLooseWeapon = true,
        }));
    ok &= expectFalse("right-hand trigger equip cannot start handoff-only ownership",
        shouldStartHeldWeaponEquipOwnership(HeldWeaponEquipOwnershipInput{
            .modes = FiringGripModeAvailability{
                .ambidextrousHandoffAvailable = true,
            },
            .holdingLooseWeapon = true,
        }));
    ok &= expectTrue("provider detach mode can start right-hand ownership",
        shouldStartHeldWeaponEquipOwnership(HeldWeaponEquipOwnershipInput{
            .modes = FiringGripModeAvailability{
                .primaryDetachEnabled = true,
            },
            .holdingLooseWeapon = true,
        }));
    ok &= expectTrue("integrated detach can start physical-right ownership",
        shouldStartHeldWeaponEquipOwnership(HeldWeaponEquipOwnershipInput{
            .modes = FiringGripModeAvailability{
                .integratedDetachEnabled = true,
            },
            .holdingLooseWeapon = true,
        }));
    ok &= expectTrue("integrated detach can start physical-left ownership",
        shouldStartHeldWeaponEquipOwnership(HeldWeaponEquipOwnershipInput{
            .modes = FiringGripModeAvailability{
                .integratedDetachEnabled = true,
            },
            .handIsLeft = true,
            .holdingLooseWeapon = true,
        }));
    ok &= expectFalse("trigger equip ownership requires a retained loose weapon",
        shouldStartHeldWeaponEquipOwnership(HeldWeaponEquipOwnershipInput{
            .modes = FiringGripModeAvailability{
                .primaryDetachEnabled = true,
                .ambidextrousHandoffAvailable = true,
            },
            .handIsLeft = true,
        }));
    ok &= expectTrue("addon grip-zone flag enables settle equip",
        canSettleEquipInGripZone(true));
    ok &= expectFalse("grip-zone settle equip stays off without an explicit grip-zone capability",
        canSettleEquipInGripZone(false));
    ok &= expectTrue("left trigger-equip ownership tracks its carry frame without grip-zone settle",
        shouldTrackHeldWeaponGripFrame(true, false, true));
    ok &= expectTrue("grip-zone settle independently tracks the held weapon frame",
        shouldTrackHeldWeaponGripFrame(true, true, false));
    ok &= expectFalse("grip-frame tracking requires an actually held loose weapon",
        shouldTrackHeldWeaponGripFrame(false, true, true));
    ok &= expectFalse("held weapon skips grip-frame work when no consumer is active",
        shouldTrackHeldWeaponGripFrame(true, false, false));
    ok &= expectTrue("toggle ownership remains while its logical firing grip is closed",
        shouldRetainPrimaryOnlyOwnership(false, true, true));
    ok &= expectFalse("toggle ownership releases on an explicit logical open without physical detach authority",
        shouldRetainPrimaryOnlyOwnership(false, false, true));
    RuntimeState togglePrimaryOnlyState{
        .active = true,
        .ownershipKey = 0x20u,
    };
    const auto togglePrimaryOnlyRelease = update(
        togglePrimaryOnlyState,
        Input{
            .weaponEquipped = true,
            .ownershipKey = 0x20u,
            .primaryGripRetained =
                shouldRetainPrimaryOnlyOwnership(false, false, true),
        });
    ok &= expectTrue("toggle-open primary-only ownership requests the equipped weapon drop",
        togglePrimaryOnlyRelease.dropRequested);
    ok &= expectTrue("detaching ownership remains while the firing grip is held",
        shouldRetainPrimaryOnlyOwnership(true, true, true));
    ok &= expectFalse("detaching ownership releases when the firing grip opens",
        shouldRetainPrimaryOnlyOwnership(true, false, true));
    ok &= expectTrue("disabled last-grip drop keeps the logically open detaching firing grip under toggle input",
        shouldRetainPrimaryOnlyOwnership(true, false, false));
    ok &= expectFalse("disabled last-grip drop does not change toggle release without detach authority",
        shouldRetainPrimaryOnlyOwnership(false, false, false));
    RuntimeState retainedPrimaryOnlyState{
        .active = true,
        .ownershipKey = 0x23u,
    };
    const auto retainedPrimaryOnlyRelease = update(
        retainedPrimaryOnlyState,
        Input{
            .weaponEquipped = true,
            .ownershipKey = 0x23u,
            .primaryGripRetained =
                shouldRetainPrimaryOnlyOwnership(true, false, false),
        });
    ok &= expectTrue("refused last-carrier release keeps primary-only ownership active",
        retainedPrimaryOnlyRelease.active &&
            !retainedPrimaryOnlyRelease.dropRequested);
    RuntimeState ambidextrousLifecycleState{
        .active = true,
        .ownershipKey = 0x21u,
    };
    const auto ambidextrousWeaponChanged = update(ambidextrousLifecycleState,
        Input{
            .weaponEquipped = true,
            .ownershipKey = 0x22u,
            .primaryGripRetained = shouldRetainPrimaryOnlyOwnership(false, false, true),
        });
    ok &= expectTrue("ambidextrous-only ownership still clears when equipped identity changes",
        ambidextrousWeaponChanged.cleared);
    ok &= expectFalse("ambidextrous identity cleanup never drops the newly equipped weapon",
        ambidextrousWeaponChanged.dropRequested);
    ok &= expectTrue("manual grip feature is available for an equipped instance", featureAvailable(true, true, true, 20));
    ok &= expectFalse("manual grip feature is unavailable without active weapon node", featureAvailable(true, true, false, 20));
    ok &= expectTrue("manual primary ownership is available while colliders build", featureAvailable(true, true, true, 20));
    ok &= expectFalse("manual grip feature requires an equipped instance witness", featureAvailable(true, true, true, 0));
    ok &= expectTrue("pending trigger-equip grip waits while runtime weapon is not ready",
        shouldKeepPendingPrimaryOnlyStart(PendingPrimaryOnlyStartInput{
            .pending = true,
            .gripHeld = true,
            .source = PrimaryOnlyStartSource::HeldWeaponEquip,
            .ownershipModeEnabled = true,
            .primaryPoseBlockerAvailable = true,
        }));
    ok &= expectTrue("accepted trigger equip keeps its chosen hand after physical release",
        shouldKeepPendingPrimaryOnlyStart(PendingPrimaryOnlyStartInput{
            .pending = true,
            .gripHeld = false,
            .source = PrimaryOnlyStartSource::HeldWeaponEquip,
            .ownershipModeEnabled = true,
            .primaryPoseBlockerAvailable = true,
        }));
    ok &= expectTrue("committed shoulder retrieval survives release while the weapon node resolves",
        shouldKeepPendingPrimaryOnlyStart(PendingPrimaryOnlyStartInput{
            .pending = true,
            .gripHeld = false,
            .source = PrimaryOnlyStartSource::ShoulderRetrieval,
            .ownershipModeEnabled = true,
            .primaryPoseBlockerAvailable = true,
        }));
    ok &= expectFalse("committed shoulder retrieval still fails closed without pose authority",
        shouldKeepPendingPrimaryOnlyStart(PendingPrimaryOnlyStartInput{
            .pending = true,
            .gripHeld = false,
            .source = PrimaryOnlyStartSource::ShoulderRetrieval,
            .ownershipModeEnabled = true,
            .primaryPoseBlockerAvailable = false,
        }));
    ok &= expectTrue("committed shoulder retrieval starts after squeeze release",
        shouldStartPendingPrimaryOnlyGrip(true, false, PrimaryOnlyStartSource::ShoulderRetrieval));
    ok &= expectTrue("accepted held-weapon equip starts after squeeze release",
        shouldStartPendingPrimaryOnlyGrip(true, false, PrimaryOnlyStartSource::HeldWeaponEquip));
    ok &= expectFalse("uncommitted grip input does not start after squeeze release",
        shouldStartPendingPrimaryOnlyGrip(true, false, PrimaryOnlyStartSource::GripInput));
    ok &= expectFalse("committed transfer cannot start on a different equipped identity",
        shouldStartPendingPrimaryOnlyGrip(false, true, PrimaryOnlyStartSource::HeldWeaponEquip));
    ok &= expectFalse("held-weapon equip still requires pose authority after release",
        shouldKeepPendingPrimaryOnlyStart(PendingPrimaryOnlyStartInput{
            .pending = true,
            .source = PrimaryOnlyStartSource::HeldWeaponEquip,
            .primaryPoseBlockerAvailable = false,
        }));
    ok &= expectFalse("released grip input cannot persist without an accepted equip",
        shouldKeepPendingPrimaryOnlyStart(PendingPrimaryOnlyStartInput{
            .pending = true,
        }));
    ok &= expectTrue("pending trigger-equip grip is retained for visual-only sidearm release",
        shouldKeepPendingPrimaryOnlyStart(PendingPrimaryOnlyStartInput{
            .pending = true,
            .gripHeld = true,
            .ownershipModeEnabled = true,
            .primaryPoseBlockerAvailable = true,
        }));

    ok &= expectTrue("same equipped weapon preserves ownership across collision rebuild",
        canPreserveManualOwnership(0xAAu, 0xAAu, 0xBBu));
    ok &= expectFalse("different equipped weapon cannot inherit manual ownership",
        canPreserveManualOwnership(0xAAu, 0xCCu, 0xBBu));
    ok &= expectFalse("unpublished collision generation cannot preserve ownership",
        canPreserveManualOwnership(0xAAu, 0xAAu, 0u));
    ok &= expectTrue("primary-only ownership survives unpublished collision generation",
        canPreserveManualOwnership(0xAAu, 0xAAu, 0u, false));
    ok &= expectFalse("provisional primary-only ownership still rejects a different equipped identity",
        canPreserveManualOwnership(0xAAu, 0xCCu, 0u, false));

    GripReleaseDebounceState primaryReleaseDebounce{};
    auto primaryReleaseDecision = debouncePrimaryGripRelease(primaryReleaseDebounce, false);
    ok &= expectTrue("one open primary sample retains firing grip", primaryReleaseDecision.retained);
    ok &= expectFalse("one open primary sample does not confirm release", primaryReleaseDecision.releaseConfirmed);
    primaryReleaseDecision = debouncePrimaryGripRelease(primaryReleaseDebounce, true);
    ok &= expectTrue("held primary sample resets release debounce", primaryReleaseDecision.retained);
    primaryReleaseDecision = debouncePrimaryGripRelease(primaryReleaseDebounce, false);
    primaryReleaseDecision = debouncePrimaryGripRelease(primaryReleaseDebounce, false);
    ok &= expectFalse("stable open primary samples release firing grip", primaryReleaseDecision.retained);
    ok &= expectTrue("stable open primary samples confirm release", primaryReleaseDecision.releaseConfirmed);

    PrimaryReleaseIntentState releaseIntent{};
    auto release = resolvePrimaryReleaseIntent(releaseIntent, {
        .ownershipKey = 7, .logicalReleased = true, .freeSupportIndicatorActive = true,
    });
    ok &= expectTrue("hover consumes release without a delayed detach", release.retained && release.blockedBySupportHover);
    release = resolvePrimaryReleaseIntent(releaseIntent, { .ownershipKey = 7 });
    ok &= expectTrue("leaving support hover cannot replay the refused release", release.retained);
    release = resolvePrimaryReleaseIntent(releaseIntent, {
        .ownershipKey = 7, .logicalReleased = true, .supportGripActive = true,
    });
    ok &= expectFalse("a new release with acquired support permits handoff", release.retained);
    release = resolvePrimaryReleaseIntent(releaseIntent, { .ownershipKey = 8 });
    ok &= expectTrue("weapon identity change clears a pending release", release.retained);

    RuntimeState manualState{};
    auto manualDecision = update(manualState,
        Input{
            .weaponEquipped = true,
            .ownershipKey = 10,
            .startRequested = false,
            .primaryGripRetained = false,
            .supportGripRetained = false,
        });
    ok &= expectFalse("native equip alone does not start manual ownership", manualDecision.active);
    ok &= expectFalse("native equip alone does not request drop", manualDecision.dropRequested);

    manualDecision = update(manualState,
        Input{
            .weaponEquipped = true,
            .ownershipKey = 10,
            .startRequested = true,
            .primaryGripRetained = true,
            .supportGripRetained = false,
        });
    ok &= expectTrue("first retained grip starts manual ownership", manualDecision.started);
    ok &= expectTrue("manual ownership remains active while primary grip retained", manualDecision.active);

    manualDecision = update(manualState,
        Input{
            .weaponEquipped = true,
            .ownershipKey = 10,
            .startRequested = false,
            .primaryGripRetained = false,
            .supportGripRetained = false,
        });
    ok &= expectTrue("manual ownership requests drop when all grips release", manualDecision.dropRequested);
    ok &= expectFalse("drop request clears manual ownership state", manualState.active);

    manualDecision = update(manualState,
        Input{
            .weaponEquipped = true,
            .ownershipKey = 11,
            .startRequested = true,
            .primaryGripRetained = false,
            .supportGripRetained = true,
        });
    ok &= expectTrue("support grip can start manual ownership", manualDecision.started);
    manualDecision = update(manualState,
        Input{
            .weaponEquipped = true,
            .ownershipKey = 12,
            .startRequested = false,
            .primaryGripRetained = false,
            .supportGripRetained = false,
        });
    ok &= expectTrue("equipped instance change clears manual ownership", manualDecision.cleared);
    ok &= expectFalse("equipped instance change does not drop the newly equipped weapon", manualDecision.dropRequested);

    using namespace rock::equipped_weapon_drop_policy;
    static_assert(canStartAutoDrop(true, false, false));
    static_assert(canStartAutoDrop(false, true, false));
    static_assert(!canStartAutoDrop(true, true, false));
    static_assert(!canStartAutoDrop(false, false, false));
    static_assert(!canStartAutoDrop(true, false, true));
    static_assert(!canStartAutoDrop(false, true, true));
    ok &= expectTrue("ROCK shoulder stash is available without realistic detach",
        equippedWeaponShoulderStashAvailable(true));
    ok &= expectFalse("ROCK shoulder stash setting remains authoritative",
        equippedWeaponShoulderStashAvailable(false));
    ok &= expectEqual("primary-only carry stashes from the firing hand",
        resolveEquippedWeaponStashCarryHand(true, false, false, false, false),
        SourceHand::Right);
    ok &= expectEqual("primary-only carry follows a left firing hand",
        resolveEquippedWeaponStashCarryHand(true, false, false, false, true),
        SourceHand::Left);
    ok &= expectEqual("part carry with only the left grip stashes from the left hand",
        resolveEquippedWeaponStashCarryHand(false, true, true, false, false),
        SourceHand::Left);
    ok &= expectEqual("part carry with only the right grip stashes from the right hand",
        resolveEquippedWeaponStashCarryHand(false, true, false, true, false),
        SourceHand::Right);
    ok &= expectTrue("ordinary release without a stash commit routes to physical drop",
        shouldAttemptPhysicalDrop(false));
    ok &= expectFalse("selected stash never falls through to physical drop",
        shouldAttemptPhysicalDrop(true));
    ok &= expectTrue("successful physical drop commits collider retirement",
        physicalDropCommitted(PhysicalDropCommitInput{ .dropSucceeded = true }));
    ok &= expectTrue("unresolved dropped reference still commits collider retirement",
        physicalDropCommitted(PhysicalDropCommitInput{ .droppedReferenceUnavailable = true }));
    ok &= expectFalse("failed physical drop preserves equipped collision",
        physicalDropCommitted(PhysicalDropCommitInput{}));
    ok &= expectEqual("part carry with both grips has no stash carry hand",
        resolveEquippedWeaponStashCarryHand(false, true, true, true, false),
        SourceHand::None);
    ok &= expectEqual("inactive grip states have no stash carry hand",
        resolveEquippedWeaponStashCarryHand(false, false, false, false, false),
        SourceHand::None);

    ok &= testPartGripReporting();

    ok &= testPartStructureIdentity();

    ok &= testAccessoryClassification();

    using namespace rock::weapon_part_runtime;
    std::array<Target, 3> weaponPartTargets{};
    weaponPartTargets[0].active = true;
    weaponPartTargets[0].ownerToken = 10;
    weaponPartTargets[0].weaponGenerationKey = 0xABC;
    weaponPartTargets[0].flags = MatchBodyId;
    weaponPartTargets[0].grabMode = GrabMode::AttachOnly;
    weaponPartTargets[0].bodyId = 42;
    weaponPartTargets[0].priority = 1;
    weaponPartTargets[0].groupId = 7;
    weaponPartTargets[1].active = true;
    weaponPartTargets[1].ownerToken = 11;
    weaponPartTargets[1].weaponGenerationKey = 0xABC;
    weaponPartTargets[1].flags = MatchBodyId;
    weaponPartTargets[1].grabMode = GrabMode::FullTwoHandAuthority;
    weaponPartTargets[1].bodyId = 42;
    weaponPartTargets[1].priority = 2;
    weaponPartTargets[2].active = true;
    weaponPartTargets[2].ownerToken = 12;
    weaponPartTargets[2].weaponGenerationKey = 0xABC;
    weaponPartTargets[2].flags = MatchSourceName;
    weaponPartTargets[2].grabMode = GrabMode::AttachOnly;
    std::memcpy(weaponPartTargets[2].sourceName.data(), "BoltNode", 8);
    weaponPartTargets[2].priority = 3;

    const auto unmatchedWhitelist = resolveTarget(weaponPartTargets,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 100,
            .sourceRoot = 0x900,
            .sourceName = "Receiver",
        });
    ok &= expectTrue("weapon part whitelist becomes active for matching generation", unmatchedWhitelist.whitelistActive);
    ok &= expectFalse("weapon part whitelist fails closed for unregistered contact", unmatchedWhitelist.matched);

    const auto matchedBody = resolveTarget(weaponPartTargets,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 42,
            .sourceRoot = 0x900,
            .sourceName = "Receiver",
        });
    ok &= expectTrue("weapon part body target matches", matchedBody.matched);
    ok &= expectEqual("higher priority matching target selects full authority",
        matchedBody.grabMode,
        GrabMode::FullTwoHandAuthority);
    ok &= expectEqual("matching target owner is preserved",
        matchedBody.ownerToken,
        static_cast<std::uint64_t>(11));

    const auto matchedName = resolveTarget(weaponPartTargets,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 100,
            .sourceRoot = 0x900,
            .sourceName = "BoltNode",
        });
    ok &= expectTrue("weapon part source-name target matches", matchedName.matched);
    ok &= expectEqual("source-name target selects attach-only mode",
        matchedName.grabMode,
        GrabMode::AttachOnly);

    const auto otherGeneration = resolveTarget(weaponPartTargets,
        Contact{
            .weaponGenerationKey = 0xDEF,
            .bodyId = 100,
            .sourceRoot = 0x900,
            .sourceName = "Receiver",
        });
    ok &= expectFalse("weapon part whitelist does not apply to other generation", otherGeneration.whitelistActive);

    std::array<Target, 1> strictWeaponPartTarget{};
    strictWeaponPartTarget[0].active = true;
    strictWeaponPartTarget[0].ownerToken = 20;
    strictWeaponPartTarget[0].weaponGenerationKey = 0xABC;
    strictWeaponPartTarget[0].flags = MatchBodyId | MatchSourceName | MatchPartKind;
    strictWeaponPartTarget[0].grabMode = GrabMode::AttachOnly;
    strictWeaponPartTarget[0].bodyId = 42;
    strictWeaponPartTarget[0].partKind = rock::WeaponPartKind::Bolt;
    std::memcpy(strictWeaponPartTarget[0].sourceName.data(), "BoltNode", 8);

    const auto strictMatched = resolveTarget(strictWeaponPartTarget,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 42,
            .sourceName = "BoltNode",
            .partKind = rock::WeaponPartKind::Bolt,
        });
    ok &= expectTrue("weapon part target requires and accepts all requested match fields", strictMatched.matched);

    const auto strictWrongName = resolveTarget(strictWeaponPartTarget,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 42,
            .sourceName = "Receiver",
            .partKind = rock::WeaponPartKind::Bolt,
        });
    ok &= expectFalse("weapon part target rejects partial match with wrong source name", strictWrongName.matched);

    const auto strictWrongPart = resolveTarget(strictWeaponPartTarget,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 42,
            .sourceName = "BoltNode",
            .partKind = rock::WeaponPartKind::Receiver,
        });
    ok &= expectFalse("weapon part target rejects partial match with wrong semantic part", strictWrongPart.matched);

    // Non-exclusive whitelist targets grant grab modes without activating
    // whitelist gating for everything else.
    std::array<Target, 2> mixedExclusivityTargets{};
    mixedExclusivityTargets[0].active = true;
    mixedExclusivityTargets[0].ownerToken = 30;
    mixedExclusivityTargets[0].flags = MatchActionRole | NonExclusive;
    mixedExclusivityTargets[0].grabMode = GrabMode::AttachOnly;
    mixedExclusivityTargets[0].actionRole = rock::WeaponActionRole::Bolt;

    const auto nonExclusiveBolt = resolveTarget(mixedExclusivityTargets,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 42,
            .sourceName = "BoltNode",
            .actionRole = rock::WeaponActionRole::Bolt,
        });
    ok &= expectTrue("non-exclusive bolt target matches bolt contact", nonExclusiveBolt.matched);
    ok &= expectEqual("non-exclusive bolt target grants attach-only", nonExclusiveBolt.grabMode, GrabMode::AttachOnly);
    ok &= expectFalse("non-exclusive target does not activate whitelist gating", nonExclusiveBolt.whitelistActive);

    const auto nonExclusiveMiss = resolveTarget(mixedExclusivityTargets,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 43,
            .sourceName = "Receiver",
        });
    ok &= expectFalse("non-bolt contact stays unmatched under non-exclusive target", nonExclusiveMiss.matched);
    ok &= expectFalse("non-bolt contact is not whitelist-gated by non-exclusive target", nonExclusiveMiss.whitelistActive);

    mixedExclusivityTargets[1].active = true;
    mixedExclusivityTargets[1].ownerToken = 31;
    mixedExclusivityTargets[1].flags = MatchBodyId;
    mixedExclusivityTargets[1].grabMode = GrabMode::FullTwoHandAuthority;
    mixedExclusivityTargets[1].bodyId = 77;
    const auto mixedUnmatched = resolveTarget(mixedExclusivityTargets,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 43,
            .sourceName = "Receiver",
        });
    ok &= expectTrue("exclusive target still activates whitelist gating alongside non-exclusive", mixedUnmatched.whitelistActive);
    ok &= expectFalse("mixed whitelist still fails closed for unmatched contact", mixedUnmatched.matched);
    {
        const Contact bolt{ .weaponGenerationKey = 0xABC,
            .bodyId = 42, .actionRole = rock::WeaponActionRole::Bolt };
        auto targets = mixedExclusivityTargets;
        targets[1].bodyId = 42;
        targets[1].priority = 10;
        ok &= expectEqual("marker snapshot respects full-authority target priority",
            resolveTarget(std::span<const Target>(targets), bolt).grabMode,
            GrabMode::FullTwoHandAuthority);
        ok &= expectEqual("a cleared target outside the snapshot count cannot suppress an attach-only marker",
            resolveTarget(std::span<const Target>(targets).first(1), bolt).grabMode,
            GrabMode::AttachOnly);
        ok &= expectFalse("clearing all provider targets removes marker eligibility",
            resolveTarget(std::span<const Target>(targets).first(0), bolt).matched);
        targets[0].weaponGenerationKey = 0xDEF;
        ok &= expectFalse("a stale weapon generation cannot retain an attach-only marker",
            resolveTarget(std::span<const Target>(targets).first(1), bolt).matched);
    }

    std::array<Target, 1> semanticsOnlyTarget{};
    semanticsOnlyTarget[0].active = true;
    semanticsOnlyTarget[0].ownerToken = 32;
    semanticsOnlyTarget[0].flags = NonExclusive;
    semanticsOnlyTarget[0].grabMode = GrabMode::AttachOnly;
    const auto semanticsOnly = resolveTarget(semanticsOnlyTarget,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 42,
            .sourceName = "BoltNode",
        });
    ok &= expectFalse("NonExclusive without a matcher is unusable", semanticsOnly.matched);
    ok &= expectFalse("NonExclusive without a matcher activates nothing", semanticsOnly.whitelistActive);

    {
        using namespace rock::weapon_interaction_acquisition_policy;
        State acquisitionState{};
        ok &= expectEqual("legacy-palm overlap publishes touch acquisition provenance",
            resolve(acquisitionState, true, true),
            rock::WeaponInteractionAcquisitionSource::PhysicalContact);
        ok &= expectEqual("first overlap-gap frame retains touch provenance",
            resolve(acquisitionState, false, true),
            rock::WeaponInteractionAcquisitionSource::PhysicalContact);
        ok &= expectEqual("second overlap-gap frame retains touch provenance",
            resolve(acquisitionState, false, true),
            rock::WeaponInteractionAcquisitionSource::PhysicalContact);
        ok &= expectEqual("probe provenance begins after the bounded touch lease",
            resolve(acquisitionState, false, true),
            rock::WeaponInteractionAcquisitionSource::ProximityProbe);
        ok &= expectEqual("provenance never manufactures a contact candidate",
            resolve(acquisitionState, false, false),
            rock::WeaponInteractionAcquisitionSource::None);
    }

    rock::collision_suppression_registry::DelayedRestoreTimer postDropRestore{};
    ok &= expectTrue("post-drop suppression uses grab release delay seconds", postDropRestore.begin(42, 1, 0.8f));
    ok &= expectFalse("post-drop suppression remains active before configured delay", postDropRestore.advance(true, 0.79f));
    ok &= expectTrue("post-drop suppression expires at configured delay", postDropRestore.advance(true, 0.01f));

    {
        using namespace rock::weapon_collision_geometry_math;
        ok &= expectFalse("small unscaled source-local hull stays below the build threshold",
            scaledHullDiagonalCanBuild(0.04f, 1.0f, 0.5f));
        ok &= expectTrue("authored node scale participates in source-local hull validation",
            scaledHullDiagonalCanBuild(0.04f, 78.0f, 0.5f));
        ok &= expectFalse("zero-scale source-local hull fails closed",
            scaledHullDiagonalCanBuild(4.0f, 0.0f, 0.5f));

        std::vector<HullSelectionInput> balancedInputs{
            { .center = { 0.0f, -100.0f, 0.0f }, .min = { -2.0f, -102.0f, -2.0f }, .max = { 2.0f, -98.0f, 2.0f }, .pointCount = 40, .coverageClass = 1, .priority = 62 },
            { .center = { 0.0f, 100.0f, 0.0f }, .min = { -2.0f, 98.0f, -2.0f }, .max = { 2.0f, 102.0f, 2.0f }, .pointCount = 40, .coverageClass = 1, .priority = 62 },
            { .center = { 0.0f, 0.0f, 0.0f }, .min = { -1.0f, -1.0f, -1.0f }, .max = { 1.0f, 1.0f, 1.0f }, .pointCount = 20, .coverageClass = 3, .priority = 56 },
            { .center = { 0.0f, 0.0f, 0.0f }, .min = { -1.0f, -1.0f, -1.0f }, .max = { 1.0f, 1.0f, 1.0f }, .pointCount = 20, .coverageClass = 3, .priority = 80 },
            { .center = { 0.0f, 0.0f, 0.0f }, .min = { -1.0f, -1.0f, -1.0f }, .max = { 1.0f, 1.0f, 1.0f }, .pointCount = 20, .coverageClass = 7, .priority = 12, .cosmetic = true },
        };
        const auto balancedSelection = selectBalancedHullIndices(balancedInputs, 3);
        ok &= expectTrue("balanced hull selection keeps the higher-priority magazine shell",
            std::find(balancedSelection.begin(), balancedSelection.end(), 3) != balancedSelection.end());
        ok &= expectFalse("balanced hull selection drops the lower-priority same-class candidate first",
            std::find(balancedSelection.begin(), balancedSelection.end(), 2) != balancedSelection.end());
        ok &= expectFalse("balanced hull selection does not spend structural capacity on cosmetic ammunition",
            std::find(balancedSelection.begin(), balancedSelection.end(), 4) != balancedSelection.end());

        const std::vector<DetachedComponentInput> basReloadMagazineInputs{
            { .min = { -8.0f, -4.0f, -6.0f }, .max = { 8.0f, 4.0f, 6.0f }, .assembledAnchor = true },
            { .min = { -3.0f, -6.0f, -12.0f }, .max = { 3.0f, 1.0f, -4.0f } },
            { .min = { -4.0f, -28.0f, -122.0f }, .max = { 4.0f, -14.0f, -108.0f } },
            { .min = { -2.0f, -25.0f, -116.0f }, .max = { 2.0f, -18.0f, -110.0f } },
        };
        const auto basReloadMagazineFilter = findDetachedSourceComponentIndices(basReloadMagazineInputs, 2.0f, 24.0f);
        ok &= expectEqual("remote reload magazine component is rejected before collider budgeting",
            basReloadMagazineFilter.verdict,
            DetachedComponentVerdict::Filtered);
        ok &= expectEqual("remote reload magazine component rejects every member", basReloadMagazineFilter.excludedIndices.size(), std::size_t{ 2 });
        ok &= expectTrue("remote reload magazine shell is rejected",
            std::find(basReloadMagazineFilter.excludedIndices.begin(), basReloadMagazineFilter.excludedIndices.end(), 2) !=
                basReloadMagazineFilter.excludedIndices.end());
        ok &= expectTrue("remote reload magazine ammunition is rejected",
            std::find(basReloadMagazineFilter.excludedIndices.begin(), basReloadMagazineFilter.excludedIndices.end(), 3) !=
                basReloadMagazineFilter.excludedIndices.end());
        ok &= expectTrue("remote reload magazine has a measured empty separation",
            basReloadMagazineFilter.minimumExcludedGap >= 24.0f);

        auto nearbyMagazineInputs = basReloadMagazineInputs;
        nearbyMagazineInputs[2].min = { -3.0f, -5.0f, -27.0f };
        nearbyMagazineInputs[2].max = { 3.0f, 1.0f, -19.0f };
        nearbyMagazineInputs[3].min = { -2.0f, -4.0f, -25.0f };
        nearbyMagazineInputs[3].max = { 2.0f, 0.0f, -21.0f };
        const auto nearbyMagazineFilter = findDetachedSourceComponentIndices(nearbyMagazineInputs, 2.0f, 24.0f);
        ok &= expectTrue("nearby disconnected magazine fails open for authored mesh gaps", nearbyMagazineFilter.excludedIndices.empty());

        auto distantStructuralInputs = basReloadMagazineInputs;
        distantStructuralInputs[2].assembledAnchor = true;
        const auto distantStructuralFilter = findDetachedSourceComponentIndices(distantStructuralInputs, 2.0f, 24.0f);
        ok &= expectTrue("distant structural weapon geometry is never rejected by origin distance", distantStructuralFilter.excludedIndices.empty());

        auto coherentAuthoredSourceInputs = basReloadMagazineInputs;
        coherentAuthoredSourceInputs[0].coherenceGroup = 17;
        coherentAuthoredSourceInputs[2].coherenceGroup = 17;
        const auto coherentAuthoredSourceFilter = findDetachedSourceComponentIndices(coherentAuthoredSourceInputs, 2.0f, 24.0f);
        ok &= expectTrue("spatial policy does not split hulls from one authored source", coherentAuthoredSourceFilter.excludedIndices.empty());

        auto ambiguousInputs = basReloadMagazineInputs;
        ambiguousInputs[0].assembledAnchor = false;
        const auto ambiguousFilter = findDetachedSourceComponentIndices(ambiguousInputs, 2.0f, 24.0f);
        ok &= expectEqual("inventory without assembled anchor evidence fails open",
            ambiguousFilter.verdict,
            DetachedComponentVerdict::FailOpenNoAssembledAnchor);
        ok &= expectTrue("ambiguous inventory preserves all sources", ambiguousFilter.excludedIndices.empty());
    }

    return ok ? 0 : 1;
}
