#include "physics-interaction/weapon/DynamicWeaponCollisionPolicy.h"
#include "physics-interaction/visual/HandWorldClaimRegistryPolicy.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

namespace
{
    constexpr float kEpsilon = 0.001f;

    bool expectNear(const char* label, float actual, float expected, float epsilon = kEpsilon)
    {
        if (std::fabs(actual - expected) <= epsilon) {
            return true;
        }
        std::printf("%s expected %.5f got %.5f\n", label, expected, actual);
        return false;
    }

    bool expectPoint(const char* label, const RE::NiPoint3& actual, const RE::NiPoint3& expected)
    {
        bool ok = true;
        ok &= expectNear((std::string(label) + ".x").c_str(), actual.x, expected.x);
        ok &= expectNear((std::string(label) + ".y").c_str(), actual.y, expected.y);
        ok &= expectNear((std::string(label) + ".z").c_str(), actual.z, expected.z);
        return ok;
    }

    RE::NiMatrix3 rotationZ90()
    {
        RE::NiMatrix3 result{};
        result.entry[0][0] = 0.0f;
        result.entry[0][1] = 1.0f;
        result.entry[0][2] = 0.0f;
        result.entry[1][0] = -1.0f;
        result.entry[1][1] = 0.0f;
        result.entry[1][2] = 0.0f;
        result.entry[2][0] = 0.0f;
        result.entry[2][1] = 0.0f;
        result.entry[2][2] = 1.0f;
        return result;
    }
}

int main()
{
    using namespace rock::dynamic_weapon_collision_policy;
    bool ok = true;

    const auto freeRecovery = resolveContactMotorRecovery(0.8f, 1.0f, 0.03f, 0.01f, 0.03f, false);
    ok &= expectNear("free aim preserves damping", freeRecovery.damping, 0.8f);
    ok &= expectNear("free aim preserves recovery", freeRecovery.constantRecoveryVelocity, 1.0f);
    const auto blockedRecovery = resolveContactMotorRecovery(0.8f, 1.0f, 0.03f, 0.01f, 0.01f, true);
    ok &= expectNear("blocked weapon damps relative velocity", blockedRecovery.damping, 1.0f);
    ok &= expectNear("blocked weapon has no constant recovery kick", blockedRecovery.constantRecoveryVelocity, 0.0f, 0.0f);
    const auto enteringRecovery = resolveContactMotorRecovery(0.8f, 1.0f, 0.03f, 0.01f, 0.02f, true);
    const auto leavingRecovery = resolveContactMotorRecovery(0.8f, 1.0f, 0.03f, 0.01f, 0.02f, false);
    ok &= expectNear("contact entry blends damping", enteringRecovery.damping, 0.9f);
    ok &= expectNear("contact entry blends recovery", enteringRecovery.constantRecoveryVelocity, 0.5f);
    ok &= expectNear("release does not snap damping", leavingRecovery.damping, enteringRecovery.damping);
    ok &= expectNear("release does not snap recovery", leavingRecovery.constantRecoveryVelocity, enteringRecovery.constantRecoveryVelocity);
    const auto tunedRecovery = resolveContactMotorRecovery(1.2f, 0.4f, 0.03f, 0.01f, 0.01f, true);
    ok &= expectNear("stronger supplied damping is preserved", tunedRecovery.damping, 1.2f);
    const auto equalTauContact = resolveContactMotorRecovery(0.8f, 1.0f, 0.03f, 0.03f, 0.03f, true);
    const auto equalTauFree = resolveContactMotorRecovery(0.8f, 1.0f, 0.03f, 0.03f, 0.03f, false);
    ok &= expectNear("equal tau still admits contact recovery", equalTauContact.constantRecoveryVelocity, 0.0f);
    ok &= expectNear("equal tau restores free recovery", equalTauFree.constantRecoveryVelocity, 1.0f);

    // Fixed target, small angular error: the verified native position motor
    // requests min(error/dt, proportional*error + constant) recovery speed.
    // Contact must approach zero proportionally instead of closing each tiny
    // lever disturbance in one step. Free aim keeps its measured old response.
    for (const float hz : {60.0f, 90.0f, 180.0f, 270.0f}) {
        constexpr float angularError = 0.001f;
        constexpr float proportionalRecovery = 2.0f;
        const float freeSpeed = (std::min)(angularError * hz,
            proportionalRecovery * angularError + freeRecovery.constantRecoveryVelocity);
        const float contactSpeed = (std::min)(angularError * hz,
            proportionalRecovery * angularError + blockedRecovery.constantRecoveryVelocity);
        ok &= expectNear("free angular correction remains responsive", freeSpeed, angularError * hz);
        ok &= expectNear("contact recovery is proportional across step rates", contactSpeed, 0.002f);
    }

    const auto postSolveTiming = [](float deltaSeconds) {
        auto timing = rock::havok_physics_timing::makeTimingSample(
            deltaSeconds, deltaSeconds, 0.0f, deltaSeconds, 1);
        timing.phase = rock::havok_physics_timing::PhysicsStepPhase::SubstepPostSolve;
        return timing;
    };
    // Losing callbacks for the same elapsed time must produce the same
    // motor contact state, regardless of render rate or substep count.
    for (const int hz : {60, 90, 120, 180, 270}) {
        const auto timing = postSolveTiming(1.0f / static_cast<float>(hz));
        float retained = advanceContactRetention(0.0f, true, false, timing);
        const int solvesUntilExpiry = hz / 30;
        for (int solve = 1; solve <= solvesUntilExpiry; ++solve) {
            retained = advanceContactRetention(retained, false, false, timing);
            if ((retained > 0.0f) != (solve < solvesUntilExpiry)) {
                std::printf("weapon contact expiry mismatch hz=%d solve=%d remaining=%.8f\n", hz, solve, retained);
                ok = false;
            }
        }
    }
    const auto timing90 = postSolveTiming(1.0f / 90.0f);
    float retained = advanceContactRetention(0.0f, true, false, timing90);
    retained = advanceContactRetention(retained, false, false, timing90);
    ok &= expectNear("fresh callback renews measured retention",
        advanceContactRetention(retained, true, false, timing90), kContactRetentionSeconds);
    for (int solve = 0; solve < 2; ++solve) {
        retained = advanceContactRetention(retained, false, false, postSolveTiming(1.0f / 180.0f));
    }
    for (int solve = 0; solve < 3; ++solve) {
        retained = advanceContactRetention(retained, false, false, postSolveTiming(1.0f / 270.0f));
    }
    ok &= expectNear("rate changes preserve the contact deadline", retained, 0.0f, 0.0f);
    ok &= expectNear("teleport clears even fresh contact",
        advanceContactRetention(kContactRetentionSeconds, true, true, timing90), 0.0f, 0.0f);
    ok &= expectNear("long measured step expires contact",
        advanceContactRetention(kContactRetentionSeconds, false, false, postSolveTiming(0.1f)), 0.0f, 0.0f);
    auto invalidTiming = timing90;
    invalidTiming.usedFallback = true;
    ok &= expectNear("fallback timing cannot retain contact",
        advanceContactRetention(kContactRetentionSeconds, true, false, invalidTiming), 0.0f, 0.0f);
    invalidTiming = timing90;
    invalidTiming.valid = false;
    ok &= expectNear("invalid timing cannot retain contact",
        advanceContactRetention(kContactRetentionSeconds, true, false, invalidTiming), 0.0f, 0.0f);
    invalidTiming = timing90;
    invalidTiming.phase = rock::havok_physics_timing::PhysicsStepPhase::SubstepPreCollide;
    ok &= expectNear("unsolved callback cannot refresh contact",
        advanceContactRetention(kContactRetentionSeconds, true, false, invalidTiming), 0.0f, 0.0f);
    invalidTiming = timing90;
    invalidTiming.substepDeltaSeconds = (std::numeric_limits<float>::quiet_NaN)();
    ok &= expectNear("unmeasured delta cannot retain contact",
        advanceContactRetention(kContactRetentionSeconds, true, false, invalidTiming), 0.0f, 0.0f);

    const auto belowDivergence = advanceDivergenceDwell(
        0.2f,
        kDivergenceTeleportDistanceGameUnits,
        0.1f);
    ok &= expectNear("weapon divergence resets at threshold", belowDivergence.elapsedSeconds, 0.0f);
    ok &= !belowDivergence.recoverNow;

    const auto accumulatingDivergence = advanceDivergenceDwell(
        0.1f,
        kDivergenceTeleportDistanceGameUnits + 1.0f,
        0.1f);
    ok &= expectNear("weapon divergence accumulates measured dwell", accumulatingDivergence.elapsedSeconds, 0.2f);
    ok &= !accumulatingDivergence.recoverNow;

    const auto persistentDivergence = advanceDivergenceDwell(
        accumulatingDivergence.elapsedSeconds,
        kDivergenceTeleportDistanceGameUnits + 1.0f,
        0.1f);
    ok &= expectNear("weapon divergence reaches recovery dwell", persistentDivergence.elapsedSeconds, 0.3f);
    ok &= persistentDivergence.recoverNow;

    constexpr auto dynamicWeaponMask = rock::collision_layer_policy::buildRockDynamicWeaponProxyExpectedMask();
    for (std::uint32_t layer = 0; layer < rock::collision_layer_policy::FO4_LAYER_MATRIX_ADDRESSABLE_COUNT; ++layer) {
        const bool enabled = rock::collision_layer_policy::maskEnablesLayer(dynamicWeaponMask, layer);
        const bool expected = rock::collision_layer_policy::isDynamicWeaponProxySolverObstacleLayer(layer);
        if (enabled != expected) {
            std::printf("dynamic weapon layer mismatch at row %u expected=%d actual=%d\n", layer, expected ? 1 : 0, enabled ? 1 : 0);
            ok = false;
        }
    }
    ok &= !rock::collision_layer_policy::maskEnablesLayer(
        dynamicWeaponMask,
        rock::collision_layer_policy::FO4_LAYER_CLUTTER);
    ok &= !rock::collision_layer_policy::maskEnablesLayer(
        dynamicWeaponMask,
        rock::collision_layer_policy::FO4_LAYER_CLUTTER_LARGE);
    ok &= rock::collision_layer_policy::maskEnablesLayer(
        dynamicWeaponMask,
        rock::collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER);
    ok &= rock::collision_layer_policy::maskEnablesLayer(
        dynamicWeaponMask,
        rock::collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER);

    constexpr auto rightHandMask =
        rock::collision_layer_policy::buildRockDynamicHandProxyExpectedMask(
            false);
    constexpr auto leftHandMask =
        rock::collision_layer_policy::buildRockDynamicHandProxyExpectedMask(
            true);
    ok &= rock::collision_layer_policy::maskEnablesLayer(
        rightHandMask,
        rock::collision_layer_policy::ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY);
    ok &= rock::collision_layer_policy::maskEnablesLayer(
        rightHandMask,
        rock::collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY);
    ok &= !rock::collision_layer_policy::maskEnablesLayer(
        rightHandMask,
        rock::collision_layer_policy::ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY);
    ok &= rock::collision_layer_policy::maskEnablesLayer(
        leftHandMask,
        rock::collision_layer_policy::ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY);
    ok &= rock::collision_layer_policy::maskEnablesLayer(
        leftHandMask,
        rock::collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY);
    ok &= !rock::collision_layer_policy::maskEnablesLayer(
        leftHandMask,
        rock::collision_layer_policy::ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY);
    ok &= rock::collision_layer_policy::maskEnablesLayer(
        dynamicWeaponMask,
        rock::collision_layer_policy::ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY);
    ok &= rock::collision_layer_policy::maskEnablesLayer(
        dynamicWeaponMask,
        rock::collision_layer_policy::ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY);

    const auto geometry = makeBoundingBoxGeometry(
        RE::NiPoint3{ -10.0f, -2.0f, -1.0f },
        RE::NiPoint3{ 30.0f, 4.0f, 3.0f });
    ok &= geometry.valid;
    ok &= expectPoint("box center", geometry.centerWeaponLocal, RE::NiPoint3{ 10.0f, 1.0f, 1.0f });
    ok &= expectPoint("box half extents", geometry.halfExtentsWeaponLocal, RE::NiPoint3{ 20.0f, 3.0f, 2.0f });

    const auto envelopeHalfExtents = makeBoundingBoxHalfExtentsHavok(geometry, 2.0f, 1.0f, 0.1f);
    ok &= expectPoint("scaled padded Havok half extents", envelopeHalfExtents, RE::NiPoint3{ 4.1f, 0.7f, 0.5f });

    const auto envelopeMassProperties = makeBoundingBoxMassProperties(geometry, 2.0f, 1.0f, 0.1f, 10.0f);
    ok &= envelopeMassProperties.valid;
    ok &= expectPoint("bounding envelope mass half extents", envelopeMassProperties.halfExtentsHavok, envelopeHalfExtents);
    ok &= expectPoint(
        "bounding envelope inverse principal inertia",
        envelopeMassProperties.inverseInertia,
        RE::NiPoint3{ 0.405405f, 0.017585f, 0.017341f });
    ok &= expectNear("bounding envelope inverse mass", envelopeMassProperties.inverseMass, 0.1f);
    ok &= !makeBoundingBoxMassProperties(geometry, 2.0f, 1.0f, 0.1f, 0.0f).valid;
    ok &= !makeBoundingBoxMassProperties(geometry, 0.0f, 1.0f, 0.1f, 10.0f).valid;

    const auto compoundFrame = makeCompoundChildFrame(
        RE::NiPoint3{ -5.0f, 2.0f, 3.0f },
        geometry.centerWeaponLocal,
        2.0f,
        0.1f);
    ok &= compoundFrame.valid;
    ok &= expectNear("compound point scale", compoundFrame.pointScaleHavok, 0.2f);
    ok &= expectPoint("compound child translation", compoundFrame.translationHavok, RE::NiPoint3{ -3.0f, 0.2f, 0.4f });

    ok &= !makeCompoundChildFrame(
               RE::NiPoint3{},
               RE::NiPoint3{},
               0.0f,
               0.1f)
               .valid;

    ok &= expectNear("invalid weapon mass falls back", sanitizeWeaponMass((std::numeric_limits<float>::quiet_NaN)()), 2.0f);
    ok &= expectNear("zero weapon mass falls back", sanitizeWeaponMass(0.0f), 2.0f);
    ok &= expectNear("tiny weapon mass stays physical", sanitizeWeaponMass(0.05f), 0.1f);
    ok &= expectNear("authored weapon mass is preserved", sanitizeWeaponMass(12.5f), 12.5f);
    ok &= expectNear("weapon mass matches loose-grab ceiling", sanitizeWeaponMass(75.0f), 50.0f);

    RE::NiTransform weaponRoot = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    weaponRoot.rotate = rotationZ90();
    weaponRoot.translate = RE::NiPoint3{ 100.0f, 50.0f, -25.0f };
    weaponRoot.scale = 2.0f;
    const RE::NiTransform bodyTarget = makeProxyBodyTarget(weaponRoot, geometry.centerWeaponLocal);
    const RE::NiTransform gripAuthorityTarget = makeGripAuthorityTarget(weaponRoot);
    const RE::NiTransform bodyTargetFromGrip = makeContactBodyTargetFromGripAuthority(
        gripAuthorityTarget,
        geometry.centerWeaponLocal,
        weaponRoot.scale);
    const RE::NiTransform bodyInGripAuthority = makeContactBodyInGripAuthoritySpace(
        geometry.centerWeaponLocal,
        weaponRoot.scale);
    const RE::NiTransform reconstructed = reconstructWeaponRoot(bodyTarget, geometry.centerWeaponLocal, weaponRoot.scale);
    ok &= expectPoint("body center target", bodyTarget.translate, RE::NiPoint3{ 98.0f, 70.0f, -23.0f });
    ok &= expectPoint("grip authority stays at weapon root", gripAuthorityTarget.translate, weaponRoot.translate);
    ok &= expectPoint("grip authority reconstructs contact center", bodyTargetFromGrip.translate, bodyTarget.translate);
    ok &= expectNear("grip authority reconstructs contact rotation", rotationDeltaDegrees(bodyTargetFromGrip, bodyTarget), 0.0f, 0.05f);
    ok &= expectPoint("grip authority relation preserves center offset", bodyInGripAuthority.translate, RE::NiPoint3{ 20.0f, 2.0f, 2.0f });
    ok &= expectNear(
        "grip authority relation has identity rotation",
        rotationDeltaDegrees(bodyInGripAuthority, rock::transform_math::makeIdentityTransform<RE::NiTransform>()),
        0.0f,
        0.05f);
    ok &= expectPoint("reconstructed root", reconstructed.translate, weaponRoot.translate);
    ok &= expectNear("reconstructed rotation", rotationDeltaDegrees(reconstructed, weaponRoot), 0.0f, 0.05f);

    RE::NiTransform rotatedLiveBody = bodyTarget;
    rotatedLiveBody.rotate = rock::transform_math::makeIdentityTransform<RE::NiTransform>().rotate;
    const auto gripRecovery = evaluateGripRecovery(
        rotatedLiveBody,
        gripAuthorityTarget,
        geometry.centerWeaponLocal,
        weaponRoot.scale,
        25.0f);
    ok &= expectNear(
        "grip recovery measures reconstructed authority instead of body center",
        gripRecovery.distanceGameUnits,
        std::sqrt(808.0f));
    ok &= gripRecovery.resetNow;
    ok &= !evaluateGripRecovery(
               rotatedLiveBody,
               gripAuthorityTarget,
               geometry.centerWeaponLocal,
               weaponRoot.scale,
               30.0f)
               .resetNow;

    RE::NiTransform sampledRequested = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    sampledRequested.translate = RE::NiPoint3{ 10.0f, 0.0f, 0.0f };
    RE::NiTransform sampledLive = sampledRequested;
    sampledLive.translate = RE::NiPoint3{ 8.0f, 0.0f, 0.0f };
    RE::NiTransform currentRequested = sampledRequested;
    currentRequested.translate = RE::NiPoint3{ 12.0f, 0.0f, 0.0f };
    const RE::NiPoint3 origin{};
    const RE::NiTransform resolvedTranslation = resolveCurrentIntentFromSample(
        makeProxyBodyTarget(sampledRequested, origin),
        makeProxyBodyTarget(sampledLive, origin),
        origin,
        1.0f,
        currentRequested);
    ok &= expectPoint("one-way translation correction preserves new intent delta", resolvedTranslation.translate, RE::NiPoint3{ 10.0f, 0.0f, 0.0f });

    sampledRequested = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    sampledLive = sampledRequested;
    sampledLive.rotate = rotationZ90();
    currentRequested = sampledRequested;
    const RE::NiTransform resolvedRotation = resolveCurrentIntentFromSample(
        makeProxyBodyTarget(sampledRequested, origin),
        makeProxyBodyTarget(sampledLive, origin),
        origin,
        1.0f,
        currentRequested);
    ok &= expectNear("one-way rotation correction", rotationDeltaDegrees(resolvedRotation, sampledLive), 0.0f, 0.05f);

    // Simultaneous locomotion and angular wall deflection must preserve the
    // world movement direction. The old composition turned this +X step +Y.
    currentRequested.translate = RE::NiPoint3{4.0f, 0.0f, 0.0f};
    const auto movingDeflected = resolveCurrentIntentFromSample(
        makeProxyBodyTarget(sampledRequested, origin), makeProxyBodyTarget(sampledLive, origin),
        origin, 1.0f, currentRequested);
    ok &= expectPoint("deflection must not rotate locomotion", movingDeflected.translate, currentRequested.translate);
    ok &= expectNear("locomotion preserves wall deflection", rotationDeltaDegrees(movingDeflected, sampledLive), 0.0f, 0.05f);

    // Turn and walk with a displaced, scaled weapon and an off-centre collider.
    // Its sampled local displacement must rotate with the new target, while
    // the actual deflection remains present for the weapon and both hands.
    sampledRequested = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    sampledRequested.translate = RE::NiPoint3{100.0f, 200.0f, 300.0f};
    sampledRequested.scale = 2.0f;
    sampledLive = sampledRequested;
    sampledLive.translate.x -= 2.0f;
    sampledLive.rotate = rotationZ90();
    currentRequested = sampledRequested;
    currentRequested.translate.x += 4.0f;
    currentRequested.rotate = rotationZ90();
    const RE::NiPoint3 colliderCenter{2.0f, 6.0f, -1.0f};
    const auto turningDeflected = resolveCurrentIntentFromSample(
        makeProxyBodyTarget(sampledRequested, colliderCenter), makeProxyBodyTarget(sampledLive, colliderCenter),
        colliderCenter, sampledRequested.scale, currentRequested);
    ok &= expectPoint("turn carries local wall displacement", turningDeflected.translate, RE::NiPoint3{104.0f, 198.0f, 300.0f});
    ok &= expectNear("turn preserves sampled angular residual", turningDeflected.rotate.entry[0][0], -1.0f);
    ok &= expectNear("transport preserves weapon scale", turningDeflected.scale, 2.0f);
    for (const auto& handLocal : {RE::NiPoint3{0.0f, 0.0f, 0.0f}, RE::NiPoint3{0.0f, 12.0f, 0.0f}}) {
        auto local = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
        local.translate = handLocal;
        const auto requestedHand = rock::transform_math::composeTransforms(currentRequested, local);
        const auto resolvedHand = reframeAttachedHand(currentRequested, turningDeflected, requestedHand);
        const auto recoveredLocal = rock::transform_math::composeTransforms(
            rock::transform_math::invertTransform(turningDeflected), resolvedHand);
        ok &= expectPoint("both attached hand seats survive moving collision correction", recoveredLocal.translate, handLocal);
    }
    const auto unchangedIntent = resolveCurrentIntentFromSample(
        makeProxyBodyTarget(sampledRequested, colliderCenter), makeProxyBodyTarget(sampledLive, colliderCenter),
        colliderCenter, sampledRequested.scale, sampledRequested);
    ok &= expectPoint("stationary wall displacement stays exact", unchangedIntent.translate, sampledLive.translate);
    ok &= expectNear("stationary wall rotation stays exact", rotationDeltaDegrees(unchangedIntent, sampledLive), 0.0f, 0.05f);

    const auto nativeOneHand = selectAttachedHands(false, false, true, false, false);
    struct NativeNode
    {
        NativeNode* parent = nullptr;
        RE::NiTransform local = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
        RE::NiTransform world = local;
    };
    NativeNode nativeHand{}, animatedParent{&nativeHand}, nativeWeapon{&animatedParent};
    animatedParent.local.translate = RE::NiPoint3{1.0f, 2.0f, 0.0f};
    nativeWeapon.local.translate = RE::NiPoint3{0.0f, 3.0f, 0.0f};
    auto physicalDriver = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    RE::NiTransform nativeIntent{};
    float step = 0.0f;
    for (const float residual : {0.04f, 0.06f, 0.03f, 0.0f}) {
        physicalDriver.translate.x = step;
        // The rendered parent can carry any previous correction. Neither it
        // nor a stale descendant world may become the next physical target.
        nativeHand.world = physicalDriver;
        nativeHand.world.translate.y += residual;
        nativeHand.world.rotate = rotationZ90();
        animatedParent.world.translate.x = -500.0f;
        nativeWeapon.world.translate.y = 900.0f;
        ok &= reconstructNativeIntent(&nativeWeapon, &nativeHand, physicalDriver, nativeIntent);
        ok &= expectPoint("native intent ignores correction cutoff crossings", nativeIntent.translate, RE::NiPoint3{step + 1.0f, 5.0f, 0.0f});
        ok &= expectNear("native intent ignores rendered parent deflection", rotationDeltaDegrees(nativeIntent, physicalDriver), 0.0f, 0.05f);
        step += 4.0f;
    }
    nativeWeapon.local.translate.z = 2.0f;
    ok &= reconstructNativeIntent(&nativeWeapon, &nativeHand, physicalDriver, nativeIntent);
    ok &= expectNear("native animation locals remain live", nativeIntent.translate.z, 2.0f);
    nativeWeapon.parent = &nativeHand;
    ok &= reconstructNativeIntent(&nativeWeapon, &nativeHand, physicalDriver, nativeIntent);
    ok &= expectPoint("direct native attachment stays supported", nativeIntent.translate, RE::NiPoint3{12.0f, 3.0f, 2.0f});
    nativeWeapon.parent = nullptr;
    ok &= !reconstructNativeIntent(&nativeWeapon, &nativeHand, physicalDriver, nativeIntent);
    nativeWeapon.parent = &animatedParent;
    animatedParent.parent = &nativeWeapon;
    ok &= !reconstructNativeIntent(&nativeWeapon, &nativeHand, physicalDriver, nativeIntent);
    animatedParent.parent = &nativeHand;
    nativeWeapon.local.translate.x = std::numeric_limits<float>::quiet_NaN();
    ok &= !reconstructNativeIntent(&nativeWeapon, &nativeHand, physicalDriver, nativeIntent);

    auto corrected = physicalDriver;
    for (const float residual : {0.0f, 0.03f, 0.06f, 0.04f, 5.0f}) {
        corrected.translate.y = physicalDriver.translate.y + residual;
        const auto decision = evaluateVisualCorrection(physicalDriver, corrected);
        ok &= decision.apply;
        ok &= expectNear("valid correction retains authority through the old cutoff", decision.translationGameUnits, residual);
    }
    corrected = physicalDriver;
    corrected.rotate = rotationZ90();
    ok &= evaluateVisualCorrection(physicalDriver, corrected).apply;
    corrected.translate.x = std::numeric_limits<float>::quiet_NaN();
    ok &= !evaluateVisualCorrection(physicalDriver, corrected).apply;

    ok &= !nativeOneHand.left && nativeOneHand.right;
    const auto leftPrimaryOnly = selectAttachedHands(false, true, true, false, false);
    ok &= leftPrimaryOnly.left && !leftPrimaryOnly.right;
    const auto rightTwoHand = selectAttachedHands(false, true, false, true, false);
    ok &= rightTwoHand.left && rightTwoHand.right;
    const auto leftTwoHand = selectAttachedHands(false, true, true, false, true);
    ok &= leftTwoHand.left && leftTwoHand.right;
    const auto partCarryBoth = selectAttachedHands(true, false, false, true, true);
    ok &= partCarryBoth.left && partCarryBoth.right;
    const auto partCarryLeftOnly = selectAttachedHands(true, false, true, true, false);
    ok &= partCarryLeftOnly.left && !partCarryLeftOnly.right;

    RE::NiTransform requestedWeapon = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    requestedWeapon.translate = RE::NiPoint3{ 10.0f, 20.0f, 30.0f };
    RE::NiTransform requestedHand = requestedWeapon;
    requestedHand.translate = RE::NiPoint3{ 12.0f, 23.0f, 34.0f };
    RE::NiTransform resolvedWeapon = requestedWeapon;
    resolvedWeapon.rotate = rotationZ90();
    resolvedWeapon.translate = RE::NiPoint3{ 5.0f, 7.0f, 11.0f };
    const RE::NiTransform reframedHand = reframeAttachedHand(
        requestedWeapon,
        resolvedWeapon,
        requestedHand);
    const RE::NiTransform requestedHandWeaponLocal = rock::transform_math::composeTransforms(
        rock::transform_math::invertTransform(requestedWeapon),
        requestedHand);
    const RE::NiTransform reframedHandWeaponLocal = rock::transform_math::composeTransforms(
        rock::transform_math::invertTransform(resolvedWeapon),
        reframedHand);
    ok &= expectPoint(
        "collision-reframed hand keeps weapon-local offset",
        reframedHandWeaponLocal.translate,
        requestedHandWeaponLocal.translate);
    ok &= expectNear(
        "collision-reframed hand keeps weapon-local rotation",
        rotationDeltaDegrees(reframedHandWeaponLocal, requestedHandWeaponLocal),
        0.0f,
        0.05f);

    // A stationary bipod weapon and an advancing controller. Reframing the
    // already-braced animation target would move it in the opposite direction.
    // Use the grip layer as collision input; the animation must keep ownership
    // even when collision clears and re-registers its claim each frame.
    {
        namespace claims = rock::hand_world_claim_registry_policy;
        claims::Registry registry{};
        const auto braced = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
        auto animated = braced;
        animated.translate.x = 7.0f;
        (void)claims::commit(registry, "animation", false, 120, animated);
        for (const float forward : { 0.0f, 10.0f, -10.0f, 30.0f }) {
            auto intent = braced;
            intent.translate.x = forward;
            auto grip = intent;
            grip.translate.x += 2.0f;
            (void)claims::commit(registry, "grip", false, 100, grip);
            (void)claims::remove(registry, "collision", false);
            const auto* input = claims::winner(registry, false, "collision", 109);
            if (!input) {
                ok = false;
                continue;
            }
            const auto collision = reframeAttachedHand(intent, braced, input->target);
            ok &= expectNear("braced grip does not inherit inverse controller displacement", collision.translate.x, 2.0f);
            (void)claims::commit(registry, "collision", false, 110, collision);
            (void)claims::commit(registry, "animation", false, 120, animated);
            const auto* finalHand = claims::winner(registry, false);
            ok &= finalHand && claims::tagView(*finalHand) == "animation";
            if (finalHand) ok &= expectPoint("animation stays at the bolt while controller moves", finalHand->target.translate, animated.translate);
        }
        (void)claims::remove(registry, "animation", false);
        const auto* released = claims::winner(registry, false);
        ok &= released && claims::tagView(*released) == "collision";
        if (released) ok &= expectNear("animation release returns to the corrected grip", released->target.translate.x, 2.0f);
    }

    return ok ? 0 : 1;
}
