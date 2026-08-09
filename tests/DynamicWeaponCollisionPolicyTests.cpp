#include "physics-interaction/weapon/DynamicWeaponCollisionPolicy.h"
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

    constexpr auto dynamicWeaponMask = rock::collision_layer_policy::buildRockDynamicWeaponProxyExpectedMask();
    for (std::uint32_t layer = 0; layer < rock::collision_layer_policy::FO4_LAYER_MATRIX_ADDRESSABLE_COUNT; ++layer) {
        const bool enabled = rock::collision_layer_policy::maskEnablesLayer(dynamicWeaponMask, layer);
        const bool expected = rock::collision_layer_policy::isWorldSurfaceLayer(layer);
        if (enabled != expected) {
            std::printf("dynamic weapon layer mismatch at row %u expected=%d actual=%d\n", layer, expected ? 1 : 0, enabled ? 1 : 0);
            ok = false;
        }
    }

    const auto geometry = makeBoxGeometry(
        RE::NiPoint3{ -10.0f, -2.0f, -1.0f },
        RE::NiPoint3{ 30.0f, 4.0f, 3.0f });
    ok &= geometry.valid;
    ok &= expectPoint("box center", geometry.centerWeaponLocal, RE::NiPoint3{ 10.0f, 1.0f, 1.0f });
    ok &= expectPoint("box half extents", geometry.halfExtentsWeaponLocal, RE::NiPoint3{ 20.0f, 3.0f, 2.0f });

    const auto corners = makeBoxCornerPointsHavok(geometry, 2.0f, 1.0f, 0.1f);
    RE::NiPoint3 cornerMax{};
    for (const auto& corner : corners) {
        cornerMax.x = (std::max)(cornerMax.x, std::fabs(corner.x));
        cornerMax.y = (std::max)(cornerMax.y, std::fabs(corner.y));
        cornerMax.z = (std::max)(cornerMax.z, std::fabs(corner.z));
    }
    ok &= expectPoint("scaled padded Havok half extents", cornerMax, RE::NiPoint3{ 4.1f, 0.7f, 0.5f });

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
    const RE::NiTransform reconstructed = reconstructWeaponRoot(bodyTarget, geometry.centerWeaponLocal, weaponRoot.scale);
    ok &= expectPoint("body center target", bodyTarget.translate, RE::NiPoint3{ 98.0f, 70.0f, -23.0f });
    ok &= expectPoint("reconstructed root", reconstructed.translate, weaponRoot.translate);
    ok &= expectNear("reconstructed rotation", rotationDeltaDegrees(reconstructed, weaponRoot), 0.0f, 0.05f);

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

    const auto nativeOneHand = selectAttachedHands(false, false, true, false, false);
    ok &= !nativeOneHand.left && nativeOneHand.right;
    const auto leftPrimaryOnly = selectAttachedHands(false, true, true, false, false);
    ok &= leftPrimaryOnly.left && !leftPrimaryOnly.right;
    const auto rightTwoHand = selectAttachedHands(false, true, false, true, false);
    ok &= rightTwoHand.left && rightTwoHand.right;
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

    return ok ? 0 : 1;
}
