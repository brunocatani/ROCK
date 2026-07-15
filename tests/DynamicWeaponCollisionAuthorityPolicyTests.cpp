#include "physics-interaction/weapon/DynamicWeaponCollisionAuthorityPolicy.h"

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

    struct Matrix3
    {
        float entry[3][3]{};
    };

    struct Transform
    {
        Matrix3 rotate{};
        Vec3 translate{};
        float scale = 1.0f;
    };

    Transform identity()
    {
        return rock::transform_math::makeIdentityTransform<Transform>();
    }

    bool near(const char* label, float actual, float expected)
    {
        if (std::fabs(actual - expected) <= 0.0001f) {
            return true;
        }
        std::printf("%s expected %.4f got %.4f\n", label, expected, actual);
        return false;
    }

    bool truth(const char* label, bool actual, bool expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected %s got %s\n", label, expected ? "true" : "false", actual ? "true" : "false");
        return false;
    }
}

int main()
{
    using namespace rock::dynamic_weapon_collision_authority_policy;
    bool ok = true;

    ok &= truth("free residual stays free", evaluateContactResidual(false, 0.05f, 0.1f), false);
    ok &= truth("translation enters contact", evaluateContactResidual(false, 0.16f, 0.0f), true);
    ok &= truth("rotation enters contact", evaluateContactResidual(false, 0.0f, 0.76f), true);
    ok &= truth("hysteresis retains grazing contact", evaluateContactResidual(true, 0.06f, 0.0f), true);
    ok &= truth("contact exits below stay thresholds", evaluateContactResidual(true, 0.04f, 0.19f), false);
    ok &= truth("non-finite residual fails closed", evaluateContactResidual(true, INFINITY, 0.0f), false);

    Transform invalidTransform = identity();
    invalidTransform.rotate.entry[2][1] = INFINITY;
    ok &= truth("non-finite transform fails closed", isFiniteTransform(invalidTransform), false);

    Transform anchorWorld = identity();
    anchorWorld.translate.x = 12.0f;
    Transform anchorLocal = identity();
    anchorLocal.translate.x = 2.0f;
    const Transform root = reconstructWeaponRootFromAnchor(anchorWorld, anchorLocal);
    ok &= near("anchor reconstruction preserves root x", root.translate.x, 10.0f);

    Transform rotatedRoot = identity();
    rotatedRoot.rotate.entry[0][0] = 0.0f;
    rotatedRoot.rotate.entry[0][1] = -1.0f;
    rotatedRoot.rotate.entry[1][0] = 1.0f;
    rotatedRoot.rotate.entry[1][1] = 0.0f;
    rotatedRoot.translate = { 5.0f, 7.0f, 1.0f };
    Transform rotatedAnchorLocal = identity();
    rotatedAnchorLocal.translate = { 2.0f, -1.0f, 3.0f };
    const Transform composedAnchor = rock::transform_math::composeTransforms(rotatedRoot, rotatedAnchorLocal);
    const Transform reconstructedRotatedRoot = reconstructWeaponRootFromAnchor(composedAnchor, rotatedAnchorLocal);
    ok &= near("rotated anchor reconstruction preserves root x", reconstructedRotatedRoot.translate.x, rotatedRoot.translate.x);
    ok &= near("rotated anchor reconstruction preserves root y", reconstructedRotatedRoot.translate.y, rotatedRoot.translate.y);
    ok &= near("rotated anchor reconstruction preserves root rotation 01", reconstructedRotatedRoot.rotate.entry[0][1], rotatedRoot.rotate.entry[0][1]);
    ok &= near("rotated anchor reconstruction preserves root rotation 10", reconstructedRotatedRoot.rotate.entry[1][0], rotatedRoot.rotate.entry[1][0]);

    Transform currentRequest = identity();
    currentRequest.translate.x = 10.0f;
    Transform commandedAnchor = identity();
    commandedAnchor.translate.x = 12.0f;
    Transform liveAnchor = identity();
    liveAnchor.translate.x = 11.0f;
    const Transform translated = applySampledAnchorCorrectionToCurrentIntent(
        currentRequest, commandedAnchor, liveAnchor);
    ok &= near("current intent receives sampled translation correction", translated.translate.x, 9.0f);

    currentRequest = identity();
    currentRequest.translate.x = 2.0f;
    commandedAnchor = identity();
    liveAnchor = identity();
    liveAnchor.rotate.entry[0][0] = 0.0f;
    liveAnchor.rotate.entry[0][1] = -1.0f;
    liveAnchor.rotate.entry[1][0] = 1.0f;
    liveAnchor.rotate.entry[1][1] = 0.0f;
    const Transform rotated = applySampledAnchorCorrectionToCurrentIntent(
        currentRequest, commandedAnchor, liveAnchor);
    ok &= near("lever correction rotates current intent x", rotated.translate.x, 0.0f);
    // Ni stores basis axes in the opposite matrix convention from the usual
    // row-vector sketch; this authored matrix rotates +X toward world -Y.
    ok &= near("lever correction rotates current intent y", rotated.translate.y, -2.0f);
    ok &= near("lever correction publishes live orientation", rotated.rotate.entry[1][0], 1.0f);

    Transform nonIdentityCommand = rotatedRoot;
    Transform nonIdentityLive = identity();
    nonIdentityLive.translate = { -4.0f, 3.0f, 2.0f };
    const Transform exactLive = applySampledAnchorCorrectionToCurrentIntent(
        nonIdentityCommand, nonIdentityCommand, nonIdentityLive);
    ok &= near("matching current command publishes exact live x", exactLive.translate.x, nonIdentityLive.translate.x);
    ok &= near("matching current command publishes exact live y", exactLive.translate.y, nonIdentityLive.translate.y);
    ok &= near("matching current command publishes exact live rotation 00", exactLive.rotate.entry[0][0], nonIdentityLive.rotate.entry[0][0]);
    ok &= near("matching current command publishes exact live rotation 11", exactLive.rotate.entry[1][1], nonIdentityLive.rotate.entry[1][1]);

    return ok ? 0 : 1;
}
