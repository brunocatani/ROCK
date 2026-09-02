#include "physics-interaction/grab/GrabThreePhase.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>

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

    bool expectReason(const char* label, const char* actual, const char* expected)
    {
        if (actual && std::strcmp(actual, expected) == 0) {
            return true;
        }

        std::printf("%s expected %s got %s\n", label, expected, actual ? actual : "null");
        return false;
    }

    bool expectNear(const char* label, float actual, float expected, float tolerance)
    {
        if (std::fabs(actual - expected) <= tolerance) {
            return true;
        }

        std::printf("%s expected %.4f got %.4f\n", label, expected, actual);
        return false;
    }

    float pointDot(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    rock::grab_three_phase::GrabPocketFrame makePocket()
    {
        rock::grab_three_phase::GrabPocketFrame pocket{};
        pocket.valid = true;
        pocket.palmCenterWorld = RE::NiPoint3{ 0.0f, 0.0f, 0.0f };
        pocket.palmNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f };
        pocket.pocketCenterWorld = RE::NiPoint3{ 0.0f, 0.0f, 4.0f };
        pocket.pocketRadiusGameUnits = 9.0f;
        pocket.pocketDepthGameUnits = 4.0f;
        return pocket;
    }
}

int main()
{
    using namespace rock::grab_three_phase;

    bool ok = true;

    const RE::NiTransform identityHand = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    const auto identityPocket = buildGrabPocketFrameWithPalmCenter(
        identityHand,
        false,
        RE::NiPoint3{ 10.0f, 20.0f, 30.0f },
        7.0f,
        9.0f);
    ok &= expectTrue("proxy-basis pocket frame valid", identityPocket.valid);
    ok &= expectNear("proxy-basis pocket keeps reversed palm normal X", identityPocket.palmNormalWorld.x, 0.0f, 0.001f);
    ok &= expectNear("proxy-basis pocket keeps reversed palm normal Y", identityPocket.palmNormalWorld.y, -1.0f, 0.001f);
    ok &= expectNear("proxy-basis pocket keeps reversed palm normal Z", identityPocket.palmNormalWorld.z, 0.0f, 0.001f);
    ok &= expectNear("proxy-basis pocket projects finger off palm normal", pointDot(identityPocket.fingerForwardWorld, identityPocket.palmNormalWorld), 0.0f, 0.001f);
    ok &= expectNear("proxy-basis pocket projects cross-palm off palm normal", pointDot(identityPocket.crossPalmWorld, identityPocket.palmNormalWorld), 0.0f, 0.001f);
    ok &= expectNear("proxy-basis pocket keeps tangent and bitangent orthogonal", pointDot(identityPocket.fingerForwardWorld, identityPocket.crossPalmWorld), 0.0f, 0.001f);

    constexpr float kTiltDegrees = 15.0f;
    constexpr float kExpectedFingerComponent = 0.2588190451f;
    constexpr float kExpectedCrossPalmComponent = 0.9659258263f;
    const RE::NiPoint3 fingerForward{ 1.0f, 0.0f, 0.0f };
    const auto rightPresentationAxis = buildGripPresentationAxisTowardFingertips(
        RE::NiPoint3{ 0.0f, 0.0f, 1.0f }, fingerForward, kTiltDegrees);
    const auto leftPresentationAxis = buildGripPresentationAxisTowardFingertips(
        RE::NiPoint3{ 0.0f, 0.0f, -1.0f }, fingerForward, kTiltDegrees);
    ok &= expectNear("right grip tilt points toward fingertips", rightPresentationAxis.x, kExpectedFingerComponent, 0.001f);
    ok &= expectNear("right grip tilt keeps signed cross-palm Z", rightPresentationAxis.z, kExpectedCrossPalmComponent, 0.001f);
    ok &= expectNear("left grip tilt points toward fingertips", leftPresentationAxis.x, kExpectedFingerComponent, 0.001f);
    ok &= expectNear("left grip tilt keeps reversed cross-palm Z", leftPresentationAxis.z, -kExpectedCrossPalmComponent, 0.001f);
    ok &= expectNear(
        "grip tilt keeps the same fingertip angle across hands",
        pointDot(rightPresentationAxis, fingerForward),
        pointDot(leftPresentationAxis, fingerForward),
        0.001f);
    const auto rejectedSignedPresentationAxis = buildGripPresentationAxisTowardFingertips(
        RE::NiPoint3{ 0.0f, 0.0f, 1.0f }, fingerForward, -kTiltDegrees);
    ok &= expectNear(
        "grip tilt rejects the retired signed-angle convention",
        lengthSquared(rejectedSignedPresentationAxis),
        0.0f,
        0.001f);

    const auto pocket = makePocket();
    const auto insideGate = evaluatePocketGate(pocket, RE::NiPoint3{ 3.0f, 0.0f, 2.0f }, 1.5f);
    ok &= expectTrue("pocket gate accepts a grip point inside the radius in front of the palm", insideGate.inside);
    ok &= expectReason("pocket gate inside reason", insideGate.reason, "insidePocket");
    ok &= expectNear("pocket gate reports the palm distance", insideGate.gripToPalmDistanceGameUnits, std::sqrt(13.0f), 0.001f);
    ok &= expectNear("pocket gate reports the signed palm distance", insideGate.signedPalmDistanceGameUnits, 2.0f, 0.001f);

    const auto behindGate = evaluatePocketGate(pocket, RE::NiPoint3{ 0.0f, 0.0f, -2.0f }, 1.5f);
    ok &= expectFalse("pocket gate rejects a grip point behind the palm tolerance", behindGate.inside);
    ok &= expectReason("pocket gate behind reason", behindGate.reason, "behindPalm");

    const auto toleratedGate = evaluatePocketGate(pocket, RE::NiPoint3{ 0.0f, 0.0f, -1.0f }, 1.5f);
    ok &= expectTrue("pocket gate tolerates a grip point slightly behind the palm plane", toleratedGate.inside);

    const auto outsideGate = evaluatePocketGate(pocket, RE::NiPoint3{ 9.5f, 0.0f, 1.0f }, 1.5f);
    ok &= expectFalse("pocket gate rejects a grip point outside the radius", outsideGate.inside);
    ok &= expectReason("pocket gate outside reason", outsideGate.reason, "outsidePocketRadius");

    const auto marginGate = evaluatePocketGate(pocket, RE::NiPoint3{ 8.0f, 0.0f, 1.0f }, 1.5f, 1.5f);
    ok &= expectFalse("pocket gate margin shrinks the radius for pull arrival", marginGate.inside);
    const auto marginBehindGate = evaluatePocketGate(pocket, RE::NiPoint3{ 0.0f, 0.0f, -0.5f }, 1.5f, 1.5f);
    ok &= expectFalse("pocket gate margin shrinks the behind-palm tolerance for pull arrival", marginBehindGate.inside);
    const auto marginInsideGate = evaluatePocketGate(pocket, RE::NiPoint3{ 5.0f, 0.0f, 1.0f }, 1.5f, 1.5f);
    ok &= expectTrue("pocket gate margin keeps a well-seated point inside", marginInsideGate.inside);

    GrabPocketFrame invalidPocket{};
    const auto invalidGate = evaluatePocketGate(invalidPocket, RE::NiPoint3{ 0.0f, 0.0f, 1.0f }, 1.5f);
    ok &= expectFalse("pocket gate fails closed without a pocket frame", invalidGate.inside);
    ok &= expectReason("pocket gate invalid reason", invalidGate.reason, "invalidPocketOrGripPoint");


    return ok ? 0 : 1;
}
