#include "physics-interaction/hand/RenderedBoneTransportPolicy.h"

#include <cmath>
#include <cstdio>
#include <limits>

namespace
{
    using namespace rock::rendered_bone_transport_policy;
    namespace isolation = rock::tracked_hand_isolation_policy;

    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectNear(const char* label, float actual, float expected, float epsilon)
    {
        if (std::fabs(actual - expected) <= epsilon) {
            return true;
        }
        std::printf("%s expected %.5f got %.5f\n", label, expected, actual);
        return false;
    }

    template <class Enum>
    bool expectEnum(const char* label, Enum actual, Enum expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected %d got %d\n", label, static_cast<int>(expected), static_cast<int>(actual));
        return false;
    }

    RE::NiTransform identity()
    {
        return rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    }

    RE::NiTransform yawed(float degrees, float x, float y, float z, float scale = 1.0f)
    {
        const float radians = degrees * 0.017453292519943295f;
        const float c = std::cos(radians);
        const float s = std::sin(radians);
        RE::NiTransform t = identity();
        t.rotate.entry[0][0] = c;
        t.rotate.entry[0][1] = -s;
        t.rotate.entry[1][0] = s;
        t.rotate.entry[1][1] = c;
        t.translate = RE::NiPoint3(x, y, z);
        t.scale = scale;
        return t;
    }

    RE::NiTransform compose(const RE::NiTransform& a, const RE::NiTransform& b)
    {
        return rock::transform_math::composeTransforms(a, b);
    }

    bool expectSameTransform(const char* label, const RE::NiTransform& actual, const RE::NiTransform& expected)
    {
        bool ok = expectNear(label, isolation::translationGameUnits(actual, expected), 0.0f, 0.002f);
        ok &= expectNear(label, isolation::rotationDegrees(actual, expected), 0.0f, 0.02f);
        return ok;
    }
}

int main()
{
    bool ok = true;

    // Chain membership by name: hand, fingers and the three forearm bones of each side only.
    ok &= expectEnum("right hand", chainSideForBone("RArm_Hand"), HandChainSide::Right);
    ok &= expectEnum("left hand", chainSideForBone("LArm_Hand"), HandChainSide::Left);
    ok &= expectEnum("right finger", chainSideForBone("RArm_Finger23"), HandChainSide::Right);
    ok &= expectEnum("left finger tip", chainSideForBone("LArm_Finger53"), HandChainSide::Left);
    ok &= expectEnum("right forearm 1", chainSideForBone("RArm_ForeArm1"), HandChainSide::Right);
    ok &= expectEnum("left forearm 3", chainSideForBone("LArm_ForeArm3"), HandChainSide::Left);
    ok &= expectEnum("upper arm stays", chainSideForBone("RArm_UpperArm"), HandChainSide::None);
    ok &= expectEnum("twist stays", chainSideForBone("LArm_UpperTwist1"), HandChainSide::None);
    ok &= expectEnum("collarbone stays", chainSideForBone("RArm_Collarbone"), HandChainSide::None);
    ok &= expectEnum("spine stays", chainSideForBone("SPINE1"), HandChainSide::None);
    ok &= expectEnum("bare prefix", chainSideForBone("RArm_"), HandChainSide::None);
    ok &= expectEnum("empty", chainSideForBone(""), HandChainSide::None);

    const RE::NiTransform renderedRoot = yawed(30.0f, 100.0f, -40.0f, 60.0f);
    const RE::NiTransform fingerLocal = yawed(-20.0f, 6.0f, 1.0f, -0.5f);
    const RE::NiTransform forearmLocal = yawed(5.0f, -12.0f, 0.0f, 0.0f);
    const RE::NiTransform renderedFinger = compose(renderedRoot, fingerLocal);
    const RE::NiTransform renderedForearm = compose(renderedRoot, forearmLocal);

    // Claim-free: the isolated root is the rendered root, nothing moves.
    {
        const HandTransport transport = makeHandTransport(renderedRoot, true, renderedRoot, true);
        ok &= expectTrue("identity inactive", !transport.active);
        ok &= expectSameTransform("identity passthrough", transportWorld(transport, renderedFinger), renderedFinger);
    }

    // Claimed: the chain is carried rigidly to the controller root.
    {
        const RE::NiTransform controllerRoot = compose(yawed(10.0f, 5.0f, -3.0f, 2.0f), renderedRoot);
        const HandTransport transport = makeHandTransport(controllerRoot, true, renderedRoot, true);
        ok &= expectTrue("claimed active", transport.active);
        ok &= expectSameTransform("root lands on controller", transportWorld(transport, renderedRoot), controllerRoot);
        ok &= expectSameTransform("finger keeps its local", transportWorld(transport, renderedFinger), compose(controllerRoot, fingerLocal));
        ok &= expectSameTransform("forearm keeps its local", transportWorld(transport, renderedForearm), compose(controllerRoot, forearmLocal));
        // Finger pose relative to the root is unchanged by the transport.
        const RE::NiTransform relative = compose(
            rock::transform_math::invertTransform(transportWorld(transport, renderedRoot)),
            transportWorld(transport, renderedFinger));
        ok &= expectSameTransform("relative pose preserved", relative, fingerLocal);
    }

    // Missing or broken roots fall back to the rendered bones.
    {
        RE::NiTransform broken = renderedRoot;
        broken.translate.x = std::numeric_limits<float>::quiet_NaN();
        ok &= expectTrue("controller missing", !makeHandTransport(renderedRoot, false, renderedRoot, true).active);
        ok &= expectTrue("rendered missing", !makeHandTransport(renderedRoot, true, renderedRoot, false).active);
        ok &= expectTrue("non-finite controller", !makeHandTransport(broken, true, renderedRoot, true).active);
        ok &= expectTrue("non-finite rendered", !makeHandTransport(renderedRoot, true, broken, true).active);
    }

    if (!ok) {
        std::printf("RenderedBoneTransportPolicyTests FAILED\n");
        return 1;
    }
    std::printf("RenderedBoneTransportPolicyTests passed\n");
    return 0;
}
