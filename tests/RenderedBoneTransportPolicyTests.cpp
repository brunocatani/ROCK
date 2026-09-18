#include "physics-interaction/hand/RenderedBoneTransportPolicy.h"

#include <cmath>
#include <array>
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
        ok &= expectNear(label, actual.scale, expected.scale, 0.00001f);
        return ok;
    }

    struct BoneSample
    {
        std::string_view name;
        RE::NiTransform world;
        RE::NiTransform nodeWorld;
    };
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
        // The carried top bone re-expressed under its unmoved parent.
        const RE::NiTransform parentWorld = yawed(-15.0f, 90.0f, -30.0f, 70.0f);
        const RE::NiTransform carriedForearm = transportWorld(transport, renderedForearm);
        ok &= expectSameTransform("local under parent", compose(parentWorld, localUnderParent(parentWorld, carriedForearm)), carriedForearm);
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

    // Identical roots with scale drift far from the origin stay inactive: a
    // transpose inverse of a drifted basis would otherwise report tens of
    // game units of delta on every claim-free frame.
    {
        RE::NiTransform far = yawed(35.0f, -32000.0f, 32000.0f, 4800.0f);
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                far.rotate.entry[row][column] *= 1.0003f;
            }
        }
        const HandTransport transport = makeHandTransport(far, true, far, true);
        ok &= expectTrue("drifted identical roots inactive", !transport.active);
        RE::NiTransform moved = far;
        moved.translate.x += 4.0f;
        const HandTransport active = makeHandTransport(moved, true, far, true);
        ok &= expectTrue("drifted moved roots active", active.active);
        ok &= expectNear("drifted delta translation", isolation::translationGameUnits(active.delta, identity()), 4.0f, 0.05f);
        ok &= expectNear("drifted delta rotation", isolation::rotationDegrees(active.delta, identity()), 0.0f, 0.05f);
        ok &= expectTrue("delta orthonormal", rock::transform_math::storedRotationOrthonormalityError(active.delta.rotate) < 1e-5);
    }

    // API 2.3 regression: the pre-IK live array is displaced while the saved
    // final wrist and the current scene node already agree with the input.
    // Reusing the saved wrist leaves the entire collider chain 4.2 gu away.
    // Include a rotated/rescaled snapshot and both hands: all local geometry
    // must survive transport, while current refNodes must not move a second time.
    for (const bool left : { false, true }) {
        for (const bool rotated : { false, true }) {
            const auto target = yawed(40.0f, 10.0f, -25.0f, 70.0f, rotated ? 1.3f : 1.0f);
            auto sampledRoot = target;
            sampledRoot.translate.z -= 4.2f;
            if (rotated) sampledRoot = yawed(-30.0f, 8.0f, -29.0f, 65.8f, 0.9f);
            const auto sampledFinger = compose(sampledRoot, fingerLocal);
            std::array<BoneSample, 4> bones{{
                { left ? "LArm_Hand" : "RArm_Hand", sampledRoot, target },
                { left ? "LArm_Finger23" : "RArm_Finger23", sampledFinger, compose(target, fingerLocal) },
                { left ? "LArm_ForeArm1" : "RArm_ForeArm1", compose(sampledRoot, forearmLocal), compose(target, forearmLocal) },
                { left ? "RArm_Hand" : "LArm_Hand", renderedRoot, renderedRoot },
            }};
            const auto oldTransport = makeHandTransport(target, true, target, true);
            ok &= expectTrue("previous-final transport reproduces offset",
                isolation::translationGameUnits(transportWorld(oldTransport, sampledRoot), target) >= 4.19f);
            ok &= expectTrue("sampled chain transported", transportSnapshotHand(bones, left ? HandChainSide::Left : HandChainSide::Right, target));
            ok &= expectSameTransform("sample wrist reaches current input", bones[0].world, target);
            ok &= expectSameTransform("finger pivot follows current input", bones[1].world, compose(target, fingerLocal));
            ok &= expectSameTransform("forearm shares hand frame", bones[2].world, compose(target, forearmLocal));
            ok &= expectSameTransform("live node is not transported twice", bones[0].nodeWorld, target);
            ok &= expectSameTransform("live finger node is unchanged", bones[1].nodeWorld, compose(target, fingerLocal));
            ok &= expectSameTransform("other hand is untouched", bones[3].world, renderedRoot);
        }
    }
    {
        const auto target = yawed(30.0f, 100.0f, -40.0f, 60.0f, 1.35f);
        const auto transport = makeHandTransport(target, true, renderedRoot, true);
        ok &= expectTrue("scale-only transport active", transport.active);
        ok &= expectSameTransform("scaled finger follows wrist scale", transportWorld(transport, renderedFinger), compose(target, fingerLocal));
    }
    {
        std::array<BoneSample, 2> bones{{
            { "RArm_Hand", renderedRoot, renderedRoot },
            { "RArm_Finger23", renderedFinger, renderedFinger },
        }};
        ok &= expectTrue("missing opposite wrist rejected", !transportSnapshotHand(bones, HandChainSide::Left, renderedRoot));
        bones[1].world.translate.x = std::numeric_limits<float>::quiet_NaN();
        ok &= expectTrue("broken chain rejected", !transportSnapshotHand(bones, HandChainSide::Right, yawed(70.0f, 3.0f, 4.0f, 5.0f)));
        ok &= expectSameTransform("rejected chain leaves wrist unchanged", bones[0].world, renderedRoot);
    }

    if (!ok) {
        std::printf("RenderedBoneTransportPolicyTests FAILED\n");
        return 1;
    }
    std::printf("RenderedBoneTransportPolicyTests passed\n");
    return 0;
}
