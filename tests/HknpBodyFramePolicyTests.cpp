#include "physics-interaction/TransformMath.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/weapon/LooseWeaponCollisionPolicy.h"

#include "RE/NetImmerse/NiMatrix3.h"
#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

#include <cmath>
#include <cstdio>
#include <string>

namespace
{
    constexpr float kEpsilon = 0.001f;

    bool expectNear(const char* label, float actual, float expected, float epsilon = kEpsilon)
    {
        const float delta = std::fabs(actual - expected);
        if (delta <= epsilon) {
            return true;
        }

        std::printf("%s expected %.5f got %.5f\n", label, expected, actual);
        return false;
    }

    bool expectVectorNear(const char* label, const RE::NiPoint3& actual, const RE::NiPoint3& expected)
    {
        bool ok = true;
        ok &= expectNear((std::string(label) + ".x").c_str(), actual.x, expected.x);
        ok &= expectNear((std::string(label) + ".y").c_str(), actual.y, expected.y);
        ok &= expectNear((std::string(label) + ".z").c_str(), actual.z, expected.z);
        return ok;
    }

    bool expectMatrixEntryNear(const char* label, const RE::NiMatrix3& matrix, int row, int column, float expected)
    {
        return expectNear((std::string(label) + "[" + std::to_string(row) + "][" + std::to_string(column) + "]").c_str(),
            matrix.entry[row][column],
            expected);
    }

    RE::NiPoint3 nativeBodyLocalPointToWorldGame(const float* bodyFloats, const RE::NiPoint3& localGame, const RE::NiPoint3& translationGame)
    {
        return RE::NiPoint3{
            translationGame.x + localGame.x * bodyFloats[0] + localGame.y * bodyFloats[4] + localGame.z * bodyFloats[8],
            translationGame.y + localGame.x * bodyFloats[1] + localGame.y * bodyFloats[5] + localGame.z * bodyFloats[9],
            translationGame.z + localGame.x * bodyFloats[2] + localGame.y * bodyFloats[6] + localGame.z * bodyFloats[10],
        };
    }
}

int main()
{
    /*
     * This test protects the FO4VR hknp BODY boundary. Ghidra shows native BODY
     * transform blocks are local axes in world, and ROCK's TransformMath also
     * expects stored rows to be local axes. BODY read/write conversion therefore
     * copies those blocks directly; transposing them makes off-center grab
     * pivots appear to slide around rotated objects.
     */
    bool ok = true;

    // The Boston session allocated weapon authority motion 4253. Rejecting it
    // at 4096 switched live readback from MOTION to BODY and inverted rotation.
    const auto expectMotionSlot = [&](const char* label, std::uint32_t motion,
                                      std::uint32_t count, std::uint32_t flags, bool expected) {
        const bool actual = rock::body_frame::motionSlotCanBeRead(motion, count, flags);
        if (actual != expected) {
            std::printf("%s expected %d got %d\n", label, expected, actual);
            ok = false;
        }
    };
    expectMotionSlot("last slot below old cutoff", 4095, 8192, 0x80002000, true);
    expectMotionSlot("old cutoff is a valid slot", 4096, 8192, 0x80002000, true);
    expectMotionSlot("Boston weapon authority", 4253, 8192, 0x80002000, true);
    expectMotionSlot("last live array slot", 8191, 8192, 0x80002000, true);
    expectMotionSlot("one past live array", 8192, 8192, 0x80002000, false);
    expectMotionSlot("capacity alone does not admit a slot", 4253, 4096, 0x80002000, false);
    expectMotionSlot("count exceeds capacity", 4253, 8192, 0x80001000, false);
    expectMotionSlot("static motion is not dynamic", 0, 8192, 0x80002000, false);
    expectMotionSlot("free motion is rejected", 0x7FFFFFFF, 8192, 0x80002000, false);
    expectMotionSlot("corrupt array size is rejected", 4253, 0xFFFFFFFF, 0xFFFFFFFF, false);

    constexpr float bodyFloats[16]{
        0.0f, -1.0f, 0.0f, 2.0f,
        0.0f, 0.0f, 1.0f, -3.0f,
        -1.0f, 0.0f, 0.0f, 4.0f,
        10.0f, 20.0f, 30.0f, 0.0f,
    };
    const RE::NiPoint3 translationGame{ bodyFloats[12], bodyFloats[13], bodyFloats[14] };
    const RE::NiPoint3 localPivotGame{ 3.0f, -2.0f, 5.0f };

    RE::NiTransform bodyWorld = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    bodyWorld.rotate = rock::transform_math::hknpBodyColumnsToNiStoredAxes<RE::NiMatrix3>(bodyFloats);
    bodyWorld.translate = translationGame;

    const RE::NiPoint3 expectedWorld = nativeBodyLocalPointToWorldGame(bodyFloats, localPivotGame, translationGame);
    const RE::NiPoint3 rockWorld = rock::transform_math::localPointToWorld(bodyWorld, localPivotGame);
    ok &= expectVectorNear("BODY local pivot through ROCK transform", rockWorld, expectedWorld);

    const RE::NiPoint3 roundTripLocal = rock::transform_math::worldPointToLocal(bodyWorld, expectedWorld);
    ok &= expectVectorNear("BODY pivot world/local round trip", roundTripLocal, localPivotGame);

    const RE::NiMatrix3 hkTransformRotation = rock::transform_math::niStoredAxesToHknpBodyColumns(bodyWorld.rotate);
    ok &= expectMatrixEntryNear("hkTransform BODY block", hkTransformRotation, 0, 0, bodyFloats[0]);
    ok &= expectMatrixEntryNear("hkTransform BODY block", hkTransformRotation, 0, 1, bodyFloats[1]);
    ok &= expectMatrixEntryNear("hkTransform BODY block", hkTransformRotation, 0, 2, bodyFloats[2]);
    ok &= expectMatrixEntryNear("hkTransform BODY block", hkTransformRotation, 1, 0, bodyFloats[4]);
    ok &= expectMatrixEntryNear("hkTransform BODY block", hkTransformRotation, 1, 1, bodyFloats[5]);
    ok &= expectMatrixEntryNear("hkTransform BODY block", hkTransformRotation, 1, 2, bodyFloats[6]);
    ok &= expectMatrixEntryNear("hkTransform BODY block", hkTransformRotation, 2, 0, bodyFloats[8]);
    ok &= expectMatrixEntryNear("hkTransform BODY block", hkTransformRotation, 2, 1, bodyFloats[9]);
    ok &= expectMatrixEntryNear("hkTransform BODY block", hkTransformRotation, 2, 2, bodyFloats[10]);

    const RE::NiMatrix3 transposedReadback = rock::transform_math::havokColumnsToNiRows<RE::NiMatrix3>(bodyFloats);
    RE::NiTransform transposedBodyWorld = bodyWorld;
    transposedBodyWorld.rotate = transposedReadback;
    const RE::NiPoint3 wrongWorld = rock::transform_math::localPointToWorld(transposedBodyWorld, localPivotGame);
    const float wrongDelta = std::sqrt(
        (wrongWorld.x - expectedWorld.x) * (wrongWorld.x - expectedWorld.x) +
        (wrongWorld.y - expectedWorld.y) * (wrongWorld.y - expectedWorld.y) +
        (wrongWorld.z - expectedWorld.z) * (wrongWorld.z - expectedWorld.z));
    if (wrongDelta <= 1.0f) {
        std::printf("transposed BODY readback unexpectedly matched native local-axis math\n");
        ok = false;
    }

    // Two shape BODY frames may share a single MOTION. Compound children must
    // recover the same mesh centre and axes through either BODY's native math.
    auto rootWorld=rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    rootWorld.translate={42,51,63};
    auto sourceInRoot=rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    sourceInRoot.rotate=bodyWorld.rotate;
    sourceInRoot.translate={3,7,11};
    const RE::NiPoint3 centre{2,4,6};
    const auto sourceWorld=rock::transform_math::composeTransforms(rootWorld,sourceInRoot);
    const auto meshCentreWorld=rock::transform_math::localPointToWorld(sourceWorld,centre);
    constexpr float secondBody[16]{1,0,0,0,0,1,0,0,0,0,1,0,-10,8,12,0};
    for (const auto* nativeBody : {bodyFloats,secondBody}) {
        auto frame=rock::transform_math::makeIdentityTransform<RE::NiTransform>();
        frame.rotate=rock::transform_math::hknpBodyColumnsToNiStoredAxes<RE::NiMatrix3>(nativeBody);
        frame.translate={nativeBody[12],nativeBody[13],nativeBody[14]};
        const auto rootInBody=rock::transform_math::composeTransforms(rock::transform_math::invertTransform(frame),rootWorld);
        const auto child=rock::loose_weapon_collision_policy::partInShapeBody(rootInBody,sourceInRoot,centre);
        ok &= expectVectorNear("compound centre in native BODY",nativeBodyLocalPointToWorldGame(nativeBody,child.translate,frame.translate),meshCentreWorld);
        for (const RE::NiPoint3 axis : {RE::NiPoint3{1,0,0},RE::NiPoint3{0,1,0},RE::NiPoint3{0,0,1}}) {
            const auto inBody=rock::transform_math::localPointToWorld(child,axis);
            const auto expected=rock::transform_math::localPointToWorld(sourceWorld,centre+axis);
            ok &= expectVectorNear("compound axis in native BODY",nativeBodyLocalPointToWorldGame(nativeBody,inBody,frame.translate),expected);
        }
        auto animated=sourceInRoot; animated.translate.y+=5;
        const auto movingChild=rock::loose_weapon_collision_policy::partInShapeBody(rootInBody,animated,centre);
        const auto movingWorld=rock::transform_math::composeTransforms(rootWorld,animated);
        ok &= expectVectorNear("animated part retains BODY calibration",
            nativeBodyLocalPointToWorldGame(nativeBody,movingChild.translate,frame.translate),
            rock::transform_math::localPointToWorld(movingWorld,centre));
    }
    // The previous COM substitution must fail for this rotated shape body.
    const auto wrongChild=rock::loose_weapon_collision_policy::partInShapeBody(rootWorld,sourceInRoot,centre);
    const auto wrongCentre=nativeBodyLocalPointToWorldGame(bodyFloats,wrongChild.translate,translationGame);
    if ((wrongCentre-meshCentreWorld).Length()<1.0f) { std::puts("shared MOTION unexpectedly matched an offset shape BODY"); ok=false; }
    return ok ? 0 : 1;
}
