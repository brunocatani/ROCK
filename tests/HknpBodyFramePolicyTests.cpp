#include "physics-interaction/TransformMath.h"

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

    return ok ? 0 : 1;
}
