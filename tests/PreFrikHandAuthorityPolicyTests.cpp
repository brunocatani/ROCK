#include "physics-interaction/native/hooks/MainLoopHookPolicy.h"
#include "physics-interaction/visual/PreFrikHandAuthorityPolicy.h"

#include "RE/NetImmerse/NiTransform.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace
{
    constexpr float kEpsilon = 0.001f;

    RE::NiTransform identityTransform()
    {
        RE::NiTransform result{};
        result.rotate.entry[0][0] = 1.0f;
        result.rotate.entry[1][1] = 1.0f;
        result.rotate.entry[2][2] = 1.0f;
        result.scale = 1.0f;
        return result;
    }

    bool expectNear(const char* label, const float actual, const float expected)
    {
        if (std::fabs(actual - expected) <= kEpsilon) {
            return true;
        }
        std::printf("%s expected %.4f got %.4f\n", label, expected, actual);
        return false;
    }

    bool expectTrue(const char* label, const bool value)
    {
        if (value) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, const bool value)
    {
        if (!value) {
            return true;
        }
        std::printf("%s expected false\n", label);
        return false;
    }
}

int main()
{
    using namespace rock::prefrik_hand_authority_policy;
    bool ok = true;

    ok &= expectTrue("immediate scheduler successor", isImmediateSuccessor(41, 42));
    ok &= expectFalse("same scheduler generation rejected", isImmediateSuccessor(42, 42));
    ok &= expectFalse("skipped scheduler generation rejected", isImmediateSuccessor(40, 42));
    ok &= expectFalse("zero scheduler generation rejected", isImmediateSuccessor(0, 0));

    RE::NiTransform driver = identityTransform();
    driver.translate = RE::NiPoint3{ 10.0f, 20.0f, 30.0f };
    RE::NiTransform target = identityTransform();
    target.translate = RE::NiPoint3{ 13.0f, 25.0f, 37.0f };
    const auto local = captureDriverToTargetLocal(driver, target);
    RE::NiTransform movedDriver = driver;
    movedDriver.translate = RE::NiPoint3{ 12.0f, 18.0f, 34.0f };
    const auto reconstructed = reconstructTargetWorld(movedDriver, local);
    ok &= expectNear("driver-relative target follows x", reconstructed.translate.x, 15.0f);
    ok &= expectNear("driver-relative target follows y", reconstructed.translate.y, 23.0f);
    ok &= expectNear("driver-relative target follows z", reconstructed.translate.z, 41.0f);

    RE::NiTransform sourceRaw = identityTransform();
    const RE::NiPoint3 outwardDeviation{ 2.0f, 0.0f, 0.0f };

    RE::NiTransform inwardRaw = sourceRaw;
    inwardRaw.translate = RE::NiPoint3{ -1.0f, 0.0f, 0.0f };
    const auto inward = transportContactTarget(
        sourceRaw,
        inwardRaw,
        outwardDeviation,
        40.0f);
    ok &= expectTrue("inward contact transport valid", inward.valid);
    ok &= expectNear("inward motion remains blocked", inward.targetWorld.translate.x, 2.0f);
    ok &= expectNear("inward motion removal reported", inward.removedInwardMotionGameUnits, 1.0f);

    RE::NiTransform tangentRaw = sourceRaw;
    tangentRaw.translate = RE::NiPoint3{ 0.0f, 3.0f, -2.0f };
    const auto tangent = transportContactTarget(
        sourceRaw,
        tangentRaw,
        outwardDeviation,
        40.0f);
    ok &= expectTrue("tangential contact transport valid", tangent.valid);
    ok &= expectNear("tangent retains blocked depth", tangent.targetWorld.translate.x, 2.0f);
    ok &= expectNear("tangent follows y", tangent.targetWorld.translate.y, 3.0f);
    ok &= expectNear("tangent follows z", tangent.targetWorld.translate.z, -2.0f);

    RE::NiTransform outwardRaw = sourceRaw;
    outwardRaw.translate = RE::NiPoint3{ 1.5f, 0.0f, 0.0f };
    const auto outward = transportContactTarget(
        sourceRaw,
        outwardRaw,
        outwardDeviation,
        40.0f);
    ok &= expectTrue("outward contact transport valid", outward.valid);
    ok &= expectNear("outward motion releases from surface", outward.targetWorld.translate.x, 3.5f);

    RE::NiTransform divergentRaw = sourceRaw;
    divergentRaw.translate = RE::NiPoint3{ 41.0f, 0.0f, 0.0f };
    ok &= expectFalse(
        "divergent raw motion fails closed",
        transportContactTarget(
            sourceRaw,
            divergentRaw,
            outwardDeviation,
            40.0f).valid);

    RE::NiTransform solvedRigid = sourceRaw;
    solvedRigid.translate = outwardDeviation;
    solvedRigid.rotate.entry[0][0] = 0.0f;
    solvedRigid.rotate.entry[0][1] = 1.0f;
    solvedRigid.rotate.entry[1][0] = -1.0f;
    solvedRigid.rotate.entry[1][1] = 0.0f;
    const auto rigidTransport = transportRigidContactTarget(
        sourceRaw,
        solvedRigid,
        tangentRaw,
        40.0f);
    ok &= expectTrue("rigid contact transport valid", rigidTransport.valid);
    ok &= expectNear(
        "rigid contact keeps rotation 00",
        rigidTransport.targetWorld.rotate.entry[0][0],
        0.0f);
    ok &= expectNear(
        "rigid contact keeps rotation 01",
        rigidTransport.targetWorld.rotate.entry[0][1],
        1.0f);
    ok &= expectNear(
        "rigid contact keeps blocked x",
        rigidTransport.targetWorld.translate.x,
        2.0f);
    ok &= expectNear(
        "rigid contact follows tangent y",
        rigidTransport.targetWorld.translate.y,
        3.0f);

    RE::NiTransform pureRotation = solvedRigid;
    pureRotation.translate = sourceRaw.translate;
    const auto pureRotationTransport = transportRigidContactTarget(
        sourceRaw,
        pureRotation,
        tangentRaw,
        40.0f);
    ok &= expectTrue(
        "pure rotation contact transport valid",
        pureRotationTransport.valid);
    ok &= expectNear(
        "pure rotation follows raw translation",
        pureRotationTransport.targetWorld.translate.y,
        3.0f);
    ok &= expectNear(
        "pure rotation survives transport",
        pureRotationTransport.targetWorld.rotate.entry[1][0],
        -1.0f);

    std::uint8_t callBytes[rock::main_loop_hook_policy::kRelativeCallSize]{};
    callBytes[0] = rock::main_loop_hook_policy::kRelativeCallOpcode;
    constexpr std::uintptr_t callAddress = 0x1000;
    constexpr std::uintptr_t callTarget = 0x2800;
    const auto displacement = static_cast<std::int32_t>(
        callTarget -
        (callAddress + rock::main_loop_hook_policy::kRelativeCallSize));
    std::memcpy(callBytes + 1, &displacement, sizeof(displacement));
    std::uintptr_t decodedTarget = 0;
    ok &= expectTrue(
        "relative call decodes",
        rock::main_loop_hook_policy::decodeRelativeCallTarget(
            callBytes,
            callAddress,
            decodedTarget));
    ok &= expectTrue("relative call target exact", decodedTarget == callTarget);

    std::uint8_t thunkBytes[
        rock::main_loop_hook_policy::kCommonLibAbsoluteJumpThunkSize]{};
    thunkBytes[0] = rock::main_loop_hook_policy::kAbsoluteJumpOpcode;
    thunkBytes[1] = rock::main_loop_hook_policy::kRipIndirectJumpModRm;
    constexpr std::uint64_t thunkTarget = 0x1234'5678'9ABC'DEF0ull;
    std::memcpy(thunkBytes + 6, &thunkTarget, sizeof(thunkTarget));
    decodedTarget = 0;
    ok &= expectTrue(
        "CommonLib absolute thunk decodes",
        rock::main_loop_hook_policy::decodeCommonLibAbsoluteJumpTarget(
            thunkBytes,
            decodedTarget));
    ok &= expectTrue(
        "CommonLib absolute thunk target exact",
        decodedTarget == static_cast<std::uintptr_t>(thunkTarget));

    return ok ? 0 : 1;
}
