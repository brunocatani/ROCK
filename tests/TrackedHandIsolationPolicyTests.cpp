#include "physics-interaction/hand/TrackedHandIsolationPolicy.h"

#include <cmath>
#include <cstdio>

namespace
{
    using namespace rock::tracked_hand_isolation_policy;

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
        t.translate = RE::NiPoint3{ x, y, z };
        t.scale = scale;
        return t;
    }

    RE::NiTransform compose(const RE::NiTransform& parent, const RE::NiTransform& child)
    {
        return rock::transform_math::composeTransforms(parent, child);
    }
}

int main()
{
    bool ok = true;
    {
        RelationState startup{};
        FrameResult raw{};
        raw.valid = true;
        raw.source = RawHandSource::Flattened;
        ok &= expectFalse("startup cannot acquire an uncalibrated hand", canDriveExternalPose(startup, raw));
        startup.valid = true;
        ok &= expectTrue("calibrated free hand can acquire recoil", canDriveExternalPose(startup, raw));
        raw.source = RawHandSource::FlattenedContaminated;
        ok &= expectFalse("rendered feedback is never recoil input", canDriveExternalPose(startup, raw));
        raw.source = RawHandSource::Reconstructed;
        ok &= expectTrue("qualified controller reconstruction can continue recoil", canDriveExternalPose(startup, raw));
        raw.valid = false;
        ok &= expectFalse("missing controller frame releases recoil", canDriveExternalPose(startup, raw));
    }

    // A skeleton with a small solver residual and a body scale: relation calibrates on a free frame.
    const RE::NiTransform residual = yawed(0.5f, 0.1f, -0.05f, 0.02f, 1.02f);
    const RE::NiTransform firstPersonA = yawed(20.0f, 100.0f, 50.0f, 80.0f);
    const RE::NiTransform bodyNodeA = compose(firstPersonA, residual);
    const RE::NiTransform palmBlend = yawed(6.0f, 0.0f, 0.0f, 0.0f);
    const RE::NiTransform flattenedA = compose(bodyNodeA, palmBlend);

    RelationState state{};
    {
        const FrameInput input{
            .firstPersonHandWorld = firstPersonA,
            .firstPersonHandValid = true,
            .bodyHandNodeWorld = bodyNodeA,
            .bodyHandNodeValid = true,
            .flattenedHandWorld = flattenedA,
            .flattenedHandValid = true,
            .claimConsumed = false,
        };
        const FrameResult result = resolveFrame(state, input);
        ok &= expectEnum("free frame uses flattened", result.source, RawHandSource::Flattened);
        ok &= expectTrue("free frame valid", result.valid);
        ok &= expectNear("free frame raw x", result.rawHandWorld.translate.x, flattenedA.translate.x, 0.0001f);
        ok &= expectTrue("relation calibrated", state.valid);
        ok &= expectTrue("one accepted sample", state.acceptedSamples == 1);
        ok &= expectTrue("probe valid", result.probeValid);
        ok &= expectNear("probe translation zero", result.probeTranslationGameUnits, 0.0f, 0.001f);
        ok &= expectNear("probe rotation zero", result.probeRotationDegrees, 0.0f, 0.01f);
    }

    // Claimed frame: the flattened bone is ROCK's previous claim, far from the controller; the raw hand follows the first-person hand.
    {
        const RE::NiTransform firstPersonB = yawed(35.0f, 130.0f, 40.0f, 90.0f);
        const RE::NiTransform claimBodyNode = yawed(-10.0f, 300.0f, 300.0f, 300.0f); // where FRIK solved ROCK's claim
        const RE::NiTransform blendB = yawed(2.0f, 0.0f, 0.0f, 0.0f);
        const RE::NiTransform flattenedB = compose(claimBodyNode, blendB);
        const FrameInput input{
            .firstPersonHandWorld = firstPersonB,
            .firstPersonHandValid = true,
            .bodyHandNodeWorld = claimBodyNode,
            .bodyHandNodeValid = true,
            .flattenedHandWorld = flattenedB,
            .flattenedHandValid = true,
            .claimConsumed = true,
        };
        const FrameResult result = resolveFrame(state, input);
        ok &= expectEnum("claimed frame reconstructs", result.source, RawHandSource::Reconstructed);
        ok &= expectTrue("claimed frame valid", result.valid);
        const RE::NiTransform expected = compose(compose(firstPersonB, residual), blendB);
        ok &= expectNear("reconstructed x", result.rawHandWorld.translate.x, expected.translate.x, 0.001f);
        ok &= expectNear("reconstructed y", result.rawHandWorld.translate.y, expected.translate.y, 0.001f);
        ok &= expectNear("reconstructed z", result.rawHandWorld.translate.z, expected.translate.z, 0.001f);
        ok &= expectNear("reconstructed rotation", rotationDegrees(result.rawHandWorld, expected), 0.0f, 0.01f);
        ok &= expectNear("reconstructed scale", result.rawHandWorld.scale, expected.scale, 0.0001f);
        ok &= expectTrue("far from the claimed bone", translationGameUnits(result.rawHandWorld, flattenedB) > 100.0f);
        ok &= expectTrue("claimed frame does not calibrate", state.acceptedSamples == 1);
        ok &= expectFalse("claimed frame has no probe", result.probeValid);
    }

    // Claimed frame without a relation or without the first-person hand degrades to the contaminated bone.
    {
        RelationState empty{};
        const FrameInput input{
            .firstPersonHandWorld = firstPersonA,
            .firstPersonHandValid = true,
            .bodyHandNodeWorld = bodyNodeA,
            .bodyHandNodeValid = true,
            .flattenedHandWorld = flattenedA,
            .flattenedHandValid = true,
            .claimConsumed = true,
        };
        ok &= expectEnum("no relation degrades", resolveFrame(empty, input).source, RawHandSource::FlattenedContaminated);

        FrameInput noFirstPerson = input;
        noFirstPerson.firstPersonHandValid = false;
        ok &= expectEnum("no first-person hand degrades", resolveFrame(state, noFirstPerson).source, RawHandSource::FlattenedContaminated);

        FrameInput nothing = input;
        nothing.flattenedHandValid = false;
        nothing.firstPersonHandValid = false;
        const FrameResult none = resolveFrame(state, nothing);
        ok &= expectEnum("nothing available", none.source, RawHandSource::Unavailable);
        ok &= expectFalse("nothing invalid", none.valid);
    }

    // Calibration rejects implausible relations (scope-menu collapse, missing arm) and keeps the previous one.
    {
        RelationState guarded = state;
        const RE::NiTransform collapsed = yawed(0.0f, 100.0f, 50.0f, 80.0f, 0.00001f);
        ok &= expectFalse("collapsed scale rejected", calibrateRelation(guarded, firstPersonA, collapsed));
        ok &= expectFalse("far body rejected", calibrateRelation(guarded, firstPersonA, yawed(0.0f, 150.0f, 50.0f, 80.0f)));
        ok &= expectFalse("twisted body rejected", calibrateRelation(guarded, firstPersonA, compose(firstPersonA, yawed(30.0f, 0.0f, 0.0f, 0.0f))));
        RE::NiTransform nan = firstPersonA;
        nan.translate.y = std::nanf("");
        ok &= expectFalse("non-finite rejected", calibrateRelation(guarded, nan, bodyNodeA));
        ok &= expectTrue("previous relation kept", guarded.valid && guarded.rejectedSamples == 4 && guarded.acceptedSamples == 1);
        ok &= expectNear("previous relation unchanged", guarded.firstPersonToBodyHand.translate.x, residual.translate.x, 0.0001f);
    }

    // A frame that carries a native recoil kick does not calibrate but still serves the flattened bone.
    {
        RelationState untouched = state;
        FrameInput input{
            .firstPersonHandWorld = firstPersonA,
            .firstPersonHandValid = true,
            .bodyHandNodeWorld = compose(bodyNodeA, yawed(3.0f, 2.0f, 0.0f, 0.0f)), // kick composed on the tracked target
            .bodyHandNodeValid = true,
            .flattenedHandWorld = flattenedA,
            .flattenedHandValid = true,
            .claimConsumed = false,
            .calibrationAllowed = false,
        };
        const FrameResult result = resolveFrame(untouched, input);
        ok &= expectEnum("recoil frame uses flattened", result.source, RawHandSource::Flattened);
        ok &= expectTrue("recoil frame did not calibrate", untouched.acceptedSamples == state.acceptedSamples && untouched.rejectedSamples == state.rejectedSamples);
        ok &= expectNear("recoil frame relation unchanged", untouched.firstPersonToBodyHand.translate.x, state.firstPersonToBodyHand.translate.x, 0.0001f);
    }

    // Palm blend extraction is exact.
    {
        const RE::NiTransform blend = makePalmBlend(bodyNodeA, flattenedA);
        ok &= expectNear("blend rotation", rotationDegrees(blend, palmBlend), 0.0f, 0.01f);
        ok &= expectNear("blend translation", translationGameUnits(blend, identity()), 0.0f, 0.001f);
    }

    // FRIK echo loop with drifted bases: the rendered hand is ROCK's own
    // previous target, its basis carries ~1e-4 of scale drift, and the
    // reconstruction must not square that drift frame after frame.
    {
        const auto drift = [](RE::NiTransform t, float factor) {
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    t.rotate.entry[row][column] *= factor;
                }
            }
            return t;
        };
        constexpr float kDrift = 1.0003f;
        const RE::NiTransform fp = yawed(25.0f, -32000.0f, 32000.0f, 4800.0f);
        const RE::NiTransform node = compose(fp, yawed(0.5f, 0.4f, -0.2f, 0.1f));
        RelationState loopState{};
        FrameInput freeFrame{
            .firstPersonHandWorld = drift(fp, kDrift),
            .firstPersonHandValid = true,
            .bodyHandNodeWorld = drift(node, kDrift),
            .bodyHandNodeValid = true,
            .flattenedHandWorld = drift(node, kDrift),
            .flattenedHandValid = true,
            .claimConsumed = false,
        };
        const FrameResult freeResult = resolveFrame(loopState, freeFrame);
        ok &= expectTrue("drift calibration accepted", loopState.valid && loopState.acceptedSamples == 1);
        ok &= expectTrue("drift free frame orthonormal", rock::transform_math::storedRotationOrthonormalityError(freeResult.rawHandWorld.rotate) < 1e-5);

        RE::NiTransform rendered = node;
        float firstX = 0.0f;
        for (int frame = 0; frame < 60; ++frame) {
            FrameInput claimed{
                .firstPersonHandWorld = drift(fp, kDrift),
                .firstPersonHandValid = true,
                .bodyHandNodeWorld = drift(rendered, kDrift),
                .bodyHandNodeValid = true,
                .flattenedHandWorld = drift(rendered, kDrift),
                .flattenedHandValid = true,
                .claimConsumed = true,
            };
            const FrameResult claimedResult = resolveFrame(loopState, claimed);
            ok &= expectEnum("echo frame reconstructed", claimedResult.source, RawHandSource::Reconstructed);
            ok &= expectTrue("echo frame orthonormal", rock::transform_math::storedRotationOrthonormalityError(claimedResult.rawHandWorld.rotate) < 1e-5);
            if (frame == 0) {
                firstX = claimedResult.rawHandWorld.translate.x;
            } else {
                ok &= expectNear("echo frame stable", claimedResult.rawHandWorld.translate.x, firstX, 0.01f);
            }
            // FRIK renders ROCK's target: the raw hand plus a claim offset, rotation copied.
            rendered = claimedResult.rawHandWorld;
            rendered.translate.z += 3.0f;
            if (!ok) {
                break;
            }
        }
    }

    // Scope recovery must use the corrected controller input even for an
    // unclaimed hand. A displayed native hand is not a fallback input here.
    for (const bool claimed : { false, true }) {
        RelationState relation{};
        relation.firstPersonToBodyHand = identity();
        relation.valid = true;
        const auto controller = yawed(15.0f, 10.0f, 20.0f, 30.0f);
        const auto displaced = yawed(15.0f, 242.0f, 20.0f, 30.0f);
        const auto blend = yawed(6.0f, 0.0f, 0.0f, 0.0f);
        FrameInput corrected{
            .firstPersonHandWorld = controller,
            .firstPersonHandValid = true,
            .firstPersonInputCorrected = true,
            .bodyHandNodeWorld = displaced,
            .bodyHandNodeValid = true,
            .flattenedHandWorld = compose(displaced, blend),
            .flattenedHandValid = true,
            .claimConsumed = claimed,
            .calibrationAllowed = false,
        };
        const auto result = resolveFrame(relation, corrected);
        ok &= expectEnum("scope controller input reconstructed", result.source, RawHandSource::Reconstructed);
        ok &= expectNear("scope input rejects displayed displacement", result.rawHandWorld.translate.x, 10.0f, 0.001f);
        ok &= expectNear("current palm blend retained", rotationDegrees(result.rawHandWorld, compose(controller, blend)), 0.0f, 0.01f);
        ok &= expectTrue("scope output cannot recalibrate relation", relation.acceptedSamples == 0);
        corrected.firstPersonHandValid = false;
        ok &= expectFalse("missing corrected input fails closed", resolveFrame(relation, corrected).valid);
        corrected.firstPersonHandValid = true;
        relation.valid = false;
        ok &= expectFalse("missing relation fails closed", resolveFrame(relation, corrected).valid);
    }

    if (!ok) {
        std::printf("TrackedHandIsolationPolicyTests FAILED\n");
        return 1;
    }
    std::printf("TrackedHandIsolationPolicyTests passed\n");
    return 0;
}
