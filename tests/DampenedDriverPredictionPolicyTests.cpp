#include "physics-interaction/visual/DampenedDriverPredictionPolicy.h"
#include "physics-interaction/visual/HandWorldClaimRegistryPolicy.h"

#include <cmath>
#include <cstdio>

namespace
{
    using namespace rock::dampened_driver_prediction_policy;

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

    RE::NiTransform identity()
    {
        return rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    }

    RE::NiTransform translated(float x, float y, float z)
    {
        RE::NiTransform t = identity();
        t.translate = RE::NiPoint3{ x, y, z };
        return t;
    }

    RE::NiTransform yawed(float degrees)
    {
        const float radians = degrees * 0.017453292519943295f;
        const float c = std::cos(radians);
        const float s = std::sin(radians);
        RE::NiTransform t = identity();
        t.rotate.entry[0][0] = c;
        t.rotate.entry[0][1] = -s;
        t.rotate.entry[1][0] = s;
        t.rotate.entry[1][1] = c;
        return t;
    }
}

int main()
{
    bool ok = true;
    const DampenFactors frikDefault{ .enabled = true, .translation = 0.6f, .rotation = 0.6f };

    // Translation: FRIK keeps 60% of the previous value, after removing the camera step.
    {
        const RE::NiTransform predicted = predictDampened(translated(10.0f, 0.0f, 0.0f), translated(0.0f, 0.0f, 0.0f), RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, frikDefault);
        // raw - 0.6 * (raw - previous - camera) = 10 - 0.6 * 9
        ok &= expectNear("translation dampened", predicted.translate.x, 4.6f, 0.0001f);
        ok &= expectNear("translation y untouched", predicted.translate.y, 0.0f, 0.0001f);
        const RE::NiTransform riding = predictDampened(translated(10.0f, 0.0f, 0.0f), translated(0.0f, 0.0f, 0.0f), RE::NiPoint3{ 10.0f, 0.0f, 0.0f }, frikDefault);
        ok &= expectNear("player movement is not dampened", riding.translate.x, 10.0f, 0.0001f);
    }

    // Rotation: slerp from the previous dampened world toward the raw world by 1 - factor.
    {
        const RE::NiTransform predicted = predictDampened(yawed(90.0f), identity(), RE::NiPoint3{}, frikDefault);
        ok &= expectNear("rotation dampened to 40% of the step", rock::hand_visual_lerp_math::rotationDistanceDegrees(identity(), predicted), 36.0f, 0.05f);
        ok &= expectNear("rotation stays short of the raw", rock::hand_visual_lerp_math::rotationDistanceDegrees(predicted, yawed(90.0f)), 54.0f, 0.05f);
        ok &= expectNear("prediction is a rotation", static_cast<float>(rock::transform_math::storedRotationOrthonormalityError(predicted.rotate)), 0.0f, 0.001f);
        const DampenFactors translationOnly{ .enabled = true, .translation = 0.6f, .rotation = 0.0f };
        ok &= expectNear("zero rotation factor keeps the raw rotation", rock::hand_visual_lerp_math::rotationDistanceDegrees(predictDampened(yawed(90.0f), identity(), RE::NiPoint3{}, translationOnly), yawed(90.0f)), 0.0f, 0.001f);
    }

    // Disabled factors return the raw world; configuration selection follows the vanilla scope menu.
    {
        const DampenFactors off{};
        const RE::NiTransform raw = translated(10.0f, 2.0f, 3.0f);
        const RE::NiTransform predicted = predictDampened(raw, translated(0.0f, 0.0f, 0.0f), RE::NiPoint3{}, off);
        ok &= expectNear("disabled keeps raw x", predicted.translate.x, 10.0f, 0.0001f);
        ok &= expectNear("disabled keeps raw y", predicted.translate.y, 2.0f, 0.0001f);

        FrikDampenConfig config{};
        config.normal = frikDefault;
        config.vanillaScope = DampenFactors{ .enabled = true, .translation = 0.2f, .rotation = 0.2f };
        ok &= expectTrue("invalid config selects nothing", !selectFactors(config, false).enabled);
        config.valid = true;
        ok &= expectNear("normal factors outside the scope", selectFactors(config, false).translation, 0.6f, 0.0001f);
        ok &= expectNear("scope factors in the scope", selectFactors(config, true).translation, 0.2f, 0.0001f);
        ok &= expectTrue("factor bounds", isUsableFactor(0.0f) && isUsableFactor(0.95f) && !isUsableFactor(1.0f) && !isUsableFactor(-0.1f) && !isUsableFactor(std::nanf("")));
    }

    // Walking/turning in a scope must not carry the old filtered world into
    // the first resumed frame. The next observation restores normal filtering.
    for (const float direction : { -1.0f, 1.0f }) {
        ObservedDriver history{
            .world = translated(100.0f, 200.0f, 3.0f),
            .camera = RE::NiPoint3{ 90.0f, 180.0f, 0.0f },
            .sequence = 100,
            .dampeningEnabled = true,
            .valid = true,
        };
        const DampenFactors off{};
        RE::NiTransform raw = history.world;
        RE::NiPoint3 camera = history.camera;
        for (std::uint64_t sequence = 101; sequence <= 400; ++sequence) {
            camera.y += direction * 0.5f;
            raw = yawed(static_cast<float>(sequence - 100) * 0.3f);
            raw.translate = camera + RE::NiPoint3{ 10.0f, 20.0f, 3.0f };
            const auto disabled = predictFromObservation(raw, camera, sequence, off, history);
            ok &= expectTrue("scope uses raw driver", disabled.mode == PredictionMode::RawDisabled);
            // Simulated provider observation; production reads the live node.
            history = { raw, camera, sequence, false, true };
        }

        camera.y += direction * 0.5f;
        raw.translate.y += direction * 0.5f;
        raw.translate.x += 6.0f;
        raw.rotate = yawed(135.0f).rotate;
        const auto resumed = predictFromObservation(raw, camera, 401, frikDefault, history);
        ok &= expectTrue("resumed mode", resumed.mode == PredictionMode::RawResuming);
        ok &= expectNear("resuming follows current hand", resumed.world.translate.x, raw.translate.x, 0.0001f);
        ok &= expectNear("resuming follows locomotion", resumed.world.translate.y, raw.translate.y, 0.0001f);
        ok &= expectNear("resuming follows current rotation", rock::hand_visual_lerp_math::rotationDistanceDegrees(resumed.world, raw), 0.0f, 0.01f);

        history = { raw, camera, 401, true, true };
        camera.y += direction * 0.5f;
        raw.translate.y += direction * 0.5f;
        raw.translate.x += 10.0f;
        const auto continuous = predictFromObservation(raw, camera, 402, frikDefault, history);
        ok &= expectTrue("normal filtering resumes from observation", continuous.mode == PredictionMode::Dampened);
        ok &= expectNear("normal hand step is dampened", continuous.world.translate.x, history.world.translate.x + 4.0f, 0.0001f);
        ok &= expectNear("camera step stays uncompromised", continuous.world.translate.y, raw.translate.y, 0.0001f);

        const DampenFactors scoped{ .enabled = true, .translation = 0.2f, .rotation = 0.2f };
        const auto enabledScope = predictFromObservation(raw, camera, 402, scoped, history);
        ok &= expectTrue("enabled scopes retain continuous filtering", enabledScope.mode == PredictionMode::Dampened);
        ok &= expectNear("enabled scope uses its own factor", enabledScope.world.translate.x, history.world.translate.x + 8.0f, 0.0001f);
    }

    // Stale, duplicate, rewound, absent, or unobserved history must never be
    // combined with a current-frame camera delta.
    {
        ObservedDriver history{ translated(-1000.0f, 0.0f, 0.0f), RE::NiPoint3{}, 200, true, true };
        const auto raw = yawed(90.0f);
        for (const std::uint64_t sequence : { 0ull, 199ull, 200ull, 202ull, 313ull }) {
            const auto skipped = predictFromObservation(raw, RE::NiPoint3{}, sequence, frikDefault, history);
            ok &= expectTrue("discontinuous history rejected", skipped.mode == PredictionMode::RawDiscontinuous);
            ok &= expectNear("discontinuity keeps raw translation", skipped.world.translate.x, raw.translate.x, 0.0001f);
            ok &= expectNear("discontinuity keeps raw rotation", rock::hand_visual_lerp_math::rotationDistanceDegrees(skipped.world, raw), 0.0f, 0.01f);
        }
        history.valid = false;
        ok &= expectTrue("failed observation cannot seed prediction", predictFromObservation(raw, RE::NiPoint3{}, 201, frikDefault, history).mode == PredictionMode::RawDiscontinuous);
        history = {};
        ok &= expectTrue("skeleton reset clears continuity", predictFromObservation(raw, RE::NiPoint3{}, 201, frikDefault, history).mode == PredictionMode::RawDiscontinuous);
    }

    // A displaced native output is never the next controller history after
    // a scope pause. Both directions retain movement and the existing factor.
    for (const float direction : { -1.0f, 1.0f }) {
        ScopeInputContinuity continuity{};
        continuity.beginPass(true, true, false);
        ok &= expectTrue("scope suspension arms continuity", continuity.suspendedByScope && !continuity.recovering);
        ObservedDriver history{ translated(100.0f, 0.0f, 0.0f), {}, 100, false, true };
        RE::NiTransform native = history.world;
        native.translate.x -= direction * 232.0f;
        auto camera = RE::NiPoint3{};
        auto raw = history.world;
        for (std::uint64_t frame = 101; frame <= 110; ++frame) {
            camera.y += direction * 2.0f;
            raw.translate.y += direction * 2.0f;
            continuity.beginPass(false, true, false);
            const auto prediction = predictFromObservation(raw, camera, frame, frikDefault, history);
            native.translate.y = raw.translate.y;
            ok &= expectTrue("scope native drift rejected", !continuity.acceptNative(prediction.world, native));
            ok &= expectNear("accepted hand stays with current controller x", prediction.world.translate.x, raw.translate.x, 0.0001f);
            ok &= expectNear("accepted hand follows player movement", prediction.world.translate.y, raw.translate.y, 0.0001f);
            history = { prediction.world, camera, frame, true, true };
            native.translate.x = raw.translate.x + 0.6f * (native.translate.x - raw.translate.x);
        }
        ok &= expectTrue("matching observation ends recovery", continuity.acceptNative(raw, raw) && !continuity.recovering);
        continuity.beginPass(false, true, false);
        ok &= expectTrue("normal native motion is not clamped", continuity.acceptNative(raw, translated(900.0f, 0.0f, 0.0f)));
        continuity.beginPass(true, true, true);
        continuity.beginPass(false, true, true);
        ok &= expectTrue("enabled scope dampening stays native", !continuity.recovering);
        continuity.beginPass(true, true, false);
        continuity.beginPass(false, true, false);
        ok &= expectTrue("rapid second exit rearms", continuity.recovering);
        continuity.beginPass(true, true, false);
        ok &= expectTrue("scope reentry clears previous recovery", !continuity.recovering);
        continuity.beginPass(false, false, false);
        ok &= expectTrue("disabled normal dampening clears recovery", !continuity.recovering);
        // The menu can change between pre-FRIK and post-FRIK observation.
        continuity.beginPass(true, true, false);
        continuity.beginPass(false, true, false);
        ok &= expectTrue("late close protects this observation", !continuity.acceptNative(raw, translated(-232.0f, 0.0f, 0.0f)));
        auto rotated = raw;
        rotated.rotate = yawed(15.0f).rotate;
        ok &= expectTrue("matching translation alone cannot end recovery", !continuity.acceptNative(raw, rotated));
    }

    // Correct native first-person copies through the same driver relation;
    // never move a valid claim toward the observed scope reset.
    {
        namespace registry = rock::hand_world_claim_registry_policy;
        const auto driver = translated(100.0f, 200.0f, 0.0f);
        const auto hand = translated(103.0f, 204.0f, 5.0f);
        const auto relation = captureDriverHandRelation(driver, hand);
        ok &= expectTrue("hand relation captured", relation.valid);
        const auto movedDriver = translated(110.0f, 220.0f, 0.0f);
        const auto movedHand = rock::transform_math::composeTransforms(movedDriver, relation.handInDriver);
        ok &= expectNear("local hand offset survives movement", movedHand.translate.x, 113.0f, 0.0001f);
        ok &= expectNear("local hand height survives movement", movedHand.translate.z, 5.0f, 0.0001f);
        ok &= expectTrue("displaced relation cannot be learned", !captureDriverHandRelation(driver, translated(332.0f, 200.0f, 0.0f)).valid);

        registry::Registry claims{};
        registry::DriverFrame frame{};
        frame.sequence = 100;
        frame.hands[registry::handIndex(true)] = { driver, true };
        // A moving input frame rebases the claim by the controller's 10/20
        // step; the displaced native pose is deliberately not its driver.
        registry::Claim claim{};
        claim.valid = true;
        claim.isLeft = true;
        claim.driver = registry::RebaseDriver::LeftHand;
        claim.target = hand;
        claim.driverAtPublish = { driver, true };
        claims.claims[0] = claim;
        frame.hands[registry::handIndex(true)] = { movedDriver, true };
        registry::reanchorClaims(claims, frame);
        ok &= expectNear("claim follows accepted input x", claims.claims[0].target.translate.x, 113.0f, 0.001f);
        ok &= expectNear("claim follows accepted input y", claims.claims[0].target.translate.y, 224.0f, 0.001f);
    }

    if (!ok) {
        std::printf("DampenedDriverPredictionPolicyTests FAILED\n");
        return 1;
    }
    std::printf("DampenedDriverPredictionPolicyTests passed\n");
    return 0;
}
