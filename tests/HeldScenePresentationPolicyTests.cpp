#include "physics-interaction/grab/HeldScenePresentationPolicy.h"

#include "RE/NetImmerse/NiTransform.h"

#include <cmath>
#include <cstdio>
#include <limits>

namespace
{
    bool expect(const char* label, bool condition)
    {
        if (condition) {
            return true;
        }
        std::printf("%s\n", label);
        return false;
    }

    bool near(float lhs, float rhs, float tolerance = 0.0001f)
    {
        return std::fabs(lhs - rhs) <= tolerance;
    }

    void makeIdentity(float* transform)
    {
        for (std::size_t index = 0;
             index < rock::held_scene_presentation_policy::kPredictionFloatCount;
             ++index) {
            transform[index] = 0.0f;
        }
        transform[0] = 1.0f;
        transform[5] = 1.0f;
        transform[10] = 1.0f;
    }
}

int main()
{
    using namespace rock::held_scene_presentation_policy;

    bool ok = true;

    RE::NiTransform previousTarget =
        rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    RE::NiTransform currentTarget = previousTarget;
    RE::NiTransform previousSolvedBody = previousTarget;
    currentTarget.translate.x = 3.0f;
    previousSolvedBody.translate.x = 0.25f;
    const auto targetTransport = buildTargetTransport(
        previousTarget,
        currentTarget,
        previousSolvedBody);
    ok &= expect(
        "target transport must carry the solved physical residual onto the current target",
        targetTransport.apply &&
            targetTransport.reason == TargetTransportRejectReason::None &&
            near(targetTransport.presentedWorld.translate.x, 3.25f) &&
            near(targetTransport.targetTranslationStepGameUnits, 3.0f) &&
            near(targetTransport.physicalResidualGameUnits, 0.25f) &&
            near(targetTransport.transportAdvanceGameUnits, 3.0f));

    RE::NiTransform discontinuousTarget = currentTarget;
    discontinuousTarget.translate.x = 30.0f;
    ok &= expect(
        "a discontinuous target step must fail closed",
        buildTargetTransport(
            previousTarget,
            discontinuousTarget,
            previousSolvedBody).reason ==
            TargetTransportRejectReason::ExcessiveTargetTranslationStep);

    RE::NiTransform excessiveResidual = previousSolvedBody;
    excessiveResidual.translate.x = 110.0f;
    ok &= expect(
        "an excessive physical residual must fail closed",
        buildTargetTransport(
            previousTarget,
            currentTarget,
            excessiveResidual).reason ==
            TargetTransportRejectReason::ExcessivePhysicalResidual);

    RE::NiTransform secondarySolved = previousSolvedBody;
    secondarySolved.translate.y = 12.0f;
    RE::NiTransform secondaryPresented{};
    ok &= expect("sibling body must receive the same translation without losing its offset",
        transportAssemblyBody(previousSolvedBody, targetTransport.presentedWorld, secondarySolved, secondaryPresented) &&
        near(secondaryPresented.translate.x, 3.25f) && near(secondaryPresented.translate.y, 12.0f));
    auto rotatedTarget = previousTarget;
    // 30 degrees in the stored Ni basis; long parts must orbit the primary.
    rotatedTarget.rotate.entry[0][0] = rotatedTarget.rotate.entry[1][1] = std::sqrt(3.0f) / 2.0f;
    rotatedTarget.rotate.entry[0][1] = 0.5f;
    rotatedTarget.rotate.entry[1][0] = -0.5f;
    secondarySolved = previousTarget;
    secondarySolved.translate.x = 10.0f;
    secondarySolved.scale = 2.0f;
    ok &= expect("assembly rotation must rotate the part offset and preserve its scale",
        transportAssemblyBody(previousTarget, rotatedTarget, secondarySolved, secondaryPresented) &&
        near(secondaryPresented.translate.x, 5.0f * std::sqrt(3.0f)) &&
        near(secondaryPresented.translate.y, 5.0f) && near(secondaryPresented.scale, 2.0f));
    secondarySolved.translate.x = 11.0f;
    ok &= expect("a new solved part offset must remain free to articulate",
        transportAssemblyBody(previousTarget, rotatedTarget, secondarySolved, secondaryPresented) &&
        near(secondaryPresented.translate.y, 5.5f));
    secondarySolved.translate.x = 200.0f;
    ok &= expect("an excessive distal correction must reject the assembly candidate",
        !transportAssemblyBody(previousTarget, rotatedTarget, secondarySolved, secondaryPresented));

    struct Node
    {
        Node* parent = nullptr;
        RE::NiTransform world = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
        RE::NiTransform local = world;
    };
    Node root{}, child{&root}, sibling{};
    using Pose = ScenePose<Node, RE::NiTransform>;
    Pose scene[]{ {&child, currentTarget}, {&sibling, previousTarget}, {&root, previousTarget}, {&child, currentTarget} };
    ok &= expect("nested and sibling owners must produce a valid deduplicated plan", prepareScenePoses(scene, 4));
    std::size_t rootIndex = 4, childIndex = 4, duplicates = 0;
    for (std::size_t index = 0; index < 4; ++index) {
        if (scene[index].node == &root) rootIndex = index;
        if (scene[index].node == &child && !scene[index].duplicate) childIndex = index;
        duplicates += scene[index].duplicate ? 1 : 0;
    }
    ok &= expect("parent must precede child and duplicate owner must be written once",
        rootIndex < childIndex && duplicates == 1);
    int rootWrites = 0, childWrites = 0, nativeRootRefreshes = 0, nativeChildRefreshes = 0;
    applyScenePoses(scene, 4, [&](Node* node) {
        if (node == &root) {
            ++rootWrites;
            child.world = rock::transform_math::composeTransforms(root.world, child.local);
        }
        if (node == &child) ++childWrites;
    }, [&](Node* node) {
        if (node == &root) {
            ++nativeRootRefreshes;
            ok &= expect("native refresh must observe the final child pose", near(child.world.translate.x, 3.0f));
        }
        if (node == &child) ++nativeChildRefreshes;
    });
    ok &= expect("parent subtree refresh must not overwrite the final child pose",
        rootWrites == 1 && childWrites == 1 && near(child.world.translate.x, 3.0f));
    ok &= expect("a nested owner needs only its ancestor's final native refresh",
        nativeRootRefreshes == 1 && nativeChildRefreshes == 0);
    child.world = rock::transform_math::composeTransforms(root.world, child.local);
    ok &= expect("subsequent native parent refresh must retain the corrected child local",
        near(child.world.translate.x, 3.0f));
    Pose conflictingAliases[]{ {&root, previousTarget}, {&root, currentTarget} };
    ok &= expect("conflicting body frames for one owner must fail before writing",
        !prepareScenePoses(conflictingAliases, 2));
    root.parent = &child;
    Pose cyclic[]{ {&child, currentTarget} };
    ok &= expect("cyclic ancestry must fail within the bounded traversal", !prepareScenePoses(cyclic, 1));
    root.parent = nullptr;

    Node weaponRoot{}, receiver{&weaponRoot}, slide{&weaponRoot}, magazine{&weaponRoot};
    receiver.local.translate.x = 10.0f;
    slide.local.translate.x = 20.0f;
    slide.local.translate.y = 2.0f;  // A live animated slide offset must survive.
    magazine.local.translate.z = -5.0f;
    RE::NiTransform presentedReceiver = previousTarget;
    presentedReceiver.translate.x = 13.0f;
    RE::NiTransform presentedMagazine = previousTarget;
    presentedMagazine.translate = RE::NiPoint3(3.0f, 1.0f, -5.5f);
    Pose weaponPoses[3]{{&receiver, presentedReceiver}, {&magazine, presentedMagazine}};
    std::size_t weaponPoseCount = 2;
    ok &= expect("loose weapon root must join the body-owner batch",
        appendAssemblyRootPose(weaponPoses, weaponPoseCount, 3, &weaponRoot, presentedReceiver, receiver.local) &&
        weaponPoseCount == 3 && prepareScenePoses(weaponPoses, weaponPoseCount));
    float cachedMagazineY = 0.0f;
    applyScenePoses(weaponPoses, weaponPoseCount, [&](Node* node) {
        if (node == &weaponRoot) {
            for (auto* part : {&receiver, &slide, &magazine}) {
                part->world = rock::transform_math::composeTransforms(weaponRoot.world, part->local);
            }
        }
    }, [&](Node* node) {
        if (node == &weaponRoot) cachedMagazineY = magazine.world.translate.y;
    });
    ok &= expect("mesh-only sibling slide must advance with the receiver and retain its animation",
        near(weaponRoot.world.translate.x, 3.0f) && near(slide.world.translate.x, 23.0f) &&
        near(slide.world.translate.y, 2.0f) && near(slide.local.translate.x, 20.0f));
    ok &= expect("independent magazine physics must survive the whole-root refresh",
        near(magazine.world.translate.x, 3.0f) && near(magazine.world.translate.y, 1.0f) &&
            near(magazine.world.translate.z, -5.5f));
    ok &= expect("geometry cache must see the final articulated magazine, not the initial parent pass",
        near(cachedMagazineY, 1.0f));
    const auto repeatedMagazine = rock::transform_math::composeTransforms(weaponRoot.world, magazine.local);
    ok &= expect("a later parent update must retain the magazine pose without double transport",
        near(repeatedMagazine.translate.y, 1.0f) && near(repeatedMagazine.translate.z, -5.5f));
    Pose unrelated[2]{{&sibling, presentedReceiver}};
    std::size_t unrelatedCount = 1;
    ok &= expect("a replaced or unrelated reference root must fail before scene writes",
        !appendAssemblyRootPose(unrelated, unrelatedCount, 2, &weaponRoot, presentedReceiver, receiver.local) && unrelatedCount == 1);
    Pose rootOwned[2]{{&weaponRoot, presentedReceiver}};
    std::size_t rootOwnedCount = 1;
    ok &= expect("a physics-owned reference root must retain its independent body pose",
        appendAssemblyRootPose(rootOwned, rootOwnedCount, 2, &weaponRoot, presentedReceiver, receiver.local) && rootOwnedCount == 1);

    // Nuka-Cola regression: the body lives below the reference root. Moving
    // only that owner leaves a mesh-only sibling and the root one movement
    // step behind. An intermediate non-owner parent must also be updated
    // before deriving the selected owner's new local.
    Node bottleRoot{}, neck{&bottleRoot}, bottleBody{&neck}, labelMesh{&bottleRoot};
    neck.local.translate.z = 1.0f;
    neck.world = neck.local;
    bottleBody.local.translate.z = 0.6f;
    bottleBody.world.translate.z = 1.6f;
    labelMesh.local.translate.y = 2.0f;
    labelMesh.world = labelMesh.local;
    const auto bodyInBottleRoot = bottleBody.world;
    auto presentedBottleBody = bottleBody.world;
    presentedBottleBody.translate.x = 5.0f;
    Pose bottlePoses[2]{{&bottleBody, presentedBottleBody}};
    std::size_t bottlePoseCount = 1;
    ok &= expect("ordinary reference root must join the presentation batch",
        appendAssemblyRootPose(bottlePoses, bottlePoseCount, 2, &bottleRoot, presentedBottleBody, bodyInBottleRoot) &&
        prepareScenePoses(bottlePoses, bottlePoseCount));
    RE::NiTransform cachedLabel{}, cachedBody{};
    int bottleRefreshes = 0;
    const auto updateBottle = [&](Node* node) {
        if (node == &bottleRoot) {
            neck.world = rock::transform_math::composeTransforms(bottleRoot.world, neck.local);
            bottleBody.world = rock::transform_math::composeTransforms(neck.world, bottleBody.local);
            labelMesh.world = rock::transform_math::composeTransforms(bottleRoot.world, labelMesh.local);
        }
    };
    applyScenePoses(bottlePoses, bottlePoseCount, updateBottle, [&](Node* node) {
        ++bottleRefreshes;
        updateBottle(node);
        cachedLabel = labelMesh.world;
        cachedBody = bottleBody.world;
    });
    ok &= expect("all bottle branches and native geometry caches must share the current movement step",
        bottleRefreshes == 1 && near(bottleRoot.world.translate.x, 5.0f) &&
        near(cachedLabel.translate.x, 5.0f) && near(cachedLabel.translate.y, 2.0f) &&
        near(cachedBody.translate.x, 5.0f) && near(cachedBody.translate.z, 1.6f));
    ok &= expect("intermediate parents must not double-apply movement to the body's local",
        near(bottleBody.local.translate.x, 0.0f) && near(bottleBody.local.translate.z, 0.6f));
    updateBottle(&bottleRoot);
    ok &= expect("the later native parent update must retain the same presented pose",
        near(bottleBody.world.translate.x, 5.0f) && near(labelMesh.world.translate.x, 5.0f));

    ok &= expect("earlier grab must own all shared parts, including when left updates second",
        preferEarlierTrace(4, 8) && !preferEarlierTrace(8, 4));
    ok &= expect("release must transfer ownership to the remaining registration",
        preferEarlierTrace(8, 0) && !preferEarlierTrace(0, 8));

    const auto normalTiming = evaluateTiming(true, 0.011f, 0.002f);
    ok &= expect(
        "the main scene writer must use raw frame time plus native remainder",
        normalTiming.apply &&
            normalTiming.reason == RejectReason::None &&
            near(normalTiming.rawFrameSeconds, 0.011f) &&
            near(normalTiming.nativeRemainderSeconds, 0.002f) &&
            near(normalTiming.predictionSeconds, 0.013f));
    ok &= expect(
        "the proxy and all unknown callers must retain the native transform",
        evaluateTiming(false, 0.011f, 0.0f).reason ==
            RejectReason::UnsupportedCallsite);
    ok &= expect(
        "a zero raw frame must fail closed",
        evaluateTiming(true, 0.0f, 0.0f).reason ==
            RejectReason::InvalidRawFrame);
    ok &= expect(
        "an excessive raw frame must fail closed",
        evaluateTiming(true, 0.1f, 0.0f).reason ==
            RejectReason::InvalidRawFrame);
    ok &= expect(
        "a non-finite raw frame must fail closed",
        evaluateTiming(
            true,
            (std::numeric_limits<float>::quiet_NaN)(),
            0.0f).reason == RejectReason::InvalidRawFrame);
    ok &= expect(
        "a small negative native remainder must retain the engine time sum",
        evaluateTiming(true, 0.011f, -0.001f).apply &&
            near(
                evaluateTiming(true, 0.011f, -0.001f).predictionSeconds,
                0.010f));
    ok &= expect(
        "an excessive negative native remainder must fail closed",
        evaluateTiming(true, 0.011f, -0.1f).reason ==
            RejectReason::InvalidRemainder);
    ok &= expect(
        "a non-positive total prediction must fail closed",
        evaluateTiming(true, 0.011f, -0.020f).reason ==
            RejectReason::ExcessivePredictionTime);
    ok &= expect(
        "an excessive total prediction must fail closed",
        evaluateTiming(true, 0.033f, 0.020f).reason ==
            RejectReason::ExcessivePredictionTime);

    float original[kPredictionFloatCount]{};
    float predicted[kPredictionFloatCount]{};
    float corrected[kPredictionFloatCount]{};
    makeIdentity(original);
    makeIdentity(predicted);
    original[3] = 0.25f;
    original[7] = 0.50f;
    original[11] = 0.75f;
    original[12] = 69.0f;
    original[13] = 139.0f;
    original[14] = 209.0f;
    predicted[12] = 1.0f;
    predicted[13] = 2.0f;
    predicted[14] = 3.0f;

    const auto normalTransform = buildWriterTransform(
        original,
        predicted,
        70.0f,
        corrected);
    ok &= expect(
        "a bounded native prediction must replace the scene-writer pose",
        normalTransform.apply &&
            near(normalTransform.translationDeltaGameUnits, std::sqrt(3.0f)) &&
            near(normalTransform.rotationDeltaDegrees, 0.0f) &&
            near(corrected[12], 70.0f) &&
            near(corrected[13], 140.0f) &&
            near(corrected[14], 210.0f));
    ok &= expect(
        "writer padding must remain native",
        near(corrected[3], original[3]) &&
            near(corrected[7], original[7]) &&
            near(corrected[11], original[11]));

    float nonFinitePrediction[kPredictionFloatCount]{};
    makeIdentity(nonFinitePrediction);
    nonFinitePrediction[12] =
        (std::numeric_limits<float>::infinity)();
    ok &= expect(
        "a non-finite native prediction must fail closed",
        buildWriterTransform(
            original,
            nonFinitePrediction,
            70.0f,
            corrected).reason == RejectReason::InvalidPredictedTransform);

    float distantPrediction[kPredictionFloatCount]{};
    makeIdentity(distantPrediction);
    distantPrediction[12] = 2.0f;
    distantPrediction[13] = 2.0f;
    distantPrediction[14] = 3.0f;
    ok &= expect(
        "an excessive translation correction must fail closed",
        buildWriterTransform(
            original,
            distantPrediction,
            70.0f,
            corrected).reason == RejectReason::ExcessiveTranslationDelta);

    float rotatedPrediction[kPredictionFloatCount]{};
    makeIdentity(rotatedPrediction);
    rotatedPrediction[0] = 0.0f;
    rotatedPrediction[1] = -1.0f;
    rotatedPrediction[4] = 1.0f;
    rotatedPrediction[5] = 0.0f;
    rotatedPrediction[12] = 1.0f;
    rotatedPrediction[13] = 2.0f;
    rotatedPrediction[14] = 3.0f;
    ok &= expect(
        "an excessive rotation correction must fail closed",
        buildWriterTransform(
            original,
            rotatedPrediction,
            70.0f,
            corrected).reason == RejectReason::ExcessiveRotationDelta);

    ok &= expect(
        "an invalid Havok scale must fail closed",
        buildWriterTransform(
            original,
            predicted,
            0.0f,
            corrected).reason == RejectReason::InvalidScale);

    return ok ? 0 : 1;
}
