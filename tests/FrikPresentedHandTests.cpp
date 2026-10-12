#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"

#include <cstdio>
#include <cstring>
#include <limits>

namespace
{
    using Api = frik::api::FRIKApiV2;
    RE::NiTransform renderedHand{};
    RE::NiTransform temporaryHand{};
    bool skeletonReady = true;
    bool boneAvailable = true;
    unsigned trackedReads = 0;
    const char* requestedBone = nullptr;
    bool poseRegistered = false, acceptFingerLocals = true;
    unsigned poseRegistrations = 0, fingerPublications = 0, clears = 0;
    Api::FingerLocalTransformOverride receivedFingers{};

    bool FRIK_CALL registerPose(const char*, Api::Hand, const Api::HandPoseData&, int)
    {
        ++poseRegistrations;
        poseRegistered = true;
        return true;
    }
    Api::HandPoseTagState FRIK_CALL poseState(const char*, Api::Hand)
    {
        return poseRegistered ? Api::HandPoseTagState::Active : Api::HandPoseTagState::None;
    }
    bool FRIK_CALL publishLocals(const char*, Api::Hand, const Api::FingerLocalTransformOverride* locals, int)
    {
        ++fingerPublications;
        // Published FRIK 2.3 contract, and the exact rejection observed in the
        // 2026-10-12 12:03:21.932 runtime log: locals require a tagged pose.
        if (!poseRegistered || !acceptFingerLocals) return false;
        receivedFingers = *locals;
        return true;
    }
    bool FRIK_CALL clearPose(const char*, Api::Hand)
    {
        ++clears;
        poseRegistered = false;
        return true;
    }

    bool FRIK_CALL readBone(const char* name, RE::NiTransform* out)
    {
        requestedBone = name;
        *out = renderedHand;
        return skeletonReady && boneAvailable;
    }

    bool FRIK_CALL readTracked(Api::Hand, Api::TrackedHandKind, RE::NiTransform* out)
    {
        ++trackedReads;
        *out = temporaryHand;
        return skeletonReady;
    }

    bool expect(const char* label, bool result)
    {
        if (!result) {
            std::printf("FAILED: %s\n", label);
        }
        return result;
    }
}

int main()
{
    namespace bridge = rock::frik_visual_authority;
    Api api{};
    api.getBoneWorldTransform = &readBone;
    api.getTrackedHandTransform = &readTracked;
    api.setHandPoseCustom = &registerPose;
    api.getHandPoseSetTagState = &poseState;
    api.setHandPoseCustomLocalTransforms = &publishLocals;
    api.clearHandPose = &clearPose;
    Api::inst = &api;
    renderedHand.scale = 1.0f;
    renderedHand.translate = { 1.0f, 2.0f, 3.0f };
    temporaryHand = renderedHand;
    temporaryHand.translate.x = 10.0f;
    RE::NiTransform actual{};
    bool ok = true;

    Api::FingerLocalTransformOverride authored{};
    authored.enabledMask = 0x7FFF;
    for (auto& local : authored.localTransforms) local = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    authored.localTransforms[4].translate = { 0.25f, -0.5f, 0.75f };
    constexpr const char* tag = "secondary-pose-contract-test";
    ok &= expect("authored locals acquire their required tagged pose", bridge::setAuthoredFingerPose(tag, Api::Hand::Left, authored, 100) &&
        poseRegistrations == 1 && fingerPublications == 1 && receivedFingers.enabledMask == authored.enabledMask &&
        receivedFingers.localTransforms[4].translate.z == 0.75f);
    ok &= expect("unchanged authored pose does not re-register each frame", bridge::setAuthoredFingerPose(tag, Api::Hand::Left, authored, 100) &&
        poseRegistrations == 1 && fingerPublications == 1);
    // A released skeleton erases provider registrations behind the client's
    // cache. The next publication must reacquire before sending bone locals.
    poseRegistered = false;
    ok &= expect("skeleton recreation restores the required pose", bridge::setAuthoredFingerPose(tag, Api::Hand::Left, authored, 100) &&
        poseRegistrations == 2 && fingerPublications == 2);
    acceptFingerLocals = false;
    authored.localTransforms[4].translate.z = 0.8f;
    ok &= expect("partial publication cannot leave a scalar pose behind", !bridge::setAuthoredFingerPose(tag, Api::Hand::Left, authored, 100) &&
        !poseRegistered && clears == 1);
    acceptFingerLocals = true;
    ok &= expect("publication recovers after a provider refusal", bridge::setAuthoredFingerPose(tag, Api::Hand::Left, authored, 100) && poseRegistered);
    (void)bridge::clearHandPose(tag, Api::Hand::Left);

    // BeforeArmSolve during left carry: the live first-person hand holds
    // re-glue, while the body tree retains the last completed presentation.
    ok &= expect("early left read retains the completed presentation",
        bridge::tryGetPresentedHandWorldTransform(true, actual) && actual.translate.x == 1.0f &&
            std::strcmp(requestedBone, "LArm_Hand") == 0);
    ok &= expect("right hand uses the matching body bone",
        bridge::tryGetPresentedHandWorldTransform(false, actual) &&
            std::strcmp(requestedBone, "RArm_Hand") == 0);

    // World final publishes a new rendered pose; no explicit ROCK cache can
    // keep the old one alive or substitute the controller for a claimed hand.
    renderedHand.translate.x = 4.0f;
    ok &= expect("world final becomes visible on the next read",
        bridge::tryGetPresentedHandWorldTransform(true, actual) && actual.translate.x == 4.0f);
    ok &= expect("presentation never samples transient tracked input", trackedReads == 0);

    skeletonReady = false;
    ok &= expect("released skeleton fails closed", !bridge::tryGetPresentedHandWorldTransform(true, actual));
    skeletonReady = true;
    boneAvailable = false;
    ok &= expect("missing hand fails closed", !bridge::tryGetPresentedHandWorldTransform(true, actual));
    boneAvailable = true;
    renderedHand.translate.x = (std::numeric_limits<float>::quiet_NaN)();
    ok &= expect("invalid presentation fails closed", !bridge::tryGetPresentedHandWorldTransform(true, actual));
    Api::inst = nullptr;
    ok &= expect("missing provider fails closed", !bridge::tryGetPresentedHandWorldTransform(true, actual));
    return ok ? 0 : 1;
}
