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
    Api::inst = &api;
    renderedHand.scale = 1.0f;
    renderedHand.translate = { 1.0f, 2.0f, 3.0f };
    temporaryHand = renderedHand;
    temporaryHand.translate.x = 10.0f;
    RE::NiTransform actual{};
    bool ok = true;

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
