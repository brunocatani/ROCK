#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/AuthoredWeaponGripPose.h"
#include "physics-interaction/weapon/WeaponGripTransfer.h"
#include "physics-interaction/weapon/WeaponGripCalibration.h"
#include "physics-interaction/weapon/WeaponAuthority.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <random>
#include <vector>

namespace rock::root_flattened_finger_skeleton_runtime
{
    // A capture with a supplied snapshot must never query the game scene again.
    bool queriedLiveScene = false;
    bool resolveLiveFingerSkeletonSnapshot(bool, Snapshot&, std::string*, SkeletonBoneCaptureSpace)
    {
        queriedLiveScene = true;
        return false;
    }
}

namespace
{
    struct TestVector
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    struct TestTransform
    {
        struct
        {
            float entry[3][3]{};
        } rotate{};
        TestVector translate{};
        float scale = 1.0f;
    };

    bool expectBool(const char* name, bool actual, bool expected)
    {
        if (actual != expected) {
            std::printf("%s expected %s got %s\n", name, expected ? "true" : "false", actual ? "true" : "false");
            return false;
        }
        return true;
    }

}

static bool testPublicationSafety()
{
    using namespace rock::grab_finger_local_transform_math;
    bool ok = true;
    TestTransform hiddenScaleTransform{};
    hiddenScaleTransform.rotate.entry[0][0] = 1.0f;
    hiddenScaleTransform.rotate.entry[1][1] = 1.0f;
    hiddenScaleTransform.rotate.entry[2][2] = 1.0f;
    hiddenScaleTransform.scale = 0.00001f;
    ok &= expectBool("hidden FRIK node scale still has usable rotation",
        sceneTransformHasUsableBasis(hiddenScaleTransform), true);

    TestTransform safePublishedFingerTransform = hiddenScaleTransform;
    safePublishedFingerTransform.scale = 1.0f;
    ok &= expectBool("rigid local finger transform is safe to publish",
        inspectFingerLocalTransformForPublication(safePublishedFingerTransform) == FingerLocalTransformSafetyFailure::None, true);

    TestTransform hiddenPublishedFingerTransform = safePublishedFingerTransform;
    hiddenPublishedFingerTransform.scale = 0.00001f;
    ok &= expectBool("near-zero local finger scale is rejected before publication",
        inspectFingerLocalTransformForPublication(hiddenPublishedFingerTransform) == FingerLocalTransformSafetyFailure::ScaleMagnitude, true);

    TestTransform giantPublishedFingerTransform = safePublishedFingerTransform;
    giantPublishedFingerTransform.scale = 1000.0f;
    ok &= expectBool("giant local finger scale is rejected before publication",
        inspectFingerLocalTransformForPublication(giantPublishedFingerTransform) == FingerLocalTransformSafetyFailure::ScaleMagnitude, true);

    TestTransform distantPublishedFingerTransform = safePublishedFingerTransform;
    distantPublishedFingerTransform.translate.x = kMaxPublishedFingerLocalTranslationGameUnits + 1.0f;
    ok &= expectBool("world-sized local finger translation is rejected before publication",
        inspectFingerLocalTransformForPublication(distantPublishedFingerTransform) == FingerLocalTransformSafetyFailure::TranslationMagnitude, true);

    TestTransform shearedPublishedFingerTransform = safePublishedFingerTransform;
    shearedPublishedFingerTransform.rotate.entry[0][1] = 0.5f;
    ok &= expectBool("sheared local finger basis is rejected before publication",
        inspectFingerLocalTransformForPublication(shearedPublishedFingerTransform) == FingerLocalTransformSafetyFailure::RotationNorm, true);

    TestTransform reflectedPublishedFingerTransform = safePublishedFingerTransform;
    reflectedPublishedFingerTransform.rotate.entry[2][2] = -1.0f;
    ok &= expectBool("reflected local finger basis is rejected before publication",
        inspectFingerLocalTransformForPublication(reflectedPublishedFingerTransform) == FingerLocalTransformSafetyFailure::RotationDeterminant, true);

    return ok;
}

int main()
{
    bool ok = true;
    ok &= testPublicationSafety();
    return ok ? 0 : 1;
}
