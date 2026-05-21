#include "physics-interaction/grab/GrabPinchPocket.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace
{
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

    bool expectReason(const char* label, const char* actual, const char* expected)
    {
        if (actual && std::strcmp(actual, expected) == 0) {
            return true;
        }
        std::printf("%s expected %s got %s\n", label, expected, actual ? actual : "(null)");
        return false;
    }

    bool expectNear(const char* label, float actual, float expected, float epsilon = 0.0001f)
    {
        if (std::fabs(actual - expected) <= epsilon) {
            return true;
        }
        std::printf("%s expected %.4f got %.4f\n", label, expected, actual);
        return false;
    }

    rock::grab_pinch_pocket_policy::MeshExtentMetrics bounds(float x, float y, float z)
    {
        return rock::grab_pinch_pocket_policy::computeMeshExtentsFromBounds(
            RE::NiPoint3{ 0.0f, 0.0f, 0.0f },
            RE::NiPoint3{ x, y, z },
            1.0f);
    }

    rock::grab_pinch_pocket_policy::ObjectDecisionInput validInput(
        rock::grab_pinch_pocket_policy::MeshExtentMetrics mesh)
    {
        return rock::grab_pinch_pocket_policy::ObjectDecisionInput{
            .config = rock::grab_pinch_pocket_policy::Config{},
            .mesh = mesh,
            .closeGrab = true,
            .handPocketOnlyGrab = false,
            .authoredGrabNode = false,
            .looseWeaponGrab = false,
            .ownerMatchesResolvedBody = true,
            .hasFingerSnapshot = true,
            .hasPinchSurface = true,
            .multipleAcceptedBodies = false,
            .thumbIndexGapGameUnits = 6.0f,
            .pocketToSurfaceDistanceGameUnits = 2.0f,
        };
    }
}

int main()
{
    using namespace rock::grab_pinch_pocket_policy;

    bool ok = true;

    auto compact = evaluateObject(validInput(bounds(7.0f, 5.0f, 2.0f)));
    ok &= expectTrue("compact object accepted", compact.accept);
    ok &= expectTrue("compact object flag", compact.compactObject);
    ok &= expectReason("compact object reason", compact.reason, "pinchCompact");

    auto thinRod = evaluateObject(validInput(bounds(17.5f, 3.5f, 2.0f)));
    ok &= expectTrue("short thin rod accepted", thinRod.accept);
    ok &= expectTrue("short thin rod flag", thinRod.thinRod);
    ok &= expectReason("short thin rod reason", thinRod.reason, "pinchThinRod");

    auto longRod = evaluateObject(validInput(bounds(30.0f, 3.0f, 2.0f)));
    ok &= expectFalse("long rod rejected", longRod.accept);
    ok &= expectReason("long rod reason", longRod.reason, "objectTooLarge");

    auto largeProp = evaluateObject(validInput(bounds(16.0f, 12.0f, 8.0f)));
    ok &= expectFalse("large prop rejected", largeProp.accept);
    ok &= expectReason("large prop reason", largeProp.reason, "objectTooLarge");

    auto farGrab = validInput(bounds(5.0f, 4.0f, 2.0f));
    farGrab.closeGrab = false;
    auto decision = evaluateObject(farGrab);
    ok &= expectFalse("far grab rejected", decision.accept);
    ok &= expectReason("far grab reason", decision.reason, "notCloseGrab");

    auto handPocketOnly = validInput(bounds(5.0f, 4.0f, 2.0f));
    handPocketOnly.handPocketOnlyGrab = true;
    decision = evaluateObject(handPocketOnly);
    ok &= expectFalse("hand-pocket-only rejected", decision.accept);
    ok &= expectReason("hand-pocket-only reason", decision.reason, "handPocketOnlyTarget");

    auto looseWeapon = validInput(bounds(5.0f, 4.0f, 2.0f));
    looseWeapon.looseWeaponGrab = true;
    decision = evaluateObject(looseWeapon);
    ok &= expectFalse("loose weapon rejected", decision.accept);
    ok &= expectReason("loose weapon reason", decision.reason, "looseWeapon");

    auto multiBody = validInput(bounds(5.0f, 4.0f, 2.0f));
    multiBody.multipleAcceptedBodies = true;
    decision = evaluateObject(multiBody);
    ok &= expectFalse("multi-body rejected", decision.accept);
    ok &= expectReason("multi-body reason", decision.reason, "multiBody");

    auto ownerMismatch = validInput(bounds(5.0f, 4.0f, 2.0f));
    ownerMismatch.ownerMatchesResolvedBody = false;
    decision = evaluateObject(ownerMismatch);
    ok &= expectFalse("owner mismatch rejected", decision.accept);
    ok &= expectReason("owner mismatch reason", decision.reason, "ownerMismatch");

    auto missingSnapshot = validInput(bounds(5.0f, 4.0f, 2.0f));
    missingSnapshot.hasFingerSnapshot = false;
    decision = evaluateObject(missingSnapshot);
    ok &= expectFalse("missing finger snapshot rejected", decision.accept);
    ok &= expectReason("missing finger snapshot reason", decision.reason, "missingFingerSnapshot");

    auto gapTooWide = validInput(bounds(5.0f, 4.0f, 2.0f));
    gapTooWide.thumbIndexGapGameUnits = 20.0f;
    decision = evaluateObject(gapTooWide);
    ok &= expectFalse("wide thumb-index gap rejected", decision.accept);
    ok &= expectReason("wide thumb-index gap reason", decision.reason, "fingerGapRejected");

    auto surfaceTooFar = validInput(bounds(5.0f, 4.0f, 2.0f));
    surfaceTooFar.pocketToSurfaceDistanceGameUnits = 12.0f;
    decision = evaluateObject(surfaceTooFar);
    ok &= expectFalse("far surface rejected", decision.accept);
    ok &= expectReason("far surface reason", decision.reason, "surfaceTooFarFromPocket");

    auto noMesh = validInput(MeshExtentMetrics{});
    decision = evaluateObject(noMesh);
    ok &= expectFalse("missing mesh rejected", decision.accept);
    ok &= expectReason("missing mesh reason", decision.reason, "noMeshExtents");

    Config detectionConfig{};
    detectionConfig.detectionDirectionHandspace = RE::NiPoint3{ 0.0f, 3.0f, 4.0f };
    detectionConfig.detectionAxisBlend = 2.0f;
    auto sanitized = sanitizeConfig(detectionConfig);
    ok &= expectNear("pinch detection direction x", sanitized.detectionDirectionHandspace.x, 0.0f);
    ok &= expectNear("pinch detection direction y", sanitized.detectionDirectionHandspace.y, 0.6f);
    ok &= expectNear("pinch detection direction z", sanitized.detectionDirectionHandspace.z, 0.8f);
    ok &= expectNear("pinch detection axis blend clamped", sanitized.detectionAxisBlend, 1.0f);

    detectionConfig = Config{};
    detectionConfig.detectionDirectionHandspace = RE::NiPoint3{ 0.0f, 0.0f, 0.0f };
    sanitized = sanitizeConfig(detectionConfig);
    ok &= expectNear("pinch detection fallback x", sanitized.detectionDirectionHandspace.x, kDefaultDetectionDirectionHandspaceX);
    ok &= expectNear("pinch detection fallback y", sanitized.detectionDirectionHandspace.y, kDefaultDetectionDirectionHandspaceY);
    ok &= expectNear("pinch detection fallback z", sanitized.detectionDirectionHandspace.z, kDefaultDetectionDirectionHandspaceZ);

    return ok ? 0 : 1;
}
