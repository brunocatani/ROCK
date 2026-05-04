#include <cstdio>
#include <cstring>

#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/native/PhysicsScale.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/WeaponAuthority.h"

struct TestMatrix
{
    float entry[3][4]{};
};

namespace
{
    bool expectBool(const char* label, bool actual, bool expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected %d got %d\n", label, expected ? 1 : 0, actual ? 1 : 0);
        return false;
    }

    bool expectFloat(const char* label, float actual, float expected)
    {
        constexpr float kEpsilon = 0.0001f;
        const float delta = actual >= expected ? actual - expected : expected - actual;
        if (delta <= kEpsilon) {
            return true;
        }

        std::printf("%s expected %.4f got %.4f\n", label, expected, actual);
        return false;
    }

    bool expectSource(
        const char* label,
        rock::body_frame::BodyFrameSource actual,
        rock::body_frame::BodyFrameSource expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected %s got %s\n",
            label,
            rock::body_frame::bodyFrameSourceName(expected),
            rock::body_frame::bodyFrameSourceName(actual));
        return false;
    }

    bool expectString(const char* label, const char* actual, const char* expected)
    {
        if (actual && std::strcmp(actual, expected) == 0) {
            return true;
        }

        std::printf("%s expected '%s' got '%s'\n", label, expected, actual ? actual : "(null)");
        return false;
    }
}

int main()
{
    namespace body_frame = rock::body_frame;

    bool ok = true;

    ok &= expectBool("motion index zero is invalid", body_frame::hasUsableMotionIndex(0), false);
    ok &= expectBool("normal motion index is valid", body_frame::hasUsableMotionIndex(42), true);
    ok &= expectBool("free motion index is invalid", body_frame::hasUsableMotionIndex(0x7FFF'FFFF), false);
    ok &= expectBool("out-of-range motion index is invalid", body_frame::hasUsableMotionIndex(4096), false);

    ok &= expectSource("collider frame uses live motion transform when available",
        body_frame::chooseColliderFrameSource(true, true),
        body_frame::BodyFrameSource::MotionCenterOfMass);
    ok &= expectSource("collider frame falls back to body transform without motion",
        body_frame::chooseColliderFrameSource(true, false),
        body_frame::BodyFrameSource::BodyTransform);
    ok &= expectSource("collider frame can fall back to motion only without body transform",
        body_frame::chooseColliderFrameSource(false, true),
        body_frame::BodyFrameSource::MotionCenterOfMass);

    ok &= expectSource("live grab frame uses motion transform when motion exists",
        body_frame::chooseLiveBodyFrameSource(true, true),
        body_frame::BodyFrameSource::MotionCenterOfMass);
    ok &= expectSource("live grab frame falls back to body transform without motion",
        body_frame::chooseLiveBodyFrameSource(true, false),
        body_frame::BodyFrameSource::BodyTransform);
    ok &= expectSource("live grab frame rejects missing sources",
        body_frame::chooseLiveBodyFrameSource(false, false),
        body_frame::BodyFrameSource::Fallback);
    ok &= expectString("motion source code is compact for telemetry",
        body_frame::bodyFrameSourceCode(body_frame::BodyFrameSource::MotionCenterOfMass),
        "MOTION");

    ok &= expectBool("body slot rejects invalid sentinel",
        body_frame::bodySlotCanBeRead(0x7FFF'FFFFu, 64),
        false);
    ok &= expectBool("body slot rejects ids beyond readable high water",
        body_frame::bodySlotCanBeRead(65, 64),
        false);
    ok &= expectBool("body slot rejects suspicious high water",
        body_frame::bodySlotCanBeRead(42, 9000),
        false);
    ok &= expectBool("body slot accepts id at high water",
        body_frame::bodySlotCanBeRead(64, 64),
        true);

    namespace physics_scale = rock::physics_scale;
    const float engineHavokToGame = 69.99125f;
    const auto engineScale = physics_scale::makeSnapshot(1.0f / engineHavokToGame, engineHavokToGame, 71.0f, 7);
    ok &= expectFloat("physics scale reciprocal drift is zero for engine pair", physics_scale::reciprocalDriftGameUnits(engineScale), 0.0f);

    const auto mismatchedScale = physics_scale::makeSnapshot(1.0f / 70.0f, engineHavokToGame, 71.0f, 8);
    ok &= expectBool("physics scale reciprocal mismatch crosses warning threshold", physics_scale::hasReciprocalMismatch(mismatchedScale, 0.001f), true);

    namespace weapon_lifecycle = rock::weapon_authority_lifecycle_policy;
    const auto scaleInvalidation = weapon_lifecycle::evaluateGeneratedCollisionScaleInvalidation(true, 0x1234, 0);
    ok &= expectBool("scale invalidation destroys existing generated weapon bodies", scaleInvalidation.destroyExistingBodies, true);
    ok &= expectBool("scale invalidation marks generated weapon pending", scaleInvalidation.pending, true);
    ok &= expectBool("scale invalidation resets generated weapon cached settings", scaleInvalidation.resetCachedSettings, true);
    ok &= expectFloat("scale invalidation preserves pending generation key", static_cast<float>(scaleInvalidation.pendingKey), static_cast<float>(0x1234));

    const float zQuarterTurn[4]{ 0.0f, 0.0f, 0.7071067f, 0.7071067f };
    const auto motionRotation = rock::transform_math::havokQuaternionToNiRows<TestMatrix>(zQuarterTurn);
    ok &= expectFloat("motion quaternion row 0 col 0", motionRotation.entry[0][0], 0.0f);
    ok &= expectFloat("motion quaternion row 0 col 1", motionRotation.entry[0][1], -1.0f);
    ok &= expectFloat("motion quaternion row 1 col 0", motionRotation.entry[1][0], 1.0f);
    ok &= expectFloat("motion quaternion row 1 col 1", motionRotation.entry[1][1], 0.0f);
    ok &= expectFloat("motion quaternion row 2 col 2", motionRotation.entry[2][2], 1.0f);

    const RE::NiPoint3 selectionOrigin{ 100.0f, 50.0f, -20.0f };
    const RE::NiPoint3 hitPoint{ 110.0f, 50.0f, -20.0f };
    const RE::NiPoint3 bodyTransform{ 130.0f, 50.0f, -20.0f };
    const RE::NiPoint3 ownerNode{ 140.0f, 50.0f, -20.0f };
    const RE::NiPoint3 motionCom{ 250.0f, 50.0f, -20.0f };
    const RE::NiPoint3 fallback{ 300.0f, 50.0f, -20.0f };

    auto anchor = body_frame::chooseSelectionDistanceAnchor(true, hitPoint, true, bodyTransform, true, ownerNode, true, motionCom, fallback);
    ok &= expectSource("selection anchor prefers hit point", anchor.source, body_frame::BodyFrameSource::ContactPoint);
    ok &= expectFloat("selection hit distance", body_frame::distance(anchor.position, selectionOrigin), 10.0f);

    anchor = body_frame::chooseSelectionDistanceAnchor(false, hitPoint, true, bodyTransform, true, ownerNode, true, motionCom, fallback);
    ok &= expectSource("selection anchor prefers body transform over COM", anchor.source, body_frame::BodyFrameSource::BodyTransform);
    ok &= expectFloat("selection body distance", body_frame::distance(anchor.position, selectionOrigin), 30.0f);

    anchor = body_frame::chooseSelectionDistanceAnchor(false, hitPoint, false, bodyTransform, true, ownerNode, true, motionCom, fallback);
    ok &= expectSource("selection anchor falls back to owner node before COM", anchor.source, body_frame::BodyFrameSource::OwnerNode);
    ok &= expectFloat("selection owner distance", body_frame::distance(anchor.position, selectionOrigin), 40.0f);

    anchor = body_frame::chooseSelectionDistanceAnchor(false, hitPoint, false, bodyTransform, false, ownerNode, true, motionCom, fallback);
    ok &= expectSource("selection anchor uses COM only after body and owner are unavailable", anchor.source, body_frame::BodyFrameSource::MotionCenterOfMass);
    ok &= expectFloat("selection COM fallback distance", body_frame::distance(anchor.position, selectionOrigin), 150.0f);

    return ok ? 0 : 1;
}
