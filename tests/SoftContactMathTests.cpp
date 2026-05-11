#define NOMMNOSOUND
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "F4SE/Impl/PCH.h"

#ifdef near
#undef near
#endif
#ifdef far
#undef far
#endif
#ifdef MEM_RELEASE
#undef MEM_RELEASE
#endif
#ifdef MEM_COMMIT
#undef MEM_COMMIT
#endif
#ifdef MEM_RESERVE
#undef MEM_RESERVE
#endif
#ifdef PAGE_EXECUTE_READ
#undef PAGE_EXECUTE_READ
#endif
#ifdef PAGE_EXECUTE_READWRITE
#undef PAGE_EXECUTE_READWRITE
#endif
#ifdef MAX_PATH
#undef MAX_PATH
#endif

#include <cmath>
#include <cstdio>

#include "physics-interaction/contact/SoftContactMath.h"
#include "physics-interaction/contact/SoftContactWorldPolicy.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"

namespace
{
    constexpr float kEpsilon = 0.0001f;

    bool nearlyEqual(float actual, float expected, float epsilon = kEpsilon)
    {
        return std::fabs(actual - expected) <= epsilon;
    }

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

    bool expectNear(const char* label, float actual, float expected, float epsilon = kEpsilon)
    {
        if (nearlyEqual(actual, expected, epsilon)) {
            return true;
        }

        std::printf("%s expected %.4f got %.4f\n", label, expected, actual);
        return false;
    }
}

int main()
{
    using namespace rock::soft_contact_math;

    bool ok = true;

    {
        const Capsule movable{
            .start = RE::NiPoint3(1.5f, 0.0f, 0.0f),
            .end = RE::NiPoint3(1.5f, 10.0f, 0.0f),
            .radius = 1.0f,
            .id = 10,
            .valid = true,
        };
        const Capsule target{
            .start = RE::NiPoint3(0.0f, 0.0f, 0.0f),
            .end = RE::NiPoint3(0.0f, 10.0f, 0.0f),
            .radius = 1.0f,
            .id = 20,
            .valid = true,
        };

        const auto contact = solveCapsulePair(movable, target, RE::NiPoint3(0.0f, 0.0f, 1.0f), 0.0f);
        ok &= expectTrue("parallel capsules overlap", contact.active);
        ok &= expectNear("parallel penetration", contact.penetration, 0.5f);
        ok &= expectNear("parallel normal x", contact.normal.x, 1.0f);
        ok &= expectNear("parallel normal y", contact.normal.y, 0.0f);
        ok &= expectNear("parallel normal z", contact.normal.z, 0.0f);
        ok &= expectTrue("movable id preserved", contact.movableId == 10);
        ok &= expectTrue("target id preserved", contact.targetId == 20);
    }

    {
        const Capsule movable{
            .start = RE::NiPoint3(5.0f, 0.0f, 0.0f),
            .end = RE::NiPoint3(5.0f, 10.0f, 0.0f),
            .radius = 1.0f,
            .valid = true,
        };
        const Capsule target{
            .start = RE::NiPoint3(0.0f, 0.0f, 0.0f),
            .end = RE::NiPoint3(0.0f, 10.0f, 0.0f),
            .radius = 1.0f,
            .valid = true,
        };

        const auto contact = solveCapsulePair(movable, target, RE::NiPoint3(0.0f, 0.0f, 1.0f), 0.0f);
        ok &= expectFalse("separated capsules inactive", contact.active);
        ok &= expectNear("separated penetration negative", contact.penetration, -3.0f);
    }

    {
        const Capsule movable{
            .start = RE::NiPoint3(0.0f, 0.0f, 0.0f),
            .end = RE::NiPoint3(0.0f, 0.0f, 0.0f),
            .radius = 1.0f,
            .valid = true,
        };
        const Capsule target{
            .start = RE::NiPoint3(0.0f, 0.0f, 0.0f),
            .end = RE::NiPoint3(0.0f, 0.0f, 0.0f),
            .radius = 1.0f,
            .valid = true,
        };

        const auto contact = solveCapsulePair(movable, target, RE::NiPoint3(0.0f, 1.0f, 0.0f), 0.25f);
        ok &= expectTrue("coincident capsules active", contact.active);
        ok &= expectNear("coincident penetration includes padding", contact.penetration, 2.25f);
        ok &= expectNear("coincident fallback normal y", contact.normal.y, 1.0f);
    }

    {
        const RE::NiPoint3 previous(0.0f, 0.0f, 0.0f);
        const RE::NiPoint3 desired(10.0f, 0.0f, 0.0f);
        const auto step = smoothCorrection(previous, desired, 1000.0f, 1.0f / 90.0f, 4.0f);
        ok &= expectNear("correction clamp length", length(step.correction), 4.0f, 0.001f);
        ok &= expectTrue("correction alpha positive", step.alpha > 0.0f);
        ok &= expectTrue("correction alpha bounded", step.alpha <= 1.0f);
    }

    {
        const auto correction = projectTrackedMagnetCorrection(
            RE::NiPoint3(1.0f, 0.0f, 0.0f),
            3.5f,
            2.0f);
        ok &= expectNear("tracked magnet projection clamps x", correction.x, 2.0f);
        ok &= expectNear("tracked magnet projection keeps tangent free y", correction.y, 0.0f);

        const auto separated = projectTrackedMagnetCorrection(
            RE::NiPoint3(1.0f, 0.0f, 0.0f),
            -0.1f,
            2.0f);
        ok &= expectNear("tracked magnet projection clears on separation", length(separated), 0.0f);
    }

    {
        ok &= expectNear("compliant response softens shallow contact", compliantHardStopResponseScale(1.0f, 0.25f, 4.0f), 0.3671875f);
        ok &= expectNear("compliant response reaches hard stop", compliantHardStopResponseScale(4.0f, 0.25f, 4.0f), 1.0f);

        const auto correction = projectCompliantTrackedMagnetCorrection(
            RE::NiPoint3(1.0f, 0.0f, 0.0f),
            2.0f,
            10.0f,
            0.25f,
            4.0f);
        ok &= expectNear("compliant projection scales current frame correction", correction.x, 1.25f);
        ok &= expectNear("compliant projection keeps tangent free y", correction.y, 0.0f);
        ok &= expectTrue("response arbitration uses actual correction length",
            preferStrongerContactResponse(2.0f, 2.0f, 1.5f, 5.0f));
        ok &= expectFalse("response arbitration rejects weaker correction length",
            preferStrongerContactResponse(1.5f, 5.0f, 2.0f, 2.0f));
        ok &= expectTrue("response arbitration falls back to penetration on equal correction",
            preferStrongerContactResponse(2.0f, 3.0f, 2.0f, 2.0f));
    }

    {
        const Capsule palmSide{
            .start = RE::NiPoint3(0.0f, -4.0f, 0.0f),
            .end = RE::NiPoint3(0.0f, 4.0f, 0.0f),
            .radius = 1.0f,
            .id = 30,
            .valid = true,
        };
        const Aabb weaponBounds{
            .min = RE::NiPoint3(0.75f, -1.0f, -1.0f),
            .max = RE::NiPoint3(2.0f, 1.0f, 1.0f),
            .id = 40,
            .valid = true,
        };

        const auto contact = solveCapsuleAabb(palmSide, weaponBounds, RE::NiPoint3(0.0f, 0.0f, 1.0f), 0.0f);
        ok &= expectTrue("capsule aabb side overlap active", contact.active);
        ok &= expectNear("capsule aabb side penetration", contact.penetration, 0.25f);
        ok &= expectNear("capsule aabb side normal x", contact.normal.x, -1.0f);
        ok &= expectTrue("capsule aabb movable id preserved", contact.movableId == 30);
        ok &= expectTrue("capsule aabb target id preserved", contact.targetId == 40);
    }

    {
        const Capsule insideWeapon{
            .start = RE::NiPoint3(0.0f, 0.0f, 0.0f),
            .end = RE::NiPoint3(2.0f, 0.0f, 0.0f),
            .radius = 0.5f,
            .valid = true,
        };
        const Aabb weaponBounds{
            .min = RE::NiPoint3(-1.0f, -1.0f, -1.0f),
            .max = RE::NiPoint3(1.0f, 1.0f, 1.0f),
            .valid = true,
        };

        const auto contact = solveCapsuleAabb(insideWeapon, weaponBounds, RE::NiPoint3(0.0f, 0.0f, 1.0f), 0.0f);
        ok &= expectTrue("capsule aabb interior active", contact.active);
        ok &= expectNear("capsule aabb interior penetration includes exit distance", contact.penetration, 1.5f);
        ok &= expectNear("capsule aabb interior normal x", contact.normal.x, -1.0f);
    }

    {
        const Capsule crossingWeapon{
            .start = RE::NiPoint3(-4.0f, 0.25f, 0.0f),
            .end = RE::NiPoint3(4.0f, 0.25f, 0.0f),
            .radius = 0.5f,
            .valid = true,
        };
        const Aabb weaponBounds{
            .min = RE::NiPoint3(-1.0f, -1.0f, -1.0f),
            .max = RE::NiPoint3(1.0f, 1.0f, 1.0f),
            .valid = true,
        };

        const auto contact = solveCapsuleAabb(crossingWeapon, weaponBounds, RE::NiPoint3(0.0f, 0.0f, 1.0f), 0.0f);
        ok &= expectTrue("capsule aabb pass-through active", contact.active);
        ok &= expectNear("capsule aabb pass-through uses deepest interval point", contact.penetration, 1.25f);
        ok &= expectNear("capsule aabb pass-through movable x", contact.movablePoint.x, 0.0f);
        ok &= expectNear("capsule aabb pass-through normal y", contact.normal.y, 1.0f);
    }

    {
        using namespace rock::collision_layer_policy;
        using namespace rock::soft_contact_world_policy;
        ok &= expectTrue("world policy accepts static layer", acceptsWorldSurfaceLayer(FO4_LAYER_STATIC));
        ok &= expectTrue("world policy accepts animstatic layer", acceptsWorldSurfaceLayer(FO4_LAYER_ANIMSTATIC));
        ok &= expectFalse("world policy rejects dynamic clutter", acceptsWorldSurfaceLayer(FO4_LAYER_CLUTTER));
        ok &= expectFalse("world policy rejects actor layer", acceptsWorldSurfaceLayer(FO4_LAYER_BIPED));
        ok &= expectFalse("world policy rejects query layer", acceptsWorldSurfaceLayer(FO4_LAYER_ITEMPICK));
        ok &= expectFalse("world policy rejects ROCK hand layer", acceptsWorldSurfaceLayer(ROCK_LAYER_HAND));
        ok &= expectTrue("world policy accepts filter info low layer", acceptsWorldSurfaceFilterInfo(0x00050000u | FO4_LAYER_STATIC));
    }

    {
        const auto contact = solvePointPlaneContact(
            RE::NiPoint3(0.25f, 0.0f, 0.0f),
            RE::NiPoint3(0.0f, 0.0f, 0.0f),
            RE::NiPoint3(1.0f, 0.0f, 0.0f),
            1.0f,
            0.5f,
            77,
            88);
        ok &= expectTrue("world plane contact active inside skin", contact.active);
        ok &= expectNear("world plane penetration", contact.penetration, 1.25f);
        ok &= expectNear("world plane correction normal x", contact.normal.x, 1.0f);
        ok &= expectNear("world plane projected target x", contact.targetPoint.x, 0.0f);
        ok &= expectTrue("world plane movable id preserved", contact.movableId == 77);
        ok &= expectTrue("world plane target id preserved", contact.targetId == 88);
    }

    {
        const auto contact = solvePointPlaneContact(
            RE::NiPoint3(3.0f, 0.0f, 0.0f),
            RE::NiPoint3(0.0f, 0.0f, 0.0f),
            RE::NiPoint3(1.0f, 0.0f, 0.0f),
            1.0f,
            0.5f,
            1,
            2);
        ok &= expectFalse("world plane inactive beyond skin", contact.active);
        ok &= expectNear("world plane negative penetration outside", contact.penetration, -1.5f);
    }

    {
        const auto normal = orientNormalTowardPoint(
            RE::NiPoint3(0.0f, 0.0f, 0.0f),
            RE::NiPoint3(-1.0f, 0.0f, 0.0f),
            RE::NiPoint3(1.0f, 0.0f, 0.0f),
            RE::NiPoint3(0.0f, 0.0f, 1.0f));
        ok &= expectNear("world plane normal oriented toward hand", normal.x, 1.0f);
    }

    {
        ok &= expectNear("effective query padding keeps broad query", effectiveQueryPadding(1.5f, 0.35f, 1.5f, 0.35f), 1.5f);
        ok &= expectNear("effective query padding covers larger contact shell", effectiveQueryPadding(0.1f, 0.7f, 1.5f, 0.35f), 0.7f);
        ok &= expectNear("effective query padding sanitizes invalid query", effectiveQueryPadding(-1.0f, 0.35f, 1.5f, 0.35f), 1.5f);
    }

    {
        const RE::NiPoint3 anchor(0.0f, 0.0f, 0.0f);
        const RE::NiPoint3 normal(0.0f, 0.0f, 1.0f);
        ok &= expectNear("world cached plane tangent drift ignores normal offset",
            tangentDistanceFromAnchor(RE::NiPoint3(3.0f, 4.0f, 20.0f), anchor, normal),
            5.0f);
        ok &= expectTrue("world cached plane accepts bounded tangent drift",
            withinTangentDriftLimit(RE::NiPoint3(6.0f, 0.0f, 3.0f), anchor, normal, 10.0f));
        ok &= expectFalse("world cached plane rejects stale tangent drift",
            withinTangentDriftLimit(RE::NiPoint3(11.0f, 0.0f, 3.0f), anchor, normal, 10.0f));
    }

    {
        const RE::NiPoint3 normal(1.0f, 0.0f, 0.0f);
        ok &= expectTrue("normal sweep always allowed",
            shouldAllowPostReleaseReentrySweep(false, RE::NiPoint3(1.0f, 0.0f, 0.0f), normal, 0.025f));
        ok &= expectFalse("post-release sweep moving away is blocked",
            shouldAllowPostReleaseReentrySweep(true, RE::NiPoint3(0.25f, 0.0f, 0.0f), normal, 0.025f));
        ok &= expectFalse("post-release tangent sweep is blocked",
            shouldAllowPostReleaseReentrySweep(true, RE::NiPoint3(0.0f, 0.25f, 0.0f), normal, 0.025f));
        ok &= expectTrue("post-release sweep pushing into surface reacquires",
            shouldAllowPostReleaseReentrySweep(true, RE::NiPoint3(-0.25f, 0.0f, 0.0f), normal, 0.025f));
    }

    {
        HapticEdgeState state{};
        const HapticEdgeConfig config{
            .enabled = true,
            .baseIntensity = 0.18f,
            .maxIntensity = 0.55f,
            .speedScale = 0.006f,
            .minApproachSpeed = 3.0f,
            .cooldownSeconds = 0.12f,
        };
        auto first = updateHapticEdge(state, true, 50.0f, 1.0f / 90.0f, config);
        ok &= expectTrue("world haptic fires on contact edge", first.fire);
        ok &= expectNear("world haptic speed-scaled intensity clamps", first.intensity, 0.48f, 0.001f);

        auto held = updateHapticEdge(state, true, 80.0f, 1.0f / 90.0f, config);
        ok &= expectFalse("world haptic does not repeat while held", held.fire);

        auto cleared = updateHapticEdge(state, false, 0.0f, 0.2f, config);
        ok &= expectFalse("world haptic does not fire while inactive", cleared.fire);

        auto reacquired = updateHapticEdge(state, true, 4.0f, 1.0f / 90.0f, config);
        ok &= expectTrue("world haptic rearms after contact clears", reacquired.fire);
    }

    {
        HapticEdgeState state{};
        const HapticEdgeConfig disabled{
            .enabled = false,
            .baseIntensity = 0.18f,
            .maxIntensity = 0.55f,
            .speedScale = 0.006f,
            .minApproachSpeed = 3.0f,
            .cooldownSeconds = 0.12f,
        };
        auto decision = updateHapticEdge(state, true, 50.0f, 1.0f / 90.0f, disabled);
        ok &= expectFalse("world haptic disabled suppresses edge", decision.fire);
    }

    return ok ? 0 : 1;
}
