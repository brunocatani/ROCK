#include "physics-interaction/contact/SoftContactMath.h"
#include "physics-interaction/contact/SoftContactRuntime.h"
#include "physics-interaction/contact/SoftContactWorldPolicy.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"

#include <array>
#include <cstdio>

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

    bool expectNear(const char* label, float actual, float expected, float epsilon)
    {
        const float delta = actual > expected ? actual - expected : expected - actual;
        if (delta <= epsilon) {
            return true;
        }
        std::printf("%s expected %.4f got %.4f\n", label, expected, actual);
        return false;
    }

    bool expectEqual(const char* label, std::uint32_t actual, std::uint32_t expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected %u got %u\n", label, expected, actual);
        return false;
    }
}

int main()
{
    using namespace rock;
    using namespace rock::collision_layer_policy;
    using namespace rock::soft_contact_math;
    using namespace rock::soft_contact_world_policy;

    bool ok = true;

    ok &= expectTrue("soft-contact policy accepts static", acceptsWorldSurfaceLayer(FO4_LAYER_STATIC));
    ok &= expectTrue("soft-contact policy accepts animstatic", acceptsWorldSurfaceLayer(FO4_LAYER_ANIMSTATIC));
    ok &= expectTrue("soft-contact policy accepts terrain", acceptsWorldSurfaceLayer(FO4_LAYER_TERRAIN));
    ok &= expectTrue("soft-contact policy accepts ground", acceptsWorldSurfaceLayer(FO4_LAYER_GROUND));
    ok &= expectTrue("soft-contact policy accepts invisible wall", acceptsWorldSurfaceLayer(FO4_LAYER_INVISIBLE_WALL));
    ok &= expectTrue("soft-contact policy accepts trees", acceptsWorldSurfaceLayer(FO4_LAYER_TREES));
    ok &= expectTrue("soft-contact policy accepts transparent support", acceptsWorldSurfaceLayer(FO4_LAYER_TRANSPARENT));
    ok &= expectTrue("soft-contact policy accepts transparent small", acceptsWorldSurfaceLayer(FO4_LAYER_TRANSPARENT_SMALL));
    ok &= expectTrue("soft-contact policy accepts transparent small anim", acceptsWorldSurfaceLayer(FO4_LAYER_TRANSPARENT_SMALL_ANIM));
    ok &= expectTrue("soft-contact policy accepts stair helper", acceptsWorldSurfaceLayer(FO4_LAYER_STAIRHELPER));
    ok &= expectTrue("soft-contact policy accepts avoid box", acceptsWorldSurfaceLayer(FO4_LAYER_AVOIDBOX));
    ok &= expectTrue("soft-contact policy accepts collision box", acceptsWorldSurfaceLayer(FO4_LAYER_COLLISIONBOX));
    ok &= expectFalse("soft-contact policy rejects clutter", acceptsWorldSurfaceLayer(FO4_LAYER_CLUTTER));
    ok &= expectFalse("soft-contact policy rejects weapon", acceptsWorldSurfaceLayer(FO4_LAYER_WEAPON));
    ok &= expectFalse("soft-contact policy rejects actor", acceptsWorldSurfaceLayer(FO4_LAYER_BIPED));
    ok &= expectFalse("soft-contact policy rejects query-only", acceptsWorldSurfaceLayer(FO4_LAYER_ITEMPICK));
    ok &= expectFalse("soft-contact policy rejects ROCK hand", acceptsWorldSurfaceLayer(ROCK_LAYER_HAND));
    ok &= expectFalse("soft-contact policy rejects ROCK weapon", acceptsWorldSurfaceLayer(ROCK_LAYER_WEAPON));
    ok &= expectFalse("soft-contact policy rejects ROCK body", acceptsWorldSurfaceLayer(ROCK_LAYER_BODY));
    ok &= expectEqual("filter layer strips high bits", layerFromFilterInfo(0xCAFE0081u), FO4_LAYER_STATIC);
    ok &= expectTrue("filter acceptance uses stripped layer", acceptsWorldSurfaceFilterInfo(0xCAFE0081u));

    const RE::NiPoint3 point{ 0.0f, 0.0f, 0.25f };
    const RE::NiPoint3 planePoint{ 0.0f, 0.0f, 0.0f };
    const RE::NiPoint3 planeNormal{ 0.0f, 0.0f, 1.0f };
    const auto active = solvePointPlaneContact(point, planePoint, planeNormal, 0.2f, 0.1f, 10u, 20u);
    ok &= expectTrue("point-plane contact active inside radius plus skin", active.active);
    ok &= expectNear("point-plane contact penetration", active.penetration, 0.05f, 0.001f);
    ok &= expectNear("point-plane target projection z", active.targetPoint.z, 0.0f, 0.001f);

    const auto separated = solvePointPlaneContact(RE::NiPoint3{ 0.0f, 0.0f, 0.4f }, planePoint, planeNormal, 0.2f, 0.1f, 10u, 20u);
    ok &= expectFalse("point-plane contact releases after separation", separated.active);

    std::array<CapsuleContact, 2> cornerContacts{
        solvePointPlaneContact(RE::NiPoint3{ 0.1f, 0.1f, 0.0f }, planePoint, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, 0.25f, 0.05f, 11u, 21u),
        solvePointPlaneContact(RE::NiPoint3{ 0.1f, 0.1f, 0.0f }, planePoint, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }, 0.25f, 0.05f, 12u, 22u),
    };
    const auto cornerCorrection = projectTrackedMagnetPlaneSetCorrection(cornerContacts, 2u, 2.0f);
    ok &= expectNear("plane-set correction combines first axis", cornerCorrection.x, 0.2f, 0.001f);
    ok &= expectNear("plane-set correction combines second axis", cornerCorrection.y, 0.2f, 0.001f);
    ok &= expectNear("plane-set correction leaves unrelated axis", cornerCorrection.z, 0.0f, 0.001f);

    const auto clampedCornerCorrection = projectTrackedMagnetPlaneSetCorrection(cornerContacts, 2u, 0.25f);
    ok &= expectTrue("plane-set correction respects max correction",
        length(clampedCornerCorrection) <= 0.251f);

    ok &= expectTrue("cached plane keeps bounded tangent drift",
        withinTangentDriftLimit(RE::NiPoint3{ 2.0f, 0.0f, 0.0f }, planePoint, planeNormal, 3.0f));
    ok &= expectFalse("cached plane rejects excessive tangent drift",
        withinTangentDriftLimit(RE::NiPoint3{ 4.0f, 0.0f, 0.0f }, planePoint, planeNormal, 3.0f));
    ok &= expectTrue("cached plane keeps bounded radial distance",
        withinClearDistanceLimit(RE::NiPoint3{ 0.0f, 0.0f, -2.0f }, planePoint, 3.0f));
    ok &= expectFalse("cached plane rejects excessive distance through plane",
        withinClearDistanceLimit(RE::NiPoint3{ 0.0f, 0.0f, -4.0f }, planePoint, 3.0f));
    ok &= expectFalse("cached plane rejects excessive distance on any axis",
        withinClearDistanceLimit(RE::NiPoint3{ 2.5f, 2.5f, 0.0f }, planePoint, 3.0f));
    ok &= expectTrue("normal frame sweep is allowed when no cached plane was released",
        shouldAllowPostReleaseReentrySweep(false, RE::NiPoint3{ 0.0f, 0.0f, 1.0f }, planeNormal, 0.05f));
    ok &= expectTrue("post-release reentry allows renewed approach",
        shouldAllowPostReleaseReentrySweep(true, RE::NiPoint3{ 0.0f, 0.0f, -0.1f }, planeNormal, 0.05f));
    ok &= expectFalse("post-release reentry blocks separation sweep",
        shouldAllowPostReleaseReentrySweep(true, RE::NiPoint3{ 0.0f, 0.0f, 0.1f }, planeNormal, 0.05f));

    HapticEdgeConfig haptics{};
    haptics.baseIntensity = 0.20f;
    haptics.maxIntensity = 0.60f;
    haptics.speedScale = 0.01f;
    haptics.minApproachSpeed = 5.0f;
    haptics.cooldownSeconds = 0.20f;
    HapticEdgeState hapticState{};
    auto pulse = updateHapticEdge(hapticState, true, 4.0f, 0.016f, haptics);
    ok &= expectFalse("haptic ignores slow entry", pulse.fire);
    pulse = updateHapticEdge(hapticState, false, 0.0f, 0.016f, haptics);
    ok &= expectFalse("haptic inactive frame does not fire", pulse.fire);
    pulse = updateHapticEdge(hapticState, true, 20.0f, 0.016f, haptics);
    ok &= expectTrue("haptic fires on fast entry", pulse.fire);
    ok &= expectNear("haptic intensity scales from speed", pulse.intensity, 0.40f, 0.001f);
    pulse = updateHapticEdge(hapticState, true, 20.0f, 0.016f, haptics);
    ok &= expectFalse("haptic does not repeat while held active", pulse.fire);
    pulse = updateHapticEdge(hapticState, false, 0.0f, 0.05f, haptics);
    ok &= expectFalse("haptic release during cooldown does not fire", pulse.fire);
    pulse = updateHapticEdge(hapticState, true, 20.0f, 0.05f, haptics);
    ok &= expectFalse("haptic cooldown blocks immediate reentry", pulse.fire);
    pulse = updateHapticEdge(hapticState, false, 0.0f, 0.25f, haptics);
    ok &= expectFalse("haptic cooldown drains while inactive", pulse.fire);
    pulse = updateHapticEdge(hapticState, true, 20.0f, 0.016f, haptics);
    ok &= expectTrue("haptic fires after cooldown drains", pulse.fire);

    SoftContactDebugContact debugContact{};
    debugContact.source = SoftContactDebugSource::NativeWorld;
    ok &= expectTrue("debug contact carries native source",
        debugContact.source == SoftContactDebugSource::NativeWorld);

    return ok ? 0 : 1;
}
