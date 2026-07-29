#include "physics-interaction/collision/CollisionLayerPolicy.h"

#include <cstdio>

namespace
{
    bool expectSuppressed(const char* label, std::uint32_t layer)
    {
        if (rock::collision_layer_policy::isNativePlayerCollisionSuppressionLayer(layer)) {
            return true;
        }
        std::printf("%s expected native player suppression for layer=%u\n", label, layer);
        return false;
    }

    bool expectPreserved(const char* label, std::uint32_t layer)
    {
        if (!rock::collision_layer_policy::isNativePlayerCollisionSuppressionLayer(layer)) {
            return true;
        }
        std::printf("%s expected native player preservation for layer=%u\n", label, layer);
        return false;
    }

    bool expectControllerObjectSuppressed(const char* label, std::uint32_t layer)
    {
        if (rock::collision_layer_policy::isNativeCharacterControllerObjectSuppressionLayer(layer)) {
            return true;
        }
        std::printf("%s expected native character controller object suppression for layer=%u\n", label, layer);
        return false;
    }

    bool expectControllerObjectPreserved(const char* label, std::uint32_t layer)
    {
        if (!rock::collision_layer_policy::isNativeCharacterControllerObjectSuppressionLayer(layer)) {
            return true;
        }
        std::printf("%s expected native character controller object preservation for layer=%u\n", label, layer);
        return false;
    }

    bool expectWorldSurface(const char* label, std::uint32_t layer)
    {
        if (rock::collision_layer_policy::isWorldSurfaceLayer(layer)) {
            return true;
        }
        std::printf("%s expected world surface layer=%u\n", label, layer);
        return false;
    }

    bool expectNonWorldSurface(const char* label, std::uint32_t layer)
    {
        if (!rock::collision_layer_policy::isWorldSurfaceLayer(layer)) {
            return true;
        }
        std::printf("%s expected non-world surface layer=%u\n", label, layer);
        return false;
    }
}

int main()
{
    using namespace rock::collision_layer_policy;

    bool ok = true;

    ok &= expectSuppressed("player biped body layer is suppressible", FO4_LAYER_BIPED);
    ok &= expectSuppressed("player deadbip body layer is suppressible", FO4_LAYER_DEADBIP);
    ok &= expectSuppressed("player biped-no-cc body layer is suppressible", FO4_LAYER_BIPED_NO_CC);

    ok &= expectPreserved("static world support is preserved", FO4_LAYER_STATIC);
    ok &= expectPreserved("animstatic world support is preserved", FO4_LAYER_ANIMSTATIC);
    ok &= expectPreserved("character controller is handled by the controller hook", FO4_LAYER_CHARCONTROLLER);
    ok &= expectPreserved("ROCK hand layer is never native-player-suppressed", ROCK_LAYER_HAND);
    ok &= expectPreserved("ROCK weapon layer is never native-player-suppressed", ROCK_LAYER_WEAPON);
    ok &= expectPreserved("ROCK body layer is never native-player-suppressed", ROCK_LAYER_BODY);
    ok &= expectPreserved("ordinary clutter is not a native player body layer", FO4_LAYER_CLUTTER);
    ok &= expectPreserved("ordinary weapon is not a native player body layer", FO4_LAYER_WEAPON);

    ok &= expectControllerObjectSuppressed("character controller suppresses clutter objects", FO4_LAYER_CLUTTER);
    ok &= expectControllerObjectSuppressed("character controller suppresses weapon objects", FO4_LAYER_WEAPON);
    ok &= expectControllerObjectSuppressed("character controller suppresses small debris", FO4_LAYER_DEBRIS_SMALL);
    ok &= expectControllerObjectSuppressed("character controller suppresses large debris", FO4_LAYER_DEBRIS_LARGE);
    ok &= expectControllerObjectSuppressed("character controller suppresses shell casings", FO4_LAYER_SHELLCASING);
    ok &= expectControllerObjectSuppressed("character controller suppresses large clutter", FO4_LAYER_CLUTTER_LARGE);

    ok &= expectControllerObjectPreserved("character controller keeps static support out of object suppression", FO4_LAYER_STATIC);
    ok &= expectControllerObjectPreserved("character controller keeps animstatic support out of object suppression", FO4_LAYER_ANIMSTATIC);
    ok &= expectControllerObjectPreserved("character controller keeps terrain support out of object suppression", FO4_LAYER_TERRAIN);
    ok &= expectControllerObjectPreserved("character controller keeps actor layers out of object suppression", FO4_LAYER_BIPED);
    ok &= expectControllerObjectPreserved("character controller keeps ROCK hand layer out of object suppression", ROCK_LAYER_HAND);
    ok &= expectControllerObjectPreserved("character controller keeps ROCK weapon layer out of object suppression", ROCK_LAYER_WEAPON);
    ok &= expectControllerObjectPreserved("character controller keeps ROCK body layer out of object suppression", ROCK_LAYER_BODY);

    ok &= expectWorldSurface("static is a dynamic hand world surface", FO4_LAYER_STATIC);
    ok &= expectWorldSurface("animstatic is a dynamic hand world surface", FO4_LAYER_ANIMSTATIC);
    ok &= expectWorldSurface("transparent support is a dynamic hand world surface", FO4_LAYER_TRANSPARENT);
    ok &= expectWorldSurface("trees are dynamic hand world surfaces", FO4_LAYER_TREES);
    ok &= expectWorldSurface("terrain is a dynamic hand world surface", FO4_LAYER_TERRAIN);
    ok &= expectWorldSurface("ground is a dynamic hand world surface", FO4_LAYER_GROUND);
    ok &= expectWorldSurface("transparent small support is a dynamic hand world surface", FO4_LAYER_TRANSPARENT_SMALL);
    ok &= expectWorldSurface("invisible wall is a dynamic hand world surface", FO4_LAYER_INVISIBLE_WALL);
    ok &= expectWorldSurface("transparent small anim support is a dynamic hand world surface", FO4_LAYER_TRANSPARENT_SMALL_ANIM);
    ok &= expectWorldSurface("stair helper is a dynamic hand world surface", FO4_LAYER_STAIRHELPER);
    ok &= expectWorldSurface("avoid box is a dynamic hand world surface", FO4_LAYER_AVOIDBOX);
    ok &= expectWorldSurface("collision box is a dynamic hand world surface", FO4_LAYER_COLLISIONBOX);
    ok &= expectNonWorldSurface("clutter stays out of dynamic hand world collision", FO4_LAYER_CLUTTER);
    ok &= expectNonWorldSurface("weapon layer stays out of dynamic hand world collision", FO4_LAYER_WEAPON);
    ok &= expectNonWorldSurface("actor layer stays out of dynamic hand world collision", FO4_LAYER_BIPED);
    ok &= expectNonWorldSurface("query-only item pick stays out of dynamic hand world collision", FO4_LAYER_ITEMPICK);
    ok &= expectNonWorldSurface("ROCK hand layer stays out of dynamic hand world collision", ROCK_LAYER_HAND);
    ok &= expectNonWorldSurface("ROCK weapon layer stays out of dynamic hand world collision", ROCK_LAYER_WEAPON);
    ok &= expectNonWorldSurface("ROCK body layer stays out of dynamic hand world collision", ROCK_LAYER_BODY);

    /*
     * Held-object row: the entire point is that a hand-held object stops
     * solving against the equipped weapon's generated hulls while keeping every
     * other contact it had. A regression here is invisible in-game until a
     * player notices gear either jittering against their gun again or ghosting
     * through the world.
     */
    {
        const auto heldMask = buildRockHeldObjectExpectedMask(true);
        const auto expectHeld = [&](const char* label, std::uint32_t layer, bool expected) {
            if (maskEnablesLayer(heldMask, layer) == expected) {
                return true;
            }
            std::printf("%s: held-object mask layer=%u expected=%d\n", label, layer, expected ? 1 : 0);
            return false;
        };

        ok &= expectHeld("held objects must never solve against ROCK weapon hulls", ROCK_LAYER_WEAPON, false);
        ok &= expectHeld("held objects keep static world collision", FO4_LAYER_STATIC, true);
        ok &= expectHeld("held objects keep clutter collision", FO4_LAYER_CLUTTER, true);
        ok &= expectHeld("held objects keep loose weapon collision", FO4_LAYER_WEAPON, true);
        ok &= expectHeld("held objects keep actor collision", FO4_LAYER_BIPED, true);
        ok &= expectHeld("held objects keep terrain collision", FO4_LAYER_TERRAIN, true);
        ok &= expectHeld("held objects keep ROCK hand collision", ROCK_LAYER_HAND, true);
        ok &= expectHeld("held objects keep ROCK body-bone collision", ROCK_LAYER_BODY, true);
        ok &= expectHeld("held objects stay off the player capsule", FO4_LAYER_CHARCONTROLLER, false);
        ok &= expectHeld("held objects stay out of query-only picking", FO4_LAYER_ITEMPICK, false);
        ok &= expectHeld("held objects stay out of line-of-sight picking", FO4_LAYER_LINEOFSIGHT, false);
        ok &= expectHeld("held objects stay off the non-collidable row", FO4_LAYER_NONCOLLIDABLE, false);

        if (!maskEnablesLayer(buildRockWeaponExpectedMask(true, true, true, true), ROCK_LAYER_HELD_OBJECT)) {
            // Both rows must independently exclude each other so the pair stays
            // disabled no matter which row applyRockGeneratedLayerPolicies writes last.
        } else {
            std::printf("weapon mask must not re-enable the held-object layer\n");
            ok = false;
        }

        if (ROCK_LAYER_HELD_OBJECT == ROCK_LAYER_BODY ||
            ROCK_LAYER_HELD_OBJECT == ROCK_LAYER_DYNAMIC_HAND_PROXY ||
            ROCK_LAYER_HELD_OBJECT == ROCK_LAYER_HAND ||
            ROCK_LAYER_HELD_OBJECT == ROCK_LAYER_WEAPON) {
            std::printf("held-object layer must own a distinct matrix row\n");
            ok = false;
        }
        if (!isMatrixAddressableLayer(ROCK_LAYER_HELD_OBJECT)) {
            std::printf("held-object layer must be addressable in the 64-row matrix\n");
            ok = false;
        }

        // The layer swap must preserve every non-layer filter bit, or a
        // coexisting suppression lease would be silently dropped.
        constexpr std::uint32_t sampleFilter = 0x000B'4000u | FO4_LAYER_CLUTTER;
        const auto swapped = withFilterLayer(sampleFilter, ROCK_LAYER_HELD_OBJECT);
        if (filterLayer(swapped) != ROCK_LAYER_HELD_OBJECT ||
            (swapped & ~FO4_LAYER_FILTER_MASK) != (sampleFilter & ~FO4_LAYER_FILTER_MASK)) {
            std::printf("held-object layer swap must preserve all non-layer filter bits\n");
            ok = false;
        }
        if (filterLayer(withFilterLayer(swapped, FO4_LAYER_CLUTTER)) != FO4_LAYER_CLUTTER) {
            std::printf("held-object layer restore must return the authored layer\n");
            ok = false;
        }
    }

    return ok ? 0 : 1;
}
