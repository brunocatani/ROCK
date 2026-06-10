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

    ok &= expectWorldSurface("static is a world soft-contact surface", FO4_LAYER_STATIC);
    ok &= expectWorldSurface("animstatic is a world soft-contact surface", FO4_LAYER_ANIMSTATIC);
    ok &= expectWorldSurface("transparent support is a world soft-contact surface", FO4_LAYER_TRANSPARENT);
    ok &= expectWorldSurface("trees are world soft-contact surfaces", FO4_LAYER_TREES);
    ok &= expectWorldSurface("terrain is a world soft-contact surface", FO4_LAYER_TERRAIN);
    ok &= expectWorldSurface("ground is a world soft-contact surface", FO4_LAYER_GROUND);
    ok &= expectWorldSurface("transparent small support is a world soft-contact surface", FO4_LAYER_TRANSPARENT_SMALL);
    ok &= expectWorldSurface("invisible wall is a world soft-contact surface", FO4_LAYER_INVISIBLE_WALL);
    ok &= expectWorldSurface("transparent small anim support is a world soft-contact surface", FO4_LAYER_TRANSPARENT_SMALL_ANIM);
    ok &= expectWorldSurface("stair helper is a world soft-contact surface", FO4_LAYER_STAIRHELPER);
    ok &= expectWorldSurface("avoid box is a world soft-contact surface", FO4_LAYER_AVOIDBOX);
    ok &= expectWorldSurface("collision box is a world soft-contact surface", FO4_LAYER_COLLISIONBOX);
    ok &= expectNonWorldSurface("clutter stays out of world soft contact", FO4_LAYER_CLUTTER);
    ok &= expectNonWorldSurface("weapon layer stays out of world soft contact", FO4_LAYER_WEAPON);
    ok &= expectNonWorldSurface("actor layer stays out of world soft contact", FO4_LAYER_BIPED);
    ok &= expectNonWorldSurface("query-only item pick stays out of world soft contact", FO4_LAYER_ITEMPICK);
    ok &= expectNonWorldSurface("ROCK hand layer stays out of world soft contact", ROCK_LAYER_HAND);
    ok &= expectNonWorldSurface("ROCK weapon layer stays out of world soft contact", ROCK_LAYER_WEAPON);
    ok &= expectNonWorldSurface("ROCK body layer stays out of world soft contact", ROCK_LAYER_BODY);

    return ok ? 0 : 1;
}
