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

    return ok ? 0 : 1;
}
