#include "physics-interaction/collision/NativePlayerCollisionPolicy.h"
#include "physics-interaction/grab/GrabHeldObject.h"

#include <array>

#include <cstdio>
#include <cstring>

namespace
{
    bool expectSuppressed(const char* label, std::uint32_t layer)
    {
        if (rock::collision_layer_policy::isNativePlayerCollisionBodyLayer(layer)) {
            return true;
        }
        std::printf("%s expected native player tracking for layer=%u\n", label, layer);
        return false;
    }

    bool expectPreserved(const char* label, std::uint32_t layer)
    {
        if (!rock::collision_layer_policy::isNativePlayerCollisionBodyLayer(layer)) {
            return true;
        }
        std::printf("%s expected no native player tracking for layer=%u\n", label, layer);
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


    using namespace rock::native_player_collision;
    const auto expect = [&](const char* label, bool passed) {
        if (!passed) { std::printf("%s\n", label); }
        return passed;
    };
    constexpr std::uintptr_t listener = 0x100000;
    constexpr std::uintptr_t playerController = listener + 0x10;
    ok &= expect("proxy listener identifies its player controller interface",
        proxyListenerMatchesPlayer(listener, playerController));
    ok &= expect("direct controller comparison is not valid for a listener callback",
        !proxyListenerMatchesPlayer(playerController, playerController));
    ok &= expect("another actor's listener cannot acquire player filtering",
        !proxyListenerMatchesPlayer(listener + 0x1000, playerController));
    ok &= expect("incorrect listener adjustment is rejected",
        !proxyListenerMatchesPlayer(listener, listener + 8));
    ok &= expect("missing listener or controller cannot acquire player filtering",
        !proxyListenerMatchesPlayer(0, 0x10) && !proxyListenerMatchesPlayer(listener, 0));
    ok &= expect("wrapped listener address cannot match a controller",
        !proxyListenerMatchesPlayer(UINTPTR_MAX - 7, 8));
    for (auto layer : { FO4_LAYER_CHARCONTROLLER, FO4_LAYER_BIPED,
            FO4_LAYER_BIPED_NO_CC, FO4_LAYER_DEADBIP }) {
        ok &= expect("player locomotion retains native NPC and ragdoll contacts",
            !evaluatePlayerCharacterControllerContact({ true, true, true, layer }).suppress);
    }

    // Exercise the same paired-row compactor as the movement hook with NPCs,
    // attacks, scenery, loose objects and an independently held body together.
    for (bool holding : { false, true }) {
        constexpr std::array layers{ FO4_LAYER_BIPED, FO4_LAYER_CLUTTER,
            FO4_LAYER_CHARCONTROLLER, FO4_LAYER_DEADBIP, FO4_LAYER_BIPED_NO_CC,
            FO4_LAYER_WEAPON, FO4_LAYER_WEAPON, FO4_LAYER_STATIC, ROCK_LAYER_HAND,
            FO4_LAYER_STATIC };
        constexpr auto stride = rock::held_grab_cc_policy::kGeneratedContactStride;
        constexpr auto bodyOffset = rock::held_grab_cc_policy::kGeneratedContactBodyIdOffset;
        alignas(std::uint32_t) std::array<char, stride * layers.size()> contacts{}, constraints{};
        for (std::uint32_t id = 0; id < layers.size(); ++id) {
            std::memcpy(contacts.data() + id * stride + bodyOffset, &id, sizeof(id));
            std::memcpy(constraints.data() + id * stride, &id, sizeof(id));
        }
        int contactCount = static_cast<int>(layers.size());
        int constraintCount = contactCount;
        const rock::held_grab_cc_policy::GeneratedContactBufferView view{
            .valid = true, .manifoldEntries = contacts.data(), .constraintEntries = constraints.data(),
            .manifoldCountPtr = &contactCount, .constraintCountPtr = &constraintCount,
            .manifoldCount = contactCount, .constraintCount = constraintCount, .pairCount = contactCount,
        };
        const auto result = rock::held_grab_cc_policy::filterGeneratedContactBuffers(view,
            [&](std::uint32_t id) {
                return (holding && id == 9) || evaluatePlayerCharacterControllerContact({
                    .filterEnabled = true, .playerController = true, .targetLayerKnown = true,
                    .targetLayer = layers[id], .targetIsLooseWeapon = id == 6,
                }).suppress;
            });
        constexpr std::array<std::uint32_t, 7> expected{ 0, 2, 3, 4, 5, 7, 9 };
        const int expectedCount = holding ? 6 : 7;
        ok &= expect("object filtering preserves NPC, attack and support contacts while holding or empty-handed",
            result.valid && result.keptPairCount == expectedCount &&
                contactCount == expectedCount && constraintCount == expectedCount);
        for (int i = 0; i < expectedCount; ++i) {
            std::uint32_t contactId = 0, constraintId = 0;
            std::memcpy(&contactId, contacts.data() + i * stride + bodyOffset, sizeof(contactId));
            std::memcpy(&constraintId, constraints.data() + i * stride, sizeof(constraintId));
            ok &= expect("surviving NPC and native contact constraints stay paired and in order",
                contactId == expected[i] && constraintId == expected[i]);
        }
    }
    constexpr std::array attacks{ FO4_LAYER_WEAPON, FO4_LAYER_PROJECTILE,
        FO4_LAYER_SPELL, FO4_LAYER_CONEPROJECTILE, FO4_LAYER_SPELLEXPLOSION };
    for (auto layer : attacks) {
        ok &= expect("incoming physical attack remains admitted",
            !suppressPhysicalPair(true, false, FO4_LAYER_BIPED, layer));
        ok &= expect("incoming physical attack remains admitted in reverse order",
            !suppressPhysicalPair(false, true, layer, FO4_LAYER_BIPED));
        ok &= expect("controller retains incoming attack contact",
            !evaluatePlayerCharacterControllerContact({ true, true, true, layer }).suppress);
    }
    const PlayerCharacterControllerContactPolicyInput droppedWeapon{
        .filterEnabled = true, .playerController = true, .targetLayerKnown = true,
        .targetLayer = FO4_LAYER_WEAPON, .targetIsLooseWeapon = true,
    };
    ok &= expect("player controller does not bump a positively identified dropped gun",
        evaluatePlayerCharacterControllerContact(droppedWeapon).suppress);
    auto npcDroppedWeapon = droppedWeapon;
    npcDroppedWeapon.playerController = false;
    ok &= expect("NPC controller keeps its normal dropped-gun contact",
        !evaluatePlayerCharacterControllerContact(npcDroppedWeapon).suppress);
    auto unclassifiedWeapon = droppedWeapon;
    unclassifiedWeapon.targetIsLooseWeapon = false;
    ok &= expect("equipped or unidentified weapon contact remains native",
        !evaluatePlayerCharacterControllerContact(unclassifiedWeapon).suppress);
    ok &= expect("native player body does not bump a dropped gun",
        suppressPhysicalPair(true, false, FO4_LAYER_BIPED, FO4_LAYER_WEAPON, true));
    ok &= expect("dropped-gun suppression is symmetric",
        suppressPhysicalPair(false, true, FO4_LAYER_WEAPON, FO4_LAYER_BIPED, true));
    ok &= expect("ROCK hand still contacts dropped gun",
        !suppressPhysicalPair(false, false, ROCK_LAYER_HAND, FO4_LAYER_WEAPON, true));
    ok &= expect("ROCK weapon still contacts dropped gun",
        !suppressPhysicalPair(false, false, ROCK_LAYER_WEAPON, FO4_LAYER_WEAPON, true));
    ok &= expect("ROCK body still contacts dropped gun",
        !suppressPhysicalPair(false, false, ROCK_LAYER_BODY, FO4_LAYER_WEAPON, true));
    ok &= expect("loose-weapon identity cannot suppress projectile layer",
        !suppressPhysicalPair(true, false, FO4_LAYER_BIPED, FO4_LAYER_PROJECTILE, true));
    for (auto layer : { FO4_LAYER_STATIC, FO4_LAYER_ANIMSTATIC, FO4_LAYER_CLUTTER,
            FO4_LAYER_CLUTTER_LARGE, FO4_LAYER_PROPS, ROCK_LAYER_HAND, ROCK_LAYER_WEAPON,
            ROCK_LAYER_BODY, ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY, ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY,
            ROCK_LAYER_DYNAMIC_WEAPON_PROXY, ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER }) {
        ok &= expect("native player does not duplicate scenery or generated-tool contact",
            suppressPhysicalPair(true, false, FO4_LAYER_BIPED, layer));
        ok &= expect("physical filtering is symmetric",
            suppressPhysicalPair(false, true, layer, FO4_LAYER_BIPED));
        ok &= expect("NPC contact with that same obstacle remains native",
            !suppressPhysicalPair(false, false, FO4_LAYER_BIPED, layer));
    }
    for (auto layer : { FO4_LAYER_BIPED, FO4_LAYER_DEADBIP, FO4_LAYER_BIPED_NO_CC,
            FO4_LAYER_CHARCONTROLLER, FO4_LAYER_UNIDENTIFIED,
            FO4_LAYER_ITEMPICK, FO4_LAYER_LINEOFSIGHT, FO4_LAYER_PATHPICK }) {
        ok &= expect("non-obstacle and unknown contacts remain native",
            !suppressPhysicalPair(true, false, FO4_LAYER_BIPED, layer));
    }
    ok &= expect("player's own native bodies do not collide with one another",
        suppressPhysicalPair(true, true, FO4_LAYER_BIPED, FO4_LAYER_BIPED));

    // Registration may alter ROCK rows, never the native actor/attack/scenery
    // matrix that also governs NPCs. Check every native-to-native pair.
    std::array<std::uint64_t, 64> matrix{};
    for (std::size_t i = 0; i < matrix.size(); ++i) {
        matrix[i] = 0xB2978EBA769DC523ull ^ (i * 0x9E3779B97F4A7C15ull);
    }
    const auto original = matrix;
    for (const bool npcDynamicCollisions : { false, true, false }) {
        applyRockGeneratedLayerPolicies(matrix.data(), true, false, false, npcDynamicCollisions);
        ok &= expect("NPC dynamic collision toggles all three pairs symmetrically",
            rockDynamicNpcPairsMatch(matrix.data(), npcDynamicCollisions));
        for (const auto layer : { ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY,
                ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY, ROCK_LAYER_DYNAMIC_WEAPON_PROXY }) {
            ok &= expect("native player remains excluded from experimental dynamic contacts",
                suppressPhysicalPair(true, false, FO4_LAYER_BIPED_NO_CC, layer));
            ok &= expect("NPC dynamic contacts remain admitted by the player filter",
                !suppressPhysicalPair(false, false, FO4_LAYER_BIPED_NO_CC, layer));
            for (const auto excluded : { FO4_LAYER_BIPED, FO4_LAYER_DEADBIP, FO4_LAYER_CHARCONTROLLER }) {
                ok &= expect("experiment does not admit other native actor layers",
                    layerPairSymmetricMatches(matrix.data(), layer, excluded, false));
            }
        }
        for (std::uint32_t a = 0; a < FO4_LAYER_VANILLA_CONFIGURED_COUNT; ++a) {
            for (std::uint32_t b = 0; b < FO4_LAYER_VANILLA_CONFIGURED_COUNT; ++b) {
                if (!isRockOwnedMatrixLayer(a) && !isRockOwnedMatrixLayer(b)) {
                    ok &= expect("generated registration preserves native-to-native matrix bits",
                        maskEnablesLayer(matrix[a], b) == maskEnablesLayer(original[a], b));
                }
            }
        }
    }
    matrix[FO4_LAYER_BIPED_NO_CC] = withLayer(matrix[FO4_LAYER_BIPED_NO_CC], ROCK_LAYER_DYNAMIC_WEAPON_PROXY);
    ok &= expect("NPC row-only drift is detected", !rockDynamicNpcPairsMatch(matrix.data(), false));

    const BodyIdentity player{ 27, 8, 0x100000, 0x200000 };
    ok &= expect("live body identity matches", matchesLiveBody(player, player));
    auto reused = player;
    reused.motionIndex++;
    ok &= expect("recycled motion slot is rejected", !matchesLiveBody(player, reused));
    reused = player; reused.collisionObject++;
    ok &= expect("recycled collision object is rejected", !matchesLiveBody(player, reused));
    reused = player; reused.ownerNode++;
    ok &= expect("recycled scene owner is rejected", !matchesLiveBody(player, reused));
    reused = player; reused.bodyId++;
    ok &= expect("different body ID is rejected", !matchesLiveBody(player, reused));
    ok &= expect("missing identity cannot suppress", !matchesLiveBody({}, {}));

    // Exercise the production compactor with mixed attack, self, world and NPC
    // pairs. Preserve native order, and never access the tail outside its count.
    std::array<BodyPair, 7> pairs{ BodyPair{27, 101}, {102, 27}, {27, 27},
        {201, 101}, {27, 103}, {104, 27}, {999, 999} };
    const auto resolveLayer = [](std::uint32_t id) {
        switch (id) {
        case 101: return FO4_LAYER_STATIC;
        case 102: return FO4_LAYER_PROJECTILE;
        case 103: return ROCK_LAYER_WEAPON;
        case 104: return FO4_LAYER_WEAPON;
        default: return FO4_LAYER_BIPED;
        }
    };
    const auto shouldSuppress = [&](const BodyPair& pair) {
        return suppressPhysicalPair(pair.bodyA == 27, pair.bodyB == 27,
            resolveLayer(pair.bodyA), resolveLayer(pair.bodyB));
    };
    const int kept = filterPhysicalPairs(pairs.data(), 6, shouldSuppress);
    ok &= expect("mixed batch retains projectile, NPC and melee pairs", kept == 3 &&
        pairs[0].bodyA == 102 && pairs[1].bodyA == 201 && pairs[2].bodyA == 104);
    ok &= expect("native batch tail is untouched", pairs[6].bodyA == 999 && pairs[6].bodyB == 999);
    ok &= expect("empty batch remains empty", filterPhysicalPairs(pairs.data(), 0, shouldSuppress) == 0);
    ok &= expect("missing batch cannot be dereferenced", filterPhysicalPairs(nullptr, 2, shouldSuppress) == 2);
    ok &= expect("all-preserved batch retains count", filterPhysicalPairs(pairs.data(), kept,
        [](const BodyPair&) { return false; }) == kept);
    ok &= expect("all-suppressed batch becomes empty", filterPhysicalPairs(pairs.data(), kept,
        [](const BodyPair&) { return true; }) == 0);

    return ok ? 0 : 1;
}
