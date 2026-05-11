#include <cstdio>

#include "api/ROCKProviderApi.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/collision/ContactPipelinePolicy.h"

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

    template <class T>
    bool expectEq(const char* label, T actual, T expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s mismatch\n", label);
        return false;
    }
}

int main()
{
    using namespace rock;
    using namespace rock::contact_pipeline_policy;
    using namespace rock::collision_layer_policy;
    using rock::provider::RockProviderExternalSourceKind;
    using rock::provider::RockProviderHand;

    bool ok = true;

    static_assert(isWorldSurfaceLayer(FO4_LAYER_STATIC));
    static_assert(isWorldSurfaceLayer(FO4_LAYER_ANIMSTATIC));
    static_assert(!isWorldSurfaceLayer(FO4_LAYER_CLUTTER));
    static_assert(isQueryOnlyLayer(FO4_LAYER_ITEMPICK));
    static_assert(isQueryOnlyLayer(FO4_LAYER_LINEOFSIGHT));
    static_assert(isQueryOnlyLayer(FO4_LAYER_PATHPICK));
    static_assert(!isQueryOnlyLayer(ROCK_LAYER_HAND));
    static_assert(isVanillaConfiguredLayer(FO4_LAYER_DROPPINGPICK));
    static_assert(!isVanillaConfiguredLayer(ROCK_LAYER_BODY));
    static_assert(isMatrixAddressableLayer(ROCK_LAYER_BODY));
    static_assert(isRockExtendedLayer(ROCK_LAYER_BODY));
    static_assert(!isMatrixAddressableLayer(64));
    static_assert(layerBitOrZero(64) == 0);

    constexpr std::uint64_t handMask = buildRockHandExpectedMask(true);
    ok &= expectTrue("hand layer collides with weapon layer", (handMask & (1ULL << ROCK_LAYER_WEAPON)) != 0);
    ok &= expectTrue("hand layer collides with live biped", (handMask & (1ULL << FO4_LAYER_BIPED)) != 0);
    ok &= expectTrue("hand layer collides with dead biped", (handMask & (1ULL << FO4_LAYER_DEADBIP)) != 0);
    ok &= expectTrue("hand layer collides with biped no-cc", (handMask & (1ULL << FO4_LAYER_BIPED_NO_CC)) != 0);
    ok &= expectTrue("hand layer physically collides with static world", (handMask & (1ULL << FO4_LAYER_STATIC)) != 0);
    ok &= expectTrue("hand layer physically collides with animstatic world", (handMask & (1ULL << FO4_LAYER_ANIMSTATIC)) != 0);
    constexpr std::uint64_t handMaskNoWorld = buildRockHandExpectedMask(true, false);
    ok &= expectFalse("hand layer can disable static world collision", (handMaskNoWorld & (1ULL << FO4_LAYER_STATIC)) != 0);
    ok &= expectFalse("hand layer can disable animstatic world collision", (handMaskNoWorld & (1ULL << FO4_LAYER_ANIMSTATIC)) != 0);
    constexpr std::uint64_t weaponMaskNoWorld = buildRockWeaponExpectedMask(false, false, false);
    ok &= expectFalse("weapon layer can disable static world collision", (weaponMaskNoWorld & (1ULL << FO4_LAYER_STATIC)) != 0);
    ok &= expectFalse("configured layer mask excludes unused slot 47", (allConfiguredLayerBits() & (1ULL << 47)) != 0);
    ok &= expectTrue("matrix layer mask includes ROCK body slot 47", (allMatrixAddressableLayerBits() & (1ULL << ROCK_LAYER_BODY)) != 0);

    constexpr std::uint64_t bodyMask = buildRockBodyExpectedMask(true);
    ok &= expectTrue("body layer collides with ROCK weapon layer", maskEnablesLayer(bodyMask, ROCK_LAYER_WEAPON));
    ok &= expectTrue("body layer collides with static world", maskEnablesLayer(bodyMask, FO4_LAYER_STATIC));
    ok &= expectTrue("body layer collides with animstatic world", maskEnablesLayer(bodyMask, FO4_LAYER_ANIMSTATIC));
    ok &= expectTrue("body layer collides with clutter", maskEnablesLayer(bodyMask, FO4_LAYER_CLUTTER));
    ok &= expectTrue("body layer collides with weapon props", maskEnablesLayer(bodyMask, FO4_LAYER_WEAPON));
    ok &= expectTrue("body layer collides with small debris", maskEnablesLayer(bodyMask, FO4_LAYER_DEBRIS_SMALL));
    ok &= expectTrue("body layer collides with large debris", maskEnablesLayer(bodyMask, FO4_LAYER_DEBRIS_LARGE));
    ok &= expectFalse("body layer does not collide with hand layer", maskEnablesLayer(bodyMask, ROCK_LAYER_HAND));
    ok &= expectFalse("body layer does not self collide", maskEnablesLayer(bodyMask, ROCK_LAYER_BODY));
    ok &= expectFalse("body layer does not collide with live biped", maskEnablesLayer(bodyMask, FO4_LAYER_BIPED));
    ok &= expectFalse("body layer does not collide with dead biped", maskEnablesLayer(bodyMask, FO4_LAYER_DEADBIP));
    ok &= expectFalse("body layer does not collide with biped no-cc", maskEnablesLayer(bodyMask, FO4_LAYER_BIPED_NO_CC));
    ok &= expectFalse("body layer does not collide with item pick", maskEnablesLayer(bodyMask, FO4_LAYER_ITEMPICK));
    ok &= expectFalse("body layer does not collide with projectile", maskEnablesLayer(bodyMask, FO4_LAYER_PROJECTILE));
    ok &= expectFalse("body layer does not collide with spell", maskEnablesLayer(bodyMask, FO4_LAYER_SPELL));
    ok &= expectFalse("body layer does not collide with cone projectile", maskEnablesLayer(bodyMask, FO4_LAYER_CONEPROJECTILE));
    ok &= expectFalse("body layer does not collide with noncollidable", maskEnablesLayer(bodyMask, FO4_LAYER_NONCOLLIDABLE));
    ok &= expectFalse("body layer does not collide with char controller", maskEnablesLayer(bodyMask, FO4_LAYER_CHARCONTROLLER));

    std::uint64_t matrix[64]{};
    applyRockGeneratedLayerPolicies(matrix, true, true, true, false, false);
    ok &= expectTrue("tool actor pairs are symmetric after policy",
        rockToolActorPairsMatch(matrix, buildRockHandExpectedMask(true), buildRockWeaponExpectedMask(false, false, true, true)));
    ok &= expectTrue("body pairs are symmetric after policy", rockBodyPairsMatch(matrix, bodyMask));
    ok &= expectTrue("body-weapon pair symmetric enabled",
        layerPairSymmetricMatches(matrix, ROCK_LAYER_BODY, ROCK_LAYER_WEAPON, true));
    ok &= expectTrue("body-hand pair symmetric disabled",
        layerPairSymmetricMatches(matrix, ROCK_LAYER_BODY, ROCK_LAYER_HAND, false));
    ok &= expectTrue("body self pair symmetric disabled",
        layerPairSymmetricMatches(matrix, ROCK_LAYER_BODY, ROCK_LAYER_BODY, false));
    ok &= expectTrue("body-query pair symmetric disabled",
        layerPairSymmetricMatches(matrix, ROCK_LAYER_BODY, FO4_LAYER_ITEMPICK, false));

    std::uint64_t bodyActorDriftMatrix[64]{};
    applyRockGeneratedLayerPolicies(bodyActorDriftMatrix, true, true, true, false, false);
    bodyActorDriftMatrix[ROCK_LAYER_BODY] |= (1ULL << FO4_LAYER_BIPED) | (1ULL << FO4_LAYER_BIPED_NO_CC);
    bodyActorDriftMatrix[FO4_LAYER_BIPED] |= (1ULL << ROCK_LAYER_BODY);
    bodyActorDriftMatrix[FO4_LAYER_BIPED_NO_CC] |= (1ULL << ROCK_LAYER_BODY);
    ok &= expectFalse("exact body pair guard catches engine-restored actor body drift",
        rockBodyPairsMatch(bodyActorDriftMatrix, bodyMask));
    ok &= expectTrue("managed body pair guard ignores engine-restored actor body drift",
        rockBodyManagedPairsMatch(bodyActorDriftMatrix, bodyMask));
    ok &= expectTrue("managed body mask ignores engine-restored actor body bits",
        bodyManagedLayerMaskMatches(bodyActorDriftMatrix[ROCK_LAYER_BODY], bodyMask));
    bodyActorDriftMatrix[ROCK_LAYER_BODY] &= ~(1ULL << ROCK_LAYER_WEAPON);
    bodyActorDriftMatrix[ROCK_LAYER_WEAPON] &= ~(1ULL << ROCK_LAYER_BODY);
    ok &= expectFalse("managed body pair guard catches weapon body drift",
        rockBodyManagedPairsMatch(bodyActorDriftMatrix, bodyMask));
    ok &= expectFalse("managed body mask catches weapon body drift",
        bodyManagedLayerMaskMatches(bodyActorDriftMatrix[ROCK_LAYER_BODY], bodyMask));

    matrix[FO4_LAYER_BIPED_NO_CC] &= ~(1ULL << ROCK_LAYER_HAND);
    ok &= expectFalse("tool actor pair guard catches biped no-cc reverse drift",
        rockToolActorPairsMatch(matrix, buildRockHandExpectedMask(true), buildRockWeaponExpectedMask(false, false)));

    ok &= expectTrue("invalid contact body pair is skipped before layer reads",
        shouldSkipContactSignalBeforeLayerRead(ContactSignalPrefilter{
            .bodyIdA = kInvalidContactBodyId,
            .bodyIdB = 10,
            .bodyAIsRockSource = true,
            .bodyBIsRockSource = false,
        }));
    ok &= expectTrue("same contact body pair is skipped before layer reads",
        shouldSkipContactSignalBeforeLayerRead(ContactSignalPrefilter{
            .bodyIdA = 10,
            .bodyIdB = 10,
            .bodyAIsRockSource = true,
            .bodyBIsRockSource = true,
        }));
    ok &= expectTrue("non-rock contact body pair is skipped before layer reads",
        shouldSkipContactSignalBeforeLayerRead(ContactSignalPrefilter{
            .bodyIdA = 10,
            .bodyIdB = 20,
            .bodyAIsRockSource = false,
            .bodyBIsRockSource = false,
        }));
    ok &= expectFalse("rock source contact body pair is kept for layer reads",
        shouldSkipContactSignalBeforeLayerRead(ContactSignalPrefilter{
            .bodyIdA = 10,
            .bodyIdB = 20,
            .bodyAIsRockSource = true,
            .bodyBIsRockSource = false,
        }));

    const ContactEndpoint rightHand{
        .bodyId = 101,
        .layer = ROCK_LAYER_HAND,
        .kind = ContactEndpointKind::RightHand,
    };
    const ContactEndpoint leftHand{
        .bodyId = 102,
        .layer = ROCK_LAYER_HAND,
        .kind = ContactEndpointKind::LeftHand,
    };
    const ContactEndpoint weapon{
        .bodyId = 201,
        .layer = ROCK_LAYER_WEAPON,
        .kind = ContactEndpointKind::Weapon,
    };
    const ContactEndpoint body{
        .bodyId = 251,
        .layer = ROCK_LAYER_BODY,
        .kind = ContactEndpointKind::Body,
    };
    const ContactEndpoint external{
        .bodyId = 301,
        .layer = FO4_LAYER_BIPED,
        .kind = ContactEndpointKind::External,
    };
    const ContactEndpoint rightHeld{
        .bodyId = 401,
        .layer = FO4_LAYER_CLUTTER,
        .kind = ContactEndpointKind::RightHeldObject,
    };
    const ContactEndpoint staticWorld{
        .bodyId = 501,
        .layer = FO4_LAYER_STATIC,
        .kind = ContactEndpointKind::WorldSurface,
    };
    const ContactEndpoint dynamicProp{
        .bodyId = 601,
        .layer = FO4_LAYER_CLUTTER,
        .kind = ContactEndpointKind::DynamicProp,
    };
    const ContactEndpoint actor{
        .bodyId = 701,
        .layer = FO4_LAYER_BIPED,
        .kind = ContactEndpointKind::Actor,
    };

    auto handWeapon = classifyContact(rightHand, weapon);
    ok &= expectEq("hand-weapon route", handWeapon.route, ContactRoute::HandWeapon);
    ok &= expectFalse("hand-weapon is not external provider publication", handWeapon.publishExternalContact);
    ok &= expectTrue("hand-weapon drives weapon support contact", handWeapon.drivesWeaponSupportContact);
    ok &= expectTrue("hand-weapon keeps hand semantic evidence", handWeapon.recordHandSemanticContact);
    ok &= expectFalse("hand-weapon does not drive dynamic push", handWeapon.driveHandDynamicPush);

    auto handExternal = classifyContact(leftHand, external);
    ok &= expectEq("hand-external route", handExternal.route, ContactRoute::HandExternal);
    ok &= expectTrue("hand-external publishes provider contact", handExternal.publishExternalContact);
    ok &= expectEq("hand-external source body", handExternal.sourceBodyId, leftHand.bodyId);
    ok &= expectEq("hand-external target body", handExternal.targetBodyId, external.bodyId);
    ok &= expectEq("hand-external source kind", handExternal.providerSourceKind, RockProviderExternalSourceKind::Hand);
    ok &= expectEq("hand-external source hand", handExternal.providerSourceHand, RockProviderHand::Left);
    ok &= expectTrue("hand-external keeps dynamic push", handExternal.driveHandDynamicPush);

    auto weaponExternal = classifyContact(weapon, external);
    ok &= expectEq("weapon-external route", weaponExternal.route, ContactRoute::WeaponExternal);
    ok &= expectTrue("weapon-external publishes provider contact", weaponExternal.publishExternalContact);
    ok &= expectEq("weapon-external source kind", weaponExternal.providerSourceKind, RockProviderExternalSourceKind::Weapon);
    ok &= expectTrue("weapon-external keeps weapon dynamic push", weaponExternal.driveWeaponDynamicPush);

    auto heldExternal = classifyContact(external, rightHeld);
    ok &= expectEq("held-external route", heldExternal.route, ContactRoute::HeldObjectExternal);
    ok &= expectTrue("held-external publishes provider contact", heldExternal.publishExternalContact);
    ok &= expectEq("held-external source kind", heldExternal.providerSourceKind, RockProviderExternalSourceKind::HeldObject);
    ok &= expectEq("held-external source hand", heldExternal.providerSourceHand, RockProviderHand::Right);

    auto handSurface = classifyContact(rightHand, staticWorld);
    ok &= expectEq("hand-world route", handSurface.route, ContactRoute::HandWorldSurface);
    ok &= expectTrue("hand-world records surface evidence", handSurface.recordWorldSurfaceEvidence);
    ok &= expectFalse("hand-world is not provider contact", handSurface.publishExternalContact);
    ok &= expectFalse("hand-world does not drive dynamic push", handSurface.driveHandDynamicPush);

    auto weaponSurface = classifyContact(weapon, staticWorld);
    ok &= expectEq("weapon-world route", weaponSurface.route, ContactRoute::WeaponWorldSurface);
    ok &= expectTrue("weapon-world records surface evidence", weaponSurface.recordWorldSurfaceEvidence);
    ok &= expectFalse("weapon-world does not drive dynamic push", weaponSurface.driveWeaponDynamicPush);

    auto handProp = classifyContact(rightHand, dynamicProp);
    ok &= expectEq("hand-prop route", handProp.route, ContactRoute::HandPassivePush);
    ok &= expectTrue("hand-prop drives dynamic push", handProp.driveHandDynamicPush);
    ok &= expectTrue("hand-prop keeps hand semantic evidence", handProp.recordHandSemanticContact);

    auto handHeld = classifyContact(leftHand, rightHeld);
    ok &= expectEq("hand-held route keeps semantic contact", handHeld.route, ContactRoute::HandPassivePush);
    ok &= expectTrue("hand-held keeps hand semantic evidence", handHeld.recordHandSemanticContact);
    ok &= expectFalse("hand-held does not drive dynamic push", handHeld.driveHandDynamicPush);

    auto weaponActor = classifyContact(actor, weapon);
    ok &= expectEq("weapon-actor route", weaponActor.route, ContactRoute::WeaponPassivePush);
    ok &= expectTrue("weapon-actor drives dynamic push", weaponActor.driveWeaponDynamicPush);

    auto bodyWeapon = classifyContact(body, weapon);
    ok &= expectEq("body-weapon route", bodyWeapon.route, ContactRoute::BodyContact);
    ok &= expectTrue("body-weapon records internal body contact", bodyWeapon.recordBodyContact);
    ok &= expectFalse("body-weapon does not publish provider contact", bodyWeapon.publishExternalContact);
    ok &= expectFalse("body-weapon does not drive weapon dynamic push", bodyWeapon.driveWeaponDynamicPush);

    auto bodyProp = classifyContact(dynamicProp, body);
    ok &= expectEq("body-prop route", bodyProp.route, ContactRoute::BodyContact);
    ok &= expectTrue("body-prop records body contact", bodyProp.recordBodyContact);
    ok &= expectFalse("body-prop does not drive hand dynamic push", bodyProp.driveHandDynamicPush);

    auto noRock = classifyContact(actor, dynamicProp);
    ok &= expectEq("non-rock pair ignored", noRock.route, ContactRoute::Ignore);
    ok &= expectFalse("non-rock pair does not publish", noRock.publishExternalContact);

    return ok ? 0 : 1;
}
