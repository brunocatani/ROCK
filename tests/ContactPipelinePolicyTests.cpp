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

    constexpr std::uint64_t handMask = buildRockHandExpectedMask(true);
    ok &= expectTrue("hand layer collides with weapon layer", (handMask & (1ULL << ROCK_LAYER_WEAPON)) != 0);
    ok &= expectFalse("hand layer does not physically collide with static world", (handMask & (1ULL << FO4_LAYER_STATIC)) != 0);
    ok &= expectFalse("hand layer does not physically collide with animstatic world", (handMask & (1ULL << FO4_LAYER_ANIMSTATIC)) != 0);

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

    auto noRock = classifyContact(actor, dynamicProp);
    ok &= expectEq("non-rock pair ignored", noRock.route, ContactRoute::Ignore);
    ok &= expectFalse("non-rock pair does not publish", noRock.publishExternalContact);

    return ok ? 0 : 1;
}
