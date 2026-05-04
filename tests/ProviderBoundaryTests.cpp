#include <cstddef>
#include <cstdio>
#include <type_traits>

#include "api/ROCKProviderApi.h"
#include "physics-interaction/ExternalBodyRegistry.h"
#include "physics-interaction/OffhandInteractionReservation.h"

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

    bool expectUInt(const char* label, std::uint32_t actual, std::uint32_t expected)
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
    using namespace rock::provider;
    using namespace frik::rock;

    static_assert(std::is_standard_layout_v<RockProviderFrameSnapshot>);
    static_assert(std::is_trivially_copyable_v<RockProviderFrameSnapshot>);
    static_assert(sizeof(RockProviderTransform) == 52);
    static_assert(sizeof(RockProviderFrameSnapshot) == 256);
    static_assert(alignof(RockProviderFrameSnapshot) == 8);
    static_assert(offsetof(RockProviderFrameSnapshot, gameToHavokScale) > offsetof(RockProviderFrameSnapshot, externalBodyCount));
    static_assert(offsetof(RockProviderFrameSnapshot, havokToGameScale) > offsetof(RockProviderFrameSnapshot, gameToHavokScale));
    static_assert(offsetof(RockProviderFrameSnapshot, physicsScaleRevision) > offsetof(RockProviderFrameSnapshot, havokToGameScale));
    static_assert(sizeof(RockProviderExternalBodyRegistration) == 32);
    static_assert(sizeof(RockProviderExternalContact) == 40);
    static_assert(ROCK_PROVIDER_API_VERSION >= 3);
    static_assert(sizeof(RockProviderExternalContactV2) == 128);
    static_assert(alignof(RockProviderExternalContactV2) == 8);
    static_assert(sizeof(RockProviderPoint3) == 12);
    static_assert(sizeof(RockProviderBounds3) == 32);
    static_assert(sizeof(RockProviderWeaponEvidenceDetailV3) == 192);
    static_assert(alignof(RockProviderWeaponEvidenceDetailV3) == 8);
    static_assert(std::is_standard_layout_v<RockProviderWeaponEvidenceDetailV3>);
    static_assert(std::is_trivially_copyable_v<RockProviderWeaponEvidenceDetailV3>);
    static_assert(offsetof(RockProviderApi, getWeaponEvidenceDetailCountV3) > offsetof(RockProviderApi, getExternalContactSnapshotV2));

    bool ok = true;

    ExternalBodyRegistry registry;
    const RockProviderExternalBodyRegistration firstBatch[] = {
        RockProviderExternalBodyRegistration{
            .size = sizeof(RockProviderExternalBodyRegistration),
            .bodyId = 101,
            .ownerToken = 0xABC,
            .generation = 7,
            .role = RockProviderExternalBodyRole::ReloadMobile,
            .contactPolicy = RockProviderExternalBodyContactPolicy::ReportHandContacts,
            .ownerHand = RockProviderHand::Left,
        },
        RockProviderExternalBodyRegistration{
            .size = sizeof(RockProviderExternalBodyRegistration),
            .bodyId = 102,
            .ownerToken = 0xABC,
            .generation = 7,
            .role = RockProviderExternalBodyRole::ReloadSocket,
            .contactPolicy = RockProviderExternalBodyContactPolicy::ReportHandContacts,
            .ownerHand = RockProviderHand::None,
        },
    };

    ok &= expectBool("registry accepts owner batch", registry.registerBodies(0xABC, firstBatch, 2), true);
    ok &= expectBool("registry recognizes first external body", registry.containsBody(101), true);
    ok &= expectBool("registry recognizes second external body", registry.containsBody(102), true);
    ok &= expectUInt("registry body count after batch", registry.bodyCount(), 2);

    registry.recordHandContact(true, 501, 101, 77);
    RockProviderExternalContact contacts[2]{};
    ok &= expectUInt("registry copies one external contact", registry.copyContacts(contacts, 2), 1);
    ok &= expectBool("contact records left hand", contacts[0].hand == RockProviderHand::Left, true);
    ok &= expectUInt("contact records external body", contacts[0].externalBodyId, 101);
    ok &= expectUInt("contact records owner generation", contacts[0].generation, 7);

    RockProviderExternalContactV2 v2Contacts[2]{};
    ok &= expectUInt("registry copies one v2 external contact", registry.copyContactsV2(v2Contacts, 2), 1);
    ok &= expectUInt("v2 contact records source body", v2Contacts[0].sourceBodyId, 501);
    ok &= expectUInt("v2 contact records target body", v2Contacts[0].targetExternalBodyId, 101);
    ok &= expectBool("v2 contact records source kind", v2Contacts[0].sourceKind == RockProviderExternalSourceKind::Hand, true);
    ok &= expectBool("v2 contact records body-pair quality", v2Contacts[0].quality == RockProviderExternalContactQuality::BodyPairOnly, true);

    ok &= expectBool("registry accepts same-owner body refresh", registry.registerBodies(0xABC, firstBatch, 2), true);
    ok &= expectUInt("same-owner body refresh preserves pending v2 contact", registry.copyContactsV2(v2Contacts, 2), 1);
    ok &= expectBool("same-owner body refresh keeps contact sequence", v2Contacts[0].sequence == 1, true);

    registry.clearOwner(0xABC);
    ok &= expectUInt("registry clears owner batch", registry.bodyCount(), 0);
    ok &= expectUInt("registry clears owner contacts", registry.copyContacts(contacts, 2), 0);
    ok &= expectUInt("registry clears owner v2 contacts", registry.copyContactsV2(v2Contacts, 2), 0);

    RockProviderExternalBodyRegistration ragdollBody{
        .size = sizeof(RockProviderExternalBodyRegistration),
        .bodyId = 201,
        .ownerToken = 0xBAD,
        .generation = 9,
        .role = RockProviderExternalBodyRole::ActorRagdollBone,
        .contactPolicy = static_cast<RockProviderExternalBodyContactPolicy>(
            static_cast<std::uint32_t>(RockProviderExternalBodyContactPolicy::ReportHandContacts) |
            static_cast<std::uint32_t>(RockProviderExternalBodyContactPolicy::ReportAllSourceKinds) |
            static_cast<std::uint32_t>(RockProviderExternalBodyContactPolicy::SuppressRockDynamicPush)),
        .ownerHand = RockProviderHand::None,
    };
    ok &= expectBool("registry accepts actor ragdoll external body", registry.registerBodies(0xBAD, &ragdollBody, 1), true);
    ok &= expectBool("registry reports suppress dynamic push policy", registry.suppressesRockDynamicPush(201), true);

    RockProviderExternalContactV2 weaponContact{};
    weaponContact.size = sizeof(RockProviderExternalContactV2);
    weaponContact.sourceBodyId = 601;
    weaponContact.targetExternalBodyId = 201;
    weaponContact.frameIndex = 88;
    weaponContact.sourceKind = RockProviderExternalSourceKind::Weapon;
    weaponContact.quality = RockProviderExternalContactQuality::AggregateImpulse;
    weaponContact.contactPointWeightSum = 1.25f;
    ok &= expectBool("registry records all-source v2 contact", registry.recordContactV2(weaponContact), true);
    ok &= expectUInt("registry exposes all-source v2 contact", registry.copyContactsV2(v2Contacts, 2), 1);
    ok &= expectBool("all-source v2 contact records weapon source", v2Contacts[0].sourceKind == RockProviderExternalSourceKind::Weapon, true);
    ok &= expectUInt("all-source v2 contact inherits owner generation", v2Contacts[0].generation, 9);

    RockProviderExternalContactV2 rawPointContact{};
    rawPointContact.size = sizeof(RockProviderExternalContactV2);
    rawPointContact.sourceBodyId = 602;
    rawPointContact.targetExternalBodyId = 201;
    rawPointContact.frameIndex = 89;
    rawPointContact.sourceKind = RockProviderExternalSourceKind::HeldObject;
    rawPointContact.quality = RockProviderExternalContactQuality::RawPoint;
    rawPointContact.contactPointHavok[0] = 1.0f;
    rawPointContact.contactPointHavok[1] = 2.0f;
    rawPointContact.contactPointHavok[2] = 3.0f;
    rawPointContact.contactNormalHavok[1] = 1.0f;
    rawPointContact.contactPointWeightSum = 4.0f;
    ok &= expectBool("registry records raw-point v2 contact", registry.recordContactV2(rawPointContact), true);
    ok &= expectUInt("registry exposes raw-point contact after weapon contact", registry.copyContactsV2(v2Contacts, 2), 2);
    ok &= expectBool("raw-point contact preserves quality", v2Contacts[1].quality == RockProviderExternalContactQuality::RawPoint, true);
    ok &= expectUInt("raw-point contact preserves source body", v2Contacts[1].sourceBodyId, 602);

    registry.clearOwner(0);
    RockProviderExternalBodyRegistration windowBody{
        .size = sizeof(RockProviderExternalBodyRegistration),
        .bodyId = 301,
        .ownerToken = 0x123,
        .generation = 11,
        .role = RockProviderExternalBodyRole::ActorRagdollBone,
        .contactPolicy = RockProviderExternalBodyContactPolicy::ReportAllSourceKinds,
        .ownerHand = RockProviderHand::None,
    };
    ok &= expectBool("registry accepts contact-window body", registry.registerBodies(0x123, &windowBody, 1), true);
    for (std::uint32_t source = 701; source <= 703; ++source) {
        RockProviderExternalContactV2 contact{};
        contact.size = sizeof(RockProviderExternalContactV2);
        contact.sourceBodyId = source;
        contact.targetExternalBodyId = 301;
        contact.sourceKind = RockProviderExternalSourceKind::Hand;
        ok &= expectBool("registry records contact-window event", registry.recordContactV2(contact), true);
    }
    RockProviderExternalContactV2 latestContacts[2]{};
    ok &= expectUInt("registry exposes requested latest contact window", registry.copyContactsV2(latestContacts, 2), 2);
    ok &= expectUInt("latest window starts at second retained contact", latestContacts[0].sourceBodyId, 702);
    ok &= expectUInt("latest window ends at newest contact", latestContacts[1].sourceBodyId, 703);
    RockProviderExternalContact latestLegacyContacts[2]{};
    ok &= expectUInt("registry exposes requested latest legacy contact window", registry.copyContacts(latestLegacyContacts, 2), 2);
    ok &= expectUInt("latest legacy window starts at second retained contact", latestLegacyContacts[0].handBodyId, 702);
    ok &= expectUInt("latest legacy window ends at newest contact", latestLegacyContacts[1].handBodyId, 703);

    ok &= expectBool("normal offhand reservation allows support grip",
        offhand_interaction_reservation::allowsSupportGrip(OffhandInteractionReservation::Normal),
        true);
    ok &= expectBool("reload reserved offhand blocks support grip",
        offhand_interaction_reservation::allowsSupportGrip(OffhandInteractionReservation::ReloadReserved),
        false);
    ok &= expectBool("reload pose override blocks support grip",
        offhand_interaction_reservation::allowsSupportGrip(OffhandInteractionReservation::ReloadPoseOverride),
        false);

    return ok ? 0 : 1;
}
