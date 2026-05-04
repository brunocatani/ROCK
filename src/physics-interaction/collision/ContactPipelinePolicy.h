#pragma once

#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "api/ROCKProviderApi.h"

#include <cstdint>

namespace rock::contact_pipeline_policy
{
    /*
     * Contact handling is split into capture, classification, and routing so
     * ROCK can keep Havok event decoding narrow while giving every consumer a
     * stable semantic contract. HIGGS uses listener callbacks to first decide
     * whether a body pair is one of its hand/weapon/held contacts, then routes
     * haptics, contact state, and ignored-contact behavior separately. ROCK's
     * FO4VR hknp path follows that architecture here without copying Skyrim's
     * hkp event types or layer values.
     */

    inline constexpr std::uint32_t kInvalidContactBodyId = 0x7FFF'FFFFu;
    inline constexpr std::uint32_t kInvalidContactBodyIdAllBits = 0xFFFF'FFFFu;
    inline constexpr std::uint32_t kUnknownLayer = 0xFFFF'FFFFu;

    enum class ContactEndpointKind : std::uint32_t
    {
        Unknown = 0,
        RightHand,
        LeftHand,
        Weapon,
        RightHeldObject,
        LeftHeldObject,
        External,
        WorldSurface,
        DynamicProp,
        Actor,
        QueryOnly,
    };

    enum class ContactRoute : std::uint32_t
    {
        Ignore = 0,
        HandExternal,
        WeaponExternal,
        HeldObjectExternal,
        HandWeapon,
        HandWorldSurface,
        WeaponWorldSurface,
        HandPassivePush,
        WeaponPassivePush,
        HeldObjectOther,
        RockInternal,
        UnknownRockContact,
    };

    struct ContactEndpoint
    {
        std::uint32_t bodyId = kInvalidContactBodyId;
        std::uint32_t layer = kUnknownLayer;
        ContactEndpointKind kind = ContactEndpointKind::Unknown;
    };

    struct ContactClassification
    {
        ContactRoute route = ContactRoute::Ignore;
        ContactEndpoint source{};
        ContactEndpoint target{};
        std::uint32_t sourceBodyId = kInvalidContactBodyId;
        std::uint32_t targetBodyId = kInvalidContactBodyId;
        bool publishExternalContact = false;
        bool recordHandSemanticContact = false;
        bool recordWorldSurfaceEvidence = false;
        bool drivesWeaponSupportContact = false;
        bool driveHandDynamicPush = false;
        bool driveWeaponDynamicPush = false;
        ::rock::provider::RockProviderExternalSourceKind providerSourceKind{ ::rock::provider::RockProviderExternalSourceKind::Unknown };
        ::rock::provider::RockProviderHand providerSourceHand{ ::rock::provider::RockProviderHand::None };
    };

    inline constexpr bool isValidBodyId(std::uint32_t bodyId)
    {
        return bodyId != kInvalidContactBodyId && bodyId != kInvalidContactBodyIdAllBits;
    }

    struct ContactSignalPrefilter
    {
        std::uint32_t bodyIdA = kInvalidContactBodyId;
        std::uint32_t bodyIdB = kInvalidContactBodyId;
        bool bodyAIsRockSource = false;
        bool bodyBIsRockSource = false;
    };

    inline constexpr bool shouldSkipContactSignalBeforeLayerRead(const ContactSignalPrefilter& prefilter)
    {
        if (!isValidBodyId(prefilter.bodyIdA) || !isValidBodyId(prefilter.bodyIdB) || prefilter.bodyIdA == prefilter.bodyIdB) {
            return true;
        }

        return !prefilter.bodyAIsRockSource && !prefilter.bodyBIsRockSource;
    }

    inline constexpr bool isRightHand(ContactEndpointKind kind)
    {
        return kind == ContactEndpointKind::RightHand;
    }

    inline constexpr bool isLeftHand(ContactEndpointKind kind)
    {
        return kind == ContactEndpointKind::LeftHand;
    }

    inline constexpr bool isHand(ContactEndpointKind kind)
    {
        return isRightHand(kind) || isLeftHand(kind);
    }

    inline constexpr bool isHeldObject(ContactEndpointKind kind)
    {
        return kind == ContactEndpointKind::RightHeldObject || kind == ContactEndpointKind::LeftHeldObject;
    }

    inline constexpr bool isRockSource(ContactEndpointKind kind)
    {
        return isHand(kind) || kind == ContactEndpointKind::Weapon || isHeldObject(kind);
    }

    inline constexpr bool isExternal(ContactEndpointKind kind)
    {
        return kind == ContactEndpointKind::External;
    }

    inline constexpr bool isWorldSurface(ContactEndpointKind kind)
    {
        return kind == ContactEndpointKind::WorldSurface;
    }

    inline constexpr bool isRightOwned(ContactEndpointKind kind)
    {
        return kind == ContactEndpointKind::RightHand || kind == ContactEndpointKind::RightHeldObject;
    }

    inline constexpr bool isLeftOwned(ContactEndpointKind kind)
    {
        return kind == ContactEndpointKind::LeftHand || kind == ContactEndpointKind::LeftHeldObject;
    }

    inline constexpr ::rock::provider::RockProviderHand providerHandFor(ContactEndpointKind kind)
    {
        if (isLeftOwned(kind)) {
            return ::rock::provider::RockProviderHand::Left;
        }
        if (isRightOwned(kind)) {
            return ::rock::provider::RockProviderHand::Right;
        }
        return ::rock::provider::RockProviderHand::None;
    }

    inline constexpr ::rock::provider::RockProviderExternalSourceKind providerSourceKindFor(ContactEndpointKind kind)
    {
        if (isHand(kind)) {
            return ::rock::provider::RockProviderExternalSourceKind::Hand;
        }
        if (kind == ContactEndpointKind::Weapon) {
            return ::rock::provider::RockProviderExternalSourceKind::Weapon;
        }
        if (isHeldObject(kind)) {
            return ::rock::provider::RockProviderExternalSourceKind::HeldObject;
        }
        return ::rock::provider::RockProviderExternalSourceKind::Unknown;
    }

    inline constexpr bool canDriveDynamicPush(ContactEndpointKind targetKind)
    {
        return targetKind != ContactEndpointKind::Unknown && targetKind != ContactEndpointKind::WorldSurface && targetKind != ContactEndpointKind::QueryOnly &&
               !isRockSource(targetKind);
    }

    inline constexpr ContactEndpointKind classifyNonRockLayer(std::uint32_t layer)
    {
        if (collision_layer_policy::isWorldSurfaceLayer(layer)) {
            return ContactEndpointKind::WorldSurface;
        }
        if (collision_layer_policy::isDynamicPropInteractionLayer(layer)) {
            return ContactEndpointKind::DynamicProp;
        }
        if (collision_layer_policy::isActorOrBipedLayer(layer)) {
            return ContactEndpointKind::Actor;
        }
        if (collision_layer_policy::isQueryOnlyLayer(layer)) {
            return ContactEndpointKind::QueryOnly;
        }
        return ContactEndpointKind::Unknown;
    }

    inline constexpr const char* routeName(ContactRoute route)
    {
        switch (route) {
        case ContactRoute::Ignore:
            return "Ignore";
        case ContactRoute::HandExternal:
            return "HandExternal";
        case ContactRoute::WeaponExternal:
            return "WeaponExternal";
        case ContactRoute::HeldObjectExternal:
            return "HeldObjectExternal";
        case ContactRoute::HandWeapon:
            return "HandWeapon";
        case ContactRoute::HandWorldSurface:
            return "HandWorldSurface";
        case ContactRoute::WeaponWorldSurface:
            return "WeaponWorldSurface";
        case ContactRoute::HandPassivePush:
            return "HandPassivePush";
        case ContactRoute::WeaponPassivePush:
            return "WeaponPassivePush";
        case ContactRoute::HeldObjectOther:
            return "HeldObjectOther";
        case ContactRoute::RockInternal:
            return "RockInternal";
        case ContactRoute::UnknownRockContact:
            return "UnknownRockContact";
        default:
            return "Unknown";
        }
    }

    inline constexpr ContactEndpoint makeNonRockEndpoint(std::uint32_t bodyId, std::uint32_t layer)
    {
        return ContactEndpoint{ .bodyId = bodyId, .layer = layer, .kind = classifyNonRockLayer(layer) };
    }

    inline constexpr ContactClassification makeClassification(ContactRoute route, ContactEndpoint source, ContactEndpoint target)
    {
        ContactClassification result{};
        result.route = route;
        result.source = source;
        result.target = target;
        result.sourceBodyId = source.bodyId;
        result.targetBodyId = target.bodyId;
        return result;
    }

    inline constexpr ContactClassification classifyContact(ContactEndpoint a, ContactEndpoint b)
    {
        if (!isValidBodyId(a.bodyId) || !isValidBodyId(b.bodyId) || a.bodyId == b.bodyId) {
            return {};
        }

        const bool aRock = isRockSource(a.kind);
        const bool bRock = isRockSource(b.kind);
        if (!aRock && !bRock) {
            return {};
        }

        if (aRock && bRock) {
            if (isHand(a.kind) && b.kind == ContactEndpointKind::Weapon) {
                auto result = makeClassification(ContactRoute::HandWeapon, a, b);
                result.recordHandSemanticContact = true;
                result.drivesWeaponSupportContact = true;
                return result;
            }
            if (a.kind == ContactEndpointKind::Weapon && isHand(b.kind)) {
                auto result = makeClassification(ContactRoute::HandWeapon, b, a);
                result.recordHandSemanticContact = true;
                result.drivesWeaponSupportContact = true;
                return result;
            }
            if (isHand(a.kind) && isHeldObject(b.kind)) {
                auto result = makeClassification(ContactRoute::HandPassivePush, a, b);
                result.recordHandSemanticContact = true;
                return result;
            }
            if (isHeldObject(a.kind) && isHand(b.kind)) {
                auto result = makeClassification(ContactRoute::HandPassivePush, b, a);
                result.recordHandSemanticContact = true;
                return result;
            }
            if (isHeldObject(a.kind) != isHeldObject(b.kind)) {
                return makeClassification(ContactRoute::HeldObjectOther, isHeldObject(a.kind) ? a : b, isHeldObject(a.kind) ? b : a);
            }
            return makeClassification(ContactRoute::RockInternal, a, b);
        }

        const ContactEndpoint rock = aRock ? a : b;
        const ContactEndpoint other = aRock ? b : a;

        if (isExternal(other.kind)) {
            auto result = makeClassification(
                isHand(rock.kind)         ? ContactRoute::HandExternal :
                rock.kind == ContactEndpointKind::Weapon ? ContactRoute::WeaponExternal :
                isHeldObject(rock.kind)   ? ContactRoute::HeldObjectExternal :
                                            ContactRoute::UnknownRockContact,
                rock,
                other);
            result.publishExternalContact = result.route != ContactRoute::UnknownRockContact;
            result.providerSourceKind = providerSourceKindFor(rock.kind);
            result.providerSourceHand = providerHandFor(rock.kind);
            result.recordHandSemanticContact = isHand(rock.kind);
            result.driveHandDynamicPush = isHand(rock.kind) && canDriveDynamicPush(other.kind);
            result.driveWeaponDynamicPush = rock.kind == ContactEndpointKind::Weapon && canDriveDynamicPush(other.kind);
            return result;
        }

        if (isWorldSurface(other.kind)) {
            if (isHand(rock.kind)) {
                auto result = makeClassification(ContactRoute::HandWorldSurface, rock, other);
                result.recordHandSemanticContact = true;
                result.recordWorldSurfaceEvidence = true;
                return result;
            }
            if (rock.kind == ContactEndpointKind::Weapon) {
                auto result = makeClassification(ContactRoute::WeaponWorldSurface, rock, other);
                result.recordWorldSurfaceEvidence = true;
                return result;
            }
        }

        if (isHand(rock.kind)) {
            auto result = makeClassification(ContactRoute::HandPassivePush, rock, other);
            result.recordHandSemanticContact = true;
            result.driveHandDynamicPush = canDriveDynamicPush(other.kind);
            return result;
        }

        if (rock.kind == ContactEndpointKind::Weapon) {
            auto result = makeClassification(ContactRoute::WeaponPassivePush, rock, other);
            result.driveWeaponDynamicPush = canDriveDynamicPush(other.kind);
            return result;
        }

        if (isHeldObject(rock.kind)) {
            return makeClassification(ContactRoute::HeldObjectOther, rock, other);
        }

        return makeClassification(ContactRoute::UnknownRockContact, rock, other);
    }
}
