#pragma once
#include <ROCK/Collision.h>
#include "CommonMarshalling.h"
namespace rock::api::boundary {
    inline void convert(rock::api::collision::WorldRaycastRequestV1& out, const rock::provider::RockProviderWorldRaycastRequestV1& in) {
        convert(out.startGame, in.startGame);
        convert(out.directionGame, in.directionGame);
        out.maxDistanceGame = static_cast<decltype(out.maxDistanceGame)>(in.maxDistanceGame);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderWorldRaycastRequestV1& out, const rock::api::collision::WorldRaycastRequestV1& in) {
        convert(out.startGame, in.startGame);
        convert(out.directionGame, in.directionGame);
        out.maxDistanceGame = static_cast<decltype(out.maxDistanceGame)>(in.maxDistanceGame);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::collision::WorldRaycastResultV1& out, const rock::provider::RockProviderWorldRaycastResultV1& in) {
        out.hit = static_cast<decltype(out.hit)>(in.hit);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.hitFraction = static_cast<decltype(out.hitFraction)>(in.hitFraction);
        out.hitDistanceGame = static_cast<decltype(out.hitDistanceGame)>(in.hitDistanceGame);
        convert(out.hitPointGame, in.hitPointGame);
        convert(out.hitNormalGame, in.hitNormalGame);
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderWorldRaycastResultV1& out, const rock::api::collision::WorldRaycastResultV1& in) {
        out.hit = static_cast<decltype(out.hit)>(in.hit);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.hitFraction = static_cast<decltype(out.hitFraction)>(in.hitFraction);
        out.hitDistanceGame = static_cast<decltype(out.hitDistanceGame)>(in.hitDistanceGame);
        convert(out.hitPointGame, in.hitPointGame);
        convert(out.hitNormalGame, in.hitNormalGame);
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::collision::BodyContactV1& out, const rock::provider::RockProviderBodyContactV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.targetBodyId = static_cast<decltype(out.targetBodyId)>(in.targetBodyId);
        out.bodyLayer = static_cast<decltype(out.bodyLayer)>(in.bodyLayer);
        out.targetLayer = static_cast<decltype(out.targetLayer)>(in.targetLayer);
        out.zone = static_cast<decltype(out.zone)>(in.zone);
        out.side = static_cast<decltype(out.side)>(in.side);
        out.role = static_cast<decltype(out.role)>(in.role);
        out.descriptorIndex = static_cast<decltype(out.descriptorIndex)>(in.descriptorIndex);
        out.targetKind = static_cast<decltype(out.targetKind)>(in.targetKind);
        out.targetZone = static_cast<decltype(out.targetZone)>(in.targetZone);
        out.targetSide = static_cast<decltype(out.targetSide)>(in.targetSide);
        out.targetRole = static_cast<decltype(out.targetRole)>(in.targetRole);
        out.targetDescriptorIndex = static_cast<decltype(out.targetDescriptorIndex)>(in.targetDescriptorIndex);
        out.inPowerArmor = static_cast<decltype(out.inPowerArmor)>(in.inPowerArmor);
        out.targetInPowerArmor = static_cast<decltype(out.targetInPowerArmor)>(in.targetInPowerArmor);
        out.hasContactPointGame = static_cast<decltype(out.hasContactPointGame)>(in.hasContactPointGame);
        out.reserved0 = static_cast<decltype(out.reserved0)>(in.reserved0);
        convert(out.contactPointGame, in.contactPointGame);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderBodyContactV1& out, const rock::api::collision::BodyContactV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.targetBodyId = static_cast<decltype(out.targetBodyId)>(in.targetBodyId);
        out.bodyLayer = static_cast<decltype(out.bodyLayer)>(in.bodyLayer);
        out.targetLayer = static_cast<decltype(out.targetLayer)>(in.targetLayer);
        out.zone = static_cast<decltype(out.zone)>(in.zone);
        out.side = static_cast<decltype(out.side)>(in.side);
        out.role = static_cast<decltype(out.role)>(in.role);
        out.descriptorIndex = static_cast<decltype(out.descriptorIndex)>(in.descriptorIndex);
        out.targetKind = static_cast<decltype(out.targetKind)>(in.targetKind);
        out.targetZone = static_cast<decltype(out.targetZone)>(in.targetZone);
        out.targetSide = static_cast<decltype(out.targetSide)>(in.targetSide);
        out.targetRole = static_cast<decltype(out.targetRole)>(in.targetRole);
        out.targetDescriptorIndex = static_cast<decltype(out.targetDescriptorIndex)>(in.targetDescriptorIndex);
        out.inPowerArmor = static_cast<decltype(out.inPowerArmor)>(in.inPowerArmor);
        out.targetInPowerArmor = static_cast<decltype(out.targetInPowerArmor)>(in.targetInPowerArmor);
        out.hasContactPointGame = static_cast<decltype(out.hasContactPointGame)>(in.hasContactPointGame);
        out.reserved0 = static_cast<decltype(out.reserved0)>(in.reserved0);
        convert(out.contactPointGame, in.contactPointGame);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::collision::ExternalBodyRegistration& out, const rock::provider::RockProviderExternalBodyRegistration& in) {
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.generation = static_cast<decltype(out.generation)>(in.generation);
        out.role = static_cast<decltype(out.role)>(in.role);
        out.contactPolicy = static_cast<decltype(out.contactPolicy)>(in.contactPolicy);
        out.ownerHand = static_cast<decltype(out.ownerHand)>(in.ownerHand);
    }
    inline void convert(rock::provider::RockProviderExternalBodyRegistration& out, const rock::api::collision::ExternalBodyRegistration& in) {
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.generation = static_cast<decltype(out.generation)>(in.generation);
        out.role = static_cast<decltype(out.role)>(in.role);
        out.contactPolicy = static_cast<decltype(out.contactPolicy)>(in.contactPolicy);
        out.ownerHand = static_cast<decltype(out.ownerHand)>(in.ownerHand);
    }
    inline void convert(rock::api::collision::ExternalContactRecordV1& out, const rock::provider::RockProviderExternalContactRecordV1& in) {
        out.parentOwnerToken = static_cast<decltype(out.parentOwnerToken)>(in.parentOwnerToken);
        out.scopeToken = static_cast<decltype(out.scopeToken)>(in.scopeToken);
        out.sequence = static_cast<decltype(out.sequence)>(in.sequence);
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.sourceBodyId = static_cast<decltype(out.sourceBodyId)>(in.sourceBodyId);
        out.targetExternalBodyId = static_cast<decltype(out.targetExternalBodyId)>(in.targetExternalBodyId);
        out.bodyGeneration = static_cast<decltype(out.bodyGeneration)>(in.bodyGeneration);
        out.sourceKind = static_cast<decltype(out.sourceKind)>(in.sourceKind);
        out.sourceHand = static_cast<decltype(out.sourceHand)>(in.sourceHand);
        out.targetRole = static_cast<decltype(out.targetRole)>(in.targetRole);
        out.quality = static_cast<decltype(out.quality)>(in.quality);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        for (std::size_t i=0; i<std::size(out.sourceVelocityHavok); ++i) out.sourceVelocityHavok[i] = in.sourceVelocityHavok[i];
        for (std::size_t i=0; i<std::size(out.contactPointHavok); ++i) out.contactPointHavok[i] = in.contactPointHavok[i];
        for (std::size_t i=0; i<std::size(out.contactNormalHavok); ++i) out.contactNormalHavok[i] = in.contactNormalHavok[i];
        out.contactPointWeightSum = static_cast<decltype(out.contactPointWeightSum)>(in.contactPointWeightSum);
        out.sourcePartKind = static_cast<decltype(out.sourcePartKind)>(in.sourcePartKind);
        out.sourceRole = static_cast<decltype(out.sourceRole)>(in.sourceRole);
        out.sourceSubRole = static_cast<decltype(out.sourceSubRole)>(in.sourceSubRole);
        out.collisionGeneration = static_cast<decltype(out.collisionGeneration)>(in.collisionGeneration);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderExternalContactRecordV1& out, const rock::api::collision::ExternalContactRecordV1& in) {
        out.parentOwnerToken = static_cast<decltype(out.parentOwnerToken)>(in.parentOwnerToken);
        out.scopeToken = static_cast<decltype(out.scopeToken)>(in.scopeToken);
        out.sequence = static_cast<decltype(out.sequence)>(in.sequence);
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.sourceBodyId = static_cast<decltype(out.sourceBodyId)>(in.sourceBodyId);
        out.targetExternalBodyId = static_cast<decltype(out.targetExternalBodyId)>(in.targetExternalBodyId);
        out.bodyGeneration = static_cast<decltype(out.bodyGeneration)>(in.bodyGeneration);
        out.sourceKind = static_cast<decltype(out.sourceKind)>(in.sourceKind);
        out.sourceHand = static_cast<decltype(out.sourceHand)>(in.sourceHand);
        out.targetRole = static_cast<decltype(out.targetRole)>(in.targetRole);
        out.quality = static_cast<decltype(out.quality)>(in.quality);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        for (std::size_t i=0; i<std::size(out.sourceVelocityHavok); ++i) out.sourceVelocityHavok[i] = in.sourceVelocityHavok[i];
        for (std::size_t i=0; i<std::size(out.contactPointHavok); ++i) out.contactPointHavok[i] = in.contactPointHavok[i];
        for (std::size_t i=0; i<std::size(out.contactNormalHavok); ++i) out.contactNormalHavok[i] = in.contactNormalHavok[i];
        out.contactPointWeightSum = static_cast<decltype(out.contactPointWeightSum)>(in.contactPointWeightSum);
        out.sourcePartKind = static_cast<decltype(out.sourcePartKind)>(in.sourcePartKind);
        out.sourceRole = static_cast<decltype(out.sourceRole)>(in.sourceRole);
        out.sourceSubRole = static_cast<decltype(out.sourceSubRole)>(in.sourceSubRole);
        out.collisionGeneration = static_cast<decltype(out.collisionGeneration)>(in.collisionGeneration);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::collision::ExternalContactStreamStateV1& out, const rock::provider::RockProviderExternalContactStreamStateV1& in) {
        out.oldestRetainedSequence = static_cast<decltype(out.oldestRetainedSequence)>(in.oldestRetainedSequence);
        out.latestEmittedSequence = static_cast<decltype(out.latestEmittedSequence)>(in.latestEmittedSequence);
        out.firstCopiedSequence = static_cast<decltype(out.firstCopiedSequence)>(in.firstCopiedSequence);
        out.lastCopiedSequence = static_cast<decltype(out.lastCopiedSequence)>(in.lastCopiedSequence);
        out.overwrittenCount = static_cast<decltype(out.overwrittenCount)>(in.overwrittenCount);
        out.copiedCount = static_cast<decltype(out.copiedCount)>(in.copiedCount);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
    }
    inline void convert(rock::provider::RockProviderExternalContactStreamStateV1& out, const rock::api::collision::ExternalContactStreamStateV1& in) {
        out.oldestRetainedSequence = static_cast<decltype(out.oldestRetainedSequence)>(in.oldestRetainedSequence);
        out.latestEmittedSequence = static_cast<decltype(out.latestEmittedSequence)>(in.latestEmittedSequence);
        out.firstCopiedSequence = static_cast<decltype(out.firstCopiedSequence)>(in.firstCopiedSequence);
        out.lastCopiedSequence = static_cast<decltype(out.lastCopiedSequence)>(in.lastCopiedSequence);
        out.overwrittenCount = static_cast<decltype(out.overwrittenCount)>(in.overwrittenCount);
        out.copiedCount = static_cast<decltype(out.copiedCount)>(in.copiedCount);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
    }
    inline void convert(rock::api::collision::SemanticHandContactV1& out, const rock::provider::RockProviderSemanticHandContactV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.role = static_cast<decltype(out.role)>(in.role);
        out.finger = static_cast<decltype(out.finger)>(in.finger);
        out.segment = static_cast<decltype(out.segment)>(in.segment);
        out.handBodyId = static_cast<decltype(out.handBodyId)>(in.handBodyId);
        out.targetBodyId = static_cast<decltype(out.targetBodyId)>(in.targetBodyId);
        out.targetFormId = static_cast<decltype(out.targetFormId)>(in.targetFormId);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.contactState = static_cast<decltype(out.contactState)>(in.contactState);
        out.framesSinceContact = static_cast<decltype(out.framesSinceContact)>(in.framesSinceContact);
        out.contactSequence = static_cast<decltype(out.contactSequence)>(in.contactSequence);
        convert(out.contactPointGame, in.contactPointGame);
        convert(out.contactNormalGame, in.contactNormalGame);
        out.collisionGeneration = static_cast<decltype(out.collisionGeneration)>(in.collisionGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderSemanticHandContactV1& out, const rock::api::collision::SemanticHandContactV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.role = static_cast<decltype(out.role)>(in.role);
        out.finger = static_cast<decltype(out.finger)>(in.finger);
        out.segment = static_cast<decltype(out.segment)>(in.segment);
        out.handBodyId = static_cast<decltype(out.handBodyId)>(in.handBodyId);
        out.targetBodyId = static_cast<decltype(out.targetBodyId)>(in.targetBodyId);
        out.targetFormId = static_cast<decltype(out.targetFormId)>(in.targetFormId);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.contactState = static_cast<decltype(out.contactState)>(in.contactState);
        out.framesSinceContact = static_cast<decltype(out.framesSinceContact)>(in.framesSinceContact);
        out.contactSequence = static_cast<decltype(out.contactSequence)>(in.contactSequence);
        convert(out.contactPointGame, in.contactPointGame);
        convert(out.contactNormalGame, in.contactNormalGame);
        out.collisionGeneration = static_cast<decltype(out.collisionGeneration)>(in.collisionGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::collision::PlayerColliderDescriptorV1& out, const rock::provider::RockProviderPlayerColliderDescriptorV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.kind = static_cast<decltype(out.kind)>(in.kind);
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.role = static_cast<decltype(out.role)>(in.role);
        out.zone = static_cast<decltype(out.zone)>(in.zone);
        out.side = static_cast<decltype(out.side)>(in.side);
        out.descriptorIndex = static_cast<decltype(out.descriptorIndex)>(in.descriptorIndex);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.lengthGameUnits = static_cast<decltype(out.lengthGameUnits)>(in.lengthGameUnits);
        out.radiusGameUnits = static_cast<decltype(out.radiusGameUnits)>(in.radiusGameUnits);
        convert(out.transform, in.transform);
        out.collisionGeneration = static_cast<decltype(out.collisionGeneration)>(in.collisionGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderPlayerColliderDescriptorV1& out, const rock::api::collision::PlayerColliderDescriptorV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.kind = static_cast<decltype(out.kind)>(in.kind);
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.role = static_cast<decltype(out.role)>(in.role);
        out.zone = static_cast<decltype(out.zone)>(in.zone);
        out.side = static_cast<decltype(out.side)>(in.side);
        out.descriptorIndex = static_cast<decltype(out.descriptorIndex)>(in.descriptorIndex);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.lengthGameUnits = static_cast<decltype(out.lengthGameUnits)>(in.lengthGameUnits);
        out.radiusGameUnits = static_cast<decltype(out.radiusGameUnits)>(in.radiusGameUnits);
        convert(out.transform, in.transform);
        out.collisionGeneration = static_cast<decltype(out.collisionGeneration)>(in.collisionGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::collision::HandCollisionAvailabilityV1& out, const rock::provider::RockProviderHandCollisionAvailabilityV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.collisionSequence = static_cast<decltype(out.collisionSequence)>(in.collisionSequence);
        out.collisionGeneration = static_cast<decltype(out.collisionGeneration)>(in.collisionGeneration);
        out.handBodyCount = static_cast<decltype(out.handBodyCount)>(in.handBodyCount);
        out.dynamicTwinCount = static_cast<decltype(out.dynamicTwinCount)>(in.dynamicTwinCount);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.otherHandContactSlots = in.reserved[0];
        out.weaponContactSlots = in.reserved[1];
        out.dynamicCollisionLayer = in.reserved[2];
        out.pairSuppressionLeaseCount = in.reserved[3];
        out.collisionEnabledBodyCount = static_cast<decltype(out.collisionEnabledBodyCount)>(in.collisionEnabledBodyCount);
        out.filterKnownBodyCount = static_cast<decltype(out.filterKnownBodyCount)>(in.filterKnownBodyCount);
    }
    inline void convert(rock::provider::RockProviderHandCollisionAvailabilityV1& out, const rock::api::collision::HandCollisionAvailabilityV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.collisionSequence = static_cast<decltype(out.collisionSequence)>(in.collisionSequence);
        out.collisionGeneration = static_cast<decltype(out.collisionGeneration)>(in.collisionGeneration);
        out.handBodyCount = static_cast<decltype(out.handBodyCount)>(in.handBodyCount);
        out.dynamicTwinCount = static_cast<decltype(out.dynamicTwinCount)>(in.dynamicTwinCount);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.reserved[0] = in.otherHandContactSlots;
        out.reserved[1] = in.weaponContactSlots;
        out.reserved[2] = in.dynamicCollisionLayer;
        out.reserved[3] = in.pairSuppressionLeaseCount;
        out.collisionEnabledBodyCount = static_cast<decltype(out.collisionEnabledBodyCount)>(in.collisionEnabledBodyCount);
        out.filterKnownBodyCount = static_cast<decltype(out.filterKnownBodyCount)>(in.filterKnownBodyCount);
    }
}
