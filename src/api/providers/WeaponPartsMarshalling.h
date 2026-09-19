#pragma once
#include <ROCK/WeaponParts.h>
#include "CommonMarshalling.h"
namespace rock::api::boundary {
    inline void convert(rock::api::weaponparts::WeaponPartTargetV1& out, const rock::provider::RockProviderWeaponPartTargetV1& in) {
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.grabMode = static_cast<decltype(out.grabMode)>(in.grabMode);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.sourceKey = provider::runtime::sourceKey(in.weaponGenerationKey, in.sourceRoot);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.partKind = static_cast<decltype(out.partKind)>(in.partKind);
        out.reloadRole = static_cast<decltype(out.reloadRole)>(in.reloadRole);
        out.supportRole = static_cast<decltype(out.supportRole)>(in.supportRole);
        out.socketRole = static_cast<decltype(out.socketRole)>(in.socketRole);
        out.actionRole = static_cast<decltype(out.actionRole)>(in.actionRole);
        out.groupId = static_cast<decltype(out.groupId)>(in.groupId);
        out.priority = static_cast<decltype(out.priority)>(in.priority);
        for (std::size_t i=0; i<std::size(out.sourceName); ++i) out.sourceName[i] = in.sourceName[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderWeaponPartTargetV1& out, const rock::api::weaponparts::WeaponPartTargetV1& in) {
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.grabMode = static_cast<decltype(out.grabMode)>(in.grabMode);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.sourceRoot = provider::runtime::resolveSourceKey(in.weaponGenerationKey, in.sourceKey);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.partKind = static_cast<decltype(out.partKind)>(in.partKind);
        out.reloadRole = static_cast<decltype(out.reloadRole)>(in.reloadRole);
        out.supportRole = static_cast<decltype(out.supportRole)>(in.supportRole);
        out.socketRole = static_cast<decltype(out.socketRole)>(in.socketRole);
        out.actionRole = static_cast<decltype(out.actionRole)>(in.actionRole);
        out.groupId = static_cast<decltype(out.groupId)>(in.groupId);
        out.priority = static_cast<decltype(out.priority)>(in.priority);
        for (std::size_t i=0; i<std::size(out.sourceName); ++i) out.sourceName[i] = in.sourceName[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::weaponparts::WeaponPartDriveTargetV1& out, const rock::provider::RockProviderWeaponPartDriveTargetV1& in) {
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.driveSpace = static_cast<decltype(out.driveSpace)>(in.driveSpace);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.sourceKey = provider::runtime::sourceKey(in.weaponGenerationKey, in.sourceRoot);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.groupId = static_cast<decltype(out.groupId)>(in.groupId);
        out.priority = static_cast<decltype(out.priority)>(in.priority);
        out.leaseFrames = static_cast<decltype(out.leaseFrames)>(in.leaseFrames);
        convert(out.targetTransform, in.targetTransform);
        for (std::size_t i=0; i<std::size(out.sourceName); ++i) out.sourceName[i] = in.sourceName[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderWeaponPartDriveTargetV1& out, const rock::api::weaponparts::WeaponPartDriveTargetV1& in) {
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.driveSpace = static_cast<decltype(out.driveSpace)>(in.driveSpace);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.sourceRoot = provider::runtime::resolveSourceKey(in.weaponGenerationKey, in.sourceKey);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.groupId = static_cast<decltype(out.groupId)>(in.groupId);
        out.priority = static_cast<decltype(out.priority)>(in.priority);
        out.leaseFrames = static_cast<decltype(out.leaseFrames)>(in.leaseFrames);
        convert(out.targetTransform, in.targetTransform);
        for (std::size_t i=0; i<std::size(out.sourceName); ++i) out.sourceName[i] = in.sourceName[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::weaponparts::WeaponPartGripStateV1& out, const rock::provider::RockProviderWeaponPartGripStateV1& in) {
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.gripKind = static_cast<decltype(out.gripKind)>(in.gripKind);
        out.active = static_cast<decltype(out.active)>(in.active);
        out.attachOnly = static_cast<decltype(out.attachOnly)>(in.attachOnly);
        out.gripSequence = static_cast<decltype(out.gripSequence)>(in.gripSequence);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.partKind = static_cast<decltype(out.partKind)>(in.partKind);
        out.reloadRole = static_cast<decltype(out.reloadRole)>(in.reloadRole);
        out.supportRole = static_cast<decltype(out.supportRole)>(in.supportRole);
        out.socketRole = static_cast<decltype(out.socketRole)>(in.socketRole);
        out.actionRole = static_cast<decltype(out.actionRole)>(in.actionRole);
        out.sourceKey = provider::runtime::sourceKey(in.weaponGenerationKey, in.sourceRoot);
        out.providerOwnerToken = static_cast<decltype(out.providerOwnerToken)>(in.providerOwnerToken);
        out.providerGroupId = static_cast<decltype(out.providerGroupId)>(in.providerGroupId);
        out.providerGrabMode = static_cast<decltype(out.providerGrabMode)>(in.providerGrabMode);
        out.hasHandPartLocal = static_cast<decltype(out.hasHandPartLocal)>(in.hasHandPartLocal);
        out.handPartLocalSpace = static_cast<decltype(out.handPartLocalSpace)>(in.handPartLocalSpace);
        convert(out.handPartLocal, in.handPartLocal);
        for (std::size_t i=0; i<std::size(out.sourceName); ++i) out.sourceName[i] = in.sourceName[i];
        out.omodFormId = static_cast<decltype(out.omodFormId)>(in.omodFormId);
        out.attachPointFormId = static_cast<decltype(out.attachPointFormId)>(in.attachPointFormId);
        out.classificationSource = static_cast<decltype(out.classificationSource)>(in.classificationSource);
        out.authoredSupportGrip = static_cast<decltype(out.authoredSupportGrip)>(in.authoredSupportGrip);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderWeaponPartGripStateV1& out, const rock::api::weaponparts::WeaponPartGripStateV1& in) {
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.gripKind = static_cast<decltype(out.gripKind)>(in.gripKind);
        out.active = static_cast<decltype(out.active)>(in.active);
        out.attachOnly = static_cast<decltype(out.attachOnly)>(in.attachOnly);
        out.gripSequence = static_cast<decltype(out.gripSequence)>(in.gripSequence);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.partKind = static_cast<decltype(out.partKind)>(in.partKind);
        out.reloadRole = static_cast<decltype(out.reloadRole)>(in.reloadRole);
        out.supportRole = static_cast<decltype(out.supportRole)>(in.supportRole);
        out.socketRole = static_cast<decltype(out.socketRole)>(in.socketRole);
        out.actionRole = static_cast<decltype(out.actionRole)>(in.actionRole);
        out.sourceRoot = provider::runtime::resolveSourceKey(in.weaponGenerationKey, in.sourceKey);
        out.providerOwnerToken = static_cast<decltype(out.providerOwnerToken)>(in.providerOwnerToken);
        out.providerGroupId = static_cast<decltype(out.providerGroupId)>(in.providerGroupId);
        out.providerGrabMode = static_cast<decltype(out.providerGrabMode)>(in.providerGrabMode);
        out.hasHandPartLocal = static_cast<decltype(out.hasHandPartLocal)>(in.hasHandPartLocal);
        out.handPartLocalSpace = static_cast<decltype(out.handPartLocalSpace)>(in.handPartLocalSpace);
        convert(out.handPartLocal, in.handPartLocal);
        for (std::size_t i=0; i<std::size(out.sourceName); ++i) out.sourceName[i] = in.sourceName[i];
        out.omodFormId = static_cast<decltype(out.omodFormId)>(in.omodFormId);
        out.attachPointFormId = static_cast<decltype(out.attachPointFormId)>(in.attachPointFormId);
        out.classificationSource = static_cast<decltype(out.classificationSource)>(in.classificationSource);
        out.authoredSupportGrip = static_cast<decltype(out.authoredSupportGrip)>(in.authoredSupportGrip);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::weaponparts::WeaponContactQuery& out, const rock::provider::RockProviderWeaponContactQuery& in) {
        for (std::size_t i=0; i<std::size(out.pointGame); ++i) out.pointGame[i] = in.pointGame[i];
        out.radiusGame = static_cast<decltype(out.radiusGame)>(in.radiusGame);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderWeaponContactQuery& out, const rock::api::weaponparts::WeaponContactQuery& in) {
        for (std::size_t i=0; i<std::size(out.pointGame); ++i) out.pointGame[i] = in.pointGame[i];
        out.radiusGame = static_cast<decltype(out.radiusGame)>(in.radiusGame);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::weaponparts::WeaponContactResult& out, const rock::provider::RockProviderWeaponContactResult& in) {
        out.valid = static_cast<decltype(out.valid)>(in.valid);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.partKind = static_cast<decltype(out.partKind)>(in.partKind);
        out.reloadRole = static_cast<decltype(out.reloadRole)>(in.reloadRole);
        out.supportRole = static_cast<decltype(out.supportRole)>(in.supportRole);
        out.socketRole = static_cast<decltype(out.socketRole)>(in.socketRole);
        out.actionRole = static_cast<decltype(out.actionRole)>(in.actionRole);
        out.interactionKey = provider::runtime::sourceKey(in.weaponGenerationKey, in.interactionRoot);
        out.sourceKey = provider::runtime::sourceKey(in.weaponGenerationKey, in.sourceRoot);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.probeDistanceGame = static_cast<decltype(out.probeDistanceGame)>(in.probeDistanceGame);
        out.reserved = static_cast<decltype(out.reserved)>(in.reserved);
    }
    inline void convert(rock::provider::RockProviderWeaponContactResult& out, const rock::api::weaponparts::WeaponContactResult& in) {
        out.valid = static_cast<decltype(out.valid)>(in.valid);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.partKind = static_cast<decltype(out.partKind)>(in.partKind);
        out.reloadRole = static_cast<decltype(out.reloadRole)>(in.reloadRole);
        out.supportRole = static_cast<decltype(out.supportRole)>(in.supportRole);
        out.socketRole = static_cast<decltype(out.socketRole)>(in.socketRole);
        out.actionRole = static_cast<decltype(out.actionRole)>(in.actionRole);
        out.interactionRoot = provider::runtime::resolveSourceKey(in.weaponGenerationKey, in.interactionKey);
        out.sourceRoot = provider::runtime::resolveSourceKey(in.weaponGenerationKey, in.sourceKey);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.probeDistanceGame = static_cast<decltype(out.probeDistanceGame)>(in.probeDistanceGame);
        out.reserved = static_cast<decltype(out.reserved)>(in.reserved);
    }
    inline void convert(rock::api::weaponparts::WeaponEvidenceDetailV1& out, const rock::provider::RockProviderWeaponEvidenceDetailV1& in) {
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.partKind = static_cast<decltype(out.partKind)>(in.partKind);
        out.reloadRole = static_cast<decltype(out.reloadRole)>(in.reloadRole);
        out.supportRole = static_cast<decltype(out.supportRole)>(in.supportRole);
        out.socketRole = static_cast<decltype(out.socketRole)>(in.socketRole);
        out.actionRole = static_cast<decltype(out.actionRole)>(in.actionRole);
        out.fallbackGripPose = static_cast<decltype(out.fallbackGripPose)>(in.fallbackGripPose);
        out.interactionKey = provider::runtime::sourceKey(in.weaponGenerationKey, in.interactionRoot);
        out.sourceKey = provider::runtime::sourceKey(in.weaponGenerationKey, in.sourceRoot);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        convert(out.localBoundsGame, in.localBoundsGame);
        out.pointCount = static_cast<decltype(out.pointCount)>(in.pointCount);
        for (std::size_t i=0; i<std::size(out.sourceName); ++i) out.sourceName[i] = in.sourceName[i];
        out.omodFormId = static_cast<decltype(out.omodFormId)>(in.omodFormId);
        out.attachPointFormId = static_cast<decltype(out.attachPointFormId)>(in.attachPointFormId);
        out.classificationSource = static_cast<decltype(out.classificationSource)>(in.classificationSource);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderWeaponEvidenceDetailV1& out, const rock::api::weaponparts::WeaponEvidenceDetailV1& in) {
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.partKind = static_cast<decltype(out.partKind)>(in.partKind);
        out.reloadRole = static_cast<decltype(out.reloadRole)>(in.reloadRole);
        out.supportRole = static_cast<decltype(out.supportRole)>(in.supportRole);
        out.socketRole = static_cast<decltype(out.socketRole)>(in.socketRole);
        out.actionRole = static_cast<decltype(out.actionRole)>(in.actionRole);
        out.fallbackGripPose = static_cast<decltype(out.fallbackGripPose)>(in.fallbackGripPose);
        out.interactionRoot = provider::runtime::resolveSourceKey(in.weaponGenerationKey, in.interactionKey);
        out.sourceRoot = provider::runtime::resolveSourceKey(in.weaponGenerationKey, in.sourceKey);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        convert(out.localBoundsGame, in.localBoundsGame);
        out.pointCount = static_cast<decltype(out.pointCount)>(in.pointCount);
        for (std::size_t i=0; i<std::size(out.sourceName); ++i) out.sourceName[i] = in.sourceName[i];
        out.omodFormId = static_cast<decltype(out.omodFormId)>(in.omodFormId);
        out.attachPointFormId = static_cast<decltype(out.attachPointFormId)>(in.attachPointFormId);
        out.classificationSource = static_cast<decltype(out.classificationSource)>(in.classificationSource);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::weaponparts::WeaponPartResolutionQueryV1& out, const rock::provider::RockProviderWeaponPartResolutionQueryV1& in) {
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.partKind = static_cast<decltype(out.partKind)>(in.partKind);
        out.reloadRole = static_cast<decltype(out.reloadRole)>(in.reloadRole);
        out.supportRole = static_cast<decltype(out.supportRole)>(in.supportRole);
        out.socketRole = static_cast<decltype(out.socketRole)>(in.socketRole);
        out.actionRole = static_cast<decltype(out.actionRole)>(in.actionRole);
        out.sourceKey = provider::runtime::sourceKey(in.weaponGenerationKey, in.sourceRoot);
        for (std::size_t i=0; i<std::size(out.sourceName); ++i) out.sourceName[i] = in.sourceName[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderWeaponPartResolutionQueryV1& out, const rock::api::weaponparts::WeaponPartResolutionQueryV1& in) {
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.partKind = static_cast<decltype(out.partKind)>(in.partKind);
        out.reloadRole = static_cast<decltype(out.reloadRole)>(in.reloadRole);
        out.supportRole = static_cast<decltype(out.supportRole)>(in.supportRole);
        out.socketRole = static_cast<decltype(out.socketRole)>(in.socketRole);
        out.actionRole = static_cast<decltype(out.actionRole)>(in.actionRole);
        out.sourceRoot = provider::runtime::resolveSourceKey(in.weaponGenerationKey, in.sourceKey);
        for (std::size_t i=0; i<std::size(out.sourceName); ++i) out.sourceName[i] = in.sourceName[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::weaponparts::WeaponPartResolutionResultV1& out, const rock::provider::RockProviderWeaponPartResolutionResultV1& in) {
        out.whitelistActive = static_cast<decltype(out.whitelistActive)>(in.whitelistActive);
        out.matched = static_cast<decltype(out.matched)>(in.matched);
        out.grabMode = static_cast<decltype(out.grabMode)>(in.grabMode);
        out.groupId = static_cast<decltype(out.groupId)>(in.groupId);
        out.priority = static_cast<decltype(out.priority)>(in.priority);
        out.reserved0 = static_cast<decltype(out.reserved0)>(in.reserved0);
        out.winningOwnerToken = static_cast<decltype(out.winningOwnerToken)>(in.winningOwnerToken);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderWeaponPartResolutionResultV1& out, const rock::api::weaponparts::WeaponPartResolutionResultV1& in) {
        out.whitelistActive = static_cast<decltype(out.whitelistActive)>(in.whitelistActive);
        out.matched = static_cast<decltype(out.matched)>(in.matched);
        out.grabMode = static_cast<decltype(out.grabMode)>(in.grabMode);
        out.groupId = static_cast<decltype(out.groupId)>(in.groupId);
        out.priority = static_cast<decltype(out.priority)>(in.priority);
        out.reserved0 = static_cast<decltype(out.reserved0)>(in.reserved0);
        out.winningOwnerToken = static_cast<decltype(out.winningOwnerToken)>(in.winningOwnerToken);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::weaponparts::WeaponPartPoseV1& out, const rock::provider::RockProviderWeaponPartPoseV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.sourceKey = provider::runtime::sourceKeyForBody(in.weaponGenerationKey, in.bodyId);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.partKind = static_cast<decltype(out.partKind)>(in.partKind);
        out.omodFormId = static_cast<decltype(out.omodFormId)>(in.omodFormId);
        out.attachPointFormId = static_cast<decltype(out.attachPointFormId)>(in.attachPointFormId);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.actionRole = static_cast<decltype(out.actionRole)>(in.actionRole);
        convert(out.sourceParentLocal, in.sourceParentLocal);
        convert(out.weaponRootLocal, in.weaponRootLocal);
        for (std::size_t i=0; i<std::size(out.sourceName); ++i) out.sourceName[i] = in.sourceName[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderWeaponPartPoseV1& out, const rock::api::weaponparts::WeaponPartPoseV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.partKind = static_cast<decltype(out.partKind)>(in.partKind);
        out.omodFormId = static_cast<decltype(out.omodFormId)>(in.omodFormId);
        out.attachPointFormId = static_cast<decltype(out.attachPointFormId)>(in.attachPointFormId);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.actionRole = static_cast<decltype(out.actionRole)>(in.actionRole);
        convert(out.sourceParentLocal, in.sourceParentLocal);
        convert(out.weaponRootLocal, in.weaponRootLocal);
        for (std::size_t i=0; i<std::size(out.sourceName); ++i) out.sourceName[i] = in.sourceName[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::weaponparts::WeaponPartDriveApplicationResultV1& out, const rock::provider::RockProviderWeaponPartDriveApplicationResultV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.ownerToken = static_cast<decltype(out.ownerToken)>(in.ownerToken);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.groupId = static_cast<decltype(out.groupId)>(in.groupId);
        out.priority = static_cast<decltype(out.priority)>(in.priority);
        out.result = static_cast<decltype(out.result)>(in.result);
        convert(out.appliedSourceParentLocal, in.appliedSourceParentLocal);
        for (std::size_t i=0; i<std::size(out.sourceName); ++i) out.sourceName[i] = in.sourceName[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderWeaponPartDriveApplicationResultV1& out, const rock::api::weaponparts::WeaponPartDriveApplicationResultV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.ownerToken = static_cast<decltype(out.ownerToken)>(in.ownerToken);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.groupId = static_cast<decltype(out.groupId)>(in.groupId);
        out.priority = static_cast<decltype(out.priority)>(in.priority);
        out.result = static_cast<decltype(out.result)>(in.result);
        convert(out.appliedSourceParentLocal, in.appliedSourceParentLocal);
        for (std::size_t i=0; i<std::size(out.sourceName); ++i) out.sourceName[i] = in.sourceName[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
}
