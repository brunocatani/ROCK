#pragma once
#include <ROCK/Weapon.h>
#include "CommonMarshalling.h"
namespace rock::api::boundary {
    inline void convert(rock::api::weapon::WeaponClassificationV1& out, const rock::provider::RockProviderWeaponClassificationV1& in) {
        out.valid = static_cast<decltype(out.valid)>(in.valid);
        out.keywordFlags = static_cast<decltype(out.keywordFlags)>(in.keywordFlags);
        out.sizeClass = static_cast<decltype(out.sizeClass)>(in.sizeClass);
        out.source = static_cast<decltype(out.source)>(in.source);
        out.formId = static_cast<decltype(out.formId)>(in.formId);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.confidence = static_cast<decltype(out.confidence)>(in.confidence);
        out.provenanceFlags = static_cast<decltype(out.provenanceFlags)>(in.provenanceFlags);
    }
    inline void convert(rock::provider::RockProviderWeaponClassificationV1& out, const rock::api::weapon::WeaponClassificationV1& in) {
        out.valid = static_cast<decltype(out.valid)>(in.valid);
        out.keywordFlags = static_cast<decltype(out.keywordFlags)>(in.keywordFlags);
        out.sizeClass = static_cast<decltype(out.sizeClass)>(in.sizeClass);
        out.source = static_cast<decltype(out.source)>(in.source);
        out.formId = static_cast<decltype(out.formId)>(in.formId);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.confidence = static_cast<decltype(out.confidence)>(in.confidence);
        out.provenanceFlags = static_cast<decltype(out.provenanceFlags)>(in.provenanceFlags);
    }
    inline void convert(rock::api::weapon::WeaponEmitterV1& out, const rock::provider::RockProviderWeaponEmitterV1& in) {
        out.kind = static_cast<decltype(out.kind)>(in.kind);
        out.source = static_cast<decltype(out.source)>(in.source);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.active = static_cast<decltype(out.active)>(in.active);
        out.visible = static_cast<decltype(out.visible)>(in.visible);
        out.addOnNodeValue = static_cast<decltype(out.addOnNodeValue)>(in.addOnNodeValue);
        out.omodFormId = static_cast<decltype(out.omodFormId)>(in.omodFormId);
        out.attachPointFormId = static_cast<decltype(out.attachPointFormId)>(in.attachPointFormId);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        convert(out.weaponLocalTransform, in.weaponLocalTransform);
        convert(out.forwardWeaponLocal, in.forwardWeaponLocal);
        for (std::size_t i=0; i<std::size(out.sourceName); ++i) out.sourceName[i] = in.sourceName[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderWeaponEmitterV1& out, const rock::api::weapon::WeaponEmitterV1& in) {
        out.kind = static_cast<decltype(out.kind)>(in.kind);
        out.source = static_cast<decltype(out.source)>(in.source);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.active = static_cast<decltype(out.active)>(in.active);
        out.visible = static_cast<decltype(out.visible)>(in.visible);
        out.addOnNodeValue = static_cast<decltype(out.addOnNodeValue)>(in.addOnNodeValue);
        out.omodFormId = static_cast<decltype(out.omodFormId)>(in.omodFormId);
        out.attachPointFormId = static_cast<decltype(out.attachPointFormId)>(in.attachPointFormId);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        convert(out.weaponLocalTransform, in.weaponLocalTransform);
        convert(out.forwardWeaponLocal, in.forwardWeaponLocal);
        for (std::size_t i=0; i<std::size(out.sourceName); ++i) out.sourceName[i] = in.sourceName[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::weapon::EquippedWeaponGripStateV1& out, const rock::provider::RockProviderEquippedWeaponGripStateV1& in) {
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.weaponFormId = static_cast<decltype(out.weaponFormId)>(in.weaponFormId);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        convert(out.weaponWorld, in.weaponWorld);
        convert(out.rightHandInWeapon, in.rightHandInWeapon);
        convert(out.leftHandInWeapon, in.leftHandInWeapon);
        convert(out.muzzleOriginGame, in.muzzleOriginGame);
        convert(out.muzzleDirectionGame, in.muzzleDirectionGame);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderEquippedWeaponGripStateV1& out, const rock::api::weapon::EquippedWeaponGripStateV1& in) {
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.weaponFormId = static_cast<decltype(out.weaponFormId)>(in.weaponFormId);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        convert(out.weaponWorld, in.weaponWorld);
        convert(out.rightHandInWeapon, in.rightHandInWeapon);
        convert(out.leftHandInWeapon, in.leftHandInWeapon);
        convert(out.muzzleOriginGame, in.muzzleOriginGame);
        convert(out.muzzleDirectionGame, in.muzzleDirectionGame);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::weapon::EquippedWeaponHandlingRequestV1& out, const rock::provider::RockProviderEquippedWeaponHandlingRequestV1& in) {
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.leaseFrames = static_cast<decltype(out.leaseFrames)>(in.leaseFrames);
        out.gripZoneEquipRadiusGameUnits = static_cast<decltype(out.gripZoneEquipRadiusGameUnits)>(in.gripZoneEquipRadiusGameUnits);
        out.gripZoneEquipSettleSeconds = static_cast<decltype(out.gripZoneEquipSettleSeconds)>(in.gripZoneEquipSettleSeconds);
        out.firingGripReattachRadiusGameUnits = static_cast<decltype(out.firingGripReattachRadiusGameUnits)>(in.firingGripReattachRadiusGameUnits);
        out.gripZoneHoverHapticIntensity = static_cast<decltype(out.gripZoneHoverHapticIntensity)>(in.gripZoneHoverHapticIntensity);
        out.firingGripProximitySupportRadiusGameUnits = static_cast<decltype(out.firingGripProximitySupportRadiusGameUnits)>(in.firingGripProximitySupportRadiusGameUnits);
        out.weaponGripHapticDurationSeconds = static_cast<decltype(out.weaponGripHapticDurationSeconds)>(in.weaponGripHapticDurationSeconds);
        out.firingGripAttachHapticIntensity = static_cast<decltype(out.firingGripAttachHapticIntensity)>(in.firingGripAttachHapticIntensity);
        out.firingGripDetachHapticIntensity = static_cast<decltype(out.firingGripDetachHapticIntensity)>(in.firingGripDetachHapticIntensity);
        out.supportGripHapticIntensity = static_cast<decltype(out.supportGripHapticIntensity)>(in.supportGripHapticIntensity);
        out.firingGripPromotionRadiusGameUnits = static_cast<decltype(out.firingGripPromotionRadiusGameUnits)>(in.firingGripPromotionRadiusGameUnits);
        out.leftFiringAimYawDegrees = static_cast<decltype(out.leftFiringAimYawDegrees)>(in.leftFiringAimYawDegrees);
        out.leftFiringAimPitchDegrees = static_cast<decltype(out.leftFiringAimPitchDegrees)>(in.leftFiringAimPitchDegrees);
        for (std::size_t i=0; i<std::size(out.leftFiringAimOffsetGameUnits); ++i) out.leftFiringAimOffsetGameUnits[i] = in.leftFiringAimOffsetGameUnits[i];
        out.equipVisualBridgeTimeoutSeconds = static_cast<decltype(out.equipVisualBridgeTimeoutSeconds)>(in.equipVisualBridgeTimeoutSeconds);
        out.equipVisualBridgeBlendSeconds = static_cast<decltype(out.equipVisualBridgeBlendSeconds)>(in.equipVisualBridgeBlendSeconds);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderEquippedWeaponHandlingRequestV1& out, const rock::api::weapon::EquippedWeaponHandlingRequestV1& in) {
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.leaseFrames = static_cast<decltype(out.leaseFrames)>(in.leaseFrames);
        out.gripZoneEquipRadiusGameUnits = static_cast<decltype(out.gripZoneEquipRadiusGameUnits)>(in.gripZoneEquipRadiusGameUnits);
        out.gripZoneEquipSettleSeconds = static_cast<decltype(out.gripZoneEquipSettleSeconds)>(in.gripZoneEquipSettleSeconds);
        out.firingGripReattachRadiusGameUnits = static_cast<decltype(out.firingGripReattachRadiusGameUnits)>(in.firingGripReattachRadiusGameUnits);
        out.gripZoneHoverHapticIntensity = static_cast<decltype(out.gripZoneHoverHapticIntensity)>(in.gripZoneHoverHapticIntensity);
        out.firingGripProximitySupportRadiusGameUnits = static_cast<decltype(out.firingGripProximitySupportRadiusGameUnits)>(in.firingGripProximitySupportRadiusGameUnits);
        out.weaponGripHapticDurationSeconds = static_cast<decltype(out.weaponGripHapticDurationSeconds)>(in.weaponGripHapticDurationSeconds);
        out.firingGripAttachHapticIntensity = static_cast<decltype(out.firingGripAttachHapticIntensity)>(in.firingGripAttachHapticIntensity);
        out.firingGripDetachHapticIntensity = static_cast<decltype(out.firingGripDetachHapticIntensity)>(in.firingGripDetachHapticIntensity);
        out.supportGripHapticIntensity = static_cast<decltype(out.supportGripHapticIntensity)>(in.supportGripHapticIntensity);
        out.firingGripPromotionRadiusGameUnits = static_cast<decltype(out.firingGripPromotionRadiusGameUnits)>(in.firingGripPromotionRadiusGameUnits);
        out.leftFiringAimYawDegrees = static_cast<decltype(out.leftFiringAimYawDegrees)>(in.leftFiringAimYawDegrees);
        out.leftFiringAimPitchDegrees = static_cast<decltype(out.leftFiringAimPitchDegrees)>(in.leftFiringAimPitchDegrees);
        for (std::size_t i=0; i<std::size(out.leftFiringAimOffsetGameUnits); ++i) out.leftFiringAimOffsetGameUnits[i] = in.leftFiringAimOffsetGameUnits[i];
        out.equipVisualBridgeTimeoutSeconds = static_cast<decltype(out.equipVisualBridgeTimeoutSeconds)>(in.equipVisualBridgeTimeoutSeconds);
        out.equipVisualBridgeBlendSeconds = static_cast<decltype(out.equipVisualBridgeBlendSeconds)>(in.equipVisualBridgeBlendSeconds);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::weapon::EquippedWeaponHandlingStateV1& out, const rock::provider::RockProviderEquippedWeaponHandlingStateV1& in) {
        out.authorityFlags = static_cast<decltype(out.authorityFlags)>(in.authorityFlags);
        out.runtimeFlags = static_cast<decltype(out.runtimeFlags)>(in.runtimeFlags);
        out.ownerToken = static_cast<decltype(out.ownerToken)>(in.ownerToken);
        out.expiresAfterFrame = static_cast<decltype(out.expiresAfterFrame)>(in.expiresAfterFrame);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.weaponFormId = static_cast<decltype(out.weaponFormId)>(in.weaponFormId);
        out.currentFiringHand = static_cast<decltype(out.currentFiringHand)>(in.currentFiringHand);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
        out.authorityFlags &= 0x3Fu;
        out.runtimeFlags &= ~(1u<<1);
    }
    inline void convert(rock::provider::RockProviderEquippedWeaponHandlingStateV1& out, const rock::api::weapon::EquippedWeaponHandlingStateV1& in) {
        out.authorityFlags = static_cast<decltype(out.authorityFlags)>(in.authorityFlags);
        out.runtimeFlags = static_cast<decltype(out.runtimeFlags)>(in.runtimeFlags);
        out.ownerToken = static_cast<decltype(out.ownerToken)>(in.ownerToken);
        out.expiresAfterFrame = static_cast<decltype(out.expiresAfterFrame)>(in.expiresAfterFrame);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.weaponFormId = static_cast<decltype(out.weaponFormId)>(in.weaponFormId);
        out.currentFiringHand = static_cast<decltype(out.currentFiringHand)>(in.currentFiringHand);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::weapon::EquippedWeaponStateV1& out, const rock::provider::RockProviderEquippedWeaponStateV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.weaponFormId = static_cast<decltype(out.weaponFormId)>(in.weaponFormId);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.transitionSequence = static_cast<decltype(out.transitionSequence)>(in.transitionSequence);
        out.terminalSequence = static_cast<decltype(out.terminalSequence)>(in.terminalSequence);
        out.transitionSource = static_cast<decltype(out.transitionSource)>(in.transitionSource);
        out.terminalResult = static_cast<decltype(out.terminalResult)>(in.terminalResult);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.transitionWeaponFormId = static_cast<decltype(out.transitionWeaponFormId)>(in.transitionWeaponFormId);
        out.terminalWeaponFormId = static_cast<decltype(out.terminalWeaponFormId)>(in.terminalWeaponFormId);
        out.terminalSource = static_cast<decltype(out.terminalSource)>(in.terminalSource);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderEquippedWeaponStateV1& out, const rock::api::weapon::EquippedWeaponStateV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.weaponFormId = static_cast<decltype(out.weaponFormId)>(in.weaponFormId);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.transitionSequence = static_cast<decltype(out.transitionSequence)>(in.transitionSequence);
        out.terminalSequence = static_cast<decltype(out.terminalSequence)>(in.terminalSequence);
        out.transitionSource = static_cast<decltype(out.transitionSource)>(in.transitionSource);
        out.terminalResult = static_cast<decltype(out.terminalResult)>(in.terminalResult);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.transitionWeaponFormId = static_cast<decltype(out.transitionWeaponFormId)>(in.transitionWeaponFormId);
        out.terminalWeaponFormId = static_cast<decltype(out.terminalWeaponFormId)>(in.terminalWeaponFormId);
        out.terminalSource = static_cast<decltype(out.terminalSource)>(in.terminalSource);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::weapon::ScopeSightStateV1& out, const rock::provider::RockProviderScopeSightStateV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.publicationSequence = static_cast<decltype(out.publicationSequence)>(in.publicationSequence);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.weaponFormId = static_cast<decltype(out.weaponFormId)>(in.weaponFormId);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.activationSource = static_cast<decltype(out.activationSource)>(in.activationSource);
        out.nativeScopeOverlayIndex = static_cast<decltype(out.nativeScopeOverlayIndex)>(in.nativeScopeOverlayIndex);
        convert(out.anchorWeaponLocal, in.anchorWeaponLocal);
        convert(out.sightBoundsWeaponLocal, in.sightBoundsWeaponLocal);
        out.sightBodyCount = static_cast<decltype(out.sightBodyCount)>(in.sightBodyCount);
        out.sightBodyId = static_cast<decltype(out.sightBodyId)>(in.sightBodyId);
        out.omodFormId = static_cast<decltype(out.omodFormId)>(in.omodFormId);
        out.attachPointFormId = static_cast<decltype(out.attachPointFormId)>(in.attachPointFormId);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderScopeSightStateV1& out, const rock::api::weapon::ScopeSightStateV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.publicationSequence = static_cast<decltype(out.publicationSequence)>(in.publicationSequence);
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.weaponFormId = static_cast<decltype(out.weaponFormId)>(in.weaponFormId);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.activationSource = static_cast<decltype(out.activationSource)>(in.activationSource);
        out.nativeScopeOverlayIndex = static_cast<decltype(out.nativeScopeOverlayIndex)>(in.nativeScopeOverlayIndex);
        convert(out.anchorWeaponLocal, in.anchorWeaponLocal);
        convert(out.sightBoundsWeaponLocal, in.sightBoundsWeaponLocal);
        out.sightBodyCount = static_cast<decltype(out.sightBodyCount)>(in.sightBodyCount);
        out.sightBodyId = static_cast<decltype(out.sightBodyId)>(in.sightBodyId);
        out.omodFormId = static_cast<decltype(out.omodFormId)>(in.omodFormId);
        out.attachPointFormId = static_cast<decltype(out.attachPointFormId)>(in.attachPointFormId);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::weapon::WeaponCompositionStateV1& out, const rock::provider::RockProviderWeaponCompositionStateV1& in) {
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.compositionSignature = static_cast<decltype(out.compositionSignature)>(in.compositionSignature);
        out.weaponFormId = static_cast<decltype(out.weaponFormId)>(in.weaponFormId);
        out.entryCount = static_cast<decltype(out.entryCount)>(in.entryCount);
        out.semanticCoverageMask = static_cast<decltype(out.semanticCoverageMask)>(in.semanticCoverageMask);
        out.missingCoverageMask = static_cast<decltype(out.missingCoverageMask)>(in.missingCoverageMask);
        out.publicationSequence = static_cast<decltype(out.publicationSequence)>(in.publicationSequence);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderWeaponCompositionStateV1& out, const rock::api::weapon::WeaponCompositionStateV1& in) {
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.compositionSignature = static_cast<decltype(out.compositionSignature)>(in.compositionSignature);
        out.weaponFormId = static_cast<decltype(out.weaponFormId)>(in.weaponFormId);
        out.entryCount = static_cast<decltype(out.entryCount)>(in.entryCount);
        out.semanticCoverageMask = static_cast<decltype(out.semanticCoverageMask)>(in.semanticCoverageMask);
        out.missingCoverageMask = static_cast<decltype(out.missingCoverageMask)>(in.missingCoverageMask);
        out.publicationSequence = static_cast<decltype(out.publicationSequence)>(in.publicationSequence);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::weapon::WeaponCompositionEntryV1& out, const rock::provider::RockProviderWeaponCompositionEntryV1& in) {
        out.omodFormId = static_cast<decltype(out.omodFormId)>(in.omodFormId);
        out.attachPointFormId = static_cast<decltype(out.attachPointFormId)>(in.attachPointFormId);
        out.stableIndex = static_cast<decltype(out.stableIndex)>(in.stableIndex);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.semanticCoverageMask = static_cast<decltype(out.semanticCoverageMask)>(in.semanticCoverageMask);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderWeaponCompositionEntryV1& out, const rock::api::weapon::WeaponCompositionEntryV1& in) {
        out.omodFormId = static_cast<decltype(out.omodFormId)>(in.omodFormId);
        out.attachPointFormId = static_cast<decltype(out.attachPointFormId)>(in.attachPointFormId);
        out.stableIndex = static_cast<decltype(out.stableIndex)>(in.stableIndex);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.semanticCoverageMask = static_cast<decltype(out.semanticCoverageMask)>(in.semanticCoverageMask);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::weapon::AuthoredGripPoseV1& out, const rock::provider::RockProviderAuthoredGripPoseV1& in) {
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.weaponFormId = static_cast<decltype(out.weaponFormId)>(in.weaponFormId);
        out.source = static_cast<decltype(out.source)>(in.source);
        out.variantKey = static_cast<decltype(out.variantKey)>(in.variantKey);
        out.captureSequence = static_cast<decltype(out.captureSequence)>(in.captureSequence);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.rightFingerLocalTransformMask = static_cast<decltype(out.rightFingerLocalTransformMask)>(in.rightFingerLocalTransformMask);
        out.leftFingerLocalTransformMask = static_cast<decltype(out.leftFingerLocalTransformMask)>(in.leftFingerLocalTransformMask);
        convert(out.rightHandInWeapon, in.rightHandInWeapon);
        convert(out.leftHandInWeapon, in.leftHandInWeapon);
        for (std::size_t i=0; i<std::size(out.rightFingerLocalTransforms); ++i) convert(out.rightFingerLocalTransforms[i], in.rightFingerLocalTransforms[i]);
        for (std::size_t i=0; i<std::size(out.leftFingerLocalTransforms); ++i) convert(out.leftFingerLocalTransforms[i], in.leftFingerLocalTransforms[i]);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderAuthoredGripPoseV1& out, const rock::api::weapon::AuthoredGripPoseV1& in) {
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.weaponFormId = static_cast<decltype(out.weaponFormId)>(in.weaponFormId);
        out.source = static_cast<decltype(out.source)>(in.source);
        out.variantKey = static_cast<decltype(out.variantKey)>(in.variantKey);
        out.captureSequence = static_cast<decltype(out.captureSequence)>(in.captureSequence);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.rightFingerLocalTransformMask = static_cast<decltype(out.rightFingerLocalTransformMask)>(in.rightFingerLocalTransformMask);
        out.leftFingerLocalTransformMask = static_cast<decltype(out.leftFingerLocalTransformMask)>(in.leftFingerLocalTransformMask);
        convert(out.rightHandInWeapon, in.rightHandInWeapon);
        convert(out.leftHandInWeapon, in.leftHandInWeapon);
        for (std::size_t i=0; i<std::size(out.rightFingerLocalTransforms); ++i) convert(out.rightFingerLocalTransforms[i], in.rightFingerLocalTransforms[i]);
        for (std::size_t i=0; i<std::size(out.leftFingerLocalTransforms); ++i) convert(out.leftFingerLocalTransforms[i], in.leftFingerLocalTransforms[i]);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
}
