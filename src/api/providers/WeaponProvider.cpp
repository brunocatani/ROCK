#include "Boundary.h"
#include "CommonMarshalling.h"
#include "EventBoundary.h"
#include "physics-interaction/weapon/EquipPickupScope.h"
#include <ROCK/Weapon.h>
#include <algorithm>
#include <array>

namespace rock::api::weapon {
namespace {
using namespace boundary;
namespace p = provider;


template <class Record>
Status checkOutputs(Record* records, std::uint32_t capacity, std::uint32_t maxCapacity, std::uint32_t* outCount) noexcept {
    if (!outCount) return Status::InvalidArgument;
    *outCount = 0;
    if (capacity > maxCapacity) return Status::CapacityFull;
    if (capacity && !records) return Status::InvalidArgument;
    for (std::uint32_t i = 0; i < capacity; ++i)
        if (const auto s = checkOutput(records + i); s != Status::Ok) return s;
    return Status::Ok;
}

Hand otherHand(Hand hand) noexcept { return hand == Hand::Left ? Hand::Right : Hand::Left; }

Status ROCK_CALL getWeaponState(OwnerToken owner, std::uint32_t slot, WeaponState* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    outState->weaponSlot = slot;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        p::RockProviderEquippedWeaponStateV1 equipped{};
        const auto status = static_cast<Status>(p::runtime::apiGetEquippedWeaponStateV1(owner, &equipped));
        if (status != Status::Ok) return status;
        outState->sample = p::runtime::sample();
        outState->sample.frameIndex = equipped.frameIndex;
        outState->sample.worldGeneration = equipped.worldGeneration;
        outState->sample.skeletonGeneration = equipped.skeletonGeneration;
        outState->sample.providerGeneration = equipped.providerGeneration;
        outState->flags = equipped.flags;
        outState->weaponFormId = equipped.weaponFormId;
        outState->weaponGenerationKey = equipped.weaponGenerationKey;
        outState->transitionSequence = equipped.transitionSequence;
        outState->terminalSequence = equipped.terminalSequence;
        outState->transitionSource = static_cast<TransitionSource>(equipped.transitionSource);
        outState->terminalResult = static_cast<TransitionResult>(equipped.terminalResult);
        outState->terminalSource = static_cast<TransitionSource>(equipped.terminalSource);
        outState->transitionWeaponFormId = equipped.transitionWeaponFormId;
        outState->terminalWeaponFormId = equipped.terminalWeaponFormId;

        using Runtime = p::RockProviderEquippedWeaponHandlingRuntimeFlagV1;
        p::RockProviderEquippedWeaponHandlingStateV1 handling{};
        if (!p::runtime::apiGetEquippedWeaponHandlingStateV1(&handling) ||
            !(handling.runtimeFlags & static_cast<std::uint32_t>(Runtime::WeaponPresent))) return Status::Ok;
        const auto runtime = handling.runtimeFlags;
        const auto map = [&](Runtime from, HandlingStateFlag to) {
            if (runtime & static_cast<std::uint32_t>(from)) outState->handlingFlags |= static_cast<std::uint32_t>(to);
        };
        map(Runtime::FiringGripOccupied, HandlingStateFlag::FiringGripHeld);
        map(Runtime::ManualOwnershipActive, HandlingStateFlag::ManualOwnership);
        map(Runtime::PartCarryActive, HandlingStateFlag::PartCarry);
        map(Runtime::LeftFiringInfrastructureAvailable, HandlingStateFlag::LeftFiringAvailable);
        outState->firingHand = static_cast<Hand>(handling.currentFiringHand);
        p::RockProviderEquippedWeaponGripStateV1 grip{};
        if (p::runtime::apiGetEquippedWeaponGripStateV1(owner, &grip) &&
            (grip.flags & static_cast<std::uint32_t>(p::RockProviderEquippedWeaponGripStateFlagV1::TwoHandGripActive)))
            outState->supportHand = otherHand(outState->firingHand);
        return Status::Ok;
    });
}

Status ROCK_CALL getGrip(OwnerToken owner, std::uint32_t slot, Grip* outGrip) noexcept {
    if (const auto s = checkOutput(outGrip); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    outGrip->weaponSlot = slot;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        p::RockProviderEquippedWeaponGripStateV1 in{};
        if (!p::runtime::apiGetEquippedWeaponGripStateV1(owner, &in)) return Status::NotReady;
        using G = p::RockProviderEquippedWeaponGripStateFlagV1;
        constexpr auto retired = static_cast<std::uint32_t>(G::TwoHandGripActive) | static_cast<std::uint32_t>(G::FiringHandLeft);
        outGrip->flags = in.flags & ~retired;
        outGrip->weaponFormId = in.weaponFormId;
        outGrip->weaponGenerationKey = in.weaponGenerationKey;
        outGrip->weaponWorld = in.weaponWorld;
        outGrip->rightHandInWeapon = in.rightHandInWeapon;
        outGrip->leftHandInWeapon = in.leftHandInWeapon;
        outGrip->muzzleOriginGame = in.muzzleOriginGame;
        outGrip->muzzleDirectionGame = in.muzzleDirectionGame;
        return Status::Ok;
    });
}

Status ROCK_CALL getClassification(OwnerToken owner, std::uint32_t slot, Classification* outClassification) noexcept {
    if (const auto s = checkOutput(outClassification); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    outClassification->weaponSlot = slot;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        p::RockProviderWeaponClassificationV1 in{};
        const bool ok = p::runtime::apiQueryEquippedWeaponClassificationV1(&in);
        outClassification->valid = in.valid;
        outClassification->keywordFlags = in.keywordFlags;
        outClassification->sizeClass = static_cast<SizeClass>(in.sizeClass);
        outClassification->source = static_cast<ClassificationSource>(in.source);
        outClassification->weaponFormId = in.formId;
        outClassification->confidence = in.confidence;
        outClassification->weaponGenerationKey = in.weaponGenerationKey;
        outClassification->provenanceFlags = in.provenanceFlags;
        return ok ? Status::Ok : Status::NotReady;
    });
}

Status ROCK_CALL copyEmitters(OwnerToken owner, std::uint32_t slot, Emitter* outEmitters, std::uint32_t capacity, std::uint32_t* outCount) noexcept {
    if (const auto s = checkOutputs(outEmitters, capacity, kMaxEmitters, outCount); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        std::array<p::RockProviderWeaponEmitterV1, kMaxEmitters> native{};
        *outCount = p::runtime::apiGetWeaponEmitterCountV1();
        const auto copied = p::runtime::apiCopyWeaponEmittersV1(native.data(), capacity);
        for (std::uint32_t i = 0; i < (std::min)(capacity, copied); ++i) {
            const auto& in = native[i];
            auto& out = outEmitters[i];
            out.kind = static_cast<EmitterKind>(in.kind);
            out.source = static_cast<EmitterSource>(in.source);
            out.flags = in.flags;
            out.active = in.active;
            out.visible = in.visible;
            out.addOnNodeValue = in.addOnNodeValue;
            out.omodFormId = in.omodFormId;
            out.attachPointFormId = in.attachPointFormId;
            out.weaponGenerationKey = in.weaponGenerationKey;
            out.weaponLocalTransform = in.weaponLocalTransform;
            out.forwardWeaponLocal = in.forwardWeaponLocal;
            std::copy_n(in.sourceName, std::size(out.sourceName), out.sourceName);
        }
        return Status::Ok;
    });
}

Status ROCK_CALL getCompositionState(OwnerToken owner, std::uint32_t slot, CompositionState* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    outState->weaponSlot = slot;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        p::RockProviderWeaponCompositionStateV1 in{};
        const auto status = static_cast<Status>(p::runtime::apiGetWeaponCompositionStateV1(owner, &in));
        outState->weaponFormId = in.weaponFormId;
        outState->weaponGenerationKey = in.weaponGenerationKey;
        outState->compositionSignature = in.compositionSignature;
        outState->semanticCoverageMask = in.semanticCoverageMask;
        outState->missingCoverageMask = in.missingCoverageMask;
        outState->publicationSequence = in.publicationSequence;
        outState->entryCount = in.entryCount;
        return status;
    });
}

Status ROCK_CALL copyCompositionEntries(OwnerToken owner, std::uint32_t slot, CompositionEntry* outEntries, std::uint32_t capacity, std::uint32_t* outCount) noexcept {
    if (const auto s = checkOutputs(outEntries, capacity, kMaxCompositionEntries, outCount); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        std::array<p::RockProviderWeaponCompositionEntryV1, kMaxCompositionEntries> native{};
        const auto status = static_cast<Status>(p::runtime::apiCopyWeaponCompositionEntriesV1(owner, native.data(), capacity, outCount));
        for (std::uint32_t i = 0; i < (std::min)(capacity, *outCount); ++i) {
            const auto& in = native[i];
            auto& out = outEntries[i];
            out.omodFormId = in.omodFormId;
            out.attachPointFormId = in.attachPointFormId;
            out.stableIndex = in.stableIndex;
            out.flags = in.flags;
            out.semanticCoverageMask = in.semanticCoverageMask;
        }
        return status;
    });
}

Status ROCK_CALL getAuthoredGripPose(OwnerToken owner, std::uint32_t slot, AuthoredGripPose* outPose) noexcept {
    if (const auto s = checkOutput(outPose); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    outPose->weaponSlot = slot;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        p::RockProviderAuthoredGripPoseV1 in{};
        const auto status = static_cast<Status>(p::runtime::apiGetSelectedAuthoredGripPoseV1(owner, &in));
        outPose->weaponFormId = in.weaponFormId;
        outPose->weaponGenerationKey = in.weaponGenerationKey;
        outPose->variantKey = in.variantKey;
        outPose->captureSequence = in.captureSequence;
        outPose->source = static_cast<AuthoredGripSource>(in.source);
        outPose->flags = in.flags;
        outPose->rightFingerLocalTransformMask = in.rightFingerLocalTransformMask;
        outPose->leftFingerLocalTransformMask = in.leftFingerLocalTransformMask;
        outPose->rightHandInWeapon = in.rightHandInWeapon;
        outPose->leftHandInWeapon = in.leftHandInWeapon;
        for (std::uint32_t i = 0; i < kFingerLocalTransformCount; ++i) {
            outPose->rightFingerLocalTransforms[i] = in.rightFingerLocalTransforms[i];
            outPose->leftFingerLocalTransforms[i] = in.leftFingerLocalTransforms[i];
        }
        outPose->worldGeneration = in.worldGeneration;
        outPose->skeletonGeneration = in.skeletonGeneration;
        outPose->providerGeneration = in.providerGeneration;
        return status;
    });
}

Status ROCK_CALL getScopeSightState(OwnerToken owner, std::uint32_t slot, ScopeSightState* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    outState->weaponSlot = slot;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        p::RockProviderScopeSightStateV1 in{};
        const auto status = static_cast<Status>(p::runtime::apiGetScopeSightStateV1(owner, &in));
        outState->sample = p::runtime::sample();
        outState->sample.frameIndex = in.frameIndex;
        outState->sample.publicationSequence = in.publicationSequence;
        outState->sample.worldGeneration = in.worldGeneration;
        outState->sample.skeletonGeneration = in.skeletonGeneration;
        outState->sample.providerGeneration = in.providerGeneration;
        outState->flags = in.flags;
        outState->weaponGenerationKey = in.weaponGenerationKey;
        outState->weaponFormId = in.weaponFormId;
        outState->activationSource = static_cast<ScopeActivationSource>(in.activationSource);
        outState->nativeScopeOverlayIndex = in.nativeScopeOverlayIndex;
        outState->anchorWeaponLocal = in.anchorWeaponLocal;
        outState->sightBoundsWeaponLocal = in.sightBoundsWeaponLocal;
        outState->sightBodyCount = in.sightBodyCount;
        outState->sightBodyId = in.sightBodyId;
        outState->omodFormId = in.omodFormId;
        outState->attachPointFormId = in.attachPointFormId;
        return status;
    });
}

Status ROCK_CALL setHandlingAuthority(OwnerToken owner, const HandlingRequest* request) noexcept {
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    constexpr std::uint32_t knownFeatures = 0x3F;
    if (request->features & ~knownFeatures) return Status::InvalidArgument;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        p::RockProviderEquippedWeaponHandlingRequestV1 native{};
        native.flags = request->features;
        native.leaseFrames = request->leaseFrames;
        native.gripZoneEquipRadiusGameUnits = request->gripZoneEquipRadiusGameUnits;
        native.gripZoneEquipSettleSeconds = request->gripZoneEquipSettleSeconds;
        native.firingGripReattachRadiusGameUnits = request->firingGripReattachRadiusGameUnits;
        native.gripZoneHoverHapticIntensity = request->gripZoneHoverHapticIntensity;
        native.firingGripProximitySupportRadiusGameUnits = request->firingGripProximitySupportRadiusGameUnits;
        native.weaponGripHapticDurationSeconds = request->weaponGripHapticDurationSeconds;
        native.firingGripAttachHapticIntensity = request->firingGripAttachHapticIntensity;
        native.firingGripDetachHapticIntensity = request->firingGripDetachHapticIntensity;
        native.supportGripHapticIntensity = request->supportGripHapticIntensity;
        native.firingGripPromotionRadiusGameUnits = request->firingGripPromotionRadiusGameUnits;
        native.leftFiringAimYawDegrees = request->leftFiringAimYawDegrees;
        native.leftFiringAimPitchDegrees = request->leftFiringAimPitchDegrees;
        std::copy_n(request->leftFiringAimOffsetGameUnits, 3, native.leftFiringAimOffsetGameUnits);
        native.equipVisualBridgeTimeoutSeconds = request->equipVisualBridgeTimeoutSeconds;
        native.equipVisualBridgeBlendSeconds = request->equipVisualBridgeBlendSeconds;
        native.worldGeneration = request->worldGeneration;
        native.skeletonGeneration = request->skeletonGeneration;
        native.providerGeneration = request->providerGeneration;
        return static_cast<Status>(p::runtime::apiSetEquippedWeaponHandlingAuthorityV1(owner, &native));
    });
}
Status ROCK_CALL clearHandlingAuthority(OwnerToken owner) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return static_cast<Status>(p::runtime::apiClearEquippedWeaponHandlingAuthorityV1(owner));
    });
}
Status ROCK_CALL getHandlingAuthority(OwnerToken owner, HandlingAuthority* outAuthority) noexcept {
    if (const auto s = checkOutput(outAuthority); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] { return p::runtime::getHandlingAuthority(*outAuthority); });
}

Status ROCK_CALL captureInventoryWeapon(OwnerToken owner, std::uint32_t baseFormId, std::uint32_t stackIndex, InventoryWeapon* outItem) noexcept {
    if (const auto s = checkOutput(outItem); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        return p::runtime::captureInventoryWeapon(owner, baseFormId, stackIndex, *outItem);
    }, p::OwnerAccess::Active);
}
Status ROCK_CALL requestInventoryEquip(OwnerToken owner, const EquipRequest* request, std::uint64_t* outCommandId) noexcept {
    if (!outCommandId) return Status::InvalidArgument;
    *outCommandId = 0;
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return p::runtime::requestInventoryEquip(owner, *request, *outCommandId);
    }, p::OwnerAccess::Active);
}
Status ROCK_CALL getInventoryEquipResult(OwnerToken owner, std::uint64_t commandId, EquipResult* outResult) noexcept {
    if (const auto s = checkOutput(outResult); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] { return p::runtime::getInventoryEquipResult(owner, commandId, *outResult); });
}
Status ROCK_CALL cancelInventoryEquip(OwnerToken owner, std::uint64_t commandId) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] { return p::runtime::cancelInventoryEquip(owner, commandId); });
}
Status ROCK_CALL isEquipPickup(OwnerToken owner, std::uint32_t referenceFormId, std::uint32_t count, std::uint32_t* outMatches) noexcept {
    if (!outMatches) return Status::InvalidArgument;
    *outMatches = 0;
    return invoke(owner, kInterfaceId, 1, false, [&] {
        *outMatches = equip_pickup::matches(referenceFormId, count) ? 1u : 0u;
        return Status::Ok;
    });
}
Status ROCK_CALL getSample(OwnerToken owner, Sample* outSample) noexcept {
    return readSample(owner, kInterfaceId, outSample);
}
Status ROCK_CALL copyEvents(OwnerToken owner, std::uint64_t after, Event* events, std::uint32_t capacity, Stream* stream) noexcept {
    return copyEventStream(owner, kInterfaceId, after, events, capacity, kEventCapacityPerOwner, stream);
}
}
const Api& table() noexcept {
    static const Api value{
        &getWeaponState,
        &getGrip,
        &getClassification,
        &copyEmitters,
        &getCompositionState,
        &copyCompositionEntries,
        &getAuthoredGripPose,
        &getScopeSightState,
        &setHandlingAuthority,
        &clearHandlingAuthority,
        &getHandlingAuthority,
        &captureInventoryWeapon,
        &requestInventoryEquip,
        &getInventoryEquipResult,
        &cancelInventoryEquip,
        &isEquipPickup,
        &getSample,
        &copyEvents,
    };
    return value;
}
}
