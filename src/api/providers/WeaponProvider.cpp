#include "Boundary.h"
#include "physics-interaction/weapon/EquipPickupScope.h"
#include <ROCK/Weapon.h>

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

Status ROCK_CALL getWeaponState(OwnerToken owner, std::uint32_t slot, WeaponState* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    outState->weaponSlot = slot;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        if (const auto s = p::runtime::copyPublishedWeaponState(*outState); s != Status::Ok) return s;
        outState->weaponSlot = slot;
        ::rock::provider_state_policy::WeaponHandling handling{};
        if (p::runtime::queryWeaponHandling(handling)) ::rock::provider_state_policy::applyHandling(*outState, handling);
        return Status::Ok;
    });
}

// The single-weapon reads: run, then name the slot the record describes.
template <class Record, class Read>
Status readSlot(OwnerToken owner, std::uint32_t slot, Record* outRecord, Read&& read) noexcept {
    if (const auto s = checkOutput(outRecord); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    outRecord->weaponSlot = slot;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        const auto status = read(*outRecord);
        outRecord->weaponSlot = slot;
        return status;
    });
}

Status ROCK_CALL getGrip(OwnerToken owner, std::uint32_t slot, Grip* outGrip) noexcept {
    return readSlot(owner, slot, outGrip, [](Grip& grip) { return p::runtime::getWeaponGrip(grip); });
}
Status ROCK_CALL getClassification(OwnerToken owner, std::uint32_t slot, Classification* outClassification) noexcept {
    return readSlot(owner, slot, outClassification, [](Classification& value) { return p::runtime::getWeaponClassification(value); });
}
Status ROCK_CALL getCompositionState(OwnerToken owner, std::uint32_t slot, CompositionState* outState) noexcept {
    return readSlot(owner, slot, outState, [](CompositionState& state) { return p::runtime::getWeaponCompositionState(state); });
}
Status ROCK_CALL getAuthoredGripPose(OwnerToken owner, std::uint32_t slot, AuthoredGripPose* outPose) noexcept {
    return readSlot(owner, slot, outPose, [](AuthoredGripPose& pose) { return p::runtime::getAuthoredGripPose(pose); });
}
Status ROCK_CALL getScopeSightState(OwnerToken owner, std::uint32_t slot, ScopeSightState* outState) noexcept {
    return readSlot(owner, slot, outState, [](ScopeSightState& state) { return p::runtime::getScopeSightState(state); });
}

Status ROCK_CALL copyEmitters(OwnerToken owner, std::uint32_t slot, Emitter* outEmitters, std::uint32_t capacity, std::uint32_t* outCount) noexcept {
    if (const auto s = checkOutputs(outEmitters, capacity, kMaxEmitters, outCount); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        *outCount = p::runtime::weaponEmitterCount();
        (void)p::runtime::copyWeaponEmitters(outEmitters, capacity);
        return Status::Ok;
    });
}

Status ROCK_CALL copyCompositionEntries(OwnerToken owner, std::uint32_t slot, CompositionEntry* outEntries, std::uint32_t capacity, std::uint32_t* outCount) noexcept {
    if (const auto s = checkOutputs(outEntries, capacity, kMaxCompositionEntries, outCount); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        return p::runtime::copyWeaponCompositionEntries(outEntries, capacity, *outCount);
    });
}

Status ROCK_CALL setHandlingAuthority(OwnerToken owner, const HandlingRequest* request) noexcept {
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    constexpr std::uint32_t knownFeatures = 0x3F;
    if (request->features & ~knownFeatures) return Status::InvalidArgument;
    return invoke(owner, kInterfaceId, 2, true, [&] { return p::runtime::setHandlingAuthority(owner, *request); });
}
Status ROCK_CALL clearHandlingAuthority(OwnerToken owner) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] { return p::runtime::clearHandlingAuthority(owner); });
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
    };
    return value;
}
}
