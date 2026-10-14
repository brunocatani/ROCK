#include "api/providers/Boundary.h"
#include "physics-interaction/weapon/EquipPickupScope.h"
#include <ROCK/Weapon.h>
#include <cassert>

namespace rock::api::weapon { const Api& table() noexcept; }

namespace {
    using namespace rock;
    constexpr api::OwnerToken testOwner = 42;
    std::uint32_t permissions = 3;
    bool weaponPresent = true;
    bool firingLeft = true;
    bool twoHand = true;
    std::uint32_t emitterCount = 5;
    unsigned handlingRequests = 0;
}

// Runtime boundary fixture: the published Weapon table is the code under test.
namespace rock::provider::runtime {
    api::Status authorize(std::uint64_t owner, api::InterfaceId family, std::uint32_t permission, bool, OwnerAccess) {
        if (owner != testOwner) return api::Status::OwnerNotRegistered;
        if (family != api::InterfaceId::Weapon || (permission & ~permissions)) return api::Status::PermissionDenied;
        return api::Status::Ok;
    }
    api::Sample sample() { return {20, 30, 1, 2, 3, 4}; }
    void reportBoundaryFailure(std::uint64_t, api::InterfaceId) noexcept { assert(false); }
    api::Status apiGetEquippedWeaponStateV1(std::uint64_t, RockProviderEquippedWeaponStateV1* out) {
        out->frameIndex = 19; out->worldGeneration = 7; out->skeletonGeneration = 8; out->providerGeneration = 9;
        out->flags = static_cast<std::uint32_t>(api::weapon::WeaponStateFlag::Valid);
        out->weaponFormId = 123; out->weaponGenerationKey = 456; out->transitionSequence = 3;
        return api::Status::Ok;
    }
    bool apiGetEquippedWeaponHandlingStateV1(RockProviderEquippedWeaponHandlingStateV1* out) {
        using F = RockProviderEquippedWeaponHandlingRuntimeFlagV1;
        out->currentFiringHand = firingLeft ? api::Hand::Left : api::Hand::Right;
        out->runtimeFlags = static_cast<std::uint32_t>(F::FiringGripOccupied) | static_cast<std::uint32_t>(F::PartCarryActive) |
            static_cast<std::uint32_t>(F::AuthorityActive) | (weaponPresent ? static_cast<std::uint32_t>(F::WeaponPresent) : 0u);
        return true;
    }
    bool apiGetEquippedWeaponGripStateV1(std::uint64_t, RockProviderEquippedWeaponGripStateV1* out) {
        using G = RockProviderEquippedWeaponGripStateFlagV1;
        out->flags = static_cast<std::uint32_t>(G::Valid) | static_cast<std::uint32_t>(G::MuzzleWorldValid) |
            (twoHand ? static_cast<std::uint32_t>(G::TwoHandGripActive) : 0u) | (firingLeft ? static_cast<std::uint32_t>(G::FiringHandLeft) : 0u);
        out->weaponFormId = 123; out->muzzleOriginGame = {1, 2, 3};
        return true;
    }
    bool apiQueryEquippedWeaponClassificationV1(RockProviderWeaponClassificationV1* out) { out->valid = 1; out->formId = 123; return true; }
    std::uint32_t apiGetWeaponEmitterCountV1() { return emitterCount; }
    std::uint32_t apiCopyWeaponEmittersV1(RockProviderWeaponEmitterV1* out, std::uint32_t capacity) {
        const auto copied = (std::min)(capacity, emitterCount);
        for (std::uint32_t i = 0; i < copied; ++i) out[i].omodFormId = 100 + i;
        return copied;
    }
    api::Status apiSetEquippedWeaponHandlingAuthorityV1(std::uint64_t, const RockProviderEquippedWeaponHandlingRequestV1* request) {
        assert(request->flags == 3 && request->leaseFrames == 10 && request->leftFiringAimOffsetGameUnits[2] == 4);
        ++handlingRequests;
        return api::Status::Ok;
    }
    api::Status apiClearEquippedWeaponHandlingAuthorityV1(std::uint64_t) { return api::Status::Ok; }
    api::Status apiGetScopeSightStateV1(std::uint64_t, RockProviderScopeSightStateV1*) { return api::Status::Ok; }
    api::Status apiGetWeaponCompositionStateV1(std::uint64_t, RockProviderWeaponCompositionStateV1*) { return api::Status::Ok; }
    api::Status apiCopyWeaponCompositionEntriesV1(std::uint64_t, RockProviderWeaponCompositionEntryV1*, std::uint32_t, std::uint32_t* count) {
        *count = 0; return api::Status::Ok;
    }
    api::Status apiGetSelectedAuthoredGripPoseV1(std::uint64_t, RockProviderAuthoredGripPoseV1*) { return api::Status::Ok; }
    api::Status getHandlingAuthority(api::weapon::HandlingAuthority& out) { out.active = 1; out.owner = testOwner; return api::Status::Ok; }
    api::Status captureInventoryWeapon(std::uint64_t, std::uint32_t, std::uint32_t, api::weapon::InventoryWeapon&) { return api::Status::Ok; }
    api::Status requestInventoryEquip(std::uint64_t, const api::weapon::EquipRequest&, std::uint64_t& command) { command = 9; return api::Status::RequestQueued; }
    api::Status getInventoryEquipResult(std::uint64_t, std::uint64_t, api::weapon::EquipResult&) { return api::Status::Ok; }
    api::Status cancelInventoryEquip(std::uint64_t, std::uint64_t) { return api::Status::Ok; }
}
namespace rock::provider::events {
    api::Status copy(api::OwnerToken, std::uint64_t, api::weapon::Event*, std::uint32_t, api::Stream&) { return api::Status::Ok; }
}

int main() {
    using namespace rock::api;
    using namespace rock::api::weapon;
    const auto& api = table();

    // Slots: only equip index 0 is published; slot 1 is valid but unavailable.
    WeaponState state;
    assert(api.getWeaponState(testOwner, kMaxWeaponSlots, &state) == Status::InvalidArgument);
    assert(api.getWeaponState(testOwner, 1, &state) == Status::TargetUnavailable && state.weaponSlot == 1 && state.weaponFormId == 0);
    Grip grip;
    assert(api.getGrip(testOwner, 1, &grip) == Status::TargetUnavailable);

    // Slot 0: hand assignment is data. Left fires; the two-hand grip puts the
    // right hand on the support grip.
    assert(api.getWeaponState(testOwner, 0, &state) == Status::Ok);
    assert(state.weaponSlot == 0 && state.weaponFormId == 123 && state.weaponGenerationKey == 456 && state.transitionSequence == 3);
    assert(state.firingHand == Hand::Left && state.supportHand == Hand::Right);
    assert(state.handlingFlags == (static_cast<std::uint32_t>(HandlingStateFlag::FiringGripHeld) | static_cast<std::uint32_t>(HandlingStateFlag::PartCarry)));
    assert(state.sample.frameIndex == 19 && state.sample.publicationSequence == 30 && state.sample.worldGeneration == 7 &&
           state.sample.providerGeneration == 9 && state.sample.collisionGeneration == 4);
    firingLeft = false; twoHand = false;
    assert(api.getWeaponState(testOwner, 0, &state) == Status::Ok && state.firingHand == Hand::Right && state.supportHand == Hand::None);
    weaponPresent = false;
    assert(api.getWeaponState(testOwner, 0, &state) == Status::Ok && state.firingHand == Hand::None && state.supportHand == Hand::None &&
           state.handlingFlags == 0 && state.weaponFormId == 123);
    weaponPresent = true; firingLeft = true; twoHand = true;

    // The retired grip bits never reach a consumer.
    assert(api.getGrip(testOwner, 0, &grip) == Status::Ok && grip.weaponSlot == 0 && grip.muzzleOriginGame.z == 3);
    assert(grip.flags == (static_cast<std::uint32_t>(GripFlag::Valid) | static_cast<std::uint32_t>(GripFlag::MuzzleWorldValid)));

    // Emitters: one call reports the full count and copies what fits.
    Emitter emitters[2];
    std::uint32_t count = 99;
    assert(api.copyEmitters(testOwner, 0, emitters, 2, &count) == Status::Ok && count == 5 && emitters[1].omodFormId == 101);
    assert(api.copyEmitters(testOwner, 0, nullptr, 0, &count) == Status::Ok && count == 5);
    assert(api.copyEmitters(testOwner, 0, emitters, kMaxEmitters + 1, &count) == Status::CapacityFull && count == 0);
    assert(api.copyEmitters(testOwner, 1, emitters, 2, &count) == Status::TargetUnavailable);

    // Handling authority: unknown features are refused before ROCK sees them.
    HandlingRequest request;
    request.features = 3; request.leaseFrames = 10; request.leftFiringAimOffsetGameUnits[2] = 4;
    assert(api.setHandlingAuthority(testOwner, &request) == Status::Ok && handlingRequests == 1);
    request.features = 1u << 6;
    assert(api.setHandlingAuthority(testOwner, &request) == Status::InvalidArgument && handlingRequests == 1);
    HandlingAuthority authority;
    assert(api.getHandlingAuthority(testOwner, &authority) == Status::Ok && authority.active == 1 && authority.owner == testOwner);

    // Equip pickup intent is visible only inside ROCK's acquisition scope.
    std::uint32_t matches = 7;
    assert(api.isEquipPickup(testOwner, 500, 1, &matches) == Status::Ok && matches == 0);
    {
        const rock::equip_pickup::Scope acquiring(500, 1);
        assert(api.isEquipPickup(testOwner, 500, 1, &matches) == Status::Ok && matches == 1);
        assert(api.isEquipPickup(testOwner, 500, 2, &matches) == Status::Ok && matches == 0);
    }

    // Permissions are Weapon's own.
    permissions = 1;
    request.features = 3;
    assert(api.setHandlingAuthority(testOwner, &request) == Status::PermissionDenied);
    EquipRequest equip;
    std::uint64_t command = 0;
    assert(api.requestInventoryEquip(testOwner, &equip, &command) == Status::PermissionDenied && command == 0);
    permissions = 0;
    assert(api.getWeaponState(testOwner, 0, &state) == Status::PermissionDenied && state.weaponFormId == 0);
}
