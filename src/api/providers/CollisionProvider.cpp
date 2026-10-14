#include "Boundary.h"
#include "SnapshotReads.h"
#include <ROCK/Collision.h>
#include <algorithm>
#include <array>

namespace rock::api::collision {
namespace {
using namespace boundary;
namespace p = provider;

bool validHand(Hand hand) noexcept { return hand == Hand::Right || hand == Hand::Left; }

// Validates a caller array of public records before any write.
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

Status ROCK_CALL copySemanticHandContacts(OwnerToken owner, Hand hand, std::uint32_t maxFramesSinceContact, SemanticHandContact* outContacts, std::uint32_t capacity, std::uint32_t* outCount) noexcept {
    if (const auto s = checkOutputs(outContacts, capacity, kMaxSemanticContacts, outCount); s != Status::Ok) return s;
    if (!validHand(hand)) return Status::InvalidArgument;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        return p::runtime::copySemanticHandContacts(hand, maxFramesSinceContact, outContacts, capacity, *outCount);
    });
}
Status ROCK_CALL copyPlayerColliderDescriptors(OwnerToken owner, PlayerColliderDescriptor* outDescriptors, std::uint32_t capacity, std::uint32_t* outCount) noexcept {
    if (const auto s = checkOutputs(outDescriptors, capacity, kMaxColliderDescriptors, outCount); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        return p::runtime::copyPlayerColliderDescriptors(outDescriptors, capacity, *outCount);
    });
}
Status ROCK_CALL copyBodyContacts(OwnerToken owner, BodyContact* outContacts, std::uint32_t capacity, std::uint32_t* outCount) noexcept {
    if (const auto s = checkOutputs(outContacts, capacity, kMaxBodyContacts, outCount); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        *outCount = p::runtime::copyBodyContacts(outContacts, capacity);
        return Status::Ok;
    });
}
Status ROCK_CALL getHandCollisionAvailability(OwnerToken owner, Hand hand, HandCollisionAvailability* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    if (!validHand(hand)) return Status::InvalidArgument;
    return invoke(owner, kInterfaceId, 1, true, [&] { return p::runtime::getHandCollisionAvailability(hand, *outState); });
}
Status ROCK_CALL queryWorldRaycast(OwnerToken owner, const WorldRaycastRequest* request, WorldRaycastResult* outResult) noexcept {
    if (const auto s = checkOutput(outResult); s != Status::Ok) return s;
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] { return p::runtime::queryWorldRaycast(owner, *request, *outResult); });
}
Status ROCK_CALL requestEjectionClearance(OwnerToken owner, const EjectionClearanceRequest* request) noexcept {
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    if (const auto s = requireWeaponSlot(request->weaponSlot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] { return p::runtime::requestEjectionClearance(owner, *request); });
}
Status ROCK_CALL cancelEjectionClearance(OwnerToken owner, std::uint32_t referenceFormId) noexcept {
    return invoke(owner, kInterfaceId, 0, true, [&] { return p::runtime::cancelEjectionClearance(owner, referenceFormId); });
}
Status ROCK_CALL watchObjectRest(OwnerToken owner, const ObjectRestRequest* request, ObjectRestState* outState) noexcept {
    if (!outState) return Status::InvalidArgument;
    *outState = ObjectRestState::Waiting;
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] { return p::runtime::watchObjectRest(owner, *request, *outState); });
}
Status ROCK_CALL cancelObjectRest(OwnerToken owner, std::uint32_t referenceFormId) noexcept {
    return invoke(owner, kInterfaceId, 0, true, [&] { return p::runtime::cancelObjectRest(owner, referenceFormId); });
}
Status ROCK_CALL getSample(OwnerToken owner, Sample* outSample) noexcept {
    return readSample(owner, kInterfaceId, outSample);
}
}
const Api& table() noexcept {
    static const Api value{
        &copySemanticHandContacts,
        &copyPlayerColliderDescriptors,
        &copyBodyContacts,
        &getHandCollisionAvailability,
        &queryWorldRaycast,
        &getEnvironment,
        &requestEjectionClearance,
        &cancelEjectionClearance,
        &watchObjectRest,
        &cancelObjectRest,
        &getSample,
    };
    return value;
}
}
