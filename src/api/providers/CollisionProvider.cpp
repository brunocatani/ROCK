#include "CollisionMarshalling.h"
#include <ROCK/Discovery.h>
#include "api/EventStreams.h"

namespace rock::api::collision {
namespace {
using namespace boundary;
Status ROCK_CALL clearExternalBodies(std::uint64_t ownerToken) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        provider::runtime::apiClearExternalBodies(ownerToken);
        return Status::Ok;
    });
}
Status ROCK_CALL getBodyContactSnapshotV1(OwnerToken ownerToken, BodyContactV1* outContacts, std::uint32_t maxContacts, std::uint32_t* outCopied) noexcept {
    if (!outCopied) return Status::InvalidArgument;
    *outCopied = {};
    if (maxContacts > 128) return Status::CapacityFull;
    if (maxContacts && !outContacts) return Status::InvalidArgument;
    for (std::uint32_t i=0;i<maxContacts;++i) if (const auto status=checkOutput(outContacts+i); status!=Status::Ok) return status;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        std::array<provider::RockProviderBodyContactV1, 128> native_outContacts{};
        *outCopied = static_cast<std::uint32_t>(provider::runtime::apiGetBodyContactSnapshotV1(native_outContacts.data(), maxContacts));
        for (std::uint32_t i=0; i<std::min<std::uint32_t>(maxContacts, *outCopied); ++i) convert(outContacts[i], native_outContacts[i]);
        return Status::Ok;
    });
}
Status ROCK_CALL registerExternalBodiesForScopeV1(std::uint64_t ownerToken, std::uint64_t scopeToken, const ExternalBodyRegistration* bodies, std::uint32_t bodyCount) noexcept {
    if (bodyCount > 2048) return Status::CapacityFull;
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        if (bodyCount && !bodies) return Status::InvalidArgument;
        std::array<provider::RockProviderExternalBodyRegistration, 2048> native_bodies{};
        for (std::uint32_t i=0; i<bodyCount; ++i) {
            if (const auto s = checkInput(bodies+i); s != Status::Ok) return s;
            convert(native_bodies[i], bodies[i]);
            if (bodies[i].worldGeneration != provider::runtime::sample().worldGeneration || !bodies[i].worldGeneration) return Status::GenerationMismatch;
            native_bodies[i].ownerToken = scopeToken;
        }
        const auto result = provider::runtime::apiRegisterExternalBodiesForScopeV1(ownerToken, scopeToken, native_bodies.data(), bodyCount);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL clearExternalBodiesForScopeV1(std::uint64_t ownerToken, std::uint64_t scopeToken) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        const auto result = provider::runtime::apiClearExternalBodiesForScopeV1(ownerToken, scopeToken);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL copyExternalContactsSinceV1(std::uint64_t ownerToken, std::uint64_t scopeToken, std::uint64_t afterSequence, ExternalContactRecordV1* outContacts, std::uint32_t maxContacts, ExternalContactStreamStateV1* outStreamState) noexcept {
    if (maxContacts > 512) return Status::CapacityFull;
    if (maxContacts && !outContacts) return Status::InvalidArgument;
    for (std::uint32_t i=0;i<maxContacts;++i) if (const auto status=checkOutput(outContacts+i); status!=Status::Ok) return status;
    if (const auto s = checkOutput(outStreamState); s != Status::Ok) return s;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        std::array<provider::RockProviderExternalContactRecordV1, 512> native_outContacts{};
        provider::RockProviderExternalContactStreamStateV1 native_outStreamState{};
        const auto result = provider::runtime::apiCopyExternalContactsSinceV1(ownerToken, scopeToken, afterSequence, native_outContacts.data(), maxContacts, &native_outStreamState);
        for (std::uint32_t i=0; i<std::min<std::uint32_t>(maxContacts, native_outStreamState.copiedCount); ++i) convert(outContacts[i], native_outContacts[i]);
        convert(*outStreamState, native_outStreamState);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL copySemanticHandContactsV1(std::uint64_t ownerToken, Hand hand, std::uint32_t maxFramesSinceContact, SemanticHandContactV1* outContacts, std::uint32_t maxContacts, std::uint32_t* outContactCount) noexcept {
    if (maxContacts > 20) return Status::CapacityFull;
    if (maxContacts && !outContacts) return Status::InvalidArgument;
    for (std::uint32_t i=0;i<maxContacts;++i) if (const auto status=checkOutput(outContacts+i); status!=Status::Ok) return status;
    if (!outContactCount) return Status::InvalidArgument;
    *outContactCount = {};
    if (hand!=Hand::Right && hand!=Hand::Left) return Status::InvalidArgument;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        std::array<provider::RockProviderSemanticHandContactV1, 20> native_outContacts{};
        const auto result = provider::runtime::apiCopySemanticHandContactsV1(ownerToken, static_cast<provider::RockProviderHand>(hand), maxFramesSinceContact, native_outContacts.data(), maxContacts, outContactCount);
        for (std::uint32_t i=0; i<std::min<std::uint32_t>(maxContacts, *outContactCount); ++i) convert(outContacts[i], native_outContacts[i]);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL copyPlayerColliderDescriptorsV1(std::uint64_t ownerToken, PlayerColliderDescriptorV1* outDescriptors, std::uint32_t maxDescriptors, std::uint32_t* outDescriptorCount) noexcept {
    if (maxDescriptors > 96) return Status::CapacityFull;
    if (maxDescriptors && !outDescriptors) return Status::InvalidArgument;
    for (std::uint32_t i=0;i<maxDescriptors;++i) if (const auto status=checkOutput(outDescriptors+i); status!=Status::Ok) return status;
    if (!outDescriptorCount) return Status::InvalidArgument;
    *outDescriptorCount = {};
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        std::array<provider::RockProviderPlayerColliderDescriptorV1, 96> native_outDescriptors{};
        const auto result = provider::runtime::apiCopyPlayerColliderDescriptorsV1(ownerToken, native_outDescriptors.data(), maxDescriptors, outDescriptorCount);
        for (std::uint32_t i=0; i<std::min<std::uint32_t>(maxDescriptors, *outDescriptorCount); ++i) convert(outDescriptors[i], native_outDescriptors[i]);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL getHandCollisionAvailabilityV1(std::uint64_t ownerToken, Hand hand, HandCollisionAvailabilityV1* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    if (hand!=Hand::Right && hand!=Hand::Left) return Status::InvalidArgument;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::RockProviderHandCollisionAvailabilityV1 native_outState{};
        const auto result = provider::runtime::apiGetHandCollisionAvailabilityV1(ownerToken, static_cast<provider::RockProviderHand>(hand), &native_outState);
        convert(*outState, native_outState);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL queryWorldRaycastV1(std::uint64_t ownerToken, const WorldRaycastRequestV1* request, WorldRaycastResultV1* outResult) noexcept {
    if (const auto s = checkOutput(outResult); s != Status::Ok) return s;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        if (const auto s = checkInput(request); s != Status::Ok) return s;
        provider::RockProviderWorldRaycastRequestV1 native_request{};
        convert(native_request, *request);
        provider::RockProviderWorldRaycastResultV1 native_outResult{};
        const auto result = provider::runtime::apiQueryWorldRaycastV1(ownerToken, &native_request, &native_outResult);
        convert(*outResult, native_outResult);
        return static_cast<Status>(result);
    });
}

#include "CollisionEndpoints.inl"
}
const ApiV1& table() noexcept {
    static const ApiV1 value{
        &clearExternalBodies,
        &getBodyContactSnapshotV1,
        &registerExternalBodiesForScopeV1,
        &clearExternalBodiesForScopeV1,
        &copyExternalContactsSinceV1,
        &copySemanticHandContactsV1,
        &copyPlayerColliderDescriptorsV1,
        &getHandCollisionAvailabilityV1,
        &queryWorldRaycastV1,
        &getSample,
        &getEnvironment,
        &copyEvents,
    };
    return value;
}
}
