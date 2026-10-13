#include "Boundary.h"
#include "EventBoundary.h"
#include <ROCK/Core.h>
#include <cstring>

namespace rock::api::core {
namespace {
using namespace boundary;
Status ROCK_CALL getModVersion(OwnerToken owner, const char** outValue) noexcept {
    if (!outValue) return Status::InvalidArgument;
    *outValue = nullptr;
    return invoke(owner, kInterfaceId, 1, false, [&] {
        *outValue = provider::runtime::apiGetModVersion();
        return Status::Ok;
    });
}
Status ROCK_CALL isProviderReady(OwnerToken owner, std::uint32_t* outValue) noexcept {
    if (!outValue) return Status::InvalidArgument;
    *outValue = 0;
    return invoke(owner, kInterfaceId, 1, false, [&] {
        *outValue = provider::runtime::apiIsProviderReady() ? 1u : 0u;
        return Status::Ok;
    });
}
Status ROCK_CALL getSnapshot(OwnerToken owner, Snapshot* outSnapshot) noexcept {
    if (const auto s = checkOutput(outSnapshot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, false, [&] {
        provider::RockProviderFrameSnapshot snapshot{};
        if (!provider::runtime::apiGetFrameSnapshot(&snapshot)) return Status::NotReady;
        *outSnapshot = provider::runtime::toCoreSnapshot(snapshot);
        return Status::Ok;
    });
}
Status ROCK_CALL registerConsumer(const Registration* registration, Owner* outOwner) noexcept {
    if (provider::events::inSynchronousCallback()) return Status::Busy;
    if (const auto s = checkInput(registration); s != Status::Ok) return s;
    if (const auto s = checkOutput(outOwner); s != Status::Ok) return s;
    try {
        provider::RockProviderConsumerRegistrationV1 request{};
        static_assert(sizeof(request.modName) == sizeof(registration->modName));
        std::memcpy(request.modName, registration->modName, sizeof(request.modName));
        request.requestedCapabilities = 0xFFFFFFFFu;
        provider::RockProviderConsumerHandleV1 handle{};
        const auto status = provider::runtime::apiRegisterConsumerV1(&request, &handle);
        outOwner->ownerToken = handle.ownerToken;
        outOwner->providerGeneration = handle.providerGeneration;
        return static_cast<Status>(status);
    } catch (...) { return Status::InternalError; }
}
Status ROCK_CALL unregisterConsumer(OwnerToken owner) noexcept {
    return invoke(owner, kInterfaceId, 0, true, [&] {
        return static_cast<Status>(provider::runtime::apiUnregisterConsumerV1(owner));
    });
}
Status ROCK_CALL registerFrameCallback(OwnerToken owner, FrameCallback callback, void* userData, std::uint64_t* outCallbackToken) noexcept {
    if (outCallbackToken) *outCallbackToken = 0;
    return invoke(owner, kInterfaceId, 4, true, [&] {
        return static_cast<Status>(provider::runtime::apiRegisterFrameCallbackForOwnerV1(owner, callback, userData, outCallbackToken));
    });
}
Status ROCK_CALL unregisterFrameCallback(OwnerToken owner, std::uint64_t callbackToken) noexcept {
    return invoke(owner, kInterfaceId, 4, true, [&] {
        return static_cast<Status>(provider::runtime::apiUnregisterFrameCallbackForOwnerV1(owner, callbackToken));
    });
}
Status ROCK_CALL bindInterface(OwnerToken owner, InterfaceId family, std::uint32_t major, std::uint32_t permissions) noexcept {
    try { return provider::runtime::bind(owner, family, major, permissions); }
    catch (...) { return Status::InternalError; }
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
        &getModVersion,
        &isProviderReady,
        &getSnapshot,
        &registerConsumer,
        &unregisterConsumer,
        &registerFrameCallback,
        &unregisterFrameCallback,
        &bindInterface,
        &getSample,
        &copyEvents,
    };
    return value;
}
}
