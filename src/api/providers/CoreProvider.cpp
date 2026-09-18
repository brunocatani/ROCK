#include "CoreMarshalling.h"
#include <ROCK/Discovery.h>
#include "api/EventStreams.h"

namespace rock::api::core {
namespace {
using namespace boundary;
Status ROCK_CALL getModVersion(OwnerToken ownerToken, const char** outValue) noexcept {
    if (!outValue) return Status::InvalidArgument;
    *outValue = nullptr;
    return invoke(ownerToken, kInterfaceId, 1, false, [&]() {
        *outValue = provider::runtime::apiGetModVersion();
        return Status::Ok;
    });
}
Status ROCK_CALL isProviderReady(OwnerToken ownerToken, std::uint32_t* outValue) noexcept {
    if (!outValue) return Status::InvalidArgument;
    *outValue = {};
    return invoke(ownerToken, kInterfaceId, 1, false, [&]() -> Status {
        *outValue = static_cast<std::uint32_t>(provider::runtime::apiIsProviderReady());
        return Status::Ok;
    });
}
Status ROCK_CALL getFrameSnapshot(OwnerToken ownerToken, SnapshotV1* outSnapshot) noexcept {
    if (const auto s = checkOutput(outSnapshot); s != Status::Ok) return s;
    return invoke(ownerToken, kInterfaceId, 1, false, [&]() -> Status {
        provider::RockProviderFrameSnapshot native_outSnapshot{};
        const auto result = provider::runtime::apiGetFrameSnapshot(&native_outSnapshot);
        convert(*outSnapshot, native_outSnapshot);
        return result ? Status::Ok : Status::NotReady;
    });
}
Status ROCK_CALL registerConsumerV1(const RegistrationV1* registration, OwnerV1* outHandle) noexcept;
Status ROCK_CALL unregisterConsumerV1(std::uint64_t ownerToken) noexcept;
Status ROCK_CALL registerAnimationPhaseCallbackV1(std::uint64_t ownerToken, PhaseCallbackV1 callback, void* userData, std::uint64_t* outCallbackToken) noexcept;
Status ROCK_CALL unregisterAnimationPhaseCallbackV1(std::uint64_t ownerToken, std::uint64_t callbackToken) noexcept;
Status ROCK_CALL registerFrameCallbackForOwnerV1(std::uint64_t ownerToken, FrameCallbackV1 callback, void* userData, std::uint64_t* outCallbackToken) noexcept;
Status ROCK_CALL unregisterFrameCallbackForOwnerV1(std::uint64_t ownerToken, std::uint64_t callbackToken) noexcept;

#include "CoreEndpoints.inl"
}
const ApiV1& table() noexcept {
    static const ApiV1 value{
        &getModVersion,
        &isProviderReady,
        &getFrameSnapshot,
        &registerConsumerV1,
        &unregisterConsumerV1,
        &registerAnimationPhaseCallbackV1,
        &unregisterAnimationPhaseCallbackV1,
        &registerFrameCallbackForOwnerV1,
        &unregisterFrameCallbackForOwnerV1,
        &getSample,
        &bindInterface,
        &copyEvents,
    };
    return value;
}
}
