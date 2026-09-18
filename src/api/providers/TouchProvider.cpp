#include "TouchMarshalling.h"
#include <ROCK/Discovery.h>
#include "api/EventStreams.h"

namespace rock::api::touch {
namespace {
using namespace boundary;
Status ROCK_CALL setTouchGrabTargetsForScopeV1(std::uint64_t ownerToken, std::uint64_t scopeToken, const TouchGrabTargetV1* targets, std::uint32_t targetCount) noexcept {
    if (targetCount > 256) return Status::CapacityFull;
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        if (targetCount && !targets) return Status::InvalidArgument;
        std::array<provider::RockProviderTouchGrabTargetV1, 256> native_targets{};
        for (std::uint32_t i=0; i<targetCount; ++i) {
            if (const auto s = checkInput(targets+i); s != Status::Ok) return s;
            convert(native_targets[i], targets[i]);
        }
        const auto result = provider::runtime::apiSetTouchGrabTargetsForScopeV1(ownerToken, scopeToken, native_targets.data(), targetCount);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL clearTouchGrabTargetsForScopeV1(std::uint64_t ownerToken, std::uint64_t scopeToken) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        const auto result = provider::runtime::apiClearTouchGrabTargetsForScopeV1(ownerToken, scopeToken);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL copyTouchGrabStatesForScopeV1(std::uint64_t ownerToken, std::uint64_t scopeToken, TouchGrabStateV1* outStates, std::uint32_t maxStates, std::uint32_t* outStateCount) noexcept {
    if (maxStates > 256) return Status::CapacityFull;
    if (maxStates && !outStates) return Status::InvalidArgument;
    for (std::uint32_t i=0;i<maxStates;++i) if (const auto status=checkOutput(outStates+i); status!=Status::Ok) return status;
    if (!outStateCount) return Status::InvalidArgument;
    *outStateCount = {};
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        std::array<provider::RockProviderTouchGrabStateV1, 256> native_outStates{};
        const auto result = provider::runtime::apiCopyTouchGrabStatesForScopeV1(ownerToken, scopeToken, native_outStates.data(), maxStates, outStateCount);
        for (std::uint32_t i=0; i<std::min<std::uint32_t>(maxStates, *outStateCount); ++i) convert(outStates[i], native_outStates[i]);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL requestTouchGrabYieldV1(std::uint64_t ownerToken, std::uint64_t scopeToken, std::uint64_t targetId, std::uint32_t targetGeneration) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        const auto result = provider::runtime::apiRequestTouchGrabYieldV1(ownerToken, scopeToken, targetId, targetGeneration);
        return static_cast<Status>(result);
    });
}

#include "TouchEndpoints.inl"
}
const ApiV1& table() noexcept {
    static const ApiV1 value{
        &setTouchGrabTargetsForScopeV1,
        &clearTouchGrabTargetsForScopeV1,
        &copyTouchGrabStatesForScopeV1,
        &requestTouchGrabYieldV1,
        &getSample,
        &copyEvents,
    };
    return value;
}
}
