#include "Boundary.h"
#include "EventBoundary.h"
#include "api/AuthoredInteractionService.h"
#include <ROCK/Touch.h>

namespace rock::api::touch {
namespace {
using namespace boundary;
Status ROCK_CALL setTouchGrabTargetsForScope(OwnerToken owner, std::uint64_t scopeToken, const TouchGrabTarget* targets, std::uint32_t targetCount) noexcept {
    if (targetCount > kMaxTargets) return Status::CapacityFull;
    if (targetCount && !targets) return Status::InvalidArgument;
    for (std::uint32_t i = 0; i < targetCount; ++i) {
        if (const auto s = checkInput(targets + i); s != Status::Ok) return s;
    }
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return provider::runtime::setTouchGrabTargetsForScope(owner, scopeToken, targets, targetCount);
    });
}
Status ROCK_CALL clearTouchGrabTargetsForScope(OwnerToken owner, std::uint64_t scopeToken) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return provider::runtime::clearTouchGrabTargetsForScope(owner, scopeToken);
    });
}
Status ROCK_CALL copyTouchGrabStatesForScope(OwnerToken owner, std::uint64_t scopeToken, TouchGrabState* outStates, std::uint32_t maxStates, std::uint32_t* outStateCount) noexcept {
    if (maxStates > kMaxTargets) return Status::CapacityFull;
    if ((maxStates && !outStates) || !outStateCount) return Status::InvalidArgument;
    for (std::uint32_t i = 0; i < maxStates; ++i) {
        if (const auto s = checkOutput(outStates + i); s != Status::Ok) return s;
    }
    *outStateCount = 0;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        return provider::runtime::copyTouchGrabStatesForScope(owner, scopeToken, outStates, maxStates, *outStateCount);
    });
}
Status ROCK_CALL requestTouchGrabYield(OwnerToken owner, std::uint64_t scopeToken, std::uint64_t targetId, std::uint32_t targetGeneration) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return provider::runtime::requestTouchGrabYield(owner, scopeToken, targetId, targetGeneration);
    });
}
Status ROCK_CALL setAuthoredHand(OwnerToken owner, const AuthoredHand* request) noexcept {
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] { return authored_interaction::set(owner, *request); });
}
Status ROCK_CALL clearAuthoredHand(OwnerToken owner, Hand hand) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] { return authored_interaction::clear(owner, hand); });
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
        &setTouchGrabTargetsForScope,
        &clearTouchGrabTargetsForScope,
        &copyTouchGrabStatesForScope,
        &requestTouchGrabYield,
        &setAuthoredHand,
        &clearAuthoredHand,
        &getSample,
        &copyEvents,
    };
    return value;
}
}
