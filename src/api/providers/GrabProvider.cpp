#include <ROCK/GrabV1_1.h>
#include "GrabMarshalling.h"
#include <ROCK/Discovery.h>
#include "EventBoundary.h"

namespace rock::api::grab {
namespace {
using namespace boundary;
Status ROCK_CALL requestForceGrabV1(std::uint64_t ownerToken, const ForceGrabRequestV1* request, std::uint64_t* outCommandId) noexcept {
    if (!outCommandId) return Status::InvalidArgument;
    *outCommandId = {};
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        if (const auto s = checkInput(request); s != Status::Ok) return s;
        provider::RockProviderForceGrabRequestV1 native_request{};
        convert(native_request, *request);
        if (request->flags & ~(1u << 0)) return Status::InvalidArgument;
        const auto result = provider::runtime::apiRequestForceGrabV1(ownerToken, &native_request, outCommandId);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL getInteractionCommandResultV1(std::uint64_t ownerToken, std::uint64_t commandId, InteractionCommandResultV1* outResult) noexcept {
    if (const auto s = checkOutput(outResult); s != Status::Ok) return s;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::RockProviderInteractionCommandResultV1 native_outResult{};
        const auto result = provider::runtime::apiGetInteractionCommandResultV1(ownerToken, commandId, &native_outResult);
        convert(*outResult, native_outResult);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL requestForceReleaseV1(std::uint64_t ownerToken, const ForceReleaseRequestV1* request, std::uint64_t* outCommandId) noexcept {
    if (!outCommandId) return Status::InvalidArgument;
    *outCommandId = {};
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        if (const auto s = checkInput(request); s != Status::Ok) return s;
        provider::RockProviderForceReleaseRequestV1 native_request{};
        convert(native_request, *request);
        const auto result = provider::runtime::apiRequestForceReleaseV1(ownerToken, &native_request, outCommandId);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL requestThrownDropV1(std::uint64_t ownerToken, const ThrownDropRequestV1* request, std::uint64_t* outCommandId) noexcept {
    if (!outCommandId) return Status::InvalidArgument;
    *outCommandId = {};
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        if (const auto s = checkInput(request); s != Status::Ok) return s;
        provider::RockProviderThrownDropRequestV1 native_request{};
        convert(native_request, *request);
        const auto result = provider::runtime::apiRequestThrownDropV1(ownerToken, &native_request, outCommandId);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL cancelInteractionCommandV1(std::uint64_t ownerToken, std::uint64_t commandId) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        const auto result = provider::runtime::apiCancelInteractionCommandV1(ownerToken, commandId);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL acquireOffhandReservationV1(std::uint64_t ownerToken, const OffhandReservationRequestV1* request) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        if (const auto s = checkInput(request); s != Status::Ok) return s;
        provider::RockProviderOffhandReservationRequestV1 native_request{};
        convert(native_request, *request);
        const auto result = provider::runtime::apiAcquireOffhandReservationV1(ownerToken, &native_request);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL renewOffhandReservationV1(std::uint64_t ownerToken, const OffhandReservationRequestV1* request) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        if (const auto s = checkInput(request); s != Status::Ok) return s;
        provider::RockProviderOffhandReservationRequestV1 native_request{};
        convert(native_request, *request);
        const auto result = provider::runtime::apiRenewOffhandReservationV1(ownerToken, &native_request);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL releaseOffhandReservationV1(std::uint64_t ownerToken) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        const auto result = provider::runtime::apiReleaseOffhandReservationV1(ownerToken);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL getOffhandReservationStateV1(std::uint64_t ownerToken, OffhandReservationStateV1* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::RockProviderOffhandReservationStateV1 native_outState{};
        const auto result = provider::runtime::apiGetOffhandReservationStateV1(ownerToken, &native_outState);
        convert(*outState, native_outState);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL getHandTargetDetailsV1(std::uint64_t ownerToken, Hand hand, HandTargetDetailsV1* outDetails) noexcept {
    if (const auto s = checkOutput(outDetails); s != Status::Ok) return s;
    if (hand!=Hand::Right && hand!=Hand::Left) return Status::InvalidArgument;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::RockProviderHandTargetDetailsV1 native_outDetails{};
        const auto result = provider::runtime::apiGetHandTargetDetailsV1(ownerToken, static_cast<provider::RockProviderHand>(hand), &native_outDetails);
        convert(*outDetails, native_outDetails);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL requestPowerArmorGrabV1(std::uint64_t ownerToken, const PowerArmorGrabRequestV1* request, std::uint64_t* outCommandId) noexcept {
    if (!outCommandId) return Status::InvalidArgument;
    *outCommandId = {};
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        if (const auto s = checkInput(request); s != Status::Ok) return s;
        provider::RockProviderPowerArmorGrabRequestV1 native_request{};
        convert(native_request, *request);
        const auto result = provider::runtime::apiRequestPowerArmorGrabV1(ownerToken, &native_request, outCommandId);
        return static_cast<Status>(result);
    });
}

#include "GrabEndpoints.inl"
Status ROCK_CALL getHeldPlacementState(OwnerToken owner,v1_1::HeldPlacementState* out) noexcept {
    if (const auto s=checkOutput(out); s!=Status::Ok) return s;
    return invoke(owner,kInterfaceId,1,true,[&] {return provider::runtime::getHeldPlacementState(*out);});
}
Status ROCK_CALL submitHeldPlacementIntent(OwnerToken owner,const v1_1::HeldPlacementIntent* request) noexcept {
    if (const auto s=checkInput(request); s!=Status::Ok) return s;
    return invoke(owner,kInterfaceId,2,true,[&] {return provider::runtime::submitHeldPlacementIntent(owner,*request);});
}
Status ROCK_CALL clearHeldPlacementIntent(OwnerToken owner) noexcept {
    return invoke(owner,kInterfaceId,2,true,[&] {return provider::runtime::clearHeldPlacementIntent(owner);});
}

}
const v1_1::Api& tableV1_1() noexcept {
    static const v1_1::Api value{{
        &requestForceGrabV1,
        &getInteractionCommandResultV1,
        &requestForceReleaseV1,
        &requestThrownDropV1,
        &getHandInteractionStateV1,
        &cancelInteractionCommandV1,
        &acquireOffhandReservationV1,
        &renewOffhandReservationV1,
        &releaseOffhandReservationV1,
        &getOffhandReservationStateV1,
        &getHandTargetDetailsV1,
        &requestPowerArmorGrabV1,
        &getSample,
        &requestInventoryGrab,
        &copyEvents,
        &setEventCallback,
        &clearEventCallback,
    },&getHeldPlacementState,&submitHeldPlacementIntent,&clearHeldPlacementIntent};
    return value;
}
const ApiV1& table() noexcept { return tableV1_1().v1; }

}
