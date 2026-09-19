#include "PlayerControllerMarshalling.h"
#include <ROCK/Discovery.h>

namespace rock::api::playercontroller {
namespace {
using namespace boundary;
Status ROCK_CALL getPlayerControllerStateV1(std::uint64_t ownerToken, PlayerControllerStateV1* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::RockProviderPlayerControllerStateV1 native_outState{};
        const auto result = provider::runtime::apiGetPlayerControllerStateV1(ownerToken, 0, &native_outState);
        convert(*outState, native_outState);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL requestPlayerControllerJumpV1(std::uint64_t ownerToken, const PlayerControllerJumpRequestV1* request) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        if (const auto s = checkInput(request); s != Status::Ok) return s;
        provider::RockProviderPlayerControllerJumpRequestV1 native_request{};
        convert(native_request, *request);
        const auto result = provider::runtime::apiRequestPlayerControllerJumpV1(ownerToken, &native_request);
        return static_cast<Status>(result);
    });
}

#include "PlayerControllerEndpoints.inl"
}
const ApiV1& table() noexcept {
    static const ApiV1 value{
        &getPlayerControllerStateV1,
        &requestPlayerControllerJumpV1,
        &getSample,
    };
    return value;
}
}
