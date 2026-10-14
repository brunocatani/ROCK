#include "Boundary.h"
#include <ROCK/PlayerController.h>

namespace rock::api::playercontroller {
namespace {
using namespace boundary;
Status ROCK_CALL getPlayerControllerState(OwnerToken owner, PlayerControllerState* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        return provider::runtime::getPlayerControllerState(owner, *outState);
    });
}
Status ROCK_CALL requestPlayerControllerJump(OwnerToken owner, const PlayerControllerJumpRequest* request) noexcept {
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return provider::runtime::requestPlayerControllerJump(owner, *request);
    });
}
Status ROCK_CALL getSample(OwnerToken owner, Sample* outSample) noexcept {
    return readSample(owner, kInterfaceId, outSample);
}
}
const Api& table() noexcept {
    static const Api value{
        &getPlayerControllerState,
        &requestPlayerControllerJump,
        &getSample,
    };
    return value;
}
}
