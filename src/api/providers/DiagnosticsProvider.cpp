#include "DiagnosticsMarshalling.h"
#include <ROCK/Discovery.h>
#include "api/EventStreams.h"

namespace rock::api::diagnostics {
namespace {
using namespace boundary;
Status ROCK_CALL publishDebugOverlayV1(std::uint64_t ownerToken, const DebugOverlayPublicationV1* publication) noexcept;
Status ROCK_CALL clearDebugOverlayV1(std::uint64_t ownerToken) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        const auto result = provider::runtime::apiClearDebugOverlayV1(ownerToken);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL setColliderVisualizationOverrideV1(std::uint64_t ownerToken, const ColliderVisualizationRequestV1* request) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        if (const auto s = checkInput(request); s != Status::Ok) return s;
        provider::RockProviderColliderVisualizationRequestV1 native_request{};
        convert(native_request, *request);
        const auto result = provider::runtime::apiSetColliderVisualizationOverrideV1(ownerToken, &native_request);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL clearColliderVisualizationOverrideV1(std::uint64_t ownerToken) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        const auto result = provider::runtime::apiClearColliderVisualizationOverrideV1(ownerToken);
        return static_cast<Status>(result);
    });
}

#include "DiagnosticsEndpoints.inl"
}
const ApiV1& table() noexcept {
    static const ApiV1 value{
        &publishDebugOverlayV1,
        &clearDebugOverlayV1,
        &setColliderVisualizationOverrideV1,
        &clearColliderVisualizationOverrideV1,
        &getSample,
        &copyEvents,
    };
    return value;
}
}
