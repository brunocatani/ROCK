#include "CommonMarshalling.h"
#include "physics-interaction/PhysicsLog.h"
#include <ROCK/Discovery.h>
#include "EventBoundary.h"

namespace rock::api::diagnostics {
namespace {
using namespace boundary;
// Retired in ROCK 1.0: ROCK draws nothing for other mods; they draw through
// RPS UI Framework world drawing. The slots stay so getSample and copyEvents
// keep binding until the Diagnostics major-2 rework deletes them (provider
// API reset ledger, entry 2).
Status retiredDrawing(std::atomic_bool& reported, const char* function) noexcept {
    if (!reported.exchange(true, std::memory_order_relaxed)) {
        ROCK_LOG_WARN(Api, "Diagnostics {} is retired in ROCK 1.0; draw through RPS UI Framework world drawing", function);
    }
    return Status::UnsupportedVersion;
}
Status ROCK_CALL publishDebugOverlayV1(std::uint64_t, const DebugOverlayPublicationV1*) noexcept {
    static std::atomic_bool reported{ false };
    return retiredDrawing(reported, "publishDebugOverlayV1");
}
Status ROCK_CALL clearDebugOverlayV1(std::uint64_t) noexcept {
    static std::atomic_bool reported{ false };
    return retiredDrawing(reported, "clearDebugOverlayV1");
}
Status ROCK_CALL setColliderVisualizationOverrideV1(std::uint64_t, const ColliderVisualizationRequestV1*) noexcept {
    static std::atomic_bool reported{ false };
    return retiredDrawing(reported, "setColliderVisualizationOverrideV1");
}
Status ROCK_CALL clearColliderVisualizationOverrideV1(std::uint64_t) noexcept {
    static std::atomic_bool reported{ false };
    return retiredDrawing(reported, "clearColliderVisualizationOverrideV1");
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
