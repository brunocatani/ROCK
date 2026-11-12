#include "OutdatedConsumers.h"

#include "InterfaceNegotiation.h"
#include "physics-interaction/PhysicsLog.h"

#include <RPS/Runtime/CallerModule.h>

#include <atomic>

namespace rock::api::discovery
{
    namespace
    {
        namespace caller_module = RPS::Runtime::CallerModule;

        enum class Mismatch
        {
            None,
            ConsumerOlder,  // built for an older ROCK API
            ConsumerNewer,  // needs a newer ROCK than this one
        };

        // Consumers request the exact major, minor and table size their headers
        // declare (ROCK/Client.h), so every version refusal is a real mismatch.
        Mismatch classify(Status status, InterfaceId id, std::uint32_t requestedMajor) noexcept
        {
            switch (status) {
            case Status::UnsupportedMajor:
                for (const auto& entry : registeredInterfaces())
                    if (entry.descriptor.interfaceId == id)
                        return requestedMajor < entry.descriptor.major ? Mismatch::ConsumerOlder : Mismatch::ConsumerNewer;
                return Mismatch::None;
            case Status::UnknownInterface:
                // A retired interface (Diagnostics, References, the old storage
                // family) asked for at major 1, or an interface from a later ROCK.
                return requestedMajor < core::kMajor ? Mismatch::ConsumerOlder : Mismatch::ConsumerNewer;
            case Status::UnsupportedMinor:
            case Status::TableTooSmall:
                return Mismatch::ConsumerNewer;
            default:
                return Mismatch::None;
            }
        }
    }

    void noteRefusal(Status status, InterfaceId id, std::uint32_t requestedMajor, const void* callerAddress) noexcept
    {
        const auto mismatch = classify(status, id, requestedMajor);
        if (mismatch == Mismatch::None || !caller_module::firstFromCaller(callerAddress))
            return;
        try {
            const auto dll = caller_module::moduleNameForLog(callerAddress);
            if (mismatch == Mismatch::ConsumerOlder) {
                ROCK_LOG_ERROR(Api,
                    "{} asked for interface {} major {}: it was built for an older ROCK and gets no ROCK features "
                    "until it is updated.",
                    dll, static_cast<std::uint32_t>(id), requestedMajor);
            } else {
                ROCK_LOG_ERROR(Api,
                    "{} asked for interface {} major {} (status {}): it needs a newer ROCK than {} and gets no ROCK "
                    "features until ROCK is updated.",
                    dll, static_cast<std::uint32_t>(id), requestedMajor, static_cast<std::uint32_t>(status),
                    Version::NAME);
            }
        } catch (...) {}
    }

    void noteRpsUiWithoutDiscovery() noexcept
    {
        static std::atomic_flag reported = ATOMIC_FLAG_INIT;
        if (reported.test_and_set())
            return;
        try {
            ROCK_LOG_ERROR(Api,
                "RPS_UI_Framework.dll does not export its 2.0 discovery: it is older than ROCK supports. ROCK's world "
                "drawing, ammo counters and input cooperation stay off until it is updated.");
        } catch (...) {}
    }
}
