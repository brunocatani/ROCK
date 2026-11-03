#include "OutdatedConsumers.h"

#include "InterfaceNegotiation.h"
#include "physics-interaction/PhysicsLog.h"
#include "rock_support/Logger.h"

#include <RPS/Runtime/RequiredUpdate.h>

#include <string>

namespace rock::api::discovery
{
    namespace
    {
        namespace required_update = RPS::Runtime::RequiredUpdate;

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

        // DLL file names for the log; characters outside ASCII become '?'.
        std::string narrowForLog(const std::wstring& text)
        {
            std::string result;
            result.reserve(text.size());
            for (const wchar_t c : text)
                result.push_back(c > 0 && c < 0x80 ? static_cast<char>(c) : '?');
            return result;
        }

        void flushLog() noexcept
        {
            try {
                if (const auto& log = rock::logger::internal::loggerInstance)
                    log->flush();
            } catch (...) {}
        }
    }

    void noteRefusal(Status status, InterfaceId id, std::uint32_t requestedMajor, const void* callerAddress) noexcept
    {
        const auto mismatch = classify(status, id, requestedMajor);
        if (mismatch == Mismatch::None || !required_update::firstFromCaller(callerAddress))
            return;
        try {
            auto dll = required_update::moduleFileNameAt(callerAddress);
            const std::string dllForLog = dll.empty() ? std::string{ "an unidentified module" } : narrowForLog(dll);
            if (dll.empty())
                dll = L"A mod (see ROCK.log)";
            const std::wstring rockVersion(Version::NAME.begin(), Version::NAME.end());
            std::wstring paragraph;
            if (mismatch == Mismatch::ConsumerOlder) {
                ROCK_LOG_CRITICAL(Api,
                    "{} asked for interface {} major {}: it was built for an older ROCK. The game stops at the main "
                    "menu until it is updated or removed.",
                    dllForLog, static_cast<std::uint32_t>(id), requestedMajor);
                paragraph = dll + L" was built for an older ROCK and does not work with ROCK " + rockVersion +
                    L". Update it or remove it.";
            } else {
                ROCK_LOG_CRITICAL(Api,
                    "{} asked for interface {} major {} (status {}): it needs a newer ROCK than {}. The game stops at "
                    "the main menu until ROCK is updated.",
                    dllForLog, static_cast<std::uint32_t>(id), requestedMajor, static_cast<std::uint32_t>(status),
                    Version::NAME);
                paragraph = dll + L" needs a newer ROCK than " + rockVersion + L". Update ROCK.";
            }
            required_update::report(std::move(paragraph));
        } catch (...) {}
    }

    void watchForOutdatedConsumers() noexcept
    {
        if (!required_update::watchMainMenu(L"ROCK: update required", &flushLog)) {
            try {
                ROCK_LOG_ERROR(Api, "The UI is unavailable; outdated consumers cannot stop the game at the main menu");
            } catch (...) {}
        }
    }
}
