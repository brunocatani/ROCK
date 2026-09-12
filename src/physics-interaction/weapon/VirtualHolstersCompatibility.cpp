#include "physics-interaction/weapon/VirtualHolstersCompatibility.h"

#include "physics-interaction/PhysicsLog.h"

#include <Windows.h>

namespace rock::virtual_holsters
{
    namespace
    {
        // Original VirtualHolstersAPI.h V1 prefix, in its exact virtual order.
        // No virtual destructor: the provider owns the singleton. The original
        // zone query ignores isLeft; IsLeftHandedMode identifies its one hand.
        class ApiV1
        {
        public:
            virtual std::uint32_t __cdecl GetVersion() const = 0;
            virtual bool __cdecl IsHandInHolsterZone(bool isLeft) const = 0;
            virtual std::uint32_t __cdecl GetCurrentHolster() const = 0;
            virtual bool __cdecl IsHolsterFree(std::uint32_t index) const = 0;
            virtual const char* __cdecl GetHolsteredWeaponName(std::uint32_t index) const = 0;
            virtual bool __cdecl IsWeaponAlreadyHolstered(const char* name) const = 0;
            virtual bool __cdecl GetHolsterPosition(std::uint32_t index, float& x, float& y, float& z) const = 0;
            virtual float __cdecl GetHolsterRadius(std::uint32_t index) const = 0;
            virtual bool __cdecl IsInitialized() const = 0;
            virtual bool __cdecl IsGripAssignedToHolster() const = 0;
            virtual std::uint32_t __cdecl GetHolsterButtonId() const = 0;
            virtual bool __cdecl IsLeftHandedMode() const = 0;
        };

        struct Binding
        {
            bool loaded{ false };
            ApiV1* api{ nullptr };
        };

        const Binding& binding()
        {
            static const Binding instance = [] {
                const auto module = GetModuleHandleW(L"VirtualHolsters.dll");
                if (!module) {
                    return Binding{};
                }
                using GetApi = ApiV1* (__cdecl*)();
                const auto getApi = reinterpret_cast<GetApi>(GetProcAddress(module, "VHAPI_GetApi"));
                ApiV1* api = nullptr;
                try {
                    api = getApi ? getApi() : nullptr;
                    if (api && api->GetVersion() != 1) {
                        api = nullptr;
                    }
                } catch (...) {
                    api = nullptr;
                }
                if (api) {
                    logger::info("ROCK: VirtualHolsters V1 connected; configured-hand weapon release protection available; ROCK equipped-weapon shoulder sheath/retrieval disabled.");
                } else {
                    logger::warn("ROCK: VirtualHolsters.dll detected without a compatible V1 API; weapon release protection unavailable; ROCK equipped-weapon shoulder sheath/retrieval disabled.");
                }
                return Binding{ .loaded = true, .api = api };
            }();
            return instance;
        }
    }

    bool isLoaded()
    {
        return binding().loaded;
    }

    Snapshot readSnapshot() noexcept
    {
        try {
            const auto* api = binding().api;
            if (!api || !api->IsInitialized()) {
                return {};
            }
            Snapshot result{
                .ready = true,
                .isLeft = api->IsLeftHandedMode(),
                .buttonId = api->GetHolsterButtonId(),
            };
            // Use the original's accepted contact, including its occupied-slot
            // rejection and entry/exit margin. Do not invent a second detector.
            if (api->IsHandInHolsterZone(result.isLeft)) {
                result.slot = api->GetCurrentHolster();
                result.inZone = result.slot >= 1 && result.slot <= 7 &&
                    api->IsHolsterFree(result.slot);
            }
            return result;
        } catch (...) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                "VirtualHolsters state query failed; no new holster input claim accepted");
            return {};
        }
    }
}
