#pragma once
#include "Input.h"
#include <cstddef>
#include <type_traits>

#pragma pack(push, 8)
namespace rock::api::input::v1_1 {
    inline constexpr InterfaceId kInterfaceId = input::kInterfaceId;
    inline constexpr std::uint32_t kMajor = 1;
    inline constexpr std::uint32_t kMinor = 1;

    enum class PlacementClickFlag : std::uint32_t { InputReserved=2 };
    struct PlacementClickState {
        std::uint32_t size{sizeof(PlacementClickState)};
        std::uint32_t flags{};
        std::uint32_t objectFormId{}; // Submitted candidate; zero while draining a spent click.
        std::uint32_t reserved{};
        SampleV1 sample{};
    };
    // Negotiate Input major 1, minimum minor 1 and sizeof(Api), then bind Read.
    // Call on the ROCK frame thread. A reserved right-stick click belongs to
    // an add-on's held placement action, including release; other users yield.
    // Native keyframing persists independently of this observational query.
    struct Api {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor;
        static constexpr std::uint32_t minorVersion=kMinor;
        input::ApiV1 v1;
        Status(ROCK_CALL* getPlacementClickState)(OwnerToken, PlacementClickState*) noexcept;
    };
    static_assert(std::is_standard_layout_v<Api>);
    static_assert(offsetof(Api, v1)==0);
    static_assert(offsetof(Api, getPlacementClickState)==sizeof(input::ApiV1));
    static_assert(sizeof(Api)==sizeof(input::ApiV1)+sizeof(void*));
}
#pragma pack(pop)
