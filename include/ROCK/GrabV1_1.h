#pragma once
#include "Grab.h"
#include <cstddef>
#include <type_traits>

#pragma pack(push, 8)
namespace rock::api::grab::v1_1 {
    inline constexpr InterfaceId kInterfaceId=grab::kInterfaceId;
    inline constexpr std::uint32_t kMajor=1, kMinor=1;
    enum class HeldPlacementFlag : std::uint32_t { LooseObject=1, FixedSurfaceContact=2 };
    struct HeldPlacementHand { std::uint32_t formId{}, flags{}; };
    enum class PlacementResult : std::uint32_t { None, Pending, Anchored, Rejected };
    struct HeldPlacementState {
        std::uint32_t size{sizeof(HeldPlacementState)}, buttonAvailable{};
        std::uint64_t frameToken{};
        SampleV1 sample{};
        HeldPlacementHand right{}, left{};
        std::uint32_t buttonHeld{}, buttonPressed{}, buttonAgeMilliseconds{}, reserved{};
        std::uint64_t lastRequestToken{};
        PlacementResult lastResult{};
        std::uint32_t lastRequestFormId{};
    };
    enum class PlacementIntentFlag : std::uint32_t { ReserveClick=1, Anchor=2 };
    struct HeldPlacementIntent {
        std::uint32_t size{sizeof(HeldPlacementIntent)}, flags{};
        std::uint64_t frameToken{};
        SampleV1 sample{};
        std::uint32_t formId{}, reserved{};
    };
    // Negotiate major 1/minor 1/sizeof(Api); Read for get, Write for submit/clear.
    // Call on ROCK's frame thread, preferably Core's BeforeRock phase. Read
    // the state and submit its unchanged token/sample in that same phase.
    // Input is the physical right Axis0 click (32), sampled without consuming it.
    // Intent expires on the next ROCK update; one owner per update, Busy on a
    // competing owner. formId is the add-on's currently eligible held reference,
    // or zero. ReserveClick covers the entire press/release gesture. Anchor
    // requires ReserveClick and a nonzero formId. RequestQueued is acceptance,
    // not completion; lastRequestToken/lastResult report the native operation.
    // ROCK validates both hands, reference/world lifetime and the entire body
    // set, then releases constraints without throwing and saves a native fixed
    // placement. Static parts remain static. No engine pointers or body IDs
    // cross this interface. Pending work cancels on lost intent, owner or world.
    // Clearing/unregistering never thaws a completed placement. Normal grab
    // and pull retain their existing semantics. No F4SE decoration co-save.
    struct Api {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor, minorVersion=kMinor;
        grab::ApiV1 v1;
        Status(ROCK_CALL* getHeldPlacementState)(OwnerToken,HeldPlacementState*) noexcept;
        Status(ROCK_CALL* submitHeldPlacementIntent)(OwnerToken,const HeldPlacementIntent*) noexcept;
        Status(ROCK_CALL* clearHeldPlacementIntent)(OwnerToken) noexcept;
    };
    static_assert(std::is_standard_layout_v<Api>);
    static_assert(offsetof(Api,v1)==0);
    static_assert(offsetof(Api,getHeldPlacementState)==sizeof(grab::ApiV1));
    static_assert(sizeof(Api)==sizeof(grab::ApiV1)+3*sizeof(void*));
    static_assert(sizeof(HeldPlacementState)==96 && sizeof(HeldPlacementIntent)==56);
}
#pragma pack(pop)
