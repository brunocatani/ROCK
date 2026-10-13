#pragma once
#include "Touch.h"

#pragma pack(push, 8)
namespace rock::api::touch::v1_1 {
    inline constexpr std::uint32_t kMajor=1, kMinor=1;
    enum class AuthoredFlag : std::uint32_t {
        Indicator=1, ReserveGrab=2, Attached=4,
    };
    struct AuthoredHand {
        std::uint32_t size{sizeof(AuthoredHand)}, version{1};
        Sample sample{};
        Hand hand{Hand::None};
        std::uint32_t flags{}, referenceFormId{}, leaseFrames{2};
        Point3 contactWorldGame{};
        std::uint32_t reserved{};
    };
    // Opt in using Touch 1.1 / sizeof(Api), Write permission. Calls use ROCK's
    // frame owner thread, including BeforeRock. One lease per physical hand;
    // a competing owner is rejected. Refresh every frame (lease 1..3 frames).
    // Indicator joins the existing authored weapon marker presentation.
    // ReserveGrab consumes new ROCK grabs before static/climbing acquisition;
    // it does not revoke an existing object or climbing hold. Attached requires
    // ReserveGrab and suppresses ONLY the owned dynamic hand's simulation pairs
    // against world-surface layers. Other hand, actors, tools and queries remain.
    // Clear, expiry, owner loss and generation/lifecycle loss remove authority.
    // Caller owns reach/occupancy checks, grip pose and mechanism movement.
    // The original 1.0 prefix, record layouts and all existing calls are intact.
    struct Api {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor, minorVersion=kMinor;
        ApiV1 v1;
        Status(ROCK_CALL* setAuthoredHand)(OwnerToken,const AuthoredHand*) noexcept;
        Status(ROCK_CALL* clearAuthoredHand)(OwnerToken,Hand) noexcept;
    };
    static_assert(std::is_standard_layout_v<Api> && offsetof(Api,v1)==0);
    static_assert(offsetof(Api,setAuthoredHand)==sizeof(ApiV1));
    static_assert(sizeof(Api)==sizeof(ApiV1)+2*sizeof(void*));
}
#pragma pack(pop)
