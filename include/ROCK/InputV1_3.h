#pragma once
#include "InputV1_2.h"
#pragma pack(push,8)
namespace rock::api::input::v1_3 {
    inline constexpr InterfaceId kInterfaceId=input::kInterfaceId;
    inline constexpr std::uint32_t kMajor=1,kMinor=3;
    enum class ReloadGestureFlag : std::uint32_t { BlockFire=1 };
    struct ReloadGestureRequest {
        std::uint32_t size{sizeof(ReloadGestureRequest)},weaponFormId{};
        std::uint64_t weaponGenerationKey{};
        SampleV1 sample{};
        Hand hand{Hand::Left};
        std::uint32_t leaseFrames{2},flags{};
    };
    struct ReloadGestureState {
        std::uint32_t size{sizeof(ReloadGestureState)},active{};
        OwnerToken owner{};
        std::uint64_t tapSequence{},expiresAfterFrame{};
        SampleV1 sample{};
        Hand hand{Hand::None};
        std::uint32_t held{},scopeHeld{};
    };
    // Opt-in alternative to Input 1.2's exclusive Accept capture. One shared
    // equipped-weapon lease, exact sample/generations, owner-thread Write for
    // set/clear, Read for get. Only the physical firing hand may acquire it.
    // ROCK classifies the whole gesture using fManualScopeHoldSeconds: a short
    // release increments tapSequence once; a hold stays ROCK scope input and
    // its release never becomes a tap. Disabled immersive scopes still classify
    // taps but never request scope activation. Menus, input loss, hand changes
    // and lease changes cancel a gesture and require release before rearming.
    // BlockFire suppresses only that hand's native trigger button/axis/actions;
    // raw input, grip and scope input stay available. Clearing it requires a
    // physical trigger release before firing resumes. UI suppression still wins.
    // The 1.0/1.1/1.2 prefix and exclusive-capture semantics remain unchanged.
    struct Api {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor,minorVersion=kMinor;
        v1_2::Api v1_2;
        Status(ROCK_CALL* setReloadGesture)(OwnerToken,const ReloadGestureRequest*) noexcept;
        Status(ROCK_CALL* clearReloadGesture)(OwnerToken) noexcept;
        Status(ROCK_CALL* getReloadGesture)(OwnerToken,ReloadGestureState*) noexcept;
    };
    static_assert(offsetof(Api,v1_2)==0&&offsetof(Api,setReloadGesture)==sizeof(v1_2::Api));
    static_assert(sizeof(Api)==sizeof(v1_2::Api)+3*sizeof(void*));
    static_assert(sizeof(ReloadGestureRequest)==64&&sizeof(ReloadGestureState)==80);
}
#pragma pack(pop)
