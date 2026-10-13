#pragma once
#include "InputV1_3.h"
#pragma pack(push,8)
namespace rock::api::input::v1_4 {
    inline constexpr InterfaceId kInterfaceId=input::kInterfaceId;
    inline constexpr std::uint32_t kMajor=1,kMinor=4;
    // Opt into regular ROCK gameplay priority before tap/hold reload routing.
    // Uses the existing 1.3 request, state readback and clear operation, with
    // the same permissions, owner thread, validation, trigger gate and lease.
    // The native primary wand's workbench/use target (or take target while
    // that hand is free) owns its entire press and release. No reload tap or
    // scope hold is published for that gesture, even if the target changes.
    // Without a native action, short release is a tap and hold is scope input.
    // A secondary firing wand retains ROCK's existing reload/scope mapping;
    // the primary wand's native actions remain independent. UI suppression wins.
    // Switching between 1.3 and 1.4 acquisition cancels any unfinished gesture.
    // Existing setReloadGesture retains its exclusive game-Accept masking.
    struct Api {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor,minorVersion=kMinor;
        v1_3::Api v1_3;
        Status(ROCK_CALL* setGameplayReloadGesture)(OwnerToken,const v1_3::ReloadGestureRequest*) noexcept;
    };
    static_assert(offsetof(Api,v1_3)==0&&offsetof(Api,setGameplayReloadGesture)==sizeof(v1_3::Api));
    static_assert(sizeof(Api)==sizeof(v1_3::Api)+sizeof(void*));
}
#pragma pack(pop)
