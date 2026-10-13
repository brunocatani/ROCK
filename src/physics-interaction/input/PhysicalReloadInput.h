#pragma once
#include <ROCK/Input.h>
#include "physics-interaction/input/InputRemapRuntime.h"
namespace rock::physical_reload_input {
    // Input reload gesture lease (one owner, the equipped weapon's firing hand).
    api::Status set(api::OwnerToken,const api::input::ReloadGestureRequest&);
    api::Status clear(api::OwnerToken);
    api::Status read(api::input::ReloadGestureState&);
    bool gestureActive(bool left);
    bool nativeActivationPriority(bool left);
    bool takeGestureChange();
    bool routeGesture(bool left,bool tap,bool scopeHeld);
    bool fireBlocked(bool left) noexcept;
    void clearOwner(api::OwnerToken);
    void reset();
    void beginFrame(std::uint64_t frame);
    // Tracks the leased hand's physical Accept level; returns true while that
    // hand's normal reload/scope gesture must yield (unavailable, rearming or
    // draining).
    bool observe(bool left,const input_remap_runtime::RawButtonState&);
    // Controller-hook path: only atomic mask access, no provider/engine calls.
    std::uint64_t gameMask(bool left,std::uint64_t rawPressed) noexcept;
}
