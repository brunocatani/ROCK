#pragma once
#include <ROCK/InputV1_2.h>
#include "physics-interaction/input/InputRemapRuntime.h"
namespace rock::physical_reload_input {
    api::Status set(api::OwnerToken,const api::input::v1_2::ReloadInputRequest&);
    api::Status clear(api::OwnerToken);
    api::Status read(api::input::v1_2::ReloadInputState&);
    void clearOwner(api::OwnerToken);
    void reset();
    void beginFrame(std::uint64_t frame);
    // Consumes the selected physical edge for the external owner; returns true
    // while that hand's normal reload/scope gesture must yield.
    bool observe(bool left,const input_remap_runtime::RawButtonState&);
    // Controller-hook path: only atomic mask access, no provider/engine calls.
    std::uint64_t gameMask(bool left,std::uint64_t rawPressed) noexcept;
}
