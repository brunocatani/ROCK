#include "experimental/LooseReloadBridge.h"
#include "physics-interaction/weapon/CarriedWeaponRuntime.h"

namespace
{
    // Publications and queries belong to ROCK's animation/interaction thread.
    // A foreign-thread query sees an empty value, never another thread's state.
    thread_local loose_reload_experiment::Snapshot snapshot;
}
namespace rock::loose_reload_bridge
{
    void publish(const loose_reload_experiment::Snapshot& value) noexcept { snapshot = value; }
    void clear() noexcept { snapshot = {}; }
}
extern "C" __declspec(dllexport) bool ROCK_LooseReloadExperiment_QueryV1(
    std::uint64_t frame, loose_reload_experiment::Snapshot* output) noexcept
{
    if (!output || output->size != sizeof(*output) || output->version != 1) return false;
    *output = {};
    if (!rock::CarriedWeaponRuntime::isInteractionThread() ||
        !loose_reload_experiment::current(snapshot, frame)) return false;
    *output = snapshot;
    return true;
}
