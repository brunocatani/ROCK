#pragma once
#include "RE/NetImmerse/NiTransform.h"
#include <cstdint>
#include <span>
namespace RE { class TESObjectREFR; class hknpWorld; }
namespace rock::native_object_placement {
    // Game-thread action, with ROCK physics callbacks paused. No pointers or
    // body IDs survive the call. Reference pose and Havok state use native saves.
    bool available();
    enum class ScriptPreparation { Ready, Pending, Rejected };
    ScriptPreparation prepareLoadScript(RE::TESObjectREFR* ref, bool start);
    bool anchor(RE::TESObjectREFR* ref, RE::hknpWorld* world,
        const RE::NiTransform& pose, std::span<const std::uint32_t> bodyIds);
}
