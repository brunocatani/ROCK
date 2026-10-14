#pragma once

#include <ROCK/Grab.h>
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/NetImmerse/NiTransform.h"

namespace rock::reference_interaction
{
    // Game/animation owner thread only. Borrowed nodes never survive a call.
    RE::TESObjectREFR* resolveBody(RE::hknpWorld* world, std::uint32_t bodyId);
    RE::TESObjectREFR* resolveNode(RE::NiAVObject* node);
    RE::TESObjectREFR* resolveQuery(const api::grab::ReferenceQuery& query);
    bool describe(RE::TESObjectREFR* ref, api::grab::ReferenceInteraction& out, std::int32_t markerIndex = 0);
    bool isPowerArmorFurniture(RE::TESObjectREFR* ref, bool* outAvailable = nullptr);
    bool describePowerArmor(RE::TESObjectREFR* ref, api::grab::PowerArmorTarget& out, std::int32_t markerIndex = 0);
    const char* pointName(api::PowerArmorPoint point) noexcept;
    bool pointTransform(RE::TESObjectREFR* frame, api::PowerArmorPoint point,
        RE::NiTransform& outWorld, RE::NiTransform* outLocal = nullptr);
    RE::NiAVObject* findNamedNode(RE::NiAVObject* root, const char* name);
}
