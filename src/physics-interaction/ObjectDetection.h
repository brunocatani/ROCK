#pragma once

#include "HavokOffsets.h"
#include "PhysicsLog.h"
#include "PhysicsUtils.h"

#include "RE/Bethesda/BSHavok.h"
#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Bethesda/bhkPickData.h"
#include "RE/Havok/hknpAabbQuery.h"
#include "RE/Havok/hknpAllHitsCollector.h"
#include "RE/Havok/hknpBody.h"
#include "RE/Havok/hknpBodyId.h"
#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiCollisionObject.h"

namespace frik::rock
{
    struct SelectedObject
    {
        RE::TESObjectREFR* refr = nullptr;
        RE::hknpBodyId bodyId{ 0x7FFF'FFFF };
        RE::NiAVObject* hitNode = nullptr;
        RE::NiAVObject* visualNode = nullptr;
        float distance = FLT_MAX;
        bool isFarSelection = false;

        void clear()
        {
            refr = nullptr;
            bodyId.value = 0x7FFF'FFFF;
            hitNode = nullptr;
            visualNode = nullptr;
            distance = FLT_MAX;
            isFarSelection = false;
        }

        bool isValid() const { return refr != nullptr; }
    };

    inline void* getQueryFilterRef(RE::hknpWorld* world)
    {
        if (!world)
            return nullptr;
        auto modifierMgr = *reinterpret_cast<std::uintptr_t*>(reinterpret_cast<std::uintptr_t>(world) + offsets::kHknpWorld_ModifierManager);
        if (!modifierMgr)
            return nullptr;
        return *reinterpret_cast<void**>(modifierMgr + offsets::kModifierMgr_FilterPtr);
    }

    bool isGrabbable(RE::TESObjectREFR* ref, RE::TESObjectREFR* otherHandRef = nullptr);

    RE::TESObjectREFR* resolveBodyToRef(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, RE::hknpBodyId bodyId);

    SelectedObject findCloseObject(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const RE::NiPoint3& palmPos, const RE::NiPoint3& palmForward, float nearRange, bool isLeft,
        RE::TESObjectREFR* otherHandRef = nullptr);

    SelectedObject findFarObject(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const RE::NiPoint3& handPos, const RE::NiPoint3& pointingDir, float farRange,
        RE::TESObjectREFR* otherHandRef = nullptr);
}
