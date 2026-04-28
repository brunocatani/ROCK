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
        RE::NiPoint3 hitPointWorld{};
        float distance = FLT_MAX;
        bool isFarSelection = false;
        bool hasHitPoint = false;

        void clear()
        {
            refr = nullptr;
            bodyId.value = 0x7FFF'FFFF;
            hitNode = nullptr;
            visualNode = nullptr;
            hitPointWorld = {};
            distance = FLT_MAX;
            isFarSelection = false;
            hasHitPoint = false;
        }

        bool isValid() const { return refr != nullptr; }
    };

    bool isGrabbable(RE::TESObjectREFR* ref, RE::TESObjectREFR* otherHandRef = nullptr);

    RE::TESObjectREFR* resolveBodyToRef(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, RE::hknpBodyId bodyId);

    SelectedObject findCloseObject(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const RE::NiPoint3& palmPos, const RE::NiPoint3& palmForward, float nearRange, bool isLeft,
        RE::TESObjectREFR* otherHandRef = nullptr);

    SelectedObject findFarObject(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const RE::NiPoint3& handPos, const RE::NiPoint3& pointingDir, float farRange,
        RE::TESObjectREFR* otherHandRef = nullptr);
}
