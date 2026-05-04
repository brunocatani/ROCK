#pragma once

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsUtils.h"

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

#include <cstdint>

namespace frik::rock
{
    struct SelectedObject
    {
        RE::TESObjectREFR* refr = nullptr;
        RE::hknpBodyId bodyId{ 0x7FFF'FFFF };
        RE::NiAVObject* hitNode = nullptr;
        RE::NiAVObject* visualNode = nullptr;
        RE::NiPoint3 hitPointWorld{};
        RE::NiPoint3 hitNormalWorld{};
        float distance = FLT_MAX;
        float hitFraction = 1.0f;
        std::uint32_t hitShapeKey = 0xFFFF'FFFF;
        std::uint32_t hitShapeCollisionFilterInfo = 0;
        bool isFarSelection = false;
        bool hasHitPoint = false;
        bool hasHitNormal = false;
        bool hasHitShapeKey = false;

        void clear()
        {
            refr = nullptr;
            bodyId.value = 0x7FFF'FFFF;
            hitNode = nullptr;
            visualNode = nullptr;
            hitPointWorld = {};
            hitNormalWorld = {};
            distance = FLT_MAX;
            hitFraction = 1.0f;
            hitShapeKey = 0xFFFF'FFFF;
            hitShapeCollisionFilterInfo = 0;
            isFarSelection = false;
            hasHitPoint = false;
            hasHitNormal = false;
            hasHitShapeKey = false;
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
