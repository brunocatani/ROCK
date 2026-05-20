#pragma once

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/actor/ActorEquipmentGrab.h"
#include "physics-interaction/hand/HandSelection.h"
#include "physics-interaction/object/GrabTargetKind.h"

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

namespace rock
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
        float signedAlongDistance = FLT_MAX;
        float lateralDistance = FLT_MAX;
        float hitFraction = 1.0f;
        float selectionScore = FLT_MAX;
        std::uint32_t hitShapeKey = 0xFFFF'FFFF;
        std::uint32_t hitShapeCollisionFilterInfo = 0;
        float hmdConeDot = -1.0f;
        grab_target::Kind targetKind = grab_target::Kind::None;
        bool isFarSelection = false;
        bool hasHitPoint = false;
        bool hasHitNormal = false;
        bool hasHitShapeKey = false;
        bool hasSelectionScore = false;
        bool hasHmdConeDot = false;
        actor_equipment_grab::ActorEquipmentSelection actorEquipment{};

        void clear()
        {
            refr = nullptr;
            bodyId.value = 0x7FFF'FFFF;
            hitNode = nullptr;
            visualNode = nullptr;
            hitPointWorld = {};
            hitNormalWorld = {};
            distance = FLT_MAX;
            signedAlongDistance = FLT_MAX;
            lateralDistance = FLT_MAX;
            hitFraction = 1.0f;
            selectionScore = FLT_MAX;
            hitShapeKey = 0xFFFF'FFFF;
            hitShapeCollisionFilterInfo = 0;
            hmdConeDot = -1.0f;
            targetKind = grab_target::Kind::None;
            isFarSelection = false;
            hasHitPoint = false;
            hasHitNormal = false;
            hasHitShapeKey = false;
            hasSelectionScore = false;
            hasHmdConeDot = false;
            actorEquipment = {};
        }

        bool isValid() const { return refr != nullptr; }
    };

    inline bool resolveFarSelectionHmdConeAnchor(RE::hknpWorld* hknpWorld, const SelectedObject& selection, RE::NiPoint3& outAnchor)
    {
        /*
         * Far HMD-cone revalidation happens after the original query frame, so
         * prefer a live object anchor over the selection-time hit point. The
         * stored hit remains the last fallback for selections whose body/node
         * authority is temporarily unavailable.
         */
        if (!selection.isValid() || !selection.isFarSelection) {
            return false;
        }

        if (selection.targetKind == grab_target::Kind::ActorEquipment) {
            if (selection.actorEquipment.visualNode) {
                outAnchor = selection.actorEquipment.visualNode->world.translate;
                return true;
            }
            if (selection.actorEquipment.hitNode) {
                outAnchor = selection.actorEquipment.hitNode->world.translate;
                return true;
            }
            if (selection.actorEquipment.hasHitPoint) {
                outAnchor = selection.actorEquipment.hitPointWorld;
                return true;
            }
        }

        if (hknpWorld && selection.bodyId.value != 0x7FFF'FFFF) {
            RE::NiTransform bodyWorld{};
            if (tryResolveLiveBodyWorldTransform(hknpWorld, selection.bodyId, bodyWorld)) {
                outAnchor = bodyWorld.translate;
                return true;
            }
            if (auto* ownerNode = getOwnerNodeFromBody(hknpWorld, selection.bodyId)) {
                outAnchor = ownerNode->world.translate;
                return true;
            }
        }

        if (selection.visualNode) {
            outAnchor = selection.visualNode->world.translate;
            return true;
        }
        if (selection.hitNode) {
            outAnchor = selection.hitNode->world.translate;
            return true;
        }
        if (selection.hasHitPoint) {
            outAnchor = selection.hitPointWorld;
            return true;
        }

        return false;
    }

    inline bool selectedObjectPassesFarHmdCone(
        RE::hknpWorld* hknpWorld,
        const SelectedObject& selection,
        const FarSelectionHmdConeGate& hmdConeGate,
        float* outDot = nullptr)
    {
        if (!selection.isValid() || !selection.isFarSelection || !hmdConeGate.enabled) {
            if (outDot) {
                *outDot = selection.hasHmdConeDot ? selection.hmdConeDot : 1.0f;
            }
            return true;
        }

        RE::NiPoint3 anchorWorld{};
        if (!resolveFarSelectionHmdConeAnchor(hknpWorld, selection, anchorWorld)) {
            if (outDot) {
                *outDot = -1.0f;
            }
            return false;
        }

        return hmdConeGate.acceptsHitPoint(anchorWorld, outDot);
    }

    inline bool selectedObjectPassesFarHmdCone(const SelectedObject& selection, const FarSelectionHmdConeGate& hmdConeGate, float* outDot = nullptr)
    {
        return selectedObjectPassesFarHmdCone(nullptr, selection, hmdConeGate, outDot);
    }

    struct GrabTargetClassification
    {
        grab_target::Kind kind{ grab_target::Kind::None };
        const char* reason{ "none" };
        bool grabbable{ false };
        actor_equipment_grab::ActorEquipmentSelection actorEquipment{};
    };

    bool isGrabbable(RE::TESObjectREFR* ref, const OtherHandSelectionContext& otherHandContext = {});
    GrabTargetClassification classifySelectionGrabTarget(
        RE::TESObjectREFR* ref,
        RE::hknpWorld* hknpWorld,
        RE::hknpBodyId bodyId,
        const OtherHandSelectionContext& otherHandContext = {},
        bool isFarSelection = false,
        RE::NiAVObject* hitNode = nullptr,
        const RE::NiPoint3& hitPointWorld = {},
        bool hasHitPoint = false);

    RE::TESObjectREFR* resolveBodyToRef(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, RE::hknpBodyId bodyId);

    SelectedObject findCloseObject(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const RE::NiPoint3& palmPos, const RE::NiPoint3& palmForward, float nearRange, bool isLeft,
        const OtherHandSelectionContext& otherHandContext = {});

    SelectedObject findFarObject(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const RE::NiPoint3& handPos, const RE::NiPoint3& pointingDir, float farRange,
        const FarSelectionHmdConeGate& hmdConeGate,
        const OtherHandSelectionContext& otherHandContext = {});
}
