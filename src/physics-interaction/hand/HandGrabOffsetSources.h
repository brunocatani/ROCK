#pragma once

#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/saved/SavedGrabOffsetStore.h"

#include <cstdint>

namespace rock::hand_grab_detail
{
    struct LooseWeaponPrimaryAttachFrame
    {
        bool valid = false;
        RE::NiTransform desiredRootWorld{};
        bool sourceVisible = false;
        RE::NiTransform desiredObjectWorld{};
        RE::NiTransform desiredBodyWorld{};
        RE::NiPoint3 gripPointWorld{};
        const char* reason = "notEvaluated";
    };

    enum class GrabOffsetSourceKind : std::uint8_t
    {
        None,
        SavedObject
    };

    struct ResolvedGrabOffsetSource
    {
        bool valid = false;
        GrabOffsetSourceKind kind = GrabOffsetSourceKind::None;
        saved_grab_offset::HandOffset handOffset{};
    };

    struct GrabOffsetAttachSource
    {
        bool valid = false;
        RE::NiTransform desiredRootWorld{};
        const char* reason = "noGrabOffset";
    };

    struct GrabOffsetFingerPoseSource
    {
        bool valid = false;
        grab_finger_pose_runtime::SolvedGrabFingerPose pose{};
        const char* reason = "noGrabOffset";
    };

    [[nodiscard]] const char* grabOffsetSourceReason(GrabOffsetSourceKind kind) noexcept;
    [[nodiscard]] bool tryLoadSavedGrabOffsetHandOffset(
        RE::TESObjectREFR* refr,
        bool isLeft,
        saved_grab_offset::HandOffset& out);
    [[nodiscard]] ResolvedGrabOffsetSource resolveGrabOffsetSource(bool isLeft, RE::TESObjectREFR* refr);
    [[nodiscard]] GrabOffsetAttachSource resolveGrabOffsetAttachSource(
        const RE::NiTransform& proxyWorld,
        bool proxyWorldValid,
        const ResolvedGrabOffsetSource& resolvedOffset);
    [[nodiscard]] GrabOffsetFingerPoseSource resolveGrabOffsetFingerPoseSource(
        const ResolvedGrabOffsetSource& resolvedOffset);
    [[nodiscard]] LooseWeaponPrimaryAttachFrame resolveLooseWeaponPrimaryAttachFrame(
        bool looseWeaponGrab,
        bool grabbedFromPullCatch,
        bool isLeft,
        const SelectedObject& selection,
        const RE::NiAVObject* rootNode,
        const RE::NiTransform& rootBodyLocalAtGrab,
        const RE::NiTransform& objectToBodyAtGrab,
        const RE::NiTransform& grabBodyWorldAtGrab,
        const RE::NiPoint3& grabPivotAWorld,
        const RE::NiTransform& handWorldAtGrab,
        const GrabOffsetAttachSource& grabOffsetSource);
}
