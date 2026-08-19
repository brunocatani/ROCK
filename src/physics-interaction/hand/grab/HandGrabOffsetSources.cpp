#include "physics-interaction/hand/grab/HandGrabOffsetSources.h"

/*
 * rock::hand_grab_detail offset sources: decides WHERE a grab attaches when the
 * answer does not come from live mesh evidence. Covers saved per-object offsets,
 * loose-weapon primary attach frames, and the FRIK weapon offset cache.
 *
 * Resolution is ordered and fails closed. An unreadable or non-finite stored
 * offset is skipped, never applied in part.
 */

#include "physics-interaction/hand/grab/HandGrabMath.h"
#include "physics-interaction/weapon/LooseWeaponGripZone.h"

#include <cmath>
#include <cstdint>

namespace rock::hand_grab_detail
{
    namespace
    {
        bool isFiniteNiTransform(const RE::NiTransform& value)
        {
            for (std::uint32_t row = 0; row < 3; ++row) {
                for (std::uint32_t column = 0; column < 3; ++column) {
                    if (!std::isfinite(value.rotate.entry[row][column])) {
                        return false;
                    }
                }
            }
            return std::isfinite(value.translate.x) &&
                   std::isfinite(value.translate.y) &&
                   std::isfinite(value.translate.z) &&
                   std::isfinite(value.scale) &&
                   value.scale > 0.0001f;
        }
    
        const RE::TESObjectWEAP* selectedLooseWeaponForm(const SelectedObject& selection)
        {
            auto* selectedBase = selection.refr ? selection.refr->GetObjectReference() : nullptr;
            return selectedBase ? selectedBase->As<RE::TESObjectWEAP>() : nullptr;
        }
    
        bool isThrowableLooseWeapon(const RE::TESObjectWEAP* weapon)
        {
            return weapon &&
                   (weapon->weaponData.type == RE::WEAPON_TYPE::kGrenade ||
                       weapon->weaponData.type == RE::WEAPON_TYPE::kMine);
        }
    }
    
    [[nodiscard]] const char* grabOffsetSourceReason(GrabOffsetSourceKind kind) noexcept
    {
        switch (kind) {
        case GrabOffsetSourceKind::SavedObject:
            return "savedGrabOffset";
        case GrabOffsetSourceKind::None:
        default:
            return "noGrabOffset";
        }
    }
    
    // Cache-only disk lookup; false when this object+hand has no saved
    // offset (normal - most objects never had one saved).
    bool tryLoadSavedGrabOffsetHandOffset(RE::TESObjectREFR* refr, bool isLeft, saved_grab_offset::HandOffset& out)
    {
        if (!refr) {
            return false;
        }
        auto* baseForm = refr->GetObjectReference();
        if (!baseForm) {
            return false;
        }
        const auto formRef = saved_grab_offset::formRefFromRuntimeId(baseForm->GetFormID());
        if (formRef.empty()) {
            return false;
        }
        saved_grab_offset::SavedGrabOffsetFile file{};
        if (!saved_grab_offset::load(formRef, file, nullptr)) {
            return false;
        }
        const auto& handOffset = isLeft ? file.left : file.right;
        if (!handOffset.present) {
            return false;
        }
        out = handOffset;
        return true;
    }
    
    ResolvedGrabOffsetSource resolveGrabOffsetSource(bool isLeft, RE::TESObjectREFR* refr)
    {
        /*
         * Per-object SavedGrabOffsets are the only explicit hand-placement
         * authority. The built-in calibrated grenade/Molotov presets were
         * retired (2026-07-13) so throwables run the generic seat machinery
         * (center seed, seat depth stop, forced-arrival long-axis
         * alignment, swept finger poses); re-add hardcoded presets from
         * fresh captures if that flow returns.
         */
        ResolvedGrabOffsetSource source{};
        if (!refr) {
            return source;
        }
    
        if (tryLoadSavedGrabOffsetHandOffset(refr, isLeft, source.handOffset)) {
            source.valid = true;
            source.kind = GrabOffsetSourceKind::SavedObject;
        }
        return source;
    }
    
    /*
     * proxyWorld/proxyWorldValid are resolved by the caller (a live
     * GrabAuthorityProxy read, Hand::tryComputeGrabProxyLocalPalmPocketFrameWorld)
     * since this file's helpers are free functions with no Hand access.
     */
    GrabOffsetAttachSource resolveGrabOffsetAttachSource(
        const RE::NiTransform& proxyWorld,
        bool proxyWorldValid,
        const ResolvedGrabOffsetSource& resolvedOffset)
    {
        GrabOffsetAttachSource source{};
        if (!proxyWorldValid || !resolvedOffset.valid) {
            return source;
        }
    
        const auto& handOffset = resolvedOffset.handOffset;
        RE::NiTransform objectProxyLocal = transform_math::makeIdentityTransform<RE::NiTransform>();
        objectProxyLocal.translate = { handOffset.translateGame[0], handOffset.translateGame[1], handOffset.translateGame[2] };
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                objectProxyLocal.rotate.entry[row][column] = handOffset.rotate[row * 3 + column];
            }
        }
    
        source.desiredRootWorld = grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorld, objectProxyLocal);
        source.valid = isFiniteNiTransform(source.desiredRootWorld);
        source.reason = grabOffsetSourceReason(resolvedOffset.kind);
        return source;
    }
    
    /*
     * A resolved offset's finger pose is only meaningful for the loose-
     * weapon synthetic primary attach (pull-catch/force-grab): that path
     * has no mesh contact to solve fingers from, so it otherwise falls
     * back to a generic canned named pose (see
     * publishLooseWeaponPrimaryAttachHandPose / grabSelectedObject).
     */
    GrabOffsetFingerPoseSource resolveGrabOffsetFingerPoseSource(const ResolvedGrabOffsetSource& resolvedOffset)
    {
        GrabOffsetFingerPoseSource source{};
        if (!resolvedOffset.valid || !resolvedOffset.handOffset.hasFingerPose) {
            return source;
        }
    
        const auto& handOffset = resolvedOffset.handOffset;
        source.pose.solved = true;
        source.pose.values = { handOffset.fingerValues[0], handOffset.fingerValues[1], handOffset.fingerValues[2],
            handOffset.fingerValues[3], handOffset.fingerValues[4] };
        source.pose.hasJointValues = handOffset.hasFingerJointValues;
        if (handOffset.hasFingerJointValues) {
            for (std::size_t i = 0; i < source.pose.jointValues.size(); ++i) {
                source.pose.jointValues[i] = handOffset.fingerJointValues[i];
            }
        }
        source.valid = true;
        source.reason = grabOffsetSourceReason(resolvedOffset.kind);
        return source;
    }
    
    LooseWeaponPrimaryAttachFrame resolveLooseWeaponPrimaryAttachFrame(
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
        const GrabOffsetAttachSource& grabOffsetSource)
    {
        LooseWeaponPrimaryAttachFrame frame{};
        /*
         * A close grab is a free mesh hold on either hand regardless of
         * source; the firing-grip transition happens later through the
         * grip-zone equip path (loose_weapon_grip_zone), not by forcing
         * the attach at grab.
         */
        if (!grabbedFromPullCatch && !selection.forcedArrival) {
            frame.reason = "closeGrabFreeHold";
            return frame;
        }
    
        if (grabOffsetSource.valid) {
            /*
             * The unified resolver has already selected either the
             * flag-gated calibrated grenade preset or the per-object
             * saved offset. Either is explicit hand-placement authority,
             * so it overrides both the generic FRIK weapon offset and the
             * throwable live-pose default.
             */
            frame.desiredRootWorld = grabOffsetSource.desiredRootWorld;
            frame.sourceVisible = false;
            frame.reason = grabOffsetSource.reason;
        } else {
            if (!looseWeaponGrab) {
                frame.reason = "notLooseWeapon";
                return frame;
            }
            /*
             * Only non-throwable programmatic loose-weapon arrivals snap to a
             * canonical attach pose. Grenades, mines, and Molotov variants are
             * hand-thrown objects: force-grab and pull-catch commits keep the
             * normal mesh/body relation so the object is translated into the
             * pocket without forcing a root rotation from FRIK or the live hand.
             */
            const auto* looseWeapon = selectedLooseWeaponForm(selection);
            const bool throwableArrival = isThrowableLooseWeapon(looseWeapon) && (grabbedFromPullCatch || selection.forcedArrival);
            if (throwableArrival) {
                frame.reason = selection.forcedArrival ? "throwableForcedArrivalPreservePose" : "throwablePullCatchPreservePose";
                return frame;
            }
            if (!rootNode || !isFiniteNiTransform(rootNode->world)) {
                frame.reason = "missingWeaponRoot";
                return frame;
            }
    
            /*
             * Both firing hands use the same weapon-relative authority resolver.
             * It enforces custom hFRIK > learned authored > embedded hFRIK and
             * performs no filesystem work on this grab path. Weapon world is the
             * live hand world composed with the inverse of that canonical hold.
             */
            RE::NiTransform handWorld{};
            RE::NiTransform handWeaponLocal{};
            const char* holdReason = "canonicalHoldUnavailable";
            const bool haveDesiredRoot = loose_weapon_grip_zone::tryResolveLooseWeaponFiringHandHold(
                isLeft, selection.refr, handWorld, handWeaponLocal, &holdReason);
            if (haveDesiredRoot) {
                frame.desiredRootWorld = multiplyTransforms(
                    handWorld, transform_math::invertTransform(handWeaponLocal));
                frame.sourceVisible = false;
                frame.reason = holdReason;
            } else if (!selection.forcedArrival) {
                frame.reason = holdReason;
                return frame;
            }
    
            if (!haveDesiredRoot) {
                /*
                 * Palm-anchored fallback for non-throwable forced arrivals
                 * without a usable FRIK offset: root axes follow the live hand
                 * basis and the root origin sits on the hand grab pivot. Any
                 * fixed choice is correct here -- the goal is a deterministic
                 * commit pose, not a per-weapon tuned grip.
                 */
                if (!isFiniteNiTransform(handWorldAtGrab)) {
                    frame.reason = "nonFiniteHandWorld";
                    return frame;
                }
                frame.desiredRootWorld.rotate = handWorldAtGrab.rotate;
                frame.desiredRootWorld.translate = grabPivotAWorld;
                frame.sourceVisible = false;
                frame.reason = "forcedArrivalPalmPose";
            }
        }
    
        frame.desiredRootWorld.scale =
            rootNode && std::isfinite(rootNode->world.scale) && rootNode->world.scale > 0.0001f ? rootNode->world.scale : 1.0f;
        if (!isFiniteNiTransform(frame.desiredRootWorld)) {
            frame.reason = "nonFiniteDesiredRoot";
            return frame;
        }
    
        frame.desiredBodyWorld = multiplyTransforms(frame.desiredRootWorld, rootBodyLocalAtGrab);
        frame.desiredObjectWorld = deriveNodeWorldFromBodyWorld(frame.desiredBodyWorld, objectToBodyAtGrab);
        const RE::NiPoint3 desiredPivotBodyLocal = transform_math::worldPointToLocal(frame.desiredBodyWorld, grabPivotAWorld);
        frame.gripPointWorld = transform_math::localPointToWorld(grabBodyWorldAtGrab, desiredPivotBodyLocal);
        if (!isFiniteNiTransform(frame.desiredBodyWorld) ||
            !isFiniteNiTransform(frame.desiredObjectWorld) ||
            !std::isfinite(frame.gripPointWorld.x) ||
            !std::isfinite(frame.gripPointWorld.y) ||
            !std::isfinite(frame.gripPointWorld.z)) {
            frame.reason = "nonFiniteDesiredBody";
            return frame;
        }
    
        frame.valid = true;
        return frame;
    }

}
