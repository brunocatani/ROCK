#pragma once

/*
 * Weapon authority helpers are grouped here because lifecycle, visual authority, muzzle authority, and visual composition all decide final weapon ownership.
 */


// ---- WeaponAuthorityLifecyclePolicy.h ----

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <limits>
#include <string_view>

namespace rock::weapon_authority_lifecycle_policy
{
    /*
     * Game-stopping menus and world interruptions must make every ROCK-owned
     * authority yield before returning to the game. ROCK keeps FRIK weapon
     * authority, input blocking, and left-hand collision suppression in separate
     * systems, so the interruption decision is centralized here to keep all
     * three lifetimes in lockstep.
     */
    inline bool shouldClearWeaponAuthorityForUpdateInterruption(bool menuOpen, bool rockDisabled, bool skeletonUnavailable)
    {
        return menuOpen || rockDisabled || skeletonUnavailable;
    }

    inline bool isWeaponContactGenerationCurrent(std::uint64_t contactGenerationKey, std::uint64_t currentGenerationKey)
    {
        /*
         * Zero means the older path could not report a source generation. Treat
         * it as usable so legacy non-generated contacts do not get dropped, but
         * reject explicit mismatches because that is the melee/modded-weapon
         * failure mode: a contact from one generated source tree drives a grip
         * solve on another weapon tree after a swap or root change.
         */
        return contactGenerationKey == 0 || currentGenerationKey == 0 || contactGenerationKey == currentGenerationKey;
    }

}

// ---- WeaponVisualAuthorityMath.h ----

#include "physics-interaction/TransformMath.h"

#include <array>

namespace rock::weapon_visual_authority_math
{
    /*
     * ROCK-owned equipped weapon authority must write the same final visual
     * weapon frame that later drives generated weapon collision bodies. Keeping
     * world-target to parent-local conversion as pure math avoids duplicating
     * node write convention code across two-handed grip, one-handed mesh grip,
     * and debug verification paths. Two-handed support also needs a locked hand
     * frame: controllers guide the weapon solve, while visible hands are
     * recomposed from stored weapon-local frames so the mesh contact point cannot
     * slide along the gun.
     */

    enum class TwoHandedExternalAuthorityStep
    {
        ApplyWeaponVisual,
        PublishHandPose,
        ApplyLockedHandVisual
    };

    enum class LockedHandRole
    {
        Primary,
        Support
    };

    inline constexpr std::array<TwoHandedExternalAuthorityStep, 3> twoHandedExternalAuthorityOrder()
    {
        /*
         * The order is part of the cross-mod contract with FRIK. The weapon must
         * be in its final ROCK-owned frame before hand targets are composed, and
         * the hand pose must be published before FRIK applies/finalizes the
         * external wrist target so right-hand fingers inherit the pivoted wrist.
         */
        return {
            TwoHandedExternalAuthorityStep::ApplyWeaponVisual,
            TwoHandedExternalAuthorityStep::PublishHandPose,
            TwoHandedExternalAuthorityStep::ApplyLockedHandVisual
        };
    }

    inline constexpr int authorityOrderIndex(const TwoHandedExternalAuthorityStep step)
    {
        const auto order = twoHandedExternalAuthorityOrder();
        for (int index = 0; index < static_cast<int>(order.size()); ++index) {
            if (order[static_cast<std::size_t>(index)] == step) {
                return index;
            }
        }
        return -1;
    }

    inline constexpr bool handPosePrecedesLockedHandAuthority()
    {
        return authorityOrderIndex(TwoHandedExternalAuthorityStep::PublishHandPose) <
               authorityOrderIndex(TwoHandedExternalAuthorityStep::ApplyLockedHandVisual);
    }

    inline constexpr bool weaponVisualPrecedesLockedHandAuthority()
    {
        return authorityOrderIndex(TwoHandedExternalAuthorityStep::ApplyWeaponVisual) <
               authorityOrderIndex(TwoHandedExternalAuthorityStep::ApplyLockedHandVisual);
    }

    inline constexpr bool shouldPublishTwoHandedGripPose(const LockedHandRole role)
    {
        /*
         * The primary/right hand already has FRIK's native weapon grip pose. ROCK
         * only needs to move that wrist with the final weapon frame. Publishing a
         * primary mesh pose would replace the user's tuned FRIK grip pose, while
         * the support hand still needs ROCK's mesh/contact pose.
         */
        return role == LockedHandRole::Support;
    }

    inline constexpr bool shouldUseMeshGripFrameRotationAtGrabStart(const LockedHandRole)
    {
        /*
         * Mesh semantics may choose the grip point, but locked hand authority
         * must preserve the wrist rotation already produced by FRIK/INI/native
         * weapon setup at the moment support grip starts. Replacing the right
         * wrist with the mesh-derived grip frame is what causes the visible
         * hand to snap up or down on grab activation.
         */
        return false;
    }

    inline constexpr bool shouldSelectMeshGripPointAtGrabStart(const LockedHandRole role)
    {
        /*
         * The support hand is allowed to select the mesh contact it actually
         * touched. The primary/right hand is not reselected from weapon part
         * names or bounds when support grip starts; the current FRIK-configured
         * hand-to-weapon relationship is already the correct primary grip.
         */
        return role == LockedHandRole::Support;
    }

    template <class Transform>
    inline Transform worldTargetToParentLocal(const Transform& parentWorld, const Transform& targetWorld)
    {
        return transform_math::composeTransforms(transform_math::invertTransform(parentWorld), targetWorld);
    }

    template <class Transform>
    inline Transform weaponLocalFrameToWorld(const Transform& weaponWorld, const Transform& weaponLocalFrame)
    {
        return transform_math::composeTransforms(weaponWorld, weaponLocalFrame);
    }

    template <class Transform>
    [[nodiscard]] inline Transform makePresentationWorldDelta(
        const Transform& oldWeaponWorld,
        const Transform& newWeaponWorld)
    {
        /*
         * Native animation and OMOD controllers have already evaluated the
         * descendant's presentation world. Preserve that exact weapon-relative
         * frame while ROCK moves the weapon root; descendant locals remain
         * native/controller-owned and are deliberately not consulted here.
         */
        return transform_math::composeTransforms(
            newWeaponWorld,
            transform_math::invertTransform(oldWeaponWorld));
    }

    template <class Transform>
    [[nodiscard]] inline Transform applyPresentationWorldDelta(
        const Transform& presentationWorldDelta,
        const Transform& presentationWorld)
    {
        return transform_math::composeTransforms(
            presentationWorldDelta,
            presentationWorld);
    }

}

// ---- NativeScopeRotationMath.h ----

namespace rock::native_scope_rotation_math
{
    /*
     * Scope tuning consistently defines pitch about X, yaw about Z, and roll
     * about Y. The caller decides which calibrated frame owns those axes:
     * Weapon-local for the fallback camera and model-local for overlay tuning.
     * The stored Ni basis composes pitch, then yaw, then roll.
     */
    template <class Transform>
    [[nodiscard]] inline Transform makePitchYawRollLocal(
        const float pitchDegrees,
        const float yawDegrees,
        const float rollDegrees)
    {
        constexpr float kDegreesToRadians = 0.017453292519943295769f;

        const auto makePitchStored = [](const float radians) {
            Transform result = transform_math::makeIdentityTransform<Transform>();
            if (radians == 0.0f) {
                return result;
            }
            const float cosine = std::cos(radians);
            const float sine = std::sin(radians);
            result.rotate.entry[1][1] = cosine;
            result.rotate.entry[1][2] = sine;
            result.rotate.entry[2][1] = -sine;
            result.rotate.entry[2][2] = cosine;
            return result;
        };
        const auto makeYawStored = [](const float radians) {
            Transform result = transform_math::makeIdentityTransform<Transform>();
            if (radians == 0.0f) {
                return result;
            }
            const float cosine = std::cos(radians);
            const float sine = std::sin(radians);
            result.rotate.entry[0][0] = cosine;
            result.rotate.entry[0][1] = sine;
            result.rotate.entry[1][0] = -sine;
            result.rotate.entry[1][1] = cosine;
            return result;
        };
        const auto makeRollStored = [](const float radians) {
            Transform result = transform_math::makeIdentityTransform<Transform>();
            if (radians == 0.0f) {
                return result;
            }
            const float cosine = std::cos(radians);
            const float sine = std::sin(radians);
            result.rotate.entry[0][0] = cosine;
            result.rotate.entry[0][2] = -sine;
            result.rotate.entry[2][0] = sine;
            result.rotate.entry[2][2] = cosine;
            return result;
        };

        const Transform pitch = makePitchStored(pitchDegrees * kDegreesToRadians);
        const Transform yaw = makeYawStored(yawDegrees * kDegreesToRadians);
        const Transform roll = makeRollStored(rollDegrees * kDegreesToRadians);

        Transform result = transform_math::makeIdentityTransform<Transform>();
        result.rotate = transform_math::multiplyStoredRotations(
            transform_math::multiplyStoredRotations(pitch.rotate, yaw.rotate),
            roll.rotate);
        return result;
    }
}

// ---- NativeScopeCameraFollowMath.h ----

namespace rock::native_scope_camera_follow_math
{
    /*
     * Equipped weapon geometry uses +Y as the muzzle/forward axis. The rear
     * plane of the assembled sight is therefore its minimum-Y face; centering
     * the other two axes places FO4VR's native activation camera at the
     * physical ocular end instead of at the firing controller near the cheek.
     */
    template <class Point>
    [[nodiscard]] inline Point rearPlaneCenterFromSightBounds(
        const Point& sightBoundsMinWeaponLocal,
        const Point& sightBoundsMaxWeaponLocal)
    {
        Point result = sightBoundsMinWeaponLocal;
        result.x = (sightBoundsMinWeaponLocal.x + sightBoundsMaxWeaponLocal.x) * 0.5f;
        result.y = sightBoundsMinWeaponLocal.y;
        result.z = (sightBoundsMinWeaponLocal.z + sightBoundsMaxWeaponLocal.z) * 0.5f;
        return result;
    }

    // FRIK's neutral camera basis maps camera X/Y/Z to weapon Y/Z/X.
    // Its vec2Vec steering establishes only forward. Resolve the remaining
    // twist from weapon-local +Z, never from the controller at capture time.
    template <class Rotation>
    [[nodiscard]] inline bool tryBuildWeaponAlignedCameraRotation(
        const Rotation& cameraWeaponLocal, Rotation& result)
    {
        const auto& row = cameraWeaponLocal.entry[0];
        const double length = std::sqrt(static_cast<double>(row[0]) * row[0] +
            static_cast<double>(row[1]) * row[1] + static_cast<double>(row[2]) * row[2]);
        if (!std::isfinite(length) || length <= 0.000001) return false;
        const double x = row[0] / length, y = row[1] / length, z = row[2] / length;
        // forward cross weapon-up. A forward parallel to weapon-up cannot
        // define a roll; reject it rather than capture an arbitrary frame.
        const double rightLength = std::sqrt(x * x + y * y);
        if (rightLength <= 0.000001) return false;
        const double rightX = y / rightLength, rightY = -x / rightLength;
        result = {};
        transform_math::detail::setMatrixEntry(result, 0, 0, x);
        transform_math::detail::setMatrixEntry(result, 0, 1, y);
        transform_math::detail::setMatrixEntry(result, 0, 2, z);
        transform_math::detail::setMatrixEntry(result, 1, 0, rightY * z);
        transform_math::detail::setMatrixEntry(result, 1, 1, -rightX * z);
        transform_math::detail::setMatrixEntry(result, 1, 2, rightX * y - rightY * x);
        transform_math::detail::setMatrixEntry(result, 2, 0, rightX);
        transform_math::detail::setMatrixEntry(result, 2, 1, rightY);
        return true;
    }

    // Signed twist around optical forward; NaN means the frame is unusable.
    // Used by capture and final-presentation diagnostics, not by the solve.
    template <class Rotation>
    [[nodiscard]] inline float weaponLocalCameraRollDegrees(const Rotation& cameraWeaponLocal)
    {
        Rotation aligned{};
        if (!tryBuildWeaponAlignedCameraRotation(cameraWeaponLocal, aligned)) {
            return std::numeric_limits<float>::quiet_NaN();
        }
        double up = 0.0, right = 0.0;
        for (int column = 0; column < 3; ++column) {
            up += static_cast<double>(cameraWeaponLocal.entry[1][column]) * aligned.entry[1][column];
            right += static_cast<double>(cameraWeaponLocal.entry[1][column]) * aligned.entry[2][column];
        }
        if (!std::isfinite(up) || !std::isfinite(right) || std::hypot(up, right) <= 0.000001) {
            return std::numeric_limits<float>::quiet_NaN();
        }
        return static_cast<float>(std::atan2(right, up) * 57.29577951308232);
    }

    // Keep the post-FRIK optical direction and scale, but establish roll in
    // the weapon frame before retaining it across hand-role/menu changes.
    template <class Transform, class Point>
    [[nodiscard]] inline bool tryCaptureRigidAnchorFrameWeaponLocal(
        const Transform& weaponWorld,
        const Transform& nativeScopeCameraWorld,
        const Point& anchorWeaponLocal,
        Transform& result)
    {
        result = {};
        result.scale = 0.0f;
        if (!std::isfinite(weaponWorld.scale) || weaponWorld.scale <= 0.0001f ||
            !std::isfinite(nativeScopeCameraWorld.scale) || nativeScopeCameraWorld.scale <= 0.0001f ||
            !std::isfinite(anchorWeaponLocal.x) || !std::isfinite(anchorWeaponLocal.y) || !std::isfinite(anchorWeaponLocal.z)) return false;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(weaponWorld.rotate.entry[row][column]) ||
                    !std::isfinite(nativeScopeCameraWorld.rotate.entry[row][column])) return false;
            }
        }
        const auto cameraWeaponLocal = transform_math::multiplyStoredRotations(
            nativeScopeCameraWorld.rotate, transform_math::transposeRotation(weaponWorld.rotate));
        Transform captured{};
        if (!tryBuildWeaponAlignedCameraRotation(cameraWeaponLocal, captured.rotate)) return false;
        captured.translate = anchorWeaponLocal;
        captured.scale = nativeScopeCameraWorld.scale / weaponWorld.scale;
        if (!std::isfinite(captured.scale) || captured.scale <= 0.0001f) return false;
        result = captured;
        return true;
    }

    /*
     * Apply a correction around the equipped Weapon axes without rotating the
     * firing-grip anchor itself. Stored Ni axes require base * offset here;
     * composing whole transforms would also orbit the anchor translation.
     */
    template <class Transform>
    [[nodiscard]] inline Transform applyWeaponLocalRotationOffset(
        const Transform& scopeFrameWeaponLocal,
        const float pitchDegrees,
        const float yawDegrees,
        const float rollDegrees)
    {
        const Transform rotationOffset =
            native_scope_rotation_math::makePitchYawRollLocal<Transform>(
                pitchDegrees,
                yawDegrees,
                rollDegrees);
        Transform result = scopeFrameWeaponLocal;
        result.rotate = transform_math::multiplyStoredRotations(
            scopeFrameWeaponLocal.rotate,
            rotationOffset.rotate);
        return result;
    }

    template <class Transform>
    [[nodiscard]] inline Transform resolveRigidAnchorFrameWorld(
        const Transform& weaponWorld,
        const Transform& scopeFrameWeaponLocal)
    {
        return transform_math::composeTransforms(weaponWorld, scopeFrameWeaponLocal);
    }

}

// ---- NativeScopeOverlayFollowMath.h ----

namespace rock::native_scope_overlay_follow_math
{
    // FRIK 0.79 WeaponPositionAdjuster seeds this camera-local basis before
    // aligning it to the carried weapon. ScopeParent retains its native
    // controller frame; calibrating against the already aimed camera bakes
    // the first two-hand/carry rotation into the overlay for the whole equip.
    template <class Transform>
    [[nodiscard]] inline Transform resolveUnsteeredCameraWorld(
        const Transform& cameraParentWorld, const Transform& cameraLocal)
    {
        Transform baseline = cameraLocal;
        baseline.rotate = {};
        baseline.rotate.entry[2][0] = 1.0f;
        baseline.rotate.entry[0][1] = 1.0f;
        baseline.rotate.entry[1][2] = 1.0f;
        return transform_math::composeTransforms(cameraParentWorld, baseline);
    }

    /*
     * FO4VR attaches world_scope.nif beneath ScopeParent, not beneath the
     * native activation camera. Bethesda's camera-to-model translation was
     * authored for a one-hand pose, but its rotation and scale are the native
     * overlay calibration. Capture those components independently so ROCK can
     * replace only the obsolete translation with the generated sight anchor.
     */
    template <class Transform>
    [[nodiscard]] inline Transform captureModelRootCalibrationInCameraLocal(
        const Transform& nativeScopeCameraWorld,
        const Transform& nativeScopeModelRootWorld)
    {
        Transform calibration = transform_math::composeTransforms(
            transform_math::invertTransform(nativeScopeCameraWorld),
            nativeScopeModelRootWorld);
        calibration.translate = {};
        return calibration;
    }

    /*
     * Fine tuning is model-local after Bethesda's native calibration:
     * X is lateral, Y is the optical/depth axis, and Z is vertical for the
     * stock world-scope model. Pitch rotates about X, yaw about Z, and roll
     * about Y. Euler composition is pitch, then yaw, then roll.
     */
    template <class Transform>
    [[nodiscard]] inline Transform makeModelRootFineTuneLocal(
        const float offsetXGameUnits,
        const float offsetYGameUnits,
        const float offsetZGameUnits,
        const float pitchDegrees,
        const float yawDegrees,
        const float rollDegrees)
    {
        Transform fineTune =
            native_scope_rotation_math::makePitchYawRollLocal<Transform>(
                pitchDegrees,
                yawDegrees,
                rollDegrees);
        fineTune.translate.x = offsetXGameUnits;
        fineTune.translate.y = offsetYGameUnits;
        fineTune.translate.z = offsetZGameUnits;
        return fineTune;
    }

    template <class Transform>
    [[nodiscard]] inline Transform resolveScopeModelRootWorld(
        const Transform& correctedScopeCameraWorld,
        const Transform& modelRootCalibrationInCameraLocal,
        const Transform& modelRootFineTuneLocal)
    {
        return transform_math::composeTransforms(
            transform_math::composeTransforms(correctedScopeCameraWorld, modelRootCalibrationInCameraLocal),
            modelRootFineTuneLocal);
    }

    /*
     * Solve ScopeParent backwards from the target model-root world transform.
     * The live root local is retained so stock and loose replacement NIFs are
     * both compensated without hard-coded mesh offsets.
     */
    template <class Transform>
    [[nodiscard]] inline Transform resolveScopeParentWorldForModelRoot(
        const Transform& targetScopeModelRootWorld,
        const Transform& scopeModelRootLocal)
    {
        return transform_math::composeTransforms(
            targetScopeModelRootWorld,
            transform_math::invertTransform(scopeModelRootLocal));
    }
}

// ---- LeftFiringPositionOnlyMath.h ----

namespace rock::left_firing_position_only_math
{
    template <class Transform>
    [[nodiscard]] inline Transform orientationOnly(const Transform& source)
    {
        Transform result = source;
        result.translate = {};
        result.scale = 1.0f;
        return result;
    }

    /*
     * Weapon geometry uses +Y as muzzle-forward and +X as its lateral axis.
     * Conjugating the right weapon-in-wand orientation by the X reflection
     * mirrors the lateral component while retaining a proper rotation. The
     * authored hand relation is deliberately absent: wrist correction is a
     * presentation input and must never become weapon aim authority.
     */
    template <class Transform>
    [[nodiscard]] inline Transform mirrorRightWeaponInWandOrientation(
        const Transform& rightWeaponInWand)
    {
        Transform lateralMirror =
            transform_math::makeIdentityTransform<Transform>();
        lateralMirror.rotate.entry[0][0] = -1.0f;
        return orientationOnly(transform_math::composeTransforms(
            lateralMirror,
            transform_math::composeTransforms(
                orientationOnly(rightWeaponInWand),
                lateralMirror)));
    }

    /*
     * Reapply hFRIK's real hand damping without asking hFRIK to smooth an
     * external hand target. The reference hand-in-carrier orientation is
     * frozen when ROCK takes left-firing authority. On later frames the raw
     * wand extrapolates that reference while the physical hand follows
     * hFRIK's damped offset node. Their world-rotation delta is therefore the
     * exact damping correction already selected by hFRIK. Applying it to the
     * carrier lets the weapon and authored hand consume one shared filter.
     * Translation remains owned by the position-only grip solve below.
     */
    template <class Transform>
    [[nodiscard]] inline Transform resolveDampedAimCarrierWorld(
        const Transform& rawCarrierWorld,
        const Transform& referenceHandInCarrierOrientation,
        const Transform& dampedPhysicalHandWorld)
    {
        const Transform rawHandWorldOrientation =
            transform_math::composeTransforms(
                orientationOnly(rawCarrierWorld),
                orientationOnly(referenceHandInCarrierOrientation));
        const Transform dampingWorldRotation =
            transform_math::composeTransforms(
                orientationOnly(dampedPhysicalHandWorld),
                transform_math::invertTransform(
                    rawHandWorldOrientation));
        const Transform dampedCarrierOrientation =
            transform_math::composeTransforms(
                orientationOnly(dampingWorldRotation),
                orientationOnly(rawCarrierWorld));

        Transform result = rawCarrierWorld;
        result.rotate = dampedCarrierOrientation.rotate;
        return result;
    }

    /*
     * Build the left weapon from the mirrored native orientation, then move
     * only its translation until the same weapon-local firing point reaches
     * the physical left-hand target. Scale remains owned by the live weapon.
     */
    template <class Transform, class Point>
    [[nodiscard]] inline Transform resolveWeaponWorldPositionOnly(
        const Transform& leftWandWorld,
        const Transform& leftWeaponInWandOrientation,
        const Transform& liveWeaponWorld,
        const Point& firingGripWeaponLocal,
        const Point& physicalGripTargetWorld)
    {
        Transform solvedWeaponWorld = transform_math::composeTransforms(
            leftWandWorld,
            orientationOnly(leftWeaponInWandOrientation));
        solvedWeaponWorld.translate = liveWeaponWorld.translate;
        solvedWeaponWorld.scale = liveWeaponWorld.scale;

        const Point currentGripWorld = transform_math::localPointToWorld(
            solvedWeaponWorld,
            firingGripWeaponLocal);
        solvedWeaponWorld.translate.x +=
            physicalGripTargetWorld.x - currentGripWorld.x;
        solvedWeaponWorld.translate.y +=
            physicalGripTargetWorld.y - currentGripWorld.y;
        solvedWeaponWorld.translate.z +=
            physicalGripTargetWorld.z - currentGripWorld.z;
        return solvedWeaponWorld;
    }

    /*
     * Support release on a left-fired weapon. The two-hand solve aimed the
     * weapon at the support hand, so the wand aim can sit tens of degrees
     * away when that hand lets go. The right carry eases the difference out
     * through its parent-local weapon return; the left carry has no native
     * parent, so it eases in the physical firing-hand frame: the last
     * rendered two-hand pose rides the hand while it blends into the
     * wand-aimed position-only pose. Both locals share the firing hand, so
     * the grip never leaves the controller during the blend.
     */
    template <class Transform>
    [[nodiscard]] inline Transform weaponWorldToPhysicalHandLocal(
        const Transform& physicalHandWorld,
        const Transform& weaponWorld)
    {
        return transform_math::composeTransforms(
            transform_math::invertTransform(physicalHandWorld),
            weaponWorld);
    }

    template <class Transform>
    [[nodiscard]] inline Transform physicalHandLocalToWeaponWorld(
        const Transform& physicalHandWorld,
        const Transform& weaponHandLocal,
        const float liveWeaponScale)
    {
        Transform weaponWorld = transform_math::composeTransforms(
            physicalHandWorld,
            weaponHandLocal);
        weaponWorld.scale = liveWeaponScale;
        return weaponWorld;
    }
}

// ---- ScopeSafeHandFrameMath.h ----

namespace rock::scope_safe_hand_frame_math
{
    /*
     * hFRIK intentionally collapses its body root while FO4VR's native
     * ScopeMenu is open. hFRIK's damped arm-driver offset nodes remain live,
     * so remember each authored hand frame relative to that same driver before
     * the collapse and reconstruct it from the current driver while scoped.
     */
    template <class Transform>
    [[nodiscard]] inline Transform captureDriverToHandLocal(
        const Transform& driverWorld,
        const Transform& handWorld)
    {
        return transform_math::composeTransforms(
            transform_math::invertTransform(driverWorld),
            handWorld);
    }

    template <class Transform>
    [[nodiscard]] inline Transform resolveHandWorld(
        const Transform& driverWorld,
        const Transform& driverToHandLocal)
    {
        return transform_math::composeTransforms(driverWorld, driverToHandLocal);
    }

    enum class ResolutionMode
    {
        RootFlattened,
        DriverReconstructed,
        LastKnown,
        Unavailable,
    };

    /*
     * ScopeMenu visibility is presentation state, not weapon-solver
     * ownership. While the scope button remains requested, retain the driver
     * basis across a transient ScopeMenu close so a presentation pulse cannot
     * create a root/driver oscillation. A real button release is different:
     * hFRIK has restored its visible root, and a still-active manual grip must
     * not keep a stale hidden-scope driver authoritative for the exit frame.
     */
    [[nodiscard]] inline constexpr bool retainDriverFrameAuthority(
        bool scopeMenuOpen,
        bool manualScopeActivationRequested,
        bool manualOwnershipActive,
        bool driverFrameAuthorityWasActive)
    {
        return scopeMenuOpen ||
               (manualScopeActivationRequested && manualOwnershipActive && driverFrameAuthorityWasActive);
    }

    /*
     * Driver-frame authority prefers the cached driver relation over the
     * visible root while the ScopeMenu is open, so a scoped sample cannot
     * replace a good driver-to-hand calibration. A hand that holds no
     * relation has no calibration to protect, and the only capture that
     * creates one runs in the root branch this refusal closes, so a
     * provider holding the vanilla ScopeMenu open after an interruption
     * cleared the cache pins that hand unavailable for the whole hold.
     * Seed such a hand from the isolated controller hand, and only while
     * that frame is a calibrated, uncontaminated source: never from ROCK's
     * own rendered output. Refreshes still wait for the menu to close.
     */
    [[nodiscard]] inline constexpr bool canSampleScopeRootHand(
        bool driverFrameAuthorityActive,
        bool hasDriverToHandLocal,
        bool rawHandFrameCalibrated)
    {
        return !driverFrameAuthorityActive ||
               (!hasDriverToHandLocal && rawHandFrameCalibrated);
    }

    /*
     * The animation graph restores the first-person arms to their unplaced
     * player-relative pose every frame and re-places them within it, on the
     * order of 1e5 game units from the controller. A calibrated raw hand
     * frame does not bound that: the isolation relation is measured once
     * and kept, so a later unplaced sample still reads as calibrated. A
     * scoped seed is the one sample that becomes a cached relation and is
     * never re-taken while the menu is held, so an implausible one latches
     * for the whole hold instead of failing for a frame. Bound it against
     * the driver it will be expressed relative to: a placed hand sits about
     * 9-11 units from its controller.
     */
    inline constexpr float kMaxScopeSeedHandToDriverGameUnits = 50.0f;

    [[nodiscard]] inline constexpr bool isPlausibleScopeSeedHand(float handToDriverGameUnits)
    {
        return handToDriverGameUnits >= 0.0f &&
               handToDriverGameUnits <= kMaxScopeSeedHandToDriverGameUnits;
    }

    [[nodiscard]] inline constexpr bool shouldStartRootRebase(
        bool manualScopeActivationRequested,
        bool driverFrameAuthorityStoppedThisFrame,
        bool reconstructedHandValid,
        bool recentScopedHandAvailable)
    {
        // The scoped frame is hidden, so it is useful continuity only while
        // the user still requests the scope. On a real button release the
        // restored visible root is authoritative immediately; blending from
        // the hidden frame can stretch both arms and the weapon through space.
        return manualScopeActivationRequested && driverFrameAuthorityStoppedThisFrame &&
               (reconstructedHandValid || recentScopedHandAvailable);
    }

    [[nodiscard]] inline constexpr ResolutionMode resolveMode(
        bool scopeMenuOpen,
        bool rootHandValid,
        bool reconstructedHandValid,
        bool hasLastHandWorld,
        std::uint32_t consecutiveDriverMissFrames,
        std::uint32_t maxDriverMissGraceFrames)
    {
        if (!scopeMenuOpen) {
            return rootHandValid ? ResolutionMode::RootFlattened : ResolutionMode::Unavailable;
        }
        if (reconstructedHandValid) {
            return ResolutionMode::DriverReconstructed;
        }
        /*
         * Scoped callers only offer a root sample for a hand with no driver
         * relation to reconstruct from, so there is no calibration here for
         * it to replace. Take it ahead of the frozen last-known pose: the
         * root branch is the only place the relation is captured, and
         * deferring would freeze the hand for the grace window and land
         * back here every fourth frame.
         */
        if (rootHandValid) {
            return ResolutionMode::RootFlattened;
        }
        return hasLastHandWorld && consecutiveDriverMissFrames < maxDriverMissGraceFrames ?
                   ResolutionMode::LastKnown :
                   ResolutionMode::Unavailable;
    }

    [[nodiscard]] inline constexpr ResolutionMode resolveCollisionIsolatedMode(
        bool collisionPresentationWasLive,
        bool scopeDriverFrameAuthorityActive,
        bool rootHandValid,
        bool reconstructedHandValid,
        bool hasLastHandWorld,
        std::uint32_t consecutiveDriverMissFrames,
        std::uint32_t maxDriverMissGraceFrames)
    {
        /*
         * ROCK runs after FRIK's skeleton pass. If the previous render interval
         * retained collision hand authority, the current root already contains
         * that presentation result even after its tag is cleared. Reconstruct
         * from the unaffected physical driver or fail closed; root and history
         * would both feed collision output back into the next drive target.
         */
        if (collisionPresentationWasLive) {
            return reconstructedHandValid ?
                       ResolutionMode::DriverReconstructed :
                       ResolutionMode::Unavailable;
        }
        return resolveMode(
            scopeDriverFrameAuthorityActive,
            rootHandValid,
            reconstructedHandValid,
            hasLastHandWorld,
            consecutiveDriverMissFrames,
            maxDriverMissGraceFrames);
    }

    [[nodiscard]] inline constexpr bool shouldPublishLockedHandVisualAuthority([[maybe_unused]] bool scopeMenuOpen)
    {
        // Since FRIK API v2.3 FRIK culls the body geometry while scoped instead
        // of collapsing the root, so the arm IK keeps solving and hand
        // authority keeps publishing through the scope edge.
        return true;
    }

    enum class HandAuthorityRole : std::uint8_t
    {
        PrimaryGrip = 1u << 0u,
        SupportGrip = 1u << 1u,
        PrimaryDetach = 1u << 2u,
    };

    using HandAuthorityRoleMask = std::uint8_t;

    [[nodiscard]] inline constexpr HandAuthorityRoleMask roleMask(HandAuthorityRole role)
    {
        return static_cast<HandAuthorityRoleMask>(role);
    }

    [[nodiscard]] inline constexpr bool hasRole(HandAuthorityRoleMask mask, HandAuthorityRole role)
    {
        return (mask & roleMask(role)) != 0;
    }

    struct DesiredHandAuthorityInput
    {
        bool gripping{ false };
        bool primaryHandAuthorityEnabled{ false };
        bool firingHandIsLeft{ false };
        bool leftPartGripActive{ false };
        bool rightPartGripActive{ false };
    };

    /*
     * ScopeMenu hides the arms but does not end ROCK ownership. On exit, the
     * live role set is derived from the weapon state instead of treating every
     * hFRIK tag as stale. PrimaryOnly relies on native/mirrored carry and owns
     * no wrist tag; Gripping owns the firing wrist only in full-authority mode;
     * every active part grip owns its physical support wrist in either firing
     * topology or PartCarry.
     */
    [[nodiscard]] inline constexpr HandAuthorityRoleMask desiredRolesForHand(
        const DesiredHandAuthorityInput& input,
        bool isLeft)
    {
        HandAuthorityRoleMask desired = 0;
        if (input.gripping && input.primaryHandAuthorityEnabled && input.firingHandIsLeft == isLeft) {
            desired |= roleMask(HandAuthorityRole::PrimaryGrip);
        }
        if (isLeft ? input.leftPartGripActive : input.rightPartGripActive) {
            desired |= roleMask(HandAuthorityRole::SupportGrip);
        }
        return desired;
    }

    enum class DeferredClearAction
    {
        RetainLiveRole,
        ClearStaleRole,
        WaitForReplacementPublication,
    };

    /*
     * A stale tag may be removed only after a replacement role for the same
     * hand has published this frame. That ordering prevents hFRIK from briefly
     * restoring the tracked arm between two ROCK roles. Reacquiring the same
     * role simply cancels its deferred clear and keeps the existing tag alive.
     */
    [[nodiscard]] inline constexpr DeferredClearAction resolveDeferredClearAction(
        HandAuthorityRole pendingRole,
        HandAuthorityRoleMask desiredRoles,
        HandAuthorityRoleMask publishedRolesThisFrame)
    {
        if (hasRole(desiredRoles, pendingRole)) {
            return DeferredClearAction::RetainLiveRole;
        }
        if (desiredRoles == 0 || (desiredRoles & publishedRolesThisFrame) != 0) {
            return DeferredClearAction::ClearStaleRole;
        }
        return DeferredClearAction::WaitForReplacementPublication;
    }

    /*
     * ScopeMenu and the short exit rebase use reconstructed hand frames whose
     * update ownership differs from the visible weapon node. Neither is a
     * valid source for replacing the native, visible right-hand calibration.
     */
    [[nodiscard]] inline constexpr bool canRefreshRightFiringCanonicalFrame(
        bool scopeMenuOpen,
        bool rightHandRootRebaseActive)
    {
        return !scopeMenuOpen && !rightHandRootRebaseActive;
    }

    /*
     * A support grip acquired while ScopeMenu is open must not recapture the
     * firing hold from hFRIK's hidden/reconstructed hand. Reuse only the exact
     * pre-scope right-hand grip from the same generated weapon.
     */
    [[nodiscard]] inline constexpr bool shouldReuseRightFiringCanonicalGrip(
        bool scopeMenuOpen,
        bool firingHandIsLeft,
        bool canonicalValid,
        std::uint64_t canonicalWeaponGenerationKey,
        std::uint64_t currentWeaponGenerationKey)
    {
        return scopeMenuOpen &&
               !firingHandIsLeft &&
               canonicalValid &&
               canonicalWeaponGenerationKey != 0 &&
               canonicalWeaponGenerationKey == currentWeaponGenerationKey;
    }

    [[nodiscard]] inline float rebaseAlpha(float elapsedSeconds, float durationSeconds)
    {
        if (!std::isfinite(durationSeconds) || durationSeconds <= 0.0f) {
            return 1.0f;
        }
        const float elapsed = std::isfinite(elapsedSeconds) ? elapsedSeconds : durationSeconds;
        return std::clamp(elapsed / durationSeconds, 0.0f, 1.0f);
    }

    template <class Transform>
    [[nodiscard]] inline Transform interpolateRebaseTransform(
        const Transform& from,
        const Transform& to,
        float alpha)
    {
        const float t = std::clamp(std::isfinite(alpha) ? alpha : 1.0f, 0.0f, 1.0f);
        Transform result = to;
        result.translate.x = from.translate.x + (to.translate.x - from.translate.x) * t;
        result.translate.y = from.translate.y + (to.translate.y - from.translate.y) * t;
        result.translate.z = from.translate.z + (to.translate.z - from.translate.z) * t;
        result.scale = from.scale + (to.scale - from.scale) * t;

        float fromQuaternion[4]{};
        float toQuaternion[4]{};
        transform_math::niRowsToHavokQuaternion(from.rotate, fromQuaternion);
        transform_math::niRowsToHavokQuaternion(to.rotate, toQuaternion);
        const float dot = fromQuaternion[0] * toQuaternion[0] +
                          fromQuaternion[1] * toQuaternion[1] +
                          fromQuaternion[2] * toQuaternion[2] +
                          fromQuaternion[3] * toQuaternion[3];
        if (dot < 0.0f) {
            for (float& component : toQuaternion) {
                component = -component;
            }
        }

        float blendedQuaternion[4]{};
        float lengthSquared = 0.0f;
        for (std::size_t index = 0; index < 4; ++index) {
            blendedQuaternion[index] = fromQuaternion[index] + (toQuaternion[index] - fromQuaternion[index]) * t;
            lengthSquared += blendedQuaternion[index] * blendedQuaternion[index];
        }
        if (std::isfinite(lengthSquared) && lengthSquared > 0.000001f) {
            const float inverseLength = 1.0f / std::sqrt(lengthSquared);
            for (float& component : blendedQuaternion) {
                component *= inverseLength;
            }
            result.rotate = transform_math::havokQuaternionToNiRows<decltype(result.rotate)>(blendedQuaternion);
        }
        return result;
    }
}

// ---- WeaponMuzzleAuthorityMath.h ----

namespace rock::weapon_muzzle_authority_math
{
    /*
     * ROCK writes the final equipped weapon transform after FRIK's normal weapon
     * update has already run. FRIK's proven muzzle fix copies the projectile node
     * world transform into the fire node so flashes/projectiles originate at the
     * current barrel tip. Keeping the transform rule here gives ROCK the same
     * final-owner behavior without depending on FRIK's weapon adjuster pass.
     */
    template <class Transform>
    [[nodiscard]] Transform fireNodeLocalFromProjectileWorld(const Transform& projectileWorld)
    {
        return projectileWorld;
    }
}

// ---- WeaponVisualCompositionPolicy.h ----

#include <cstdint>
#include <string_view>

namespace rock::weapon_visual_composition_policy
{
    /*
     * Weapon mod swaps can expose the weapon node before the edited part's
     * renderer data has settled. ROCK therefore separates runtime visual
     * composition from transforms: geometry/visibility identity participates in
     * rebuild decisions, while ordinary animation and weapon motion do not.
     */
    inline constexpr std::uint64_t kWeaponVisualCompositionOffset = 1469598103934665603ull;
    inline constexpr std::uint64_t kWeaponVisualCompositionPrime = 1099511628211ull;

    struct VisualRecord
    {
        std::uintptr_t nodeAddress{ 0 };
        std::uintptr_t parentAddress{ 0 };
        std::string_view name{};
        std::uint32_t depth{ 0 };
        std::uint32_t childIndex{ 0 };
        std::uint32_t childCount{ 0 };
        bool visible{ false };
        bool triShape{ false };
        std::uintptr_t rendererData{ 0 };
        std::uintptr_t skinInstance{ 0 };
        std::uintptr_t vertexBlock{ 0 };
        std::uintptr_t triangleBlock{ 0 };
        std::uint64_t vertexDesc{ 0 };
        std::uint32_t numTriangles{ 0 };
        std::uint32_t numVertices{ 0 };
        std::uint32_t geometryType{ 0 };
    };

    inline void mixValue(std::uint64_t& key, std::uint64_t value)
    {
        key ^= value;
        key *= kWeaponVisualCompositionPrime;
    }

    inline void mixString(std::uint64_t& key, std::string_view value)
    {
        for (const char ch : value) {
            mixValue(key, static_cast<unsigned char>(ch));
        }
        mixValue(key, value.size());
    }

    inline void mixVisualRecord(std::uint64_t& key, const VisualRecord& record)
    {
        mixValue(key, record.nodeAddress);
        mixValue(key, record.parentAddress);
        mixString(key, record.name);
        mixValue(key, record.depth);
        mixValue(key, record.childIndex);
        mixValue(key, record.childCount);
        mixValue(key, record.visible ? 1ULL : 0ULL);
        mixValue(key, record.triShape ? 1ULL : 0ULL);
        mixValue(key, record.rendererData);
        mixValue(key, record.skinInstance);
        mixValue(key, record.vertexBlock);
        mixValue(key, record.triangleBlock);
        mixValue(key, record.vertexDesc);
        mixValue(key, record.numTriangles);
        mixValue(key, record.numVertices);
        mixValue(key, record.geometryType);
    }

}

// ---- WeaponGeneratedSourceCompletenessPolicy.h ----

#include "physics-interaction/weapon/WeaponTypes.h"

#include <cstdint>

namespace rock::weapon_generated_source_completeness_policy
{
    /*
     * FO4VR weapon attachments can become extractable after the first visible
     * weapon root has already stabilized. A spawned modded firearm may therefore
     * produce a partial generated source set, then gain a stock or attachment
     * only after an unrelated mod swap forces a rebuild. ROCK tracks source-set
     * completeness separately from visual identity so firearms can self-enrich
     * when late parts appear while existing bodies remain live until a better
     * replacement set is ready.
     */
    struct GeneratedSourceCompleteness
    {
        std::uint64_t signature{ 0 };
        std::uint64_t geometryHash{ 0 };
        std::uint64_t boundsExtentScore{ 0 };
        std::size_t sourceCount{ 0 };
        std::size_t pointCount{ 0 };
        std::size_t childClusterCount{ 0 };
        std::uint32_t semanticPartMask{ 0 };
        std::uint32_t gameplayCriticalCount{ 0 };
        std::size_t durableSourceCount{ 0 };
        std::size_t durableChildClusterCount{ 0 };
        std::size_t durablePointCount{ 0 };
        std::uint64_t durableBoundsExtentScore{ 0 };
        std::uint64_t durableGeometryHash{ 0 };
        std::size_t transientReloadSourceCount{ 0 };
        std::uint32_t missingRequiredPackageCoverageMask{ 0 };
        bool firearmLikePackage{ false };
        bool hasRequiredFrontCoverage{ false };
        bool hasRequiredRearCoverage{ false };
    };

    inline std::uint32_t partMask(WeaponPartKind kind)
    {
        const auto index = static_cast<std::uint32_t>(kind);
        if (index >= 31) {
            return 0;
        }
        return 1u << index;
    }

    inline bool hasPart(const GeneratedSourceCompleteness& completeness, WeaponPartKind kind)
    {
        return (completeness.semanticPartMask & partMask(kind)) != 0;
    }

    inline constexpr std::uint32_t kMissingFrontPackageCoverage = 1u << 0;
    inline constexpr std::uint32_t kMissingRearPackageCoverage = 1u << 1;

    inline bool hasFrontPackageCoverage(const GeneratedSourceCompleteness& completeness)
    {
        return hasPart(completeness, WeaponPartKind::Barrel) ||
               hasPart(completeness, WeaponPartKind::MuzzleDevice) ||
               hasPart(completeness, WeaponPartKind::Handguard) ||
               hasPart(completeness, WeaponPartKind::Foregrip) ||
               hasPart(completeness, WeaponPartKind::Pump);
    }

    inline bool hasCompactRearPackageCoverage(const GeneratedSourceCompleteness& completeness)
    {
        return hasPart(completeness, WeaponPartKind::Stock) ||
               hasPart(completeness, WeaponPartKind::Grip);
    }

    inline bool hasLongGunRearPackageCoverage(const GeneratedSourceCompleteness& completeness)
    {
        return hasPart(completeness, WeaponPartKind::Stock);
    }

    inline bool hasFirearmActionPackageEvidence(const GeneratedSourceCompleteness& completeness)
    {
        return hasPart(completeness, WeaponPartKind::Magazine) ||
               hasPart(completeness, WeaponPartKind::Magwell) ||
               hasPart(completeness, WeaponPartKind::Bolt) ||
               hasPart(completeness, WeaponPartKind::Slide) ||
               hasPart(completeness, WeaponPartKind::ChargingHandle) ||
               hasPart(completeness, WeaponPartKind::BreakAction) ||
               hasPart(completeness, WeaponPartKind::Cylinder) ||
               hasPart(completeness, WeaponPartKind::Chamber) ||
               hasPart(completeness, WeaponPartKind::LaserCell) ||
               hasPart(completeness, WeaponPartKind::Lever);
    }

    inline bool isFirearmLikeSourcePackage(const GeneratedSourceCompleteness& completeness)
    {
        /*
         * This is retained as telemetry for logs and debug visualizers only.
         * CommonLib/WEAP classification can be unavailable or Unknown while the
         * visible source package clearly has receiver/action/magazine evidence,
         * but the result no longer gates collision publication.
         */
        if (completeness.sourceCount == 0 || !hasPart(completeness, WeaponPartKind::Receiver)) {
            return false;
        }

        const bool actionOrSocketEvidence = hasFirearmActionPackageEvidence(completeness);
        const bool authoredLongGunCoverage = hasFrontPackageCoverage(completeness) && hasCompactRearPackageCoverage(completeness);
        const bool sightedWeaponPackage =
            (hasPart(completeness, WeaponPartKind::Sight) || hasPart(completeness, WeaponPartKind::Scope)) &&
            (actionOrSocketEvidence || authoredLongGunCoverage);
        return actionOrSocketEvidence || sightedWeaponPackage || authoredLongGunCoverage;
    }

    inline bool hasLongGunPackageEvidence(const GeneratedSourceCompleteness& completeness)
    {
        /*
         * Retained as package telemetry. It used to drive front/rear coherence
         * blocking, but modded weapons can expose valid visible parts without
         * those names. Collision generation now consumes the visible mesh set.
         */
        return hasPart(completeness, WeaponPartKind::Stock) ||
               hasPart(completeness, WeaponPartKind::Handguard) ||
               hasPart(completeness, WeaponPartKind::Foregrip) ||
               hasPart(completeness, WeaponPartKind::Pump) ||
               hasPart(completeness, WeaponPartKind::Bolt) ||
               hasPart(completeness, WeaponPartKind::ChargingHandle) ||
               hasPart(completeness, WeaponPartKind::Lever);
    }

    inline bool hasRequiredRearPackageCoverage(const GeneratedSourceCompleteness& completeness)
    {
        return hasLongGunPackageEvidence(completeness) ?
            hasLongGunRearPackageCoverage(completeness) :
            hasCompactRearPackageCoverage(completeness);
    }

    inline std::uint32_t missingRequiredPackageCoverageMask(const GeneratedSourceCompleteness& completeness)
    {
        /*
         * Collision generation is visibility-driven: if a mesh is present,
         * buildable, and survives geometry dedupe, it gets collision. The old
         * front/rear firearm package gate guessed that a stable source package
         * was incomplete when names like stock/barrel were absent; modded FO4VR
         * weapons routinely violate those lexical assumptions, so the gate
         * blocked exactly the last-applied parts we were trying to recover.
         * Keep the package evidence fields as telemetry only.
         */
        (void)completeness;
        return 0;
    }

    inline GeneratedSourceCompleteness withDerivedPackageCoverage(GeneratedSourceCompleteness completeness)
    {
        completeness.firearmLikePackage = isFirearmLikeSourcePackage(completeness);
        completeness.hasRequiredFrontCoverage = hasFrontPackageCoverage(completeness);
        completeness.hasRequiredRearCoverage = hasRequiredRearPackageCoverage(completeness);
        completeness.missingRequiredPackageCoverageMask = missingRequiredPackageCoverageMask(completeness);
        return completeness;
    }

    inline std::uint32_t permanentGameplayCriticalPartMask()
    {
        /*
         * Reload ammo displays are intentionally excluded from this permanence
         * mask. They can appear/disappear during action animation, while stock,
         * receiver, barrel/support, magazine/socket, and action components are
         * persistent weapon structure for collision and support-grip routing.
         */
        return partMask(WeaponPartKind::Receiver) |
               partMask(WeaponPartKind::Barrel) |
               partMask(WeaponPartKind::MuzzleDevice) |
               partMask(WeaponPartKind::Bipod) |
               partMask(WeaponPartKind::Handguard) |
               partMask(WeaponPartKind::Foregrip) |
               partMask(WeaponPartKind::Pump) |
               partMask(WeaponPartKind::Stock) |
               partMask(WeaponPartKind::Grip) |
               partMask(WeaponPartKind::Magazine) |
               partMask(WeaponPartKind::Magwell) |
               partMask(WeaponPartKind::Bolt) |
               partMask(WeaponPartKind::Slide) |
               partMask(WeaponPartKind::ChargingHandle) |
               partMask(WeaponPartKind::BreakAction) |
               partMask(WeaponPartKind::Cylinder) |
               partMask(WeaponPartKind::Chamber) |
               partMask(WeaponPartKind::LaserCell) |
               partMask(WeaponPartKind::Lever);
    }

    inline bool isTransientReloadPart(WeaponPartKind kind)
    {
        return kind == WeaponPartKind::Shell ||
               kind == WeaponPartKind::Round ||
               kind == WeaponPartKind::CosmeticAmmo;
    }

    inline std::uint64_t makeGeneratedWeaponBodySetKey(
        std::uint64_t equippedWeaponKey,
        const GeneratedSourceCompleteness& sourceCompleteness,
        std::uint64_t bodySetEpoch)
    {
        if (equippedWeaponKey == 0 || sourceCompleteness.signature == 0 || bodySetEpoch == 0) {
            return 0;
        }

        std::uint64_t key = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
        weapon_visual_composition_policy::mixString(key, "ROCKGeneratedWeaponBodySetV1");
        weapon_visual_composition_policy::mixValue(key, equippedWeaponKey);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.signature);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.geometryHash);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.boundsExtentScore);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.sourceCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.pointCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.childClusterCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.semanticPartMask);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.gameplayCriticalCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.durableSourceCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.durableChildClusterCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.durablePointCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.durableBoundsExtentScore);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.durableGeometryHash);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.transientReloadSourceCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.missingRequiredPackageCoverageMask);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.firearmLikePackage ? 1ULL : 0ULL);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.hasRequiredFrontCoverage ? 1ULL : 0ULL);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.hasRequiredRearCoverage ? 1ULL : 0ULL);
        weapon_visual_composition_policy::mixValue(key, bodySetEpoch);
        return key;
    }
}

// ---- WeaponGenerationIdentityPolicy.h ----

namespace rock::weapon_generation_identity_policy
{
    /*
     * Generated collision rebuild authority comes from stable equipped weapon
     * content, not from reload-time visual tree churn or transient runtime
     * wrapper addresses. Visual composition is still collected when a rebuild is
     * already allowed, but it must not make a stable equipped weapon look like a
     * new collider lifecycle.
     */
    struct EquippedWeaponGenerationIdentity
    {
        std::uint32_t formID{ 0 };
        std::uintptr_t formAddress{ 0 };
        std::uintptr_t instanceDataAddress{ 0 };
        std::uintptr_t instanceKeywordDataAddress{ 0 };
        std::uint64_t instanceContentKey{ 0 };
        std::uintptr_t objectInstanceExtraAddress{ 0 };
        std::uint64_t objectIndexDataSignature{ 0 };
        std::uint32_t objectIndexDataCount{ 0 };
        std::uint32_t activeModCount{ 0 };
        std::uint32_t disabledModCount{ 0 };
        // BGSEquipType behavior identity. The effective value is read through
        // TESObjectWEAP's equip-slot virtual using the currently equipped
        // instance data; baseEquipSlotFormID remains diagnostic provenance.
        std::uint32_t effectiveEquipSlotFormID{ 0 };
        std::uint32_t baseEquipSlotFormID{ 0 };
        std::uintptr_t equippedDataAddress{ 0 };
        std::uintptr_t equippedObjectAddress{ 0 };
        std::string_view displayName{};
        float weightGame{ 0.0f };
        bool hasEquippedWeapon{ false };
        bool effectiveEquipSlotUsesInstanceData{ false };
        WeaponSizeClass sizeClass{ WeaponSizeClass::Rifle };
        WeaponClassificationSource classificationSource{ WeaponClassificationSource::None };
        std::uint64_t keywordFlags{ 0 };
        bool usedEffectiveInstanceKeywordData{ false };
        bool classificationResolved{ false };
    };

    inline std::uint64_t makeEquippedWeaponIdentityKey(const EquippedWeaponGenerationIdentity& identity)
    {
        if (!identity.hasEquippedWeapon) {
            return 0;
        }

        std::uint64_t key = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
        weapon_visual_composition_policy::mixString(key, "ROCKWeaponGenerationIdentityV2");
        weapon_visual_composition_policy::mixValue(key, identity.hasEquippedWeapon ? 1ULL : 0ULL);
        weapon_visual_composition_policy::mixValue(key, identity.formID);
        weapon_visual_composition_policy::mixValue(key, identity.instanceContentKey);
        weapon_visual_composition_policy::mixValue(key, identity.objectIndexDataSignature);
        weapon_visual_composition_policy::mixValue(key, identity.objectIndexDataCount);
        weapon_visual_composition_policy::mixValue(key, identity.activeModCount);
        weapon_visual_composition_policy::mixValue(key, identity.disabledModCount);
        weapon_visual_composition_policy::mixString(key, identity.displayName);
        return key;
    }

    /*
     * Manual ownership is instance-bound, unlike generated collision, which is
     * intentionally content-bound. Use the strongest available witness and do
     * not mix weaker equip-wrapper addresses into it: FO4VR can rebuild those
     * wrappers while the same instanceData remains equipped. The ordered
     * fallbacks preserve fail-closed ownership for weapons without instanceData.
     * Witnesses are opaque main-thread values; retained addresses are never
     * dereferenced.
     */
    inline std::uint64_t makeEquippedWeaponOwnershipKey(const EquippedWeaponGenerationIdentity& identity)
    {
        if (!identity.hasEquippedWeapon) {
            return 0;
        }

        std::uint64_t witnessKind = 0;
        std::uintptr_t witnessAddress = 0;
        if (identity.instanceDataAddress != 0) {
            witnessKind = 1;
            witnessAddress = identity.instanceDataAddress;
        } else if (identity.objectInstanceExtraAddress != 0) {
            witnessKind = 2;
            witnessAddress = identity.objectInstanceExtraAddress;
        } else if (identity.equippedObjectAddress != 0) {
            witnessKind = 3;
            witnessAddress = identity.equippedObjectAddress;
        } else if (identity.equippedDataAddress != 0) {
            witnessKind = 4;
            witnessAddress = identity.equippedDataAddress;
        }
        if (witnessAddress == 0) {
            return 0;
        }

        std::uint64_t key = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
        weapon_visual_composition_policy::mixString(key, "ROCKEquippedWeaponOwnershipV2");
        weapon_visual_composition_policy::mixValue(key, identity.formID);
        weapon_visual_composition_policy::mixValue(key, witnessKind);
        weapon_visual_composition_policy::mixValue(key, witnessAddress);
        return key;
    }

}
