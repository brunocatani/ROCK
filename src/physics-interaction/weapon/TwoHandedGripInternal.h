#pragma once

#include "physics-interaction/weapon/TwoHandedGrip.h"

#include "api/ROCKProviderApiInternal.h"
#include "physics-interaction/actor/ActorEquipmentGrab.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/DynamicWeaponCollisionPolicy.h"
#include "physics-interaction/weapon/EquippedWeaponHandlingRuntime.h"
#include "physics-interaction/weapon/NativeScopeSightAnchorPolicy.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponCollision.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

/*
 * Shared internals of the TwoHandedGrip implementation, split across the
 * grip/, scope/, and telemetry/ translation units. These helpers lived in
 * the anonymous namespace of the former single TwoHandedGrip.cpp; the
 * using-directive below preserves unqualified name lookup for the moved
 * method bodies. Include only from TwoHandedGrip implementation files.
 */

namespace rock
{
    namespace two_handed_grip_internal
    {
        inline constexpr const char* PRIMARY_GRIP_TAG = "ROCK_WeaponPrimaryGrip";
        inline constexpr const char* AUTHORED_PRIMARY_POSE_BLOCK_TAG = "ROCK_AuthoredPrimaryPose";
        inline constexpr const char* PRIMARY_DETACH_TAG = "ROCK_WeaponPrimaryDetach";
        inline constexpr const char* SUPPORT_GRIP_TAG = "ROCK_WeaponSupportGrip";
        inline constexpr const char* RETURN_HAND_TAG = "ROCK_WeaponReturn";
        inline constexpr const char* WEAPON_COLLISION_HAND_TAG =
            "ROCK_WeaponCollisionHand";
        inline constexpr const char* WEAPON_NODE_OWNERSHIP_TAG = "ROCK_LeftFiringCarry";
        inline constexpr const char* WEAPON_RECOIL_CONTROLLER_TAG = "ROCK_FiringGripRecoil";
        inline constexpr int GRIP_HAND_POSE_PRIORITY = 100;
        inline constexpr int WEAPON_COLLISION_HAND_PRIORITY = 110;
        inline constexpr int RETURN_HAND_VISUAL_PRIORITY = 85;
        inline constexpr float SUPPORT_NORMAL_TWIST_FACTOR = 0.5f;
        inline constexpr float DYNAMIC_SUPPORT_MINIMUM_SMOOTHED_ROTATION_RADIANS =
            0.5f * 0.01745329251994329577f;
        inline constexpr float RADIANS_TO_DEGREES = 57.295779513082320876f;
        /*
         * Clock domain: a consecutive-publication count by design. The hFRIK
         * driver either publishes each frame or it does not; the tolerance is
         * a number of missed publications, not an elapsed duration.
         */
        inline constexpr std::uint32_t SCOPE_DRIVER_MISS_GRACE_FRAMES = 3;
        inline constexpr float SCOPE_ROOT_REBASE_DURATION_SECONDS = 0.075f;
        inline constexpr std::uint32_t SCOPE_TRANSITION_TRACE_FRAMES = 6;
        inline constexpr float GRIP_FAILURE_DETAILED_LOG_COOLDOWN_SECONDS = 2.0f;
        inline constexpr float WEAPON_OPPOSITION_MAX_FINGER_GAP_GAME_UNITS =
            24.0f;
        inline constexpr float WEAPON_OPPOSITION_SEGMENT_PROBE_RADIUS_GAME_UNITS =
            0.5f;
        inline constexpr std::size_t WEAPON_PRESENTATION_MAX_DEPTH = 128;
        inline constexpr std::size_t WEAPON_PRESENTATION_MAX_OBJECTS = 8192;

        inline const char* twoHandedStateDiagnosticName(
            const TwoHandedState state)
        {
            switch (state) {
            case TwoHandedState::Inactive:
                return "inactive";
            case TwoHandedState::Touching:
                return "touching";
            case TwoHandedState::Gripping:
                return "gripping";
            case TwoHandedState::PartCarry:
                return "part-carry";
            case TwoHandedState::PrimaryOnly:
                return "primary-only";
            }
            return "unknown";
        }

        inline const char* supportAuthorityDiagnosticName(
            const weapon_support_authority_policy::
                WeaponSupportAuthorityMode mode)
        {
            switch (mode) {
            case weapon_support_authority_policy::
                WeaponSupportAuthorityMode::FullTwoHandedSolver:
                return "full";
            case weapon_support_authority_policy::
                WeaponSupportAuthorityMode::VisualOnlySupport:
                return "visual-only";
            }
            return "unknown";
        }

        inline const char* weaponInteractionAcquisitionSourceName(
            const WeaponInteractionAcquisitionSource source)
        {
            switch (source) {
            case WeaponInteractionAcquisitionSource::PhysicalContact:
                return "contact";
            case WeaponInteractionAcquisitionSource::ProximityProbe:
                return "probe";
            case WeaponInteractionAcquisitionSource::AuthoredSeat:
                return "authored-seat";
            case WeaponInteractionAcquisitionSource::None:
                return "none";
            }
            return "unknown";
        }

        inline const char* handFrameResolutionDiagnosticName(
            const scope_safe_hand_frame_math::ResolutionMode mode)
        {
            switch (mode) {
            case scope_safe_hand_frame_math::ResolutionMode::RootFlattened:
                return "root";
            case scope_safe_hand_frame_math::ResolutionMode::
                DriverReconstructed:
                return "driver";
            case scope_safe_hand_frame_math::ResolutionMode::LastKnown:
                return "last-known";
            case scope_safe_hand_frame_math::ResolutionMode::Unavailable:
                return "unavailable";
            }
            return "unknown";
        }

        inline float diagnosticPointDistance(
            const RE::NiPoint3& left,
            const RE::NiPoint3& right)
        {
            const RE::NiPoint3 delta = left - right;
            const float distance = delta.Length();
            return std::isfinite(distance) ? distance : -1.0f;
        }

        inline float diagnosticRotationDeterminant(const RE::NiMatrix3& rotation)
        {
            const float determinant =
                rotation.entry[0][0] *
                    (rotation.entry[1][1] * rotation.entry[2][2] -
                        rotation.entry[1][2] * rotation.entry[2][1]) -
                rotation.entry[0][1] *
                    (rotation.entry[1][0] * rotation.entry[2][2] -
                        rotation.entry[1][2] * rotation.entry[2][0]) +
                rotation.entry[0][2] *
                    (rotation.entry[1][0] * rotation.entry[2][1] -
                        rotation.entry[1][1] * rotation.entry[2][0]);
            return std::isfinite(determinant) ? determinant : -1.0f;
        }

        inline grab_pinch_pocket_policy::Config
            currentWeaponOppositionPocketConfig()
        {
            return grab_pinch_pocket_policy::sanitizeConfig(
                grab_pinch_pocket_policy::Config{
                    .enabled =
                        g_rockConfig.rockGrabPinchPocketEnabled,
                    .compactMaxExtentGameUnits =
                        g_rockConfig.
                            rockGrabPinchCompactMaxExtentGameUnits,
                    .thinRodMaxLengthGameUnits =
                        g_rockConfig.
                            rockGrabPinchThinRodMaxLengthGameUnits,
                    .thinRodMaxCrossSectionGameUnits =
                        g_rockConfig.
                            rockGrabPinchThinRodMaxCrossSectionGameUnits,
                    .maxPocketDistanceGameUnits =
                        g_rockConfig.
                            rockGrabPinchMaxPocketDistanceGameUnits,
                    .minFingerGapGameUnits =
                        g_rockConfig.
                            rockGrabPinchMinFingerGapGameUnits,
                    .maxFingerGapGameUnits =
                        g_rockConfig.
                            rockGrabPinchMaxFingerGapGameUnits,
                    .thumbIndexMaxOpenValue =
                        g_rockConfig.
                            rockGrabPinchThumbIndexMaxOpenValue,
                    .otherFingerCurlValue =
                        g_rockConfig.
                            rockGrabPinchOtherFingerCurlValue,
                    .surfaceInsetGameUnits =
                        g_rockConfig.
                            rockGrabPinchSurfaceInsetGameUnits,
                    .detectionDirectionHandspace =
                        g_rockConfig.
                            rockGrabPinchDetectionDirectionHandspace,
                    .detectionAxisBlend =
                        g_rockConfig.
                            rockGrabPinchDetectionAxisBlend,
                });
        }

        inline void applyStableWeaponOppositionPose(
            grab_finger_pose_runtime::SolvedGrabFingerPose& pose,
            const grab_pinch_pocket_policy::Config& config,
            const std::size_t opposedFingerIndex)
        {
            const auto stable =
                grab_pinch_pocket_policy::
                    buildStableOppositionFingerPose(
                        config,
                        g_rockConfig.rockGrabFingerMinValue,
                        opposedFingerIndex);
            pose.values = stable.values;
            pose.jointValues = stable.jointValues;
            pose.surfaceAimTarget = {};
            pose.surfaceAimNormal = {};
            pose.surfaceAimTargetValid = {};
            pose.surfaceAimNormalValid = {};
            pose.surfaceAimTargetObjectLocal = {};
            pose.surfaceAimNormalObjectLocal = {};
            pose.surfaceAimTargetObjectLocalValid = {};
            pose.surfaceAimNormalObjectLocalValid = {};
            pose.contactArcRotationRadians = {};
            pose.contactArcRotationValid = {};
            pose.hasObjectLocalSurfaceAim = false;
            pose.usedAlternateThumbCurve = false;
            pose.usedAlternateThumbSurfaceHit = false;
            pose.hasJointValues = true;
            pose.solved = true;
        }

        /*
         * The part-carry two-anchor solve feeds its own rotation back as the
         * next frame's base, chaining several float matrix products per frame.
         * Without re-orthonormalization the rotation's row norms decay and the
         * matrix acquires shear, which visibly stretches the weapon mesh and
         * collapses the grip geometry (telemetry: rigid grip separation decayed
         * ~0.05% per frame). Rows are the stored local axes.
         */
        inline RE::NiMatrix3 orthonormalizeStoredRotation(const RE::NiMatrix3& rotation)
        {
            const RE::NiPoint3 row0{ rotation.entry[0][0], rotation.entry[0][1], rotation.entry[0][2] };
            const RE::NiPoint3 row1{ rotation.entry[1][0], rotation.entry[1][1], rotation.entry[1][2] };

            const RE::NiPoint3 axis0 = weaponSolverNormalize(row0);
            RE::NiPoint3 axis2 = weaponSolverCross(axis0, row1);
            axis2 = weaponSolverNormalize(axis2);
            const RE::NiPoint3 axis1 = weaponSolverCross(axis2, axis0);

            RE::NiMatrix3 result = rotation;
            result.entry[0][0] = axis0.x;
            result.entry[0][1] = axis0.y;
            result.entry[0][2] = axis0.z;
            result.entry[1][0] = axis1.x;
            result.entry[1][1] = axis1.y;
            result.entry[1][2] = axis1.z;
            result.entry[2][0] = axis2.x;
            result.entry[2][1] = axis2.y;
            result.entry[2][2] = axis2.z;
            return result;
        }

        /*
         * Ambidextrous firing grip (left-hand fire) needs BOTH hFRIK blockers:
         * the finger-pose block (right hand must stop receiving FRIK's weapon
         * pose) and the weapon-node ownership block (FRIK must stop gluing the
         * weapon to the right hand). Fail closed to right-only behavior when
         * either is missing (older FRIK build).
         */
        inline bool leftFiringInfrastructureAvailable()
        {
            return frik_visual_authority::canBlockPrimaryHandWeaponPose() &&
                frik_visual_authority::canBlockPrimaryWeaponNodeOwnership();
        }

        inline RE::NiNode* sourceRootNodeOrFallback(RE::NiAVObject* sourceRoot, RE::NiNode* fallback)
        {
            if (sourceRoot) {
                if (auto* sourceNode = sourceRoot->IsNode()) {
                    return sourceNode;
                }
            }
            return fallback;
        }

        inline RE::NiPoint3 lerpPoint(const RE::NiPoint3& from, const RE::NiPoint3& to, float alpha)
        {
            const float t = (std::max)(0.0f, (std::min)(1.0f, alpha));
            return RE::NiPoint3{ from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t, from.z + (to.z - from.z) * t };
        }

        inline bool isFiniteRotation(const RE::NiMatrix3& rotation)
        {
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(rotation.entry[row][column])) {
                        return false;
                    }
                }
            }
            return true;
        }

        inline bool isFiniteTransform(const RE::NiTransform& transform)
        {
            return isFiniteRotation(transform.rotate) && std::isfinite(transform.translate.x) && std::isfinite(transform.translate.y) &&
                   std::isfinite(transform.translate.z) && std::isfinite(transform.scale);
        }

        inline bool isInvertibleTransform(const RE::NiTransform& transform)
        {
            return isFiniteTransform(transform) &&
                std::abs(transform.scale) > 0.0001f;
        }

        inline RE::NiTransform makeLeftFiringWandAimTrim(
            const EquippedWeaponHandlingSettings& handlingSettings)
        {
            constexpr float kDegreesToRadians =
                0.017453292519943295769f;
            const float yawRadians =
                handlingSettings.leftFiringAimYawDegrees *
                kDegreesToRadians;
            const float pitchRadians =
                handlingSettings.leftFiringAimPitchDegrees *
                kDegreesToRadians;

            RE::NiTransform yawTrim =
                transform_math::makeIdentityTransform<RE::NiTransform>();
            if (yawRadians != 0.0f) {
                const float cosine = std::cos(yawRadians);
                const float sine = std::sin(yawRadians);
                yawTrim.rotate.entry[0][0] = cosine;
                yawTrim.rotate.entry[0][1] = -sine;
                yawTrim.rotate.entry[1][0] = sine;
                yawTrim.rotate.entry[1][1] = cosine;
            }

            RE::NiTransform pitchTrim =
                transform_math::makeIdentityTransform<RE::NiTransform>();
            if (pitchRadians != 0.0f) {
                const float cosine = std::cos(pitchRadians);
                const float sine = std::sin(pitchRadians);
                pitchTrim.rotate.entry[1][1] = cosine;
                pitchTrim.rotate.entry[1][2] = sine;
                pitchTrim.rotate.entry[2][1] = -sine;
                pitchTrim.rotate.entry[2][2] = cosine;
            }

            return transform_math::composeTransforms(yawTrim, pitchTrim);
        }

        inline bool validateWeaponPresentationSubtree(
            RE::NiAVObject* object,
            const RE::NiTransform& presentationWorldDelta,
            const std::size_t depth,
            std::size_t& objectCount)
        {
            if (!object || depth > WEAPON_PRESENTATION_MAX_DEPTH ||
                objectCount >= WEAPON_PRESENTATION_MAX_OBJECTS ||
                !isFiniteTransform(object->world)) {
                return false;
            }
            ++objectCount;

            const RE::NiTransform reframedWorld =
                weapon_visual_authority_math::applyPresentationWorldDelta(
                    presentationWorldDelta,
                    object->world);
            if (!isFiniteTransform(reframedWorld)) {
                return false;
            }

            auto* node = object->IsNode();
            if (!node) {
                return true;
            }
            for (const auto& child : node->children) {
                if (child && !validateWeaponPresentationSubtree(
                                 child.get(),
                                 presentationWorldDelta,
                                 depth + 1,
                                 objectCount)) {
                    return false;
                }
            }
            return true;
        }

        inline void applyWeaponPresentationDeltaToDescendants(
            RE::NiNode* parent,
            const RE::NiTransform& presentationWorldDelta)
        {
            for (const auto& child : parent->children) {
                if (!child) {
                    continue;
                }
                child->world =
                    weapon_visual_authority_math::applyPresentationWorldDelta(
                        presentationWorldDelta,
                        child->world);
                if (auto* childNode = child->IsNode()) {
                    applyWeaponPresentationDeltaToDescendants(
                        childNode,
                        presentationWorldDelta);
                }
            }
        }

        inline bool tryResolveWeaponRootLocal(
            const RE::NiNode* parent,
            const RE::NiTransform& weaponWorld,
            RE::NiTransform& outWeaponLocal)
        {
            if (!isFiniteTransform(weaponWorld)) {
                return false;
            }

            outWeaponLocal = weaponWorld;
            if (parent) {
                if (!isInvertibleTransform(parent->world)) {
                    return false;
                }
                outWeaponLocal =
                    weapon_visual_authority_math::worldTargetToParentLocal(
                        parent->world,
                        weaponWorld);
            }
            return isFiniteTransform(outWeaponLocal);
        }

        inline bool restoreWeaponRootPreservingPresentedDescendants(
            RE::NiNode* weaponNode,
            const RE::NiTransform& weaponWorld)
        {
            if (!weaponNode) {
                return false;
            }

            RE::NiTransform weaponLocal{};
            if (!tryResolveWeaponRootLocal(
                    weaponNode->parent,
                    weaponWorld,
                    weaponLocal)) {
                return false;
            }
            weaponNode->local = weaponLocal;
            weaponNode->world = weaponWorld;
            return true;
        }

        inline bool moveWeaponPresentationRigidly(
            RE::NiNode* weaponNode,
            const RE::NiTransform& solvedWeaponWorld)
        {
            if (!weaponNode || !isInvertibleTransform(weaponNode->world) ||
                !isFiniteTransform(solvedWeaponWorld)) {
                return false;
            }

            RE::NiTransform weaponLocal{};
            if (!tryResolveWeaponRootLocal(
                    weaponNode->parent,
                    solvedWeaponWorld,
                    weaponLocal)) {
                return false;
            }

            const RE::NiTransform oldWeaponWorld = weaponNode->world;
            const RE::NiTransform presentationWorldDelta =
                weapon_visual_authority_math::makePresentationWorldDelta(
                    oldWeaponWorld,
                    solvedWeaponWorld);
            if (!isFiniteTransform(presentationWorldDelta)) {
                return false;
            }

            // Validate the entire bounded tree before the first write so an
            // invalid modded node cannot leave a partially moved presentation.
            std::size_t objectCount = 0;
            if (!validateWeaponPresentationSubtree(
                    weaponNode,
                    presentationWorldDelta,
                    0,
                    objectCount)) {
                return false;
            }

            weaponNode->local = weaponLocal;
            weaponNode->world = solvedWeaponWorld;
            applyWeaponPresentationDeltaToDescendants(
                weaponNode,
                presentationWorldDelta);
            return true;
        }

        inline bool areTransformsNearlyEqual(const RE::NiTransform& lhs, const RE::NiTransform& rhs, const float epsilon = 0.001f)
        {
            if (std::abs(lhs.translate.x - rhs.translate.x) > epsilon ||
                std::abs(lhs.translate.y - rhs.translate.y) > epsilon ||
                std::abs(lhs.translate.z - rhs.translate.z) > epsilon ||
                std::abs(lhs.scale - rhs.scale) > epsilon) {
                return false;
            }
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (std::abs(lhs.rotate.entry[row][column] - rhs.rotate.entry[row][column]) > epsilon) {
                        return false;
                    }
                }
            }
            return true;
        }

        inline bool arePointsNearlyEqual(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs, const float epsilon = 0.001f)
        {
            return std::abs(lhs.x - rhs.x) <= epsilon &&
                   std::abs(lhs.y - rhs.y) <= epsilon &&
                   std::abs(lhs.z - rhs.z) <= epsilon;
        }

        inline bool tryGetComposedNodeWorld(const RE::NiAVObject* node, RE::NiTransform& outWorld)
        {
            if (!node) {
                return false;
            }
            outWorld = node->parent ?
                transform_math::composeTransforms(node->parent->world, node->local) :
                node->world;
            return isFiniteTransform(outWorld);
        }

        inline bool isUsableHandAuthorityTransform(const RE::NiTransform& transform)
        {
            // hFRIK uses 0.00001 as its intentional ScopeMenu hide scale.
            return isFiniteTransform(transform) && std::abs(transform.scale) > 0.0001f;
        }

        struct AuthoredSupportPalmSeatProximity
        {
            RE::NiTransform authoredHandWorld{};
            RE::NiPoint3 authoredPalmSeatWeaponLocal{};
            RE::NiPoint3 authoredPalmSeatWorld{};
            RE::NiPoint3 liveTouchProbeWeaponLocal{};
            RE::NiPoint3 liveTouchProbeWorld{};
            float weaponRelativeDistanceGameUnits{ 0.0f };
            float worldReadbackDistanceGameUnits{ 0.0f };
            float frameAgreementErrorGameUnits{ 0.0f };
        };

        [[nodiscard]] inline bool resolveAuthoredSupportPalmSeatProximity(
            const RE::NiTransform& weaponWorld,
            const RE::NiTransform& liveHandWorld,
            const RE::NiTransform& authoredHandWeaponLocal,
            const bool isLeft,
            AuthoredSupportPalmSeatProximity& out)
        {
            out = {};
            if (!isFiniteTransform(weaponWorld) ||
                std::abs(weaponWorld.scale) <= 0.0001f ||
                !isUsableHandAuthorityTransform(liveHandWorld) ||
                !isFiniteTransform(authoredHandWeaponLocal) ||
                std::abs(authoredHandWeaponLocal.scale) <= 0.0001f) {
                return false;
            }

            out.authoredHandWorld = transform_math::composeTransforms(
                weaponWorld,
                authoredHandWeaponLocal);
            out.authoredPalmSeatWeaponLocal =
                computeGrabLegacyPalmPivotAWorldFromHandBasis(
                    authoredHandWeaponLocal,
                    isLeft);
            out.authoredPalmSeatWorld = transform_math::localPointToWorld(
                weaponWorld,
                out.authoredPalmSeatWeaponLocal);
            out.liveTouchProbeWorld =
                computeGrabLegacyPalmPivotAWorldFromHandBasis(
                    liveHandWorld,
                    isLeft);
            out.liveTouchProbeWeaponLocal = transform_math::worldPointToLocal(
                weaponWorld,
                out.liveTouchProbeWorld);
            if (!isFiniteTransform(out.authoredHandWorld) ||
                !std::isfinite(out.authoredPalmSeatWeaponLocal.x) ||
                !std::isfinite(out.authoredPalmSeatWeaponLocal.y) ||
                !std::isfinite(out.authoredPalmSeatWeaponLocal.z) ||
                !std::isfinite(out.authoredPalmSeatWorld.x) ||
                !std::isfinite(out.authoredPalmSeatWorld.y) ||
                !std::isfinite(out.authoredPalmSeatWorld.z) ||
                !std::isfinite(out.liveTouchProbeWeaponLocal.x) ||
                !std::isfinite(out.liveTouchProbeWeaponLocal.y) ||
                !std::isfinite(out.liveTouchProbeWeaponLocal.z) ||
                !std::isfinite(out.liveTouchProbeWorld.x) ||
                !std::isfinite(out.liveTouchProbeWorld.y) ||
                !std::isfinite(out.liveTouchProbeWorld.z)) {
                return false;
            }

            // The touch probe and yellow authored palm seat are points, not
            // wrist bones. Compare them in one current Weapon frame so a ROCK
            // physical-left reparent or world translation cannot contaminate
            // acquisition.
            const float localDeltaX =
                out.liveTouchProbeWeaponLocal.x -
                out.authoredPalmSeatWeaponLocal.x;
            const float localDeltaY =
                out.liveTouchProbeWeaponLocal.y -
                out.authoredPalmSeatWeaponLocal.y;
            const float localDeltaZ =
                out.liveTouchProbeWeaponLocal.z -
                out.authoredPalmSeatWeaponLocal.z;
            const float weaponLocalDistance = std::sqrt(
                localDeltaX * localDeltaX +
                localDeltaY * localDeltaY +
                localDeltaZ * localDeltaZ);
            out.weaponRelativeDistanceGameUnits =
                weaponLocalDistance * std::abs(weaponWorld.scale);

            // Readback is diagnostic only. The Weapon-relative value above is
            // the acquisition gate.
            const float worldDeltaX =
                out.liveTouchProbeWorld.x - out.authoredPalmSeatWorld.x;
            const float worldDeltaY =
                out.liveTouchProbeWorld.y - out.authoredPalmSeatWorld.y;
            const float worldDeltaZ =
                out.liveTouchProbeWorld.z - out.authoredPalmSeatWorld.z;
            out.worldReadbackDistanceGameUnits = std::sqrt(
                worldDeltaX * worldDeltaX +
                worldDeltaY * worldDeltaY +
                worldDeltaZ * worldDeltaZ);
            out.frameAgreementErrorGameUnits = std::abs(
                out.weaponRelativeDistanceGameUnits -
                out.worldReadbackDistanceGameUnits);
            return std::isfinite(out.weaponRelativeDistanceGameUnits) &&
                   std::isfinite(out.worldReadbackDistanceGameUnits) &&
                   std::isfinite(out.frameAgreementErrorGameUnits);
        }

        struct RankedSupportGripTriangle
        {
            float distanceSquared = 0.0f;
            std::uint64_t deterministicOrdinal = 0;
            TriangleData weaponLocalTriangle{};
        };

        inline bool rankedSupportGripTriangleLess(const RankedSupportGripTriangle& lhs, const RankedSupportGripTriangle& rhs)
        {
            if (lhs.distanceSquared == rhs.distanceSquared) {
                return lhs.deterministicOrdinal <
                       rhs.deterministicOrdinal;
            }
            return lhs.distanceSquared < rhs.distanceSquared;
        }

        inline constexpr std::size_t
            kSupportGripFingerLaneCount = 5;
        inline constexpr std::size_t
            kSupportGripFingerLaneReferenceCapacity = 10;
        inline constexpr std::size_t
            kSupportGripGlobalRankingIndex =
                kSupportGripFingerLaneCount;

        struct SupportGripFingerReferenceSet
        {
            RE::NiPoint3 seatPointWorld{};
            std::array<
                std::array<
                    RE::NiPoint3,
                    kSupportGripFingerLaneReferenceCapacity>,
                kSupportGripFingerLaneCount>
                lanePointsWorld{};
            std::array<std::size_t,
                kSupportGripFingerLaneCount>
                lanePointCounts{};
            bool seatPointValid{ false };
        };

        struct TransformedSupportGripTriangleView
        {
            std::span<const TriangleData> localTriangles{};
            RE::NiTransform localToWorld{};

            [[nodiscard]] std::size_t size() const noexcept { return localTriangles.size(); }

            [[nodiscard]] TriangleData operator[](std::size_t index) const
            {
                const auto& triangle = localTriangles[index];
                return TriangleData{
                    transform_math::localPointToWorld(localToWorld, triangle.v0),
                    transform_math::localPointToWorld(localToWorld, triangle.v1),
                    transform_math::localPointToWorld(localToWorld, triangle.v2),
                };
            }
        };

        inline void selectNearestSupportGripFingerTriangles(
            std::span<const WeaponCollision::SupportGripEvidenceView> evidenceViews,
            const RE::NiTransform& weaponWorld,
            const SupportGripFingerReferenceSet& referenceSet,
            std::size_t maxTriangles,
            std::array<std::vector<RankedSupportGripTriangle>,
                kSupportGripFingerLaneCount + 1>& rankingScratch,
            std::vector<TriangleData>& outTriangles)
        {
            for (auto& ranking : rankingScratch) {
                ranking.clear();
            }
            outTriangles.clear();
            const std::size_t boundedLimit = (std::min)(maxTriangles, grab_finger_pose_runtime::kMaxFingerPoseCandidateTriangles);
            if (boundedLimit == 0 ||
                evidenceViews.empty() ||
                !referenceSet.seatPointValid ||
                !weapon_support_acquisition_math::isUsableTransform(
                    weaponWorld)) {
                return;
            }

            SupportGripFingerReferenceSet localReferences{};
            localReferences.seatPointWorld =
                transform_math::worldPointToLocal(
                    weaponWorld,
                    referenceSet.seatPointWorld);
            localReferences.seatPointValid =
                grab_finger_pose_runtime::isFinitePoint(
                    localReferences.seatPointWorld);
            if (!localReferences.seatPointValid) {
                return;
            }
            for (std::size_t lane = 0;
                 lane < kSupportGripFingerLaneCount;
                 ++lane) {
                const std::size_t count = (std::min)(
                    referenceSet.lanePointCounts[lane],
                    kSupportGripFingerLaneReferenceCapacity);
                for (std::size_t point = 0; point < count; ++point) {
                    const RE::NiPoint3 localPoint =
                        transform_math::worldPointToLocal(
                            weaponWorld,
                            referenceSet.lanePointsWorld[lane][point]);
                    if (!grab_finger_pose_runtime::isFinitePoint(
                            localPoint)) {
                        continue;
                    }
                    localReferences.lanePointsWorld[lane]
                        [localReferences.lanePointCounts[lane]++] =
                        localPoint;
                }
                if (localReferences.lanePointCounts[lane] >
                    kSupportGripFingerLaneReferenceCapacity) {
                    return;
                }
            }

            const std::size_t laneLimit = (std::max)(
                static_cast<std::size_t>(1),
                (boundedLimit + kSupportGripFingerLaneCount - 1) /
                    kSupportGripFingerLaneCount);
            for (std::size_t lane = 0;
                 lane < kSupportGripFingerLaneCount;
                 ++lane) {
                rankingScratch[lane].reserve(laneLimit);
            }
            rankingScratch[kSupportGripGlobalRankingIndex].reserve(
                boundedLimit);

            const auto retainNearest = [](
                                           std::vector<RankedSupportGripTriangle>& ranking,
                                           const std::size_t limit,
                                           const RankedSupportGripTriangle& candidate) {
                if (ranking.size() < limit) {
                    ranking.push_back(candidate);
                    std::push_heap(
                        ranking.begin(),
                        ranking.end(),
                        rankedSupportGripTriangleLess);
                    return;
                }
                if (rankedSupportGripTriangleLess(
                        candidate,
                        ranking.front())) {
                    std::pop_heap(
                        ranking.begin(),
                        ranking.end(),
                        rankedSupportGripTriangleLess);
                    ranking.back() = candidate;
                    std::push_heap(
                        ranking.begin(),
                        ranking.end(),
                        rankedSupportGripTriangleLess);
                }
            };

            std::uint64_t deterministicOrdinal = 0;
            for (const auto& evidenceView : evidenceViews) {
                if (evidenceView.localTriangles.empty() ||
                    !weapon_support_acquisition_math::isUsableTransform(
                        evidenceView.localToWorld)) {
                    continue;
                }
                const RE::NiTransform sourceToWeapon =
                    transform_math::composeTransforms(
                        transform_math::invertTransform(weaponWorld),
                        evidenceView.localToWorld);
                if (!weapon_support_acquisition_math::isUsableTransform(
                        sourceToWeapon)) {
                    continue;
                }

                for (const auto& sourceTriangle :
                     evidenceView.localTriangles) {
                    const std::uint64_t ordinal =
                        deterministicOrdinal++;
                    if (!grab_finger_pose_runtime::isFinitePoint(
                            sourceTriangle.v0) ||
                        !grab_finger_pose_runtime::isFinitePoint(
                            sourceTriangle.v1) ||
                        !grab_finger_pose_runtime::isFinitePoint(
                            sourceTriangle.v2)) {
                        continue;
                    }

                    const TriangleData weaponLocalTriangle{
                        transform_math::localPointToWorld(
                            sourceToWeapon,
                            sourceTriangle.v0),
                        transform_math::localPointToWorld(
                            sourceToWeapon,
                            sourceTriangle.v1),
                        transform_math::localPointToWorld(
                            sourceToWeapon,
                            sourceTriangle.v2),
                    };
                    float globalMinimumDistanceSquared =
                        (std::numeric_limits<float>::infinity)();
                    (void)closestPointOnTriangleToPoint(
                        localReferences.seatPointWorld,
                        weaponLocalTriangle,
                        globalMinimumDistanceSquared);
                    for (std::size_t lane = 0;
                         lane < kSupportGripFingerLaneCount;
                         ++lane) {
                        float laneMinimumDistanceSquared =
                            (std::numeric_limits<float>::infinity)();
                        for (std::size_t referenceIndex = 0;
                             referenceIndex <
                                 localReferences.lanePointCounts[lane];
                             ++referenceIndex) {
                            float distanceSquared = 0.0f;
                            (void)closestPointOnTriangleToPoint(
                                localReferences.lanePointsWorld[lane]
                                    [referenceIndex],
                                weaponLocalTriangle,
                                distanceSquared);
                            if (std::isfinite(distanceSquared)) {
                                laneMinimumDistanceSquared = (std::min)(
                                    laneMinimumDistanceSquared,
                                    distanceSquared);
                            }
                        }
                        if (std::isfinite(laneMinimumDistanceSquared)) {
                            globalMinimumDistanceSquared = (std::min)(
                                globalMinimumDistanceSquared,
                                laneMinimumDistanceSquared);
                            retainNearest(
                                rankingScratch[lane],
                                laneLimit,
                                RankedSupportGripTriangle{
                                    .distanceSquared =
                                        laneMinimumDistanceSquared,
                                    .deterministicOrdinal = ordinal,
                                    .weaponLocalTriangle =
                                        weaponLocalTriangle,
                                });
                        }
                    }
                    if (std::isfinite(globalMinimumDistanceSquared)) {
                        retainNearest(
                            rankingScratch[
                                kSupportGripGlobalRankingIndex],
                            boundedLimit,
                            RankedSupportGripTriangle{
                                .distanceSquared =
                                    globalMinimumDistanceSquared,
                                .deterministicOrdinal = ordinal,
                                .weaponLocalTriangle =
                                    weaponLocalTriangle,
                            });
                    }
                }
            }

            for (auto& ranking : rankingScratch) {
                std::sort(
                    ranking.begin(),
                    ranking.end(),
                    rankedSupportGripTriangleLess);
            }
            std::vector<std::uint64_t> selectedOrdinals{};
            selectedOrdinals.reserve(boundedLimit);
            outTriangles.reserve(boundedLimit);
            const auto appendUnique = [&](
                                          const RankedSupportGripTriangle& ranked) {
                if (outTriangles.size() >= boundedLimit ||
                    std::find(
                        selectedOrdinals.begin(),
                        selectedOrdinals.end(),
                        ranked.deterministicOrdinal) !=
                        selectedOrdinals.end()) {
                    return false;
                }
                selectedOrdinals.push_back(
                    ranked.deterministicOrdinal);
                outTriangles.push_back(ranked.weaponLocalTriangle);
                return true;
            };

            std::array<std::size_t,
                kSupportGripFingerLaneCount>
                laneCursors{};
            bool laneCandidateRemaining = true;
            while (outTriangles.size() < boundedLimit &&
                   laneCandidateRemaining) {
                laneCandidateRemaining = false;
                for (std::size_t lane = 0;
                     lane < kSupportGripFingerLaneCount &&
                     outTriangles.size() < boundedLimit;
                     ++lane) {
                    auto& cursor = laneCursors[lane];
                    const auto& ranking = rankingScratch[lane];
                    while (cursor < ranking.size()) {
                        laneCandidateRemaining = true;
                        const auto& candidate = ranking[cursor++];
                        if (appendUnique(candidate)) {
                            break;
                        }
                    }
                }
            }
            for (const auto& ranked :
                 rankingScratch[kSupportGripGlobalRankingIndex]) {
                if (outTriangles.size() >= boundedLimit) {
                    break;
                }
                (void)appendUnique(ranked);
            }
        }

        struct NativeScopeCameraFollowCapture
        {
            RE::NiNode* camera{ nullptr };
            RE::NiTransform weaponWorldBefore{};
            RE::NiTransform cameraWorldBefore{};
            bool valid{ false };
        };

        struct NativeScopeCameraFollowResult
        {
            RE::NiTransform targetCameraWorld{};
            RE::NiTransform immediateCameraWorldAfter{};
            bool targetValid{ false };
            bool writeApplied{ false };
            bool immediateReadbackValid{ false };
        };

        struct ScopeHandAuthorityCleanupVisualSnapshot
        {
            RE::NiNode* weapon{ nullptr };
            RE::NiTransform weaponWorld{};
            RE::NiNode* scopeCamera{ nullptr };
            RE::NiTransform scopeCameraWorld{};
            bool weaponValid{ false };
            bool scopeCameraValid{ false };
        };

        inline ScopeHandAuthorityCleanupVisualSnapshot captureScopeHandAuthorityCleanupVisuals(RE::NiNode* weaponNode)
        {
            ScopeHandAuthorityCleanupVisualSnapshot snapshot{};
            if (weaponNode && isFiniteTransform(weaponNode->world)) {
                snapshot.weapon = weaponNode;
                snapshot.weaponWorld = weaponNode->world;
                snapshot.weaponValid = true;
            }

            const auto* playerNodes = f4vr::getPlayerNodes();
            auto* scopeCamera = playerNodes ? playerNodes->primaryWeaponScopeCamera : nullptr;
            if (scopeCamera) {
                RE::NiTransform scopeCameraWorld = scopeCamera->world;
                if (scopeCamera->parent) {
                    scopeCameraWorld = transform_math::composeTransforms(scopeCamera->parent->world, scopeCamera->local);
                }
                if (isFiniteTransform(scopeCameraWorld)) {
                    snapshot.scopeCamera = scopeCamera;
                    snapshot.scopeCameraWorld = scopeCameraWorld;
                    snapshot.scopeCameraValid = true;
                }
            }
            return snapshot;
        }

        inline void restoreScopeHandAuthorityCleanupVisuals(const ScopeHandAuthorityCleanupVisualSnapshot& snapshot)
        {
            if (snapshot.weaponValid && snapshot.weapon) {
                (void)restoreWeaponRootPreservingPresentedDescendants(
                    snapshot.weapon,
                    snapshot.weaponWorld);
            }

            if (snapshot.scopeCameraValid && snapshot.scopeCamera) {
                if (snapshot.scopeCamera->parent) {
                    snapshot.scopeCamera->local = weapon_visual_authority_math::worldTargetToParentLocal(
                        snapshot.scopeCamera->parent->world,
                        snapshot.scopeCameraWorld);
                    f4vr::updateTransforms(snapshot.scopeCamera);
                } else {
                    snapshot.scopeCamera->local = snapshot.scopeCameraWorld;
                    snapshot.scopeCamera->world = snapshot.scopeCameraWorld;
                }
            }
        }

        inline NativeScopeCameraFollowCapture captureNativeScopeCameraFollow(const RE::NiNode* weaponNode)
        {
            const auto* playerNodes = f4vr::getPlayerNodes();
            auto* scopeCamera = playerNodes ? playerNodes->primaryWeaponScopeCamera : nullptr;
            if (!weaponNode || !scopeCamera || !isFiniteTransform(weaponNode->world)) {
                return {};
            }

            RE::NiTransform cameraWorld = scopeCamera->world;
            if (scopeCamera->parent) {
                cameraWorld = transform_math::composeTransforms(scopeCamera->parent->world, scopeCamera->local);
            }
            if (!isFiniteTransform(cameraWorld)) {
                return {};
            }

            return NativeScopeCameraFollowCapture{
                .camera = scopeCamera,
                .weaponWorldBefore = weaponNode->world,
                .cameraWorldBefore = cameraWorld,
                .valid = true,
            };
        }

        inline NativeScopeCameraFollowResult applyNativeScopeCameraWorldTarget(const NativeScopeCameraFollowCapture& capture, const RE::NiTransform& targetCameraWorld)
        {
            NativeScopeCameraFollowResult result{};
            if (!capture.valid || !capture.camera || !isFiniteTransform(targetCameraWorld)) {
                return result;
            }
            result.targetCameraWorld = targetCameraWorld;
            result.targetValid = true;

            auto* scopeCamera = capture.camera;
            if (scopeCamera->parent) {
                const RE::NiTransform targetCameraLocal = weapon_visual_authority_math::worldTargetToParentLocal(
                    scopeCamera->parent->world,
                    targetCameraWorld);
                if (!isFiniteTransform(targetCameraLocal)) {
                    return result;
                }
                scopeCamera->local = targetCameraLocal;
                f4vr::updateTransforms(scopeCamera);
                result.writeApplied = true;
                const RE::NiTransform immediateCameraWorld = scopeCamera->world;
                if (isFiniteTransform(immediateCameraWorld)) {
                    result.immediateCameraWorldAfter = immediateCameraWorld;
                    result.immediateReadbackValid = true;
                }
                return result;
            }

            scopeCamera->local = targetCameraWorld;
            scopeCamera->world = targetCameraWorld;
            result.writeApplied = true;
            result.immediateCameraWorldAfter = scopeCamera->world;
            result.immediateReadbackValid = isFiniteTransform(result.immediateCameraWorldAfter);
            return result;
        }

        inline NativeScopeCameraDebugSnapshot makeNativeScopeCameraDebugSnapshot(const NativeScopeCameraDebugSnapshot& previous, const std::uint64_t weaponGenerationKey,
            const NativeScopeCameraWriteSource writeSource, const NativeScopeCameraFollowCapture& capture, const NativeScopeCameraFollowResult& result,
            const native_scope_sight_anchor_policy::AnchorSource anchorSource)
        {
            NativeScopeCameraDebugSnapshot snapshot{};
            snapshot.applySequence = previous.applySequence + 1;
            snapshot.weaponGenerationKey = weaponGenerationKey;
            snapshot.framesSinceApply = 0;
            snapshot.writeSource = writeSource;
            snapshot.captureValid = capture.valid;
            snapshot.targetValid = result.targetValid;
            snapshot.writeApplied = result.writeApplied;
            snapshot.immediateReadbackValid = result.immediateReadbackValid;
            snapshot.anchorSource = anchorSource;
            if (capture.valid) {
                snapshot.cameraWorldBefore = capture.cameraWorldBefore;
            }
            if (result.targetValid) {
                snapshot.targetCameraWorld = result.targetCameraWorld;
            }
            if (result.immediateReadbackValid) {
                snapshot.immediateCameraWorldAfter = result.immediateCameraWorldAfter;
            }
            return snapshot;
        }

        inline DirectSkeletonBoneReader& rootFlattenedTwoHandedReader()
        {
            static DirectSkeletonBoneReader reader;
            return reader;
        }

        inline const DirectSkeletonBoneEntry* findSnapshotBone(const DirectSkeletonBoneSnapshot& snapshot, std::string_view name)
        {
            for (const auto& bone : snapshot.bones) {
                if (bone.name == name) {
                    return &bone;
                }
            }
            return nullptr;
        }

        inline bool buildFullHandLocalTransformsForMeshPose(
            bool isLeft,
            const grab_finger_pose_runtime::SolvedGrabFingerPose& meshFingerPose,
            const frik_visual_authority::HandPoseData& handPose,
            const DirectSkeletonBoneSnapshot* capturedFingerSnapshot,
            std::array<RE::NiTransform, 15>& outLocalTransforms,
            std::uint16_t& outMask)
        {
            auto* api = frik_visual_authority::api();
            const bool canPublish =
                grab_finger_local_transform_math::shouldPublishLocalTransformPose(
                    g_rockConfig.rockGrabMeshLocalTransformPoseEnabled,
                    meshFingerPose.solved,
                    true,
                    api && api->getHandPoseLocalTransformsForPose != nullptr,
                    api && api->setHandPoseCustomLocalTransforms != nullptr);
            if (!canPublish) {
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: full-hand local transform override skipped hand={} enabled={} api={} baselineApi={} publishApi={}",
                    isLeft ? "left" : "right",
                    g_rockConfig.rockGrabMeshLocalTransformPoseEnabled ? "yes" : "no",
                    api ? "yes" : "no",
                    (api && api->getHandPoseLocalTransformsForPose) ? "yes" : "no",
                    (api && api->setHandPoseCustomLocalTransforms) ? "yes" : "no");
                return false;
            }

            frik_visual_authority::FingerLocalTransformOverride baseline{};
            if (!frik_visual_authority::getHandPoseLocalTransformsForPose(handFromBool(isLeft), handPose, &baseline)) {
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: full-hand local transform override failed hand={} reason=baseline-query", isLeft ? "left" : "right");
                return false;
            }

            frik_visual_authority::FingerLocalTransformOverride corrected{};
            const char* failureReason = "unknown";
            if (!grab_finger_local_transform_runtime::buildSurfaceCorrectedLocalTransforms(isLeft,
                    meshFingerPose,
                    baseline,
                    grab_finger_local_transform_runtime::Options{
                        .enabled = g_rockConfig.rockGrabMeshLocalTransformPoseEnabled,
                        .smoothingSpeed = g_rockConfig.rockGrabFingerLocalTransformSmoothingSpeed,
                        .maxCorrectionDegrees = g_rockConfig.rockGrabFingerLocalTransformMaxCorrectionDegrees,
                        .surfaceAimStrength = g_rockConfig.rockGrabFingerSurfaceAimStrength,
                        .thumbOppositionStrength = g_rockConfig.rockGrabThumbOppositionStrength,
                        .thumbAlternateCurveStrength = g_rockConfig.rockGrabThumbAlternateCurveStrength,
                        .thumbSurfaceSafetyEnabled = g_rockConfig.rockGrabThumbSurfaceSafetyEnabled,
                        .thumbSurfaceSafetyMarginGameUnits = g_rockConfig.rockGrabThumbSurfaceSafetyMarginGameUnits,
                    },
                    corrected,
                    &failureReason,
                    capturedFingerSnapshot)) {
                ROCK_LOG_WARN(Weapon,
                    "TwoHandedGrip: full-hand local transform override failed hand={} reason={}",
                    isLeft ? "left" : "right",
                    failureReason ? failureReason : "unknown");
                return false;
            }

            outMask = corrected.enabledMask;
            for (std::size_t i = 0; i < outLocalTransforms.size(); ++i) {
                outLocalTransforms[i] = corrected.localTransforms[i];
            }
            return outMask == grab_finger_local_transform_math::kFullFingerLocalTransformMask;
        }

        inline bool tryGetRootFlattenedHandBoneTransform(bool isLeft, RE::NiTransform& outTransform)
        {
            outTransform = {};
            DirectSkeletonBoneSnapshot snapshot{};
            if (!rootFlattenedTwoHandedReader().capture(skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
                    skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                    snapshot)) {
                return false;
            }

            const auto* handBone = findSnapshotBone(snapshot, isLeft ? "LArm_Hand" : "RArm_Hand");
            if (!handBone || !isUsableHandAuthorityTransform(handBone->world)) {
                return false;
            }

            outTransform = handBone->world;
            return true;
        }

        /*
         * LEFT_CARRY_CLOCK / LEFT_CARRY_STAGE probe sample (debug grab-frame
         * logging only): the left hand and elbow read from the game's root
         * flattened bone array (what the renderer skins) and from the body
         * skeleton nodes, so a stage whose array diverges from its nodes is
         * visible.
         */
        struct LeftCarryProbeSample
        {
            RE::NiTransform handArray{};
            RE::NiTransform forearmArray{};
            RE::NiTransform handNode{};
            RE::NiTransform forearmNode{};
            bool arrayValid = false;
            bool nodeValid = false;
        };

        inline LeftCarryProbeSample sampleLeftCarryProbe()
        {
            LeftCarryProbeSample sample{};
            DirectSkeletonBoneSnapshot snapshot{};
            if (rootFlattenedTwoHandedReader().capture(
                    skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
                    skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                    snapshot)) {
                const auto* handBone = findSnapshotBone(snapshot, "LArm_Hand");
                const auto* forearmBone = findSnapshotBone(snapshot, "LArm_ForeArm1");
                if (handBone && forearmBone &&
                    isUsableHandAuthorityTransform(handBone->world) &&
                    isFiniteTransform(forearmBone->world)) {
                    sample.handArray = handBone->world;
                    sample.forearmArray = forearmBone->world;
                    sample.arrayValid = true;
                }
            }
            auto* const rootNode = f4vr::getRootNode();
            const auto* const handNode = rootNode ? f4vr::findNode(rootNode, "LArm_Hand") : nullptr;
            const auto* const forearmNode = rootNode ? f4vr::findNode(rootNode, "LArm_ForeArm1") : nullptr;
            if (handNode && forearmNode &&
                isUsableHandAuthorityTransform(handNode->world) &&
                isFiniteTransform(forearmNode->world)) {
                sample.handNode = handNode->world;
                sample.forearmNode = forearmNode->world;
                sample.nodeValid = true;
            }
            return sample;
        }

        /*
         * Left-carry solver probe: the body-skeleton left arm chain hFRIK
         * solves toward ROCK's external hand target. Rows X and Y of each
         * world rotation are that bone's local axes in world space (hFRIK
         * composes world vectors as rotate^T * local), so a roll of a twist
         * bone about its own axis shows as row Y turning while row X holds.
         */
        inline constexpr std::array<std::pair<const char*, const char*>, 6> LEFT_CARRY_ARM_BONES{ {
            { "LArm_UpperArm", "upper" },
            { "LArm_UpperTwist1", "upT1" },
            { "LArm_ForeArm1", "fa1" },
            { "LArm_ForeArm2", "fa2" },
            { "LArm_ForeArm3", "fa3" },
            { "LArm_Hand", "hand" },
        } };

        struct LeftCarryArmSample
        {
            std::array<RE::NiTransform, LEFT_CARRY_ARM_BONES.size()> bones{};
            bool valid = false;
        };

        inline LeftCarryArmSample sampleLeftCarryArm()
        {
            LeftCarryArmSample sample{};
            auto* const rootNode = f4vr::getRootNode();
            if (!rootNode) {
                return sample;
            }
            for (std::size_t index = 0; index < LEFT_CARRY_ARM_BONES.size(); ++index) {
                const auto* const node = f4vr::findNode(rootNode, LEFT_CARRY_ARM_BONES[index].first);
                if (!node || !isFiniteTransform(node->world)) {
                    return sample;
                }
                sample.bones[index] = node->world;
            }
            sample.valid = true;
            return sample;
        }

        inline std::string formatLeftCarryArmFrame(const RE::NiTransform& world)
        {
            const auto& rows = world.rotate.entry;
            return std::format(
                "({:.3f},{:.3f},{:.3f}|{:.4f},{:.4f},{:.4f}|{:.4f},{:.4f},{:.4f})",
                world.translate.x,
                world.translate.y,
                world.translate.z,
                rows[0][0],
                rows[0][1],
                rows[0][2],
                rows[1][0],
                rows[1][1],
                rows[1][2]);
        }

        inline std::string formatLeftCarryArmSample(const char* stage, const LeftCarryArmSample& sample)
        {
            std::string out;
            if (!sample.valid) {
                out += stage;
                out += ".valid=0";
                return out;
            }
            for (std::size_t index = 0; index < LEFT_CARRY_ARM_BONES.size(); ++index) {
                if (index != 0) {
                    out += ' ';
                }
                out += stage;
                out += '.';
                out += LEFT_CARRY_ARM_BONES[index].second;
                out += '=';
                out += formatLeftCarryArmFrame(sample.bones[index]);
            }
            return out;
        }

        /*
         * Conjugates a transform across the lateral mirror: M o X o M with
         * M = diag(-1, 1, 1). Reflections are involutions with symmetric
         * matrices, so the diagonal form is convention-proof and composed in
         * pairs every final rotation stays proper. Shared by the left-firing
         * and right-support mirrored-seat builders.
         */
        inline RE::NiTransform conjugateAcrossLateralMirror(
            const RE::NiTransform& transform)
        {
            RE::NiTransform lateralMirror{};
            lateralMirror.MakeIdentity();
            lateralMirror.rotate.entry[0][0] = -1.0f;
            return transform_math::composeTransforms(
                lateralMirror,
                transform_math::composeTransforms(transform, lateralMirror));
        }
    }

    // Implementation TUs resolve the helper names unqualified, exactly as
    // they did inside the original anonymous namespace.
    using namespace two_handed_grip_internal;

    struct TwoHandedGrip::FingerPoseSolveScratch
    {
        struct HandScratch
        {
            std::array<std::vector<RankedSupportGripTriangle>,
                kSupportGripFingerLaneCount + 1>
                rankings;
            std::vector<TriangleData> localTriangles;
            std::vector<TriangleData> worldTriangles;
            grab_finger_pose_runtime::FingerPoseTriangleSpatialIndex spatialIndex;
        };

        std::array<HandScratch, 2> hands{};
    };
}
