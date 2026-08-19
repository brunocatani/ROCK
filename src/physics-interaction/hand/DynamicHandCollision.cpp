#include "physics-interaction/hand/DynamicHandCollision.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/body/BodyBoneColliderSet.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/grab/GrabInertiaPolicy.h"
#include "physics-interaction/hand/DynamicHandCollisionKinematics.h"
#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/native/havok/HavokMaterialRegistry.h"
#include "physics-interaction/native/havok/HavokRefCount.h"
#include "physics-interaction/native/havok/HavokRuntime.h"
#include "physics-interaction/native/query/PhysicsScale.h"
#include "physics-interaction/native/query/PhysicsUtils.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/visual/PreFrikHandAuthorityPolicy.h"
#include "physics-interaction/weapon/DynamicWeaponCollisionPolicy.h"

#include <algorithm>
#include <cmath>

namespace rock
{
    namespace
    {
        constexpr const char* RIGHT_DYNAMIC_HAND_TAG = "ROCK_DynamicHand_Right";
        constexpr const char* LEFT_DYNAMIC_HAND_TAG = "ROCK_DynamicHand_Left";
        constexpr const char* SURFACE_FINGER_POSE_TAG =
            "ROCK_SurfaceFingerCollision";
        constexpr const char* SURFACE_MESH_GRAB_POSE_TAG =
            "ROCK_SurfaceMeshGrab";
        constexpr std::uint32_t kDynamicRightHandProxyCollisionGroup = 0x000C;
        constexpr std::uint32_t kDynamicLeftHandProxyCollisionGroup = 0x000E;
        static_assert(kDynamicRightHandProxyCollisionGroup != kDynamicLeftHandProxyCollisionGroup);
        constexpr std::uint32_t kRaiseManifoldProcessedEvents = 0x40u;
        constexpr std::uint32_t kRebuildBodyCollisionState = 0u;
        constexpr int kSurfaceLatchVisualPriority = 100;
        constexpr float kCompoundContactRetentionSeconds = 1.0f / 90.0f;

        const char* dynamicHandTag(bool isLeft)
        {
            return isLeft ? LEFT_DYNAMIC_HAND_TAG : RIGHT_DYNAMIC_HAND_TAG;
        }

        std::size_t handIndex(bool isLeft)
        {
            return isLeft ? 1u : 0u;
        }

        std::uint32_t dynamicHandProxyFilterInfo(
            bool isLeft,
            bool suppressCollision = false)
        {
            const auto collisionGroup = isLeft ?
                kDynamicLeftHandProxyCollisionGroup :
                kDynamicRightHandProxyCollisionGroup;
            const auto baseFilter =
                (collisionGroup << 16) |
                (collision_layer_policy::dynamicHandProxyLayerForHand(isLeft) &
                    collision_layer_policy::FO4_LAYER_FILTER_MASK);
            return suppressCollision ?
                (baseFilter | collision_suppression_registry::kSuppressionNoCollideBit) :
                baseFilter;
        }

        bool isFinitePoint(const RE::NiPoint3& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
        }

        bool isFiniteTransform(const RE::NiTransform& transform)
        {
            if (!isFinitePoint(transform.translate) || !std::isfinite(transform.scale)) {
                return false;
            }
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(transform.rotate.entry[row][column])) {
                        return false;
                    }
                }
            }
            return true;
        }

        bool makeCompoundChildTransform(
            const RE::NiTransform& shapeInContactBody,
            havok_compound_shape_builder::ChildTransform& outTransform)
        {
            outTransform = {};
            if (!isFiniteTransform(shapeInContactBody) ||
                std::abs(shapeInContactBody.scale - 1.0f) > 0.0001f) {
                return false;
            }
            const float gameToHavok = physics_scale::gameToHavok();
            if (!physics_scale::isUsableScale(gameToHavok)) {
                return false;
            }
            // Ni basis rows map directly to hkTransformf basis columns.
            outTransform.column0 = {
                shapeInContactBody.rotate.entry[0][0],
                shapeInContactBody.rotate.entry[0][1],
                shapeInContactBody.rotate.entry[0][2],
                0.0f,
            };
            outTransform.column1 = {
                shapeInContactBody.rotate.entry[1][0],
                shapeInContactBody.rotate.entry[1][1],
                shapeInContactBody.rotate.entry[1][2],
                0.0f,
            };
            outTransform.column2 = {
                shapeInContactBody.rotate.entry[2][0],
                shapeInContactBody.rotate.entry[2][1],
                shapeInContactBody.rotate.entry[2][2],
                0.0f,
            };
            outTransform.translation = {
                shapeInContactBody.translate.x * gameToHavok,
                shapeInContactBody.translate.y * gameToHavok,
                shapeInContactBody.translate.z * gameToHavok,
                1.0f,
            };
            return true;
        }

        /*
         * Generated hand collider frames author their basis axes as stored
         * matrix COLUMNS (HandColliderTypes::matrixFromAxes), while
         * TransformMath scene NiTransforms store axes as ROWS. Every relative
         * or scene composition below (compound child frames, post-solve child
         * reconstruction, render follow, latch relations) runs in the scene
         * row convention — the exact contract the proven weapon compound uses
         * (makeContactBodyTarget / reconstructWeaponRoot). Composing across
         * the two conventions transposes the rotation delta, and a transposed
         * rotation IS its inverse: the rendered hand rotates exactly opposite
         * to the solver correction. That was the reverted first compound
         * attempt's failure. The conversion is an involution (transpose the
         * stored rotation, keep translation/scale); the queued body DRIVE
         * target and the raw live-body READBACK stay in the collider column
         * convention, matching the keyframed collider drive contract.
         */
        RE::NiTransform colliderFrameToSceneFrame(
            const RE::NiTransform& colliderFrame)
        {
            RE::NiTransform result = colliderFrame;
            result.rotate =
                transform_math::transposeRotation(colliderFrame.rotate);
            return result;
        }

        RE::NiTransform sceneFrameToColliderFrame(
            const RE::NiTransform& sceneFrame)
        {
            return colliderFrameToSceneFrame(sceneFrame);
        }

        /*
         * Anatomical response direction from the contact geometry, evaluated
         * in the palm collider frame: local X = along the fingers, local
         * Y = palm depth (+Y back of hand, -Y palm face), local Z =
         * cross-palm (HandColliderTypes::makePalmBoxHullPoints). The blocked
         * hand is pushed back along the contact deviation, so the touched
         * surface lies along MINUS the deviation. A palm-face touch
         * (-Y dominant) opens/spreads the fingers onto the surface; a
         * fingertip push (+X dominant) curls them; a cross-palm (Z-dominant)
         * touch, a back-of-hand touch, or a -X pull leaves the fingers
         * exactly as they are.
         */
        std::int8_t classifySurfaceFingerDirection(
            const RE::NiTransform& palmSceneFrame,
            const RE::NiPoint3& deviationWorldGame)
        {
            const RE::NiPoint3 deviationLocal =
                transform_math::worldVectorToLocal(
                    palmSceneFrame,
                    deviationWorldGame);
            const RE::NiPoint3 touchLocal{
                -deviationLocal.x,
                -deviationLocal.y,
                -deviationLocal.z,
            };
            if (!std::isfinite(touchLocal.x) ||
                !std::isfinite(touchLocal.y) ||
                !std::isfinite(touchLocal.z)) {
                return 0;
            }
            const float absX = std::abs(touchLocal.x);
            const float absY = std::abs(touchLocal.y);
            const float absZ = std::abs(touchLocal.z);
            if (absY >= absX && absY >= absZ) {
                return touchLocal.y < 0.0f ? std::int8_t{ 1 } :
                                             std::int8_t{ 0 };
            }
            if (absX >= absY && absX >= absZ) {
                return touchLocal.x > 0.0f ? std::int8_t{ -1 } :
                                             std::int8_t{ 0 };
            }
            return 0;
        }

        /*
         * A dynamic compound must not rely on the engine's default mass
         * derivation: the weapon compound needed explicit envelope inertia to
         * behave (its default multi-hull derivation was unusable), and the
         * first hand-compound attempt shipped without this step. Mirror the
         * weapon contract: keep the engine-derived inverse mass, override the
         * three packed inverse-inertia axes with a solid-box envelope over the
         * children, normalize for solver stability, rebuild, and re-assert.
         */
        bool applyHandCompoundEnvelopeMassProperties(
            RE::hknpWorld* world,
            const RE::hknpBodyId bodyId,
            const bool isLeft,
            const RE::NiPoint3& childBoundsMinGame,
            const RE::NiPoint3& childBoundsMaxGame)
        {
            const auto geometry =
                dynamic_weapon_collision_policy::makeBoundingBoxGeometry(
                    childBoundsMinGame,
                    childBoundsMaxGame);
            const float bodyMass = std::clamp(
                std::isfinite(
                    g_rockConfig.rockHandCollisionDynamicCompoundMass) ?
                    g_rockConfig.rockHandCollisionDynamicCompoundMass :
                    2.0f,
                0.1f,
                50.0f);
            const auto envelope =
                dynamic_weapon_collision_policy::makeBoundingBoxMassProperties(
                    geometry,
                    1.0f,
                    0.0f,
                    physics_scale::gameToHavok(),
                    bodyMass);
            if (!envelope.valid) {
                ROCK_LOG_ERROR(
                    Hand,
                    "{} dynamic hand compound envelope mass properties invalid: body={} min=({:.2f},{:.2f},{:.2f}) max=({:.2f},{:.2f},{:.2f}) mass={:.3f}",
                    isLeft ? "Left" : "Right",
                    bodyId.value,
                    childBoundsMinGame.x,
                    childBoundsMinGame.y,
                    childBoundsMinGame.z,
                    childBoundsMaxGame.x,
                    childBoundsMaxGame.y,
                    childBoundsMaxGame.z,
                    bodyMass);
                return false;
            }

            const float inverseInertiaMultiplier = g_rockConfig.
                rockHandCollisionDynamicInverseInertiaMultiplier;
            if (!std::isfinite(inverseInertiaMultiplier) ||
                inverseInertiaMultiplier <= 0.0f) {
                ROCK_LOG_ERROR(
                    Hand,
                    "{} dynamic hand inverse-inertia multiplier invalid: body={} multiplier={}",
                    isLeft ? "Left" : "Right",
                    bodyId.value,
                    inverseInertiaMultiplier);
                return false;
            }

            const auto normalizedInertia =
                grab_inertia_policy::normalizeInverseInertiaAxesForGrab(
                    envelope.inverseInertia.x * inverseInertiaMultiplier,
                    envelope.inverseInertia.y * inverseInertiaMultiplier,
                    envelope.inverseInertia.z * inverseInertiaMultiplier,
                    g_rockConfig.rockGrabMaxInertiaRatio,
                    g_rockConfig.rockGrabMinInertia);
            if (!normalizedInertia.valid) {
                ROCK_LOG_ERROR(
                    Hand,
                    "{} dynamic hand compound inertia normalization failed: body={}",
                    isLeft ? "Left" : "Right",
                    bodyId.value);
                return false;
            }

            const auto initialMotion = havok_runtime::snapshotBody(world, bodyId);
            if (!initialMotion.valid || !initialMotion.motion) {
                ROCK_LOG_ERROR(
                    Hand,
                    "{} dynamic hand compound motion unavailable for mass properties: body={}",
                    isLeft ? "Left" : "Right",
                    bodyId.value);
                return false;
            }

            auto* initialPacked = reinterpret_cast<std::int16_t*>(
                reinterpret_cast<char*>(initialMotion.motion) +
                MOTION_PACKED_INERTIA_OFFSET);
            const std::int16_t desiredPackedInertia[3] = {
                repackBfloat16(normalizedInertia.normalized[0]),
                repackBfloat16(normalizedInertia.normalized[1]),
                repackBfloat16(normalizedInertia.normalized[2]),
            };
            const std::int16_t desiredPackedMass = initialPacked[3];
            if (desiredPackedInertia[0] <= 0 || desiredPackedInertia[1] <= 0 ||
                desiredPackedInertia[2] <= 0 || desiredPackedMass <= 0) {
                ROCK_LOG_ERROR(
                    Hand,
                    "{} dynamic hand compound packed mass properties invalid: body={} inertia=[{},{},{}] inverseMass={}",
                    isLeft ? "Left" : "Right",
                    bodyId.value,
                    desiredPackedInertia[0],
                    desiredPackedInertia[1],
                    desiredPackedInertia[2],
                    desiredPackedMass);
                return false;
            }

            initialPacked[0] = desiredPackedInertia[0];
            initialPacked[1] = desiredPackedInertia[1];
            initialPacked[2] = desiredPackedInertia[2];
            if (!havok_runtime::rebuildMotionMassProperties(
                    world,
                    initialMotion.motionIndex)) {
                ROCK_LOG_ERROR(
                    Hand,
                    "{} dynamic hand compound mass-properties rebuild failed: body={} motion={}",
                    isLeft ? "Left" : "Right",
                    bodyId.value,
                    initialMotion.motionIndex);
                return false;
            }

            const auto rebuiltMotion = havok_runtime::snapshotBody(world, bodyId);
            if (!rebuiltMotion.valid || !rebuiltMotion.motion ||
                rebuiltMotion.motionIndex != initialMotion.motionIndex) {
                ROCK_LOG_ERROR(
                    Hand,
                    "{} dynamic hand compound motion changed during mass-properties rebuild: body={} before={} after={}",
                    isLeft ? "Left" : "Right",
                    bodyId.value,
                    initialMotion.motionIndex,
                    rebuiltMotion.motionIndex);
                return false;
            }

            auto* rebuiltPacked = reinterpret_cast<std::int16_t*>(
                reinterpret_cast<char*>(rebuiltMotion.motion) +
                MOTION_PACKED_INERTIA_OFFSET);
            rebuiltPacked[0] = desiredPackedInertia[0];
            rebuiltPacked[1] = desiredPackedInertia[1];
            rebuiltPacked[2] = desiredPackedInertia[2];
            rebuiltPacked[3] = desiredPackedMass;

            ROCK_LOG_INFO(
                Hand,
                "{} dynamic hand compound envelope mass properties: body={} motion={} halfHavok=({:.4f},{:.4f},{:.4f}) appliedInverseInertia=({:.6f},{:.6f},{:.6f}) multiplier={:.3f} inverseMass={:.6f} ratio={:.2f}->{:.2f}",
                isLeft ? "Left" : "Right",
                bodyId.value,
                rebuiltMotion.motionIndex,
                envelope.halfExtentsHavok.x,
                envelope.halfExtentsHavok.y,
                envelope.halfExtentsHavok.z,
                unpackBfloat16(rebuiltPacked[0]),
                unpackBfloat16(rebuiltPacked[1]),
                unpackBfloat16(rebuiltPacked[2]),
                inverseInertiaMultiplier,
                unpackBfloat16(rebuiltPacked[3]),
                normalizedInertia.originalRatio,
                normalizedInertia.normalizedRatio);
            return true;
        }

        float pointLength(const RE::NiPoint3& value)
        {
            const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
            return std::isfinite(lengthSquared) && lengthSquared >= 0.0f ? std::sqrt(lengthSquared) : 0.0f;
        }

        RE::NiPoint3 subtractPoints(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            return RE::NiPoint3{
                lhs.x - rhs.x,
                lhs.y - rhs.y,
                lhs.z - rhs.z,
            };
        }

        const dynamic_hand_twin::TwinSlotFrame* twinFrameForSlot(
            const dynamic_hand_twin::TwinTargets& handTwins,
            const dynamic_hand_twin::ForearmTwinTargets& forearmTwins,
            bool isLeft,
            std::size_t bodyIndex)
        {
            if (bodyIndex == DynamicHandCollisionRuntime::kPalmSlot) {
                return &handTwins.palm;
            }
            if (bodyIndex < DynamicHandCollisionRuntime::kFirstForearmSlot) {
                const std::size_t fingerIndex =
                    dynamic_hand_collision_telemetry::fingerIndexForBodyIndex(
                        bodyIndex);
                const std::size_t segmentIndex =
                    dynamic_hand_collision_telemetry::
                        fingerSegmentIndexForBodyIndex(bodyIndex);
                return fingerIndex < handTwins.fingers.size() &&
                               segmentIndex <
                                   handTwins.fingers[fingerIndex].size() ?
                    &handTwins.fingers[fingerIndex][segmentIndex] :
                    nullptr;
            }

            const std::size_t forearmIndex = bodyIndex - DynamicHandCollisionRuntime::kFirstForearmSlot;
            const auto& sideForearms = forearmTwins.forHand(isLeft);
            return forearmIndex < sideForearms.size() ? &sideForearms[forearmIndex] : nullptr;
        }

        float dotPoints(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
        }

        bool buildPoseFingerSegmentCenters(
            const RE::NiTransform& rawHandWorld,
            const dynamic_hand_twin::TwinTargets& handTwins,
            const frik_visual_authority::FingerLocalTransformOverride& pose,
            std::array<RE::NiPoint3,
                hand_collider_semantics::kHandFingerRoleCount>& outCenters)
        {
            constexpr std::uint16_t kFullFingerMask = 0x7FFFu;
            if ((pose.enabledMask & kFullFingerMask) != kFullFingerMask ||
                !isFiniteTransform(rawHandWorld)) {
                return false;
            }

            for (std::size_t finger = 0;
                 finger < hand_collider_semantics::kHandFingerCount;
                 ++finger) {
                const std::size_t baseIndex =
                    finger * hand_collider_semantics::kHandFingerSegmentCount;
                const RE::NiTransform baseWorld =
                    transform_math::composeTransforms(
                        rawHandWorld,
                        pose.localTransforms[baseIndex]);
                const RE::NiTransform middleWorld =
                    transform_math::composeTransforms(
                        baseWorld,
                        pose.localTransforms[baseIndex + 1]);
                const RE::NiTransform tipWorld =
                    transform_math::composeTransforms(
                        middleWorld,
                        pose.localTransforms[baseIndex + 2]);
                if (!isFiniteTransform(baseWorld) ||
                    !isFiniteTransform(middleWorld) ||
                    !isFiniteTransform(tipWorld)) {
                    return false;
                }

                outCenters[baseIndex] = RE::NiPoint3{
                    (baseWorld.translate.x + middleWorld.translate.x) * 0.5f,
                    (baseWorld.translate.y + middleWorld.translate.y) * 0.5f,
                    (baseWorld.translate.z + middleWorld.translate.z) * 0.5f,
                };
                outCenters[baseIndex + 1] = RE::NiPoint3{
                    (middleWorld.translate.x + tipWorld.translate.x) * 0.5f,
                    (middleWorld.translate.y + tipWorld.translate.y) * 0.5f,
                    (middleWorld.translate.z + tipWorld.translate.z) * 0.5f,
                };
                const auto& tipFrame =
                    handTwins.fingers[finger][static_cast<std::size_t>(
                        hand_collider_semantics::HandFingerSegment::Tip)];
                if (!tipFrame.valid || !std::isfinite(tipFrame.length) ||
                    tipFrame.length <= 0.0f) {
                    return false;
                }
                const RE::NiPoint3 tipOffset =
                    transform_math::localVectorToWorld(
                        tipWorld,
                        RE::NiPoint3{ tipFrame.length * 0.5f, 0.0f, 0.0f });
                outCenters[baseIndex + 2] =
                    tipWorld.translate + tipOffset;
            }
            return true;
        }

        /*
         * World-space closing/opening probe travel measured around the
         * CURRENT flexion pose. The surface response re-baselines on the
         * live open values every frame so sustained blocked contact keeps
         * walking the fingers toward a fist (or fully spread); the probes
         * must be measured around that same pose or the travel directions
         * go stale as the hand curls away from the capture pose. Near the
         * anatomical stops the clamped probe collapses to zero travel,
         * which the policy reads as "not helpful" — the curl parks there
         * without any special-casing.
         */
        bool computeSurfaceFingerProbeTravel(
            const bool isLeft,
            const RE::NiTransform& rawHandWorld,
            const dynamic_hand_twin::TwinTargets& handTwins,
            const std::array<float,
                hand_collider_semantics::kHandFingerCount>& openValues,
            const float probeDeltaOpenUnits,
            std::array<RE::NiPoint3,
                hand_collider_semantics::kHandFingerRoleCount>&
                outClosingTravelWorld,
            std::array<RE::NiPoint3,
                hand_collider_semantics::kHandFingerRoleCount>&
                outOpeningTravelWorld)
        {
            auto closingOpenValues = openValues;
            auto openingOpenValues = openValues;
            for (std::size_t finger = 0;
                 finger < hand_collider_semantics::kHandFingerCount;
                 ++finger) {
                closingOpenValues[finger] = std::max(
                    0.0f,
                    closingOpenValues[finger] - probeDeltaOpenUnits);
                openingOpenValues[finger] = std::min(
                    1.0f,
                    openingOpenValues[finger] + probeDeltaOpenUnits);
            }

            frik_visual_authority::FingerLocalTransformOverride currentLocals{};
            frik_visual_authority::FingerLocalTransformOverride closingLocals{};
            frik_visual_authority::FingerLocalTransformOverride openingLocals{};
            const auto hand = frik_visual_authority::handFromBool(isLeft);
            if (!frik_visual_authority::getHandPoseLocalTransformsForPose(
                    hand,
                    frik_visual_authority::makeHandPoseDataFromJointValues(
                        grab_finger_pose_math::expandFingerCurlsToJointValues(
                            openValues)),
                    &currentLocals) ||
                !frik_visual_authority::getHandPoseLocalTransformsForPose(
                    hand,
                    frik_visual_authority::makeHandPoseDataFromJointValues(
                        grab_finger_pose_math::expandFingerCurlsToJointValues(
                            closingOpenValues)),
                    &closingLocals) ||
                !frik_visual_authority::getHandPoseLocalTransformsForPose(
                    hand,
                    frik_visual_authority::makeHandPoseDataFromJointValues(
                        grab_finger_pose_math::expandFingerCurlsToJointValues(
                            openingOpenValues)),
                    &openingLocals)) {
                return false;
            }

            std::array<RE::NiPoint3,
                hand_collider_semantics::kHandFingerRoleCount>
                currentCenters{};
            std::array<RE::NiPoint3,
                hand_collider_semantics::kHandFingerRoleCount>
                closingCenters{};
            std::array<RE::NiPoint3,
                hand_collider_semantics::kHandFingerRoleCount>
                openingCenters{};
            if (!buildPoseFingerSegmentCenters(
                    rawHandWorld,
                    handTwins,
                    currentLocals,
                    currentCenters) ||
                !buildPoseFingerSegmentCenters(
                    rawHandWorld,
                    handTwins,
                    closingLocals,
                    closingCenters) ||
                !buildPoseFingerSegmentCenters(
                    rawHandWorld,
                    handTwins,
                    openingLocals,
                    openingCenters)) {
                return false;
            }

            for (std::size_t linearIndex = 0;
                 linearIndex < hand_collider_semantics::kHandFingerRoleCount;
                 ++linearIndex) {
                outClosingTravelWorld[linearIndex] = subtractPoints(
                    closingCenters[linearIndex],
                    currentCenters[linearIndex]);
                outOpeningTravelWorld[linearIndex] = subtractPoints(
                    openingCenters[linearIndex],
                    currentCenters[linearIndex]);
                if (!isFinitePoint(outClosingTravelWorld[linearIndex]) ||
                    !isFinitePoint(outOpeningTravelWorld[linearIndex])) {
                    return false;
                }
            }
            return true;
        }

        /*
         * Contact-noise smoothing for the rendered hand: the solver resolves a
         * driven-into-surface compound slightly differently each substep, and the
         * raw deviation twitch is visible while the hand rests still. The
         * exponential filter only shapes CONTACT deviations (free space is
         * exactly zero and gated before application), so tracking latency is
         * untouched. speed <= 0 disables.
         */
        RE::NiPoint3 smoothAppliedDeviation(const RE::NiPoint3& applied, const RE::NiPoint3& target, float smoothingSpeed, float deltaSeconds)
        {
            if (!std::isfinite(smoothingSpeed) || smoothingSpeed <= 0.0f) {
                return target;
            }
            const float dt = std::clamp(std::isfinite(deltaSeconds) ? deltaSeconds : (1.0f / 90.0f), 0.0f, 0.1f);
            const float alpha = std::clamp(1.0f - std::exp(-smoothingSpeed * dt), 0.0f, 1.0f);
            return RE::NiPoint3{
                applied.x + (target.x - applied.x) * alpha,
                applied.y + (target.y - applied.y) * alpha,
                applied.z + (target.z - applied.z) * alpha,
            };
        }
    }

    void DynamicHandCollisionRuntime::storeAtomicTransform(
        AtomicTransform& target,
        const RE::NiTransform& value)
    {
        std::size_t index = 0;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                target.rotation[index++].store(
                    value.rotate.entry[row][column],
                    std::memory_order_relaxed);
            }
        }
        target.translation[0].store(value.translate.x, std::memory_order_relaxed);
        target.translation[1].store(value.translate.y, std::memory_order_relaxed);
        target.translation[2].store(value.translate.z, std::memory_order_relaxed);
        target.scale.store(value.scale, std::memory_order_relaxed);
    }

    RE::NiTransform DynamicHandCollisionRuntime::loadAtomicTransform(
        const AtomicTransform& source)
    {
        RE::NiTransform value{};
        std::size_t index = 0;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                value.rotate.entry[row][column] =
                    source.rotation[index++].load(std::memory_order_relaxed);
            }
        }
        value.translate = {
            source.translation[0].load(std::memory_order_relaxed),
            source.translation[1].load(std::memory_order_relaxed),
            source.translation[2].load(std::memory_order_relaxed),
        };
        value.scale = source.scale.load(std::memory_order_relaxed);
        return value;
    }

    void DynamicHandCollisionRuntime::publishPhysicsTelemetry(ProxySlot& slot, const PhysicsTelemetrySample& sample)
    {
        auto& telemetry = slot.physicsTelemetry;
        telemetry.sequence.fetch_add(1, std::memory_order_acq_rel);  // odd: write in progress
        storeAtomicTransform(telemetry.requestedTargetWorld, sample.requestedTargetWorld);
        storeAtomicTransform(telemetry.commandedTargetWorld, sample.commandedTargetWorld);
        storeAtomicTransform(telemetry.liveBodyWorld, sample.liveBodyWorld);
        telemetry.requestedX.store(sample.requestedTargetWorldGame.x, std::memory_order_relaxed);
        telemetry.requestedY.store(sample.requestedTargetWorldGame.y, std::memory_order_relaxed);
        telemetry.requestedZ.store(sample.requestedTargetWorldGame.z, std::memory_order_relaxed);
        telemetry.commandedX.store(sample.commandedTargetWorldGame.x, std::memory_order_relaxed);
        telemetry.commandedY.store(sample.commandedTargetWorldGame.y, std::memory_order_relaxed);
        telemetry.commandedZ.store(sample.commandedTargetWorldGame.z, std::memory_order_relaxed);
        telemetry.liveX.store(sample.liveBodyWorldGame.x, std::memory_order_relaxed);
        telemetry.liveY.store(sample.liveBodyWorldGame.y, std::memory_order_relaxed);
        telemetry.liveZ.store(sample.liveBodyWorldGame.z, std::memory_order_relaxed);
        telemetry.targetVelocityX.store(sample.targetVelocityWorldGameUnitsPerSecond.x, std::memory_order_relaxed);
        telemetry.targetVelocityY.store(sample.targetVelocityWorldGameUnitsPerSecond.y, std::memory_order_relaxed);
        telemetry.targetVelocityZ.store(sample.targetVelocityWorldGameUnitsPerSecond.z, std::memory_order_relaxed);
        telemetry.approachSpeed.store(sample.approachSpeedGameUnitsPerSecond, std::memory_order_relaxed);
        telemetry.physicsDeltaSeconds.store(sample.physicsDeltaSeconds, std::memory_order_relaxed);
        telemetry.physicsRawDeltaSeconds.store(sample.physicsRawDeltaSeconds, std::memory_order_relaxed);
        telemetry.physicsRemainderDeltaSeconds.store(sample.physicsRemainderDeltaSeconds, std::memory_order_relaxed);
        telemetry.physicsAccumulatedDeltaSeconds.store(sample.physicsAccumulatedDeltaSeconds, std::memory_order_relaxed);
        telemetry.physicsSubstepProgress.store(sample.physicsSubstepProgress, std::memory_order_relaxed);
        telemetry.sourceGameFrameIndex.store(sample.sourceGameFrameIndex, std::memory_order_relaxed);
        telemetry.sourceQueueSequence.store(sample.sourceQueueSequence, std::memory_order_relaxed);
        telemetry.solveSequence.store(sample.solveSequence, std::memory_order_relaxed);
        telemetry.physicsSubstepCount.store(sample.physicsSubstepCount, std::memory_order_relaxed);
        telemetry.physicsSubstepIndex.store(sample.physicsSubstepIndex, std::memory_order_relaxed);
        telemetry.valid.store(sample.valid, std::memory_order_relaxed);
        telemetry.targetVelocityValid.store(sample.targetVelocityValid, std::memory_order_relaxed);
        telemetry.contactActive.store(sample.contactActive, std::memory_order_relaxed);
        telemetry.worldContactActive.store(sample.worldContactActive, std::memory_order_relaxed);
        telemetry.recoveryTeleport.store(sample.recoveryTeleport, std::memory_order_relaxed);
        telemetry.sequence.fetch_add(1, std::memory_order_release);  // even: complete sample
    }

    bool DynamicHandCollisionRuntime::readPhysicsTelemetry(
        const ProxySlot& slot,
        PhysicsTelemetrySample& outSample,
        std::uint64_t& outSequence)
    {
        constexpr int kMaxSnapshotAttempts = 4;
        const auto& telemetry = slot.physicsTelemetry;
        for (int attempt = 0; attempt < kMaxSnapshotAttempts; ++attempt) {
            const std::uint64_t begin = telemetry.sequence.load(std::memory_order_acquire);
            if ((begin & 1u) != 0) {
                continue;
            }

            PhysicsTelemetrySample sample{};
            sample.requestedTargetWorld =
                loadAtomicTransform(telemetry.requestedTargetWorld);
            sample.commandedTargetWorld =
                loadAtomicTransform(telemetry.commandedTargetWorld);
            sample.liveBodyWorld =
                loadAtomicTransform(telemetry.liveBodyWorld);
            sample.requestedTargetWorldGame = RE::NiPoint3{
                telemetry.requestedX.load(std::memory_order_relaxed),
                telemetry.requestedY.load(std::memory_order_relaxed),
                telemetry.requestedZ.load(std::memory_order_relaxed),
            };
            sample.commandedTargetWorldGame = RE::NiPoint3{
                telemetry.commandedX.load(std::memory_order_relaxed),
                telemetry.commandedY.load(std::memory_order_relaxed),
                telemetry.commandedZ.load(std::memory_order_relaxed),
            };
            sample.liveBodyWorldGame = RE::NiPoint3{
                telemetry.liveX.load(std::memory_order_relaxed),
                telemetry.liveY.load(std::memory_order_relaxed),
                telemetry.liveZ.load(std::memory_order_relaxed),
            };
            sample.targetVelocityWorldGameUnitsPerSecond = RE::NiPoint3{
                telemetry.targetVelocityX.load(std::memory_order_relaxed),
                telemetry.targetVelocityY.load(std::memory_order_relaxed),
                telemetry.targetVelocityZ.load(std::memory_order_relaxed),
            };
            sample.approachSpeedGameUnitsPerSecond = telemetry.approachSpeed.load(std::memory_order_relaxed);
            sample.physicsDeltaSeconds = telemetry.physicsDeltaSeconds.load(std::memory_order_relaxed);
            sample.physicsRawDeltaSeconds = telemetry.physicsRawDeltaSeconds.load(std::memory_order_relaxed);
            sample.physicsRemainderDeltaSeconds = telemetry.physicsRemainderDeltaSeconds.load(std::memory_order_relaxed);
            sample.physicsAccumulatedDeltaSeconds = telemetry.physicsAccumulatedDeltaSeconds.load(std::memory_order_relaxed);
            sample.physicsSubstepProgress = telemetry.physicsSubstepProgress.load(std::memory_order_relaxed);
            sample.sourceGameFrameIndex = telemetry.sourceGameFrameIndex.load(std::memory_order_relaxed);
            sample.sourceQueueSequence = telemetry.sourceQueueSequence.load(std::memory_order_relaxed);
            sample.solveSequence = telemetry.solveSequence.load(std::memory_order_relaxed);
            sample.physicsSubstepCount = telemetry.physicsSubstepCount.load(std::memory_order_relaxed);
            sample.physicsSubstepIndex = telemetry.physicsSubstepIndex.load(std::memory_order_relaxed);
            sample.valid = telemetry.valid.load(std::memory_order_relaxed);
            sample.targetVelocityValid = telemetry.targetVelocityValid.load(std::memory_order_relaxed);
            sample.contactActive = telemetry.contactActive.load(std::memory_order_relaxed);
            sample.worldContactActive = telemetry.worldContactActive.load(std::memory_order_relaxed);
            sample.recoveryTeleport = telemetry.recoveryTeleport.load(std::memory_order_relaxed);

            const std::uint64_t end = telemetry.sequence.load(std::memory_order_acquire);
            if (begin == end && (end & 1u) == 0) {
                outSample = sample;
                outSequence = end;
                return true;
            }
        }

        outSample = {};
        outSequence = 0;
        return false;
    }

    void DynamicHandCollisionRuntime::clearPhysicsContactState(ProxySlot& slot)
    {
        slot.droveThisSubstep = false;
        slot.lastPostSolveDeviationGame = {};
        slot.lastPostSolveDeviationValid = false;
        slot.lastPostSolveContact = false;
        slot.droveTargetVelocityGameUnitsPerSecond = {};
        slot.drovePhysicsDeltaSeconds = 0.0f;
        slot.droveSourceGameFrameIndex = 0;
        slot.droveSourceQueueSequence = 0;
        slot.droveTargetVelocityValid = false;
        slot.droveRecoveryTeleport = false;
        publishPhysicsTelemetry(slot, {});
    }

    void DynamicHandCollisionRuntime::updateHandHaptic(
        HandSlots& handSlots,
        dynamic_hand_collision_telemetry::HandSample& handTelemetry,
        bool authorityAllowsFeedback,
        float deltaSeconds)
    {
        handTelemetry.contactEntrySequence = handSlots.contactEntrySequenceAtomic.load(std::memory_order_acquire);
        handTelemetry.contactEntryApproachSpeedGameUnitsPerSecond =
            handSlots.contactEntryApproachSpeedAtomic.load(std::memory_order_relaxed);
        handTelemetry.entryContactMask = handSlots.contactEntryMaskAtomic.load(std::memory_order_relaxed);

        const dynamic_hand_collision_feedback::ContactPulseConfig config{
            .enabled = g_rockConfig.rockHandCollisionDynamicHapticsEnabled,
            .baseIntensity = g_rockConfig.rockHandCollisionDynamicHapticBaseIntensity,
            .maxIntensity = g_rockConfig.rockHandCollisionDynamicHapticMaxIntensity,
            .speedScale = g_rockConfig.rockHandCollisionDynamicHapticSpeedScale,
            .minApproachSpeedGameUnitsPerSecond = g_rockConfig.rockHandCollisionDynamicHapticMinApproachSpeedGameUnitsPerSecond,
            .cooldownSeconds = g_rockConfig.rockHandCollisionDynamicHapticCooldownSeconds,
        };
        const auto decision = dynamic_hand_collision_feedback::updateContactPulse(
            handSlots.hapticState,
            handTelemetry.contactEntrySequence,
            handTelemetry.contactEntryApproachSpeedGameUnitsPerSecond,
            deltaSeconds,
            authorityAllowsFeedback,
            config);
        if (!decision.fire) {
            return;
        }

        auto& pulse = _pendingHapticEvents.hands[handTelemetry.isLeft ? 1u : 0u];
        pulse.fire = true;
        pulse.isLeft = handTelemetry.isLeft;
        pulse.intensity = decision.intensity;
        pulse.approachSpeedGameUnitsPerSecond = decision.approachSpeedGameUnitsPerSecond;
        pulse.contactEntrySequence = decision.entrySequence;
        pulse.contactMask = handTelemetry.entryContactMask;
    }

    bool DynamicHandCollisionRuntime::getTelemetrySnapshot(dynamic_hand_collision_telemetry::Snapshot& outSnapshot) const
    {
        outSnapshot = _telemetrySnapshot;
        return outSnapshot.updateSequence != 0;
    }

    dynamic_hand_collision_telemetry::HapticEvents DynamicHandCollisionRuntime::consumeHapticEvents()
    {
        const auto events = _pendingHapticEvents;
        _pendingHapticEvents = {};
        return events;
    }

    bool DynamicHandCollisionRuntime::tryClassifyDynamicBodyContactSourceAtomic(
        const std::uint32_t bodyId,
        const std::uint32_t shapeKey,
        DynamicBodyContactSource& outSource) const noexcept
    {
        outSource = {};
        if (bodyId == hand_semantic_contact_state::kInvalidBodyId) {
            return false;
        }
        for (std::size_t hand = 0; hand < _hands.size(); ++hand) {
            const auto& handState = _hands[hand];
            if (handState.bodies[0].bodyIdAtomic.load(
                    std::memory_order_acquire) != bodyId) {
                continue;
            }
            const auto childIndex =
                handState.compoundShape.tryResolveChildIndex(shapeKey);
            if (!childIndex || *childIndex >= kBodiesPerHand) {
                return false;
            }
            outSource.valid = true;
            outSource.isLeft = hand == 1;
            outSource.slot = static_cast<std::uint8_t>(*childIndex);
            outSource.bodyId = bodyId;
            return true;
        }
        return false;
    }

    void DynamicHandCollisionRuntime::recordDynamicBodyContactCallback(
        const DynamicBodyContactSource& source,
        const bool otherIsHand,
        const bool otherIsWeapon) noexcept
    {
        if (!source.valid || source.slot >= kBodiesPerHand) {
            return;
        }
        const std::uint32_t slotBit =
            1u << static_cast<std::uint32_t>(source.slot);
        auto& hand = _hands[source.isLeft ? 1u : 0u];
        hand.pendingSolverContactMaskAtomic.fetch_or(
            slotBit,
            std::memory_order_release);
        if (!otherIsHand && !otherIsWeapon) {
            // Anything the compound can hit that is not the other hand or the
            // equipped weapon is world evidence (surface + car layers).
            hand.pendingWorldContactMaskAtomic.fetch_or(
                slotBit,
                std::memory_order_release);
        }
        if (!_dynamicInteractionsEnabledAtomic.load(
                std::memory_order_acquire)) {
            return;
        }
        if (otherIsHand) {
            hand.pendingOtherHandContactMaskAtomic.fetch_or(
                slotBit,
                std::memory_order_release);
        }
        if (otherIsWeapon) {
            hand.pendingWeaponContactMaskAtomic.fetch_or(
                slotBit,
                std::memory_order_release);
        }
    }

    bool DynamicHandCollisionRuntime::tryClassifySurfaceContactSourceAtomic(
        const std::uint32_t bodyId,
        const std::uint32_t shapeKey,
        dynamic_hand_surface_contact_state::ContactSource& outSource) const noexcept
    {
        outSource = {};
        if (bodyId == hand_semantic_contact_state::kInvalidBodyId) {
            return false;
        }

        for (std::size_t hand = 0; hand < _hands.size(); ++hand) {
            const auto& handState = _hands[hand];
            if (handState.bodies[0].bodyIdAtomic.load(
                    std::memory_order_acquire) != bodyId) {
                continue;
            }
            const auto childIndex =
                handState.compoundShape.tryResolveChildIndex(shapeKey);
            if (!childIndex || *childIndex >= kBodiesPerHand ||
                !dynamic_hand_collision_telemetry::
                    isSurfaceGrabSourceSlot(*childIndex)) {
                return false;
            }
            const std::size_t slot = *childIndex;
            const bool palm = slot == kPalmSlot;
            const std::size_t fingerIndex =
                dynamic_hand_collision_telemetry::
                    fingerIndexForBodyIndex(slot);
            const auto finger = palm ?
                hand_collider_semantics::HandFinger::None :
                static_cast<hand_collider_semantics::HandFinger>(
                    fingerIndex);
            const auto segment = palm ?
                hand_collider_semantics::HandFingerSegment::None :
                hand_collider_semantics::HandFingerSegment::Tip;
            outSource.valid = true;
            outSource.isLeft = hand == 1;
            outSource.slot = palm ? 0u : fingerIndex + 1u;
            outSource.role = palm ?
                hand_collider_semantics::HandColliderRole::PalmAnchor :
                hand_collider_semantics::roleForFingerSegment(
                    finger,
                    segment);
            outSource.finger = finger;
            outSource.segment = segment;
            outSource.bodyId = bodyId;
            return true;
        }
        return false;
    }

    void DynamicHandCollisionRuntime::recordSurfaceContactCallback(
        const dynamic_hand_surface_contact_state::ContactSource& source,
        const std::uint32_t otherBodyId,
        const hand_semantic_contact_state::SemanticContactVector* contactPointGame,
        const hand_semantic_contact_state::SemanticContactVector* contactNormalGame) noexcept
    {
        _surfaceImpulsePairSequenceAtomic.fetch_add(1, std::memory_order_release);
        _surfaceEligiblePairSequenceAtomic.fetch_add(1, std::memory_order_release);
        if (_surfaceContacts.record(
                source,
                otherBodyId,
                contactPointGame,
                contactNormalGame)) {
            _surfaceContactPublishSequenceAtomic.fetch_add(1, std::memory_order_release);
        }
    }

    void DynamicHandCollisionRuntime::recordSurfaceManifoldProcessedCallback(
        const dynamic_hand_surface_contact_state::ContactSource& source,
        const std::uint32_t otherBodyId,
        const bool otherLayerRead,
        const std::uint32_t otherLayer,
        const hand_semantic_contact_state::SemanticContactVector*
            contactPointGame,
        const hand_semantic_contact_state::SemanticContactVector*
            contactNormalGame) noexcept
    {
        _surfaceProcessedPairSequenceAtomic.fetch_add(1, std::memory_order_release);
        if (!otherLayerRead ||
            !collision_layer_policy::isDynamicHandProxySurfaceLayer(otherLayer)) {
            return;
        }
        _surfaceEligiblePairSequenceAtomic.fetch_add(1, std::memory_order_release);
        if (_surfaceContacts.record(
                source,
                otherBodyId,
                contactPointGame,
                contactNormalGame)) {
            _surfaceContactPublishSequenceAtomic.fetch_add(1, std::memory_order_release);
        }
    }

    hand_semantic_contact_state::SemanticContactCollection
    DynamicHandCollisionRuntime::collectFreshSurfaceContacts(
        const bool isLeft,
        const std::uint32_t maximumAgeFrames) const noexcept
    {
        return _surfaceContacts.collectFresh(isLeft, maximumAgeFrames);
    }

    bool DynamicHandCollisionRuntime::beginSurfaceLatch(
        const dynamic_hand_surface_contact_state::ContactSource& source,
        const std::uint32_t targetBodyId,
        RE::hknpWorld* world,
        const SurfaceLatchPresentation* presentation,
        SurfaceLatchFailure* outFailure)
    {
        if (outFailure) {
            *outFailure = SurfaceLatchFailure::None;
        }
        const auto reject = [&](const SurfaceLatchFailure failure) {
            if (outFailure) {
                *outFailure = failure;
            }
            return false;
        };

        const bool isLeft = source.isLeft;
        const std::uint32_t sourceBodyId = source.bodyId;
        if (!g_rockConfig.rockHandCollisionDynamicDrive ||
            !frik_visual_authority::isAvailable() ||
            !world || targetBodyId == hand_semantic_contact_state::kInvalidBodyId ||
            sourceBodyId == hand_semantic_contact_state::kInvalidBodyId ||
            sourceBodyId == targetBodyId) {
            return reject(SurfaceLatchFailure::PrerequisiteUnavailable);
        }

        if (!source.valid ||
            source.slot >= dynamic_hand_surface_contact_state::
                kContactRolesPerHand) {
            return reject(SurfaceLatchFailure::ContactSourceMismatch);
        }

        auto& handSlots = _hands[handIndex(isLeft)];
        if (handSlots.surfaceLatch.active ||
            !handSlots.lastPresentedHandWorldValid ||
            !isFiniteTransform(handSlots.lastPresentedHandWorld)) {
            return reject(SurfaceLatchFailure::HandTransformUnavailable);
        }

        const auto targetSnapshot = havok_runtime::snapshotBody(
            world,
            RE::hknpBodyId{ targetBodyId });
        RE::NiTransform targetWorld{};
        if (!targetSnapshot.valid || !targetSnapshot.body ||
            !havok_runtime::tryResolveLiveBodyWorldTransform(
                world,
                RE::hknpBodyId{ targetBodyId },
                targetWorld) ||
            !isFiniteTransform(targetWorld)) {
            return reject(SurfaceLatchFailure::TargetTransformUnavailable);
        }

        HandSlots::SurfaceLatch candidate{};
        candidate.targetBodyId = targetBodyId;
        candidate.targetBodyIdentity = targetSnapshot.body;
        candidate.targetCollisionIdentity = targetSnapshot.collisionObject;
        const bool meshPresentationRequested =
            presentation && presentation->valid &&
            isFiniteTransform(presentation->handWorld) &&
            isFinitePoint(presentation->meshAnchorWorld) &&
            isFinitePoint(presentation->meshNormalWorld) &&
            std::isfinite(
                presentation->shellToMeshDistanceGameUnits) &&
            presentation->shellToMeshDistanceGameUnits >= 0.0f;
        candidate.lastHandWorld = meshPresentationRequested ?
            presentation->handWorld :
            handSlots.lastPresentedHandWorld;
        const auto targetInverse = transform_math::invertTransform(targetWorld);
        candidate.handInTargetBody = transform_math::composeTransforms(
            targetInverse,
            candidate.lastHandWorld);

        bool sourceCaptured = false;
        /*
         * Mesh authority moves only the presented hand. The compound children keep
         * their already-solved shell transforms relative to the target, so the
         * oversized Havok shell remains a collision guard without becoming the
         * visible/API grab anchor. Translating the compound into the render mesh
         * would require body-wide no-collide and would discard nearby-world
         * collision fidelity while latched.
         */
        const auto& compoundOwner = handSlots.bodies[0];
        RE::NiTransform liveCompoundWorld{};
        if (!compoundOwner.created || compoundOwner.createdWorld != world ||
            compoundOwner.body.getBodyId().value != sourceBodyId ||
            !havok_runtime::tryResolveLiveBodyWorldTransform(
                world,
                compoundOwner.body.getBodyId(),
                liveCompoundWorld) ||
            !isFiniteTransform(liveCompoundWorld)) {
            return reject(SurfaceLatchFailure::SourceProxyUnavailable);
        }
        std::array<RE::NiTransform, kBodiesPerHand> childFrames{};
        {
            std::scoped_lock poseLock(handSlots.compoundPoseMutex);
            childFrames = handSlots.consumedChildInContactBodyGame;
        }
        // Latch relations are captured and recomposed entirely in the scene
        // convention; the drive-target selection converts back per frame.
        const RE::NiTransform liveCompoundSceneWorld =
            colliderFrameToSceneFrame(liveCompoundWorld);
        for (std::size_t bodyIndex = 0; bodyIndex < kBodiesPerHand; ++bodyIndex) {
            const RE::NiTransform proxyWorld =
                transform_math::composeTransforms(
                    liveCompoundSceneWorld,
                    childFrames[bodyIndex]);
            if (!isFiniteTransform(proxyWorld)) {
                continue;
            }
            candidate.lastProxyWorld[bodyIndex] = proxyWorld;
            candidate.proxyInTargetBody[bodyIndex] =
                transform_math::composeTransforms(
                    targetInverse,
                    proxyWorld);
            candidate.proxyRelationshipValid[bodyIndex] = true;
            sourceCaptured = true;
        }
        if (!sourceCaptured || !isFiniteTransform(candidate.handInTargetBody)) {
            return reject(SurfaceLatchFailure::SourceProxyUnavailable);
        }

        const bool meshAuthorityAccepted = meshPresentationRequested;

        if (meshAuthorityAccepted) {
            candidate.meshAuthoritative = true;
            candidate.meshAnchorWorld = presentation->meshAnchorWorld;
            candidate.meshNormalWorld = presentation->meshNormalWorld;
            candidate.meshAnchorInTargetBody =
                transform_math::worldPointToLocal(
                    targetWorld,
                    presentation->meshAnchorWorld);
            candidate.meshNormalInTargetBody =
                transform_math::worldVectorToLocal(
                    targetWorld,
                    presentation->meshNormalWorld);
            candidate.shellToMeshDistanceGameUnits =
                presentation->shellToMeshDistanceGameUnits;
            candidate.meshFingerJointValues =
                presentation->fingerJointValues;
            candidate.meshFingerContactMask =
                presentation->fingerContactMask;
            candidate.meshFingerPoseValid =
                presentation->fingerPoseValid;
            clearSurfaceFingerResponse(handSlots, isLeft);
        }

        candidate.active = true;
        handSlots.surfaceLatch = candidate;
        handSlots.appliedDeviation = {};
        handSlots.teleportRecoverySecondsRemaining = 0.0f;
        ROCK_LOG_INFO(
            Hand,
            "Dynamic surface latch acquired: hand={} sourceBody={} targetBody={} authority={} gap={:.2f}gu fingerPose={} contactMask=0x{:02X}",
            isLeft ? "left" : "right",
            sourceBodyId,
            targetBodyId,
            meshAuthorityAccepted ? "mesh" : "collision",
            candidate.shellToMeshDistanceGameUnits,
            candidate.meshFingerPoseValid ? "yes" : "no",
            candidate.meshFingerContactMask);
        return true;
    }

    void DynamicHandCollisionRuntime::endSurfaceLatch(
        const bool isLeft) noexcept
    {
        auto& handSlots = _hands[handIndex(isLeft)];
        if (!handSlots.surfaceLatch.active) {
            return;
        }
        const auto targetBodyId = handSlots.surfaceLatch.targetBodyId;
        const bool meshAuthoritative =
            handSlots.surfaceLatch.meshAuthoritative;
        clearSurfaceMeshPose(handSlots, isLeft);
        handSlots.surfaceLatch = {};
        handSlots.appliedDeviation = {};
        handSlots.teleportRecoverySecondsRemaining = 0.0f;
        if (handSlots.visualActive) {
            (void)frik_visual_authority::clearExternalHandWorldTransform(
                dynamicHandTag(isLeft),
                frik_visual_authority::handFromBool(isLeft));
            handSlots.visualActive = false;
        }
        ROCK_LOG_INFO(
            Hand,
            "Dynamic surface latch released: hand={} targetBody={} authority={}",
            isLeft ? "left" : "right",
            targetBodyId,
            meshAuthoritative ? "mesh" : "collision");
    }

    bool DynamicHandCollisionRuntime::isSurfaceLatchActive(
        const bool isLeft) const noexcept
    {
        return _hands[handIndex(isLeft)].surfaceLatch.active;
    }

    bool DynamicHandCollisionRuntime::isSurfaceLatchMeshAuthoritative(
        const bool isLeft) const noexcept
    {
        const auto& latch = _hands[handIndex(isLeft)].surfaceLatch;
        return latch.active && latch.meshAuthoritative;
    }

    bool DynamicHandCollisionRuntime::isSurfaceLatchMeshFingerPoseActive(
        const bool isLeft) const noexcept
    {
        const auto& latch = _hands[handIndex(isLeft)].surfaceLatch;
        return latch.active && latch.meshAuthoritative &&
               latch.meshFingerPoseValid && latch.meshPosePublished &&
               frik_visual_authority::isHandPoseTagActive(
                   SURFACE_MESH_GRAB_POSE_TAG,
                   frik_visual_authority::handFromBool(isLeft));
    }

    bool DynamicHandCollisionRuntime::getLastPresentedHandWorld(
        const bool isLeft,
        RE::NiTransform& outHandWorld) const noexcept
    {
        const auto& handSlots = _hands[handIndex(isLeft)];
        if (!handSlots.lastPresentedHandWorldValid ||
            !isFiniteTransform(handSlots.lastPresentedHandWorld)) {
            return false;
        }
        outHandWorld = handSlots.lastPresentedHandWorld;
        return true;
    }

    bool DynamicHandCollisionRuntime::ensureHandCreated(
        HandSlots& handSlots,
        bool isLeft,
        const PhysicsFrameContext& frame,
        const Hand& hand,
        const BodyBoneColliderSet& bodyBoneColliders,
        const std::array<const dynamic_hand_twin::TwinSlotFrame*,
            kBodiesPerHand>& twinFrames,
        const RE::NiTransform& compoundRootTarget,
        const std::array<RE::NiTransform, kBodiesPerHand>& driveTargets,
        std::uint64_t geometryGeneration)
    {
        auto& slot = handSlots.bodies[0];
        if (slot.created) {
            if (slot.createdWorld == frame.hknpWorld) {
                if (_transitionCollisionSuppressed ||
                    (slot.createdGeometryGeneration == geometryGeneration &&
                        !slot.rebuildRequestedAtomic.load(std::memory_order_acquire))) {
                    return true;
                }
            }
            retireHand(handSlots, frame.bhkWorld, isLeft);
        }

        if (!frame.hknpWorld || !frame.bhkWorld || geometryGeneration == 0 ||
            !isFiniteTransform(compoundRootTarget)) {
            return false;
        }
        for (std::size_t child = 0; child < kBodiesPerHand; ++child) {
            if (!twinFrames[child] || !twinFrames[child]->valid ||
                !isFiniteTransform(driveTargets[child])) {
                return false;
            }
        }

        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};

        std::array<RE::hknpShape*, kBodiesPerHand> childShapes{};
        std::array<havok_compound_shape_builder::CompoundChild,
            kBodiesPerHand> compoundChildren{};
        // Child relative frames are computed in the SCENE row convention so
        // makeCompoundChildTransform's row->hkTransform-column copy matches
        // the weapon compound contract.
        const auto compoundRootInverse = transform_math::invertTransform(
            colliderFrameToSceneFrame(compoundRootTarget));
        bool childrenValid = true;
        RE::NiPoint3 childBoundsMinGame{};
        RE::NiPoint3 childBoundsMaxGame{};
        for (std::size_t child = 0; child < kBodiesPerHand; ++child) {
            const auto& frameForChild = *twinFrames[child];
            childShapes[child] = child >= kFirstForearmSlot ?
                bodyBoneColliders.buildDynamicForearmTwinShape(frameForChild) :
                hand.buildDynamicTwinShape(
                    frameForChild,
                    child == kPalmSlot);
            RE::NiTransform childInBody = transform_math::composeTransforms(
                compoundRootInverse,
                colliderFrameToSceneFrame(driveTargets[child]));
            childInBody.scale = 1.0f;
            if (!childShapes[child] ||
                !makeCompoundChildTransform(
                    childInBody,
                    compoundChildren[child].transform)) {
                childrenValid = false;
                break;
            }
            compoundChildren[child].shape = childShapes[child];
            handSlots.childInContactBodyGame[child] = childInBody;

            // Conservative body-local envelope for the inertia box: each
            // child's center expanded by its authored reach in every axis.
            const float childReach =
                (std::isfinite(frameForChild.length) ?
                        std::max(frameForChild.length, 0.0f) * 0.5f : 0.0f) +
                (std::isfinite(frameForChild.radius) ?
                        std::max(frameForChild.radius, 0.0f) : 0.0f) +
                (std::isfinite(frameForChild.convexRadius) ?
                        std::max(frameForChild.convexRadius, 0.0f) : 0.0f);
            const RE::NiPoint3 childMin{
                childInBody.translate.x - childReach,
                childInBody.translate.y - childReach,
                childInBody.translate.z - childReach,
            };
            const RE::NiPoint3 childMax{
                childInBody.translate.x + childReach,
                childInBody.translate.y + childReach,
                childInBody.translate.z + childReach,
            };
            if (child == 0) {
                childBoundsMinGame = childMin;
                childBoundsMaxGame = childMax;
            } else {
                childBoundsMinGame.x = std::min(childBoundsMinGame.x, childMin.x);
                childBoundsMinGame.y = std::min(childBoundsMinGame.y, childMin.y);
                childBoundsMinGame.z = std::min(childBoundsMinGame.z, childMin.z);
                childBoundsMaxGame.x = std::max(childBoundsMaxGame.x, childMax.x);
                childBoundsMaxGame.y = std::max(childBoundsMaxGame.y, childMax.y);
                childBoundsMaxGame.z = std::max(childBoundsMaxGame.z, childMax.z);
            }
        }
        if (!childrenValid || !handSlots.compoundShape.create(compoundChildren)) {
            for (auto* childShape : childShapes) {
                if (childShape) {
                    havok_ref_count::release(childShape);
                }
            }
            handSlots.compoundShape.reset();
            ROCK_LOG_ERROR(
                Hand,
                "{} dynamic hand compound child construction failed generation={}",
                isLeft ? "Left" : "Right",
                geometryGeneration);
            return false;
        }
        for (auto* childShape : childShapes) {
            havok_ref_count::release(childShape);
        }

        const auto expectedFilterInfo = dynamicHandProxyFilterInfo(
            isLeft,
            _transitionCollisionSuppressed);
        if (!slot.body.create(
                frame.hknpWorld,
                frame.bhkWorld,
                handSlots.compoundShape.get(),
                expectedFilterInfo,
                havok_material_registry::registerGeneratedBodyMaterial(frame.hknpWorld),
                BethesdaMotionType::Dynamic,
                isLeft ? "ROCK_LeftHandDynamicCompound" :
                         "ROCK_RightHandDynamicCompound",
                kTrackedDynamicBodyCreationOptions)) {
            handSlots.compoundShape.reset();
            ROCK_LOG_ERROR(
                Hand,
                "{} dynamic hand compound body creation failed",
                isLeft ? "Left" : "Right");
            return false;
        }

        const auto proxyBodyId = slot.body.getBodyId();
        const auto createdBody = havok_runtime::snapshotBody(frame.hknpWorld, proxyBodyId);
        if (!createdBody.valid || createdBody.collisionFilterInfo != expectedFilterInfo) {
            ROCK_LOG_ERROR(
                Hand,
                "{} dynamic hand compound filter publication failed: body={} requested=0x{:08X} observed=0x{:08X} readable={}",
                isLeft ? "Left" : "Right",
                proxyBodyId.value,
                expectedFilterInfo,
                createdBody.collisionFilterInfo,
                createdBody.valid ? "yes" : "no");
            slot.body.retireDeferred(frame.bhkWorld);
            handSlots.compoundShape.reset();
            return false;
        }
        const bool processedManifoldFlagEnabled = havok_runtime::enableBodyFlags(
            frame.hknpWorld,
            proxyBodyId.value,
            kRaiseManifoldProcessedEvents,
            kRebuildBodyCollisionState);
        const auto flaggedBody = havok_runtime::snapshotBody(
            frame.hknpWorld,
            proxyBodyId);
        const bool processedManifoldEventsEnabled =
            processedManifoldFlagEnabled && flaggedBody.valid &&
            flaggedBody.body &&
            (flaggedBody.body->flags & kRaiseManifoldProcessedEvents) ==
                kRaiseManifoldProcessedEvents;
        if (!processedManifoldEventsEnabled) {
            ROCK_LOG_ERROR(
                Hand,
                "{} dynamic hand compound failed processed-manifold opt-in body={}",
                isLeft ? "Left" : "Right",
                proxyBodyId.value);
            slot.body.retireDeferred(frame.bhkWorld);
            handSlots.compoundShape.reset();
            return false;
        }

        if (!applyHandCompoundEnvelopeMassProperties(
                frame.hknpWorld,
                proxyBodyId,
                isLeft,
                childBoundsMinGame,
                childBoundsMaxGame)) {
            slot.body.retireDeferred(frame.bhkWorld);
            handSlots.compoundShape.reset();
            return false;
        }

        if (!placeGeneratedKeyframedBodyImmediately(
                slot.body,
                compoundRootTarget)) {
            slot.body.retireDeferred(frame.bhkWorld);
            handSlots.compoundShape.reset();
            return false;
        }

        slot.shape = nullptr;
        slot.createdWorld = frame.hknpWorld;
        slot.createdBhkWorld = frame.bhkWorld;
        slot.createdGeometryGeneration = geometryGeneration;
        slot.created = true;
        for (auto& semanticSlot : handSlots.bodies) {
            semanticSlot.bodyIdAtomic.store(
                proxyBodyId.value,
                std::memory_order_release);
        }
        slot.rebuildRequestedAtomic.store(false, std::memory_order_release);
        clearPhysicsContactState(slot);

        initializeGeneratedKeyframedBodyDriveState(
            slot.driveState,
            compoundRootTarget);
        {
            std::scoped_lock poseLock(handSlots.compoundPoseMutex);
            for (std::size_t child = 0; child < kBodiesPerHand; ++child) {
                handSlots.pendingCompoundChildTransforms[child] =
                    compoundChildren[child].transform;
                handSlots.consumedChildInContactBodyGame[child] =
                    handSlots.childInContactBodyGame[child];
            }
            handSlots.queuedCompoundPoseSequence = 1;
            handSlots.consumedCompoundPoseSequence = 1;
        }
        handSlots.compoundGeometryGeneration = geometryGeneration;

        ROCK_LOG_INFO(Hand,
            "{} animated dynamic hand compound created: body={} root=palm-keyframed-target children={} keyBits={} generation={} layer={} group={} manifoldEvents={}",
            isLeft ? "Left" : "Right",
            slot.body.getBodyId().value,
            handSlots.compoundShape.childCount(),
            handSlots.compoundShape.shapeKeyBitCount(),
            geometryGeneration,
            collision_layer_policy::dynamicHandProxyLayerForHand(isLeft),
            expectedFilterInfo >> 16,
            processedManifoldEventsEnabled ? "yes" : "no");
        return true;
    }

    bool DynamicHandCollisionRuntime::queueCompoundPose(
        HandSlots& handSlots,
        const RE::NiTransform& compoundRootTarget,
        const std::array<RE::NiTransform, kBodiesPerHand>& driveTargets)
    {
        if (!handSlots.bodies[0].created ||
            !isFiniteTransform(compoundRootTarget)) {
            return false;
        }
        const RE::NiTransform compoundRootInverse =
            transform_math::invertTransform(
                colliderFrameToSceneFrame(compoundRootTarget));
        std::array<havok_compound_shape_builder::ChildTransform,
            kBodiesPerHand> pendingTransforms{};
        std::array<RE::NiTransform, kBodiesPerHand> childFrames{};
        for (std::size_t child = 0; child < kBodiesPerHand; ++child) {
            RE::NiTransform childInBody = transform_math::composeTransforms(
                compoundRootInverse,
                colliderFrameToSceneFrame(driveTargets[child]));
            childInBody.scale = 1.0f;
            if (!makeCompoundChildTransform(
                    childInBody,
                    pendingTransforms[child])) {
                return false;
            }
            childFrames[child] = childInBody;
        }
        {
            std::scoped_lock poseLock(handSlots.compoundPoseMutex);
            handSlots.pendingCompoundChildTransforms = pendingTransforms;
            handSlots.childInContactBodyGame = childFrames;
            ++handSlots.queuedCompoundPoseSequence;
        }
        return true;
    }

    void DynamicHandCollisionRuntime::retireSlot(ProxySlot& slot, void* bhkWorld)
    {
        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        slot.bodyIdAtomic.store(
            hand_semantic_contact_state::kInvalidBodyId,
            std::memory_order_release);
        if (slot.created) {
            /*
             * Live-world teardown must go through the deferred-retirement queue:
             * the hknp broadphase can still reach this body until the next
             * physics step (see 2026-07-08 use-after-free lesson).
             */
            slot.body.retireDeferred(bhkWorld ? bhkWorld : slot.createdBhkWorld);
        }
        if (slot.shape) {
            havok_ref_count::release(slot.shape);
            slot.shape = nullptr;
        }
        clearGeneratedKeyframedBodyDriveState(slot.driveState);
        slot.created = false;
        slot.createdWorld = nullptr;
        slot.createdBhkWorld = nullptr;
        slot.createdGeometryGeneration = 0;
        slot.droveThisSubstep = false;
        slot.divergenceDwellSeconds = 0.0f;
        slot.commandedTargetGame = {};
        slot.requestedTargetGame = {};
        clearPhysicsContactState(slot);
        slot.teleportedAtomic.store(false, std::memory_order_release);
        slot.rebuildRequestedAtomic.store(false, std::memory_order_release);
    }

    void DynamicHandCollisionRuntime::retireHand(HandSlots& handSlots, void* bhkWorld, bool isLeft)
    {
        clearSurfaceMeshPose(handSlots, isLeft);
        clearVisual(handSlots, isLeft);
        clearSurfaceFingerResponse(handSlots, isLeft);
        for (auto& slot : handSlots.bodies) {
            retireSlot(slot, bhkWorld);
        }
        {
            std::scoped_lock poseLock(handSlots.compoundPoseMutex);
            handSlots.compoundShape.reset();
            handSlots.pendingCompoundChildTransforms = {};
            handSlots.childInContactBodyGame = {};
            handSlots.consumedChildInContactBodyGame = {};
            handSlots.queuedCompoundPoseSequence = 0;
            handSlots.consumedCompoundPoseSequence = 0;
        }
        handSlots.compoundGeometryGeneration = 0;
        handSlots.pendingSolverContactMaskAtomic.store(
            0,
            std::memory_order_release);
        handSlots.pendingWorldContactMaskAtomic.store(
            0,
            std::memory_order_release);
        handSlots.retainedSolverContactMask = 0;
        handSlots.retainedWorldContactMask = 0;
        handSlots.solverContactRetentionSeconds = 0.0f;
        handSlots.physicsContactActive = false;
        handSlots.contactEntrySequenceAtomic.store(0, std::memory_order_release);
        handSlots.contactEntryApproachSpeedAtomic.store(0.0f, std::memory_order_relaxed);
        handSlots.contactEntryMaskAtomic.store(0, std::memory_order_relaxed);
        handSlots.hapticState = {};
        handSlots.lastPresentedHandWorld = {};
        handSlots.lastPresentedHandWorldValid = false;
        handSlots.surfaceLatch = {};
    }

    void DynamicHandCollisionRuntime::clearVisual(HandSlots& handSlots, bool isLeft)
    {
        handSlots.preFrikContactAuthority = {};
        handSlots.appliedDeviation = {};
        handSlots.teleportRecoverySecondsRemaining = 0.0f;
        if (!handSlots.visualActive) {
            return;
        }
        (void)frik_visual_authority::clearExternalHandWorldTransform(dynamicHandTag(isLeft), frik_visual_authority::handFromBool(isLeft));
        handSlots.visualActive = false;
    }

    void DynamicHandCollisionRuntime::refreshContactVisualAuthorityBeforeFrik(
        const std::uint64_t currentSchedulerSequence,
        const bool rightRawHandValid,
        const RE::NiTransform& rightRawHandWorld,
        const bool leftRawHandValid,
        const RE::NiTransform& leftRawHandWorld,
        const float maximumRawMotionGameUnits)
    {
        const auto refreshHand = [&](const bool isLeft,
                                     const bool currentRawHandValid,
                                     const RE::NiTransform& currentRawHandWorld) {
            auto& handSlots = _hands[isLeft ? 1u : 0u];
            if (handSlots.surfaceLatch.active) {
                handSlots.preFrikContactAuthority = {};
                return;
            }

            const auto source = handSlots.preFrikContactAuthority;
            const bool sourceCurrent =
                source.valid &&
                handSlots.visualActive &&
                currentRawHandValid &&
                prefrik_hand_authority_policy::isImmediateSuccessor(
                    source.sourceSchedulerSequence,
                    currentSchedulerSequence);
            if (!sourceCurrent) {
                if (handSlots.visualActive || source.valid) {
                    clearVisual(handSlots, isLeft);
                }
                return;
            }

            const auto transported =
                prefrik_hand_authority_policy::transportRigidContactTarget(
                    source.sourceRawHandWorld,
                    source.sourceSolvedHandWorld,
                    currentRawHandWorld,
                    maximumRawMotionGameUnits);
            if (!transported.valid ||
                !frik_visual_authority::applyExternalHandWorldTransform(
                    dynamicHandTag(isLeft),
                    frik_visual_authority::handFromBool(isLeft),
                    transported.targetWorld,
                    g_rockConfig.rockHandCollisionDynamicVisualPriority)) {
                clearVisual(handSlots, isLeft);
                return;
            }

            handSlots.lastPresentedHandWorld = transported.targetWorld;
            handSlots.lastPresentedHandWorldValid = true;
        };

        refreshHand(false, rightRawHandValid, rightRawHandWorld);
        refreshHand(true, leftRawHandValid, leftRawHandWorld);
    }

    void DynamicHandCollisionRuntime::clearSurfaceFingerResponse(
        HandSlots& handSlots,
        const bool isLeft)
    {
        if (handSlots.surfaceFingerResponse.posePublished) {
            (void)frik_visual_authority::clearHandPose(
                SURFACE_FINGER_POSE_TAG,
                frik_visual_authority::handFromBool(isLeft));
        }
        handSlots.surfaceFingerResponse = {};
    }

    void DynamicHandCollisionRuntime::clearSurfaceMeshPose(
        HandSlots& handSlots,
        const bool isLeft)
    {
        if (handSlots.surfaceLatch.meshPosePublished) {
            (void)frik_visual_authority::clearHandPose(
                SURFACE_MESH_GRAB_POSE_TAG,
                frik_visual_authority::handFromBool(isLeft));
        }
        handSlots.surfaceLatch.meshPosePublished = false;
    }

    bool DynamicHandCollisionRuntime::captureSurfaceFingerResponse(
        HandSlots& handSlots,
        const bool isLeft,
        const HandState handState,
        const RE::NiTransform& rawHandWorld,
        const dynamic_hand_twin::TwinTargets& handTwins)
    {
        if (!isFiniteTransform(rawHandWorld) ||
            !frik_visual_authority::isAvailable()) {
            return false;
        }

        root_flattened_finger_skeleton_runtime::Snapshot liveSnapshot{};
        if (!root_flattened_finger_skeleton_runtime::
                resolveLiveFingerSkeletonSnapshot(isLeft, liveSnapshot) ||
            !liveSnapshot.valid) {
            return false;
        }

        HandSlots::SurfaceFingerResponse candidate{};
        const float selectedCloseValue = std::clamp(
            g_rockConfig.rockSelectedCloseFingerAnimValue,
            0.0f,
            1.0f);
        const float fallbackOpenValue = handState == HandState::SelectedClose ?
            selectedCloseValue :
            1.0f;
        candidate.baselineOpenValues.fill(fallbackOpenValue);
        for (std::size_t finger = 1;
             finger < hand_collider_semantics::kHandFingerCount;
             ++finger) {
            const auto& chain = liveSnapshot.fingers[finger];
            if (!chain.valid) {
                return false;
            }
            const float fingerLength =
                root_flattened_finger_skeleton_runtime::distance(
                    chain.points[0],
                    chain.points[1]) +
                root_flattened_finger_skeleton_runtime::distance(
                    chain.points[1],
                    chain.points[2]);
            const float chordLength =
                root_flattened_finger_skeleton_runtime::distance(
                    chain.points[0],
                    chain.points[2]);
            const auto estimate =
                grab_finger_pose_math::estimateCalibratedChainCurlFromChord(
                    finger,
                    isLeft,
                    liveSnapshot.inPowerArmor,
                    fingerLength,
                    chordLength);
            if (estimate.valid && std::isfinite(estimate.openValue)) {
                candidate.baselineOpenValues[finger] = std::clamp(
                    estimate.openValue,
                    0.0f,
                    1.0f);
            }
        }

        /*
         * The compound children keep chasing the LIVE published role frames
         * while the response runs, so no collider intent frames are cached
         * here. The capture only needs every finger twin to be publishing —
         * both the per-frame probe pass and the physical curl depend on
         * live role frames existing for the whole hand.
         */
        for (std::size_t finger = 0;
             finger < hand_collider_semantics::kHandFingerCount;
             ++finger) {
            for (std::size_t segment = 0;
                 segment < hand_collider_semantics::kHandFingerSegmentCount;
                 ++segment) {
                const auto& twin = handTwins.fingers[finger][segment];
                if (!twin.valid || !isFiniteTransform(twin.target)) {
                    return false;
                }
            }
        }

        candidate.currentOpenValues = candidate.baselineOpenValues;
        candidate.active = true;
        handSlots.surfaceFingerResponse = candidate;
        ROCK_LOG_SAMPLE_DEBUG(
            Hand,
            2000,
            "{} surface finger intent captured: open=({:.2f},{:.2f},{:.2f},{:.2f},{:.2f}) powerArmor={}",
            isLeft ? "Left" : "Right",
            candidate.baselineOpenValues[0],
            candidate.baselineOpenValues[1],
            candidate.baselineOpenValues[2],
            candidate.baselineOpenValues[3],
            candidate.baselineOpenValues[4],
            liveSnapshot.inPowerArmor ? "yes" : "no");
        return true;
    }

    std::uint32_t DynamicHandCollisionRuntime::updateSurfaceFingerResponse(
        HandSlots& handSlots,
        const bool isLeft,
        const HandState handState,
        const RE::NiTransform& rawHandWorld,
        const dynamic_hand_twin::TwinTargets& handTwins,
        const dynamic_hand_collision_telemetry::HandSample& handTelemetry,
        const float deltaSeconds,
        const bool freezeCurrentPose)
    {
        if (!g_rockConfig.rockHandCollisionSurfaceFingerResponseEnabled) {
            clearSurfaceFingerResponse(handSlots, isLeft);
            return 0;
        }

        // Flexion is a WORLD-surface response only: touching the other hand
        // or the equipped weapon must never curl or spread fingers.
        bool anyFingerContact = false;
        for (std::size_t bodyIndex =
                 dynamic_hand_collision_telemetry::kFirstFingerSlot;
             bodyIndex < kFirstForearmSlot;
             ++bodyIndex) {
            anyFingerContact = anyFingerContact ||
                               handTelemetry.twins[bodyIndex].worldContactActive;
        }
        if (!handSlots.surfaceFingerResponse.active) {
            if (!anyFingerContact) {
                return 0;
            }
            if (!captureSurfaceFingerResponse(
                    handSlots,
                    isLeft,
                    handState,
                    rawHandWorld,
                    handTwins)) {
                ROCK_LOG_SAMPLE_WARN(
                    Hand,
                    2000,
                    "{} surface finger intent capture unavailable",
                    isLeft ? "Left" : "Right");
                return 0;
            }
        }

        auto& response = handSlots.surfaceFingerResponse;
        const auto policyConfig =
            surface_finger_collision_policy::sanitize(
                surface_finger_collision_policy::Config{
                    .probeDeltaOpenUnits =
                        g_rockConfig.
                            rockHandCollisionSurfaceFingerProbeDeltaOpenUnits,
                    .responseGain =
                        g_rockConfig.rockHandCollisionSurfaceFingerResponseGain,
                    .maximumDeflectionOpenUnits =
                        g_rockConfig.
                            rockHandCollisionSurfaceFingerMaximumDeflectionOpenUnits,
                    .minimumHelpfulProbeTravelGameUnits =
                        g_rockConfig.
                            rockHandCollisionSurfaceFingerMinimumHelpfulTravelGameUnits,
                });
        std::array<std::array<
                       surface_finger_collision_policy::SegmentContact,
                       surface_finger_collision_policy::kSegmentCount>,
            surface_finger_collision_policy::kFingerCount>
            contacts{};
        std::array<std::int8_t,
            surface_finger_collision_policy::kFingerCount>
            forcedDirections{};
        const bool palmFrameValid = handTwins.palm.valid &&
                                    isFiniteTransform(handTwins.palm.target);
        const RE::NiTransform palmSceneFrame = palmFrameValid ?
            colliderFrameToSceneFrame(handTwins.palm.target) :
            rawHandWorld;
        /*
         * The probes are remeasured around the CURRENT pose every frame
         * because the solve below re-baselines on the current open values:
         * each frame of blocked contact walks the pose one bounded step
         * further, so the colliders — which chase the live role frames of
         * that same pose — physically curl and slide along the surface
         * until the contact resolves or the anatomical stop is reached.
         */
        std::array<RE::NiPoint3,
            hand_collider_semantics::kHandFingerRoleCount>
            closingProbeTravelWorld{};
        std::array<RE::NiPoint3,
            hand_collider_semantics::kHandFingerRoleCount>
            openingProbeTravelWorld{};
        bool probesValid = false;
        if (!freezeCurrentPose && anyFingerContact) {
            probesValid = computeSurfaceFingerProbeTravel(
                isLeft,
                rawHandWorld,
                handTwins,
                response.currentOpenValues,
                policyConfig.probeDeltaOpenUnits,
                closingProbeTravelWorld,
                openingProbeTravelWorld);
            if (!probesValid) {
                ROCK_LOG_SAMPLE_WARN(
                    Hand,
                    2000,
                    "{} surface finger probe travel unavailable this frame",
                    isLeft ? "Left" : "Right");
            }
        }
        if (!freezeCurrentPose && probesValid) {
            for (std::size_t finger = 0;
                 finger < hand_collider_semantics::kHandFingerCount;
                 ++finger) {
                RE::NiPoint3 fingerDeviationSum{};
                for (std::size_t segment = 0;
                     segment < hand_collider_semantics::kHandFingerSegmentCount;
                     ++segment) {
                    const std::size_t bodyIndex =
                        dynamic_hand_collision_telemetry::
                            bodyIndexForFingerSegment(finger, segment);
                    const auto& twin = handTelemetry.twins[bodyIndex];
                    if (!twin.worldContactActive ||
                        !std::isfinite(twin.contactDeviationGameUnits) ||
                        twin.contactDeviationGameUnits <= 0.0f) {
                        continue;
                    }
                    fingerDeviationSum.x += twin.contactDeviationWorldGame.x;
                    fingerDeviationSum.y += twin.contactDeviationWorldGame.y;
                    fingerDeviationSum.z += twin.contactDeviationWorldGame.z;
                    const float inverseDepth =
                        1.0f / twin.contactDeviationGameUnits;
                    const RE::NiPoint3 safeDirection{
                        twin.contactDeviationWorldGame.x * inverseDepth,
                        twin.contactDeviationWorldGame.y * inverseDepth,
                        twin.contactDeviationWorldGame.z * inverseDepth,
                    };
                    const std::size_t linearIndex =
                        finger *
                            hand_collider_semantics::kHandFingerSegmentCount +
                        segment;
                    contacts[finger][segment] =
                        surface_finger_collision_policy::SegmentContact{
                            .blockedDepthGameUnits =
                                twin.contactDeviationGameUnits,
                            .closingProbeTravelGameUnits =
                                dotPoints(
                                    closingProbeTravelWorld[linearIndex],
                                    safeDirection),
                            .openingProbeTravelGameUnits =
                                dotPoints(
                                    openingProbeTravelWorld[linearIndex],
                                    safeDirection),
                            .active = true,
                        };
                }
                forcedDirections[finger] = classifySurfaceFingerDirection(
                    palmSceneFrame,
                    fingerDeviationSum);
            }
        }

        /*
         * Re-baselined on currentOpenValues: the bounded deflection is a
         * per-frame STEP, not a total offset, so sustained blocked contact
         * accumulates all the way to a fist (or fully spread) and the
         * smoothing speed below sets the physical curl rate. Losing contact
         * flips the target back toward the captured baseline, which rides
         * the fingers along the surface instead of parking them.
         */
        const auto solve = freezeCurrentPose ?
            surface_finger_collision_policy::SolveResult{
                .targetOpenValues = response.currentOpenValues,
            } :
            surface_finger_collision_policy::solve(
                response.currentOpenValues,
                contacts,
                forcedDirections,
                policyConfig);
        const auto previousDirections = response.lastDirections;
        if (!freezeCurrentPose) {
            response.lastDirections = solve.directions;
        }
        const auto& targetOpenValues = anyFingerContact || freezeCurrentPose ?
            solve.targetOpenValues :
            response.baselineOpenValues;
        constexpr std::uint32_t kFingerSlotMask =
            ((1u << static_cast<std::uint32_t>(kFirstForearmSlot)) - 1u) &
            ~((1u << static_cast<std::uint32_t>(
                   dynamic_hand_collision_telemetry::kFirstFingerSlot)) - 1u);
        const bool dynamicInteractionFingerContact =
            ((handTelemetry.otherHandContactMask |
                 handTelemetry.weaponContactMask) &
                kFingerSlotMask) != 0;
        response.currentOpenValues =
            surface_finger_collision_policy::advanceOpenValues(
                response.currentOpenValues,
                targetOpenValues,
                dynamicInteractionFingerContact ?
                    0.0f :
                    g_rockConfig.
                        rockHandCollisionSurfaceFingerSmoothingSpeed,
                deltaSeconds);
        if (response.lastDirections != previousDirections) {
            ROCK_LOG_SAMPLE_DEBUG(
                Hand,
                1000,
                "{} surface finger directions: thumb={} index={} middle={} ring={} pinky={} (-1 close, +1 open)",
                isLeft ? "Left" : "Right",
                static_cast<int>(response.lastDirections[0]),
                static_cast<int>(response.lastDirections[1]),
                static_cast<int>(response.lastDirections[2]),
                static_cast<int>(response.lastDirections[3]),
                static_cast<int>(response.lastDirections[4]));
        }

        response.lastHelpfulDynamicSlotMask = 0;
        for (std::size_t linearIndex = 0;
             linearIndex < hand_collider_semantics::kHandFingerRoleCount;
             ++linearIndex) {
            if ((solve.helpfulSegmentMask & (1u << linearIndex)) != 0) {
                response.lastHelpfulDynamicSlotMask |=
                    1u << static_cast<std::uint32_t>(
                        dynamic_hand_collision_telemetry::kFirstFingerSlot +
                        linearIndex);
            }
        }

        if (!anyFingerContact && !freezeCurrentPose) {
            const float dt = std::clamp(
                std::isfinite(deltaSeconds) ? deltaSeconds : (1.0f / 90.0f),
                0.0f,
                0.1f);
            response.noContactSeconds += dt;
            if (response.noContactSeconds >=
                g_rockConfig.
                    rockHandCollisionSurfaceFingerReleaseDelaySeconds) {
                clearSurfaceFingerResponse(handSlots, isLeft);
                return 0;
            }
        } else {
            response.noContactSeconds = 0.0f;
        }

        const auto jointValues =
            grab_finger_pose_math::expandFingerCurlsToJointValues(
                response.currentOpenValues);
        if (!frik_visual_authority::setHandPoseCustomWithPriority(
                SURFACE_FINGER_POSE_TAG,
                frik_visual_authority::handFromBool(isLeft),
                frik_visual_authority::makeHandPoseDataFromJointValues(
                    jointValues),
                g_rockConfig.rockHandCollisionDynamicVisualPriority)) {
            clearSurfaceFingerResponse(handSlots, isLeft);
            return 0;
        }
        response.posePublished = true;
        if (!frik_visual_authority::isHandPoseTagActive(
                SURFACE_FINGER_POSE_TAG,
                frik_visual_authority::handFromBool(isLeft))) {
            /*
             * Another tagged pose won hFRIK arbitration. Keep the immutable
             * intent alive so this claim can recover when authority returns,
             * but do not report any curl as a physical resolution while it is
             * not actually driving the rendered fingers.
             */
            response.lastHelpfulDynamicSlotMask = 0;
        }
        return response.lastHelpfulDynamicSlotMask;
    }

    void DynamicHandCollisionRuntime::applyTransitionCollisionSuppression(
        RE::hknpWorld* world,
        bool suppressCollision)
    {
        if (_transitionCollisionSuppressed == suppressCollision) {
            return;
        }

        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        for (std::size_t handIndexValue = 0;
             handIndexValue < _hands.size();
             ++handIndexValue) {
            auto& handSlots = _hands[handIndexValue];
            const auto filterInfo = dynamicHandProxyFilterInfo(
                handIndexValue == 1,
                suppressCollision);
            for (auto& slot : handSlots.bodies) {
                if (!slot.created || !slot.body.isValid() || slot.createdWorld != world) {
                    continue;
                }
                slot.body.setCollisionFilterInfo(filterInfo, 1);
                clearPhysicsContactState(slot);
            }
        }
        _transitionCollisionSuppressed = suppressCollision;
        _transitionCollisionSuppressedAtomic.store(
            suppressCollision,
            std::memory_order_release);
        ROCK_LOG_INFO(
            Hand,
            "Dynamic hand collision animation transition {} retained bodies",
            suppressCollision ? "suspended for" : "resumed on");
    }

    void DynamicHandCollisionRuntime::retireAll(void* bhkWorld)
    {
        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        retireHand(_hands[0], bhkWorld, false);
        retireHand(_hands[1], bhkWorld, true);
        _surfaceContacts.clear();
    }

    void DynamicHandCollisionRuntime::reset()
    {
        retireAll(nullptr);
        auto pairStateMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        _weaponPairLeases.abandonWorld();
        _telemetrySnapshot = {};
        _pendingHapticEvents = {};
        _surfaceImpulsePairSequenceAtomic.store(0, std::memory_order_release);
        _surfaceProcessedPairSequenceAtomic.store(0, std::memory_order_release);
        _surfaceEligiblePairSequenceAtomic.store(0, std::memory_order_release);
        _surfaceContactPublishSequenceAtomic.store(0, std::memory_order_release);
        _telemetryUpdateSequence = 0;
        _logCounter = 0;
        _transitionState = {};
        _transitionCollisionSuppressed = false;
        _transitionCollisionSuppressedAtomic.store(false, std::memory_order_release);
        _dynamicInteractionsEnabledAtomic.store(false, std::memory_order_release);
        _desiredWeaponBodyIdAtomic.store(
            hand_semantic_contact_state::kInvalidBodyId,
            std::memory_order_release);
        _pairFilterReadyAtomic.store(false, std::memory_order_release);
        for (std::size_t hand = 0; hand < _hands.size(); ++hand) {
            _weaponOwnedAtomic[hand].store(false, std::memory_order_release);
            _suppressedWeaponPairCountAtomic[hand].store(
                0,
                std::memory_order_release);
            _hands[hand].pendingOtherHandContactMaskAtomic.store(
                0,
                std::memory_order_release);
            _hands[hand].pendingWeaponContactMaskAtomic.store(
                0,
                std::memory_order_release);
            _hands[hand].otherHandContactMask = 0;
            _hands[hand].weaponContactMask = 0;
            _hands[hand].otherHandContactGraceFrames = 0;
            _hands[hand].weaponContactGraceFrames = 0;
        }
    }

    void DynamicHandCollisionRuntime::updateFrame(const PhysicsFrameContext& frame,
        bool physicsWritesAllowed,
        const Hand& rightHand,
        const Hand& leftHand,
        const BodyBoneColliderSet& bodyBoneColliders,
        bool rightHandWeaponOwned,
        bool leftHandWeaponOwned,
        std::uint32_t dynamicWeaponBodyId,
        bool rightVisualReturnActive,
        bool leftVisualReturnActive)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::DynamicHandCollisionFrame);

        _surfaceContacts.advanceFrame();
        _pendingHapticEvents = {};
        dynamic_hand_collision_telemetry::Snapshot telemetry{};
        telemetry.updateSequence = ++_telemetryUpdateSequence;
        telemetry.surfaceImpulsePairSequence =
            _surfaceImpulsePairSequenceAtomic.load(std::memory_order_acquire);
        telemetry.surfaceProcessedPairSequence =
            _surfaceProcessedPairSequenceAtomic.load(std::memory_order_acquire);
        telemetry.surfaceEligiblePairSequence =
            _surfaceEligiblePairSequenceAtomic.load(std::memory_order_acquire);
        telemetry.surfaceContactPublishSequence =
            _surfaceContactPublishSequenceAtomic.load(std::memory_order_acquire);
        telemetry.runtimeEnabled = g_rockConfig.rockHandCollisionDynamicDrive;
        telemetry.worldReady = frame.worldReady;
        telemetry.menuBlocked = frame.menuBlocked;
        telemetry.physicsWritesAllowed = physicsWritesAllowed;
        telemetry.hands[0].isLeft = false;
        telemetry.hands[1].isLeft = true;

        const bool dynamicInteractionsEnabled =
            g_rockConfig.rockHandDynamicInteractionsEnabled &&
            g_rockConfig.rockHandCollisionDynamicDrive;
        _dynamicInteractionsEnabledAtomic.store(
            dynamicInteractionsEnabled,
            std::memory_order_release);
        _desiredWeaponBodyIdAtomic.store(
            dynamicWeaponBodyId,
            std::memory_order_release);
        _weaponOwnedAtomic[0].store(
            rightHandWeaponOwned,
            std::memory_order_release);
        _weaponOwnedAtomic[1].store(
            leftHandWeaponOwned,
            std::memory_order_release);

        constexpr std::uint8_t kInteractionContactGraceFrames = 2;
        for (std::size_t hand = 0; hand < _hands.size(); ++hand) {
            auto& slots = _hands[hand];
            const auto updateContactMask = [](
                                               std::atomic<std::uint32_t>& pending,
                                               std::uint32_t& current,
                                               std::uint8_t& grace) {
                const std::uint32_t observed = pending.exchange(
                    0,
                    std::memory_order_acq_rel);
                if (observed != 0) {
                    current = observed;
                    grace = kInteractionContactGraceFrames;
                } else if (grace > 0) {
                    --grace;
                } else {
                    current = 0;
                }
            };
            updateContactMask(
                slots.pendingOtherHandContactMaskAtomic,
                slots.otherHandContactMask,
                slots.otherHandContactGraceFrames);
            updateContactMask(
                slots.pendingWeaponContactMaskAtomic,
                slots.weaponContactMask,
                slots.weaponContactGraceFrames);
            auto& handTelemetry = telemetry.hands[hand];
            handTelemetry.dynamicInteractionsEnabled =
                dynamicInteractionsEnabled;
            handTelemetry.pairFilterReady =
                _pairFilterReadyAtomic.load(std::memory_order_acquire);
            handTelemetry.otherHandContactMask =
                slots.otherHandContactMask;
            handTelemetry.weaponContactMask = slots.weaponContactMask;
            handTelemetry.suppressedWeaponPairCount =
                _suppressedWeaponPairCountAtomic[hand].load(
                    std::memory_order_acquire);
            handTelemetry.weaponPairSuppressed =
                handTelemetry.suppressedWeaponPairCount != 0;
            handTelemetry.dynamicInteractionLayer =
                collision_layer_policy::dynamicHandProxyLayerForHand(
                    hand == 1);
        }

        if (!g_rockConfig.rockHandCollisionDynamicDrive) {
            const auto hasCreatedCompound = [](const HandSlots& handSlots) {
                return std::any_of(handSlots.bodies.begin(), handSlots.bodies.end(), [](const ProxySlot& slot) {
                    return slot.created;
                });
            };
            if (hasCreatedCompound(_hands[0]) || hasCreatedCompound(_hands[1])) {
                retireAll(frame.bhkWorld);
            }
            _hands[0].hapticState = {};
            _hands[1].hapticState = {};
            _transitionState = {};
            _transitionCollisionSuppressed = false;
            _transitionCollisionSuppressedAtomic.store(
                false,
                std::memory_order_release);
            telemetry.transitionCollisionSuppressed = false;
            _telemetrySnapshot = telemetry;
            return;
        }

        if (!frame.worldReady || frame.menuBlocked || !physicsWritesAllowed) {
            clearVisual(_hands[0], false);
            clearVisual(_hands[1], true);
            clearSurfaceFingerResponse(_hands[0], false);
            clearSurfaceFingerResponse(_hands[1], true);
            telemetry.hands[0].visualAuthorityAvailable = frik_visual_authority::isAvailable();
            telemetry.hands[1].visualAuthorityAvailable = telemetry.hands[0].visualAuthorityAvailable;
            updateHandHaptic(_hands[0], telemetry.hands[0], false, frame.deltaSeconds);
            updateHandHaptic(_hands[1], telemetry.hands[1], false, frame.deltaSeconds);
            telemetry.transitionCollisionSuppressed =
                _transitionCollisionSuppressed;
            _telemetrySnapshot = telemetry;
            return;
        }

        if (++_logCounter >= 360) {
            _logCounter = 0;
            ROCK_LOG_DEBUG(Hand,
                "DynamicHandCollision active: compoundChildrenPerHand={} maxLinVelHk={:.1f} divergenceTeleport={:.1f} minDeviation={:.3f} smoothingSpeed={:.1f} haptics={} priority={} compoundCreated R={} L={} surfaceCallbacks(impulse/manifold/eligible/published)={}/{}/{}/{}",
                kBodiesPerHand,
                g_rockConfig.rockHandCollisionDynamicMaxLinearVelocityHavok,
                g_rockConfig.rockHandCollisionDynamicDivergenceTeleportGameUnits,
                g_rockConfig.rockHandCollisionDynamicRenderFollowMinDeviationGameUnits,
                g_rockConfig.rockHandCollisionDynamicRenderFollowSmoothingSpeed,
                g_rockConfig.rockHandCollisionDynamicHapticsEnabled ? "yes" : "no",
                g_rockConfig.rockHandCollisionDynamicVisualPriority,
                _hands[0].bodies[kPalmSlot].created ? "yes" : "no",
                _hands[1].bodies[kPalmSlot].created ? "yes" : "no",
                telemetry.surfaceImpulsePairSequence,
                telemetry.surfaceProcessedPairSequence,
                telemetry.surfaceEligiblePairSequence,
                telemetry.surfaceContactPublishSequence);
        }

        const auto handTargetsStable = [&](bool isLeft, const HandFrameInput& handInput, const Hand& hand) {
            if (handInput.disabled) {
                return true;
            }
            const auto& handTwins = hand.dynamicTwinTargets();
            const auto& forearmTwins = bodyBoneColliders.dynamicForearmTwinTargets();
            const auto& handSlots = _hands[handIndex(isLeft)];
            for (std::size_t bodyIndex = 0; bodyIndex < kBodiesPerHand; ++bodyIndex) {
                const auto* twinFrame = twinFrameForSlot(handTwins, forearmTwins, isLeft, bodyIndex);
                const bool targetRequired =
                    bodyIndex == kPalmSlot ||
                    handSlots.bodies[0].created;
                if (targetRequired &&
                    (!twinFrame || !twinFrame->valid || !isFiniteTransform(twinFrame->target))) {
                    return false;
                }
            }
            return true;
        };
        const bool transitionTargetsStable =
            handTargetsStable(false, frame.right, rightHand) &&
            handTargetsStable(true, frame.left, leftHand);
        const auto transitionStep = dynamic_hand_collision_transition::advance(
            _transitionState,
            frame.reloadBoundaryActive,
            transitionTargetsStable);
        _transitionState = transitionStep.state;
        if (transitionStep.collisionStateChanged) {
            applyTransitionCollisionSuppression(frame.hknpWorld, transitionStep.suppressCollision);
        }

        auto updateHand = [&](bool isLeft, const HandFrameInput& handInput, const Hand& hand, bool weaponOwned, bool visualReturnActive) {
            const std::size_t index = handIndex(isLeft);
            auto& handSlots = _hands[index];
            auto& handTelemetry = telemetry.hands[index];
            handTelemetry.isLeft = isLeft;
            handTelemetry.handDisabled = handInput.disabled;
            handTelemetry.visualAuthorityAvailable = frik_visual_authority::isAvailable();

            if (!handSlots.surfaceLatch.active) {
                handSlots.lastPresentedHandWorld = handInput.rawHandWorld;
                handSlots.lastPresentedHandWorldValid =
                    isFiniteTransform(handInput.rawHandWorld);
            }

            if (handInput.disabled) {
                clearVisual(handSlots, isLeft);
                clearSurfaceFingerResponse(handSlots, isLeft);
                updateHandHaptic(handSlots, handTelemetry, false, frame.deltaSeconds);
                return;
            }

            const auto& handTwins = hand.dynamicTwinTargets();
            const auto& forearmTwins = bodyBoneColliders.dynamicForearmTwinTargets();
            std::array<const dynamic_hand_twin::TwinSlotFrame*,
                kBodiesPerHand> twinFrames{};
            std::array<RE::NiTransform, kBodiesPerHand> driveTargets{};
            RE::NiTransform resolvedHandWorld = handInput.rawHandWorld;
            bool resolvedHandWorldValid = false;

            /*
             * A fixed-surface latch follows the target body's live rigid
             * transform. If identity/readback disappears, retain the last safe
             * transforms for this frame; TouchGrabRuntime owns invalidation and
             * releases the latch from its normal lifecycle pass.
             */
            if (handSlots.surfaceLatch.active) {
                auto& latch = handSlots.surfaceLatch;
                const auto targetSnapshot = havok_runtime::snapshotBody(
                    frame.hknpWorld,
                    RE::hknpBodyId{ latch.targetBodyId });
                RE::NiTransform targetWorld{};
                if (targetSnapshot.valid &&
                    targetSnapshot.body == latch.targetBodyIdentity &&
                    targetSnapshot.collisionObject == latch.targetCollisionIdentity &&
                    havok_runtime::tryResolveLiveBodyWorldTransform(
                        frame.hknpWorld,
                        RE::hknpBodyId{ latch.targetBodyId },
                        targetWorld) &&
                    isFiniteTransform(targetWorld)) {
                    latch.lastHandWorld = transform_math::composeTransforms(
                        targetWorld,
                        latch.handInTargetBody);
                    if (latch.meshAuthoritative) {
                        latch.meshAnchorWorld =
                            transform_math::localPointToWorld(
                                targetWorld,
                                latch.meshAnchorInTargetBody);
                        latch.meshNormalWorld =
                            transform_math::localVectorToWorld(
                                targetWorld,
                                latch.meshNormalInTargetBody);
                    }
                    for (std::size_t bodyIndex = 0;
                         bodyIndex < kBodiesPerHand;
                         ++bodyIndex) {
                        if (!latch.proxyRelationshipValid[bodyIndex]) {
                            continue;
                        }
                        latch.lastProxyWorld[bodyIndex] =
                            transform_math::composeTransforms(
                                targetWorld,
                                latch.proxyInTargetBody[bodyIndex]);
                    }
                }
            }

            /*
             * The drive keeps chasing the published role frames even while
             * another system owns the hand pose: parked compounds recover through
             * the divergence teleport, and continuing to track keeps the
             * unsuppress handoff seamless. Ownership gates only decide VISUAL
             * authority.
             */
            bool allChildrenValid = true;
            for (std::size_t bodyIndex = 0; bodyIndex < kBodiesPerHand; ++bodyIndex) {
                auto& twinTelemetry = handTelemetry.twins[bodyIndex];
                twinTelemetry.role = dynamic_hand_collision_telemetry::roleForBodyIndex(bodyIndex);
                const auto* twinFrame = twinFrameForSlot(handTwins, forearmTwins, isLeft, bodyIndex);
                twinFrames[bodyIndex] = twinFrame;
                if (!twinFrame || !twinFrame->valid ||
                    !isFiniteTransform(twinFrame->target)) {
                    allChildrenValid = false;
                    continue;
                }
                twinTelemetry.lengthGameUnits = twinFrame->length;
                twinTelemetry.radiusGameUnits = twinFrame->radius;
                twinTelemetry.convexRadiusGameUnits = twinFrame->convexRadius;
                twinTelemetry.handTargetResponseScale =
                    dynamic_hand_collision_kinematics::sanitizeHandTargetResponseScale(twinFrame->handTargetResponseScale);
                /*
                 * Latch proxy worlds live in the scene convention; queued
                 * drive targets must stay in the collider column convention
                 * like the published role frames. Finger children always
                 * chase the LIVE role frames — the published flexion pose
                 * curls the rendered skeleton, the role frames follow those
                 * bones, and the compound children follow the role frames,
                 * so the physical colliders curl and slide with the hand
                 * instead of staying welded at a frozen capture pose.
                 */
                const RE::NiTransform driveTarget =
                    handSlots.surfaceLatch.active &&
                        handSlots.surfaceLatch.proxyRelationshipValid[bodyIndex] ?
                    sceneFrameToColliderFrame(
                        handSlots.surfaceLatch.lastProxyWorld[bodyIndex]) :
                    twinFrame->target;
                twinTelemetry.publishedTargetValid =
                    isFiniteTransform(driveTarget);
                twinTelemetry.publishedTargetWorld = driveTarget;
                driveTargets[bodyIndex] = driveTarget;
            }

            const std::uint64_t geometryGeneration =
                (handTwins.geometryGeneration * 0x9E3779B185EBCA87ull) ^
                (forearmTwins.geometryGeneration + 0xC2B2AE3D27D4EB4Full);
            /*
             * HandBoneColliderSet is the pose authority. Rooting the compound
             * on its exact palm collider target makes the palm child identity
             * and expresses every other collider in that already-working
             * keyframed frame. The raw skeleton hand is presentation input,
             * not a second physics coordinate system.
             */
            const RE::NiTransform compoundRootTarget =
                driveTargets[kPalmSlot];
            if (!allChildrenValid ||
                !ensureHandCreated(
                    handSlots,
                    isLeft,
                    frame,
                    hand,
                    bodyBoneColliders,
                    twinFrames,
                    compoundRootTarget,
                    driveTargets,
                    geometryGeneration) ||
                !queueCompoundPose(
                    handSlots,
                    compoundRootTarget,
                    driveTargets)) {
                if (handSlots.bodies[0].created &&
                    !_transitionCollisionSuppressed) {
                    retireHand(handSlots, frame.bhkWorld, isLeft);
                }
                clearVisual(handSlots, isLeft);
                updateHandHaptic(
                    handSlots,
                    handTelemetry,
                    false,
                    frame.deltaSeconds);
                return;
            }

            auto& compoundOwner = handSlots.bodies[0];
            const auto queueResult = queueGeneratedKeyframedBodyTarget(
                compoundOwner.driveState,
                compoundRootTarget,
                frame.deltaSeconds,
                g_rockConfig.rockHandCollisionDynamicDivergenceTeleportGameUnits,
                frame.gameFrameIndex);
            if (!queueResult.queued) {
                compoundOwner.rebuildRequestedAtomic.store(
                    true,
                    std::memory_order_release);
            }

            for (std::size_t bodyIndex = 0; bodyIndex < kBodiesPerHand; ++bodyIndex) {
                auto& twinTelemetry = handTelemetry.twins[bodyIndex];
                twinTelemetry.bodyCreated = compoundOwner.created;
                twinTelemetry.bodyId = compoundOwner.created ?
                    compoundOwner.body.getBodyId().value :
                    dynamic_hand_collision_telemetry::kInvalidBodyId;

                PhysicsTelemetrySample physicsSample{};
                std::uint64_t physicsSampleSequence = 0;
                if (!readPhysicsTelemetry(
                        handSlots.bodies[bodyIndex],
                        physicsSample,
                        physicsSampleSequence) ||
                    !physicsSample.valid ||
                    !isFinitePoint(physicsSample.requestedTargetWorldGame) ||
                    !isFinitePoint(physicsSample.commandedTargetWorldGame) ||
                    !isFinitePoint(physicsSample.liveBodyWorldGame) ||
                    !isFiniteTransform(physicsSample.requestedTargetWorld) ||
                    !isFiniteTransform(physicsSample.liveBodyWorld)) {
                    continue;
                }

                twinTelemetry.physicsSampleSequence = physicsSampleSequence;
                twinTelemetry.sourceGameFrameIndex = physicsSample.sourceGameFrameIndex;
                twinTelemetry.sourceQueueSequence = physicsSample.sourceQueueSequence;
                twinTelemetry.solveSequence = physicsSample.solveSequence;
                twinTelemetry.physicsSampleValid = true;
                twinTelemetry.targetVelocityValid = physicsSample.targetVelocityValid;
                twinTelemetry.contactActive = physicsSample.contactActive;
                twinTelemetry.worldContactActive = physicsSample.worldContactActive;
                twinTelemetry.recoveryTeleport = physicsSample.recoveryTeleport;
                twinTelemetry.requestedTargetWorldGame = physicsSample.requestedTargetWorldGame;
                twinTelemetry.commandedTargetWorldGame = physicsSample.commandedTargetWorldGame;
                twinTelemetry.liveBodyWorldGame = physicsSample.liveBodyWorldGame;
                twinTelemetry.targetVelocityWorldGameUnitsPerSecond = physicsSample.targetVelocityWorldGameUnitsPerSecond;
                twinTelemetry.approachSpeedGameUnitsPerSecond = physicsSample.approachSpeedGameUnitsPerSecond;
                twinTelemetry.physicsDeltaSeconds = physicsSample.physicsDeltaSeconds;
                twinTelemetry.physicsRawDeltaSeconds = physicsSample.physicsRawDeltaSeconds;
                twinTelemetry.physicsRemainderDeltaSeconds = physicsSample.physicsRemainderDeltaSeconds;
                twinTelemetry.physicsAccumulatedDeltaSeconds = physicsSample.physicsAccumulatedDeltaSeconds;
                twinTelemetry.physicsSubstepProgress = physicsSample.physicsSubstepProgress;
                twinTelemetry.physicsSubstepCount = physicsSample.physicsSubstepCount;
                twinTelemetry.physicsSubstepIndex = physicsSample.physicsSubstepIndex;
                twinTelemetry.solverResidualWorldGame = subtractPoints(
                    twinTelemetry.liveBodyWorldGame,
                    twinTelemetry.commandedTargetWorldGame);
                twinTelemetry.requestedGapWorldGame = subtractPoints(
                    twinTelemetry.liveBodyWorldGame,
                    twinTelemetry.requestedTargetWorldGame);
                twinTelemetry.solverResidualGameUnits = pointLength(twinTelemetry.solverResidualWorldGame);
                twinTelemetry.requestedGapGameUnits = pointLength(twinTelemetry.requestedGapWorldGame);

                if (physicsSample.contactActive) {
                    twinTelemetry.contactDeviationWorldGame = twinTelemetry.requestedGapWorldGame;
                    twinTelemetry.contactDeviationGameUnits = twinTelemetry.requestedGapGameUnits;
                    // Palm/finger points are the hand target (scale 1). A
                    // forearm point has less shoulder-lever response to that
                    // target, so its source publication maps the blocked point
                    // displacement into the IK target displacement first.
                    twinTelemetry.handTargetCorrectionWorldGame = RE::NiPoint3{
                        twinTelemetry.contactDeviationWorldGame.x * twinTelemetry.handTargetResponseScale,
                        twinTelemetry.contactDeviationWorldGame.y * twinTelemetry.handTargetResponseScale,
                        twinTelemetry.contactDeviationWorldGame.z * twinTelemetry.handTargetResponseScale,
                    };
                    twinTelemetry.handTargetCorrectionGameUnits = pointLength(twinTelemetry.handTargetCorrectionWorldGame);
                    handTelemetry.contactMask |= 1u << static_cast<std::uint32_t>(bodyIndex);
                    ++handTelemetry.contactCount;
                }

                if (!resolvedHandWorldValid &&
                    physicsSample.contactActive) {
                    /*
                     * Full rigid transport of the solver correction onto the
                     * raw hand: resolved = live * inverse(requested) * raw.
                     * Both sampled transforms are published in the scene
                     * convention, so this delta rotates the rendered hand the
                     * same way the solver rotated the compound — the weapon's
                     * reframeAttachedHand contract.
                     */
                    const RE::NiTransform requestedInverse =
                        transform_math::invertTransform(
                            physicsSample.requestedTargetWorld);
                    resolvedHandWorld = transform_math::composeTransforms(
                        physicsSample.liveBodyWorld,
                        transform_math::composeTransforms(
                            requestedInverse,
                            handInput.rawHandWorld));
                    resolvedHandWorld.scale = handInput.rawHandWorld.scale;
                    resolvedHandWorldValid =
                        isFiniteTransform(resolvedHandWorld);
                }
            }

            handTelemetry.anyContact = handTelemetry.contactCount > 0;
            const bool physicallyOwnedByStrongerSystem =
                suppressesGeneratedHandContactEvidence(hand.getState()) ||
                weaponOwned ||
                _transitionCollisionSuppressed;
            const bool visuallyOwnedByStrongerSystem =
                physicallyOwnedByStrongerSystem || visualReturnActive;
            const bool latchAuthorityBlocked =
                handSlots.surfaceLatch.active &&
                visuallyOwnedByStrongerSystem;
            std::uint32_t helpfulFingerSlotMask = 0;
            if (handSlots.surfaceLatch.meshAuthoritative) {
                clearSurfaceFingerResponse(handSlots, isLeft);
            } else if (!visuallyOwnedByStrongerSystem &&
                       handTelemetry.visualAuthorityAvailable) {
                helpfulFingerSlotMask = updateSurfaceFingerResponse(
                    handSlots,
                    isLeft,
                    hand.getState(),
                    handInput.rawHandWorld,
                    handTwins,
                    handTelemetry,
                    frame.deltaSeconds,
                    handSlots.surfaceLatch.active);
            } else {
                clearSurfaceFingerResponse(handSlots, isLeft);
            }
            handTelemetry.surfaceFingerResponseActive =
                handSlots.surfaceFingerResponse.active;
            handTelemetry.surfaceFingerHelpfulSlotMask =
                helpfulFingerSlotMask;
            if (handSlots.surfaceFingerResponse.active) {
                handTelemetry.surfaceFingerOpenValues =
                    handSlots.surfaceFingerResponse.currentOpenValues;
                handTelemetry.surfaceFingerDirections =
                    handSlots.surfaceFingerResponse.lastDirections;
            }

            const RE::NiPoint3 combined =
                handTelemetry.anyContact && resolvedHandWorldValid ?
                subtractPoints(
                    resolvedHandWorld.translate,
                    handInput.rawHandWorld.translate) :
                RE::NiPoint3{};
            handTelemetry.combinedContactDeviationWorldGame = combined;
            handTelemetry.combinedContactDeviationGameUnits = pointLength(combined);

            if (handSlots.surfaceLatch.active) {
                handSlots.preFrikContactAuthority = {};
                if (latchAuthorityBlocked) {
                    endSurfaceLatch(isLeft);
                    updateHandHaptic(
                        handSlots,
                        handTelemetry,
                        false,
                        frame.deltaSeconds);
                    handTelemetry.ownedByStrongerSystem = true;
                    handTelemetry.visualActive = false;
                    return;
                }
                handTelemetry.ownedByStrongerSystem = true;
                updateHandHaptic(
                    handSlots,
                    handTelemetry,
                    false,
                    frame.deltaSeconds);
                handSlots.appliedDeviation = {};
                handSlots.teleportRecoverySecondsRemaining = 0.0f;
                const auto& latchTarget =
                    handSlots.surfaceLatch.lastHandWorld;
                if (!handTelemetry.visualAuthorityAvailable ||
                    !isFiniteTransform(latchTarget)) {
                    endSurfaceLatch(isLeft);
                    handTelemetry.visualActive = handSlots.visualActive;
                    return;
                }
                auto& latch = handSlots.surfaceLatch;
                if (latch.meshAuthoritative &&
                    latch.meshFingerPoseValid) {
                    if (frik_visual_authority::setHandPoseCustomWithPriority(
                            SURFACE_MESH_GRAB_POSE_TAG,
                            frik_visual_authority::handFromBool(isLeft),
                            frik_visual_authority::
                                makeHandPoseDataFromJointValues(
                                    latch.meshFingerJointValues),
                            kSurfaceLatchVisualPriority)) {
                        latch.meshPosePublished = true;
                    } else {
                        ROCK_LOG_SAMPLE_WARN(
                            Hand,
                            2000,
                            "{} surface mesh finger pose apply failed",
                            isLeft ? "Left" : "Right");
                    }
                }
                if (frik_visual_authority::applyExternalHandWorldTransform(
                        dynamicHandTag(isLeft),
                        frik_visual_authority::handFromBool(isLeft),
                        latchTarget,
                        kSurfaceLatchVisualPriority)) {
                    handSlots.visualActive = true;
                    handSlots.lastPresentedHandWorld = latchTarget;
                    handSlots.lastPresentedHandWorldValid = true;
                } else {
                    ROCK_LOG_SAMPLE_WARN(
                        Hand,
                        2000,
                        "{} dynamic surface latch render apply failed",
                        isLeft ? "Left" : "Right");
                    endSurfaceLatch(isLeft);
                }
                handTelemetry.visualActive = handSlots.visualActive;
                return;
            }

            handTelemetry.ownedByStrongerSystem = visuallyOwnedByStrongerSystem;
            updateHandHaptic(
                handSlots,
                handTelemetry,
                !physicallyOwnedByStrongerSystem && handTelemetry.visualAuthorityAvailable,
                frame.deltaSeconds);
            if (visuallyOwnedByStrongerSystem || !handTelemetry.visualAuthorityAvailable) {
                clearVisual(handSlots, isLeft);
                handTelemetry.appliedVisualDeviationWorldGame = handSlots.appliedDeviation;
                handTelemetry.appliedVisualDeviationGameUnits = pointLength(handSlots.appliedDeviation);
                handTelemetry.visualActive = handSlots.visualActive;
                return;
            }

            /*
             * A divergence teleport opens the recovery window: the applied
             * deviation glides home over the configured duration (speed ~3/T
             * reaches ~95% by the window's end) instead of snapping at the
             * contact-smoothing rate.
             */
            bool teleportedThisFrame = false;
            for (auto& slot : handSlots.bodies) {
                if (slot.teleportedAtomic.exchange(false, std::memory_order_acq_rel)) {
                    teleportedThisFrame = true;
                }
            }
            const float recoveryDuration = g_rockConfig.rockHandCollisionDynamicTeleportRecoverySeconds;
            if (teleportedThisFrame && recoveryDuration > 0.0f) {
                handSlots.teleportRecoverySecondsRemaining = recoveryDuration;
            }
            const float frameDt = std::clamp(std::isfinite(frame.deltaSeconds) ? frame.deltaSeconds : (1.0f / 90.0f), 0.0f, 0.1f);
            float smoothingSpeed = 0.0f;
            if (handSlots.teleportRecoverySecondsRemaining > 0.0f) {
                handSlots.teleportRecoverySecondsRemaining = std::max(0.0f, handSlots.teleportRecoverySecondsRemaining - frameDt);
                const float recoverySpeed = 3.0f / std::max(recoveryDuration, 0.05f);
                smoothingSpeed = g_rockConfig.rockHandCollisionDynamicRenderFollowSmoothingSpeed;
                smoothingSpeed = smoothingSpeed > 0.0f ? std::min(smoothingSpeed, recoverySpeed) : recoverySpeed;
            }

            handSlots.appliedDeviation = smoothAppliedDeviation(
                handSlots.appliedDeviation,
                combined,
                smoothingSpeed,
                frame.deltaSeconds);
            handTelemetry.teleportRecoverySecondsRemaining = handSlots.teleportRecoverySecondsRemaining;

            const auto& applied = handSlots.appliedDeviation;
            const float appliedLengthSq = applied.x * applied.x + applied.y * applied.y + applied.z * applied.z;
            const float minDeviation = g_rockConfig.rockHandCollisionDynamicRenderFollowMinDeviationGameUnits;
            const float rotationCorrectionDegrees = resolvedHandWorldValid ?
                prefrik_hand_authority_policy::rotationDeltaDegrees(
                    handInput.rawHandWorld,
                    resolvedHandWorld) :
                0.0f;
            constexpr float kMinimumVisibleRotationDegrees = 0.10f;
            if (!handTelemetry.anyContact || !resolvedHandWorldValid ||
                !std::isfinite(appliedLengthSq) ||
                (appliedLengthSq <= minDeviation * minDeviation &&
                    rotationCorrectionDegrees <=
                        kMinimumVisibleRotationDegrees)) {
                clearVisual(handSlots, isLeft);
                handTelemetry.appliedVisualDeviationWorldGame = handSlots.appliedDeviation;
                handTelemetry.appliedVisualDeviationGameUnits = pointLength(handSlots.appliedDeviation);
                handTelemetry.visualActive = handSlots.visualActive;
                return;
            }

            handTelemetry.appliedVisualDeviationWorldGame = applied;
            handTelemetry.appliedVisualDeviationGameUnits = std::sqrt(appliedLengthSq);

            RE::NiTransform target = resolvedHandWorld;
            target.translate = RE::NiPoint3{
                handInput.rawHandWorld.translate.x + applied.x,
                handInput.rawHandWorld.translate.y + applied.y,
                handInput.rawHandWorld.translate.z + applied.z,
            };
            target.scale = handInput.rawHandWorld.scale;
            if (!isFiniteTransform(target)) {
                clearVisual(handSlots, isLeft);
                handTelemetry.appliedVisualDeviationWorldGame = handSlots.appliedDeviation;
                handTelemetry.appliedVisualDeviationGameUnits = pointLength(handSlots.appliedDeviation);
                handTelemetry.visualActive = handSlots.visualActive;
                return;
            }

            handSlots.lastPresentedHandWorld = target;
            handSlots.lastPresentedHandWorldValid = true;

            if (frik_visual_authority::applyExternalHandWorldTransform(
                    dynamicHandTag(isLeft),
                    frik_visual_authority::handFromBool(isLeft),
                    target,
                    g_rockConfig.rockHandCollisionDynamicVisualPriority)) {
                handSlots.visualActive = true;
                auto& source = handSlots.preFrikContactAuthority;
                source = {};
                source.sourceRawHandWorld = handInput.rawHandWorld;
                source.sourceSolvedHandWorld = target;
                source.appliedDeviationWorldGame = applied;
                source.sourceSchedulerSequence =
                    frame.preFrikSchedulerSequence;
                source.sourceGameFrameIndex = frame.gameFrameIndex;
                for (const auto& twin : handTelemetry.twins) {
                    if (twin.physicsSampleValid) {
                        source.solveSequence =
                            std::max(source.solveSequence, twin.solveSequence);
                    }
                }
                source.valid =
                    source.sourceSchedulerSequence != 0 &&
                    prefrik_hand_authority_policy::isUsableTransform(
                        source.sourceRawHandWorld) &&
                    prefrik_hand_authority_policy::isUsableTransform(
                        source.sourceSolvedHandWorld) &&
                    prefrik_hand_authority_policy::isFinitePoint(
                        source.appliedDeviationWorldGame);
            } else {
                ROCK_LOG_SAMPLE_WARN(Hand, 2000, "{} dynamic hand render-follow apply failed", isLeft ? "Left" : "Right");
                clearVisual(handSlots, isLeft);
            }
            handTelemetry.appliedVisualDeviationWorldGame = handSlots.appliedDeviation;
            handTelemetry.appliedVisualDeviationGameUnits = pointLength(handSlots.appliedDeviation);
            handTelemetry.visualActive = handSlots.visualActive;
        };

        updateHand(false, frame.right, rightHand, rightHandWeaponOwned, rightVisualReturnActive);
        updateHand(true, frame.left, leftHand, leftHandWeaponOwned, leftVisualReturnActive);
        telemetry.transitionCollisionSuppressed =
            _transitionCollisionSuppressed;
        _telemetrySnapshot = telemetry;
    }

    void DynamicHandCollisionRuntime::flushPendingPhysicsDrive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::DynamicHandCollisionPhysicsDrive);

        std::array<HavokPairCollisionLeaseSet::DesiredPair,
            HavokPairCollisionLeaseSet::kMaximumPairs>
            desiredPairs{};
        std::size_t desiredPairCount = 0;
        const bool interactionsEnabled =
            _dynamicInteractionsEnabledAtomic.load(
                std::memory_order_acquire);
        const std::uint32_t weaponBodyId =
            _desiredWeaponBodyIdAtomic.load(std::memory_order_acquire);
        if (interactionsEnabled && world &&
            weaponBodyId != hand_semantic_contact_state::kInvalidBodyId) {
            for (std::size_t hand = 0; hand < _hands.size(); ++hand) {
                if (!_weaponOwnedAtomic[hand].load(
                        std::memory_order_acquire)) {
                    continue;
                }
                const std::uint32_t handBodyId =
                    _hands[hand].bodies[0].bodyIdAtomic.load(
                        std::memory_order_acquire);
                if (handBodyId !=
                        hand_semantic_contact_state::kInvalidBodyId &&
                    desiredPairCount < desiredPairs.size()) {
                    desiredPairs[desiredPairCount++] = {
                        .bodyA = handBodyId,
                        .bodyB = weaponBodyId,
                        .ownerGroup = static_cast<std::uint8_t>(hand),
                    };
                }
            }
        }
        const auto pairResult = _weaponPairLeases.reconcile(
            world,
            desiredPairs.data(),
            desiredPairCount);
        _pairFilterReadyAtomic.store(
            pairResult.filterAvailable,
            std::memory_order_release);
        for (std::size_t hand = 0; hand < _hands.size(); ++hand) {
            _suppressedWeaponPairCountAtomic[hand].store(
                pairResult.activePairsByOwnerGroup[hand],
                std::memory_order_release);
        }

        if (!g_rockConfig.rockHandCollisionDynamicDrive || !world) {
            return;
        }

        for (std::size_t hand = 0; hand < _hands.size(); ++hand) {
            const bool isLeft = hand == 1;
            auto& handSlots = _hands[hand];
            auto& slot = handSlots.bodies[0];
            if (!slot.created || slot.createdWorld != world) {
                continue;
            }

            {
                std::scoped_lock poseLock(handSlots.compoundPoseMutex);
                if (handSlots.queuedCompoundPoseSequence !=
                    handSlots.consumedCompoundPoseSequence) {
                    const auto updateResult =
                        handSlots.compoundShape.updateTransforms(
                            handSlots.pendingCompoundChildTransforms);
                    if (!updateResult.succeeded) {
                        slot.rebuildRequestedAtomic.store(
                            true,
                            std::memory_order_release);
                        clearPhysicsContactState(slot);
                        continue;
                    }
                    handSlots.consumedCompoundPoseSequence =
                        handSlots.queuedCompoundPoseSequence;
                    handSlots.consumedChildInContactBodyGame =
                        handSlots.childInContactBodyGame;
                }
            }

            const float divergenceThreshold = g_rockConfig.
                rockHandCollisionDynamicDivergenceTeleportGameUnits;
            const bool teleportArmed =
                slot.divergenceDwellSeconds >= g_rockConfig.
                    rockHandCollisionDynamicDivergenceTeleportDwellSeconds;
            GeneratedBodyDriveMode mode{
                .dynamicVelocity = true,
                .divergenceTeleportGameUnits =
                    teleportArmed ? divergenceThreshold : 0.0f,
            };

            /*
             * This is the established dynamic-collider drive contract: the
             * dynamic body itself receives the hard-keyframe velocity. The
             * compound changes only shape ownership; it does not introduce a
             * keyframed proxy, constraint, or alternate skeleton frame.
             */
            constexpr float kPressCapActivationDeviationGameUnits = 0.25f;
            const float pressCapHavok = g_rockConfig.
                rockHandCollisionDynamicContactPressMaxVelocityHavok;
            if (pressCapHavok > 0.0f &&
                slot.lastPostSolveDeviationValid) {
                const auto& deviation = slot.lastPostSolveDeviationGame;
                const float deviationLength = std::sqrt(
                    deviation.x * deviation.x +
                    deviation.y * deviation.y +
                    deviation.z * deviation.z);
                if (std::isfinite(deviationLength) &&
                    deviationLength >
                        kPressCapActivationDeviationGameUnits) {
                    mode.hasContactPressDirection = true;
                    mode.contactPressDirection[0] =
                        -deviation.x / deviationLength;
                    mode.contactPressDirection[1] =
                        -deviation.y / deviationLength;
                    mode.contactPressDirection[2] =
                        -deviation.z / deviationLength;
                    mode.contactPressMaxVelocityHavok = pressCapHavok;
                }
            }

            const auto result = driveGeneratedKeyframedBody(
                world,
                slot.body,
                slot.driveState,
                timing,
                isLeft ? "LeftHandDynamicCompound" :
                         "RightHandDynamicCompound",
                0,
                g_rockConfig.rockHandCollisionDynamicMaxLinearVelocityHavok,
                0.0f,
                mode);
            if (result.shouldRequestRebuild()) {
                slot.rebuildRequestedAtomic.store(
                    true,
                    std::memory_order_release);
                clearPhysicsContactState(slot);
                continue;
            }

            if (!result.driven ||
                !result.hasRequestedTargetGameTransform ||
                !result.hasCommandedTargetGameTransform) {
                clearPhysicsContactState(slot);
                continue;
            }

            slot.requestedTargetWorld = result.requestedTargetGameTransform;
            slot.commandedTargetWorld = result.commandedTargetGameTransform;
            slot.requestedTargetGame = slot.requestedTargetWorld.translate;
            slot.commandedTargetGame = slot.commandedTargetWorld.translate;
            const float havokToGameScale = physics_scale::havokToGame();
            slot.droveTargetVelocityValid =
                result.hasSampledTargetLinearVelocityHavok &&
                physics_scale::isUsableScale(havokToGameScale);
            slot.droveTargetVelocityGameUnitsPerSecond =
                slot.droveTargetVelocityValid ?
                RE::NiPoint3{
                    result.sampledTargetLinearVelocityHavok.x *
                        havokToGameScale,
                    result.sampledTargetLinearVelocityHavok.y *
                        havokToGameScale,
                    result.sampledTargetLinearVelocityHavok.z *
                        havokToGameScale,
                } :
                RE::NiPoint3{};
            slot.drovePhysicsDeltaSeconds = result.driveDeltaSeconds;
            slot.droveSourceGameFrameIndex = result.sourceFrameIndex;
            slot.droveSourceQueueSequence = result.sourceSequence;
            slot.droveRecoveryTeleport = result.teleported;
            slot.droveThisSubstep = true;

            const float requestedGap = result.hasLiveBodyTransform ?
                prefrik_hand_authority_policy::translationDeltaGameUnits(
                    result.liveBodyGameTransform,
                    result.requestedTargetGameTransform) :
                0.0f;
            if (result.teleported) {
                slot.divergenceDwellSeconds = 0.0f;
                slot.teleportedAtomic.store(true, std::memory_order_release);
            } else if (result.attempted && result.hasLiveBodyTransform &&
                       std::isfinite(requestedGap) &&
                       divergenceThreshold > 0.0f &&
                       requestedGap > divergenceThreshold) {
                slot.divergenceDwellSeconds += std::clamp(
                    result.driveDeltaSeconds,
                    0.0f,
                    0.1f);
            } else {
                slot.divergenceDwellSeconds = 0.0f;
            }
        }
    }

    void DynamicHandCollisionRuntime::samplePostSolveDeviations(
        RE::hknpWorld* world,
        const std::uint64_t solveSequence,
        const havok_physics_timing::PhysicsTimingSample& timing)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::DynamicHandCollisionPostSolve);

        if (!g_rockConfig.rockHandCollisionDynamicDrive || !world) {
            return;
        }

        for (auto& handSlots : _hands) {
            auto& owner = handSlots.bodies[0];
            if (!owner.created || owner.createdWorld != world ||
                !owner.droveThisSubstep) {
                continue;
            }
            owner.droveThisSubstep = false;

            RE::NiTransform liveCompoundWorld{};
            if (!havok_runtime::tryResolveLiveBodyWorldTransform(
                    world,
                    owner.body.getBodyId(),
                    liveCompoundWorld) ||
                !isFiniteTransform(liveCompoundWorld)) {
                for (auto& semanticSlot : handSlots.bodies) {
                    clearPhysicsContactState(semanticSlot);
                }
                continue;
            }

            constexpr std::uint32_t kAllChildBits =
                (1u << static_cast<std::uint32_t>(kBodiesPerHand)) - 1u;
            const std::uint32_t observedContactMask =
                handSlots.pendingSolverContactMaskAtomic.exchange(
                    0,
                    std::memory_order_acq_rel) &
                kAllChildBits;
            const std::uint32_t observedWorldContactMask =
                handSlots.pendingWorldContactMaskAtomic.exchange(
                    0,
                    std::memory_order_acq_rel) &
                kAllChildBits;
            if (observedContactMask != 0) {
                handSlots.retainedSolverContactMask = observedContactMask;
                handSlots.retainedWorldContactMask = observedWorldContactMask;
                handSlots.solverContactRetentionSeconds =
                    kCompoundContactRetentionSeconds;
            } else {
                handSlots.solverContactRetentionSeconds = std::max(
                    0.0f,
                    handSlots.solverContactRetentionSeconds -
                        std::clamp(
                            timing.substepDeltaSeconds,
                            0.0f,
                            0.1f));
                if (handSlots.solverContactRetentionSeconds <= 0.0f) {
                    handSlots.retainedSolverContactMask = 0;
                    handSlots.retainedWorldContactMask = 0;
                }
            }
            const std::uint32_t contactMask =
                handSlots.retainedSolverContactMask;
            const std::uint32_t worldContactMask =
                handSlots.retainedWorldContactMask;
            float maxEntryApproachSpeed = 0.0f;
            std::array<RE::NiTransform, kBodiesPerHand> childFrames{};
            {
                std::scoped_lock poseLock(handSlots.compoundPoseMutex);
                childFrames =
                    handSlots.consumedChildInContactBodyGame;
            }

            /*
             * The drive targets and the raw body readback are collider
             * column-convention frames; the stored child frames are scene
             * row-convention. Convert the body-level transforms once so every
             * published per-child transform is a genuine scene transform the
             * render-follow math can compose with the raw hand.
             */
            const RE::NiTransform requestedSceneWorld =
                colliderFrameToSceneFrame(owner.requestedTargetWorld);
            const RE::NiTransform commandedSceneWorld =
                colliderFrameToSceneFrame(owner.commandedTargetWorld);
            const RE::NiTransform liveCompoundSceneWorld =
                colliderFrameToSceneFrame(liveCompoundWorld);

            for (std::size_t bodyIndex = 0; bodyIndex < handSlots.bodies.size(); ++bodyIndex) {
                auto& slot = handSlots.bodies[bodyIndex];
                const RE::NiTransform requestedChildWorld =
                    transform_math::composeTransforms(
                        requestedSceneWorld,
                        childFrames[bodyIndex]);
                const RE::NiTransform commandedChildWorld =
                    transform_math::composeTransforms(
                        commandedSceneWorld,
                        childFrames[bodyIndex]);
                const RE::NiTransform liveChildWorld =
                    transform_math::composeTransforms(
                        liveCompoundSceneWorld,
                        childFrames[bodyIndex]);
                const bool contact =
                    (contactMask &
                        (1u << static_cast<std::uint32_t>(bodyIndex))) != 0;
                const bool worldContact =
                    (worldContactMask &
                        (1u << static_cast<std::uint32_t>(bodyIndex))) != 0;

                RE::NiPoint3 deviation{};
                float approachSpeedGameUnitsPerSecond = 0.0f;
                if (contact) {
                    deviation = RE::NiPoint3{
                        liveChildWorld.translate.x -
                            requestedChildWorld.translate.x,
                        liveChildWorld.translate.y -
                            requestedChildWorld.translate.y,
                        liveChildWorld.translate.z -
                            requestedChildWorld.translate.z,
                    };
                    if (!owner.droveRecoveryTeleport &&
                        owner.droveTargetVelocityValid) {
                        approachSpeedGameUnitsPerSecond =
                            dynamic_hand_collision_feedback::
                                projectedApproachSpeedGameUnitsPerSecond(
                                    owner.
                                        droveTargetVelocityGameUnitsPerSecond,
                                    deviation);
                    }
                    maxEntryApproachSpeed = std::max(maxEntryApproachSpeed, approachSpeedGameUnitsPerSecond);
                }

                publishPhysicsTelemetry(slot,
                    PhysicsTelemetrySample{
                        .requestedTargetWorld = requestedChildWorld,
                        .commandedTargetWorld = commandedChildWorld,
                        .liveBodyWorld = liveChildWorld,
                        .requestedTargetWorldGame = requestedChildWorld.translate,
                        .commandedTargetWorldGame = commandedChildWorld.translate,
                        .liveBodyWorldGame = liveChildWorld.translate,
                        .targetVelocityWorldGameUnitsPerSecond = owner.droveTargetVelocityGameUnitsPerSecond,
                        .approachSpeedGameUnitsPerSecond = approachSpeedGameUnitsPerSecond,
                        .physicsDeltaSeconds = owner.drovePhysicsDeltaSeconds,
                        .physicsRawDeltaSeconds = timing.rawDeltaSeconds,
                        .physicsRemainderDeltaSeconds = timing.remainderDeltaSeconds,
                        .physicsAccumulatedDeltaSeconds = timing.accumulatedDeltaSeconds,
                        .physicsSubstepProgress = timing.substepProgress,
                        .sourceGameFrameIndex = owner.droveSourceGameFrameIndex,
                        .sourceQueueSequence = owner.droveSourceQueueSequence,
                        .solveSequence = solveSequence,
                        .physicsSubstepCount = timing.substepCount,
                        .physicsSubstepIndex = timing.substepIndex,
                        .valid = true,
                        .targetVelocityValid = owner.droveTargetVelocityValid,
                        .contactActive = contact,
                        .worldContactActive = worldContact,
                        .recoveryTeleport = owner.droveRecoveryTeleport,
                    });
            }

            const RE::NiPoint3 aggregateDeviation{
                liveCompoundWorld.translate.x -
                    owner.requestedTargetWorld.translate.x,
                liveCompoundWorld.translate.y -
                    owner.requestedTargetWorld.translate.y,
                liveCompoundWorld.translate.z -
                    owner.requestedTargetWorld.translate.z,
            };
            owner.lastPostSolveDeviationGame = aggregateDeviation;
            owner.lastPostSolveDeviationValid = contactMask != 0;
            owner.lastPostSolveContact = contactMask != 0;

            /*
             * Manifold contact fires the substep surfaces first touch,
             * BEFORE any penetration deviation exists, so a grazing or
             * resting touch reports ~zero approach speed. Latching the
             * haptic entry there consumes the episode and the later real
             * press stays silent. The entry therefore waits until the
             * approach speed can actually fire a pulse — restoring the old
             * residual-detection feel where contact only counted once the
             * hand was genuinely pressing in.
             */
            const bool anyContact = contactMask != 0;
            const float entryGateSpeed = std::max(
                0.0f,
                g_rockConfig.
                    rockHandCollisionDynamicHapticMinApproachSpeedGameUnitsPerSecond);
            if (anyContact && !handSlots.physicsContactActive &&
                maxEntryApproachSpeed >= entryGateSpeed &&
                maxEntryApproachSpeed > 0.0f) {
                handSlots.contactEntryApproachSpeedAtomic.store(maxEntryApproachSpeed, std::memory_order_relaxed);
                handSlots.contactEntryMaskAtomic.store(contactMask, std::memory_order_relaxed);
                handSlots.contactEntrySequenceAtomic.fetch_add(1, std::memory_order_release);
                handSlots.physicsContactActive = true;
            } else if (!anyContact) {
                handSlots.physicsContactActive = false;
            }
        }
    }
}
