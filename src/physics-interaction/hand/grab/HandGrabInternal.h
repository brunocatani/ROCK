// This folder owns the Hand class grab flow.
// physics-interaction/grab owns shared grab types and policies.
// Keep hand-specific flow code here.
#pragma once

#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/hand/grab/HandGrabMath.h"
#include "physics-interaction/hand/grab/HandGrabTrace.h"
#include "physics-interaction/hand/grab/HandGrabVisualDetail.h"
#include "physics-interaction/hand/grab/HandGrabContactEvidence.h"
#include "physics-interaction/hand/grab/HandGrabFingerPose.h"
#include "physics-interaction/hand/grab/HandGrabSupportModel.h"
#include "physics-interaction/hand/grab/HandGrabOffsetSources.h"
#include "physics-interaction/hand/grab/HandGrabBodySetRuntime.h"
#include "physics-interaction/hand/grab/HandGrabPivotAuthority.h"

#include "physics-interaction/body/BodyBoneColliderSet.h"
#include "physics-interaction/native/havok/HavokOffsets.h"

#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/native/CharacterControllerRuntime.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/debug/DebugMath.h"
#include "physics-interaction/debug/GrabClockDebugFeed.h"
#include "physics-interaction/hand/HeldBodyRenderPose.h"
#include "physics-interaction/native/SceneWriterProbe.h"
#include "physics-interaction/grenade/LooseGrenadeRuntime.h"
#include "physics-interaction/grab/GrabAuthorityProxy.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/grab/GrabConstraintMath.h"
#include "physics-interaction/grab/GrabContact.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/saved/SavedGrabOffsetStore.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/GrabMassPolicy.h"
#include "physics-interaction/grab/GrabMotionController.h"
#include "physics-interaction/grab/GrabNodeInfoMath.h"
#include "physics-interaction/grab/GrabNodeNamePolicy.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/object/MechanicalConnectedBodySet.h"
#include "physics-interaction/object/CarInteractionPolicy.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/LooseWeaponGripZone.h"
#include "physics-interaction/weapon/WeaponTypePolicy.h"
#include "physics-interaction/object/SkinnedBodyResolver.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/visual/PreFrikHandAuthorityPolicy.h"
#include "physics-interaction/hand/skeleton/HandFrame.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/core/PhysicsHooks.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/native/query/PhysicsShapeCast.h"
#include "physics-interaction/native/query/PhysicsRecursiveWrappers.h"
#include "physics-interaction/native/query/PhysicsUtils.h"
#include "physics-interaction/native/query/PhysicsScale.h"
#include "physics-interaction/native/havok/HavokMaterialRegistry.h"
#include "physics-interaction/native/havok/HavokRefCount.h"
#include "RE/Havok/hkVector4.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/NetImmerse/NiUpdateData.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "physics-interaction/TransformMath.h"
#include "rock_support/Fo4VrRuntime.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <format>
#include <algorithm>
#include <array>
#include <atomic>
#include <initializer_list>
#include <limits>
#include <string>
#include <string_view>
#include <xmmintrin.h>


namespace rock
{
    namespace hand_grab_detail
    {
        static_assert(kGrabCollisionSuppressionArmBodyCountPerHand == kBodyBoneGrabSuppressionArmBodyCountPerSide,
            "Normal grab arm-collider suppression capacity must match the body collider arm-chain query.");



        inline active_grab_body_lifecycle::BodyReleaseIntent releaseIntentFromDisposition(GrabReleaseDisposition disposition) noexcept
        {
            using active_grab_body_lifecycle::BodyReleaseIntent;
            switch (disposition) {
            case GrabReleaseDisposition::PhysicalDrop:
                return BodyReleaseIntent::PhysicalDrop;
            case GrabReleaseDisposition::OwnershipHandoff:
                return BodyReleaseIntent::OwnershipHandoff;
            case GrabReleaseDisposition::PendingInventoryTransfer:
            case GrabReleaseDisposition::TransferToInventory:
            case GrabReleaseDisposition::PendingConsumeTransfer:
                return BodyReleaseIntent::NonPhysicalTransfer;
            }
            return BodyReleaseIntent::NonPhysicalTransfer;
        }



        inline RE::NiPoint3 resolveSupportFrameAxisWorld(const RE::NiPoint3& normalWorld,
            const RE::NiPoint3& modelAxisWorld,
            const RE::NiPoint3& pinchAxisWorld,
            const RE::NiPoint3& acrossPalmAxisWorld,
            const RE::NiPoint3& fingerAxisWorld)
        {
            const RE::NiPoint3 normal = normalizeOrZero(normalWorld);
            if (lengthSquared(normal) <= 0.000001f) {
                return {};
            }

            const RE::NiPoint3 candidates[] = {
                modelAxisWorld,
                pinchAxisWorld,
                acrossPalmAxisWorld,
                fingerAxisWorld,
            };
            for (const auto& candidate : candidates) {
                const RE::NiPoint3 projected = normalizeOrZero(projectOntoPlane(candidate, normal));
                if (lengthSquared(projected) > 0.000001f) {
                    return projected;
                }
            }

            return stablePerpendicularAxis(normal);
        }




        inline RE::NiPoint3 rotationAxisProxyLocal(const RE::NiMatrix3& proxyWorldRotation, const RE::NiPoint3& axisWorld)
        {
            RE::NiTransform proxyWorld = transform_math::makeIdentityTransform<RE::NiTransform>();
            proxyWorld.rotate = proxyWorldRotation;
            return normalizeOrZero(hand_bone_collider_geometry_math::generatedColliderWorldVectorToLocal(proxyWorld, axisWorld));
        }

        inline bool computeHardKeyframeVelocityForTarget(
            RE::hknpWorld* world,
            RE::hknpBodyId bodyId,
            const RE::NiTransform& targetWorld,
            float deltaTime,
            float outLinearVelocityHavok[4],
            float outAngularVelocityRadians[4])
        {
            /*
             * The generated keyframed proxy uses FO4VR's native hard-keyframe
             * helper for velocity telemetry when the live palm motion is not
             * directly readable. Held-object angular correction is solver-owned
             * by the grab constraint's ragdoll atom motor.
             */
            if (outLinearVelocityHavok) {
                outLinearVelocityHavok[0] = 0.0f;
                outLinearVelocityHavok[1] = 0.0f;
                outLinearVelocityHavok[2] = 0.0f;
                outLinearVelocityHavok[3] = 0.0f;
            }
            if (outAngularVelocityRadians) {
                outAngularVelocityRadians[0] = 0.0f;
                outAngularVelocityRadians[1] = 0.0f;
                outAngularVelocityRadians[2] = 0.0f;
                outAngularVelocityRadians[3] = 0.0f;
            }

            if (!world || bodyId.value == INVALID_BODY_ID || !havok_physics_timing::isUsableDelta(deltaTime) ||
                !outLinearVelocityHavok || !outAngularVelocityRadians ||
                !havok_runtime::getBody(world, bodyId)) {
                return false;
            }

            alignas(16) float targetPositionHavok[4]{
                targetWorld.translate.x * gameToHavokScale(),
                targetWorld.translate.y * gameToHavokScale(),
                targetWorld.translate.z * gameToHavokScale(),
                0.0f,
            };
            alignas(16) float targetRotationHavok[4]{};
            transform_math::niRowsToHavokQuaternion(targetWorld.rotate, targetRotationHavok);

            using ComputeHardKeyFrame_t = void (*)(RE::hknpWorld*, RE::hknpBodyId, float*, float*, float, float*, float*);
            static REL::Relocation<ComputeHardKeyFrame_t> compute{ REL::Offset(offsets::kFunc_ComputeHardKeyFrame) };
            compute(world, bodyId, targetPositionHavok, targetRotationHavok, deltaTime, outLinearVelocityHavok, outAngularVelocityRadians);

            outLinearVelocityHavok[3] = 0.0f;
            outAngularVelocityRadians[3] = 0.0f;
            return havok_runtime::isFinite3(outLinearVelocityHavok) && havok_runtime::isFinite3(outAngularVelocityRadians);
        }



        inline const char* primaryBodyChoiceReasonName(object_physics_body_set::PrimaryBodyChoiceReason reason)
        {
            using object_physics_body_set::PrimaryBodyChoiceReason;
            switch (reason) {
            case PrimaryBodyChoiceReason::PreferredHitAccepted:
                return "preferredHitAccepted";
            case PrimaryBodyChoiceReason::SurfaceOwnerAccepted:
                return "surfaceOwnerAccepted";
            case PrimaryBodyChoiceReason::NearestAcceptedFallback:
                return "nearestAcceptedFallback";
            case PrimaryBodyChoiceReason::NoAcceptedBody:
                return "noAcceptedBody";
            case PrimaryBodyChoiceReason::None:
            default:
                return "none";
            }
        }




        inline RE::NiPoint3 clampAngularVelocityVector(const RE::NiPoint3& value, float maxRadiansPerSecond)
        {
            if (!std::isfinite(maxRadiansPerSecond) || maxRadiansPerSecond <= 0.0f) {
                return RE::NiPoint3{};
            }

            const float magnitude = vectorMagnitude(value);
            if (!std::isfinite(magnitude) || magnitude <= 0.000001f) {
                return RE::NiPoint3{};
            }
            if (magnitude <= maxRadiansPerSecond) {
                return value;
            }

            const float scale = maxRadiansPerSecond / magnitude;
            return RE::NiPoint3{ value.x * scale, value.y * scale, value.z * scale };
        }

        inline grab_motion_controller::ContactSupportShape classifyContactSupportShapeFromGrabFrame(const CanonicalGrabFrame& frame)
        {
            if (frame.hasGripSupportModel) {
                switch (frame.gripSupportKind) {
                case grab_support_model_math::GripSupportKind::OpposedPinch:
                    return grab_motion_controller::ContactSupportShape::SphereLike;
                case grab_support_model_math::GripSupportKind::LongHandleAxis:
                    return grab_motion_controller::ContactSupportShape::LongHandle;
                case grab_support_model_math::GripSupportKind::PalmWrap:
                    return grab_motion_controller::ContactSupportShape::Wrap;
                case grab_support_model_math::GripSupportKind::SameSurface:
                    return frame.pivotAuthorityNormalTrusted ?
                        grab_motion_controller::ContactSupportShape::Surface :
                        grab_motion_controller::ContactSupportShape::ThinFace;
                case grab_support_model_math::GripSupportKind::SinglePoint:
                    return grab_motion_controller::ContactSupportShape::Point;
                default:
                    break;
                }
            }

            const auto fallback = grab_motion_controller::classifyContactSupportShape(
                grab_motion_controller::ContactSupportShape::Unknown,
                frame.pivotAuthorityNormalTrusted,
                frame.hasContactPatchEvidence,
                frame.contactPatchSampleCount,
                frame.multiFingerContactGroupCount,
                frame.multiFingerContactSpreadGameUnits,
                frame.longObjectLeverGameUnits,
                g_rockConfig.rockGrabSmallObjectReferenceLeverGameUnits,
                g_rockConfig.rockGrabLongObjectReferenceLeverGameUnits);

            const std::uint32_t sampleCount = (std::min)(frame.contactPatchSampleCount, static_cast<std::uint32_t>(frame.contactPatchSamples.size()));
            if (!frame.hasContactPatchEvidence || sampleCount < 2) {
                return fallback;
            }

            std::array<RE::NiPoint3, kMaxGrabContactPatchSamples> points{};
            std::uint32_t acceptedCount = 0;
            RE::NiPoint3 normalSum{};
            for (std::uint32_t i = 0; i < sampleCount; ++i) {
                const auto& sample = frame.contactPatchSamples[i];
                if (!sample.accepted) {
                    continue;
                }
                points[acceptedCount++] = sample.point;
                normalSum = normalSum + sample.normal;
            }
            if (acceptedCount < 2) {
                return fallback;
            }

            RE::NiPoint3 centroid{};
            for (std::uint32_t i = 0; i < acceptedCount; ++i) {
                centroid = centroid + points[i];
            }
            centroid = scalePoint(centroid, 1.0f / static_cast<float>(acceptedCount));

            RE::NiPoint3 normal = normalizeOrZero(frame.gripNormalLocal);
            if (lengthSquared(normal) <= 0.000001f) {
                normal = normalizeOrZero(normalSum);
            }
            if (lengthSquared(normal) <= 0.000001f) {
                return fallback;
            }

            RE::NiPoint3 tangent{};
            float tangentLengthSq = 0.0f;
            for (std::uint32_t i = 1; i < acceptedCount; ++i) {
                const RE::NiPoint3 candidate = points[i] - points[0];
                const RE::NiPoint3 projected = candidate - scalePoint(normal, dotProduct(candidate, normal));
                const float candidateLengthSq = lengthSquared(projected);
                if (candidateLengthSq > tangentLengthSq) {
                    tangent = projected;
                    tangentLengthSq = candidateLengthSq;
                }
            }
            tangent = normalizeOrZero(tangent);
            if (lengthSquared(tangent) <= 0.000001f) {
                return fallback;
            }
            const RE::NiPoint3 bitangent = normalizeOrZero(crossProduct(normal, tangent));
            if (lengthSquared(bitangent) <= 0.000001f) {
                return fallback;
            }

            float minT = (std::numeric_limits<float>::max)();
            float maxT = -(std::numeric_limits<float>::max)();
            float minB = (std::numeric_limits<float>::max)();
            float maxB = -(std::numeric_limits<float>::max)();
            float minDepth = (std::numeric_limits<float>::max)();
            float maxDepth = -(std::numeric_limits<float>::max)();
            for (std::uint32_t i = 0; i < acceptedCount; ++i) {
                const RE::NiPoint3 delta = points[i] - centroid;
                const float t = dotProduct(delta, tangent);
                const float b = dotProduct(delta, bitangent);
                const float depth = dotProduct(delta, normal);
                minT = (std::min)(minT, t);
                maxT = (std::max)(maxT, t);
                minB = (std::min)(minB, b);
                maxB = (std::max)(maxB, b);
                minDepth = (std::min)(minDepth, depth);
                maxDepth = (std::max)(maxDepth, depth);
            }

            const float spanT = maxT - minT;
            const float spanB = maxB - minB;
            const float majorSpan = (std::max)(spanT, spanB);
            const float minorSpan = (std::min)(spanT, spanB);
            const float depthSpan = maxDepth - minDepth;
            const float smallReference = (std::max)(1.0f, g_rockConfig.rockGrabSmallObjectReferenceLeverGameUnits);
            const float longReference = (std::max)(smallReference, g_rockConfig.rockGrabLongObjectReferenceLeverGameUnits);
            const bool smallObject = frame.longObjectLeverGameUnits > 0.0f && frame.longObjectLeverGameUnits <= smallReference;
            const bool longObject = frame.longObjectLeverGameUnits >= longReference * 1.35f;
            const bool thinLine = majorSpan > 0.75f && minorSpan <= (std::max)(0.75f, majorSpan * 0.20f);
            const bool flatSurface = majorSpan > 0.75f && minorSpan > 0.75f && depthSpan <= (std::max)(0.75f, majorSpan * 0.20f);

            if (smallObject && acceptedCount >= 3) {
                return grab_motion_controller::ContactSupportShape::SphereLike;
            }
            if (longObject && (thinLine || acceptedCount <= 2)) {
                return grab_motion_controller::ContactSupportShape::LongHandle;
            }
            if (thinLine) {
                return grab_motion_controller::ContactSupportShape::ThinEdge;
            }
            if (flatSurface) {
                return frame.pivotAuthorityNormalTrusted ?
                    grab_motion_controller::ContactSupportShape::Surface :
                    grab_motion_controller::ContactSupportShape::ThinFace;
            }
            return fallback;
        }

        inline grab_motion_controller::AngularAuthorityInput makeAngularAuthorityInput(const CanonicalGrabFrame& frame)
        {
            return grab_motion_controller::AngularAuthorityInput{
                .enabled = g_rockConfig.rockGrabPivotQualityAngularScalingEnabled,
                .positionOnlyPivot = frame.pivotAuthorityPositionOnly,
                .normalTrusted = frame.pivotAuthorityNormalTrusted,
                .contactPatchEvidence = frame.hasContactPatchEvidence,
                .contactPatchSampleCount = frame.contactPatchSampleCount,
                .multiFingerContactGroupCount = frame.multiFingerContactGroupCount,
                .multiFingerContactSpreadGameUnits = frame.multiFingerContactSpreadGameUnits,
                .longObjectLeverGameUnits = frame.longObjectLeverGameUnits,
                .smallObjectReferenceLeverGameUnits = g_rockConfig.rockGrabSmallObjectReferenceLeverGameUnits,
                .positionOnlyAngularScale = g_rockConfig.rockGrabPositionOnlyAngularScale,
                .smallObjectAngularScale = g_rockConfig.rockGrabSmallObjectAngularScale,
                .lowContactSupportAngularScale = g_rockConfig.rockGrabLowContactSupportAngularScale,
                .minAngularAuthorityScale = g_rockConfig.rockGrabMinAngularAuthorityScale,
                .weakPivotTwistScale = g_rockConfig.rockGrabWeakPivotTwistScale,
                .contactSupportShape = classifyContactSupportShapeFromGrabFrame(frame),
                .longObjectReferenceLeverGameUnits = g_rockConfig.rockGrabLongObjectReferenceLeverGameUnits,
            };
        }

        inline grab_motion_controller::HeldAuthorityState evaluateRuntimeHeldAuthority(
            const CanonicalGrabFrame& frame,
            bool heldBodyContactSoftening)
        {
            return grab_motion_controller::evaluateHeldAuthority(grab_motion_controller::HeldAuthorityInput{
                .angular = makeAngularAuthorityInput(frame),
                .heldBodyColliding = heldBodyContactSoftening,
            });
        }


        inline std::uint32_t bodySetRejectCount(
            const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
            physics_body_classifier::BodyRejectReason reason)
        {
            const auto index = static_cast<std::size_t>(reason);
            if (index >= bodySet.diagnostics.rejectCounts.size()) {
                return 0;
            }
            return bodySet.diagnostics.rejectCounts[index];
        }



        inline held_object_drive_policy::HeldBodySetDriveDecision classifyHeldBodySetDrive(
            const object_physics_body_set::ObjectPhysicsBodySet& beforePrepBodySet,
            const object_physics_body_set::ObjectPhysicsBodySet& preparedBodySet,
            bool incompleteNativeScan)
        {
            const auto uniqueMotionRecords = preparedBodySet.uniqueAcceptedMotionRecords();
            return held_object_drive_policy::evaluateHeldBodySetDrive(held_object_drive_policy::HeldBodySetDriveInput{
                .acceptedBodyCount = static_cast<std::uint32_t>(preparedBodySet.acceptedCount()),
                .uniqueMotionCount = static_cast<std::uint32_t>(uniqueMotionRecords.size()),
                .rejectedFixedOrNonDynamicCount =
                    bodySetRejectCount(preparedBodySet, physics_body_classifier::BodyRejectReason::StaticMotion) +
                    bodySetRejectCount(preparedBodySet, physics_body_classifier::BodyRejectReason::NotDynamicAfterActivePrep),
                .scanFailureCount = beforePrepBodySet.diagnostics.scanFailures + preparedBodySet.diagnostics.scanFailures,
                .invalidPhysicsSystemCount = beforePrepBodySet.diagnostics.invalidPhysicsSystems + preparedBodySet.diagnostics.invalidPhysicsSystems,
                .incompleteNativeScan = incompleteNativeScan,
            });
        }

        template <std::size_t N>
        inline float recordDeviationAverage(std::array<float, N>& history, std::size_t& count, std::size_t& next, float sample)
        {
            if constexpr (N == 0) {
                return std::isfinite(sample) ? sample : 0.0f;
            } else {
                const float sanitizedSample = std::isfinite(sample) && sample > 0.0f ? sample : 0.0f;
                history[next] = sanitizedSample;
                next = (next + 1) % N;
                if (count < N) {
                    ++count;
                }

                float total = 0.0f;
                for (std::size_t i = 0; i < count; ++i) {
                    total += history[i];
                }
                return count > 0 ? total / static_cast<float>(count) : sanitizedSample;
            }
        }



        inline bool sharedContextMatchesSelection(const GrabSharedObjectContext& sharedContext, const SelectedObject& selection)
        {
            return sharedContext.hasPeerState() && selection.refr && sharedContext.peerSavedObjectState->refr == selection.refr;
        }

        inline bool isLooseWeaponGrabTarget(const SelectedObject& selection)
        {
            if (!selection.refr || !grab_target::canUseRockActiveGrab(selection.targetKind)) {
                return false;
            }

            auto* selectedBase = selection.refr->GetObjectReference();
            return selectedBase && selectedBase->Is(RE::ENUM_FORM_ID::kWEAP);
        }

        inline bool isFiniteNiTransform(const RE::NiTransform& value)
        {
            bool rotationFinite = true;
            for (std::uint32_t row = 0; row < 3; ++row) {
                for (std::uint32_t column = 0; column < 3; ++column) {
                    rotationFinite = rotationFinite && std::isfinite(value.rotate.entry[row][column]);
                }
            }
            return rotationFinite &&
                   std::isfinite(value.translate.x) &&
                   std::isfinite(value.translate.y) &&
                   std::isfinite(value.translate.z) &&
                   std::isfinite(value.scale) &&
                   value.scale > 0.0001f;
        }

        // ---- Transform primitives. Everything below composes these. ----



        // True when node is root itself or sits anywhere under it in the scene graph.

        inline const RE::TESObjectWEAP* looseWeaponFormFromRef(RE::TESObjectREFR* refr)
        {
            auto* selectedBase = refr ? refr->GetObjectReference() : nullptr;
            return selectedBase ? selectedBase->As<RE::TESObjectWEAP>() : nullptr;
        }

        inline const RE::TESObjectWEAP* selectedLooseWeaponForm(const SelectedObject& selection)
        {
            return looseWeaponFormFromRef(selection.refr);
        }

        inline bool isThrowableLooseWeapon(const RE::TESObjectWEAP* weapon)
        {
            if (!weapon) {
                return false;
            }
            /*
             * WEAPON_TYPE is stored as a single enum value in FO4VR. Keep this
             * as direct equality instead of EnumSet::any so guns cannot alias
             * thrown types through bit-style tests.
             */
            return weapon->weaponData.type == RE::WEAPON_TYPE::kGrenade ||
                   weapon->weaponData.type == RE::WEAPON_TYPE::kMine;
        }

        inline frik_visual_authority::HandPoseKind looseWeaponPrimaryAttachPoseKind(const RE::TESObjectWEAP* weapon)
        {
            return weapon && weapon_type_policy::isMelee(weapon->weaponData.type.get()) ?
                       frik_visual_authority::HandPoseKind::HoldingMelee :
                       frik_visual_authority::HandPoseKind::HoldingGun;
        }

        inline bool publishLooseWeaponPrimaryAttachHandPose(bool isLeft, RE::TESObjectREFR* refr)
        {
            const auto* weapon = looseWeaponFormFromRef(refr);
            auto* weaponRoot = refr ? refr->Get3D() : nullptr;
            if (weapon && weaponRoot) {
                const auto authored = authored_weapon_grip_library::find(weapon, weaponRoot, f4vr::isInPowerArmor());
                if (authored.found && authored.rightFiringFingerPose.complete()) {
                    frik_visual_authority::FingerLocalTransformOverride exactRightPose{};
                    exactRightPose.enabledMask = authored.rightFiringFingerPose.enabledMask;
                    for (std::size_t index = 0; index < authored.rightFiringFingerPose.localTransforms.size(); ++index) {
                        exactRightPose.localTransforms[index] = authored.rightFiringFingerPose.localTransforms[index];
                    }

                    frik_visual_authority::FingerLocalTransformOverride exactPose = exactRightPose;
                    if (!isLeft || frik_visual_authority::mirrorPrimaryWeaponFingerLocalTransforms(exactRightPose, exactPose)) {
                        constexpr const char* tag = "ROCK_Grab";
                        constexpr int priority = 100;
                        const char* blockTag = isLeft ? "ROCK_GrabPrimaryPoseLeft" : "ROCK_GrabPrimaryPoseRight";
                        const bool blockedNativePose = frik_visual_authority::blockPrimaryHandWeaponPose(blockTag, true);
                        const bool scalarPublished =
                            blockedNativePose && frik_visual_authority::setHandPoseCustomWithPriority(tag, handFromBool(isLeft), frik_visual_authority::HandPoseData{}, priority);
                        const bool localsPublished =
                            scalarPublished && frik_visual_authority::setHandPoseCustomLocalTransformsWithPriority(tag, handFromBool(isLeft), &exactPose, priority);
                        if (localsPublished) {
                            ROCK_LOG_INFO(Hand, "{} hand loose weapon attach: applying exact native-idle firing pose source={} mask=0x{:04X}", isLeft ? "left" : "right",
                                authored.reason, exactPose.enabledMask);
                            return true;
                        }

                        (void)frik_visual_authority::clearHandPose(tag, handFromBool(isLeft));
                        if (blockedNativePose) {
                            (void)frik_visual_authority::blockPrimaryHandWeaponPose(blockTag, false);
                        }
                    }
                }
            }

            return frik_visual_authority::setHandPoseWithPriority(
                "ROCK_Grab",
                handFromBool(isLeft),
                looseWeaponPrimaryAttachPoseKind(weapon),
                100);
        }


        constexpr const char* kHeldObjectDriveName = "proxyConstraint";
        inline void copyPeerInertiaSnapshot(SavedObjectState& target, const SavedObjectState& peer)
        {
            target.savedPackedInertia[0] = peer.savedPackedInertia[0];
            target.savedPackedInertia[1] = peer.savedPackedInertia[1];
            target.savedPackedInertia[2] = peer.savedPackedInertia[2];
            target.savedPackedMass = peer.savedPackedMass;
            target.inertiaModified = peer.inertiaModified;
            target.motionInertiaStates = peer.motionInertiaStates;
        }


        inline GrabSurfaceHit makeCollisionQueryGrabSurfaceHit(const SelectedObject& selection, RE::NiAVObject* fallbackOwnerNode)
        {
            GrabSurfaceHit result{};
            if (!selection.hasHitPoint || !selection.hasHitNormal) {
                return result;
            }

            result.position = selection.hitPointWorld;
            result.normal = normalizeOrZero(selection.hitNormalWorld);
            result.triangleIndex = -1;
            result.distance = selection.distance;
            result.sourceNode = selection.hitNode ? selection.hitNode : fallbackOwnerNode;
            result.sourceShape = nullptr;
            result.sourceKind = GrabSurfaceSourceKind::CollisionQuery;
            result.shapeKey = selection.hitShapeKey;
            result.shapeCollisionFilterInfo = selection.hitShapeCollisionFilterInfo;
            result.hitFraction = selection.hitFraction;
            result.hasSelectionHit = true;
            result.selectionToMeshDistanceGameUnits = 0.0f;
            result.signedAlongPalmDistanceGameUnits = selection.signedAlongDistance;
            result.lateralPalmDistanceGameUnits = selection.lateralDistance;
            result.hasShapeKey = selection.hasHitShapeKey;
            result.hasTriangle = false;
            result.valid = lengthSquared(result.normal) > 0.0f;
            return result;
        }

    }

    namespace hand_grab_detail
    {


        inline std::string formatContactPatchRejectedBodies(const RuntimeGrabContactPatch& result)
        {
            if (result.rejectedBodyIdCount == 0) {
                return "-";
            }

            std::string out;
            for (std::uint32_t index = 0; index < result.rejectedBodyIdCount; ++index) {
                if (!out.empty()) {
                    out += ",";
                }
                out += std::format("{}", result.rejectedBodyIds[index]);
            }
            if (result.rejectedBodyHits > static_cast<int>(result.rejectedBodyIdCount)) {
                out += "+";
            }
            return out;
        }






























    }

    namespace hand_grab_detail
    {
        struct GrabCaptureTransformRefreshSample
        {
            const char* role = "unknown";
            RE::NiAVObject* node = nullptr;
            bool validBefore = true;
            bool validAfter = true;
            float positionDeltaGameUnits = 0.0f;
            float rotationDeltaDegrees = 0.0f;
        };

        struct GrabCaptureTransformRefreshResult
        {
            std::array<GrabCaptureTransformRefreshSample, 6> samples{};
            std::uint32_t count = 0;
            bool ok = true;
        };

        inline bool grabCaptureRefreshAlreadyVisited(const GrabCaptureTransformRefreshResult& result, const RE::NiAVObject* node)
        {
            for (std::uint32_t i = 0; i < result.count; ++i) {
                if (result.samples[i].node == node) {
                    return true;
                }
            }
            return false;
        }

        inline void refreshGrabCaptureNodeTransform(GrabCaptureTransformRefreshResult& result, const char* role, RE::NiAVObject* node)
        {
            if (!node || result.count >= result.samples.size() || grabCaptureRefreshAlreadyVisited(result, node)) {
                return;
            }

            auto& sample = result.samples[result.count++];
            sample.role = role ? role : "unknown";
            sample.node = node;

            const RE::NiTransform before = node->world;
            sample.validBefore = grab_three_phase::isFinite(before);

            RE::NiUpdateData update{};
            node->UpdateTransforms(update);

            const RE::NiTransform after = node->world;
            sample.validAfter = grab_three_phase::isFinite(after);
            sample.positionDeltaGameUnits = translationDeltaGameUnits(before, after);
            sample.rotationDeltaDegrees = rotationDeltaDegrees(before.rotate, after.rotate);
            result.ok = result.ok && sample.validAfter;
        }

        inline GrabCaptureTransformRefreshResult refreshGrabCaptureTransforms(
            RE::NiAVObject* rootNode,
            RE::NiAVObject* meshSourceNode,
            RE::NiAVObject* collidableNode)
        {
            GrabCaptureTransformRefreshResult result{};
            refreshGrabCaptureNodeTransform(result, "root", rootNode);
            refreshGrabCaptureNodeTransform(result, "mesh", meshSourceNode);
            refreshGrabCaptureNodeTransform(result, "collidable", collidableNode);
            return result;
        }













        /*
         * Cap the surface triangles handed to a contact builder. Returns the set to use:
         * the original when it already fits, otherwise storage filled with the nearest N.
         * storage must outlive the returned reference - the callers keep it in the frame
         * that consumes the result.
         */
        inline const std::vector<GrabSurfaceTriangleData>& limitGrabSurfaceTrianglesNear(
            const std::vector<GrabSurfaceTriangleData>& sourceTriangles,
            const RE::NiPoint3& centerWorld,
            std::vector<GrabSurfaceTriangleData>& storage,
            const char* handNameText,
            const char* useLabel)
        {
            if (sourceTriangles.size() <= kMaxGrabRuntimeSurfaceContactTriangles) {
                return sourceTriangles;
            }
            storage = selectNearestGrabSurfaceTriangles(sourceTriangles, centerWorld, kMaxGrabRuntimeSurfaceContactTriangles);
            ROCK_LOG_DEBUG(Hand,
                "{} hand MESH CONTACT TRIANGLES: use={} sourceTris={} localTris={} center=({:.1f},{:.1f},{:.1f})",
                handNameText,
                useLabel,
                sourceTriangles.size(),
                storage.size(),
                centerWorld.x,
                centerWorld.y,
                centerWorld.z);
            return storage;
        }

        inline RE::NiAVObject* findNamedNodeRecursive(RE::NiAVObject* root, std::string_view name, int maxDepth = 12)
        {
            if (!root || name.empty() || maxDepth < 0) {
                return nullptr;
            }

            const char* nodeName = root->name.c_str();
            if (nodeName && name == nodeName) {
                return root;
            }

            auto* node = root->IsNode();
            if (!node) {
                return nullptr;
            }

            auto& children = node->GetRuntimeData().children;
            for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                if (auto* found = findNamedNodeRecursive(children[i].get(), name, maxDepth - 1)) {
                    return found;
                }
            }
            return nullptr;
        }

        inline RE::NiAVObject* findAuthoredGrabNodeRecursive(RE::NiAVObject* root,
            std::string_view name,
            bool isLeft,
            bool rejectOppositeHandAnchor,
            int maxDepth = 12)
        {
            /*
             * ROCK gives authored grab nodes priority over mesh-derived contact
             * and keeps marker names out of mesh selection. A hand-side guard is
             * still required because FO4VR assets can legitimately carry both
             * ROCK:GrabR and ROCK:GrabL markers; the right hand should never bind
             * to the left marker because that silently flips authored poses.
             */
            auto* found = findNamedNodeRecursive(root, name, maxDepth);
            if (!found || !rejectOppositeHandAnchor) {
                return found;
            }

            const char* foundName = found->name.c_str();
            if (foundName && grab_node_name_policy::isOppositeHandGrabNodeName(foundName, isLeft)) {
                return nullptr;
            }
            return found;
        }

        inline RE::NiTransform getLiveBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId)
        {
            RE::NiTransform result = makeIdentityTransform();
            tryResolveLiveBodyWorldTransform(world, bodyId, result);
            return result;
        }


        inline RE::NiTransform getGrabAuthorityBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId)
        {
            RE::NiTransform result = makeIdentityTransform();
            tryGetGrabAuthorityBodyWorldTransform(world, bodyId, result);
            return result;
        }

        inline RE::NiTransform computeRuntimeBodyLocalTransform(const RE::NiTransform& nodeWorld, const RE::NiTransform& bodyWorld)
        {
            return multiplyTransforms(invertTransform(nodeWorld), bodyWorld);
        }

        constexpr const char* kGrabObjectRotationReferenceName = "generatedProxyAuthorityLocal";
        constexpr float kGrabFrameMismatchRawProxyRotationWarnDegrees = 20.0f;
        constexpr float kGrabFrameMismatchProxyRotationWarnDegrees = 5.0f;
        constexpr float kGrabFrameMismatchObjectRotationWarnDegrees = 25.0f;
        constexpr float kGrabFrameMismatchGripErrorWarnGameUnits = 5.0f;


















        /*
         * Seat depth stop: the frozen authority frame seats pivot B exactly onto
         * pivot A, so any mesh that extends past the grip point toward the palm
         * ends up inside the hand. Measure that extent as a support distance:
         * the farthest the cached object-local mesh reaches along -palmNormal
         * from the grip point, counting only geometry inside a lateral footprint
         * around the palm axis (mesh far to the side clears the palm and must
         * not push the seat out). Pushing pivot A out by this distance seats the
         * object's SURFACE on the palm. Triangle vertices alone under-sample
         * coarse meshes (a crate face keeps its vertices at corners, outside the
         * footprint), so each triangle is also probed with closest-point queries
         * against samples along the palm axis.
         */

        /*
         * Grip-axis tilt is measured FROM the cross-palm axis, and that axis
         * comes from the non-mirrored authored handspace convention
         * (HandFrame.h): the live bone transform already carries handedness, so
         * authored +Z is thumbward on one hand and pinkyward on the other. One
         * shared tilt therefore rotates the target axis toward the fingers on
         * one hand and away from them on the other - the config value cannot be
         * hand-neutral, so the sign follows the hand here.
         *
         * Ground truth, 59 user-verified holds captured 2026-07-25, measured in
         * this exact convention: right-hand rods sit at +34 deg (n=15, IQR
         * +23..+45), left-hand rods at -29 deg (n=2). Same magnitude, mirrored
         * sign, which is what this helper encodes.
         */

        /*
         * Principal axis of the rendered mesh via area-weighted PCA over the
         * extracted world-space triangles. NIF axes are not authored
         * consistently across props, so long-object orientation must come from
         * the geometry itself. elongationRatio = sqrt(lambda1/lambda2), the RMS
         * extent ratio between the dominant and second axis: ~1 for compact
         * objects, >2 for bottle/broom shapes. secondAxisWorld is the second
         * principal axis and secondElongationRatio = sqrt(lambda2/lambda3)
         * (lambda3 recovered from the covariance trace): ~1 for round cross
         * sections where roll about the long axis is meaningless, high for
         * plank/board shapes with a well-defined flat face. Both axis signs
         * are arbitrary; callers must align to the nearest hemisphere of
         * their target axis. Double accumulators because world coordinates
         * are large; covariance is built about the area-weighted mean.
         */

        /*
         * World-side rigid rotation of an NiTransform about a world pivot
         * point. Stored NiMatrix3 rows are the world images of the local axes
         * (hand_frame::transformHandspaceLocalToWorld documents the engine
         * convention), so a world rotation applies vector Rodrigues to each
         * stored row and to the pivot-relative translation. The pivot point
         * itself is the fixed point: worldPointToLocal(rotated, pivotWorld)
         * equals worldPointToLocal(original, pivotWorld).
         */





        /*
         * ANCHOR_CLOCK probe (diagnostic, 2026-08-16 grab locomotion stutter):
         * one shared player-root sample lets the physics-flush, producer, and
         * pre-FRIK stages be compared per frame to locate where the stick
         * locomotion/turn step lands relative to ROCK's writes, and whether
         * the engine re-syncs the held node from the body after ROCK owns it.
         */

    }
}
