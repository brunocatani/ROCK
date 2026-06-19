#include "physics-interaction/weapon/TwoHandedGrip.h"

#include "api/ROCKProviderApi.h"
#include "physics-interaction/actor/ActorEquipmentGrab.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponCollision.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string_view>

namespace rock
{
    namespace
    {
        constexpr const char* PRIMARY_GRIP_TAG = "ROCK_WeaponPrimaryGrip";
        constexpr const char* PRIMARY_DETACH_TAG = "ROCK_WeaponPrimaryDetach";
        constexpr const char* SUPPORT_GRIP_TAG = "ROCK_WeaponSupportGrip";
        constexpr int GRIP_HAND_POSE_PRIORITY = 100;
        constexpr float SUPPORT_NORMAL_TWIST_FACTOR = 0.5f;

        constexpr std::array<float, 15> BARREL_WRAP_POSE = { 0.85f, 0.80f, 0.75f, 0.35f, 0.30f, 0.25f, 0.30f, 0.25f, 0.20f, 0.35f, 0.30f, 0.25f, 0.40f, 0.35f, 0.30f };
        constexpr std::array<float, 15> HANDGUARD_CLAMP_POSE = { 0.75f, 0.72f, 0.68f, 0.45f, 0.42f, 0.38f, 0.46f, 0.42f, 0.38f, 0.48f, 0.44f, 0.40f, 0.54f, 0.48f, 0.42f };
        constexpr std::array<float, 15> FOREGRIP_POSE = { 0.90f, 0.86f, 0.82f, 0.70f, 0.66f, 0.60f, 0.74f, 0.68f, 0.62f, 0.72f, 0.66f, 0.60f, 0.66f, 0.58f, 0.50f };
        constexpr std::array<float, 15> PUMP_GRIP_POSE = { 0.82f, 0.78f, 0.72f, 0.58f, 0.54f, 0.48f, 0.60f, 0.56f, 0.50f, 0.62f, 0.56f, 0.50f, 0.58f, 0.50f, 0.44f };
        constexpr std::array<float, 15> MAGWELL_HOLD_POSE = { 0.58f, 0.52f, 0.46f, 0.40f, 0.36f, 0.32f, 0.42f, 0.38f, 0.34f, 0.42f, 0.38f, 0.34f, 0.44f, 0.38f, 0.32f };
        constexpr std::array<float, 15> RECEIVER_SUPPORT_POSE = { 0.46f, 0.40f, 0.34f, 0.34f, 0.30f, 0.26f, 0.36f, 0.32f, 0.28f, 0.36f, 0.32f, 0.28f, 0.36f, 0.30f, 0.24f };

        const std::array<float, 15>& poseValuesForGrip(WeaponGripPoseId poseId)
        {
            switch (poseId) {
            case WeaponGripPoseId::HandguardClamp:
                return HANDGUARD_CLAMP_POSE;
            case WeaponGripPoseId::VerticalForegrip:
            case WeaponGripPoseId::AngledForegrip:
                return FOREGRIP_POSE;
            case WeaponGripPoseId::PumpGrip:
                return PUMP_GRIP_POSE;
            case WeaponGripPoseId::MagwellHold:
                return MAGWELL_HOLD_POSE;
            case WeaponGripPoseId::ReceiverSupport:
                return RECEIVER_SUPPORT_POSE;
            case WeaponGripPoseId::BarrelWrap:
            case WeaponGripPoseId::None:
            default:
                return BARREL_WRAP_POSE;
            }
        }

        RE::NiNode* sourceRootNodeOrFallback(RE::NiAVObject* sourceRoot, RE::NiNode* fallback)
        {
            if (sourceRoot) {
                if (auto* sourceNode = sourceRoot->IsNode()) {
                    return sourceNode;
                }
            }
            return fallback;
        }

        RE::NiPoint3 lerpPoint(const RE::NiPoint3& from, const RE::NiPoint3& to, float alpha)
        {
            const float t = (std::max)(0.0f, (std::min)(1.0f, alpha));
            return RE::NiPoint3{ from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t, from.z + (to.z - from.z) * t };
        }

        constexpr std::uint16_t SUPPORT_THUMB_LOCAL_TRANSFORM_MASK = 0x0007;
        constexpr float MIN_THUMB_OPPOSITION_DISTANCE = 0.001f;

        bool isFiniteRotation(const RE::NiMatrix3& rotation)
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

        bool isFiniteTransform(const RE::NiTransform& transform)
        {
            return isFiniteRotation(transform.rotate) && std::isfinite(transform.translate.x) && std::isfinite(transform.translate.y) &&
                   std::isfinite(transform.translate.z) && std::isfinite(transform.scale);
        }

        float lengthSquared(const RE::NiPoint3& value)
        {
            return value.x * value.x + value.y * value.y + value.z * value.z;
        }

        RE::NiPoint3 normalizeOrFallback(const RE::NiPoint3& value, const RE::NiPoint3& fallback)
        {
            const float valueLengthSquared = lengthSquared(value);
            if (std::isfinite(valueLengthSquared) && valueLengthSquared > 0.000001f) {
                const float invLength = 1.0f / std::sqrt(valueLengthSquared);
                return RE::NiPoint3{ value.x * invLength, value.y * invLength, value.z * invLength };
            }

            const float fallbackLengthSquared = lengthSquared(fallback);
            if (std::isfinite(fallbackLengthSquared) && fallbackLengthSquared > 0.000001f) {
                const float invLength = 1.0f / std::sqrt(fallbackLengthSquared);
                return RE::NiPoint3{ fallback.x * invLength, fallback.y * invLength, fallback.z * invLength };
            }

            return RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
        }

        struct LiveThumbTransform
        {
            RE::NiTransform world{};
            RE::NiTransform parentWorld{};
            RE::NiTransform local{};
            bool valid = false;
        };

        DirectSkeletonBoneReader& rootFlattenedTwoHandedReader()
        {
            static DirectSkeletonBoneReader reader;
            return reader;
        }

        const DirectSkeletonBoneEntry* findSnapshotBone(const DirectSkeletonBoneSnapshot& snapshot, std::string_view name)
        {
            for (const auto& bone : snapshot.bones) {
                if (bone.name == name) {
                    return &bone;
                }
            }
            return nullptr;
        }

        const DirectSkeletonBoneEntry* findSnapshotBoneByTreeIndex(const DirectSkeletonBoneSnapshot& snapshot, int treeIndex)
        {
            if (treeIndex < 0) {
                return nullptr;
            }

            for (const auto& bone : snapshot.bones) {
                if (bone.treeIndex == treeIndex) {
                    return &bone;
                }
            }
            return nullptr;
        }

        bool resolveLiveThumbTransforms(bool isLeft, std::array<LiveThumbTransform, 3>& outNodes)
        {
            outNodes = {};

            DirectSkeletonBoneSnapshot snapshot{};
            if (!rootFlattenedTwoHandedReader().capture(skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
                    skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                    snapshot)) {
                return false;
            }

            for (std::size_t segment = 0; segment < outNodes.size(); ++segment) {
                const char* boneName = root_flattened_finger_skeleton_runtime::fingerBoneName(isLeft, 0, segment);
                const auto* node = boneName ? findSnapshotBone(snapshot, boneName) : nullptr;
                const auto* parent = node ? findSnapshotBoneByTreeIndex(snapshot, node->parentTreeIndex) : nullptr;
                if (!node || !parent || !isFiniteTransform(node->world) || !isFiniteTransform(parent->world)) {
                    return false;
                }

                const RE::NiTransform local = transform_math::composeTransforms(transform_math::invertTransform(parent->world), node->world);
                if (!isFiniteTransform(local)) {
                    return false;
                }

                outNodes[segment] = LiveThumbTransform{
                    .world = node->world,
                    .parentWorld = parent->world,
                    .local = local,
                    .valid = true,
                };
            }
            return true;
        }

        bool buildAlternateThumbLocalTransforms(
            bool isLeft,
            const RE::NiPoint3& supportGripPivotWorldPoint,
            const RE::NiPoint3& gripWorldPoint,
            float thumbScalarValue,
            std::array<RE::NiTransform, 15>& outLocalTransforms,
            std::uint16_t& outMask)
        {
            /*
             * ROCK switches the support thumb to an alternate local-transform
             * target when the weapon mesh solve requires thumb opposition. FRIK
             * only exposes scalar curls by default, so derive local thumb targets
             * from the root-flattened chain and publish them through the local
             * pose API.
             */
            outLocalTransforms = {};
            outMask = 0;

            std::array<LiveThumbTransform, 3> thumbNodes{};
            if (!resolveLiveThumbTransforms(isLeft, thumbNodes)) {
                return false;
            }

            const float sanitizedThumbValue = std::isfinite(thumbScalarValue) ? std::clamp(thumbScalarValue, 0.0f, 1.0f) : 1.0f;
            const float oppositionStrength = std::clamp(0.45f + (1.0f - sanitizedThumbValue) * 0.55f, 0.45f, 1.0f);

            for (std::size_t segment = 0; segment < thumbNodes.size(); ++segment) {
                const auto& node = thumbNodes[segment];
                if (!node.valid) {
                    return false;
                }

                const RE::NiPoint3 currentAxisWorld = normalizeOrFallback(
                    transform_math::rotateLocalVectorToWorld(node.world.rotate, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }),
                    RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
                const RE::NiPoint3 toGrip = weapon_support_thumb_pose_policy::vectorToGripFromPredictedThumbNode(
                    node.world.translate,
                    supportGripPivotWorldPoint,
                    gripWorldPoint);
                if (lengthSquared(toGrip) <= MIN_THUMB_OPPOSITION_DISTANCE * MIN_THUMB_OPPOSITION_DISTANCE) {
                    return false;
                }

                const RE::NiPoint3 targetAxisWorld = normalizeOrFallback(toGrip, currentAxisWorld);
                const float dotToTarget = std::clamp(weaponSolverDot(currentAxisWorld, targetAxisWorld), -1.0f, 1.0f);
                const float angle = std::acos(dotToTarget) * oppositionStrength;
                if (!std::isfinite(angle)) {
                    return false;
                }

                RE::NiMatrix3 rotationDelta = transform_math::makeIdentityRotation<RE::NiMatrix3>();
                if (angle > 0.0001f) {
                    RE::NiPoint3 axis = weaponSolverCross(currentAxisWorld, targetAxisWorld);
                    if (lengthSquared(axis) <= 0.000001f) {
                        axis = weaponSolverOrthogonalAxis(currentAxisWorld);
                    }
                    rotationDelta = weaponSolverAxisAngleStored<RE::NiMatrix3, RE::NiPoint3>(axis, angle);
                }

                const RE::NiMatrix3 targetWorldRotation =
                    weaponSolverApplyWorldRotationToStoredBasis<RE::NiMatrix3, RE::NiPoint3>(rotationDelta, node.world.rotate);
                RE::NiTransform localTransform = node.local;
                localTransform.rotate = transform_math::multiplyStoredRotations(targetWorldRotation, transform_math::transposeRotation(node.parentWorld.rotate));
                if (!isFiniteTransform(localTransform)) {
                    return false;
                }

                outLocalTransforms[segment] = localTransform;
                outMask = static_cast<std::uint16_t>(outMask | (1U << segment));
            }

            return outMask == SUPPORT_THUMB_LOCAL_TRANSFORM_MASK;
        }

        bool buildFullHandLocalTransformsForMeshPose(
            bool isLeft,
            const grab_finger_pose_runtime::SolvedGrabFingerPose& meshFingerPose,
            const frik_visual_authority::HandPoseData& handPose,
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
                    api && api->setHandPoseCustomLocalTransformsWithPriority != nullptr);
            if (!canPublish) {
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: full-hand local transform override skipped hand={} enabled={} api={} baselineApi={} publishApi={}",
                    isLeft ? "left" : "right",
                    g_rockConfig.rockGrabMeshLocalTransformPoseEnabled ? "yes" : "no",
                    api ? "yes" : "no",
                    (api && api->getHandPoseLocalTransformsForPose) ? "yes" : "no",
                    (api && api->setHandPoseCustomLocalTransformsWithPriority) ? "yes" : "no");
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
                    &failureReason)) {
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

    }

    static bool tryGetHandBoneTransform(bool isLeft, RE::NiTransform& outTransform)
    {
        outTransform = {};
        DirectSkeletonBoneSnapshot snapshot{};
        if (!rootFlattenedTwoHandedReader().capture(skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                snapshot)) {
            return false;
        }

        const auto* handBone = findSnapshotBone(snapshot, isLeft ? "LArm_Hand" : "RArm_Hand");
        if (!handBone || !isFiniteTransform(handBone->world)) {
            return false;
        }

        outTransform = handBone->world;
        return true;
    }

    RE::NiPoint3 TwoHandedGrip::worldToWeaponLocal(const RE::NiPoint3& worldPos, const RE::NiAVObject* weaponNode)
    {
        if (!weaponNode) {
            return {};
        }
        return weapon_collision_geometry_math::worldPointToLocal(weaponNode->world.rotate, weaponNode->world.translate, weaponNode->world.scale, worldPos);
    }

    RE::NiPoint3 TwoHandedGrip::weaponLocalToWorld(const RE::NiPoint3& localPos, const RE::NiAVObject* weaponNode)
    {
        if (!weaponNode) {
            return {};
        }
        return weapon_collision_geometry_math::localPointToWorld(weaponNode->world.rotate, weaponNode->world.translate, weaponNode->world.scale, localPos);
    }

    RE::NiPoint3 TwoHandedGrip::resolveSupportGripWorld(RE::NiNode* weaponNode) const
    {
        if (auto* supportAttachmentRoot = resolveCurrentSupportAttachmentRoot(weaponNode)) {
            return transform_math::localPointToWorld(supportAttachmentRoot->world, _offhandGripSourceLocal);
        }
        return weaponLocalToWorld(_offhandGripLocal, weaponNode);
    }

    RE::NiPoint3 TwoHandedGrip::resolveSupportGripWeaponLocal(RE::NiNode* weaponNode) const
    {
        return worldToWeaponLocal(resolveSupportGripWorld(weaponNode), weaponNode);
    }

    RE::NiPoint3 TwoHandedGrip::resolveSupportNormalWeaponLocal(RE::NiNode* weaponNode) const
    {
        if (auto* supportAttachmentRoot = resolveCurrentSupportAttachmentRoot(weaponNode)) {
            const RE::NiPoint3 supportNormalWorld = transform_math::localVectorToWorld(supportAttachmentRoot->world, _supportNormalSourceLocal);
            return transform_math::worldVectorToLocal(weaponNode->world, supportNormalWorld);
        }
        return _supportNormalLocal;
    }

    RE::NiTransform TwoHandedGrip::resolveSupportHandWorld(RE::NiNode* weaponNode) const
    {
        if (auto* supportAttachmentRoot = resolveCurrentSupportAttachmentRoot(weaponNode)) {
            return transform_math::composeTransforms(supportAttachmentRoot->world, _supportHandSourceLocal);
        }
        if (!weaponNode) {
            return RE::NiTransform{};
        }
        return weapon_support_authority_policy::buildVisualOnlySupportHandWorld(weaponNode->world, _supportHandWeaponLocal);
    }

    RE::NiAVObject* TwoHandedGrip::resolveCurrentSupportAttachmentRoot(RE::NiNode* weaponNode) const
    {
        if (!_hasSupportSourceLocalFrame || !_supportAttachmentRoot || !weaponNode) {
            return nullptr;
        }
        return actor_equipment_grab::nodeContainsNode(weaponNode, _supportAttachmentRoot, 64) ? _supportAttachmentRoot : nullptr;
    }

    void TwoHandedGrip::update(
        RE::NiNode* weaponNode,
        const WeaponInteractionContact& leftWeaponContact,
        const WeaponInteractionContact& rightWeaponContact,
        bool leftGripPressed,
        bool supportHandHoldingObject,
        float dt,
        std::uint64_t currentWeaponGenerationKey,
        const WeaponCollision& weaponCollision,
        const WeaponInteractionRuntimeState& runtimeState,
        weapon_support_authority_policy::WeaponSupportAuthorityMode supportAuthorityMode,
        bool primaryDetachEnabled,
        const EquippedWeaponPrimaryGripInput& primaryGripInput,
        const weapon_two_handed_grip_math::PrimaryReattachInput& primaryReattachInput)
    {
        _hasSolvedWeaponTransform = false;

        if (!runtime_state::isLocalSkeletonReady() || !weaponNode) {
            if (_state != TwoHandedState::Inactive) {
                transitionToInactive(false);
            }
            return;
        }

        const WeaponInteractionDecision decision = routeWeaponInteraction(leftWeaponContact, runtimeState);
        const bool leftTouchingSupport = decision.kind == WeaponInteractionKind::SupportGrip;
        RE::NiNode* interactionWeaponNode = sourceRootNodeOrFallback(decision.interactionRoot, weaponNode);

        switch (_state) {
        case TwoHandedState::Inactive:
            if (leftTouchingSupport && !supportHandHoldingObject) {
                transitionToTouching(interactionWeaponNode, decision);
            }
            break;

        case TwoHandedState::Touching:
            if (supportHandHoldingObject) {
                _state = TwoHandedState::Inactive;
                break;
            }
            if (leftTouchingSupport) {
                _touchFrames = 0;
            } else {
                _touchFrames++;
                if (_touchFrames > TOUCH_TIMEOUT_FRAMES) {
                    _state = TwoHandedState::Inactive;
                    break;
                }
            }
            if (weapon_two_handed_grip_math::canStartSupportGrip(leftTouchingSupport, leftGripPressed, supportHandHoldingObject)) {
                transitionToGripping(interactionWeaponNode, decision, weaponCollision, supportAuthorityMode, runtimeState.providerPartAuthority);
            }
            break;

        case TwoHandedState::Gripping:
            if (!_activeWeaponNode) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because active weapon source root is unavailable");
                transitionToInactive(false);
            } else if (!weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(_activeWeaponGenerationKey, currentWeaponGenerationKey)) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because weapon generation changed during support grip");
                transitionToInactive(false);
            } else if (!providerPartAuthorityStillCurrent(currentWeaponGenerationKey)) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because provider weapon-part target is no longer current");
                transitionToInactive(false);
            } else if (!runtimeState.supportGripAllowed) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because offhand reservation disabled support grip");
                transitionToInactive(false);
            } else if (!weapon_two_handed_grip_math::shouldContinueSupportGrip(leftGripPressed, supportHandHoldingObject)) {
                const auto releaseAction = weapon_two_handed_grip_math::resolveSupportReleaseManualAction(primaryDetachEnabled, primaryGripInput.held);
                if (releaseAction == weapon_two_handed_grip_math::SupportReleaseManualAction::KeepPrimaryOwnership) {
                    transitionToPrimaryOnly(_activeWeaponNode, currentWeaponGenerationKey, "support-released-primary-held");
                } else if (releaseAction == weapon_two_handed_grip_math::SupportReleaseManualAction::DropEquippedWeapon) {
                    requestEquippedWeaponDrop(
                        "support-released-primary-not-held",
                        equipped_weapon_drop_policy::sourceForSupportRelease(primaryGripInput.released));
                } else {
                    transitionToInactive(ownsWeaponTransform());
                }
            } else if (primaryDetachEnabled &&
                       _authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver &&
                       !primaryGripInput.held) {
                if (transitionToPrimaryDetached()) {
                    updatePrimaryDetachedGrip(_activeWeaponNode, dt, primaryGripInput, primaryReattachInput, rightWeaponContact, weaponCollision);
                }
            } else {
                updateGripping(_activeWeaponNode, dt);
            }
            break;

        case TwoHandedState::PrimaryDetached:
            if (!_activeWeaponNode) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing primary-detached authority because active weapon source root is unavailable");
                transitionToInactive(false);
            } else if (!weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(_activeWeaponGenerationKey, currentWeaponGenerationKey)) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing primary-detached authority because weapon generation changed");
                transitionToInactive(false);
            } else if (!providerPartAuthorityStillCurrent(currentWeaponGenerationKey)) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing primary-detached authority because provider weapon-part target is no longer current");
                transitionToInactive(false);
            } else if (!runtimeState.supportGripAllowed) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing primary-detached authority because offhand reservation disabled support grip");
                transitionToInactive(false);
            } else if (!primaryDetachEnabled) {
                transitionToInactive(ownsWeaponTransform());
            } else if (!weapon_two_handed_grip_math::shouldContinueSupportGrip(leftGripPressed, supportHandHoldingObject)) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: support hand released while primary is detached and not holding a weapon part; dropping equipped weapon");
                requestEquippedWeaponDrop("support-released-primary-detached-no-weapon-owner", equipped_weapon_drop_policy::SourceHand::Left);
            } else {
                updatePrimaryDetachedGrip(_activeWeaponNode, dt, primaryGripInput, primaryReattachInput, rightWeaponContact, weaponCollision);
            }
            break;

        case TwoHandedState::PrimaryOnly:
            if (!_activeWeaponNode) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing primary-only authority because active weapon source root is unavailable");
                transitionToInactive(false);
            } else if (!weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(_activeWeaponGenerationKey, currentWeaponGenerationKey)) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing primary-only authority because weapon generation changed");
                transitionToInactive(false);
            } else if (!primaryDetachEnabled) {
                transitionToInactive(false);
            } else if (leftTouchingSupport && weapon_two_handed_grip_math::canStartSupportGrip(leftTouchingSupport, leftGripPressed, supportHandHoldingObject)) {
                transitionToGripping(interactionWeaponNode, decision, weaponCollision, supportAuthorityMode, runtimeState.providerPartAuthority);
            } else {
                updatePrimaryOnlyGrip(_activeWeaponNode, currentWeaponGenerationKey, primaryGripInput);
            }
            break;
        }
    }

    void TwoHandedGrip::reset()
    {
        _equippedWeaponDropRequest = {};
        _primaryControlState = weapon_two_handed_grip_math::PrimaryWeaponControlState::AttachedFiringGrip;
        _primaryReattachState = {};
        clearPrimaryGripPose(false);
        clearPrimaryDetachVisualAuthority(false);
        clearSupportGripPose(true);
        restoreFrikPrimaryWeaponPose();
        if (_state != TwoHandedState::Inactive) {
            transitionToInactive(false);
            return;
        }
        _state = TwoHandedState::Inactive;
        _touchFrames = 0;
        _rotationBlend = 0.0f;
        _offhandGripLocal = {};
        _primaryGripLocal = {};
        _grabNormal = {};
        _supportNormalLocal = {};
        _offhandGripSourceLocal = {};
        _supportNormalSourceLocal = {};
        _supportHandSourceLocal = {};
        _supportAttachmentWeaponLocal = {};
        _authorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
        _supportGripPose = WeaponGripPoseId::BarrelWrap;
        _supportPartKind = WeaponPartKind::Other;
        _hasSolvedWeaponTransform = false;
        _activeWeaponNode = nullptr;
        _activeSourceRoot = nullptr;
        _supportAttachmentRoot = nullptr;
        _activeWeaponGenerationKey = 0;
        clearProviderPartAuthority();
        _weaponNodeLocalBaseline = {};
        _hasWeaponNodeLocalBaseline = false;
        _primaryHandWeaponLocal = {};
        _supportHandWeaponLocal = {};
        _hasHandWeaponLocalFrames = false;
        _hasSupportSourceLocalFrame = false;
        _hasSupportAttachmentWeaponLocal = false;
        _supportFingerPose = {};
        _supportFingerSplayRadians = {};
        _hasSupportFingerPose = false;
        _hasSupportFingerSplay = false;
        _supportFingerLocalTransforms = {};
        _supportFingerLocalTransformMask = 0;
        _hasSupportFingerLocalTransforms = false;
        _primaryGripConfidence = 0.0f;
        resetLockedHandVisualLerp();
    }

    bool TwoHandedGrip::ownsWeaponTransform() const
    {
        return (_state == TwoHandedState::Gripping || _state == TwoHandedState::PrimaryDetached) &&
               weapon_support_authority_policy::supportGripOwnsWeaponTransform(_authorityMode);
    }

    bool TwoHandedGrip::getSolvedWeaponTransform(RE::NiTransform& outTransform) const
    {
        if (!_hasSolvedWeaponTransform) {
            return false;
        }
        outTransform = _lastSolvedWeaponTransform;
        return true;
    }

    bool TwoHandedGrip::getDebugAuthoritySnapshot(TwoHandedGripDebugSnapshot& outSnapshot) const
    {
        if (!_hasSolvedWeaponTransform || !_hasHandWeaponLocalFrames) {
            return false;
        }

        outSnapshot.weaponWorld = _lastSolvedWeaponTransform;
        outSnapshot.rightRequestedHandWorld = transform_math::composeTransforms(_lastSolvedWeaponTransform, _primaryHandWeaponLocal);
        outSnapshot.leftRequestedHandWorld = resolveSupportHandWorld(_activeWeaponNode);
        outSnapshot.rightGripWorld = transform_math::localPointToWorld(_lastSolvedWeaponTransform, _primaryGripLocal);
        outSnapshot.leftGripWorld = resolveSupportGripWorld(_activeWeaponNode);
        return true;
    }

    void TwoHandedGrip::resetLockedHandVisualLerp()
    {
        _primaryHandVisualLerp = {};
        _supportHandVisualLerp = {};
    }

    RE::NiTransform TwoHandedGrip::resolveLockedHandVisualTarget(
        const RE::NiTransform& targetWorld,
        const RE::NiTransform* liveHandWorld,
        float dt,
        LockedHandVisualLerpState& state)
    {
        /*
         * Two-handed weapon smoothing is visual-only. The weapon solver keeps
         * immediate aim authority; this only eases the FRIK external hand target
         * into the locked hand-to-weapon relation captured at grip start.
         */
        if (!g_rockConfig.rockWeaponSupportGripHandLerpEnabled) {
            state = {};
            return targetWorld;
        }

        if (!state.active) {
            const RE::NiTransform startWorld = (liveHandWorld && isFiniteTransform(*liveHandWorld)) ? *liveHandWorld : targetWorld;
            const float initialDistance =
                hand_visual_lerp_math::distanceGameUnits(startWorld.translate, targetWorld.translate);
            const float durationSeconds =
                hand_visual_lerp_math::computeDistanceMappedDurationGameUnits(
                    initialDistance,
                    g_rockConfig.rockWeaponSupportGripHandLerpTimeMin,
                    g_rockConfig.rockWeaponSupportGripHandLerpTimeMax,
                    g_rockConfig.rockWeaponSupportGripHandLerpMinDistance,
                    g_rockConfig.rockWeaponSupportGripHandLerpMaxDistance);
            if (durationSeconds <= 0.0f) {
                state = {};
                state.lastAlpha = 1.0f;
                return targetWorld;
            }

            state.active = true;
            state.startWorld = startWorld;
            state.elapsedSeconds = 0.0f;
            state.durationSeconds = durationSeconds;
            state.lastAlpha = 0.0f;
        }

        state.elapsedSeconds =
            hand_visual_lerp_math::advanceTimedBlendElapsed(state.elapsedSeconds, dt, state.durationSeconds);
        const auto blended =
            hand_visual_lerp_math::blendTransformOverDuration(state.startWorld, targetWorld, state.elapsedSeconds, state.durationSeconds);
        state.lastAlpha = hand_visual_lerp_math::timedBlendAlpha(state.elapsedSeconds, state.durationSeconds);
        return blended.transform;
    }

    void TwoHandedGrip::transitionToTouching(RE::NiNode* weaponNode, const WeaponInteractionDecision& decision)
    {
        if (!weaponNode) {
            _state = TwoHandedState::Inactive;
            return;
        }

        _state = TwoHandedState::Touching;
        _touchFrames = 0;
        _supportGripPose = decision.gripPose != WeaponGripPoseId::None ? decision.gripPose : WeaponGripPoseId::BarrelWrap;
        _supportPartKind = decision.partKind;
        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: touching weapon='{}' bodyId={} partKind={} pose={} interactionRoot={:x} sourceRoot={:x} generation={:016X}",
            weaponNode->name.c_str(),
            decision.bodyId,
            static_cast<int>(_supportPartKind),
            static_cast<int>(_supportGripPose),
            reinterpret_cast<std::uintptr_t>(decision.interactionRoot),
            reinterpret_cast<std::uintptr_t>(decision.sourceRoot),
            decision.weaponGenerationKey);
    }

    void TwoHandedGrip::transitionToGripping(
        RE::NiNode* weaponNode,
        const WeaponInteractionDecision& decision,
        const WeaponCollision& weaponCollision,
        weapon_support_authority_policy::WeaponSupportAuthorityMode supportAuthorityMode,
        const WeaponProviderPartAuthority& providerPartAuthority)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::TwoHandedGripStart);

        if (!weaponNode) {
            transitionToInactive(false);
            return;
        }

        constexpr bool supportHandIsLeft = true;
        constexpr bool primaryHandIsLeft = false;

        RE::NiAVObject* sourceRoot = weaponNode;
        RE::NiAVObject* supportAttachmentRoot = decision.sourceRoot ? decision.sourceRoot : sourceRoot;
        _authorityMode = supportAuthorityMode;
        _supportGripPose = decision.gripPose != WeaponGripPoseId::None ? decision.gripPose : _supportGripPose;
        _supportPartKind = decision.partKind;
        _activeWeaponNode = weaponNode;
        _activeSourceRoot = sourceRoot;
        _supportAttachmentRoot = supportAttachmentRoot;
        _activeWeaponGenerationKey = decision.weaponGenerationKey;
        _providerPartAuthority = providerPartAuthority.active ? providerPartAuthority : WeaponProviderPartAuthority{};
        _weaponNodeLocalBaseline = weaponNode->local;
        _hasWeaponNodeLocalBaseline = true;
        _hasSupportFingerPose = false;
        _supportFingerSplayRadians = {};
        _hasSupportFingerSplay = false;
        _supportFingerLocalTransforms = {};
        _supportFingerLocalTransformMask = 0;
        _hasSupportFingerLocalTransforms = false;
        _offhandGripSourceLocal = {};
        _supportNormalSourceLocal = {};
        _supportHandSourceLocal = {};
        _supportAttachmentWeaponLocal = {};
        _hasSupportSourceLocalFrame = false;
        _hasSupportAttachmentWeaponLocal = false;
        _primaryGripConfidence = 0.0f;
        resetLockedHandVisualLerp();
        clearPrimaryGripPose(primaryHandIsLeft);
        clearSupportGripPose(supportHandIsLeft);

        killFrikOffhandGrip();

        RE::NiTransform primaryTransform{};
        RE::NiTransform supportTransform{};
        if (!tryGetHandBoneTransform(primaryHandIsLeft, primaryTransform) || !tryGetHandBoneTransform(supportHandIsLeft, supportTransform)) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: support grip start skipped because root flattened hand transforms are unavailable");
            restoreFrikOffhandGrip();
            return;
        }

        const RE::NiPoint3 primaryPalmPos = computeGrabLegacyPalmPivotAWorldFromHandBasis(primaryTransform, primaryHandIsLeft);
        _primaryGripLocal = worldToWeaponLocal(primaryPalmPos, sourceRoot);
        _primaryGripConfidence = 1.0f;
        const RE::NiPoint3 primaryGripWorldPoint = primaryPalmPos;
        const RE::NiTransform adjustedPrimaryTransform = primaryTransform;

        RE::NiPoint3 palmPos = computeGrabLegacyPalmPivotAWorldFromHandBasis(supportTransform, supportHandIsLeft);
        RE::NiPoint3 palmDir = computePalmNormalFromHandBasis(supportTransform, supportHandIsLeft);

        std::vector<TriangleData> triangles;
        const bool cachedTrianglesFound = weaponCollision.tryBuildSupportGripEvidenceTriangles(decision.bodyId, weaponNode, triangles);

        GrabPoint grabPoint;
        bool meshFound = false;
        if (!triangles.empty()) {
            meshFound = findClosestGrabPoint(triangles,
                palmPos,
                palmDir,
                g_rockConfig.rockGrabLateralWeight,
                g_rockConfig.rockGrabDirectionalWeight,
                grabPoint,
                g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits);
        }

        if (meshFound) {
            _offhandGripLocal = worldToWeaponLocal(grabPoint.position, sourceRoot);
            _grabNormal = grabPoint.normal;
        } else {
            _offhandGripLocal = worldToWeaponLocal(palmPos, sourceRoot);
            _grabNormal = palmDir;
        }
        const RE::NiPoint3 supportGripWorldPoint = meshFound ? grabPoint.position : palmPos;
        const RE::NiTransform adjustedSupportTransform =
            weapon_two_handed_grip_math::alignHandFrameToGripPoint(supportTransform, palmPos, supportGripWorldPoint);
        _primaryHandWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(sourceRoot->world), adjustedPrimaryTransform);
        _supportHandWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(sourceRoot->world), adjustedSupportTransform);
        _hasHandWeaponLocalFrames = true;
        _supportNormalLocal = transform_math::worldVectorToLocal(sourceRoot->world, palmDir);
        if (supportAttachmentRoot) {
            _offhandGripSourceLocal = transform_math::worldPointToLocal(supportAttachmentRoot->world, supportGripWorldPoint);
            _supportNormalSourceLocal = transform_math::worldVectorToLocal(supportAttachmentRoot->world, palmDir);
            _supportHandSourceLocal = transform_math::composeTransforms(transform_math::invertTransform(supportAttachmentRoot->world), adjustedSupportTransform);
            _supportAttachmentWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(sourceRoot->world), supportAttachmentRoot->world);
            _hasSupportSourceLocalFrame = true;
            _hasSupportAttachmentWeaponLocal = true;
        }
        const RE::NiPoint3 primaryToSupportWorld = sub(supportGripWorldPoint, primaryGripWorldPoint);
        _lockedGripSeparationWorld = std::sqrt(dot(primaryToSupportWorld, primaryToSupportWorld));

        grab_finger_pose_runtime::SolvedGrabFingerPose meshFingerPose{};
        const grab_finger_pose_runtime::SolvedGrabFingerPose* meshFingerPosePtr = nullptr;
        if (g_rockConfig.rockGrabMeshFingerPoseEnabled) {
            const RE::NiPoint3 gripWorldPoint = meshFound ? grabPoint.position : palmPos;
            auto supportFingerPoseTargets = grab_finger_pose_runtime::makeSharedGripPoseTarget(gripWorldPoint, _grabNormal);
            supportFingerPoseTargets.useSeatPointForMissingTargets = false;
            supportFingerPoseTargets.useWholeMeshForMissingTargets = true;
            root_flattened_finger_skeleton_runtime::Snapshot liveFingerSnapshot{};
            const auto* liveFingerSnapshotPtr =
                root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(supportHandIsLeft, liveFingerSnapshot) ? &liveFingerSnapshot : nullptr;
            const auto solvedFingerPose = grab_finger_pose_runtime::solveGrabFingerPoseFromTriangles(
                triangles, supportTransform, supportHandIsLeft, palmPos, supportFingerPoseTargets, g_rockConfig.rockGrabFingerMinValue,
                g_rockConfig.rockGrabMaxTriangleDistance, true, liveFingerSnapshotPtr,
                g_rockConfig.rockGrabFingerRejectBacksideHits, g_rockConfig.rockGrabFingerSurfacePlaneToleranceGameUnits);
            if (solvedFingerPose.solved) {
                meshFingerPose = solvedFingerPose;
                meshFingerPosePtr = &meshFingerPose;
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: mesh finger pose hand={} values=({:.2f},{:.2f},{:.2f},{:.2f},{:.2f}) hits={} candidateTris={} altThumb={}",
                    supportHandIsLeft ? "left" : "right",
                    meshFingerPose.values[0],
                    meshFingerPose.values[1],
                    meshFingerPose.values[2],
                    meshFingerPose.values[3],
                    meshFingerPose.values[4],
                    solvedFingerPose.hitCount,
                    solvedFingerPose.candidateTriangleCount,
                    solvedFingerPose.usedAlternateThumbCurve ? "yes" : "no");
                if (solvedFingerPose.hasThumbCurveDiagnostics) {
                    ROCK_LOG_DEBUG(Weapon,
                        "TwoHandedGrip: thumb curve primary(hit={} value={:.2f} behind={}) alternate(hit={} value={:.2f} behind={}) selected={}",
                        solvedFingerPose.thumbPrimaryCurve.hit ? "yes" : "no",
                        solvedFingerPose.thumbPrimaryCurve.value,
                        solvedFingerPose.thumbPrimaryCurve.openedByBehindContact ? "yes" : "no",
                        solvedFingerPose.thumbAlternateCurve.hit ? "yes" : "no",
                        solvedFingerPose.thumbAlternateCurve.value,
                        solvedFingerPose.thumbAlternateCurve.openedByBehindContact ? "yes" : "no",
                        solvedFingerPose.usedAlternateThumbCurve ? "alternate" : "primary");
                }

                const bool canPublishAlternateThumb = !g_rockConfig.rockGrabMeshLocalTransformPoseEnabled &&
                    weapon_support_thumb_pose_policy::shouldPublishAlternateThumbLocalOverride(
                    solvedFingerPose.solved,
                    solvedFingerPose.usedAlternateThumbCurve,
                    frik_visual_authority::api() && frik_visual_authority::api()->setHandPoseCustomLocalTransformsWithPriority);
                if (canPublishAlternateThumb) {
                    std::array<RE::NiTransform, 15> localTransforms{};
                    std::uint16_t localTransformMask = 0;
                    if (buildAlternateThumbLocalTransforms(supportHandIsLeft, palmPos, gripWorldPoint, meshFingerPose.values[0], localTransforms, localTransformMask)) {
                        _supportFingerLocalTransforms = localTransforms;
                        _supportFingerLocalTransformMask = localTransformMask;
                        _hasSupportFingerLocalTransforms = true;
                        ROCK_LOG_DEBUG(Weapon,
                            "TwoHandedGrip: alternate thumb local transform override prepared hand={} mask=0x{:04X}",
                            supportHandIsLeft ? "left" : "right",
                            _supportFingerLocalTransformMask);
                    } else {
                        ROCK_LOG_WARN(Weapon, "TwoHandedGrip: alternate thumb selected but local transform override could not be built");
                    }
                }
            }
        }

        setSupportGripPose(supportHandIsLeft, _supportGripPose, meshFingerPosePtr);
        if (meshFingerPosePtr && _hasSupportFingerPose) {
            std::array<RE::NiTransform, 15> localTransforms{};
            std::uint16_t localTransformMask = 0;
            const auto supportHandPose = _hasSupportFingerSplay ?
                frik_visual_authority::makeHandPoseDataFromJointValues(_supportFingerPose, _supportFingerSplayRadians) :
                frik_visual_authority::makeHandPoseDataFromJointValues(_supportFingerPose);
            if (buildFullHandLocalTransformsForMeshPose(
                    supportHandIsLeft,
                    *meshFingerPosePtr,
                    supportHandPose,
                    localTransforms,
                    localTransformMask)) {
                _supportFingerLocalTransforms = localTransforms;
                _supportFingerLocalTransformMask = localTransformMask;
                _hasSupportFingerLocalTransforms = true;
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: full-hand local transform override prepared hand={} mask=0x{:04X}",
                    supportHandIsLeft ? "left" : "right",
                    _supportFingerLocalTransformMask);
            }
        }

        _state = TwoHandedState::Gripping;
        _primaryControlState = weapon_two_handed_grip_math::PrimaryWeaponControlState::AttachedFiringGrip;
        _primaryReattachState = {};
        _rotationBlend = 0.0f;
        _gripLogCounter = 0;

        float gripDist = std::sqrt(dot(sub(_offhandGripLocal, _primaryGripLocal), sub(_offhandGripLocal, _primaryGripLocal)));
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: grip active weapon='{}', "
            "primaryLocal=({:.3f},{:.3f},{:.3f}), supportLocal=({:.3f},{:.3f},{:.3f}), "
            "gripSeparation={:.3f}, primaryGripSource={}, primaryGripConfidence={:.2f}, meshGrab={}, triangles={}, cachedTriangles={}, partKind={}, pose={}, authorityMode={}, weaponRoot='{}', sourceRoot='{}', generation={:016X}",
            weaponNode->name.c_str(), _primaryGripLocal.x, _primaryGripLocal.y, _primaryGripLocal.z, _offhandGripLocal.x, _offhandGripLocal.y, _offhandGripLocal.z, gripDist,
            "root-flattened", _primaryGripConfidence, meshFound ? "YES" : "FALLBACK", triangles.size(), cachedTrianglesFound ? "yes" : "no", static_cast<int>(_supportPartKind),
            static_cast<int>(_supportGripPose), static_cast<int>(_authorityMode), weaponNode->name.c_str(), sourceRoot ? sourceRoot->name.c_str() : "(null)", _activeWeaponGenerationKey);
    }

    void TwoHandedGrip::transitionToInactive(bool publishRestoredWeaponTransform)
    {
        clearPrimaryGripPose(false);
        clearPrimaryDetachVisualAuthority(false);
        clearSupportGripPose(true);
        restoreFrikOffhandGrip();
        restoreFrikPrimaryWeaponPose();
        _primaryControlState = weapon_two_handed_grip_math::PrimaryWeaponControlState::AttachedFiringGrip;
        _primaryReattachState = {};
        bool restoredWeaponTransformAvailable = false;
        RE::NiTransform restoredWeaponTransform{};
        if (publishRestoredWeaponTransform && _hasWeaponNodeLocalBaseline && _activeWeaponNode) {
            if (_activeWeaponNode->parent) {
                restoredWeaponTransform = transform_math::composeTransforms(_activeWeaponNode->parent->world, _weaponNodeLocalBaseline);
            } else {
                restoredWeaponTransform = _weaponNodeLocalBaseline;
            }
            restoredWeaponTransformAvailable = true;
        }

        _state = TwoHandedState::Inactive;
        _touchFrames = 0;
        _rotationBlend = 0.0f;
        _offhandGripLocal = {};
        _primaryGripLocal = {};
        _grabNormal = {};
        _supportNormalLocal = {};
        _offhandGripSourceLocal = {};
        _supportNormalSourceLocal = {};
        _supportAttachmentWeaponLocal = {};
        _lockedGripSeparationWorld = 0.0f;
        _authorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
        _supportGripPose = WeaponGripPoseId::BarrelWrap;
        _supportPartKind = WeaponPartKind::Other;
        _hasSolvedWeaponTransform = publishRestoredWeaponTransform && restoredWeaponTransformAvailable;
        if (_hasSolvedWeaponTransform) {
            _lastSolvedWeaponTransform = restoredWeaponTransform;
        }
        _primaryHandWeaponLocal = {};
        _supportHandWeaponLocal = {};
        _supportHandSourceLocal = {};
        _supportAttachmentWeaponLocal = {};
        _hasHandWeaponLocalFrames = false;
        _hasSupportSourceLocalFrame = false;
        _hasSupportAttachmentWeaponLocal = false;
        _supportFingerPose = {};
        _supportFingerSplayRadians = {};
        _hasSupportFingerPose = false;
        _hasSupportFingerSplay = false;
        _supportFingerLocalTransforms = {};
        _supportFingerLocalTransformMask = 0;
        _hasSupportFingerLocalTransforms = false;
        _primaryGripConfidence = 0.0f;
        _activeWeaponNode = nullptr;
        _activeSourceRoot = nullptr;
        _supportAttachmentRoot = nullptr;
        _activeWeaponGenerationKey = 0;
        _weaponNodeLocalBaseline = {};
        _hasWeaponNodeLocalBaseline = false;
        resetLockedHandVisualLerp();

        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: grip released");
    }

    void TwoHandedGrip::updateGripping(RE::NiNode* weaponNode, float dt)
    {
        if (_authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport) {
            updateVisualOnlySupportGrip(weaponNode, dt);
            return;
        }

        updateFullWeaponAuthorityGrip(weaponNode, dt);
    }

    bool TwoHandedGrip::providerPartAuthorityStillCurrent(std::uint64_t currentWeaponGenerationKey) const
    {
        if (!_providerPartAuthority.active) {
            return true;
        }
        if (currentWeaponGenerationKey == 0 || currentWeaponGenerationKey != _providerPartAuthority.weaponGenerationKey) {
            return false;
        }

        ::rock::provider::RockProviderWeaponPartTargetQueryV1 query{};
        query.weaponGenerationKey = _providerPartAuthority.weaponGenerationKey;
        query.bodyId = _providerPartAuthority.bodyId;
        query.partKind = _providerPartAuthority.partKind;
        query.reloadRole = _providerPartAuthority.reloadRole;
        query.supportRole = _providerPartAuthority.supportRole;
        query.socketRole = _providerPartAuthority.socketRole;
        query.actionRole = _providerPartAuthority.actionRole;
        query.sourceRoot = _providerPartAuthority.sourceRoot;
        std::memcpy(query.sourceName, _providerPartAuthority.sourceName.data(), _providerPartAuthority.sourceName.size());
        query.sourceName[sizeof(query.sourceName) - 1] = '\0';

        ::rock::provider::RockProviderWeaponPartTargetResolutionV1 resolution{};
        if (!::rock::provider::resolveWeaponPartTargetV1(query, resolution)) {
            return false;
        }
        return resolution.matched != 0 &&
               resolution.ownerToken == _providerPartAuthority.ownerToken &&
               resolution.groupId == _providerPartAuthority.groupId &&
               static_cast<std::uint32_t>(resolution.grabMode) == _providerPartAuthority.grabMode;
    }

    void TwoHandedGrip::clearProviderPartAuthority()
    {
        _providerPartAuthority = {};
    }

    void TwoHandedGrip::updateFullWeaponAuthorityGrip(RE::NiNode* weaponNode, float dt)
    {
        constexpr bool supportHandIsLeft = true;
        constexpr bool primaryHandIsLeft = false;

        _rotationBlend = (std::min)(1.0f, _rotationBlend + dt * ROTATION_BLEND_SPEED);

        RE::NiTransform primaryTransform{};
        RE::NiTransform supportTransform{};
        if (!tryGetHandBoneTransform(primaryHandIsLeft, primaryTransform) || !tryGetHandBoneTransform(supportHandIsLeft, supportTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing support grip because root flattened hand transforms are unavailable");
            transitionToInactive(false);
            return;
        }

        RE::NiPoint3 primaryController = computeGrabLegacyPalmPivotAWorldFromHandBasis(primaryTransform, primaryHandIsLeft);
        RE::NiPoint3 supportController = computeGrabLegacyPalmPivotAWorldFromHandBasis(supportTransform, supportHandIsLeft);

        const RE::NiPoint3 currentSupportWorld = resolveSupportGripWorld(weaponNode);
        const RE::NiPoint3 currentPrimaryGripWorld = transform_math::localPointToWorld(weaponNode->world, _primaryGripLocal);
        const float currentGripSeparationWorld = std::sqrt(dot(sub(currentSupportWorld, currentPrimaryGripWorld), sub(currentSupportWorld, currentPrimaryGripWorld)));
        const float lockedGripSeparationWorld = _hasSupportSourceLocalFrame ? currentGripSeparationWorld : _lockedGripSeparationWorld;
        const RE::NiPoint3 supportGripLocal = resolveSupportGripWeaponLocal(weaponNode);
        const RE::NiPoint3 lockedSupportControllerTarget = makeLockedSupportGripTarget(
            primaryController,
            supportController,
            currentSupportWorld,
            lockedGripSeparationWorld,
            0.001f);
        const RE::NiPoint3 blendedSupportTarget = lerpPoint(currentSupportWorld, lockedSupportControllerTarget, _rotationBlend);

        WeaponTwoHandedSolverInput<RE::NiTransform, RE::NiPoint3> solverInput{};
        solverInput.weaponWorldTransform = weaponNode->world;
        solverInput.primaryGripLocal = _primaryGripLocal;
        solverInput.supportGripLocal = supportGripLocal;
        solverInput.primaryTargetWorld = primaryController;
        solverInput.supportTargetWorld = blendedSupportTarget;
        solverInput.supportNormalLocal = resolveSupportNormalWeaponLocal(weaponNode);
        solverInput.supportNormalTargetWorld = computePalmNormalFromHandBasis(supportTransform, supportHandIsLeft);
        solverInput.useSupportNormalTwist = true;
        solverInput.supportNormalTwistFactor = SUPPORT_NORMAL_TWIST_FACTOR;

        const auto solved = solveTwoHandedWeaponTransformFrikPivot(solverInput);
        if (!solved.solved) {
            return;
        }

        if (!applyWeaponVisualAuthority(weaponNode, solved.weaponWorldTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing support grip because ROCK visual weapon authority failed");
            transitionToInactive(false);
            return;
        }

        static_assert(weapon_visual_authority_math::handPosePrecedesLockedHandAuthority());
        static_assert(weapon_visual_authority_math::weaponVisualPrecedesLockedHandAuthority());
        publishGripHandPoses(supportHandIsLeft);

        const bool applyPrimaryHandAuthority = weapon_support_authority_policy::supportGripAppliesPrimaryHandAuthority(_authorityMode);
        if (!applyLockedHandVisualAuthority(weaponNode, applyPrimaryHandAuthority, true, dt, &primaryTransform, &supportTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing support grip because ROCK locked hand authority failed");
            transitionToInactive(false);
            return;
        }

        _lastSolvedWeaponTransform = weaponNode->world;
        _hasSolvedWeaponTransform = true;

        RE::NiPoint3 primaryGripFinal = transform_math::localPointToWorld(_lastSolvedWeaponTransform, _primaryGripLocal);
        RE::NiPoint3 offhandGripFinal = resolveSupportGripWorld(weaponNode);

        if (++_gripLogCounter >= 90) {
            _gripLogCounter = 0;
            float separation = std::sqrt(dot(sub(primaryGripFinal, offhandGripFinal), sub(primaryGripFinal, offhandGripFinal)));
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: blend={:.2f}, separation={:.1f}gu, "
                "primaryGrip=({:.1f},{:.1f},{:.1f}), offhandGrip=({:.1f},{:.1f},{:.1f}), handLerp=({:.2f}/{:.3f}s,{:.2f}/{:.3f}s)",
                _rotationBlend,
                separation,
                primaryGripFinal.x,
                primaryGripFinal.y,
                primaryGripFinal.z,
                offhandGripFinal.x,
                offhandGripFinal.y,
                offhandGripFinal.z,
                _primaryHandVisualLerp.lastAlpha,
                _primaryHandVisualLerp.durationSeconds,
                _supportHandVisualLerp.lastAlpha,
                _supportHandVisualLerp.durationSeconds);
        }
    }

    bool TwoHandedGrip::transitionToPrimaryDetached()
    {
        if (_state == TwoHandedState::PrimaryDetached) {
            return true;
        }

        clearPrimaryGripPose(false);
        if (!blockFrikPrimaryWeaponPose()) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: primary detach skipped because hFRIK primary weapon-pose blocker is unavailable");
            return false;
        }
        _primaryHandVisualLerp = {};
        _supportHandVisualLerp = {};
        _rotationBlend = 1.0f;
        _state = TwoHandedState::PrimaryDetached;
        _primaryControlState = weapon_two_handed_grip_math::PrimaryWeaponControlState::DetachedFree;
        _primaryReattachState = {};
        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: primary hand detached; ROCK owns primary hand and support grip owns equipped weapon authority");
        return true;
    }

    bool TwoHandedGrip::beginDetachedPrimaryWeaponPartGrip(
        RE::NiNode* weaponNode,
        const WeaponInteractionDecision& decision,
        const WeaponCollision& weaponCollision)
    {
        if (!weaponNode || decision.kind != WeaponInteractionKind::SupportGrip) {
            return false;
        }

        constexpr bool primaryHandIsLeft = false;
        RE::NiTransform primaryTransform{};
        if (!tryGetHandBoneTransform(primaryHandIsLeft, primaryTransform)) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: detached primary weapon-part grip skipped because primary hand transform is unavailable");
            return false;
        }

        RE::NiAVObject* sourceRoot = weaponNode;
        RE::NiPoint3 palmPos = computeGrabLegacyPalmPivotAWorldFromHandBasis(primaryTransform, primaryHandIsLeft);
        RE::NiPoint3 palmDir = computePalmNormalFromHandBasis(primaryTransform, primaryHandIsLeft);

        std::vector<TriangleData> triangles;
        const bool cachedTrianglesFound = weaponCollision.tryBuildSupportGripEvidenceTriangles(decision.bodyId, weaponNode, triangles);

        GrabPoint grabPoint;
        bool meshFound = false;
        if (!triangles.empty()) {
            meshFound = findClosestGrabPoint(triangles,
                palmPos,
                palmDir,
                g_rockConfig.rockGrabLateralWeight,
                g_rockConfig.rockGrabDirectionalWeight,
                grabPoint,
                g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits);
        }

        const RE::NiPoint3 gripWorldPoint = meshFound ? grabPoint.position : palmPos;
        const RE::NiTransform adjustedPrimaryTransform =
            weapon_two_handed_grip_math::alignHandFrameToGripPoint(primaryTransform, palmPos, gripWorldPoint);
        _primaryGripLocal = worldToWeaponLocal(gripWorldPoint, sourceRoot);
        _primaryGripConfidence = meshFound ? 1.0f : 0.5f;
        _primaryHandWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(sourceRoot->world), adjustedPrimaryTransform);
        _hasHandWeaponLocalFrames = true;
        _primaryHandVisualLerp = {};
        _rotationBlend = 0.0f;
        _primaryControlState = weapon_two_handed_grip_math::PrimaryWeaponControlState::DetachedGrabbingWeaponPart;

        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: detached primary weapon-part grab started bodyId={} partKind={} pose={} meshGrab={} triangles={} cachedTriangles={} generation={:016X}",
            decision.bodyId,
            static_cast<int>(decision.partKind),
            static_cast<int>(decision.gripPose),
            meshFound ? "YES" : "FALLBACK",
            triangles.size(),
            cachedTrianglesFound ? "yes" : "no",
            decision.weaponGenerationKey);
        return true;
    }

    bool TwoHandedGrip::beginPrimaryOnlyGrip(RE::NiNode* weaponNode, std::uint64_t currentWeaponGenerationKey)
    {
        if (!weaponNode || currentWeaponGenerationKey == 0 || _state != TwoHandedState::Inactive) {
            return false;
        }

        return transitionToPrimaryOnly(weaponNode, currentWeaponGenerationKey, "primary-grip-start");
    }

    EquippedWeaponManualDropRequest TwoHandedGrip::consumeEquippedWeaponDropRequest()
    {
        const EquippedWeaponManualDropRequest request = _equippedWeaponDropRequest;
        _equippedWeaponDropRequest = {};
        return request;
    }

    void TwoHandedGrip::notePrimaryRockWorldGrabState(bool active)
    {
        if (_state != TwoHandedState::PrimaryDetached) {
            return;
        }
        if (_primaryControlState == weapon_two_handed_grip_math::PrimaryWeaponControlState::DetachedGrabbingWeaponPart) {
            return;
        }

        _primaryControlState = active ?
            weapon_two_handed_grip_math::PrimaryWeaponControlState::DetachedGrabbingWorldObject :
            weapon_two_handed_grip_math::PrimaryWeaponControlState::DetachedFree;
    }

    bool TwoHandedGrip::transitionToPrimaryOnly(RE::NiNode* weaponNode, std::uint64_t currentWeaponGenerationKey, const char* reason)
    {
        if (!weaponNode || currentWeaponGenerationKey == 0) {
            return false;
        }

        constexpr bool primaryHandIsLeft = false;
        constexpr bool supportHandIsLeft = true;

        if (_state == TwoHandedState::Inactive) {
            _activeWeaponNode = weaponNode;
            _activeSourceRoot = weaponNode;
            _activeWeaponGenerationKey = currentWeaponGenerationKey;
            _weaponNodeLocalBaseline = weaponNode->local;
            _hasWeaponNodeLocalBaseline = true;

            RE::NiTransform primaryTransform{};
            if (tryGetHandBoneTransform(primaryHandIsLeft, primaryTransform)) {
                _primaryGripLocal = worldToWeaponLocal(computeGrabLegacyPalmPivotAWorldFromHandBasis(primaryTransform, primaryHandIsLeft), weaponNode);
                _primaryGripConfidence = 1.0f;
            } else {
                _primaryGripLocal = {};
                _primaryGripConfidence = 0.0f;
            }
        }

        clearPrimaryGripPose(primaryHandIsLeft);
        clearSupportGripPose(supportHandIsLeft);
        clearPrimaryDetachVisualAuthority(primaryHandIsLeft);
        restoreFrikOffhandGrip();
        restoreFrikPrimaryWeaponPose();
        _primaryReattachState = {};
        _primaryControlState = weapon_two_handed_grip_math::PrimaryWeaponControlState::AttachedFiringGrip;
        clearProviderPartAuthority();
        _hasSolvedWeaponTransform = false;
        _hasHandWeaponLocalFrames = false;
        _supportHandWeaponLocal = {};
        _supportHandSourceLocal = {};
        _supportAttachmentWeaponLocal = {};
        _hasSupportSourceLocalFrame = false;
        _hasSupportAttachmentWeaponLocal = false;
        _offhandGripSourceLocal = {};
        _supportNormalSourceLocal = {};
        _supportAttachmentRoot = nullptr;
        _supportFingerPose = {};
        _supportFingerSplayRadians = {};
        _hasSupportFingerPose = false;
        _hasSupportFingerSplay = false;
        _supportFingerLocalTransforms = {};
        _supportFingerLocalTransformMask = 0;
        _hasSupportFingerLocalTransforms = false;
        _primaryHandVisualLerp = {};
        _state = TwoHandedState::PrimaryOnly;

        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: primary-only equipped weapon ownership active reason={} generation={:016X}",
            reason ? reason : "unknown",
            _activeWeaponGenerationKey);
        return true;
    }

    void TwoHandedGrip::requestEquippedWeaponDrop(const char* reason, equipped_weapon_drop_policy::SourceHand sourceHand)
    {
        if (_equippedWeaponDropRequest.requested) {
            transitionToInactive(false);
            return;
        }

        _equippedWeaponDropRequest = EquippedWeaponManualDropRequest{
            .requested = true,
            .sourceHand = sourceHand,
        };
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: equipped weapon drop requested reason={} sourceHand={} generation={:016X}",
            reason ? reason : "unknown",
            equipped_weapon_drop_policy::sourceHandName(sourceHand),
            _activeWeaponGenerationKey);
        transitionToInactive(false);
    }

    void TwoHandedGrip::updatePrimaryOnlyGrip(
        RE::NiNode* weaponNode,
        std::uint64_t currentWeaponGenerationKey,
        const EquippedWeaponPrimaryGripInput& primaryGripInput)
    {
        equipped_weapon_manual_ownership_policy::RuntimeState manualState{
            .active = true,
            .weaponGenerationKey = _activeWeaponGenerationKey,
        };
        const auto manualDecision = equipped_weapon_manual_ownership_policy::update(manualState,
            equipped_weapon_manual_ownership_policy::Input{
                .weaponEquipped = weaponNode != nullptr,
                .weaponGenerationKey = currentWeaponGenerationKey,
                .startRequested = false,
                .primaryGripRetained = primaryGripInput.held,
                .supportGripRetained = false,
            });

        if (manualDecision.dropRequested) {
            requestEquippedWeaponDrop("primary-only-grip-released", equipped_weapon_drop_policy::SourceHand::Right);
            return;
        }

        if (manualDecision.cleared) {
            transitionToInactive(false);
            return;
        }

        _hasSolvedWeaponTransform = false;
    }

    bool TwoHandedGrip::completePrimaryReattach(RE::NiNode* weaponNode, float dt, const char* reason, bool continueSupportGrip)
    {
        if (!weaponNode) {
            return false;
        }

        constexpr bool primaryHandIsLeft = false;
        RE::NiTransform primaryTransform{};
        if (!tryGetHandBoneTransform(primaryHandIsLeft, primaryTransform)) {
            return false;
        }

        const RE::NiPoint3 primaryPalm = computeGrabLegacyPalmPivotAWorldFromHandBasis(primaryTransform, primaryHandIsLeft);
        const RE::NiPoint3 primaryGripWorld = weaponLocalToWorld(_primaryGripLocal, weaponNode);
        const RE::NiTransform adjustedPrimaryTransform =
            weapon_two_handed_grip_math::alignHandFrameToGripPoint(primaryTransform, primaryPalm, primaryGripWorld);
        _primaryHandWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), adjustedPrimaryTransform);
        _primaryHandVisualLerp = {};
        clearPrimaryDetachVisualAuthority(primaryHandIsLeft);
        restoreFrikPrimaryWeaponPose();
        _primaryReattachState = {};
        _primaryControlState = weapon_two_handed_grip_math::PrimaryWeaponControlState::AttachedFiringGrip;
        _state = continueSupportGrip ? TwoHandedState::Gripping : TwoHandedState::PrimaryOnly;
        if (continueSupportGrip) {
            updateFullWeaponAuthorityGrip(weaponNode, dt);
        }
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: primary hand reattached to firing grip reason={} continueSupportGrip={}",
            reason ? reason : "unknown",
            continueSupportGrip ? "yes" : "no");
        return true;
    }

    void TwoHandedGrip::updatePrimaryDetachedGrip(
        RE::NiNode* weaponNode,
        float dt,
        const EquippedWeaponPrimaryGripInput& primaryGripInput,
        const weapon_two_handed_grip_math::PrimaryReattachInput& primaryReattachInput,
        const WeaponInteractionContact& rightWeaponContact,
        const WeaponCollision& weaponCollision)
    {
        constexpr bool supportHandIsLeft = true;

        auto reattachInput = primaryReattachInput;
        reattachInput.detachedRockOwned = true;
        const auto reattachDecision = weapon_two_handed_grip_math::updatePrimaryReattach(_primaryReattachState, reattachInput);
        if (reattachDecision.armed) {
            ROCK_LOG_INFO(Weapon, "TwoHandedGrip: primary firing-grip reattach armed by Grab+Trigger combo");
        }
        if (reattachDecision.completed) {
            _primaryControlState = weapon_two_handed_grip_math::PrimaryWeaponControlState::ReattachingToFiringGrip;
            ROCK_LOG_INFO(Weapon, "TwoHandedGrip: detached primary weapon-part/world grab ending for firing-grip reattach");
            if (completePrimaryReattach(weaponNode, dt, "primary-grab-trigger-release", true)) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: primary firing-grip reattach complete");
            }
            return;
        }

        if (_primaryControlState == weapon_two_handed_grip_math::PrimaryWeaponControlState::DetachedGrabbingWeaponPart) {
            if (!primaryGripInput.held) {
                _primaryControlState = weapon_two_handed_grip_math::PrimaryWeaponControlState::DetachedFree;
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: detached primary weapon-part grab ended");
            } else {
                updateFullWeaponAuthorityGrip(weaponNode, dt);
                return;
            }
        } else if (primaryGripInput.pressed) {
            WeaponInteractionRuntimeState primaryRuntimeState{};
            const WeaponInteractionDecision primaryDecision = routeWeaponInteraction(rightWeaponContact, primaryRuntimeState);
            if (beginDetachedPrimaryWeaponPartGrip(weaponNode, primaryDecision, weaponCollision)) {
                updateFullWeaponAuthorityGrip(weaponNode, dt);
                return;
            }
        }

        RE::NiTransform supportTransform{};
        if (!tryGetHandBoneTransform(supportHandIsLeft, supportTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing primary-detached grip because root flattened hand transforms are unavailable");
            transitionToInactive(false);
            return;
        }

        if (!_hasHandWeaponLocalFrames) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing primary-detached grip because captured hand frames are unavailable");
            transitionToInactive(false);
            return;
        }

        RE::NiTransform solvedWeaponWorld{};
        if (_hasSupportSourceLocalFrame && _hasSupportAttachmentWeaponLocal && resolveCurrentSupportAttachmentRoot(weaponNode)) {
            const RE::NiTransform solvedSourceWorld =
                transform_math::composeTransforms(supportTransform, transform_math::invertTransform(_supportHandSourceLocal));
            solvedWeaponWorld = transform_math::composeTransforms(solvedSourceWorld, transform_math::invertTransform(_supportAttachmentWeaponLocal));
        } else {
            solvedWeaponWorld = transform_math::composeTransforms(supportTransform, transform_math::invertTransform(_supportHandWeaponLocal));
        }
        if (!isFiniteTransform(solvedWeaponWorld)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing primary-detached grip because support-only weapon solve produced invalid transform");
            transitionToInactive(false);
            return;
        }

        if (!applyWeaponVisualAuthority(weaponNode, solvedWeaponWorld)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing primary-detached grip because ROCK visual weapon authority failed");
            transitionToInactive(false);
            return;
        }

        publishGripHandPoses(supportHandIsLeft);
        if (!applyLockedHandVisualAuthority(weaponNode, false, true, dt, nullptr, &supportTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing primary-detached grip because ROCK support hand authority failed");
            transitionToInactive(false);
            return;
        }

        _lastSolvedWeaponTransform = weaponNode->world;
        _hasSolvedWeaponTransform = true;

        if (++_gripLogCounter >= 90) {
            _gripLogCounter = 0;
            const RE::NiPoint3 offhandGripFinal = resolveSupportGripWorld(weaponNode);
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: primary-detached support authority offhandGrip=({:.1f},{:.1f},{:.1f}) supportHand=({:.1f},{:.1f},{:.1f})",
                offhandGripFinal.x,
                offhandGripFinal.y,
                offhandGripFinal.z,
                supportTransform.translate.x,
                supportTransform.translate.y,
                supportTransform.translate.z);
        }
    }

    void TwoHandedGrip::updateVisualOnlySupportGrip(RE::NiNode* weaponNode, float dt)
    {
        constexpr bool supportHandIsLeft = true;

        static_assert(weapon_visual_authority_math::handPosePrecedesLockedHandAuthority());
        publishGripHandPoses(supportHandIsLeft);

        RE::NiTransform supportTransform{};
        const RE::NiTransform* liveSupportTransform = tryGetHandBoneTransform(supportHandIsLeft, supportTransform) ? &supportTransform : nullptr;
        if (!applyLockedHandVisualAuthority(weaponNode, false, true, dt, nullptr, liveSupportTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing visual-only support grip because ROCK support hand authority failed");
            transitionToInactive(false);
            return;
        }

        _lastSolvedWeaponTransform = weaponNode ? weaponNode->world : RE::NiTransform{};
        _hasSolvedWeaponTransform = false;

        if (weaponNode && ++_gripLogCounter >= 90) {
            _gripLogCounter = 0;
            const RE::NiPoint3 offhandGripFinal = resolveSupportGripWorld(weaponNode);
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: visual-only support follows weapon='{}', offhandGrip=({:.1f},{:.1f},{:.1f}), handLerp={:.2f}/{:.3f}s",
                weaponNode->name.c_str(),
                offhandGripFinal.x,
                offhandGripFinal.y,
                offhandGripFinal.z,
                _supportHandVisualLerp.lastAlpha,
                _supportHandVisualLerp.durationSeconds);
        }
    }

    void TwoHandedGrip::setSupportGripPose(bool isLeft, WeaponGripPoseId poseId, const grab_finger_pose_runtime::SolvedGrabFingerPose* meshFingerPose)
    {
        if (meshFingerPose && meshFingerPose->solved) {
            _supportFingerPose = meshFingerPose->hasJointValues ? meshFingerPose->jointValues : grab_finger_pose_math::expandFingerCurlsToJointValues(meshFingerPose->values);
            _supportFingerSplayRadians = {};
            _hasSupportFingerSplay = grab_finger_pose_runtime::resolveSurfaceContactSplayValues(isLeft, *meshFingerPose, _supportFingerSplayRadians);
            _hasSupportFingerPose = true;
            return;
        }

        const auto& poseValues = poseValuesForGrip(poseId);
        _supportFingerPose = poseValues;
        _supportFingerSplayRadians = {};
        _hasSupportFingerPose = true;
        _hasSupportFingerSplay = false;
    }

    void TwoHandedGrip::clearSupportGripPose(bool isLeft)
    {
        _supportFingerPose = {};
        _supportFingerSplayRadians = {};
        _hasSupportFingerPose = false;
        _hasSupportFingerSplay = false;
        _supportFingerLocalTransforms = {};
        _supportFingerLocalTransformMask = 0;
        _hasSupportFingerLocalTransforms = false;

        (void)frik_visual_authority::clearHandPose(SUPPORT_GRIP_TAG, handFromBool(isLeft));
        (void)frik_visual_authority::clearExternalHandWorldTransform(SUPPORT_GRIP_TAG, handFromBool(isLeft));
    }

    bool TwoHandedGrip::applyWeaponVisualAuthority(RE::NiNode* weaponNode, const RE::NiTransform& solvedWeaponWorld)
    {
        if (!weaponNode) {
            return false;
        }

        if (weaponNode->parent) {
            weaponNode->local = weapon_visual_authority_math::worldTargetToParentLocal(weaponNode->parent->world, solvedWeaponWorld);
            f4vr::updateTransformsDown(weaponNode, true);
        } else {
            weaponNode->local = solvedWeaponWorld;
            weaponNode->world = solvedWeaponWorld;
            f4vr::updateTransformsDown(weaponNode, false);
        }
        return true;
    }

    bool TwoHandedGrip::applyLockedHandVisualAuthority(
        RE::NiNode* weaponNode,
        bool applyPrimaryHand,
        bool applySupportHand,
        float dt,
        const RE::NiTransform* livePrimaryHandWorld,
        const RE::NiTransform* liveSupportHandWorld)
    {
        if (!weaponNode || !_hasHandWeaponLocalFrames) {
            return false;
        }

        if (!frik_visual_authority::isAvailable()) {
            return false;
        }

        if (!applyPrimaryHand && !applySupportHand) {
            return true;
        }

        bool primaryApplied = true;
        bool supportApplied = true;
        if (applyPrimaryHand) {
            const RE::NiTransform primaryHandWorld =
                weapon_visual_authority_math::weaponLocalFrameToWorld(weaponNode->world, _primaryHandWeaponLocal);
            const RE::NiTransform appliedPrimaryHandWorld =
                resolveLockedHandVisualTarget(primaryHandWorld, livePrimaryHandWorld, dt, _primaryHandVisualLerp);
            primaryApplied =
                frik_visual_authority::applyExternalHandWorldTransform(PRIMARY_GRIP_TAG, frik_visual_authority::Hand::Right, appliedPrimaryHandWorld, GRIP_HAND_POSE_PRIORITY);
        }
        if (applySupportHand) {
            const RE::NiTransform supportHandWorld = resolveSupportHandWorld(weaponNode);
            const RE::NiTransform appliedSupportHandWorld =
                resolveLockedHandVisualTarget(supportHandWorld, liveSupportHandWorld, dt, _supportHandVisualLerp);
            supportApplied =
                frik_visual_authority::applyExternalHandWorldTransform(SUPPORT_GRIP_TAG, frik_visual_authority::Hand::Left, appliedSupportHandWorld, GRIP_HAND_POSE_PRIORITY);
        }
        if (primaryApplied && supportApplied) {
            return true;
        }

        if (applyPrimaryHand && primaryApplied) {
            (void)frik_visual_authority::clearExternalHandWorldTransform(PRIMARY_GRIP_TAG, frik_visual_authority::Hand::Right);
        }
        if (applySupportHand && supportApplied) {
            (void)frik_visual_authority::clearExternalHandWorldTransform(SUPPORT_GRIP_TAG, frik_visual_authority::Hand::Left);
        }
        return false;
    }

    void TwoHandedGrip::publishGripHandPoses(bool supportHandIsLeft)
    {
        if (!frik_visual_authority::isAvailable()) {
            return;
        }

        if (weapon_visual_authority_math::shouldPublishTwoHandedGripPose(weapon_visual_authority_math::LockedHandRole::Support) && _hasSupportFingerPose) {
            const auto supportHandPose = _hasSupportFingerSplay ?
                frik_visual_authority::makeHandPoseDataFromJointValues(_supportFingerPose, _supportFingerSplayRadians) :
                frik_visual_authority::makeHandPoseDataFromJointValues(_supportFingerPose);
            (void)frik_visual_authority::setHandPoseCustomWithPriority(
                SUPPORT_GRIP_TAG,
                handFromBool(supportHandIsLeft),
                supportHandPose,
                GRIP_HAND_POSE_PRIORITY);
        }

        if (_hasSupportFingerLocalTransforms) {
            frik_visual_authority::FingerLocalTransformOverride overrideData{};
            overrideData.enabledMask = _supportFingerLocalTransformMask;
            for (std::size_t i = 0; i < _supportFingerLocalTransforms.size(); ++i) {
                overrideData.localTransforms[i] = _supportFingerLocalTransforms[i];
            }
            (void)frik_visual_authority::setHandPoseCustomLocalTransformsWithPriority(SUPPORT_GRIP_TAG, handFromBool(supportHandIsLeft), &overrideData, GRIP_HAND_POSE_PRIORITY);
        }
    }

    void TwoHandedGrip::clearPrimaryGripPose(bool isLeft)
    {
        (void)frik_visual_authority::clearHandPose(PRIMARY_GRIP_TAG, handFromBool(isLeft));
        (void)frik_visual_authority::clearExternalHandWorldTransform(PRIMARY_GRIP_TAG, handFromBool(isLeft));
    }

    void TwoHandedGrip::clearPrimaryDetachVisualAuthority(bool isLeft)
    {
        (void)frik_visual_authority::clearHandPose(PRIMARY_DETACH_TAG, handFromBool(isLeft));
        (void)frik_visual_authority::clearExternalHandWorldTransform(PRIMARY_DETACH_TAG, handFromBool(isLeft));
    }

    void TwoHandedGrip::killFrikOffhandGrip()
    {
        if (frik_visual_authority::blockOffHandWeaponGripping("ROCK_TwoHanded", true)) {
            ROCK_LOG_DEBUG(Weapon, "FRIK offhand grip suppressed");
        }
    }

    void TwoHandedGrip::restoreFrikOffhandGrip()
    {
        if (frik_visual_authority::blockOffHandWeaponGripping("ROCK_TwoHanded", false)) {
            ROCK_LOG_DEBUG(Weapon, "FRIK offhand grip restored");
        }
    }

    bool TwoHandedGrip::blockFrikPrimaryWeaponPose()
    {
        if (frik_visual_authority::blockPrimaryHandWeaponPose("ROCK_PrimaryDetach", true)) {
            ROCK_LOG_DEBUG(Weapon, "FRIK primary weapon pose suppressed");
            return true;
        }
        return false;
    }

    void TwoHandedGrip::restoreFrikPrimaryWeaponPose()
    {
        if (frik_visual_authority::blockPrimaryHandWeaponPose("ROCK_PrimaryDetach", false)) {
            ROCK_LOG_DEBUG(Weapon, "FRIK primary weapon pose restored");
        }
    }

}
