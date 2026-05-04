

#include "TwoHandedGrip.h"

#include "DirectSkeletonBoneReader.h"
#include "GrabFingerLocalTransformRuntime.h"
#include "GrabFingerPoseRuntime.h"
#include "PalmTransform.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "TransformMath.h"
#include "WeaponAuthorityLifecyclePolicy.h"
#include "WeaponCollisionGeometryMath.h"
#include "WeaponSupportAuthorityPolicy.h"
#include "WeaponSupportThumbPosePolicy.h"
#include "WeaponTwoHandedGripMath.h"
#include "WeaponTwoHandedSolver.h"
#include "WeaponVisualAuthorityMath.h"
#include "api/FRIKApi.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string_view>

namespace frik::rock
{
    namespace
    {
        constexpr const char* PRIMARY_GRIP_TAG = "ROCK_WeaponPrimaryGrip";
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
             * HIGGS switches the thumb to an alternate animation curve; FRIK does
             * not have that curve. Derive an equivalent local thumb target from
             * the root flattened thumb chain by rotating each thumb segment's
             * local X axis toward the solved weapon contact point, then hand the
             * local transforms back to FRIK as the pose publication API.
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
            const std::array<float, 15>& jointValues,
            std::array<RE::NiTransform, 15>& outLocalTransforms,
            std::uint16_t& outMask)
        {
            auto* api = frik::api::FRIKApi::inst;
            const bool canPublish = api &&
                grab_finger_local_transform_math::shouldPublishLocalTransformPose(
                    g_rockConfig.rockGrabMeshLocalTransformPoseEnabled,
                    meshFingerPose.solved,
                    meshFingerPose.hasJointValues,
                    api->getHandPoseLocalTransformsForJointPositions != nullptr,
                    api->setHandPoseCustomLocalTransformsWithPriority != nullptr);
            if (!canPublish) {
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: full-hand local transform override skipped hand={} enabled={} api={} baselineApi={} publishApi={}",
                    isLeft ? "left" : "right",
                    g_rockConfig.rockGrabMeshLocalTransformPoseEnabled ? "yes" : "no",
                    api ? "yes" : "no",
                    (api && api->getHandPoseLocalTransformsForJointPositions) ? "yes" : "no",
                    (api && api->setHandPoseCustomLocalTransformsWithPriority) ? "yes" : "no");
                return false;
            }

            frik::api::FRIKApi::FingerLocalTransformOverride baseline{};
            if (!api->getHandPoseLocalTransformsForJointPositions(handFromBool(isLeft), jointValues.data(), &baseline)) {
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: full-hand local transform override failed hand={} reason=baseline-query", isLeft ? "left" : "right");
                return false;
            }

            frik::api::FRIKApi::FingerLocalTransformOverride corrected{};
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

    void TwoHandedGrip::update(
        RE::NiNode* weaponNode,
        const WeaponInteractionContact& leftWeaponContact,
        bool leftGripPressed,
        bool supportHandHoldingObject,
        float dt,
        std::uint64_t currentWeaponGenerationKey,
        const WeaponInteractionRuntimeState& runtimeState,
        weapon_support_authority_policy::WeaponSupportAuthorityMode supportAuthorityMode)
    {
        _hasSolvedWeaponTransform = false;

        if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady() || !weaponNode) {
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
                transitionToGripping(interactionWeaponNode, decision, supportAuthorityMode);
            }
            break;

        case TwoHandedState::Gripping:
            if (!_activeWeaponNode) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because active weapon source root is unavailable");
                transitionToInactive(false);
            } else if (!weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(_activeWeaponGenerationKey, currentWeaponGenerationKey)) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because weapon generation changed during support grip");
                transitionToInactive(false);
            } else if (!runtimeState.supportGripAllowed) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because offhand reservation disabled support grip");
                transitionToInactive(false);
            } else if (!weapon_two_handed_grip_math::shouldContinueSupportGrip(leftGripPressed, supportHandHoldingObject)) {
                transitionToInactive(ownsWeaponTransform());
            } else {
                updateGripping(_activeWeaponNode, dt);
            }
            break;
        }
    }

    void TwoHandedGrip::reset()
    {
        clearPrimaryGripPose(false);
        clearSupportGripPose(true);
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
        _authorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
        _supportGripPose = WeaponGripPoseId::BarrelWrap;
        _supportPartKind = WeaponPartKind::Other;
        _hasSolvedWeaponTransform = false;
        _activeWeaponNode = nullptr;
        _activeSourceRoot = nullptr;
        _activeWeaponGenerationKey = 0;
        _weaponNodeLocalBaseline = {};
        _hasWeaponNodeLocalBaseline = false;
        _primaryHandWeaponLocal = {};
        _supportHandWeaponLocal = {};
        _hasHandWeaponLocalFrames = false;
        _supportFingerPose = {};
        _supportFingerValues = {};
        _hasSupportFingerPose = false;
        _hasSupportFingerValues = false;
        _supportFingerLocalTransforms = {};
        _supportFingerLocalTransformMask = 0;
        _hasSupportFingerLocalTransforms = false;
        _primaryGripConfidence = 0.0f;
    }

    bool TwoHandedGrip::ownsWeaponTransform() const
    {
        return _state == TwoHandedState::Gripping && weapon_support_authority_policy::supportGripOwnsWeaponTransform(_authorityMode);
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
        outSnapshot.leftRequestedHandWorld = transform_math::composeTransforms(_lastSolvedWeaponTransform, _supportHandWeaponLocal);
        outSnapshot.rightGripWorld = transform_math::localPointToWorld(_lastSolvedWeaponTransform, _primaryGripLocal);
        outSnapshot.leftGripWorld = transform_math::localPointToWorld(_lastSolvedWeaponTransform, _offhandGripLocal);
        return true;
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
        weapon_support_authority_policy::WeaponSupportAuthorityMode supportAuthorityMode)
    {
        if (!weaponNode) {
            transitionToInactive(false);
            return;
        }

        constexpr bool supportHandIsLeft = true;
        constexpr bool primaryHandIsLeft = false;

        RE::NiAVObject* sourceRoot = decision.interactionRoot ? decision.interactionRoot : weaponNode;
        _authorityMode = supportAuthorityMode;
        _supportGripPose = decision.gripPose != WeaponGripPoseId::None ? decision.gripPose : _supportGripPose;
        _supportPartKind = decision.partKind;
        _activeWeaponNode = weaponNode;
        _activeSourceRoot = sourceRoot;
        _activeWeaponGenerationKey = decision.weaponGenerationKey;
        _weaponNodeLocalBaseline = weaponNode->local;
        _hasWeaponNodeLocalBaseline = true;
        _hasSupportFingerPose = false;
        _hasSupportFingerValues = false;
        _supportFingerLocalTransforms = {};
        _supportFingerLocalTransformMask = 0;
        _hasSupportFingerLocalTransforms = false;
        _primaryGripConfidence = 0.0f;
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

        const RE::NiPoint3 primaryPalmPos = computeGrabPivotAPositionFromHandBasis(primaryTransform, primaryHandIsLeft);
        _primaryGripLocal = worldToWeaponLocal(primaryPalmPos, sourceRoot);
        _primaryGripConfidence = 1.0f;
        const RE::NiPoint3 primaryGripWorldPoint = primaryPalmPos;
        const RE::NiTransform adjustedPrimaryTransform = primaryTransform;

        RE::NiPoint3 palmPos = computeGrabPivotAPositionFromHandBasis(supportTransform, supportHandIsLeft);
        RE::NiPoint3 palmDir = computePalmNormalFromHandBasis(supportTransform, supportHandIsLeft);

        std::vector<TriangleData> triangles;
        extractAllTriangles(sourceRoot, triangles);

        GrabPoint grabPoint;
        bool meshFound = false;
        if (!triangles.empty()) {
            meshFound = findClosestGrabPoint(triangles, palmPos, palmDir, g_rockConfig.rockGrabLateralWeight, g_rockConfig.rockGrabDirectionalWeight, grabPoint);
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
        const RE::NiPoint3 primaryToSupportWorld = sub(supportGripWorldPoint, primaryGripWorldPoint);
        _lockedGripSeparationWorld = std::sqrt(dot(primaryToSupportWorld, primaryToSupportWorld));

        grab_finger_pose_runtime::SolvedGrabFingerPose meshFingerPose{};
        const grab_finger_pose_runtime::SolvedGrabFingerPose* meshFingerPosePtr = nullptr;
        if (g_rockConfig.rockGrabMeshFingerPoseEnabled) {
            const RE::NiPoint3 gripWorldPoint = meshFound ? grabPoint.position : palmPos;
            root_flattened_finger_skeleton_runtime::Snapshot liveFingerSnapshot{};
            const auto* liveFingerSnapshotPtr =
                root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(supportHandIsLeft, liveFingerSnapshot) ? &liveFingerSnapshot : nullptr;
            const auto solvedFingerPose = grab_finger_pose_runtime::solveGrabFingerPoseFromTriangles(
                triangles, supportTransform, supportHandIsLeft, palmPos, gripWorldPoint, g_rockConfig.rockGrabFingerMinValue,
                g_rockConfig.rockGrabMaxTriangleDistance, true, liveFingerSnapshotPtr);
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
                    frik::api::FRIKApi::inst && frik::api::FRIKApi::inst->setHandPoseCustomLocalTransformsWithPriority);
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
            if (buildFullHandLocalTransformsForMeshPose(supportHandIsLeft, *meshFingerPosePtr, _supportFingerPose, localTransforms, localTransformMask)) {
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
        _rotationBlend = 0.0f;
        _gripLogCounter = 0;

        float gripDist = std::sqrt(dot(sub(_offhandGripLocal, _primaryGripLocal), sub(_offhandGripLocal, _primaryGripLocal)));
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: grip active weapon='{}', "
            "primaryLocal=({:.3f},{:.3f},{:.3f}), supportLocal=({:.3f},{:.3f},{:.3f}), "
            "gripSeparation={:.3f}, primaryGripSource={}, primaryGripConfidence={:.2f}, meshGrab={}, triangles={}, partKind={}, pose={}, authorityMode={}, sourceRoot='{}', generation={:016X}",
            weaponNode->name.c_str(), _primaryGripLocal.x, _primaryGripLocal.y, _primaryGripLocal.z, _offhandGripLocal.x, _offhandGripLocal.y, _offhandGripLocal.z, gripDist,
            "root-flattened", _primaryGripConfidence, meshFound ? "YES" : "FALLBACK", triangles.size(), static_cast<int>(_supportPartKind),
            static_cast<int>(_supportGripPose), static_cast<int>(_authorityMode), sourceRoot ? sourceRoot->name.c_str() : "(null)", _activeWeaponGenerationKey);
    }

    void TwoHandedGrip::transitionToInactive(bool publishRestoredWeaponTransform)
    {
        clearPrimaryGripPose(false);
        clearSupportGripPose(true);
        restoreFrikOffhandGrip();
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
        _hasHandWeaponLocalFrames = false;
        _supportFingerPose = {};
        _supportFingerValues = {};
        _hasSupportFingerPose = false;
        _hasSupportFingerValues = false;
        _supportFingerLocalTransforms = {};
        _supportFingerLocalTransformMask = 0;
        _hasSupportFingerLocalTransforms = false;
        _primaryGripConfidence = 0.0f;
        _activeWeaponNode = nullptr;
        _activeSourceRoot = nullptr;
        _activeWeaponGenerationKey = 0;
        _weaponNodeLocalBaseline = {};
        _hasWeaponNodeLocalBaseline = false;

        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: grip released");
    }

    void TwoHandedGrip::updateGripping(RE::NiNode* weaponNode, float dt)
    {
        if (_authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport) {
            updateVisualOnlySupportGrip(weaponNode);
            return;
        }

        updateFullWeaponAuthorityGrip(weaponNode, dt);
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

        RE::NiPoint3 primaryController = computeGrabPivotAPositionFromHandBasis(primaryTransform, primaryHandIsLeft);
        RE::NiPoint3 supportController = computeGrabPivotAPositionFromHandBasis(supportTransform, supportHandIsLeft);

        const RE::NiPoint3 currentSupportWorld = weaponLocalToWorld(_offhandGripLocal, weaponNode);
        const RE::NiPoint3 lockedSupportControllerTarget = makeLockedSupportGripTarget(
            primaryController,
            supportController,
            currentSupportWorld,
            _lockedGripSeparationWorld,
            0.001f);
        const RE::NiPoint3 blendedSupportTarget = lerpPoint(currentSupportWorld, lockedSupportControllerTarget, _rotationBlend);

        WeaponTwoHandedSolverInput<RE::NiTransform, RE::NiPoint3> solverInput{};
        solverInput.weaponWorldTransform = weaponNode->world;
        solverInput.primaryGripLocal = _primaryGripLocal;
        solverInput.supportGripLocal = _offhandGripLocal;
        solverInput.primaryTargetWorld = primaryController;
        solverInput.supportTargetWorld = blendedSupportTarget;
        solverInput.supportNormalLocal = _supportNormalLocal;
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

        if (!applyLockedHandVisualAuthority(weaponNode, true, true)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing support grip because ROCK locked hand authority failed");
            transitionToInactive(false);
            return;
        }

        _lastSolvedWeaponTransform = weaponNode->world;
        _hasSolvedWeaponTransform = true;

        RE::NiPoint3 primaryGripFinal = transform_math::localPointToWorld(_lastSolvedWeaponTransform, _primaryGripLocal);
        RE::NiPoint3 offhandGripFinal = transform_math::localPointToWorld(_lastSolvedWeaponTransform, _offhandGripLocal);

        if (++_gripLogCounter >= 90) {
            _gripLogCounter = 0;
            float separation = std::sqrt(dot(sub(primaryGripFinal, offhandGripFinal), sub(primaryGripFinal, offhandGripFinal)));
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: blend={:.2f}, separation={:.1f}gu, "
                "primaryGrip=({:.1f},{:.1f},{:.1f}), offhandGrip=({:.1f},{:.1f},{:.1f})",
                _rotationBlend, separation, primaryGripFinal.x, primaryGripFinal.y, primaryGripFinal.z, offhandGripFinal.x, offhandGripFinal.y, offhandGripFinal.z);
        }
    }

    void TwoHandedGrip::updateVisualOnlySupportGrip(RE::NiNode* weaponNode)
    {
        constexpr bool supportHandIsLeft = true;

        static_assert(weapon_visual_authority_math::handPosePrecedesLockedHandAuthority());
        publishGripHandPoses(supportHandIsLeft);

        if (!applyLockedHandVisualAuthority(weaponNode, false, true)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing visual-only support grip because ROCK support hand authority failed");
            transitionToInactive(false);
            return;
        }

        _lastSolvedWeaponTransform = weaponNode ? weaponNode->world : RE::NiTransform{};
        _hasSolvedWeaponTransform = false;

        if (weaponNode && ++_gripLogCounter >= 90) {
            _gripLogCounter = 0;
            const RE::NiPoint3 offhandGripFinal = transform_math::localPointToWorld(weaponNode->world, _offhandGripLocal);
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: visual-only support follows weapon='{}', offhandGrip=({:.1f},{:.1f},{:.1f})",
                weaponNode->name.c_str(),
                offhandGripFinal.x,
                offhandGripFinal.y,
                offhandGripFinal.z);
        }
    }

    void TwoHandedGrip::setSupportGripPose(bool isLeft, WeaponGripPoseId poseId, const grab_finger_pose_runtime::SolvedGrabFingerPose* meshFingerPose)
    {
        (void)isLeft;
        if (meshFingerPose && meshFingerPose->solved) {
            _supportFingerValues = meshFingerPose->values;
            _hasSupportFingerValues = true;
            _supportFingerPose = meshFingerPose->hasJointValues ? meshFingerPose->jointValues : grab_finger_pose_math::expandFingerCurlsToJointValues(meshFingerPose->values);
            _hasSupportFingerPose = true;
            return;
        }

        const auto& poseValues = poseValuesForGrip(poseId);
        _supportFingerPose = poseValues;
        _supportFingerValues = {
            poseValues[0],
            poseValues[3],
            poseValues[6],
            poseValues[9],
            poseValues[12],
        };
        _hasSupportFingerPose = true;
        _hasSupportFingerValues = true;
    }

    void TwoHandedGrip::clearSupportGripPose(bool isLeft)
    {
        _supportFingerPose = {};
        _supportFingerValues = {};
        _hasSupportFingerPose = false;
        _hasSupportFingerValues = false;
        _supportFingerLocalTransforms = {};
        _supportFingerLocalTransformMask = 0;
        _hasSupportFingerLocalTransforms = false;

        if (frik::api::FRIKApi::inst) {
            frik::api::FRIKApi::inst->clearHandPose(SUPPORT_GRIP_TAG, handFromBool(isLeft));
            if (frik::api::FRIKApi::inst->clearExternalHandWorldTransform) {
                frik::api::FRIKApi::inst->clearExternalHandWorldTransform(SUPPORT_GRIP_TAG, handFromBool(isLeft));
            }
        }
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

    bool TwoHandedGrip::applyLockedHandVisualAuthority(RE::NiNode* weaponNode, bool applyPrimaryHand, bool applySupportHand)
    {
        if (!weaponNode || !_hasHandWeaponLocalFrames) {
            return false;
        }

        auto* api = frik::api::FRIKApi::inst;
        if (!api || !api->applyExternalHandWorldTransform) {
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
            primaryApplied =
                api->applyExternalHandWorldTransform(PRIMARY_GRIP_TAG, frik::api::FRIKApi::Hand::Right, primaryHandWorld, GRIP_HAND_POSE_PRIORITY);
        }
        if (applySupportHand) {
            const RE::NiTransform supportHandWorld =
                weapon_support_authority_policy::buildVisualOnlySupportHandWorld(weaponNode->world, _supportHandWeaponLocal);
            supportApplied =
                api->applyExternalHandWorldTransform(SUPPORT_GRIP_TAG, frik::api::FRIKApi::Hand::Left, supportHandWorld, GRIP_HAND_POSE_PRIORITY);
        }
        if (primaryApplied && supportApplied) {
            return true;
        }

        if (api->clearExternalHandWorldTransform) {
            if (applyPrimaryHand && primaryApplied) {
                api->clearExternalHandWorldTransform(PRIMARY_GRIP_TAG, frik::api::FRIKApi::Hand::Right);
            }
            if (applySupportHand && supportApplied) {
                api->clearExternalHandWorldTransform(SUPPORT_GRIP_TAG, frik::api::FRIKApi::Hand::Left);
            }
        }
        return false;
    }

    void TwoHandedGrip::publishGripHandPoses(bool supportHandIsLeft)
    {
        auto* api = frik::api::FRIKApi::inst;
        if (!api) {
            return;
        }

        if (weapon_visual_authority_math::shouldPublishTwoHandedGripPose(weapon_visual_authority_math::LockedHandRole::Support) && _hasSupportFingerPose) {
            if (api->setHandPoseCustomJointPositionsWithPriority) {
                api->setHandPoseCustomJointPositionsWithPriority(SUPPORT_GRIP_TAG, handFromBool(supportHandIsLeft), _supportFingerPose.data(), GRIP_HAND_POSE_PRIORITY);
            } else if (api->setHandPoseCustomFingerPositionsWithPriority && _hasSupportFingerValues) {
                api->setHandPoseCustomFingerPositionsWithPriority(SUPPORT_GRIP_TAG,
                    handFromBool(supportHandIsLeft),
                    _supportFingerValues[0],
                    _supportFingerValues[1],
                    _supportFingerValues[2],
                    _supportFingerValues[3],
                    _supportFingerValues[4],
                    GRIP_HAND_POSE_PRIORITY);
            }
        }

        if (_hasSupportFingerLocalTransforms && api->setHandPoseCustomLocalTransformsWithPriority) {
            frik::api::FRIKApi::FingerLocalTransformOverride overrideData{};
            overrideData.enabledMask = _supportFingerLocalTransformMask;
            for (std::size_t i = 0; i < _supportFingerLocalTransforms.size(); ++i) {
                overrideData.localTransforms[i] = _supportFingerLocalTransforms[i];
            }
            api->setHandPoseCustomLocalTransformsWithPriority(SUPPORT_GRIP_TAG, handFromBool(supportHandIsLeft), &overrideData, GRIP_HAND_POSE_PRIORITY);
        }
    }

    void TwoHandedGrip::clearPrimaryGripPose(bool isLeft)
    {
        if (frik::api::FRIKApi::inst) {
            frik::api::FRIKApi::inst->clearHandPose(PRIMARY_GRIP_TAG, handFromBool(isLeft));
            if (frik::api::FRIKApi::inst->clearExternalHandWorldTransform) {
                frik::api::FRIKApi::inst->clearExternalHandWorldTransform(PRIMARY_GRIP_TAG, handFromBool(isLeft));
            }
        }
    }

    void TwoHandedGrip::killFrikOffhandGrip()
    {
        if (!frik::api::FRIKApi::inst)
            return;
        frik::api::FRIKApi::inst->blockOffHandWeaponGripping("ROCK_TwoHanded", true);
        ROCK_LOG_DEBUG(Weapon, "FRIK offhand grip suppressed");
    }

    void TwoHandedGrip::restoreFrikOffhandGrip()
    {
        if (!frik::api::FRIKApi::inst)
            return;
        frik::api::FRIKApi::inst->blockOffHandWeaponGripping("ROCK_TwoHanded", false);
        ROCK_LOG_DEBUG(Weapon, "FRIK offhand grip restored");
    }

}
