

#include "TwoHandedGrip.h"

#include "GrabFingerPoseRuntime.h"
#include "PalmTransform.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "TransformMath.h"
#include "WeaponCollisionGeometryMath.h"
#include "WeaponTwoHandedGripMath.h"
#include "WeaponTwoHandedSolver.h"
#include "WeaponVisualAuthorityMath.h"
#include "api/FRIKApi.h"
#include "f4vr/F4VRUtils.h"

#include <algorithm>
#include <array>
#include <cmath>

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

        RE::NiPoint3 lerpPoint(const RE::NiPoint3& from, const RE::NiPoint3& to, float alpha)
        {
            const float t = (std::max)(0.0f, (std::min)(1.0f, alpha));
            return RE::NiPoint3{ from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t, from.z + (to.z - from.z) * t };
        }

    }

    static RE::NiTransform getHandBoneTransform(bool isLeft)
    {
        if (!frik::api::FRIKApi::inst) {
            return transform_math::makeIdentityTransform<RE::NiTransform>();
        }
        return frik::api::FRIKApi::inst->getHandWorldTransform(handFromBool(isLeft));
    }

    RE::NiPoint3 TwoHandedGrip::worldToWeaponLocal(const RE::NiPoint3& worldPos, const RE::NiNode* weaponNode)
    {
        if (!weaponNode) {
            return {};
        }
        return weapon_collision_geometry_math::worldPointToLocal(weaponNode->world.rotate, weaponNode->world.translate, weaponNode->world.scale, worldPos);
    }

    RE::NiPoint3 TwoHandedGrip::weaponLocalToWorld(const RE::NiPoint3& localPos, const RE::NiNode* weaponNode)
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
        const WeaponReloadRuntimeState& reloadState)
    {
        _hasSolvedWeaponTransform = false;

        if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady() || !weaponNode) {
            if (_state != TwoHandedState::Inactive) {
                transitionToInactive(false);
            }
            return;
        }

        const WeaponInteractionDecision decision = routeWeaponInteraction(leftWeaponContact, reloadState);
        const bool leftTouchingSupport = decision.kind == WeaponInteractionKind::SupportGrip;

        switch (_state) {
        case TwoHandedState::Inactive:
            if (leftTouchingSupport && !supportHandHoldingObject) {
                transitionToTouching(weaponNode, decision);
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
                transitionToGripping(weaponNode, decision);
            }
            break;

        case TwoHandedState::Gripping:
            if (weaponNode != _activeWeaponNode) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because weapon node changed during support grip");
                transitionToInactive(false);
            } else if (!reloadState.supportGripAllowed) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because reload state disabled support grip");
                transitionToInactive(false);
            } else if (!weapon_two_handed_grip_math::shouldContinueSupportGrip(leftGripPressed, supportHandHoldingObject)) {
                transitionToInactive(true);
            } else {
                updateGripping(weaponNode, dt);
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
        _supportGripPose = WeaponGripPoseId::BarrelWrap;
        _supportPartKind = WeaponPartKind::Other;
        _hasSolvedWeaponTransform = false;
        _activeWeaponNode = nullptr;
        _weaponNodeLocalBaseline = {};
        _hasWeaponNodeLocalBaseline = false;
        _primaryHandWeaponLocal = {};
        _supportHandWeaponLocal = {};
        _hasHandWeaponLocalFrames = false;
        _supportFingerPose = {};
        _hasSupportFingerPose = false;
        _primaryGripConfidence = 0.0f;
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
        _state = TwoHandedState::Touching;
        _touchFrames = 0;
        _supportGripPose = decision.gripPose != WeaponGripPoseId::None ? decision.gripPose : WeaponGripPoseId::BarrelWrap;
        _supportPartKind = decision.partKind;
        ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: touching weapon='{}' bodyId={} partKind={} pose={}", weaponNode->name.c_str(), decision.bodyId,
            static_cast<int>(_supportPartKind), static_cast<int>(_supportGripPose));
    }

    void TwoHandedGrip::transitionToGripping(RE::NiNode* weaponNode, const WeaponInteractionDecision& decision)
    {
        constexpr bool supportHandIsLeft = true;
        constexpr bool primaryHandIsLeft = false;

        _supportGripPose = decision.gripPose != WeaponGripPoseId::None ? decision.gripPose : _supportGripPose;
        _supportPartKind = decision.partKind;
        _activeWeaponNode = weaponNode;
        _weaponNodeLocalBaseline = weaponNode->local;
        _hasWeaponNodeLocalBaseline = true;
        _hasSupportFingerPose = false;
        _primaryGripConfidence = 0.0f;
        clearPrimaryGripPose(primaryHandIsLeft);
        clearSupportGripPose(supportHandIsLeft);

        killFrikOffhandGrip();

        auto primaryTransform = getHandBoneTransform(primaryHandIsLeft);
        const RE::NiPoint3 primaryPalmPos = computeGrabPivotAPositionFromHandBasis(primaryTransform, primaryHandIsLeft);
        _primaryGripLocal = worldToWeaponLocal(primaryPalmPos, weaponNode);
        _primaryGripConfidence = 1.0f;
        const RE::NiPoint3 primaryGripWorldPoint = primaryPalmPos;
        const RE::NiTransform adjustedPrimaryTransform = primaryTransform;

        auto supportTransform = getHandBoneTransform(supportHandIsLeft);
        RE::NiPoint3 palmPos = computeGrabPivotAPositionFromHandBasis(supportTransform, supportHandIsLeft);
        RE::NiPoint3 palmDir = computePalmNormalFromHandBasis(supportTransform, supportHandIsLeft);

        std::vector<TriangleData> triangles;
        extractAllTriangles(weaponNode, triangles);

        GrabPoint grabPoint;
        bool meshFound = false;
        if (!triangles.empty()) {
            meshFound = findClosestGrabPoint(triangles, palmPos, palmDir, 1.0f, 0.3f, grabPoint);
        }

        if (meshFound) {
            _offhandGripLocal = worldToWeaponLocal(grabPoint.position, weaponNode);
            _grabNormal = grabPoint.normal;
        } else {
            _offhandGripLocal = worldToWeaponLocal(palmPos, weaponNode);
            _grabNormal = palmDir;
        }
        const RE::NiPoint3 supportGripWorldPoint = meshFound ? grabPoint.position : palmPos;
        const RE::NiTransform adjustedSupportTransform =
            weapon_two_handed_grip_math::alignHandFrameToGripPoint(supportTransform, palmPos, supportGripWorldPoint);
        _primaryHandWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), adjustedPrimaryTransform);
        _supportHandWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), adjustedSupportTransform);
        _hasHandWeaponLocalFrames = true;
        _supportNormalLocal = transform_math::worldVectorToLocal(weaponNode->world, palmDir);
        const RE::NiPoint3 primaryToSupportWorld = sub(supportGripWorldPoint, primaryGripWorldPoint);
        _lockedGripSeparationWorld = std::sqrt(dot(primaryToSupportWorld, primaryToSupportWorld));

        std::array<float, 5> meshFingerPose{};
        const std::array<float, 5>* meshFingerPosePtr = nullptr;
        if (g_rockConfig.rockGrabMeshFingerPoseEnabled) {
            const RE::NiPoint3 gripWorldPoint = meshFound ? grabPoint.position : palmPos;
            const auto solvedFingerPose = grab_finger_pose_runtime::solveGrabFingerPoseFromTriangles(
                triangles, supportTransform, supportHandIsLeft, palmPos, gripWorldPoint, g_rockConfig.rockGrabFingerMinValue);
            if (solvedFingerPose.solved) {
                meshFingerPose = solvedFingerPose.values;
                meshFingerPosePtr = &meshFingerPose;
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: mesh finger pose hand={} values=({:.2f},{:.2f},{:.2f},{:.2f},{:.2f}) hits={} candidateTris={}",
                    supportHandIsLeft ? "left" : "right", meshFingerPose[0], meshFingerPose[1], meshFingerPose[2], meshFingerPose[3], meshFingerPose[4],
                    solvedFingerPose.hitCount, solvedFingerPose.candidateTriangleCount);
            }
        }

        setSupportGripPose(supportHandIsLeft, _supportGripPose, meshFingerPosePtr);

        _state = TwoHandedState::Gripping;
        _rotationBlend = 0.0f;
        _gripLogCounter = 0;

        float gripDist = std::sqrt(dot(sub(_offhandGripLocal, _primaryGripLocal), sub(_offhandGripLocal, _primaryGripLocal)));
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: grip active weapon='{}', "
            "primaryLocal=({:.3f},{:.3f},{:.3f}), supportLocal=({:.3f},{:.3f},{:.3f}), "
            "gripSeparation={:.3f}, primaryGripSource={}, primaryGripConfidence={:.2f}, meshGrab={}, triangles={}, partKind={}, pose={}",
            weaponNode->name.c_str(), _primaryGripLocal.x, _primaryGripLocal.y, _primaryGripLocal.z, _offhandGripLocal.x, _offhandGripLocal.y, _offhandGripLocal.z, gripDist,
            "current-frik", _primaryGripConfidence, meshFound ? "YES" : "FALLBACK", triangles.size(), static_cast<int>(_supportPartKind),
            static_cast<int>(_supportGripPose));
    }

    void TwoHandedGrip::transitionToInactive(bool publishRestoredWeaponTransform)
    {
        clearPrimaryGripPose(false);
        clearSupportGripPose(true);
        restoreFrikOffhandGrip();
        bool restoredWeaponTransformAvailable = false;
        RE::NiTransform restoredWeaponTransform{};
        if (_hasWeaponNodeLocalBaseline && _activeWeaponNode) {
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
        _hasSupportFingerPose = false;
        _primaryGripConfidence = 0.0f;
        _activeWeaponNode = nullptr;
        _weaponNodeLocalBaseline = {};
        _hasWeaponNodeLocalBaseline = false;

        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: grip released");
    }

    void TwoHandedGrip::updateGripping(RE::NiNode* weaponNode, float dt)
    {
        constexpr bool supportHandIsLeft = true;
        constexpr bool primaryHandIsLeft = false;

        _rotationBlend = (std::min)(1.0f, _rotationBlend + dt * ROTATION_BLEND_SPEED);

        auto primaryTransform = getHandBoneTransform(primaryHandIsLeft);
        auto supportTransform = getHandBoneTransform(supportHandIsLeft);
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

        if (!applyLockedHandVisualAuthority(weaponNode)) {
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

    void TwoHandedGrip::setSupportGripPose(bool isLeft, WeaponGripPoseId poseId, const std::array<float, 5>* meshFingerPose)
    {
        (void)isLeft;
        if (meshFingerPose) {
            _supportFingerPose = grab_finger_pose_math::expandFingerCurlsToJointValues(*meshFingerPose);
            _hasSupportFingerPose = true;
            return;
        }

        const auto& poseValues = poseValuesForGrip(poseId);
        _supportFingerPose = poseValues;
        _hasSupportFingerPose = true;
    }

    void TwoHandedGrip::clearSupportGripPose(bool isLeft)
    {
        _supportFingerPose = {};
        _hasSupportFingerPose = false;

        if (frik::api::FRIKApi::inst) {
            frik::api::FRIKApi::inst->clearHandPose(SUPPORT_GRIP_TAG, handFromBool(isLeft));
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

    bool TwoHandedGrip::applyLockedHandVisualAuthority(RE::NiNode* weaponNode)
    {
        if (!weaponNode || !_hasHandWeaponLocalFrames) {
            return false;
        }

        auto* api = frik::api::FRIKApi::inst;
        if (!api || !api->applyExternalHandWorldTransform) {
            return false;
        }

        const RE::NiTransform primaryHandWorld =
            weapon_visual_authority_math::weaponLocalFrameToWorld(weaponNode->world, _primaryHandWeaponLocal);
        const RE::NiTransform supportHandWorld =
            weapon_visual_authority_math::weaponLocalFrameToWorld(weaponNode->world, _supportHandWeaponLocal);

        return api->applyExternalHandWorldTransform(PRIMARY_GRIP_TAG, frik::api::FRIKApi::Hand::Right, primaryHandWorld, GRIP_HAND_POSE_PRIORITY) &&
               api->applyExternalHandWorldTransform(SUPPORT_GRIP_TAG, frik::api::FRIKApi::Hand::Left, supportHandWorld, GRIP_HAND_POSE_PRIORITY);
    }

    void TwoHandedGrip::publishGripHandPoses(bool supportHandIsLeft)
    {
        auto* api = frik::api::FRIKApi::inst;
        if (!api || !api->setHandPoseCustomJointPositionsWithPriority) {
            return;
        }

        if (weapon_visual_authority_math::shouldPublishTwoHandedGripPose(weapon_visual_authority_math::LockedHandRole::Support) && _hasSupportFingerPose) {
            api->setHandPoseCustomJointPositionsWithPriority(SUPPORT_GRIP_TAG, handFromBool(supportHandIsLeft), _supportFingerPose.data(), GRIP_HAND_POSE_PRIORITY);
        }
    }

    void TwoHandedGrip::clearPrimaryGripPose(bool isLeft)
    {
        if (frik::api::FRIKApi::inst) {
            frik::api::FRIKApi::inst->clearHandPose(PRIMARY_GRIP_TAG, handFromBool(isLeft));
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
