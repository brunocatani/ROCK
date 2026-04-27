

#include "TwoHandedGrip.h"

#include "GrabFingerPoseRuntime.h"
#include "PalmTransform.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "TransformMath.h"
#include "WeaponCollisionGeometryMath.h"
#include "WeaponTwoHandedSolver.h"
#include "api/FRIKApi.h"
#include "f4vr/F4VRUtils.h"

#include <algorithm>
#include <array>

namespace frik::rock
{
    namespace
    {
        constexpr const char* SUPPORT_GRIP_TAG = "ROCK_WeaponSupportGrip";
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

    void TwoHandedGrip::snapHandToWorldPos(RE::NiAVObject* handBone, const RE::NiPoint3& targetWorld, const RE::NiNode* weaponNode)
    {
        auto* parent = handBone->parent;
        if (!parent)
            return;

        handBone->local.translate = transform_math::worldPointToLocal(parent->world, targetWorld);

        f4vr::updateTransformsDown(handBone, true, weaponNode ? weaponNode->name.c_str() : nullptr);
    }

    void TwoHandedGrip::update(
        RE::NiNode* weaponNode,
        const WeaponInteractionContact& leftWeaponContact,
        bool leftGripPressed,
        float dt,
        const WeaponReloadRuntimeState& reloadState)
    {
        _hasSolvedWeaponTransform = false;

        if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady() || !weaponNode) {
            if (_state != TwoHandedState::Inactive) {
                transitionToInactive();
            }
            return;
        }

        const WeaponInteractionDecision decision = routeWeaponInteraction(leftWeaponContact, reloadState);
        const bool leftTouchingSupport = decision.kind == WeaponInteractionKind::SupportGrip;

        switch (_state) {
        case TwoHandedState::Inactive:
            if (leftTouchingSupport) {
                transitionToTouching(weaponNode, decision);
            }
            break;

        case TwoHandedState::Touching:
            if (leftTouchingSupport) {
                _touchFrames = 0;
            } else {
                _touchFrames++;
                if (_touchFrames > TOUCH_TIMEOUT_FRAMES) {
                    _state = TwoHandedState::Inactive;
                    break;
                }
            }
            if (leftGripPressed && leftTouchingSupport) {
                transitionToGripping(weaponNode, decision);
            }
            break;

        case TwoHandedState::Gripping:
            if (!leftGripPressed) {
                transitionToInactive();
            } else {
                updateGripping(weaponNode, dt);
            }
            break;
        }
    }

    void TwoHandedGrip::reset()
    {
        if (_state != TwoHandedState::Inactive) {
            transitionToInactive();
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
    }

    bool TwoHandedGrip::getSolvedWeaponTransform(RE::NiTransform& outTransform) const
    {
        if (!_hasSolvedWeaponTransform) {
            return false;
        }
        outTransform = _lastSolvedWeaponTransform;
        return true;
    }

    void TwoHandedGrip::transitionToTouching(RE::NiNode* weaponNode, const WeaponInteractionDecision& decision)
    {
        _state = TwoHandedState::Touching;
        _touchFrames = 0;
        _supportGripPose = decision.gripPose != WeaponGripPoseId::None ? decision.gripPose : WeaponGripPoseId::BarrelWrap;
        _supportPartKind = decision.partKind;
        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: TOUCHING weapon='{}' bodyId={} partKind={} pose={}", weaponNode->name.c_str(), decision.bodyId,
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

        killFrikOffhandGrip();

        auto primaryTransform = getHandBoneTransform(primaryHandIsLeft);
        _primaryGripLocal = worldToWeaponLocal(computeGrabPivotAPositionFromHandBasis(primaryTransform, primaryHandIsLeft), weaponNode);

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
        _supportNormalLocal = transform_math::worldVectorToLocal(weaponNode->world, palmDir);

        std::array<float, 5> meshFingerPose{};
        const std::array<float, 5>* meshFingerPosePtr = nullptr;
        if (g_rockConfig.rockGrabMeshFingerPoseEnabled) {
            const RE::NiPoint3 gripWorldPoint = meshFound ? grabPoint.position : palmPos;
            const auto solvedFingerPose = grab_finger_pose_runtime::solveGrabFingerPoseFromTriangles(
                triangles, supportTransform, supportHandIsLeft, palmPos, gripWorldPoint, g_rockConfig.rockGrabFingerMinValue);
            if (solvedFingerPose.solved) {
                meshFingerPose = solvedFingerPose.values;
                meshFingerPosePtr = &meshFingerPose;
                ROCK_LOG_INFO(Weapon,
                    "TwoHandedGrip: mesh finger pose hand={} values=({:.2f},{:.2f},{:.2f},{:.2f},{:.2f}) hits={} candidateTris={}",
                    supportHandIsLeft ? "left" : "right", meshFingerPose[0], meshFingerPose[1], meshFingerPose[2], meshFingerPose[3], meshFingerPose[4],
                    solvedFingerPose.hitCount, solvedFingerPose.candidateTriangleCount);
            }
        }

        setSupportGripPose(supportHandIsLeft, _supportGripPose, meshFingerPosePtr);

        _state = TwoHandedState::Gripping;
        _rotationBlend = 0.0f;

        float gripDist = std::sqrt(dot(sub(_offhandGripLocal, _primaryGripLocal), sub(_offhandGripLocal, _primaryGripLocal)));
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: GRIP ACTIVATED — weapon='{}', "
            "primaryLocal=({:.3f},{:.3f},{:.3f}), supportLocal=({:.3f},{:.3f},{:.3f}), "
            "gripSeparation={:.3f}, meshGrab={}, triangles={}, partKind={}, pose={}",
            weaponNode->name.c_str(), _primaryGripLocal.x, _primaryGripLocal.y, _primaryGripLocal.z, _offhandGripLocal.x, _offhandGripLocal.y, _offhandGripLocal.z, gripDist,
            meshFound ? "YES" : "FALLBACK", triangles.size(), static_cast<int>(_supportPartKind), static_cast<int>(_supportGripPose));
    }

    void TwoHandedGrip::transitionToInactive()
    {
        clearSupportGripPose(true);
        restoreFrikOffhandGrip();
        if (_hasWeaponNodeLocalBaseline && _activeWeaponNode) {
            _activeWeaponNode->local = _weaponNodeLocalBaseline;
            f4vr::updateTransforms(_activeWeaponNode);
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

        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: GRIP RELEASED");
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
        const RE::NiPoint3 blendedSupportTarget = lerpPoint(currentSupportWorld, supportController, _rotationBlend);

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

        const auto solved = solveTwoHandedWeaponTransform(solverInput);
        if (!solved.solved) {
            return;
        }

        auto* parent = weaponNode->parent;
        const RE::NiTransform solvedLocal =
            parent ? transform_math::composeTransforms(transform_math::invertTransform(parent->world), solved.weaponWorldTransform) : solved.weaponWorldTransform;

        weaponNode->local.rotate = solvedLocal.rotate;
        weaponNode->local.translate = solvedLocal.translate;
        weaponNode->local.scale = solvedLocal.scale;

        f4vr::updateTransforms(weaponNode);
        _lastSolvedWeaponTransform = weaponNode->world;
        _hasSolvedWeaponTransform = true;

        RE::NiPoint3 primaryGripFinal = weaponLocalToWorld(_primaryGripLocal, weaponNode);
        RE::NiPoint3 offhandGripFinal = weaponLocalToWorld(_offhandGripLocal, weaponNode);

        auto* primaryHandNode = frik::api::FRIKApi::inst->getHandNode(frik::api::FRIKApi::Hand::Right);
        auto* offhandHandNode = frik::api::FRIKApi::inst->getHandNode(frik::api::FRIKApi::Hand::Left);

        if (primaryHandNode) {
            snapHandToWorldPos(primaryHandNode, primaryGripFinal, weaponNode);
        }
        if (offhandHandNode) {
            snapHandToWorldPos(offhandHandNode, offhandGripFinal, weaponNode);
        }

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
        auto* api = frik::api::FRIKApi::inst;
        if (!api) {
            return;
        }

        const auto hand = handFromBool(isLeft);
        if (meshFingerPose && api->setHandPoseCustomFingerPositionsWithPriority) {
            api->setHandPoseCustomFingerPositionsWithPriority(SUPPORT_GRIP_TAG, hand, (*meshFingerPose)[0], (*meshFingerPose)[1], (*meshFingerPose)[2], (*meshFingerPose)[3],
                (*meshFingerPose)[4], 100);
            return;
        }

        const auto& poseValues = poseValuesForGrip(poseId);
        if (api->setHandPoseCustomJointPositionsWithPriority) {
            api->setHandPoseCustomJointPositionsWithPriority(SUPPORT_GRIP_TAG, hand, poseValues.data(), 100);
            return;
        }

        api->setHandPoseCustomJointPositions(SUPPORT_GRIP_TAG, hand, poseValues.data());
    }

    void TwoHandedGrip::clearSupportGripPose(bool isLeft)
    {
        if (!frik::api::FRIKApi::inst) {
            return;
        }
        frik::api::FRIKApi::inst->clearHandPose(SUPPORT_GRIP_TAG, handFromBool(isLeft));
    }

    void TwoHandedGrip::killFrikOffhandGrip()
    {
        if (!frik::api::FRIKApi::inst)
            return;
        frik::api::FRIKApi::inst->blockOffHandWeaponGripping("ROCK_TwoHanded", true);
        ROCK_LOG_INFO(Weapon, "FRIK offhand grip SUPPRESSED");
    }

    void TwoHandedGrip::restoreFrikOffhandGrip()
    {
        if (!frik::api::FRIKApi::inst)
            return;
        frik::api::FRIKApi::inst->blockOffHandWeaponGripping("ROCK_TwoHanded", false);
        ROCK_LOG_INFO(Weapon, "FRIK offhand grip RESTORED");
    }

}
