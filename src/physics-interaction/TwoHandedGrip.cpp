

#include "TwoHandedGrip.h"

#include "GrabFingerPoseRuntime.h"
#include "PalmTransform.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "TransformMath.h"
#include "WeaponCollisionGeometryMath.h"
#include "api/FRIKApi.h"
#include "common/Quaternion.h"
#include "f4vr/F4VRUtils.h"

#include <array>

namespace frik::rock
{

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

    void TwoHandedGrip::update(RE::NiNode* weaponNode, bool offhandTouching, bool gripPressed, bool isLeftHanded, float dt)
    {
        if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady() || !weaponNode) {
            if (_state != TwoHandedState::Inactive) {
                transitionToInactive(isLeftHanded);
            }
            return;
        }

        switch (_state) {
        case TwoHandedState::Inactive:
            if (offhandTouching) {
                transitionToTouching(weaponNode);
            }
            break;

        case TwoHandedState::Touching:
            if (offhandTouching) {
                _touchFrames = 0;
            } else {
                _touchFrames++;
                if (_touchFrames > TOUCH_TIMEOUT_FRAMES) {
                    _state = TwoHandedState::Inactive;
                    break;
                }
            }
            if (gripPressed) {
                transitionToGripping(weaponNode, isLeftHanded);
            }
            break;

        case TwoHandedState::Gripping:
            if (!gripPressed) {
                transitionToInactive(isLeftHanded);
            } else {
                updateGripping(weaponNode, isLeftHanded, dt);
            }
            break;
        }
    }

    void TwoHandedGrip::reset()
    {
        if (_state == TwoHandedState::Gripping) {
            restoreFrikOffhandGrip();
            clearBarrelGripPose(true);
            clearBarrelGripPose(false);
        }
        _state = TwoHandedState::Inactive;
        _touchFrames = 0;
        _rotationBlend = 0.0f;
        _offhandGripLocal = {};
        _primaryGripLocal = {};
        _grabNormal = {};
    }

    void TwoHandedGrip::transitionToTouching(RE::NiNode* weaponNode)
    {
        _state = TwoHandedState::Touching;
        _touchFrames = 0;
        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: TOUCHING weapon='{}'", weaponNode->name.c_str());
    }

    void TwoHandedGrip::transitionToGripping(RE::NiNode* weaponNode, bool isLeftHanded)
    {
        bool offhandIsLeft = !isLeftHanded;

        killFrikOffhandGrip();

        auto primaryTransform = getHandBoneTransform(!offhandIsLeft);
        _primaryGripLocal = worldToWeaponLocal(computeGrabPivotAPositionFromHandBasis(primaryTransform, !offhandIsLeft), weaponNode);

        auto offhandTransform = getHandBoneTransform(offhandIsLeft);
        RE::NiPoint3 palmPos = computeGrabPivotAPositionFromHandBasis(offhandTransform, offhandIsLeft);
        RE::NiPoint3 palmDir = computePalmNormalFromHandBasis(offhandTransform, offhandIsLeft);

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

        std::array<float, 5> meshFingerPose{};
        const std::array<float, 5>* meshFingerPosePtr = nullptr;
        if (g_rockConfig.rockGrabMeshFingerPoseEnabled) {
            const RE::NiPoint3 gripWorldPoint = meshFound ? grabPoint.position : palmPos;
            const auto solvedFingerPose = grab_finger_pose_runtime::solveGrabFingerPoseFromTriangles(
                triangles, offhandTransform, offhandIsLeft, palmPos, gripWorldPoint, g_rockConfig.rockGrabFingerMinValue);
            if (solvedFingerPose.solved) {
                meshFingerPose = solvedFingerPose.values;
                meshFingerPosePtr = &meshFingerPose;
                ROCK_LOG_INFO(Weapon,
                    "TwoHandedGrip: mesh finger pose hand={} values=({:.2f},{:.2f},{:.2f},{:.2f},{:.2f}) hits={} candidateTris={}",
                    offhandIsLeft ? "left" : "right", meshFingerPose[0], meshFingerPose[1], meshFingerPose[2], meshFingerPose[3], meshFingerPose[4],
                    solvedFingerPose.hitCount, solvedFingerPose.candidateTriangleCount);
            }
        }

        setBarrelGripPose(offhandIsLeft, meshFingerPosePtr);

        _state = TwoHandedState::Gripping;
        _rotationBlend = 0.0f;

        float gripDist = std::sqrt(dot(sub(_offhandGripLocal, _primaryGripLocal), sub(_offhandGripLocal, _primaryGripLocal)));
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: GRIP ACTIVATED — weapon='{}', "
            "primaryLocal=({:.3f},{:.3f},{:.3f}), offhandLocal=({:.3f},{:.3f},{:.3f}), "
            "gripSeparation={:.3f}, meshGrab={}, triangles={}",
            weaponNode->name.c_str(), _primaryGripLocal.x, _primaryGripLocal.y, _primaryGripLocal.z, _offhandGripLocal.x, _offhandGripLocal.y, _offhandGripLocal.z, gripDist,
            meshFound ? "YES" : "FALLBACK", triangles.size());
    }

    void TwoHandedGrip::transitionToInactive(bool isLeftHanded)
    {
        bool offhandIsLeft = !isLeftHanded;
        clearBarrelGripPose(offhandIsLeft);
        restoreFrikOffhandGrip();

        _state = TwoHandedState::Inactive;
        _touchFrames = 0;
        _rotationBlend = 0.0f;
        _offhandGripLocal = {};
        _primaryGripLocal = {};
        _grabNormal = {};

        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: GRIP RELEASED");
    }

    void TwoHandedGrip::updateGripping(RE::NiNode* weaponNode, bool isLeftHanded, float dt)
    {
        bool offhandIsLeft = !isLeftHanded;

        _rotationBlend = (std::min)(1.0f, _rotationBlend + dt * ROTATION_BLEND_SPEED);

        auto primaryTransform = getHandBoneTransform(!offhandIsLeft);
        auto offhandTransform = getHandBoneTransform(offhandIsLeft);
        RE::NiPoint3 primaryController = computeGrabPivotAPositionFromHandBasis(primaryTransform, !offhandIsLeft);
        RE::NiPoint3 offhandController = computeGrabPivotAPositionFromHandBasis(offhandTransform, offhandIsLeft);

        RE::NiPoint3 axisLocal = sub(_offhandGripLocal, _primaryGripLocal);
        float axisLocalLen = std::sqrt(dot(axisLocal, axisLocal));
        if (axisLocalLen < 0.001f)
            return;
        axisLocal = normalize(axisLocal);

        RE::NiPoint3 axisDesired = sub(offhandController, primaryController);
        float axisDesiredLen = std::sqrt(dot(axisDesired, axisDesired));
        if (axisDesiredLen < 0.001f)
            return;
        axisDesired = normalize(axisDesired);

        RE::NiPoint3 axisCurrent = normalize(sub(weaponLocalToWorld(_offhandGripLocal, weaponNode), weaponLocalToWorld(_primaryGripLocal, weaponNode)));

        common::Quaternion rotDelta;
        rotDelta.vec2Vec(axisCurrent, axisDesired);

        if (_rotationBlend < 0.99f) {
            common::Quaternion identity(0, 0, 0, 1);
            identity.slerp(_rotationBlend, rotDelta);
            rotDelta = identity;
        }

        RE::NiMatrix3 worldRotDelta = rotDelta.getMatrix();

        RE::NiMatrix3 localRotDelta;
        auto* parent = weaponNode->parent;
        if (parent) {
            auto& pRot = parent->world.rotate;
            RE::NiMatrix3 pRotInv;
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    pRotInv.entry[r][c] = pRot.entry[c][r];
            localRotDelta = pRotInv * (worldRotDelta * pRot);
        } else {
            localRotDelta = worldRotDelta;
        }

        weaponNode->local.rotate = localRotDelta * weaponNode->local.rotate;

        RE::NiPoint3 primaryGripWorld = weaponLocalToWorld(_primaryGripLocal, weaponNode);

        RE::NiPoint3 posOffset = sub(primaryController, primaryGripWorld);

        if (parent) {
            const RE::NiPoint3 localOffset = transform_math::worldVectorToLocal(parent->world, posOffset);
            weaponNode->local.translate.x += localOffset.x;
            weaponNode->local.translate.y += localOffset.y;
            weaponNode->local.translate.z += localOffset.z;
        } else {
            weaponNode->local.translate.x += posOffset.x;
            weaponNode->local.translate.y += posOffset.y;
            weaponNode->local.translate.z += posOffset.z;
        }

        f4vr::updateTransforms(weaponNode);

        RE::NiPoint3 primaryGripFinal = weaponLocalToWorld(_primaryGripLocal, weaponNode);
        RE::NiPoint3 offhandGripFinal = weaponLocalToWorld(_offhandGripLocal, weaponNode);

        auto primaryHandEnum = offhandIsLeft ? frik::api::FRIKApi::Hand::Right : frik::api::FRIKApi::Hand::Left;
        auto offhandHandEnum = offhandIsLeft ? frik::api::FRIKApi::Hand::Left : frik::api::FRIKApi::Hand::Right;
        auto* primaryHandNode = frik::api::FRIKApi::inst->getHandNode(primaryHandEnum);
        auto* offhandHandNode = frik::api::FRIKApi::inst->getHandNode(offhandHandEnum);

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

    void TwoHandedGrip::setBarrelGripPose(bool isLeft, const std::array<float, 5>* meshFingerPose)
    {
        static constexpr float BARREL_GRIP_POSE[] = { 0.85f, 0.80f, 0.75f, 0.35f, 0.30f, 0.25f, 0.30f, 0.25f, 0.20f, 0.35f, 0.30f, 0.25f, 0.40f, 0.35f, 0.30f };

        auto* api = frik::api::FRIKApi::inst;
        if (!api) {
            return;
        }

        const auto hand = handFromBool(isLeft);
        if (meshFingerPose && api->setHandPoseCustomFingerPositionsWithPriority) {
            api->setHandPoseCustomFingerPositionsWithPriority("ROCK_BarrelGrip", hand, (*meshFingerPose)[0], (*meshFingerPose)[1], (*meshFingerPose)[2], (*meshFingerPose)[3],
                (*meshFingerPose)[4], 100);
            return;
        }

        if (api->setHandPoseCustomJointPositionsWithPriority) {
            api->setHandPoseCustomJointPositionsWithPriority("ROCK_BarrelGrip", hand, BARREL_GRIP_POSE, 100);
            return;
        }

        api->setHandPoseCustomJointPositions("ROCK_BarrelGrip", hand, BARREL_GRIP_POSE);
    }

    void TwoHandedGrip::clearBarrelGripPose(bool isLeft) { frik::api::FRIKApi::inst->clearHandPose("ROCK_BarrelGrip", handFromBool(isLeft)); }

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
