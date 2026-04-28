#include "Hand.h"

#include "HavokOffsets.h"

#include "BodyCollisionControl.h"
#include "DebugPivotMath.h"
#include "GrabConstraint.h"
#include "GrabConstraintMath.h"
#include "GrabFingerPoseRuntime.h"
#include "GrabMotionController.h"
#include "HandVisualLerpMath.h"
#include "HeldObjectDampingMath.h"
#include "HeldObjectPhysicsMath.h"
#include "MeshGrab.h"
#include "ObjectPhysicsBodySet.h"
#include "PalmTransform.h"
#include "PhysicsRecursiveWrappers.h"
#include "PhysicsUtils.h"
#include "PullMotionMath.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "TransformMath.h"
#include "api/FRIKApi.h"
#include "f4vr/F4VRUtils.h"

#include <cmath>
#include <format>
#include <array>
#include <xmmintrin.h>

namespace frik::rock
{
    namespace
    {
        RE::NiPoint3 getMatrixColumn(const RE::NiMatrix3& matrix, int column) { return RE::NiPoint3(matrix.entry[0][column], matrix.entry[1][column], matrix.entry[2][column]); }

        RE::NiPoint3 getMatrixRow(const RE::NiMatrix3& matrix, int row) { return RE::NiPoint3(matrix.entry[row][0], matrix.entry[row][1], matrix.entry[row][2]); }

        const char* nodeDebugName(const RE::NiAVObject* node)
        {
            if (!node) {
                return "(null)";
            }

            const char* name = node->name.c_str();
            return name ? name : "(unnamed)";
        }

        float translationDeltaGameUnits(const RE::NiTransform& a, const RE::NiTransform& b)
        {
            const RE::NiPoint3 delta = a.translate - b.translate;
            return std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
        }

        float rotationDeltaDegrees(const RE::NiMatrix3& a, const RE::NiMatrix3& b)
        {
            const RE::NiMatrix3 delta = a.Transpose() * b;
            float cosTheta = (delta.entry[0][0] + delta.entry[1][1] + delta.entry[2][2] - 1.0f) * 0.5f;
            if (cosTheta < -1.0f) {
                cosTheta = -1.0f;
            } else if (cosTheta > 1.0f) {
                cosTheta = 1.0f;
            }
            return std::acos(cosTheta) * (180.0f / 3.14159265358979323846f);
        }

        float axisDeltaDegrees(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            const float dot = a.x * b.x + a.y * b.y + a.z * b.z;
            const float lenA = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
            const float lenB = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
            if (lenA < 0.0001f || lenB < 0.0001f) {
                return -1.0f;
            }

            float cosTheta = dot / (lenA * lenB);
            if (cosTheta < -1.0f) {
                cosTheta = -1.0f;
            } else if (cosTheta > 1.0f) {
                cosTheta = 1.0f;
            }
            return std::acos(cosTheta) * (180.0f / 3.14159265358979323846f);
        }

        float max3(float a, float b, float c) { return (std::max)((std::max)(a, b), c); }

        RE::NiMatrix3 matrixFromHkColumns(const float* hkMatrix)
        {
            RE::NiMatrix3 result{};
            result.entry[0][0] = hkMatrix[0];
            result.entry[1][0] = hkMatrix[1];
            result.entry[2][0] = hkMatrix[2];
            result.entry[0][1] = hkMatrix[4];
            result.entry[1][1] = hkMatrix[5];
            result.entry[2][1] = hkMatrix[6];
            result.entry[0][2] = hkMatrix[8];
            result.entry[1][2] = hkMatrix[9];
            result.entry[2][2] = hkMatrix[10];
            return result;
        }

        RE::NiMatrix3 matrixFromHkRows(const float* hkMatrix)
        {
            RE::NiMatrix3 result{};
            result.entry[0][0] = hkMatrix[0];
            result.entry[0][1] = hkMatrix[1];
            result.entry[0][2] = hkMatrix[2];
            result.entry[1][0] = hkMatrix[4];
            result.entry[1][1] = hkMatrix[5];
            result.entry[1][2] = hkMatrix[6];
            result.entry[2][0] = hkMatrix[8];
            result.entry[2][1] = hkMatrix[9];
            result.entry[2][2] = hkMatrix[10];
            return result;
        }

        RE::NiTransform makeIdentityTransform()
        {
            return transform_math::makeIdentityTransform<RE::NiTransform>();
        }

        physics_recursive_wrappers::MotionPreset motionPresetFromSavedPropsId(std::uint16_t propsId)
        {
            switch (propsId & 0xFF) {
            case 0:
                return physics_recursive_wrappers::MotionPreset::Static;
            case 2:
                return physics_recursive_wrappers::MotionPreset::Keyframed;
            case 1:
            default:
                return physics_recursive_wrappers::MotionPreset::Dynamic;
            }
        }

        std::uint16_t motionPropsIdFromRecordMotionType(physics_body_classifier::BodyMotionType motionType, std::uint16_t fallback)
        {
            switch (motionType) {
            case physics_body_classifier::BodyMotionType::Static:
                return 0;
            case physics_body_classifier::BodyMotionType::Dynamic:
                return 1;
            case physics_body_classifier::BodyMotionType::Keyframed:
                return 2;
            default:
                return fallback;
            }
        }

        RE::NiTransform invertTransform(const RE::NiTransform& transform) { return transform_math::invertTransform(transform); }

        RE::NiTransform multiplyTransforms(const RE::NiTransform& parent, const RE::NiTransform& child) { return transform_math::composeTransforms(parent, child); }

        RE::NiTransform deriveNodeWorldFromBodyWorld(const RE::NiTransform& bodyWorld, const RE::NiTransform& bodyLocalTransform)
        {
            return multiplyTransforms(bodyWorld, invertTransform(bodyLocalTransform));
        }

        std::vector<GrabLocalTriangle> cacheTrianglesInLocalSpace(const std::vector<TriangleData>& worldTriangles, const RE::NiTransform& nodeWorld)
        {
            std::vector<GrabLocalTriangle> localTriangles;
            localTriangles.reserve(worldTriangles.size());
            for (const auto& triangle : worldTriangles) {
                localTriangles.push_back(GrabLocalTriangle{
                    transform_math::worldPointToLocal(nodeWorld, triangle.v0),
                    transform_math::worldPointToLocal(nodeWorld, triangle.v1),
                    transform_math::worldPointToLocal(nodeWorld, triangle.v2),
                });
            }
            return localTriangles;
        }

        std::vector<TriangleData> rebuildTrianglesInWorldSpace(const std::vector<GrabLocalTriangle>& localTriangles, const RE::NiTransform& nodeWorld)
        {
            std::vector<TriangleData> worldTriangles;
            worldTriangles.reserve(localTriangles.size());
            for (const auto& triangle : localTriangles) {
                worldTriangles.push_back(TriangleData{
                    transform_math::localPointToWorld(nodeWorld, triangle.v0),
                    transform_math::localPointToWorld(nodeWorld, triangle.v1),
                    transform_math::localPointToWorld(nodeWorld, triangle.v2),
                });
            }
            return worldTriangles;
        }

        RE::NiTransform getBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId)
        {
            RE::NiTransform result = makeIdentityTransform();
            if (!world || bodyId.value == INVALID_BODY_ID) {
                return result;
            }

            auto* bodyArray = world->GetBodyArray();
            auto* body = reinterpret_cast<float*>(&bodyArray[bodyId.value]);

            result.rotate = havokRotationBlocksToNiMatrix(body);
            result.translate.x = body[12] * kHavokToGameScale;
            result.translate.y = body[13] * kHavokToGameScale;
            result.translate.z = body[14] * kHavokToGameScale;
            return result;
        }

        RE::NiTransform computeRuntimeBodyLocalTransform(const RE::NiTransform& nodeWorld, const RE::NiTransform& bodyWorld)
        {
            return multiplyTransforms(invertTransform(nodeWorld), bodyWorld);
        }

        struct HeldMotionCompensationResult
        {
            RE::NiPoint3 primaryLocalLinearVelocity{};
            bool hasPrimaryVelocity = false;
        };

        HeldMotionCompensationResult applyHeldMotionCompensation(RE::hknpWorld* world,
            RE::hknpBodyId primaryBodyId,
            const std::vector<std::uint32_t>& heldBodyIds,
            const HeldObjectPlayerSpaceFrame& playerSpaceFrame,
            const RE::NiPoint3& previousPlayerSpaceVelocityHavok,
            float damping,
            bool residualDampingEnabled)
        {
            HeldMotionCompensationResult result{};
            if (!world) {
                return result;
            }

            const float keep = residualDampingEnabled ? held_object_damping_math::velocityKeepFactor(damping) : 1.0f;
            const bool applyPlayerSpaceVelocity = playerSpaceFrame.enabled && !playerSpaceFrame.warp;
            const RE::NiPoint3 currentPlayerVelocity = applyPlayerSpaceVelocity ? playerSpaceFrame.velocityHavok : RE::NiPoint3{};
            const RE::NiPoint3 previousPlayerVelocity = applyPlayerSpaceVelocity ? previousPlayerSpaceVelocityHavok : RE::NiPoint3{};

            auto* bodyArray = world->GetBodyArray();
            if (!bodyArray) {
                return result;
            }

            typedef void (*setVelDeferred_t)(void*, std::uint32_t, const float*, const float*);
            static REL::Relocation<setVelDeferred_t> setBodyVelocityDeferred{ REL::Offset(offsets::kFunc_SetBodyVelocityDeferred) };

            constexpr std::size_t kMaxDampedMotionSlots = 96;
            std::array<std::uint32_t, kMaxDampedMotionSlots> dampedMotionSlots{};
            std::size_t dampedMotionSlotCount = 0;

            auto motionSlotAlreadyDamped = [&dampedMotionSlots, &dampedMotionSlotCount](std::uint32_t motionIndex) {
                for (std::size_t i = 0; i < dampedMotionSlotCount; ++i) {
                    if (dampedMotionSlots[i] == motionIndex) {
                        return true;
                    }
                }
                return false;
            };

            auto compensateBody = [&](std::uint32_t bodyId) {
                if (bodyId == INVALID_BODY_ID) {
                    return;
                }

                const auto& body = bodyArray[bodyId];
                const std::uint32_t motionIndex = body.motionIndex;
                if (motionIndex == 0 || motionIndex > 4096 || motionSlotAlreadyDamped(motionIndex)) {
                    return;
                }

                if (dampedMotionSlotCount >= dampedMotionSlots.size()) {
                    return;
                }

                auto* motion = world->GetBodyMotion(RE::hknpBodyId{ bodyId });
                if (!motion) {
                    return;
                }

                dampedMotionSlots[dampedMotionSlotCount++] = motionIndex;

                const RE::NiPoint3 currentLinearVelocity{ motion->linearVelocity.x, motion->linearVelocity.y, motion->linearVelocity.z };
                const RE::NiPoint3 localLinearVelocity{
                    currentLinearVelocity.x - previousPlayerVelocity.x,
                    currentLinearVelocity.y - previousPlayerVelocity.y,
                    currentLinearVelocity.z - previousPlayerVelocity.z,
                };

                if (bodyId == primaryBodyId.value) {
                    result.primaryLocalLinearVelocity = localLinearVelocity;
                    result.hasPrimaryVelocity = true;
                }

                alignas(16) float linearVelocity[4] = {
                    currentPlayerVelocity.x + localLinearVelocity.x * keep,
                    currentPlayerVelocity.y + localLinearVelocity.y * keep,
                    currentPlayerVelocity.z + localLinearVelocity.z * keep,
                    0.0f,
                };
                alignas(16) float angularVelocity[4] = {
                    motion->angularVelocity.x * keep,
                    motion->angularVelocity.y * keep,
                    motion->angularVelocity.z * keep,
                    0.0f,
                };

                setBodyVelocityDeferred(world, bodyId, linearVelocity, angularVelocity);
            };

            compensateBody(primaryBodyId.value);
            for (const auto bodyId : heldBodyIds) {
                compensateBody(bodyId);
            }

            return result;
        }

        void setHeldLinearVelocity(RE::hknpWorld* world, RE::hknpBodyId primaryBodyId, const std::vector<std::uint32_t>& heldBodyIds, const RE::NiPoint3& linearVelocity)
        {
            if (!world) {
                return;
            }

            auto* bodyArray = world->GetBodyArray();
            if (!bodyArray) {
                return;
            }

            typedef void (*setVelDeferred_t)(void*, std::uint32_t, const float*, const float*);
            static REL::Relocation<setVelDeferred_t> setBodyVelocityDeferred{ REL::Offset(offsets::kFunc_SetBodyVelocityDeferred) };

            constexpr std::size_t kMaxVelocityMotionSlots = 96;
            std::array<std::uint32_t, kMaxVelocityMotionSlots> updatedMotionSlots{};
            std::size_t updatedMotionSlotCount = 0;

            auto alreadyUpdated = [&updatedMotionSlots, &updatedMotionSlotCount](std::uint32_t motionIndex) {
                for (std::size_t i = 0; i < updatedMotionSlotCount; ++i) {
                    if (updatedMotionSlots[i] == motionIndex) {
                        return true;
                    }
                }
                return false;
            };

            auto setBody = [&](std::uint32_t bodyId) {
                if (bodyId == INVALID_BODY_ID) {
                    return;
                }

                const auto& body = bodyArray[bodyId];
                const std::uint32_t motionIndex = body.motionIndex;
                if (motionIndex == 0 || motionIndex > 4096 || alreadyUpdated(motionIndex)) {
                    return;
                }

                if (updatedMotionSlotCount >= updatedMotionSlots.size()) {
                    return;
                }

                auto* motion = world->GetBodyMotion(RE::hknpBodyId{ bodyId });
                if (!motion) {
                    return;
                }

                updatedMotionSlots[updatedMotionSlotCount++] = motionIndex;
                alignas(16) float linear[4] = { linearVelocity.x, linearVelocity.y, linearVelocity.z, 0.0f };
                alignas(16) float angular[4] = { motion->angularVelocity.x, motion->angularVelocity.y, motion->angularVelocity.z, 0.0f };
                setBodyVelocityDeferred(world, bodyId, linear, angular);
            };

            setBody(primaryBodyId.value);
            for (const auto bodyId : heldBodyIds) {
                setBody(bodyId);
            }
        }

        float readBodyMass(RE::hknpWorld* world, RE::hknpBodyId bodyId)
        {
            if (!world || bodyId.value == INVALID_BODY_ID) {
                return 0.0f;
            }

            auto* motion = world->GetBodyMotion(bodyId);
            if (!motion) {
                return 0.0f;
            }

            auto packedInvMass = *reinterpret_cast<std::uint16_t*>(reinterpret_cast<char*>(motion) + 0x26);
            if (packedInvMass == 0) {
                return 0.0f;
            }

            std::uint32_t asUint = static_cast<std::uint32_t>(packedInvMass) << 16;
            float invMass = 0.0f;
            std::memcpy(&invMass, &asUint, sizeof(float));
            if (!std::isfinite(invMass) || invMass <= 0.0001f) {
                return 0.0f;
            }
            return 1.0f / invMass;
        }

        void applyRockGrabHandPose(bool isLeft,
            const grab_finger_pose_runtime::SolvedGrabFingerPose& fingerPose,
            std::array<float, 15>& currentJointPose,
            bool& hasCurrentJointPose,
            float deltaTime)
        {
            auto* api = frik::api::FRIKApi::inst;
            if (!api) {
                return;
            }

            const auto hand = handFromBool(isLeft);
            if (g_rockConfig.rockGrabMeshFingerPoseEnabled && g_rockConfig.rockGrabMeshJointPoseEnabled && fingerPose.solved && fingerPose.hasJointValues &&
                api->setHandPoseCustomJointPositionsWithPriority) {
                if (!hasCurrentJointPose) {
                    currentJointPose = fingerPose.jointValues;
                    hasCurrentJointPose = true;
                } else {
                    currentJointPose = grab_finger_pose_math::advanceJointValues(
                        currentJointPose, fingerPose.jointValues, g_rockConfig.rockGrabFingerPoseSmoothingSpeed, deltaTime);
                }

                api->setHandPoseCustomJointPositionsWithPriority("ROCK_Grab", hand, currentJointPose.data(), 100);
                if (g_rockConfig.rockDebugGrabFrameLogging) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand FINGER JOINT POSE: thumb=({:.2f},{:.2f},{:.2f}) index=({:.2f},{:.2f},{:.2f}) hits={} candidateTris={}",
                        isLeft ? "Left" : "Right", currentJointPose[0], currentJointPose[1], currentJointPose[2], currentJointPose[3], currentJointPose[4],
                        currentJointPose[5], fingerPose.hitCount, fingerPose.candidateTriangleCount);
                }
                return;
            }

            if (g_rockConfig.rockGrabMeshFingerPoseEnabled && fingerPose.solved && api->setHandPoseCustomFingerPositionsWithPriority) {
                hasCurrentJointPose = false;
                api->setHandPoseCustomFingerPositionsWithPriority("ROCK_Grab", hand, fingerPose.values[0], fingerPose.values[1], fingerPose.values[2], fingerPose.values[3],
                    fingerPose.values[4], 100);
                if (g_rockConfig.rockDebugGrabFrameLogging) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand FINGER POSE: mesh values=({:.2f},{:.2f},{:.2f},{:.2f},{:.2f}) hits={} candidateTris={}",
                        isLeft ? "Left" : "Right", fingerPose.values[0], fingerPose.values[1], fingerPose.values[2], fingerPose.values[3], fingerPose.values[4],
                        fingerPose.hitCount, fingerPose.candidateTriangleCount);
                }
                return;
            }

            hasCurrentJointPose = false;
            api->setHandPoseWithPriority("ROCK_Grab", hand, frik::api::FRIKApi::HandPoses::Fist, 100);
            if (g_rockConfig.rockGrabMeshFingerPoseEnabled && g_rockConfig.rockDebugGrabFrameLogging) {
                ROCK_LOG_DEBUG(Hand, "{} hand FINGER POSE: fallback fist solved={} hits={} candidateTris={}", isLeft ? "Left" : "Right", fingerPose.solved ? "yes" : "no",
                    fingerPose.hitCount, fingerPose.candidateTriangleCount);
            }
        }
    }

    static void nativeVRGrabDrop(void* playerChar, int handIndex)
    {
        typedef void func_t(void*, int);
        static REL::Relocation<func_t> func{ REL::Offset(offsets::kFunc_NativeVRGrabDrop) };
        func(playerChar, handIndex);
    }

    void Hand::clearGrabHandCollisionSuppressionState()
    {
        hand_collision_suppression_math::clear(_grabHandCollisionSuppression);
        _grabHandCollisionBroadPhaseSuppressed = false;
    }

    void Hand::suppressHandCollisionForGrab(RE::hknpWorld* world)
    {
        if (!world || !hasCollisionBody())
            return;

        const auto handBodyId = _handBody.getBodyId();
        if (handBodyId.value == INVALID_BODY_ID)
            return;

        std::uint32_t currentFilter = 0;
        if (!body_collision::tryReadFilterInfo(world, handBodyId, currentFilter)) {
            return;
        }

        const bool firstSuppression = !_grabHandCollisionSuppression.active || _grabHandCollisionSuppression.bodyId != handBodyId.value;
        const auto disabledFilter = hand_collision_suppression_math::beginSuppression(_grabHandCollisionSuppression, handBodyId.value, currentFilter);
        if (disabledFilter != currentFilter) {
            body_collision::setFilterInfo(world, handBodyId, disabledFilter);
        }

        const bool broadPhaseSet = body_collision::setBroadPhaseEnabled(world, handBodyId, false);
        _grabHandCollisionBroadPhaseSuppressed = _grabHandCollisionBroadPhaseSuppressed || broadPhaseSet;

        if (disabledFilter != currentFilter || firstSuppression) {
            ROCK_LOG_DEBUG(Hand, "{} hand: grab hand collision suppressed bodyId={} filter=0x{:08X}->0x{:08X} wasDisabledBeforeGrab={} broadphase={}", handName(),
                handBodyId.value, currentFilter, disabledFilter, _grabHandCollisionSuppression.wasNoCollideBeforeSuppression ? "yes" : "no",
                broadPhaseSet ? "disabled" : "unchanged");
        }
    }

    void Hand::restoreHandCollisionAfterGrab(RE::hknpWorld* world)
    {
        if (!_grabHandCollisionSuppression.active)
            return;

        if (!world || !hasCollisionBody()) {
            clearGrabHandCollisionSuppressionState();
            return;
        }

        const auto handBodyId = _handBody.getBodyId();
        if (handBodyId.value == INVALID_BODY_ID || handBodyId.value != _grabHandCollisionSuppression.bodyId) {
            clearGrabHandCollisionSuppressionState();
            return;
        }

        std::uint32_t currentFilter = 0;
        if (!body_collision::tryReadFilterInfo(world, handBodyId, currentFilter)) {
            clearGrabHandCollisionSuppressionState();
            return;
        }

        const auto restoredFilter = hand_collision_suppression_math::restoreFilter(_grabHandCollisionSuppression, currentFilter);

        if (restoredFilter != currentFilter) {
            body_collision::setFilterInfo(world, handBodyId, restoredFilter);
        }

        const bool broadPhaseRestored = _grabHandCollisionBroadPhaseSuppressed && body_collision::setBroadPhaseEnabled(world, handBodyId, true);

        ROCK_LOG_DEBUG(Hand, "{} hand: grab hand collision restored bodyId={} filter=0x{:08X}->0x{:08X} restoreDisabled={} broadphase={}", handName(), handBodyId.value,
            currentFilter, restoredFilter, _grabHandCollisionSuppression.wasNoCollideBeforeSuppression ? "yes" : "no", broadPhaseRestored ? "enabled" : "unchanged");
        clearGrabHandCollisionSuppressionState();
    }

    bool Hand::getGrabPivotDebugSnapshot(RE::hknpWorld* world, GrabPivotDebugSnapshot& out) const
    {
        out = {};

        if (!world || !isHolding() || !_activeConstraint.isValid() || !_activeConstraint.constraintData || !_handBody.isValid() ||
            _savedObjectState.bodyId.value == INVALID_BODY_ID) {
            return false;
        }

        const auto handBodyId = _handBody.getBodyId();
        if (handBodyId.value == INVALID_BODY_ID) {
            return false;
        }

        auto* bodyArray = world->GetBodyArray();
        if (!bodyArray) {
            return false;
        }

        auto* handBody = reinterpret_cast<const float*>(&bodyArray[handBodyId.value]);
        auto* objectBody = reinterpret_cast<const float*>(&bodyArray[_savedObjectState.bodyId.value]);
        auto* constraintData = static_cast<const char*>(_activeConstraint.constraintData);
        auto* pivotALocal = reinterpret_cast<const float*>(constraintData + offsets::kTransformA_Pos);
        auto* pivotBLocal = reinterpret_cast<const float*>(constraintData + offsets::kTransformB_Pos);

        out.handPivotWorld = debug_pivot_math::bodyLocalPointToWorldGamePoint<RE::NiPoint3>(handBody, pivotALocal, kHavokToGameScale);
        out.objectPivotWorld = debug_pivot_math::bodyLocalPointToWorldGamePoint<RE::NiPoint3>(objectBody, pivotBLocal, kHavokToGameScale);
        out.handBodyWorld = debug_pivot_math::bodyOriginToWorldGamePoint<RE::NiPoint3>(handBody, kHavokToGameScale);
        out.objectBodyWorld = debug_pivot_math::bodyOriginToWorldGamePoint<RE::NiPoint3>(objectBody, kHavokToGameScale);

        const RE::NiPoint3 error = out.handPivotWorld - out.objectPivotWorld;
        out.pivotErrorGameUnits = std::sqrt(error.x * error.x + error.y * error.y + error.z * error.z);
        return true;
    }

    bool Hand::startDynamicPull(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform)
    {
        /*
         * Long-range pull remains a dynamic-object operation. ROCK promotes the selected
         * object tree through FO4VR's recursive wrappers, scans the resulting dynamic body
         * set, and then applies short-lived predicted velocity to the accepted motions.
         * This mirrors HIGGS' pull behavior without introducing a keyframed object path
         * that would conflict with the dynamic grab constraint used after arrival.
         */
        if (!world || _state != HandState::SelectionLocked || !_currentSelection.isValid() || !_currentSelection.isFarSelection) {
            return false;
        }

        auto* selectedRef = _currentSelection.refr;
        if (!selectedRef || selectedRef->IsDeleted() || selectedRef->IsDisabled()) {
            clearSelectionState(true);
            return false;
        }

        auto* rootNode = selectedRef->Get3D();
        if (!rootNode) {
            ROCK_LOG_WARN(Hand, "{} hand PULL failed: selected ref has no 3D root", handName());
            clearSelectionState(true);
            return false;
        }

        auto* ownerCell = selectedRef->GetParentCell();
        auto* bhkWorld = ownerCell ? ownerCell->GetbhkWorld() : nullptr;
        if (!bhkWorld) {
            ROCK_LOG_WARN(Hand, "{} hand PULL failed: selected ref has no bhkWorld", handName());
            clearSelectionState(true);
            return false;
        }

        std::uint16_t selectedOriginalMotionPropsId = 1;
        if (_currentSelection.bodyId.value != INVALID_BODY_ID) {
            if (auto* selectedMotion = world->GetBodyMotion(_currentSelection.bodyId)) {
                selectedOriginalMotionPropsId = *reinterpret_cast<std::uint16_t*>(reinterpret_cast<char*>(selectedMotion) + offsets::kMotion_PropertiesId);
            }
        }

        object_physics_body_set::BodySetScanOptions scanOptions{};
        scanOptions.mode = physics_body_classifier::InteractionMode::ActiveGrab;
        scanOptions.rightHandBodyId = _isLeft ? INVALID_BODY_ID : _handBody.getBodyId().value;
        scanOptions.leftHandBodyId = _isLeft ? _handBody.getBodyId().value : INVALID_BODY_ID;
        scanOptions.sourceBodyId = _handBody.getBodyId().value;
        scanOptions.heldBySameHand = &_heldBodyIds;
        scanOptions.maxDepth = g_rockConfig.rockObjectPhysicsTreeMaxDepth;

        const auto beforePrepBodySet = object_physics_body_set::scanObjectPhysicsBodySet(bhkWorld, world, selectedRef, scanOptions);
        const bool motionConverted =
            physics_recursive_wrappers::setMotionRecursive(rootNode, physics_recursive_wrappers::MotionPreset::Dynamic, true, true, true);
        const bool collisionEnabled = physics_recursive_wrappers::enableCollisionRecursive(rootNode, true, true, true);
        auto restoreFailedPullPrep = [&]() {
            if (motionConverted && selectedOriginalMotionPropsId != 1) {
                physics_recursive_wrappers::setMotionRecursive(rootNode, motionPresetFromSavedPropsId(selectedOriginalMotionPropsId), false, true, true);
            }
        };
        const auto preparedBodySet = object_physics_body_set::scanObjectPhysicsBodySet(bhkWorld, world, selectedRef, scanOptions);
        const auto primaryChoice = preparedBodySet.choosePrimaryBody(_currentSelection.bodyId.value, object_physics_body_set::PurePoint3{ handWorldTransform.translate });

        if (primaryChoice.bodyId == INVALID_BODY_ID) {
            ROCK_LOG_WARN(Hand,
                "{} hand PULL failed: no accepted dynamic body after recursive prep formID={:08X} beforeBodies={} afterBodies={} accepted={} rejected={} "
                "setMotion={} enableCollision={}",
                handName(),
                selectedRef->GetFormID(),
                beforePrepBodySet.records.size(),
                preparedBodySet.records.size(),
                preparedBodySet.acceptedCount(),
                preparedBodySet.rejectedCount(),
                motionConverted ? "ok" : "failed",
                collisionEnabled ? "ok" : "failed");
            restoreFailedPullPrep();
            clearSelectionState(true);
            return false;
        }

        _pulledPrimaryBodyId = primaryChoice.bodyId;
        _pulledBodyIds = preparedBodySet.uniqueAcceptedMotionBodyIds();
        if (_pulledBodyIds.empty()) {
            _pulledBodyIds.push_back(_pulledPrimaryBodyId);
        }

        for (const auto bodyId : _pulledBodyIds) {
            physics_recursive_wrappers::activateBody(world, bodyId);
        }

        _currentSelection.bodyId = RE::hknpBodyId{ _pulledPrimaryBodyId };
        if (!_currentSelection.visualNode) {
            _currentSelection.visualNode = rootNode;
        }
        if (!_currentSelection.hitNode) {
            _currentSelection.hitNode = getOwnerNodeFromBody(world, RE::hknpBodyId{ _pulledPrimaryBodyId });
        }

        auto* motion = world->GetBodyMotion(RE::hknpBodyId{ _pulledPrimaryBodyId });
        if (!motion) {
            ROCK_LOG_WARN(Hand, "{} hand PULL failed: primary dynamic body has no motion bodyId={}", handName(), _pulledPrimaryBodyId);
            restoreFailedPullPrep();
            clearPullRuntimeState();
            clearSelectionState(true);
            return false;
        }

        const auto palmWorld = computePalmPositionFromHandBasis(handWorldTransform, _isLeft);
        const auto palmHavok = niPointToHkVector(palmWorld);
        const auto selectedPointWorld = _currentSelection.hasHitPoint ? _currentSelection.hitPointWorld : hkVectorToNiPoint(motion->position);
        const auto selectedPointHavok = niPointToHkVector(selectedPointWorld);
        _pullPointOffsetHavok = RE::NiPoint3{
            selectedPointHavok.x - motion->position.x,
            selectedPointHavok.y - motion->position.y,
            selectedPointHavok.z - motion->position.z,
        };

        const RE::NiPoint3 objectPointHavok{
            motion->position.x + _pullPointOffsetHavok.x,
            motion->position.y + _pullPointOffsetHavok.y,
            motion->position.z + _pullPointOffsetHavok.z,
        };
        const RE::NiPoint3 palmPointHavok{ palmHavok.x, palmHavok.y, palmHavok.z };
        const float pullDistance = (palmPointHavok - objectPointHavok).Length();

        _pullElapsedSeconds = 0.0f;
        _pullDurationSeconds = pull_motion_math::computePullDurationSeconds(pullDistance, g_rockConfig.rockPullDurationA, g_rockConfig.rockPullDurationB, g_rockConfig.rockPullDurationC);
        _pullTargetHavok = {};
        _pullHasTarget = false;
        _state = HandState::Pulled;
        stopSelectionHighlight();

        ROCK_LOG_INFO(Hand,
            "{} hand PULL start: formID={:08X} primaryBody={} bodyCount={} distanceHk={:.3f} duration={:.3f}s setMotion={} enableCollision={}",
            handName(),
            selectedRef->GetFormID(),
            _pulledPrimaryBodyId,
            _pulledBodyIds.size(),
            pullDistance,
            _pullDurationSeconds,
            motionConverted ? "ok" : "failed",
            collisionEnabled ? "ok" : "failed");
        return true;
    }

    bool Hand::updateDynamicPull(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform, float deltaTime)
    {
        if (_state != HandState::Pulled || !world || _pulledPrimaryBodyId == INVALID_BODY_ID || !_currentSelection.isValid()) {
            return false;
        }

        auto* selectedRef = _currentSelection.refr;
        if (!selectedRef || selectedRef->IsDeleted() || selectedRef->IsDisabled()) {
            clearSelectionState(true);
            return false;
        }

        auto* motion = world->GetBodyMotion(RE::hknpBodyId{ _pulledPrimaryBodyId });
        if (!motion) {
            clearSelectionState(true);
            return false;
        }

        const auto palmWorld = computePalmPositionFromHandBasis(handWorldTransform, _isLeft);
        const auto palmHavokVector = niPointToHkVector(palmWorld);
        const RE::NiPoint3 palmHavok{ palmHavokVector.x, palmHavokVector.y, palmHavokVector.z };
        const RE::NiPoint3 objectPointHavok{
            motion->position.x + _pullPointOffsetHavok.x,
            motion->position.y + _pullPointOffsetHavok.y,
            motion->position.z + _pullPointOffsetHavok.z,
        };

        const float distanceGameUnits = (palmHavok - objectPointHavok).Length() * kHavokToGameScale;
        _currentSelection.distance = distanceGameUnits;
        if (distanceGameUnits <= g_rockConfig.rockPullAutoGrabDistanceGameUnits) {
            _currentSelection.isFarSelection = false;
            _currentSelection.hitPointWorld = RE::NiPoint3{
                objectPointHavok.x * kHavokToGameScale,
                objectPointHavok.y * kHavokToGameScale,
                objectPointHavok.z * kHavokToGameScale,
            };
            _currentSelection.hasHitPoint = true;
            clearPullRuntimeState();
            _state = HandState::SelectedClose;
            ROCK_LOG_DEBUG(Hand, "{} hand PULL arrived -> close grab window dist={:.1f}", handName(), distanceGameUnits);
            return true;
        }

        const auto motionResult = pull_motion_math::computePullMotion<RE::NiPoint3>(
            pull_motion_math::PullMotionInput<RE::NiPoint3>{
                .handHavok = palmHavok,
                .objectPointHavok = objectPointHavok,
                .previousTargetHavok = _pullTargetHavok,
                .elapsedSeconds = _pullElapsedSeconds,
                .durationSeconds = _pullDurationSeconds,
                .applyVelocitySeconds = g_rockConfig.rockPullApplyVelocityTime,
                .trackHandSeconds = g_rockConfig.rockPullTrackHandTime,
                .destinationOffsetHavok = g_rockConfig.rockPullDestinationZOffsetHavok,
                .maxVelocityHavok = g_rockConfig.rockPullMaxVelocityHavok,
                .hasPreviousTarget = _pullHasTarget,
            });

        _pullElapsedSeconds += (std::max)(0.0f, deltaTime);
        if (motionResult.refreshTarget) {
            _pullTargetHavok = motionResult.targetHavok;
            _pullHasTarget = true;
        }

        if (motionResult.expired || !motionResult.applyVelocity) {
            ROCK_LOG_DEBUG(Hand, "{} hand PULL expired before arrival dist={:.1f}", handName(), distanceGameUnits);
            clearSelectionState(true);
            return false;
        }

        setHeldLinearVelocity(world, RE::hknpBodyId{ _pulledPrimaryBodyId }, _pulledBodyIds, motionResult.velocityHavok);
        for (const auto bodyId : _pulledBodyIds) {
            physics_recursive_wrappers::activateBody(world, bodyId);
        }

        return false;
    }

    bool Hand::grabSelectedObject(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform, float tau, float damping, float maxForce, float proportionalRecovery,
        float constantRecovery)
    {
        if (!hasSelection() || !world)
            return false;
        if (!hasCollisionBody())
            return false;

        const auto& sel = _currentSelection;
        if (!sel.refr || sel.bodyId.value == 0x7FFF'FFFF)
            return false;
        if (sel.refr->IsDeleted() || sel.refr->IsDisabled())
            return false;

        auto objectBodyId = sel.bodyId;
        auto* rootNode = sel.refr->Get3D();
        if (!rootNode) {
            ROCK_LOG_WARN(Hand, "{} hand GRAB failed: selected ref has no 3D root", handName());
            return false;
        }

        auto* ownerCell = sel.refr->GetParentCell();
        auto* bhkWorld = ownerCell ? ownerCell->GetbhkWorld() : nullptr;
        if (!bhkWorld) {
            ROCK_LOG_WARN(Hand, "{} hand GRAB failed: selected ref has no bhkWorld for object-tree scan", handName());
            return false;
        }

        auto& body = world->GetBody(objectBodyId);

        auto* baseObj = sel.refr->GetObjectReference();
        const char* objName = "(unnamed)";
        if (baseObj) {
            auto nameView = RE::TESFullName::GetFullName(*baseObj, false);
            if (!nameView.empty())
                objName = nameView.data();
        }

        const char* motionTypeStr = "UNKNOWN";
        std::uint16_t selectedOriginalMotionPropsId = 1;
        {
            auto* objMotion = world->GetBodyMotion(objectBodyId);
            auto* bodyPtr = reinterpret_cast<char*>(&body);
            if (objMotion) {
                auto* motionPtr = reinterpret_cast<char*>(objMotion);

                std::uint32_t bodyFlags = *reinterpret_cast<std::uint32_t*>(bodyPtr + 0x40);
                std::uint8_t bodyMotionPropsId = *reinterpret_cast<std::uint8_t*>(bodyPtr + 0x72);
                std::uint16_t motionPropsId = *reinterpret_cast<std::uint16_t*>(motionPtr + offsets::kMotion_PropertiesId);
                selectedOriginalMotionPropsId = motionPropsId;
                std::uint16_t maxLinVelPacked = *reinterpret_cast<std::uint16_t*>(motionPtr + 0x3A);
                std::uint16_t maxAngVelPacked = *reinterpret_cast<std::uint16_t*>(motionPtr + 0x3C);

                std::uint32_t linVelU32 = static_cast<std::uint32_t>(maxLinVelPacked) << 16;
                std::uint32_t angVelU32 = static_cast<std::uint32_t>(maxAngVelPacked) << 16;
                float maxLinVel, maxAngVel;
                std::memcpy(&maxLinVel, &linVelU32, sizeof(float));
                std::memcpy(&maxAngVel, &angVelU32, sizeof(float));

                bool isDynamicInit = (bodyFlags & 0x2) != 0;
                bool isKeyframed = (bodyFlags & 0x4) != 0;

                switch (motionPropsId & 0xFF) {
                case 0:
                    motionTypeStr = "STATIC";
                    break;
                case 1:
                    motionTypeStr = "DYNAMIC";
                    break;
                case 2:
                    motionTypeStr = "KEYFRAMED";
                    break;
                default:
                    motionTypeStr = "OTHER";
                    break;
                }

                ROCK_LOG_DEBUG(Hand,
                    "{} hand GRAB MOTION: body={} motionPropsId={} ({}) "
                    "bodyFlags=0x{:08X} dynInit={} keyfr={} bodyPropsId={} "
                    "maxLinVel={:.1f} maxAngVel={:.1f}",
                    handName(), objectBodyId.value, motionPropsId, motionTypeStr, bodyFlags, isDynamicInit, isKeyframed, bodyMotionPropsId, maxLinVel, maxAngVel);

                {
                    static bool libraryDumped = false;
                    if (!libraryDumped) {
                        libraryDumped = true;
                        auto* worldPtr = reinterpret_cast<char*>(world);
                        auto* libraryPtr = *reinterpret_cast<char**>(worldPtr + 0x5D0);
                        if (libraryPtr) {
                            auto* arrayBase = *reinterpret_cast<char**>(libraryPtr + 0x28);

                            int arraySize = *reinterpret_cast<int*>(libraryPtr + 0x30);
                            ROCK_LOG_TRACE(Hand, "Motion properties library: {} entries, stride=0x40", arraySize);
                            int dumpCount = (arraySize < 16) ? arraySize : 16;
                            if (arrayBase) {
                                for (int i = 0; i < dumpCount; i++) {
                                    auto* entry = arrayBase + i * 0x40;

                                    auto* f = reinterpret_cast<float*>(entry);
                                    auto* u = reinterpret_cast<std::uint32_t*>(entry);
                                    ROCK_LOG_TRACE(Hand,
                                        "  [{}] +00: {:.4f} {:.4f} {:.4f} {:.4f} | "
                                        "+10: {:.4f} {:.4f} {:.4f} {:.4f} | "
                                        "+20: {:.4f} {:.4f} {:.4f} {:.4f} | "
                                        "+30: {:.4f} {:.4f} {:.4f} {:.4f}",
                                        i, f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8], f[9], f[10], f[11], f[12], f[13], f[14], f[15]);
                                    ROCK_LOG_TRACE(Hand,
                                        "  [{}] hex: {:08X} {:08X} {:08X} {:08X} | "
                                        "{:08X} {:08X} {:08X} {:08X}",
                                        i, u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7]);
                                }
                            }
                        }
                    }
                }
            }
        }

        object_physics_body_set::BodySetScanOptions scanOptions{};
        scanOptions.mode = physics_body_classifier::InteractionMode::ActiveGrab;
        scanOptions.rightHandBodyId = _isLeft ? INVALID_BODY_ID : _handBody.getBodyId().value;
        scanOptions.leftHandBodyId = _isLeft ? _handBody.getBodyId().value : INVALID_BODY_ID;
        scanOptions.sourceBodyId = _handBody.getBodyId().value;
        scanOptions.heldBySameHand = &_heldBodyIds;
        scanOptions.maxDepth = g_rockConfig.rockObjectPhysicsTreeMaxDepth;

        const auto beforePrepBodySet = object_physics_body_set::scanObjectPhysicsBodySet(bhkWorld, world, sel.refr, scanOptions);

        const bool motionConverted =
            physics_recursive_wrappers::setMotionRecursive(rootNode, physics_recursive_wrappers::MotionPreset::Dynamic, true, true, true);
        const bool collisionEnabled = physics_recursive_wrappers::enableCollisionRecursive(rootNode, true, true, true);

        auto preparedBodySet = object_physics_body_set::scanObjectPhysicsBodySet(bhkWorld, world, sel.refr, scanOptions);
        const auto primaryChoice = preparedBodySet.choosePrimaryBody(sel.bodyId.value, object_physics_body_set::PurePoint3{ handWorldTransform.translate });

        ROCK_LOG_DEBUG(Hand,
            "{} hand object-tree prep: ref='{}' formID={:08X} beforeBodies={} afterBodies={} accepted={} rejected={} "
            "setMotion={} enableCollision={} primaryBody={} choiceReason={}",
            handName(),
            objName,
            sel.refr->GetFormID(),
            beforePrepBodySet.records.size(),
            preparedBodySet.records.size(),
            preparedBodySet.acceptedCount(),
            preparedBodySet.rejectedCount(),
            motionConverted ? "ok" : "failed",
            collisionEnabled ? "ok" : "failed",
            primaryChoice.bodyId,
            static_cast<int>(primaryChoice.reason));

        if (primaryChoice.bodyId == INVALID_BODY_ID) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: no accepted dynamic body after recursive prep for '{}' formID={:08X} visitedNodes={} collisionObjects={}",
                handName(),
                objName,
                sel.refr->GetFormID(),
                preparedBodySet.diagnostics.visitedNodes,
                preparedBodySet.diagnostics.collisionObjects);
            if (motionConverted && selectedOriginalMotionPropsId != 1) {
                physics_recursive_wrappers::setMotionRecursive(rootNode, motionPresetFromSavedPropsId(selectedOriginalMotionPropsId), false, true, true);
            }
            return false;
        }

        objectBodyId = RE::hknpBodyId{ primaryChoice.bodyId };
        auto& preparedBody = world->GetBody(objectBodyId);
        _savedObjectState.bodyId = objectBodyId;
        _savedObjectState.refr = sel.refr;
        _savedObjectState.originalFilterInfo = *reinterpret_cast<std::uint32_t*>(reinterpret_cast<char*>(&preparedBody) + offsets::kBody_CollisionFilterInfo);
        _savedObjectState.originalMotionPropsId = selectedOriginalMotionPropsId;
        if (const auto* originalPrimaryRecord = beforePrepBodySet.findRecord(objectBodyId.value)) {
            _savedObjectState.originalMotionPropsId = motionPropsIdFromRecordMotionType(originalPrimaryRecord->motionType, selectedOriginalMotionPropsId);
        }

        ROCK_LOG_INFO(Hand, "{} hand GRAB: '{}' formID={:08X} bodyId={}", handName(), objName, sel.refr->GetFormID(), objectBodyId.value);

        if (g_rockConfig.rockDebugShowGrabNotifications) {
            auto msg = std::format("[ROCK] {} GRAB: {} ({})", _isLeft ? "L" : "R", objName, motionTypeStr);
            f4vr::showNotification(msg);
        }

        _heldBodyIds = preparedBodySet.acceptedBodyIds();
        if (_heldBodyIds.empty()) {
            _heldBodyIds.push_back(objectBodyId.value);
        }

        {
            auto* player = RE::PlayerCharacter::GetSingleton();
            if (player) {
                nativeVRGrabDrop(player, 0);
                nativeVRGrabDrop(player, 1);
            }
        }

        auto* collidableNode = sel.hitNode ? sel.hitNode : rootNode;
        auto* meshSourceNode = sel.visualNode ? sel.visualNode : rootNode;
        if (!meshSourceNode) {
            meshSourceNode = collidableNode;
        }
        RE::NiTransform objectWorldTransform;
        if (collidableNode) {
            objectWorldTransform = collidableNode->world;
        } else {
            objectWorldTransform = handWorldTransform;
        }

        static bool s_runtimeScaleLogged = false;
        if (!s_runtimeScaleLogged) {
            auto* vrScaleSetting = f4vr::getIniSetting("fVrScale:VR");
            const float vrScale = vrScaleSetting ? vrScaleSetting->GetFloat() : -1.0f;
            ROCK_LOG_DEBUG(Hand, "Runtime scale {}: handScale={:.3f} collidableScale={:.3f} vrScale={:.3f}", handName(), handWorldTransform.scale,
                collidableNode ? collidableNode->world.scale : -1.0f, vrScale);
            s_runtimeScaleLogged = true;
        }

        RE::NiPoint3 grabSurfacePoint = objectWorldTransform.translate;
        bool meshGrabFound = false;
        MeshExtractionStats meshStats;
        std::vector<TriangleData> grabMeshTriangles;
        const char* grabPointMode = "objectNodeOriginFallback";
        const char* grabFallbackReason = meshSourceNode ? "noTriangles" : "noMeshSourceNode";

        if (meshSourceNode) {
            extractAllTriangles(meshSourceNode, grabMeshTriangles, 10, &meshStats);

            ROCK_LOG_DEBUG(Hand,
                "{} hand mesh extraction: meshNode='{}' ownerNode='{}' rootNode='{}' shapes={} "
                "static={}/{} dynamic={}/{} skinned={}/{} dynamicSkinnedSkipped={} emptyShapes={} totalTris={}",
                handName(), nodeDebugName(meshSourceNode), nodeDebugName(collidableNode), nodeDebugName(rootNode), meshStats.visitedShapes, meshStats.staticShapes,
                meshStats.staticTriangles, meshStats.dynamicShapes, meshStats.dynamicTriangles, meshStats.skinnedShapes, meshStats.skinnedTriangles,
                meshStats.dynamicSkinnedSkipped, meshStats.emptyShapes, meshStats.totalTriangles());

            {
                RE::BSTriShape* firstTriShape = meshSourceNode->IsTriShape();
                if (!firstTriShape) {
                    auto* meshNode = meshSourceNode->IsNode();
                    if (meshNode) {
                        auto& kids = meshNode->GetRuntimeData().children;
                        const auto childCount = kids.size();
                        for (auto ci = decltype(childCount){ 0 }; ci < childCount; ci++) {
                            auto* kid = kids[ci].get();
                            if (kid && kid->IsTriShape()) {
                                firstTriShape = kid->IsTriShape();
                                break;
                            }
                        }
                    }
                }
                if (firstTriShape) {
                    auto* tsBase = reinterpret_cast<char*>(firstTriShape);
                    std::uint64_t vtxDesc = *reinterpret_cast<std::uint64_t*>(tsBase + VROffset::vertexDesc);
                    std::uint32_t stride = static_cast<std::uint32_t>(vtxDesc & 0xF) * 4;
                    std::uint32_t posOff = static_cast<std::uint32_t>((vtxDesc >> 2) & 0x3C);
                    bool fullPrec = ((vtxDesc >> 54) & 1) != 0;
                    std::uint8_t geomType = *reinterpret_cast<std::uint8_t*>(tsBase + 0x198);
                    void* skinInst = *reinterpret_cast<void**>(tsBase + VROffset::skinInstance);

                    ROCK_LOG_TRACE(MeshGrab, "VertexDiag '{}': vtxDesc=0x{:016X} stride={} posOffset={} fullPrec={} geomType={} skinned={}",
                        firstTriShape->name.c_str() ? firstTriShape->name.c_str() : "(null)", vtxDesc, stride, posOff, fullPrec ? 1 : 0, geomType, skinInst ? 1 : 0);
                }
            }

            if (!grabMeshTriangles.empty()) {
                auto& t0 = grabMeshTriangles[0];
                float cx = (t0.v0.x + t0.v1.x + t0.v2.x) / 3.0f;
                float cy = (t0.v0.y + t0.v1.y + t0.v2.y) / 3.0f;
                float cz = (t0.v0.z + t0.v1.z + t0.v2.z) / 3.0f;

                auto* bodyArray = world->GetBodyArray();
                auto* objFloats = reinterpret_cast<float*>(&bodyArray[objectBodyId.value]);
                float distToBody = std::sqrt((cx - objFloats[12] * 70.0f) * (cx - objFloats[12] * 70.0f) + (cy - objFloats[13] * 70.0f) * (cy - objFloats[13] * 70.0f) +
                    (cz - objFloats[14] * 70.0f) * (cz - objFloats[14] * 70.0f));

                ROCK_LOG_TRACE(MeshGrab, "TRI[0] centroid=({:.1f},{:.1f},{:.1f}) distToBody={:.1f}gu", cx, cy, cz, distToBody);
            }

            if (!grabMeshTriangles.empty()) {
                RE::NiPoint3 grabPivotAWorld = computeGrabPivotAPositionFromHandBasis(handWorldTransform, _isLeft);
                RE::NiPoint3 palmDir = computePalmNormalFromHandBasis(handWorldTransform, _isLeft);

                GrabPoint grabPt;
                if (findClosestGrabPoint(grabMeshTriangles, grabPivotAWorld, palmDir, 1.0f, 1.0f, grabPt)) {
                    grabSurfacePoint = grabPt.position;
                    meshGrabFound = true;
                    grabPointMode = "meshSurface";
                    grabFallbackReason = "none";
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand MESH GRAB: mode={} tris={} staticTris={} dynamicTris={} skinnedTris={} "
                        "closest=({:.1f},{:.1f},{:.1f})",
                        handName(), grabPointMode, grabMeshTriangles.size(), meshStats.staticTriangles, meshStats.dynamicTriangles, meshStats.skinnedTriangles, grabPt.position.x,
                        grabPt.position.y, grabPt.position.z);
                } else {
                    grabFallbackReason = "noClosestSurfacePoint";
                }
            } else {
                grabFallbackReason = "noTriangles";
            }
        }

        if (!meshGrabFound) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB POINT FALLBACK: mode={} reason={} meshNode='{}' ownerNode='{}' rootNode='{}' "
                "shapes={} static={}/{} dynamic={}/{} skinned={}/{} dynamicSkinnedSkipped={} emptyShapes={} "
                "fallbackPoint=({:.1f},{:.1f},{:.1f})",
                handName(), grabPointMode, grabFallbackReason, nodeDebugName(meshSourceNode), nodeDebugName(collidableNode), nodeDebugName(rootNode), meshStats.visitedShapes,
                meshStats.staticShapes, meshStats.staticTriangles, meshStats.dynamicShapes, meshStats.dynamicTriangles, meshStats.skinnedShapes, meshStats.skinnedTriangles,
                meshStats.dynamicSkinnedSkipped, meshStats.emptyShapes, grabSurfacePoint.x, grabSurfacePoint.y, grabSurfacePoint.z);
        }

        _grabStartTime = 0.0f;

        {
            const RE::NiPoint3 palmPos = computePalmPositionFromHandBasis(handWorldTransform, _isLeft);
            const RE::NiPoint3 grabPivotAWorld = computeGrabPivotAPositionFromHandBasis(handWorldTransform, _isLeft);
            RE::NiTransform shiftedObject = objectWorldTransform;
            shiftedObject.translate.x += grabPivotAWorld.x - grabSurfacePoint.x;
            shiftedObject.translate.y += grabPivotAWorld.y - grabSurfacePoint.y;
            shiftedObject.translate.z += grabPivotAWorld.z - grabSurfacePoint.z;

            _grabHandSpace = multiplyTransforms(invertTransform(handWorldTransform), shiftedObject);

            const RE::NiTransform liveBodyWorldAtGrab = getBodyWorldTransform(world, objectBodyId);
            auto* ownerCellAtGrab = sel.refr ? sel.refr->GetParentCell() : nullptr;
            auto* bhkWorldAtGrab = ownerCellAtGrab ? ownerCellAtGrab->GetbhkWorld() : nullptr;
            auto* bodyCollisionObjectAtGrab = bhkWorldAtGrab ? RE::bhkNPCollisionObject::Getbhk(bhkWorldAtGrab, objectBodyId) : nullptr;
            auto* ownerNodeAtGrab = bodyCollisionObjectAtGrab ? bodyCollisionObjectAtGrab->sceneObject : nullptr;
            _heldNode = collidableNode;
            _grabBodyLocalTransform = collidableNode ? computeRuntimeBodyLocalTransform(objectWorldTransform, liveBodyWorldAtGrab) : makeIdentityTransform();
            _grabOwnerBodyLocalTransform = ownerNodeAtGrab ? computeRuntimeBodyLocalTransform(ownerNodeAtGrab->world, liveBodyWorldAtGrab) : makeIdentityTransform();
            _grabRootBodyLocalTransform = rootNode ? computeRuntimeBodyLocalTransform(rootNode->world, liveBodyWorldAtGrab) : makeIdentityTransform();

            _grabConstraintHandSpace = _grabHandSpace;
            _adjustedHandTransform = handWorldTransform;
            _hasAdjustedHandTransform = true;
            _grabVisualLerpElapsed = 0.0f;
            const RE::NiPoint3 initialGrabDelta = grabPivotAWorld - grabSurfacePoint;
            const float initialGrabDistance =
                std::sqrt(initialGrabDelta.x * initialGrabDelta.x + initialGrabDelta.y * initialGrabDelta.y + initialGrabDelta.z * initialGrabDelta.z);
            _grabVisualLerpDuration = held_object_physics_math::computeHandLerpDuration(initialGrabDistance, g_rockConfig.rockGrabHandLerpTimeMin,
                g_rockConfig.rockGrabHandLerpTimeMax, g_rockConfig.rockGrabHandLerpMinDistance, g_rockConfig.rockGrabHandLerpMaxDistance);
            _grabLocalMeshTriangles.clear();
            _grabSurfacePointLocal = transform_math::worldPointToLocal(objectWorldTransform, grabSurfacePoint);
            _hasGrabMeshPoseData = false;
            _grabFingerPoseFrameCounter = 0;
            if (!grabMeshTriangles.empty()) {
                _grabLocalMeshTriangles = cacheTrianglesInLocalSpace(grabMeshTriangles, objectWorldTransform);
                _hasGrabMeshPoseData = !_grabLocalMeshTriangles.empty();
            }
            _heldLocalLinearVelocityHistory = {};
            _heldLocalLinearVelocityHistoryCount = 0;
            _heldLocalLinearVelocityHistoryNext = 0;
            _lastPlayerSpaceVelocityHavok = {};

            if (g_rockConfig.rockDebugGrabFrameLogging) {
                const RE::NiTransform& constraintGrabHandSpace = _grabConstraintHandSpace;

                const RE::NiPoint3 rawLateral = getMatrixColumn(handWorldTransform.rotate, 0);
                const RE::NiPoint3 rawFinger = getMatrixColumn(handWorldTransform.rotate, 2);
                const RE::NiPoint3 rawBack = getMatrixColumn(handWorldTransform.rotate, 1);
                const RE::NiPoint3 constraintFinger = getMatrixColumn(handWorldTransform.rotate, 2);
                const RE::NiPoint3 constraintBack = getMatrixColumn(handWorldTransform.rotate, 1);
                const RE::NiPoint3 constraintLateral = getMatrixColumn(handWorldTransform.rotate, 0);
                const RE::NiPoint3 grabSpaceRawFinger = getMatrixColumn(_grabHandSpace.rotate, 0);
                const RE::NiPoint3 grabSpaceConstraintFinger = getMatrixColumn(constraintGrabHandSpace.rotate, 0);
                const RE::NiPoint3 grabPosDelta = constraintGrabHandSpace.translate - _grabHandSpace.translate;

                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB FRAME SNAPSHOT: rawVsConstraint rotDelta={:.2f}deg posDelta=({:.2f},{:.2f},{:.2f}) "
                    "rawFinger=({:.3f},{:.3f},{:.3f}) rawBack=({:.3f},{:.3f},{:.3f}) rawLat=({:.3f},{:.3f},{:.3f}) "
                    "constraintFinger=({:.3f},{:.3f},{:.3f}) constraintBack=({:.3f},{:.3f},{:.3f}) constraintLat=({:.3f},{:.3f},{:.3f})",
                    handName(), rotationDeltaDegrees(_grabHandSpace.rotate, constraintGrabHandSpace.rotate), grabPosDelta.x, grabPosDelta.y, grabPosDelta.z, rawFinger.x,
                    rawFinger.y, rawFinger.z, rawBack.x, rawBack.y, rawBack.z, rawLateral.x, rawLateral.y, rawLateral.z, constraintFinger.x, constraintFinger.y, constraintFinger.z,
                    constraintBack.x, constraintBack.y, constraintBack.z, constraintLateral.x, constraintLateral.y, constraintLateral.z);

                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB FRAME TARGETS: grabHSRaw.pos=({:.2f},{:.2f},{:.2f}) grabHSConstraint.pos=({:.2f},{:.2f},{:.2f}) "
                    "grabHSRawFinger=({:.3f},{:.3f},{:.3f}) grabHSConstraintFinger=({:.3f},{:.3f},{:.3f}) "
                    "bodyLocal.pos=({:.2f},{:.2f},{:.2f}) bodyLocalFinger=({:.3f},{:.3f},{:.3f})",
                    handName(), _grabHandSpace.translate.x, _grabHandSpace.translate.y, _grabHandSpace.translate.z, constraintGrabHandSpace.translate.x,
                    constraintGrabHandSpace.translate.y, constraintGrabHandSpace.translate.z, grabSpaceRawFinger.x, grabSpaceRawFinger.y, grabSpaceRawFinger.z,
                    grabSpaceConstraintFinger.x, grabSpaceConstraintFinger.y, grabSpaceConstraintFinger.z, _grabBodyLocalTransform.translate.x, _grabBodyLocalTransform.translate.y,
                    _grabBodyLocalTransform.translate.z, _grabBodyLocalTransform.rotate.entry[0][0], _grabBodyLocalTransform.rotate.entry[1][0],
                    _grabBodyLocalTransform.rotate.entry[2][0]);

                const RE::NiPoint3 rootBodyLocalFinger = getMatrixColumn(_grabRootBodyLocalTransform.rotate, 0);
                const RE::NiPoint3 ownerBodyLocalFinger = getMatrixColumn(_grabOwnerBodyLocalTransform.rotate, 0);
                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB NODE FRAMES: owner='{}'({:p}) hasCol={} ownsBodyCol={} "
                    "root='{}'({:p}) held='{}'({:p}) mesh='{}'({:p}) sameOwnerHeld={} sameRootHeld={} "
                    "ownerBodyLocal.pos=({:.2f},{:.2f},{:.2f}) ownerBodyLocalFinger=({:.3f},{:.3f},{:.3f}) "
                    "rootBodyLocal.pos=({:.2f},{:.2f},{:.2f}) rootBodyLocalFinger=({:.3f},{:.3f},{:.3f})",
                    handName(), nodeDebugName(ownerNodeAtGrab), static_cast<const void*>(ownerNodeAtGrab),
                    (ownerNodeAtGrab && ownerNodeAtGrab->collisionObject.get()) ? "yes" : "no",
                    (ownerNodeAtGrab && ownerNodeAtGrab->collisionObject.get() == bodyCollisionObjectAtGrab) ? "yes" : "no", nodeDebugName(rootNode),
                    static_cast<const void*>(rootNode), nodeDebugName(collidableNode), static_cast<const void*>(collidableNode), nodeDebugName(meshSourceNode),
                    static_cast<const void*>(meshSourceNode), ownerNodeAtGrab == collidableNode ? "yes" : "no", rootNode == collidableNode ? "yes" : "no",
                    _grabOwnerBodyLocalTransform.translate.x, _grabOwnerBodyLocalTransform.translate.y, _grabOwnerBodyLocalTransform.translate.z, ownerBodyLocalFinger.x,
                    ownerBodyLocalFinger.y, ownerBodyLocalFinger.z, _grabRootBodyLocalTransform.translate.x, _grabRootBodyLocalTransform.translate.y,
                    _grabRootBodyLocalTransform.translate.z, rootBodyLocalFinger.x, rootBodyLocalFinger.y, rootBodyLocalFinger.z);
            }

            ROCK_LOG_DEBUG(Hand,
                "{} GRAB HAND SPACE: pos=({:.1f},{:.1f},{:.1f}) "
                "palmPos=({:.1f},{:.1f},{:.1f}) pivotA=({:.1f},{:.1f},{:.1f}) grabPt=({:.1f},{:.1f},{:.1f})",
                handName(), _grabHandSpace.translate.x, _grabHandSpace.translate.y, _grabHandSpace.translate.z, palmPos.x, palmPos.y, palmPos.z, grabPivotAWorld.x,
                grabPivotAWorld.y, grabPivotAWorld.z, grabSurfacePoint.x, grabSurfacePoint.y, grabSurfacePoint.z);
            ROCK_LOG_DEBUG(Hand, "{} BODY LOCAL: pos=({:.2f},{:.2f},{:.2f}) scale={:.3f}", handName(), _grabBodyLocalTransform.translate.x, _grabBodyLocalTransform.translate.y,
                _grabBodyLocalTransform.translate.z, _grabBodyLocalTransform.scale);
        }

        {
            auto* bodyArray = world->GetBodyArray();
            auto* handFloats = reinterpret_cast<float*>(&bodyArray[_handBody.getBodyId().value]);
            auto* objFloats = reinterpret_cast<float*>(&bodyArray[objectBodyId.value]);
            ROCK_LOG_TRACE(Hand, "{} DIAG: handBody pos=({:.3f},{:.3f},{:.3f}) objBody pos=({:.3f},{:.3f},{:.3f})", handName(), handFloats[12], handFloats[13], handFloats[14],
                objFloats[12], objFloats[13], objFloats[14]);
            ROCK_LOG_TRACE(Hand, "{} DIAG: handNi pos=({:.1f},{:.1f},{:.1f}) objNi pos=({:.1f},{:.1f},{:.1f})", handName(), handWorldTransform.translate.x,
                handWorldTransform.translate.y, handWorldTransform.translate.z, objectWorldTransform.translate.x, objectWorldTransform.translate.y,
                objectWorldTransform.translate.z);

            float comX, comY, comZ;
            if (getBodyCOMWorld(world, objectBodyId, comX, comY, comZ)) {
                float comOffX, comOffY, comOffZ;
                getBodyCOMOffset(objFloats, comOffX, comOffY, comOffZ);
                float comDist =
                    std::sqrt((comX - objFloats[12]) * (comX - objFloats[12]) + (comY - objFloats[13]) * (comY - objFloats[13]) + (comZ - objFloats[14]) * (comZ - objFloats[14]));
                ROCK_LOG_TRACE(Hand,
                    "{} B8 COM: com=({:.3f},{:.3f},{:.3f}) origin=({:.3f},{:.3f},{:.3f}) "
                    "offset=({:.4f},{:.4f},{:.4f}) dist={:.4f} ({:.1f}gu)",
                    handName(), comX, comY, comZ, objFloats[12], objFloats[13], objFloats[14], comOffX, comOffY, comOffZ, comDist, comDist * 70.0f);
            }
        }

        {
            alignas(16) float zeroVel[4] = { 0, 0, 0, 0 };
            typedef void (*setVelDeferred_t)(void*, std::uint32_t, const float*, const float*);
            static REL::Relocation<setVelDeferred_t> setBodyVelocityDeferred{ REL::Offset(offsets::kFunc_SetBodyVelocityDeferred) };
            setBodyVelocityDeferred(world, objectBodyId.value, zeroVel, zeroVel);

            for (auto bid : _heldBodyIds) {
                if (bid != objectBodyId.value) {
                    setBodyVelocityDeferred(world, bid, zeroVel, zeroVel);
                }
            }
        }

        suppressHandCollisionForGrab(world);

        normalizeGrabbedInertia(world, objectBodyId, _savedObjectState);

        {
            RE::NiPoint3 palmPos = computePalmPositionFromHandBasis(handWorldTransform, _isLeft);
            RE::NiPoint3 grabPivotAWorld = computeGrabPivotAPositionFromHandBasis(handWorldTransform, _isLeft);
            constexpr float INV_HAVOK_SCALE = 1.0f / 70.0f;
            const RE::NiTransform desiredBodyTransformHandSpace = multiplyTransforms(_grabConstraintHandSpace, _grabBodyLocalTransform);

            float pivotAHk[4];
            pivotAHk[0] = grabPivotAWorld.x * INV_HAVOK_SCALE;
            pivotAHk[1] = grabPivotAWorld.y * INV_HAVOK_SCALE;
            pivotAHk[2] = grabPivotAWorld.z * INV_HAVOK_SCALE;
            pivotAHk[3] = 0.0f;

            float grabHk[4];
            grabHk[0] = grabSurfacePoint.x * INV_HAVOK_SCALE;
            grabHk[1] = grabSurfacePoint.y * INV_HAVOK_SCALE;
            grabHk[2] = grabSurfacePoint.z * INV_HAVOK_SCALE;
            grabHk[3] = 0.0f;

            {
                float pivotAToGrab = std::sqrt(
                    (pivotAHk[0] - grabHk[0]) * (pivotAHk[0] - grabHk[0]) + (pivotAHk[1] - grabHk[1]) * (pivotAHk[1] - grabHk[1]) +
                    (pivotAHk[2] - grabHk[2]) * (pivotAHk[2] - grabHk[2]));
                float palmToHandOrigin = std::sqrt((palmPos.x - handWorldTransform.translate.x) * (palmPos.x - handWorldTransform.translate.x) +
                    (palmPos.y - handWorldTransform.translate.y) * (palmPos.y - handWorldTransform.translate.y) +
                    (palmPos.z - handWorldTransform.translate.z) * (palmPos.z - handWorldTransform.translate.z));
                float pivotAToHandOrigin = std::sqrt((grabPivotAWorld.x - handWorldTransform.translate.x) * (grabPivotAWorld.x - handWorldTransform.translate.x) +
                    (grabPivotAWorld.y - handWorldTransform.translate.y) * (grabPivotAWorld.y - handWorldTransform.translate.y) +
                    (grabPivotAWorld.z - handWorldTransform.translate.z) * (grabPivotAWorld.z - handWorldTransform.translate.z));
                ROCK_LOG_DEBUG(Hand,
                    "GRAB DIAG {}: palmPos=({:.1f},{:.1f},{:.1f}) handPos=({:.1f},{:.1f},{:.1f}) "
                    "pivotA=({:.1f},{:.1f},{:.1f}) grabSurface=({:.1f},{:.1f},{:.1f}) meshGrab={} grabPointMode={} fallbackReason={} "
                    "pivotAToGrab_hk={:.4f} ({:.1f} game units) palmToHandOrigin={:.1f} pivotAToHandOrigin={:.1f} game units",
                    handName(), palmPos.x, palmPos.y, palmPos.z, handWorldTransform.translate.x, handWorldTransform.translate.y, handWorldTransform.translate.z, grabPivotAWorld.x,
                    grabPivotAWorld.y, grabPivotAWorld.z, grabSurfacePoint.x, grabSurfacePoint.y, grabSurfacePoint.z, meshGrabFound, grabPointMode, grabFallbackReason, pivotAToGrab,
                    pivotAToGrab * 70.0f, palmToHandOrigin, pivotAToHandOrigin);
            }

            _activeConstraint = createGrabConstraint(world, _handBody.getBodyId(), objectBodyId, pivotAHk, grabHk, desiredBodyTransformHandSpace, tau, damping, maxForce,
                proportionalRecovery, constantRecovery);
        }

        if (!_activeConstraint.isValid()) {
            ROCK_LOG_ERROR(Hand, "{} hand GRAB FAILED: constraint creation failed", handName());
            restoreGrabbedInertia(world, _savedObjectState);
            if (world && _savedObjectState.originalMotionPropsId != 0 && _savedObjectState.originalMotionPropsId != 1) {
                physics_recursive_wrappers::setMotionRecursive(rootNode, motionPresetFromSavedPropsId(_savedObjectState.originalMotionPropsId), false, true, true);
            }
            restoreHandCollisionAfterGrab(world);
            _savedObjectState.clear();
            _heldBodyIds.clear();
            return false;
        }

        {
            auto* cd = static_cast<char*>(_activeConstraint.constraintData);
            float* pivotA = reinterpret_cast<float*>(cd + offsets::kTransformA_Pos);
            float* pivotB = reinterpret_cast<float*>(cd + offsets::kTransformB_Pos);
            float* tA_col0 = reinterpret_cast<float*>(cd + offsets::kTransformA_Col0);
            float* tB_col0 = reinterpret_cast<float*>(cd + offsets::kTransformB_Col0);
            ROCK_LOG_TRACE(Hand, "{} DIAG: pivotA=({:.3f},{:.3f},{:.3f}) pivotB=({:.3f},{:.3f},{:.3f})", handName(), pivotA[0], pivotA[1], pivotA[2], pivotB[0], pivotB[1],
                pivotB[2]);
            ROCK_LOG_TRACE(Hand, "{} DIAG: tA_col0=({:.3f},{:.3f},{:.3f}) tB_col0=({:.3f},{:.3f},{:.3f})", handName(), tA_col0[0], tA_col0[1], tA_col0[2], tB_col0[0], tB_col0[1],
                tB_col0[2]);
            ROCK_LOG_TRACE(Hand, "{} DIAG: tau={:.3f} damping={:.2f} force={:.0f} propRecov={:.1f} constRecov={:.1f}", handName(), tau, damping, maxForce, proportionalRecovery,
                constantRecovery);
        }

        ROCK_LOG_DEBUG(Hand, "{} hand constraint grab: constraintId={}, hand body={}, obj body={}, {} held bodies", handName(), _activeConstraint.constraintId,
            _handBody.getBodyId().value, objectBodyId.value, _heldBodyIds.size());

        {
            typedef void enableBodyFlags_t(void*, std::uint32_t, std::uint32_t, std::uint32_t);
            static REL::Relocation<enableBodyFlags_t> enableBodyFlags{ REL::Offset(offsets::kFunc_EnableBodyFlags) };
            for (auto bid : _heldBodyIds) {
                enableBodyFlags(world, bid, 0x80, 0);
            }
        }
        _heldBodyContactFrame.store(100, std::memory_order_release);

        {
            int count = (std::min)(static_cast<int>(_heldBodyIds.size()), MAX_HELD_BODIES);
            for (int i = 0; i < count; i++) {
                _heldBodyIdsSnapshot[i] = _heldBodyIds[i];
            }
            _heldBodyIdsCount.store(count, std::memory_order_release);
            _isHoldingFlag.store(true, std::memory_order_release);
        }

        stopSelectionHighlight();
        const auto fingerPose = g_rockConfig.rockGrabMeshFingerPoseEnabled ?
            grab_finger_pose_runtime::solveGrabFingerPoseFromTriangles(
                grabMeshTriangles, handWorldTransform, _isLeft, computeGrabPivotAPositionFromHandBasis(handWorldTransform, _isLeft), grabSurfacePoint,
                g_rockConfig.rockGrabFingerMinValue) :
            grab_finger_pose_runtime::SolvedGrabFingerPose{};
        _grabFingerProbeStart = fingerPose.probeStart;
        _grabFingerProbeEnd = fingerPose.probeEnd;
        _hasGrabFingerProbeDebug = fingerPose.candidateTriangleCount > 0;
        applyRockGrabHandPose(_isLeft, fingerPose, _grabFingerJointPose, _hasGrabFingerJointPose, 0.0f);

        _state = HandState::HeldInit;
        clearPullRuntimeState();

        ROCK_LOG_INFO(Hand, "{} hand grab success -> HeldInit: bodyId={}", handName(), objectBodyId.value);
        return true;
    }

    void Hand::updateHeldObject(RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform,
        const HeldObjectPlayerSpaceFrame& playerSpaceFrame,
        float deltaTime,
        float forceFadeInTime,
        float tauMin)
    {
        if (!isHolding() || !world)
            return;
        if (!_activeConstraint.isValid()) {
            releaseGrabbedObject(world);
            return;
        }

        if (!_savedObjectState.refr || _savedObjectState.refr->IsDeleted() || _savedObjectState.refr->IsDisabled()) {
            releaseGrabbedObject(world);
            return;
        }

        suppressHandCollisionForGrab(world);

        _grabStartTime += deltaTime;

        {
            RE::NiTransform adjustedTarget{};
            if (computeAdjustedHandTransformTarget(adjustedTarget)) {
                if (!_hasAdjustedHandTransform) {
                    _adjustedHandTransform = handWorldTransform;
                    _grabVisualLerpElapsed = 0.0f;
                    _hasAdjustedHandTransform = true;
                }

                const bool lerpExpired = _grabVisualLerpElapsed >= _grabVisualLerpDuration;
                if (!g_rockConfig.rockGrabHandLerpEnabled || lerpExpired) {
                    _adjustedHandTransform = adjustedTarget;
                } else {
                    const auto advanced = hand_visual_lerp_math::advanceTransform(_adjustedHandTransform, adjustedTarget, g_rockConfig.rockGrabLerpSpeed,
                        g_rockConfig.rockGrabLerpAngularSpeed, deltaTime);
                    _adjustedHandTransform = advanced.transform;
                    _grabVisualLerpElapsed += (std::max)(0.0f, deltaTime);
                    if (advanced.reachedTarget || _grabVisualLerpElapsed >= _grabVisualLerpDuration) {
                        _adjustedHandTransform = adjustedTarget;
                    }
                }
            } else {
                _hasAdjustedHandTransform = false;
            }
        }

        if (_activeConstraint.constraintData) {
            auto* cd = static_cast<char*>(_activeConstraint.constraintData);
            const RE::NiTransform desiredBodyTransformHandSpace = multiplyTransforms(_grabConstraintHandSpace, _grabBodyLocalTransform);

            const RE::NiTransform desiredBodyToHandSpace = invertTransform(desiredBodyTransformHandSpace);

            {
                auto* target = reinterpret_cast<float*>(cd + ATOM_RAGDOLL_MOT + 0x10);
                grab_constraint_math::writeHavokRotationColumns(target, desiredBodyToHandSpace.rotate);
            }

            {
                auto* bodyArray = world->GetBodyArray();
                if (bodyArray && _handBody.getBodyId().value != INVALID_BODY_ID) {
                    auto* handBody = reinterpret_cast<const float*>(&bodyArray[_handBody.getBodyId().value]);
                    const RE::NiPoint3 grabPivotAWorld = computeGrabPivotAPositionFromHandBasis(handWorldTransform, _isLeft);
                    const RE::NiPoint3 pivotAHk{ grabPivotAWorld.x * kGameToHavokScale, grabPivotAWorld.y * kGameToHavokScale, grabPivotAWorld.z * kGameToHavokScale };
                    const RE::NiPoint3 handDelta{ pivotAHk.x - handBody[12], pivotAHk.y - handBody[13], pivotAHk.z - handBody[14] };
                    const RE::NiPoint3 pivotALocal = worldDeltaToBodyLocal(handBody, handDelta);
                    auto* pivotA = reinterpret_cast<float*>(cd + offsets::kTransformA_Pos);
                    pivotA[0] = pivotALocal.x;
                    pivotA[1] = pivotALocal.y;
                    pivotA[2] = pivotALocal.z;
                    pivotA[3] = 0.0f;
                }
            }

            {
                const RE::NiPoint3 pivotAHandspace = authoredHandspaceToRawHandspaceForHand(computeGrabPivotAHandspacePosition(_isLeft), _isLeft);
                const RE::NiPoint3 transformed = transform_math::localPointToWorld(desiredBodyToHandSpace, pivotAHandspace);

                float objScale = (_heldNode ? _heldNode->world.scale : 1.0f);
                constexpr float kHavokScale = 1.0f / 70.0f;
                float combinedScale = objScale * kHavokScale;
                auto* pivotB = reinterpret_cast<float*>(cd + offsets::kTransformB_Pos);

                alignas(16) float newPivotB[4] = { transformed.x * combinedScale, transformed.y * combinedScale, transformed.z * combinedScale, 0.0f };
                _mm_store_ps(pivotB, _mm_load_ps(newPivotB));
            }
        }

        float grabPositionErrorGameUnits = 0.0f;
        float grabRotationErrorDegrees = 0.0f;
        {
            const RE::NiTransform desiredBodyTransformHandSpace = multiplyTransforms(_grabConstraintHandSpace, _grabBodyLocalTransform);
            const RE::NiTransform desiredBodyWorld = multiplyTransforms(handWorldTransform, desiredBodyTransformHandSpace);
            const RE::NiTransform liveBodyWorld = getBodyWorldTransform(world, _savedObjectState.bodyId);
            grabPositionErrorGameUnits = translationDeltaGameUnits(liveBodyWorld, desiredBodyWorld);
            grabRotationErrorDegrees = rotationDeltaDegrees(liveBodyWorld.rotate, desiredBodyWorld.rotate);
        }

        const bool heldBodyColliding = isHeldBodyColliding();
        tickHeldBodyContact();

        grab_motion_controller::MotorInput motorInput{};
        motorInput.enabled = g_rockConfig.rockGrabAdaptiveMotorEnabled;
        motorInput.heldBodyColliding = heldBodyColliding;
        motorInput.positionErrorGameUnits = grabPositionErrorGameUnits;
        motorInput.rotationErrorDegrees = grabRotationErrorDegrees;
        motorInput.fullPositionErrorGameUnits = g_rockConfig.rockGrabAdaptivePositionFullError;
        motorInput.fullRotationErrorDegrees = g_rockConfig.rockGrabAdaptiveRotationFullError;
        motorInput.baseLinearTau = g_rockConfig.rockGrabLinearTau;
        motorInput.baseAngularTau = g_rockConfig.rockGrabAngularTau;
        motorInput.collisionTau = tauMin;
        motorInput.maxTau = g_rockConfig.rockGrabTauMax;
        motorInput.currentLinearTau = _activeConstraint.linearMotor ? _activeConstraint.linearMotor->tau : _activeConstraint.currentTau;
        motorInput.currentAngularTau = _activeConstraint.angularMotor ? _activeConstraint.angularMotor->tau : _activeConstraint.currentTau;
        motorInput.tauLerpSpeed = g_rockConfig.rockGrabTauLerpSpeed;
        motorInput.deltaTime = deltaTime;
        motorInput.baseMaxForce = _activeConstraint.targetMaxForce;
        motorInput.maxForceMultiplier = g_rockConfig.rockGrabAdaptiveMaxForceMultiplier;
        motorInput.mass = readBodyMass(world, _savedObjectState.bodyId);
        motorInput.forceToMassRatio = g_rockConfig.rockGrabMaxForceToMassRatio;
        motorInput.angularToLinearForceRatio = g_rockConfig.rockGrabAngularToLinearForceRatio;
        motorInput.fadeElapsed = _state == HandState::HeldInit ? _grabStartTime : forceFadeInTime;
        motorInput.fadeDuration = forceFadeInTime;
        motorInput.fadeStartAngularRatio = g_rockConfig.rockGrabFadeInStartAngularRatio;

        const auto motorTargets = grab_motion_controller::solveMotorTargets(motorInput);
        const float currentLinearForce = motorTargets.linearMaxForce;
        const float currentAngularForce = motorTargets.angularMaxForce;
        const float currentLinearTau = motorTargets.linearTau;
        const float currentAngularTau = motorTargets.angularTau;

        if (_state == HandState::HeldInit) {
            if (motorTargets.fadeFactor >= 0.999f) {
                _state = HandState::HeldBody;
                ROCK_LOG_DEBUG(Hand, "{} hand: HeldInit -> HeldBody (force fade-in complete, {:.2f}s)", handName(), _grabStartTime);
            }
        }

        if (_activeConstraint.angularMotor) {
            _activeConstraint.angularMotor->tau = currentAngularTau;
            _activeConstraint.angularMotor->maxForce = currentAngularForce;
            _activeConstraint.angularMotor->minForce = -currentAngularForce;
            _activeConstraint.angularMotor->damping = g_rockConfig.rockGrabAngularDamping;
            _activeConstraint.angularMotor->proportionalRecoveryVelocity = g_rockConfig.rockGrabAngularProportionalRecovery;
            _activeConstraint.angularMotor->constantRecoveryVelocity = g_rockConfig.rockGrabAngularConstantRecovery;
        }
        if (_activeConstraint.linearMotor) {
            _activeConstraint.linearMotor->tau = currentLinearTau;
            _activeConstraint.linearMotor->maxForce = currentLinearForce;
            _activeConstraint.linearMotor->minForce = -currentLinearForce;
            _activeConstraint.linearMotor->damping = g_rockConfig.rockGrabLinearDamping;
            _activeConstraint.linearMotor->proportionalRecoveryVelocity = g_rockConfig.rockGrabLinearProportionalRecovery;
            _activeConstraint.linearMotor->constantRecoveryVelocity = g_rockConfig.rockGrabLinearConstantRecovery;
        }
        _activeConstraint.currentTau = currentLinearTau;
        _activeConstraint.currentMaxForce = currentLinearForce;

        {
            const auto compensationResult = applyHeldMotionCompensation(world, _savedObjectState.bodyId, _heldBodyIds, playerSpaceFrame,
                _lastPlayerSpaceVelocityHavok, g_rockConfig.rockGrabVelocityDamping, g_rockConfig.rockGrabResidualVelocityDamping);
            if (compensationResult.hasPrimaryVelocity) {
                _heldLocalLinearVelocityHistory[_heldLocalLinearVelocityHistoryNext] = compensationResult.primaryLocalLinearVelocity;
                _heldLocalLinearVelocityHistoryNext = (_heldLocalLinearVelocityHistoryNext + 1) % _heldLocalLinearVelocityHistory.size();
                if (_heldLocalLinearVelocityHistoryCount < _heldLocalLinearVelocityHistory.size()) {
                    ++_heldLocalLinearVelocityHistoryCount;
                }
            }
            _lastPlayerSpaceVelocityHavok = (playerSpaceFrame.enabled && !playerSpaceFrame.warp) ? playerSpaceFrame.velocityHavok : RE::NiPoint3{};
        }

        if (g_rockConfig.rockGrabMeshFingerPoseEnabled && _hasGrabMeshPoseData && !_grabLocalMeshTriangles.empty()) {
            const int updateInterval = (std::max)(1, g_rockConfig.rockGrabFingerPoseUpdateInterval);
            ++_grabFingerPoseFrameCounter;
            if (_grabFingerPoseFrameCounter >= updateInterval) {
                _grabFingerPoseFrameCounter = 0;
                const RE::NiTransform liveBodyWorld = getBodyWorldTransform(world, _savedObjectState.bodyId);
                const RE::NiTransform currentNodeWorld = deriveNodeWorldFromBodyWorld(liveBodyWorld, _grabBodyLocalTransform);
                const auto worldTriangles = rebuildTrianglesInWorldSpace(_grabLocalMeshTriangles, currentNodeWorld);
                const RE::NiPoint3 grabSurfaceWorld = transform_math::localPointToWorld(currentNodeWorld, _grabSurfacePointLocal);
                const auto fingerPose = grab_finger_pose_runtime::solveGrabFingerPoseFromTriangles(worldTriangles, handWorldTransform, _isLeft,
                    computeGrabPivotAPositionFromHandBasis(handWorldTransform, _isLeft), grabSurfaceWorld, g_rockConfig.rockGrabFingerMinValue);
                _grabFingerProbeStart = fingerPose.probeStart;
                _grabFingerProbeEnd = fingerPose.probeEnd;
                _hasGrabFingerProbeDebug = fingerPose.candidateTriangleCount > 0;
                applyRockGrabHandPose(_isLeft, fingerPose, _grabFingerJointPose, _hasGrabFingerJointPose, deltaTime);
            }
        }

        _heldLogCounter++;
        if (_heldLogCounter >= 45) {
            _heldLogCounter = 0;

            if (g_rockConfig.rockDebugGrabFrameLogging) {
                const RE::NiTransform desiredNodeWorldRaw = multiplyTransforms(handWorldTransform, _grabHandSpace);
                const RE::NiTransform desiredNodeWorldConstraint = multiplyTransforms(handWorldTransform, _grabConstraintHandSpace);
                const RE::NiTransform desiredBodyTransformHandSpaceRaw = multiplyTransforms(_grabHandSpace, _grabBodyLocalTransform);
                const RE::NiTransform desiredBodyTransformHandSpaceConstraint = multiplyTransforms(_grabConstraintHandSpace, _grabBodyLocalTransform);
                const RE::NiTransform desiredBodyWorldRaw = multiplyTransforms(handWorldTransform, desiredBodyTransformHandSpaceRaw);
                const RE::NiTransform desiredBodyWorldConstraint = multiplyTransforms(handWorldTransform, desiredBodyTransformHandSpaceConstraint);
                const RE::NiMatrix3 invRot = desiredBodyTransformHandSpaceConstraint.rotate.Transpose();
                const RE::NiTransform liveBodyWorld = getBodyWorldTransform(world, _savedObjectState.bodyId);
                auto* ownerCell = _savedObjectState.refr ? _savedObjectState.refr->GetParentCell() : nullptr;
                auto* heldBhkWorld = ownerCell ? ownerCell->GetbhkWorld() : nullptr;
                auto* bodyCollisionObject = heldBhkWorld ? RE::bhkNPCollisionObject::Getbhk(heldBhkWorld, _savedObjectState.bodyId) : nullptr;
                auto* ownerNode = bodyCollisionObject ? bodyCollisionObject->sceneObject : nullptr;
                auto* hitNode = (_currentSelection.refr == _savedObjectState.refr) ? _currentSelection.hitNode : nullptr;
                auto* rootNode = _savedObjectState.refr ? _savedObjectState.refr->Get3D() : nullptr;

                struct NodeFrameMetrics
                {
                    RE::NiAVObject* node = nullptr;
                    RE::NiTransform world = {};
                    RE::NiTransform expectedWorld = {};
                    RE::NiPoint3 finger = {};
                    RE::NiPoint3 expectedFinger = {};
                    float rotErrDeg = -1.0f;
                    float posErrGameUnits = -1.0f;
                    bool hasCollisionObject = false;
                    bool ownsBodyCollisionObject = false;
                };

                const auto captureNodeMetrics = [&](RE::NiAVObject* node, const RE::NiTransform& bodyLocalTransform) {
                    NodeFrameMetrics metrics{};
                    metrics.node = node;
                    metrics.world = node ? node->world : liveBodyWorld;
                    metrics.expectedWorld = node ? deriveNodeWorldFromBodyWorld(liveBodyWorld, bodyLocalTransform) : liveBodyWorld;
                    metrics.finger = getMatrixColumn(metrics.world.rotate, 0);
                    metrics.expectedFinger = getMatrixColumn(metrics.expectedWorld.rotate, 0);
                    if (node) {
                        metrics.rotErrDeg = rotationDeltaDegrees(metrics.world.rotate, metrics.expectedWorld.rotate);
                        metrics.posErrGameUnits = translationDeltaGameUnits(metrics.world, metrics.expectedWorld);
                        auto* nodeCollisionObject = node->collisionObject.get();
                        metrics.hasCollisionObject = (nodeCollisionObject != nullptr);
                        metrics.ownsBodyCollisionObject = (nodeCollisionObject != nullptr && nodeCollisionObject == bodyCollisionObject);
                    }
                    return metrics;
                };

                const NodeFrameMetrics ownerMetrics = captureNodeMetrics(ownerNode, _grabOwnerBodyLocalTransform);
                const NodeFrameMetrics hitMetrics = captureNodeMetrics(hitNode, _grabBodyLocalTransform);
                const NodeFrameMetrics heldMetrics = captureNodeMetrics(_heldNode, _grabBodyLocalTransform);
                const NodeFrameMetrics rootMetrics = captureNodeMetrics(rootNode, _grabRootBodyLocalTransform);

                const RE::NiPoint3 worldPosDelta = desiredNodeWorldConstraint.translate - desiredNodeWorldRaw.translate;
                const RE::NiPoint3 rawFinger = getMatrixColumn(handWorldTransform.rotate, 2);
                const RE::NiPoint3 constraintFinger = getMatrixColumn(handWorldTransform.rotate, 2);
                const RE::NiPoint3 desiredRawFinger = getMatrixColumn(desiredBodyWorldRaw.rotate, 0);
                const RE::NiPoint3 desiredConstraintFinger = getMatrixColumn(desiredBodyWorldConstraint.rotate, 0);
                const RE::NiPoint3 bodyFinger = getMatrixColumn(liveBodyWorld.rotate, 0);
                const float rawRowMax = max3(axisDeltaDegrees(getMatrixRow(liveBodyWorld.rotate, 0), getMatrixRow(desiredBodyWorldRaw.rotate, 0)),
                    axisDeltaDegrees(getMatrixRow(liveBodyWorld.rotate, 1), getMatrixRow(desiredBodyWorldRaw.rotate, 1)),
                    axisDeltaDegrees(getMatrixRow(liveBodyWorld.rotate, 2), getMatrixRow(desiredBodyWorldRaw.rotate, 2)));
                const float rawColMax = max3(axisDeltaDegrees(getMatrixColumn(liveBodyWorld.rotate, 0), getMatrixColumn(desiredBodyWorldRaw.rotate, 0)),
                    axisDeltaDegrees(getMatrixColumn(liveBodyWorld.rotate, 1), getMatrixColumn(desiredBodyWorldRaw.rotate, 1)),
                    axisDeltaDegrees(getMatrixColumn(liveBodyWorld.rotate, 2), getMatrixColumn(desiredBodyWorldRaw.rotate, 2)));
                const float constraintRow0 = axisDeltaDegrees(getMatrixRow(liveBodyWorld.rotate, 0), getMatrixRow(desiredBodyWorldConstraint.rotate, 0));
                const float constraintRow1 = axisDeltaDegrees(getMatrixRow(liveBodyWorld.rotate, 1), getMatrixRow(desiredBodyWorldConstraint.rotate, 1));
                const float constraintRow2 = axisDeltaDegrees(getMatrixRow(liveBodyWorld.rotate, 2), getMatrixRow(desiredBodyWorldConstraint.rotate, 2));
                const float constraintCol0 = axisDeltaDegrees(getMatrixColumn(liveBodyWorld.rotate, 0), getMatrixColumn(desiredBodyWorldConstraint.rotate, 0));
                const float constraintCol1 = axisDeltaDegrees(getMatrixColumn(liveBodyWorld.rotate, 1), getMatrixColumn(desiredBodyWorldConstraint.rotate, 1));
                const float constraintCol2 = axisDeltaDegrees(getMatrixColumn(liveBodyWorld.rotate, 2), getMatrixColumn(desiredBodyWorldConstraint.rotate, 2));
                const float constraintRowMax = max3(constraintRow0, constraintRow1, constraintRow2);
                const float constraintColMax = max3(constraintCol0, constraintCol1, constraintCol2);

                auto* constraintData = static_cast<char*>(_activeConstraint.constraintData);
                auto* target_bRca = reinterpret_cast<float*>(constraintData + ATOM_RAGDOLL_MOT + 0x10);
                auto* transformB_rot = reinterpret_cast<float*>(constraintData + offsets::kTransformB_Col0);
                const RE::NiMatrix3 targetAsHkColumns = matrixFromHkColumns(target_bRca);
                const RE::NiMatrix3 targetAsHkRows = matrixFromHkRows(target_bRca);
                const RE::NiMatrix3 transformBAsHkColumns = matrixFromHkColumns(transformB_rot);
                const RE::NiMatrix3 constraintTargetInv = desiredBodyTransformHandSpaceConstraint.rotate.Transpose();
                const RE::NiMatrix3 rawTargetInv = desiredBodyTransformHandSpaceRaw.rotate.Transpose();
                const int ragdollMotorEnabled = *(constraintData + ATOM_RAGDOLL_MOT + 0x02) ? 1 : 0;

                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB FRAME HOLD: rawVsConstraintWorld={:.2f}deg rawVsConstraintTarget={:.2f}deg "
                    "bodyVsRaw={:.2f}deg bodyVsConstraint={:.2f}deg "
                    "ownerErr={:.2f}deg/{:.2f}gu hitErr={:.2f}deg/{:.2f}gu "
                    "heldErr={:.2f}deg/{:.2f}gu rootErr={:.2f}deg/{:.2f}gu "
                    "worldPosDelta=({:.2f},{:.2f},{:.2f}) "
                    "rawFinger=({:.3f},{:.3f},{:.3f}) constraintFinger=({:.3f},{:.3f},{:.3f})",
                    handName(), rotationDeltaDegrees(desiredNodeWorldRaw.rotate, desiredNodeWorldConstraint.rotate),
                    rotationDeltaDegrees(desiredBodyTransformHandSpaceRaw.rotate, desiredBodyTransformHandSpaceConstraint.rotate),
                    rotationDeltaDegrees(liveBodyWorld.rotate, desiredBodyWorldRaw.rotate), rotationDeltaDegrees(liveBodyWorld.rotate, desiredBodyWorldConstraint.rotate),
                    ownerMetrics.rotErrDeg, ownerMetrics.posErrGameUnits, hitMetrics.rotErrDeg, hitMetrics.posErrGameUnits, heldMetrics.rotErrDeg, heldMetrics.posErrGameUnits,
                    rootMetrics.rotErrDeg, rootMetrics.posErrGameUnits, worldPosDelta.x, worldPosDelta.y, worldPosDelta.z, rawFinger.x, rawFinger.y, rawFinger.z,
                    constraintFinger.x, constraintFinger.y, constraintFinger.z);

                ROCK_LOG_TRACE(Hand,
                    "{} GRAB FRAME NODES: bodyColl={:p} "
                    "owner='{}'({:p}) hasCol={} ownsBodyCol={} "
                    "hit='{}'({:p}) hasCol={} ownsBodyCol={} "
                    "held='{}'({:p}) hasCol={} ownsBodyCol={} "
                    "root='{}'({:p}) hasCol={} ownsBodyCol={} "
                    "sameOwnerHeld={} sameHitHeld={} sameRootHeld={}",
                    handName(), static_cast<const void*>(bodyCollisionObject), nodeDebugName(ownerMetrics.node), static_cast<const void*>(ownerMetrics.node),
                    ownerMetrics.hasCollisionObject ? "yes" : "no", ownerMetrics.ownsBodyCollisionObject ? "yes" : "no", nodeDebugName(hitMetrics.node),
                    static_cast<const void*>(hitMetrics.node), hitMetrics.hasCollisionObject ? "yes" : "no", hitMetrics.ownsBodyCollisionObject ? "yes" : "no",
                    nodeDebugName(heldMetrics.node), static_cast<const void*>(heldMetrics.node), heldMetrics.hasCollisionObject ? "yes" : "no",
                    heldMetrics.ownsBodyCollisionObject ? "yes" : "no", nodeDebugName(rootMetrics.node), static_cast<const void*>(rootMetrics.node),
                    rootMetrics.hasCollisionObject ? "yes" : "no", rootMetrics.ownsBodyCollisionObject ? "yes" : "no", ownerMetrics.node == heldMetrics.node ? "yes" : "no",
                    hitMetrics.node == heldMetrics.node ? "yes" : "no", rootMetrics.node == heldMetrics.node ? "yes" : "no");

                ROCK_LOG_TRACE(Hand,
                    "{} GRAB FRAME VISUALS: desiredRawFinger=({:.3f},{:.3f},{:.3f}) desiredConstraintFinger=({:.3f},{:.3f},{:.3f}) "
                    "bodyFinger=({:.3f},{:.3f},{:.3f}) "
                    "ownerFinger=({:.3f},{:.3f},{:.3f}) ownerExpectedFinger=({:.3f},{:.3f},{:.3f}) "
                    "hitFinger=({:.3f},{:.3f},{:.3f}) hitExpectedFinger=({:.3f},{:.3f},{:.3f}) "
                    "heldFinger=({:.3f},{:.3f},{:.3f}) heldExpectedFinger=({:.3f},{:.3f},{:.3f}) "
                    "rootFinger=({:.3f},{:.3f},{:.3f}) rootExpectedFinger=({:.3f},{:.3f},{:.3f}) "
                    "targetInvFinger=({:.3f},{:.3f},{:.3f})",
                    handName(), desiredRawFinger.x, desiredRawFinger.y, desiredRawFinger.z, desiredConstraintFinger.x, desiredConstraintFinger.y, desiredConstraintFinger.z,
                    bodyFinger.x, bodyFinger.y, bodyFinger.z, ownerMetrics.finger.x, ownerMetrics.finger.y, ownerMetrics.finger.z, ownerMetrics.expectedFinger.x,
                    ownerMetrics.expectedFinger.y, ownerMetrics.expectedFinger.z, hitMetrics.finger.x, hitMetrics.finger.y, hitMetrics.finger.z, hitMetrics.expectedFinger.x,
                    hitMetrics.expectedFinger.y, hitMetrics.expectedFinger.z, heldMetrics.finger.x, heldMetrics.finger.y, heldMetrics.finger.z, heldMetrics.expectedFinger.x,
                    heldMetrics.expectedFinger.y, heldMetrics.expectedFinger.z, rootMetrics.finger.x, rootMetrics.finger.y, rootMetrics.finger.z, rootMetrics.expectedFinger.x,
                    rootMetrics.expectedFinger.y, rootMetrics.expectedFinger.z, invRot.entry[0][0], invRot.entry[1][0], invRot.entry[2][0]);

                ROCK_LOG_TRACE(Hand,
                    "{} GRAB ANGULAR PROBE: rawAxisErr(rowMax={:.2f} colMax={:.2f}) "
                    "constraintAxisErr(rowMax={:.2f} colMax={:.2f} rows=({:.2f},{:.2f},{:.2f}) cols=({:.2f},{:.2f},{:.2f})) "
                    "targetMem(colsVsConstraintInv={:.2f} rowsVsConstraintInv={:.2f} "
                    "colsVsConstraintForward={:.2f} colsVsRawInv={:.2f} colsVsTransformB={:.2f}) "
                    "motorErr=({:.2f}gu,{:.2f}deg,{:.2f}) mass={:.2f} "
                    "ragEnabled={} angMotor={:p} angTau={:.3f} linTau={:.3f} linF={:.0f} angF={:.0f}",
                    handName(), rawRowMax, rawColMax, constraintRowMax, constraintColMax, constraintRow0, constraintRow1, constraintRow2, constraintCol0, constraintCol1,
                    constraintCol2, rotationDeltaDegrees(targetAsHkColumns, constraintTargetInv), rotationDeltaDegrees(targetAsHkRows, constraintTargetInv),
                    rotationDeltaDegrees(targetAsHkColumns, desiredBodyTransformHandSpaceConstraint.rotate), rotationDeltaDegrees(targetAsHkColumns, rawTargetInv),
                    rotationDeltaDegrees(targetAsHkColumns, transformBAsHkColumns), grabPositionErrorGameUnits, grabRotationErrorDegrees, motorTargets.errorFactor, motorInput.mass,
                    ragdollMotorEnabled, static_cast<const void*>(_activeConstraint.angularMotor), currentAngularTau, currentLinearTau, currentLinearForce, currentAngularForce);
            }

            auto* bodyArray = world->GetBodyArray();
            auto* h = reinterpret_cast<float*>(&bodyArray[_handBody.getBodyId().value]);
            auto* o = reinterpret_cast<float*>(&bodyArray[_savedObjectState.bodyId.value]);

            auto* cd = static_cast<char*>(_activeConstraint.constraintData);
            auto* pivotA_local = reinterpret_cast<float*>(cd + offsets::kTransformA_Pos);
            auto* pivotB_local = reinterpret_cast<float*>(cd + offsets::kTransformB_Pos);

            float pax = h[12] + h[0] * pivotA_local[0] + h[4] * pivotA_local[1] + h[8] * pivotA_local[2];
            float pay = h[13] + h[1] * pivotA_local[0] + h[5] * pivotA_local[1] + h[9] * pivotA_local[2];
            float paz = h[14] + h[2] * pivotA_local[0] + h[6] * pivotA_local[1] + h[10] * pivotA_local[2];

            float pbx = o[12] + o[0] * pivotB_local[0] + o[4] * pivotB_local[1] + o[8] * pivotB_local[2];
            float pby = o[13] + o[1] * pivotB_local[0] + o[5] * pivotB_local[1] + o[9] * pivotB_local[2];
            float pbz = o[14] + o[2] * pivotB_local[0] + o[6] * pivotB_local[1] + o[10] * pivotB_local[2];

            float ex = pax - pbx, ey = pay - pby, ez = paz - pbz;
            float pivotErr = std::sqrt(ex * ex + ey * ey + ez * ez);

            float dx = o[12] - h[12], dy = o[13] - h[13], dz = o[14] - h[14];
            float bodyDist = std::sqrt(dx * dx + dy * dy + dz * dz);

            float objVelMag = 0.0f;
            {
                auto* objMotion = world->GetBodyMotion(_savedObjectState.bodyId);
                if (objMotion) {
                    auto* mv = reinterpret_cast<float*>(reinterpret_cast<char*>(objMotion) + 0x40);
                    objVelMag = std::sqrt(mv[0] * mv[0] + mv[1] * mv[1] + mv[2] * mv[2]);
                }
            }

            ROCK_LOG_DEBUG(Hand,
                "{} HELD: angTau={:.3f} linTau={:.3f} linF={:.0f} angF={:.0f} "
                "ERR={:.4f}({:.1f}gu) bDist={:.3f} objVel={:.3f} "
                "paW=({:.1f},{:.1f},{:.1f}) pbW=({:.1f},{:.1f},{:.1f}) "
                "handW=({:.1f},{:.1f},{:.1f}) objW=({:.1f},{:.1f},{:.1f})",
                handName(), currentAngularTau, currentLinearTau, currentLinearForce, currentAngularForce, pivotErr, pivotErr * 70.0f, bodyDist, objVelMag, pax * 70.0f, pay * 70.0f,
                paz * 70.0f, pbx * 70.0f, pby * 70.0f, pbz * 70.0f, h[12] * 70.0f, h[13] * 70.0f, h[14] * 70.0f, o[12] * 70.0f, o[13] * 70.0f, o[14] * 70.0f);

            _notifCounter++;
            if (g_rockConfig.rockDebugShowGrabNotifications && _notifCounter >= 6) {
                _notifCounter = 0;
                float paToHand = std::sqrt((pax - h[12]) * (pax - h[12]) + (pay - h[13]) * (pay - h[13]) + (paz - h[14]) * (paz - h[14])) * 70.0f;
                float pbToObj = std::sqrt((pbx - o[12]) * (pbx - o[12]) + (pby - o[13]) * (pby - o[13]) + (pbz - o[14]) * (pbz - o[14])) * 70.0f;
                f4vr::showNotification(
                    std::format("[ROCK] err={:.1f}gu F={:.0f} vel={:.2f} paOff={:.1f} pbOff={:.1f}", pivotErr * 70.0f, currentLinearForce, objVelMag, paToHand, pbToObj));
            }
        }

        {
            typedef void activateBody_t(void*, std::uint32_t);
            static REL::Relocation<activateBody_t> activateBody{ REL::Offset(offsets::kFunc_ActivateBody) };
            activateBody(world, _savedObjectState.bodyId.value);
        }
    }

    void Hand::releaseGrabbedObject(RE::hknpWorld* world)
    {
        if (!isHolding())
            return;

        ROCK_LOG_INFO(Hand, "{} hand RELEASE: bodyId={} constraintId={}", handName(), _savedObjectState.bodyId.value,
            _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu);

        if (world && _heldLocalLinearVelocityHistoryCount > 0) {
            std::array<RE::NiPoint3, GRAB_RELEASE_VELOCITY_HISTORY> orderedHistory{};
            const std::size_t historySize = _heldLocalLinearVelocityHistory.size();
            const std::size_t firstIndex = (_heldLocalLinearVelocityHistoryNext + historySize - _heldLocalLinearVelocityHistoryCount) % historySize;
            for (std::size_t i = 0; i < _heldLocalLinearVelocityHistoryCount; ++i) {
                orderedHistory[i] = _heldLocalLinearVelocityHistory[(firstIndex + i) % historySize];
            }

            const RE::NiPoint3 localReleaseVelocity = held_object_physics_math::maxMagnitudeVelocity(orderedHistory, _heldLocalLinearVelocityHistoryCount);
            const RE::NiPoint3 releaseVelocity =
                held_object_physics_math::composeReleaseVelocity(localReleaseVelocity, _lastPlayerSpaceVelocityHavok, g_rockConfig.rockThrowVelocityMultiplier);
            setHeldLinearVelocity(world, _savedObjectState.bodyId, _heldBodyIds, releaseVelocity);
            ROCK_LOG_DEBUG(Hand,
                "{} hand RELEASE VELOCITY: local=({:.3f},{:.3f},{:.3f}) player=({:.3f},{:.3f},{:.3f}) final=({:.3f},{:.3f},{:.3f}) history={} multiplier={:.2f}",
                handName(), localReleaseVelocity.x, localReleaseVelocity.y, localReleaseVelocity.z, _lastPlayerSpaceVelocityHavok.x, _lastPlayerSpaceVelocityHavok.y,
                _lastPlayerSpaceVelocityHavok.z, releaseVelocity.x, releaseVelocity.y, releaseVelocity.z, _heldLocalLinearVelocityHistoryCount,
                g_rockConfig.rockThrowVelocityMultiplier);
        }

        restoreGrabbedInertia(world, _savedObjectState);

        if (world && _savedObjectState.originalMotionPropsId != 0 && _savedObjectState.originalMotionPropsId != 1) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand: object tree stays DYNAMIC on release "
                "(successful active prep is not partially restored per-body)",
                handName());
        }

        _isHoldingFlag.store(false, std::memory_order_release);
        _heldBodyIdsCount.store(0, std::memory_order_release);
        _heldBodyContactFrame.store(100, std::memory_order_release);

        if (world) {
            typedef void disableBodyFlags_t(void*, std::uint32_t, std::uint32_t, std::uint32_t);
            static REL::Relocation<disableBodyFlags_t> disableBodyFlags{ REL::Offset(offsets::kFunc_DisableBodyFlags) };
            for (auto bid : _heldBodyIds) {
                disableBodyFlags(world, bid, 0x80, 0);
            }
        }

        if (_activeConstraint.isValid()) {
            destroyGrabConstraint(world, _activeConstraint);
        }

        restoreHandCollisionAfterGrab(world);

        if (frik::api::FRIKApi::inst) {
            frik::api::FRIKApi::inst->clearHandPose("ROCK_Grab", handFromBool(_isLeft));
        }
        _savedObjectState.clear();
        _activeConstraint.clear();
        _heldBodyIds.clear();
        _grabHandSpace = RE::NiTransform();
        _adjustedHandTransform = RE::NiTransform();
        _hasAdjustedHandTransform = false;
        _grabVisualLerpElapsed = 0.0f;
        _grabVisualLerpDuration = g_rockConfig.rockGrabLerpMaxTime;
        _grabFingerProbeStart = {};
        _grabFingerProbeEnd = {};
        _hasGrabFingerProbeDebug = false;
        _grabFingerJointPose = {};
        _hasGrabFingerJointPose = false;
        _grabLocalMeshTriangles.clear();
        _grabSurfacePointLocal = {};
        _hasGrabMeshPoseData = false;
        _grabFingerPoseFrameCounter = 0;
        _heldLocalLinearVelocityHistory = {};
        _heldLocalLinearVelocityHistoryCount = 0;
        _heldLocalLinearVelocityHistoryNext = 0;
        _lastPlayerSpaceVelocityHavok = {};
        _grabConstraintHandSpace = RE::NiTransform();
        _grabBodyLocalTransform = RE::NiTransform();
        _grabRootBodyLocalTransform = RE::NiTransform();
        _grabOwnerBodyLocalTransform = RE::NiTransform();
        _heldNode = nullptr;
        _currentSelection.clear();
        _state = HandState::Idle;

        ROCK_LOG_DEBUG(Hand, "{} hand: Idle", handName());
    }
}
