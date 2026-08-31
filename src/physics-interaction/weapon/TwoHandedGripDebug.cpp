#include "physics-interaction/weapon/TwoHandedGrip.h"
#include "physics-interaction/weapon/TwoHandedGripInternal.h"

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
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace rock
{
    using namespace two_handed_grip_internal;

    namespace
    {
        constexpr float RADIANS_TO_DEGREES =
            57.295779513082320876f;

        struct ParityAxes
        {
            RE::NiPoint3 x{};
            RE::NiPoint3 y{};
            RE::NiPoint3 z{};
            bool valid{ false };
        };

        struct ParityHandFrame
        {
            RE::NiTransform handInWeapon{};
            RE::NiPoint3 palmWeaponLocal{};
            RE::NiPoint3 fingerAxisWeaponLocal{};
            RE::NiPoint3 palmAxisWeaponLocal{};
            RE::NiPoint3 crossAxisWeaponLocal{};
            bool valid{ false };
        };

        struct ParityGripAxisComparison
        {
            RE::NiPoint3 expectedLeftAxis{};
            RE::NiPoint3 actualLeftAxis{};
            float angularErrorDegrees{ 0.0f };
            float primaryPointMirrorError{ 0.0f };
            float supportPointMirrorError{ 0.0f };
            bool valid{ false };
        };

        struct ParityRotationResidual
        {
            float totalDegrees{ -1.0f };
            // Components are expressed about the authored anatomical axes:
            // finger +X, palm normal -Y, and cross-palm +Z/raw -Z.
            RE::NiPoint3 semanticComponentsDegrees{};
            bool valid{ false };
        };

        struct PoseHierarchyBoneNames
        {
            std::string_view label{};
            std::string_view left{};
            std::string_view right{};
            int fingerIndex{ -1 };
        };

        inline constexpr std::array<PoseHierarchyBoneNames, 23>
            kPoseHierarchyBoneNames{
                PoseHierarchyBoneNames{ "Collarbone", "LArm_Collarbone", "RArm_Collarbone", -1 },
                PoseHierarchyBoneNames{ "UpperArm", "LArm_UpperArm", "RArm_UpperArm", -1 },
                PoseHierarchyBoneNames{ "UpperTwist1", "LArm_UpperTwist1", "RArm_UpperTwist1", -1 },
                PoseHierarchyBoneNames{ "UpperTwist2", "LArm_UpperTwist2", "RArm_UpperTwist2", -1 },
                PoseHierarchyBoneNames{ "ForeArm1", "LArm_ForeArm1", "RArm_ForeArm1", -1 },
                PoseHierarchyBoneNames{ "ForeArm2", "LArm_ForeArm2", "RArm_ForeArm2", -1 },
                PoseHierarchyBoneNames{ "ForeArm3", "LArm_ForeArm3", "RArm_ForeArm3", -1 },
                PoseHierarchyBoneNames{ "Hand", "LArm_Hand", "RArm_Hand", -1 },
                PoseHierarchyBoneNames{ "Finger11", "LArm_Finger11", "RArm_Finger11", 0 },
                PoseHierarchyBoneNames{ "Finger12", "LArm_Finger12", "RArm_Finger12", 1 },
                PoseHierarchyBoneNames{ "Finger13", "LArm_Finger13", "RArm_Finger13", 2 },
                PoseHierarchyBoneNames{ "Finger21", "LArm_Finger21", "RArm_Finger21", 3 },
                PoseHierarchyBoneNames{ "Finger22", "LArm_Finger22", "RArm_Finger22", 4 },
                PoseHierarchyBoneNames{ "Finger23", "LArm_Finger23", "RArm_Finger23", 5 },
                PoseHierarchyBoneNames{ "Finger31", "LArm_Finger31", "RArm_Finger31", 6 },
                PoseHierarchyBoneNames{ "Finger32", "LArm_Finger32", "RArm_Finger32", 7 },
                PoseHierarchyBoneNames{ "Finger33", "LArm_Finger33", "RArm_Finger33", 8 },
                PoseHierarchyBoneNames{ "Finger41", "LArm_Finger41", "RArm_Finger41", 9 },
                PoseHierarchyBoneNames{ "Finger42", "LArm_Finger42", "RArm_Finger42", 10 },
                PoseHierarchyBoneNames{ "Finger43", "LArm_Finger43", "RArm_Finger43", 11 },
                PoseHierarchyBoneNames{ "Finger51", "LArm_Finger51", "RArm_Finger51", 12 },
                PoseHierarchyBoneNames{ "Finger52", "LArm_Finger52", "RArm_Finger52", 13 },
                PoseHierarchyBoneNames{ "Finger53", "LArm_Finger53", "RArm_Finger53", 14 },
            };

        [[nodiscard]] bool finitePoint(const RE::NiPoint3& point)
        {
            return std::isfinite(point.x) && std::isfinite(point.y) &&
                   std::isfinite(point.z);
        }

        [[nodiscard]] float pointDistance(
            const RE::NiPoint3& left,
            const RE::NiPoint3& right)
        {
            const float x = left.x - right.x;
            const float y = left.y - right.y;
            const float z = left.z - right.z;
            return std::sqrt(x * x + y * y + z * z);
        }

        [[nodiscard]] ParityGripAxisComparison compareParityGripAxes(
            const RE::NiPoint3& rightPrimary,
            const RE::NiPoint3& rightSupport,
            const RE::NiPoint3& leftPrimary,
            const RE::NiPoint3& leftSupport)
        {
            ParityGripAxisComparison result{};
            const RE::NiPoint3 rightAxis{
                rightSupport.x - rightPrimary.x,
                rightSupport.y - rightPrimary.y,
                rightSupport.z - rightPrimary.z,
            };
            result.expectedLeftAxis = RE::NiPoint3{
                -rightAxis.x,
                rightAxis.y,
                rightAxis.z,
            };
            result.actualLeftAxis = RE::NiPoint3{
                leftSupport.x - leftPrimary.x,
                leftSupport.y - leftPrimary.y,
                leftSupport.z - leftPrimary.z,
            };
            result.primaryPointMirrorError = pointDistance(
                leftPrimary,
                RE::NiPoint3{
                    -rightPrimary.x,
                    rightPrimary.y,
                    rightPrimary.z,
                });
            result.supportPointMirrorError = pointDistance(
                leftSupport,
                RE::NiPoint3{
                    -rightSupport.x,
                    rightSupport.y,
                    rightSupport.z,
                });

            const auto vectorDot = [](
                                       const RE::NiPoint3& left,
                                       const RE::NiPoint3& right) {
                return left.x * right.x + left.y * right.y +
                       left.z * right.z;
            };
            const float expectedLength = std::sqrt(vectorDot(
                result.expectedLeftAxis,
                result.expectedLeftAxis));
            const float actualLength = std::sqrt(vectorDot(
                result.actualLeftAxis,
                result.actualLeftAxis));
            if (!std::isfinite(expectedLength) ||
                !std::isfinite(actualLength) ||
                expectedLength <= 0.0001f || actualLength <= 0.0001f) {
                return result;
            }
            const float cosine = std::clamp(
                vectorDot(
                    result.expectedLeftAxis,
                    result.actualLeftAxis) /
                    (expectedLength * actualLength),
                -1.0f,
                1.0f);
            result.angularErrorDegrees =
                std::acos(cosine) * RADIANS_TO_DEGREES;
            result.valid = std::isfinite(result.angularErrorDegrees) &&
                           std::isfinite(result.primaryPointMirrorError) &&
                           std::isfinite(result.supportPointMirrorError);
            return result;
        }

        [[nodiscard]] float rotationDistanceDegrees(
            const RE::NiTransform& left,
            const RE::NiTransform& right)
        {
            return weapon_support_acquisition_math::rotationDistanceRadians(
                       left.rotate,
                       right.rotate) *
                   RADIANS_TO_DEGREES;
        }

        [[nodiscard]] ParityRotationResidual compareRotationResidual(
            const RE::NiTransform& expected,
            const RE::NiTransform& observed)
        {
            ParityRotationResidual result{};
            if (!isFiniteTransform(expected) ||
                !isFiniteTransform(observed)) {
                return result;
            }

            const RE::NiTransform expectedOrientation =
                left_firing_position_only_math::orientationOnly(expected);
            const RE::NiTransform observedOrientation =
                left_firing_position_only_math::orientationOnly(observed);
            const RE::NiTransform expectedToObserved =
                transform_math::composeTransforms(
                    transform_math::invertTransform(expectedOrientation),
                    observedOrientation);
            if (!isFiniteTransform(expectedToObserved)) {
                return result;
            }

            float quaternion[4]{};
            // NiTransform stores the engine basis by rows while ROCK applies
            // it as Transpose()*v. Convert that physical expected-local
            // rotation so the signed anatomical components describe the
            // correction from expected to observed, not its inverse.
            const auto physicalExpectedToObserved =
                transform_math::transposeRotation(
                    expectedToObserved.rotate);
            transform_math::niRowsToHavokQuaternion(
                physicalExpectedToObserved,
                quaternion);
            if (!std::isfinite(quaternion[0]) ||
                !std::isfinite(quaternion[1]) ||
                !std::isfinite(quaternion[2]) ||
                !std::isfinite(quaternion[3])) {
                return result;
            }
            if (quaternion[3] < 0.0f) {
                for (float& component : quaternion) {
                    component = -component;
                }
            }

            const float vectorLength = std::sqrt(
                quaternion[0] * quaternion[0] +
                quaternion[1] * quaternion[1] +
                quaternion[2] * quaternion[2]);
            const float angleDegrees =
                2.0f *
                std::atan2(
                    vectorLength,
                    std::clamp(quaternion[3], 0.0f, 1.0f)) *
                RADIANS_TO_DEGREES;
            if (!std::isfinite(vectorLength) ||
                !std::isfinite(angleDegrees)) {
                return result;
            }

            RE::NiPoint3 rawComponentsDegrees{};
            if (vectorLength > 0.000001f) {
                const float degreesPerAxisUnit =
                    angleDegrees / vectorLength;
                rawComponentsDegrees = RE::NiPoint3{
                    quaternion[0] * degreesPerAxisUnit,
                    quaternion[1] * degreesPerAxisUnit,
                    quaternion[2] * degreesPerAxisUnit,
                };
            }
            result.totalDegrees = angleDegrees;
            result.semanticComponentsDegrees = RE::NiPoint3{
                rawComponentsDegrees.x,
                -rawComponentsDegrees.y,
                -rawComponentsDegrees.z,
            };
            result.valid =
                finitePoint(result.semanticComponentsDegrees) &&
                std::isfinite(result.totalDegrees);
            return result;
        }

        [[nodiscard]] ParityAxes captureTransformAxes(
            const RE::NiTransform& transform)
        {
            ParityAxes result{};
            if (!isFiniteTransform(transform) ||
                std::abs(transform.scale) <= 0.0001f) {
                return result;
            }
            const RE::NiTransform orientation =
                left_firing_position_only_math::orientationOnly(transform);
            result.x = normalizeDirection(transform_math::localVectorToWorld(
                orientation,
                RE::NiPoint3{ 1.0f, 0.0f, 0.0f }));
            result.y = normalizeDirection(transform_math::localVectorToWorld(
                orientation,
                RE::NiPoint3{ 0.0f, 1.0f, 0.0f }));
            result.z = normalizeDirection(transform_math::localVectorToWorld(
                orientation,
                RE::NiPoint3{ 0.0f, 0.0f, 1.0f }));
            result.valid = finitePoint(result.x) && finitePoint(result.y) &&
                           finitePoint(result.z);
            return result;
        }

        [[nodiscard]] ParityHandFrame captureHandLocalFrame(
            const RE::NiTransform& handInWeapon,
            const bool isLeft)
        {
            ParityHandFrame result{};
            if (!isFiniteTransform(handInWeapon) ||
                std::abs(handInWeapon.scale) <= 0.0001f) {
                return result;
            }
            result.handInWeapon = handInWeapon;
            result.palmWeaponLocal =
                computeGrabLegacyPalmPivotAWorldFromHandBasis(
                    handInWeapon,
                    isLeft);
            result.fingerAxisWeaponLocal = transformHandspaceDirection(
                handInWeapon,
                RE::NiPoint3{ 1.0f, 0.0f, 0.0f },
                isLeft);
            result.palmAxisWeaponLocal = transformHandspaceDirection(
                handInWeapon,
                RE::NiPoint3{ 0.0f, -1.0f, 0.0f },
                isLeft);
            result.crossAxisWeaponLocal = transformHandspaceDirection(
                handInWeapon,
                RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
                isLeft);
            result.valid = finitePoint(result.palmWeaponLocal) &&
                           finitePoint(result.fingerAxisWeaponLocal) &&
                           finitePoint(result.palmAxisWeaponLocal) &&
                           finitePoint(result.crossAxisWeaponLocal);
            return result;
        }

        [[nodiscard]] ParityHandFrame captureHandWorldFrame(
            const RE::NiTransform& weaponWorld,
            const RE::NiTransform& handWorld,
            const bool isLeft)
        {
            if (!isFiniteTransform(weaponWorld) ||
                std::abs(weaponWorld.scale) <= 0.0001f ||
                !isFiniteTransform(handWorld)) {
                return {};
            }
            return captureHandLocalFrame(
                transform_math::composeTransforms(
                    transform_math::invertTransform(weaponWorld),
                    handWorld),
                isLeft);
        }

        [[nodiscard]] ParityAxes captureRelativeNodeAxes(
            const RE::NiNode* parent,
            const RE::NiNode* child)
        {
            if (!parent || !child || !isFiniteTransform(parent->world) ||
                std::abs(parent->world.scale) <= 0.0001f ||
                !isFiniteTransform(child->world)) {
                return {};
            }
            return captureTransformAxes(transform_math::composeTransforms(
                transform_math::invertTransform(parent->world),
                child->world));
        }

        [[nodiscard]] std::string formatParityPoint(
            const RE::NiPoint3& point)
        {
            return fmt::format(
                "{:.4f}/{:.4f}/{:.4f}",
                point.x,
                point.y,
                point.z);
        }

        [[nodiscard]] std::string formatParityAxes(
            const ParityAxes& axes)
        {
            return fmt::format(
                "X:{},Y:{},Z:{}",
                formatParityPoint(axes.x),
                formatParityPoint(axes.y),
                formatParityPoint(axes.z));
        }

        [[nodiscard]] std::string formatParityHandAxes(
            const ParityHandFrame& hand)
        {
            return fmt::format(
                "finger:{},palm:{},cross:{}",
                formatParityPoint(hand.fingerAxisWeaponLocal),
                formatParityPoint(hand.palmAxisWeaponLocal),
                formatParityPoint(hand.crossAxisWeaponLocal));
        }

        [[nodiscard]] std::string formatParityRotationResidual(
            const ParityRotationResidual& residual)
        {
            if (!residual.valid) {
                return "missing";
            }
            return fmt::format(
                "total:{:.4f},finger:{:.4f},palm:{:.4f},cross:{:.4f}",
                residual.totalDegrees,
                residual.semanticComponentsDegrees.x,
                residual.semanticComponentsDegrees.y,
                residual.semanticComponentsDegrees.z);
        }

        [[nodiscard]] float vectorLength(const RE::NiPoint3& value)
        {
            const float length = std::sqrt(
                value.x * value.x + value.y * value.y +
                value.z * value.z);
            return std::isfinite(length) ? length : -1.0f;
        }

        [[nodiscard]] float directionAngleDegrees(
            const RE::NiPoint3& first,
            const RE::NiPoint3& second)
        {
            const float firstLength = vectorLength(first);
            const float secondLength = vectorLength(second);
            if (firstLength <= 0.000001f ||
                secondLength <= 0.000001f) {
                return -1.0f;
            }
            const float cosine = std::clamp(
                (first.x * second.x + first.y * second.y +
                    first.z * second.z) /
                    (firstLength * secondLength),
                -1.0f,
                1.0f);
            const float angle = std::acos(cosine) * RADIANS_TO_DEGREES;
            return std::isfinite(angle) ? angle : -1.0f;
        }

        [[nodiscard]] float recoilDiagnosticStrength(
            const RE::NiTransform& delta)
        {
            if (!isFiniteTransform(delta)) {
                return 0.0f;
            }
            const auto identity =
                transform_math::makeIdentityTransform<RE::NiTransform>();
            const ParityRotationResidual rotation =
                compareRotationResidual(identity, delta);
            const float translation = vectorLength(delta.translate);
            if (translation < 0.0f || !rotation.valid) {
                return 0.0f;
            }
            return translation + rotation.totalDegrees * 0.1f;
        }

        [[nodiscard]] std::string formatRecoilDelta(
            const RE::NiTransform& delta)
        {
            const auto identity =
                transform_math::makeIdentityTransform<RE::NiTransform>();
            return fmt::format(
                "T:{} R:({})",
                formatParityPoint(delta.translate),
                formatParityRotationResidual(
                    compareRotationResidual(identity, delta)));
        }
    }

    void TwoHandedGrip::updateBilateralHandCalibrationTrace(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey)
    {
        auto& trace = _bilateralHandCalibrationTrace;
        if (!weaponNode || weaponNode != _activeWeaponNode ||
            currentWeaponGenerationKey == 0 ||
            currentEquippedWeaponOwnershipKey == 0) {
            trace = {};
            return;
        }

        const bool identityChanged =
            trace.weaponNodeIdentity != weaponNode ||
            trace.weaponGenerationKey != currentWeaponGenerationKey ||
            trace.equippedWeaponOwnershipKey !=
                currentEquippedWeaponOwnershipKey;
        if (identityChanged) {
            trace = BilateralHandCalibrationTraceState{
                .weaponNodeIdentity = weaponNode,
                .weaponGenerationKey = currentWeaponGenerationKey,
                .equippedWeaponOwnershipKey =
                    currentEquippedWeaponOwnershipKey,
                .traceSequence =
                    ++_bilateralHandCalibrationTraceSequence,
            };
        }

        const auto resetPendingStability = [&trace]() {
            trace.unownedStableFrames = {};
            trace.rightHoldStableFrames = 0;
        };
        if (_scopeMenuOpenThisFrame ||
            _scopeDriverFrameAuthorityActive ||
            !hasIndependentNaturalHandDriverPair() ||
            !isFiniteTransform(weaponNode->world) ||
            std::abs(weaponNode->world.scale) <= 0.0001f) {
            resetPendingStability();
            return;
        }

        const auto* playerNodes = f4vr::getPlayerNodes();
        RE::NiNode* const leftWand =
            playerNodes ? playerNodes->SecondaryWandNode : nullptr;
        RE::NiNode* const rightWand =
            playerNodes ? playerNodes->primaryWandNode : nullptr;
        const auto& leftDriver = _currentHandDriverFrames[0];
        const auto& rightDriver = _currentHandDriverFrames[1];
        if (!leftWand || !rightWand ||
            !leftDriver.valid || !rightDriver.valid ||
            !isFiniteTransform(leftWand->world) ||
            !isFiniteTransform(rightWand->world) ||
            !isFiniteTransform(leftDriver.world) ||
            !isFiniteTransform(rightDriver.world) ||
            std::abs(leftWand->world.scale) <= 0.0001f ||
            std::abs(rightWand->world.scale) <= 0.0001f ||
            std::abs(leftDriver.world.scale) <= 0.0001f ||
            std::abs(rightDriver.world.scale) <= 0.0001f) {
            resetPendingStability();
            return;
        }

        constexpr std::uint16_t kStableAuthorityFrames = 8;
        constexpr float kMaximumBoneToCarrierDistance = 30.0f;
        constexpr float kMaximumWeaponToHandDistance = 100.0f;
        const auto relationUsable = [](
                                        const RE::NiTransform& relation,
                                        const float maximumDistance) {
            if (!isFiniteTransform(relation) ||
                std::abs(relation.scale) <= 0.0001f) {
                return false;
            }
            const float distance = std::sqrt(
                relation.translate.x * relation.translate.x +
                relation.translate.y * relation.translate.y +
                relation.translate.z * relation.translate.z);
            return std::isfinite(distance) &&
                   distance <= maximumDistance;
        };
        const auto captureRelation = [&relationUsable](
                                         const RE::NiTransform& carrierWorld,
                                         const RE::NiTransform& handWorld,
                                         const float maximumDistance,
                                         RE::NiTransform& outRelation) {
            outRelation = transform_math::composeTransforms(
                transform_math::invertTransform(carrierWorld),
                handWorld);
            return relationUsable(outRelation, maximumDistance);
        };
        const auto advanceStableFrame = [kStableAuthorityFrames](
                                            std::uint16_t& frames) {
            if (frames < kStableAuthorityFrames) {
                ++frames;
            }
            return frames >= kStableAuthorityFrames;
        };

        /*
         * Preserve one native right-hand control sample before takeover. This
         * is not a new calibration authority: it only proves which rendered
         * right-hand hold the later candidate is being compared against.
         */
        const bool rightHoldEligible =
            !_firingHandIsLeft &&
            !hasVisualAuthorityForHand(false) &&
            !partGrip(true).active && !partGrip(false).active &&
            !_rightHandHoldingObjectForPose &&
            !_weaponCollisionHandPresentationFromPreviousFrame[1] &&
            hasRightFiringHandCanonicalFrame(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey) &&
            hasRightNativeWeaponAimFrame(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey);
        RE::NiTransform rightHoldWorld{};
        RE::NiTransform rightHoldInWand{};
        RE::NiTransform rightHoldInWeapon{};
        const bool rightHoldSampleValid =
            rightHoldEligible &&
            tryGetRootFlattenedHandBoneTransform(false, rightHoldWorld) &&
            isFiniteTransform(rightHoldWorld) &&
            captureRelation(
                rightWand->world,
                rightHoldWorld,
                kMaximumBoneToCarrierDistance,
                rightHoldInWand) &&
            captureRelation(
                weaponNode->world,
                rightHoldWorld,
                kMaximumWeaponToHandDistance,
                rightHoldInWeapon);
        if (!trace.rightHoldValid && rightHoldSampleValid) {
            if (advanceStableFrame(trace.rightHoldStableFrames)) {
                trace.rightHoldValid = true;
                const ParityRotationResidual canonicalResidual =
                    compareRotationResidual(
                        _rightFiringHandCanonicalWeaponLocal,
                        rightHoldInWeapon);
                const char* canonicalSource =
                    _rightFiringHandCanonicalSource ==
                            RightFiringCanonicalSource::AuthoredAnimation ?
                        "authored-animation" :
                        "native-carry";
                ROCK_LOG_INFO(
                    Weapon,
                    "AMBICALIBRATION RIGHT_HOLD seq={} generation={:016X} ownership={:016X} source={} canonicalToObservedRotation=({}) canonicalToObservedTranslation={:.4f}gu handInWandT=({})",
                    trace.traceSequence,
                    currentWeaponGenerationKey,
                    currentEquippedWeaponOwnershipKey,
                    canonicalSource,
                    formatParityRotationResidual(canonicalResidual),
                    pointDistance(
                        _rightFiringHandCanonicalWeaponLocal.translate,
                        rightHoldInWeapon.translate),
                    formatParityPoint(rightHoldInWand.translate));
                traceAmbidextrousPoseHierarchy(
                    weaponNode,
                    "right-control",
                    false);
            }
        } else if (!trace.rightHoldValid) {
            trace.rightHoldStableFrames = 0;
        }

        /*
         * Sample only the current offhand, after several frames with no ROCK
         * visual, collision, object, return, or grip authority. The two sides
         * are therefore learned in separate topology phases and neither one
         * is synthesized from the other.
         */
        for (const bool isLeft : { true, false }) {
            const std::size_t handIndex = isLeft ? 0u : 1u;
            if (trace.observedValid[handIndex]) {
                continue;
            }
            const bool holdingObject = isLeft ?
                _leftHandHoldingObjectForPose :
                _rightHandHoldingObjectForPose;
            const bool unownedEligible =
                isLeft != _firingHandIsLeft &&
                !hasVisualAuthorityForHand(isLeft) &&
                !partGrip(isLeft).active && !holdingObject &&
                !_weaponCollisionHandPresentationFromPreviousFrame[handIndex];
            RE::NiTransform handWorld{};
            RE::NiTransform boneInWand{};
            RE::NiTransform boneInDriver{};
            RE::NiNode* const wand = isLeft ? leftWand : rightWand;
            const auto& driver = isLeft ? leftDriver : rightDriver;
            const bool sampleValid =
                unownedEligible &&
                tryGetRootFlattenedHandBoneTransform(isLeft, handWorld) &&
                isFiniteTransform(handWorld) &&
                captureRelation(
                    wand->world,
                    handWorld,
                    kMaximumBoneToCarrierDistance,
                    boneInWand) &&
                captureRelation(
                    driver.world,
                    handWorld,
                    kMaximumBoneToCarrierDistance,
                    boneInDriver);
            if (!sampleValid) {
                trace.unownedStableFrames[handIndex] = 0;
                continue;
            }
            if (!advanceStableFrame(
                    trace.unownedStableFrames[handIndex])) {
                continue;
            }

            trace.observedBoneInWand[handIndex] = boneInWand;
            trace.observedValid[handIndex] = true;
            const RE::NiTransform& calibratedDriver = isLeft ?
                _leftNaturalBoneInDampedDriver :
                _rightNaturalBoneInDampedDriver;
            const RE::NiTransform calibratedHandWorld =
                transform_math::composeTransforms(
                    driver.world,
                    calibratedDriver);
            RE::NiTransform calibratedWand{};
            if (!captureRelation(
                    wand->world,
                    calibratedHandWorld,
                    kMaximumBoneToCarrierDistance,
                    calibratedWand)) {
                trace.observedValid[handIndex] = false;
                trace.unownedStableFrames[handIndex] = 0;
                continue;
            }
            const ParityRotationResidual wandResidual =
                compareRotationResidual(calibratedWand, boneInWand);
            const ParityRotationResidual driverResidual =
                compareRotationResidual(calibratedDriver, boneInDriver);
            ROCK_LOG_INFO(
                Weapon,
                "AMBICALIBRATION UNOWNED seq={} hand={} calibratedWandToObservedRotation=({}) calibratedWandToObservedTranslation={:.4f}gu calibratedDriverToObservedRotation=({}) calibratedDriverToObservedTranslation={:.4f}gu observedWandT=({}) observedDriverT=({})",
                trace.traceSequence,
                isLeft ? "left" : "right",
                formatParityRotationResidual(wandResidual),
                pointDistance(
                    calibratedWand.translate,
                    boneInWand.translate),
                formatParityRotationResidual(driverResidual),
                pointDistance(
                    calibratedDriver.translate,
                    boneInDriver.translate),
                formatParityPoint(boneInWand.translate),
                formatParityPoint(boneInDriver.translate));
        }

        const bool bilateralObserved =
            trace.observedValid[0] && trace.observedValid[1];
        if (!bilateralObserved) {
            return;
        }

        if (!trace.bilateralLogged) {
            const RE::NiTransform predictedRightFromLeft =
                left_firing_position_only_math::mirrorOppositeHandFrame(
                    trace.observedBoneInWand[0]);
            const RE::NiTransform predictedLeftFromRight =
                left_firing_position_only_math::mirrorOppositeHandFrame(
                    trace.observedBoneInWand[1]);
            const ParityRotationResidual leftToRightResidual =
                compareRotationResidual(
                    predictedRightFromLeft,
                    trace.observedBoneInWand[1]);
            const ParityRotationResidual rightToLeftResidual =
                compareRotationResidual(
                    predictedLeftFromRight,
                    trace.observedBoneInWand[0]);
            ROCK_LOG_INFO(
                Weapon,
                "AMBICALIBRATION BILATERAL seq={} syntheticMirrorToPhysicalRotation=(rightFromLeft:({}),leftFromRight:({})) syntheticMirrorToPhysicalTranslation=(rightFromLeft:{:.4f}gu,leftFromRight:{:.4f}gu) convention=(finger:+X,palm:-Y,cross:authored+Z/raw-Z)",
                trace.traceSequence,
                formatParityRotationResidual(leftToRightResidual),
                formatParityRotationResidual(rightToLeftResidual),
                pointDistance(
                    predictedRightFromLeft.translate,
                    trace.observedBoneInWand[1].translate),
                pointDistance(
                    predictedLeftFromRight.translate,
                    trace.observedBoneInWand[0].translate));
            trace.bilateralLogged = true;
        }

        if (_firingHandIsLeft &&
            !trace.leftFiringCandidateLogged &&
            _hasFiringHandWeaponLocal &&
            hasRightFiringHandCanonicalFrame(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey)) {
            const RE::NiTransform identity =
                transform_math::makeIdentityTransform<RE::NiTransform>();
            RE::NiTransform authoredMirrorCandidate{};
            if (tryBuildMirroredLeftFiringHandWeaponLocalImpl(
                    _rightFiringHandCanonicalWeaponLocal,
                    _rightFiringGripCanonicalWeaponLocal,
                    identity,
                    identity,
                    authoredMirrorCandidate,
                    false,
                    false)) {
                const ParityRotationResidual candidateResidual =
                    compareRotationResidual(
                        _primaryHandWeaponLocal,
                        authoredMirrorCandidate);
                const RE::NiPoint3 currentPalm =
                    computeGrabLegacyPalmPivotAWorldFromHandBasis(
                        _primaryHandWeaponLocal,
                        true);
                const RE::NiPoint3 authoredMirrorPalm =
                    computeGrabLegacyPalmPivotAWorldFromHandBasis(
                        authoredMirrorCandidate,
                        true);
                ROCK_LOG_INFO(
                    Weapon,
                    "AMBICALIBRATION LEFT_FIRING_CANDIDATE seq={} authoredMirrorResidualRotation=({}) authoredMirrorResidualTranslation={:.4f}gu palmSeatDelta={:.4f}gu rightControl={} currentT=({}) authoredMirrorT=({})",
                    trace.traceSequence,
                    formatParityRotationResidual(candidateResidual),
                    pointDistance(
                        _primaryHandWeaponLocal.translate,
                        authoredMirrorCandidate.translate),
                    pointDistance(currentPalm, authoredMirrorPalm),
                    trace.rightHoldValid ? "ready" : "missing",
                    formatParityPoint(
                        _primaryHandWeaponLocal.translate),
                    formatParityPoint(
                        authoredMirrorCandidate.translate));
                trace.leftFiringCandidateLogged = true;
            }
        }

        const auto& supportCandidate = _authoredSupportGripCandidate;
        if (_firingHandIsLeft &&
            !trace.rightSupportCandidateLogged &&
            supportCandidate.valid &&
            supportCandidate.rightMirrorValid &&
            supportCandidate.weaponNode == weaponNode &&
            supportCandidate.weaponGenerationKey ==
                currentWeaponGenerationKey) {
            RE::NiTransform authoredMirrorRightSupport{};
            if (tryBuildMirroredRightSupportHandWeaponLocalImpl(
                    supportCandidate.leftHandWeaponLocal,
                    authoredMirrorRightSupport)) {
                const ParityRotationResidual candidateResidual =
                    compareRotationResidual(
                        supportCandidate.rightHandWeaponLocal,
                        authoredMirrorRightSupport);
                const RE::NiPoint3 currentPalm =
                    computeGrabLegacyPalmPivotAWorldFromHandBasis(
                        supportCandidate.rightHandWeaponLocal,
                        false);
                const RE::NiPoint3 authoredMirrorPalm =
                    computeGrabLegacyPalmPivotAWorldFromHandBasis(
                        authoredMirrorRightSupport,
                        false);
                ROCK_LOG_INFO(
                    Weapon,
                    "AMBICALIBRATION RIGHT_SUPPORT_CANDIDATE seq={} capture={} authoredMirrorResidualRotation=({}) authoredMirrorResidualTranslation={:.4f}gu palmSeatDelta={:.4f}gu currentT=({}) authoredMirrorT=({})",
                    trace.traceSequence,
                    supportCandidate.captureSequence,
                    formatParityRotationResidual(candidateResidual),
                    pointDistance(
                        supportCandidate.rightHandWeaponLocal.translate,
                        authoredMirrorRightSupport.translate),
                    pointDistance(currentPalm, authoredMirrorPalm),
                    formatParityPoint(
                        supportCandidate.rightHandWeaponLocal.translate),
                    formatParityPoint(
                        authoredMirrorRightSupport.translate));
                trace.rightSupportCandidateLogged = true;
            }
        }
    }

    void TwoHandedGrip::traceAmbidextrousPoseHierarchy(
        RE::NiNode* weaponNode,
        const char* phase,
        const bool supportedTopology)
    {
        if (!weaponNode || !phase || !_ambidextrousPoseHierarchyReader ||
            weaponNode != _activeWeaponNode ||
            _activeWeaponGenerationKey == 0 ||
            !isFiniteTransform(weaponNode->world) ||
            std::abs(weaponNode->world.scale) <= 0.0001f) {
            return;
        }

        auto& trace = _ambidextrousPoseHierarchyTrace;
        const bool identityChanged =
            trace.weaponNodeIdentity != weaponNode ||
            trace.weaponGenerationKey != _activeWeaponGenerationKey ||
            trace.canonicalCaptureSequence !=
                _rightFiringHandCanonicalCaptureSequence;
        if (identityChanged) {
            trace = AmbidextrousPoseHierarchyTraceState{
                .weaponNodeIdentity = weaponNode,
                .weaponGenerationKey = _activeWeaponGenerationKey,
                .canonicalCaptureSequence =
                    _rightFiringHandCanonicalCaptureSequence,
                .traceSequence =
                    ++_ambidextrousPoseHierarchyTraceSequence,
            };
            _ambidextrousPoseHierarchyReader->resetCache();
        }

        const std::size_t topologyIndex = supportedTopology ? 1u : 0u;
        const std::size_t modeIndex = _firingHandIsLeft ? 1u : 0u;
        auto& topology = trace.topologies[topologyIndex];
        if (topology.modes[modeIndex].valid) {
            return;
        }

        DirectSkeletonBoneSnapshot snapshot{};
        if (!_ambidextrousPoseHierarchyReader->capture(
                skeleton_bone_debug_math::DebugSkeletonBoneMode::
                    CoreBodyAndFingers,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::
                    GameRootFlattenedBoneTree,
                snapshot)) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                1000,
                "AMBIPOSE capture unavailable phase={} topology={} firing={}",
                phase,
                supportedTopology ? "supported" : "one-hand",
                _firingHandIsLeft ? "left" : "right");
            return;
        }

        const auto findBone = [&snapshot](const std::string_view name)
            -> const DirectSkeletonBoneEntry* {
            for (const auto& bone : snapshot.bones) {
                if (bone.name == name) {
                    return &bone;
                }
            }
            return nullptr;
        };
        const auto findBoneByTreeIndex =
            [&snapshot](const int treeIndex)
            -> const DirectSkeletonBoneEntry* {
            if (treeIndex < 0) {
                return nullptr;
            }
            for (const auto& bone : snapshot.bones) {
                if (bone.treeIndex == treeIndex) {
                    return &bone;
                }
            }
            return nullptr;
        };

        PoseHierarchyModeTrace captured{};
        captured.equippedWeaponOwnershipKey =
            _activeEquippedWeaponOwnershipKey;
        captured.captureSequence =
            ++_ambidextrousPoseHierarchyTraceSequence;
        const RE::NiTransform weaponInverse =
            transform_math::invertTransform(weaponNode->world);

        for (const bool isLeft : { true, false }) {
            auto& hand = captured.hands[isLeft ? 0u : 1u];
            std::size_t resolvedBoneCount = 0;
            for (std::size_t boneIndex = 0;
                 boneIndex < kPoseHierarchyBoneNames.size();
                 ++boneIndex) {
                const auto& names = kPoseHierarchyBoneNames[boneIndex];
                const auto* bone = findBone(isLeft ? names.left : names.right);
                if (!bone || !isFiniteTransform(bone->world) ||
                    std::abs(bone->world.scale) <= 0.0001f) {
                    continue;
                }

                auto& outBone = hand.bones[boneIndex];
                outBone.weaponLocal = transform_math::composeTransforms(
                    weaponInverse,
                    bone->world);
                outBone.weaponLocalValid =
                    isFiniteTransform(outBone.weaponLocal) &&
                    std::abs(outBone.weaponLocal.scale) > 0.0001f;

                const auto* parent =
                    findBoneByTreeIndex(bone->parentTreeIndex);
                if (parent && isFiniteTransform(parent->world) &&
                    std::abs(parent->world.scale) > 0.0001f) {
                    outBone.parentLocal =
                        transform_math::composeTransforms(
                            transform_math::invertTransform(parent->world),
                            bone->world);
                    outBone.parentLocalValid =
                        isFiniteTransform(outBone.parentLocal) &&
                        std::abs(outBone.parentLocal.scale) > 0.0001f;
                }
                if (outBone.weaponLocalValid) {
                    ++resolvedBoneCount;
                }
            }

            const bool isFiringRole = isLeft == _firingHandIsLeft;
            if (isFiringRole) {
                hand.publishedFingerLocals = isLeft ?
                    _leftFiringFingerLocalTransforms :
                    _rightFiringFingerLocalTransforms;
                hand.publishedFingerMask = isLeft ?
                    _leftFiringFingerLocalTransformMask :
                    _rightFiringFingerLocalTransformMask;
            } else {
                const auto& support = partGrip(isLeft);
                if (support.active && support.hasFingerLocalTransforms) {
                    hand.publishedFingerLocals =
                        support.fingerLocalTransforms;
                    hand.publishedFingerMask =
                        support.fingerLocalTransformMask;
                }
            }

            constexpr std::size_t kHandBoneIndex = 7;
            hand.valid =
                hand.bones[kHandBoneIndex].weaponLocalValid;
            ROCK_LOG_INFO(
                Weapon,
                "AMBIPOSE CAPTURE seq={} sample={} phase={} topology={} firing={} hand={} role={} bones={}/{} fingerMask=0x{:04X} source={} powerArmor={}",
                trace.traceSequence,
                captured.captureSequence,
                phase,
                supportedTopology ? "supported" : "one-hand",
                _firingHandIsLeft ? "left" : "right",
                isLeft ? "left" : "right",
                isFiringRole ? "firing" : "support",
                resolvedBoneCount,
                kPoseHierarchyBoneNames.size(),
                hand.publishedFingerMask,
                skeleton_bone_debug_math::snapshotSourceName(
                    snapshot.source),
                snapshot.inPowerArmor ? "yes" : "no");
        }

        const std::size_t firingHandIndex =
            _firingHandIsLeft ? 0u : 1u;
        const std::size_t supportHandIndex =
            _firingHandIsLeft ? 1u : 0u;
        captured.valid =
            captured.hands[firingHandIndex].valid &&
            (!supportedTopology ||
                captured.hands[supportHandIndex].valid);
        if (!captured.valid) {
            return;
        }
        topology.modes[modeIndex] = captured;

        const auto& rightMode = topology.modes[0];
        const auto& leftMode = topology.modes[1];
        if (!rightMode.valid || !leftMode.valid ||
            topology.comparisonLogged) {
            return;
        }

        const auto compareRole = [this, &trace, &rightMode, &leftMode,
                                     supportedTopology](
                                     const char* role,
                                     const bool sourceIsLeft,
                                     const bool targetIsLeft) {
            const auto& source =
                rightMode.hands[sourceIsLeft ? 0u : 1u];
            const auto& target =
                leftMode.hands[targetIsLeft ? 0u : 1u];

            frik_visual_authority::FingerLocalTransformOverride
                sourceFingerLocals{};
            sourceFingerLocals.enabledMask =
                authored_weapon_grip_library::kCompleteFiringFingerMask;
            bool sourceFingerLocalsReady = true;
            for (const auto& names : kPoseHierarchyBoneNames) {
                if (names.fingerIndex < 0) {
                    continue;
                }
                const auto& sourceBone =
                    source.bones[static_cast<std::size_t>(
                        names.fingerIndex + 8)];
                if (!sourceBone.parentLocalValid) {
                    sourceFingerLocalsReady = false;
                    break;
                }
                sourceFingerLocals.localTransforms[
                    static_cast<std::size_t>(names.fingerIndex)] =
                    sourceBone.parentLocal;
            }
            frik_visual_authority::FingerLocalTransformOverride
                mirroredFingerLocals{};
            const bool exactFingerMirrorReady =
                sourceFingerLocalsReady &&
                frik_visual_authority::mirrorFingerLocalTransforms(
                    sourceIsLeft ?
                        frik_visual_authority::Hand::Left :
                        frik_visual_authority::Hand::Right,
                    sourceFingerLocals,
                    mirroredFingerLocals) &&
                mirroredFingerLocals.enabledMask ==
                    authored_weapon_grip_library::
                        kCompleteFiringFingerMask;

            float maximumArmTranslation = 0.0f;
            float maximumArmRotation = 0.0f;
            float maximumFingerMirrorTranslation = 0.0f;
            float maximumFingerMirrorRotation = 0.0f;
            float maximumSourcePublishedRotation = 0.0f;
            float maximumTargetPublishedRotation = 0.0f;
            float handTranslation = -1.0f;
            float handRotation = -1.0f;
            std::size_t comparedBones = 0;
            std::size_t comparedFingers = 0;

            for (std::size_t boneIndex = 0;
                 boneIndex < kPoseHierarchyBoneNames.size();
                 ++boneIndex) {
                const auto& names = kPoseHierarchyBoneNames[boneIndex];
                const auto& sourceBone = source.bones[boneIndex];
                const auto& targetBone = target.bones[boneIndex];
                if (!sourceBone.weaponLocalValid ||
                    !targetBone.weaponLocalValid) {
                    ROCK_LOG_INFO(
                        Weapon,
                        "AMBIPOSE BONE seq={} topology={} role={} bone={} source={} target={} status=missing sourceValid={} targetValid={}",
                        trace.traceSequence,
                        supportedTopology ? "supported" : "one-hand",
                        role,
                        names.label,
                        sourceIsLeft ? "left" : "right",
                        targetIsLeft ? "left" : "right",
                        sourceBone.weaponLocalValid ? "yes" : "no",
                        targetBone.weaponLocalValid ? "yes" : "no");
                    continue;
                }

                const RE::NiTransform syntheticMirror =
                    left_firing_position_only_math::
                        mirrorOppositeHandFrame(
                            sourceBone.weaponLocal);
                const float syntheticTranslation = pointDistance(
                    syntheticMirror.translate,
                    targetBone.weaponLocal.translate);
                const ParityRotationResidual syntheticRotation =
                    compareRotationResidual(
                        syntheticMirror,
                        targetBone.weaponLocal);
                ++comparedBones;
                if (boneIndex <= 7) {
                    maximumArmTranslation = (std::max)(
                        maximumArmTranslation,
                        syntheticTranslation);
                    if (syntheticRotation.valid) {
                        maximumArmRotation = (std::max)(
                            maximumArmRotation,
                            syntheticRotation.totalDegrees);
                    }
                }
                if (boneIndex == 7) {
                    handTranslation = syntheticTranslation;
                    handRotation = syntheticRotation.valid ?
                        syntheticRotation.totalDegrees :
                        -1.0f;
                }

                std::string exactFingerMirror = "not-finger";
                std::string sourceLiveToPublished = "not-published";
                std::string targetLiveToPublished = "not-published";
                if (names.fingerIndex >= 0) {
                    const std::size_t fingerIndex =
                        static_cast<std::size_t>(names.fingerIndex);
                    if (exactFingerMirrorReady &&
                        targetBone.parentLocalValid) {
                        const auto& expectedLocal =
                            mirroredFingerLocals.localTransforms[
                                fingerIndex];
                        const float translation = pointDistance(
                            expectedLocal.translate,
                            targetBone.parentLocal.translate);
                        const auto rotation = compareRotationResidual(
                            expectedLocal,
                            targetBone.parentLocal);
                        exactFingerMirror = fmt::format(
                            "T:{:.4f}gu,R:({})",
                            translation,
                            formatParityRotationResidual(rotation));
                        maximumFingerMirrorTranslation = (std::max)(
                            maximumFingerMirrorTranslation,
                            translation);
                        if (rotation.valid) {
                            maximumFingerMirrorRotation = (std::max)(
                                maximumFingerMirrorRotation,
                                rotation.totalDegrees);
                        }
                        ++comparedFingers;
                    }

                    const std::uint16_t fingerBit =
                        static_cast<std::uint16_t>(1u << fingerIndex);
                    if (sourceBone.parentLocalValid &&
                        (source.publishedFingerMask & fingerBit) != 0) {
                        const auto residual = compareRotationResidual(
                            source.publishedFingerLocals[fingerIndex],
                            sourceBone.parentLocal);
                        sourceLiveToPublished = fmt::format(
                            "T:{:.4f}gu,R:({})",
                            pointDistance(
                                source.publishedFingerLocals[fingerIndex]
                                    .translate,
                                sourceBone.parentLocal.translate),
                            formatParityRotationResidual(residual));
                        if (residual.valid) {
                            maximumSourcePublishedRotation = (std::max)(
                                maximumSourcePublishedRotation,
                                residual.totalDegrees);
                        }
                    }
                    if (targetBone.parentLocalValid &&
                        (target.publishedFingerMask & fingerBit) != 0) {
                        const auto residual = compareRotationResidual(
                            target.publishedFingerLocals[fingerIndex],
                            targetBone.parentLocal);
                        targetLiveToPublished = fmt::format(
                            "T:{:.4f}gu,R:({})",
                            pointDistance(
                                target.publishedFingerLocals[fingerIndex]
                                    .translate,
                                targetBone.parentLocal.translate),
                            formatParityRotationResidual(residual));
                        if (residual.valid) {
                            maximumTargetPublishedRotation = (std::max)(
                                maximumTargetPublishedRotation,
                                residual.totalDegrees);
                        }
                    }
                }

                const ParityRotationResidual directParentRotation =
                    sourceBone.parentLocalValid &&
                            targetBone.parentLocalValid ?
                        compareRotationResidual(
                            sourceBone.parentLocal,
                            targetBone.parentLocal) :
                        ParityRotationResidual{};
                ROCK_LOG_INFO(
                    Weapon,
                    "AMBIPOSE BONE seq={} topology={} role={} bone={} source={} target={} syntheticWeaponMirror=(T:{:.4f}gu,R:({})) sourceWeaponT=({}) targetWeaponT=({}) parentLocal=(sourceT:{},targetT:{},directR:({})) exactFingerMirror=({}) liveToPublished=(source:{},target:{})",
                    trace.traceSequence,
                    supportedTopology ? "supported" : "one-hand",
                    role,
                    names.label,
                    sourceIsLeft ? "left" : "right",
                    targetIsLeft ? "left" : "right",
                    syntheticTranslation,
                    formatParityRotationResidual(syntheticRotation),
                    formatParityPoint(sourceBone.weaponLocal.translate),
                    formatParityPoint(targetBone.weaponLocal.translate),
                    formatParityPoint(sourceBone.parentLocal.translate),
                    formatParityPoint(targetBone.parentLocal.translate),
                    formatParityRotationResidual(directParentRotation),
                    exactFingerMirror,
                    sourceLiveToPublished,
                    targetLiveToPublished);
            }

            ROCK_LOG_INFO(
                Weapon,
                "AMBIPOSE SUMMARY seq={} topology={} role={} source={} target={} compared=(bones:{},fingers:{}) syntheticMirrorMax=(armT:{:.4f}gu,armR:{:.4f}deg,handT:{:.4f}gu,handR:{:.4f}deg) exactFingerMirrorMax=(T:{:.4f}gu,R:{:.4f}deg) liveToPublishedMaxRotation=(source:{:.4f}deg,target:{:.4f}deg)",
                trace.traceSequence,
                supportedTopology ? "supported" : "one-hand",
                role,
                sourceIsLeft ? "left" : "right",
                targetIsLeft ? "left" : "right",
                comparedBones,
                comparedFingers,
                maximumArmTranslation,
                maximumArmRotation,
                handTranslation,
                handRotation,
                maximumFingerMirrorTranslation,
                maximumFingerMirrorRotation,
                maximumSourcePublishedRotation,
                maximumTargetPublishedRotation);
        };

        compareRole("firing", false, true);
        if (supportedTopology) {
            compareRole("support", true, false);
        }
        topology.comparisonLogged = true;
    }

    void TwoHandedGrip::traceWeaponRecoilSample(
        const RE::NiTransform& nativeKickLocal,
        const RE::NiTransform& controlledKickLocal,
        const bool responseAccepted,
        const bool visualOnlySupportRecoilAssist) noexcept
    {
        try {
            auto& trace = _weaponRecoilDiagnosticTrace;
            trace.currentSampleLogged = false;
            if (!_activeWeaponNode || _activeWeaponGenerationKey == 0 ||
                !isFiniteTransform(_activeWeaponNode->world) ||
                std::abs(_activeWeaponNode->world.scale) <= 0.0001f ||
                !isFiniteTransform(nativeKickLocal) ||
                !isFiniteTransform(controlledKickLocal)) {
                return;
            }

            constexpr float kMinimumSignificantStrength = 0.0001f;
            if (recoilDiagnosticStrength(controlledKickLocal) <=
                kMinimumSignificantStrength) {
                return;
            }

            const bool identityChanged =
                trace.weaponNodeIdentity != _activeWeaponNode ||
                trace.weaponGenerationKey !=
                    _activeWeaponGenerationKey;
            if (identityChanged) {
                trace = WeaponRecoilDiagnosticTraceState{
                    .weaponNodeIdentity = _activeWeaponNode,
                    .weaponGenerationKey =
                        _activeWeaponGenerationKey,
                    .traceSequence =
                        ++_weaponRecoilDiagnosticTraceSequence,
                };
            }

            const auto* playerNodes = f4vr::getPlayerNodes();
            const auto* kickbackNode = playerNodes ?
                playerNodes->primaryWeaponKickbackRecoilNode :
                nullptr;
            const auto* kickParent = kickbackNode ?
                kickbackNode->parent :
                nullptr;
            const auto* leftHandedMode =
                f4vr::getIniSetting("bLeftHandedMode:VR");
            if (!playerNodes || !kickParent || !leftHandedMode ||
                !isFiniteTransform(kickParent->world) ||
                std::abs(kickParent->world.scale) <= 0.0001f ||
                !playerNodes->primaryWandNode ||
                !playerNodes->SecondaryWandNode ||
                !isFiniteTransform(
                    playerNodes->primaryWandNode->world) ||
                !isFiniteTransform(
                    playerNodes->SecondaryWandNode->world) ||
                std::abs(
                    playerNodes->primaryWandNode->world.scale) <=
                    0.0001f ||
                std::abs(
                    playerNodes->SecondaryWandNode->world.scale) <=
                    0.0001f) {
                return;
            }

            const bool nativePrimaryIsLeft =
                leftHandedMode->GetBinary();
            const bool targetIsNativeOffhand =
                _firingHandIsLeft != nativePrimaryIsLeft;
            const RE::NiTransform& primaryWandWorld =
                playerNodes->primaryWandNode->world;
            const RE::NiTransform& offhandWandWorld =
                playerNodes->SecondaryWandNode->world;
            RE::NiTransform kickParentInPrimaryWand{};
            if (!weapon_recoil_authority_math::
                    tryResolveFrameLocalTransform(
                        primaryWandWorld,
                        kickParent->world,
                        kickParentInPrimaryWand)) {
                return;
            }
            RE::NiTransform resolvedKickParentWorld{};
            RE::NiTransform resolvedKickLocal{};
            if (!weapon_recoil_authority_math::tryResolveKickFrame(
                    controlledKickLocal,
                    kickParent->world,
                    primaryWandWorld,
                    offhandWandWorld,
                    targetIsNativeOffhand,
                    resolvedKickParentWorld,
                    resolvedKickLocal)) {
                return;
            }
            const RE::NiTransform& targetWandWorld =
                targetIsNativeOffhand ?
                    offhandWandWorld :
                    primaryWandWorld;
            RE::NiTransform resolvedKickParentInTargetWand{};
            if (!weapon_recoil_authority_math::
                    tryResolveFrameLocalTransform(
                        targetWandWorld,
                        resolvedKickParentWorld,
                        resolvedKickParentInTargetWand)) {
                return;
            }

            RE::NiTransform kickedWeaponWorld{};
            RE::NiTransform weaponLocalDelta{};
            if (!weapon_recoil_authority_math::
                    tryApplyKickToWorldTarget(
                        resolvedKickParentWorld,
                        resolvedKickLocal,
                        _activeWeaponNode->world,
                        kickedWeaponWorld) ||
                !weapon_recoil_authority_math::
                    tryResolveFrameLocalTransform(
                        _activeWeaponNode->world,
                        kickedWeaponWorld,
                        weaponLocalDelta)) {
                return;
            }

            trace.currentSampleSequence =
                _weaponRecoilSampleSequence;
            trace.currentKickParentWorld =
                resolvedKickParentWorld;
            trace.currentKickLocal = resolvedKickLocal;
            trace.currentWeaponLocalDelta = weaponLocalDelta;

            constexpr std::uint16_t kMaximumSamplesPerSide = 64;
            const float strength =
                recoilDiagnosticStrength(weaponLocalDelta);
            const std::size_t sideIndex =
                _firingHandIsLeft ? 1u : 0u;
            if (trace.lastControlledKickValid[sideIndex]) {
                const auto repeatedRotation = compareRotationResidual(
                    trace.lastControlledKickLocals[sideIndex],
                    controlledKickLocal);
                if (pointDistance(
                        trace.lastControlledKickLocals[sideIndex]
                            .translate,
                        controlledKickLocal.translate) <= 0.00001f &&
                    repeatedRotation.valid &&
                    repeatedRotation.totalDegrees <= 0.001f) {
                    return;
                }
            }
            if (!std::isfinite(strength) ||
                strength <= kMinimumSignificantStrength ||
                trace.emittedSamples[sideIndex] >=
                    kMaximumSamplesPerSide) {
                return;
            }

            trace.lastControlledKickLocals[sideIndex] =
                controlledKickLocal;
            trace.lastControlledKickValid[sideIndex] = true;
            ++trace.emittedSamples[sideIndex];
            trace.currentSampleLogged = true;
            const auto& supportGrip = partGrip(!_firingHandIsLeft);
            const char* rockRoute = "none";
            if (_firingHandIsLeft &&
                _weaponNodeOwnershipBlockEngaged &&
                isManualOwnershipActive()) {
                rockRoute =
                    _state == TwoHandedState::Gripping &&
                            _authorityMode ==
                                weapon_support_authority_policy::
                                    WeaponSupportAuthorityMode::
                                        FullTwoHandedSolver ?
                        "primary-solver" :
                        "one-hand-pair";
            }
            ROCK_LOG_INFO(
                Weapon,
                "AMBIRECOIL SAMPLE seq={} sample={} firing={} state={} support=(active:{},authority:{}) response=(accepted:{},delivery:{},visualAssist:{}) targetNativeOffhand={} nativeLocal=({}) controlledLocal=({}) resolvedLocal=({}) kickParentInPrimaryWand=({}) kickParentInTargetWand=({}) weaponLocalDelta=({}) rockRoute={}",
                trace.traceSequence,
                trace.currentSampleSequence,
                _firingHandIsLeft ? "left" : "right",
                static_cast<std::uint32_t>(_state),
                supportGrip.active ? "yes" : "no",
                _authorityMode ==
                            weapon_support_authority_policy::
                                WeaponSupportAuthorityMode::
                                    FullTwoHandedSolver ?
                    "full" :
                    "visual-only",
                responseAccepted ? "yes" : "no",
                responseAccepted ?
                    (_firingHandIsLeft ?
                         "suppressed-rock-owned" :
                         "primary-direct") :
                    "native",
                visualOnlySupportRecoilAssist ? "yes" : "no",
                targetIsNativeOffhand ? "yes" : "no",
                formatRecoilDelta(nativeKickLocal),
                formatRecoilDelta(controlledKickLocal),
                formatRecoilDelta(resolvedKickLocal),
                formatParityPoint(
                    kickParentInPrimaryWand.translate),
                formatParityPoint(
                    resolvedKickParentInTargetWand.translate),
                formatRecoilDelta(weaponLocalDelta),
                rockRoute);

            auto& peak = trace.peaks[sideIndex];
            if (!peak.valid || strength > peak.strength * 1.01f) {
                peak = WeaponRecoilPeakTrace{
                    .weaponLocalDelta = weaponLocalDelta,
                    .strength = strength,
                    .valid = true,
                };
                ++trace.peakRevision;
                ROCK_LOG_INFO(
                    Weapon,
                    "AMBIRECOIL PEAK seq={} revision={} firing={} strength={:.6f} weaponLocalDelta=({})",
                    trace.traceSequence,
                    trace.peakRevision,
                    _firingHandIsLeft ? "left" : "right",
                    strength,
                    formatRecoilDelta(weaponLocalDelta));
            }

            if (!trace.peaks[0].valid || !trace.peaks[1].valid ||
                trace.comparedPeakRevision == trace.peakRevision) {
                return;
            }
            trace.comparedPeakRevision = trace.peakRevision;
            const auto& rightPeak = trace.peaks[0].weaponLocalDelta;
            const auto& leftPeak = trace.peaks[1].weaponLocalDelta;
            const RE::NiTransform sagittalMirroredRight =
                weapon_recoil_authority_math::
                    mirrorLocalAcrossSagittal(rightPeak);
            const auto identity =
                transform_math::makeIdentityTransform<RE::NiTransform>();
            const auto rightRotation =
                compareRotationResidual(identity, rightPeak);
            const auto leftRotation =
                compareRotationResidual(identity, leftPeak);
            const auto mirroredRightRotation =
                compareRotationResidual(
                    identity,
                    sagittalMirroredRight);
            ROCK_LOG_INFO(
                Weapon,
                "AMBIRECOIL PARITY seq={} revision={} rightPeak=({}) leftPeak=({}) direct=(translationAxis:{:.4f}deg,rotationAxis:{:.4f}deg,rotationResidual:({})) sagittalAlternative=(translationAxis:{:.4f}deg,rotationAxis:{:.4f}deg,rotationResidual:({}))",
                trace.traceSequence,
                trace.peakRevision,
                formatRecoilDelta(rightPeak),
                formatRecoilDelta(leftPeak),
                directionAngleDegrees(
                    rightPeak.translate,
                    leftPeak.translate),
                rightRotation.valid && leftRotation.valid ?
                    directionAngleDegrees(
                        rightRotation.semanticComponentsDegrees,
                        leftRotation.semanticComponentsDegrees) :
                    -1.0f,
                formatParityRotationResidual(
                    compareRotationResidual(rightPeak, leftPeak)),
                directionAngleDegrees(
                    sagittalMirroredRight.translate,
                    leftPeak.translate),
                mirroredRightRotation.valid && leftRotation.valid ?
                    directionAngleDegrees(
                        mirroredRightRotation.semanticComponentsDegrees,
                        leftRotation.semanticComponentsDegrees) :
                    -1.0f,
                formatParityRotationResidual(
                    compareRotationResidual(
                        sagittalMirroredRight,
                        leftPeak)));
        } catch (...) {
            _weaponRecoilDiagnosticTrace.currentSampleLogged = false;
        }
    }

    void TwoHandedGrip::traceWeaponRecoilConsumer(
        const char* consumer,
        const RE::NiTransform* inputWorld,
        const RE::NiTransform* outputWorld) noexcept
    {
        try {
            const auto& trace = _weaponRecoilDiagnosticTrace;
            if (!consumer || !trace.currentSampleLogged ||
                trace.currentSampleSequence !=
                    _weaponRecoilSampleSequence) {
                return;
            }
            if (!inputWorld || !outputWorld ||
                !isFiniteTransform(*inputWorld) ||
                !isFiniteTransform(*outputWorld) ||
                std::abs(inputWorld->scale) <= 0.0001f ||
                std::abs(outputWorld->scale) <= 0.0001f) {
                ROCK_LOG_INFO(
                    Weapon,
                    "AMBIRECOIL CONSUMER seq={} sample={} consumer={} routeOnly=yes",
                    trace.traceSequence,
                    trace.currentSampleSequence,
                    consumer);
                return;
            }

            RE::NiTransform expectedOutput = *inputWorld;
            const bool alreadyIntegratedSupportedHand =
                std::strcmp(consumer, "frik-primary-hand") == 0 &&
                _leftFiringWeaponRecoilSupportConstrainedThisUpdate;
            if (!alreadyIntegratedSupportedHand &&
                !weapon_recoil_authority_math::
                    tryApplyKickToWorldTarget(
                        trace.currentKickParentWorld,
                        trace.currentKickLocal,
                        *inputWorld,
                        expectedOutput)) {
                return;
            }
            RE::NiTransform observedLocalDelta{};
            if (!weapon_recoil_authority_math::
                    tryResolveFrameLocalTransform(
                        *inputWorld,
                        *outputWorld,
                        observedLocalDelta)) {
                return;
            }
            const float expectedTranslationError = pointDistance(
                expectedOutput.translate,
                outputWorld->translate);
            const auto expectedRotationError =
                compareRotationResidual(
                    expectedOutput,
                    *outputWorld);
            ROCK_LOG_INFO(
                Weapon,
                "AMBIRECOIL CONSUMER seq={} sample={} consumer={} inputT=({}) outputT=({}) observedLocalDelta=({}) resolvedKickLocal=({}) expectedToObserved=(T:{:.6f}gu,R:({})) alreadyIntegrated={}",
                trace.traceSequence,
                trace.currentSampleSequence,
                consumer,
                formatParityPoint(inputWorld->translate),
                formatParityPoint(outputWorld->translate),
                formatRecoilDelta(observedLocalDelta),
                formatRecoilDelta(trace.currentKickLocal),
                expectedTranslationError,
                formatParityRotationResidual(expectedRotationError),
                alreadyIntegratedSupportedHand ? "yes" : "no");
        } catch (...) {
        }
    }

    void TwoHandedGrip::traceAmbidextrousSupportParity(
        RE::NiNode* weaponNode,
        const bool settled)
    {
        const bool supportHandIsLeft = !_firingHandIsLeft;
        const WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        if (!weaponNode || !supportGrip.active ||
            supportGrip.gripSequence == 0) {
            return;
        }

        const bool identityChanged =
            _ambidextrousParitySupportTrace.weaponGenerationKey !=
                _activeWeaponGenerationKey ||
            _ambidextrousParitySupportTrace.
                    equippedWeaponOwnershipKey !=
                _activeEquippedWeaponOwnershipKey ||
            _ambidextrousParitySupportTrace.supportGripSequence !=
                supportGrip.gripSequence ||
            _ambidextrousParitySupportTrace.firingHandIsLeft !=
                _firingHandIsLeft;
        if (identityChanged) {
            _ambidextrousParitySupportTrace =
                AmbidextrousParitySupportTraceState{
                    .weaponGenerationKey = _activeWeaponGenerationKey,
                    .equippedWeaponOwnershipKey =
                        _activeEquippedWeaponOwnershipKey,
                    .supportGripSequence = supportGrip.gripSequence,
                    .firingHandIsLeft = _firingHandIsLeft,
                };
        }

        if (_authorityMode == weapon_support_authority_policy::
                                  WeaponSupportAuthorityMode::
                                      VisualOnlySupport) {
            if (!_ambidextrousParitySupportTrace.attachLogged) {
                traceAmbidextrousWeaponParity(
                    weaponNode,
                    "support-visual-only");
                _ambidextrousParitySupportTrace.attachLogged = true;
                _ambidextrousParitySupportTrace.settledLogged = true;
            }
            return;
        }

        if (!_ambidextrousParitySupportTrace.attachLogged) {
            traceAmbidextrousWeaponParity(
                weaponNode,
                "support-attached");
            _ambidextrousParitySupportTrace.attachLogged = true;
        }
        if (settled &&
            !_ambidextrousParitySupportTrace.settledLogged) {
            traceAmbidextrousWeaponParity(
                weaponNode,
                "support-settled");
            _ambidextrousParitySupportTrace.settledLogged = true;
        }
    }

    void TwoHandedGrip::traceAmbidextrousWeaponParity(
        RE::NiNode* weaponNode,
        const char* phase,
        const RE::NiTransform* leftAimCarrierWorld)
    {
        if (!weaponNode || !phase || _activeWeaponGenerationKey == 0 ||
            _activeEquippedWeaponOwnershipKey == 0 ||
            !isFiniteTransform(weaponNode->world) ||
            std::abs(weaponNode->world.scale) <= 0.0001f) {
            return;
        }

        const std::uint64_t traceSequence =
            ++_ambidextrousParityTraceSequence;
        const bool firingHandIsLeft = _firingHandIsLeft;
        const bool supportHandIsLeft = !firingHandIsLeft;
        const WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        const char* canonicalSource = "none";
        switch (_rightFiringHandCanonicalSource) {
        case RightFiringCanonicalSource::NativeCarry:
            canonicalSource = "native-carry";
            break;
        case RightFiringCanonicalSource::AuthoredAnimation:
            canonicalSource = "authored-animation";
            break;
        case RightFiringCanonicalSource::None:
        default:
            break;
        }
        const char* supportSource = "none";
        if (supportGrip.active) {
            supportSource = supportGrip.providerPartAuthority.active ?
                                "provider" :
                                (supportGrip.authoredSupportGrip ?
                                     "authored" :
                                     "dynamic");
        }

        const auto* playerNodes = f4vr::getPlayerNodes();
        RE::NiNode* rightWand =
            playerNodes ? playerNodes->primaryWandNode : nullptr;
        RE::NiNode* leftWand =
            playerNodes ? playerNodes->SecondaryWandNode : nullptr;
        RE::NiNode* rightDampedDriver =
            playerNodes ? playerNodes->primaryWeaponOffsetNOde : nullptr;
        RE::NiNode* leftDampedDriver = playerNodes ?
            playerNodes->SecondaryMeleeWeaponOffsetNode2 :
            nullptr;

        const bool rightNativeAimValid = hasRightNativeWeaponAimFrame(
            weaponNode,
            _activeWeaponGenerationKey,
            _activeEquippedWeaponOwnershipKey);
        const bool rightCanonicalCurrent = hasRightFiringHandCanonicalFrame(
            weaponNode,
            _activeWeaponGenerationKey,
            _activeEquippedWeaponOwnershipKey);
        RE::NiTransform expectedLeftWeaponInWand{};
        ParityAxes rightBaselineAxes{};
        ParityAxes expectedLeftAxes{};
        if (rightNativeAimValid) {
            expectedLeftWeaponInWand =
                left_firing_position_only_math::
                    mirrorRightWeaponInWandOrientation(
                        _rightNativeWeaponAimFrame.
                            weaponInWandOrientation);
            rightBaselineAxes = captureTransformAxes(
                _rightNativeWeaponAimFrame.weaponInWandOrientation);
            expectedLeftAxes =
                captureTransformAxes(expectedLeftWeaponInWand);
        }

        RE::NiTransform actualFiringWeaponInWand{};
        ParityAxes actualFiringAxes{};
        bool actualFiringWeaponInWandValid = false;
        RE::NiNode* firingWand =
            firingHandIsLeft ? leftWand : rightWand;
        if (firingWand && isFiniteTransform(firingWand->world) &&
            std::abs(firingWand->world.scale) > 0.0001f) {
            actualFiringWeaponInWand = transform_math::composeTransforms(
                transform_math::invertTransform(firingWand->world),
                weaponNode->world);
            actualFiringWeaponInWandValid =
                isFiniteTransform(actualFiringWeaponInWand);
            if (actualFiringWeaponInWandValid) {
                actualFiringAxes =
                    captureTransformAxes(actualFiringWeaponInWand);
            }
        }

        float rawAimParityErrorDegrees = -1.0f;
        if (rightNativeAimValid && actualFiringWeaponInWandValid) {
            rawAimParityErrorDegrees = firingHandIsLeft ?
                rotationDistanceDegrees(
                    expectedLeftWeaponInWand,
                    actualFiringWeaponInWand) :
                rotationDistanceDegrees(
                    _rightNativeWeaponAimFrame.
                        weaponInWandOrientation,
                    actualFiringWeaponInWand);
        }
        float aimCarrierParityErrorDegrees = -1.0f;
        if (firingHandIsLeft && rightNativeAimValid &&
            leftAimCarrierWorld &&
            isFiniteTransform(*leftAimCarrierWorld) &&
            std::abs(leftAimCarrierWorld->scale) > 0.0001f) {
            const RE::NiTransform weaponInAimCarrier =
                transform_math::composeTransforms(
                    transform_math::invertTransform(
                        *leftAimCarrierWorld),
                    weaponNode->world);
            if (isFiniteTransform(weaponInAimCarrier)) {
                aimCarrierParityErrorDegrees = rotationDistanceDegrees(
                    expectedLeftWeaponInWand,
                    weaponInAimCarrier);
            }
        }
        const auto nodeRotationDelta = [](
                                           const RE::NiNode* first,
                                           const RE::NiNode* second) {
            if (!first || !second || !isFiniteTransform(first->world) ||
                !isFiniteTransform(second->world)) {
                return -1.0f;
            }
            return rotationDistanceDegrees(first->world, second->world);
        };
        const float rightRawToDampedDegrees =
            nodeRotationDelta(rightWand, rightDampedDriver);
        const float leftRawToDampedDegrees =
            nodeRotationDelta(leftWand, leftDampedDriver);

        PhysicalHandInputFrame physicalLeftInput{};
        PhysicalHandInputFrame physicalRightInput{};
        const bool physicalLeftValid = tryResolvePhysicalHandInputFrame(
            true,
            physicalLeftInput);
        const bool physicalRightValid = tryResolvePhysicalHandInputFrame(
            false,
            physicalRightInput);
        const RE::NiTransform& physicalLeftWorld =
            physicalLeftInput.handWorld;
        const RE::NiTransform& physicalRightWorld =
            physicalRightInput.handWorld;
        RE::NiTransform rootLeftWorld{};
        RE::NiTransform rootRightWorld{};
        const bool rootLeftValid =
            tryGetRootFlattenedHandBoneTransform(true, rootLeftWorld);
        const bool rootRightValid =
            tryGetRootFlattenedHandBoneTransform(false, rootRightWorld);
        const ParityAxes rightDampedInRawWand =
            captureRelativeNodeAxes(rightWand, rightDampedDriver);
        const ParityAxes leftDampedInRawWand =
            captureRelativeNodeAxes(leftWand, leftDampedDriver);
        const ParityHandFrame physicalRightInRawWand =
            physicalRightValid && rightWand ?
            captureHandWorldFrame(
                rightWand->world,
                physicalRightWorld,
                false) :
            ParityHandFrame{};
        const ParityHandFrame physicalLeftInRawWand =
            physicalLeftValid && leftWand ?
            captureHandWorldFrame(
                leftWand->world,
                physicalLeftWorld,
                true) :
            ParityHandFrame{};
        const RE::NiTransform& physicalFiringWorld = firingHandIsLeft ?
            physicalLeftWorld :
            physicalRightWorld;
        const bool physicalFiringValid = firingHandIsLeft ?
            physicalLeftValid :
            physicalRightValid;
        const RE::NiTransform& physicalSupportWorld = supportHandIsLeft ?
            physicalLeftWorld :
            physicalRightWorld;
        const bool physicalSupportValid = supportHandIsLeft ?
            physicalLeftValid :
            physicalRightValid;

        const ParityHandFrame canonicalRightHand =
            captureHandLocalFrame(
                _rightFiringHandCanonicalWeaponLocal,
                false);
        const ParityHandFrame presentedFiringHand =
            _hasFiringHandWeaponLocal ?
            captureHandLocalFrame(
                _primaryHandWeaponLocal,
                firingHandIsLeft) :
            ParityHandFrame{};
        const ParityHandFrame physicalFiringHand = physicalFiringValid ?
            captureHandWorldFrame(
                weaponNode->world,
                physicalFiringWorld,
                firingHandIsLeft) :
            ParityHandFrame{};
        const RE::NiTransform& rootFiringWorld = firingHandIsLeft ?
            rootLeftWorld :
            rootRightWorld;
        const bool rootFiringValid = firingHandIsLeft ?
            rootLeftValid :
            rootRightValid;
        const ParityHandFrame rootFiringHand = rootFiringValid ?
            captureHandWorldFrame(
                weaponNode->world,
                rootFiringWorld,
                firingHandIsLeft) :
            ParityHandFrame{};

        RE::NiTransform presentedFiringWorld{};
        bool presentedFiringWorldValid = false;
        if (_hasFiringHandWeaponLocal) {
            presentedFiringWorld = transform_math::composeTransforms(
                weaponNode->world,
                _primaryHandWeaponLocal);
            presentedFiringWorldValid =
                isFiniteTransform(presentedFiringWorld);
        }

        RE::NiTransform presentedSupportWorld{};
        bool presentedSupportWorldValid = false;
        if (supportGrip.active && supportGrip.hasHandWeaponLocal) {
            presentedSupportWorld =
                resolvePartGripHandWorld(supportGrip, weaponNode);
            presentedSupportWorldValid =
                isFiniteTransform(presentedSupportWorld);
        }
        const ParityHandFrame presentedSupportHand =
            presentedSupportWorldValid ?
            captureHandWorldFrame(
                weaponNode->world,
                presentedSupportWorld,
                supportHandIsLeft) :
            ParityHandFrame{};
        const ParityHandFrame physicalSupportHand = physicalSupportValid ?
            captureHandWorldFrame(
                weaponNode->world,
                physicalSupportWorld,
                supportHandIsLeft) :
            ParityHandFrame{};
        const RE::NiTransform& rootSupportWorld = supportHandIsLeft ?
            rootLeftWorld :
            rootRightWorld;
        const bool rootSupportValid = supportHandIsLeft ?
            rootLeftValid :
            rootRightValid;
        const ParityHandFrame rootSupportHand = rootSupportValid ?
            captureHandWorldFrame(
                weaponNode->world,
                rootSupportWorld,
                supportHandIsLeft) :
            ParityHandFrame{};
        const ParityHandFrame candidateLeftSupport =
            _authoredSupportGripCandidate.valid ?
            captureHandLocalFrame(
                _authoredSupportGripCandidate.leftHandWeaponLocal,
                true) :
            ParityHandFrame{};
        const ParityHandFrame candidateRightSupport =
            _authoredSupportGripCandidate.valid &&
                    _authoredSupportGripCandidate.rightMirrorValid ?
            captureHandLocalFrame(
                _authoredSupportGripCandidate.rightHandWeaponLocal,
                false) :
            ParityHandFrame{};

        float firingPresentationDistance = -1.0f;
        float firingPresentationRotation = -1.0f;
        if (physicalFiringValid && presentedFiringWorldValid) {
            firingPresentationDistance = pointDistance(
                physicalFiringWorld.translate,
                presentedFiringWorld.translate);
            firingPresentationRotation = rotationDistanceDegrees(
                physicalFiringWorld,
                presentedFiringWorld);
        }
        float firingReadbackDistance = -1.0f;
        float firingReadbackRotation = -1.0f;
        if (presentedFiringWorldValid && rootFiringValid) {
            firingReadbackDistance = pointDistance(
                presentedFiringWorld.translate,
                rootFiringWorld.translate);
            firingReadbackRotation = rotationDistanceDegrees(
                presentedFiringWorld,
                rootFiringWorld);
        }
        float supportPresentationDistance = -1.0f;
        float supportPresentationRotation = -1.0f;
        if (physicalSupportValid && presentedSupportWorldValid) {
            supportPresentationDistance = pointDistance(
                physicalSupportWorld.translate,
                presentedSupportWorld.translate);
            supportPresentationRotation = rotationDistanceDegrees(
                physicalSupportWorld,
                presentedSupportWorld);
        }
        float supportReadbackDistance = -1.0f;
        float supportReadbackRotation = -1.0f;
        if (presentedSupportWorldValid && rootSupportValid) {
            supportReadbackDistance = pointDistance(
                presentedSupportWorld.translate,
                rootSupportWorld.translate);
            supportReadbackRotation = rotationDistanceDegrees(
                presentedSupportWorld,
                rootSupportWorld);
        }

        ParityGripAxisComparison gripAxisComparison{};
        if (firingHandIsLeft && canonicalRightHand.valid &&
            candidateLeftSupport.valid &&
            candidateRightSupport.valid &&
            std::isfinite(_primaryGripLocal.x) &&
            std::isfinite(_primaryGripLocal.y) &&
            std::isfinite(_primaryGripLocal.z)) {
            gripAxisComparison = compareParityGripAxes(
                _rightFiringGripCanonicalWeaponLocal,
                candidateLeftSupport.palmWeaponLocal,
                _primaryGripLocal,
                candidateRightSupport.palmWeaponLocal);
        }
        RE::NiPoint3 activeSupportGripLocal{};
        bool activeSupportGripLocalValid = false;
        if (supportGrip.active) {
            activeSupportGripLocal =
                resolvePartGripWeaponLocal(supportGrip, weaponNode);
            activeSupportGripLocalValid =
                finitePoint(activeSupportGripLocal);
        }

        ROCK_LOG_INFO(Weapon,
            "AMBIPARITY FRAME seq={} phase={} generation={:016X} ownership={:016X} state={} firing={} authority={} canonical={} canonicalCurrent={} canonicalCapture={} nativeAim={} independentDriverCalibration=(pair:{},L:{},R:{}) candidate=(valid:{},rightMirror:{},nodeMatch:{},generationMatch:{},capture:{}) support=(active:{},hand:{},source:{},grip:{},capture:{},fingerMask:0x{:04X}) firingFingerMask=0x{:04X}",
            traceSequence,
            phase,
            _activeWeaponGenerationKey,
            _activeEquippedWeaponOwnershipKey,
            static_cast<std::uint32_t>(_state),
            firingHandIsLeft ? "left" : "right",
            _authorityMode == weapon_support_authority_policy::
                                  WeaponSupportAuthorityMode::
                                      FullTwoHandedSolver ?
                "full" :
                "visual-only",
            canonicalSource,
            rightCanonicalCurrent ? "yes" : "no",
            _rightFiringHandCanonicalCaptureSequence,
            rightNativeAimValid ? "ready" : "missing",
            hasIndependentNaturalHandDriverPair() ? "ready" : "missing",
            hasCurrentNaturalHandDriverCalibration(true) ?
                "current" :
                "missing",
            hasCurrentNaturalHandDriverCalibration(false) ?
                "current" :
                "missing",
            _authoredSupportGripCandidate.valid ? "yes" : "no",
            _authoredSupportGripCandidate.rightMirrorValid ? "yes" : "no",
            _authoredSupportGripCandidate.weaponNode == weaponNode ?
                "yes" :
                "no",
            _authoredSupportGripCandidate.weaponGenerationKey ==
                    _activeWeaponGenerationKey ?
                "yes" :
                "no",
            _authoredSupportGripCandidate.captureSequence,
            supportGrip.active ? "yes" : "no",
            supportHandIsLeft ? "left" : "right",
            supportSource,
            supportGrip.gripSequence,
            supportGrip.authoredSupportCaptureSequence,
            supportGrip.fingerLocalTransformMask,
            firingHandIsLeft ?
                _leftFiringFingerLocalTransformMask :
                _rightFiringFingerLocalTransformMask);

        ROCK_LOG_INFO(Weapon,
            "AMBIPARITY AIM seq={} rightBaselineAxes=({}) expectedLeftAxes=({}) actualFiringAxes=({}) actualCarrier={} rawAimParity={:.4f}deg aimCarrierParity={:.4f}deg rawToDamped=(L:{:.4f}deg,R:{:.4f}deg)",
            traceSequence,
            formatParityAxes(rightBaselineAxes),
            formatParityAxes(expectedLeftAxes),
            formatParityAxes(actualFiringAxes),
            firingHandIsLeft ? "left-raw-wand" : "right-raw-wand",
            rawAimParityErrorDegrees,
            aimCarrierParityErrorDegrees,
            leftRawToDampedDegrees,
            rightRawToDampedDegrees);

        ROCK_LOG_INFO(Weapon,
            "AMBIPARITY CARRIERS seq={} rightDampedInRawWand=({}) leftDampedInRawWand=({}) rightPhysicalHandInRawWand=({}) leftPhysicalHandInRawWand=({})",
            traceSequence,
            formatParityAxes(rightDampedInRawWand),
            formatParityAxes(leftDampedInRawWand),
            formatParityHandAxes(physicalRightInRawWand),
            formatParityHandAxes(physicalLeftInRawWand));

        ROCK_LOG_INFO(Weapon,
            "AMBIPARITY FIRING seq={} physical={} presented={} rootReadback={} physicalToPresented=({:.4f}gu,{:.4f}deg) presentedToRoot=({:.4f}gu,{:.4f}deg) physicalT=({}) presentedT=({}) rootT=({}) canonicalRightT=({}) physicalPalm=({}) presentedPalm=({}) rootPalm=({}) solverPrimary=({})",
            traceSequence,
            physicalFiringHand.valid ? "ready" : "missing",
            presentedFiringHand.valid ? "ready" : "missing",
            rootFiringHand.valid ? "ready" : "missing",
            firingPresentationDistance,
            firingPresentationRotation,
            firingReadbackDistance,
            firingReadbackRotation,
            formatParityPoint(physicalFiringHand.handInWeapon.translate),
            formatParityPoint(presentedFiringHand.handInWeapon.translate),
            formatParityPoint(rootFiringHand.handInWeapon.translate),
            formatParityPoint(canonicalRightHand.handInWeapon.translate),
            formatParityPoint(physicalFiringHand.palmWeaponLocal),
            formatParityPoint(presentedFiringHand.palmWeaponLocal),
            formatParityPoint(rootFiringHand.palmWeaponLocal),
            formatParityPoint(_primaryGripLocal));

        ROCK_LOG_INFO(Weapon,
            "AMBIPARITY FIRING_AXES seq={} convention=(finger:+X,palm:-Y,cross:authored+Z/raw-Z) canonicalRight=({}) physical=({}) presented=({}) rootReadback=({})",
            traceSequence,
            formatParityHandAxes(canonicalRightHand),
            formatParityHandAxes(physicalFiringHand),
            formatParityHandAxes(presentedFiringHand),
            formatParityHandAxes(rootFiringHand));

        ROCK_LOG_INFO(Weapon,
            "AMBIPARITY SUPPORT seq={} physical={} presented={} rootReadback={} physicalToPresented=({:.4f}gu,{:.4f}deg) presentedToRoot=({:.4f}gu,{:.4f}deg) physicalT=({}) presentedT=({}) rootT=({}) candidateLeftT=({}) candidateRightT=({}) physicalPalm=({}) presentedPalm=({}) rootPalm=({}) activeGripValid={} activeGrip=({})",
            traceSequence,
            physicalSupportHand.valid ? "ready" : "missing",
            presentedSupportHand.valid ? "ready" : "missing",
            rootSupportHand.valid ? "ready" : "missing",
            supportPresentationDistance,
            supportPresentationRotation,
            supportReadbackDistance,
            supportReadbackRotation,
            formatParityPoint(physicalSupportHand.handInWeapon.translate),
            formatParityPoint(presentedSupportHand.handInWeapon.translate),
            formatParityPoint(rootSupportHand.handInWeapon.translate),
            formatParityPoint(candidateLeftSupport.handInWeapon.translate),
            formatParityPoint(candidateRightSupport.handInWeapon.translate),
            formatParityPoint(physicalSupportHand.palmWeaponLocal),
            formatParityPoint(presentedSupportHand.palmWeaponLocal),
            formatParityPoint(rootSupportHand.palmWeaponLocal),
            activeSupportGripLocalValid ? "yes" : "no",
            formatParityPoint(activeSupportGripLocal));

        ROCK_LOG_INFO(Weapon,
            "AMBIPARITY SUPPORT_AXES seq={} candidateLeft=({}) candidateRight=({}) physical=({}) presented=({}) rootReadback=({})",
            traceSequence,
            formatParityHandAxes(candidateLeftSupport),
            formatParityHandAxes(candidateRightSupport),
            formatParityHandAxes(physicalSupportHand),
            formatParityHandAxes(presentedSupportHand),
            formatParityHandAxes(rootSupportHand));

        ROCK_LOG_INFO(Weapon,
            "AMBIPARITY GRIP_AXIS seq={} valid={} rightPrimary=({}) rightSupport=({}) expectedLeftAxis=({}) actualLeftPrimary=({}) actualLeftSupport=({}) actualLeftAxis=({}) axisParityError={:.4f}deg pointMirrorError=(primary:{:.4f}gu,support:{:.4f}gu)",
            traceSequence,
            gripAxisComparison.valid ? "yes" : "no",
            formatParityPoint(_rightFiringGripCanonicalWeaponLocal),
            formatParityPoint(candidateLeftSupport.palmWeaponLocal),
            formatParityPoint(gripAxisComparison.expectedLeftAxis),
            formatParityPoint(_primaryGripLocal),
            formatParityPoint(candidateRightSupport.palmWeaponLocal),
            formatParityPoint(gripAxisComparison.actualLeftAxis),
            gripAxisComparison.angularErrorDegrees,
            gripAxisComparison.primaryPointMirrorError,
            gripAxisComparison.supportPointMirrorError);

        const std::string_view phaseName{ phase };
        if (phaseName == "left-primary") {
            traceAmbidextrousPoseHierarchy(
                weaponNode,
                phase,
                false);
        } else if (phaseName == "support-settled" ||
                   phaseName == "support-visual-only") {
            traceAmbidextrousPoseHierarchy(
                weaponNode,
                phase,
                true);
        }
    }

    void TwoHandedGrip::traceNativeScopeTransitionFinalState(RE::NiNode* weaponNode)
    {
        if (!_nativeScopeTransitionFinalTracePending) {
            return;
        }
        _nativeScopeTransitionFinalTracePending = false;

        struct HmdRelativeTrace
        {
            RE::NiPoint3 position{};
            bool valid{ false };
        };

        const auto* playerNodes = f4vr::getPlayerNodes();
        RE::NiTransform hmdWorld{};
        const bool hmdWorldValid =
            playerNodes && playerNodes->HmdNode &&
            isFiniteTransform(playerNodes->HmdNode->world);
        if (hmdWorldValid) {
            hmdWorld = playerNodes->HmdNode->world;
        }

        const auto captureWorld = [&hmdWorld, hmdWorldValid](
                                      const RE::NiTransform& world,
                                      const bool worldValid) {
            HmdRelativeTrace trace{};
            if (!hmdWorldValid || !worldValid) {
                return trace;
            }
            trace.position = transform_math::worldPointToLocal(
                hmdWorld,
                world.translate);
            trace.valid = std::isfinite(trace.position.x) &&
                          std::isfinite(trace.position.y) &&
                          std::isfinite(trace.position.z);
            return trace;
        };
        const auto captureNode = [&captureWorld](const RE::NiAVObject* node) {
            return captureWorld(
                node ? node->world : RE::NiTransform{},
                node && isFiniteTransform(node->world));
        };

        RE::NiTransform scopeCameraWorld{};
        bool scopeCameraWorldValid = false;
        if (playerNodes && playerNodes->primaryWeaponScopeCamera) {
            const auto* scopeCamera = playerNodes->primaryWeaponScopeCamera;
            scopeCameraWorld = scopeCamera->world;
            if (scopeCamera->parent &&
                isFiniteTransform(scopeCamera->parent->world) &&
                isFiniteTransform(scopeCamera->local)) {
                scopeCameraWorld = transform_math::composeTransforms(
                    scopeCamera->parent->world,
                    scopeCamera->local);
            }
            scopeCameraWorldValid = isFiniteTransform(scopeCameraWorld);
        }

        RE::NiTransform leftRootWorld{};
        RE::NiTransform rightRootWorld{};
        const bool leftRootWorldValid =
            tryGetRootFlattenedHandBoneTransform(true, leftRootWorld);
        const bool rightRootWorldValid =
            tryGetRootFlattenedHandBoneTransform(false, rightRootWorld);
        const auto* playerCamera = f4vr::getPlayerCamera();
        const bool cameraValuesValid =
            playerCamera &&
            std::isfinite(playerCamera->zoomInput) &&
            std::isfinite(playerCamera->worldFOV) &&
            std::isfinite(playerCamera->firstPersonFOV) &&
            std::isfinite(playerCamera->fovAdjustCurrent) &&
            std::isfinite(playerCamera->fovAdjustTarget) &&
            std::isfinite(playerCamera->fovAdjustPerSec) &&
            std::isfinite(playerCamera->fovAnimatorAdjust);

        const HmdRelativeTrace weaponTrace = captureWorld(
            weaponNode ? weaponNode->world : RE::NiTransform{},
            weaponNode && isFiniteTransform(weaponNode->world));
        const HmdRelativeTrace leftRootTrace = captureWorld(
            leftRootWorld,
            leftRootWorldValid);
        const HmdRelativeTrace rightRootTrace = captureWorld(
            rightRootWorld,
            rightRootWorldValid);
        const HmdRelativeTrace scopeCameraTrace = captureWorld(
            scopeCameraWorld,
            scopeCameraWorldValid);
        const HmdRelativeTrace scopeParentTrace = captureNode(
            playerNodes ? playerNodes->ScopeParentNode : nullptr);
        const HmdRelativeTrace cameraRootTrace = captureNode(
            playerCamera ? playerCamera->cameraRoot.get() : nullptr);
        const HmdRelativeTrace roomTrace = captureNode(
            playerNodes ? playerNodes->roomnode : nullptr);
        const HmdRelativeTrace uprightHmdTrace = captureNode(
            playerNodes ? playerNodes->UprightHmdNode : nullptr);
        const HmdRelativeTrace skeletonRootTrace =
            captureNode(f4vr::getRootNode());

        ROCK_LOG_INFO(Weapon,
            "SCOPE-VIEW-FINAL seq={} sample={}/{} buttonRequested={} rendererActive={} menuOpen={} driverAuthority={} state={} hmdValid={} hmdWorld=({:.2f},{:.2f},{:.2f}) weaponHmdValid={} weaponHmd=({:.2f},{:.2f},{:.2f}) leftRootHmdValid={} leftRootHmd=({:.2f},{:.2f},{:.2f}) rightRootHmdValid={} rightRootHmd=({:.2f},{:.2f},{:.2f}) scopeCameraHmdValid={} scopeCameraHmd=({:.2f},{:.2f},{:.2f}) scopeParentHmdValid={} scopeParentHmd=({:.2f},{:.2f},{:.2f}) cameraRootHmdValid={} cameraRootHmd=({:.2f},{:.2f},{:.2f}) roomHmdValid={} roomHmd=({:.2f},{:.2f},{:.2f}) uprightHmdValid={} uprightHmd=({:.2f},{:.2f},{:.2f}) skeletonRootHmdValid={} skeletonRootHmd=({:.2f},{:.2f},{:.2f}) cameraValuesValid={} zoomInput={:.4f} worldFov={:.4f} firstPersonFov={:.4f} fovAdjust=({:.4f},{:.4f},{:.4f},{:.4f})",
            _nativeScopeTransitionFinalTraceSequence,
            _nativeScopeTransitionFinalTraceSample,
            SCOPE_TRANSITION_TRACE_FRAMES,
            _manualScopeActivationRequested ? "yes" : "no",
            _nativeScopeRequestActive ? "yes" : "no",
            _scopeMenuOpenThisFrame ? "yes" : "no",
            _scopeDriverFrameAuthorityActive ? "yes" : "no",
            static_cast<std::uint32_t>(_state),
            hmdWorldValid ? "yes" : "no",
            hmdWorld.translate.x,
            hmdWorld.translate.y,
            hmdWorld.translate.z,
            weaponTrace.valid ? "yes" : "no",
            weaponTrace.position.x,
            weaponTrace.position.y,
            weaponTrace.position.z,
            leftRootTrace.valid ? "yes" : "no",
            leftRootTrace.position.x,
            leftRootTrace.position.y,
            leftRootTrace.position.z,
            rightRootTrace.valid ? "yes" : "no",
            rightRootTrace.position.x,
            rightRootTrace.position.y,
            rightRootTrace.position.z,
            scopeCameraTrace.valid ? "yes" : "no",
            scopeCameraTrace.position.x,
            scopeCameraTrace.position.y,
            scopeCameraTrace.position.z,
            scopeParentTrace.valid ? "yes" : "no",
            scopeParentTrace.position.x,
            scopeParentTrace.position.y,
            scopeParentTrace.position.z,
            cameraRootTrace.valid ? "yes" : "no",
            cameraRootTrace.position.x,
            cameraRootTrace.position.y,
            cameraRootTrace.position.z,
            roomTrace.valid ? "yes" : "no",
            roomTrace.position.x,
            roomTrace.position.y,
            roomTrace.position.z,
            uprightHmdTrace.valid ? "yes" : "no",
            uprightHmdTrace.position.x,
            uprightHmdTrace.position.y,
            uprightHmdTrace.position.z,
            skeletonRootTrace.valid ? "yes" : "no",
            skeletonRootTrace.position.x,
            skeletonRootTrace.position.y,
            skeletonRootTrace.position.z,
            cameraValuesValid ? "yes" : "no",
            cameraValuesValid ? playerCamera->zoomInput : 0.0f,
            cameraValuesValid ? playerCamera->worldFOV : 0.0f,
            cameraValuesValid ? playerCamera->firstPersonFOV : 0.0f,
            cameraValuesValid ? playerCamera->fovAdjustCurrent : 0.0f,
            cameraValuesValid ? playerCamera->fovAdjustTarget : 0.0f,
            cameraValuesValid ? playerCamera->fovAdjustPerSec : 0.0f,
            cameraValuesValid ? playerCamera->fovAnimatorAdjust : 0.0f);
    }

    bool TwoHandedGrip::getDebugAuthoritySnapshot(TwoHandedGripDebugSnapshot& outSnapshot) const
    {
        const auto& leftGrip = partGrip(true);
        const auto& rightGrip = partGrip(false);
        if (!_hasSolvedWeaponTransform || !_activeWeaponNode) {
            return false;
        }
        if (!_hasFiringHandWeaponLocal && !leftGrip.active && !rightGrip.active) {
            return false;
        }

        outSnapshot.weaponWorld = _lastSolvedWeaponTransform;
        if (rightGrip.active) {
            outSnapshot.rightRequestedHandWorld = resolvePartGripHandWorld(rightGrip, _activeWeaponNode);
            outSnapshot.rightGripWorld = resolvePartGripWorld(rightGrip, _activeWeaponNode);
        } else {
            outSnapshot.rightRequestedHandWorld = transform_math::composeTransforms(_lastSolvedWeaponTransform, _primaryHandWeaponLocal);
            outSnapshot.rightGripWorld = transform_math::localPointToWorld(_lastSolvedWeaponTransform, _primaryGripLocal);
        }
        if (leftGrip.active) {
            outSnapshot.leftRequestedHandWorld = resolvePartGripHandWorld(leftGrip, _activeWeaponNode);
            outSnapshot.leftGripWorld = resolvePartGripWorld(leftGrip, _activeWeaponNode);
        } else {
            outSnapshot.leftRequestedHandWorld = RE::NiTransform{};
            outSnapshot.leftGripWorld = RE::NiPoint3{};
        }
        return true;
    }

    bool TwoHandedGrip::getAuthoredSupportGripDebugSnapshot(
        AuthoredSupportGripDebugSnapshot& outSnapshot) const
    {
        outSnapshot = _authoredSupportGripDebugSnapshot;
        return outSnapshot.valid;
    }
}
