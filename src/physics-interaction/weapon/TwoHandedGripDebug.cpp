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

        RE::NiTransform physicalLeftWorld{};
        RE::NiTransform physicalRightWorld{};
        RE::NiTransform leftDriverWorld{};
        RE::NiTransform rightDriverWorld{};
        const bool physicalLeftValid = tryResolvePhysicalHandFrame(
            true,
            physicalLeftWorld,
            leftDriverWorld);
        const bool physicalRightValid = tryResolvePhysicalHandFrame(
            false,
            physicalRightWorld,
            rightDriverWorld);
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
            "AMBIPARITY FRAME seq={} phase={} generation={:016X} ownership={:016X} state={} firing={} authority={} canonical={} canonicalCurrent={} canonicalCapture={} nativeAim={} naturalWand=(L:{},R:{}) naturalDriver=(L:{},R:{}) candidate=(valid:{},rightMirror:{},nodeMatch:{},generationMatch:{},capture:{}) support=(active:{},hand:{},source:{},grip:{},capture:{},fingerMask:0x{:04X}) firingFingerMask=0x{:04X}",
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
            _hasLeftNaturalBoneInWand ? "ready" : "missing",
            _hasRightNaturalBoneInWand ? "ready" : "missing",
            _hasLeftNaturalBoneInDampedDriver ? "ready" : "missing",
            _hasRightNaturalBoneInDampedDriver ? "ready" : "missing",
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
