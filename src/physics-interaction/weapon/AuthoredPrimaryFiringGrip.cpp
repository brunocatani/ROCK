#include "physics-interaction/weapon/AuthoredPrimaryFiringGrip.h"

#include "physics-interaction/animation/NativeAnimationAuthority.h"
#include "physics-interaction/animation/NativeAnimationAuthorityPolicy.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/TwoHandedGrip.h"

#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiTransform.h"

#include <cmath>

namespace rock
{
    namespace
    {
        [[nodiscard]] float translationDistance(
            const RE::NiTransform& lhs,
            const RE::NiTransform& rhs)
        {
            const float x = lhs.translate.x - rhs.translate.x;
            const float y = lhs.translate.y - rhs.translate.y;
            const float z = lhs.translate.z - rhs.translate.z;
            return std::sqrt(x * x + y * y + z * z);
        }

        [[nodiscard]] bool finiteTransform(const RE::NiTransform& transform)
        {
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(transform.rotate.entry[row][column])) {
                        return false;
                    }
                }
            }
            return std::isfinite(transform.translate.x) &&
                   std::isfinite(transform.translate.y) &&
                   std::isfinite(transform.translate.z) &&
                   std::isfinite(transform.scale) &&
                   std::abs(transform.scale) > 0.000001f;
        }
    }

    void AuthoredPrimaryFiringGripRuntime::endSession(const char* reason)
    {
        if (!_active) {
            return;
        }

        ROCK_LOG_DEBUG(Animation,
            "Authored primary firing grip weapon alignment suspended reason={}",
            reason ? reason : "unknown");
        _active = false;
        _sessionLogged = false;
    }

    void AuthoredPrimaryFiringGripRuntime::reset(const char* reason)
    {
        endSession(reason);
        _weaponNodeIdentity = nullptr;
        _weaponOwnershipKey = 0;
        _captureSequenceFloor = 0;
        _nativeReloadWasActive = false;
        _sessionLogged = false;
        _applyFailureLogged = false;
    }

    void AuthoredPrimaryFiringGripRuntime::update(
        const AuthoredPrimaryFiringGripFrameInput& input,
        TwoHandedGrip& weaponAuthority)
    {
        const auto captureStatus =
            native_animation_authority::queryPrimaryFiringGripCaptureStatus();

        if (!input.enabled) {
            reset("experiment-disabled");
            return;
        }

        if (input.nativeReloadAuthorityActive) {
            if (!_nativeReloadWasActive) {
                _captureSequenceFloor = captureStatus.captureSequence;
            }
            _nativeReloadWasActive = true;
            endSession("native-reload-authority");
            return;
        }
        if (_nativeReloadWasActive) {
            // Do not reuse the frozen pre-reload relation on the exact release
            // frame. One new native graph sample must establish the idle grip.
            _nativeReloadWasActive = false;
            _captureSequenceFloor = captureStatus.captureSequence;
            endSession("native-reload-ended-awaiting-fresh-capture");
            return;
        }

        const std::uint64_t currentWeaponKey =
            input.weaponNode ? input.weaponOwnershipKey : 0;
        if (input.weaponNode != _weaponNodeIdentity ||
            currentWeaponKey != _weaponOwnershipKey) {
            endSession("weapon-boundary");
            _weaponNodeIdentity = input.weaponNode;
            _weaponOwnershipKey = currentWeaponKey;
            _captureSequenceFloor = captureStatus.captureSequence;
            _sessionLogged = false;
            _applyFailureLogged = false;
            return;
        }

        const native_animation_authority_policy::AuthoredPrimaryFiringGripEligibility eligibility{
            .enabled = input.enabled,
            .runtimeInitialized = input.runtimeInitialized,
            .visualAuthorityAvailable = input.visualAuthorityAvailable,
            .localSkeletonReady = input.localSkeletonReady,
            .menuBlocking = input.menuBlocking,
            .compatibilityBlocking = input.compatibilityBlocking,
            .weaponDrawn = input.weaponDrawn,
            .weaponVisible = input.weaponVisible,
            .weaponKeyValid = currentWeaponKey != 0,
            .captureValid = captureStatus.valid,
            .captureNewerThanWeaponBoundary =
                captureStatus.captureSequence > _captureSequenceFloor,
            .nativeReloadAuthorityActive = input.nativeReloadAuthorityActive,
            .manualWeaponAuthorityActive = input.manualWeaponAuthorityActive,
            .weaponVisualReturnActive = input.weaponVisualReturnActive,
            .primaryHandHoldingObject = input.primaryHandHoldingObject,
            .leftHandedMode = input.leftHandedMode,
        };
        if (!native_animation_authority_policy::shouldApplyAuthoredPrimaryFiringGrip(eligibility)) {
            endSession("frame-ineligible");
            return;
        }

        const RE::NiTransform liveWeaponWorld = input.weaponNode->world;
        const RE::NiTransform trackedHandWorld =
            frik_visual_authority::getHandWorldTransform(
                frik_visual_authority::Hand::Primary);
        if (!finiteTransform(liveWeaponWorld) || !finiteTransform(trackedHandWorld)) {
            endSession("live-transform-invalid");
            return;
        }

        RE::NiTransform solvedWeaponWorld{};
        RE::NiTransform currentAuthoredHandWorld{};
        std::uint64_t resolvedCaptureSequence = 0;
        if (!native_animation_authority::tryResolvePrimaryFiringGripAlignment(
                input.weaponNode,
                liveWeaponWorld,
                trackedHandWorld,
                solvedWeaponWorld,
                currentAuthoredHandWorld,
                resolvedCaptureSequence) ||
            resolvedCaptureSequence <= _captureSequenceFloor) {
            endSession("capture-resolution-failed");
            return;
        }

        if (!weaponAuthority.applyAuthoredPrimaryGripWeaponAlignment(
                input.weaponNode,
                solvedWeaponWorld,
                input.weaponGenerationKey)) {
            if (!_applyFailureLogged) {
                ROCK_LOG_WARN(Animation,
                    "Authored primary firing grip could not apply weapon alignment weaponKey=0x{:X} capture={}",
                    currentWeaponKey,
                    resolvedCaptureSequence);
                _applyFailureLogged = true;
            }
            endSession("weapon-alignment-failed");
            return;
        }

        _active = true;
        _applyFailureLogged = false;
        if (!_sessionLogged) {
            ROCK_LOG_INFO(Animation,
                "Authored primary firing grip weapon alignment active weaponKey=0x{:X} generation=0x{:X} capture={} handMismatch={:.3f}gu weaponCorrection={:.3f}gu originalWeaponT=({:.3f},{:.3f},{:.3f}) alignedWeaponT=({:.3f},{:.3f},{:.3f}) alignedLocalT=({:.3f},{:.3f},{:.3f}) authority=weapon-only primaryHand=controller-driven",
                currentWeaponKey,
                input.weaponGenerationKey,
                resolvedCaptureSequence,
                translationDistance(trackedHandWorld, currentAuthoredHandWorld),
                translationDistance(liveWeaponWorld, solvedWeaponWorld),
                liveWeaponWorld.translate.x,
                liveWeaponWorld.translate.y,
                liveWeaponWorld.translate.z,
                input.weaponNode->world.translate.x,
                input.weaponNode->world.translate.y,
                input.weaponNode->world.translate.z,
                input.weaponNode->local.translate.x,
                input.weaponNode->local.translate.y,
                input.weaponNode->local.translate.z);
            _sessionLogged = true;
        }
    }
}
