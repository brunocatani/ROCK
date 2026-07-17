#include "physics-interaction/weapon/AuthoredPrimaryFiringGrip.h"

#include "physics-interaction/animation/NativeAnimationAuthority.h"
#include "physics-interaction/animation/NativeAnimationAuthorityPolicy.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"

#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiTransform.h"

#include <cmath>

namespace rock
{
    namespace
    {
        constexpr char kAuthorityTag[] = "rock.authored-primary-firing-grip";
        // Normal grab/contact/weapon authorities occupy priorities 80-100.
        // This baseline grip yields to all of them without a competing writer.
        constexpr int kAuthorityPriority = 70;

        [[nodiscard]] float translationDistance(
            const RE::NiTransform& lhs,
            const RE::NiTransform& rhs)
        {
            const float x = lhs.translate.x - rhs.translate.x;
            const float y = lhs.translate.y - rhs.translate.y;
            const float z = lhs.translate.z - rhs.translate.z;
            return std::sqrt(x * x + y * y + z * z);
        }
    }

    void AuthoredPrimaryFiringGripRuntime::clearAuthority(const char* reason)
    {
        if (!_published) {
            return;
        }

        if (!frik_visual_authority::clearExternalHandWorldTransform(
                kAuthorityTag,
                frik_visual_authority::Hand::Primary)) {
            if (!_clearFailureLogged) {
                ROCK_LOG_WARN(Animation,
                    "Authored primary firing grip could not release FRIK authority reason={}",
                    reason ? reason : "unknown");
                _clearFailureLogged = true;
            }
            return;
        }

        ROCK_LOG_DEBUG(Animation,
            "Authored primary firing grip released reason={}",
            reason ? reason : "unknown");
        _published = false;
        _sessionLogged = false;
        _clearFailureLogged = false;
    }

    void AuthoredPrimaryFiringGripRuntime::reset(const char* reason)
    {
        clearAuthority(reason);
        _weaponOwnershipKey = 0;
        _captureSequenceFloor = 0;
        _nativeReloadWasActive = false;
        _sessionLogged = false;
        _publishFailureLogged = false;
        if (!_published) {
            _clearFailureLogged = false;
        }
    }

    void AuthoredPrimaryFiringGripRuntime::update(
        const AuthoredPrimaryFiringGripFrameInput& input)
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
            clearAuthority("native-reload-authority");
            return;
        }
        if (_nativeReloadWasActive) {
            // Do not reuse the frozen pre-reload relation on the exact release
            // frame. One new native graph sample must establish the idle grip.
            _nativeReloadWasActive = false;
            _captureSequenceFloor = captureStatus.captureSequence;
            clearAuthority("native-reload-ended-awaiting-fresh-capture");
            return;
        }

        const std::uint64_t currentWeaponKey =
            input.weaponNode ? input.weaponOwnershipKey : 0;
        if (currentWeaponKey != _weaponOwnershipKey) {
            clearAuthority("weapon-boundary");
            _weaponOwnershipKey = currentWeaponKey;
            _captureSequenceFloor = captureStatus.captureSequence;
            _sessionLogged = false;
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
            .primaryHandHoldingObject = input.primaryHandHoldingObject,
            .leftHandedMode = input.leftHandedMode,
        };
        if (!native_animation_authority_policy::shouldApplyAuthoredPrimaryFiringGrip(eligibility)) {
            clearAuthority("frame-ineligible");
            return;
        }

        RE::NiTransform targetHandWorld{};
        std::uint64_t resolvedCaptureSequence = 0;
        if (!native_animation_authority::tryResolvePrimaryFiringGripWorldTarget(
                input.weaponNode,
                input.weaponNode->world,
                targetHandWorld,
                resolvedCaptureSequence) ||
            resolvedCaptureSequence <= _captureSequenceFloor) {
            clearAuthority("capture-resolution-failed");
            return;
        }

        const RE::NiTransform trackedHandWorld =
            frik_visual_authority::getHandWorldTransform(
                frik_visual_authority::Hand::Primary);
        if (!frik_visual_authority::applyExternalHandWorldTransform(
                kAuthorityTag,
                frik_visual_authority::Hand::Primary,
                targetHandWorld,
                kAuthorityPriority)) {
            if (!_publishFailureLogged) {
                ROCK_LOG_WARN(Animation,
                    "Authored primary firing grip could not publish FRIK authority weaponKey=0x{:X} capture={}",
                    currentWeaponKey,
                    resolvedCaptureSequence);
                _publishFailureLogged = true;
            }
            clearAuthority("publish-failed");
            return;
        }

        _published = true;
        _publishFailureLogged = false;
        if (!_sessionLogged) {
            ROCK_LOG_INFO(Animation,
                "Authored primary firing grip active weaponKey=0x{:X} capture={} trackedToAuthored={:.3f}gu weaponT=({:.3f},{:.3f},{:.3f}) handTargetT=({:.3f},{:.3f},{:.3f}) priority={} hand=primary-only",
                currentWeaponKey,
                resolvedCaptureSequence,
                translationDistance(trackedHandWorld, targetHandWorld),
                input.weaponNode->world.translate.x,
                input.weaponNode->world.translate.y,
                input.weaponNode->world.translate.z,
                targetHandWorld.translate.x,
                targetHandWorld.translate.y,
                targetHandWorld.translate.z,
                kAuthorityPriority);
            _sessionLogged = true;
        }
    }
}
