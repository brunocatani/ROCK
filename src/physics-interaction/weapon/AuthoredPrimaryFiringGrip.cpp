#include "physics-interaction/weapon/AuthoredPrimaryFiringGrip.h"

#include "physics-interaction/animation/NativeAnimationAuthority.h"
#include "physics-interaction/animation/NativeAnimationAuthorityPolicy.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/grab/FrikWeaponOffsetCache.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
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

    void AuthoredPrimaryFiringGripRuntime::reset(
        const char* reason,
        TwoHandedGrip& weaponAuthority)
    {
        weaponAuthority.clearAuthoredPrimaryFiringGripCanonical(reason);
        endSession(reason);
        _weaponNodeIdentity = nullptr;
        _weaponOwnershipKey = 0;
        _frikOffsetCacheRevision = 0;
        _captureSequenceFloor = 0;
        _supportCaptureSequenceFloor = 0;
        _nativeReloadWasActive = false;
        _sessionLogged = false;
        _applyFailureLogged = false;
        _canonicalPublishFailureLogged = false;
        _libraryPublishFailureLogged = false;
        _customFrikOffsetOverrideActive = false;
        _supportCaptureFailureReasonLogged = 0;
        _supportCaptureFailureMaskLogged = 0;
        _supportCaptureFailureLogged = false;
    }

    void AuthoredPrimaryFiringGripRuntime::update(
        const AuthoredPrimaryFiringGripFrameInput& input,
        TwoHandedGrip& weaponAuthority)
    {
        // A candidate is valid only across this pre-update/update pair. Clear
        // first so every early return falls back to ordinary dynamic grabbing.
        weaponAuthority.clearAuthoredSupportGripCandidate();
        const auto captureStatus =
            native_animation_authority::queryPrimaryFiringGripCaptureStatus();
        const auto supportCaptureStatus =
            native_animation_authority::queryAuthoredSupportGripCaptureStatus();

        if (!input.enabled) {
            reset("experiment-disabled", weaponAuthority);
            return;
        }

        if (input.nativeReloadAuthorityActive) {
            if (!_nativeReloadWasActive) {
                _captureSequenceFloor = captureStatus.captureSequence;
                _supportCaptureSequenceFloor = supportCaptureStatus.captureSequence;
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
            _supportCaptureSequenceFloor = supportCaptureStatus.captureSequence;
            endSession("native-reload-ended-awaiting-fresh-capture");
            return;
        }

        const std::uint64_t currentWeaponKey =
            input.weaponNode ? input.weaponOwnershipKey : 0;
        if (input.weaponNode != _weaponNodeIdentity ||
            currentWeaponKey != _weaponOwnershipKey) {
            weaponAuthority.clearAuthoredPrimaryFiringGripCanonical(
                "weapon-boundary");
            endSession("weapon-boundary");
            _weaponNodeIdentity = input.weaponNode;
            _weaponOwnershipKey = currentWeaponKey;
            _captureSequenceFloor = captureStatus.captureSequence;
            _supportCaptureSequenceFloor = supportCaptureStatus.captureSequence;
            _sessionLogged = false;
            _applyFailureLogged = false;
            _canonicalPublishFailureLogged = false;
            _libraryPublishFailureLogged = false;
            _supportCaptureFailureLogged = false;

            _customFrikOffsetOverrideActive = false;
            _frikOffsetCacheRevision = frik_weapon_offset_cache::currentRevision();
            if (input.weapon && input.weaponNode) {
                const auto frikOffset =
                    frik_weapon_offset_cache::findPrimaryWeaponOffset(input.weapon, input.weaponNode);
                _customFrikOffsetOverrideActive =
                    frikOffset.found &&
                    frikOffset.source == frik_weapon_offset_cache::OffsetSource::CustomFile;
                if (_customFrikOffsetOverrideActive) {
                    ROCK_LOG_INFO(Animation,
                        "Authored primary firing grip yielded to custom hFRIK weapon offset weaponKey=0x{:X} source={}",
                        currentWeaponKey,
                        frikOffset.reason);
                }
            }
            return;
        }

        const auto frikOffsetCacheRevision = frik_weapon_offset_cache::currentRevision();
        if (frikOffsetCacheRevision != _frikOffsetCacheRevision) {
            const bool previousCustomOverride = _customFrikOffsetOverrideActive;
            _frikOffsetCacheRevision = frikOffsetCacheRevision;

            const auto frikOffset =
                frik_weapon_offset_cache::findPrimaryWeaponOffset(input.weapon, input.weaponNode);
            _customFrikOffsetOverrideActive =
                frikOffset.found &&
                frikOffset.source == frik_weapon_offset_cache::OffsetSource::CustomFile;
            if (_customFrikOffsetOverrideActive != previousCustomOverride) {
                ROCK_LOG_INFO(Animation,
                    "Authored primary firing grip custom hFRIK override {} weaponKey=0x{:X} cacheRevision={} source={}",
                    _customFrikOffsetOverrideActive ? "activated" : "released",
                    currentWeaponKey,
                    frikOffsetCacheRevision,
                    frikOffset.reason);

                weaponAuthority.clearAuthoredPrimaryFiringGripCanonical(
                    "custom-frik-weapon-offset-change");
                _captureSequenceFloor = captureStatus.captureSequence;
                _supportCaptureSequenceFloor = supportCaptureStatus.captureSequence;
                endSession("custom-frik-weapon-offset-change");
                return;
            }
        }

        if (_customFrikOffsetOverrideActive) {
            weaponAuthority.clearAuthoredPrimaryFiringGripCanonical(
                "custom-frik-weapon-offset");
            endSession("custom-frik-weapon-offset");
            return;
        }

        if (input.leftHandedMode) {
            // Bethesda/hFRIK game-left mode is a distinct topology, not the
            // ROCK ambidextrous role. Never carry a right-authored canonical
            // across that mode; require a new right-mode graph sample later.
            weaponAuthority.clearAuthoredPrimaryFiringGripCanonical(
                "game-left-handed-mode");
            _captureSequenceFloor = captureStatus.captureSequence;
            _supportCaptureSequenceFloor = supportCaptureStatus.captureSequence;
            endSession("game-left-handed-mode");
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
            .conflictingWeaponTransformAuthorityActive =
                input.conflictingWeaponTransformAuthorityActive,
            .weaponVisualReturnActive = input.weaponVisualReturnActive,
            .primaryHandHoldingObject = input.primaryHandHoldingObject,
            .leftHandedMode = input.leftHandedMode,
            .rockFiringHandIsLeft = input.rockFiringHandIsLeft,
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
        RE::NiTransform authoredPrimaryHandInWeapon{};
        std::uint64_t resolvedCaptureSequence = 0;
        if (!native_animation_authority::tryResolvePrimaryFiringGripAlignment(
                input.weaponNode,
                liveWeaponWorld,
                trackedHandWorld,
                solvedWeaponWorld,
                currentAuthoredHandWorld,
                authoredPrimaryHandInWeapon,
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

        if (!weaponAuthority.setAuthoredPrimaryFiringGripCanonical(
                input.weaponNode,
                authoredPrimaryHandInWeapon,
                input.weaponGenerationKey,
                currentWeaponKey,
                resolvedCaptureSequence)) {
            if (!_canonicalPublishFailureLogged) {
                ROCK_LOG_WARN(Animation,
                    "Authored primary firing grip could not publish mirrored-left canonical weaponKey=0x{:X} generation=0x{:X} capture={}",
                    currentWeaponKey,
                    input.weaponGenerationKey,
                    resolvedCaptureSequence);
                _canonicalPublishFailureLogged = true;
            }
        } else {
            _canonicalPublishFailureLogged = false;
        }

        if (!authored_weapon_grip_library::publish(
                input.weapon,
                input.weaponNode,
                input.inPowerArmor,
                authoredPrimaryHandInWeapon,
                resolvedCaptureSequence)) {
            if (!_libraryPublishFailureLogged) {
                ROCK_LOG_WARN(Animation,
                    "Authored primary firing grip could not publish loose-weapon relation weaponKey=0x{:X} capture={}",
                    currentWeaponKey,
                    resolvedCaptureSequence);
                _libraryPublishFailureLogged = true;
            }
        } else {
            _libraryPublishFailureLogged = false;
        }

        _active = true;
        _applyFailureLogged = false;

        const auto supportCaptureFailureReason =
            static_cast<std::uint32_t>(supportCaptureStatus.failureReason);
        if (!supportCaptureStatus.valid) {
            if (!_supportCaptureFailureLogged ||
                supportCaptureFailureReason != _supportCaptureFailureReasonLogged ||
                supportCaptureStatus.invalidOrMissingFingerMask !=
                    _supportCaptureFailureMaskLogged) {
                ROCK_LOG_WARN(Animation,
                    "Authored support grip capture unavailable weaponKey=0x{:X} reason={} secondaryPass={} capture={} fingerMask=0x{:04X}",
                    currentWeaponKey,
                    native_animation_authority::authoredSupportGripCaptureFailureReasonName(
                        supportCaptureStatus.failureReason),
                    supportCaptureStatus.secondaryPassSequence,
                    supportCaptureStatus.captureSequence,
                    supportCaptureStatus.invalidOrMissingFingerMask);
                _supportCaptureFailureReasonLogged = supportCaptureFailureReason;
                _supportCaptureFailureMaskLogged =
                    supportCaptureStatus.invalidOrMissingFingerMask;
                _supportCaptureFailureLogged = true;
            }
        } else {
            _supportCaptureFailureLogged = false;
        }

        RE::NiTransform authoredSupportHandInWeapon{};
        std::array<RE::NiTransform, 15> authoredSupportFingerLocals{};
        std::uint16_t authoredSupportFingerMask = 0;
        std::uint64_t authoredSupportCaptureSequence = 0;
        if (supportCaptureStatus.valid &&
            supportCaptureStatus.captureSequence > _supportCaptureSequenceFloor &&
            native_animation_authority::tryResolveAuthoredSupportGrip(
                input.weaponNode,
                authoredSupportHandInWeapon,
                authoredSupportFingerLocals,
                authoredSupportFingerMask,
                authoredSupportCaptureSequence) &&
            authoredSupportCaptureSequence > _supportCaptureSequenceFloor) {
            (void)weaponAuthority.setAuthoredSupportGripCandidate(
                input.weaponNode,
                authoredSupportHandInWeapon,
                authoredSupportFingerLocals,
                authoredSupportFingerMask,
                input.weaponGenerationKey,
                authoredSupportCaptureSequence);
        }

        if (!_sessionLogged) {
            ROCK_LOG_INFO(Animation,
                "Authored primary firing grip weapon alignment active weaponKey=0x{:X} generation=0x{:X} capture={} handMismatch={:.3f}gu weaponCorrection={:.3f}gu originalWeaponT=({:.3f},{:.3f},{:.3f}) alignedWeaponT=({:.3f},{:.3f},{:.3f}) alignedLocalT=({:.3f},{:.3f},{:.3f}) authority=weapon-only primaryHand=controller-driven physicalLeftSource=mirrored-authored-canonical",
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
