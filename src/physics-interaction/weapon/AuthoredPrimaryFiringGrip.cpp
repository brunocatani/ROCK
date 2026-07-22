#include "physics-interaction/weapon/AuthoredPrimaryFiringGrip.h"

#include "physics-interaction/animation/AuthoredWeaponGripCapture.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/grab/FrikWeaponOffsetCache.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/TwoHandedGrip.h"

#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiTransform.h"

#include <cmath>
#include <cstddef>

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

        [[nodiscard]] bool buildMirroredLeftFingerPose(const authored_weapon_grip_library::FiringFingerPose& rightPose, authored_weapon_grip_library::FiringFingerPose& outLeftPose)
        {
            outLeftPose = {};
            if (!rightPose.complete()) {
                return false;
            }

            frik_visual_authority::FingerLocalTransformOverride right{};
            right.enabledMask = rightPose.enabledMask;
            for (std::size_t index = 0; index < rightPose.localTransforms.size(); ++index) {
                right.localTransforms[index] = rightPose.localTransforms[index];
            }

            frik_visual_authority::FingerLocalTransformOverride left{};
            if (!frik_visual_authority::mirrorPrimaryWeaponFingerLocalTransforms(right, left) || left.enabledMask != authored_weapon_grip_library::kCompleteFiringFingerMask) {
                return false;
            }

            outLeftPose.enabledMask = left.enabledMask;
            for (std::size_t index = 0; index < outLeftPose.localTransforms.size(); ++index) {
                if (!finiteTransform(left.localTransforms[index])) {
                    outLeftPose = {};
                    return false;
                }
                outLeftPose.localTransforms[index] = left.localTransforms[index];
            }
            return true;
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

    void AuthoredPrimaryFiringGripRuntime::clearStableAuthoredSupportGripSnapshot()
    {
        _stableAuthoredSupportGrip = {};
    }

    void AuthoredPrimaryFiringGripRuntime::reset(
        const char* reason,
        TwoHandedGrip& weaponAuthority)
    {
        weaponAuthority.setAuthoredPrimaryFiringGripFingerPoseSuppressed(false);
        weaponAuthority.clearAuthoredPrimaryFiringGripCanonical(reason);
        endSession(reason);
        _weaponNodeIdentity = nullptr;
        _weaponOwnershipKey = 0;
        _frikOffsetCacheRevision = 0;
        _captureSequenceFloor = 0;
        _supportCaptureSequenceFloor = 0;
        clearStableAuthoredSupportGripSnapshot();
        _mirroredLeftFingerPose = {};
        _mirroredFingerPoseCaptureSequence = 0;
        _mirroredFingerPoseValid = false;
        _fingerMirrorFailureLogged = false;
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
            authored_weapon_grip_capture::queryPrimaryFiringGripCaptureStatus();
        const auto supportCaptureStatus =
            authored_weapon_grip_capture::queryAuthoredSupportGripCaptureStatus();

        weaponAuthority.setAuthoredPrimaryFiringGripFingerPoseSuppressed(input.nativeReloadAuthorityActive);

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
            clearStableAuthoredSupportGripSnapshot();
            _sessionLogged = false;
            _applyFailureLogged = false;
            _canonicalPublishFailureLogged = false;
            _libraryPublishFailureLogged = false;
            _supportCaptureFailureLogged = false;
            _mirroredLeftFingerPose = {};
            _mirroredFingerPoseCaptureSequence = 0;
            _mirroredFingerPoseValid = false;
            _fingerMirrorFailureLogged = false;

            _customFrikOffsetOverrideActive = false;
            _frikOffsetCacheRevision = frik_weapon_offset_cache::currentRevision();
            if (input.weapon && input.weaponNode) {
                const auto customFrikOffset =
                    frik_weapon_offset_cache::findCustomGripOverride(input.weapon, input.weaponNode);
                _customFrikOffsetOverrideActive = customFrikOffset.found;
                if (_customFrikOffsetOverrideActive) {
                    ROCK_LOG_INFO(Animation,
                        "Authored primary firing grip yielded to custom hFRIK weapon offset weaponKey=0x{:X} source={}",
                        currentWeaponKey,
                        customFrikOffset.reason);
                }
            }
            return;
        }

        const auto frikOffsetCacheRevision = frik_weapon_offset_cache::currentRevision();
        if (frikOffsetCacheRevision != _frikOffsetCacheRevision) {
            const bool previousCustomOverride = _customFrikOffsetOverrideActive;
            _frikOffsetCacheRevision = frikOffsetCacheRevision;

            const auto customFrikOffset =
                frik_weapon_offset_cache::findCustomGripOverride(input.weapon, input.weaponNode);
            _customFrikOffsetOverrideActive = customFrikOffset.found;
            if (_customFrikOffsetOverrideActive != previousCustomOverride) {
                ROCK_LOG_INFO(Animation,
                    "Authored primary firing grip custom hFRIK override {} weaponKey=0x{:X} cacheRevision={} source={}",
                    _customFrikOffsetOverrideActive ? "activated" : "released",
                    currentWeaponKey,
                    frikOffsetCacheRevision,
                    customFrikOffset.reason);

                weaponAuthority.clearAuthoredPrimaryFiringGripCanonical(
                    "custom-frik-weapon-offset-change");
                _captureSequenceFloor = captureStatus.captureSequence;
                _supportCaptureSequenceFloor = supportCaptureStatus.captureSequence;
                clearStableAuthoredSupportGripSnapshot();
                endSession("custom-frik-weapon-offset-change");
                return;
            }
        }

        if (_customFrikOffsetOverrideActive) {
            weaponAuthority.clearAuthoredPrimaryFiringGripCanonical(
                "custom-frik-weapon-offset");
            clearStableAuthoredSupportGripSnapshot();
            endSession("custom-frik-weapon-offset");
            return;
        }

        const auto authoredLookup = authored_weapon_grip_library::find(input.weapon, input.weaponNode, input.inPowerArmor);
        const bool harvestedRelationAvailable = authoredLookup.found && authoredLookup.source == authored_weapon_grip_library::CaptureSource::NativeIdlePreharvest;
        const auto* rightFingerPose = harvestedRelationAvailable && authoredLookup.rightFiringFingerPose.complete() ? &authoredLookup.rightFiringFingerPose : nullptr;
        if (rightFingerPose && _mirroredFingerPoseCaptureSequence != authoredLookup.captureSequence) {
            _mirroredLeftFingerPose = {};
            _mirroredFingerPoseValid = buildMirroredLeftFingerPose(*rightFingerPose, _mirroredLeftFingerPose);
            _mirroredFingerPoseCaptureSequence = authoredLookup.captureSequence;
            if (!_mirroredFingerPoseValid && !_fingerMirrorFailureLogged) {
                ROCK_LOG_WARN(Animation, "Authored primary firing grip could not mirror exact native-idle finger pose for left firing weaponKey=0x{:X} capture={}",
                    currentWeaponKey, authoredLookup.captureSequence);
                _fingerMirrorFailureLogged = true;
            }
        }
        const auto* leftFingerPose = rightFingerPose && _mirroredFingerPoseValid ? &_mirroredLeftFingerPose : nullptr;

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
                    authored_weapon_grip_capture::authoredSupportGripCaptureFailureReasonName(
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

        const auto publishLiveAuthoredSupportCandidate =
            [&](const std::uint64_t primaryGripCaptureSequence) {
            RE::NiTransform authoredSupportHandInWeapon{};
            std::array<RE::NiTransform, 15> authoredSupportFingerLocals{};
            std::uint16_t authoredSupportFingerMask = 0;
            std::uint64_t authoredSupportCaptureSequence = 0;
            if (!supportCaptureStatus.valid ||
                supportCaptureStatus.captureSequence <= _supportCaptureSequenceFloor ||
                !authored_weapon_grip_capture::tryResolveAuthoredSupportGrip(
                    input.weaponNode,
                    authoredSupportHandInWeapon,
                    authoredSupportFingerLocals,
                    authoredSupportFingerMask,
                    authoredSupportCaptureSequence) ||
                authoredSupportCaptureSequence <= _supportCaptureSequenceFloor) {
                return false;
            }

            if (!weaponAuthority.setAuthoredSupportGripCandidate(
                    input.weaponNode,
                    authoredSupportHandInWeapon,
                    authoredSupportFingerLocals,
                    authoredSupportFingerMask,
                    input.weaponGenerationKey,
                    authoredSupportCaptureSequence)) {
                return false;
            }

            _stableAuthoredSupportGrip = StableAuthoredSupportGripSnapshot{
                .weaponNodeIdentity = input.weaponNode,
                .handWeaponLocal = authoredSupportHandInWeapon,
                .fingerLocalTransforms = authoredSupportFingerLocals,
                .fingerLocalTransformMask = authoredSupportFingerMask,
                .weaponOwnershipKey = currentWeaponKey,
                .weaponGenerationKey = input.weaponGenerationKey,
                .primaryGripCaptureSequence = primaryGripCaptureSequence,
                .supportCaptureSequence = authoredSupportCaptureSequence,
                .valid = true,
            };
            return true;
        };

        const auto publishStableAuthoredSupportCandidate =
            [&](const std::uint64_t primaryGripCaptureSequence) {
            constexpr std::uint16_t kCompleteFingerLocalTransformMask = 0x7FFFu;
            const auto& stable = _stableAuthoredSupportGrip;
            if (!stable.valid ||
                !input.weaponNode ||
                stable.weaponNodeIdentity != input.weaponNode ||
                currentWeaponKey == 0 ||
                stable.weaponOwnershipKey != currentWeaponKey ||
                input.weaponGenerationKey == 0 ||
                stable.weaponGenerationKey != input.weaponGenerationKey ||
                primaryGripCaptureSequence == 0 ||
                stable.primaryGripCaptureSequence != primaryGripCaptureSequence ||
                stable.fingerLocalTransformMask !=
                    kCompleteFingerLocalTransformMask) {
                return false;
            }

            return weaponAuthority.setAuthoredSupportGripCandidate(
                input.weaponNode,
                stable.handWeaponLocal,
                stable.fingerLocalTransforms,
                stable.fingerLocalTransformMask,
                stable.weaponGenerationKey,
                stable.supportCaptureSequence);
        };

        /*
         * Physical-left firing already owns the weapon transform through
         * TwoHandedGrip, so the right-controller inverse alignment below must
         * stay disabled. The harvested right canonical is still the exact
         * weapon-relative source for both finger sets; bind it to the stable
         * equipped identity and publish only the mirrored left pose.
         */
        if (input.rockFiringHandIsLeft) {
            const bool canonicalReady =
                input.runtimeInitialized &&
                input.visualAuthorityAvailable &&
                input.localSkeletonReady &&
                !input.menuBlocking &&
                !input.compatibilityBlocking &&
                input.weaponDrawn &&
                input.weaponVisible &&
                currentWeaponKey != 0 &&
                input.weaponGenerationKey != 0 &&
                harvestedRelationAvailable &&
                rightFingerPose &&
                leftFingerPose;
            if (canonicalReady) {
                if (weaponAuthority.setAuthoredPrimaryFiringGripCanonical(
                        input.weaponNode,
                        authoredLookup.rightHandWeaponLocal,
                        input.weaponGenerationKey,
                        currentWeaponKey,
                        authoredLookup.captureSequence,
                        rightFingerPose,
                        leftFingerPose)) {
                    _canonicalPublishFailureLogged = false;
                    (void)weaponAuthority.publishAuthoredPrimaryFiringGripFingerPose(true);
                } else if (!_canonicalPublishFailureLogged) {
                    ROCK_LOG_WARN(Animation,
                        "Authored primary firing grip could not bind physical-left canonical weaponKey=0x{:X} generation=0x{:X} capture={}",
                        currentWeaponKey,
                        input.weaponGenerationKey,
                        authoredLookup.captureSequence);
                    _canonicalPublishFailureLogged = true;
                }
                const bool stableSupportPublished =
                    publishStableAuthoredSupportCandidate(
                        authoredLookup.captureSequence);
                if (!stableSupportPublished) {
                    const auto& stable = _stableAuthoredSupportGrip;
                    ROCK_LOG_SAMPLE_WARN(Animation, 2000,
                        "Authored physical-right support snapshot unavailable weaponKey=0x{:X} generation=0x{:X} canonical={} stable=(valid={} nodeMatch={} ownership=0x{:X} generation=0x{:X} canonical={} support={})",
                        currentWeaponKey,
                        input.weaponGenerationKey,
                        authoredLookup.captureSequence,
                        stable.valid,
                        stable.weaponNodeIdentity == input.weaponNode,
                        stable.weaponOwnershipKey,
                        stable.weaponGenerationKey,
                        stable.primaryGripCaptureSequence,
                        stable.supportCaptureSequence);
                }
            }
            endSession("physical-left-firing-canonical-only");
            return;
        }

        const authored_weapon_grip_capture_policy::AuthoredPrimaryFiringGripEligibility eligibility{
            .runtimeInitialized = input.runtimeInitialized,
            .visualAuthorityAvailable = input.visualAuthorityAvailable,
            .localSkeletonReady = input.localSkeletonReady,
            .menuBlocking = input.menuBlocking,
            .compatibilityBlocking = input.compatibilityBlocking,
            .weaponDrawn = input.weaponDrawn,
            .weaponVisible = input.weaponVisible,
            .weaponKeyValid = currentWeaponKey != 0,
            .captureValid = harvestedRelationAvailable || captureStatus.valid,
            .captureNewerThanWeaponBoundary = harvestedRelationAvailable || captureStatus.captureSequence > _captureSequenceFloor,
            .nativeReloadAuthorityActive = input.nativeReloadAuthorityActive,
            .conflictingWeaponTransformAuthorityActive =
                input.conflictingWeaponTransformAuthorityActive,
            .weaponVisualReturnActive = input.weaponVisualReturnActive,
            .primaryHandHoldingObject = input.primaryHandHoldingObject,
            .rockFiringHandIsLeft = input.rockFiringHandIsLeft,
        };
        if (!authored_weapon_grip_capture_policy::shouldApplyAuthoredPrimaryFiringGrip(eligibility)) {
            if (!input.rockFiringHandIsLeft) {
                weaponAuthority.clearAuthoredPrimaryFiringGripFingerPose();
            }
            endSession("frame-ineligible");
            return;
        }

        const RE::NiTransform liveWeaponWorld = input.weaponNode->world;
        const RE::NiTransform trackedHandWorld =
            frik_visual_authority::getHandWorldTransform(
                frik_visual_authority::Hand::Primary);
        if (!finiteTransform(liveWeaponWorld) || !finiteTransform(trackedHandWorld)) {
            weaponAuthority.clearAuthoredPrimaryFiringGripFingerPose();
            endSession("live-transform-invalid");
            return;
        }

        RE::NiTransform solvedWeaponWorld{};
        RE::NiTransform currentAuthoredHandWorld{};
        RE::NiTransform authoredPrimaryHandInWeapon{};
        std::uint64_t resolvedCaptureSequence = 0;
        bool alignmentResolved = false;
        if (harvestedRelationAvailable) {
            authoredPrimaryHandInWeapon = authoredLookup.rightHandWeaponLocal;
            resolvedCaptureSequence = authoredLookup.captureSequence;
            currentAuthoredHandWorld = transform_math::composeTransforms(liveWeaponWorld, authoredPrimaryHandInWeapon);
            solvedWeaponWorld = transform_math::composeTransforms(trackedHandWorld, transform_math::invertTransform(authoredPrimaryHandInWeapon));
            alignmentResolved = finiteTransform(authoredPrimaryHandInWeapon) && finiteTransform(currentAuthoredHandWorld) && finiteTransform(solvedWeaponWorld);
        } else {
            alignmentResolved = authored_weapon_grip_capture::tryResolvePrimaryFiringGripAlignment(
                input.weaponNode,
                liveWeaponWorld,
                trackedHandWorld,
                solvedWeaponWorld,
                currentAuthoredHandWorld,
                authoredPrimaryHandInWeapon,
                resolvedCaptureSequence) &&
                resolvedCaptureSequence > _captureSequenceFloor;
        }
        if (!alignmentResolved) {
            weaponAuthority.clearAuthoredPrimaryFiringGripFingerPose();
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
            weaponAuthority.clearAuthoredPrimaryFiringGripFingerPose();
            endSession("weapon-alignment-failed");
            return;
        }

        if (!weaponAuthority.setAuthoredPrimaryFiringGripCanonical(
                input.weaponNode,
                authoredPrimaryHandInWeapon,
                input.weaponGenerationKey,
                currentWeaponKey,
                resolvedCaptureSequence, rightFingerPose, leftFingerPose)) {
            if (!_canonicalPublishFailureLogged) {
                ROCK_LOG_WARN(Animation,
                    "Authored primary firing grip could not publish mirrored-left canonical weaponKey=0x{:X} generation=0x{:X} capture={}",
                    currentWeaponKey,
                    input.weaponGenerationKey,
                    resolvedCaptureSequence);
                _canonicalPublishFailureLogged = true;
            }
            weaponAuthority.clearAuthoredPrimaryFiringGripFingerPose();
        } else {
            _canonicalPublishFailureLogged = false;
            if (rightFingerPose) {
                (void)weaponAuthority.publishAuthoredPrimaryFiringGripFingerPose(false);
            } else {
                weaponAuthority.clearAuthoredPrimaryFiringGripFingerPose();
            }
        }

        if (!harvestedRelationAvailable &&
            !authored_weapon_grip_library::publish(
                input.weapon,
                input.weaponNode,
                input.inPowerArmor,
                authoredPrimaryHandInWeapon,
                resolvedCaptureSequence,
                authored_weapon_grip_library::CaptureSource::LiveEquippedGraph)) {
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

        (void)publishLiveAuthoredSupportCandidate(resolvedCaptureSequence);

        if (!_sessionLogged) {
            ROCK_LOG_INFO(Animation,
                "Authored primary firing grip weapon alignment active weaponKey=0x{:X} generation=0x{:X} capture={} source={} exactFingerPose={} handMismatch={:.3f}gu "
                "weaponCorrection={:.3f}gu originalWeaponT=({:.3f},{:.3f},{:.3f}) alignedWeaponT=({:.3f},{:.3f},{:.3f}) alignedLocalT=({:.3f},{:.3f},{:.3f}) authority=weapon-only "
                "primaryHand=controller-driven physicalLeftSource=mirrored-authored-canonical",
                currentWeaponKey,
                input.weaponGenerationKey,
                resolvedCaptureSequence, harvestedRelationAvailable ? "native-idle-preharvest" : "live-equipped-fallback",
                rightFingerPose ? (leftFingerPose ? "right-and-left" : "right-only") : "fallback",
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
