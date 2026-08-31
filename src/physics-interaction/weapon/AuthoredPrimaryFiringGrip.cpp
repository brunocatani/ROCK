#include "physics-interaction/weapon/AuthoredPrimaryFiringGrip.h"

#include "physics-interaction/animation/AuthoredWeaponGripCapture.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/grab/FrikWeaponOffsetCache.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/MinigunFiringGripPolicy.h"
#include "physics-interaction/weapon/TwoHandedGrip.h"

#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiTransform.h"

#include <algorithm>
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

        [[nodiscard]] float pointDistance(
            const RE::NiPoint3& lhs,
            const RE::NiPoint3& rhs)
        {
            const float x = lhs.x - rhs.x;
            const float y = lhs.y - rhs.y;
            const float z = lhs.z - rhs.z;
            return std::sqrt(x * x + y * y + z * z);
        }

        [[nodiscard]] float rotationMatrixMaxDelta(
            const RE::NiTransform& lhs,
            const RE::NiTransform& rhs)
        {
            float maximum = 0.0f;
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    maximum = (std::max)(
                        maximum,
                        std::abs(
                            lhs.rotate.entry[row][column] -
                            rhs.rotate.entry[row][column]));
                }
            }
            return maximum;
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

        [[nodiscard]] bool tryResolveCompiledMinigunFiringHandInWeapon(
            const bool inPowerArmor,
            RE::NiTransform& outHandInWeapon)
        {
            const auto& source =
                minigun_firing_grip_policy::weaponInFiringHand(
                    inPowerArmor);
            RE::NiTransform weaponInHand{};
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    weaponInHand.rotate.entry[row][column] =
                        source.rotation[static_cast<std::size_t>(
                            row * 3 + column)];
                }
            }
            weaponInHand.translate = {
                source.translation[0],
                source.translation[1],
                source.translation[2],
            };
            weaponInHand.scale = source.scale;
            if (!finiteTransform(weaponInHand)) {
                outHandInWeapon = {};
                return false;
            }

            // hFRIK authored Weapon-in-Hand. ROCK's firing canonical stores
            // the inverse Hand-in-Weapon relation.
            outHandInWeapon =
                transform_math::invertTransform(weaponInHand);
            return finiteTransform(outHandInWeapon);
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
        // The published candidate remains frame-scoped. Each eligible frame
        // must republish either a fresh capture or the identity-bound stable
        // snapshot, so every unrelated early return still falls back to the
        // ordinary dynamic path.
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
                clearStableAuthoredSupportGripSnapshot();
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
        RE::NiTransform compiledMinigunFiringHandInWeapon{};
        const bool compiledMinigunFiringSeat =
            minigun_firing_grip_policy::usesCompiledFiringSeat(
                input.weaponKeywordFlags) &&
            tryResolveCompiledMinigunFiringHandInWeapon(
                input.inPowerArmor,
                compiledMinigunFiringHandInWeapon);
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
                _customFrikOffsetOverrideActive =
                    customFrikOffset.found &&
                    !compiledMinigunFiringSeat;
                if (customFrikOffset.found) {
                    if (compiledMinigunFiringSeat) {
                        ROCK_LOG_INFO(Animation,
                            "Minigun custom hFRIK weapon offset promoted to compiled authored firing seat weaponKey=0x{:X} source={}",
                            currentWeaponKey,
                            customFrikOffset.reason);
                    } else {
                        ROCK_LOG_INFO(Animation,
                            "Authored primary firing grip yielded to custom hFRIK weapon offset weaponKey=0x{:X} source={}",
                            currentWeaponKey,
                            customFrikOffset.reason);
                    }
                }
            }
            /*
             * Continue when the exact weapon library already has a native-
             * idle relation. Returning here exposed one native/hFRIK frame on
             * every equip before ROCK could publish the known authored pose.
             * A stale live capture remains rejected by the sequence floor
             * below, while the library lookup is independently keyed by the
             * new weapon node, ownership, and variant.
             */
        }

        const auto frikOffsetCacheRevision = frik_weapon_offset_cache::currentRevision();
        if (frikOffsetCacheRevision != _frikOffsetCacheRevision) {
            const bool previousCustomOverride = _customFrikOffsetOverrideActive;
            _frikOffsetCacheRevision = frikOffsetCacheRevision;

            const auto customFrikOffset =
                frik_weapon_offset_cache::findCustomGripOverride(input.weapon, input.weaponNode);
            _customFrikOffsetOverrideActive =
                customFrikOffset.found &&
                !compiledMinigunFiringSeat;
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

        const auto variant = authored_weapon_grip_library::identifyWeaponVariant(
            input.weaponNode,
            input.weaponInstanceContentKey,
            input.weaponInstanceContentKnown);
        const auto authoredLookup = authored_weapon_grip_library::findResolvedVariant(input.weapon, variant, input.inPowerArmor);
        const bool harvestedRelationAvailable = authoredLookup.found && authored_weapon_grip_library::isNativeIdleAuthority(authoredLookup.source);
        const RE::NiTransform& selectedRightHandInWeapon =
            compiledMinigunFiringSeat ?
                compiledMinigunFiringHandInWeapon :
                authoredLookup.rightHandWeaponLocal;
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
            const auto& stable = _stableAuthoredSupportGrip;
            if (!authored_weapon_grip_capture_policy::
                    shouldReuseStableAuthoredSupportGrip(
                        authored_weapon_grip_capture_policy::
                            StableAuthoredSupportGripReuseInput{
                                .snapshotValid = stable.valid,
                                .weaponNodeValid = input.weaponNode != nullptr,
                                .weaponNodeMatches =
                                    stable.weaponNodeIdentity == input.weaponNode,
                                .currentWeaponOwnershipKey = currentWeaponKey,
                                .snapshotWeaponOwnershipKey =
                                    stable.weaponOwnershipKey,
                                .currentWeaponGenerationKey =
                                    input.weaponGenerationKey,
                                .snapshotWeaponGenerationKey =
                                    stable.weaponGenerationKey,
                                .currentPrimaryGripCaptureSequence =
                                    primaryGripCaptureSequence,
                                .snapshotPrimaryGripCaptureSequence =
                                    stable.primaryGripCaptureSequence,
                                .snapshotSupportGripCaptureSequence =
                                    stable.supportCaptureSequence,
                                .snapshotFingerLocalTransformMask =
                                    stable.fingerLocalTransformMask,
                            })) {
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
         * stay disabled. The harvested right canonical remains the exact
         * finger source; bind either its normal wrist or the compiled minigun
         * firing seat to the stable equipped identity. The authored support
         * relation remains independent and unchanged.
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
                        selectedRightHandInWeapon,
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
            .equippedWeaponTransitionActive =
                input.equippedWeaponTransitionActive,
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
        const auto authoredDecision =
            authored_weapon_grip_capture_policy::
                evaluateAuthoredPrimaryFiringGrip(eligibility);
        if (authoredDecision.action !=
            authored_weapon_grip_capture_policy::
                AuthoredPrimaryAction::Apply) {
            const bool retainRequested =
                authoredDecision.action ==
                authored_weapon_grip_capture_policy::
                    AuthoredPrimaryAction::RetainPoseOnly;
            const bool retainedForHandoff =
                retainRequested &&
                weaponAuthority.
                    retainAuthoredPrimaryFiringGripFingerPoseForHandoff(
                        input.weaponNode,
                        input.weaponGenerationKey,
                        currentWeaponKey);
            if (!input.rockFiringHandIsLeft && !retainedForHandoff) {
                weaponAuthority.clearAuthoredPrimaryFiringGripFingerPose();
            }
            endSession(
                retainRequested && !retainedForHandoff ?
                    "handoff-pose-unavailable" :
                    authored_weapon_grip_capture_policy::
                        authoredPrimaryDecisionReasonName(
                            authoredDecision.reason));
            return;
        }

        const RE::NiTransform liveWeaponWorld = input.weaponNode->world;
        RE::NiTransform trackedHandWorld{};
        // ROCK owns the presented right hand, so controller intent must come
        // from the physical driver frame. Reading the presented hand here
        // would feed ROCK's previous output back into the solve.
        if (!weaponAuthority.tryGetAuthoredPrimaryTrackedFiringHandWorld(
                trackedHandWorld)) {
            weaponAuthority.clearAuthoredPrimaryFiringGripFingerPose();
            endSession("physical-hand-frame-unavailable");
            return;
        }
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
            alignmentResolved =
                finiteTransform(authoredPrimaryHandInWeapon) &&
                finiteTransform(currentAuthoredHandWorld);
        } else {
            alignmentResolved =
                authored_weapon_grip_capture::
                    tryGetPrimaryFiringGripRelation(
                        input.weaponNode,
                        authoredPrimaryHandInWeapon,
                        resolvedCaptureSequence) &&
                resolvedCaptureSequence > _captureSequenceFloor;
            if (alignmentResolved) {
                currentAuthoredHandWorld =
                    transform_math::composeTransforms(
                        liveWeaponWorld,
                        authoredPrimaryHandInWeapon);
                alignmentResolved =
                    finiteTransform(currentAuthoredHandWorld);
            }
        }
        if (alignmentResolved && compiledMinigunFiringSeat) {
            authoredPrimaryHandInWeapon =
                compiledMinigunFiringHandInWeapon;
            currentAuthoredHandWorld =
                transform_math::composeTransforms(
                    liveWeaponWorld,
                    authoredPrimaryHandInWeapon);
            alignmentResolved =
                finiteTransform(currentAuthoredHandWorld);
        }
        if (!alignmentResolved) {
            weaponAuthority.clearAuthoredPrimaryFiringGripFingerPose();
            endSession("capture-resolution-failed");
            return;
        }

        /*
         * Plausibility gate: a mid-equip/mid-rebuild weapon node can sit
         * thousands of units from the hand for a few frames (observed
         * 9410gu leaving a weapon workbench). Applying that as a
         * "correction" hurls the weapon across the cell and flickers the
         * session. Normal carry mismatch stays under ~5gu; fail closed far
         * above that and log loudly until the transforms are sane again.
         */
        constexpr float kMaxPlausibleHandMismatchGameUnits = 50.0f;
        const float liveHandMismatch =
            translationDistance(trackedHandWorld, currentAuthoredHandWorld);
        if (!(liveHandMismatch <= kMaxPlausibleHandMismatchGameUnits)) {
            ROCK_LOG_SAMPLE_WARN(Animation, 1000,
                "Authored primary firing grip suspended on implausible hand mismatch weaponKey=0x{:X} mismatch={:.1f}gu bound={:.1f}gu",
                currentWeaponKey,
                liveHandMismatch,
                kMaxPlausibleHandMismatchGameUnits);
            weaponAuthority.clearAuthoredPrimaryFiringGripFingerPose();
            endSession("authored-hand-mismatch-implausible");
            return;
        }

        const RE::NiPoint3 authoredGripWeaponLocal =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(
                authoredPrimaryHandInWeapon,
                false);
        const RE::NiPoint3 trackedPalmWorld =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(
                trackedHandWorld,
                false);
        solvedWeaponWorld =
            authored_weapon_grip_capture_policy::
                resolveAuthoredPrimaryWeaponWorldPositionOnly(
                    liveWeaponWorld,
                    authoredGripWeaponLocal,
                    trackedPalmWorld,
                    [](const RE::NiTransform& transform,
                        const RE::NiPoint3& point) {
                        return transform_math::localPointToWorld(
                            transform,
                            point);
                    });
        if (!finiteTransform(solvedWeaponWorld)) {
            weaponAuthority.clearAuthoredPrimaryFiringGripFingerPose();
            endSession("position-only-alignment-invalid");
            return;
        }

        // The authored wrist correction belongs to the presented hand: the
        // weapon keeps its native rotation while the right hand seats at the
        // authored grip on that weapon frame.
        const RE::NiTransform solvedFiringHandWorld =
            transform_math::composeTransforms(
                solvedWeaponWorld,
                authoredPrimaryHandInWeapon);
        if (!finiteTransform(solvedFiringHandWorld)) {
            weaponAuthority.clearAuthoredPrimaryFiringGripFingerPose();
            endSession("position-only-hand-target-invalid");
            return;
        }

        if (!weaponAuthority.applyAuthoredPrimaryGripWeaponAlignment(
                input.weaponNode,
                solvedWeaponWorld,
                solvedFiringHandWorld,
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
                resolvedCaptureSequence,
                rightFingerPose,
                leftFingerPose)) {
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

        bool authoredLibraryEntryAvailable =
            !compiledMinigunFiringSeat && harvestedRelationAvailable;
        if (!compiledMinigunFiringSeat && !harvestedRelationAvailable) {
            authoredLibraryEntryAvailable =
                authored_weapon_grip_library::publishResolvedVariant(
                    input.weapon,
                    variant,
                    input.inPowerArmor,
                    authoredPrimaryHandInWeapon,
                    resolvedCaptureSequence,
                    authored_weapon_grip_library::CaptureSource::LiveEquippedGraph);
        }
        if (!compiledMinigunFiringSeat &&
            !authoredLibraryEntryAvailable) {
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

        const RE::NiPoint3 publishedGripWorld =
            transform_math::localPointToWorld(
                input.weaponNode->world,
                authoredGripWeaponLocal);
        ROCK_LOG_SAMPLE_DEBUG(
            Animation,
            1000,
            "Authored position-only trace weaponKey=0x{:X} generation=0x{:X} palmError={:.4f}gu rotationReadbackMaxDelta={:.7f} scaleDelta={:.7f}",
            currentWeaponKey,
            input.weaponGenerationKey,
            pointDistance(publishedGripWorld, trackedPalmWorld),
            rotationMatrixMaxDelta(
                liveWeaponWorld,
                input.weaponNode->world),
            std::abs(
                liveWeaponWorld.scale -
                input.weaponNode->world.scale));

        // A shot can temporarily suppress Bethesda's paired support-arm pass.
        // Keep the candidate frame-fresh by republishing the last verified
        // same-weapon snapshot instead of treating that animation gap as the
        // permanent loss of authored support-grip capability.
        if (!publishLiveAuthoredSupportCandidate(resolvedCaptureSequence)) {
            (void)publishStableAuthoredSupportCandidate(
                resolvedCaptureSequence);
        }

        if (!_sessionLogged) {
            ROCK_LOG_INFO(Animation,
                "Authored primary firing grip weapon alignment active weaponKey=0x{:X} generation=0x{:X} capture={} source={} exactFingerPose={} handMismatch={:.3f}gu "
                "weaponCorrection={:.3f}gu originalWeaponT=({:.3f},{:.3f},{:.3f}) alignedWeaponT=({:.3f},{:.3f},{:.3f}) alignedLocalT=({:.3f},{:.3f},{:.3f}) "
                "mode=position-only primaryHand=weapon-relative-authored physicalLeftSource=authored-seat-plus-native-weapon-aim",
                currentWeaponKey,
                input.weaponGenerationKey,
                resolvedCaptureSequence,
                compiledMinigunFiringSeat ?
                    "compiled-minigun-frik-seat" :
                    harvestedRelationAvailable ?
                        "native-idle-preharvest" :
                        "live-equipped-fallback",
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
