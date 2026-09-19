#include "physics-interaction/weapon/AuthoredPrimaryFiringGrip.h"
#include "physics-interaction/hand/HandFingerMirrorMath.h"

#include "physics-interaction/animation/AuthoredWeaponGripCapture.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/VanillaWeaponGripFrame.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/MinigunFiringGripPolicy.h"
#include "physics-interaction/weapon/TwoHandedGrip.h"
#include "physics-interaction/weapon/telemetry/VanillaWeaponAlignmentTelemetry.h"

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

        // Exact skeleton mirror of the authored right finger locals; the
        // bone order and therefore the mask are identical on both hands.
        [[nodiscard]] bool buildMirroredLeftFingerPose(const authored_weapon_grip_library::FiringFingerPose& rightPose, authored_weapon_grip_library::FiringFingerPose& outLeftPose)
        {
            outLeftPose = {};
            if (!rightPose.complete() ||
                !hand_finger_mirror_math::mirrorFingerLocalsAcrossHands<RE::NiTransform>(
                    std::span<const RE::NiTransform>(rightPose.localTransforms),
                    std::span<RE::NiTransform>(outLeftPose.localTransforms))) {
                outLeftPose = {};
                return false;
            }
            outLeftPose.enabledMask = rightPose.enabledMask;
            return true;
        }
    }

    void AuthoredPrimaryFiringGripRuntime::endSession(const char* reason)
    {
        const std::string_view suspensionReason{ reason ? reason : "unknown" };
        if (_reportPoseSuspension && suspensionReason != "weapon-boundary" && _lastSuspensionReason != suspensionReason) {
            const auto now = GetTickCount64();
            if (_lastSuspensionLogMs == 0 || now - _lastSuspensionLogMs >= 2000) {
                ROCK_LOG_INFO(Animation,
                    "Authored primary pose suspended formID={:08X} weaponKey={:016X} reason={} previouslyActive={}",
                    _weaponFormId, _weaponOwnershipKey, suspensionReason, _active);
                _lastSuspensionReason = suspensionReason;
                _lastSuspensionLogMs = now;
            }
        }
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
        _weaponFormId = 0;
        _lastSuspensionReason = {};
        _lastSuspensionLogMs = 0;
        _reportPoseSuspension = false;
        _captureSequenceFloor = 0;
        _supportCaptureSequenceFloor = 0;
        clearStableAuthoredSupportGripSnapshot();
        _liveSupportWitness = {};
        _mirroredLeftFingerPose = {};
        _mirroredFingerPoseCaptureSequence = 0;
        _mirroredFingerPoseValid = false;
        _fingerMirrorFailureLogged = false;
        _nativeReloadWasActive = false;
        _sessionLogged = false;
        _applyFailureLogged = false;
        _canonicalPublishFailureLogged = false;
        _libraryPublishFailureLogged = false;
        _positionOnlyHoldPublishFailureLogged = false;
        _supportCaptureFailureReasonLogged = 0;
        _supportCaptureFailureMaskLogged = 0;
        _supportCaptureFailureLogged = false;
    }

    void AuthoredPrimaryFiringGripRuntime::update(
        const AuthoredPrimaryFiringGripFrameInput& input,
        TwoHandedGrip& weaponAuthority)
    {
        vanilla_weapon_alignment_telemetry::recordInput(input);
        _weaponFormId = input.weapon ? input.weapon->formID : 0;
        _reportPoseSuspension = _weaponFormId != 0 && input.weaponDrawn && input.weaponVisible &&
            !input.menuBlocking && !input.compatibilityBlocking && !input.nativeReloadAuthorityActive &&
            !input.equippedWeaponTransitionActive && !input.primaryHandHoldingObject &&
            !input.weaponVisualReturnActive && !input.rockFiringHandIsLeft;
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
                _liveSupportWitness = {};
                const auto& stable = _stableAuthoredSupportGrip;
                const bool preserveLeftSupportSnapshot =
                    input.rockFiringHandIsLeft &&
                    stable.valid &&
                    stable.weaponNodeIdentity == input.weaponNode &&
                    input.weaponOwnershipKey != 0 &&
                    stable.weaponOwnershipKey ==
                        input.weaponOwnershipKey &&
                    stable.weaponInstanceContentKnown &&
                    input.weaponInstanceContentKnown &&
                    stable.weaponInstanceContentKey ==
                        input.weaponInstanceContentKey;
                if (!preserveLeftSupportSnapshot) {
                    clearStableAuthoredSupportGripSnapshot();
                }
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
            _liveSupportWitness = {};
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
            _lastSuspensionReason = {};
            _captureSequenceFloor = captureStatus.captureSequence;
            _supportCaptureSequenceFloor = supportCaptureStatus.captureSequence;
            _liveSupportWitness = {};
            /*
             * Keep a content-fingerprinted support snapshot across the
             * boundary: a sheath/retrieve or re-equip replaces node and
             * ownership while the weapon-local support relation stays valid
             * for identical instance content. Its stale scene witnesses keep
             * it unusable until adoption revalidates content, power-armor
             * topology, and the authored canonical against the new identity.
             * Content-unknown snapshots cannot be revalidated - clear them.
             */
            if (!_stableAuthoredSupportGrip.weaponInstanceContentKnown) {
                clearStableAuthoredSupportGripSnapshot();
            }
            _sessionLogged = false;
            _applyFailureLogged = false;
            _canonicalPublishFailureLogged = false;
            _libraryPublishFailureLogged = false;
            _positionOnlyHoldPublishFailureLogged = false;
            _supportCaptureFailureLogged = false;
            _mirroredLeftFingerPose = {};
            _mirroredFingerPoseCaptureSequence = 0;
            _mirroredFingerPoseValid = false;
            _fingerMirrorFailureLogged = false;

            /*
             * Continue when the exact weapon library already has a native-
             * idle relation. Returning here exposed one native/hFRIK frame on
             * every equip before ROCK could publish the known authored pose.
             * A stale live capture remains rejected by the sequence floor
             * below, while the library lookup is independently keyed by the
             * new weapon node, ownership, and variant.
             */
        }

        const auto variant = authored_weapon_grip_library::identifyWeaponVariant(
            input.weaponNode,
            input.weaponInstanceContentKey,
            input.weaponInstanceContentKnown);
        RE::NiPoint3 modelDisplacement{};
        if (!vanilla_weapon_grip_frame::resolveModelTranslation(
                input.weapon ? input.weapon->formID : 0, input.weaponNode, modelDisplacement)) {
            weaponAuthority.clearAuthoredPrimaryFiringGripCanonical("invalid-model-registration");
            weaponAuthority.clearAuthoredPrimaryFiringGripFingerPose();
            clearStableAuthoredSupportGripSnapshot();
            endSession("invalid-model-registration");
            return;
        }
        auto authoredLookup = authored_weapon_grip_library::findResolvedVariant(input.weapon, variant, input.inPowerArmor);
        authored_weapon_grip_library::applyPipeDefaultOffset(authoredLookup, input.rockFiringHandIsLeft);
        vanilla_weapon_alignment_telemetry::recordAuthoredSelection(
            input, variant, authoredLookup, modelDisplacement, compiledMinigunFiringSeat);
        if (authoredLookup.found) {
            authoredLookup.rightHandWeaponLocal = vanilla_weapon_grip_frame::translateGrip(authoredLookup.rightHandWeaponLocal, modelDisplacement);
            if (authoredLookup.hasSupportRelation) {
                authoredLookup.supportHandWeaponLocal = vanilla_weapon_grip_frame::translateGrip(authoredLookup.supportHandWeaponLocal, modelDisplacement);
            }
        }
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

        /*
         * Support-relation precedence. A native-idle support relation
         * (sampled from the idle clip by the preharvest, or restored from
         * its disk record) is the settled authored pose and is published
         * directly, in both hand topologies, from frame zero. The live
         * equipped-graph capture and its stable snapshot are only the
         * fallback for a clip the preharvest could not serve.
         */
        const bool librarySupportAuthoritative =
            harvestedRelationAvailable &&
            authoredLookup.hasSupportRelation &&
            authored_weapon_grip_library::isNativeIdleAuthority(
                authoredLookup.supportSource) &&
            authoredLookup.supportCaptureSequence != 0 &&
            authoredLookup.supportFingerPose.complete();
        const auto publishLibraryAuthoredSupportCandidate = [&]() {
            if (harvestedRelationAvailable && authoredLookup.supportPoseAbsent &&
                authored_weapon_grip_library::isNativeIdleAuthority(authoredLookup.source)) {
                _stableAuthoredSupportGrip = {};
                return weaponAuthority.setAuthoredSupportGripAbsent(input.weaponNode,
                    input.weaponGenerationKey, authoredLookup.captureSequence);
            }
            if (!librarySupportAuthoritative ||
                !input.weaponNode ||
                input.weaponGenerationKey == 0) {
                return false;
            }
            return weaponAuthority.setAuthoredSupportGripCandidate(
                input.weaponNode,
                authoredLookup.supportHandWeaponLocal,
                authoredLookup.supportFingerPose.localTransforms,
                authoredLookup.supportFingerPose.enabledMask,
                input.weaponGenerationKey,
                authoredLookup.supportCaptureSequence);
        };
        // Runtime evidence that the clip-sampled relation matches what the
        // native right-primary graph settles to; sampled, debug only.
        const auto traceLiveSupportAgainstLibrary = [&]() {
            RE::NiTransform liveSupportHandInWeapon{};
            std::array<RE::NiTransform, 15> liveFingerLocals{};
            std::uint16_t liveFingerMask = 0;
            std::uint64_t liveCaptureSequence = 0;
            if (!supportCaptureStatus.valid ||
                supportCaptureStatus.captureSequence <= _supportCaptureSequenceFloor ||
                !authored_weapon_grip_capture::tryResolveAuthoredSupportGrip(
                    input.weaponNode,
                    liveSupportHandInWeapon,
                    liveFingerLocals,
                    liveFingerMask,
                    liveCaptureSequence)) {
                return;
            }
            liveSupportHandInWeapon = vanilla_weapon_grip_frame::translateGrip(liveSupportHandInWeapon, modelDisplacement);
            ROCK_LOG_SAMPLE_DEBUG(Animation, 2000,
                "Authored support library-vs-live trace weaponKey=0x{:X} libraryT=({:.3f},{:.3f},{:.3f}) liveT=({:.3f},{:.3f},{:.3f}) deltaT={:.3f}gu valueMatch={}",
                currentWeaponKey,
                authoredLookup.supportHandWeaponLocal.translate.x,
                authoredLookup.supportHandWeaponLocal.translate.y,
                authoredLookup.supportHandWeaponLocal.translate.z,
                liveSupportHandInWeapon.translate.x,
                liveSupportHandInWeapon.translate.y,
                liveSupportHandInWeapon.translate.z,
                translationDistance(
                    authoredLookup.supportHandWeaponLocal,
                    liveSupportHandInWeapon),
                authored_weapon_grip_authority_policy::handRelationValueMatches(
                    authoredLookup.supportHandWeaponLocal,
                    liveSupportHandInWeapon) ? "yes" : "no");
        };

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
            [&](const std::uint64_t primaryGripCaptureSequence,
                const RE::NiTransform& canonicalHandWeaponLocal) {
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

            const auto rawSupportHandInWeapon = authoredSupportHandInWeapon;
            authoredSupportHandInWeapon = vanilla_weapon_grip_frame::translateGrip(authoredSupportHandInWeapon, modelDisplacement);
            if (!weaponAuthority.setAuthoredSupportGripCandidate(
                    input.weaponNode,
                    authoredSupportHandInWeapon,
                    authoredSupportFingerLocals,
                    authoredSupportFingerMask,
                    input.weaponGenerationKey,
                    authoredSupportCaptureSequence)) {
                return false;
            }

            // The per-frame candidate above is published unconditionally.
            // The snapshot and the library only take a converged run: an
            // equip/draw blend sweeps the graph's left arm through space
            // for several frames, and the takeover frame of a direct
            // physical-left equip is the first of them.
            const bool valueMatchesAnchor =
                _liveSupportWitness.agreeingFrames != 0 &&
                authored_weapon_grip_authority_policy::
                    handRelationValueMatches(
                        _liveSupportWitness.anchorHandWeaponLocal,
                        authoredSupportHandInWeapon);
            _liveSupportWitness.agreeingFrames =
                authored_weapon_grip_capture_policy::
                    advanceStableAuthoredSupportCaptureWitness(
                        _liveSupportWitness.agreeingFrames,
                        valueMatchesAnchor);
            if (!valueMatchesAnchor) {
                _liveSupportWitness.anchorHandWeaponLocal =
                    authoredSupportHandInWeapon;
            }
            if (!authored_weapon_grip_capture_policy::
                    stableAuthoredSupportCaptureConverged(
                        _liveSupportWitness.agreeingFrames)) {
                return true;
            }

            _stableAuthoredSupportGrip = StableAuthoredSupportGripSnapshot{
                .weaponNodeIdentity = input.weaponNode,
                .handWeaponLocal = authoredSupportHandInWeapon,
                .fingerLocalTransforms = authoredSupportFingerLocals,
                .fingerLocalTransformMask = authoredSupportFingerMask,
                .weaponOwnershipKey = currentWeaponKey,
                .weaponGenerationKey = input.weaponGenerationKey,
                .weaponInstanceContentKey =
                    input.weaponInstanceContentKey,
                .primaryGripCaptureSequence = primaryGripCaptureSequence,
                .supportCaptureSequence = authoredSupportCaptureSequence,
                .canonicalHandWeaponLocal = canonicalHandWeaponLocal,
                .inPowerArmor = input.inPowerArmor,
                .weaponInstanceContentKnown =
                    input.weaponInstanceContentKnown,
                .valid = true,
            };
            // Mirror the converged relation into the authored library so a
            // later direct physical-left equip of the same content can seat
            // authored support grabs without a native-right frame. The
            // library keeps a native-idle relation over this live one and
            // dedups value-equivalent republication.
            if (input.weaponInstanceContentKnown) {
                authored_weapon_grip_library::FiringFingerPose supportPose{};
                supportPose.localTransforms = authoredSupportFingerLocals;
                supportPose.enabledMask = authoredSupportFingerMask;
                (void)authored_weapon_grip_library::publishSupportRelation(
                    input.weapon,
                    variant,
                    input.inPowerArmor,
                    rawSupportHandInWeapon,
                    supportPose,
                    authoredSupportCaptureSequence,
                    authored_weapon_grip_library::CaptureSource::LiveEquippedGraph);
            }
            return true;
        };

        const auto canonicalRelationMatchesSnapshot =
            [&](const std::uint64_t primaryGripCaptureSequence,
                const RE::NiTransform& currentCanonical) {
            const auto& stable = _stableAuthoredSupportGrip;
            if (primaryGripCaptureSequence == 0 ||
                stable.primaryGripCaptureSequence == 0) {
                return false;
            }
            return primaryGripCaptureSequence ==
                       stable.primaryGripCaptureSequence ||
                   authored_weapon_grip_authority_policy::
                       handRelationValueMatches(
                           stable.canonicalHandWeaponLocal,
                           currentCanonical);
        };

        const auto rebindStableAuthoredSupportCandidate =
            [&](const std::uint64_t primaryGripCaptureSequence,
                const RE::NiTransform& currentCanonical) {
            auto& stable = _stableAuthoredSupportGrip;
            if (!authored_weapon_grip_capture_policy::
                    shouldRebindStableAuthoredSupportGrip(
                        authored_weapon_grip_capture_policy::
                            StableAuthoredSupportGripRebindInput{
                                .snapshotValid = stable.valid,
                                .weaponNodeValid =
                                    input.weaponNode != nullptr,
                                .weaponNodeMatches =
                                    stable.weaponNodeIdentity ==
                                    input.weaponNode,
                                .currentWeaponOwnershipKey =
                                    currentWeaponKey,
                                .snapshotWeaponOwnershipKey =
                                    stable.weaponOwnershipKey,
                                .currentWeaponGenerationKey =
                                    input.weaponGenerationKey,
                                .snapshotWeaponGenerationKey =
                                    stable.weaponGenerationKey,
                                .currentWeaponInstanceContentKnown =
                                    input.weaponInstanceContentKnown,
                                .snapshotWeaponInstanceContentKnown =
                                    stable.weaponInstanceContentKnown,
                                .currentWeaponInstanceContentKey =
                                    input.weaponInstanceContentKey,
                                .snapshotWeaponInstanceContentKey =
                                    stable.weaponInstanceContentKey,
                                .canonicalRelationMatches =
                                    canonicalRelationMatchesSnapshot(
                                        primaryGripCaptureSequence,
                                        currentCanonical),
                                .powerArmorMatches =
                                    stable.inPowerArmor ==
                                    input.inPowerArmor,
                                .snapshotSupportGripCaptureSequence =
                                    stable.supportCaptureSequence,
                                .snapshotFingerLocalTransformMask =
                                    stable.fingerLocalTransformMask,
                            })) {
                return false;
            }

            const std::uint64_t previousGeneration =
                stable.weaponGenerationKey;
            stable.weaponGenerationKey = input.weaponGenerationKey;
            ROCK_LOG_INFO(
                Animation,
                "Authored left-carry support snapshot rebound across same-content generation old=0x{:X} new=0x{:X} weaponKey=0x{:X} content=0x{:X}",
                previousGeneration,
                stable.weaponGenerationKey,
                currentWeaponKey,
                stable.weaponInstanceContentKey);
            return true;
        };

        const auto adoptStableAuthoredSupportCandidate =
            [&](const std::uint64_t primaryGripCaptureSequence,
                const RE::NiTransform& currentCanonical) {
            auto& stable = _stableAuthoredSupportGrip;
            if (stable.weaponNodeIdentity == input.weaponNode &&
                stable.weaponOwnershipKey == currentWeaponKey &&
                stable.weaponGenerationKey == input.weaponGenerationKey) {
                return false;
            }
            if (!authored_weapon_grip_capture_policy::
                    shouldAdoptStableAuthoredSupportGrip(
                        authored_weapon_grip_capture_policy::
                            StableAuthoredSupportGripAdoptInput{
                                .snapshotValid = stable.valid,
                                .weaponNodeValid =
                                    input.weaponNode != nullptr,
                                .currentWeaponOwnershipKey =
                                    currentWeaponKey,
                                .currentWeaponGenerationKey =
                                    input.weaponGenerationKey,
                                .currentWeaponInstanceContentKnown =
                                    input.weaponInstanceContentKnown,
                                .snapshotWeaponInstanceContentKnown =
                                    stable.weaponInstanceContentKnown,
                                .currentWeaponInstanceContentKey =
                                    input.weaponInstanceContentKey,
                                .snapshotWeaponInstanceContentKey =
                                    stable.weaponInstanceContentKey,
                                .canonicalRelationMatches =
                                    canonicalRelationMatchesSnapshot(
                                        primaryGripCaptureSequence,
                                        currentCanonical),
                                .powerArmorMatches =
                                    stable.inPowerArmor ==
                                    input.inPowerArmor,
                                .snapshotSupportGripCaptureSequence =
                                    stable.supportCaptureSequence,
                                .snapshotFingerLocalTransformMask =
                                    stable.fingerLocalTransformMask,
                            })) {
                return false;
            }

            stable.weaponNodeIdentity = input.weaponNode;
            stable.weaponOwnershipKey = currentWeaponKey;
            stable.weaponGenerationKey = input.weaponGenerationKey;
            stable.primaryGripCaptureSequence = primaryGripCaptureSequence;
            stable.canonicalHandWeaponLocal = currentCanonical;
            ROCK_LOG_INFO(
                Animation,
                "Authored support snapshot adopted across weapon boundary weaponKey=0x{:X} generation=0x{:X} content=0x{:X} canonical={}",
                currentWeaponKey,
                stable.weaponGenerationKey,
                stable.weaponInstanceContentKey,
                primaryGripCaptureSequence);
            return true;
        };

        const auto seedStableAuthoredSupportFromLibrary =
            [&](const std::uint64_t primaryGripCaptureSequence,
                const RE::NiTransform& currentCanonical) {
            const auto& stable = _stableAuthoredSupportGrip;
            const bool snapshotOwnsCurrentContent =
                stable.valid &&
                stable.weaponInstanceContentKnown &&
                input.weaponInstanceContentKnown &&
                stable.weaponInstanceContentKey ==
                    input.weaponInstanceContentKey;
            if (snapshotOwnsCurrentContent ||
                !authoredLookup.found ||
                !authoredLookup.hasSupportRelation ||
                !input.weaponNode ||
                currentWeaponKey == 0 ||
                input.weaponGenerationKey == 0 ||
                primaryGripCaptureSequence == 0 ||
                !input.weaponInstanceContentKnown ||
                authoredLookup.supportCaptureSequence == 0 ||
                !authoredLookup.supportFingerPose.complete()) {
                return false;
            }

            _stableAuthoredSupportGrip = StableAuthoredSupportGripSnapshot{
                .weaponNodeIdentity = input.weaponNode,
                .handWeaponLocal =
                    authoredLookup.supportHandWeaponLocal,
                .fingerLocalTransforms =
                    authoredLookup.supportFingerPose.localTransforms,
                .fingerLocalTransformMask =
                    authoredLookup.supportFingerPose.enabledMask,
                .weaponOwnershipKey = currentWeaponKey,
                .weaponGenerationKey = input.weaponGenerationKey,
                .weaponInstanceContentKey =
                    input.weaponInstanceContentKey,
                .primaryGripCaptureSequence = primaryGripCaptureSequence,
                .supportCaptureSequence =
                    authoredLookup.supportCaptureSequence,
                .canonicalHandWeaponLocal = currentCanonical,
                .inPowerArmor = input.inPowerArmor,
                .weaponInstanceContentKnown = true,
                .valid = true,
            };
            ROCK_LOG_INFO(
                Animation,
                "Authored support snapshot seeded from library weaponKey=0x{:X} generation=0x{:X} content=0x{:X} supportCapture={}",
                currentWeaponKey,
                input.weaponGenerationKey,
                input.weaponInstanceContentKey,
                authoredLookup.supportCaptureSequence);
            return true;
        };

        const auto publishStableAuthoredSupportCandidate =
            [&](const std::uint64_t primaryGripCaptureSequence,
                const RE::NiTransform& currentCanonical) {
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
                                .canonicalRelationMatches =
                                    canonicalRelationMatchesSnapshot(
                                        primaryGripCaptureSequence,
                                        currentCanonical),
                                .powerArmorMatches =
                                    stable.inPowerArmor ==
                                    input.inPowerArmor,
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
                        input.weaponInstanceContentKnown ? input.weaponInstanceContentKey : 0,
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
                bool stableSupportPublished =
                    publishLibraryAuthoredSupportCandidate();
                if (!stableSupportPublished) {
                    (void)rebindStableAuthoredSupportCandidate(
                        authoredLookup.captureSequence,
                        selectedRightHandInWeapon);
                    stableSupportPublished =
                        publishStableAuthoredSupportCandidate(
                            authoredLookup.captureSequence,
                            selectedRightHandInWeapon);
                }
                // A replaced weapon identity (sheath/retrieve, re-equip)
                // first tries to adopt the retained same-content snapshot,
                // then falls back to the library relation converged from an
                // earlier native-right carry.
                if (!stableSupportPublished &&
                    adoptStableAuthoredSupportCandidate(
                        authoredLookup.captureSequence,
                        selectedRightHandInWeapon)) {
                    stableSupportPublished =
                        publishStableAuthoredSupportCandidate(
                            authoredLookup.captureSequence,
                            selectedRightHandInWeapon);
                }
                if (!stableSupportPublished &&
                    seedStableAuthoredSupportFromLibrary(
                        authoredLookup.captureSequence,
                        selectedRightHandInWeapon)) {
                    stableSupportPublished =
                        publishStableAuthoredSupportCandidate(
                            authoredLookup.captureSequence,
                            selectedRightHandInWeapon);
                }
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

        RE::NiTransform liveWeaponWorld{};
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
        if (!weaponAuthority.tryGetRightWeaponAimWorld(input.weaponNode->world.scale, liveWeaponWorld) ||
            !finiteTransform(trackedHandWorld)) {
            weaponAuthority.clearAuthoredPrimaryFiringGripFingerPose();
            endSession("live-transform-invalid");
            return;
        }

        RE::NiTransform solvedWeaponWorld{};
        RE::NiTransform currentAuthoredHandWorld{};
        RE::NiTransform authoredPrimaryHandInWeapon{};
        RE::NiTransform rawPrimaryHandInWeapon{};
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
        if (alignmentResolved && !harvestedRelationAvailable && !compiledMinigunFiringSeat) {
            rawPrimaryHandInWeapon = authoredPrimaryHandInWeapon;
            authoredPrimaryHandInWeapon = vanilla_weapon_grip_frame::translateGrip(authoredPrimaryHandInWeapon, modelDisplacement);
            currentAuthoredHandWorld = transform_math::composeTransforms(liveWeaponWorld, authoredPrimaryHandInWeapon);
            alignmentResolved = finiteTransform(currentAuthoredHandWorld);
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

        vanilla_weapon_alignment_telemetry::recordSolve(
            input.weapon ? input.weapon->formID : 0, resolvedCaptureSequence,
            harvestedRelationAvailable ? "native-idle" : "live-equipped",
            authoredPrimaryHandInWeapon, trackedHandWorld, solvedWeaponWorld);

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
                input.weaponInstanceContentKnown ? input.weaponInstanceContentKey : 0,
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
                    rawPrimaryHandInWeapon,
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

        /*
         * The authored relation above remains the presented wrist/finger
         * authority. Loose weapon placement needs a different relation: the
         * physical hand measured against the final position-only equipped
         * weapon. Miniguns retain hFRIK's independent loose-weapon authority
         * instead of contaminating the native-idle library with this compiled
         * equipped-only firing seat.
         */
        const RE::NiTransform rightPositionOnlyHandWeaponLocal =
            transform_math::composeTransforms(
                transform_math::invertTransform(solvedWeaponWorld),
                trackedHandWorld);
        const RE::NiPoint3 cachedPositionOnlyGripWeaponLocal =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(
                rightPositionOnlyHandWeaponLocal,
                false);
        // Float rounding of the world coordinates the pivot went through
        // (AuthoredWeaponGripCapturePolicy.h); the identity itself is exact.
        const float positionOnlyHoldGripErrorTolerance =
            authored_weapon_grip_capture_policy::positionOnlyHoldGripErrorTolerance(
                (std::max)({ std::fabs(solvedWeaponWorld.translate.x),
                    std::fabs(solvedWeaponWorld.translate.y),
                    std::fabs(solvedWeaponWorld.translate.z),
                    std::fabs(trackedHandWorld.translate.x),
                    std::fabs(trackedHandWorld.translate.y),
                    std::fabs(trackedHandWorld.translate.z) }));
        const float positionOnlyHoldGripError = pointDistance(
            cachedPositionOnlyGripWeaponLocal,
            authoredGripWeaponLocal);
        const bool positionOnlyHoldValid =
            finiteTransform(rightPositionOnlyHandWeaponLocal) &&
            std::isfinite(positionOnlyHoldGripError) &&
            positionOnlyHoldGripError <= positionOnlyHoldGripErrorTolerance;
        const bool positionOnlyHoldPublished =
            compiledMinigunFiringSeat ||
            (authoredLibraryEntryAvailable &&
                positionOnlyHoldValid &&
                authored_weapon_grip_library::publishPositionOnlyHold(
                    input.weapon,
                    input.inPowerArmor,
                    resolvedCaptureSequence,
                    rightPositionOnlyHandWeaponLocal));
        if (!positionOnlyHoldPublished) {
            if (!_positionOnlyHoldPublishFailureLogged) {
                ROCK_LOG_WARN(Animation,
                    "Authored position-only loose hold publication failed weaponKey=0x{:X} capture={} entry={} valid={} gripError={:.4f}gu tolerance={:.4f}gu",
                    currentWeaponKey,
                    resolvedCaptureSequence,
                    authoredLibraryEntryAvailable ? "yes" : "no",
                    positionOnlyHoldValid ? "yes" : "no",
                    positionOnlyHoldGripError,
                    positionOnlyHoldGripErrorTolerance);
                _positionOnlyHoldPublishFailureLogged = true;
            }
        } else {
            _positionOnlyHoldPublishFailureLogged = false;
            if (!compiledMinigunFiringSeat) {
                ROCK_LOG_SAMPLE_DEBUG(Animation, 1000,
                    "Authored position-only loose hold cached weaponKey=0x{:X} capture={} gripError={:.4f}gu",
                    currentWeaponKey,
                    resolvedCaptureSequence,
                    positionOnlyHoldGripError);
            }
        }

        _active = true;
        _lastSuspensionReason = {};
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
        if (publishLibraryAuthoredSupportCandidate()) {
            traceLiveSupportAgainstLibrary();
        } else if (!publishLiveAuthoredSupportCandidate(
                       resolvedCaptureSequence,
                       authoredPrimaryHandInWeapon)) {
            bool stableSupportPublished =
                publishStableAuthoredSupportCandidate(
                    resolvedCaptureSequence,
                    authoredPrimaryHandInWeapon);
            if (!stableSupportPublished &&
                adoptStableAuthoredSupportCandidate(
                    resolvedCaptureSequence,
                    authoredPrimaryHandInWeapon)) {
                stableSupportPublished =
                    publishStableAuthoredSupportCandidate(
                        resolvedCaptureSequence,
                        authoredPrimaryHandInWeapon);
            }
            if (!stableSupportPublished &&
                seedStableAuthoredSupportFromLibrary(
                    resolvedCaptureSequence,
                    authoredPrimaryHandInWeapon)) {
                (void)publishStableAuthoredSupportCandidate(
                    resolvedCaptureSequence,
                    authoredPrimaryHandInWeapon);
            }
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
