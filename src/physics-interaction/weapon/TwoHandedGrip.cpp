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
#include "physics-interaction/visual/PreFrikHandAuthorityPolicy.h"
#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <string_view>
#include <vector>

namespace rock
{
    using two_handed_grip_detail::applyNativeScopeCameraWorldTarget;
    using two_handed_grip_detail::arePointsNearlyEqual;
    using two_handed_grip_detail::areTransformsNearlyEqual;
    using two_handed_grip_detail::AuthoredSupportPalmSeatProximity;
    using two_handed_grip_detail::AUTHORED_PRIMARY_POSE_BLOCK_TAG;
    using two_handed_grip_detail::buildFullHandLocalTransformsForMeshPose;
    using two_handed_grip_detail::buildProviderPartTargetQuery;
    using two_handed_grip_detail::captureNativeScopeCameraFollow;
    using two_handed_grip_detail::captureScopeHandAuthorityCleanupVisuals;
    using two_handed_grip_detail::configuredGunstockFineTune;
    using two_handed_grip_detail::currentWeaponOppositionPocketConfig;
    using two_handed_grip_detail::DYNAMIC_SUPPORT_MINIMUM_SMOOTHED_ROTATION_RADIANS;
    using two_handed_grip_detail::evaluateAuthoredSupportGripDirectionGate;
    using two_handed_grip_detail::GRIP_HAND_POSE_PRIORITY;
    using two_handed_grip_detail::GUNSTOCK_ALIGNMENT_TAG;
    using two_handed_grip_detail::isFiniteTransform;
    using two_handed_grip_detail::isInvertibleTransform;
    using two_handed_grip_detail::isUsableHandAuthorityTransform;
    using two_handed_grip_detail::kSupportGripFingerLaneCount;
    using two_handed_grip_detail::kSupportGripFingerLaneReferenceCapacity;
    using two_handed_grip_detail::kSupportGripGlobalRankingIndex;
    using two_handed_grip_detail::leftFiringInfrastructureAvailable;
    using two_handed_grip_detail::lerpPoint;
    using two_handed_grip_detail::makeNativeScopeCameraDebugSnapshot;
    using two_handed_grip_detail::moveWeaponPresentationRigidly;
    using two_handed_grip_detail::NativeScopeCameraFollowCapture;
    using two_handed_grip_detail::NativeScopeCameraFollowResult;
    using two_handed_grip_detail::orthonormalizeStoredRotation;
    using two_handed_grip_detail::PRIMARY_DETACH_TAG;
    using two_handed_grip_detail::PRIMARY_GRIP_TAG;
    using two_handed_grip_detail::RankedSupportGripTriangle;
    using two_handed_grip_detail::resolveAuthoredSupportPalmSeatProximity;
    using two_handed_grip_detail::resolveAuthoredSupportPalmSeatProximityFromPoints;
    using two_handed_grip_detail::restoreScopeHandAuthorityCleanupVisuals;
    using two_handed_grip_detail::restoreWeaponRootPreservingPresentedDescendants;
    using two_handed_grip_detail::RETURN_HAND_TAG;
    using two_handed_grip_detail::RETURN_HAND_VISUAL_PRIORITY;
    using two_handed_grip_detail::rootFlattenedTwoHandedReader;
    using two_handed_grip_detail::SCOPE_DRIVER_MISS_GRACE_FRAMES;
    using two_handed_grip_detail::SCOPE_ROOT_REBASE_DURATION_SECONDS;
    using two_handed_grip_detail::SCOPE_TRANSITION_TRACE_FRAMES;
    using two_handed_grip_detail::ScopeHandAuthorityCleanupVisualSnapshot;
    using two_handed_grip_detail::selectNearestSupportGripFingerTriangles;
    using two_handed_grip_detail::SUPPORT_GRIP_TAG;
    using two_handed_grip_detail::SUPPORT_NORMAL_TWIST_FACTOR;
    using two_handed_grip_detail::SupportGripFingerReferenceSet;
    using two_handed_grip_detail::tryGetRootFlattenedHandBoneTransform;
    using two_handed_grip_detail::tryResolveWeaponRootLocal;
    using two_handed_grip_detail::WEAPON_COLLISION_HAND_PRIORITY;
    using two_handed_grip_detail::WEAPON_COLLISION_HAND_TAG;
    using two_handed_grip_detail::WEAPON_NODE_OWNERSHIP_TAG;
    using two_handed_grip_detail::WEAPON_OPPOSITION_MAX_FINGER_GAP_GAME_UNITS;
    using two_handed_grip_detail::WEAPON_OPPOSITION_SEGMENT_PROBE_RADIUS_GAME_UNITS;
    using two_handed_grip_detail::WEAPON_RECOIL_CONTROLLER_TAG;

    namespace
    {
        RE::NiNode* sourceRootNodeOrFallback(
            RE::NiAVObject* sourceRoot,
            RE::NiNode* fallback)
        {
            if (sourceRoot) {
                if (auto* sourceNode = sourceRoot->IsNode()) {
                    return sourceNode;
                }
            }
            return fallback;
        }

        struct MirroredDriverConjugation
        {
            RE::NiTransform sourceWeaponInDriver{};
            RE::NiTransform targetWeaponInDriver{};
            RE::NiTransform targetHandWeaponLocal{};
        };

        [[nodiscard]] bool tryBuildMirroredDriverConjugation(
            const RE::NiTransform& sourceHandWeaponLocal,
            const RE::NiTransform& sourceBoneInDriver,
            const RE::NiTransform& targetBoneInDriver,
            const RE::NiTransform* targetDriverTrim,
            MirroredDriverConjugation& out)
        {
            out = {};
            RE::NiTransform lateralMirror{};
            lateralMirror.MakeIdentity();
            lateralMirror.rotate.entry[0][0] = -1.0f;

            out.sourceWeaponInDriver = transform_math::composeTransforms(
                sourceBoneInDriver,
                transform_math::invertTransform(
                    sourceHandWeaponLocal));
            out.targetWeaponInDriver = transform_math::composeTransforms(
                lateralMirror,
                transform_math::composeTransforms(
                    out.sourceWeaponInDriver,
                    lateralMirror));
            if (targetDriverTrim) {
                out.targetWeaponInDriver =
                    transform_math::composeTransforms(
                        *targetDriverTrim,
                        out.targetWeaponInDriver);
            }

            const RE::NiTransform weaponInTargetHand =
                transform_math::composeTransforms(
                    transform_math::invertTransform(targetBoneInDriver),
                    out.targetWeaponInDriver);
            out.targetHandWeaponLocal =
                transform_math::invertTransform(weaponInTargetHand);
            return isFiniteTransform(out.targetHandWeaponLocal);
        }

    }

    TwoHandedGrip::TwoHandedGrip() :
        _fingerPoseSolveScratch(std::make_unique<FingerPoseSolveScratch>())
    {
        _recoilControllerRegistered =
            frik_visual_authority::registerWeaponHandRecoilController(
                WEAPON_RECOIL_CONTROLLER_TAG,
                &TwoHandedGrip::controlWeaponHandRecoil,
                this,
                GRIP_HAND_POSE_PRIORITY);
        if (!_recoilControllerRegistered) {
            ROCK_LOG_WARN(
                Weapon,
                "TwoHandedGrip: FRIK weapon-hand recoil controller registration failed; regular FRIK recoil remains active");
        }
    }

    TwoHandedGrip::~TwoHandedGrip()
    {
        clearGunstockDedicatedHandAuthority();
        if (_recoilControllerRegistered) {
            (void)frik_visual_authority::unregisterWeaponHandRecoilController(
                WEAPON_RECOIL_CONTROLLER_TAG);
            _recoilControllerRegistered = false;
        }
    }

    bool FRIK_CALL TwoHandedGrip::controlWeaponHandRecoil(
        const frik::api::FRIKApiV2::RecoilSample* const sample,
        frik::api::FRIKApiV2::RecoilResponse* const outResponse,
        void* const userData) noexcept
    {
        auto* const self = static_cast<TwoHandedGrip*>(userData);
        if (!self ||
            !sample ||
            sample->structSize < sizeof(frik::api::FRIKApiV2::RecoilSample) ||
            !outResponse) {
            return false;
        }

        const bool firingHandIsLeft = self->_firingHandIsLeft;
        const std::size_t firingHandIndex = firingHandIsLeft ? 0u : 1u;
        if (!self->hasControlledFiringRecoilAuthority(firingHandIsLeft) ||
            (!firingHandIsLeft &&
                (!self->_hasFiringRecoilReference[firingHandIndex] ||
                    self->_firingRecoilReferenceGenerationKey[firingHandIndex] !=
                        self->_activeWeaponGenerationKey))) {
            return false;
        }

        *outResponse = {};
        outResponse->structSize = sizeof(frik::api::FRIKApiV2::RecoilResponse);
        outResponse->handMask = static_cast<std::uint32_t>(
            frik::api::FRIKApiV2::RecoilHandMask::Primary);
        outResponse->delivery = frik::api::FRIKApiV2::RecoilDelivery::Direct;
        outResponse->controlledKickLocal = sample->nativeKickLocal;

        const RE::NiTransform identity =
            transform_math::makeIdentityTransform<RE::NiTransform>();
        if (!areTransformsNearlyEqual(
                sample->nativeKickLocal,
                identity,
                0.00001f)) {
            // FRIK invokes this callback synchronously on its game update
            // thread. Publish only a value ticket here; scene-node mutation
            // remains in ROCK's later presentation phase as required by the
            // API contract. Neutral frames issue no ticket, so normal IK
            // residuals can never become weapon motion.
            ++self->_firingRecoilAcceptedSequence;
            if (self->_firingRecoilAcceptedSequence == 0) {
                self->_firingRecoilAcceptedSequence = 1;
            }
            self->_firingRecoilAcceptedGenerationKey =
                self->_activeWeaponGenerationKey;
            self->_firingRecoilAcceptedHandIsLeft = firingHandIsLeft;
        } else {
            // A neutral callback is a newer FRIK frame than any unconsumed
            // kick ticket. Retire that ticket so it cannot be replayed after
            // a transient presentation skip.
            self->_firingRecoilConsumedSequence =
                self->_firingRecoilAcceptedSequence;
        }
        return true;
    }

    void TwoHandedGrip::update(
        RE::NiNode* weaponNode,
        RE::NiAVObject* observedGunstockFireNode,
        const bool observedGunstockGunType,
        const WeaponInteractionContact& leftWeaponContact,
        const WeaponInteractionContact& rightWeaponContact,
        const EquippedWeaponGripFrameInput& frameInput,
        float dt,
        const std::uint64_t sourceSchedulerSequence,
        std::uint64_t currentWeaponGenerationKey,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const WeaponCollision& weaponCollision,
        const WeaponInteractionRuntimeState& leftRuntimeState,
        const WeaponInteractionRuntimeState& rightRuntimeState,
        weapon_support_authority_policy::WeaponSupportAuthorityMode supportAuthorityMode,
        bool firingGripProximityAuthorityEnabled,
        const EquippedWeaponHandlingSettings& handlingSettings)
    {
        _handlingSettings = handlingSettings;
        _currentHandDriverFrames[0] = frameInput.leftHandDriverFrame;
        _currentHandDriverFrames[1] = frameInput.rightHandDriverFrame;
        _currentSourceSchedulerSequence = sourceSchedulerSequence;
        setNativeReloadHandAuthorityActive(
            frameInput.nativeReloadHandAuthorityActive);
        _gunstockFramePresentation = {};
        observeGunstockWeaponEligibility(
            weaponNode,
            observedGunstockFireNode,
            observedGunstockGunType,
            currentWeaponGenerationKey);
        _gunstockAlignmentBlockedThisFrame =
            frameInput.gunstockPresentationBlocked;
        setGrabbedObjectHandPoseOwnership(
            frameInput.leftHandHoldingObject,
            frameInput.rightHandHoldingObject);
        _hasSolvedWeaponTransform = false;
        _authoredSupportGripDebugSnapshot = {};
        _gunstockSupportBaselineDebugSnapshot = {};
        _scopeHandAuthorityPublishedThisFrame = {};
        _gunstockHandAuthorityActive = {};
        _firingGripReattachHoverInsideRadius = false;
        _firingGripReattachHoverHandIsLeft = _firingHandIsLeft;
        if (g_rockConfig.rockDebugDrawNativeScopeActivation &&
            _nativeScopeCameraDebugSnapshot.framesSinceApply != (std::numeric_limits<std::uint32_t>::max)()) {
            ++_nativeScopeCameraDebugSnapshot.framesSinceApply;
        }

        refreshNativeScopeAnchor(
            weaponNode,
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey,
            weaponCollision.getCurrentObservedEquippedWeaponFormID(),
            weaponCollision);
        refreshScopeSafeHandFrames(weaponNode, frameInput, dt);

        if (!runtime_state::isLocalSkeletonReady() || !weaponNode) {
            clearAllVisualReturns("skeleton-or-weapon-unavailable", true, true);
            if (_state != TwoHandedState::Inactive) {
                transitionToInactive(false);
            }
            reconcileDeferredScopeHandAuthority(weaponNode);
            return;
        }

        publishCollisionIsolatedRightNativeWeaponIntent(
            weaponNode,
            currentWeaponGenerationKey);

        refreshNaturalHandInWandFrames();
        refreshAuthoredSupportRightMirror();

        updateWeaponVisualReturn(
            weaponNode,
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey,
            dt);

        EquippedWeaponGripFrameInput stableFrameInput = frameInput;
        if (_persistentEquippedCarryActive && isManualOwnershipActive()) {
            /*
             * A persistent fixed/selected-hand carry has no grab press to
             * retain PrimaryOnly. Keep
             * the firing grip virtually closed until the player physically
             * holds it once; only that armed hand's later release is allowed
             * through the normal debounce/drop machinery. This preserves all
             * existing two-hand, detach, stash, and handoff gestures without
             * an immediate phantom drop on the first post-menu frame.
             */
            if (frameInput.primaryGripInput.held || frameInput.primaryGripInput.pressed) {
                _persistentEquippedCarryDetachArmed = true;
            }
            if (!_persistentEquippedCarryDetachArmed) {
                stableFrameInput.primaryGripInput.held = true;
                stableFrameInput.primaryGripInput.pressed = false;
                stableFrameInput.primaryGripInput.released = false;
            }
        }
        const auto primaryReleaseDecision = equipped_weapon_manual_ownership_policy::debouncePrimaryGripRelease(
            _primaryReleaseDebounce,
            stableFrameInput.primaryGripInput.held);
        stableFrameInput.primaryGripInput.held = primaryReleaseDecision.retained;
        stableFrameInput.primaryGripInput.released = primaryReleaseDecision.releaseConfirmed;

        if (isManualOwnershipActive() &&
            !reconcileCollisionGeneration(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                weaponCollision)) {
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: clearing authority because equipped weapon instance changed active={:016X} current={:016X}",
                _activeEquippedWeaponOwnershipKey,
                currentEquippedWeaponOwnershipKey);
            clearAllVisualReturns("equipped-weapon-identity-changed", true, true);
            transitionToInactive(false);
            reconcileDeferredScopeHandAuthority(weaponNode);
            return;
        }

        /*
         * Left-firing feed-forward pre-write: while ROCK owns the weapon node
         * (left-firing topology), FRIK's earlier skeleton pass has already
         * rewritten the node to its OFFHAND GLUE pose, so at this point
         * weaponNode->world is glue space, not the real carried pose. Every
         * world<->weapon-local conversion below (part-grip captures, mesh
         * grab points, promotion distances, the two-hand solver base) would
         * silently mix real-space palm/contact points with that glue frame -
         * the round-4 corrupted captures. Publishing the canonical
         * feed-forward pose FIRST makes the node a real-space basis for all
         * existing math with no per-call-site special cases; the state
         * handlers below re-publish their final solved pose as before.
         * Right-firing reads FRIK's authored carry and is untouched.
         * (PhysicsInteraction additionally publishes this before the frame's
         * weapon interaction probes - see the header note.)
         */
        (void)publishLeftFiringFeedForwardWeaponPose(weaponNode);

        if (!reconcileGunstockModeState(weaponNode)) {
            ROCK_LOG_WARN(
                Weapon,
                "TwoHandedGrip: clearing support authority because the gunstock mode-edge transaction could not establish a valid support baseline");
            transitionToInactive(false);
            reconcileDeferredScopeHandAuthority(weaponNode);
            return;
        }

        /*
         * Support-side routing follows the CURRENT firing hand: the support
         * hand is whichever physical hand does not own the firing grip. All
         * grip math below is weapon-relative; the hands only choose roles.
         */
        const bool supportHandIsLeft = !_firingHandIsLeft;
        const WeaponInteractionContact& supportWeaponContact = supportHandIsLeft ? leftWeaponContact : rightWeaponContact;
        const WeaponInteractionRuntimeState& supportRuntimeState = supportHandIsLeft ? leftRuntimeState : rightRuntimeState;
        const WeaponInteractionDecision decision = routeWeaponInteraction(supportWeaponContact, supportRuntimeState);
        refreshAuthoredSupportGripActivationState(
            weaponNode,
            currentWeaponGenerationKey,
            weaponCollision,
            false);
        const bool supportTouchingSupport =
            !_nativeReloadHandAuthorityActive &&
            decision.kind == WeaponInteractionKind::SupportGrip;
        RE::NiNode* interactionWeaponNode = sourceRootNodeOrFallback(decision.interactionRoot, weaponNode);
        const bool supportGripHeld = supportHandIsLeft ? stableFrameInput.leftGripHeld : stableFrameInput.rightGripHeld;
        const bool supportHandHoldingObject = supportHandIsLeft ? stableFrameInput.leftHandHoldingObject : stableFrameInput.rightHandHoldingObject;
        const EquippedWeaponPrimaryGripInput& primaryGripInput = stableFrameInput.primaryGripInput;

        switch (_state) {
        case TwoHandedState::Inactive:
            if (supportTouchingSupport && !supportHandHoldingObject) {
                transitionToTouching(interactionWeaponNode, decision);
            }
            break;

        case TwoHandedState::Touching:
            if (supportHandHoldingObject) {
                _state = TwoHandedState::Inactive;
                break;
            }
            if (supportTouchingSupport) {
                _touchFrames = 0;
            } else {
                _touchFrames++;
                if (_touchFrames > TOUCH_TIMEOUT_FRAMES) {
                    ROCK_LOG_INFO(
                        Weapon,
                        "TwoHandedGrip: touch contact timed out firingHand={} supportHand={} decision={} contactValid={} body={} part={} source={} generation={:016X}",
                        _firingHandIsLeft ? "left" : "right",
                        supportHandIsLeft ? "left" : "right",
                        static_cast<int>(decision.kind),
                        supportWeaponContact.valid,
                        supportWeaponContact.bodyId,
                        static_cast<int>(supportWeaponContact.partKind),
                        static_cast<int>(supportWeaponContact.acquisitionSource),
                        supportWeaponContact.weaponGenerationKey);
                    _state = TwoHandedState::Inactive;
                    break;
                }
            }
            if (weapon_two_handed_grip_math::canStartSupportGrip(supportTouchingSupport, supportGripHeld, supportHandHoldingObject)) {
                transitionToGripping(interactionWeaponNode,
                    decision,
                    weaponCollision,
                    supportAuthorityMode,
                    firingGripProximityAuthorityEnabled,
                    currentEquippedWeaponOwnershipKey,
                    supportRuntimeState.providerPartAuthority);
            }
            break;

        case TwoHandedState::Gripping:
            if (_supportGripAgeFrames < (std::numeric_limits<std::uint32_t>::max)()) {
                ++_supportGripAgeFrames;
            }
            if (!_activeWeaponNode) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because active weapon source root is unavailable");
                transitionToInactive(false);
            } else if (!weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(_activeWeaponGenerationKey, currentWeaponGenerationKey)) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because weapon generation changed during support grip");
                transitionToInactive(false);
            } else if (!providerPartAuthorityStillCurrent(partGrip(supportHandIsLeft), currentWeaponGenerationKey)) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because provider weapon-part target is no longer current");
                transitionToInactive(false);
            } else if (providerPartTargetNewlyMatchesGrip(partGrip(supportHandIsLeft), currentWeaponGenerationKey)) {
                // The still-held grab recaptures next frame under the new
                // provider resolution (e.g. an AttachOnly whitelist armed
                // mid-hold).
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: releasing support grip to recapture under newly matched provider weapon-part target");
                transitionToInactive(ownsWeaponTransform());
            } else if (!supportRuntimeState.supportGripAllowed &&
                       !_nativeReloadHandAuthorityActive) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because offhand reservation disabled support grip");
                transitionToInactive(false);
            } else if (!weapon_two_handed_grip_math::shouldContinueSupportGrip(supportGripHeld, supportHandHoldingObject)) {
                ROCK_LOG_INFO(Weapon,
                    "TwoHandedGrip: support release predicate firingHand={} supportHand={} gripHeld={} holdingObject={} scopeMenu={}",
                    _firingHandIsLeft ? "left" : "right",
                    supportHandIsLeft ? "left" : "right",
                    supportGripHeld ? "yes" : "no",
                    supportHandHoldingObject ? "yes" : "no",
                    _scopeMenuOpenThisFrame ? "open" : "closed");
                const auto releaseAction = weapon_two_handed_grip_math::resolveSupportReleaseManualAction(
                    weapon_two_handed_grip_math::SupportReleaseOwnershipInput{
                        .firingGripOwnershipEnabled = handlingSettings.firingGripOwnershipEnabled,
                        .primaryDetachEnabled = handlingSettings.primaryDetachEnabled,
                        .primaryGripHeld = primaryGripInput.held,
                    });
                if (releaseAction == weapon_two_handed_grip_math::SupportReleaseManualAction::KeepPrimaryOwnership) {
                    beginHandVisualReturn(supportHandIsLeft, "support-released-primary-held");
                    if (ownsWeaponTransform()) {
                        beginHandVisualReturn(_firingHandIsLeft, "two-hand-primary-return-to-native-carry");
                        if (!_firingHandIsLeft) {
                            beginWeaponVisualReturn("support-released-primary-held");
                        }
                    }
                    transitionToPrimaryOnly(
                        _activeWeaponNode,
                        currentWeaponGenerationKey,
                        currentEquippedWeaponOwnershipKey,
                        "support-released-primary-held");
                } else if (releaseAction == weapon_two_handed_grip_math::SupportReleaseManualAction::DropEquippedWeapon) {
                    beginHandVisualReturn(supportHandIsLeft, "support-released-drop");
                    beginHandVisualReturn(_firingHandIsLeft, "primary-released-drop");
                    requestEquippedWeaponDrop(
                        "support-released-primary-not-held",
                        equipped_weapon_drop_policy::sourceForSupportRelease(primaryGripInput.released));
                } else {
                    beginHandVisualReturn(supportHandIsLeft, "support-released");
                    beginHandVisualReturn(_firingHandIsLeft, "primary-authority-cleared");
                    if (ownsWeaponTransform()) {
                        beginWeaponVisualReturn("support-released");
                    }
                    transitionToInactive(ownsWeaponTransform());
                }
            } else if ((handlingSettings.primaryDetachEnabled || handlingSettings.ambidextrousHandoffEnabled) && !primaryGripInput.held &&
                       equipped_weapon_manual_ownership_policy::shouldDeferPrimaryReleaseActionForFreshSupportGrip(_supportGripAgeFrames)) {
                /*
                 * The firing-grip release confirmed while the support grab is
                 * only a few frames old: same physical gesture or a
                 * grab-synchronized grip flicker, never an independent
                 * release. Hold the two-handed grip unchanged; a re-pressed
                 * grip resumes normally, and promotion/detach run below once
                 * the grab has aged. leftGripHeld/rightGripHeld in the log
                 * discriminate a physical flicker (both pipelines open) from
                 * an input-path divergence (normal pipeline still held).
                 */
                if (!_freshSupportGripDeferLogged) {
                    _freshSupportGripDeferLogged = true;
                    ROCK_LOG_INFO(Weapon,
                        "TwoHandedGrip: deferring firing-grip release action while support grip is fresh age={} firingHand={} leftGripHeld={} rightGripHeld={}",
                        _supportGripAgeFrames,
                        _firingHandIsLeft ? "left" : "right",
                        stableFrameInput.leftGripHeld ? "yes" : "no",
                        stableFrameInput.rightGripHeld ? "yes" : "no");
                }
                updateGripping(_activeWeaponNode, dt);
            } else if (handlingSettings.ambidextrousHandoffEnabled && !primaryGripInput.held && tryPromoteSupportGripToFiringGrip(_activeWeaponNode)) {
                // The support hand was wrapped over the firing grip when the
                // firing hand opened: it takes over the SAME weapon-relative
                // grip in place (seamless hand switch, pistol shooting-cup
                // flow). State is PrimaryOnly under the new firing hand.
            } else if (handlingSettings.primaryDetachEnabled &&
                       _authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver &&
                       !primaryGripInput.held) {
                if (transitionToPartCarry()) {
                    updatePartCarryGrip(
                        _activeWeaponNode,
                        dt,
                        stableFrameInput,
                        leftWeaponContact,
                        rightWeaponContact,
                        weaponCollision,
                        currentWeaponGenerationKey,
                        currentEquippedWeaponOwnershipKey,
                        leftRuntimeState,
                        rightRuntimeState);
                }
            } else {
                updateGripping(_activeWeaponNode, dt);
            }
            break;

        case TwoHandedState::PartCarry:
            if (!_activeWeaponNode) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing part-carry authority because active weapon source root is unavailable");
                transitionToInactive(false);
            } else if (!weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(_activeWeaponGenerationKey, currentWeaponGenerationKey)) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing part-carry authority because weapon generation changed");
                transitionToInactive(false);
            } else if (!handlingSettings.primaryDetachEnabled) {
                transitionToInactive(ownsWeaponTransform());
            } else {
                updatePartCarryGrip(
                    _activeWeaponNode,
                    dt,
                    stableFrameInput,
                    leftWeaponContact,
                    rightWeaponContact,
                    weaponCollision,
                    currentWeaponGenerationKey,
                    currentEquippedWeaponOwnershipKey,
                    leftRuntimeState,
                    rightRuntimeState);
            }
            break;

        case TwoHandedState::PrimaryOnly:
            if (!_activeWeaponNode) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing primary-only authority because active weapon source root is unavailable");
                transitionToInactive(false);
            } else if (!handlingSettings.firingGripOwnershipEnabled) {
                transitionToInactive(false);
            } else if (supportTouchingSupport && weapon_two_handed_grip_math::canStartSupportGrip(supportTouchingSupport, supportGripHeld, supportHandHoldingObject)) {
                transitionToGripping(interactionWeaponNode,
                    decision,
                    weaponCollision,
                    supportAuthorityMode,
                    firingGripProximityAuthorityEnabled,
                    currentEquippedWeaponOwnershipKey,
                    supportRuntimeState.providerPartAuthority);
            } else {
                updatePrimaryOnlyGrip(
                    _activeWeaponNode,
                    currentEquippedWeaponOwnershipKey,
                    primaryGripInput,
                    handlingSettings.primaryDetachEnabled);
            }
            break;
        }

        refreshRightNativeCanonicalFrame(
            weaponNode,
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey);

        // Enforce the left-firing weapon-node ownership contract after every
        // state/role transition this frame (idempotent; also the parent
        // watchdog for engine-side re-attach).
        syncFiringHandWeaponNodeOwnership(weaponNode);
        updateHandVisualReturns(dt);
        // State transitions and their replacement publications must finish
        // before stale scoped roles are removed. This keeps hFRIK under one
        // continuous ROCK authority selection across scope and role edges.
        reconcileDeferredScopeHandAuthority(weaponNode);
        traceNativeScopeTransitionFinalState(weaponNode);
    }

    void TwoHandedGrip::reset()
    {
        (void)frik_visual_authority::clearExternalHandWorldTransform(
            WEAPON_COLLISION_HAND_TAG,
            frik_visual_authority::Hand::Left);
        (void)frik_visual_authority::clearExternalHandWorldTransform(
            WEAPON_COLLISION_HAND_TAG,
            frik_visual_authority::Hand::Right);
        _weaponCollisionHandAuthorityLive = {};
        _weaponCollisionHandAuthorityGenerationKey = {};
        _preFrikWeaponHandAuthority = {};
        _preFrikRetainedHandAuthorities = {};
        _currentSourceSchedulerSequence = 0;
        _weaponCollisionHandPresentationFromPreviousFrame = {};
        _weaponCollisionBaselineHandWorldValid = {};
        _nativeReloadHandAuthorityActive = false;
        _nativeReloadSupportHandIsLeft = true;
        clearFiringRecoilPresentationState();
        resetGunstockAlignment("reset");
        _gunstockModeToggle = {};
        _gunstockWeaponEligibility = {};
        _gunstockAlignmentBlockedThisFrame = false;
        clearDynamicSupportAcquisition("reset", true);
        clearAuthoredSupportGripCandidate();
        _authoredSupportGripDebugSnapshot = {};
        _gunstockAlignmentDebugSnapshot = {};
        _gunstockSupportBaselineDebugSnapshot = {};
        _authoredSupportLastStableApproachDirectionWorld = {};
        _authoredSupportLastStableDirectionGenerationKey = 0;
        _authoredSupportLastStableDirectionCaptureSequence = 0;
        _authoredSupportLastStableApproachDirectionValid = false;
        clearAllVisualReturns("reset", false, true);
        clearNativeScopeOverlayAuthority(true);
        _equippedWeaponDropRequest = {};
        _hapticEvents = {};
        _firingGripReattachHoverInsideRadius = false;
        clearNativeScopeAnchorState();
        _nativeScopeCameraDebugSnapshot = {};
        _nativeScopeActivationDebugSnapshot = {};
        clearNativeScopeRigidFrame();
        _scopeSafeHandFrames = {};
        _scopeDriverFrameAuthorityActive = false;
        _nativeScopeRequestStateValid = false;
        _nativeScopeRequestActive = false;
        _manualScopeActivationRequested = false;
        _nativeScopeTransitionTraceSequence = 0;
        _nativeScopeTransitionTraceFramesRemaining = 0;
        _nativeScopeTransitionFinalTraceSequence = 0;
        _nativeScopeTransitionFinalTraceSample = 0;
        _nativeScopeTransitionFinalTracePending = false;
        _scopeHandAuthorityPublishedThisFrame = {};
        clearPrimaryGripPose(_firingHandIsLeft);
        clearPrimaryDetachVisualAuthority(_firingHandIsLeft);
        clearSupportGripPose(true);
        clearSupportGripPose(false);
        restoreFrikPrimaryWeaponPose();
        clearRightFiringHandCanonicalFrame();
        _rightNaturalBoneInWand = {};
        _leftNaturalBoneInWand = {};
        _rightNaturalBoneInDampedDriver = {};
        _leftNaturalBoneInDampedDriver = {};
        _hasRightNaturalBoneInWand = false;
        _hasLeftNaturalBoneInWand = false;
        _hasRightNaturalBoneInDampedDriver = false;
        _hasLeftNaturalBoneInDampedDriver = false;
        _authoredPrimaryFingerPoseSuppressed = false;
        _leftHandHoldingObjectForPose = false;
        _rightHandHoldingObjectForPose = false;
        if (_state != TwoHandedState::Inactive) {
            transitionToInactive(false);
            _scopeMenuOpenThisFrame = false;
            _scopeMenuClosedThisFrame = false;
            return;
        }
        _scopeMenuOpenThisFrame = false;
        _scopeMenuClosedThisFrame = false;
        _hasSolvedWeaponTransform = false;
        clearActiveGripState();
        _lastPublishedHandWorld = {};
        _hasLastPublishedHandWorld = {};
        _lastRenderedWeaponWorld = {};
        _hasLastRenderedWeaponWorld = false;
    }

    bool TwoHandedGrip::ownsWeaponTransform() const
    {
        return (_state == TwoHandedState::Gripping || _state == TwoHandedState::PartCarry) &&
               weapon_support_authority_policy::supportGripOwnsWeaponTransform(_authorityMode);
    }

    bool TwoHandedGrip::blocksAuthoredPrimaryGripWeaponAlignment() const
    {
        /*
         * Right-firing PrimaryOnly is lifecycle/input ownership only: hFRIK
         * still publishes the native Weapon transform every frame. Treating
         * that state as a competing transform owner made the authored
         * calibration disappear immediately after a support-hand return.
         * Left-firing carry always remains ROCK-owned even in PrimaryOnly or
         * visual-only support mode, and the topology blocker is included as a
         * fail-closed witness if state and bridge cleanup ever diverge.
         */
        return _firingHandIsLeft ||
               _weaponNodeOwnershipBlockEngaged ||
               ownsWeaponTransform();
    }

    bool TwoHandedGrip::isWeaponVisualReturnActive() const
    {
        return _returningWeaponVisual.localTransition.active;
    }

    bool TwoHandedGrip::getSolvedWeaponTransform(RE::NiTransform& outTransform) const
    {
        if (!_hasSolvedWeaponTransform) {
            return false;
        }
        outTransform = _lastSolvedWeaponTransform;
        return true;
    }

    bool TwoHandedGrip::getManualCycleRockGripBaselines(
        RE::NiTransform& outRightHandInWeapon,
        RE::NiTransform& outLeftHandInWeapon) const
    {
        outRightHandInWeapon = {};
        outLeftHandInWeapon = {};

        const WeaponPartGrip& supportGrip = partGrip(true);
        if (_state != TwoHandedState::Gripping ||
            _firingHandIsLeft ||
            !ownsWeaponTransform() ||
            !_hasSolvedWeaponTransform ||
            !_activeWeaponNode ||
            !_hasFiringHandWeaponLocal ||
            !supportGrip.active ||
            !supportGrip.hasHandWeaponLocal ||
            !isFiniteTransform(_lastSolvedWeaponTransform) ||
            !isFiniteTransform(_primaryHandWeaponLocal)) {
            return false;
        }

        const RE::NiTransform supportHandWorld =
            resolvePartGripHandWorld(supportGrip, _activeWeaponNode);
        if (!isFiniteTransform(supportHandWorld)) {
            return false;
        }

        outRightHandInWeapon = _primaryHandWeaponLocal;
        outLeftHandInWeapon = transform_math::composeTransforms(
            transform_math::invertTransform(_lastSolvedWeaponTransform),
            supportHandWorld);
        if (!isFiniteTransform(outRightHandInWeapon) ||
            !isFiniteTransform(outLeftHandInWeapon)) {
            outRightHandInWeapon = {};
            outLeftHandInWeapon = {};
            return false;
        }
        return true;
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

    void TwoHandedGrip::refreshAuthoredSupportGripActivationState(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const WeaponCollision& weaponCollision,
        const bool requirePoseEvidence)
    {
        _authoredSupportGripDebugSnapshot = {};
        const bool collectPoseEvidence =
            requirePoseEvidence ||
            g_rockConfig.rockDebugDrawAuthoredGripActivationZones ||
            g_rockConfig.rockDebugShowHandAxes ||
            g_rockConfig.rockDebugShowGrabPivots;

        const auto& candidate = _authoredSupportGripCandidate;
        if (!weaponNode ||
            !candidate.valid ||
            candidate.weaponNode != weaponNode ||
            candidate.weaponGenerationKey == 0 ||
            candidate.weaponGenerationKey != currentWeaponGenerationKey ||
            candidate.captureSequence == 0 ||
            !isFiniteTransform(weaponNode->world)) {
            return;
        }

        if (_authoredSupportLastStableDirectionGenerationKey !=
                candidate.weaponGenerationKey ||
            _authoredSupportLastStableDirectionCaptureSequence !=
                candidate.captureSequence) {
            _authoredSupportLastStableApproachDirectionWorld = {};
            _authoredSupportLastStableDirectionGenerationKey =
                candidate.weaponGenerationKey;
            _authoredSupportLastStableDirectionCaptureSequence =
                candidate.captureSequence;
            _authoredSupportLastStableApproachDirectionValid = false;
        }

        const bool supportHandIsLeft = !_firingHandIsLeft;
        RE::NiTransform authoredSupportHandWeaponLocal{};
        std::array<RE::NiTransform, 15> authoredSupportFingerLocalTransforms{};
        std::uint16_t authoredSupportFingerLocalTransformMask = 0;
        if (!tryResolveAuthoredSupportGripCandidateForHand(
                supportHandIsLeft,
                weaponNode,
                candidate.weaponGenerationKey,
                authoredSupportHandWeaponLocal,
                authoredSupportFingerLocalTransforms,
                authoredSupportFingerLocalTransformMask)) {
            return;
        }

        RE::NiTransform liveSupportHandWorld{};
        if (!tryGetSolverHandTransform(supportHandIsLeft, liveSupportHandWorld)) {
            return;
        }

        RE::NiTransform activationWeaponWorld = weaponNode->world;
        RE::NiTransform alignmentHandWorld{};
        RE::NiMatrix3 gunstockCorrection{};
        RE::NiPoint3 gunstockPivotWorld{};
        if (tryResolveGunstockPrimaryGroupCorrection(
                weaponNode,
                currentWeaponGenerationKey,
                alignmentHandWorld,
                gunstockCorrection,
                gunstockPivotWorld)) {
            const RE::NiTransform correctedWeaponWorld =
                gunstock_alignment_policy::rotateRigidlyAroundPivot<
                    RE::NiTransform,
                    RE::NiMatrix3,
                    RE::NiPoint3>(
                    activationWeaponWorld,
                    gunstockCorrection,
                    gunstockPivotWorld);
            if (isFiniteTransform(correctedWeaponWorld)) {
                activationWeaponWorld = correctedWeaponWorld;
            }
        }

        AuthoredSupportPalmSeatProximity proximity{};
        if (!resolveAuthoredSupportPalmSeatProximity(
                activationWeaponWorld,
                liveSupportHandWorld,
                authoredSupportHandWeaponLocal,
                supportHandIsLeft,
                proximity)) {
            return;
        }

        auto& snapshot = _authoredSupportGripDebugSnapshot;
        snapshot.weaponWorld = activationWeaponWorld;
        snapshot.authoredPalmSeatWeaponLocal =
            proximity.authoredPalmSeatWeaponLocal;
        snapshot.authoredPalmSeatWorld = proximity.authoredPalmSeatWorld;
        snapshot.liveTouchProbeWeaponLocal =
            proximity.liveTouchProbeWeaponLocal;
        snapshot.liveTouchProbeWorld = proximity.liveTouchProbeWorld;
        snapshot.weaponRelativeDistanceGameUnits =
            proximity.weaponRelativeDistanceGameUnits;
        snapshot.worldReadbackDistanceGameUnits =
            proximity.worldReadbackDistanceGameUnits;
        snapshot.frameAgreementErrorGameUnits =
            proximity.frameAgreementErrorGameUnits;
        snapshot.touchRadiusGameUnits =
            g_rockConfig.rockWeaponInteractionTouchRadius;
        snapshot.radialCapGameUnits =
            g_rockConfig.rockWeaponAuthoredGripActivationRadius;
        snapshot.weaponGenerationKey = candidate.weaponGenerationKey;
        snapshot.captureSequence = candidate.captureSequence;
        snapshot.supportHandIsLeft = supportHandIsLeft;
        snapshot.mirroredForRightSupport = !supportHandIsLeft;
        snapshot.insideTouchRadius =
            proximity.weaponRelativeDistanceGameUnits <=
            snapshot.touchRadiusGameUnits;

        const auto identity = weaponCollision.getEquippedWeaponClassification();
        snapshot.weaponFormID = identity.formID;
        snapshot.effectiveEquipSlotFormID =
            identity.effectiveEquipSlotFormID;
        snapshot.baseEquipSlotFormID = identity.baseEquipSlotFormID;
        snapshot.effectiveEquipSlotUsesInstanceData =
            identity.effectiveEquipSlotUsesInstanceData;
        const bool meleeOrUnarmed =
            identity.sizeClass == WeaponSizeClass::Melee;
        const bool heavyGun = hasWeaponKeywordFlag(
            identity.keywordFlags,
            WeaponKeywordFlag::HeavyGun);
        snapshot.weaponFamily =
            authored_weapon_grip_activation_policy::resolveWeaponFamily(
                authored_weapon_grip_activation_policy::WeaponFamilyInput{
                    .effectiveEquipSlotFormID =
                        identity.effectiveEquipSlotFormID,
                    .equippedWeaponPresent = identity.hasEquippedWeapon,
                    .meleeOrUnarmed = meleeOrUnarmed,
                    .heavyGun = heavyGun,
                });

        const bool canonicalCurrent =
            !_firingHandIsLeft &&
            supportHandIsLeft &&
            _hasRightFiringHandCanonicalWeaponLocal &&
            _rightFiringHandCanonicalSource ==
                RightFiringCanonicalSource::AuthoredAnimation &&
            _rightFiringHandCanonicalWeaponNode == weaponNode &&
            _rightFiringHandCanonicalGenerationKey ==
                currentWeaponGenerationKey &&
            isFiniteTransform(_rightFiringHandCanonicalWeaponLocal);
        if (canonicalCurrent) {
            const RE::NiTransform firingHandWorld =
                transform_math::composeTransforms(
                    activationWeaponWorld,
                    _rightFiringHandCanonicalWeaponLocal);
            const auto normalizeVector = [](const RE::NiPoint3& input,
                                             RE::NiPoint3& output) {
                output = {};
                const float lengthSquared =
                    input.x * input.x +
                    input.y * input.y +
                    input.z * input.z;
                if (!std::isfinite(lengthSquared) ||
                    lengthSquared <= 0.000001f) {
                    return false;
                }
                const float inverseLength = 1.0f / std::sqrt(lengthSquared);
                output = RE::NiPoint3{
                    input.x * inverseLength,
                    input.y * inverseLength,
                    input.z * inverseLength,
                };
                return std::isfinite(output.x) &&
                       std::isfinite(output.y) &&
                       std::isfinite(output.z);
            };
            RE::NiPoint3 leftAxis{};
            const bool leftAxisValid = normalizeVector(
                computePalmNormalFromHandBasis(firingHandWorld, false),
                leftAxis);
            const RE::NiPoint3 thumbUp = transformHandspaceDirection(
                firingHandWorld,
                RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
                false);
            const float thumbLeftProjection =
                thumbUp.x * leftAxis.x +
                thumbUp.y * leftAxis.y +
                thumbUp.z * leftAxis.z;
            RE::NiPoint3 orthogonalUp{
                thumbUp.x - leftAxis.x * thumbLeftProjection,
                thumbUp.y - leftAxis.y * thumbLeftProjection,
                thumbUp.z - leftAxis.z * thumbLeftProjection,
            };
            RE::NiPoint3 normalizedUp{};
            const bool upAxisValid =
                leftAxisValid && normalizeVector(orthogonalUp, normalizedUp);
            if (leftAxisValid && upAxisValid) {
                snapshot.leftAxisWorld = leftAxis;
                snapshot.downAxisWorld = RE::NiPoint3{
                    -normalizedUp.x,
                    -normalizedUp.y,
                    -normalizedUp.z,
                };
                const RE::NiPoint3 referenceAxis{
                    leftAxis.y * snapshot.downAxisWorld.z -
                        leftAxis.z * snapshot.downAxisWorld.y,
                    leftAxis.z * snapshot.downAxisWorld.x -
                        leftAxis.x * snapshot.downAxisWorld.z,
                    leftAxis.x * snapshot.downAxisWorld.y -
                        leftAxis.y * snapshot.downAxisWorld.x,
                };
                snapshot.canonicalAxesValid = normalizeVector(
                    referenceAxis,
                    snapshot.referenceAxisWorld);
            }
        }

        const auto gate = evaluateAuthoredSupportGripDirectionGate(
            snapshot,
            _authoredSupportLastStableApproachDirectionWorld,
            _authoredSupportLastStableApproachDirectionValid,
            snapshot.canonicalAxesValid &&
                !_firingHandIsLeft && supportHandIsLeft);
        if (gate.directionValid &&
            gate.radialDistanceGameUnits >=
                authored_weapon_grip_activation_policy::
                    kMinimumDirectionDistanceGameUnits) {
            _authoredSupportLastStableApproachDirectionWorld =
                snapshot.approachDirectionWorld;
            _authoredSupportLastStableApproachDirectionValid = true;
        }

        if (collectPoseEvidence) {
            std::array<RE::NiPoint3,
                AuthoredSupportGripDebugSnapshot::kPoseLandmarkCount>
                surfaceQueryLandmarksWorld{};
            snapshot.poseLandmarksWorld[0] = snapshot.authoredPalmSeatWorld;
            surfaceQueryLandmarksWorld[0] =
                transform_math::localPointToWorld(
                    weaponNode->world,
                    snapshot.authoredPalmSeatWeaponLocal);
            constexpr std::array<std::size_t, 5> kDistalFingerLocalIndices{
                2, 5, 8, 11, 14
            };
            for (std::size_t fingerIndex = 0;
                 fingerIndex < kDistalFingerLocalIndices.size();
                 ++fingerIndex) {
                const std::size_t distalIndex =
                    kDistalFingerLocalIndices[fingerIndex];
                const std::size_t chainStart = distalIndex - 2;
                RE::NiTransform fingerWeaponLocal =
                    transform_math::composeTransforms(
                        authoredSupportHandWeaponLocal,
                        authoredSupportFingerLocalTransforms[chainStart]);
                fingerWeaponLocal = transform_math::composeTransforms(
                    fingerWeaponLocal,
                    authoredSupportFingerLocalTransforms[chainStart + 1]);
                fingerWeaponLocal = transform_math::composeTransforms(
                    fingerWeaponLocal,
                    authoredSupportFingerLocalTransforms[distalIndex]);
                snapshot.poseLandmarksWorld[fingerIndex + 1] =
                    transform_math::localPointToWorld(
                        activationWeaponWorld,
                        fingerWeaponLocal.translate);
                surfaceQueryLandmarksWorld[fingerIndex + 1] =
                    transform_math::localPointToWorld(
                        weaponNode->world,
                        fingerWeaponLocal.translate);
            }

            std::array<WeaponCollision::WeaponSurfaceProximityWitness,
                AuthoredSupportGripDebugSnapshot::kPoseLandmarkCount>
                poseWitnesses{};
            (void)weaponCollision.findCurrentWeaponSurfaceNearPoints(
                weaponNode,
                surfaceQueryLandmarksWorld,
                snapshot.touchRadiusGameUnits,
                poseWitnesses);
            for (std::size_t landmarkIndex = 0;
                 landmarkIndex < poseWitnesses.size();
                 ++landmarkIndex) {
                const auto& witness = poseWitnesses[landmarkIndex];
                if (!witness.valid ||
                    witness.weaponGenerationKey != currentWeaponGenerationKey) {
                    continue;
                }
                snapshot.poseSurfaceWitnessMask |=
                    static_cast<std::uint8_t>(1u << landmarkIndex);
                ++snapshot.poseSurfaceWitnessCount;
                snapshot.poseSurfaceWitnessWorld[landmarkIndex] =
                    transform_math::localPointToWorld(
                        activationWeaponWorld,
                        transform_math::worldPointToLocal(
                            weaponNode->world,
                            witness.closestPointWorld));
                snapshot.poseSurfaceDistanceGameUnits[landmarkIndex] =
                    witness.distanceGameUnits;
            }
            snapshot.poseEvidencePass =
                (snapshot.poseSurfaceWitnessMask & 0x01u) != 0 &&
                snapshot.poseSurfaceWitnessCount >= 3;
        }
        const auto& activeSupportGrip = partGrip(supportHandIsLeft);
        snapshot.currentSupportGripActive = activeSupportGrip.active;
        snapshot.currentAuthoredSupportGripActive =
            activeSupportGrip.active && activeSupportGrip.authoredSupportGrip;
        snapshot.valid = true;
    }

    void TwoHandedGrip::resetLockedHandVisualLerp()
    {
        _primaryHandVisualLerp = {};
        partGrip(true).visualLerp = {};
        partGrip(false).visualLerp = {};
    }

    bool TwoHandedGrip::isHandVisualReturnActive(const bool isLeft) const
    {
        return _returningHandVisuals[isLeft ? 0u : 1u].transition.active;
    }

    bool TwoHandedGrip::hasVisualAuthorityForHand(const bool isLeft) const
    {
        if (isNativeReloadSupportHand(isLeft)) {
            return false;
        }
        if (_gunstockHandAuthorityActive[isLeft ? 0u : 1u] ||
            _gunstockDedicatedHandAuthorityActive[isLeft ? 0u : 1u] ||
            isHandVisualReturnActive(isLeft) ||
            partGrip(isLeft).active) {
            return true;
        }
        return isLeft == _firingHandIsLeft &&
            _state == TwoHandedState::Gripping &&
            weapon_support_authority_policy::supportGripAppliesPrimaryHandAuthority(_authorityMode);
    }

    void TwoHandedGrip::recordPublishedHandWorld(const bool isLeft, const RE::NiTransform& appliedWorld)
    {
        if (!isUsableHandAuthorityTransform(appliedWorld)) {
            return;
        }
        const std::size_t index = isLeft ? 0u : 1u;
        _lastPublishedHandWorld[index] = appliedWorld;
        _hasLastPublishedHandWorld[index] = true;
        _weaponCollisionBaselineHandWorld[index] = appliedWorld;
        _weaponCollisionBaselineHandWorldValid[index] = true;
        rememberFiringRecoilReference(isLeft, appliedWorld);
    }

    bool TwoHandedGrip::hasControlledFiringRecoilAuthority(
        const bool isLeft) const
    {
        if (isLeft != _firingHandIsLeft ||
            !_activeWeaponNode ||
            _activeWeaponGenerationKey == 0 ||
            _nativeReloadHandAuthorityActive) {
            return false;
        }

        if (isLeft) {
            return _weaponNodeOwnershipBlockEngaged &&
                   isManualOwnershipActive();
        }

        return _state == TwoHandedState::Gripping &&
               weapon_support_authority_policy::
                   supportGripAppliesPrimaryHandAuthority(_authorityMode);
    }

    void TwoHandedGrip::rememberFiringRecoilReference(
        const bool isLeft,
        const RE::NiTransform& handWorld)
    {
        if (!hasControlledFiringRecoilAuthority(isLeft) ||
            !isUsableHandAuthorityTransform(handWorld)) {
            return;
        }

        const std::size_t index = isLeft ? 0u : 1u;
        _firingRecoilReferenceHandWorld[index] = handWorld;
        _firingRecoilReferenceGenerationKey[index] =
            _activeWeaponGenerationKey;
        _hasFiringRecoilReference[index] = true;
    }

    void TwoHandedGrip::clearFiringRecoilPresentationState()
    {
        _firingRecoilReferenceHandWorld = {};
        _firingRecoilReferenceGenerationKey = {};
        _firingRecoilAcceptedGenerationKey = 0;
        _firingRecoilAcceptedSequence = 0;
        _firingRecoilConsumedSequence = 0;
        _hasFiringRecoilReference = {};
        _firingRecoilAcceptedHandIsLeft = false;
    }

    void TwoHandedGrip::clearFiringRecoilPresentationState(const bool isLeft)
    {
        const std::size_t index = isLeft ? 0u : 1u;
        _firingRecoilReferenceHandWorld[index] = {};
        _firingRecoilReferenceGenerationKey[index] = 0;
        _hasFiringRecoilReference[index] = false;
        if (_firingRecoilAcceptedHandIsLeft == isLeft) {
            _firingRecoilConsumedSequence =
                _firingRecoilAcceptedSequence;
            _firingRecoilAcceptedGenerationKey = 0;
        }
    }

    void TwoHandedGrip::beginHandVisualReturn(const bool isLeft, const char* reason)
    {
        const std::size_t index = isLeft ? 0u : 1u;
        auto& state = _returningHandVisuals[index].transition;
        if (!g_rockConfig.rockWeaponVisualReturnEnabled ||
            !_hasLastPublishedHandWorld[index] ||
            !isUsableHandAuthorityTransform(_lastPublishedHandWorld[index]) ||
            !frik_visual_authority::isAvailable()) {
            clearHandVisualReturn(isLeft, "not-eligible", false);
            return;
        }

        state.begin(_lastPublishedHandWorld[index]);
        if (!frik_visual_authority::applyExternalHandWorldTransform(
                RETURN_HAND_TAG,
                handFromBool(isLeft),
                state.start,
                RETURN_HAND_VISUAL_PRIORITY)) {
            state.clear();
            clearPreFrikRetainedHandAuthority(
                RetainedHandAuthorityKind::Return,
                isLeft);
            (void)frik_visual_authority::clearExternalHandWorldTransform(RETURN_HAND_TAG, handFromBool(isLeft));
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: hand return start failed hand={}", isLeft ? "left" : "right");
            return;
        }

        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: hand return started hand={} reason={} from=({:.2f},{:.2f},{:.2f})",
            isLeft ? "left" : "right",
            reason ? reason : "unknown",
            state.start.translate.x,
            state.start.translate.y,
            state.start.translate.z);
    }

    void TwoHandedGrip::updateHandVisualReturns(const float dt)
    {
        if (_scopeMenuOpenThisFrame) {
            return;
        }

        for (const bool isLeft : { true, false }) {
            const std::size_t index = isLeft ? 0u : 1u;
            auto& state = _returningHandVisuals[index].transition;
            if (!state.active) {
                continue;
            }

            RE::NiTransform targetWorld{};
            if (!frik_visual_authority::isAvailable() ||
                !tryGetSolverHandTransform(isLeft, targetWorld) ||
                !isUsableHandAuthorityTransform(targetWorld)) {
                clearHandVisualReturn(isLeft, "tracked-hand-unavailable", true);
                continue;
            }

            const bool timingPending = !state.durationInitialized;
            const float initialDistance = timingPending ?
                hand_visual_lerp_math::distanceGameUnits(state.start.translate, targetWorld.translate) :
                0.0f;
            const float initialAngleDegrees = timingPending ?
                hand_visual_lerp_math::rotationDistanceDegrees(state.start, targetWorld) :
                0.0f;
            const auto result = hand_visual_lerp_math::advanceVisualReturn(
                state,
                targetWorld,
                dt,
                hand_visual_lerp_math::VisualReturnConfig{
                    .minSeconds = g_rockConfig.rockWeaponVisualReturnTimeMin,
                    .maxSeconds = g_rockConfig.rockWeaponVisualReturnTimeMax,
                    .minDistanceGameUnits = g_rockConfig.rockWeaponVisualReturnMinDistance,
                    .maxDistanceGameUnits = g_rockConfig.rockWeaponVisualReturnMaxDistance,
                    .minAngleDegrees = g_rockConfig.rockWeaponVisualReturnMinAngleDegrees,
                    .maxAngleDegrees = g_rockConfig.rockWeaponVisualReturnMaxAngleDegrees,
                });
            if (timingPending) {
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: hand return timing hand={} distance={:.2f}gu angle={:.1f}deg duration={:.3f}s",
                    isLeft ? "left" : "right",
                    initialDistance,
                    initialAngleDegrees,
                    state.durationSeconds);
            }
            if (!isUsableHandAuthorityTransform(result.transform) ||
                !frik_visual_authority::applyExternalHandWorldTransform(
                    RETURN_HAND_TAG,
                    handFromBool(isLeft),
                    result.transform,
                    RETURN_HAND_VISUAL_PRIORITY)) {
                clearHandVisualReturn(isLeft, "publish-failed", true);
                continue;
            }
            recordPreFrikRetainedHandAuthority(
                RetainedHandAuthorityKind::Return,
                isLeft,
                result.transform);

            if (result.reachedTarget) {
                const float completedDuration = state.durationSeconds;
                (void)frik_visual_authority::clearExternalHandWorldTransform(RETURN_HAND_TAG, handFromBool(isLeft));
                state.clear();
                clearPreFrikRetainedHandAuthority(
                    RetainedHandAuthorityKind::Return,
                    isLeft);
                _hasLastPublishedHandWorld[index] = false;
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: hand return completed hand={} duration={:.3f}s",
                    isLeft ? "left" : "right",
                    completedDuration);
            }
        }
    }

    void TwoHandedGrip::clearHandVisualReturn(const bool isLeft, const char* reason, const bool logCancellation)
    {
        const std::size_t index = isLeft ? 0u : 1u;
        auto& state = _returningHandVisuals[index].transition;
        const bool wasActive = state.active;
        (void)frik_visual_authority::clearExternalHandWorldTransform(RETURN_HAND_TAG, handFromBool(isLeft));
        state.clear();
        clearPreFrikRetainedHandAuthority(
            RetainedHandAuthorityKind::Return,
            isLeft);
        _hasLastPublishedHandWorld[index] = false;
        if (wasActive && logCancellation) {
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: hand return cancelled hand={} reason={}",
                isLeft ? "left" : "right",
                reason ? reason : "unknown");
        }
    }

    void TwoHandedGrip::cancelHandVisualReturn(const bool isLeft, const char* reason)
    {
        clearHandVisualReturn(isLeft, reason, true);
    }

    void TwoHandedGrip::beginWeaponVisualReturn(const char* reason)
    {
        if (!g_rockConfig.rockWeaponVisualReturnEnabled ||
            _returningWeaponVisual.localTransition.active ||
            !_activeWeaponNode ||
            !_hasWeaponNodeLocalBaseline ||
            _activeWeaponGenerationKey == 0 ||
            _activeEquippedWeaponOwnershipKey == 0) {
            return;
        }

        RE::NiTransform startWorld = _hasLastRenderedWeaponWorld ? _lastRenderedWeaponWorld : _activeWeaponNode->world;
        if (!isFiniteTransform(startWorld) || !isFiniteTransform(_weaponNodeLocalBaseline)) {
            return;
        }

        RE::NiNode* nativeParent = _activeWeaponNode->parent;
        if (_weaponNodeReparentedToLeftHand) {
            nativeParent = resolveFirstPersonHandNode(false);
            if (!nativeParent) {
                return;
            }
        }
        if (!nativeParent) {
            return;
        }

        const RE::NiTransform startLocal = weapon_visual_authority_math::worldTargetToParentLocal(nativeParent->world, startWorld);
        if (!isFiniteTransform(startLocal)) {
            return;
        }

        /*
         * blockPrimaryWeaponNodeOwnership is hFRIK's external LEFT-carry
         * topology switch, not a transform-write-only blocker. Retaining it
         * here makes hFRIK reparent the weapon back under LArm_Hand on the next
         * frame, which invalidates this right-parent-local return and snaps the
         * weapon immediately. Release left-carry topology before beginning the
         * overlay. ROCK runs after hFRIK and republishes the interpolated node
         * every frame, so hFRIK's earlier native write cannot reach rendering;
         * at the exact endpoint both writers already agree on the baseline.
         */
        releaseFiringHandWeaponNodeOwnership(_activeWeaponNode);
        if (_activeWeaponNode->parent != nativeParent) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: weapon return skipped because native right-hand parenting could not be restored");
            return;
        }

        ReturningWeaponVisualState returnState{};
        returnState.weaponNode = _activeWeaponNode;
        returnState.nativeParent = nativeParent;
        returnState.weaponGenerationKey = _activeWeaponGenerationKey;
        returnState.equippedWeaponOwnershipKey = _activeEquippedWeaponOwnershipKey;
        returnState.nativeBaselineLocal = _weaponNodeLocalBaseline;
        returnState.retainPrimaryPoseBlocker = _firingHandIsLeft;
        returnState.localTransition.begin(startLocal);
        returnState.localTransition.durationSeconds = hand_visual_lerp_math::computeVisualReturnDuration(
            startLocal,
            returnState.nativeBaselineLocal,
            hand_visual_lerp_math::VisualReturnConfig{
                .minSeconds = g_rockConfig.rockWeaponVisualReturnTimeMin,
                .maxSeconds = g_rockConfig.rockWeaponVisualReturnTimeMax,
                .minDistanceGameUnits = g_rockConfig.rockWeaponVisualReturnMinDistance,
                .maxDistanceGameUnits = g_rockConfig.rockWeaponVisualReturnMaxDistance,
                .minAngleDegrees = g_rockConfig.rockWeaponVisualReturnMinAngleDegrees,
                .maxAngleDegrees = g_rockConfig.rockWeaponVisualReturnMaxAngleDegrees,
            });
        returnState.localTransition.durationInitialized = true;
        if (!moveWeaponPresentationRigidly(_activeWeaponNode, startWorld)) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                2000,
                "TwoHandedGrip: weapon return rejected an invalid presentation subtree");
            return;
        }
        _returningWeaponVisual = returnState;
        _lastRenderedWeaponWorld = _activeWeaponNode->world;
        _hasLastRenderedWeaponWorld = true;
        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: weapon return started reason={} distance={:.2f}gu angle={:.1f}deg duration={:.3f}s",
            reason ? reason : "unknown",
            hand_visual_lerp_math::distanceGameUnits(startLocal.translate, returnState.nativeBaselineLocal.translate),
            hand_visual_lerp_math::rotationDistanceDegrees(startLocal, returnState.nativeBaselineLocal),
            _returningWeaponVisual.localTransition.durationSeconds);
    }

    void TwoHandedGrip::updateWeaponVisualReturn(
        RE::NiNode* currentWeaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey,
        const float dt)
    {
        auto& state = _returningWeaponVisual;
        if (!state.localTransition.active) {
            return;
        }
        if (!runtime_state::isLocalSkeletonReady() ||
            !currentWeaponNode ||
            currentWeaponNode != state.weaponNode ||
            currentWeaponGenerationKey != state.weaponGenerationKey ||
            currentEquippedWeaponOwnershipKey != state.equippedWeaponOwnershipKey ||
            !state.nativeParent ||
            !isFiniteTransform(state.nativeParent->world) ||
            currentWeaponNode->parent != state.nativeParent) {
            clearAllVisualReturns("weapon-identity-or-parent-changed", true, true);
            return;
        }

        const auto result = hand_visual_lerp_math::advanceVisualReturn(
            state.localTransition,
            state.nativeBaselineLocal,
            dt,
            hand_visual_lerp_math::VisualReturnConfig{
                .minSeconds = g_rockConfig.rockWeaponVisualReturnTimeMin,
                .maxSeconds = g_rockConfig.rockWeaponVisualReturnTimeMax,
                .minDistanceGameUnits = g_rockConfig.rockWeaponVisualReturnMinDistance,
                .maxDistanceGameUnits = g_rockConfig.rockWeaponVisualReturnMaxDistance,
                .minAngleDegrees = g_rockConfig.rockWeaponVisualReturnMinAngleDegrees,
                .maxAngleDegrees = g_rockConfig.rockWeaponVisualReturnMaxAngleDegrees,
            });
        if (!isFiniteTransform(result.transform)) {
            clearWeaponVisualReturn("non-finite-return-transform", true, true);
            return;
        }

        const RE::NiTransform returnedWeaponWorld =
            transform_math::composeTransforms(state.nativeParent->world, result.transform);
        if (!applyWeaponVisualAuthority(currentWeaponNode, returnedWeaponWorld, state.weaponGenerationKey)) {
            clearWeaponVisualReturn("weapon-return-publish-failed", true, true);
            return;
        }
        _lastSolvedWeaponTransform = currentWeaponNode->world;
        _hasSolvedWeaponTransform = true;
        if (result.reachedTarget) {
            const float completedDuration = state.localTransition.durationSeconds;
            clearWeaponVisualReturn("completed", false, true);
            ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: weapon return completed duration={:.3f}s", completedDuration);
        }
    }

    void TwoHandedGrip::clearWeaponVisualReturn(const char* reason, const bool logCancellation, const bool restoreBlockers)
    {
        const bool wasActive = _returningWeaponVisual.localTransition.active;
        const bool retainedPrimaryPoseBlocker = _returningWeaponVisual.retainPrimaryPoseBlocker;
        RE::NiNode* returnNode = _returningWeaponVisual.weaponNode;
        _returningWeaponVisual = {};
        if (restoreBlockers) {
            releaseFiringHandWeaponNodeOwnership(returnNode);
            if (retainedPrimaryPoseBlocker) {
                restoreFrikPrimaryWeaponPose();
            }
        }
        if (wasActive && logCancellation) {
            ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: weapon return cancelled reason={}", reason ? reason : "unknown");
        }
    }

    void TwoHandedGrip::clearAllVisualReturns(const char* reason, const bool logCancellation, const bool restoreBlockers)
    {
        clearHandVisualReturn(true, reason, logCancellation);
        clearHandVisualReturn(false, reason, logCancellation);
        clearWeaponVisualReturn(reason, logCancellation, restoreBlockers);
    }

    RE::NiTransform TwoHandedGrip::resolveLockedHandVisualTarget(
        const RE::NiTransform& targetWorld,
        const RE::NiTransform* liveHandWorld,
        float dt,
        LockedHandVisualLerpState& state)
    {
        /*
         * Authored, provider-owned, and visual-only support paths retain their
         * independent external-hand transition. Normal dynamic full-authority
         * acquisition is intercepted by resolveDynamicSupportAcquisitionHandTarget
         * so both hands share the pivot-preserving weapon correction alpha.
         */
        if (!g_rockConfig.rockWeaponSupportGripHandLerpEnabled) {
            state = {};
            return targetWorld;
        }

        if (!state.active) {
            const RE::NiTransform startWorld = (liveHandWorld && isFiniteTransform(*liveHandWorld)) ? *liveHandWorld : targetWorld;
            const float initialDistance =
                hand_visual_lerp_math::distanceGameUnits(startWorld.translate, targetWorld.translate);
            const float durationSeconds =
                hand_visual_lerp_math::computeDistanceMappedDurationGameUnits(
                    initialDistance,
                    g_rockConfig.rockWeaponSupportGripHandLerpTimeMin,
                    g_rockConfig.rockWeaponSupportGripHandLerpTimeMax,
                    g_rockConfig.rockWeaponSupportGripHandLerpMinDistance,
                    g_rockConfig.rockWeaponSupportGripHandLerpMaxDistance);
            if (durationSeconds <= 0.0f) {
                state = {};
                state.lastAlpha = 1.0f;
                return targetWorld;
            }

            state.active = true;
            state.startWorld = startWorld;
            state.elapsedSeconds = 0.0f;
            state.durationSeconds = durationSeconds;
            state.lastAlpha = 0.0f;
        }

        state.elapsedSeconds =
            hand_visual_lerp_math::advanceTimedBlendElapsed(state.elapsedSeconds, dt, state.durationSeconds);
        const auto blended =
            hand_visual_lerp_math::blendTransformOverDuration(state.startWorld, targetWorld, state.elapsedSeconds, state.durationSeconds);
        state.lastAlpha = hand_visual_lerp_math::timedBlendAlpha(state.elapsedSeconds, state.durationSeconds);
        return blended.transform;
    }

    void TwoHandedGrip::transitionToTouching(RE::NiNode* weaponNode, const WeaponInteractionDecision& decision)
    {
        if (!weaponNode) {
            _state = TwoHandedState::Inactive;
            return;
        }

        _state = TwoHandedState::Touching;
        _touchFrames = 0;
        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: touching weapon='{}' bodyId={} partKind={} pose={} interactionRoot={:x} sourceRoot={:x} generation={:016X}",
            weaponNode->name.c_str(),
            decision.bodyId,
            static_cast<int>(decision.partKind),
            static_cast<int>(decision.gripPose),
            reinterpret_cast<std::uintptr_t>(decision.interactionRoot),
            reinterpret_cast<std::uintptr_t>(decision.sourceRoot),
            decision.weaponGenerationKey);
    }

    void TwoHandedGrip::transitionToGripping(
        RE::NiNode* weaponNode,
        const WeaponInteractionDecision& decision,
        const WeaponCollision& weaponCollision,
        weapon_support_authority_policy::WeaponSupportAuthorityMode supportAuthorityMode,
        bool firingGripProximityAuthorityEnabled,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const WeaponProviderPartAuthority& providerPartAuthority)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::TwoHandedGripStart);

        if (!weaponNode || currentEquippedWeaponOwnershipKey == 0) {
            transitionToInactive(false);
            return;
        }
        clearDynamicSupportAcquisition(
            "new-two-hand-acquisition",
            true);

        const bool supportHandIsLeft = !_firingHandIsLeft;
        const bool primaryHandIsLeft = _firingHandIsLeft;
        RE::NiTransform nativeWeaponLocalBaseline = weaponNode->local;
        if (_returningWeaponVisual.localTransition.active && _returningWeaponVisual.weaponNode == weaponNode) {
            nativeWeaponLocalBaseline = _returningWeaponVisual.nativeBaselineLocal;
            clearWeaponVisualReturn("new-two-hand-acquisition", true, true);
        }

        /*
         * A LEFT firing hand entering a two-handed grip KEEPS its captured
         * firing-grip frames: they hold the mirrored canonical hold
         * (takeover-committed), and recapturing from the live hand both
         * replaced that authored hold with the momentary squeeze orientation
         * (round-2 arm break) and rebased the promotion grip point onto
         * whatever pose the node carried at grab time (round-4 role theft).
         * The right hand recaptures as before - its frames deliberately ride
         * FRIK's authored carry and feed the canonical snapshot.
         */
        const bool keepLeftFiringHold = _firingHandIsLeft && _hasFiringHandWeaponLocal;

        _authorityMode = supportAuthorityMode;
        _activeWeaponNode = weaponNode;
        _activeWeaponGenerationKey = decision.weaponGenerationKey;
        _activeEquippedWeaponOwnershipKey = currentEquippedWeaponOwnershipKey;
        _weaponNodeLocalBaseline = nativeWeaponLocalBaseline;
        _hasWeaponNodeLocalBaseline = true;
        if (!keepLeftFiringHold) {
            _primaryGripConfidence = 0.0f;
            _hasFiringHandWeaponLocal = false;
        }
        resetLockedHandVisualLerp();
        clearPrimaryGripPose(primaryHandIsLeft);
        clearSupportGripPose(supportHandIsLeft);

        killFrikOffhandGrip();

        RE::NiTransform primaryTransform{};
        if (!tryGetSolverHandTransform(primaryHandIsLeft, primaryTransform)) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: support grip start skipped because authoritative hand transforms are unavailable");
            restoreFrikOffhandGrip();
            return;
        }

        const bool reuseRightFiringCanonicalGrip = scope_safe_hand_frame_math::shouldReuseRightFiringCanonicalGrip(_scopeMenuOpenThisFrame, _firingHandIsLeft,
            hasRightFiringHandCanonicalFrame(
                weaponNode,
                decision.weaponGenerationKey,
                currentEquippedWeaponOwnershipKey),
            _rightFiringHandCanonicalGenerationKey,
            decision.weaponGenerationKey);
        if (_scopeMenuOpenThisFrame && !_firingHandIsLeft && !reuseRightFiringCanonicalGrip) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                "TwoHandedGrip: scoped support grip start deferred because the matching pre-scope firing grip is unavailable generation={:016X} canonicalGeneration={:016X}",
                decision.weaponGenerationKey, _rightFiringHandCanonicalGenerationKey);
            restoreFrikOffhandGrip();
            return;
        }
        if (reuseRightFiringCanonicalGrip) {
            _primaryHandWeaponLocal = _rightFiringHandCanonicalWeaponLocal;
            _primaryGripLocal = _rightFiringGripCanonicalWeaponLocal;
        }

        const RE::NiPoint3 primaryPalmPos =
            reuseRightFiringCanonicalGrip ? weaponLocalToWorld(_primaryGripLocal, weaponNode) : computeGrabLegacyPalmPivotAWorldFromHandBasis(primaryTransform, primaryHandIsLeft);
        if (!keepLeftFiringHold) {
            if (!reuseRightFiringCanonicalGrip) {
                _primaryGripLocal = worldToWeaponLocal(primaryPalmPos, weaponNode);
                _primaryHandWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), primaryTransform);
            }
            _primaryGripConfidence = 1.0f;
            _hasFiringHandWeaponLocal = true;
            _firingGripSequence = ++_gripCaptureSequence;
            // A right-hand capture here rides FRIK's authored carry: snapshot it
            // as the canonical hold that left takeovers apply mirrored.
            rememberRightFiringHandCanonicalFrame();
        }

        /*
         * At capture the firing grip point is the primary palm, so the support
         * palm distance selects visual-only attachment near the firing grip or
         * full two-handed manipulation farther out. This applies uniformly to
         * equipped weapons and is bypassed by explicit provider grab modes.
         * If the distance cannot be measured, retain full authority rather
         * than assuming the hand is inside the proximity radius.
         */
        if (firingGripProximityAuthorityEnabled) {
            RE::NiTransform supportTransform{};
            if (tryGetSolverHandTransform(supportHandIsLeft, supportTransform)) {
                const RE::NiPoint3 supportPalmPos = computeGrabLegacyPalmPivotAWorldFromHandBasis(supportTransform, supportHandIsLeft);
                const RE::NiPoint3 supportToGrip = sub(primaryPalmPos, supportPalmPos);
                const float supportPalmToGripDistance = std::sqrt(dot(supportToGrip, supportToGrip));
                if (std::isfinite(supportPalmToGripDistance)) {
                    _authorityMode = weapon_support_authority_policy::resolveFiringGripProximityAuthorityMode(
                        supportPalmToGripDistance,
                        _handlingSettings.firingGripProximitySupportRadiusGameUnits);
                    ROCK_LOG_INFO(Weapon,
                        "TwoHandedGrip: firing-grip proximity support distance={:.2f} radius={:.2f} mode={}",
                        supportPalmToGripDistance,
                        _handlingSettings.firingGripProximitySupportRadiusGameUnits,
                        _authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport ?
                            "visual-only" :
                            "full-authority");
                }
            }
        }

        RE::NiTransform supportCaptureHandWorld{};
        if (!capturePartGrip(
                supportHandIsLeft,
                weaponNode,
                decision,
                weaponCollision,
                providerPartAuthority,
                firingGripProximityAuthorityEnabled,
                &supportCaptureHandWorld)) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: support grip start skipped because part grip capture failed");
            restoreFrikOffhandGrip();
            return;
        }

        const RE::NiPoint3 supportGripWorldPoint = resolvePartGripWorld(partGrip(supportHandIsLeft), weaponNode);
        const RE::NiPoint3 primaryToSupportWorld = sub(supportGripWorldPoint, primaryPalmPos);
        _lockedGripSeparationWorld = std::sqrt(dot(primaryToSupportWorld, primaryToSupportWorld));

        WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        if (!initializeGunstockSupportRole(
                weaponNode,
                supportHandIsLeft,
                supportCaptureHandWorld,
                "support-attach")) {
            transitionToInactive(false);
            return;
        }
        const bool gunstockBaselineActive =
            isGunstockSupportBaselineActive(
                supportHandIsLeft,
                supportGrip);

        const bool useDynamicSupportAcquisition =
            !gunstockBaselineActive &&
            weapon_support_authority_policy::
                shouldUseDynamicSupportAcquisition(
                    _authorityMode,
                    supportGrip.authoredSupportGrip,
                    supportGrip.providerPartAuthority.active,
                    supportGrip.attachOnly);
        if (useDynamicSupportAcquisition &&
            !initializeDynamicSupportBaseline(
                weaponNode,
                supportHandIsLeft,
                "support-attach")) {
            ROCK_LOG_WARN(
                Weapon,
                "TwoHandedGrip: support grip start failed closed because the zero-delta dynamic baseline could not be captured hand={} grip={} generation={:016X}",
                supportHandIsLeft ? "left" : "right",
                supportGrip.gripSequence,
                supportGrip.weaponGenerationKey);
            transitionToInactive(false);
            return;
        }
        const bool dynamicBaselineActive =
            isDynamicSupportBaselineActive(
                supportHandIsLeft,
                supportGrip);
        RE::NiTransform dynamicPrimaryStartWorld{};
        RE::NiTransform dynamicSupportStartWorld{};
        if (useDynamicSupportAcquisition) {
            const auto& primaryReturn =
                _returningHandVisuals[
                    primaryHandIsLeft ? 0u : 1u]
                    .transition;
            const auto& supportReturn =
                _returningHandVisuals[
                    supportHandIsLeft ? 0u : 1u]
                    .transition;
            dynamicPrimaryStartWorld =
                primaryReturn.active &&
                    isUsableHandAuthorityTransform(
                        primaryReturn.lastApplied) ?
                primaryReturn.lastApplied :
                primaryTransform;
            dynamicSupportStartWorld =
                supportReturn.active &&
                    isUsableHandAuthorityTransform(
                        supportReturn.lastApplied) ?
                supportReturn.lastApplied :
                supportCaptureHandWorld;
        }

        _state = TwoHandedState::Gripping;
        _rotationBlend = gunstockBaselineActive ? 1.0f : 0.0f;
        _gripLogCounter = 0;
        _supportGripAgeFrames = 0;
        _freshSupportGripDeferLogged = false;

        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: grip active weapon='{}', "
            "primaryLocal=({:.3f},{:.3f},{:.3f}), supportLocal=({:.3f},{:.3f},{:.3f}), "
            "gripSeparation={:.3f}, primaryGripSource={}, primaryGripConfidence={:.2f}, partKind={}, pose={}, authorityMode={}, supportBaseline={}, generation={:016X}",
            weaponNode->name.c_str(), _primaryGripLocal.x, _primaryGripLocal.y, _primaryGripLocal.z, supportGrip.gripLocal.x, supportGrip.gripLocal.y, supportGrip.gripLocal.z,
            _lockedGripSeparationWorld, reuseRightFiringCanonicalGrip ? "pre-scope-canonical" : (_scopeMenuOpenThisFrame ? "frik-driver-reconstructed" : "root-flattened"),
            _primaryGripConfidence, static_cast<int>(supportGrip.partKind), static_cast<int>(supportGrip.gripPose), static_cast<int>(_authorityMode), gunstockBaselineActive ? "gunstock" : (dynamicBaselineActive ? "dynamic" : "inactive"), _activeWeaponGenerationKey);

        if (gunstockBaselineActive) {
            /*
             * Publish the attach transaction immediately. The solver path
             * explicitly retains weaponWorldAtCapture for this publication;
             * the support hand may begin its normal visual seat while the
             * primary weapon pose remains bit-for-bit the authority target.
             */
            updateFullWeaponAuthorityGrip(weaponNode, 0.0f);
        } else if (useDynamicSupportAcquisition) {
            beginDynamicSupportAcquisition(
                supportHandIsLeft,
                supportGrip,
                dynamicPrimaryStartWorld,
                dynamicSupportStartWorld);
            /*
             * Capture and first publication are one transaction. dt=0 keeps
             * alpha exactly zero while publishing the frozen finger pose, the
             * pivot-preserving one-hand weapon frame, and both live hand roots
             * before this update returns to PhysicsInteraction.
             */
            updateFullWeaponAuthorityGrip(weaponNode, 0.0f);
        }
    }

    void TwoHandedGrip::clearActiveGripState()
    {
        _state = TwoHandedState::Inactive;
        _touchFrames = 0;
        _rotationBlend = 0.0f;
        _partGrips = {};
        _partCarryPivotIsLeft = true;
        _partCarryGripSeparationWorld = 0.0f;
        _primaryGripLocal = {};
        _lockedGripSeparationWorld = 0.0f;
        _supportGripAgeFrames = 0;
        _freshSupportGripDeferLogged = false;
        _authorityMode = weapon_support_authority_policy::
            WeaponSupportAuthorityMode::FullTwoHandedSolver;
        _primaryHandWeaponLocal = {};
        _hasFiringHandWeaponLocal = false;
        _primaryGripConfidence = 0.0f;
        _activeWeaponNode = nullptr;
        _activeWeaponGenerationKey = 0;
        _activeEquippedWeaponOwnershipKey = 0;
        _primaryReleaseDebounce = {};
        _persistentEquippedCarryActive = false;
        _persistentEquippedCarryDetachArmed = false;
        _weaponNodeLocalBaseline = {};
        _hasWeaponNodeLocalBaseline = false;
        resetLockedHandVisualLerp();
    }

    void TwoHandedGrip::transitionToInactive(bool publishRestoredWeaponTransform)
    {
        clearFiringRecoilPresentationState();
        clearDynamicSupportAcquisition(
            "transition-to-inactive",
            true);
        const bool weaponReturnActive = _returningWeaponVisual.localTransition.active;
        // Weapon-node topology always returns to native immediately. A visual
        // return owns only ROCK's later transform publication, never hFRIK's
        // external-left-carry topology switch.
        releaseFiringHandWeaponNodeOwnership(_activeWeaponNode);
        clearPrimaryGripPose(_firingHandIsLeft);
        clearPrimaryDetachVisualAuthority(_firingHandIsLeft);
        clearSupportGripPose(true);
        clearSupportGripPose(false);
        restoreFrikOffhandGrip();
        if (!weaponReturnActive) {
            restoreFrikPrimaryWeaponPose();
        }
        bool restoredWeaponTransformAvailable = false;
        RE::NiTransform restoredWeaponTransform{};
        if (publishRestoredWeaponTransform && _hasWeaponNodeLocalBaseline && _activeWeaponNode) {
            if (_activeWeaponNode->parent) {
                restoredWeaponTransform = transform_math::composeTransforms(_activeWeaponNode->parent->world, _weaponNodeLocalBaseline);
            } else {
                restoredWeaponTransform = _weaponNodeLocalBaseline;
            }
            restoredWeaponTransformAvailable = true;
        }

        _hasSolvedWeaponTransform = weaponReturnActive || (publishRestoredWeaponTransform && restoredWeaponTransformAvailable);
        if (weaponReturnActive && _hasLastRenderedWeaponWorld) {
            _lastSolvedWeaponTransform = _lastRenderedWeaponWorld;
        } else if (_hasSolvedWeaponTransform) {
            _lastSolvedWeaponTransform = restoredWeaponTransform;
        }
        clearActiveGripState();

        // A grip release does not reset the wider presentation session.
        // Scope, gunstock, canonical, and return state must survive here.
        if (!isHandVisualReturnActive(true)) {
            _hasLastPublishedHandWorld[0] = false;
        }
        if (!isHandVisualReturnActive(false)) {
            _hasLastPublishedHandWorld[1] = false;
        }
        if (!weaponReturnActive) {
            _hasLastRenderedWeaponWorld = false;
        }
        // The firing-hand role is grip-session state: outside manual
        // ownership the weapon is FRIK/native-carried by the right hand.
        _firingHandIsLeft = false;

        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: grip released");
    }

    void TwoHandedGrip::updateGripping(RE::NiNode* weaponNode, float dt)
    {
        if (_authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport) {
            updateVisualOnlySupportGrip(weaponNode, dt);
            return;
        }

        updateFullWeaponAuthorityGrip(weaponNode, dt);
    }

    bool TwoHandedGrip::transitionToPartCarry()
    {
        if (_state == TwoHandedState::PartCarry) {
            return true;
        }

        if (!blockFrikPrimaryWeaponPose()) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: primary detach skipped because hFRIK primary weapon-pose blocker is unavailable");
            return false;
        }
        clearFiringRecoilPresentationState();
        clearDynamicSupportAcquisition(
            "transition-to-part-carry",
            true);
        // PartCarry has no firing/support role split. Any prior support
        // calibration is stale after the weapon is driven by part grips and
        // must be recaptured if a firing grip later re-establishes Gripping.
        clearSupportInputBaselines();
        beginHandVisualReturn(_firingHandIsLeft, "primary-detach-part-carry");
        clearPrimaryGripPose(_firingHandIsLeft);
        _primaryHandVisualLerp = {};
        partGrip(!_firingHandIsLeft).visualLerp = {};
        lockPartGripToWeaponRoot(!_firingHandIsLeft);
        _rotationBlend = 1.0f;
        _partCarryPivotIsLeft = !_firingHandIsLeft;
        _partCarryGripSeparationWorld = 0.0f;
        _state = TwoHandedState::PartCarry;
        _hapticEvents.firingGripDetached = true;
        _hapticEvents.firingGripDetachedHandIsLeft = _firingHandIsLeft;
        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: firing hand detached; part grips own equipped weapon authority");
        return true;
    }

    bool TwoHandedGrip::republishPartCarryWeaponTransform(RE::NiNode* weaponNode)
    {
        if (_state != TwoHandedState::PartCarry || !_hasSolvedWeaponTransform || !weaponNode) {
            return false;
        }
        return applyWeaponVisualAuthority(weaponNode, _lastSolvedWeaponTransform);
    }

    bool TwoHandedGrip::canBeginPrimaryOnlyGripForHand(const bool isLeft)
    {
        return !isLeft || leftFiringInfrastructureAvailable();
    }

    bool TwoHandedGrip::tryBuildCurrentLeftFiringGripCapture(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey,
        RE::NiTransform& outFiringHandWeaponLocal,
        RE::NiPoint3& outFiringGripWeaponLocal) const
    {
        outFiringHandWeaponLocal = {};
        outFiringGripWeaponLocal = {};
        if (!canBeginPrimaryOnlyGripForHand(true) ||
            !hasRightFiringHandCanonicalFrame(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey)) {
            return false;
        }

        RE::NiTransform rightHandWorld{};
        RE::NiTransform leftHandWorld{};
        if (!tryGetSolverHandTransform(false, rightHandWorld) ||
            !tryGetSolverHandTransform(true, leftHandWorld) ||
            !tryBuildMirroredLeftFiringHandWeaponLocal(
                _rightFiringHandCanonicalWeaponLocal,
                _rightFiringGripCanonicalWeaponLocal,
                rightHandWorld,
                leftHandWorld,
                outFiringHandWeaponLocal,
                true)) {
            return false;
        }

        outFiringGripWeaponLocal = _rightFiringGripCanonicalWeaponLocal;
        return true;
    }

    bool TwoHandedGrip::tryCaptureLeftFiringGripTransfer(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey,
        RE::NiTransform& outFiringHandWeaponLocal,
        RE::NiPoint3& outFiringGripWeaponLocal) const
    {
        outFiringHandWeaponLocal = {};
        outFiringGripWeaponLocal = {};

        const bool activeLeftCaptureCurrent =
            isManualOwnershipActive() &&
            _firingHandIsLeft &&
            _activeWeaponNode == weaponNode &&
            currentEquippedWeaponOwnershipKey != 0 &&
            _activeEquippedWeaponOwnershipKey ==
                currentEquippedWeaponOwnershipKey &&
            _hasFiringHandWeaponLocal &&
            isFiniteTransform(_primaryHandWeaponLocal) &&
            _primaryGripConfidence > 0.0f &&
            std::isfinite(_primaryGripLocal.x) &&
            std::isfinite(_primaryGripLocal.y) &&
            std::isfinite(_primaryGripLocal.z);
        if (activeLeftCaptureCurrent) {
            outFiringHandWeaponLocal = _primaryHandWeaponLocal;
            outFiringGripWeaponLocal = _primaryGripLocal;
            return true;
        }

        return tryBuildCurrentLeftFiringGripCapture(
            weaponNode,
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey,
            outFiringHandWeaponLocal,
            outFiringGripWeaponLocal);
    }

    bool TwoHandedGrip::beginPrimaryOnlyGrip(
        RE::NiNode* weaponNode,
        std::uint64_t currentWeaponGenerationKey,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const bool firingHandIsLeft,
        const RE::NiTransform* capturedFiringHandWeaponLocal,
        const RE::NiPoint3* capturedFiringGripWeaponLocal,
        const bool retainUntilPhysicalGrip)
    {
        if (!weaponNode || currentEquippedWeaponOwnershipKey == 0 || _state != TwoHandedState::Inactive ||
            !canBeginPrimaryOnlyGripForHand(firingHandIsLeft)) {
            return false;
        }
        if (firingHandIsLeft &&
            (!capturedFiringHandWeaponLocal || !isFiniteTransform(*capturedFiringHandWeaponLocal) ||
                !capturedFiringGripWeaponLocal ||
                !std::isfinite(capturedFiringGripWeaponLocal->x) ||
                !std::isfinite(capturedFiringGripWeaponLocal->y) ||
                !std::isfinite(capturedFiringGripWeaponLocal->z))) {
            return false;
        }
        if (firingHandIsLeft && !blockFrikPrimaryWeaponPose()) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "TwoHandedGrip: left primary-grip start skipped because the hFRIK primary weapon-pose blocker is unavailable");
            return false;
        }

        setFiringHand(firingHandIsLeft, "primary-grip-start-hand");
        if (firingHandIsLeft) {
            _primaryHandWeaponLocal = *capturedFiringHandWeaponLocal;
            _hasFiringHandWeaponLocal = true;
        }

        if (!transitionToPrimaryOnly(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                "primary-grip-start")) {
            _primaryHandWeaponLocal = {};
            _hasFiringHandWeaponLocal = false;
            setFiringHand(false, "primary-grip-start-failed");
            restoreFrikPrimaryWeaponPose();
            return false;
        }
        if (firingHandIsLeft) {
            // The newly equipped node inherits the exact loose-model firing
            // grip; transitionToPrimaryOnly must not recapture it from the
            // left palm against FRIK's still-right-native first frame.
            _primaryGripLocal = *capturedFiringGripWeaponLocal;
            _primaryGripConfidence = 1.0f;
        }
        // Only a fresh grab pulses; transitionToPrimaryOnly is also reached
        // from support-release paths where the firing grip never changed.
        _hapticEvents.firingGripAttached = true;
        _hapticEvents.firingGripAttachedHandIsLeft = _firingHandIsLeft;
        _firingGripSequence = ++_gripCaptureSequence;
        if (retainUntilPhysicalGrip) {
            _persistentEquippedCarryActive = true;
            _persistentEquippedCarryDetachArmed = false;
        }
        return true;
    }

    bool TwoHandedGrip::beginPersistentEquippedCarry(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey)
    {
        if (!weaponNode || currentWeaponGenerationKey == 0 || currentEquippedWeaponOwnershipKey == 0 ||
            _state != TwoHandedState::Inactive || !canBeginPrimaryOnlyGripForHand(true) ||
            !hasRightFiringHandCanonicalFrame(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey)) {
            return false;
        }

        RE::NiTransform mirroredLeftHold{};
        RE::NiPoint3 firingGripWeaponLocal{};
        if (!tryBuildCurrentLeftFiringGripCapture(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                mirroredLeftHold,
                firingGripWeaponLocal)) {
            return false;
        }

        if (!beginPrimaryOnlyGrip(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                true,
                &mirroredLeftHold,
                &firingGripWeaponLocal)) {
            return false;
        }

        _persistentEquippedCarryActive = true;
        _persistentEquippedCarryDetachArmed = false;
        const bool usedAuthoredCanonical =
            _rightFiringHandCanonicalSource ==
            RightFiringCanonicalSource::AuthoredAnimation;
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: persistent left-hand carry active generation={:016X} ownership={:016X} source={} capture={}",
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey,
            usedAuthoredCanonical ? "authored-animation" : "native-carry",
            _rightFiringHandCanonicalCaptureSequence);
        return true;
    }

    void TwoHandedGrip::clearPersistentEquippedCarry(const char* reason)
    {
        if (!_persistentEquippedCarryActive) {
            return;
        }
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: clearing persistent left-hand carry reason={}",
            reason ? reason : "unknown");
        _persistentEquippedCarryActive = false;
        _persistentEquippedCarryDetachArmed = false;
        if (isManualOwnershipActive()) {
            transitionToInactive(false);
        }
    }

    void TwoHandedGrip::restoreNativeRightEquippedCarry(const char* reason)
    {
        clearPersistentEquippedCarry(reason);
        if (!isManualOwnershipActive()) {
            return;
        }

        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: restoring native right-hand carry reason={}",
            reason ? reason : "unknown");
        transitionToInactive(false);
    }

    bool TwoHandedGrip::publishLeftFiringFeedForwardWeaponPose(RE::NiNode* weaponNode)
    {
        if (!weaponNode || weaponNode != _activeWeaponNode ||
            (_state != TwoHandedState::Gripping && _state != TwoHandedState::PrimaryOnly) ||
            !_firingHandIsLeft || !_hasFiringHandWeaponLocal) {
            return false;
        }

        RE::NiTransform leftFiringHandTransform{};
        if (!tryGetSolverHandTransform(true, leftFiringHandTransform)) {
            return false;
        }

        const RE::NiTransform feedForwardWeaponWorld = transform_math::composeTransforms(
            leftFiringHandTransform, transform_math::invertTransform(_primaryHandWeaponLocal));
        if (!isFiniteTransform(feedForwardWeaponWorld)) {
            return false;
        }
        return applyWeaponVisualAuthority(weaponNode, feedForwardWeaponWorld);
    }

    EquippedWeaponManualDropRequest TwoHandedGrip::consumeEquippedWeaponDropRequest()
    {
        const EquippedWeaponManualDropRequest request = _equippedWeaponDropRequest;
        _equippedWeaponDropRequest = {};
        return request;
    }

    void TwoHandedGrip::getHandGripReport(bool isLeft, HandGripReport& outReport) const
    {
        outReport = {};
        const bool isFiringHand = isLeft == _firingHandIsLeft;
        const WeaponPartGrip& grip = partGrip(isLeft);
        const auto kind = weapon_part_grip_report_policy::resolveHandGripKind(
            _state == TwoHandedState::Gripping,
            _state == TwoHandedState::PartCarry,
            _state == TwoHandedState::PrimaryOnly,
            isFiringHand,
            grip.active,
            grip.attachOnly,
            _authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport);
        outReport.kind = kind;
        if (kind == weapon_part_grip_report_policy::HandGripKind::None) {
            return;
        }

        outReport.active = true;
        if (kind == weapon_part_grip_report_policy::HandGripKind::FiringGrip) {
            // In PrimaryOnly the weapon rides the FRIK-native hand attach and
            // ROCK holds no captured hand-to-weapon frame; hasHandPartLocal
            // stays false there by design.
            outReport.gripSequence = _firingGripSequence;
            outReport.weaponGenerationKey = _activeWeaponGenerationKey;
            outReport.sourceRoot = reinterpret_cast<std::uintptr_t>(_activeWeaponNode);
            outReport.hasHandPartLocal = _hasFiringHandWeaponLocal;
            outReport.handPartLocal = _primaryHandWeaponLocal;
            return;
        }

        outReport.attachOnly = grip.attachOnly;
        outReport.authoredSupportGrip = grip.authoredSupportGrip;
        outReport.gripSequence = grip.gripSequence;
        outReport.weaponGenerationKey = grip.weaponGenerationKey != 0 ? grip.weaponGenerationKey : _activeWeaponGenerationKey;
        outReport.bodyId = grip.contactBodyId;
        outReport.partKind = static_cast<std::uint32_t>(grip.partKind);
        outReport.reloadRole = static_cast<std::uint32_t>(grip.reloadRole);
        outReport.supportRole = static_cast<std::uint32_t>(grip.supportRole);
        outReport.socketRole = static_cast<std::uint32_t>(grip.socketRole);
        outReport.actionRole = static_cast<std::uint32_t>(grip.actionRole);
        outReport.sourceRoot = reinterpret_cast<std::uintptr_t>(grip.attachmentRoot);
        if (grip.providerPartAuthority.active) {
            outReport.providerOwnerToken = grip.providerPartAuthority.ownerToken;
            outReport.providerGroupId = grip.providerPartAuthority.groupId;
            outReport.providerGrabMode = grip.providerPartAuthority.grabMode;
        }
        outReport.hasHandPartLocal = grip.hasSourceFrames || grip.hasHandWeaponLocal;
        outReport.handPartLocalIsSourceLocal = grip.hasSourceFrames;
        outReport.handPartLocal = grip.hasSourceFrames ? grip.handSourceLocal : grip.handWeaponLocal;
        outReport.sourceName = grip.sourceName;
        outReport.omodFormId = grip.omodFormId;
        outReport.attachPointFormId = grip.attachPointFormId;
        outReport.classificationSource = static_cast<std::uint32_t>(grip.classificationSource);
    }

    TwoHandedGripHapticEvents TwoHandedGrip::consumeHapticEvents()
    {
        const TwoHandedGripHapticEvents events = _hapticEvents;
        _hapticEvents = {};
        return events;
    }

    bool TwoHandedGrip::transitionToPrimaryOnly(
        RE::NiNode* weaponNode,
        std::uint64_t currentWeaponGenerationKey,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const char* reason)
    {
        if (!weaponNode || currentEquippedWeaponOwnershipKey == 0) {
            return false;
        }

        clearDynamicSupportAcquisition(
            reason ? reason : "transition-to-primary-only",
            true);
        const bool primaryHandIsLeft = _firingHandIsLeft;
        const bool supportHandIsLeft = !_firingHandIsLeft;
        if (!primaryHandIsLeft) {
            clearFiringRecoilPresentationState(false);
        }

        if (_state == TwoHandedState::Inactive) {
            RE::NiTransform nativeWeaponLocalBaseline = weaponNode->local;
            if (_returningWeaponVisual.localTransition.active && _returningWeaponVisual.weaponNode == weaponNode) {
                nativeWeaponLocalBaseline = _returningWeaponVisual.nativeBaselineLocal;
                clearWeaponVisualReturn("new-primary-acquisition", true, true);
            }
            _activeWeaponNode = weaponNode;
            _activeWeaponGenerationKey = currentWeaponGenerationKey;
            _activeEquippedWeaponOwnershipKey = currentEquippedWeaponOwnershipKey;
            _weaponNodeLocalBaseline = nativeWeaponLocalBaseline;
            _hasWeaponNodeLocalBaseline = true;

            RE::NiTransform primaryTransform{};
            if (tryGetSolverHandTransform(primaryHandIsLeft, primaryTransform)) {
                _primaryGripLocal = worldToWeaponLocal(computeGrabLegacyPalmPivotAWorldFromHandBasis(primaryTransform, primaryHandIsLeft), weaponNode);
                _primaryGripConfidence = 1.0f;
            } else {
                _primaryGripLocal = {};
                _primaryGripConfidence = 0.0f;
            }
        }
        _activeWeaponGenerationKey = currentWeaponGenerationKey;
        _activeEquippedWeaponOwnershipKey = currentEquippedWeaponOwnershipKey;

        clearPrimaryGripPose(primaryHandIsLeft);
        clearSupportGripPose(supportHandIsLeft);
        clearSupportGripPose(primaryHandIsLeft);
        clearPrimaryDetachVisualAuthority(primaryHandIsLeft);
        restoreFrikOffhandGrip();
        if (!_firingHandIsLeft) {
            // FRIK's primary weapon pose targets the game-primary RIGHT hand.
            // While the LEFT hand fires it stays blocked; hFRIK poses the
            // left hand itself from the weapon-node ownership block state
            // (mirrored copy of the animated right weapon hand).
            restoreFrikPrimaryWeaponPose();
        }
        _partGrips = {};
        _partCarryPivotIsLeft = true;
        _partCarryGripSeparationWorld = 0.0f;
        _hasSolvedWeaponTransform = _returningWeaponVisual.localTransition.active && _hasLastRenderedWeaponWorld;
        if (_hasSolvedWeaponTransform) {
            _lastSolvedWeaponTransform = _lastRenderedWeaponWorld;
        }
        if (!_firingHandIsLeft) {
            /*
             * Right firing hand: PrimaryOnly is FRIK-native carry, so ROCK
             * deliberately holds no hand-to-weapon frame. A LEFT firing hand
             * has no native carry - its captured frame IS the carry solve and
             * must survive this transition (wiping it here was the "weapon
             * snaps back to the right hand" takeover regression).
             */
            _primaryHandWeaponLocal = {};
            _hasFiringHandWeaponLocal = false;
        }
        _primaryHandVisualLerp = {};
        _state = TwoHandedState::PrimaryOnly;

        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: primary-only equipped weapon ownership active reason={} generation={:016X} ownership={:016X} provisional={}",
            reason ? reason : "unknown",
            _activeWeaponGenerationKey,
            _activeEquippedWeaponOwnershipKey,
            _activeWeaponGenerationKey == 0 ? "yes" : "no");
        return true;
    }

    void TwoHandedGrip::requestEquippedWeaponDrop(const char* reason, equipped_weapon_drop_policy::SourceHand sourceHand)
    {
        if (_equippedWeaponDropRequest.requested) {
            transitionToInactive(false);
            return;
        }

        _equippedWeaponDropRequest = EquippedWeaponManualDropRequest{
            .requested = true,
            .sourceHand = sourceHand,
        };
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: equipped weapon drop requested reason={} sourceHand={} generation={:016X}",
            reason ? reason : "unknown",
            equipped_weapon_drop_policy::sourceHandName(sourceHand),
            _activeWeaponGenerationKey);
        clearWeaponVisualReturn("equipped-weapon-drop", true, true);
        transitionToInactive(false);
    }

    void TwoHandedGrip::updatePrimaryOnlyGrip(
        RE::NiNode* weaponNode,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const EquippedWeaponPrimaryGripInput& primaryGripInput,
        const bool primaryDetachEnabled)
    {
        equipped_weapon_manual_ownership_policy::RuntimeState manualState{
            .active = true,
            .ownershipKey = _activeEquippedWeaponOwnershipKey,
        };
        const auto manualDecision = equipped_weapon_manual_ownership_policy::update(manualState,
            equipped_weapon_manual_ownership_policy::Input{
                .weaponEquipped = weaponNode != nullptr,
                .ownershipKey = currentEquippedWeaponOwnershipKey,
                .startRequested = false,
                .primaryGripRetained = equipped_weapon_manual_ownership_policy::shouldRetainPrimaryOnlyOwnership(
                    primaryDetachEnabled,
                    primaryGripInput.held),
                .supportGripRetained = false,
            });

        if (manualDecision.dropRequested) {
            beginHandVisualReturn(_firingHandIsLeft, "primary-only-drop");
            requestEquippedWeaponDrop("primary-only-grip-released",
                _firingHandIsLeft ? equipped_weapon_drop_policy::SourceHand::Left : equipped_weapon_drop_policy::SourceHand::Right);
            return;
        }

        if (manualDecision.cleared) {
            beginHandVisualReturn(_firingHandIsLeft, "primary-only-released");
            if (_firingHandIsLeft) {
                beginWeaponVisualReturn("left-primary-only-released");
            }
            transitionToInactive(false);
            return;
        }

        if (!_firingHandIsLeft) {
            // Right firing hand: FRIK-native carry, ROCK bookkeeping only.
            _hasSolvedWeaponTransform = false;
            return;
        }

        // Left firing hand: FRIK cannot carry (its weapon glue targets the
        // right hand and is blocked); ROCK drives the weapon rigidly from the
        // left hand through the captured weapon-relative firing-grip frame.
        // The left hand's finger pose is hFRIK's mirrored weapon-hand copy,
        // driven by the same ownership block.
        (void)solveLeftFiringWeaponCarry(weaponNode);
    }

    bool TwoHandedGrip::solveLeftFiringWeaponCarry(RE::NiNode* weaponNode)
    {
        if (!weaponNode || !_hasFiringHandWeaponLocal) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing left-firing carry because the captured firing-grip frame is unavailable");
            transitionToInactive(false);
            return false;
        }

        RE::NiTransform firingHandTransform{};
        if (!tryGetSolverHandTransform(true, firingHandTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing left-firing carry because authoritative hand transforms are unavailable");
            transitionToInactive(false);
            return false;
        }
        rememberFiringRecoilReference(true, firingHandTransform);

        const RE::NiTransform solvedWeaponWorld =
            transform_math::composeTransforms(firingHandTransform, transform_math::invertTransform(_primaryHandWeaponLocal));
        if (!isFiniteTransform(solvedWeaponWorld)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing left-firing carry because the weapon solve produced an invalid transform");
            transitionToInactive(false);
            return false;
        }

        if (!applyWeaponVisualAuthority(weaponNode, solvedWeaponWorld)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing left-firing carry because ROCK visual weapon authority failed");
            transitionToInactive(false);
            return false;
        }

        (void)publishAuthoredPrimaryFiringGripFingerPose(true);
        _lastSolvedWeaponTransform = weaponNode->world;
        _hasSolvedWeaponTransform = true;

        /*
         * Carry-time aim diagnostic (~3s cadence): the barrel direction in
         * LEFT-wand coordinates during the live carry. Matching the
         * takeover's barrelInLeftWand proves the carry chain is faithful to
         * the committed hold (residual cant then lives in the left wand /
         * melee driver chain and the aim trim is the right knob); a drift
         * from the takeover value means the left bone-in-wand relationship
         * changed after the topology swap and the hold must be resampled.
         */
        if (++_leftFiringAimLogCounter >= 270) {
            _leftFiringAimLogCounter = 0;
            auto* playerNodes = f4vr::getPlayerNodes();
            if (playerNodes && playerNodes->SecondaryWandNode && isFiniteTransform(playerNodes->SecondaryWandNode->world)) {
                const RE::NiTransform weaponInLeftWandNow = transform_math::composeTransforms(
                    transform_math::invertTransform(playerNodes->SecondaryWandNode->world), weaponNode->world);
                const RE::NiPoint3 barrelNow = sub(
                    transform_math::localPointToWorld(weaponInLeftWandNow, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }),
                    weaponInLeftWandNow.translate);
                ROCK_LOG_INFO(Weapon,
                    "TwoHandedGrip: left-firing carry aim barrelInLeftWand=({:.3f},{:.3f},{:.3f})",
                    barrelNow.x,
                    barrelNow.y,
                    barrelNow.z);
            }
        }
        return true;
    }

    bool TwoHandedGrip::firingGripContactMatchesCapturedGrip(
        RE::NiNode* weaponNode,
        const WeaponInteractionContact& handWeaponContact,
        const RE::NiTransform& handTransform,
        const bool handIsLeft) const
    {
        if (!weaponNode || !handWeaponContact.valid) {
            return false;
        }

        if (!weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(handWeaponContact.weaponGenerationKey, _activeWeaponGenerationKey)) {
            return false;
        }

        float distance = 0.0f;
        return tryComputePalmToFiringGripDistance(
                   weaponNode,
                   handTransform,
                   handIsLeft,
                   distance) &&
               distance <=
                   _handlingSettings.firingGripReattachRadiusGameUnits;
    }

    bool TwoHandedGrip::tryComputePalmToFiringGripDistance(
        RE::NiNode* weaponNode,
        const RE::NiTransform& handTransform,
        const bool handIsLeft,
        float& outDistance) const
    {
        if (!weaponNode) {
            return false;
        }
        const RE::NiPoint3 palm =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(
                handTransform,
                handIsLeft);
        const RE::NiPoint3 firingGripWorld =
            weaponLocalToWorld(_primaryGripLocal, weaponNode);
        const RE::NiPoint3 delta = sub(palm, firingGripWorld);
        const float distance = std::sqrt(dot(delta, delta));
        if (!std::isfinite(distance)) {
            return false;
        }
        outDistance = distance;
        return true;
    }

    bool TwoHandedGrip::tryComputePalmToGripDistanceForHand(
        RE::NiNode* weaponNode,
        const bool handIsLeft,
        float& outDistance) const
    {
        RE::NiTransform handTransform{};
        return tryGetSolverHandTransform(handIsLeft, handTransform) &&
               tryComputePalmToFiringGripDistance(
                   weaponNode,
                   handTransform,
                   handIsLeft,
                   outDistance);
    }

    TwoHandedGrip::CanonicalFiringHoldSelection
        TwoHandedGrip::selectCanonicalFiringHold(
            const bool handIsLeft,
            RE::NiNode* weaponNode,
            const RE::NiTransform& liveHandWorld,
            const RE::NiTransform* preferredAuthoredCanonical,
            const char* preferredAuthoredSource) const
    {
        CanonicalFiringHoldSelection selection{};
        if (preferredAuthoredCanonical) {
            selection.handWeaponLocal = *preferredAuthoredCanonical;
            selection.source = preferredAuthoredSource ?
                preferredAuthoredSource :
                "authored-canonical";
            return selection;
        }

        if (handIsLeft) {
            bool usedAuthoredCanonical = false;
            if (tryComputeMirroredLeftFiringHandWeaponLocal(
                    selection.handWeaponLocal,
                    &usedAuthoredCanonical)) {
                selection.source = usedAuthoredCanonical ?
                    "authored-mirror" :
                    "native-mirror";
                return selection;
            }
        } else if (hasRightFiringHandCanonicalFrame(
                       _activeWeaponNode,
                       _activeWeaponGenerationKey,
                       _activeEquippedWeaponOwnershipKey)) {
            selection.handWeaponLocal =
                _rightFiringHandCanonicalWeaponLocal;
            selection.source = _rightFiringHandCanonicalSource ==
                    RightFiringCanonicalSource::AuthoredAnimation ?
                "authored-canonical" :
                "native-canonical";
            return selection;
        }

        if (!weaponNode) {
            return selection;
        }
        const RE::NiPoint3 palm =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(
                liveHandWorld,
                handIsLeft);
        const RE::NiPoint3 firingGripWorld =
            weaponLocalToWorld(_primaryGripLocal, weaponNode);
        const RE::NiTransform adjustedHandWorld =
            weapon_two_handed_grip_math::alignHandFrameToGripPoint(
                liveHandWorld,
                palm,
                firingGripWorld);
        selection.handWeaponLocal = transform_math::composeTransforms(
            transform_math::invertTransform(weaponNode->world),
            adjustedHandWorld);
        return selection;
    }

    bool TwoHandedGrip::tryResolveAuthoredFiringHandCanonicalForProbe(
        const bool handIsLeft,
        RE::NiTransform& outHandWeaponLocal,
        const char*& outSource) const
    {
        outHandWeaponLocal = {};
        outSource = "unavailable";
        if (!hasRightFiringHandCanonicalFrame(
                _activeWeaponNode,
                _activeWeaponGenerationKey,
                _activeEquippedWeaponOwnershipKey) ||
            _rightFiringHandCanonicalSource !=
                RightFiringCanonicalSource::AuthoredAnimation) {
            return false;
        }

        if (handIsLeft) {
            if (_leftFiringFingerLocalTransformMask !=
                authored_weapon_grip_library::
                    kCompleteFiringFingerMask) {
                return false;
            }
            bool usedAuthoredCanonical = false;
            if (!tryComputeMirroredLeftFiringHandWeaponLocal(
                    outHandWeaponLocal,
                    &usedAuthoredCanonical) ||
                !usedAuthoredCanonical) {
                return false;
            }
            outSource = "authored-mirror-probe";
            return true;
        }

        if (_rightFiringFingerLocalTransformMask !=
            authored_weapon_grip_library::kCompleteFiringFingerMask) {
            return false;
        }
        outHandWeaponLocal = _rightFiringHandCanonicalWeaponLocal;
        outSource = "authored-canonical-probe";
        return isFiniteTransform(outHandWeaponLocal);
    }

    bool TwoHandedGrip::tryReattachFiringGrip(
        const bool handIsLeft,
        RE::NiNode* weaponNode,
        const WeaponInteractionContact& handWeaponContact,
        const bool authoredProviderAuthorityActive,
        const bool authoredAttachOnlyAuthorityActive)
    {
        if (!weaponNode ||
            !handWeaponContact.valid ||
            !weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(
                handWeaponContact.weaponGenerationKey,
                _activeWeaponGenerationKey)) {
            return false;
        }

        RE::NiTransform handTransform{};
        if (!tryGetSolverHandTransform(handIsLeft, handTransform)) {
            return false;
        }

        RE::NiTransform authoredProbeCanonical{};
        const char* authoredProbeCanonicalSource = "unavailable";
        const bool authoredProbeCanonicalAvailable =
            tryResolveAuthoredFiringHandCanonicalForProbe(
                handIsLeft,
                authoredProbeCanonical,
                authoredProbeCanonicalSource);
        const bool useAuthoredProbeCanonical =
            authored_weapon_grip_capture_policy::shouldUseAuthoredFiringGripProbe(
                authored_weapon_grip_capture_policy::AuthoredFiringGripProbeInput{
                    .proximityProbeAcquisition =
                        handWeaponContact.acquisitionSource ==
                        WeaponInteractionAcquisitionSource::ProximityProbe,
                    .providerAuthorityActive = authoredProviderAuthorityActive,
                    .attachOnly = authoredAttachOnlyAuthorityActive,
                    .authoredCanonicalAvailable =
                        authoredProbeCanonicalAvailable,
                });
        if (!useAuthoredProbeCanonical &&
            !firingGripContactMatchesCapturedGrip(
                weaponNode,
                handWeaponContact,
                handTransform,
                handIsLeft)) {
            return false;
        }

        // Validated: commit. A takeover by the other hand flips the firing
        // role here and reuses the SAME captured weapon-relative grip point.
        if (handIsLeft != _firingHandIsLeft) {
            setFiringHand(handIsLeft, "firing-grip-reattach-other-hand");
        }

        /*
         * Reattach forces the CANONICAL per-hand hold instead of freezing the
         * live squeeze orientation: the LEFT hand takes the canonical
         * right-hand hold MIRRORED (authored offsets adapted to the left bone
         * basis), the RIGHT hand re-takes its canonical native hold directly.
         * A live squeeze capture both fired with the weapon crooked and, for
         * the right hand, poisoned the canonical itself through the snapshot
         * below. The live capture remains only as the no-canonical fallback.
         */
        const auto hold = selectCanonicalFiringHold(
            handIsLeft,
            weaponNode,
            handTransform,
            useAuthoredProbeCanonical ? &authoredProbeCanonical : nullptr,
            authoredProbeCanonicalSource);
        _primaryHandWeaponLocal = hold.handWeaponLocal;
        _hasFiringHandWeaponLocal = true;
        rememberRightFiringHandCanonicalFrame();
        _firingGripSequence = ++_gripCaptureSequence;
        _primaryHandVisualLerp = {};
        clearPrimaryDetachVisualAuthority(handIsLeft);
        if (!_firingHandIsLeft) {
            restoreFrikPrimaryWeaponPose();
        }
        _hapticEvents.firingGripAttached = true;
        _hapticEvents.firingGripAttachedHandIsLeft = handIsLeft;
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: firing hand reattached at configured grip hand={} hold={}",
            handIsLeft ? "left" : "right",
            hold.source);
        return true;
    }

    void TwoHandedGrip::clearAuthoredSupportGripCandidate()
    {
        _authoredSupportGripCandidate = {};
    }

    bool TwoHandedGrip::setAuthoredSupportGripCandidate(
        RE::NiNode* weaponNode,
        const RE::NiTransform& handWeaponLocal,
        const std::array<RE::NiTransform, 15>& fingerLocalTransforms,
        const std::uint16_t fingerLocalTransformMask,
        const std::uint64_t weaponGenerationKey,
        const std::uint64_t captureSequence)
    {
        clearAuthoredSupportGripCandidate();
        if (!weaponNode ||
            weaponGenerationKey == 0 ||
            captureSequence == 0 ||
            fingerLocalTransformMask !=
                authored_weapon_grip_library::kCompleteFiringFingerMask ||
            !isFiniteTransform(handWeaponLocal) ||
            std::abs(handWeaponLocal.scale) <= 0.0001f) {
            return false;
        }
        for (const auto& fingerLocal : fingerLocalTransforms) {
            if (!isFiniteTransform(fingerLocal) ||
                std::abs(fingerLocal.scale) <= 0.0001f) {
                return false;
            }
        }

        AuthoredSupportGripCandidate candidate{
            .weaponNode = weaponNode,
            .leftHandWeaponLocal = handWeaponLocal,
            .leftFingerLocalTransforms = fingerLocalTransforms,
            .leftFingerLocalTransformMask = fingerLocalTransformMask,
            .weaponGenerationKey = weaponGenerationKey,
            .captureSequence = captureSequence,
            .valid = true,
        };

        _authoredSupportGripCandidate = candidate;
        refreshAuthoredSupportRightMirror();
        return true;
    }

    void TwoHandedGrip::refreshAuthoredSupportRightMirror()
    {
        auto& candidate = _authoredSupportGripCandidate;
        if (!candidate.valid || candidate.rightMirrorValid) {
            return;
        }

        RE::NiTransform mirroredRightHandWeaponLocal{};
        frik_visual_authority::FingerLocalTransformOverride leftFingerLocals{};
        leftFingerLocals.enabledMask = candidate.leftFingerLocalTransformMask;
        for (std::size_t index = 0;
             index < candidate.leftFingerLocalTransforms.size();
             ++index) {
            leftFingerLocals.localTransforms[index] =
                candidate.leftFingerLocalTransforms[index];
        }

        frik_visual_authority::FingerLocalTransformOverride mirroredRightFingerLocals{};
        const bool rightHandTransformMirrored =
            tryBuildMirroredRightSupportHandWeaponLocal(
                candidate.leftHandWeaponLocal,
                mirroredRightHandWeaponLocal);
        const bool rightFingerPoseMirrored =
            frik_visual_authority::mirrorFingerLocalTransforms(
                frik_visual_authority::Hand::Left,
                leftFingerLocals,
                mirroredRightFingerLocals) &&
            mirroredRightFingerLocals.enabledMask ==
                authored_weapon_grip_library::kCompleteFiringFingerMask;
        bool rightFingerPoseFinite = rightFingerPoseMirrored;
        if (rightFingerPoseFinite) {
            for (const auto& fingerLocal : mirroredRightFingerLocals.localTransforms) {
                if (!isFiniteTransform(fingerLocal) ||
                    std::abs(fingerLocal.scale) <= 0.0001f) {
                    rightFingerPoseFinite = false;
                    break;
                }
            }
        }

        if (rightHandTransformMirrored && rightFingerPoseFinite) {
            candidate.rightHandWeaponLocal = mirroredRightHandWeaponLocal;
            for (std::size_t index = 0;
                 index < candidate.rightFingerLocalTransforms.size();
                 ++index) {
                candidate.rightFingerLocalTransforms[index] =
                    mirroredRightFingerLocals.localTransforms[index];
            }
            candidate.rightFingerLocalTransformMask =
                mirroredRightFingerLocals.enabledMask;
            candidate.rightMirrorValid = true;
            return;
        }

        if (_firingHandIsLeft) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 2000,
                "TwoHandedGrip: authored right-support mirror unavailable transform={} fingers={} naturalFrames=({}, {})",
                rightHandTransformMirrored ? "ready" : "missing",
                rightFingerPoseFinite ? "ready" : "missing",
                _hasLeftNaturalBoneInWand ? "left" : "no-left",
                _hasRightNaturalBoneInWand ? "right" : "no-right");
        }
    }

    bool TwoHandedGrip::tryResolveAuthoredSupportGripCandidateForHand(
        const bool isLeft,
        RE::NiNode* weaponNode,
        const std::uint64_t weaponGenerationKey,
        RE::NiTransform& outHandWeaponLocal,
        std::array<RE::NiTransform, 15>& outFingerLocalTransforms,
        std::uint16_t& outFingerLocalTransformMask) const
    {
        outHandWeaponLocal = {};
        outFingerLocalTransforms = {};
        outFingerLocalTransformMask = 0;

        const auto& candidate = _authoredSupportGripCandidate;
        if (!candidate.valid ||
            !weaponNode ||
            candidate.weaponNode != weaponNode ||
            weaponGenerationKey == 0 ||
            candidate.weaponGenerationKey != weaponGenerationKey) {
            return false;
        }

        if (isLeft) {
            if (candidate.leftFingerLocalTransformMask !=
                authored_weapon_grip_library::
                    kCompleteFiringFingerMask) {
                return false;
            }
            outHandWeaponLocal = candidate.leftHandWeaponLocal;
            outFingerLocalTransforms = candidate.leftFingerLocalTransforms;
            outFingerLocalTransformMask = candidate.leftFingerLocalTransformMask;
        } else {
            if (!candidate.rightMirrorValid ||
                candidate.rightFingerLocalTransformMask !=
                    authored_weapon_grip_library::
                        kCompleteFiringFingerMask) {
                return false;
            }
            outHandWeaponLocal = candidate.rightHandWeaponLocal;
            outFingerLocalTransforms = candidate.rightFingerLocalTransforms;
            outFingerLocalTransformMask = candidate.rightFingerLocalTransformMask;
        }

        return isFiniteTransform(outHandWeaponLocal);
    }

    bool TwoHandedGrip::setAuthoredPrimaryFiringGripCanonical(
        RE::NiNode* weaponNode,
        const RE::NiTransform& rightHandWeaponLocal,
        const std::uint64_t weaponGenerationKey,
        const std::uint64_t weaponOwnershipKey,
        const std::uint64_t captureSequence, const authored_weapon_grip_library::FiringFingerPose* rightFingerPose,
        const authored_weapon_grip_library::FiringFingerPose* leftFingerPose)
    {
        const auto validFingerPose = [](const authored_weapon_grip_library::FiringFingerPose* pose) {
            if (!pose) {
                return true;
            }
            if (!pose->complete()) {
                return false;
            }
            return std::ranges::all_of(pose->localTransforms, [](const RE::NiTransform& transform) { return isFiniteTransform(transform) && std::abs(transform.scale) > 0.0001f; });
        };
        if (!weaponNode ||
            weaponGenerationKey == 0 ||
            weaponOwnershipKey == 0 ||
            captureSequence == 0 ||
            !isFiniteTransform(rightHandWeaponLocal) ||
            std::abs(rightHandWeaponLocal.scale) <= 0.0001f || !validFingerPose(rightFingerPose) || !validFingerPose(leftFingerPose) || (leftFingerPose && !rightFingerPose)) {
            return false;
        }

        // HandFrame helpers are frame-agnostic: feeding Hand-in-Weapon yields
        // the configured right palm seat directly in Weapon coordinates. The
        // mirror needs this authored seat, not _primaryGripLocal (which can be
        // a live squeeze or an older native-offset capture).
        const RE::NiPoint3 authoredGripWeaponLocal =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(
                rightHandWeaponLocal,
                false);
        if (!std::isfinite(authoredGripWeaponLocal.x) ||
            !std::isfinite(authoredGripWeaponLocal.y) ||
            !std::isfinite(authoredGripWeaponLocal.z)) {
            return false;
        }

        const std::uint16_t incomingRightFingerMask = rightFingerPose ? rightFingerPose->enabledMask : 0;
        const std::uint16_t incomingLeftFingerMask = leftFingerPose ? leftFingerPose->enabledMask : 0;
        const bool fingerPoseBoundary =
            _rightFiringFingerLocalTransformMask != incomingRightFingerMask ||
            _leftFiringFingerLocalTransformMask != incomingLeftFingerMask;
        const bool sourceBoundary =
            _rightFiringHandCanonicalSource !=
                RightFiringCanonicalSource::AuthoredAnimation ||
            _rightFiringHandCanonicalWeaponNode != weaponNode ||
            _rightFiringHandCanonicalGenerationKey != weaponGenerationKey ||
            _rightFiringHandCanonicalOwnershipKey != weaponOwnershipKey ||
            fingerPoseBoundary;

        _rightFiringHandCanonicalWeaponLocal = rightHandWeaponLocal;
        _rightFiringGripCanonicalWeaponLocal = authoredGripWeaponLocal;
        _rightFiringHandCanonicalWeaponNode = weaponNode;
        _rightFiringHandCanonicalGenerationKey = weaponGenerationKey;
        _rightFiringHandCanonicalOwnershipKey = weaponOwnershipKey;
        _rightFiringHandCanonicalCaptureSequence = captureSequence;
        _rightFiringHandCanonicalSource =
            RightFiringCanonicalSource::AuthoredAnimation;
        _hasRightFiringHandCanonicalWeaponLocal = true;
        _rightFiringFingerLocalTransforms = rightFingerPose ? rightFingerPose->localTransforms : std::array<RE::NiTransform, 15>{};
        _rightFiringFingerLocalTransformMask = rightFingerPose ? rightFingerPose->enabledMask : 0;
        _leftFiringFingerLocalTransforms = leftFingerPose ? leftFingerPose->localTransforms : std::array<RE::NiTransform, 15>{};
        _leftFiringFingerLocalTransformMask = leftFingerPose ? leftFingerPose->enabledMask : 0;

        if (sourceBoundary) {
            ROCK_LOG_INFO(Animation,
                "TwoHandedGrip: authored firing canonical active generation={:016X} ownership={:016X} capture={} handWeaponT=({:.3f},{:.3f},{:.3f}) "
                "gripWeapon=({:.3f},{:.3f},{:.3f}) rightFingerMask=0x{:04X} leftFingerMask=0x{:04X} leftSource=wand-and-anatomy-mirror",
                weaponGenerationKey,
                weaponOwnershipKey,
                captureSequence,
                rightHandWeaponLocal.translate.x,
                rightHandWeaponLocal.translate.y,
                rightHandWeaponLocal.translate.z,
                authoredGripWeaponLocal.x,
                authoredGripWeaponLocal.y,
                authoredGripWeaponLocal.z, _rightFiringFingerLocalTransformMask, _leftFiringFingerLocalTransformMask);
        }
        return true;
    }

    bool TwoHandedGrip::publishAuthoredPrimaryFiringGripFingerPose(const bool isLeft)
    {
        const bool targetHandHoldingObject =
            isLeft ? _leftHandHoldingObjectForPose : _rightHandHoldingObjectForPose;
        if (_authoredPrimaryFingerPoseSuppressed ||
            !authored_weapon_grip_capture_policy::shouldPublishAuthoredFiringFingerPose(
                targetHandHoldingObject) ||
            _rightFiringHandCanonicalSource != RightFiringCanonicalSource::AuthoredAnimation) {
            return false;
        }

        const auto& transforms = isLeft ? _leftFiringFingerLocalTransforms : _rightFiringFingerLocalTransforms;
        const std::uint16_t mask = isLeft ? _leftFiringFingerLocalTransformMask : _rightFiringFingerLocalTransformMask;
        if (mask != authored_weapon_grip_library::kCompleteFiringFingerMask) {
            return false;
        }

        if (_authoredPrimaryFingerPosePublished && _publishedFiringFingerPoseIsLeft != isLeft) {
            clearAuthoredPrimaryFiringGripFingerPose();
        }

        if (!_authoredPrimaryFingerPoseBlockEngaged) {
            if (!frik_visual_authority::blockPrimaryHandWeaponPose(AUTHORED_PRIMARY_POSE_BLOCK_TAG, true)) {
                return false;
            }
            _authoredPrimaryFingerPoseBlockEngaged = true;
        }

        const auto hand = handFromBool(isLeft);
        _publishedFiringFingerPoseIsLeft = isLeft;
        if (!frik_visual_authority::setHandPoseCustomWithPriority(PRIMARY_GRIP_TAG, hand, frik_visual_authority::HandPoseData{}, GRIP_HAND_POSE_PRIORITY)) {
            clearAuthoredPrimaryFiringGripFingerPose();
            return false;
        }

        frik_visual_authority::FingerLocalTransformOverride overrideData{};
        overrideData.enabledMask = mask;
        for (std::size_t index = 0; index < transforms.size(); ++index) {
            overrideData.localTransforms[index] = transforms[index];
        }
        if (!frik_visual_authority::setHandPoseCustomLocalTransformsWithPriority(PRIMARY_GRIP_TAG, hand, &overrideData, GRIP_HAND_POSE_PRIORITY)) {
            clearAuthoredPrimaryFiringGripFingerPose();
            return false;
        }

        _publishedFiringFingerPoseIsLeft = isLeft;
        _authoredPrimaryFingerPosePublished = true;
        return true;
    }

    void TwoHandedGrip::clearAuthoredPrimaryFiringGripFingerPose()
    {
        if (_authoredPrimaryFingerPosePublished || _authoredPrimaryFingerPoseBlockEngaged) {
            (void)frik_visual_authority::clearHandPose(PRIMARY_GRIP_TAG, handFromBool(_publishedFiringFingerPoseIsLeft));
        }
        if (_authoredPrimaryFingerPoseBlockEngaged) {
            (void)frik_visual_authority::blockPrimaryHandWeaponPose(AUTHORED_PRIMARY_POSE_BLOCK_TAG, false);
        }
        _publishedFiringFingerPoseIsLeft = false;
        _authoredPrimaryFingerPosePublished = false;
        _authoredPrimaryFingerPoseBlockEngaged = false;
    }

    void TwoHandedGrip::setAuthoredPrimaryFiringGripFingerPoseSuppressed(const bool suppressed)
    {
        _authoredPrimaryFingerPoseSuppressed = suppressed;
        if (suppressed) {
            clearAuthoredPrimaryFiringGripFingerPose();
        }
    }

    void TwoHandedGrip::setGrabbedObjectHandPoseOwnership(
        const bool leftHandHoldingObject,
        const bool rightHandHoldingObject)
    {
        _leftHandHoldingObjectForPose = leftHandHoldingObject;
        _rightHandHoldingObjectForPose = rightHandHoldingObject;

        if (!_authoredPrimaryFingerPosePublished) {
            return;
        }

        const bool publishedHandHoldingObject =
            _publishedFiringFingerPoseIsLeft ?
                _leftHandHoldingObjectForPose :
                _rightHandHoldingObjectForPose;
        if (publishedHandHoldingObject) {
            clearAuthoredPrimaryFiringGripFingerPose();
        }
    }

    void TwoHandedGrip::clearAuthoredPrimaryFiringGripCanonical(
        const char* reason)
    {
        if (_rightFiringHandCanonicalSource !=
            RightFiringCanonicalSource::AuthoredAnimation) {
            return;
        }

        ROCK_LOG_DEBUG(Animation,
            "TwoHandedGrip: clearing authored firing canonical reason={} generation={:016X} ownership={:016X} capture={}",
            reason ? reason : "unknown",
            _rightFiringHandCanonicalGenerationKey,
            _rightFiringHandCanonicalOwnershipKey,
            _rightFiringHandCanonicalCaptureSequence);
        clearAuthoredPrimaryFiringGripFingerPose();
        clearRightFiringHandCanonicalFrame();
    }

    bool TwoHandedGrip::applyAuthoredPrimaryGripWeaponAlignment(
        RE::NiNode* weaponNode,
        const RE::NiTransform& solvedWeaponWorld,
        const std::uint64_t currentWeaponGenerationKey)
    {
        if (blocksAuthoredPrimaryGripWeaponAlignment() || isWeaponVisualReturnActive()) {
            return false;
        }
        return applyWeaponVisualAuthority(
            weaponNode,
            solvedWeaponWorld,
            currentWeaponGenerationKey);
    }

    void TwoHandedGrip::reframeAuthoredSupportGripDebugSnapshot(
        const RE::NiTransform& finalWeaponWorld)
    {
        auto& snapshot = _authoredSupportGripDebugSnapshot;
        if (!snapshot.valid ||
            !isFiniteTransform(snapshot.weaponWorld) ||
            !isFiniteTransform(finalWeaponWorld)) {
            return;
        }

        const RE::NiTransform previousWeaponWorld = snapshot.weaponWorld;
        const auto reframePoint = [&](const RE::NiPoint3& pointWorld) {
            return transform_math::localPointToWorld(
                finalWeaponWorld,
                transform_math::worldPointToLocal(
                    previousWeaponWorld,
                    pointWorld));
        };
        const auto reframeDirection = [&](const RE::NiPoint3& directionWorld) {
            RE::NiPoint3 reframed{};
            (void)gunstock_alignment_policy::tryNormalizeDirection(
                transform_math::localVectorToWorld(
                    finalWeaponWorld,
                    transform_math::worldVectorToLocal(
                        previousWeaponWorld,
                        directionWorld)),
                reframed);
            return reframed;
        };

        snapshot.authoredPalmSeatWorld =
            transform_math::localPointToWorld(
                finalWeaponWorld,
                snapshot.authoredPalmSeatWeaponLocal);
        snapshot.leftAxisWorld = reframeDirection(snapshot.leftAxisWorld);
        snapshot.downAxisWorld = reframeDirection(snapshot.downAxisWorld);
        snapshot.referenceAxisWorld =
            reframeDirection(snapshot.referenceAxisWorld);
        for (auto& landmark : snapshot.poseLandmarksWorld) {
            landmark = reframePoint(landmark);
        }
        for (std::size_t index = 0;
             index < snapshot.poseSurfaceWitnessWorld.size();
             ++index) {
            if ((snapshot.poseSurfaceWitnessMask &
                    static_cast<std::uint8_t>(1u << index)) != 0) {
                snapshot.poseSurfaceWitnessWorld[index] =
                    reframePoint(snapshot.poseSurfaceWitnessWorld[index]);
            }
        }

        AuthoredSupportPalmSeatProximity proximity{};
        if (!resolveAuthoredSupportPalmSeatProximityFromPoints(
                finalWeaponWorld,
                snapshot.liveTouchProbeWorld,
                snapshot.authoredPalmSeatWeaponLocal,
                proximity)) {
            return;
        }
        snapshot.authoredPalmSeatWorld = proximity.authoredPalmSeatWorld;
        snapshot.liveTouchProbeWeaponLocal =
            proximity.liveTouchProbeWeaponLocal;
        const RE::NiPoint3 approach{
            snapshot.liveTouchProbeWorld.x -
                snapshot.authoredPalmSeatWorld.x,
            snapshot.liveTouchProbeWorld.y -
                snapshot.authoredPalmSeatWorld.y,
            snapshot.liveTouchProbeWorld.z -
                snapshot.authoredPalmSeatWorld.z,
        };
        snapshot.approachDirectionWorld = {};
        const bool approachValid =
            gunstock_alignment_policy::tryNormalizeDirection(
                approach,
                snapshot.approachDirectionWorld);
        snapshot.weaponRelativeDistanceGameUnits =
            proximity.weaponRelativeDistanceGameUnits;
        snapshot.worldReadbackDistanceGameUnits =
            proximity.worldReadbackDistanceGameUnits;
        snapshot.frameAgreementErrorGameUnits =
            proximity.frameAgreementErrorGameUnits;
        snapshot.insideTouchRadius =
            snapshot.weaponRelativeDistanceGameUnits <=
            snapshot.touchRadiusGameUnits;

        (void)evaluateAuthoredSupportGripDirectionGate(
            snapshot,
            snapshot.approachDirectionWorld,
            approachValid,
            snapshot.canonicalAxesValid &&
                !_firingHandIsLeft && snapshot.supportHandIsLeft);
        snapshot.weaponWorld = finalWeaponWorld;
    }

    bool TwoHandedGrip::applyWeaponVisualAuthority(
        RE::NiNode* weaponNode,
        const RE::NiTransform& solvedWeaponWorld,
        const std::uint64_t authorityGenerationKey,
        const bool notifyVisualIntentObserver)
    {
        if (!weaponNode || !isFiniteTransform(weaponNode->world) ||
            !isFiniteTransform(solvedWeaponWorld)) {
            return false;
        }

        const RE::NiTransform scaleStableSolvedWeaponWorld =
            weapon_visual_authority_math::preserveLiveWeaponWorldScale(
                weaponNode->world,
                solvedWeaponWorld);

        const std::uint64_t effectiveGenerationKey = authorityGenerationKey != 0 ? authorityGenerationKey : _activeWeaponGenerationKey;
        if (notifyVisualIntentObserver && _weaponVisualIntentObserver) {
            _weaponVisualIntentObserver(
                _weaponVisualIntentObserverContext,
                weaponNode,
                scaleStableSolvedWeaponWorld,
                effectiveGenerationKey);
        }
        const bool scopeAnchorMatchesAuthority =
            _nativeScopeAnchorValid && _nativeScopeAnchorWeaponNode == weaponNode && _nativeScopeAnchorGenerationKey == effectiveGenerationKey;

        // Capture hFRIK's engine-specific camera axis only before changing the
        // weapon. Once captured, every hand mode resolves the same immutable
        // generation-bound weapon-local scope frame.
        const NativeScopeCameraFollowCapture scopeCameraFollow = captureNativeScopeCameraFollow(weaponNode);
        if (scopeAnchorMatchesAuthority && scopeCameraFollow.valid) {
            (void)captureNativeScopeRigidFrame(weaponNode, effectiveGenerationKey, scopeCameraFollow.camera, scopeCameraFollow.cameraWorldBefore);
            (void)captureNativeScopeOverlayCalibration(scopeCameraFollow.cameraWorldBefore, effectiveGenerationKey);
        }

        if (!moveWeaponPresentationRigidly(
                weaponNode,
                scaleStableSolvedWeaponWorld)) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                2000,
                "TwoHandedGrip: rejected weapon visual authority because the root or bounded presentation subtree was invalid");
            return false;
        }

        const bool rigidFrameMatchesAuthority = _nativeScopeRigidFrame.valid && _nativeScopeRigidFrame.weaponGenerationKey == effectiveGenerationKey &&
            _nativeScopeRigidFrame.weaponNodeIdentity == weaponNode && _nativeScopeRigidFrame.scopeCameraIdentity == scopeCameraFollow.camera;
        const bool scopeTargetReady =
            rigidFrameMatchesAuthority &&
            isFiniteTransform(
                _nativeScopeRigidFrame.cameraWeaponLocal) &&
            std::abs(_nativeScopeRigidFrame.cameraWeaponLocal.scale) >
                0.0001f;
        const NativeScopeCameraFollowResult scopeCameraResult =
            scopeTargetReady ?
                applyNativeScopeCameraWorldTarget(
                    scopeCameraFollow,
                    native_scope_camera_follow_math::
                        resolveRigidAnchorFrameWorld(
                            weaponNode->world,
                            _nativeScopeRigidFrame.cameraWeaponLocal)) :
                NativeScopeCameraFollowResult{};
        if (scopeCameraResult.targetValid && scopeCameraResult.writeApplied) {
            (void)applyNativeScopeOverlayTarget(scopeCameraResult.targetCameraWorld, effectiveGenerationKey);
        }
        if (g_rockConfig.rockDebugDrawNativeScopeActivation &&
            scopeTargetReady &&
            scopeCameraResult.immediateReadbackValid) {
            const RE::NiTransform immediateCameraWeaponLocal =
                transform_math::composeTransforms(
                    transform_math::invertTransform(weaponNode->world),
                    scopeCameraResult.immediateCameraWorldAfter);
            const float rigidPositionError =
                hand_visual_lerp_math::distanceGameUnits(
                    immediateCameraWeaponLocal.translate,
                    _nativeScopeRigidFrame.cameraWeaponLocal.translate);
            const float rigidRotationError =
                hand_visual_lerp_math::rotationDistanceDegrees(
                    _nativeScopeRigidFrame.cameraWeaponLocal,
                    immediateCameraWeaponLocal);
            ROCK_LOG_SAMPLE_DEBUG(
                Weapon,
                2000,
                "TwoHandedGrip: native scope retained weapon-local frame relationError=({:.4f}gu,{:.3f}deg) generation={:016X}",
                rigidPositionError,
                rigidRotationError,
                effectiveGenerationKey);
        }
        _lastRenderedWeaponWorld = weaponNode->world;
        _hasLastRenderedWeaponWorld = isFiniteTransform(_lastRenderedWeaponWorld);
        if (g_rockConfig.rockDebugDrawNativeScopeActivation) {
            _nativeScopeCameraDebugSnapshot = makeNativeScopeCameraDebugSnapshot(_nativeScopeCameraDebugSnapshot, effectiveGenerationKey,
                NativeScopeCameraWriteSource::WeaponVisualAuthority, scopeCameraFollow, scopeCameraResult,
                scopeTargetReady ? _nativeScopeAnchorSource : native_scope_sight_anchor_policy::AnchorSource::None);
        }
        return true;
    }

    bool TwoHandedGrip::clearWeaponCollisionHandAuthority(const bool isLeft)
    {
        const std::size_t index = isLeft ? 0u : 1u;
        _preFrikWeaponHandAuthority[index] = {};
        if (!_weaponCollisionHandAuthorityLive[index]) {
            _weaponCollisionHandAuthorityGenerationKey[index] = 0;
            return true;
        }
        if (!frik_visual_authority::isAvailable() ||
            !frik_visual_authority::clearExternalHandWorldTransform(
                WEAPON_COLLISION_HAND_TAG,
                handFromBool(isLeft))) {
            return false;
        }
        _weaponCollisionHandAuthorityLive[index] = false;
        _weaponCollisionHandAuthorityGenerationKey[index] = 0;
        return true;
    }

    void TwoHandedGrip::suspendNativeReloadSupportHandAuthority(
        const bool isLeft)
    {
        const std::size_t index = isLeft ? 0u : 1u;
        (void)frik_visual_authority::clearHandPose(
            SUPPORT_GRIP_TAG,
            handFromBool(isLeft));
        (void)frik_visual_authority::clearHandPose(
            PRIMARY_GRIP_TAG,
            handFromBool(isLeft));
        clearPreFrikRetainedHandAuthority(
            RetainedHandAuthorityKind::SupportGrip,
            isLeft);
        (void)clearHandAuthorityRoleNow(
            scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip,
            isLeft);
        (void)clearHandAuthorityRoleNow(
            scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip,
            isLeft);
        (void)clearHandAuthorityRoleNow(
            scope_safe_hand_frame_math::HandAuthorityRole::PrimaryDetach,
            isLeft);
        (void)clearWeaponCollisionHandAuthority(isLeft);
        clearHandVisualReturn(isLeft, "native-reload-authority", false);
        _weaponCollisionBaselineHandWorldValid[index] = false;
        _weaponCollisionHandPresentationFromPreviousFrame[index] = false;
        _hasLastPublishedHandWorld[index] = false;

        // Gunstock publishes both participating hands as one rigid group.
        // Yield the complete group so it cannot immediately reacquire the
        // support hand after the role-specific clears above.
        clearGunstockDedicatedHandAuthority();
    }

    void TwoHandedGrip::setNativeReloadHandAuthorityActive(
        const bool active)
    {
        const bool supportHandIsLeft = !_firingHandIsLeft;
        const bool supportRoleChanged =
            _nativeReloadHandAuthorityActive &&
            _nativeReloadSupportHandIsLeft != supportHandIsLeft;

        if (!active) {
            if (_nativeReloadHandAuthorityActive) {
                ROCK_LOG_INFO(
                    Weapon,
                    "TwoHandedGrip: native reload released support-hand FRIK authority hand={}",
                    _nativeReloadSupportHandIsLeft ? "left" : "right");
            }
            _nativeReloadHandAuthorityActive = false;
            _nativeReloadSupportHandIsLeft = supportHandIsLeft;
            return;
        }

        if (_nativeReloadHandAuthorityActive && !supportRoleChanged) {
            return;
        }

        if (supportRoleChanged) {
            suspendNativeReloadSupportHandAuthority(
                _nativeReloadSupportHandIsLeft);
        }
        clearFiringRecoilPresentationState();
        _nativeReloadHandAuthorityActive = true;
        _nativeReloadSupportHandIsLeft = supportHandIsLeft;
        suspendNativeReloadSupportHandAuthority(supportHandIsLeft);
        ROCK_LOG_INFO(
            Weapon,
            "TwoHandedGrip: native reload suspended support-hand FRIK authority hand={} logicalGripRetained={}",
            supportHandIsLeft ? "left" : "right",
            partGrip(supportHandIsLeft).active ? "yes" : "no");
    }

    void TwoHandedGrip::refreshWeaponCollisionHandAuthorityBeforeFrik(
        const EquippedWeaponScopeHandDriverFrame& leftHandDriver,
        const EquippedWeaponScopeHandDriverFrame& rightHandDriver,
        const std::uint64_t currentWeaponGenerationKey,
        const bool firingHandIsLeft,
        const std::uint64_t currentSchedulerSequence)
    {
        const std::array<EquippedWeaponScopeHandDriverFrame, 2> drivers{
            leftHandDriver,
            rightHandDriver,
        };
        for (std::size_t index = 0;
             index < _preFrikWeaponHandAuthority.size();
             ++index) {
            const bool isLeft = index == 0u;
            if (isNativeReloadSupportHand(isLeft)) {
                (void)clearWeaponCollisionHandAuthority(isLeft);
                continue;
            }
            const auto& source = _preFrikWeaponHandAuthority[index];
            const auto& driver = drivers[index];
            const bool sourceCurrent =
                source.valid &&
                _weaponCollisionHandAuthorityLive[index] &&
                driver.valid &&
                currentWeaponGenerationKey != 0 &&
                source.weaponGenerationKey == currentWeaponGenerationKey &&
                source.firingHandIsLeft == firingHandIsLeft &&
                prefrik_hand_authority_policy::isImmediateSuccessor(
                    source.sourceSchedulerSequence,
                    currentSchedulerSequence) &&
                prefrik_hand_authority_policy::isUsableTransform(
                    driver.world) &&
                prefrik_hand_authority_policy::isUsableTransform(
                    source.driverToHandLocal);
            if (!sourceCurrent) {
                if (_weaponCollisionHandAuthorityLive[index] || source.valid) {
                    (void)clearWeaponCollisionHandAuthority(isLeft);
                }
                continue;
            }

            const RE::NiTransform targetWorld =
                prefrik_hand_authority_policy::reconstructTargetWorld(
                    driver.world,
                    source.driverToHandLocal);
            if (!prefrik_hand_authority_policy::isUsableTransform(
                    targetWorld) ||
                !frik_visual_authority::applyExternalHandWorldTransform(
                    WEAPON_COLLISION_HAND_TAG,
                    handFromBool(isLeft),
                    targetWorld,
                    WEAPON_COLLISION_HAND_PRIORITY)) {
                (void)clearWeaponCollisionHandAuthority(isLeft);
            }
        }
    }

    void TwoHandedGrip::recordPreFrikRetainedHandAuthority(
        const RetainedHandAuthorityKind kind,
        const bool isLeft,
        const RE::NiTransform& targetWorld)
    {
        const std::size_t handIndex = isLeft ? 0u : 1u;
        const std::size_t kindIndex = static_cast<std::size_t>(kind);
        auto& source =
            _preFrikRetainedHandAuthorities[handIndex][kindIndex];
        source = {};

        const auto& driver = _currentHandDriverFrames[handIndex];
        const bool generationRequired =
            kind != RetainedHandAuthorityKind::Return;
        if (!driver.valid ||
            _currentSourceSchedulerSequence == 0 ||
            (generationRequired && _activeWeaponGenerationKey == 0) ||
            !prefrik_hand_authority_policy::isUsableTransform(driver.world) ||
            !prefrik_hand_authority_policy::isUsableTransform(targetWorld)) {
            return;
        }

        source.driverToHandLocal =
            prefrik_hand_authority_policy::captureDriverToTargetLocal(
                driver.world,
                targetWorld);
        source.weaponGenerationKey = generationRequired ?
            _activeWeaponGenerationKey :
            0;
        source.sourceSchedulerSequence =
            _currentSourceSchedulerSequence;
        source.firingHandIsLeft = _firingHandIsLeft;
        source.valid =
            prefrik_hand_authority_policy::isUsableTransform(
                source.driverToHandLocal);
    }

    void TwoHandedGrip::clearPreFrikRetainedHandAuthority(
        const RetainedHandAuthorityKind kind,
        const bool isLeft)
    {
        _preFrikRetainedHandAuthorities[isLeft ? 0u : 1u]
            [static_cast<std::size_t>(kind)] = {};
    }

    void TwoHandedGrip::refreshRetainedHandVisualAuthoritiesBeforeFrik(
        const EquippedWeaponScopeHandDriverFrame& leftHandDriver,
        const EquippedWeaponScopeHandDriverFrame& rightHandDriver,
        const std::uint64_t currentWeaponGenerationKey,
        const bool firingHandIsLeft,
        const std::uint64_t currentSchedulerSequence)
    {
        const std::array<EquippedWeaponScopeHandDriverFrame, 2> drivers{
            leftHandDriver,
            rightHandDriver,
        };
        for (std::size_t handIndex = 0; handIndex < 2; ++handIndex) {
            const bool isLeft = handIndex == 0u;
            if (isNativeReloadSupportHand(isLeft)) {
                continue;
            }
            for (std::size_t kindIndex = 0;
                 kindIndex < kRetainedHandAuthorityKindCount;
                 ++kindIndex) {
                const auto kind =
                    static_cast<RetainedHandAuthorityKind>(kindIndex);
                const char* tag = nullptr;
                int priority = GRIP_HAND_POSE_PRIORITY;
                switch (kind) {
                case RetainedHandAuthorityKind::PrimaryGrip:
                    tag = PRIMARY_GRIP_TAG;
                    break;
                case RetainedHandAuthorityKind::SupportGrip:
                    tag = SUPPORT_GRIP_TAG;
                    break;
                case RetainedHandAuthorityKind::GunstockAlignment:
                    tag = GUNSTOCK_ALIGNMENT_TAG;
                    break;
                case RetainedHandAuthorityKind::Return:
                    tag = RETURN_HAND_TAG;
                    priority = RETURN_HAND_VISUAL_PRIORITY;
                    break;
                case RetainedHandAuthorityKind::Count:
                    continue;
                }

                auto& source =
                    _preFrikRetainedHandAuthorities[handIndex][kindIndex];
                const auto hand = handFromBool(isLeft);
                if (!frik_visual_authority::
                        hasPublishedExternalHandWorldTransform(tag, hand)) {
                    source = {};
                    continue;
                }

                const bool generationRequired =
                    kind != RetainedHandAuthorityKind::Return;
                const auto& driver = drivers[handIndex];
                const bool sourceCurrent =
                    source.valid &&
                    driver.valid &&
                    prefrik_hand_authority_policy::isImmediateSuccessor(
                        source.sourceSchedulerSequence,
                        currentSchedulerSequence) &&
                    (!generationRequired ||
                        (currentWeaponGenerationKey != 0 &&
                            source.weaponGenerationKey ==
                                currentWeaponGenerationKey &&
                            source.firingHandIsLeft == firingHandIsLeft)) &&
                    prefrik_hand_authority_policy::isUsableTransform(
                        driver.world) &&
                    prefrik_hand_authority_policy::isUsableTransform(
                        source.driverToHandLocal);
                const RE::NiTransform refreshedHandWorld = sourceCurrent ?
                    prefrik_hand_authority_policy::reconstructTargetWorld(
                        driver.world,
                        source.driverToHandLocal) :
                    RE::NiTransform{};
                if (!sourceCurrent ||
                    !prefrik_hand_authority_policy::isUsableTransform(
                        refreshedHandWorld) ||
                    !frik_visual_authority::applyExternalHandWorldTransform(
                        tag,
                        hand,
                        refreshedHandWorld,
                        priority)) {
                    (void)frik_visual_authority::
                        clearExternalHandWorldTransform(tag, hand);
                    source = {};
                }
            }
        }
    }

    void TwoHandedGrip::beginWeaponCollisionPresentationFrame(
        const std::uint64_t currentWeaponGenerationKey)
    {
        _weaponCollisionHandPresentationFromPreviousFrame =
            _weaponCollisionHandAuthorityLive;
        _weaponCollisionBaselineHandWorldValid = {};

        /*
         * FRIK V2 owns tagged transforms as persistent claims and consumes the
         * winning claim in its next regular skeleton update. Clearing and then
         * recreating this tag every ROCK frame opened a scheduler-dependent
         * window in which FRIK could select the lower-priority grip claim. Keep
         * the same owner registered and update its value after post-solve.
         * A weapon-generation edge is different: FRIK already consumed the old
         * claim for this frame, so preserve the previous-presentation witness
         * but retire the claim before the replacement weapon can publish.
         */
        bool generationClaimsCleared = true;
        for (std::size_t index = 0; index < _weaponCollisionHandAuthorityLive.size(); ++index) {
            if (!_weaponCollisionHandAuthorityLive[index] ||
                (currentWeaponGenerationKey != 0 &&
                    _weaponCollisionHandAuthorityGenerationKey[index] == currentWeaponGenerationKey)) {
                continue;
            }
            generationClaimsCleared =
                clearWeaponCollisionHandAuthority(index == 0u) &&
                generationClaimsCleared;
        }
        if (!generationClaimsCleared) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                1000,
                "TwoHandedGrip: stale dynamic weapon collision hand authority clear failed generation={:016X} live(L/R)={}/{}",
                currentWeaponGenerationKey,
                _weaponCollisionHandAuthorityLive[0],
                _weaponCollisionHandAuthorityLive[1]);
        }
    }

    void TwoHandedGrip::finishWeaponCollisionPresentationFrame(
        const bool presentationActive)
    {
        if (presentationActive) {
            return;
        }

        const bool leftCleared = clearWeaponCollisionHandAuthority(true);
        const bool rightCleared = clearWeaponCollisionHandAuthority(false);
        if (!leftCleared || !rightCleared) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                1000,
                "TwoHandedGrip: inactive dynamic weapon contact presentation hand authority clear failed left={} right={} live(L/R)={}/{}",
                leftCleared ? "ok" : "failed",
                rightCleared ? "ok" : "failed",
                _weaponCollisionHandAuthorityLive[0],
                _weaponCollisionHandAuthorityLive[1]);
        }
    }

    bool TwoHandedGrip::applyFiringWeaponRecoilPresentation(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey)
    {
        const std::uint64_t acceptedSequence =
            _firingRecoilAcceptedSequence;
        if (acceptedSequence == 0 ||
            acceptedSequence == _firingRecoilConsumedSequence) {
            return false;
        }

        // Consume first so a failed/stale sample can never kick a later weapon.
        _firingRecoilConsumedSequence = acceptedSequence;
        const bool acceptedHandIsLeft =
            _firingRecoilAcceptedHandIsLeft;
        const std::size_t acceptedHandIndex =
            acceptedHandIsLeft ? 0u : 1u;
        if (!weaponNode ||
            weaponNode != _activeWeaponNode ||
            currentWeaponGenerationKey == 0 ||
            currentWeaponGenerationKey != _activeWeaponGenerationKey ||
            currentWeaponGenerationKey !=
                _firingRecoilAcceptedGenerationKey ||
            acceptedHandIsLeft != _firingHandIsLeft ||
            !hasControlledFiringRecoilAuthority(acceptedHandIsLeft) ||
            !_hasFiringRecoilReference[acceptedHandIndex] ||
            _firingRecoilReferenceGenerationKey[acceptedHandIndex] !=
                currentWeaponGenerationKey ||
            !isFiniteTransform(weaponNode->world)) {
            return false;
        }

        RE::NiTransform presentedFiringHandWorld{};
        if (!tryGetRootFlattenedHandBoneTransform(
                acceptedHandIsLeft,
                presentedFiringHandWorld)) {
            return false;
        }

        const RE::NiTransform recoilWorldDelta =
            gunstock_alignment_policy::deriveAppliedWorldDelta(
                _firingRecoilReferenceHandWorld[acceptedHandIndex],
                presentedFiringHandWorld);
        if (!isUsableHandAuthorityTransform(recoilWorldDelta)) {
            return false;
        }

        const RE::NiTransform recoiledWeaponWorld =
            transform_math::composeTransforms(
                recoilWorldDelta,
                weaponNode->world);
        if (!isFiniteTransform(recoiledWeaponWorld)) {
            return false;
        }

        // This is a terminal presentation overlay, not new collision intent.
        // The collision bodies and muzzle sample the resulting weapon below.
        return applyWeaponVisualAuthority(
            weaponNode,
            recoiledWeaponWorld,
            currentWeaponGenerationKey,
            false);
    }

    bool TwoHandedGrip::applyWeaponCollisionResolvedAuthority(
        RE::NiNode* weaponNode,
        const RE::NiTransform& requestedWeaponWorld,
        const RE::NiTransform& resolvedWeaponWorld,
        const std::uint64_t authorityGenerationKey)
    {
        if (!weaponNode ||
            !isFiniteTransform(weaponNode->world) ||
            !isFiniteTransform(requestedWeaponWorld) ||
            !isFiniteTransform(resolvedWeaponWorld)) {
            return false;
        }


        const RE::NiTransform scaleStableRequestedWeaponWorld =
            weapon_visual_authority_math::preserveLiveWeaponWorldScale(
                weaponNode->world,
                requestedWeaponWorld);
        const RE::NiTransform scaleStableResolvedWeaponWorld =
            weapon_visual_authority_math::preserveLiveWeaponWorldScale(
                weaponNode->world,
                resolvedWeaponWorld);

        const auto attachedHands = weaponCollisionAttachedHands();

        struct CollisionHandPulse
        {
            RE::NiTransform targetWorld{};
            bool isLeft{ false };
            bool requested{ false };
            bool targetValid{ false };
            bool applied{ false };
            bool retained{ false };
        };
        std::array<CollisionHandPulse, 2> pulses{
            CollisionHandPulse{
                .isLeft = true,
                .requested = attachedHands.left,
            },
            CollisionHandPulse{
                .isLeft = false,
                .requested = attachedHands.right,
            },
        };

        bool detachedHandsCleared = true;
        for (const auto& pulse : pulses) {
            if (!pulse.requested) {
                detachedHandsCleared =
                    clearWeaponCollisionHandAuthority(pulse.isLeft) &&
                    detachedHandsCleared;
            }
        }

        const bool anyHandRequested = attachedHands.left || attachedHands.right;
        bool handTargetsReady =
            !anyHandRequested || frik_visual_authority::isAvailable();
        for (auto& pulse : pulses) {
            if (!pulse.requested) {
                continue;
            }
            /*
             * A locked firing/support pose was already published earlier in
             * this ROCK frame. It is the exact collision-free hand baseline;
             * replacing it with controller input at this higher priority makes
             * the hand slide off the part and feeds equip-time rotation back
             * through FRIK. Native passive carry has no ROCK hand publication,
             * so it deliberately falls back to the scope-safe physical input.
             * The weapon basis remains the explicit pre-physics intent because
             * weaponNode->world can contain the previous deferred claim.
             */
            const std::size_t handIndex = pulse.isLeft ? 0u : 1u;
            RE::NiTransform collisionFreeHandWorld{};
            bool collisionFreeHandValid = false;
            if (_weaponCollisionBaselineHandWorldValid[handIndex]) {
                collisionFreeHandWorld =
                    _weaponCollisionBaselineHandWorld[handIndex];
                collisionFreeHandValid =
                    isUsableHandAuthorityTransform(collisionFreeHandWorld);
            } else {
                collisionFreeHandValid =
                    tryGetSolverHandTransform(
                        pulse.isLeft,
                        collisionFreeHandWorld) &&
                    isUsableHandAuthorityTransform(collisionFreeHandWorld);
            }
            pulse.targetWorld =
                dynamic_weapon_collision_policy::reframeAttachedHand(
                    scaleStableRequestedWeaponWorld,
                    scaleStableResolvedWeaponWorld,
                    collisionFreeHandWorld);
            pulse.targetValid =
                collisionFreeHandValid &&
                isUsableHandAuthorityTransform(pulse.targetWorld);
            handTargetsReady = handTargetsReady && pulse.targetValid;
        }

        bool handPulsesSucceeded = handTargetsReady && detachedHandsCleared;
        if (handTargetsReady) {
            for (auto& pulse : pulses) {
                if (!pulse.requested) {
                    continue;
                }
                const auto hand = handFromBool(pulse.isLeft);
                pulse.applied =
                    frik_visual_authority::applyExternalHandWorldTransform(
                        WEAPON_COLLISION_HAND_TAG,
                        hand,
                        pulse.targetWorld,
                        WEAPON_COLLISION_HAND_PRIORITY);
                /*
                 * Update the same high-priority owner in place. Clearing it at
                 * either frame boundary would let FRIK's regular solve select
                 * the live priority-100 firing/support owner for one scheduling
                 * interval. Each physical hand remains reconstructed from its
                 * own unaffected driver while this claim is live.
                 */
                pulse.retained = pulse.applied;
                if (pulse.retained) {
                    const std::size_t handIndex = pulse.isLeft ? 0u : 1u;
                    _weaponCollisionHandAuthorityLive[handIndex] = true;
                    _weaponCollisionHandAuthorityGenerationKey[handIndex] =
                        authorityGenerationKey;
                    auto& preFrikSource =
                        _preFrikWeaponHandAuthority[handIndex];
                    preFrikSource = {};
                    const auto& driver =
                        _currentHandDriverFrames[handIndex];
                    if (driver.valid &&
                        _currentSourceSchedulerSequence != 0 &&
                        authorityGenerationKey != 0 &&
                        prefrik_hand_authority_policy::isUsableTransform(
                            driver.world)) {
                        preFrikSource.driverToHandLocal =
                            prefrik_hand_authority_policy::
                                captureDriverToTargetLocal(
                                    driver.world,
                                    pulse.targetWorld);
                        preFrikSource.weaponGenerationKey =
                            authorityGenerationKey;
                        preFrikSource.sourceSchedulerSequence =
                            _currentSourceSchedulerSequence;
                        preFrikSource.firingHandIsLeft = _firingHandIsLeft;
                        preFrikSource.valid =
                            prefrik_hand_authority_policy::isUsableTransform(
                                preFrikSource.driverToHandLocal);
                    }
                }
                handPulsesSucceeded =
                    handPulsesSucceeded && pulse.applied && pulse.retained;
            }
        }

        /*
         * A firing-hand pulse can propagate through the weapon's native parent
         * chain. Publish the solver-authoritative weapon last so the final
         * rendered weapon pose is exact while both hands keep the rigid
         * pre-collision weapon-local relationship captured above.
         */
        const bool weaponPublished = applyWeaponVisualAuthority(
            weaponNode,
            scaleStableResolvedWeaponWorld,
            authorityGenerationKey,
            false);
        if (!weaponPublished || !handPulsesSucceeded) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                1000,
                "TwoHandedGrip: dynamic weapon collision group publication incomplete weapon={} hands={} left(req/target/apply/live)={}/{}/{}/{} right(req/target/apply/live)={}/{}/{}/{} state={} firingHand={}",
                weaponPublished ? "ok" : "failed",
                handPulsesSucceeded ? "ok" : "failed",
                pulses[0].requested,
                pulses[0].targetValid,
                pulses[0].applied,
                pulses[0].retained,
                pulses[1].requested,
                pulses[1].targetValid,
                pulses[1].applied,
                pulses[1].retained,
                static_cast<int>(_state),
                _firingHandIsLeft ? "left" : "right");
        }
        return weaponPublished && handPulsesSucceeded;
    }

    bool TwoHandedGrip::applyFiringHandLockedVisual(RE::NiNode* weaponNode, float dt, const RE::NiTransform* liveHandWorld)
    {
        if (!weaponNode || !_hasFiringHandWeaponLocal) {
            return false;
        }
        if (!scope_safe_hand_frame_math::shouldPublishLockedHandVisualAuthority(_scopeMenuOpenThisFrame)) {
            return true;
        }
        if (!frik_visual_authority::isAvailable()) {
            return false;
        }

        const RE::NiTransform firingHandWorld =
            weapon_visual_authority_math::weaponLocalFrameToWorld(weaponNode->world, _primaryHandWeaponLocal);
        const auto& returningHand = _returningHandVisuals[_firingHandIsLeft ? 0u : 1u].transition;
        const RE::NiTransform* acquisitionStart =
            returningHand.active && isUsableHandAuthorityTransform(returningHand.lastApplied) ?
            &returningHand.lastApplied :
            liveHandWorld;
        const bool synchronizedDynamicAcquisition =
            dynamicSupportAcquisitionMatches(
                !_firingHandIsLeft,
                partGrip(!_firingHandIsLeft));
        const RE::NiTransform appliedFiringHandWorld =
            synchronizedDynamicAcquisition ?
            resolveDynamicSupportAcquisitionHandTarget(
                firingHandWorld,
                true,
                _primaryHandVisualLerp) :
            resolveLockedHandVisualTarget(
                firingHandWorld,
                acquisitionStart,
                dt,
                _primaryHandVisualLerp);
        (void)publishAuthoredPrimaryFiringGripFingerPose(_firingHandIsLeft);
        const bool applied = frik_visual_authority::applyExternalHandWorldTransform(
            PRIMARY_GRIP_TAG, handFromBool(_firingHandIsLeft), appliedFiringHandWorld, GRIP_HAND_POSE_PRIORITY);
        if (applied) {
            recordPreFrikRetainedHandAuthority(
                RetainedHandAuthorityKind::PrimaryGrip,
                _firingHandIsLeft,
                appliedFiringHandWorld);
            recordScopeHandAuthorityPublication(scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip, _firingHandIsLeft);
            clearHandVisualReturn(_firingHandIsLeft, "firing-grip-authority-acquired", false);
            recordPublishedHandWorld(_firingHandIsLeft, appliedFiringHandWorld);
        }
        return applied;
    }

    bool TwoHandedGrip::applyPartGripLockedVisual(bool isLeft, RE::NiNode* weaponNode, float dt, const RE::NiTransform* liveHandWorld)
    {
        WeaponPartGrip& grip = partGrip(isLeft);
        if (!weaponNode || !grip.active || !grip.hasHandWeaponLocal) {
            return false;
        }
        if (isNativeReloadSupportHand(isLeft)) {
            return true;
        }
        if (!scope_safe_hand_frame_math::shouldPublishLockedHandVisualAuthority(_scopeMenuOpenThisFrame)) {
            return true;
        }
        if (!frik_visual_authority::isAvailable()) {
            return false;
        }

        const RE::NiTransform partGripHandWorld = resolvePartGripHandWorld(grip, weaponNode);
        const auto& returningHand = _returningHandVisuals[isLeft ? 0u : 1u].transition;
        const RE::NiTransform* acquisitionStart =
            returningHand.active && isUsableHandAuthorityTransform(returningHand.lastApplied) ?
            &returningHand.lastApplied :
            liveHandWorld;
        const bool synchronizedDynamicAcquisition =
            dynamicSupportAcquisitionMatches(isLeft, grip);
        const RE::NiTransform appliedHandWorld =
            synchronizedDynamicAcquisition ?
            resolveDynamicSupportAcquisitionHandTarget(
                partGripHandWorld,
                false,
                grip.visualLerp) :
            resolveLockedHandVisualTarget(
                partGripHandWorld,
                acquisitionStart,
                dt,
                grip.visualLerp);
        const bool applied = frik_visual_authority::applyExternalHandWorldTransform(
            SUPPORT_GRIP_TAG, handFromBool(isLeft), appliedHandWorld, GRIP_HAND_POSE_PRIORITY);
        if (applied) {
            recordPreFrikRetainedHandAuthority(
                RetainedHandAuthorityKind::SupportGrip,
                isLeft,
                appliedHandWorld);
            recordScopeHandAuthorityPublication(scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip, isLeft);
            clearHandVisualReturn(isLeft, "part-grip-authority-acquired", false);
            recordPublishedHandWorld(isLeft, appliedHandWorld);
        }
        return applied;
    }

    bool TwoHandedGrip::applyLockedHandVisualAuthority(
        RE::NiNode* weaponNode,
        bool applyPrimaryHand,
        bool applySupportHand,
        float dt,
        const RE::NiTransform* livePrimaryHandWorld,
        const RE::NiTransform* liveSupportHandWorld)
    {
        if (!weaponNode) {
            return false;
        }

        if (!scope_safe_hand_frame_math::shouldPublishLockedHandVisualAuthority(_scopeMenuOpenThisFrame)) {
            return true;
        }

        if (!frik_visual_authority::isAvailable()) {
            return false;
        }

        if (!applyPrimaryHand && !applySupportHand) {
            return true;
        }

        const bool supportHandIsLeft = !_firingHandIsLeft;
        bool primaryApplied = true;
        bool supportApplied = true;
        if (applyPrimaryHand) {
            primaryApplied = applyFiringHandLockedVisual(weaponNode, dt, livePrimaryHandWorld);
        }
        if (applySupportHand) {
            supportApplied = applyPartGripLockedVisual(supportHandIsLeft, weaponNode, dt, liveSupportHandWorld);
        }
        if (primaryApplied && supportApplied) {
            return true;
        }

        ROCK_LOG_WARN(Weapon,
            "TwoHandedGrip: locked hand authority publication failed primary={} support={} "
            "firingHand={} supportHand={} state={} scopeMenu={} primaryFrame={} supportGrip={} supportFrame={}",
            primaryApplied ? "ok" : "failed",
            supportApplied ? "ok" : "failed",
            _firingHandIsLeft ? "left" : "right",
            supportHandIsLeft ? "left" : "right",
            static_cast<int>(_state),
            _scopeMenuOpenThisFrame ? "open" : "closed",
            _hasFiringHandWeaponLocal ? "ready" : "missing",
            partGrip(supportHandIsLeft).active ? "active" : "inactive",
            partGrip(supportHandIsLeft).hasHandWeaponLocal ? "ready" : "missing");

        if (applyPrimaryHand && primaryApplied) {
            (void)clearHandAuthorityRoleNow(scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip, _firingHandIsLeft);
        }
        if (applySupportHand && supportApplied) {
            (void)clearHandAuthorityRoleNow(scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip, supportHandIsLeft);
        }
        return false;
    }

    void TwoHandedGrip::publishGripHandPoses(bool isLeft)
    {
        if (isNativeReloadSupportHand(isLeft) ||
            !frik_visual_authority::isAvailable()) {
            return;
        }

        const WeaponPartGrip& grip = partGrip(isLeft);
        if (weapon_visual_authority_math::shouldPublishTwoHandedGripPose(weapon_visual_authority_math::LockedHandRole::Support) && grip.hasFingerPose) {
            const auto handPose = grip.hasFingerSplay ?
                frik_visual_authority::makeHandPoseDataFromJointValues(grip.fingerPose, grip.fingerSplayRadians) :
                frik_visual_authority::makeHandPoseDataFromJointValues(grip.fingerPose);
            (void)frik_visual_authority::setHandPoseCustomWithPriority(
                SUPPORT_GRIP_TAG,
                handFromBool(isLeft),
                handPose,
                GRIP_HAND_POSE_PRIORITY);
        }

        if (grip.hasFingerLocalTransforms) {
            frik_visual_authority::FingerLocalTransformOverride overrideData{};
            overrideData.enabledMask = grip.fingerLocalTransformMask;
            for (std::size_t i = 0; i < grip.fingerLocalTransforms.size(); ++i) {
                overrideData.localTransforms[i] = grip.fingerLocalTransforms[i];
            }
            (void)frik_visual_authority::setHandPoseCustomLocalTransformsWithPriority(SUPPORT_GRIP_TAG, handFromBool(isLeft), &overrideData, GRIP_HAND_POSE_PRIORITY);
        }
    }

    void TwoHandedGrip::clearPrimaryGripPose(bool isLeft)
    {
        _hasLastPublishedHandWorld[isLeft ? 0u : 1u] = false;
        if (_authoredPrimaryFingerPosePublished && _publishedFiringFingerPoseIsLeft == isLeft) {
            clearAuthoredPrimaryFiringGripFingerPose();
        } else {
            (void)frik_visual_authority::clearHandPose(PRIMARY_GRIP_TAG, handFromBool(isLeft));
        }
        deferOrClearHandAuthorityRole(
            scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip,
            isLeft);
    }

    void TwoHandedGrip::clearPrimaryDetachVisualAuthority(bool isLeft)
    {
        (void)frik_visual_authority::clearHandPose(PRIMARY_DETACH_TAG, handFromBool(isLeft));
        deferOrClearHandAuthorityRole(
            scope_safe_hand_frame_math::HandAuthorityRole::PrimaryDetach,
            isLeft);
    }

    void TwoHandedGrip::killFrikOffhandGrip()
    {
        if (frik_visual_authority::blockOffHandWeaponGripping("ROCK_TwoHanded", true)) {
            ROCK_LOG_DEBUG(Weapon, "FRIK offhand grip suppressed");
        }
    }

    void TwoHandedGrip::restoreFrikOffhandGrip()
    {
        if (frik_visual_authority::blockOffHandWeaponGripping("ROCK_TwoHanded", false)) {
            ROCK_LOG_DEBUG(Weapon, "FRIK offhand grip restored");
        }
    }

    bool TwoHandedGrip::blockFrikPrimaryWeaponPose()
    {
        if (frik_visual_authority::blockPrimaryHandWeaponPose("ROCK_PrimaryDetach", true)) {
            ROCK_LOG_DEBUG(Weapon, "FRIK primary weapon pose suppressed");
            return true;
        }
        return false;
    }

    void TwoHandedGrip::restoreFrikPrimaryWeaponPose()
    {
        if (frik_visual_authority::blockPrimaryHandWeaponPose("ROCK_PrimaryDetach", false)) {
            ROCK_LOG_DEBUG(Weapon, "FRIK primary weapon pose restored");
        }
    }

    void TwoHandedGrip::clearRightFiringHandCanonicalFrame()
    {
        _rightFiringHandCanonicalWeaponLocal = {};
        _rightFiringGripCanonicalWeaponLocal = {};
        _rightFiringHandCanonicalWeaponNode = nullptr;
        _rightFiringHandCanonicalGenerationKey = 0;
        _rightFiringHandCanonicalOwnershipKey = 0;
        _rightFiringHandCanonicalCaptureSequence = 0;
        _rightFiringHandCanonicalSource = RightFiringCanonicalSource::None;
        _hasRightFiringHandCanonicalWeaponLocal = false;
        _rightFiringFingerLocalTransforms = {};
        _leftFiringFingerLocalTransforms = {};
        _rightFiringFingerLocalTransformMask = 0;
        _leftFiringFingerLocalTransformMask = 0;
    }

    bool TwoHandedGrip::hasRightFiringHandCanonicalFrame(
        const RE::NiNode* weaponNode,
        const std::uint64_t weaponGenerationKey,
        const std::uint64_t weaponOwnershipKey) const
    {
        return weaponNode &&
               weaponGenerationKey != 0 &&
               _hasRightFiringHandCanonicalWeaponLocal &&
               _rightFiringHandCanonicalWeaponNode == weaponNode &&
               _rightFiringHandCanonicalGenerationKey == weaponGenerationKey &&
               _rightFiringHandCanonicalOwnershipKey == weaponOwnershipKey &&
               _rightFiringHandCanonicalSource != RightFiringCanonicalSource::None;
    }

    void TwoHandedGrip::rememberRightFiringHandCanonicalFrame()
    {
        if (_firingHandIsLeft || !_activeWeaponNode ||
            !_hasFiringHandWeaponLocal || _activeWeaponGenerationKey == 0) {
            return;
        }
        if (hasRightFiringHandCanonicalFrame(
                _activeWeaponNode,
                _activeWeaponGenerationKey,
                _activeEquippedWeaponOwnershipKey) &&
            _rightFiringHandCanonicalSource ==
                RightFiringCanonicalSource::AuthoredAnimation) {
            return;
        }
        _rightFiringHandCanonicalWeaponLocal = _primaryHandWeaponLocal;
        _rightFiringGripCanonicalWeaponLocal = _primaryGripLocal;
        _rightFiringHandCanonicalWeaponNode = _activeWeaponNode;
        _rightFiringHandCanonicalGenerationKey = _activeWeaponGenerationKey;
        _rightFiringHandCanonicalOwnershipKey = _activeEquippedWeaponOwnershipKey;
        _rightFiringHandCanonicalCaptureSequence = 0;
        _rightFiringHandCanonicalSource = RightFiringCanonicalSource::NativeCarry;
        _hasRightFiringHandCanonicalWeaponLocal = true;
    }

    void TwoHandedGrip::refreshNaturalHandInWandFrames()
    {
        auto* playerNodes = f4vr::getPlayerNodes();
        if (!playerNodes) {
            return;
        }

        const auto refreshHand = [&](const bool isLeft,
                                     RE::NiNode* wandNode,
                                     RE::NiNode* dampedDriverNode,
                                     RE::NiTransform& outBoneInWand,
                                     bool& outWandValid,
                                     RE::NiTransform& outBoneInDampedDriver,
                                     bool& outDampedDriverValid) {
            if (hasVisualAuthorityForHand(isLeft)) {
                return;
            }

            RE::NiTransform handWorld{};
            if (!tryGetSolverHandTransform(isLeft, handWorld)) {
                return;
            }

            const auto captureRelation = [&](RE::NiNode* sourceNode,
                                             RE::NiTransform& outRelation,
                                             bool& outValid) {
                if (!sourceNode || !isFiniteTransform(sourceNode->world)) {
                    return;
                }
                const RE::NiTransform relation =
                    transform_math::composeTransforms(
                        transform_math::invertTransform(sourceNode->world),
                        handWorld);
                if (!isFiniteTransform(relation) ||
                    std::sqrt(dot(
                        relation.translate,
                        relation.translate)) >
                        two_handed_grip_detail::
                            kMaximumBoneToDriverDistanceGameUnits) {
                    return;
                }
                outRelation = relation;
                outValid = true;
            };

            captureRelation(wandNode, outBoneInWand, outWandValid);
            captureRelation(
                dampedDriverNode,
                outBoneInDampedDriver,
                outDampedDriverValid);
        };

        refreshHand(
            false,
            playerNodes->primaryWandNode,
            playerNodes->primaryWeaponOffsetNOde,
            _rightNaturalBoneInWand,
            _hasRightNaturalBoneInWand,
            _rightNaturalBoneInDampedDriver,
            _hasRightNaturalBoneInDampedDriver);
        refreshHand(
            true,
            playerNodes->SecondaryWandNode,
            playerNodes->SecondaryMeleeWeaponOffsetNode2,
            _leftNaturalBoneInWand,
            _hasLeftNaturalBoneInWand,
            _leftNaturalBoneInDampedDriver,
            _hasLeftNaturalBoneInDampedDriver);
    }

    void TwoHandedGrip::refreshRightNativeCanonicalFrame(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey)
    {
        /*
         * Passive canonical capture: whenever the equipped weapon rides the
         * native RIGHT hand (no ROCK transform ownership), the live weapon
         * pose already carries FRIK's authored per-weapon offsets, so the
         * canonical right hold and its weapon-in-wand frame can refresh
         * continuously. Without this, a weapon that was never
         * right-firing-gripped in the session had no canonical, and a LEFT
         * takeover fell back to the raw squeeze capture - the per-weapon
         * offsets (e.g. the UMP's large forward offset) silently missing
         * from the mirrored left hold ("worked before by coincidence").
         */
        if (_weaponCollisionHandPresentationFromPreviousFrame[1] ||
            isManualOwnershipActive() || _weaponNodeOwnershipBlockEngaged ||
            !scope_safe_hand_frame_math::canRefreshRightFiringCanonicalFrame(_scopeMenuOpenThisFrame, _scopeSafeHandFrames[1].rootRebaseActive) || !weaponNode ||
            currentWeaponGenerationKey == 0 || !isFiniteTransform(weaponNode->world)) {
            return;
        }
        auto* playerNodes = f4vr::getPlayerNodes();
        RE::NiTransform rightHandWorld{};
        if (!playerNodes || !playerNodes->primaryWandNode ||
            !isFiniteTransform(playerNodes->primaryWandNode->world) ||
            !tryGetSolverHandTransform(false, rightHandWorld)) {
            return;
        }
        const RE::NiTransform boneInRightWand = transform_math::composeTransforms(
            transform_math::invertTransform(playerNodes->primaryWandNode->world), rightHandWorld);
        // Same wrist-range gate as the mirror's wand-map sampling, plus a
        // loose weapon-to-hand bound so a mid-equip/mid-teleport frame never
        // poisons the canonical.
        constexpr float kMaxWeaponToHandDistance = 100.0f;
        const RE::NiPoint3 weaponToHand = sub(weaponNode->world.translate, rightHandWorld.translate);
        if (!isFiniteTransform(boneInRightWand) ||
            std::sqrt(dot(boneInRightWand.translate, boneInRightWand.translate)) >
                two_handed_grip_detail::
                    kMaximumBoneToDriverDistanceGameUnits ||
            std::sqrt(dot(weaponToHand, weaponToHand)) > kMaxWeaponToHandDistance) {
            return;
        }
        const RE::NiTransform canonicalHold = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), rightHandWorld);
        const RE::NiPoint3 canonicalGrip = worldToWeaponLocal(computeGrabLegacyPalmPivotAWorldFromHandBasis(rightHandWorld, false), weaponNode);
        if (!isFiniteTransform(canonicalHold) || !std::isfinite(canonicalGrip.x) || !std::isfinite(canonicalGrip.y) || !std::isfinite(canonicalGrip.z)) {
            return;
        }
        // The animation capture is a more direct authority than a later
        // presentation sample. Preserve it for this exact weapon identity,
        // generation, and ownership.
        if (hasRightFiringHandCanonicalFrame(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey) &&
            _rightFiringHandCanonicalSource ==
                RightFiringCanonicalSource::AuthoredAnimation) {
            return;
        }

        _rightFiringHandCanonicalWeaponLocal = canonicalHold;
        _rightFiringGripCanonicalWeaponLocal = canonicalGrip;
        _rightFiringHandCanonicalWeaponNode = weaponNode;
        _rightFiringHandCanonicalGenerationKey = currentWeaponGenerationKey;
        _rightFiringHandCanonicalOwnershipKey = currentEquippedWeaponOwnershipKey;
        _rightFiringHandCanonicalCaptureSequence = 0;
        _rightFiringHandCanonicalSource = RightFiringCanonicalSource::NativeCarry;
        _hasRightFiringHandCanonicalWeaponLocal = true;
    }

    bool TwoHandedGrip::tryComputeMirroredLeftFiringHandWeaponLocal(
        RE::NiTransform& outHandWeaponLocal,
        bool* outUsedAuthoredCanonical,
        const bool logDiagnostic) const
    {
        if (outUsedAuthoredCanonical) {
            *outUsedAuthoredCanonical = false;
        }
        if (!hasRightFiringHandCanonicalFrame(
                _activeWeaponNode,
                _activeWeaponGenerationKey,
                _activeEquippedWeaponOwnershipKey)) {
            return false;
        }

        RE::NiTransform leftHandWorld{};
        if (!tryGetSolverHandTransform(true, leftHandWorld)) {
            return false;
        }

        /*
         * A part-gripping right hand is visually locked to the weapon part,
         * so its live bone no longer expresses the natural bone-in-wand
         * relation the wand conjugation depends on - a left reattach from a
         * right offhand carry came out at whatever angle the lock left the
         * bone. Replay the natural relation (snapshotted during native right
         * carry) onto the live right wand instead; every unlocked case keeps
         * the live sample the confirmed takeover path uses.
         */
        RE::NiTransform rightHandWorld{};
        if (partGrip(false).active && _hasRightNaturalBoneInWand) {
            auto* playerNodes = f4vr::getPlayerNodes();
            if (!playerNodes || !playerNodes->primaryWandNode || !isFiniteTransform(playerNodes->primaryWandNode->world)) {
                return false;
            }
            rightHandWorld = transform_math::composeTransforms(playerNodes->primaryWandNode->world, _rightNaturalBoneInWand);
        } else if (!tryGetSolverHandTransform(false, rightHandWorld)) {
            return false;
        }

        const bool mirrored = tryBuildMirroredLeftFiringHandWeaponLocal(
            _rightFiringHandCanonicalWeaponLocal,
            _rightFiringGripCanonicalWeaponLocal,
            rightHandWorld,
            leftHandWorld,
            outHandWeaponLocal,
            logDiagnostic);
        if (!mirrored) {
            return false;
        }

        const bool usedAuthoredCanonical =
            _rightFiringHandCanonicalSource ==
            RightFiringCanonicalSource::AuthoredAnimation;
        if (outUsedAuthoredCanonical) {
            *outUsedAuthoredCanonical = usedAuthoredCanonical;
        }
        if (logDiagnostic) {
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: left firing hold resolved source={} generation={:016X} ownership={:016X} capture={} mirroredHandWeaponT=({:.3f},{:.3f},{:.3f})",
                usedAuthoredCanonical ? "authored-animation" : "native-carry",
                _rightFiringHandCanonicalGenerationKey,
                _rightFiringHandCanonicalOwnershipKey,
                _rightFiringHandCanonicalCaptureSequence,
                outHandWeaponLocal.translate.x,
                outHandWeaponLocal.translate.y,
                outHandWeaponLocal.translate.z);
        }
        return true;
    }

    bool TwoHandedGrip::tryBuildMirroredRightSupportHandWeaponLocal(
        const RE::NiTransform& leftHandWeaponLocal,
        RE::NiTransform& outRightHandWeaponLocal) const
    {
        outRightHandWeaponLocal = {};
        if (!_hasLeftNaturalBoneInWand || !_hasRightNaturalBoneInWand ||
            !isFiniteTransform(leftHandWeaponLocal) ||
            !isFiniteTransform(_leftNaturalBoneInWand) ||
            !isFiniteTransform(_rightNaturalBoneInWand)) {
            return false;
        }

        /*
         * Mirror only ORIENTATION through the physical wand pair. The cached
         * bone-in-wand transforms remove hFRIK's asymmetric hand-bone
         * conventions, but their translations/scales are presentation state
         * and can be collapsed while ROCK owns left-primary carry. Feeding
         * those affine values through inverse composition produced enormous
         * intermediate translations and a catastrophically cancelled right
         * support seat. Position is anchored independently below, so rigid
         * zero-origin frames are the complete source authority here.
         */
        const auto orientationFrame = [](const RE::NiTransform& source) {
            RE::NiTransform result = source;
            result.translate = {};
            result.scale = 1.0f;
            return result;
        };
        const RE::NiTransform leftBoneInWandOrientation =
            orientationFrame(_leftNaturalBoneInWand);
        const RE::NiTransform rightBoneInWandOrientation =
            orientationFrame(_rightNaturalBoneInWand);
        const RE::NiTransform leftHandWeaponOrientation =
            orientationFrame(leftHandWeaponLocal);

        MirroredDriverConjugation conjugation{};
        if (!tryBuildMirroredDriverConjugation(
                leftHandWeaponOrientation,
                leftBoneInWandOrientation,
                rightBoneInWandOrientation,
                nullptr,
                conjugation)) {
            return false;
        }
        RE::NiTransform mirroredRightHandWeaponLocal =
            conjugation.targetHandWeaponLocal;

        /*
         * Position is anchored directly by the actual solver palm seat.
         * Reflect the authored left seat across weapon-local X, clear the
         * orientation solve's translation, then place the right hand from its
         * own palm offset. This avoids subtracting two huge nearly-equal
         * floats and keeps the result weapon-relative and controller-free.
         */
        const RE::NiPoint3 leftPalmWeaponLocal =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(leftHandWeaponLocal, true);
        const RE::NiPoint3 desiredRightPalmWeaponLocal{
            -leftPalmWeaponLocal.x,
            leftPalmWeaponLocal.y,
            leftPalmWeaponLocal.z,
        };
        mirroredRightHandWeaponLocal.translate = {};
        mirroredRightHandWeaponLocal.scale = leftHandWeaponLocal.scale;
        const RE::NiPoint3 rightPalmOffsetWeaponLocal =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(mirroredRightHandWeaponLocal, false);
        if (!std::isfinite(desiredRightPalmWeaponLocal.x) ||
            !std::isfinite(desiredRightPalmWeaponLocal.y) ||
            !std::isfinite(desiredRightPalmWeaponLocal.z) ||
            !std::isfinite(rightPalmOffsetWeaponLocal.x) ||
            !std::isfinite(rightPalmOffsetWeaponLocal.y) ||
            !std::isfinite(rightPalmOffsetWeaponLocal.z)) {
            return false;
        }
        mirroredRightHandWeaponLocal.translate =
            sub(desiredRightPalmWeaponLocal, rightPalmOffsetWeaponLocal);
        if (!isFiniteTransform(mirroredRightHandWeaponLocal)) {
            return false;
        }

        const RE::NiPoint3 anchoredRightPalmWeaponLocal =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(
                mirroredRightHandWeaponLocal,
                false);
        const RE::NiPoint3 anchorError =
            sub(anchoredRightPalmWeaponLocal, desiredRightPalmWeaponLocal);
        constexpr float kMaxPalmAnchorErrorGameUnits = 0.01f;
        if (!std::isfinite(anchorError.x) ||
            !std::isfinite(anchorError.y) ||
            !std::isfinite(anchorError.z) ||
            std::sqrt(dot(anchorError, anchorError)) >
                kMaxPalmAnchorErrorGameUnits) {
            return false;
        }

        outRightHandWeaponLocal = mirroredRightHandWeaponLocal;
        return true;
    }

    bool TwoHandedGrip::tryBuildMirroredLeftFiringHandWeaponLocal(
        const RE::NiTransform& canonicalRightHandWeaponLocal,
        const RE::NiPoint3& firingGripWeaponLocal,
        const RE::NiTransform& rightHandWorld,
        const RE::NiTransform& leftHandWorld,
        RE::NiTransform& outHandWeaponLocal,
        const bool logDiagnostic)
    {
        const auto& handlingSettings =
            equipped_weapon_handling_runtime::current();
        if (!isFiniteTransform(canonicalRightHandWeaponLocal) ||
            !std::isfinite(firingGripWeaponLocal.x) ||
            !std::isfinite(firingGripWeaponLocal.y) ||
            !std::isfinite(firingGripWeaponLocal.z)) {
            return false;
        }

        /*
         * WAND-CONJUGATION MIRROR. The aim requirement is controller-
         * relative: the tuned right-hand offsets align the barrel with the
         * RIGHT controller's forward, so the mirrored hold must align it
         * with the LEFT controller's forward with the lateral components
         * negated ("2 degrees left of the right wand" becomes "2 degrees
         * right of the left wand"). Left/right WAND device frames are the
         * physically exact mirror pair; the hand BONE conventions are not
         * mirrors (previous semantic-palm and bone-anchor mirrors both left
         * a residual yaw/side bias in-game). Conjugating the canonical hold
         * through the wand pair cancels every per-hand bone convention
         * inside the live-sampled bone-in-wand transforms:
         *
         *   weaponInLeftWand = Msag o weaponInRightWand o Mside
         *
         * with two reflections keeping the result a proper rotation: Msag
         * mirrors across the wand's sagittal plane (wand-local X lateral -
         * same axis family as the weapon frame the wand chain parents) and
         * Mside across the weapon's own side plane (+Y barrel, +X side),
         * which maps the grip from the weapon's right flank to its left.
         * Effect on the tuned offsets: yaw and roll negate, pitch and
         * fore/aft/vertical placement are preserved.
         *
         * The native first-person arm sync drags each hand bone to its wand
         * with a fixed per-hand map, so bone-in-wand is constant and
         * sampling it at takeover time is exact.
         */
        auto* playerNodes = f4vr::getPlayerNodes();
        if (!playerNodes) {
            return false;
        }
        // Ambidextrous stands down in game-left-handed mode, so primary is
        // always the physical RIGHT wand here.
        RE::NiNode* rightWand = playerNodes->primaryWandNode;
        RE::NiNode* leftWand = playerNodes->SecondaryWandNode;
        if (!rightWand || !leftWand ||
            !isFiniteTransform(rightWand->world) || !isFiniteTransform(leftWand->world)) {
            return false;
        }

        const RE::NiTransform boneInRightWand =
            transform_math::composeTransforms(transform_math::invertTransform(rightWand->world), rightHandWorld);
        const RE::NiTransform boneInLeftWand =
            transform_math::composeTransforms(transform_math::invertTransform(leftWand->world), leftHandWorld);
        // A hand bone rides its wand at wrist range; a large offset means a
        // stale or foreign frame - fail closed to the live-capture fallback.
        const auto transformOffsetLength = [](const RE::NiTransform& transform) {
            return std::sqrt(dot(transform.translate, transform.translate));
        };
        if (!isFiniteTransform(boneInRightWand) || !isFiniteTransform(boneInLeftWand) ||
            transformOffsetLength(boneInRightWand) >
                two_handed_grip_detail::
                    kMaximumBoneToDriverDistanceGameUnits ||
            transformOffsetLength(boneInLeftWand) >
                two_handed_grip_detail::
                    kMaximumBoneToDriverDistanceGameUnits) {
            return false;
        }

        /*
         * Global left-hold trim, applied on the WAND side of the conjugation
         * (PRE-composed in the LEFT WAND frame), never on the weapon side.
         * The error it corrects is the fixed frame-convention delta between
         * the two wand device frames, which sits to the LEFT of the
         * conjugated hold. A weapon-side (post-composed) trim conjugates
         * through each weapon's own hold and therefore acts along different
         * axes per weapon: only the calibration weapon looked right, and
         * weapons with large authored holds (UMP forward offset, hunting
         * rifle) showed the trim rotated into unrelated directions
         * (2026-07-12 regression). Pre-composing makes one calibration exact
         * for every weapon, and if the mirror is fully correct these trims
         * converge to zero.
         *
         * Axes are the left wand's hand-anatomical basis (user-calibrated
         * in-game): X = palm normal, Y = fingers forward, Z = thumb up.
         * Yaw rotates about Z (thumb), pitch about X (palm normal); if a
         * value moves the aim opposite to its documented direction, the
         * user flips its sign once. The trim is ROTATION-ONLY: position is
         * anchored per weapon below, so a translation here would fight it.
         */
        const float aimYawRadians =
            handlingSettings.leftFiringAimYawDegrees *
            two_handed_grip_detail::kDegreesToRadians;
        const float aimPitchRadians =
            handlingSettings.leftFiringAimPitchDegrees *
            two_handed_grip_detail::kDegreesToRadians;
        const bool trimActive =
            aimYawRadians != 0.0f || aimPitchRadians != 0.0f;
        RE::NiTransform wandTrim{};
        wandTrim.MakeIdentity();
        if (trimActive) {
            RE::NiTransform yawTrim{};
            yawTrim.MakeIdentity();
            if (aimYawRadians != 0.0f) {
                const float yawCos = std::cos(aimYawRadians);
                const float yawSin = std::sin(aimYawRadians);
                // yaw about wand +Z (thumb axis)
                yawTrim.rotate.entry[0][0] = yawCos;
                yawTrim.rotate.entry[0][1] = -yawSin;
                yawTrim.rotate.entry[1][0] = yawSin;
                yawTrim.rotate.entry[1][1] = yawCos;
            }
            RE::NiTransform pitchTrim{};
            pitchTrim.MakeIdentity();
            if (aimPitchRadians != 0.0f) {
                const float pitchCos = std::cos(aimPitchRadians);
                const float pitchSin = std::sin(aimPitchRadians);
                // pitch about wand +X (palm-normal axis)
                pitchTrim.rotate.entry[1][1] = pitchCos;
                pitchTrim.rotate.entry[1][2] = pitchSin;
                pitchTrim.rotate.entry[2][1] = -pitchSin;
                pitchTrim.rotate.entry[2][2] = pitchCos;
            }
            wandTrim =
                transform_math::composeTransforms(yawTrim, pitchTrim);
        }

        MirroredDriverConjugation conjugation{};
        if (!tryBuildMirroredDriverConjugation(
                canonicalRightHandWeaponLocal,
                boneInRightWand,
                boneInLeftWand,
                trimActive ? &wandTrim : nullptr,
                conjugation)) {
            return false;
        }
        const RE::NiTransform& weaponInRightWand =
            conjugation.sourceWeaponInDriver;
        const RE::NiTransform& weaponInLeftWand =
            conjugation.targetWeaponInDriver;
        RE::NiTransform mirroredHandWeaponLocal =
            conjugation.targetHandWeaponLocal;

        /*
         * PALM-ANCHORED POSITION: the wand conjugation is the ORIENTATION
         * authority only. Deriving the translation through frame mirroring
         * left per-weapon height errors that no global knob can fix (UMP
         * too low while the P226 sits too high - weapons with authored
         * FRIK rotations/offsets each landed differently, because any
         * residual rotation-convention error displaces a hold by an amount
         * proportional to that weapon's own offsets). Instead the FIRING
         * GRIP POINT is pinned per weapon: it must sit at the same place in
         * the left palm as it does in the right palm. ROCK's hand bases
         * correspond anatomically with only Z flipped - empirical, from the
         * user-tuned palm pivots R(6.0,-2.0,+0.2) / L(6.0,-2.0,-0.2) - so
         * the target is simply (x, y, -z) of the grip's right-hand-local
         * position, plus the global offset knobs as palm-space nudges.
         * Per-weapon exact by construction; residuals are global-only.
         */
        const RE::NiPoint3 gripInRightHand = transform_math::localPointToWorld(
            transform_math::invertTransform(canonicalRightHandWeaponLocal), firingGripWeaponLocal);
        const RE::NiPoint3 gripTargetInLeftHand{
            gripInRightHand.x + handlingSettings.leftFiringAimOffsetXGameUnits,
            gripInRightHand.y + handlingSettings.leftFiringAimOffsetYGameUnits,
            -gripInRightHand.z + handlingSettings.leftFiringAimOffsetZGameUnits
        };
        if (std::isfinite(gripTargetInLeftHand.x) && std::isfinite(gripTargetInLeftHand.y) && std::isfinite(gripTargetInLeftHand.z)) {
            RE::NiTransform anchoredWeaponInLeftHand = transform_math::invertTransform(mirroredHandWeaponLocal);
            const RE::NiPoint3 gripRotatedOnly = sub(
                transform_math::localPointToWorld(anchoredWeaponInLeftHand, firingGripWeaponLocal),
                anchoredWeaponInLeftHand.translate);
            anchoredWeaponInLeftHand.translate = sub(gripTargetInLeftHand, gripRotatedOnly);
            const RE::NiTransform anchoredHold = transform_math::invertTransform(anchoredWeaponInLeftHand);
            if (isFiniteTransform(anchoredHold)) {
                mirroredHandWeaponLocal = anchoredHold;
            }
        }

        // Takeover-event diagnostic: barrel (+Y weapon) direction in each
        // wand frame. A correct mirror negates x and preserves y/z; a wand
        // axis-convention mismatch shows up here as a different component
        // flipping.
        if (logDiagnostic) {
            const RE::NiPoint3 barrelInRightWand =
                sub(transform_math::localPointToWorld(weaponInRightWand, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }), weaponInRightWand.translate);
            const RE::NiPoint3 barrelInLeftWand =
                sub(transform_math::localPointToWorld(weaponInLeftWand, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }), weaponInLeftWand.translate);
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: wand-conjugated left hold barrelInRightWand=({:.3f},{:.3f},{:.3f}) barrelInLeftWand=({:.3f},{:.3f},{:.3f}) boneWandDist=({:.2f},{:.2f})",
                barrelInRightWand.x,
                barrelInRightWand.y,
                barrelInRightWand.z,
                barrelInLeftWand.x,
                barrelInLeftWand.y,
                barrelInLeftWand.z,
                transformOffsetLength(boneInRightWand),
                transformOffsetLength(boneInLeftWand));
        }

        outHandWeaponLocal = mirroredHandWeaponLocal;
        return true;
    }

    void TwoHandedGrip::setFiringHand(const bool isLeft, const char* reason)
    {
        if (_firingHandIsLeft == isLeft) {
            return;
        }

        clearFiringRecoilPresentationState();
        // Drop the old hand's role-tagged FRIK publications; the new hand's
        // grip-frame capture and pose publication are owned by the caller.
        clearPrimaryGripPose(_firingHandIsLeft);
        clearPrimaryDetachVisualAuthority(_firingHandIsLeft);
        _primaryHandVisualLerp = {};
        _primaryReleaseDebounce = {};
        if (_persistentEquippedCarryActive) {
            _persistentEquippedCarryDetachArmed = false;
        }
        _firingHandIsLeft = isLeft;
        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: firing hand switched to {} reason={}", isLeft ? "left" : "right", reason ? reason : "unknown");
    }

    bool TwoHandedGrip::tryPromoteSupportGripToFiringGrip(RE::NiNode* weaponNode)
    {
        const bool supportHandIsLeft = !_firingHandIsLeft;
        if (!weaponNode || !_handlingSettings.ambidextrousHandoffEnabled ||
            !canBeginPrimaryOnlyGripForHand(supportHandIsLeft)) {
            return false;
        }

        const WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        // Pose selection (provider/authored/dynamic) and current transform
        // authority (visual-only/full) are orthogonal to handoff capability.
        // AttachOnly glue alone cannot inherit the firing grip. The distance
        // gate below keeps every other promotable grip tied to the firing grip.
        if (!weapon_support_authority_policy::canPromoteSupportGripToFiringGrip(
                supportGrip.active,
                supportGrip.attachOnly)) {
            return false;
        }

        /*
         * Promotion distance uses the support GRIP POINT (where the hand
         * actually grabbed the weapon), not the palm pivot: a shooting-cup
         * palm sits a hand-width away from the grip center and the tight
         * reattach radius silently declined every takeover. The dedicated
         * promotion radius keeps handguard/foregrip support grips out.
         */
        const RE::NiPoint3 supportGripWorld = resolvePartGripWorld(supportGrip, weaponNode);
        const RE::NiPoint3 firingGripWorld = weaponLocalToWorld(_primaryGripLocal, weaponNode);
        const RE::NiPoint3 gripDelta = sub(supportGripWorld, firingGripWorld);
        const float supportGripToFiringGripDistance = std::sqrt(dot(gripDelta, gripDelta));
        if (!std::isfinite(supportGripToFiringGripDistance) ||
            supportGripToFiringGripDistance >
                _handlingSettings.firingGripPromotionRadiusGameUnits) {
            return false;
        }

        RE::NiTransform handTransform{};
        if (!tryGetSolverHandTransform(supportHandIsLeft, handTransform)) {
            return false;
        }

        // A left firing hand needs FRIK's right-hand weapon pose blocked for
        // the whole left-firing tenure; abort the promotion if that fails.
        if (supportHandIsLeft && !blockFrikPrimaryWeaponPose()) {
            return false;
        }

        /*
         * Commit: the support hand takes over the SAME weapon-relative firing
         * grip in place, forcing the CANONICAL per-hand hold: a LEFT takeover
         * applies the canonical right-hand hold mirrored (authored offsets
         * adapted to the left bone basis), a RIGHT takeover re-takes its
         * canonical native hold directly - the promoted hand's live bone is
         * still part-grip-locked here, so a live capture froze that locked
         * angle and (for the right) poisoned the canonical snapshot below.
         * The live capture remains only as the no-canonical fallback.
         */
        const auto hold = selectCanonicalFiringHold(
            supportHandIsLeft,
            weaponNode,
            handTransform);

        beginHandVisualReturn(_firingHandIsLeft, "ambidextrous-firing-hand-promotion");
        setFiringHand(supportHandIsLeft, "support-grip-promotion");
        if (!transitionToPrimaryOnly(weaponNode, _activeWeaponGenerationKey, _activeEquippedWeaponOwnershipKey, "firing-grip-hand-promotion")) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: firing-grip promotion failed to enter primary-only; clearing authority");
            transitionToInactive(false);
            return true;
        }

        _primaryHandWeaponLocal = hold.handWeaponLocal;
        _hasFiringHandWeaponLocal = true;
        rememberRightFiringHandCanonicalFrame();
        _firingGripSequence = ++_gripCaptureSequence;
        _primaryHandVisualLerp = {};
        _hapticEvents.firingGripAttached = true;
        _hapticEvents.firingGripAttachedHandIsLeft = _firingHandIsLeft;
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: support hand promoted to firing grip hand={} gripToGrip={:.2f} hold={}",
            _firingHandIsLeft ? "left" : "right",
            supportGripToFiringGripDistance,
            hold.source);
        return true;
    }

    RE::NiNode* TwoHandedGrip::resolveFirstPersonHandNode(const bool isLeft)
    {
        auto* firstPersonSkeleton = f4vr::getFirstPersonSkeleton();
        if (!firstPersonSkeleton) {
            return nullptr;
        }
        return f4vr::findNode(firstPersonSkeleton, isLeft ? "LArm_Hand" : "RArm_Hand");
    }

    void TwoHandedGrip::syncFiringHandWeaponNodeOwnership(RE::NiNode* weaponNode)
    {
        const bool wantLeftFiringCarry = _firingHandIsLeft &&
            (_state == TwoHandedState::Gripping || _state == TwoHandedState::PrimaryOnly);

        if (!wantLeftFiringCarry) {
            releaseFiringHandWeaponNodeOwnership(weaponNode);
            return;
        }

        if (!_weaponNodeOwnershipBlockEngaged) {
            if (!frik_visual_authority::blockPrimaryWeaponNodeOwnership(WEAPON_NODE_OWNERSHIP_TAG, true)) {
                // Fail closed: without the FRIK block the weapon node would
                // fight two per-frame owners.
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: left-firing carry aborted because the FRIK weapon-node ownership block is unavailable");
                transitionToInactive(false);
                return;
            }
            _weaponNodeOwnershipBlockEngaged = true;
            ROCK_LOG_INFO(Weapon, "TwoHandedGrip: FRIK weapon-node ownership blocked for left-firing carry");
        }

        if (!weaponNode) {
            return;
        }

        RE::NiNode* leftHand = resolveFirstPersonHandNode(true);
        if (!leftHand) {
            return;
        }
        if (weaponNode->parent == leftHand) {
            _weaponNodeReparentedToLeftHand = true;
            return;
        }

        /*
         * Re-parent under LArm_Hand preserving world so the scene graph keeps
         * the weapon riding the firing hand at every point in the frame
         * (native fire/aim sampling included). Same operation FRIK performs
         * for the game's own left-handed mode, minus the mirrored offsets.
         */
        const RE::NiTransform worldBefore = weaponNode->world;
        RE::NiTransform localInLeftHand{};
        if (!tryResolveWeaponRootLocal(
                leftHand,
                worldBefore,
                localInLeftHand)) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                2000,
                "TwoHandedGrip: left-firing weapon reparent rejected an invalid target frame");
            return;
        }
        RE::NiPointer<RE::NiAVObject> detached;
        if (weaponNode->parent) {
            weaponNode->parent->DetachChild(weaponNode, detached);
        }
        leftHand->AttachChild(weaponNode, true);
        weaponNode->local = localInLeftHand;
        weaponNode->world = worldBefore;
        _weaponNodeReparentedToLeftHand = true;
        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: equipped weapon node re-parented under LArm_Hand for left-firing carry");
    }

    void TwoHandedGrip::releaseFiringHandWeaponNodeOwnership(RE::NiNode* weaponNode)
    {
        // This function is also the idempotent right-firing topology path, so
        // it may retire only the left-carry recoil state. Right full-two-hand
        // recoil remains live until that grip state itself ends.
        clearFiringRecoilPresentationState(true);
        if (_weaponNodeReparentedToLeftHand) {
            RE::NiNode* node = weaponNode ? weaponNode : _activeWeaponNode;
            RE::NiNode* rightHand = resolveFirstPersonHandNode(false);
            if (node && rightHand && node->parent != rightHand) {
                const RE::NiTransform worldBefore = node->world;
                RE::NiTransform localInRightHand{};
                if (!tryResolveWeaponRootLocal(
                        rightHand,
                        worldBefore,
                        localInRightHand)) {
                    ROCK_LOG_SAMPLE_WARN(
                        Weapon,
                        2000,
                        "TwoHandedGrip: right-hand weapon reparent rejected an invalid target frame");
                } else {
                    RE::NiPointer<RE::NiAVObject> detached;
                    if (node->parent) {
                        node->parent->DetachChild(node, detached);
                    }
                    rightHand->AttachChild(node, true);
                    node->local = localInRightHand;
                    node->world = worldBefore;
                    ROCK_LOG_INFO(Weapon, "TwoHandedGrip: equipped weapon node re-parented back under RArm_Hand");
                }
            }
            _weaponNodeReparentedToLeftHand = false;
        }

        if (_weaponNodeOwnershipBlockEngaged) {
            // FRIK also force-reattaches native weapon-node parenting once the
            // block releases (belt and braces for teardown without nodes).
            (void)frik_visual_authority::blockPrimaryWeaponNodeOwnership(WEAPON_NODE_OWNERSHIP_TAG, false);
            _weaponNodeOwnershipBlockEngaged = false;
            ROCK_LOG_INFO(Weapon, "TwoHandedGrip: FRIK weapon-node ownership restored");
        }
    }

    bool TwoHandedGrip::getSelectedAuthoredGripPoseSnapshot(
        SelectedAuthoredGripPoseSnapshot& outSnapshot) const
    {
        outSnapshot = {};
        const auto generationKey = _activeWeaponGenerationKey != 0 ?
            _activeWeaponGenerationKey :
            _rightFiringHandCanonicalGenerationKey;
        if (generationKey == 0) {
            return false;
        }

        const bool canonicalCurrent =
            _hasRightFiringHandCanonicalWeaponLocal &&
            _rightFiringHandCanonicalGenerationKey == generationKey &&
            (!_activeWeaponNode ||
                _rightFiringHandCanonicalWeaponNode == _activeWeaponNode);
        const bool supportCurrent =
            _authoredSupportGripCandidate.valid &&
            _authoredSupportGripCandidate.weaponGenerationKey == generationKey &&
            (!_activeWeaponNode ||
                _authoredSupportGripCandidate.weaponNode == _activeWeaponNode);
        if (!canonicalCurrent && !supportCurrent) {
            return false;
        }

        outSnapshot.weaponGenerationKey = generationKey;
        if (canonicalCurrent) {
            outSnapshot.rightHandWeaponLocal =
                _rightFiringHandCanonicalWeaponLocal;
            outSnapshot.rightHandValid = true;
            outSnapshot.rightFingerLocalTransforms =
                _rightFiringFingerLocalTransforms;
            outSnapshot.rightFingerLocalTransformMask =
                _rightFiringFingerLocalTransformMask;
            outSnapshot.captureSequence =
                _rightFiringHandCanonicalCaptureSequence;
            outSnapshot.source =
                _rightFiringHandCanonicalSource ==
                        RightFiringCanonicalSource::AuthoredAnimation ?
                    SelectedAuthoredGripPoseSnapshot::Source::NativeIdlePreharvest :
                    SelectedAuthoredGripPoseSnapshot::Source::RuntimeCanonical;
        }

        if (supportCurrent) {
            outSnapshot.leftHandWeaponLocal =
                _authoredSupportGripCandidate.leftHandWeaponLocal;
            outSnapshot.leftHandValid = true;
            outSnapshot.leftFingerLocalTransforms =
                _authoredSupportGripCandidate.leftFingerLocalTransforms;
            outSnapshot.leftFingerLocalTransformMask =
                _authoredSupportGripCandidate.leftFingerLocalTransformMask;
            outSnapshot.captureSequence = (std::max)(
                outSnapshot.captureSequence,
                _authoredSupportGripCandidate.captureSequence);
            if (!canonicalCurrent) {
                outSnapshot.rightHandWeaponLocal =
                    _authoredSupportGripCandidate.rightHandWeaponLocal;
                outSnapshot.rightHandValid =
                    _authoredSupportGripCandidate.rightMirrorValid;
                outSnapshot.rightFingerLocalTransforms =
                    _authoredSupportGripCandidate.rightFingerLocalTransforms;
                outSnapshot.rightFingerLocalTransformMask =
                    _authoredSupportGripCandidate.rightFingerLocalTransformMask;
            }
            outSnapshot.source =
                SelectedAuthoredGripPoseSnapshot::Source::NativeIdlePreharvest;
        } else if (canonicalCurrent) {
            RE::NiTransform leftHandWeaponLocal{};
            if (tryComputeMirroredLeftFiringHandWeaponLocal(
                    leftHandWeaponLocal,
                    nullptr,
                    false)) {
                outSnapshot.leftHandWeaponLocal = leftHandWeaponLocal;
                outSnapshot.leftHandValid = true;
                outSnapshot.leftFingerLocalTransforms =
                    _leftFiringFingerLocalTransforms;
                outSnapshot.leftFingerLocalTransformMask =
                    _leftFiringFingerLocalTransformMask;
            }
        }

        outSnapshot.variantKey = outSnapshot.captureSequence != 0 ?
            outSnapshot.captureSequence :
            generationKey;
        outSnapshot.valid =
            outSnapshot.rightHandValid || outSnapshot.leftHandValid;
        return outSnapshot.valid;
    }

}
