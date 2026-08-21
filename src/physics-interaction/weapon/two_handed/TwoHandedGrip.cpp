#include "physics-interaction/weapon/two_handed/TwoHandedGrip.h"

/*
 * The CORE of TwoHandedGrip: the state machine and the public surface. Every
 * other TwoHandedGrip*.cpp implements one region of the same class. This file
 * owns the orchestration they hang off.
 *
 * update() is the per-frame entry point. It dispatches by state - Inactive,
 * Touching, Gripping, PartCarry, PrimaryOnly - to the region that does the real
 * work. transitionToInactive() is the fail-closed path used from about twenty
 * sites in every sibling TU, so every sibling depends on this file by design.
 *
 * transitionToInactive() and reset() share clearActiveGripState(). The split is
 * deliberate: transitionToInactive ends one GRIP, reset ends the whole
 * presentation SESSION. NOTES_03 records the field-by-field boundary.
 *
 * The constructor and destructor stay here with the FingerPoseSolveScratch
 * unique_ptr member. The complete scratch type comes from
 * TwoHandedGripInternal.h, which every TU in this folder includes.
 */
#include "physics-interaction/weapon/two_handed/TwoHandedGripInternal.h"

#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/collision/WeaponCollision.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>

namespace rock
{
    using two_handed_grip_detail::areTransformsNearlyEqual;
    using two_handed_grip_detail::GRIP_HAND_POSE_PRIORITY;
    using two_handed_grip_detail::isFiniteTransform;
    using two_handed_grip_detail::isUsableHandAuthorityTransform;
    using two_handed_grip_detail::WEAPON_COLLISION_HAND_TAG;
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
        RE::NiNode* recoilWeaponNode = nullptr;
        std::uint64_t recoilWeaponGenerationKey = 0;
        if (!self->tryResolveControlledFiringRecoilSource(
                firingHandIsLeft,
                recoilWeaponNode,
                recoilWeaponGenerationKey) ||
            (!firingHandIsLeft &&
                (!self->_hasFiringRecoilReference[firingHandIndex] ||
                    self->_firingRecoilReferenceGenerationKey[firingHandIndex] !=
                        recoilWeaponGenerationKey))) {
            return false;
        }

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
                recoilWeaponGenerationKey;
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
        updateWeaponCollisionReleaseSuppressions(
            leftWeaponContact,
            rightWeaponContact,
            currentWeaponGenerationKey,
            dt);
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
                armWeaponCollisionReleaseSuppression(
                    supportHandIsLeft,
                    currentWeaponGenerationKey);
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

    void TwoHandedGrip::armWeaponCollisionReleaseSuppression(
        const bool isLeft,
        const std::uint64_t weaponGenerationKey)
    {
        if (weaponGenerationKey == 0) {
            return;
        }

        auto& suppression =
            _weaponCollisionReleaseSuppressions[isLeft ? 0u : 1u];
        suppression = {
            .weaponGenerationKey = weaponGenerationKey,
            .active = true,
        };
        ROCK_LOG_DEBUG(
            Weapon,
            "TwoHandedGrip: dynamic weapon collision release suppression armed hand={} generation={:016X}",
            isLeft ? "left" : "right",
            weaponGenerationKey);
    }

    void TwoHandedGrip::updateWeaponCollisionReleaseSuppressions(
        const WeaponInteractionContact& leftWeaponContact,
        const WeaponInteractionContact& rightWeaponContact,
        const std::uint64_t currentWeaponGenerationKey,
        const float dt)
    {
        const std::array<const WeaponInteractionContact*, 2> contacts{
            &leftWeaponContact,
            &rightWeaponContact,
        };
        const float elapsedSeconds =
            std::isfinite(dt) && dt > 0.0f ?
                std::clamp(dt, 0.0f, 0.1f) :
                (1.0f / 90.0f);

        for (std::size_t handIndex = 0;
             handIndex < _weaponCollisionReleaseSuppressions.size();
             ++handIndex) {
            auto& suppression =
                _weaponCollisionReleaseSuppressions[handIndex];
            if (!suppression.active) {
                continue;
            }
            if (currentWeaponGenerationKey == 0 ||
                suppression.weaponGenerationKey !=
                    currentWeaponGenerationKey) {
                suppression = {};
                continue;
            }

            suppression.elapsedSeconds =
                (std::min)(
                    suppression.elapsedSeconds + elapsedSeconds,
                    60.0f);
            const auto* contact = contacts[handIndex];
            const bool stillInsideWeaponContact =
                contact &&
                contact->valid &&
                contact->weaponGenerationKey ==
                    currentWeaponGenerationKey;
            if (stillInsideWeaponContact) {
                suppression.consecutiveExitFrames = 0;
                continue;
            }
            if (suppression.consecutiveExitFrames <
                WEAPON_COLLISION_RELEASE_EXIT_FRAMES) {
                ++suppression.consecutiveExitFrames;
            }
            if (suppression.elapsedSeconds <
                    WEAPON_COLLISION_RELEASE_MINIMUM_SECONDS ||
                suppression.consecutiveExitFrames <
                    WEAPON_COLLISION_RELEASE_EXIT_FRAMES) {
                continue;
            }

            const float completedSeconds = suppression.elapsedSeconds;
            suppression = {};
            ROCK_LOG_DEBUG(
                Weapon,
                "TwoHandedGrip: dynamic weapon collision release suppression cleared hand={} elapsed={:.3f}s exitFrames={}",
                handIndex == 0u ? "left" : "right",
                completedSeconds,
                WEAPON_COLLISION_RELEASE_EXIT_FRAMES);
        }
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
        _independentWeaponPresentationBeforeFrik = {};
        _postFrikNativeRightWeaponLocal = {};
        _weaponCollisionReleaseSuppressions = {};
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
        _scopeDeferredHandAuthorityClears = {};
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
        _weaponCollisionReleaseSuppressions = {};

        const bool supportHandIsLeft = !_firingHandIsLeft;
        const bool primaryHandIsLeft = _firingHandIsLeft;
        RE::NiTransform nativeWeaponLocalBaseline = weaponNode->local;
        if (_returningWeaponVisual.localTransition.active && _returningWeaponVisual.weaponNode == weaponNode) {
            nativeWeaponLocalBaseline = _returningWeaponVisual.nativeBaselineLocal;
            clearWeaponVisualReturn("new-two-hand-acquisition", true, true);
        } else if (!_firingHandIsLeft) {
            (void)tryGetPostFrikNativeRightWeaponLocal(
                weaponNode,
                decision.weaponGenerationKey,
                nativeWeaponLocalBaseline);
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
        /*
         * Gripping -> PrimaryOnly retains the same firing hand and equipped
         * weapon. Keep the pre-hFRIK recoil reference and an accepted current-
         * frame ticket across that ownership handoff so hFRIK's hand recoil
         * and ROCK's terminal weapon recoil finish as one transaction. The
         * former clear here was correct only while authored right-hand carry
         * had no controlled weapon-recoil owner: after that owner was added it
         * split a two-hand shot at support release and let collision authority
         * retain the hand-only pose. Divergent paths already clear this state
         * in transitionToInactive, transitionToPartCarry, setFiringHand, reload,
         * and lifecycle reset.
         */

        if (_state == TwoHandedState::Inactive) {
            RE::NiTransform nativeWeaponLocalBaseline = weaponNode->local;
            if (_returningWeaponVisual.localTransition.active && _returningWeaponVisual.weaponNode == weaponNode) {
                nativeWeaponLocalBaseline = _returningWeaponVisual.nativeBaselineLocal;
                clearWeaponVisualReturn("new-primary-acquisition", true, true);
            } else if (!_firingHandIsLeft) {
                (void)tryGetPostFrikNativeRightWeaponLocal(
                    weaponNode,
                    currentWeaponGenerationKey,
                    nativeWeaponLocalBaseline);
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

}
