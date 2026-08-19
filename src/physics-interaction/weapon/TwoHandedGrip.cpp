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
        [[nodiscard]] ::rock::provider::
            RockProviderWeaponPartTargetQueryV1
            buildProviderPartTargetQuery(
                const std::uint64_t weaponGenerationKey,
                const std::uint32_t bodyId,
                const std::uint32_t partKind,
                const std::uint32_t reloadRole,
                const std::uint32_t supportRole,
                const std::uint32_t socketRole,
                const std::uint32_t actionRole,
                const std::uintptr_t sourceRoot,
                const std::span<const char> sourceName)
        {
            ::rock::provider::RockProviderWeaponPartTargetQueryV1 query{};
            query.weaponGenerationKey = weaponGenerationKey;
            query.bodyId = bodyId;
            query.partKind = partKind;
            query.reloadRole = reloadRole;
            query.supportRole = supportRole;
            query.socketRole = socketRole;
            query.actionRole = actionRole;
            query.sourceRoot = sourceRoot;
            const std::size_t copyLength = (std::min)(
                sourceName.size(),
                sizeof(query.sourceName) - 1);
            std::memcpy(query.sourceName, sourceName.data(), copyLength);
            query.sourceName[copyLength] = '\0';
            return query;
        }

        [[nodiscard]] WeaponTwoHandedSolverInput<
            RE::NiTransform,
            RE::NiPoint3>
            buildTwoHandedSolverInput(
                const RE::NiTransform& weaponWorld,
                const RE::NiPoint3& primaryGripLocal,
                const RE::NiPoint3& supportGripLocal,
                const RE::NiPoint3& primaryTargetWorld,
                const RE::NiPoint3& supportTargetWorld,
                const RE::NiPoint3& supportNormalLocal,
                const RE::NiPoint3& supportNormalTargetWorld)
        {
            WeaponTwoHandedSolverInput<
                RE::NiTransform,
                RE::NiPoint3>
                input{};
            input.weaponWorldTransform = weaponWorld;
            input.primaryGripLocal = primaryGripLocal;
            input.supportGripLocal = supportGripLocal;
            input.primaryTargetWorld = primaryTargetWorld;
            input.supportTargetWorld = supportTargetWorld;
            input.supportNormalLocal = supportNormalLocal;
            input.supportNormalTargetWorld = supportNormalTargetWorld;
            input.useSupportNormalTwist = true;
            input.supportNormalTwistFactor = SUPPORT_NORMAL_TWIST_FACTOR;
            return input;
        }

        void applyStableWeaponOppositionPose(
            grab_finger_pose_runtime::SolvedGrabFingerPose& pose,
            const grab_pinch_pocket_policy::Config& config,
            const std::size_t opposedFingerIndex)
        {
            const auto stable =
                grab_pinch_pocket_policy::
                    buildStableOppositionFingerPose(
                        config,
                        g_rockConfig.rockGrabFingerMinValue,
                        opposedFingerIndex);
            pose.values = stable.values;
            pose.jointValues = stable.jointValues;
            pose.surfaceAimTarget = {};
            pose.surfaceAimNormal = {};
            pose.surfaceAimTargetValid = {};
            pose.surfaceAimNormalValid = {};
            pose.surfaceAimTargetObjectLocal = {};
            pose.surfaceAimNormalObjectLocal = {};
            pose.surfaceAimTargetObjectLocalValid = {};
            pose.surfaceAimNormalObjectLocalValid = {};
            pose.contactArcRotationRadians = {};
            pose.contactArcRotationValid = {};
            pose.hasObjectLocalSurfaceAim = false;
            pose.usedAlternateThumbCurve = false;
            pose.usedAlternateThumbSurfaceHit = false;
            pose.hasJointValues = true;
            pose.solved = true;
        }

        RE::NiNode* sourceRootNodeOrFallback(RE::NiAVObject* sourceRoot, RE::NiNode* fallback)
        {
            if (sourceRoot) {
                if (auto* sourceNode = sourceRoot->IsNode()) {
                    return sourceNode;
                }
            }
            return fallback;
        }

        // Present cached local evidence to the existing world-space selector.
        struct TransformedSupportGripTriangleView
        {
            std::span<const TriangleData> localTriangles{};
            RE::NiTransform localToWorld{};

            [[nodiscard]] std::size_t size() const noexcept
            {
                return localTriangles.size();
            }

            [[nodiscard]] TriangleData operator[](
                const std::size_t index) const
            {
                const auto& triangle = localTriangles[index];
                return TriangleData{
                    transform_math::localPointToWorld(
                        localToWorld,
                        triangle.v0),
                    transform_math::localPointToWorld(
                        localToWorld,
                        triangle.v1),
                    transform_math::localPointToWorld(
                        localToWorld,
                        triangle.v2),
                };
            }
        };

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

    struct TwoHandedGrip::FingerPoseSolveScratch
    {
        struct HandScratch
        {
            std::array<std::vector<RankedSupportGripTriangle>,
                kSupportGripFingerLaneCount + 1>
                rankings;
            std::vector<TriangleData> localTriangles;
            std::vector<TriangleData> worldTriangles;
            grab_finger_pose_runtime::FingerPoseTriangleSpatialIndex spatialIndex;
        };

        std::array<HandScratch, 2> hands{};
    };

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

    bool TwoHandedGrip::tryCaptureRootFlattenedPalmWorld(bool isLeft, RE::NiPoint3& outPalmWorld, RE::NiTransform& outHandWorld)
    {
        outPalmWorld = {};
        if (!tryGetRootFlattenedHandBoneTransform(isLeft, outHandWorld)) {
            return false;
        }
        outPalmWorld = computeGrabLegacyPalmPivotAWorldFromHandBasis(outHandWorld, isLeft);
        return true;
    }

    RE::NiPoint3 TwoHandedGrip::worldToWeaponLocal(const RE::NiPoint3& worldPos, const RE::NiAVObject* weaponNode)
    {
        if (!weaponNode) {
            return {};
        }
        return weapon_collision_geometry_math::worldPointToLocal(weaponNode->world.rotate, weaponNode->world.translate, weaponNode->world.scale, worldPos);
    }

    RE::NiPoint3 TwoHandedGrip::weaponLocalToWorld(const RE::NiPoint3& localPos, const RE::NiAVObject* weaponNode)
    {
        if (!weaponNode) {
            return {};
        }
        return weapon_collision_geometry_math::localPointToWorld(weaponNode->world.rotate, weaponNode->world.translate, weaponNode->world.scale, localPos);
    }

    RE::NiPoint3 TwoHandedGrip::resolvePartGripWorld(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const
    {
        if (auto* supportAttachmentRoot = resolveCurrentSupportAttachmentRoot(grip, weaponNode)) {
            return transform_math::localPointToWorld(supportAttachmentRoot->world, grip.gripSourceLocal);
        }
        return weaponLocalToWorld(grip.gripLocal, weaponNode);
    }

    RE::NiPoint3 TwoHandedGrip::resolvePartGripWeaponLocal(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const
    {
        return worldToWeaponLocal(resolvePartGripWorld(grip, weaponNode), weaponNode);
    }

    RE::NiPoint3 TwoHandedGrip::resolvePartGripNormalWeaponLocal(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const
    {
        if (auto* supportAttachmentRoot = resolveCurrentSupportAttachmentRoot(grip, weaponNode)) {
            const RE::NiPoint3 supportNormalWorld = transform_math::localVectorToWorld(supportAttachmentRoot->world, grip.normalSourceLocal);
            return transform_math::worldVectorToLocal(weaponNode->world, supportNormalWorld);
        }
        return grip.normalLocal;
    }

    RE::NiTransform TwoHandedGrip::resolvePartGripHandWorld(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const
    {
        if (auto* supportAttachmentRoot = resolveCurrentSupportAttachmentRoot(grip, weaponNode)) {
            return transform_math::composeTransforms(supportAttachmentRoot->world, grip.handSourceLocal);
        }
        if (!weaponNode) {
            return RE::NiTransform{};
        }
        return weapon_support_authority_policy::buildVisualOnlySupportHandWorld(weaponNode->world, grip.handWeaponLocal);
    }

    RE::NiAVObject* TwoHandedGrip::resolveCurrentSupportAttachmentRoot(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const
    {
        if (!grip.hasSourceFrames || !grip.attachmentRoot || !weaponNode) {
            return nullptr;
        }
        return actor_equipment_grab::nodeContainsNode(weaponNode, grip.attachmentRoot, 64) ? grip.attachmentRoot : nullptr;
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

    void TwoHandedGrip::beginDynamicSupportAcquisition(
        const bool supportHandIsLeft,
        const WeaponPartGrip& supportGrip,
        const RE::NiTransform& primaryStartWorld,
        const RE::NiTransform& supportStartWorld)
    {
        if (_dynamicSupportAcquisition.active) {
            clearDynamicSupportAcquisition(
                "replaced-by-new-dynamic-support-grip",
                true);
        }

        _dynamicSupportAcquisition = {};
        _dynamicSupportAcquisition.active = true;
        _dynamicSupportAcquisition.supportHandIsLeft = supportHandIsLeft;
        _dynamicSupportAcquisition.weaponGenerationKey =
            supportGrip.weaponGenerationKey;
        _dynamicSupportAcquisition.gripSequence = supportGrip.gripSequence;
        _dynamicSupportAcquisition.primaryStartWorld = primaryStartWorld;
        _dynamicSupportAcquisition.supportStartWorld = supportStartWorld;
    }

    void TwoHandedGrip::clearDynamicSupportAcquisition(
        const char* reason,
        const bool logCancellation)
    {
        if (!_dynamicSupportAcquisition.active) {
            _dynamicSupportAcquisition = {};
            return;
        }

        if (logCancellation) {
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: dynamic support acquisition event=cancel reason={} hand={} grip={} generation={:016X} elapsed={:.3f}s duration={:.3f}s rawAlpha={:.3f} easedAlpha={:.3f} fullCorrection={:.2f}deg appliedCorrection={:.2f}deg primaryPivotError={:.4f} supportTargetError={:.4f}",
                reason ? reason : "unknown",
                _dynamicSupportAcquisition.supportHandIsLeft ?
                    "left" :
                    "right",
                _dynamicSupportAcquisition.gripSequence,
                _dynamicSupportAcquisition.weaponGenerationKey,
                _dynamicSupportAcquisition.elapsedSeconds,
                _dynamicSupportAcquisition.durationSeconds,
                _dynamicSupportAcquisition.rawAlpha,
                _dynamicSupportAcquisition.easedAlpha,
                _dynamicSupportAcquisition.fullCorrectionRadians *
                    two_handed_grip_detail::kRadiansToDegrees,
                _dynamicSupportAcquisition.lastAppliedRotationRadians *
                    two_handed_grip_detail::kRadiansToDegrees,
                _dynamicSupportAcquisition.lastPrimaryPivotError,
                _dynamicSupportAcquisition.lastSupportTargetError);
        }
        _dynamicSupportAcquisition = {};
    }

    bool TwoHandedGrip::dynamicSupportAcquisitionMatches(
        const bool supportHandIsLeft,
        const WeaponPartGrip& supportGrip) const
    {
        return _dynamicSupportAcquisition.active &&
               _dynamicSupportAcquisition.supportHandIsLeft ==
                   supportHandIsLeft &&
               _dynamicSupportAcquisition.weaponGenerationKey != 0 &&
               _dynamicSupportAcquisition.weaponGenerationKey ==
                   _activeWeaponGenerationKey &&
               _dynamicSupportAcquisition.weaponGenerationKey ==
                   supportGrip.weaponGenerationKey &&
               _dynamicSupportAcquisition.gripSequence != 0 &&
               _dynamicSupportAcquisition.gripSequence ==
                   supportGrip.gripSequence;
    }

    RE::NiTransform
    TwoHandedGrip::resolveDynamicSupportAcquisitionHandTarget(
        const RE::NiTransform& targetWorld,
        const bool primaryHand,
        LockedHandVisualLerpState& visualState)
    {
        const RE::NiTransform& startWorld =
            primaryHand ?
                _dynamicSupportAcquisition.primaryStartWorld :
                _dynamicSupportAcquisition.supportStartWorld;
        visualState.active = true;
        visualState.startWorld = startWorld;
        visualState.elapsedSeconds =
            _dynamicSupportAcquisition.elapsedSeconds;
        visualState.durationSeconds =
            _dynamicSupportAcquisition.durationSeconds;
        visualState.lastAlpha =
            _dynamicSupportAcquisition.easedAlpha;
        return hand_visual_lerp_math::interpolateTransform(
            startWorld,
            targetWorld,
            _dynamicSupportAcquisition.easedAlpha);
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

    bool TwoHandedGrip::capturePartGrip(
        bool isLeft,
        RE::NiNode* weaponNode,
        const WeaponInteractionDecision& decision,
        const WeaponCollision& weaponCollision,
        const WeaponProviderPartAuthority& providerPartAuthority,
        const bool firingGripProximityAuthorityEnabled,
        RE::NiTransform* const outCapturedHandWorld)
    {
        if (outCapturedHandWorld) {
            *outCapturedHandWorld = {};
        }
        if (!weaponNode) {
            return false;
        }
        if (!weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(decision.weaponGenerationKey, _activeWeaponGenerationKey)) {
            ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: part grip capture skipped because contact generation is stale hand={}", isLeft ? "left" : "right");
            return false;
        }

        RE::NiTransform handTransform{};
        if (!tryGetSolverHandTransform(isLeft, handTransform)) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: part grip capture skipped because authoritative hand transforms are unavailable hand={}", isLeft ? "left" : "right");
            return false;
        }
        if (outCapturedHandWorld) {
            *outCapturedHandWorld = handTransform;
        }

        WeaponPartGrip& grip = partGrip(isLeft);
        grip = {};
        RE::NiAVObject* supportAttachmentRoot = decision.sourceRoot ? decision.sourceRoot : static_cast<RE::NiAVObject*>(weaponNode);
        grip.gripPose = decision.gripPose != WeaponGripPoseId::None ? decision.gripPose : WeaponGripPoseId::BarrelWrap;
        grip.partKind = decision.partKind;
        grip.attachmentRoot = supportAttachmentRoot;
        grip.providerPartAuthority = providerPartAuthority.active ? providerPartAuthority : WeaponProviderPartAuthority{};
        grip.attachOnly = weapon_part_grip_report_policy::providerGrabModeIsAttachOnly(
            grip.providerPartAuthority.active,
            grip.providerPartAuthority.grabMode);
        grip.contactBodyId = decision.bodyId;
        grip.reloadRole = decision.reloadRole;
        grip.socketRole = decision.socketRole;
        grip.actionRole = decision.actionRole;
        grip.weaponGenerationKey = decision.weaponGenerationKey;
        grip.gripSequence = ++_gripCaptureSequence;
        {
            // The routing decision carries no support role or authored source
            // name; both come from the evidence descriptor keyed by the
            // contact body, matching the provider target-query construction.
            WeaponCollisionProfileEvidenceDescriptor descriptor{};
            RE::NiAVObject* descriptorSourceNode = nullptr;
            if (weaponCollision.tryGetProfileEvidenceDescriptorForBodyId(decision.bodyId, descriptor, descriptorSourceNode) &&
                descriptor.weaponGenerationKey == decision.weaponGenerationKey) {
                grip.supportRole = descriptor.semantic.supportGripRole;
                grip.omodFormId = descriptor.omodFormId;
                grip.attachPointFormId = descriptor.semantic.attachPointFormId;
                grip.classificationSource = descriptor.semantic.classificationSource;
                const std::size_t copyLength = (std::min)(descriptor.sourceName.size(), grip.sourceName.size() - 1);
                std::memcpy(grip.sourceName.data(), descriptor.sourceName.data(), copyLength);
                grip.sourceName[copyLength] = '\0';
            } else if (grip.providerPartAuthority.active) {
                grip.supportRole = static_cast<WeaponSupportGripRole>(grip.providerPartAuthority.supportRole);
                grip.sourceName = grip.providerPartAuthority.sourceName;
                grip.sourceName[grip.sourceName.size() - 1] = '\0';
            }
        }

        performance_profiler::ScopedTimer fingerPoseCaptureTimer(performance_profiler::Scope::EquippedWeaponFingerPoseCapture);

        const RE::NiPoint3 palmPos = computeGrabLegacyPalmPivotAWorldFromHandBasis(handTransform, isLeft);
        const RE::NiPoint3 palmDir = computePalmNormalFromHandBasis(handTransform, isLeft);

        /*
         * Acquisition-only authored priority. Physical contact and the
         * proximity probe are equivalent entry sources: an eligible authored
         * relation wins over both, but only inside the enforced family cone
         * and radial cap. The captured palm plus at least two distal
         * fingertips must also have current generated-mesh witnesses, so an
         * animation-zero/default support hand cannot escape to an unrelated
         * world-space pose. Every rejected authored candidate continues into
         * the unrestricted dynamic mesh grab below. The final authored seat
         * also selects visual-only versus full weapon authority. Provider
         * AttachOnly remains PAPER/consumer glue. Once selected, the exact
         * hand/weapon relation and 15 finger locals are latched; later
         * candidate changes cannot move it.
         */
        const bool authoredWeaponIdentityMatches =
            _authoredSupportGripCandidate.weaponNode == weaponNode;
        const bool authoredGenerationMatches =
            _authoredSupportGripCandidate.weaponGenerationKey ==
            decision.weaponGenerationKey;
        RE::NiTransform authoredSupportHandWeaponLocal{};
        std::array<RE::NiTransform, 15> authoredSupportFingerLocalTransforms{};
        std::uint16_t authoredSupportFingerLocalTransformMask = 0;
        const bool authoredSupportCandidateForHandValid =
            tryResolveAuthoredSupportGripCandidateForHand(
                isLeft,
                weaponNode,
                decision.weaponGenerationKey,
                authoredSupportHandWeaponLocal,
                authoredSupportFingerLocalTransforms,
                authoredSupportFingerLocalTransformMask);
        AuthoredSupportPalmSeatProximity authoredSupportProximity{};
        RE::NiTransform authoredSupportHandWorld{};
        RE::NiPoint3 authoredSupportPalmWeaponLocal{};
        RE::NiPoint3 authoredSupportPalmNormalWorld{};
        float authoredSupportTouchProbeDistance =
            (std::numeric_limits<float>::infinity)();
        float authoredSupportPalmToFiringGripDistance =
            (std::numeric_limits<float>::infinity)();
        bool authoredSupportFrameValid = false;
        if (authoredSupportCandidateForHandValid &&
            resolveAuthoredSupportPalmSeatProximity(
                weaponNode->world,
                handTransform,
                authoredSupportHandWeaponLocal,
                isLeft,
                authoredSupportProximity)) {
            authoredSupportHandWorld = authoredSupportProximity.authoredHandWorld;
            authoredSupportPalmWeaponLocal =
                authoredSupportProximity.authoredPalmSeatWeaponLocal;
            authoredSupportPalmNormalWorld =
                computePalmNormalFromHandBasis(
                    authoredSupportHandWorld,
                    isLeft);
            authoredSupportTouchProbeDistance =
                authoredSupportProximity.weaponRelativeDistanceGameUnits;
            authoredSupportFrameValid =
                std::isfinite(authoredSupportTouchProbeDistance) &&
                std::isfinite(authoredSupportPalmNormalWorld.x) &&
                std::isfinite(authoredSupportPalmNormalWorld.y) &&
                std::isfinite(authoredSupportPalmNormalWorld.z);
        }

        refreshAuthoredSupportGripActivationState(
            weaponNode,
            decision.weaponGenerationKey,
            weaponCollision,
            true);
        const auto& authoredActivation =
            _authoredSupportGripDebugSnapshot;
        const bool authoredActivationStateMatches =
            authoredActivation.valid &&
            authoredActivation.supportHandIsLeft == isLeft &&
            authoredActivation.weaponGenerationKey ==
                decision.weaponGenerationKey &&
            authoredActivation.captureSequence ==
                _authoredSupportGripCandidate.captureSequence;
        const bool authoredActivationZoneValid =
            authoredActivationStateMatches &&
            authoredActivation.activationSpatialPass;
        const bool authoredPoseSurfaceEvidenceValid =
            authoredActivationStateMatches &&
            authoredActivation.poseEvidencePass;
        const bool authoredSeatWeaponSurfaceValid =
            authoredActivationStateMatches &&
            (authoredActivation.poseSurfaceWitnessMask & 0x01u) != 0;
        const float authoredSupportSurfaceDistance =
            authoredSeatWeaponSurfaceValid ?
            authoredActivation.poseSurfaceDistanceGameUnits[0] :
            (std::numeric_limits<float>::infinity)();

        bool authoredSupportAuthorityGateValid =
            !firingGripProximityAuthorityEnabled;
        auto authoredSupportAuthorityMode =
            weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
        if (authoredSupportFrameValid &&
            firingGripProximityAuthorityEnabled &&
            std::isfinite(weaponNode->world.scale)) {
            const RE::NiPoint3 authoredSeatToFiringGripLocal =
                sub(authoredSupportPalmWeaponLocal, _primaryGripLocal);
            const float authoredSeatToFiringGripLocalDistance = std::sqrt(
                dot(authoredSeatToFiringGripLocal,
                    authoredSeatToFiringGripLocal));
            authoredSupportPalmToFiringGripDistance =
                authoredSeatToFiringGripLocalDistance *
                std::abs(weaponNode->world.scale);
            if (std::isfinite(authoredSupportPalmToFiringGripDistance)) {
                authoredSupportAuthorityMode =
                    weapon_support_authority_policy::resolveFiringGripProximityAuthorityMode(
                        authoredSupportPalmToFiringGripDistance,
                        _handlingSettings.firingGripProximitySupportRadiusGameUnits);
                authoredSupportAuthorityGateValid = true;
            }
        }

        const bool authoredInteractionAcquisitionValid =
            decision.acquisitionSource ==
                WeaponInteractionAcquisitionSource::PhysicalContact ||
            decision.acquisitionSource ==
                WeaponInteractionAcquisitionSource::ProximityProbe;

        const bool useAuthoredSupportGrip =
            authored_weapon_grip_capture_policy::shouldUseAuthoredSupportGrip(
                authored_weapon_grip_capture_policy::AuthoredSupportGripCandidateInput{
                    .interactionAcquisitionValid =
                        authoredInteractionAcquisitionValid,
                    .activationZoneValid = authoredActivationZoneValid,
                    .authoredPoseSurfaceEvidenceValid =
                        authoredPoseSurfaceEvidenceValid,
                    .providerAuthorityActive = providerPartAuthority.active,
                    .attachOnly = grip.attachOnly,
                    .captureValid =
                        authoredSupportCandidateForHandValid &&
                        authoredSupportFrameValid &&
                        authoredSupportAuthorityGateValid,
                    .weaponIdentityMatches = authoredWeaponIdentityMatches,
                    .generationMatches = authoredGenerationMatches,
                    .authoredSeatWeaponSurfaceValid =
                        authoredSeatWeaponSurfaceValid,
                    .completeFingerPose =
                        authoredSupportFingerLocalTransformMask ==
                        authored_weapon_grip_library::
                            kCompleteFiringFingerMask,
                });
        if (useAuthoredSupportGrip) {
            if (firingGripProximityAuthorityEnabled) {
                _authorityMode = authoredSupportAuthorityMode;
            }
            grip.authoredSupportGrip = true;
            grip.authoredSupportCaptureSequence =
                _authoredSupportGripCandidate.captureSequence;
            grip.attachmentRoot = weaponNode;
            grip.gripLocal = authoredSupportPalmWeaponLocal;
            grip.grabNormalWorld = authoredSupportPalmNormalWorld;
            grip.normalLocal = transform_math::worldVectorToLocal(
                weaponNode->world,
                authoredSupportPalmNormalWorld);
            grip.handWeaponLocal =
                authoredSupportHandWeaponLocal;
            grip.hasHandWeaponLocal = true;
            grip.hasSourceFrames = false;
            grip.hasAttachmentWeaponLocal = false;

            // hFRIK requires the role-tagged numeric pose to exist before the
            // exact per-joint local override can win at the same priority.
            setSupportGripPose(isLeft, nullptr, nullptr);
            grip.fingerLocalTransforms =
                authoredSupportFingerLocalTransforms;
            grip.fingerLocalTransformMask =
                authoredSupportFingerLocalTransformMask;
            grip.hasFingerLocalTransforms = true;
            grip.visualLerp = {};
            grip.active = true;

            performance_profiler::observeValue(
                performance_profiler::ValueMetric::EquippedWeaponFingerPoseSourceTriangles,
                0);
            performance_profiler::observeValue(
                performance_profiler::ValueMetric::EquippedWeaponFingerPoseSelectedTriangles,
                0);
            if (isLeft) {
                _hapticEvents.leftPartGripCaptured = true;
            } else {
                _hapticEvents.rightPartGripCaptured = true;
            }

            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: authored support grip captured hand={} weapon='{}' gripLocal=({:.3f},{:.3f},{:.3f}) touchToSeat={:.3f} radialCap={:.3f} surfaceDistance={:.3f} poseWitnesses={}/6 poseMask={:02X} leftDot={:.3f} downDot={:.3f} cone={} authoredSeatToFiringGrip={:.3f} seatLocal=({:.3f},{:.3f},{:.3f}) touchLocal=({:.3f},{:.3f},{:.3f}) frameError={:.4f} capture={} generation={:016X} acquisition={} authority={} priority=provider>authored>dynamic",
                isLeft ? "left" : "right",
                weaponNode->name.c_str(),
                grip.gripLocal.x,
                grip.gripLocal.y,
                grip.gripLocal.z,
                authoredSupportTouchProbeDistance,
                authoredActivation.radialCapGameUnits,
                authoredSupportSurfaceDistance,
                static_cast<unsigned>(
                    authoredActivation.poseSurfaceWitnessCount),
                static_cast<unsigned>(
                    authoredActivation.poseSurfaceWitnessMask),
                authoredActivation.leftDot,
                authoredActivation.downDot,
                authored_weapon_grip_activation_policy::allowedConeName(
                    authoredActivation.selectedCone),
                authoredSupportPalmToFiringGripDistance,
                authoredSupportPalmWeaponLocal.x,
                authoredSupportPalmWeaponLocal.y,
                authoredSupportPalmWeaponLocal.z,
                authoredSupportProximity.liveTouchProbeWeaponLocal.x,
                authoredSupportProximity.liveTouchProbeWeaponLocal.y,
                authoredSupportProximity.liveTouchProbeWeaponLocal.z,
                authoredSupportProximity.frameAgreementErrorGameUnits,
                grip.authoredSupportCaptureSequence,
                _activeWeaponGenerationKey,
                decision.acquisitionSource ==
                        WeaponInteractionAcquisitionSource::PhysicalContact ?
                    "physical-contact" :
                    "probe",
                _authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport ?
                    "visual-only" :
                    "full");
            return true;
        }

        if (authoredSupportCandidateForHandValid) {
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: authored support grip rejected; continuing to dynamic hand={} source={} family={} activation={} pose={} palm={} witnesses={}/6 mask={:02X} distance={:.3f} cap={:.3f} leftDot={:.3f} downDot={:.3f} class={} radial={} direction={} scope={} provider={} attachOnly={} capture={} identity={} generation={} fingers={}",
                isLeft ? "left" : "right",
                decision.acquisitionSource ==
                        WeaponInteractionAcquisitionSource::PhysicalContact ?
                    "physical-contact" :
                    (decision.acquisitionSource ==
                            WeaponInteractionAcquisitionSource::ProximityProbe ?
                        "probe" : "none"),
                authored_weapon_grip_activation_policy::weaponFamilyName(
                    authoredActivation.weaponFamily),
                authoredActivationZoneValid ? "pass" : "fail",
                authoredPoseSurfaceEvidenceValid ? "pass" : "fail",
                authoredSeatWeaponSurfaceValid ? "pass" : "fail",
                static_cast<unsigned>(
                    authoredActivation.poseSurfaceWitnessCount),
                static_cast<unsigned>(
                    authoredActivation.poseSurfaceWitnessMask),
                authoredActivation.weaponRelativeDistanceGameUnits,
                authoredActivation.radialCapGameUnits,
                authoredActivation.leftDot,
                authoredActivation.downDot,
                authoredActivation.classifierSupported ? "pass" : "fail",
                authoredActivation.radialPass ? "pass" : "fail",
                authoredActivation.directionPass ? "pass" : "fail",
                authoredActivation.scopePass ? "pass" : "fail",
                providerPartAuthority.active ? "yes" : "no",
                grip.attachOnly ? "yes" : "no",
                authoredSupportFrameValid &&
                        authoredSupportAuthorityGateValid ?
                    "pass" : "fail",
                authoredWeaponIdentityMatches ? "pass" : "fail",
                authoredGenerationMatches ? "pass" : "fail",
                authoredSupportFingerLocalTransformMask ==
                        authored_weapon_grip_library::
                            kCompleteFiringFingerMask ?
                    "pass" : "fail");
        }

        auto& fingerScratch = _fingerPoseSolveScratch->hands[isLeft ? 0u : 1u];
        for (auto& ranking : fingerScratch.rankings) {
            ranking.clear();
        }
        fingerScratch.localTriangles.clear();
        fingerScratch.worldTriangles.clear();
        fingerScratch.spatialIndex.clear();

        WeaponCollision::SupportGripEvidenceView evidenceView{};
        const bool cachedTrianglesFound = weaponCollision.tryGetSupportGripEvidenceView(decision.bodyId, weaponNode, evidenceView) &&
            evidenceView.weaponGenerationKey == decision.weaponGenerationKey &&
            evidenceView.weaponGenerationKey == _activeWeaponGenerationKey;
        const std::size_t contactedSourceTriangleCount =
            cachedTrianglesFound ?
            evidenceView.localTriangles.size() :
            0u;

        GrabPoint grabPoint{};
        bool meshFound = false;
        if (cachedTrianglesFound) {
            const TransformedSupportGripTriangleView worldEvidence{
                .localTriangles = evidenceView.localTriangles,
                .localToWorld = evidenceView.localToWorld,
            };
            meshFound = findClosestGrabPoint(
                worldEvidence,
                palmPos,
                palmDir,
                g_rockConfig.rockGrabLateralWeight,
                g_rockConfig.rockGrabDirectionalWeight,
                grabPoint,
                g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits);
        }

        if (meshFound) {
            grip.gripLocal = worldToWeaponLocal(grabPoint.position, weaponNode);
            grip.grabNormalWorld = grabPoint.normal;
        } else {
            grip.gripLocal = worldToWeaponLocal(palmPos, weaponNode);
            grip.grabNormalWorld = palmDir;
        }
        const RE::NiPoint3 gripWorldPoint = meshFound ? grabPoint.position : palmPos;
        const float surfaceSeatMaxRadians =
            g_rockConfig.rockWeaponSupportSurfaceSeatEnabled && meshFound ?
            g_rockConfig.rockWeaponSupportSurfaceSeatMaxDegrees *
                two_handed_grip_detail::kDegreesToRadians :
            0.0f;
        const auto surfaceSeat =
            weapon_support_acquisition_math::
                alignHandFrameToGripSurface<
                    RE::NiTransform,
                    RE::NiPoint3>(
                    handTransform,
                    palmPos,
                    palmDir,
                    gripWorldPoint,
                    grip.grabNormalWorld,
                    surfaceSeatMaxRadians);
        const RE::NiTransform adjustedHandTransform =
            surfaceSeat.valid ?
            surfaceSeat.handWorld :
            weapon_two_handed_grip_math::alignHandFrameToGripPoint(
                handTransform,
                palmPos,
                gripWorldPoint);
        grip.surfaceSeatRotationRadians =
            surfaceSeat.valid ?
            surfaceSeat.appliedRotationRadians :
            0.0f;
        const RE::NiPoint3 seatedPalmNormal =
            computePalmNormalFromHandBasis(
                adjustedHandTransform,
                isLeft);
        grip.handWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), adjustedHandTransform);
        grip.hasHandWeaponLocal = true;
        grip.normalLocal = transform_math::worldVectorToLocal(
            weaponNode->world,
            seatedPalmNormal);
        if (supportAttachmentRoot) {
            grip.gripSourceLocal = transform_math::worldPointToLocal(supportAttachmentRoot->world, gripWorldPoint);
            grip.normalSourceLocal = transform_math::worldVectorToLocal(
                supportAttachmentRoot->world,
                seatedPalmNormal);
            grip.handSourceLocal = transform_math::composeTransforms(transform_math::invertTransform(supportAttachmentRoot->world), adjustedHandTransform);
            grip.attachmentWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), supportAttachmentRoot->world);
            grip.hasSourceFrames = true;
            grip.hasAttachmentWeaponLocal = true;
        }

        /*
         * Capture the root-flattened fingers once for this transaction. The
         * compact sweep snapshot and any exact-local thumb/surface correction
         * must describe the same pre-authority hand, not two scene reads split
         * by grip publication.
         */
        DirectSkeletonBoneSnapshot capturedFingerBoneSnapshot{};
        const bool capturedFingerBoneSnapshotValid =
            rootFlattenedTwoHandedReader().capture(
                skeleton_bone_debug_math::DebugSkeletonBoneMode::
                    HandsAndForearmsOnly,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::
                    GameRootFlattenedBoneTree,
                capturedFingerBoneSnapshot);
        root_flattened_finger_skeleton_runtime::Snapshot
            capturedFingerSnapshot{};
        const bool capturedFingerSnapshotValid =
            capturedFingerBoneSnapshotValid &&
            root_flattened_finger_skeleton_runtime::
                buildFingerSkeletonSnapshot(
                    capturedFingerBoneSnapshot,
                    isLeft,
                    capturedFingerSnapshot);
        SupportGripFingerReferenceSet fingerReferenceSet{};
        fingerReferenceSet.seatPointWorld = gripWorldPoint;
        fingerReferenceSet.seatPointValid =
            grab_finger_pose_runtime::isFinitePoint(gripWorldPoint);
        if (capturedFingerSnapshotValid) {
            const RE::NiTransform rawToSeatedWorld =
                transform_math::composeTransforms(
                    adjustedHandTransform,
                    transform_math::invertTransform(handTransform));
            const auto liveLandmarks =
                root_flattened_finger_skeleton_runtime::
                    buildLandmarkSet(capturedFingerSnapshot);
            std::array<RE::NiPoint3,
                kSupportGripFingerLaneCount>
                commandedOpenDirectionsWorld{};
            const bool commandedDirectionsValid =
                grab_finger_pose_runtime::
                    resolveCommandedOpenDirectionsWorld(
                        isLeft,
                        adjustedHandTransform,
                        commandedOpenDirectionsWorld);
            const RE::NiPoint3 seatedSweepNormal =
                liveLandmarks.valid ?
                transform_math::localVectorToWorld(
                    rawToSeatedWorld,
                    liveLandmarks.palmNormalWorld) :
                RE::NiPoint3{};
            const auto appendLanePoint = [&fingerReferenceSet](
                                             const std::size_t lane,
                                             const RE::NiPoint3& pointWorld) {
                if (lane >= kSupportGripFingerLaneCount ||
                    !grab_finger_pose_runtime::isFinitePoint(
                        pointWorld)) {
                    return;
                }
                auto& count =
                    fingerReferenceSet.lanePointCounts[lane];
                if (count >=
                    kSupportGripFingerLaneReferenceCapacity) {
                    return;
                }
                fingerReferenceSet.lanePointsWorld[lane][count++] =
                    pointWorld;
            };

            for (std::size_t finger = 0;
                 finger < capturedFingerSnapshot.fingers.size();
                 ++finger) {
                const auto& chain =
                    capturedFingerSnapshot.fingers[finger];
                if (!chain.valid) {
                    continue;
                }
                for (const auto& pointWorld : chain.points) {
                    appendLanePoint(
                        finger,
                        transform_math::localPointToWorld(
                            rawToSeatedWorld,
                            pointWorld));
                }

                if (!liveLandmarks.valid ||
                    !commandedDirectionsValid ||
                    finger >= liveLandmarks.fingers.size() ||
                    !liveLandmarks.fingers[finger].valid ||
                    !std::isfinite(
                        liveLandmarks.fingers[finger].length) ||
                    liveLandmarks.fingers[finger].length <=
                        0.0001f) {
                    continue;
                }
                const RE::NiPoint3 seatedBase =
                    transform_math::localPointToWorld(
                        rawToSeatedWorld,
                        liveLandmarks.fingers[finger].base);
                const auto sweepCurve =
                    grab_finger_pose_math::
                        makeBakedCalibratedFingerCurve<
                            RE::NiPoint3>(
                            finger,
                            isLeft,
                            capturedFingerSnapshot.inPowerArmor,
                            seatedBase,
                            seatedSweepNormal,
                            commandedOpenDirectionsWorld[finger],
                            liveLandmarks.fingers[finger].length);
                const auto* tipProbe =
                    sweepCurve.probeCount > 0 ?
                    &sweepCurve.probes[0] :
                    nullptr;
                if (!tipProbe || tipProbe->sampleCount == 0 ||
                    tipProbe->sampleCount >
                        tipProbe->samples.size()) {
                    continue;
                }
                constexpr std::size_t kSweepSamples = 7;
                const RE::NiPoint3 curveNormal =
                    grab_finger_pose_runtime::normalizedOrFallback(
                        sweepCurve.normal,
                        seatedSweepNormal);
                const RE::NiPoint3 curveZero =
                    grab_finger_pose_runtime::normalizedOrFallback(
                        sweepCurve.zeroAngleVector,
                        commandedOpenDirectionsWorld[finger]);
                for (std::size_t sample = 0;
                     sample < kSweepSamples;
                     ++sample) {
                    const std::size_t row =
                        sample * (tipProbe->sampleCount - 1) /
                        (kSweepSamples - 1);
                    const auto& baked = tipProbe->samples[row];
                    const RE::NiPoint3 arm =
                        grab_finger_pose_math::rotateAroundUnitAxis(
                            curveZero,
                            curveNormal,
                            baked.angleRadians);
                    appendLanePoint(
                        finger,
                        grab_finger_pose_math::add(
                            sweepCurve.center,
                            grab_finger_pose_math::scale(
                                arm,
                                baked.reachLength)));
                }
            }
        }

        std::array<WeaponCollision::SupportGripEvidenceView,
            MAX_WEAPON_COLLISION_BODIES>
            compositeEvidenceViews{};
        std::size_t compositeEvidenceViewCount = 0;
        std::size_t sourceTriangleCount = 0;
        if (g_rockConfig.rockGrabMeshFingerPoseEnabled) {
            const std::size_t discoveredViewCount =
                weaponCollision.findSupportGripEvidenceViews(
                    weaponNode,
                    compositeEvidenceViews);
            for (std::size_t index = 0;
                 index < discoveredViewCount;
                 ++index) {
                const auto& candidateView =
                    compositeEvidenceViews[index];
                if (candidateView.weaponGenerationKey !=
                        decision.weaponGenerationKey ||
                    candidateView.weaponGenerationKey !=
                        _activeWeaponGenerationKey ||
                    candidateView.localTriangles.empty()) {
                    continue;
                }
                if (compositeEvidenceViewCount != index) {
                    compositeEvidenceViews[
                        compositeEvidenceViewCount] = candidateView;
                }
                sourceTriangleCount +=
                    candidateView.localTriangles.size();
                ++compositeEvidenceViewCount;
            }
            if (compositeEvidenceViewCount == 0 &&
                cachedTrianglesFound) {
                compositeEvidenceViews[0] = evidenceView;
                compositeEvidenceViewCount = 1;
                sourceTriangleCount =
                    evidenceView.localTriangles.size();
            }
            selectNearestSupportGripFingerTriangles(
                std::span<const WeaponCollision::SupportGripEvidenceView>(
                    compositeEvidenceViews.data(),
                    compositeEvidenceViewCount),
                weaponNode->world,
                fingerReferenceSet,
                grab_finger_pose_runtime::
                    kMaxFingerPoseCandidateTriangles,
                fingerScratch.rankings,
                fingerScratch.localTriangles);
        }
        performance_profiler::observeValue(
            performance_profiler::ValueMetric::
                EquippedWeaponFingerPoseSourceTriangles,
            static_cast<std::uint64_t>(sourceTriangleCount));
        performance_profiler::observeValue(
            performance_profiler::ValueMetric::
                EquippedWeaponFingerPoseSelectedTriangles,
            static_cast<std::uint64_t>(
                fingerScratch.localTriangles.size()));

        grab_finger_pose_runtime::SolvedGrabFingerPose meshFingerPose{};
        const grab_finger_pose_runtime::SolvedGrabFingerPose* meshFingerPosePtr = nullptr;
        std::array<float, 5> capturedFingerSplayRadians{};
        const std::array<float, 5>* capturedFingerSplayRadiansPtr = nullptr;
        bool spatialIndexBuilt = false;
        bool commandedOpenDirectionsValid = false;
        if (g_rockConfig.rockGrabMeshFingerPoseEnabled && !fingerScratch.localTriangles.empty()) {
            const RE::NiTransform seatedToRawWorld =
                transform_math::composeTransforms(
                    handTransform,
                    transform_math::invertTransform(
                        adjustedHandTransform));
            const RE::NiTransform frozenMeshWorld = weapon_two_handed_grip_math::virtualizeMeshForSeatedHand(
                weaponNode->world,
                handTransform,
                adjustedHandTransform);
            const RE::NiPoint3 frozenGripPoint = weapon_two_handed_grip_math::virtualizeWorldPointForSeatedHand(
                gripWorldPoint,
                handTransform,
                adjustedHandTransform);
            const RE::NiPoint3 frozenGripNormal =
                transform_math::localVectorToWorld(
                    seatedToRawWorld,
                    grip.grabNormalWorld);
            auto fingerPoseTargets = grab_finger_pose_runtime::makeSharedGripPoseTarget(frozenGripPoint, frozenGripNormal);
            fingerPoseTargets.useSeatPointForMissingTargets = false;
            fingerPoseTargets.useWholeMeshForMissingTargets = true;
            /*
             * Equipped support keeps the indexed frozen base, but owns its
             * presentation policy. Loose-grab thumb/index clearing and generic
             * pad-probe refinement erased useful weapon-surface opposition.
             */
            auto frozenSolve =
                grab_finger_pose_runtime::solveFrozenMeshFingerPoseBase(
                fingerScratch.localTriangles,
                frozenMeshWorld,
                handTransform,
                isLeft,
                frozenGripPoint,
                fingerPoseTargets,
                fingerScratch.spatialIndex,
                fingerScratch.worldTriangles,
                grab_finger_pose_runtime::FrozenMeshFingerPoseSolveOptions{
                    .minValue = g_rockConfig.rockGrabFingerMinValue,
                    .maxTriangleDistanceSquared = g_rockConfig.rockGrabMaxTriangleDistance,
                    .rejectBacksideHits = g_rockConfig.rockGrabFingerRejectBacksideHits,
                    .surfacePlaneToleranceGameUnits = g_rockConfig.rockGrabFingerSurfacePlaneToleranceGameUnits,
                    .allowSurfaceAimTargets = true,
                    .sweepContactRadiusGameUnits = g_rockConfig.rockGrabFingerSweepContactRadiusGameUnits,
                    .thumbSweepMaxOpenValue = g_rockConfig.rockGrabThumbSweepMaxOpenValue,
                    .fingerSweepMaxOpenValue = g_rockConfig.rockGrabFingerSweepMaxOpenValue,
                    .meshFingerPoseEnabled = g_rockConfig.rockGrabMeshFingerPoseEnabled,
                    .captureSweepDebug = false,
                },
                capturedFingerSnapshotValid ?
                    &capturedFingerSnapshot :
                    nullptr);
            grab_finger_pose_runtime::captureSurfaceAimObjectLocal(
                frozenSolve.pose,
                frozenMeshWorld);
            spatialIndexBuilt = frozenSolve.spatialIndexBuilt;
            commandedOpenDirectionsValid = frozenSolve.commandedOpenDirectionsValid;
            meshFingerPose = frozenSolve.pose;
            performance_profiler::observeValue(
                performance_profiler::ValueMetric::EquippedWeaponFingerPoseSpatialNodeVisits,
                meshFingerPose.spatialNodeVisitCount);
            performance_profiler::observeValue(
                performance_profiler::ValueMetric::EquippedWeaponFingerPoseTriangleTests,
                meshFingerPose.spatialTriangleTestCount);
            if (meshFingerPose.solved) {
                const bool completeDirectFingerEvidence =
                    grab_finger_pose_runtime::
                        hasCompleteFingerContactEvidence(
                            meshFingerPose);
                const auto oppositionConfig =
                    currentWeaponOppositionPocketConfig();
                const auto oppositionPocket =
                    !completeDirectFingerEvidence &&
                            oppositionConfig.enabled &&
                            frozenSolve.liveFingerSnapshotValid ?
                        grab_finger_pose_runtime::
                            findLocalOppositionPocketEvidence(
                                fingerScratch.worldTriangles,
                                frozenSolve.liveFingerSnapshot,
                                frozenGripPoint,
                                meshFingerPose.contactValidMask,
                                oppositionConfig.
                                    minFingerGapGameUnits,
                                (std::max)(
                                    oppositionConfig.
                                        maxFingerGapGameUnits,
                                    WEAPON_OPPOSITION_MAX_FINGER_GAP_GAME_UNITS),
                                oppositionConfig.
                                    maxPocketDistanceGameUnits,
                                (std::min)(
                                    WEAPON_OPPOSITION_SEGMENT_PROBE_RADIUS_GAME_UNITS,
                                    (std::max)(
                                        0.0f,
                                        g_rockConfig.
                                            rockGrabFingerSweepContactRadiusGameUnits))) :
                        grab_finger_pose_runtime::
                            OppositionPocketEvidence{};
                if (oppositionPocket.valid) {
                    applyStableWeaponOppositionPose(
                        meshFingerPose,
                        oppositionConfig,
                        oppositionPocket.opposedFingerIndex);
                    ROCK_LOG_INFO(
                        Weapon,
                        "TwoHandedGrip: local opposition pocket accepted hand={} kind={} directMask=0x{:02X} endpointMask=0x{:02X} directEndpoints=0x{:02X} gap={:.3f} gripSurfaceDistance={:.3f}",
                        isLeft ? "left" : "right",
                        grab_finger_pose_runtime::
                            oppositionPocketKindName(
                                oppositionPocket.kind),
                        static_cast<unsigned>(
                            meshFingerPose.contactValidMask),
                        static_cast<unsigned>(
                            oppositionPocket.endpointMask),
                        static_cast<unsigned>(
                            oppositionPocket.directEndpointMask),
                        oppositionPocket.fingerGapGameUnits,
                        oppositionPocket.
                            gripToSurfaceDistanceGameUnits);
                }
                const bool completeFingerEvidence =
                    completeDirectFingerEvidence ||
                    oppositionPocket.valid;
                if (completeFingerEvidence) {
                    meshFingerPosePtr = &meshFingerPose;
                } else {
                    ROCK_LOG_INFO(
                        Weapon,
                        "TwoHandedGrip: mesh finger pose failed closed hand={} contactMask=0x{:02X} requiredMask=0x{:02X} hits={} sources={} sourceTriangles={} candidateTriangles={}",
                        isLeft ? "left" : "right",
                        static_cast<unsigned>(
                            meshFingerPose.contactValidMask),
                        static_cast<unsigned>(
                            grab_finger_pose_runtime::
                                kCompleteFingerContactMask),
                        meshFingerPose.hitCount,
                        compositeEvidenceViewCount,
                        sourceTriangleCount,
                        meshFingerPose.candidateTriangleCount);
                }
                if (completeDirectFingerEvidence &&
                    frozenSolve.liveFingerSnapshotValid &&
                    grab_finger_pose_runtime::buildSurfaceContactSplayValues(
                        meshFingerPose,
                        frozenSolve.liveFingerSnapshot,
                        capturedFingerSplayRadians)) {
                    capturedFingerSplayRadiansPtr = &capturedFingerSplayRadians;
                }
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: mesh finger pose hand={} values=({:.2f},{:.2f},{:.2f},{:.2f},{:.2f}) hits={} contactMask=0x{:02X} sources={} sourceTris={} candidateTris={} spatial={} nodes={} tests={} commandedAnchors={} altThumb={} thumbLane={}",
                    isLeft ? "left" : "right",
                    meshFingerPose.values[0],
                    meshFingerPose.values[1],
                    meshFingerPose.values[2],
                    meshFingerPose.values[3],
                    meshFingerPose.values[4],
                    meshFingerPose.hitCount,
                    static_cast<unsigned>(
                        meshFingerPose.contactValidMask),
                    compositeEvidenceViewCount,
                    sourceTriangleCount,
                    meshFingerPose.candidateTriangleCount,
                    spatialIndexBuilt ? "yes" : "no",
                    meshFingerPose.spatialNodeVisitCount,
                    meshFingerPose.spatialTriangleTestCount,
                    commandedOpenDirectionsValid ? "yes" : "no",
                    meshFingerPose.usedAlternateThumbCurve ? "yes" : "no",
                    grab_finger_pose_math::thumbLaneName(meshFingerPose.selectedThumbLane));
                if (meshFingerPose.hasThumbCurveDiagnostics) {
                    ROCK_LOG_DEBUG(Weapon,
                        "TwoHandedGrip: thumb curve primary(hit={} value={:.2f} behind={}) opposition(hit={} value={:.2f} behind={}) sidePad(hit={} value={:.2f} behind={}) selected={}",
                        meshFingerPose.thumbPrimaryCurve.hit ? "yes" : "no",
                        meshFingerPose.thumbPrimaryCurve.value,
                        meshFingerPose.thumbPrimaryCurve.openedByBehindContact ? "yes" : "no",
                        meshFingerPose.thumbAlternateCurve.hit ? "yes" : "no",
                        meshFingerPose.thumbAlternateCurve.value,
                        meshFingerPose.thumbAlternateCurve.openedByBehindContact ? "yes" : "no",
                        meshFingerPose.thumbSidePadCurve.hit ? "yes" : "no",
                        meshFingerPose.thumbSidePadCurve.value,
                        meshFingerPose.thumbSidePadCurve.openedByBehindContact ? "yes" : "no",
                        grab_finger_pose_math::thumbLaneName(meshFingerPose.selectedThumbLane));
                }
            }
        }

        setSupportGripPose(
            isLeft,
            meshFingerPosePtr,
            capturedFingerSplayRadiansPtr,
            SupportGripPoseFallback::FullyClosed);
        if (meshFingerPosePtr && grip.hasFingerPose) {
            std::array<RE::NiTransform, 15> localTransforms{};
            std::uint16_t localTransformMask = 0;
            const auto handPose = grip.hasFingerSplay ?
                frik_visual_authority::makeHandPoseDataFromJointValues(grip.fingerPose, grip.fingerSplayRadians) :
                frik_visual_authority::makeHandPoseDataFromJointValues(grip.fingerPose);
            if (buildFullHandLocalTransformsForMeshPose(
                    isLeft,
                    *meshFingerPosePtr,
                    handPose,
                    capturedFingerBoneSnapshotValid ?
                        &capturedFingerBoneSnapshot :
                        nullptr,
                    localTransforms,
                    localTransformMask)) {
                grip.fingerLocalTransforms = localTransforms;
                grip.fingerLocalTransformMask = localTransformMask;
                grip.hasFingerLocalTransforms = true;
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: full-hand local transform override prepared hand={} mask=0x{:04X}",
                    isLeft ? "left" : "right",
                    grip.fingerLocalTransformMask);
            }
        }

        grip.visualLerp = {};
        grip.active = true;
        if (isLeft) {
            _hapticEvents.leftPartGripCaptured = true;
        } else {
            _hapticEvents.rightPartGripCaptured = true;
        }

        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: part grip captured hand={} weapon='{}' gripLocal=({:.3f},{:.3f},{:.3f}) meshGrab={} sourceTriangles={} sources={} contactedTriangles={} fingerTriangles={} cachedTriangles={} sourceNodeCurrent={} surfaceSeat={:.2f}deg authoredSupport=NO acquisition={} authority={} provider={} attachOnly={} authoredCandidate={} authoredFrame={} authoredIdentity={} authoredGeneration={} authoredSurface={} surfaceDistance={:.3f} surfaceRadius={:.3f} authoredFingerMask=0x{:04X} touchToAuthoredSeat={:.3f} authoredSeatLocal=({:.3f},{:.3f},{:.3f}) touchProbeLocal=({:.3f},{:.3f},{:.3f}) frameError={:.4f} partKind={} pose={} generation={:016X}",
            isLeft ? "left" : "right",
            weaponNode->name.c_str(),
            grip.gripLocal.x,
            grip.gripLocal.y,
            grip.gripLocal.z,
            meshFound ? "YES" : "FALLBACK",
            sourceTriangleCount,
            compositeEvidenceViewCount,
            contactedSourceTriangleCount,
            fingerScratch.localTriangles.size(),
            cachedTrianglesFound ? "yes" : "no",
            cachedTrianglesFound && evidenceView.sourceNodeCurrent ? "yes" : "no",
            grip.surfaceSeatRotationRadians *
                two_handed_grip_detail::kRadiansToDegrees,
            decision.acquisitionSource == WeaponInteractionAcquisitionSource::PhysicalContact ?
                "contact" :
                (decision.acquisitionSource == WeaponInteractionAcquisitionSource::ProximityProbe ? "probe" : "none"),
            _authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport ?
                "visual-only" :
                "full",
            providerPartAuthority.active ? "yes" : "no",
            grip.attachOnly ? "yes" : "no",
            authoredSupportCandidateForHandValid ? "yes" : "no",
            authoredSupportFrameValid ? "yes" : "no",
            authoredWeaponIdentityMatches ? "yes" : "no",
            authoredGenerationMatches ? "yes" : "no",
            authoredSeatWeaponSurfaceValid ? "yes" : "no",
            authoredSupportSurfaceDistance,
            g_rockConfig.rockWeaponInteractionTouchRadius,
            authoredSupportFingerLocalTransformMask,
            authoredSupportTouchProbeDistance,
            authoredSupportPalmWeaponLocal.x,
            authoredSupportPalmWeaponLocal.y,
            authoredSupportPalmWeaponLocal.z,
            authoredSupportProximity.liveTouchProbeWeaponLocal.x,
            authoredSupportProximity.liveTouchProbeWeaponLocal.y,
            authoredSupportProximity.liveTouchProbeWeaponLocal.z,
            authoredSupportProximity.frameAgreementErrorGameUnits,
            static_cast<int>(grip.partKind),
            static_cast<int>(grip.gripPose),
            _activeWeaponGenerationKey);
        return true;
    }

    void TwoHandedGrip::lockPartGripToWeaponRoot(bool isLeft)
    {
        /*
         * Part-carry feeds its own solved weapon transform back as the next
         * frame's base, so grips must resolve exclusively through the captured
         * weapon-root frames while it is active. Following live part-node
         * chains lets any per-frame part animation integrate into a steady
         * carry drift and pulls the locked hand visuals apart (verified by
         * telemetry: rigid-weapon grip separation grew frame over frame).
         */
        WeaponPartGrip& grip = partGrip(isLeft);
        grip.hasSourceFrames = false;
        grip.hasAttachmentWeaponLocal = false;
    }

    void TwoHandedGrip::releasePartGrip(bool isLeft, const char* reason, const bool smoothHandReturn)
    {
        WeaponPartGrip& grip = partGrip(isLeft);
        if (!grip.active) {
            return;
        }
        if (dynamicSupportAcquisitionMatches(isLeft, grip)) {
            clearDynamicSupportAcquisition(reason, true);
        }
        if (smoothHandReturn) {
            beginHandVisualReturn(isLeft, reason);
        }
        clearSupportGripPose(isLeft);
        grip = {};
        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: part grip released hand={} reason={}", isLeft ? "left" : "right", reason ? reason : "unknown");
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

    bool TwoHandedGrip::providerPartAuthorityStillCurrent(WeaponPartGrip& grip, std::uint64_t currentWeaponGenerationKey)
    {
        if (!grip.providerPartAuthority.active) {
            return true;
        }
        if (currentWeaponGenerationKey == 0 || currentWeaponGenerationKey != grip.providerPartAuthority.weaponGenerationKey) {
            return false;
        }

        const auto query = buildProviderPartTargetQuery(
            grip.providerPartAuthority.weaponGenerationKey,
            grip.providerPartAuthority.bodyId,
            grip.providerPartAuthority.partKind,
            grip.providerPartAuthority.reloadRole,
            grip.providerPartAuthority.supportRole,
            grip.providerPartAuthority.socketRole,
            grip.providerPartAuthority.actionRole,
            grip.providerPartAuthority.sourceRoot,
            grip.providerPartAuthority.sourceName);

        ::rock::provider::RockProviderWeaponPartTargetResolutionV1 resolution{};
        if (!::rock::provider::resolveWeaponPartTargetV1(query, resolution)) {
            return false;
        }
        return resolution.matched != 0 &&
               resolution.ownerToken == grip.providerPartAuthority.ownerToken &&
               resolution.groupId == grip.providerPartAuthority.groupId &&
               static_cast<std::uint32_t>(resolution.grabMode) == grip.providerPartAuthority.grabMode;
    }

    bool TwoHandedGrip::providerPartTargetNewlyMatchesGrip(const WeaponPartGrip& grip, std::uint64_t currentWeaponGenerationKey) const
    {
        /*
         * Upgrade twin of providerPartAuthorityStillCurrent: a support grip
         * captured WITHOUT provider authority whose own part NOW resolves to
         * a matched provider target — a consumer armed its whitelist while
         * the hand was already holding the part (PAPER_Toolkit: pulling the
         * trigger mid-hold switches an authority grab to attach-only). The
         * caller releases the grip; the still-held grab recaptures within a
         * couple of frames under the new resolution, through the same
         * re-resolve path the downgrade direction uses when a target
         * disappears mid-grip. The query is built from the grip's own
         * captured contact identity, not the live contact, so a flickering
         * contact cannot convert against the wrong part.
         */
        if (!grip.active || grip.providerPartAuthority.active) {
            return false;
        }
        if (currentWeaponGenerationKey == 0 || currentWeaponGenerationKey != grip.weaponGenerationKey) {
            return false;
        }

        const auto query = buildProviderPartTargetQuery(
            grip.weaponGenerationKey,
            grip.contactBodyId,
            static_cast<std::uint32_t>(grip.partKind),
            static_cast<std::uint32_t>(grip.reloadRole),
            static_cast<std::uint32_t>(grip.supportRole),
            static_cast<std::uint32_t>(grip.socketRole),
            static_cast<std::uint32_t>(grip.actionRole),
            reinterpret_cast<std::uintptr_t>(grip.attachmentRoot),
            grip.sourceName);

        ::rock::provider::RockProviderWeaponPartTargetResolutionV1 resolution{};
        return ::rock::provider::resolveWeaponPartTargetV1(query, resolution) && resolution.matched != 0;
    }

    bool TwoHandedGrip::tryRebindPartGripToCurrentGeneration(
        WeaponPartGrip& grip,
        std::uint64_t currentWeaponGenerationKey,
        const WeaponCollision& weaponCollision)
    {
        if (!grip.active) {
            return true;
        }

        WeaponCollisionProfileEvidenceDescriptor bestDescriptor{};
        RE::NiAVObject* bestSourceNode = nullptr;
        int bestScore = 0;
        float bestDistanceSquared = (std::numeric_limits<float>::max)();
        bool bestAmbiguous = false;
        const std::string_view capturedSourceName{ grip.sourceName.data() };
        const auto distanceSquaredToBounds = [&grip](const WeaponEvidenceBounds3& bounds) {
            if (!bounds.valid) {
                return (std::numeric_limits<float>::max)();
            }
            const auto axisDistance = [](float value, float minimum, float maximum) {
                if (value < minimum) {
                    return minimum - value;
                }
                if (value > maximum) {
                    return value - maximum;
                }
                return 0.0f;
            };
            const float dx = axisDistance(grip.gripLocal.x, bounds.min.x, bounds.max.x);
            const float dy = axisDistance(grip.gripLocal.y, bounds.min.y, bounds.max.y);
            const float dz = axisDistance(grip.gripLocal.z, bounds.min.z, bounds.max.z);
            return dx * dx + dy * dy + dz * dz;
        };
        const auto bodyCount = weaponCollision.getWeaponBodyCount();
        for (std::uint32_t i = 0; i < bodyCount; ++i) {
            const auto bodyId = weaponCollision.getWeaponBodyIdAtomic(i);
            WeaponCollisionProfileEvidenceDescriptor descriptor{};
            RE::NiAVObject* sourceNode = nullptr;
            if (!weaponCollision.tryGetProfileEvidenceDescriptorForBodyId(bodyId, descriptor, sourceNode) ||
                !descriptor.valid || descriptor.weaponGenerationKey != currentWeaponGenerationKey) {
                continue;
            }

            const bool sourcePointerMatches = sourceNode && sourceNode == grip.attachmentRoot;
            const bool sourceNameMatches = !capturedSourceName.empty() && descriptor.sourceName == capturedSourceName;
            if ((!sourcePointerMatches && !sourceNameMatches) || descriptor.semantic.partKind != grip.partKind) {
                continue;
            }
            if (grip.omodFormId != 0 && descriptor.omodFormId != grip.omodFormId) {
                continue;
            }
            if (grip.attachPointFormId != 0 && descriptor.semantic.attachPointFormId != grip.attachPointFormId) {
                continue;
            }

            const int score = sourcePointerMatches ? 2 : 1;
            const float distanceSquared = distanceSquaredToBounds(descriptor.localBoundsGame);
            constexpr float kDistanceTieEpsilon = 0.0001f;
            if (score > bestScore ||
                (score == bestScore && distanceSquared + kDistanceTieEpsilon < bestDistanceSquared)) {
                bestScore = score;
                bestDistanceSquared = distanceSquared;
                bestAmbiguous = false;
                bestDescriptor = descriptor;
                bestSourceNode = sourceNode;
            } else if (score == bestScore &&
                       (distanceSquared == bestDistanceSquared ||
                           (std::isfinite(distanceSquared) && std::isfinite(bestDistanceSquared) &&
                               std::fabs(distanceSquared - bestDistanceSquared) <= kDistanceTieEpsilon))) {
                bestAmbiguous = true;
            }
        }

        if (bestScore == 0 || bestAmbiguous) {
            ROCK_LOG_WARN(Weapon,
                "TwoHandedGrip: part grip rebind failed closed hand={} generation={:016X} source='{}' part={} omod={:08X} attachPoint={:08X} reason={}",
                (&grip == &_partGrips[0]) ? "left" : "right",
                currentWeaponGenerationKey,
                capturedSourceName,
                static_cast<std::uint32_t>(grip.partKind),
                grip.omodFormId,
                grip.attachPointFormId,
                bestScore == 0 ? "missing" : "ambiguous");
            return false;
        }

        grip.weaponGenerationKey = currentWeaponGenerationKey;
        // The support input-to-target relation belongs to the old collision
        // generation. The game-thread reconciliation step recaptures it from
        // current transforms only after the new generation is eligible.
        grip.supportInputBaseline = {};
        grip.contactBodyId = bestDescriptor.bodyId;
        grip.attachmentRoot = grip.authoredSupportGrip ?
            _activeWeaponNode :
            (bestSourceNode ? bestSourceNode : grip.attachmentRoot);
        grip.partKind = bestDescriptor.semantic.partKind;
        grip.reloadRole = bestDescriptor.semantic.reloadRole;
        grip.supportRole = bestDescriptor.semantic.supportGripRole;
        grip.socketRole = bestDescriptor.semantic.socketRole;
        grip.actionRole = bestDescriptor.semantic.actionRole;
        grip.omodFormId = bestDescriptor.omodFormId;
        grip.attachPointFormId = bestDescriptor.semantic.attachPointFormId;
        grip.classificationSource = bestDescriptor.semantic.classificationSource;
        const auto copyLength = (std::min)(bestDescriptor.sourceName.size(), grip.sourceName.size() - 1);
        std::memcpy(grip.sourceName.data(), bestDescriptor.sourceName.data(), copyLength);
        grip.sourceName[copyLength] = '\0';

        if (grip.providerPartAuthority.active) {
            grip.providerPartAuthority.weaponGenerationKey = currentWeaponGenerationKey;
            grip.providerPartAuthority.bodyId = bestDescriptor.bodyId;
            grip.providerPartAuthority.sourceRoot = reinterpret_cast<std::uintptr_t>(bestSourceNode);
            grip.providerPartAuthority.partKind = static_cast<std::uint32_t>(grip.partKind);
            grip.providerPartAuthority.reloadRole = static_cast<std::uint32_t>(grip.reloadRole);
            grip.providerPartAuthority.supportRole = static_cast<std::uint32_t>(grip.supportRole);
            grip.providerPartAuthority.socketRole = static_cast<std::uint32_t>(grip.socketRole);
            grip.providerPartAuthority.actionRole = static_cast<std::uint32_t>(grip.actionRole);
            std::memcpy(
                grip.providerPartAuthority.sourceName.data(),
                grip.sourceName.data(),
                grip.providerPartAuthority.sourceName.size());
        }
        return true;
    }

    bool TwoHandedGrip::reconcileCollisionGeneration(
        RE::NiNode* currentWeaponNode,
        std::uint64_t currentWeaponGenerationKey,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const WeaponCollision& weaponCollision)
    {
        if (!equipped_weapon_manual_ownership_policy::canPreserveManualOwnership(
                _activeEquippedWeaponOwnershipKey,
                currentEquippedWeaponOwnershipKey,
                currentWeaponGenerationKey,
                _state != TwoHandedState::PrimaryOnly)) {
            return false;
        }
        if (currentWeaponGenerationKey == 0) {
            // PrimaryOnly rides the native firing-hand attach and can retain
            // ownership while the complete collider set is still building.
            _activeWeaponNode = currentWeaponNode;
            _activeWeaponGenerationKey = 0;
            _weaponNodeLocalBaseline = currentWeaponNode->local;
            _hasWeaponNodeLocalBaseline = true;
            return true;
        }

        const bool generationChanged = _activeWeaponGenerationKey != currentWeaponGenerationKey;
        const bool weaponRootChanged = _activeWeaponNode != currentWeaponNode;
        if (generationChanged || weaponRootChanged) {
            const auto previousGeneration = _activeWeaponGenerationKey;
            _activeWeaponNode = currentWeaponNode;
            _activeWeaponGenerationKey = currentWeaponGenerationKey;
            if (weaponRootChanged) {
                _weaponNodeLocalBaseline = currentWeaponNode->local;
                _hasWeaponNodeLocalBaseline = true;
            }
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: preserving manual ownership across collision rebuild oldGeneration={:016X} newGeneration={:016X} ownership={:016X} rootChanged={}",
                previousGeneration,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                weaponRootChanged ? "yes" : "no");
        }

        if (generationChanged || weaponRootChanged) {
            for (auto& grip : _partGrips) {
                if (grip.active && !tryRebindPartGripToCurrentGeneration(grip, currentWeaponGenerationKey, weaponCollision)) {
                    return false;
                }
            }
        }
        return true;
    }

    void TwoHandedGrip::updateFullWeaponAuthorityGrip(RE::NiNode* weaponNode, float dt)
    {
        const bool supportHandIsLeft = !_firingHandIsLeft;
        const bool primaryHandIsLeft = _firingHandIsLeft;
        WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        const bool gunstockBaselineActive =
            isGunstockSupportBaselineActive(
                supportHandIsLeft,
                supportGrip);
        const bool dynamicBaselineActive =
            isDynamicSupportBaselineActive(
                supportHandIsLeft,
                supportGrip);
        const bool supportInputBaselineActive =
            gunstockBaselineActive || dynamicBaselineActive;
        if (supportGrip.supportInputBaseline.active &&
            !supportInputBaselineActive) {
            supportGrip.supportInputBaseline = {};
        }

        bool dynamicAcquisition =
            dynamicSupportAcquisitionMatches(
                supportHandIsLeft,
                supportGrip);
        if (_dynamicSupportAcquisition.active &&
            !dynamicAcquisition) {
            clearDynamicSupportAcquisition(
                "grip-or-generation-witness-changed",
                true);
        }
        if (!dynamicAcquisition) {
            _rotationBlend = (std::min)(
                1.0f,
                _rotationBlend +
                    (std::isfinite(dt) && dt > 0.0f ? dt : 0.0f) *
                        ROTATION_BLEND_SPEED);
        }

        RE::NiTransform primaryTransform{};
        RE::NiTransform supportTransform{};
        if (!tryGetSolverHandTransform(primaryHandIsLeft, primaryTransform) || !tryGetSolverHandTransform(supportHandIsLeft, supportTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing support grip because authoritative hand transforms are unavailable");
            transitionToInactive(false);
            return;
        }

        RE::NiTransform calibratedPrimaryTransform = primaryTransform;
        RE::NiTransform calibratedSupportTransform = supportTransform;
        bool inputBaselineResolved = true;
        if (dynamicBaselineActive) {
            const auto& primaryDriver =
                _currentHandDriverFrames[
                    primaryHandIsLeft ? 0u : 1u];
            const auto& supportDriver =
                _currentHandDriverFrames[
                    supportHandIsLeft ? 0u : 1u];
            inputBaselineResolved =
                primaryDriver.valid &&
                supportDriver.valid &&
                weapon_support_acquisition_math::
                    tryResolveDynamicSupportDriverTargets(
                        primaryDriver.world,
                        supportGrip.supportInputBaseline.
                            primaryInputToGripTargetLocal,
                        supportDriver.world,
                        supportGrip.supportInputBaseline.
                            inputToGripTargetLocal,
                        calibratedPrimaryTransform,
                        calibratedSupportTransform);
        } else if (gunstockBaselineActive) {
            inputBaselineResolved =
                weapon_support_acquisition_math::
                    tryResolveSupportInputTarget(
                        supportTransform,
                        supportGrip.supportInputBaseline.
                            inputToGripTargetLocal,
                        calibratedSupportTransform);
        }
        if (supportInputBaselineActive &&
            !inputBaselineResolved) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon,
                "TwoHandedGrip: clearing support grip because its captured input baseline became invalid mode={} hand={} grip={} generation={:016X} primaryDriver={} supportDriver={}",
                gunstockBaselineActive ? "gunstock" : "dynamic",
                supportHandIsLeft ? "left" : "right",
                supportGrip.gripSequence,
                supportGrip.weaponGenerationKey,
                _currentHandDriverFrames[
                    primaryHandIsLeft ? 0u : 1u]
                        .valid ?
                    "valid" :
                    "missing",
                _currentHandDriverFrames[
                    supportHandIsLeft ? 0u : 1u]
                        .valid ?
                    "valid" :
                    "missing");
            transitionToInactive(false);
            return;
        }

        const RE::NiPoint3 primaryController =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(
                calibratedPrimaryTransform,
                primaryHandIsLeft);
        const RE::NiPoint3 supportController =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(
                calibratedSupportTransform,
                supportHandIsLeft);

        const RE::NiPoint3 currentSupportWorld = resolvePartGripWorld(supportGrip, weaponNode);
        const RE::NiPoint3 currentPrimaryGripWorld = transform_math::localPointToWorld(weaponNode->world, _primaryGripLocal);
        const float currentGripSeparationWorld = std::sqrt(dot(sub(currentSupportWorld, currentPrimaryGripWorld), sub(currentSupportWorld, currentPrimaryGripWorld)));
        const float lockedGripSeparationWorld = supportGrip.hasSourceFrames ? currentGripSeparationWorld : _lockedGripSeparationWorld;
        const RE::NiPoint3 supportGripLocal = resolvePartGripWeaponLocal(supportGrip, weaponNode);
        const RE::NiPoint3 lockedSupportControllerTarget = makeLockedSupportGripTarget(
            primaryController,
            supportController,
            currentSupportWorld,
            lockedGripSeparationWorld,
            0.001f);

        const RE::NiPoint3 supportTargetWorld =
            dynamicAcquisition ?
                lockedSupportControllerTarget :
                lerpPoint(
                    currentSupportWorld,
                    lockedSupportControllerTarget,
                    _rotationBlend);
        const auto solverInput = buildTwoHandedSolverInput(
            weaponNode->world,
            _primaryGripLocal,
            supportGripLocal,
            primaryController,
            supportTargetWorld,
            resolvePartGripNormalWeaponLocal(supportGrip, weaponNode),
            computePalmNormalFromHandBasis(
                calibratedSupportTransform,
                supportHandIsLeft));

        GunstockSupportBaselineDebugSnapshot* gunstockDebug = nullptr;
        if (g_rockConfig.rockDebugDrawGunstockAlignment) {
            ++_gunstockSupportBaselineDebugSequence;
            if (_gunstockSupportBaselineDebugSequence == 0) {
                ++_gunstockSupportBaselineDebugSequence;
            }
            auto& snapshot = _gunstockSupportBaselineDebugSnapshot;
            snapshot = {};
            snapshot.publicationSequence =
                _gunstockSupportBaselineDebugSequence;
            snapshot.weaponGenerationKey = supportGrip.weaponGenerationKey;
            snapshot.gripSequence = supportGrip.gripSequence;
            snapshot.weaponNodeIdentity =
                reinterpret_cast<std::uintptr_t>(weaponNode);
            snapshot.gripState = _state;
            snapshot.supportInputWorld = supportTransform;
            snapshot.calibratedSupportWorld = calibratedSupportTransform;
            snapshot.supportGripHandWorld =
                resolvePartGripHandWorld(supportGrip, weaponNode);
            snapshot.weaponWorldBefore = weaponNode->world;
            snapshot.primaryTargetWorld = primaryController;
            snapshot.supportGripWorld = currentSupportWorld;
            snapshot.supportTargetWorld =
                solverInput.supportTargetWorld;
            snapshot.behaviorEnabled =
                g_rockConfig.rockGunstockModeEnabled;
            snapshot.supportHandIsLeft = supportHandIsLeft;
            snapshot.baselineActive = gunstockBaselineActive;
            snapshot.attachPublication =
                gunstockBaselineActive &&
                supportGrip.supportInputBaseline.firstPublicationPending;
            snapshot.supportInputValid =
                isFiniteTransform(snapshot.supportInputWorld);
            snapshot.calibratedSupportValid =
                isFiniteTransform(snapshot.calibratedSupportWorld);
            snapshot.supportGripHandValid =
                isFiniteTransform(snapshot.supportGripHandWorld);
            snapshot.weaponBeforeValid =
                isFiniteTransform(snapshot.weaponWorldBefore);
            gunstockDebug = &snapshot;
        }

        const auto solved = solveTwoHandedWeaponTransformFrikPivot(solverInput);
        if (!solved.solved) {
            if (dynamicAcquisition || supportInputBaselineActive) {
                clearDynamicSupportAcquisition(
                    "full-target-solve-degenerate",
                    true);
                transitionToInactive(false);
            }
            return;
        }

        RE::NiTransform appliedWeaponWorld =
            solved.weaponWorldTransform;
        if (dynamicAcquisition) {
            auto& acquisition = _dynamicSupportAcquisition;
            if (!acquisition.durationInitialized) {
                auto axisOnlyInput = solverInput;
                axisOnlyInput.useSupportNormalTwist = false;
                axisOnlyInput.supportNormalTwistFactor = 0.0f;
                const auto axisOnlySolved =
                    solveTwoHandedWeaponTransformFrikPivot(
                        axisOnlyInput);

                acquisition.fullCorrectionRadians =
                    weapon_support_acquisition_math::
                        rotationAngleRadians(solved.rotationDelta);
                acquisition.axisCorrectionRadians =
                    axisOnlySolved.solved ?
                    weapon_support_acquisition_math::
                        rotationAngleRadians(
                            axisOnlySolved.rotationDelta) :
                    0.0f;
                acquisition.twistContributionRadians =
                    axisOnlySolved.solved ?
                    weapon_support_acquisition_math::
                        rotationDistanceRadians(
                            axisOnlySolved.rotationDelta,
                            solved.rotationDelta) :
                    0.0f;
                if (!std::isfinite(
                        acquisition.fullCorrectionRadians) ||
                    !std::isfinite(
                        acquisition.axisCorrectionRadians) ||
                    !std::isfinite(
                        acquisition.twistContributionRadians)) {
                    clearDynamicSupportAcquisition(
                        "non-finite-correction-angle",
                        true);
                    transitionToInactive(false);
                    return;
                }

                const RE::NiTransform fullPrimaryHandWorld =
                    weapon_visual_authority_math::
                        weaponLocalFrameToWorld(
                            solved.weaponWorldTransform,
                            _primaryHandWeaponLocal);
                const RE::NiTransform fullSupportHandWorld =
                    weapon_visual_authority_math::
                        weaponLocalFrameToWorld(
                            solved.weaponWorldTransform,
                            supportGrip.handWeaponLocal);
                const float primarySeatDistance =
                    hand_visual_lerp_math::
                        distanceGameUnits(
                            acquisition.primaryStartWorld.translate,
                            fullPrimaryHandWorld.translate);
                const float supportSeatDistance =
                    hand_visual_lerp_math::
                        distanceGameUnits(
                            acquisition.supportStartWorld.translate,
                            fullSupportHandWorld.translate);
                acquisition.initialSeatDistanceGameUnits =
                    (std::max)(
                        primarySeatDistance,
                        supportSeatDistance);
                acquisition.durationSeconds = 0.0f;
                if (g_rockConfig
                        .rockWeaponSupportGripHandLerpEnabled) {
                    acquisition.durationSeconds =
                        hand_visual_lerp_math::
                            computeDistanceMappedDurationGameUnits(
                                acquisition
                                    .initialSeatDistanceGameUnits,
                                g_rockConfig
                                    .rockWeaponSupportGripHandLerpTimeMin,
                                g_rockConfig
                                    .rockWeaponSupportGripHandLerpTimeMax,
                                g_rockConfig
                                    .rockWeaponSupportGripHandLerpMinDistance,
                                g_rockConfig
                                    .rockWeaponSupportGripHandLerpMaxDistance);
                    if (acquisition.durationSeconds <= 0.0f &&
                        acquisition.fullCorrectionRadians >=
                            DYNAMIC_SUPPORT_MINIMUM_SMOOTHED_ROTATION_RADIANS) {
                        acquisition.durationSeconds =
                            1.0f / ROTATION_BLEND_SPEED;
                    }
                }
                acquisition.durationInitialized = true;
            }

            acquisition.elapsedSeconds =
                hand_visual_lerp_math::
                    advanceTimedBlendElapsed(
                        acquisition.elapsedSeconds,
                        dt,
                        acquisition.durationSeconds);
            acquisition.rawAlpha =
                hand_visual_lerp_math::timedBlendAlpha(
                    acquisition.elapsedSeconds,
                    acquisition.durationSeconds);
            acquisition.easedAlpha =
                weapon_support_acquisition_math::
                    timedSmoothStepAlpha(
                        acquisition.elapsedSeconds,
                        acquisition.durationSeconds);
            _rotationBlend = acquisition.easedAlpha;

            const auto acquisitionSolve =
                weapon_support_acquisition_math::
                    applyRotationAroundPrimaryPivot<
                        RE::NiTransform,
                        RE::NiPoint3>(
                        solverInput.weaponWorldTransform,
                        solved.rotationDelta,
                        _primaryGripLocal,
                        primaryController,
                        acquisition.easedAlpha);
            if (!acquisitionSolve.valid) {
                clearDynamicSupportAcquisition(
                    "pivot-preserving-slerp-invalid",
                    true);
                transitionToInactive(false);
                return;
            }
            appliedWeaponWorld =
                acquisitionSolve.weaponWorldTransform;
            acquisition.lastAppliedRotationRadians =
                acquisitionSolve.appliedRotationRadians;
            acquisition.lastPrimaryPivotError =
                acquisitionSolve.primaryError;
            const RE::NiPoint3 appliedSupportWorld =
                transform_math::localPointToWorld(
                    appliedWeaponWorld,
                    supportGripLocal);
            acquisition.lastSupportTargetError =
                weaponSolverLength(
                weaponSolverSub(
                    appliedSupportWorld,
                    lockedSupportControllerTarget));
        }

        const bool supportBaselineAttachPublication =
            supportInputBaselineActive &&
            supportGrip.supportInputBaseline.firstPublicationPending;
        if (supportBaselineAttachPublication) {
            /*
             * Acquisition may move the visual support hand, but the first
             * support publication cannot alter the primary-owned weapon
             * transform. Starting next frame, the calibrated support input
             * contributes only its post-capture tandem delta.
             */
            appliedWeaponWorld =
                supportGrip.supportInputBaseline.weaponWorldAtCapture;
        }

        if (!applyWeaponVisualAuthority(weaponNode, appliedWeaponWorld)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing support grip because ROCK visual weapon authority failed");
            transitionToInactive(false);
            return;
        }

        static_assert(weapon_visual_authority_math::handPosePrecedesLockedHandAuthority());
        static_assert(weapon_visual_authority_math::weaponVisualPrecedesLockedHandAuthority());
        publishGripHandPoses(supportHandIsLeft);

        const bool applyPrimaryHandAuthority = weapon_support_authority_policy::supportGripAppliesPrimaryHandAuthority(_authorityMode);
        if (!applyLockedHandVisualAuthority(weaponNode, applyPrimaryHandAuthority, true, dt, &primaryTransform, &supportTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing support grip because ROCK locked hand authority failed");
            transitionToInactive(false);
            return;
        }

        if (gunstockDebug) {
            gunstockDebug->weaponWorldAfter = weaponNode->world;
            gunstockDebug->weaponAfterValid =
                isFiniteTransform(gunstockDebug->weaponWorldAfter);
            gunstockDebug->appliedRotationDegrees =
                hand_visual_lerp_math::rotationDistanceDegrees(
                    gunstockDebug->weaponWorldBefore,
                    gunstockDebug->weaponWorldAfter);
            gunstockDebug->appliedTranslationGameUnits =
                hand_visual_lerp_math::distanceGameUnits(
                    gunstockDebug->weaponWorldBefore.translate,
                    gunstockDebug->weaponWorldAfter.translate);
            if (gunstockBaselineActive &&
                supportBaselineAttachPublication) {
                gunstockDebug->attachRotationDegrees =
                    hand_visual_lerp_math::rotationDistanceDegrees(
                        supportGrip.supportInputBaseline.
                            weaponWorldAtCapture,
                        gunstockDebug->weaponWorldAfter);
                gunstockDebug->attachTranslationGameUnits =
                    hand_visual_lerp_math::distanceGameUnits(
                        supportGrip.supportInputBaseline.
                            weaponWorldAtCapture.translate,
                        gunstockDebug->weaponWorldAfter.translate);
            }
            gunstockDebug->published = true;
        }

        if (supportBaselineAttachPublication) {
            const float attachRotationDegrees =
                hand_visual_lerp_math::rotationDistanceDegrees(
                    supportGrip.supportInputBaseline.weaponWorldAtCapture,
                    weaponNode->world);
            const float attachTranslationGameUnits =
                hand_visual_lerp_math::distanceGameUnits(
                    supportGrip.supportInputBaseline.
                        weaponWorldAtCapture.translate,
                    weaponNode->world.translate);
            constexpr float kAttachRotationWarningDegrees = 0.05f;
            constexpr float kAttachTranslationWarningGameUnits = 0.01f;
            const bool attachInvariantHeld =
                std::isfinite(attachRotationDegrees) &&
                std::isfinite(attachTranslationGameUnits) &&
                attachRotationDegrees <=
                    kAttachRotationWarningDegrees &&
                attachTranslationGameUnits <=
                    kAttachTranslationWarningGameUnits;
            if (attachInvariantHeld) {
                ROCK_LOG_INFO(Weapon,
                    "TwoHandedGrip: support baseline published mode={} hand={} grip={} generation={:016X} attachWeaponDelta=({:.5f}gu,{:.5f}deg) surfaceSeat={:.2f}deg tandemDeltaAuthority=armed",
                    gunstockBaselineActive ? "gunstock" : "dynamic",
                    supportHandIsLeft ? "left" : "right",
                    supportGrip.gripSequence,
                    supportGrip.weaponGenerationKey,
                    attachTranslationGameUnits,
                    attachRotationDegrees,
                    supportGrip.surfaceSeatRotationRadians *
                        two_handed_grip_detail::kRadiansToDegrees);
            } else {
                ROCK_LOG_WARN(Weapon,
                    "TwoHandedGrip: support attach invariant exceeded mode={} hand={} grip={} generation={:016X} attachWeaponDelta=({:.5f}gu,{:.5f}deg)",
                    gunstockBaselineActive ? "gunstock" : "dynamic",
                    supportHandIsLeft ? "left" : "right",
                    supportGrip.gripSequence,
                    supportGrip.weaponGenerationKey,
                    attachTranslationGameUnits,
                    attachRotationDegrees);
            }
            supportGrip.supportInputBaseline.firstPublicationPending = false;
        }

        _lastSolvedWeaponTransform = weaponNode->world;
        _hasSolvedWeaponTransform = true;

        RE::NiPoint3 primaryGripFinal = transform_math::localPointToWorld(_lastSolvedWeaponTransform, _primaryGripLocal);
        RE::NiPoint3 offhandGripFinal = resolvePartGripWorld(supportGrip, weaponNode);

        if (dynamicAcquisition) {
            auto& acquisition = _dynamicSupportAcquisition;
            if (!acquisition.firstPublicationRecorded) {
                acquisition.firstPublicationRecorded = true;
                ROCK_LOG_INFO(Weapon,
                    "TwoHandedGrip: dynamic support acquisition event=start hand={} authored=no provider=no attachOnly=no grip={} generation={:016X} captureToFirstPublicationFrames=0 seatDistance={:.3f} duration={:.3f}s fullCorrection={:.2f}deg axisCorrection={:.2f}deg twistContribution={:.2f}deg firstRawAlpha={:.3f} firstEasedAlpha={:.3f} firstAppliedCorrection={:.2f}deg primaryPivotError={:.4f} supportTargetError={:.4f}",
                    supportHandIsLeft ? "left" : "right",
                    acquisition.gripSequence,
                    acquisition.weaponGenerationKey,
                    acquisition.initialSeatDistanceGameUnits,
                    acquisition.durationSeconds,
                    acquisition.fullCorrectionRadians *
                        two_handed_grip_detail::kRadiansToDegrees,
                    acquisition.axisCorrectionRadians *
                        two_handed_grip_detail::kRadiansToDegrees,
                    acquisition.twistContributionRadians *
                        two_handed_grip_detail::kRadiansToDegrees,
                    acquisition.rawAlpha,
                    acquisition.easedAlpha,
                    acquisition.lastAppliedRotationRadians *
                        two_handed_grip_detail::kRadiansToDegrees,
                    acquisition.lastPrimaryPivotError,
                    acquisition.lastSupportTargetError);
            }

            if (acquisition.easedAlpha >= 1.0f) {
                ROCK_LOG_INFO(Weapon,
                    "TwoHandedGrip: dynamic support acquisition event=complete hand={} grip={} generation={:016X} elapsed={:.3f}s duration={:.3f}s appliedCorrection={:.2f}deg primaryPivotError={:.4f} supportTargetError={:.4f}",
                    supportHandIsLeft ? "left" : "right",
                    acquisition.gripSequence,
                    acquisition.weaponGenerationKey,
                    acquisition.elapsedSeconds,
                    acquisition.durationSeconds,
                    acquisition.lastAppliedRotationRadians *
                        two_handed_grip_detail::kRadiansToDegrees,
                    acquisition.lastPrimaryPivotError,
                    acquisition.lastSupportTargetError);
                _rotationBlend = 1.0f;
                clearDynamicSupportAcquisition(
                    "completed",
                    false);
                dynamicAcquisition = false;
            }
        }

        if (++_gripLogCounter >= 90) {
            _gripLogCounter = 0;
            float separation = std::sqrt(dot(sub(primaryGripFinal, offhandGripFinal), sub(primaryGripFinal, offhandGripFinal)));
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: blend={:.2f}, dynamicAcquisition={}, separation={:.1f}gu, "
                "primaryGrip=({:.1f},{:.1f},{:.1f}), offhandGrip=({:.1f},{:.1f},{:.1f}), handLerp=({:.2f}/{:.3f}s,{:.2f}/{:.3f}s)",
                _rotationBlend,
                dynamicAcquisition ? "active" : "inactive",
                separation,
                primaryGripFinal.x,
                primaryGripFinal.y,
                primaryGripFinal.z,
                offhandGripFinal.x,
                offhandGripFinal.y,
                offhandGripFinal.z,
                _primaryHandVisualLerp.lastAlpha,
                _primaryHandVisualLerp.durationSeconds,
                supportGrip.visualLerp.lastAlpha,
                supportGrip.visualLerp.durationSeconds);
        }
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

    bool TwoHandedGrip::tryRecaptureProviderPartGrip(
        const bool isLeft,
        const bool otherGripCarries,
        RE::NiNode* weaponNode,
        const WeaponInteractionContact& contact,
        const WeaponInteractionRuntimeState& runtimeState,
        const WeaponCollision& weaponCollision,
        const char* handRole)
    {
        WeaponPartGrip& grip = partGrip(isLeft);
        if (!grip.active ||
            grip.providerPartAuthority.active ||
            !runtimeState.providerPartAuthority.active ||
            runtimeState.providerPartAuthority.bodyId !=
                grip.contactBodyId ||
            !otherGripCarries) {
            return false;
        }

        const WeaponInteractionDecision decision =
            routeWeaponInteraction(contact, runtimeState);
        if (decision.kind != WeaponInteractionKind::SupportGrip) {
            return false;
        }

        ROCK_LOG_INFO(
            Weapon,
            "TwoHandedGrip: recapturing {} part grip under newly matched provider weapon-part target",
            handRole ? handRole : "held-hand");
        releasePartGrip(isLeft, "provider-part-target-newly-matched");
        (void)capturePartGrip(
            isLeft,
            weaponNode,
            decision,
            weaponCollision,
            runtimeState.providerPartAuthority,
            false);
        return true;
    }

    void TwoHandedGrip::updatePartCarryGrip(
        RE::NiNode* weaponNode,
        float dt,
        const EquippedWeaponGripFrameInput& frameInput,
        const WeaponInteractionContact& leftWeaponContact,
        const WeaponInteractionContact& rightWeaponContact,
        const WeaponCollision& weaponCollision,
        std::uint64_t currentWeaponGenerationKey,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const WeaponInteractionRuntimeState& leftRuntimeState,
        const WeaponInteractionRuntimeState& rightRuntimeState)
    {
        const bool supportHandIsLeft = !_firingHandIsLeft;
        const bool firingHandIsLeft = _firingHandIsLeft;
        const WeaponInteractionContact& firingHandContact = firingHandIsLeft ? leftWeaponContact : rightWeaponContact;
        const WeaponInteractionContact& supportHandContact = supportHandIsLeft ? leftWeaponContact : rightWeaponContact;
        const WeaponInteractionRuntimeState& firingRuntimeState = firingHandIsLeft ? leftRuntimeState : rightRuntimeState;
        const WeaponInteractionRuntimeState& supportRuntimeState = supportHandIsLeft ? leftRuntimeState : rightRuntimeState;
        const bool supportGripHeld = supportHandIsLeft ? frameInput.leftGripHeld : frameInput.rightGripHeld;
        const bool supportHandHoldingObject = supportHandIsLeft ? frameInput.leftHandHoldingObject : frameInput.rightHandHoldingObject;
        const bool firingHandHoldingObject = firingHandIsLeft ? frameInput.leftHandHoldingObject : frameInput.rightHandHoldingObject;
        const bool authoredProviderAuthorityActive =
            leftRuntimeState.providerPartAuthority.active ||
            rightRuntimeState.providerPartAuthority.active;
        const bool authoredAttachOnlyAuthorityActive =
            weapon_part_grip_report_policy::providerGrabModeIsAttachOnly(
                leftRuntimeState.providerPartAuthority.active,
                leftRuntimeState.providerPartAuthority.grabMode) ||
            weapon_part_grip_report_policy::providerGrabModeIsAttachOnly(
                rightRuntimeState.providerPartAuthority.active,
                rightRuntimeState.providerPartAuthority.grabMode) ||
            (partGrip(true).active && partGrip(true).attachOnly) ||
            (partGrip(false).active && partGrip(false).attachOnly);

        /*
         * Firing-grip reattach is the squeeze gesture: a held grab with a
         * free palm inside the reattach radius re-takes the grip. Nothing
         * attaches to an open hand, and the gesture cannot re-capture a fresh
         * detach because the detach requires the grab to be open. A hand
         * already part-gripping is never converted; open it first, then
         * squeeze the grip. With ambidextrous takeover available, EITHER free
         * hand can squeeze the firing grip - whichever hand takes it becomes
         * the firing hand (the grip point itself stays weapon-relative). The
         * current firing hand is tested first so same-frame ties keep today's
         * behavior.
         */
        struct FiringGripReattachCandidate
        {
            bool isLeft;
            bool eligible;
            bool gripHeld;
            const WeaponInteractionContact* contact;
        };
        const FiringGripReattachCandidate reattachCandidates[2] = {
            { firingHandIsLeft,
                firingHandIsLeft ? frameInput.leftReattachEligible : frameInput.rightReattachEligible,
                frameInput.primaryGripInput.held,
                &firingHandContact },
            { supportHandIsLeft,
                supportHandIsLeft ? frameInput.leftReattachEligible : frameInput.rightReattachEligible,
                supportGripHeld,
                &supportHandContact },
        };
        for (const FiringGripReattachCandidate& candidate : reattachCandidates) {
            if (candidate.isLeft != firingHandIsLeft &&
                (!_handlingSettings.ambidextrousHandoffEnabled ||
                    !canBeginPrimaryOnlyGripForHand(candidate.isLeft))) {
                continue;
            }
            if (!candidate.eligible || partGrip(candidate.isLeft).active) {
                continue;
            }
            float palmToGripDistance = 0.0f;
            if (!tryComputePalmToGripDistanceForHand(weaponNode, candidate.isLeft, palmToGripDistance)) {
                continue;
            }
            const bool reattachRequested = weapon_two_handed_grip_math::shouldReattachFiringGripOnGrab(
                candidate.gripHeld,
                palmToGripDistance,
                _handlingSettings.firingGripReattachRadiusGameUnits) ||
                (candidate.gripHeld &&
                    candidate.contact->acquisitionSource ==
                        WeaponInteractionAcquisitionSource::ProximityProbe &&
                    !authoredProviderAuthorityActive &&
                    !authoredAttachOnlyAuthorityActive);
            if (!_firingGripReattachHoverInsideRadius &&
                weapon_two_handed_grip_math::isFiringGripReattachHoverCandidate(
                    candidate.gripHeld,
                    palmToGripDistance,
                    _handlingSettings.firingGripReattachRadiusGameUnits)) {
                _firingGripReattachHoverInsideRadius = true;
                _firingGripReattachHoverHandIsLeft = candidate.isLeft;
            }
            if (reattachRequested &&
                tryReattachFiringGrip(
                    candidate.isLeft,
                    weaponNode,
                    *candidate.contact,
                    authoredProviderAuthorityActive,
                    authoredAttachOnlyAuthorityActive)) {
                const bool newSupportHandIsLeft = !_firingHandIsLeft;
                if (partGrip(newSupportHandIsLeft).active) {
                    // Re-lock the two-hand separation against the (possibly
                    // swapped) support grip and re-blend the support target in.
                    const RE::NiPoint3 supportGripWorld = resolvePartGripWorld(partGrip(newSupportHandIsLeft), weaponNode);
                    const RE::NiPoint3 firingGripWorld = weaponLocalToWorld(_primaryGripLocal, weaponNode);
                    const RE::NiPoint3 separationDelta = sub(supportGripWorld, firingGripWorld);
                    const float separation = std::sqrt(dot(separationDelta, separationDelta));
                    if (std::isfinite(separation)) {
                        _lockedGripSeparationWorld = separation;
                    }
                    if (candidate.isLeft != firingHandIsLeft) {
                        _rotationBlend = 0.0f;
                    }
                    _state = TwoHandedState::Gripping;
                    // Fresh two-hand configuration: the just-taken firing grip
                    // gets the same release-defer window as a fresh support grab.
                    _supportGripAgeFrames = 0;
                    _freshSupportGripDeferLogged = false;
                    RE::NiTransform supportInputWorld{};
                    if (!tryGetSolverHandTransform(
                            newSupportHandIsLeft,
                            supportInputWorld) ||
                        !initializeGunstockSupportRole(
                            weaponNode,
                            newSupportHandIsLeft,
                            supportInputWorld,
                            "part-carry-firing-grip-reattach")) {
                        ROCK_LOG_WARN(
                            Weapon,
                            "TwoHandedGrip: part-carry reattach failed closed because the new support role could not be initialized");
                        transitionToInactive(false);
                        return;
                    }
                    WeaponPartGrip& reattachedSupportGrip =
                        partGrip(newSupportHandIsLeft);
                    const bool reattachedGunstockBaselineActive =
                        isGunstockSupportBaselineActive(
                            newSupportHandIsLeft,
                            reattachedSupportGrip);
                    if (!reattachedGunstockBaselineActive &&
                        weapon_support_authority_policy::
                            shouldUseDynamicSupportAcquisition(
                                _authorityMode,
                                reattachedSupportGrip.authoredSupportGrip,
                                reattachedSupportGrip.providerPartAuthority.active,
                                reattachedSupportGrip.attachOnly) &&
                        !initializeDynamicSupportBaseline(
                            weaponNode,
                            newSupportHandIsLeft,
                            "part-carry-firing-grip-reattach")) {
                        ROCK_LOG_WARN(
                            Weapon,
                            "TwoHandedGrip: part-carry reattach failed closed because the zero-delta dynamic baseline could not be captured");
                        transitionToInactive(false);
                        return;
                    }
                    updateFullWeaponAuthorityGrip(weaponNode, dt);
                } else {
                    if (!_firingHandIsLeft && ownsWeaponTransform()) {
                        beginWeaponVisualReturn("part-carry-reattached-primary-only");
                    }
                    transitionToPrimaryOnly(
                        weaponNode,
                        currentWeaponGenerationKey,
                        currentEquippedWeaponOwnershipKey,
                        "part-carry-reattached-firing-grip");
                }
                return;
            }
        }

        bool lastReleaseWasSupportHand = true;

        WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        if (supportGrip.active) {
            if (!providerPartAuthorityStillCurrent(supportGrip, currentWeaponGenerationKey) || !supportRuntimeState.supportGripAllowed) {
                /*
                 * Provider revocation and offhand reservation are policy
                 * changes, not a player release: return the weapon to
                 * FRIK-native carry instead of dropping it.
                 */
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing part-carry authority because provider revoked the support part grip");
                transitionToInactive(false);
                return;
            }
            if (!weapon_two_handed_grip_math::shouldContinueSupportGrip(supportGripHeld, supportHandHoldingObject)) {
                releasePartGrip(supportHandIsLeft, "support-grip-released", true);
                lastReleaseWasSupportHand = true;
            }
        }

        WeaponPartGrip& freeHandGrip = partGrip(firingHandIsLeft);
        if (freeHandGrip.active) {
            if (!providerPartAuthorityStillCurrent(freeHandGrip, currentWeaponGenerationKey)) {
                releasePartGrip(firingHandIsLeft, "provider-part-authority-lost");
                lastReleaseWasSupportHand = false;
            } else if (!weapon_two_handed_grip_math::shouldContinueSupportGrip(frameInput.primaryGripInput.held, firingHandHoldingObject)) {
                releasePartGrip(firingHandIsLeft, "free-hand-grip-released", true);
                lastReleaseWasSupportHand = false;
            }
        }

        /*
         * Mid-hold conversion, part-carry flavor: a part grip captured
         * WITHOUT provider authority whose own part now resolves to a
         * matched provider target (consumer armed its whitelist while the
         * hand was already holding — per-hand trigger arming) recaptures
         * immediately under the new resolution. Gated on the OTHER grip
         * holding carry authority: converting the last carry grip to
         * attach-only glue would drop the weapon through the fail-closed
         * all-grips check below. The free hand recaptures in the same
         * update rather than release-to-recapture because its capture path
         * is press-edged; the support hand gets the same treatment for
         * symmetry (no one-frame glue gap).
         */
        (void)tryRecaptureProviderPartGrip(
            firingHandIsLeft,
            weapon_part_grip_report_policy::partGripCountsAsCarry(
                supportGrip.active,
                supportGrip.attachOnly),
            weaponNode,
            firingHandContact,
            firingRuntimeState,
            weaponCollision,
            "free-hand");
        (void)tryRecaptureProviderPartGrip(
            supportHandIsLeft,
            weapon_part_grip_report_policy::partGripCountsAsCarry(
                freeHandGrip.active,
                freeHandGrip.attachOnly),
            weaponNode,
            supportHandContact,
            supportRuntimeState,
            weaponCollision,
            "support");

        if (!freeHandGrip.active && frameInput.primaryGripInput.pressed) {
            const WeaponInteractionDecision freeHandDecision = routeWeaponInteraction(firingHandContact, firingRuntimeState);
            if (weapon_two_handed_grip_math::canStartFreeHandPartGrip(
                    freeHandDecision.kind == WeaponInteractionKind::SupportGrip,
                    frameInput.primaryGripInput.pressed,
                    firingHandHoldingObject,
                    freeHandGrip.active)) {
                if (capturePartGrip(firingHandIsLeft, weaponNode, freeHandDecision, weaponCollision, firingRuntimeState.providerPartAuthority, false)) {
                    /*
                     * AttachOnly keeps its authored source frames so the glued
                     * hand follows provider-driven part motion; it never joins
                     * the carry solve, so it cannot feed part animation back
                     * into the carry (the drift lockPartGripToWeaponRoot
                     * prevents). Separation/blend only matter for a two-anchor
                     * carry, which needs both grips to hold carry authority.
                     */
                    if (freeHandGrip.attachOnly) {
                        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: free-hand part grip captured as provider attach-only glue");
                    } else {
                        lockPartGripToWeaponRoot(firingHandIsLeft);
                        if (weapon_part_grip_report_policy::partGripCountsAsCarry(supportGrip.active, supportGrip.attachOnly)) {
                            _partCarryGripSeparationWorld = partCarryGripSeparation(weaponNode);
                            _rotationBlend = 0.0f;
                        }
                    }
                }
            }
        }

        if (!supportGrip.active) {
            const WeaponInteractionDecision supportDecision = routeWeaponInteraction(supportHandContact, supportRuntimeState);
            if (supportDecision.kind == WeaponInteractionKind::SupportGrip &&
                weapon_two_handed_grip_math::canStartSupportGrip(true, supportGripHeld, supportHandHoldingObject)) {
                if (capturePartGrip(supportHandIsLeft, weaponNode, supportDecision, weaponCollision, supportRuntimeState.providerPartAuthority, false)) {
                    // Symmetric to the free-hand capture above: attach-only
                    // glue keeps source frames and stays out of the carry.
                    if (supportGrip.attachOnly) {
                        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: support-hand part grip captured as provider attach-only glue");
                    } else {
                        lockPartGripToWeaponRoot(supportHandIsLeft);
                        if (weapon_part_grip_report_policy::partGripCountsAsCarry(freeHandGrip.active, freeHandGrip.attachOnly)) {
                            _partCarryGripSeparationWorld = partCarryGripSeparation(weaponNode);
                            _rotationBlend = 0.0f;
                        }
                    }
                }
            }
        }

        /*
         * Only carry-authority grips can hold the weapon. When the last carry
         * grip releases, a remaining AttachOnly glue cannot inherit pivot
         * authority (never upgrade), so it releases with the carry and the
         * normal manual-drop request proceeds (fail closed).
         */
        if (!weapon_part_grip_report_policy::partGripCountsAsCarry(supportGrip.active, supportGrip.attachOnly) &&
            !weapon_part_grip_report_policy::partGripCountsAsCarry(freeHandGrip.active, freeHandGrip.attachOnly)) {
            releasePartGrip(supportHandIsLeft, "carry-authority-lost", true);
            releasePartGrip(firingHandIsLeft, "carry-authority-lost", true);
            requestEquippedWeaponDrop(
                "part-carry-all-grips-released",
                lastReleaseWasSupportHand ?
                    (supportHandIsLeft ? equipped_weapon_drop_policy::SourceHand::Left : equipped_weapon_drop_policy::SourceHand::Right) :
                    (firingHandIsLeft ? equipped_weapon_drop_policy::SourceHand::Left : equipped_weapon_drop_policy::SourceHand::Right));
            return;
        }

        // The pivot must always be a carry-authority grip; after the check
        // above, the other hand is guaranteed to hold one.
        if (!weapon_part_grip_report_policy::partGripCountsAsCarry(
                partGrip(_partCarryPivotIsLeft).active,
                partGrip(_partCarryPivotIsLeft).attachOnly)) {
            _partCarryPivotIsLeft = !_partCarryPivotIsLeft;
        }

        (void)solvePartCarryWeaponAuthority(weaponNode, dt);
    }

    float TwoHandedGrip::partCarryGripSeparation(RE::NiNode* weaponNode) const
    {
        const RE::NiPoint3 leftGripWorld = resolvePartGripWorld(partGrip(true), weaponNode);
        const RE::NiPoint3 rightGripWorld = resolvePartGripWorld(partGrip(false), weaponNode);
        const RE::NiPoint3 delta = sub(leftGripWorld, rightGripWorld);
        const float separation = std::sqrt(dot(delta, delta));
        return std::isfinite(separation) ? separation : 0.0f;
    }

    bool TwoHandedGrip::solvePartCarryWeaponAuthority(RE::NiNode* weaponNode, float dt)
    {
        const bool pivotIsLeft = _partCarryPivotIsLeft;
        const WeaponPartGrip& pivotGrip = partGrip(pivotIsLeft);
        const WeaponPartGrip& aimGrip = partGrip(!pivotIsLeft);
        // An AttachOnly glue never aims the weapon; the carry solves
        // single-anchor around the pivot and the glue publishes afterwards.
        const bool aimGripCarries = weapon_part_grip_report_policy::partGripCountsAsCarry(aimGrip.active, aimGrip.attachOnly);
        if (!pivotGrip.active || !pivotGrip.hasHandWeaponLocal) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because captured hand frames are unavailable");
            transitionToInactive(false);
            return false;
        }

        RE::NiTransform pivotHandTransform{};
        if (!tryGetSolverHandTransform(pivotIsLeft, pivotHandTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because authoritative hand transforms are unavailable");
            transitionToInactive(false);
            return false;
        }

        if (aimGripCarries) {
            RE::NiTransform aimHandTransform{};
            if (!tryGetSolverHandTransform(!pivotIsLeft, aimHandTransform)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because aim hand transform is unavailable");
                transitionToInactive(false);
                return false;
            }

            _rotationBlend = (std::min)(1.0f, _rotationBlend + dt * ROTATION_BLEND_SPEED);

            /*
             * The two-anchor solve must be closed over the captured
             * weapon-root-local grip points. Part-carry feeds its own solved
             * transform back as the next frame's base, so re-resolving grips
             * through live part-node chains lets any per-frame part animation
             * integrate into a steady carry drift (verified by telemetry:
             * rigid-weapon grip separation grew frame over frame). The
             * trade-off is that a two-anchor carry does not follow externally
             * driven part motion; provider part revocation releases the grip
             * in that case.
             */
            const RE::NiPoint3 pivotPalm = computeGrabLegacyPalmPivotAWorldFromHandBasis(pivotHandTransform, pivotIsLeft);
            const RE::NiPoint3 aimPalm = computeGrabLegacyPalmPivotAWorldFromHandBasis(aimHandTransform, !pivotIsLeft);
            const RE::NiPoint3 currentAimGripWorld = weaponLocalToWorld(aimGrip.gripLocal, weaponNode);
            const RE::NiPoint3 currentPivotGripWorld = weaponLocalToWorld(pivotGrip.gripLocal, weaponNode);
            const RE::NiPoint3 currentSeparationDelta = sub(currentAimGripWorld, currentPivotGripWorld);
            const float currentSeparation = std::sqrt(dot(currentSeparationDelta, currentSeparationDelta));
            const float lockedSeparation = _partCarryGripSeparationWorld > 0.0f ? _partCarryGripSeparationWorld : currentSeparation;
            const RE::NiPoint3 lockedAimTarget = makeLockedSupportGripTarget(
                pivotPalm,
                aimPalm,
                currentAimGripWorld,
                lockedSeparation,
                0.001f);
            const RE::NiPoint3 blendedAimTarget = lerpPoint(currentAimGripWorld, lockedAimTarget, _rotationBlend);

            const auto solverInput = buildTwoHandedSolverInput(
                weaponNode->world,
                pivotGrip.gripLocal,
                aimGrip.gripLocal,
                pivotPalm,
                blendedAimTarget,
                aimGrip.normalLocal,
                computePalmNormalFromHandBasis(
                    aimHandTransform,
                    !pivotIsLeft));

            const auto solved = solveTwoHandedWeaponTransformFrikPivot(solverInput);
            if (!solved.solved) {
                return true;
            }

            // Break the rotation feedback loop's orthonormality decay before
            // the solved transform becomes next frame's base.
            RE::NiTransform stabilizedWeaponWorld = solved.weaponWorldTransform;
            stabilizedWeaponWorld.rotate = orthonormalizeStoredRotation(stabilizedWeaponWorld.rotate);

            if (!applyWeaponVisualAuthority(weaponNode, stabilizedWeaponWorld)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because ROCK visual weapon authority failed");
                transitionToInactive(false);
                return false;
            }

            static_assert(weapon_visual_authority_math::handPosePrecedesLockedHandAuthority());
            static_assert(weapon_visual_authority_math::weaponVisualPrecedesLockedHandAuthority());
            publishGripHandPoses(pivotIsLeft);
            publishGripHandPoses(!pivotIsLeft);

            if (!applyPartGripLockedVisual(pivotIsLeft, weaponNode, dt, &pivotHandTransform) ||
                !applyPartGripLockedVisual(!pivotIsLeft, weaponNode, dt, &aimHandTransform)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because ROCK part grip hand authority failed");
                transitionToInactive(false);
                return false;
            }
        } else {
            RE::NiTransform solvedWeaponWorld{};
            if (pivotGrip.hasSourceFrames && pivotGrip.hasAttachmentWeaponLocal && resolveCurrentSupportAttachmentRoot(pivotGrip, weaponNode)) {
                const RE::NiTransform solvedSourceWorld =
                    transform_math::composeTransforms(pivotHandTransform, transform_math::invertTransform(pivotGrip.handSourceLocal));
                solvedWeaponWorld = transform_math::composeTransforms(solvedSourceWorld, transform_math::invertTransform(pivotGrip.attachmentWeaponLocal));
            } else {
                solvedWeaponWorld = transform_math::composeTransforms(pivotHandTransform, transform_math::invertTransform(pivotGrip.handWeaponLocal));
            }
            if (!isFiniteTransform(solvedWeaponWorld)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because single-anchor weapon solve produced invalid transform");
                transitionToInactive(false);
                return false;
            }

            if (!applyWeaponVisualAuthority(weaponNode, solvedWeaponWorld)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because ROCK visual weapon authority failed");
                transitionToInactive(false);
                return false;
            }

            static_assert(weapon_visual_authority_math::handPosePrecedesLockedHandAuthority());
            publishGripHandPoses(pivotIsLeft);
            if (!applyPartGripLockedVisual(pivotIsLeft, weaponNode, dt, &pivotHandTransform)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because ROCK part grip hand authority failed");
                transitionToInactive(false);
                return false;
            }
        }

        /*
         * AttachOnly glue publishes after the weapon solve so it composes from
         * this frame's part transforms (including provider part drives applied
         * earlier in the frame). A glue visual failure only loses the attach;
         * the carry pivot must survive it.
         */
        if (aimGrip.active && aimGrip.attachOnly) {
            RE::NiTransform attachHandTransform{};
            const RE::NiTransform* liveAttachHandWorld =
                tryGetSolverHandTransform(!pivotIsLeft, attachHandTransform) ? &attachHandTransform : nullptr;
            publishGripHandPoses(!pivotIsLeft);
            if (!applyPartGripLockedVisual(!pivotIsLeft, weaponNode, dt, liveAttachHandWorld)) {
                releasePartGrip(!pivotIsLeft, "attach-only-visual-authority-failed");
            }
        }

        _lastSolvedWeaponTransform = weaponNode->world;
        _hasSolvedWeaponTransform = true;

        if (++_gripLogCounter >= 90) {
            _gripLogCounter = 0;
            const RE::NiPoint3 pivotGripFinal = resolvePartGripWorld(pivotGrip, weaponNode);
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: part-carry authority pivot={} anchors={} pivotGrip=({:.1f},{:.1f},{:.1f}) handLerp=({:.2f}/{:.3f}s,{:.2f}/{:.3f}s)",
                pivotIsLeft ? "left" : "right",
                aimGrip.active ? 2 : 1,
                pivotGripFinal.x,
                pivotGripFinal.y,
                pivotGripFinal.z,
                pivotGrip.visualLerp.lastAlpha,
                pivotGrip.visualLerp.durationSeconds,
                aimGrip.visualLerp.lastAlpha,
                aimGrip.visualLerp.durationSeconds);
        }
        return true;
    }

    void TwoHandedGrip::updateVisualOnlySupportGrip(RE::NiNode* weaponNode, float dt)
    {
        const bool supportHandIsLeft = !_firingHandIsLeft;

        if (_firingHandIsLeft) {
            // Visual-only support never steers aim, but with a LEFT firing
            // hand the weapon itself must still be ROCK-carried (FRIK's glue
            // is blocked); the shooting-cup right hand stays visual-only.
            if (!solveLeftFiringWeaponCarry(weaponNode)) {
                return;
            }
        }

        static_assert(weapon_visual_authority_math::handPosePrecedesLockedHandAuthority());
        publishGripHandPoses(supportHandIsLeft);

        RE::NiTransform supportTransform{};
        const RE::NiTransform* liveSupportTransform = tryGetSolverHandTransform(supportHandIsLeft, supportTransform) ? &supportTransform : nullptr;
        if (!applyLockedHandVisualAuthority(weaponNode, false, true, dt, nullptr, liveSupportTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing visual-only support grip because ROCK support hand authority failed");
            transitionToInactive(false);
            return;
        }

        _lastSolvedWeaponTransform = weaponNode ? weaponNode->world : RE::NiTransform{};
        _hasSolvedWeaponTransform = _firingHandIsLeft && _hasSolvedWeaponTransform;

        if (weaponNode && ++_gripLogCounter >= 90) {
            _gripLogCounter = 0;
            const WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
            const RE::NiPoint3 offhandGripFinal = resolvePartGripWorld(supportGrip, weaponNode);
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: visual-only support follows weapon='{}', offhandGrip=({:.1f},{:.1f},{:.1f}), handLerp={:.2f}/{:.3f}s",
                weaponNode->name.c_str(),
                offhandGripFinal.x,
                offhandGripFinal.y,
                offhandGripFinal.z,
                supportGrip.visualLerp.lastAlpha,
                supportGrip.visualLerp.durationSeconds);
        }
    }

    void TwoHandedGrip::setSupportGripPose(
        bool isLeft,
        const grab_finger_pose_runtime::SolvedGrabFingerPose* meshFingerPose,
        const std::array<float, 5>* capturedSplayRadians,
        const SupportGripPoseFallback fallback)
    {
        WeaponPartGrip& grip = partGrip(isLeft);
        if (meshFingerPose && meshFingerPose->solved) {
            grip.fingerPose = meshFingerPose->hasJointValues ? meshFingerPose->jointValues : grab_finger_pose_math::expandFingerCurlsToJointValues(meshFingerPose->values);
            grip.fingerSplayRadians = capturedSplayRadians ? *capturedSplayRadians : std::array<float, 5>{};
            grip.hasFingerSplay = capturedSplayRadians != nullptr;
            grip.hasFingerPose = true;
            return;
        }

        float fallbackValue = 0.0f;
        if (fallback == SupportGripPoseFallback::SelectedClose) {
            const float fallbackMin =
                std::clamp(std::isfinite(g_rockConfig.rockGrabFingerMinValue) ? g_rockConfig.rockGrabFingerMinValue : 0.2f, 0.0f, 1.0f);
            const float configuredFallback =
                std::isfinite(g_rockConfig.rockSelectedCloseFingerAnimValue) ? g_rockConfig.rockSelectedCloseFingerAnimValue : 0.9f;
            fallbackValue = std::clamp(
                configuredFallback,
                fallbackMin,
                1.0f);
        }
        const std::array<float, 5> fallbackCurls{
            fallbackValue,
            fallbackValue,
            fallbackValue,
            fallbackValue,
            fallbackValue,
        };
        grip.fingerPose = grab_finger_pose_math::expandFingerCurlsToJointValues(fallbackCurls);
        grip.fingerSplayRadians = {};
        grip.hasFingerPose = true;
        grip.hasFingerSplay = false;
        grip.fingerLocalTransforms = {};
        grip.fingerLocalTransformMask = 0;
        grip.hasFingerLocalTransforms = false;
        if (fallback == SupportGripPoseFallback::FullyClosed) {
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: using whole-hand-closed finger fallback hand={} value={:.2f}",
                isLeft ? "left" : "right",
                fallbackValue);
        } else {
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: using selected-close finger fallback hand={} value={:.2f}",
                isLeft ? "left" : "right",
                fallbackValue);
        }
    }

    void TwoHandedGrip::clearSupportGripPose(bool isLeft)
    {
        _hasLastPublishedHandWorld[isLeft ? 0u : 1u] = false;
        WeaponPartGrip& grip = partGrip(isLeft);
        grip.fingerPose = {};
        grip.fingerSplayRadians = {};
        grip.hasFingerPose = false;
        grip.hasFingerSplay = false;
        grip.fingerLocalTransforms = {};
        grip.fingerLocalTransformMask = 0;
        grip.hasFingerLocalTransforms = false;

        (void)frik_visual_authority::clearHandPose(SUPPORT_GRIP_TAG, handFromBool(isLeft));
        deferOrClearHandAuthorityRole(
            scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip,
            isLeft);
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

    bool TwoHandedGrip::isDynamicSupportBaselineActive(
        const bool supportHandIsLeft,
        const WeaponPartGrip& supportGrip) const
    {
        return isSupportInputBaselineActive(
            supportHandIsLeft,
            supportGrip,
            SupportInputBaselineKind::Dynamic);
    }

    bool TwoHandedGrip::initializeDynamicSupportBaseline(
        RE::NiNode* weaponNode,
        const bool supportHandIsLeft,
        const char* reason)
    {
        WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        if (_authorityMode != weapon_support_authority_policy::
                                  WeaponSupportAuthorityMode::
                                      FullTwoHandedSolver ||
            supportGrip.authoredSupportGrip ||
            supportGrip.providerPartAuthority.active ||
            supportGrip.attachOnly ||
            !weaponNode ||
            !supportGrip.active ||
            !supportGrip.hasHandWeaponLocal ||
            !_hasFiringHandWeaponLocal ||
            supportGrip.weaponGenerationKey == 0 ||
            supportGrip.gripSequence == 0) {
            return false;
        }
        if (isDynamicSupportBaselineActive(
                supportHandIsLeft,
                supportGrip)) {
            return true;
        }

        const bool primaryHandIsLeft = !supportHandIsLeft;
        const auto& primaryDriver =
            _currentHandDriverFrames[
                primaryHandIsLeft ? 0u : 1u];
        const auto& supportDriver =
            _currentHandDriverFrames[
                supportHandIsLeft ? 0u : 1u];
        const RE::NiTransform primaryGripTargetWorld =
            weapon_visual_authority_math::weaponLocalFrameToWorld(
                weaponNode->world,
                _primaryHandWeaponLocal);
        const RE::NiTransform supportGripTargetWorld =
            resolvePartGripHandWorld(supportGrip, weaponNode);
        RE::NiTransform primaryDriverToTargetLocal{};
        RE::NiTransform supportDriverToTargetLocal{};
        if (!primaryDriver.valid ||
            !supportDriver.valid ||
            !weapon_support_acquisition_math::
                tryCaptureDynamicSupportDriverBaseline(
                    primaryDriver.world,
                    primaryGripTargetWorld,
                    supportDriver.world,
                    supportGripTargetWorld,
                    primaryDriverToTargetLocal,
                    supportDriverToTargetLocal)) {
            supportGrip.supportInputBaseline = {};
            ROCK_LOG_WARN(
                Weapon,
                "TwoHandedGrip: dynamic driver baseline capture failed hand={} primaryDriver={} supportDriver={} grip={} generation={:016X} reason={}",
                supportHandIsLeft ? "left" : "right",
                primaryDriver.valid ? "valid" : "missing",
                supportDriver.valid ? "valid" : "missing",
                supportGrip.gripSequence,
                supportGrip.weaponGenerationKey,
                reason ? reason : "unknown");
            return false;
        }

        supportGrip.supportInputBaseline = {
            .inputToGripTargetLocal = supportDriverToTargetLocal,
            .primaryInputToGripTargetLocal =
                primaryDriverToTargetLocal,
            .weaponWorldAtCapture = weaponNode->world,
            .weaponGenerationKey = supportGrip.weaponGenerationKey,
            .gripSequence = supportGrip.gripSequence,
            .supportHandIsLeft = supportHandIsLeft,
            .kind = SupportInputBaselineKind::Dynamic,
            .active = true,
            .pairedDynamicDrivers = true,
            .firstPublicationPending = true,
        };
        ROCK_LOG_INFO(
            Weapon,
            "TwoHandedGrip: paired dynamic driver baseline captured hand={} grip={} generation={:016X} reason={}; rendered hands excluded from solver input",
            supportHandIsLeft ? "left" : "right",
            supportGrip.gripSequence,
            supportGrip.weaponGenerationKey,
            reason ? reason : "unknown");
        return true;
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
