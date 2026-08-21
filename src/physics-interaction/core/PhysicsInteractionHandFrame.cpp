/*
 * HAND FRAME READERS: the one place that answers "where is the interaction hand
 * this frame".
 *
 * getInteractionHandFrame is the single source other TUs use, so hand-space rules
 * stay in one file. refreshExternalHandWorldTransformsBeforeFrik publishes the
 * external hand transforms before FRIK runs, which fixes the frame order between
 * the visual hand and the physics hand.
 */

#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/core/PhysicsInteractionTransformValidation.h"

#include "RockConfig.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/hand/skeleton/HandFrame.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/native/query/PhysicsUtils.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/native_anim/NativeIdleGripPreharvest.h"
#include "rock_support/Fo4VrRuntime.h"

namespace rock
{
    using namespace physics_interaction_detail;

    HandFrame PhysicsInteraction::getInteractionHandFrame(bool isLeft)
    {
        const bool cacheReady = _handBoneCache.isReady();
        auto* const driverNode =
            isLeft ?
            f4vr::getLeftHandNode() :
            f4vr::getRightHandNode();
        const std::size_t handIndex = isLeft ? 0u : 1u;
        if (_currentPreFrikSchedulerSequence == 0 ||
            _persistentFrikHandInputIsolationSequence[handIndex] !=
                _currentPreFrikSchedulerSequence) {
            _persistentFrikHandInputIsolationSequence[handIndex] =
                _currentPreFrikSchedulerSequence;
            const bool sampledPersistentWorldAuthority =
                frik_visual_authority::hasPublishedExternalHandWorldTransform(
                    frik_visual_authority::handFromBool(isLeft));
            if (_persistentFrikHandInputIsolationActive[handIndex] !=
                sampledPersistentWorldAuthority) {
                _persistentFrikHandInputIsolationActive[handIndex] =
                    sampledPersistentWorldAuthority;
                ROCK_LOG_INFO(Hand,
                    "{} collision-isolated controller input {} for persistent FRIK V2 hand authority",
                    isLeft ? "Left" : "Right",
                    sampledPersistentWorldAuthority ? "engaged" : "released");
            }
        }
        const bool persistentWorldAuthorityPublished =
            _persistentFrikHandInputIsolationActive[handIndex];
        const HandFrame frame = _handFrameResolver.resolve(
            isLeft,
            cacheReady,
            cacheReady ?
                _handBoneCache.getWorldTransform(isLeft) :
                RE::NiTransform{},
            cacheReady ? _handBoneCache.getSkeleton() : nullptr,
            cacheReady ? _handBoneCache.getBoneTree() : nullptr,
            persistentWorldAuthorityPublished,
            driverNode != nullptr,
            driverNode ? driverNode->world : RE::NiTransform{});
        const auto physicalHand =
            frik_visual_authority::handFromBool(isLeft);
        const bool handWorldPublicationReady =
            _handFrameResolver.hasControllerReconstructionCalibration(
                isLeft,
                _handBoneCache.getSkeleton(),
                _handBoneCache.getBoneTree()) &&
            (!persistentWorldAuthorityPublished || frame.valid);
        const bool handWorldPublicationReadinessChanged =
            frik_visual_authority::isExternalHandWorldPublicationReady(
                physicalHand) != handWorldPublicationReady;
        frik_visual_authority::setExternalHandWorldPublicationReady(
            physicalHand,
            handWorldPublicationReady);
        if (handWorldPublicationReadinessChanged) {
            ROCK_LOG_INFO(Hand,
                "{} persistent FRIK V2 hand publication {} after controller calibration",
                isLeft ? "Left" : "Right",
                handWorldPublicationReady ? "enabled" : "suspended");
        }
        if (persistentWorldAuthorityPublished && !frame.valid) {
            ROCK_LOG_SAMPLE_WARN(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} persistent FRIK V2 hand authority failed closed because controller reconstruction is unavailable cacheReady={} driverReady={}",
                isLeft ? "Left" : "Right",
                cacheReady ? "yes" : "no",
                driverNode ? "yes" : "no");
        }
        return frame;
    }

    RE::NiTransform PhysicsInteraction::getInteractionHandTransform(bool isLeft)
    {
        const auto frame = getInteractionHandFrame(isLeft);
        if (frame.valid) {
            return frame.transform;
        }

        return RE::NiTransform();
    }

    void PhysicsInteraction::sampleNativeReloadHandAuthorityBeforeFrik()
    {
        const auto* equippedWeaponData =
            physics_interaction_detail::getValidatedEquippedWeaponData();
        const std::uint64_t reloadDispatchSequence =
            input_remap_runtime::nativeReloadDispatchSequence();
        const std::uint32_t gunState =
            f4vr::getNativeGunState(f4vr::getPlayer());
        const bool magazineCountKnown = equippedWeaponData != nullptr;
        const bool magazineEmpty =
            equippedWeaponData && equippedWeaponData->ammoCount == 0;
        const bool dispatchEdge =
            reloadDispatchSequence != 0 &&
            reloadDispatchSequence !=
                _nativeReloadHandAuthorityState.observedReloadDispatchSequence;
        const bool wasActive = _nativeReloadHandAuthorityActive;
        const std::uint64_t weaponGenerationKey =
            _weaponCollision.getCurrentWeaponGenerationKey();
        const std::uint64_t frameIndex =
            runtime_state::currentFrame().frameIndex;
        _nativeReloadHandAuthorityActive =
            native_reload_hand_authority_policy::update(
                _nativeReloadHandAuthorityState,
                native_reload_hand_authority_policy::Input{
                    .weaponGenerationKey = weaponGenerationKey,
                    .frameIndex = frameIndex,
                    .reloadDispatchSequence = reloadDispatchSequence,
                    .gunState = gunState,
                    .magazineCountKnown = magazineCountKnown,
                    .magazineEmpty = magazineEmpty,
                });
        if (wasActive != _nativeReloadHandAuthorityActive) {
            ROCK_LOG_INFO(
                Weapon,
                "Native reload hand authority edge active={} generation={:016X} frame={} gunState={} dispatch(sequence/edge)={}/{} magazine(known/count/empty)={}/{}/{}",
                _nativeReloadHandAuthorityActive,
                weaponGenerationKey,
                frameIndex,
                gunState,
                reloadDispatchSequence,
                dispatchEdge,
                magazineCountKnown,
                equippedWeaponData ? equippedWeaponData->ammoCount : 0,
                magazineEmpty);
        }
    }

    void PhysicsInteraction::refreshExternalHandWorldTransformsBeforeFrik(
        const std::uint64_t schedulerSequence)
    {
        _currentPreFrikSchedulerSequence = schedulerSequence;
        if (!_initialized.load(std::memory_order_acquire)) {
            _nativeReloadHandAuthorityActive = false;
            return;
        }

        sampleNativeReloadHandAuthorityBeforeFrik();

        const auto captureWand = [](RE::NiNode* node,
                                    RE::NiTransform& outWorld) {
            outWorld = {};
            if (!node || !finiteNiTransform(node->world)) {
                return false;
            }
            outWorld = node->world;
            return true;
        };

        RE::NiTransform rightWandWorld{};
        RE::NiTransform leftWandWorld{};
        const bool rightWandValid = captureWand(
            f4vr::getRightHandNode(),
            rightWandWorld);
        const bool leftWandValid = captureWand(
            f4vr::getLeftHandNode(),
            leftWandWorld);

        RE::NiTransform rightRawHandWorld{};
        RE::NiTransform leftRawHandWorld{};
        const bool rightRawHandValid =
            rightWandValid &&
            _handFrameResolver.tryReconstructCalibratedHand(
                false,
                _handBoneCache.getSkeleton(),
                _handBoneCache.getBoneTree(),
                rightWandWorld,
                rightRawHandWorld);
        const bool leftRawHandValid =
            leftWandValid &&
            _handFrameResolver.tryReconstructCalibratedHand(
                true,
                _handBoneCache.getSkeleton(),
                _handBoneCache.getBoneTree(),
                leftWandWorld,
                leftRawHandWorld);

        _rightHand.refreshGrabVisualAuthorityBeforeFrik(
            schedulerSequence,
            rightRawHandValid,
            rightRawHandWorld);
        _leftHand.refreshGrabVisualAuthorityBeforeFrik(
            schedulerSequence,
            leftRawHandValid,
            leftRawHandWorld);
        _equippedWeaponTransition.refreshHandVisualAuthorityBeforeFrik(
            schedulerSequence);

        auto* const preFrikBhkWorld = getPlayerBhkWorld();
        auto* const preFrikHknpWorld = preFrikBhkWorld ?
            getHknpWorld(preFrikBhkWorld) :
            nullptr;
        _dynamicHandCollision.refreshContactVisualAuthorityBeforeFrik(
            preFrikHknpWorld,
            schedulerSequence,
            rightRawHandValid,
            rightRawHandWorld,
            leftRawHandValid,
            leftRawHandWorld,
            g_rockConfig.
                rockHandCollisionDynamicDivergenceTeleportGameUnits);

        // Deferred FRIK hand requests must be transported by the physical
        // controller-derived hand, not by weapon-offset nodes. Hunting/combat
        // rifle firing animations rotate those weapon nodes toward their
        // native angled pose; feeding that rotation back here makes ROCK's
        // authored weapon alignment latch to the native angle.
        const EquippedWeaponScopeHandDriverFrame leftWeaponHandDriver =
            { leftRawHandValid, leftRawHandWorld };
        const EquippedWeaponScopeHandDriverFrame rightWeaponHandDriver =
            { rightRawHandValid, rightRawHandWorld };

        const bool firingHandIsLeft = _twoHandedGrip.isFiringHandLeft();
        _twoHandedGrip.setNativeReloadHandAuthorityActive(
            _nativeReloadHandAuthorityActive);
        _twoHandedGrip.refreshRetainedHandVisualAuthoritiesBeforeFrik(
            leftWeaponHandDriver,
            rightWeaponHandDriver,
            _weaponCollision.getCurrentWeaponGenerationKey(),
            firingHandIsLeft,
            schedulerSequence);
        _twoHandedGrip.refreshWeaponCollisionHandAuthorityBeforeFrik(
            leftWeaponHandDriver,
            rightWeaponHandDriver,
            _weaponCollision.getCurrentWeaponGenerationKey(),
            firingHandIsLeft,
            schedulerSequence);
        _twoHandedGrip.captureFiringRecoilReferenceBeforeFrik(
            firingHandIsLeft ?
                leftWeaponHandDriver :
                rightWeaponHandDriver,
            _weaponCollision.getCurrentWeaponGenerationKey(),
            firingHandIsLeft,
            schedulerSequence);
        _twoHandedGrip.captureIndependentWeaponPresentationBeforeFrik(
            resolveEquippedWeaponInteractionNode(),
            _weaponCollision.getCurrentWeaponGenerationKey(),
            schedulerSequence,
            leftWeaponHandDriver,
            rightWeaponHandDriver);
    }

    void PhysicsInteraction::restoreIndependentWeaponPresentationAfterFrik(
        const std::uint64_t schedulerSequence)
    {
        if (!_initialized.load(std::memory_order_acquire)) {
            return;
        }

        (void)_twoHandedGrip.
            restoreIndependentWeaponPresentationAfterFrik(
                resolveEquippedWeaponInteractionNode(),
                _weaponCollision.getCurrentWeaponGenerationKey(),
                schedulerSequence);
    }

}
