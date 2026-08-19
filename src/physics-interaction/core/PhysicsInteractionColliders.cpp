#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"

#include <algorithm>

#include "RockConfig.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/debug/overlay/DebugBodyOverlay.h"
#include "physics-interaction/grab/CustomOGA.h"
#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/native/havok/HavokOffsets.h"
#include "physics-interaction/native/havok/HavokRuntime.h"
#include "physics-interaction/native/query/PhysicsRecursiveWrappers.h"
#include "physics-interaction/native/query/PhysicsUtils.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "rock_support/Fo4VrRuntime.h"

namespace rock
{
    using namespace physics_interaction_detail;

    void PhysicsInteraction::serviceCollisionLayerDrift(
        RE::hknpWorld* hknp)
    {
        if (_collisionLayerRegistered &&
            (_expectedHandLayerMask != 0 || _expectedWeaponLayerMask != 0 || _expectedReloadLayerMask != 0 || _expectedBodyLayerMask != 0 ||
                _expectedDynamicHandProxyLayerMask != 0 || _expectedDynamicWeaponProxyLayerMask != 0 ||
                _expectedDynamicWorldCarClutterLayerMask != 0 || _expectedDynamicWorldCarLargeClutterLayerMask != 0 ||
                _nativeCharacterControllerLayerPolicyCaptured)) {
            const auto desiredHandMask = collision_layer_policy::buildRockHandExpectedMask(true, g_rockConfig.rockHandCollisionStaticWorldEnabled);
            const auto desiredWeaponMask = collision_layer_policy::buildRockWeaponExpectedMask(
                g_rockConfig.rockWeaponCollisionBlocksProjectiles,
                g_rockConfig.rockWeaponCollisionBlocksSpells,
                g_rockConfig.rockWeaponCollisionStaticWorldEnabled,
                true);
            const auto desiredReloadMask = collision_layer_policy::buildRockReloadExpectedMask(
                g_rockConfig.rockWeaponCollisionBlocksProjectiles,
                g_rockConfig.rockWeaponCollisionBlocksSpells,
                g_rockConfig.rockHandCollisionStaticWorldEnabled);
            const auto desiredBodyMask = collision_layer_policy::buildRockBodyExpectedMask(g_rockConfig.rockBodyBoneCollisionStaticWorldEnabled);
            const bool desiredDynamicProxyNpcBodyCollision = dynamicProxyNpcBodyCollisionEnabled();
            const auto desiredDynamicRightHandProxyMask =
                collision_layer_policy::buildRockDynamicHandProxyExpectedMask(
                    false,
                    g_rockConfig.rockHandDynamicInteractionsEnabled,
                    _dynamicWeaponRightHandInteractionEnabled,
                    desiredDynamicProxyNpcBodyCollision);
            const auto desiredDynamicLeftHandProxyMask =
                collision_layer_policy::buildRockDynamicHandProxyExpectedMask(
                    true,
                    g_rockConfig.rockHandDynamicInteractionsEnabled,
                    _dynamicWeaponLeftHandInteractionEnabled,
                    desiredDynamicProxyNpcBodyCollision);
            const auto desiredDynamicWeaponProxyMask =
                collision_layer_policy::buildRockDynamicWeaponProxyExpectedMask(
                    g_rockConfig.rockHandDynamicInteractionsEnabled,
                    _dynamicWeaponRightHandInteractionEnabled,
                    _dynamicWeaponLeftHandInteractionEnabled,
                    desiredDynamicProxyNpcBodyCollision);
            const bool desiredNativeControllerPolicyEnabled = g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled;
            const bool nativeControllerPolicyModeChanged =
                _nativeCharacterControllerLayerPolicyCaptured &&
                _nativeCharacterControllerLayerPolicyEnabled != desiredNativeControllerPolicyEnabled;
            if (!collision_layer_policy::matrixLayerMaskMatches(_expectedHandLayerMask, desiredHandMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(_expectedWeaponLayerMask, desiredWeaponMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(_expectedReloadLayerMask, desiredReloadMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(_expectedBodyLayerMask, desiredBodyMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(
                    _expectedDynamicHandProxyLayerMask,
                    desiredDynamicRightHandProxyMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(
                    _expectedDynamicLeftHandProxyLayerMask,
                    desiredDynamicLeftHandProxyMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(
                    _expectedDynamicWeaponProxyLayerMask,
                    desiredDynamicWeaponProxyMask) ||
                nativeControllerPolicyModeChanged) {
                ROCK_LOG_INFO(Config, "ROCK collision layer config changed; re-registering matrix policy");
                _collisionLayerRegistered = false;
                registerCollisionLayer(hknp);
            }

            if (auto* matrix = havok_runtime::getCollisionFilterMatrix(hknp)) {
                const auto currentHandMask = matrix[collision_layer_policy::ROCK_LAYER_HAND];
                const auto currentWeaponMask = matrix[collision_layer_policy::ROCK_LAYER_WEAPON];
                const auto currentReloadMask = matrix[collision_layer_policy::ROCK_LAYER_RELOAD];
                const auto currentBodyMask = matrix[collision_layer_policy::ROCK_LAYER_BODY];
                const auto currentDynamicHandProxyMask = matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_HAND_PROXY];
                const auto currentDynamicLeftHandProxyMask = matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY];
                const auto currentDynamicWeaponProxyMask = matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY];
                const auto currentDynamicWorldCarClutterMask = matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER];
                const auto currentDynamicWorldCarLargeClutterMask = matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER];
                const bool handMaskDrifted = _expectedHandLayerMask != 0 && !collision_layer_policy::matrixLayerMaskMatches(currentHandMask, _expectedHandLayerMask);
                const bool weaponMaskDrifted = _expectedWeaponLayerMask != 0 && !collision_layer_policy::matrixLayerMaskMatches(currentWeaponMask, _expectedWeaponLayerMask);
                const bool reloadMaskDrifted = _expectedReloadLayerMask != 0 && !collision_layer_policy::matrixLayerMaskMatches(currentReloadMask, _expectedReloadLayerMask);
                const bool bodyMaskDrifted = _expectedBodyLayerMask != 0 && !collision_layer_policy::bodyManagedLayerMaskMatches(currentBodyMask, _expectedBodyLayerMask);
                /*
                 * Proxy rows use the biped-tolerant compare: the BIPED /
                 * BIPED_NO_CC rows are SCISSORS-owned while its global
                 * collision policy runs, and fighting over those bits would
                 * churn the whole matrix (see dynamicProxyNpcBodyLayerBits).
                 */
                const bool dynamicHandProxyMaskDrifted = _expectedDynamicHandProxyLayerMask != 0 &&
                    !collision_layer_policy::bodyManagedLayerMaskMatches(currentDynamicHandProxyMask, _expectedDynamicHandProxyLayerMask);
                const bool dynamicLeftHandProxyMaskDrifted =
                    _expectedDynamicLeftHandProxyLayerMask != 0 &&
                    !collision_layer_policy::bodyManagedLayerMaskMatches(
                        currentDynamicLeftHandProxyMask,
                        _expectedDynamicLeftHandProxyLayerMask);
                const bool dynamicWeaponProxyMaskDrifted = _expectedDynamicWeaponProxyLayerMask != 0 &&
                    !collision_layer_policy::bodyManagedLayerMaskMatches(currentDynamicWeaponProxyMask, _expectedDynamicWeaponProxyLayerMask);
                const bool dynamicWorldCarClutterMaskDrifted = _expectedDynamicWorldCarClutterLayerMask != 0 &&
                    !collision_layer_policy::matrixLayerMaskMatches(currentDynamicWorldCarClutterMask, _expectedDynamicWorldCarClutterLayerMask);
                const bool dynamicWorldCarLargeClutterMaskDrifted = _expectedDynamicWorldCarLargeClutterLayerMask != 0 &&
                    !collision_layer_policy::matrixLayerMaskMatches(currentDynamicWorldCarLargeClutterMask, _expectedDynamicWorldCarLargeClutterLayerMask);
                const bool actorToolPairsDrifted =
                    _expectedHandLayerMask != 0 && _expectedWeaponLayerMask != 0 &&
                    !collision_layer_policy::rockToolActorPairsMatch(matrix, _expectedHandLayerMask, _expectedWeaponLayerMask);
                const bool bodyPairsDrifted = _expectedBodyLayerMask != 0 && !collision_layer_policy::rockBodyManagedPairsMatch(matrix, _expectedBodyLayerMask);
                const bool nativeControllerObjectPairsDrifted =
                    _nativeCharacterControllerLayerPolicyCaptured &&
                    !collision_layer_policy::nativeCharacterControllerObjectPairsMatch(matrix, _expectedNativeCharacterControllerLayerMask);
                if (handMaskDrifted || weaponMaskDrifted || reloadMaskDrifted || bodyMaskDrifted || dynamicHandProxyMaskDrifted || dynamicLeftHandProxyMaskDrifted || dynamicWeaponProxyMaskDrifted ||
                    dynamicWorldCarClutterMaskDrifted || dynamicWorldCarLargeClutterMaskDrifted || actorToolPairsDrifted || bodyPairsDrifted ||
                    nativeControllerObjectPairsDrifted) {
                    const auto currentNativeCharacterControllerMask =
                        _nativeCharacterControllerLayerPolicyCaptured ? matrix[collision_layer_policy::FO4_LAYER_CHARCONTROLLER] : 0;
                    ROCK_LOG_WARN(Config,
                        "ROCK configured layer mask drift detected; hand expected=0x{:016X} current=0x{:016X}, weapon expected=0x{:016X} current=0x{:016X}, reload expected=0x{:016X} current=0x{:016X}, body expected=0x{:016X} current=0x{:016X}, dynamicRightHandProxy expected=0x{:016X} current=0x{:016X}, dynamicLeftHandProxy expected=0x{:016X} current=0x{:016X}, dynamicWeaponProxy expected=0x{:016X} current=0x{:016X}, carClutter expected=0x{:016X} current=0x{:016X}, carLarge expected=0x{:016X} current=0x{:016X}, nativeController expected=0x{:016X} current=0x{:016X}, actorToolPairs={}, bodyManagedPairs={}, nativeControllerObjects={}; re-registering",
                        collision_layer_policy::matrixAddressableMask(_expectedHandLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentHandMask),
                        collision_layer_policy::matrixAddressableMask(_expectedWeaponLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentWeaponMask),
                        collision_layer_policy::matrixAddressableMask(_expectedReloadLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentReloadMask),
                        collision_layer_policy::matrixAddressableMask(_expectedBodyLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentBodyMask),
                        collision_layer_policy::matrixAddressableMask(_expectedDynamicHandProxyLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentDynamicHandProxyMask),
                        collision_layer_policy::matrixAddressableMask(_expectedDynamicLeftHandProxyLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentDynamicLeftHandProxyMask),
                        collision_layer_policy::matrixAddressableMask(_expectedDynamicWeaponProxyLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentDynamicWeaponProxyMask),
                        collision_layer_policy::matrixAddressableMask(_expectedDynamicWorldCarClutterLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentDynamicWorldCarClutterMask),
                        collision_layer_policy::matrixAddressableMask(_expectedDynamicWorldCarLargeClutterLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentDynamicWorldCarLargeClutterMask),
                        collision_layer_policy::matrixAddressableMask(_expectedNativeCharacterControllerLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentNativeCharacterControllerMask),
                        actorToolPairsDrifted ? "drifted" : "ok",
                        bodyPairsDrifted ? "drifted" : "ok",
                        nativeControllerObjectPairsDrifted ? "drifted" : "ok");
                    _collisionLayerRegistered = false;
                    registerCollisionLayer(hknp);
                }
            }
        }
    }


    void PhysicsInteraction::synchronizeDynamicWeaponHandCollisionRoles(
        RE::hknpWorld* world)
    {
        const auto attachedHands =
            _twoHandedGrip.weaponCollisionAttachedHands();
        const bool rightHandInteractionEnabled =
            !attachedHands.right;
        const bool leftHandInteractionEnabled =
            !attachedHands.left;
        if (_dynamicWeaponRightHandInteractionEnabled ==
                rightHandInteractionEnabled &&
            _dynamicWeaponLeftHandInteractionEnabled ==
                leftHandInteractionEnabled) {
            return;
        }

        _dynamicWeaponRightHandInteractionEnabled =
            rightHandInteractionEnabled;
        _dynamicWeaponLeftHandInteractionEnabled =
            leftHandInteractionEnabled;

        ROCK_LOG_DEBUG(Weapon,
            "Dynamic weapon hand collision roles changed: right={} left={}",
            rightHandInteractionEnabled ? "free" : "attached",
            leftHandInteractionEnabled ? "free" : "attached");
        _collisionLayerRegistered = false;
        registerCollisionLayer(world);
    }

    bool PhysicsInteraction::dynamicProxyNpcBodyCollisionEnabled()
    {
        /*
         * Fail closed: NPC body contact is only safe while the native
         * character-controller contact filter is on, because that toggle owns
         * the bit-14 suppression leases that exclude the player's own
         * biped-family bodies from the dynamic proxies. Without those leases
         * the driven proxies would collide with the player's own skeleton
         * (self-propulsion feedback).
         */
        return g_rockConfig.rockDynamicColliderNpcBodyCollisionEnabled &&
               g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled;
    }

    void PhysicsInteraction::registerCollisionLayer(RE::hknpWorld* world)
    {
        if (!world) {
            ROCK_LOG_ERROR(Config, "registerCollisionLayer: world is null");
            return;
        }

        bool usedFilterFallback = false;
        auto* matrix = havok_runtime::getCollisionFilterMatrix(world, &usedFilterFallback);
        if (!matrix) {
            ROCK_LOG_ERROR(Config, "Both world filter and global singleton are null — cannot configure layer");
            return;
        }
        ROCK_LOG_DEBUG(Config, "Filter source: matrix={:p}, usedFallback={}", static_cast<const void*>(matrix), usedFilterFallback ? "yes" : "no");

        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", collision_layer_policy::ROCK_LAYER_HAND, matrix[collision_layer_policy::ROCK_LAYER_HAND]);
        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", collision_layer_policy::ROCK_LAYER_WEAPON, matrix[collision_layer_policy::ROCK_LAYER_WEAPON]);
        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", collision_layer_policy::ROCK_LAYER_RELOAD, matrix[collision_layer_policy::ROCK_LAYER_RELOAD]);
        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", collision_layer_policy::ROCK_LAYER_BODY, matrix[collision_layer_policy::ROCK_LAYER_BODY]);
        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", collision_layer_policy::ROCK_LAYER_DYNAMIC_HAND_PROXY, matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_HAND_PROXY]);
        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", collision_layer_policy::ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY, matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY]);
        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY, matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY]);
        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER, matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER]);
        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER, matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER]);
        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", collision_layer_policy::FO4_LAYER_CHARCONTROLLER, matrix[collision_layer_policy::FO4_LAYER_CHARCONTROLLER]);

        if (!_nativeCharacterControllerLayerPolicyCaptured) {
            _originalNativeCharacterControllerLayerMask = matrix[collision_layer_policy::FO4_LAYER_CHARCONTROLLER];
            _nativeCharacterControllerLayerPolicyCaptured = true;
        }

        const bool npcBodyCollisionEnabled = dynamicProxyNpcBodyCollisionEnabled();
        if (g_rockConfig.rockDynamicColliderNpcBodyCollisionEnabled && !npcBodyCollisionEnabled) {
            ROCK_LOG_WARN(Config,
                "Dynamic proxy NPC body collision requested but disabled: native character-controller contact filter is off, so player biped suppression leases are inactive");
        }
        collision_layer_policy::applyRockGeneratedLayerPolicies(
            matrix,
            g_rockConfig.rockHandCollisionStaticWorldEnabled,
            g_rockConfig.rockWeaponCollisionStaticWorldEnabled,
            g_rockConfig.rockBodyBoneCollisionStaticWorldEnabled,
            g_rockConfig.rockWeaponCollisionBlocksProjectiles,
            g_rockConfig.rockWeaponCollisionBlocksSpells,
            g_rockConfig.rockHandDynamicInteractionsEnabled,
            _dynamicWeaponRightHandInteractionEnabled,
            _dynamicWeaponLeftHandInteractionEnabled,
            npcBodyCollisionEnabled);
        collision_layer_policy::applyNativeCharacterControllerObjectSuppressionPolicy(
            matrix,
            g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled,
            _originalNativeCharacterControllerLayerMask);

        _expectedHandLayerMask = collision_layer_policy::buildRockHandExpectedMask(true, g_rockConfig.rockHandCollisionStaticWorldEnabled);
        _expectedWeaponLayerMask =
            collision_layer_policy::buildRockWeaponExpectedMask(
                g_rockConfig.rockWeaponCollisionBlocksProjectiles,
                g_rockConfig.rockWeaponCollisionBlocksSpells,
                g_rockConfig.rockWeaponCollisionStaticWorldEnabled,
                true);
        _expectedReloadLayerMask =
            collision_layer_policy::buildRockReloadExpectedMask(
                g_rockConfig.rockWeaponCollisionBlocksProjectiles,
                g_rockConfig.rockWeaponCollisionBlocksSpells,
                g_rockConfig.rockHandCollisionStaticWorldEnabled);
        _expectedBodyLayerMask = collision_layer_policy::buildRockBodyExpectedMask(g_rockConfig.rockBodyBoneCollisionStaticWorldEnabled);
        _expectedDynamicHandProxyLayerMask =
            collision_layer_policy::buildRockDynamicHandProxyExpectedMask(
                false,
                g_rockConfig.rockHandDynamicInteractionsEnabled,
                _dynamicWeaponRightHandInteractionEnabled,
                npcBodyCollisionEnabled);
        _expectedDynamicLeftHandProxyLayerMask =
            collision_layer_policy::buildRockDynamicHandProxyExpectedMask(
                true,
                g_rockConfig.rockHandDynamicInteractionsEnabled,
                _dynamicWeaponLeftHandInteractionEnabled,
                npcBodyCollisionEnabled);
        _expectedDynamicWeaponProxyLayerMask =
            collision_layer_policy::buildRockDynamicWeaponProxyExpectedMask(
                g_rockConfig.rockHandDynamicInteractionsEnabled,
                _dynamicWeaponRightHandInteractionEnabled,
                _dynamicWeaponLeftHandInteractionEnabled,
                npcBodyCollisionEnabled);
        ROCK_LOG_INFO(Config,
            "Dynamic proxy NPC body collision {}: rightHand=0x{:016X} leftHand=0x{:016X} weapon=0x{:016X}",
            npcBodyCollisionEnabled ? "ENABLED (biped family 8/32/33 paired)" : "disabled",
            matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_HAND_PROXY],
            matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY],
            matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY]);
        _expectedDynamicWorldCarClutterLayerMask = matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER];
        _expectedDynamicWorldCarLargeClutterLayerMask = matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER];
        _expectedNativeCharacterControllerLayerMask =
            collision_layer_policy::nativeCharacterControllerExpectedMask(
                _originalNativeCharacterControllerLayerMask,
                g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled);
        _nativeCharacterControllerLayerPolicyEnabled = g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled;
        _collisionLayerRegistered = true;

        const bool nativeControllerObjectPairsMatch =
            collision_layer_policy::nativeCharacterControllerObjectPairsMatch(matrix, _expectedNativeCharacterControllerLayerMask);
        const char* nativeControllerObjectStatus =
            _nativeCharacterControllerLayerPolicyEnabled ?
                (nativeControllerObjectPairsMatch ? "suppressed" : "bad") :
                (nativeControllerObjectPairsMatch ? "restored" : "bad");

        ROCK_LOG_INFO(Config,
            "Registered ROCK collision layers: hand={} mask=0x{:016X}, weapon={} mask=0x{:016X}, reload={} mask=0x{:016X}, body={} mask=0x{:016X}, actorPairs(biped={},deadbip={},bipedNoCC={}), bodyPairs(hand={},weapon={},self={},static={},animstatic={},clutter={},query={},charController={}), handStaticWorld={}, weaponStaticWorld={}, bodyStaticWorld={}, projectiles={}, spells={}, nativeBubbleObjects={}",
            collision_layer_policy::ROCK_LAYER_HAND,
            matrix[collision_layer_policy::ROCK_LAYER_HAND],
            collision_layer_policy::ROCK_LAYER_WEAPON,
            matrix[collision_layer_policy::ROCK_LAYER_WEAPON],
            collision_layer_policy::ROCK_LAYER_RELOAD,
            matrix[collision_layer_policy::ROCK_LAYER_RELOAD],
            collision_layer_policy::ROCK_LAYER_BODY,
            matrix[collision_layer_policy::ROCK_LAYER_BODY],
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_HAND,
                collision_layer_policy::FO4_LAYER_BIPED,
                collision_layer_policy::maskEnablesLayer(_expectedHandLayerMask, collision_layer_policy::FO4_LAYER_BIPED)) &&
                    collision_layer_policy::layerPairSymmetricMatches(
                        matrix,
                        collision_layer_policy::ROCK_LAYER_WEAPON,
                        collision_layer_policy::FO4_LAYER_BIPED,
                        collision_layer_policy::maskEnablesLayer(_expectedWeaponLayerMask, collision_layer_policy::FO4_LAYER_BIPED)) ?
                "ok" :
                "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_HAND,
                collision_layer_policy::FO4_LAYER_DEADBIP,
                collision_layer_policy::maskEnablesLayer(_expectedHandLayerMask, collision_layer_policy::FO4_LAYER_DEADBIP)) &&
                    collision_layer_policy::layerPairSymmetricMatches(
                        matrix,
                        collision_layer_policy::ROCK_LAYER_WEAPON,
                        collision_layer_policy::FO4_LAYER_DEADBIP,
                        collision_layer_policy::maskEnablesLayer(_expectedWeaponLayerMask, collision_layer_policy::FO4_LAYER_DEADBIP)) ?
                "ok" :
                "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_HAND,
                collision_layer_policy::FO4_LAYER_BIPED_NO_CC,
                collision_layer_policy::maskEnablesLayer(_expectedHandLayerMask, collision_layer_policy::FO4_LAYER_BIPED_NO_CC)) &&
                    collision_layer_policy::layerPairSymmetricMatches(
                        matrix,
                        collision_layer_policy::ROCK_LAYER_WEAPON,
                        collision_layer_policy::FO4_LAYER_BIPED_NO_CC,
                        collision_layer_policy::maskEnablesLayer(_expectedWeaponLayerMask, collision_layer_policy::FO4_LAYER_BIPED_NO_CC)) ?
                "ok" :
                "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_BODY,
                collision_layer_policy::ROCK_LAYER_HAND,
                collision_layer_policy::maskEnablesLayer(_expectedBodyLayerMask, collision_layer_policy::ROCK_LAYER_HAND)) ? "ok" : "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_BODY,
                collision_layer_policy::ROCK_LAYER_WEAPON,
                collision_layer_policy::maskEnablesLayer(_expectedBodyLayerMask, collision_layer_policy::ROCK_LAYER_WEAPON)) ? "ok" : "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_BODY,
                collision_layer_policy::ROCK_LAYER_BODY,
                collision_layer_policy::maskEnablesLayer(_expectedBodyLayerMask, collision_layer_policy::ROCK_LAYER_BODY)) ? "ok" : "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_BODY,
                collision_layer_policy::FO4_LAYER_STATIC,
                collision_layer_policy::maskEnablesLayer(_expectedBodyLayerMask, collision_layer_policy::FO4_LAYER_STATIC)) ? "ok" : "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_BODY,
                collision_layer_policy::FO4_LAYER_ANIMSTATIC,
                collision_layer_policy::maskEnablesLayer(_expectedBodyLayerMask, collision_layer_policy::FO4_LAYER_ANIMSTATIC)) ? "ok" : "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_BODY,
                collision_layer_policy::FO4_LAYER_CLUTTER,
                collision_layer_policy::maskEnablesLayer(_expectedBodyLayerMask, collision_layer_policy::FO4_LAYER_CLUTTER)) ? "ok" : "bad",
            !collision_layer_policy::layerPairEnabledFromRow(matrix, collision_layer_policy::ROCK_LAYER_BODY, collision_layer_policy::FO4_LAYER_ITEMPICK) &&
                    !collision_layer_policy::layerPairEnabledFromRow(matrix, collision_layer_policy::FO4_LAYER_ITEMPICK, collision_layer_policy::ROCK_LAYER_BODY) ?
                "ok" :
                "bad",
            !collision_layer_policy::layerPairEnabledFromRow(matrix, collision_layer_policy::ROCK_LAYER_BODY, collision_layer_policy::FO4_LAYER_CHARCONTROLLER) &&
                    !collision_layer_policy::layerPairEnabledFromRow(matrix, collision_layer_policy::FO4_LAYER_CHARCONTROLLER, collision_layer_policy::ROCK_LAYER_BODY) ?
                "ok" :
                "bad",
            g_rockConfig.rockHandCollisionStaticWorldEnabled ? "enabled" : "disabled",
            g_rockConfig.rockWeaponCollisionStaticWorldEnabled ? "enabled" : "disabled",
            g_rockConfig.rockBodyBoneCollisionStaticWorldEnabled ? "enabled" : "disabled",
            g_rockConfig.rockWeaponCollisionBlocksProjectiles ? "enabled" : "disabled",
            g_rockConfig.rockWeaponCollisionBlocksSpells ? "enabled" : "disabled",
            nativeControllerObjectStatus);
        ROCK_LOG_INFO(
            Config,
            "Registered dynamic weapon proxy layer={} mask=0x{:016X} hands(right={},left={})",
            collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY,
            collision_layer_policy::matrixAddressableMask(
                _expectedDynamicWeaponProxyLayerMask),
            _dynamicWeaponRightHandInteractionEnabled ? "free" : "attached",
            _dynamicWeaponLeftHandInteractionEnabled ? "free" : "attached");
    }

    bool PhysicsInteraction::createHandCollisions(RE::hknpWorld* world, void* bhkWorld)
    {
        if (!runtime_state::isLocalSkeletonReady()) {
            ROCK_LOG_ERROR(Hand, "Cannot create hand collisions — skeleton not ready");
            return false;
        }

        const HandFrame rightHandFrame = getInteractionHandFrame(false);
        const HandFrame leftHandFrame = getInteractionHandFrame(true);
        if (!rightHandFrame.valid || !leftHandFrame.valid) {
            ROCK_LOG_WARN(Hand,
                "Cannot create hand collisions: collision-isolated hand frame unavailable right={} left={}",
                rightHandFrame.valid ? "ready" : "missing",
                leftHandFrame.valid ? "ready" : "missing");
            return false;
        }

        const bool rightOk = _rightHand.createCollision(
            world,
            bhkWorld,
            rightHandFrame.transform);

        const bool leftOk = _leftHand.createCollision(
            world,
            bhkWorld,
            leftHandFrame.transform);

        if (!rightOk || !leftOk) {
            ROCK_LOG_ERROR(Hand, "Hand collision creation failed (rightOk={}, leftOk={})", rightOk, leftOk);
            if (rightOk)
                _rightHand.destroyCollision(bhkWorld);
            if (leftOk)
                _leftHand.destroyCollision(bhkWorld);
            return false;
        }

        ROCK_LOG_INFO(Hand,
            "Bone-derived hand collision created: rightBodies={} leftBodies={} mode={} requireAnchor={} requireAllFingerBones={}",
            _rightHand.getHandColliderBodyCount(),
            _leftHand.getHandColliderBodyCount(),
            g_rockConfig.rockHandColliderRuntimeMode,
            g_rockConfig.rockHandBoneCollidersRequirePalmAnchor ? "true" : "false",
            g_rockConfig.rockHandBoneCollidersRequireAllFingerBones ? "true" : "false");

        _handColliderCreateRetryFrames = 0;
        return true;
    }

    void PhysicsInteraction::destroyHandCollisions(void* bhkWorld)
    {
        auto* typedBhkWorld =
            static_cast<RE::bhkWorld*>(bhkWorld);
        auto* hknpWorld =
            typedBhkWorld ?
            getHknpWorld(typedBhkWorld) :
            nullptr;
        _touchGrabRuntime.releaseAll(
            typedBhkWorld,
            hknpWorld,
            provider::RockProviderTouchGrabReleaseReasonV1::
                GenerationChanged,
            _collisionGenerationAtomic.load(
                std::memory_order_acquire));
        clearGeneratedBodyContactRegistry();
        _rightHand.destroyCollision(bhkWorld);
        _leftHand.destroyCollision(bhkWorld);
        _handColliderCreateRetryFrames = 0;
    }

    void PhysicsInteraction::updateHandCollisions(const PhysicsFrameContext& frame)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::HandColliderUpdate);

        if (!runtime_state::isLocalSkeletonReady()) {
            return;
        }

        auto* world = frame.hknpWorld;

        if (!_rightHand.hasCollisionBody() || !_leftHand.hasCollisionBody()) {
            if (frame.reloadBoundaryActive) {
                return;
            }
            if (_handColliderCreateRetryFrames > 0) {
                --_handColliderCreateRetryFrames;
                return;
            }

            ROCK_LOG_WARN(Hand,
                "Hand collider runtime missing generated bodies; recreating rightBody={} leftBody={}",
                _rightHand.hasCollisionBody() ? "yes" : "no",
                _leftHand.hasCollisionBody() ? "yes" : "no");
            destroyHandCollisions(frame.bhkWorld);
            if (!createHandCollisions(frame.hknpWorld, frame.bhkWorld)) {
                _handColliderCreateRetryFrames = 120;
            }
            return;
        }

        _rightHand.updateDelayedGrabHandCollisionRestore(world, frame.deltaSeconds);
        _leftHand.updateDelayedGrabHandCollisionRestore(world, frame.deltaSeconds);
        updateEquippedWeaponPostDropCollisionSuppression(world, frame.deltaSeconds);

        if (!frame.right.disabled) {
            _rightHand.updateCollisionTransform(world, frame.right.rawHandWorld, frame.deltaSeconds);
        }
        if (!frame.left.disabled) {
            _leftHand.updateCollisionTransform(world, frame.left.rawHandWorld, frame.deltaSeconds);
        }
    }

    bool PhysicsInteraction::createBodyBoneCollisions(RE::hknpWorld* world, void* bhkWorld)
    {
        if (!g_rockConfig.rockBodyBoneCollidersEnabled) {
            _bodyBoneColliders.destroy(bhkWorld);
            return true;
        }

        if (!runtime_state::isLocalSkeletonReady()) {
            ROCK_LOG_WARN(Body, "Cannot create body bone colliders: skeleton not ready");
            return false;
        }

        if (!_bodyBoneColliders.create(world, bhkWorld)) {
            return false;
        }

        _bodyBoneColliderCreateRetryFrames = 0;
        _bodyContactRuntime.reset();
        ROCK_LOG_INFO(Body,
            "Body bone collider set created: bodies={} legsAndFeet={}",
            _bodyBoneColliders.getBodyCount(),
            g_rockConfig.rockBodyBoneLegAndFootCollidersEnabled ? "enabled" : "disabled");
        return true;
    }

    void PhysicsInteraction::destroyBodyBoneCollisions(void* bhkWorld)
    {
        clearGeneratedBodyContactRegistry();
        _bodyBoneColliders.destroy(bhkWorld);
        _bodyContactRuntime.reset();
        _bodyBoneColliderCreateRetryFrames = 0;
    }

    void PhysicsInteraction::updateBodyBoneCollisions(const PhysicsFrameContext& frame)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::BodyColliderUpdate);

        if (!runtime_state::isLocalSkeletonReady()) {
            return;
        }

        if (!g_rockConfig.rockBodyBoneCollidersEnabled) {
            if (_bodyBoneColliders.hasBodies()) {
                ROCK_LOG_INFO(Body, "Body bone collider config disabled; destroying generated body set");
                destroyBodyBoneCollisions(frame.bhkWorld);
                _bodyContactRuntime.reset();
            }
            return;
        }

        if (!_bodyBoneColliders.hasBodies()) {
            if (frame.reloadBoundaryActive) {
                return;
            }
            if (_bodyBoneColliderCreateRetryFrames > 0) {
                --_bodyBoneColliderCreateRetryFrames;
                return;
            }

            if (!createBodyBoneCollisions(frame.hknpWorld, frame.bhkWorld)) {
                _bodyBoneColliderCreateRetryFrames = 120;
            }
            return;
        }

        _bodyBoneColliders.update(frame.hknpWorld, frame.deltaSeconds);
    }

}
