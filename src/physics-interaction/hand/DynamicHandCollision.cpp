#include "physics-interaction/hand/DynamicHandCollision.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/native/HavokMaterialRegistry.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokRefCount.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"

#include <algorithm>
#include <cmath>

namespace rock
{
    namespace
    {
        constexpr const char* RIGHT_DYNAMIC_HAND_TAG = "ROCK_DynamicHand_Right";
        constexpr const char* LEFT_DYNAMIC_HAND_TAG = "ROCK_DynamicHand_Left";
        constexpr char RIGHT_PROXY_BODY_NAME[] = "ROCK_DynamicHandProxy_R";
        constexpr char LEFT_PROXY_BODY_NAME[] = "ROCK_DynamicHandProxy_L";
        constexpr std::uint32_t kDynamicHandProxyCollisionGroup = 0x000C;

        const char* dynamicHandTag(bool isLeft)
        {
            return isLeft ? LEFT_DYNAMIC_HAND_TAG : RIGHT_DYNAMIC_HAND_TAG;
        }

        std::size_t handIndex(bool isLeft)
        {
            return isLeft ? 1u : 0u;
        }

        std::uint32_t dynamicHandProxyFilterInfo()
        {
            return (kDynamicHandProxyCollisionGroup << 16) |
                   (collision_layer_policy::ROCK_LAYER_DYNAMIC_HAND_PROXY & collision_layer_policy::FO4_LAYER_FILTER_MASK);
        }

        bool isFinitePoint(const RE::NiPoint3& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
        }

        bool isFiniteTransform(const RE::NiTransform& transform)
        {
            if (!isFinitePoint(transform.translate) || !std::isfinite(transform.scale)) {
                return false;
            }
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(transform.rotate.entry[row][column])) {
                        return false;
                    }
                }
            }
            return true;
        }

        /*
         * Sphere center in raw-hand local space so the proxy covers the palm
         * volume (grab anchor) instead of the wrist-side hand bone origin.
         * world = rotate * local + translate, so local = rotate^T * (world - t).
         */
        RE::NiPoint3 handLocalPoint(const RE::NiTransform& handWorld, const RE::NiPoint3& worldPoint)
        {
            const RE::NiPoint3 delta{
                worldPoint.x - handWorld.translate.x,
                worldPoint.y - handWorld.translate.y,
                worldPoint.z - handWorld.translate.z,
            };
            const auto& m = handWorld.rotate.entry;
            return RE::NiPoint3{
                m[0][0] * delta.x + m[1][0] * delta.y + m[2][0] * delta.z,
                m[0][1] * delta.x + m[1][1] * delta.y + m[2][1] * delta.z,
                m[0][2] * delta.x + m[1][2] * delta.y + m[2][2] * delta.z,
            };
        }

        RE::hknpShape* buildProxySphereShape(const RE::NiPoint3& localCenterGame, float radiusGame)
        {
            using CreateSphereShape_t = RE::hknpShape* (*)(RE::hkVector4f*, float);
            static REL::Relocation<CreateSphereShape_t> createSphere{ REL::Offset(offsets::kFunc_CreateSphereShape) };

            const float scale = gameToHavokScale();
            RE::hkVector4f centerHavok{
                localCenterGame.x * scale,
                localCenterGame.y * scale,
                localCenterGame.z * scale,
                0.0f,
            };
            return createSphere(&centerHavok, radiusGame * scale);
        }
    }

    bool DynamicHandCollisionRuntime::ensureCreated(HandSlot& slot, bool isLeft, const PhysicsFrameContext& frame, const HandFrameInput& handInput)
    {
        if (slot.created) {
            if (slot.createdWorld == frame.hknpWorld && !slot.rebuildRequestedAtomic.load(std::memory_order_acquire)) {
                return true;
            }
            retireSlot(slot, frame.bhkWorld, isLeft);
        }

        if (!frame.hknpWorld || !frame.bhkWorld ||
            !isFiniteTransform(handInput.rawHandWorld) || !isFinitePoint(handInput.grabAnchorWorld)) {
            return false;
        }

        const RE::NiPoint3 localCenter = handLocalPoint(handInput.rawHandWorld, handInput.grabAnchorWorld);
        if (!isFinitePoint(localCenter)) {
            return false;
        }

        auto* shape = buildProxySphereShape(localCenter, g_rockConfig.rockHandCollisionDynamicProxyRadiusGameUnits);
        if (!shape) {
            ROCK_LOG_SAMPLE_WARN(Hand, 5000, "{} dynamic hand proxy: sphere shape creation failed", isLeft ? "Left" : "Right");
            return false;
        }

        if (!slot.proxyBody.create(
                frame.hknpWorld,
                frame.bhkWorld,
                shape,
                dynamicHandProxyFilterInfo(),
                havok_material_registry::registerGeneratedBodyMaterial(frame.hknpWorld),
                BethesdaMotionType::Dynamic,
                isLeft ? LEFT_PROXY_BODY_NAME : RIGHT_PROXY_BODY_NAME)) {
            ROCK_LOG_SAMPLE_WARN(Hand, 5000, "{} dynamic hand proxy: body creation failed", isLeft ? "Left" : "Right");
            havok_ref_count::release(shape);
            return false;
        }

        slot.shape = shape;
        slot.createdWorld = frame.hknpWorld;
        slot.createdBhkWorld = frame.bhkWorld;
        slot.created = true;
        slot.rebuildRequestedAtomic.store(false, std::memory_order_release);
        slot.deviationValidAtomic.store(false, std::memory_order_release);

        initializeGeneratedKeyframedBodyDriveState(slot.driveState, handInput.rawHandWorld);
        (void)placeGeneratedKeyframedBodyImmediately(slot.proxyBody, handInput.rawHandWorld);

        ROCK_LOG_INFO(Hand,
            "{} dynamic hand proxy created: bodyId={} radius={:.2f} localCenter=({:.2f},{:.2f},{:.2f}) layer={}",
            isLeft ? "Left" : "Right",
            slot.proxyBody.getBodyId().value,
            g_rockConfig.rockHandCollisionDynamicProxyRadiusGameUnits,
            localCenter.x,
            localCenter.y,
            localCenter.z,
            collision_layer_policy::ROCK_LAYER_DYNAMIC_HAND_PROXY);
        return true;
    }

    void DynamicHandCollisionRuntime::retireSlot(HandSlot& slot, void* bhkWorld, bool isLeft)
    {
        clearVisual(slot, isLeft);
        if (slot.created) {
            /*
             * Live-world teardown must go through the deferred-retirement queue:
             * the hknp broadphase can still reach this body until the next
             * physics step (see 2026-07-08 use-after-free lesson).
             */
            slot.proxyBody.retireDeferred(bhkWorld ? bhkWorld : slot.createdBhkWorld);
        }
        if (slot.shape) {
            havok_ref_count::release(slot.shape);
            slot.shape = nullptr;
        }
        clearGeneratedKeyframedBodyDriveState(slot.driveState);
        slot.created = false;
        slot.createdWorld = nullptr;
        slot.createdBhkWorld = nullptr;
        slot.deviationValidAtomic.store(false, std::memory_order_release);
        slot.rebuildRequestedAtomic.store(false, std::memory_order_release);
    }

    void DynamicHandCollisionRuntime::clearVisual(HandSlot& slot, bool isLeft)
    {
        if (!slot.visualActive) {
            return;
        }
        (void)frik_visual_authority::clearExternalHandWorldTransform(dynamicHandTag(isLeft), frik_visual_authority::handFromBool(isLeft));
        slot.visualActive = false;
    }

    void DynamicHandCollisionRuntime::retireAll(void* bhkWorld)
    {
        retireSlot(_hands[0], bhkWorld, false);
        retireSlot(_hands[1], bhkWorld, true);
    }

    void DynamicHandCollisionRuntime::reset()
    {
        retireAll(nullptr);
        _logCounter = 0;
    }

    void DynamicHandCollisionRuntime::updateFrame(const PhysicsFrameContext& frame,
        bool physicsWritesAllowed,
        const Hand& rightHand,
        const Hand& leftHand,
        bool rightHandWeaponEquipped,
        bool leftSupportGripActive)
    {
        if (!g_rockConfig.rockHandCollisionDynamicDrive) {
            if (_hands[0].created || _hands[1].created) {
                retireAll(frame.bhkWorld);
            }
            return;
        }

        if (!frame.worldReady || frame.menuBlocked || !physicsWritesAllowed) {
            clearVisual(_hands[0], false);
            clearVisual(_hands[1], true);
            return;
        }

        if (++_logCounter >= 360) {
            _logCounter = 0;
            ROCK_LOG_DEBUG(Hand,
                "DynamicHandCollision active: radius={:.2f} maxLinVelHk={:.1f} divergenceTeleport={:.1f} minDeviation={:.3f} priority={} created R={} L={}",
                g_rockConfig.rockHandCollisionDynamicProxyRadiusGameUnits,
                g_rockConfig.rockHandCollisionDynamicMaxLinearVelocityHavok,
                g_rockConfig.rockHandCollisionDynamicDivergenceTeleportGameUnits,
                g_rockConfig.rockHandCollisionDynamicRenderFollowMinDeviationGameUnits,
                g_rockConfig.rockHandCollisionDynamicVisualPriority,
                _hands[0].created ? "yes" : "no",
                _hands[1].created ? "yes" : "no");
        }

        auto updateHand = [&](bool isLeft, const HandFrameInput& handInput, const Hand& hand, bool weaponOwned) {
            auto& slot = _hands[handIndex(isLeft)];

            if (handInput.disabled) {
                clearVisual(slot, isLeft);
                return;
            }

            if (!ensureCreated(slot, isLeft, frame, handInput)) {
                clearVisual(slot, isLeft);
                return;
            }

            /*
             * The drive keeps chasing the wand even while another system owns
             * the hand pose: a parked proxy recovers through the divergence
             * teleport, and continuing to track keeps the unsuppress handoff
             * seamless. Ownership gates only decide VISUAL authority.
             */
            (void)queueGeneratedKeyframedBodyTarget(
                slot.driveState,
                handInput.rawHandWorld,
                frame.deltaSeconds,
                g_rockConfig.rockHandCollisionDynamicDivergenceTeleportGameUnits);

            const bool ownedByStrongerSystem =
                suppressesGeneratedHandContactEvidence(hand.getState()) || weaponOwned;
            if (ownedByStrongerSystem || !frik_visual_authority::isAvailable()) {
                clearVisual(slot, isLeft);
                return;
            }

            if (!slot.deviationValidAtomic.load(std::memory_order_acquire)) {
                clearVisual(slot, isLeft);
                return;
            }

            const RE::NiPoint3 deviation{
                slot.deviationXAtomic.load(std::memory_order_acquire),
                slot.deviationYAtomic.load(std::memory_order_acquire),
                slot.deviationZAtomic.load(std::memory_order_acquire),
            };
            const float deviationLengthSq = deviation.x * deviation.x + deviation.y * deviation.y + deviation.z * deviation.z;
            const float minDeviation = g_rockConfig.rockHandCollisionDynamicRenderFollowMinDeviationGameUnits;
            if (!std::isfinite(deviationLengthSq) || deviationLengthSq <= minDeviation * minDeviation) {
                clearVisual(slot, isLeft);
                return;
            }

            RE::NiTransform target = handInput.rawHandWorld;
            target.translate.x += deviation.x;
            target.translate.y += deviation.y;
            target.translate.z += deviation.z;
            if (!isFiniteTransform(target)) {
                clearVisual(slot, isLeft);
                return;
            }

            if (frik_visual_authority::applyExternalHandWorldTransform(
                    dynamicHandTag(isLeft),
                    frik_visual_authority::handFromBool(isLeft),
                    target,
                    g_rockConfig.rockHandCollisionDynamicVisualPriority)) {
                slot.visualActive = true;
            } else {
                ROCK_LOG_SAMPLE_WARN(Hand, 2000, "{} dynamic hand render-follow apply failed", isLeft ? "Left" : "Right");
                clearVisual(slot, isLeft);
            }
        };

        updateHand(false, frame.right, rightHand, rightHandWeaponEquipped);
        updateHand(true, frame.left, leftHand, leftSupportGripActive);
    }

    void DynamicHandCollisionRuntime::flushPendingPhysicsDrive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!g_rockConfig.rockHandCollisionDynamicDrive || !world) {
            return;
        }

        for (std::size_t index = 0; index < _hands.size(); ++index) {
            auto& slot = _hands[index];
            if (!slot.created || slot.createdWorld != world) {
                continue;
            }

            const GeneratedBodyDriveMode mode{
                .dynamicVelocity = true,
                .divergenceTeleportGameUnits = g_rockConfig.rockHandCollisionDynamicDivergenceTeleportGameUnits,
            };
            const auto result = driveGeneratedKeyframedBody(
                world,
                slot.proxyBody,
                slot.driveState,
                timing,
                index == 1 ? "DynamicHandProxyL" : "DynamicHandProxyR",
                static_cast<std::uint32_t>(index),
                g_rockConfig.rockHandCollisionDynamicMaxLinearVelocityHavok,
                0.0f,
                mode);

            if (result.shouldRequestRebuild()) {
                slot.rebuildRequestedAtomic.store(true, std::memory_order_release);
                slot.deviationValidAtomic.store(false, std::memory_order_release);
                continue;
            }

            /*
             * Pre-collide live pose == previous substep's post-solve pose, so
             * body-minus-target here is the solver's resolved deviation with one
             * substep of latency. A teleport resets it to zero explicitly so the
             * rendered hand snaps home with the body.
             */
            if (result.attempted && result.hasLiveBodyTransform && !result.teleported) {
                slot.deviationXAtomic.store(result.liveBodyGamePosition.x - result.targetGamePosition.x, std::memory_order_release);
                slot.deviationYAtomic.store(result.liveBodyGamePosition.y - result.targetGamePosition.y, std::memory_order_release);
                slot.deviationZAtomic.store(result.liveBodyGamePosition.z - result.targetGamePosition.z, std::memory_order_release);
                slot.deviationValidAtomic.store(true, std::memory_order_release);
            } else if (result.teleported) {
                slot.deviationXAtomic.store(0.0f, std::memory_order_release);
                slot.deviationYAtomic.store(0.0f, std::memory_order_release);
                slot.deviationZAtomic.store(0.0f, std::memory_order_release);
                slot.deviationValidAtomic.store(true, std::memory_order_release);
            }
        }
    }
}
