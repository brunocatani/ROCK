#include "physics-interaction/hand/DynamicHandCollision.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/native/HavokMaterialRegistry.h"
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
        constexpr std::uint32_t kDynamicHandProxyCollisionGroup = 0x000C;
        constexpr float kTwinDimensionRebuildToleranceGameUnits = 0.05f;

        constexpr std::array<const char*, DynamicHandCollisionRuntime::kBodiesPerHand> kRightTwinNames{
            "ROCK_DynHandTwin_R_Palm",
            "ROCK_DynHandTwin_R_ThumbTip",
            "ROCK_DynHandTwin_R_IndexTip",
            "ROCK_DynHandTwin_R_MiddleTip",
            "ROCK_DynHandTwin_R_RingTip",
            "ROCK_DynHandTwin_R_PinkyTip",
        };
        constexpr std::array<const char*, DynamicHandCollisionRuntime::kBodiesPerHand> kLeftTwinNames{
            "ROCK_DynHandTwin_L_Palm",
            "ROCK_DynHandTwin_L_ThumbTip",
            "ROCK_DynHandTwin_L_IndexTip",
            "ROCK_DynHandTwin_L_MiddleTip",
            "ROCK_DynHandTwin_L_RingTip",
            "ROCK_DynHandTwin_L_PinkyTip",
        };
        constexpr std::array<const char*, DynamicHandCollisionRuntime::kBodiesPerHand> kRightTwinOwnerNames{
            "DynHandTwinR.Palm",
            "DynHandTwinR.Thumb",
            "DynHandTwinR.Index",
            "DynHandTwinR.Middle",
            "DynHandTwinR.Ring",
            "DynHandTwinR.Pinky",
        };
        constexpr std::array<const char*, DynamicHandCollisionRuntime::kBodiesPerHand> kLeftTwinOwnerNames{
            "DynHandTwinL.Palm",
            "DynHandTwinL.Thumb",
            "DynHandTwinL.Index",
            "DynHandTwinL.Middle",
            "DynHandTwinL.Ring",
            "DynHandTwinL.Pinky",
        };

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

        const dynamic_hand_twin::TwinSlotFrame* twinFrameForSlot(const dynamic_hand_twin::TwinTargets& twins, std::size_t bodyIndex)
        {
            if (bodyIndex == DynamicHandCollisionRuntime::kPalmSlot) {
                return &twins.palm;
            }
            const std::size_t fingerIndex = bodyIndex - 1;
            if (fingerIndex >= twins.fingertips.size()) {
                return nullptr;
            }
            return &twins.fingertips[fingerIndex];
        }

        /*
         * Rigid-hand combination of per-body deviations: each twin's deviation
         * is a half-space push-out for its own contacts, so the minimal hand
         * correction satisfying all of them is the sequential projection over
         * the set (same math family the 6e41044 manifold solver validated).
         * Overlapping deviations from one shared surface collapse instead of
         * double-counting; corner deviations from different directions compose.
         */
        RE::NiPoint3 combineTwinDeviations(
            const std::array<RE::NiPoint3, DynamicHandCollisionRuntime::kBodiesPerHand>& deviations,
            const std::array<bool, DynamicHandCollisionRuntime::kBodiesPerHand>& deviationValid)
        {
            constexpr float kTinyDeviation = 1.0e-4f;
            RE::NiPoint3 combined{};
            for (int pass = 0; pass < 3; ++pass) {
                bool changed = false;
                for (std::size_t i = 0; i < deviations.size(); ++i) {
                    if (!deviationValid[i] || !isFinitePoint(deviations[i])) {
                        continue;
                    }
                    const float length = std::sqrt(
                        deviations[i].x * deviations[i].x +
                        deviations[i].y * deviations[i].y +
                        deviations[i].z * deviations[i].z);
                    if (!std::isfinite(length) || length <= kTinyDeviation) {
                        continue;
                    }
                    const RE::NiPoint3 direction{
                        deviations[i].x / length,
                        deviations[i].y / length,
                        deviations[i].z / length,
                    };
                    const float needed =
                        length - (combined.x * direction.x + combined.y * direction.y + combined.z * direction.z);
                    if (needed <= kTinyDeviation) {
                        continue;
                    }
                    combined.x += direction.x * needed;
                    combined.y += direction.y * needed;
                    combined.z += direction.z * needed;
                    changed = true;
                }
                if (!changed) {
                    break;
                }
            }
            return combined;
        }

        /*
         * Contact-noise smoothing for the rendered hand: the solver resolves a
         * driven-into-surface twin slightly differently each substep, and the
         * raw deviation twitch is visible while the hand rests still. The
         * exponential filter only shapes CONTACT deviations (free space is
         * exactly zero and gated before application), so tracking latency is
         * untouched. speed <= 0 disables.
         */
        RE::NiPoint3 smoothAppliedDeviation(const RE::NiPoint3& applied, const RE::NiPoint3& target, float smoothingSpeed, float deltaSeconds)
        {
            if (!std::isfinite(smoothingSpeed) || smoothingSpeed <= 0.0f) {
                return target;
            }
            const float dt = std::clamp(std::isfinite(deltaSeconds) ? deltaSeconds : (1.0f / 90.0f), 0.0f, 0.1f);
            const float alpha = std::clamp(1.0f - std::exp(-smoothingSpeed * dt), 0.0f, 1.0f);
            return RE::NiPoint3{
                applied.x + (target.x - applied.x) * alpha,
                applied.y + (target.y - applied.y) * alpha,
                applied.z + (target.z - applied.z) * alpha,
            };
        }
    }

    bool DynamicHandCollisionRuntime::ensureSlotCreated(ProxySlot& slot,
        bool isLeft,
        std::size_t bodyIndex,
        const PhysicsFrameContext& frame,
        const Hand& hand,
        const dynamic_hand_twin::TwinSlotFrame& twinFrame)
    {
        if (slot.created) {
            const bool dimensionsDrifted =
                std::fabs(twinFrame.length - slot.createdLength) > kTwinDimensionRebuildToleranceGameUnits ||
                std::fabs(twinFrame.radius - slot.createdRadius) > kTwinDimensionRebuildToleranceGameUnits;
            if (slot.createdWorld == frame.hknpWorld &&
                !dimensionsDrifted &&
                !slot.rebuildRequestedAtomic.load(std::memory_order_acquire)) {
                return true;
            }
            retireSlot(slot, frame.bhkWorld);
        }

        if (!frame.hknpWorld || !frame.bhkWorld || !twinFrame.valid || !isFiniteTransform(twinFrame.target)) {
            return false;
        }

        auto* shape = hand.buildDynamicTwinShape(twinFrame, bodyIndex == kPalmSlot);
        if (!shape) {
            ROCK_LOG_SAMPLE_WARN(Hand,
                5000,
                "{} dynamic hand twin {}: shape creation failed",
                isLeft ? "Left" : "Right",
                bodyIndex);
            return false;
        }

        if (!slot.body.create(
                frame.hknpWorld,
                frame.bhkWorld,
                shape,
                dynamicHandProxyFilterInfo(),
                havok_material_registry::registerGeneratedBodyMaterial(frame.hknpWorld),
                BethesdaMotionType::Dynamic,
                (isLeft ? kLeftTwinNames : kRightTwinNames)[bodyIndex])) {
            ROCK_LOG_SAMPLE_WARN(Hand,
                5000,
                "{} dynamic hand twin {}: body creation failed",
                isLeft ? "Left" : "Right",
                bodyIndex);
            havok_ref_count::release(shape);
            return false;
        }

        slot.shape = shape;
        slot.createdWorld = frame.hknpWorld;
        slot.createdBhkWorld = frame.bhkWorld;
        slot.createdLength = twinFrame.length;
        slot.createdRadius = twinFrame.radius;
        slot.created = true;
        slot.rebuildRequestedAtomic.store(false, std::memory_order_release);
        slot.deviationValidAtomic.store(false, std::memory_order_release);

        initializeGeneratedKeyframedBodyDriveState(slot.driveState, twinFrame.target);
        (void)placeGeneratedKeyframedBodyImmediately(slot.body, twinFrame.target);

        ROCK_LOG_INFO(Hand,
            "{} dynamic hand twin created: slot={} bodyId={} length={:.2f} radius={:.2f} layer={}",
            isLeft ? "Left" : "Right",
            bodyIndex,
            slot.body.getBodyId().value,
            twinFrame.length,
            twinFrame.radius,
            collision_layer_policy::ROCK_LAYER_DYNAMIC_HAND_PROXY);
        return true;
    }

    void DynamicHandCollisionRuntime::retireSlot(ProxySlot& slot, void* bhkWorld)
    {
        if (slot.created) {
            /*
             * Live-world teardown must go through the deferred-retirement queue:
             * the hknp broadphase can still reach this body until the next
             * physics step (see 2026-07-08 use-after-free lesson).
             */
            slot.body.retireDeferred(bhkWorld ? bhkWorld : slot.createdBhkWorld);
        }
        if (slot.shape) {
            havok_ref_count::release(slot.shape);
            slot.shape = nullptr;
        }
        clearGeneratedKeyframedBodyDriveState(slot.driveState);
        slot.created = false;
        slot.createdWorld = nullptr;
        slot.createdBhkWorld = nullptr;
        slot.createdLength = 0.0f;
        slot.createdRadius = 0.0f;
        slot.deviationValidAtomic.store(false, std::memory_order_release);
        slot.rebuildRequestedAtomic.store(false, std::memory_order_release);
    }

    void DynamicHandCollisionRuntime::retireHand(HandSlots& handSlots, void* bhkWorld, bool isLeft)
    {
        clearVisual(handSlots, isLeft);
        for (auto& slot : handSlots.bodies) {
            retireSlot(slot, bhkWorld);
        }
    }

    void DynamicHandCollisionRuntime::clearVisual(HandSlots& handSlots, bool isLeft)
    {
        handSlots.appliedDeviation = {};
        if (!handSlots.visualActive) {
            return;
        }
        (void)frik_visual_authority::clearExternalHandWorldTransform(dynamicHandTag(isLeft), frik_visual_authority::handFromBool(isLeft));
        handSlots.visualActive = false;
    }

    void DynamicHandCollisionRuntime::retireAll(void* bhkWorld)
    {
        retireHand(_hands[0], bhkWorld, false);
        retireHand(_hands[1], bhkWorld, true);
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
            if (_hands[0].bodies[kPalmSlot].created || _hands[1].bodies[kPalmSlot].created) {
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
                "DynamicHandCollision active: twinsPerHand={} maxLinVelHk={:.1f} divergenceTeleport={:.1f} minDeviation={:.3f} smoothingSpeed={:.1f} priority={} palmCreated R={} L={}",
                kBodiesPerHand,
                g_rockConfig.rockHandCollisionDynamicMaxLinearVelocityHavok,
                g_rockConfig.rockHandCollisionDynamicDivergenceTeleportGameUnits,
                g_rockConfig.rockHandCollisionDynamicRenderFollowMinDeviationGameUnits,
                g_rockConfig.rockHandCollisionDynamicRenderFollowSmoothingSpeed,
                g_rockConfig.rockHandCollisionDynamicVisualPriority,
                _hands[0].bodies[kPalmSlot].created ? "yes" : "no",
                _hands[1].bodies[kPalmSlot].created ? "yes" : "no");
        }

        auto updateHand = [&](bool isLeft, const HandFrameInput& handInput, const Hand& hand, bool weaponOwned) {
            auto& handSlots = _hands[handIndex(isLeft)];

            if (handInput.disabled) {
                clearVisual(handSlots, isLeft);
                return;
            }

            const auto& twins = hand.dynamicTwinTargets();

            /*
             * The drive keeps chasing the published role frames even while
             * another system owns the hand pose: parked twins recover through
             * the divergence teleport, and continuing to track keeps the
             * unsuppress handoff seamless. Ownership gates only decide VISUAL
             * authority.
             */
            for (std::size_t bodyIndex = 0; bodyIndex < kBodiesPerHand; ++bodyIndex) {
                auto& slot = handSlots.bodies[bodyIndex];
                const auto* twinFrame = twinFrameForSlot(twins, bodyIndex);
                if (!twinFrame || !twinFrame->valid) {
                    slot.deviationValidAtomic.store(false, std::memory_order_release);
                    continue;
                }
                if (!ensureSlotCreated(slot, isLeft, bodyIndex, frame, hand, *twinFrame)) {
                    slot.deviationValidAtomic.store(false, std::memory_order_release);
                    continue;
                }
                (void)queueGeneratedKeyframedBodyTarget(
                    slot.driveState,
                    twinFrame->target,
                    frame.deltaSeconds,
                    g_rockConfig.rockHandCollisionDynamicDivergenceTeleportGameUnits);
            }

            const bool ownedByStrongerSystem =
                suppressesGeneratedHandContactEvidence(hand.getState()) || weaponOwned;
            if (ownedByStrongerSystem || !frik_visual_authority::isAvailable()) {
                clearVisual(handSlots, isLeft);
                return;
            }

            std::array<RE::NiPoint3, kBodiesPerHand> deviations{};
            std::array<bool, kBodiesPerHand> deviationValid{};
            bool anyDeviation = false;
            for (std::size_t bodyIndex = 0; bodyIndex < kBodiesPerHand; ++bodyIndex) {
                auto& slot = handSlots.bodies[bodyIndex];
                if (!slot.deviationValidAtomic.load(std::memory_order_acquire)) {
                    continue;
                }
                deviations[bodyIndex] = RE::NiPoint3{
                    slot.deviationXAtomic.load(std::memory_order_acquire),
                    slot.deviationYAtomic.load(std::memory_order_acquire),
                    slot.deviationZAtomic.load(std::memory_order_acquire),
                };
                deviationValid[bodyIndex] = true;
                anyDeviation = true;
            }

            const RE::NiPoint3 combined = anyDeviation ? combineTwinDeviations(deviations, deviationValid) : RE::NiPoint3{};
            handSlots.appliedDeviation = smoothAppliedDeviation(
                handSlots.appliedDeviation,
                combined,
                g_rockConfig.rockHandCollisionDynamicRenderFollowSmoothingSpeed,
                frame.deltaSeconds);

            const auto& applied = handSlots.appliedDeviation;
            const float appliedLengthSq = applied.x * applied.x + applied.y * applied.y + applied.z * applied.z;
            const float minDeviation = g_rockConfig.rockHandCollisionDynamicRenderFollowMinDeviationGameUnits;
            if (!std::isfinite(appliedLengthSq) || appliedLengthSq <= minDeviation * minDeviation) {
                clearVisual(handSlots, isLeft);
                return;
            }

            RE::NiTransform target = handInput.rawHandWorld;
            target.translate.x += applied.x;
            target.translate.y += applied.y;
            target.translate.z += applied.z;
            if (!isFiniteTransform(target)) {
                clearVisual(handSlots, isLeft);
                return;
            }

            if (frik_visual_authority::applyExternalHandWorldTransform(
                    dynamicHandTag(isLeft),
                    frik_visual_authority::handFromBool(isLeft),
                    target,
                    g_rockConfig.rockHandCollisionDynamicVisualPriority)) {
                handSlots.visualActive = true;
            } else {
                ROCK_LOG_SAMPLE_WARN(Hand, 2000, "{} dynamic hand render-follow apply failed", isLeft ? "Left" : "Right");
                clearVisual(handSlots, isLeft);
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

        for (std::size_t hand = 0; hand < _hands.size(); ++hand) {
            const bool isLeft = hand == 1;
            for (std::size_t bodyIndex = 0; bodyIndex < kBodiesPerHand; ++bodyIndex) {
                auto& slot = _hands[hand].bodies[bodyIndex];
                if (!slot.created || slot.createdWorld != world) {
                    continue;
                }

                const GeneratedBodyDriveMode mode{
                    .dynamicVelocity = true,
                    .divergenceTeleportGameUnits = g_rockConfig.rockHandCollisionDynamicDivergenceTeleportGameUnits,
                };
                const auto result = driveGeneratedKeyframedBody(
                    world,
                    slot.body,
                    slot.driveState,
                    timing,
                    (isLeft ? kLeftTwinOwnerNames : kRightTwinOwnerNames)[bodyIndex],
                    static_cast<std::uint32_t>(bodyIndex),
                    g_rockConfig.rockHandCollisionDynamicMaxLinearVelocityHavok,
                    0.0f,
                    mode);

                if (result.shouldRequestRebuild()) {
                    slot.rebuildRequestedAtomic.store(true, std::memory_order_release);
                    slot.deviationValidAtomic.store(false, std::memory_order_release);
                    continue;
                }

                /*
                 * Pre-collide live pose == previous substep's post-solve pose,
                 * so body-minus-target here is the solver's resolved deviation
                 * with one substep of latency. A teleport resets it to zero so
                 * the rendered hand snaps home with the body.
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
}
