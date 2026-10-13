#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/weapon/EquippedDropPickupPolicy.h"
#include "physics-interaction/input/FarPullGesturePolicy.h"
#include "physics-interaction/native/PhysicsRayCast.h"
#include "physics-interaction/native/PhysicsScale.h"

namespace rock
{
    std::array<bool, 2> PhysicsInteraction::updateEquippedDropPickup(const PhysicsFrameContext& frame,
        RE::NiNode* weapon, std::uint64_t generation, const EquippedWeaponPrimaryGripInput& right,
        const EquippedWeaponPrimaryGripInput& left, bool rightAvailable, bool leftAvailable)
    {
        std::array<bool, 2> catchHand{};
        auto& state = _equippedDropPickup;
        state.consumed = {};
        const auto body = _dynamicWeaponCollision.proxyBodyIdForDebug().value;
        const auto cancel = [&] {
            _dynamicWeaponCollision.cancelEquippedDropPull();
            state = {};
        };
        if (!_twoHandedGrip.isEquippedDrop() || !_dynamicWeaponCollision.equippedDropActive() ||
            !weapon || !generation || frame.menuBlocked || !frame.worldReady || !physicsWritesAllowedForWorld(frame.hknpWorld)) {
            cancel();
            return catchHand;
        }
        if (state.hand >= 0 && (state.world != reinterpret_cast<std::uintptr_t>(frame.hknpWorld) ||
                state.generation != generation || state.body != body)) cancel();
        RE::NiPoint3 gripLocal{};
        if (!_twoHandedGrip.tryGetSurfaceSupportPrimaryGripLocal(weapon, generation, gripLocal)) {
            cancel();
            return catchHand;
        }
        const auto feedback = [&](int hand, float intensity) {
            (void)_feedbackHaptics.queue(hand ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                g_rockConfig.rockGrabHapticDurationSeconds, intensity);
        };
        const auto beginPull = [&](int hand) {
            const auto& input = hand ? frame.left : frame.right;
            if (!_dynamicHandCollision.leaseReleasedWeapon(frame.hknpWorld, body,
                    (std::max)(g_rockConfig.rockGrabReleaseHandCollisionDelaySeconds,
                        pull_motion_math::kDurationA + pull_motion_math::kOwnerGraceSeconds)) ||
                !_dynamicWeaponCollision.startEquippedDropPull(frame, gripLocal, input.grabAnchorWorld)) return false;
            state.pulling = true;
            feedback(hand, g_rockConfig.rockPullStartHapticIntensity);
            return true;
        };
        if (state.hand >= 0) {
            const int hand = state.hand;
            const auto& button = hand ? left : right;
            const auto& input = hand ? frame.left : frame.right;
            if (!(hand ? leftAvailable : rightAvailable) || !button.held || button.released ||
                input_remap_runtime::isProviderOpenVrGameInputSuppressedForHand(hand != 0)) {
                cancel();
                return catchHand;
            }
            state.consumed[hand] = true;
            if (state.waitingForRelease) return catchHand;
            if (!state.pulling) {
                const auto point = transform_math::localPointToWorld(weapon->world, gripLocal);
                const float speed = far_pull_gesture::speedAwayFromTarget(input.gestureVelocityMetersPerSecond,
                    input.grabAnchorWorld, point);
                if (far_pull_gesture::confirmsPull(button.held, button.released, input.hasGestureVelocity, speed) && !beginPull(hand))
                    state.waitingForRelease = true;
                return catchHand;
            }
            const auto progress = _dynamicWeaponCollision.updateEquippedDropPull(frame, input.grabAnchorWorld);
            if (progress == DynamicWeaponCollisionRuntime::DropPullProgress::Arrived) catchHand[hand] = true;
            else if (progress == DynamicWeaponCollisionRuntime::DropPullProgress::Expired ||
                progress == DynamicWeaponCollisionRuntime::DropPullProgress::Unavailable) {
                _dynamicWeaponCollision.cancelEquippedDropPull();
                state.pulling = false;
                state.waitingForRelease = true; // Do not replay this hold on an unrelated target.
            }
            return catchHand;
        }

        const auto hmdGate = makeFarSelectionHmdConeGate(frame);
        for (const int hand : { 0, 1 }) {
            const auto& button = hand ? left : right;
            const auto& input = hand ? frame.left : frame.right;
            const auto& objectHand = hand ? _leftHand : _rightHand;
            if (!(hand ? leftAvailable : rightAvailable) || !button.pressed || !button.held ||
                _equipped.toggleGrabReleasePressConsumedThisFrame[equipped_weapon_toggle_grab_policy::handIndex(hand != 0)] ||
                input_remap_runtime::isProviderOpenVrGameInputSuppressedForHand(hand != 0)) continue;
            WeaponCollision::WeaponSurfaceProximityWitness witness{};
            const bool nearPickup = _weaponCollision.tryFindCurrentWeaponSurfaceNearPoint(weapon, input.grabAnchorWorld,
                selection_query_policy::kNearDetectionRangeGameUnits, witness);
            if (!nearPickup) {
                WeaponCollision::ApproximateBoundsSnapshot bounds{};
                RE::NiPoint3 direction{};
                if (!_weaponCollision.getApproximateBoundsSnapshot(bounds) || !bounds.valid || bounds.generationKey != generation ||
                    !selection_query_policy::tryNormalizeVectorForHmdCone(input.farSelectionDirectionWorld, direction)) continue;
                const auto center = transform_math::localPointToWorld(weapon->world, bounds.centerWeaponLocal);
                const auto toCenter = center - input.grabAnchorWorld;
                const float along = toCenter.x * direction.x + toCenter.y * direction.y + toCenter.z * direction.z;
                const auto rayPoint = input.grabAnchorWorld + direction * (std::max)(0.0f, along);
                if (!_weaponCollision.tryFindCurrentWeaponSurfaceNearPoint(weapon, rayPoint,
                        selection_query_policy::kFarCastRadiusGameUnits, witness) ||
                    !equipped_drop_pickup_policy::rayAdmission(input.grabAnchorWorld, direction, witness.closestPointWorld,
                        selection_query_policy::kFarDetectionRangeGameUnits, selection_query_policy::kFarCastRadiusGameUnits).valid ||
                    !hmdGate.acceptsHitPoint(witness.closestPointWorld)) continue;
            }
            if (!witness.valid || witness.weaponGenerationKey != generation || !witness.sourceNodeCurrent) continue;
            const float distance = (witness.closestPointWorld - input.grabAnchorWorld).Length();
            if (!std::isfinite(distance)) continue;
            if (objectHand.hasSelection() && objectHand.getSelection().isValid() &&
                (objectHand.getSelection().hitPointWorld - input.grabAnchorWorld).Length() < distance) continue;
            physics_ray_cast::ClosestSegmentResult obstruction{};
            if (!physics_ray_cast::castClosestSegment(frame.bhkWorld, input.grabAnchorWorld, witness.closestPointWorld,
                    selection_query_policy::kFarClipRayFilterInfo, obstruction) ||
                (obstruction.hit && (!std::isfinite(obstruction.hitFraction) ||
                    obstruction.hitFraction * distance < distance - selection_query_policy::kNearCastRadiusGameUnits))) continue;
            state = { .world = reinterpret_cast<std::uintptr_t>(frame.hknpWorld), .generation = generation,
                .body = body, .hand = hand };
            state.consumed[hand] = true;
            if (nearPickup) {
                catchHand[hand] = true;
                // A refused authored catch must require another squeeze, never
                // turn into an unexpected far-pull lock on the following frame.
                state.waitingForRelease = true;
                ROCK_LOG_INFO(Weapon, "Equipped drop near catch requested hand={} distance={:.3f} generation={:016X}",
                    hand ? "left" : "right", distance, generation);
            } else if (g_rockConfig.rockFarGrabMode == static_cast<int>(far_pull_gesture::Mode::Gesture)) {
                feedback(hand, g_rockConfig.rockSelectionLockHapticIntensity);
                ROCK_LOG_INFO(Weapon, "Equipped drop pull locked hand={} generation={:016X}", hand ? "left" : "right", generation);
            } else if (!beginPull(hand)) {
                state.waitingForRelease = true;
            }
            break;
        }
        return catchHand;
    }
}
