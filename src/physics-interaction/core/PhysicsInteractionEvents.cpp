#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"

#include <algorithm>
#include <cmath>

#include "ROCKMain.h"
#include "RockConfig.h"
#include "physics-interaction/feedback/FeedbackHaptics.h"
#include "physics-interaction/feedback/HapticPolicy.h"
#include "physics-interaction/consume/MouthConsumePolicy.h"
#include "physics-interaction/grab/GrabMassPolicy.h"
#include "physics-interaction/grab/GrabTelemetry.h"
#include "physics-interaction/grab/HeldMassMovement.h"
#include "physics-interaction/native/havok/HavokRuntime.h"
#include "physics-interaction/native/query/PhysicsScale.h"
#include "physics-interaction/native/query/PhysicsUtils.h"
#include "physics-interaction/stash/ShoulderStashPolicy.h"
#include "rock_support/Fo4VrRuntime.h"
#include "rock_support/VRControllers.h"

#include "RE/Bethesda/ActorValueInfo.h"
#include "RE/Havok/hknpMotion.h"

namespace rock
{
    using namespace physics_interaction_detail;

    namespace
    {
        bool applyPlayerSpeedReduction(float previousReduction, float targetReduction)
        {
            previousReduction = held_mass_movement::sanitizeReduction(previousReduction);
            targetReduction = held_mass_movement::sanitizeReduction(targetReduction);
            if (std::fabs(previousReduction - targetReduction) <= 0.001f &&
                (targetReduction > 0.0f || previousReduction <= 0.0f)) {
                return true;
            }

            auto* player = RE::PlayerCharacter::GetSingleton();
            auto* actorValues = RE::ActorValue::GetSingleton();
            if (!player || !actorValues || !actorValues->speedMult || !actorValues->carryWeight) {
                return false;
            }

            if (previousReduction > 0.0f) {
                player->ModActorValue(RE::ACTOR_VALUE_MODIFIER::kTemporary, *actorValues->speedMult, previousReduction);
            }
            if (targetReduction > 0.0f) {
                player->ModActorValue(RE::ACTOR_VALUE_MODIFIER::kTemporary, *actorValues->speedMult, -targetReduction);
            }

            player->ModActorValue(RE::ACTOR_VALUE_MODIFIER::kTemporary, *actorValues->carryWeight, 0.1f);
            player->ModActorValue(RE::ACTOR_VALUE_MODIFIER::kTemporary, *actorValues->carryWeight, -0.1f);
            return true;
        }

        std::uint32_t fillGrabEventBodyKinematics(RE::hknpWorld* world, std::uint32_t bodyId, GrabEventData& eventData)
        {
            if (!world || isInvalidGrabBodyId(bodyId)) {
                return 0;
            }

            auto* motion = havok_runtime::getBodyMotion(world, RE::hknpBodyId{ bodyId });
            if (!motion) {
                return 0;
            }

            std::uint32_t flags = 0;
            const float scale = havokToGameScale();
            eventData.positionGame[0] = motion->position.x * scale;
            eventData.positionGame[1] = motion->position.y * scale;
            eventData.positionGame[2] = motion->position.z * scale;
            flags |= ROCK_GRAB_EVENT_FLAG_POSITION_VALID;

            eventData.velocityGame[0] = motion->linearVelocity.x * scale;
            eventData.velocityGame[1] = motion->linearVelocity.y * scale;
            eventData.velocityGame[2] = motion->linearVelocity.z * scale;
            flags |= ROCK_GRAB_EVENT_FLAG_VELOCITY_VALID;

            const float speedHavok = std::sqrt(
                motion->linearVelocity.x * motion->linearVelocity.x +
                motion->linearVelocity.y * motion->linearVelocity.y +
                motion->linearVelocity.z * motion->linearVelocity.z);
            if (std::isfinite(speedHavok)) {
                eventData.speedGameUnitsPerSecond = speedHavok * scale;
                flags |= ROCK_GRAB_EVENT_FLAG_SPEED_VALID;
            }

            eventData.mass = readGrabEventBodyMass(world, bodyId);
            if (eventData.mass > 0.0f) {
                flags |= ROCK_GRAB_EVENT_FLAG_MASS_VALID;
            }

            return flags;
        }
    }

    void PhysicsInteraction::dispatchPhysicsMessage(std::uint32_t msgType, bool isLeft, RE::TESObjectREFR* refr, std::uint32_t formID, std::uint32_t layer)
    {
        PhysicsEventData data{ isLeft, refr, formID, layer };

        if (auto* m = ::rock::getROCKMessaging()) {
            m->Dispatch(msgType, &data, sizeof(data), nullptr);
        }
    }

    void PhysicsInteraction::pruneHeldImpactHapticCooldowns()
    {
        if (_heldImpactHapticCooldownUntil.size() < 128) {
            return;
        }

        for (auto it = _heldImpactHapticCooldownUntil.begin(); it != _heldImpactHapticCooldownUntil.end();) {
            if (it->second <= _dynamicPushElapsedSeconds) {
                it = _heldImpactHapticCooldownUntil.erase(it);
            } else {
                ++it;
            }
        }
    }

    void PhysicsInteraction::handleGrabEventHaptics(const GrabEventData& eventData)
    {
        auto queueHaptic = [this](bool isLeft, float durationSeconds, float intensity) {
            (void)_feedbackHaptics.queue(
                isLeft ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                durationSeconds,
                intensity);
        };

        if ((eventData.flags & ROCK_GRAB_EVENT_FLAG_SUPPRESS_HAPTIC) != 0) {
            return;
        }

        switch (eventData.type) {
        case GrabEventType::SelectionLocked:
            if (g_rockConfig.rockGrabHapticsEnabled) {
                queueHaptic(eventData.isLeft, g_rockConfig.rockGrabHapticDurationSeconds, g_rockConfig.rockSelectionLockHapticIntensity);
            }
            return;
        case GrabEventType::SelectionUnlocked:
            if (g_rockConfig.rockGrabHapticsEnabled) {
                queueHaptic(
                    eventData.isLeft, g_rockConfig.rockSelectionLockReleaseHapticDurationSeconds, g_rockConfig.rockSelectionLockReleaseHapticIntensity);
            }
            return;
        case GrabEventType::PullStarted:
            if (g_rockConfig.rockGrabHapticsEnabled) {
                queueHaptic(eventData.isLeft, g_rockConfig.rockGrabHapticDurationSeconds, g_rockConfig.rockPullStartHapticIntensity);
            }
            return;
        case GrabEventType::PullCatchSucceeded:
            if (g_rockConfig.rockGrabHapticsEnabled) {
                queueHaptic(eventData.isLeft, g_rockConfig.rockGrabHapticDurationSeconds, g_rockConfig.rockPullCatchHapticIntensity);
            }
            return;
        case GrabEventType::StashCandidate:
            if (g_rockConfig.rockShoulderStashHapticsEnabled) {
                const float confidence =
                    (eventData.flags & ROCK_GRAB_EVENT_FLAG_INTENSITY_VALID) != 0 ? eventData.intensityHint : 1.0f;
                queueHaptic(eventData.isLeft,
                    g_rockConfig.rockShoulderStashCandidateHapticDurationSeconds,
                    shoulder_stash_haptic_policy::computeCandidatePulseIntensity(confidence,
                        shoulder_stash_haptic_policy::CandidatePulseConfig{
                            .enabled = true,
                            .baseIntensity = g_rockConfig.rockShoulderStashCandidateHapticBaseIntensity,
                            .maxIntensity = g_rockConfig.rockShoulderStashCandidateHapticIntensity,
                        }));
            }
            return;
        case GrabEventType::Stashed:
            if (g_rockConfig.rockShoulderStashHapticsEnabled) {
                queueHaptic(eventData.isLeft,
                    g_rockConfig.rockShoulderStashCommitHapticDurationSeconds,
                    g_rockConfig.rockShoulderStashCommitHapticIntensity);
            }
            return;
        case GrabEventType::ConsumeCandidate:
            if (g_rockConfig.rockMouthConsumeHapticsEnabled) {
                const float confidence =
                    (eventData.flags & ROCK_GRAB_EVENT_FLAG_INTENSITY_VALID) != 0 ? eventData.intensityHint : 1.0f;
                queueHaptic(eventData.isLeft,
                    g_rockConfig.rockMouthConsumeCandidateHapticDurationSeconds,
                    mouth_consume_haptic_policy::computeCandidatePulseIntensity(confidence,
                        mouth_consume_haptic_policy::CandidatePulseConfig{
                            .enabled = true,
                            .baseIntensity = g_rockConfig.rockMouthConsumeCandidateHapticBaseIntensity,
                            .maxIntensity = g_rockConfig.rockMouthConsumeCandidateHapticIntensity,
                        }));
            }
            return;
        case GrabEventType::Consumed:
            if (g_rockConfig.rockMouthConsumeHapticsEnabled) {
                queueHaptic(eventData.isLeft,
                    g_rockConfig.rockMouthConsumeCommitHapticDurationSeconds,
                    g_rockConfig.rockMouthConsumeCommitHapticIntensity);
            }
            return;
        case GrabEventType::GrabCommitted:
            queueHaptic(eventData.isLeft,
                g_rockConfig.rockGrabHapticDurationSeconds,
                grab_haptic_policy::computeMassPulseIntensity(eventData.mass,
                    grab_haptic_policy::MassPulseConfig{
                        .enabled = g_rockConfig.rockGrabHapticsEnabled,
                        .baseIntensity = g_rockConfig.rockGrabHapticBaseIntensity,
                        .maxIntensity = g_rockConfig.rockGrabHapticMaxIntensity,
                        .massScale = g_rockConfig.rockGrabHapticMassScale,
                        .massExponent = g_rockConfig.rockGrabHapticMassExponent,
                    }));
            return;
        case GrabEventType::HeldImpact: {
            pruneHeldImpactHapticCooldowns();
            const std::uint64_t cooldownKey =
                (static_cast<std::uint64_t>(eventData.isLeft ? 1u : 0u) << 63) |
                (static_cast<std::uint64_t>(eventData.primaryBodyId) << 32) |
                static_cast<std::uint64_t>(eventData.secondaryBodyId);
            if (const auto it = _heldImpactHapticCooldownUntil.find(cooldownKey);
                it != _heldImpactHapticCooldownUntil.end() && it->second > _dynamicPushElapsedSeconds) {
                return;
            }

            const bool damped = (eventData.flags & ROCK_GRAB_EVENT_FLAG_HELD_IMPACT_DAMPED) != 0;
            const float intensity = grab_haptic_policy::computeImpactPulseIntensity(eventData.mass,
                eventData.speedGameUnitsPerSecond,
                damped,
                grab_haptic_policy::ImpactPulseConfig{
                    .enabled = g_rockConfig.rockHeldImpactHapticsEnabled,
                    .baseIntensity = g_rockConfig.rockHeldImpactHapticBaseIntensity,
                    .maxIntensity = g_rockConfig.rockHeldImpactHapticMaxIntensity,
                    .speedScale = g_rockConfig.rockHeldImpactHapticSpeedScale,
                    .massScale = g_rockConfig.rockHeldImpactHapticMassScale,
                    .massExponent = g_rockConfig.rockHeldImpactHapticMassExponent,
                    .minSpeedGameUnitsPerSecond = g_rockConfig.rockHeldImpactHapticMinSpeedGameUnits,
                    .dampedMultiplier = g_rockConfig.rockHeldImpactHapticDampedMultiplier,
                });
            if (intensity <= 0.0f) {
                return;
            }

            _heldImpactHapticCooldownUntil[cooldownKey] =
                _dynamicPushElapsedSeconds + (std::max)(0.0f, g_rockConfig.rockHeldImpactHapticCooldownSeconds);
            queueHaptic(eventData.isLeft, g_rockConfig.rockHeldImpactHapticDurationSeconds, intensity);
            return;
        }
        default:
            return;
        }
    }

    void PhysicsInteraction::updateFeedbackHaptics(float deltaSeconds)
    {
        std::array<feedback_haptics::HapticOutput, 2> outputs{};
        const auto outputCount = _feedbackHaptics.update(deltaSeconds, outputs.data(), outputs.size());
        for (std::size_t i = 0; i < outputCount; ++i) {
            const auto& output = outputs[i];
            if (!output.active || output.intensity <= 0.0f || output.pulseDurationSeconds <= 0.0f) {
                continue;
            }

            vrcf::VRControllers.triggerHaptic(
                output.hand == feedback_haptics::FeedbackHand::Left ? vrcf::Hand::Left : vrcf::Hand::Right,
                output.pulseDurationSeconds,
                output.intensity);
        }
    }

    void PhysicsInteraction::dispatchGrabEvent(GrabEventData eventData)
    {
        eventData.size = sizeof(GrabEventData);
        eventData.version = ROCK_GRAB_EVENT_VERSION;
        if (eventData.refr && eventData.formID == 0) {
            eventData.formID = eventData.refr->GetFormID();
        }
        eventData.frameIndex = ++_grabEventFrameCounter;

        handleGrabEventHaptics(eventData);

        if (auto* m = ::rock::getROCKMessaging()) {
            m->Dispatch(kPhysMsg_OnGrabEvent, &eventData, sizeof(eventData), nullptr);
        }
    }

    void PhysicsInteraction::dispatchSimpleGrabEvent(
        GrabEventType type,
        bool isLeft,
        RE::TESObjectREFR* refr,
        std::uint32_t primaryBodyId,
        std::uint32_t flags)
    {
        GrabEventData eventData{};
        eventData.type = type;
        switch (type) {
        case GrabEventType::SelectionLocked:
        case GrabEventType::SelectionUnlocked:
            eventData.sourceKind = GrabEventSourceKind::Hand;
            break;
        case GrabEventType::PullStarted:
        case GrabEventType::PullArrived:
        case GrabEventType::PullCatchAttempt:
        case GrabEventType::PullCatchSucceeded:
            eventData.sourceKind = GrabEventSourceKind::PulledObject;
            break;
        default:
            eventData.sourceKind = GrabEventSourceKind::HeldObject;
            break;
        }
        eventData.isLeft = isLeft;
        eventData.refr = refr;
        eventData.formID = refr ? refr->GetFormID() : 0;
        eventData.primaryBodyId = primaryBodyId;
        eventData.flags = flags;
        dispatchGrabEvent(eventData);
    }

    void PhysicsInteraction::dispatchGrabCommittedEvent(bool isLeft, RE::TESObjectREFR* refr, std::uint32_t primaryBodyId, RE::hknpWorld* world)
    {
        GrabEventData eventData{};
        eventData.type = GrabEventType::GrabCommitted;
        eventData.sourceKind = GrabEventSourceKind::HeldObject;
        eventData.isLeft = isLeft;
        eventData.refr = refr;
        eventData.formID = refr ? refr->GetFormID() : 0;
        eventData.primaryBodyId = primaryBodyId;
        eventData.flags |= fillGrabEventBodyKinematics(world, primaryBodyId, eventData);
        dispatchGrabEvent(eventData);
    }

    void PhysicsInteraction::dispatchHeldImpactGrabEvent(
        bool isLeft,
        RE::TESObjectREFR* refr,
        std::uint32_t heldBodyId,
        std::uint32_t otherBodyId,
        float mass,
        float speedGameUnitsPerSecond)
    {
        GrabEventData eventData{};
        eventData.type = GrabEventType::HeldImpact;
        eventData.sourceKind = GrabEventSourceKind::HeldObject;
        eventData.isLeft = isLeft;
        eventData.refr = refr;
        eventData.formID = refr ? refr->GetFormID() : 0;
        eventData.primaryBodyId = heldBodyId;
        eventData.secondaryBodyId = otherBodyId;
        eventData.mass = mass;
        eventData.speedGameUnitsPerSecond = speedGameUnitsPerSecond;
        if (std::isfinite(mass) && mass > 0.0f) {
            eventData.flags |= ROCK_GRAB_EVENT_FLAG_MASS_VALID;
        }
        if (std::isfinite(speedGameUnitsPerSecond) && speedGameUnitsPerSecond > 0.0f) {
            eventData.flags |= ROCK_GRAB_EVENT_FLAG_SPEED_VALID;
        }
        if (isLeft ? _leftHand.isHeldBodyColliding() : _rightHand.isHeldBodyColliding()) {
            eventData.flags |= ROCK_GRAB_EVENT_FLAG_HELD_IMPACT_DAMPED;
        }
        dispatchGrabEvent(eventData);
    }

    void PhysicsInteraction::restoreHeldMassMovementSlowdown(const char* reason)
    {
        if (_heldMassMovementSpeedReduction <= 0.0f) {
            return;
        }

        const float previousReduction = _heldMassMovementSpeedReduction;
        if (applyPlayerSpeedReduction(previousReduction, 0.0f)) {
            _heldMassMovementSpeedReduction = 0.0f;
            _heldMassMovementFadeStartReduction = 0.0f;
            _heldMassMovementFadeElapsedSeconds = 0.0f;
            _heldMassMovementLogCounter = 0;
            ROCK_LOG_DEBUG(Hand,
                "Held mass movement slowdown restored: previousReduction={:.2f} reason={}",
                previousReduction,
                reason ? reason : "restore");
        } else {
            ROCK_LOG_SAMPLE_WARN(Hand,
                300,
                "Held mass movement slowdown restore delayed: previousReduction={:.2f} reason={}",
                previousReduction,
                reason ? reason : "restore");
        }
    }

    void PhysicsInteraction::updateHeldMassMovementSlowdown(RE::hknpWorld* hknp, float deltaSeconds)
    {
        if (!g_rockConfig.rockGrabHeldMassMovementSlowdownEnabled) {
            restoreHeldMassMovementSlowdown("disabled");
            return;
        }

        float heldMass = 0.0f;
        if (hknp) {
            constexpr std::size_t kMaxMovementMassMotionSlots = 160;
            std::array<std::uint32_t, kMaxMovementMassMotionSlots> sampledMotionSlots{};
            std::size_t sampledMotionSlotCount = 0;

            auto motionAlreadySampled = [&](std::uint32_t motionIndex) {
                for (std::size_t i = 0; i < sampledMotionSlotCount; ++i) {
                    if (sampledMotionSlots[i] == motionIndex) {
                        return true;
                    }
                }
                return false;
            };

            auto sampleBody = [&](std::uint32_t bodyId) {
                if (isInvalidGrabBodyId(bodyId)) {
                    return;
                }

                auto* body = havok_runtime::getBody(hknp, RE::hknpBodyId{ bodyId });
                if (!body || !body_frame::hasUsableMotionIndex(body->motionIndex) || motionAlreadySampled(body->motionIndex)) {
                    return;
                }
                if (sampledMotionSlotCount >= sampledMotionSlots.size()) {
                    return;
                }

                const float mass = readGrabEventBodyMass(hknp, bodyId);
                if (!std::isfinite(mass) || mass <= 0.0f) {
                    return;
                }

                sampledMotionSlots[sampledMotionSlotCount++] = body->motionIndex;
                heldMass += mass;
            };

            auto sampleHand = [&](const Hand& hand) {
                if (!hand.isHolding()) {
                    return;
                }

                const auto& savedState = hand.getSavedObjectState();
                sampleBody(savedState.bodyId.value);
                for (const auto bodyId : hand.getHeldBodyIds()) {
                    sampleBody(bodyId);
                }
            };

            sampleHand(_rightHand);
            sampleHand(_leftHand);
        }

        const held_mass_movement::Config movementConfig{
            .enabled = g_rockConfig.rockGrabHeldMassMovementSlowdownEnabled,
            .massProportion = g_rockConfig.rockGrabHeldMassMovementMassProportion,
            .massExponent = g_rockConfig.rockGrabHeldMassMovementMassExponent,
            .maxReduction = g_rockConfig.rockGrabHeldMassMovementMaxReduction,
            .fadeOutSeconds = g_rockConfig.rockGrabHeldMassMovementFadeOutSeconds,
        };
        const float heldMassReduction = held_mass_movement::computeHeldMassReduction(heldMass, movementConfig);
        float targetReduction = heldMassReduction;
        if (heldMassReduction > 0.0f) {
            _heldMassMovementFadeStartReduction = heldMassReduction;
            _heldMassMovementFadeElapsedSeconds = 0.0f;
        } else if (_heldMassMovementSpeedReduction > 0.0f) {
            if (_heldMassMovementFadeStartReduction <= 0.0f) {
                _heldMassMovementFadeStartReduction = _heldMassMovementSpeedReduction;
                _heldMassMovementFadeElapsedSeconds = 0.0f;
            }
            _heldMassMovementFadeElapsedSeconds += std::isfinite(deltaSeconds) ? (std::max)(0.0f, deltaSeconds) : 0.0f;
            targetReduction = held_mass_movement::computeFadeOutReduction(
                _heldMassMovementFadeStartReduction,
                _heldMassMovementFadeElapsedSeconds,
                movementConfig.fadeOutSeconds);
        } else {
            _heldMassMovementFadeStartReduction = 0.0f;
            _heldMassMovementFadeElapsedSeconds = 0.0f;
        }

        if (std::fabs(targetReduction - _heldMassMovementSpeedReduction) <= 0.001f &&
            (targetReduction > 0.0f || _heldMassMovementSpeedReduction <= 0.0f)) {
            return;
        }

        const float previousReduction = _heldMassMovementSpeedReduction;
        if (!applyPlayerSpeedReduction(previousReduction, targetReduction)) {
            ROCK_LOG_SAMPLE_WARN(Hand,
                300,
                "Held mass movement slowdown skipped: heldMass={:.3f} previousReduction={:.2f} targetReduction={:.2f}",
                heldMass,
                previousReduction,
                targetReduction);
            return;
        }

        _heldMassMovementSpeedReduction = targetReduction;
        if (targetReduction <= 0.0f) {
            _heldMassMovementFadeStartReduction = 0.0f;
            _heldMassMovementFadeElapsedSeconds = 0.0f;
        }
        if (g_rockConfig.rockDebugGrabFrameLogging) {
            ++_heldMassMovementLogCounter;
            if (_heldMassMovementLogCounter >= 90 || heldMass <= 0.0f || previousReduction <= 0.0f) {
                _heldMassMovementLogCounter = 0;
                ROCK_LOG_DEBUG(Hand,
                    "Held mass movement slowdown: heldMass={:.3f} previousReduction={:.2f} targetReduction={:.2f}",
                    heldMass,
                    previousReduction,
                    targetReduction);
            }
        }
    }

    bool PhysicsInteraction::physicsModOwnsObject(RE::TESObjectREFR* ref) const
    {
        if (!ref)
            return false;
        std::scoped_lock lock(_ownedObjectsMutex);
        const auto it = _ownedObjects.find(ref->GetFormID());
        return it != _ownedObjects.end() && it->second != 0;
    }

    bool PhysicsInteraction::physicsModOwnsObject(RE::TESObjectREFR* ref, PhysicsObjectClaimOwner owner) const
    {
        if (!ref)
            return false;
        std::scoped_lock lock(_ownedObjectsMutex);
        const auto it = _ownedObjects.find(ref->GetFormID());
        return it != _ownedObjects.end() && (it->second & claimOwnerBit(owner)) != 0;
    }

    void PhysicsInteraction::claimObject(RE::TESObjectREFR* ref, PhysicsObjectClaimOwner owner)
    {
        if (!ref)
            return;
        auto formID = ref->GetFormID();
        std::scoped_lock lock(_ownedObjectsMutex);
        auto& ownerMask = _ownedObjects[formID];
        const auto previousMask = ownerMask;
        ownerMask |= claimOwnerBit(owner);
        ROCK_LOG_DEBUG(Hand,
            "Claimed object: formID={:08X} owner={} mask=0x{:02X}->0x{:02X} owners={}",
            formID,
            static_cast<std::uint32_t>(owner),
            previousMask,
            ownerMask,
            claimOwnerCount(ownerMask));
    }

    void PhysicsInteraction::releaseObject(RE::TESObjectREFR* ref, PhysicsObjectClaimOwner owner)
    {
        if (!ref)
            return;
        auto formID = ref->GetFormID();
        std::scoped_lock lock(_ownedObjectsMutex);
        auto it = _ownedObjects.find(formID);
        if (it == _ownedObjects.end()) {
            return;
        }

        const auto previousMask = it->second;
        it->second &= ~claimOwnerBit(owner);
        if (it->second != 0) {
            ROCK_LOG_DEBUG(Hand,
                "Released object claim: formID={:08X} owner={} mask=0x{:02X}->0x{:02X} ownersRemaining={}",
                formID,
                static_cast<std::uint32_t>(owner),
                previousMask,
                it->second,
                claimOwnerCount(it->second));
            return;
        }

        _ownedObjects.erase(it);
        ROCK_LOG_DEBUG(Hand,
            "Released object: formID={:08X} owner={} mask=0x{:02X}->0x00",
            formID,
            static_cast<std::uint32_t>(owner),
            previousMask);
    }

    void PhysicsInteraction::releaseAllObjects()
    {
        std::scoped_lock lock(_ownedObjectsMutex);
        if (!_ownedObjects.empty()) {
            ROCK_LOG_DEBUG(Hand, "Releasing all {} owned objects", _ownedObjects.size());
            _ownedObjects.clear();
        }

        (void)frik_visual_authority::blockOffHandWeaponGripping("ROCK_Physics", false);
    }

    void PhysicsInteraction::forceDropHeldObject(bool isLeft)
    {
        auto& hand = isLeft ? _leftHand : _rightHand;
        if (!hand.isHolding())
            return;

        auto* bhk = getPlayerBhkWorld();
        if (!bhk) {
            ROCK_LOG_WARN(Hand, "forceDropHeldObject: no bhkWorld available");
            return;
        }
        auto* hknp = getHknpWorld(bhk);
        if (!hknp) {
            ROCK_LOG_WARN(Hand, "forceDropHeldObject: no hknpWorld available");
            return;
        }

        auto* heldRef = hand.getHeldRef();
        ROCK_LOG_INFO(Hand, "forceDropHeldObject: {} hand dropping {}", isLeft ? "Left" : "Right", heldRef ? heldRef->GetFormID() : 0);

        hand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Delayed, makeGrabReleaseContext(hand, isLeft));
        if (heldRef)
            releaseObject(heldRef, claimOwnerForHand(isLeft));
        dispatchSimpleGrabEvent(GrabEventType::Released, isLeft, heldRef);
    }

    RE::bhkWorld* PhysicsInteraction::getPlayerBhkWorld() const
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player)
            return nullptr;

        auto* cell = player->GetParentCell();
        if (!cell)
            return nullptr;

        return cell->GetbhkWorld();
    }

    RE::hknpWorld* PhysicsInteraction::getHknpWorld(RE::bhkWorld* bhk)
    {
        if (!bhk)
            return nullptr;

        return havok_runtime::getHknpWorldFromBhk(bhk);
    }
}
