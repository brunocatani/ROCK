#include "physics-interaction/core/PhysicsInteractionInternal.h"

// Grab event dispatch and haptic feedback.

namespace rock
{
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
}
