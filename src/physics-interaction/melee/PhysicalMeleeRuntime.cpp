#include "physics-interaction/melee/PhysicalMeleeRuntime.h"

#include "RockConfig.h"
#include "api/ROCKProviderApiInternal.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/melee/MeleeTargetResolver.h"
#include "physics-interaction/melee/NativeMeleeHitBridge.h"
#include "physics-interaction/melee/PhysicalMeleePolicy.h"
#include "physics-interaction/native/PhysicsScale.h"

#include "RE/Bethesda/PlayerCharacter.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <utility>

namespace rock::physical_melee
{
    namespace
    {
        constexpr std::uint64_t kProviderImpactIdBit = 0x8000'0000'0000'0000ull;
        constexpr std::uint64_t kEpisodeGapFrames = 4;
        constexpr std::size_t kEpisodeCapacity = 128;
        constexpr std::size_t kCooldownCapacity = 128;

        struct EpisodeEntry
        {
            std::uint64_t weaponGenerationKey{ 0 };
            std::uint32_t weaponFormId{ 0 };
            std::uint32_t targetActorFormId{ 0 };
            std::uint64_t lastFrameIndex{ 0 };
            std::uint64_t episodeId{ 0 };
        };

        struct CooldownEntry
        {
            std::uint64_t weaponGenerationKey{ 0 };
            std::uint32_t weaponFormId{ 0 };
            std::uint32_t targetActorFormId{ 0 };
            std::chrono::steady_clock::time_point appliedAt{};
            bool occupied{ false };
        };

        struct RuntimeState
        {
            bool bridgeReady{ false };
            std::uint32_t worldGeneration{ 0 };
            std::uint32_t skeletonGeneration{ 0 };
            std::uint32_t providerGeneration{ 0 };
            std::uint64_t nextImpactId{ 1 };
            std::uint64_t nextEpisodeId{ 1 };
            std::uint64_t budgetFrameIndex{ 0 };
            std::uint32_t submittedThisFrame{ 0 };
            std::array<EpisodeEntry, kEpisodeCapacity> episodes{};
            std::array<CooldownEntry, kCooldownCapacity> cooldowns{};
        };

        RuntimeState s_runtime{};

        [[nodiscard]] Settings currentSettings()
        {
            return sanitizeSettings(Settings{
                .enabled = g_rockConfig.rockPhysicalMeleeEnabled,
                .minSourceSpeedGame = g_rockConfig.rockPhysicalMeleeMinSourceSpeedGame,
                .virtualWeaponMass = g_rockConfig.rockPhysicalMeleeVirtualWeaponMass,
                .damageMultiplier = g_rockConfig.rockPhysicalMeleeDamageMultiplier,
                .maxNativeDamageMultiplier = g_rockConfig.rockPhysicalMeleeMaxNativeDamageMultiplier,
                .sourceTargetCooldownSeconds = g_rockConfig.rockPhysicalMeleeSourceTargetCooldownSeconds,
                .maxDamageEventsPerFrame = static_cast<std::uint32_t>(
                    (std::max)(1, g_rockConfig.rockPhysicalMeleeMaxDamageEventsPerFrame)),
            });
        }

        [[nodiscard]] bool sameSourceTarget(
            std::uint64_t generationKey,
            std::uint32_t weaponFormId,
            std::uint32_t targetActorFormId,
            const EpisodeEntry& entry)
        {
            if (targetActorFormId == 0 || entry.targetActorFormId != targetActorFormId) {
                return false;
            }
            return generationKey != 0 ? entry.weaponGenerationKey == generationKey :
                                       entry.weaponGenerationKey == 0 && entry.weaponFormId == weaponFormId;
        }

        [[nodiscard]] std::pair<std::uint64_t, std::uint32_t> updateEpisode(
            const provider::RockProviderExternalContactV1& contact,
            std::uint32_t targetActorFormId)
        {
            EpisodeEntry* empty = nullptr;
            EpisodeEntry* oldest = &s_runtime.episodes[0];
            for (auto& entry : s_runtime.episodes) {
                if (entry.episodeId == 0) {
                    if (!empty) {
                        empty = &entry;
                    }
                    continue;
                }
                if (entry.lastFrameIndex < oldest->lastFrameIndex) {
                    oldest = &entry;
                }
                if (!sameSourceTarget(
                        contact.sourceWeaponGenerationKey,
                        contact.sourceWeaponFormId,
                        targetActorFormId,
                        entry)) {
                    continue;
                }
                const bool continued = contact.frameIndex >= entry.lastFrameIndex &&
                    contact.frameIndex - entry.lastFrameIndex <= kEpisodeGapFrames;
                entry.lastFrameIndex = contact.frameIndex;
                if (continued) {
                    return {
                        entry.episodeId,
                        static_cast<std::uint32_t>(provider::RockProviderImpactEpisodeFlagV1::Continued)
                    };
                }
                entry.episodeId = s_runtime.nextEpisodeId++;
                return {
                    entry.episodeId,
                    static_cast<std::uint32_t>(provider::RockProviderImpactEpisodeFlagV1::Started)
                };
            }

            auto* entry = empty ? empty : oldest;
            *entry = EpisodeEntry{
                .weaponGenerationKey = contact.sourceWeaponGenerationKey,
                .weaponFormId = contact.sourceWeaponFormId,
                .targetActorFormId = targetActorFormId,
                .lastFrameIndex = contact.frameIndex,
                .episodeId = s_runtime.nextEpisodeId++,
            };
            return {
                entry->episodeId,
                static_cast<std::uint32_t>(provider::RockProviderImpactEpisodeFlagV1::Started)
            };
        }

        [[nodiscard]] bool cooldownActive(
            const provider::RockProviderExternalContactRecordV1& contact,
            const Settings& settings,
            std::chrono::steady_clock::time_point now)
        {
            if (settings.sourceTargetCooldownSeconds <= 0.0f) {
                return false;
            }
            for (const auto& entry : s_runtime.cooldowns) {
                if (!entry.occupied || entry.targetActorFormId != contact.targetActorFormId) {
                    continue;
                }
                const bool sourceMatches = contact.sourceWeaponGenerationKey != 0 ?
                    entry.weaponGenerationKey == contact.sourceWeaponGenerationKey :
                    entry.weaponGenerationKey == 0 && entry.weaponFormId == contact.sourceWeaponFormId;
                if (!sourceMatches) {
                    continue;
                }
                const auto elapsed = std::chrono::duration<float>(now - entry.appliedAt).count();
                return std::isfinite(elapsed) && elapsed < settings.sourceTargetCooldownSeconds;
            }
            return false;
        }

        void recordCooldown(
            const provider::RockProviderExternalContactRecordV1& contact,
            std::chrono::steady_clock::time_point now)
        {
            CooldownEntry* empty = nullptr;
            CooldownEntry* oldest = &s_runtime.cooldowns[0];
            for (auto& entry : s_runtime.cooldowns) {
                if (!entry.occupied) {
                    if (!empty) {
                        empty = &entry;
                    }
                    continue;
                }
                if (entry.appliedAt < oldest->appliedAt) {
                    oldest = &entry;
                }
                const bool sourceMatches = contact.sourceWeaponGenerationKey != 0 ?
                    entry.weaponGenerationKey == contact.sourceWeaponGenerationKey :
                    entry.weaponGenerationKey == 0 && entry.weaponFormId == contact.sourceWeaponFormId;
                if (sourceMatches && entry.targetActorFormId == contact.targetActorFormId) {
                    entry.appliedAt = now;
                    return;
                }
            }
            auto* entry = empty ? empty : oldest;
            *entry = CooldownEntry{
                .weaponGenerationKey = contact.sourceWeaponGenerationKey,
                .weaponFormId = contact.sourceWeaponFormId,
                .targetActorFormId = contact.targetActorFormId,
                .appliedAt = now,
                .occupied = true,
            };
        }

        provider::RockProviderExternalContactRecordV1 makeRecord(
            const provider::RockProviderExternalContactV1& contact,
            const TargetResolution& target,
            std::uint32_t worldGeneration,
            std::uint32_t skeletonGeneration,
            std::uint32_t providerGeneration)
        {
            provider::RockProviderExternalContactRecordV1 record{};
            record.frameIndex = contact.frameIndex;
            record.sourceBodyId = contact.sourceBodyId;
            record.targetExternalBodyId = contact.targetExternalBodyId;
            record.bodyGeneration = contact.generation;
            record.sourceKind = contact.sourceKind;
            record.sourceHand = contact.sourceHand;
            record.targetRole = provider::RockProviderExternalBodyRole::ActorRagdollBone;
            record.quality = contact.quality;
            record.flags = contact.flags;
            std::copy_n(contact.sourceVelocityHavok, 3, record.sourceVelocityHavok);
            std::copy_n(contact.contactPointHavok, 3, record.contactPointHavok);
            std::copy_n(contact.contactNormalHavok, 3, record.contactNormalHavok);
            record.contactPointWeightSum = contact.contactPointWeightSum;
            record.sourcePartKind = contact.sourcePartKind;
            record.sourceRole = contact.sourceRole;
            record.sourceSubRole = contact.sourceSubRole;
            record.collisionGeneration = contact.collisionGeneration;
            record.worldGeneration = worldGeneration;
            record.skeletonGeneration = skeletonGeneration;
            record.providerGeneration = providerGeneration;
            record.impactId = kProviderImpactIdBit | s_runtime.nextImpactId++;
            const auto [episodeId, episodeFlags] = updateEpisode(contact, target.actorFormId);
            record.episodeId = episodeId;
            record.episodeFlags = episodeFlags;
            record.sourceEndpointIndex = contact.sourceEndpointIndex;
            record.manifoldPointCount = contact.manifoldPointCount;
            record.selectedPointIndex = contact.selectedPointIndex;
            record.nativeContactPointIndex = contact.nativeContactPointIndex;
            std::copy_n(&contact.manifoldPointsHavok[0][0], 16, &record.manifoldPointsHavok[0][0]);
            std::copy_n(contact.manifoldSeparationsHavok, 4, record.manifoldSeparationsHavok);
            std::copy_n(contact.manifoldImpulses, 4, record.manifoldImpulses);
            std::copy_n(contact.sourceAngularVelocityHavok, 3, record.sourceAngularVelocityHavok);
            std::copy_n(contact.targetVelocityHavok, 3, record.targetVelocityHavok);
            std::copy_n(contact.targetAngularVelocityHavok, 3, record.targetAngularVelocityHavok);
            std::copy_n(contact.sourceCenterOfMassHavok, 3, record.sourceCenterOfMassHavok);
            std::copy_n(contact.targetCenterOfMassHavok, 3, record.targetCenterOfMassHavok);
            std::copy_n(contact.sourceContactLocalGame, 3, record.sourceContactLocalGame);
            record.closingSpeedHavok = contact.closingSpeedHavok;
            record.tangentSpeedHavok = contact.tangentSpeedHavok;
            record.sourceSurfaceCoordinate = contact.sourceSurfaceCoordinate;
            record.sourceSurfaceDamageCoefficient = contact.sourceSurfaceDamageCoefficient;
            record.sourceWeaponGenerationKey = contact.sourceWeaponGenerationKey;
            record.sourceGeometryKey = contact.sourceGeometryKey;
            record.sourceWeaponFormId = contact.sourceWeaponFormId;
            record.sourceDescriptorIndex = contact.sourceDescriptorIndex;
            record.sourceSurfaceRegion = contact.sourceSurfaceRegion;
            record.sourceSurfaceConfidencePermille = contact.sourceSurfaceConfidencePermille;
            record.targetActorFormId = target.actorFormId;
            record.targetAnatomyFlags = target.anatomyFlags;
            record.targetBodyPartIndex = target.bodyPartIndex;
            record.targetZone = target.zone;
            record.targetSide = target.side;
            record.targetBodyPartDamageMultiplier = target.bodyPartDamageMultiplier;
            record.targetLimbActorValueFormId = target.limbActorValueFormId;
            record.targetNodeNameHash = target.nodeNameHash;
            std::copy_n(target.nodeName, provider::ROCK_PROVIDER_MAX_EVIDENCE_NAME, record.targetNodeName);
            if (target.anatomyValid()) {
                record.flags |= static_cast<std::uint32_t>(
                    provider::RockProviderExternalContactFlagV1::TargetAnatomyValid);
            }
            return record;
        }

        provider::RockProviderImpactOutcomeV1 makeOutcome(
            const provider::RockProviderExternalContactRecordV1& contact,
            std::uint64_t submissionFrameIndex,
            provider::RockProviderImpactOutcomeLifecycleV1 lifecycle,
            std::uint32_t failureReason,
            const Decision* decision = nullptr)
        {
            provider::RockProviderImpactOutcomeV1 outcome{};
            outcome.impactId = contact.impactId;
            outcome.episodeId = contact.episodeId;
            outcome.contactSequence = contact.sequence;
            outcome.submittedFrameIndex = submissionFrameIndex;
            outcome.targetActorFormId = contact.targetActorFormId;
            outcome.targetBodyId = contact.targetExternalBodyId;
            outcome.sourceBodyId = contact.sourceBodyId;
            outcome.sourceWeaponFormId = contact.sourceWeaponFormId;
            outcome.targetBodyPartIndex = contact.targetBodyPartIndex;
            outcome.sourceSurfaceRegion = contact.sourceSurfaceRegion;
            outcome.lifecycle = lifecycle;
            outcome.failureReason = failureReason;
            outcome.sourceSurfaceDamageCoefficient = contact.sourceSurfaceDamageCoefficient;
            if (decision) {
                outcome.closingSpeedGame = decision->closingSpeedGame;
                outcome.requestedNativeMultiplier = decision->nativeDamageMultiplier;
            }
            return outcome;
        }

        void reject(
            const provider::RockProviderExternalContactRecordV1& contact,
            std::uint64_t submissionFrameIndex,
            DecisionReason reason,
            const Decision* decision = nullptr)
        {
            provider::recordPhysicalMeleeOutcome(makeOutcome(
                contact,
                submissionFrameIndex,
                provider::RockProviderImpactOutcomeLifecycleV1::Rejected,
                static_cast<std::uint32_t>(reason),
                decision));
            ROCK_LOG_SAMPLE_DEBUG(Melee,
                g_rockConfig.rockLogSampleMilliseconds,
                "Physical melee rejected impact={} actor={:08X} body={} part={} surface={} reason={} speed={:.1f}",
                contact.impactId,
                contact.targetActorFormId,
                contact.targetExternalBodyId,
                contact.targetBodyPartIndex,
                static_cast<std::uint32_t>(contact.sourceSurfaceRegion),
                decisionReasonName(reason),
                decision ? decision->closingSpeedGame : 0.0f);
        }

        void resetForGenerations(
            std::uint32_t worldGeneration,
            std::uint32_t skeletonGeneration,
            std::uint32_t providerGeneration)
        {
            if (s_runtime.worldGeneration == worldGeneration &&
                s_runtime.skeletonGeneration == skeletonGeneration &&
                s_runtime.providerGeneration == providerGeneration) {
                return;
            }
            s_runtime.worldGeneration = worldGeneration;
            s_runtime.skeletonGeneration = skeletonGeneration;
            s_runtime.providerGeneration = providerGeneration;
            s_runtime.episodes = {};
            s_runtime.cooldowns = {};
            s_runtime.budgetFrameIndex = 0;
            s_runtime.submittedThisFrame = 0;
            resetNativeMeleeHitBridge();
        }
    }

    bool initializeRuntime()
    {
        s_runtime.bridgeReady = installNativeMeleeHitBridge();
        return s_runtime.bridgeReady;
    }

    void resetRuntime() noexcept
    {
        s_runtime.episodes = {};
        s_runtime.cooldowns = {};
        s_runtime.worldGeneration = 0;
        s_runtime.skeletonGeneration = 0;
        s_runtime.providerGeneration = 0;
        s_runtime.budgetFrameIndex = 0;
        s_runtime.submittedThisFrame = 0;
        resetNativeMeleeHitBridge();
    }

    void drainCompletedOutcomes(std::uint64_t currentFrameIndex)
    {
        std::array<provider::RockProviderImpactOutcomeV1, 64> outcomes{};
        const auto count = drainNativeMeleeHitOutcomes(
            currentFrameIndex,
            outcomes.data(),
            static_cast<std::uint32_t>(outcomes.size()));
        for (std::uint32_t i = 0; i < count; ++i) {
            provider::recordPhysicalMeleeOutcome(outcomes[i]);
            ROCK_LOG_SAMPLE_INFO(Melee,
                g_rockConfig.rockLogSampleMilliseconds,
                "Physical melee outcome impact={} lifecycle={} actor={:08X} body={} part={} submitted=({:.3f} health,{:.3f} limb) observed=({:.3f} health,{:.3f} limb)",
                outcomes[i].impactId,
                static_cast<std::uint32_t>(outcomes[i].lifecycle),
                outcomes[i].targetActorFormId,
                outcomes[i].targetBodyId,
                outcomes[i].targetBodyPartIndex,
                outcomes[i].submittedHealthDamage,
                outcomes[i].submittedLimbDamage,
                outcomes[i].observedHealthComponentDelta,
                outcomes[i].observedLimbComponentDelta);
        }
    }

    void processContactObservation(
        RE::bhkWorld* bhkWorld,
        const provider::RockProviderExternalContactV1& observation,
        std::uint32_t worldGeneration,
        std::uint32_t skeletonGeneration,
        std::uint32_t providerGeneration,
        std::uint64_t processingFrameIndex)
    {
        resetForGenerations(worldGeneration, skeletonGeneration, providerGeneration);
        const auto settings = currentSettings();
        if (!settings.enabled || observation.sourceKind != provider::RockProviderExternalSourceKind::Weapon) {
            return;
        }

        const auto contactPointFlags =
            static_cast<std::uint32_t>(provider::RockProviderExternalContactFlagV1::ContactPointValid) |
            static_cast<std::uint32_t>(provider::RockProviderExternalContactFlagV1::ContactPointMeasured);
        const auto* measuredContactPoint =
            (observation.flags & contactPointFlags) == contactPointFlags ?
            observation.contactPointHavok :
            nullptr;
        const auto target = resolveTarget(
            bhkWorld,
            observation.targetExternalBodyId,
            measuredContactPoint,
            physics_scale::havokToGame());
        auto contact = makeRecord(
            observation,
            target,
            worldGeneration,
            skeletonGeneration,
            providerGeneration);
        if (!provider::recordPhysicalMeleeContact(contact)) {
            ROCK_LOG_SAMPLE_WARN(Melee,
                g_rockConfig.rockLogSampleMilliseconds,
                "Physical melee collision report rejected actor={:08X} sourceBody={} targetBody={}",
                contact.targetActorFormId,
                contact.sourceBodyId,
                contact.targetExternalBodyId);
        }
        if (!target.actorValid()) {
            reject(contact, processingFrameIndex, DecisionReason::MissingTargetActor);
            return;
        }

        auto decision = evaluate(contact, settings, physics_scale::gameToHavok());
        if (!decision.accepted) {
            reject(contact, processingFrameIndex, decision.reason, &decision);
            return;
        }

        if (s_runtime.budgetFrameIndex != processingFrameIndex) {
            s_runtime.budgetFrameIndex = processingFrameIndex;
            s_runtime.submittedThisFrame = 0;
        }
        if (s_runtime.submittedThisFrame >= settings.maxDamageEventsPerFrame) {
            reject(contact, processingFrameIndex, DecisionReason::FrameBudgetExceeded, &decision);
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (cooldownActive(contact, settings, now)) {
            reject(contact, processingFrameIndex, DecisionReason::CooldownActive, &decision);
            return;
        }

        auto* aggressor = static_cast<RE::Actor*>(RE::PlayerCharacter::GetSingleton());
        if (!s_runtime.bridgeReady || !aggressor || aggressor == target.actor) {
            reject(contact, processingFrameIndex, DecisionReason::NativeSubmissionFailed, &decision);
            return;
        }

        ++s_runtime.submittedThisFrame;
        const auto hit = applyNativeMeleeHit(NativeMeleeHitInput{
            .bhkWorld = bhkWorld,
            .target = target.actor,
            .aggressor = aggressor,
            .contact = &contact,
            .submissionFrameIndex = processingFrameIndex,
            .nativeDamageMultiplier = decision.nativeDamageMultiplier,
            .closingSpeedGame = decision.closingSpeedGame,
            .havokToGameScale = physics_scale::havokToGame(),
        });
        if (!hit.submitted) {
            auto outcome = makeOutcome(
                contact,
                processingFrameIndex,
                provider::RockProviderImpactOutcomeLifecycleV1::Rejected,
                0x8000'0000u | static_cast<std::uint32_t>(hit.failure),
                &decision);
            provider::recordPhysicalMeleeOutcome(outcome);
            ROCK_LOG_SAMPLE_WARN(Melee,
                g_rockConfig.rockLogSampleMilliseconds,
                "Physical melee native submission failed impact={} actor={:08X} body={} part={} reason={}",
                contact.impactId,
                contact.targetActorFormId,
                contact.targetExternalBodyId,
                contact.targetBodyPartIndex,
                nativeMeleeHitFailureName(hit.failure));
            return;
        }

        recordCooldown(contact, now);
        auto submittedOutcome = hit.outcome;
        if (hit.failure != NativeMeleeHitFailure::None) {
            submittedOutcome.failureReason =
                0x8000'0000u | static_cast<std::uint32_t>(hit.failure);
        }
        provider::recordPhysicalMeleeOutcome(submittedOutcome);
        ROCK_LOG_SAMPLE_INFO(Melee,
            g_rockConfig.rockLogSampleMilliseconds,
            "Physical melee submitted impact={} actor={:08X} body={} node='{}' part={} zone={} side={} surface={} coefficient={:.3f} multiplier={:.3f} speed=({:.3f} hk/{:.1f} game) point=({:.1f},{:.1f},{:.1f}) normal=({:.3f},{:.3f},{:.3f})",
            contact.impactId,
            contact.targetActorFormId,
            contact.targetExternalBodyId,
            contact.targetNodeName,
            contact.targetBodyPartIndex,
            static_cast<std::uint32_t>(contact.targetZone),
            static_cast<std::uint32_t>(contact.targetSide),
            static_cast<std::uint32_t>(contact.sourceSurfaceRegion),
            contact.sourceSurfaceDamageCoefficient,
            decision.nativeDamageMultiplier,
            decision.closingSpeedHavok,
            decision.closingSpeedGame,
            hit.contactPointGame[0],
            hit.contactPointGame[1],
            hit.contactPointGame[2],
            hit.contactNormal[0],
            hit.contactNormal[1],
            hit.contactNormal[2]);
    }
}
