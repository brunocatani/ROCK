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
        constexpr std::size_t kCandidateCapacity = 1024;

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

        struct Candidate
        {
            provider::RockProviderExternalContactRecordV1 contact{};
            TargetResolution target{};
            WeaponIdentityWitness expectedWeapon{};
            Decision decision{};
            bool baseEligible{ false };
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
            ContactFrameContext frame{};
            std::array<Candidate, kCandidateCapacity> candidates{};
            std::size_t candidateCount{ 0 };
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
            return generationKey != 0 ?
                entry.weaponGenerationKey == generationKey &&
                    weaponFormId != 0 && entry.weaponFormId == weaponFormId :
                entry.weaponGenerationKey == 0 &&
                    weaponFormId != 0 && entry.weaponFormId == weaponFormId;
        }

        [[nodiscard]] EpisodeEntry* findActiveEpisode(
            const provider::RockProviderExternalContactRecordV1& contact,
            std::uint64_t frameIndex)
        {
            for (auto& entry : s_runtime.episodes) {
                if (entry.episodeId == 0 || !sameSourceTarget(
                        contact.sourceWeaponGenerationKey,
                        contact.sourceWeaponFormId,
                        contact.targetActorFormId,
                        entry)) {
                    continue;
                }
                if (frameIndex >= entry.lastFrameIndex && frameIndex - entry.lastFrameIndex <= kEpisodeGapFrames) {
                    return &entry;
                }
            }
            return nullptr;
        }

        void commitEpisode(
            const provider::RockProviderExternalContactRecordV1& contact,
            std::uint64_t episodeId,
            std::uint64_t frameIndex)
        {
            EpisodeEntry* empty = nullptr;
            EpisodeEntry* oldest = &s_runtime.episodes[0];
            for (auto& entry : s_runtime.episodes) {
                if (entry.episodeId != 0 && sameSourceTarget(
                        contact.sourceWeaponGenerationKey,
                        contact.sourceWeaponFormId,
                        contact.targetActorFormId,
                        entry)) {
                    entry.lastFrameIndex = frameIndex;
                    entry.episodeId = episodeId;
                    return;
                }
                if (entry.episodeId == 0 && !empty) {
                    empty = &entry;
                } else if (entry.episodeId != 0 && entry.lastFrameIndex < oldest->lastFrameIndex) {
                    oldest = &entry;
                }
            }
            auto* entry = empty ? empty : oldest;
            *entry = EpisodeEntry{
                .weaponGenerationKey = contact.sourceWeaponGenerationKey,
                .weaponFormId = contact.sourceWeaponFormId,
                .targetActorFormId = contact.targetActorFormId,
                .lastFrameIndex = frameIndex,
                .episodeId = episodeId,
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
                    entry.weaponGenerationKey == contact.sourceWeaponGenerationKey &&
                        entry.weaponFormId == contact.sourceWeaponFormId :
                    entry.weaponGenerationKey == 0 &&
                        entry.weaponFormId == contact.sourceWeaponFormId;
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
                    entry.weaponGenerationKey == contact.sourceWeaponGenerationKey &&
                        entry.weaponFormId == contact.sourceWeaponFormId :
                    entry.weaponGenerationKey == 0 &&
                        entry.weaponFormId == contact.sourceWeaponFormId;
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
        s_runtime.frame = {};
        s_runtime.candidateCount = 0;
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

    void beginContactFrame(const ContactFrameContext& context)
    {
        resetForGenerations(
            context.worldGeneration,
            context.skeletonGeneration,
            context.providerGeneration);
        s_runtime.frame = context;
        s_runtime.candidateCount = 0;
        if (s_runtime.budgetFrameIndex != context.processingFrameIndex) {
            s_runtime.budgetFrameIndex = context.processingFrameIndex;
            s_runtime.submittedThisFrame = 0;
        }
    }

    void processContactObservation(
        const provider::RockProviderExternalContactV1& observation,
        const WeaponIdentityWitness& expectedWeapon)
    {
        if (observation.sourceKind != provider::RockProviderExternalSourceKind::Weapon) {
            return;
        }
        if (s_runtime.candidateCount >= s_runtime.candidates.size()) {
            ROCK_LOG_SAMPLE_WARN(Melee,
                g_rockConfig.rockLogSampleMilliseconds,
                "Physical melee frame candidate buffer full; rich contact omitted sourceBody={} targetBody={}",
                observation.sourceBodyId,
                observation.targetExternalBodyId);
            return;
        }

        const auto contactPointFlags =
            static_cast<std::uint32_t>(provider::RockProviderExternalContactFlagV1::ContactPointValid) |
            static_cast<std::uint32_t>(provider::RockProviderExternalContactFlagV1::ContactPointMeasured);
        const auto* measuredContactPoint =
            (observation.flags & contactPointFlags) == contactPointFlags ?
            observation.contactPointHavok : nullptr;

        auto& candidate = s_runtime.candidates[s_runtime.candidateCount++];
        candidate = {};
        candidate.expectedWeapon = expectedWeapon;
        candidate.target = resolveTarget(
            s_runtime.frame.bhkWorld,
            observation.targetExternalBodyId,
            measuredContactPoint,
            physics_scale::havokToGame());
        candidate.contact = makeRecord(
            observation,
            candidate.target,
            s_runtime.frame.worldGeneration,
            s_runtime.frame.skeletonGeneration,
            s_runtime.frame.providerGeneration);

        if (!candidate.target.actorValid()) {
            candidate.decision.reason = DecisionReason::MissingTargetActor;
        } else {
            candidate.decision = evaluate(
                candidate.contact,
                currentSettings(),
                physics_scale::gameToHavok());
            if (candidate.decision.accepted &&
                !weaponWitnessMatches(candidate.expectedWeapon, s_runtime.frame.currentWeapon)) {
                candidate.decision.accepted = false;
                candidate.decision.reason = DecisionReason::WeaponWitnessMismatch;
            } else if (candidate.decision.accepted && !s_runtime.frame.damageSubmissionAllowed) {
                candidate.decision.accepted = false;
                candidate.decision.reason = DecisionReason::RuntimeUnavailable;
            }
            candidate.baseEligible = candidate.decision.accepted;
        }
    }

    void finishContactFrame()
    {
        const auto settings = currentSettings();
        const auto frameIndex = s_runtime.frame.processingFrameIndex;
        std::array<bool, kCandidateCapacity> handled{};

        auto report = [](provider::RockProviderExternalContactRecordV1& contact) {
            if (!provider::recordPhysicalMeleeContact(contact)) {
                ROCK_LOG_SAMPLE_WARN(Melee,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Physical melee collision report rejected actor={:08X} sourceBody={} targetBody={}",
                    contact.targetActorFormId,
                    contact.sourceBodyId,
                    contact.targetExternalBodyId);
            }
        };
        auto sameGroup = [](const Candidate& lhs, const Candidate& rhs) {
            if (lhs.contact.targetActorFormId == 0 ||
                lhs.contact.targetActorFormId != rhs.contact.targetActorFormId) {
                return false;
            }
            return lhs.contact.sourceWeaponGenerationKey != 0 ?
                lhs.contact.sourceWeaponGenerationKey == rhs.contact.sourceWeaponGenerationKey &&
                    lhs.contact.sourceWeaponFormId == rhs.contact.sourceWeaponFormId :
                rhs.contact.sourceWeaponGenerationKey == 0 &&
                    lhs.contact.sourceWeaponFormId == rhs.contact.sourceWeaponFormId;
        };
        auto better = [](const Candidate& lhs, const Candidate& rhs) {
            return isBetterImpactCandidate(
                ImpactCandidateScore{
                    .nativeDamageMultiplier = lhs.decision.nativeDamageMultiplier,
                    .positiveImpulseSum = lhs.contact.contactPointWeightSum,
                    .surfaceConfidencePermille = lhs.contact.sourceSurfaceConfidencePermille,
                    .sourceBodyId = lhs.contact.sourceBodyId,
                    .descriptorIndex = lhs.contact.sourceDescriptorIndex,
                },
                ImpactCandidateScore{
                    .nativeDamageMultiplier = rhs.decision.nativeDamageMultiplier,
                    .positiveImpulseSum = rhs.contact.contactPointWeightSum,
                    .surfaceConfidencePermille = rhs.contact.sourceSurfaceConfidencePermille,
                    .sourceBodyId = rhs.contact.sourceBodyId,
                    .descriptorIndex = rhs.contact.sourceDescriptorIndex,
                });
        };

        for (std::size_t i = 0; i < s_runtime.candidateCount; ++i) {
            if (handled[i]) {
                continue;
            }
            auto& first = s_runtime.candidates[i];
            if (!first.baseEligible) {
                handled[i] = true;
                report(first.contact);
                reject(first.contact, frameIndex, first.decision.reason, &first.decision);
                continue;
            }

            std::size_t bestIndex = i;
            for (std::size_t j = i + 1; j < s_runtime.candidateCount; ++j) {
                if (!handled[j] && s_runtime.candidates[j].baseEligible &&
                    sameGroup(first, s_runtime.candidates[j]) &&
                    better(s_runtime.candidates[j], s_runtime.candidates[bestIndex])) {
                    bestIndex = j;
                }
            }

            auto* activeEpisode = findActiveEpisode(first.contact, frameIndex);
            const auto episodeId = activeEpisode ? activeEpisode->episodeId : s_runtime.nextEpisodeId++;
            for (std::size_t j = i; j < s_runtime.candidateCount; ++j) {
                auto& grouped = s_runtime.candidates[j];
                if (handled[j] || !grouped.baseEligible || !sameGroup(first, grouped)) {
                    continue;
                }
                handled[j] = true;
                grouped.contact.episodeId = episodeId;
                grouped.contact.episodeFlags = static_cast<std::uint32_t>(
                    activeEpisode ? provider::RockProviderImpactEpisodeFlagV1::Continued :
                    j == bestIndex ? provider::RockProviderImpactEpisodeFlagV1::Started :
                                     provider::RockProviderImpactEpisodeFlagV1::Continued);
                report(grouped.contact);
                if (activeEpisode) {
                    reject(grouped.contact, frameIndex, DecisionReason::ContinuedEpisode, &grouped.decision);
                } else if (j != bestIndex) {
                    reject(grouped.contact, frameIndex, DecisionReason::SupersededCandidate, &grouped.decision);
                }
            }
            if (activeEpisode) {
                activeEpisode->lastFrameIndex = frameIndex;
                continue;
            }

            auto& best = s_runtime.candidates[bestIndex];
            const auto now = std::chrono::steady_clock::now();
            if (s_runtime.submittedThisFrame >= settings.maxDamageEventsPerFrame) {
                reject(best.contact, frameIndex, DecisionReason::FrameBudgetExceeded, &best.decision);
                commitEpisode(best.contact, episodeId, frameIndex);
                continue;
            }
            if (cooldownActive(best.contact, settings, now)) {
                reject(best.contact, frameIndex, DecisionReason::CooldownActive, &best.decision);
                commitEpisode(best.contact, episodeId, frameIndex);
                continue;
            }

            auto* aggressor = static_cast<RE::Actor*>(RE::PlayerCharacter::GetSingleton());
            if (!s_runtime.bridgeReady || !aggressor || aggressor == best.target.actor) {
                reject(best.contact, frameIndex, DecisionReason::NativeSubmissionFailed, &best.decision);
                continue;
            }

            ++s_runtime.submittedThisFrame;
            const auto hit = applyNativeMeleeHit(NativeMeleeHitInput{
                .bhkWorld = s_runtime.frame.bhkWorld,
                .target = best.target.actor,
                .aggressor = aggressor,
                .contact = &best.contact,
                .expectedWeapon = best.expectedWeapon,
                .currentWeapon = s_runtime.frame.currentWeapon,
                .submissionFrameIndex = frameIndex,
                .nativeDamageMultiplier = best.decision.nativeDamageMultiplier,
                .closingSpeedGame = best.decision.closingSpeedGame,
                .havokToGameScale = physics_scale::havokToGame(),
            });
            if (!hit.submitted) {
                auto outcome = makeOutcome(
                    best.contact,
                    frameIndex,
                    provider::RockProviderImpactOutcomeLifecycleV1::Rejected,
                    0x8000'0000u | static_cast<std::uint32_t>(hit.failure),
                    &best.decision);
                provider::recordPhysicalMeleeOutcome(outcome);
                ROCK_LOG_SAMPLE_WARN(Melee,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Physical melee native submission failed impact={} actor={:08X} body={} part={} reason={}",
                    best.contact.impactId,
                    best.contact.targetActorFormId,
                    best.contact.targetExternalBodyId,
                    best.contact.targetBodyPartIndex,
                    nativeMeleeHitFailureName(hit.failure));
                continue;
            }

            commitEpisode(best.contact, episodeId, frameIndex);
            recordCooldown(best.contact, now);
            auto submittedOutcome = hit.outcome;
            if (hit.failure != NativeMeleeHitFailure::None) {
                submittedOutcome.failureReason = 0x8000'0000u | static_cast<std::uint32_t>(hit.failure);
            }
            provider::recordPhysicalMeleeOutcome(submittedOutcome);
            ROCK_LOG_SAMPLE_INFO(Melee,
                g_rockConfig.rockLogSampleMilliseconds,
                "Physical melee submitted impact={} actor={:08X} body={} node='{}' part={} surface={} coefficient={:.3f} multiplier={:.3f} speed={:.1f}",
                best.contact.impactId,
                best.contact.targetActorFormId,
                best.contact.targetExternalBodyId,
                best.contact.targetNodeName,
                best.contact.targetBodyPartIndex,
                static_cast<std::uint32_t>(best.contact.sourceSurfaceRegion),
                best.contact.sourceSurfaceDamageCoefficient,
                best.decision.nativeDamageMultiplier,
                best.decision.closingSpeedGame);
        }
        s_runtime.candidateCount = 0;
    }
}
