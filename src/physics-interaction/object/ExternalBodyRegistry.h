#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>

#include "api/ROCKProviderApi.h"

namespace rock
{
    /*
     * Fixed-capacity external body registry. Every row retains the registered
     * ROCK owner separately from the consumer-selected child scope. This is
     * the ownership boundary that lets one plugin demultiplex actors/sessions
     * without inventing provider owner tokens or escaping unregister cleanup.
     */
    class ExternalBodyRegistry
    {
    public:
        enum class RegistrationResult : std::uint8_t
        {
            Ok,
            InvalidArgument,
            CapacityFull,
            OwnerConflict,
        };

        static constexpr std::uint32_t kMaxBodies =
            ::rock::provider::ROCK_PROVIDER_MAX_EXTERNAL_BODIES_V1;
        static constexpr std::uint32_t kMaxContacts =
            ::rock::provider::ROCK_PROVIDER_MAX_EXTERNAL_CONTACTS_V1;
        static constexpr std::uint32_t kMaxScopes =
            ::rock::provider::ROCK_PROVIDER_MAX_EXTERNAL_SCOPES_V1;

        bool registerBodies(
            const std::uint64_t ownerToken,
            const ::rock::provider::RockProviderExternalBodyRegistration* bodies,
            const std::uint32_t bodyCount)
        {
            return registerBodiesForScope(
                ownerToken,
                ownerToken,
                bodies,
                bodyCount);
        }

        bool registerBodiesForScope(
            const std::uint64_t parentOwnerToken,
            const std::uint64_t scopeToken,
            const ::rock::provider::RockProviderExternalBodyRegistration* bodies,
            const std::uint32_t bodyCount)
        {
            return registerBodiesForScopeDetailed(
                       parentOwnerToken,
                       scopeToken,
                       bodies,
                       bodyCount) == RegistrationResult::Ok;
        }

        RegistrationResult registerBodiesForScopeDetailed(
            const std::uint64_t parentOwnerToken,
            const std::uint64_t scopeToken,
            const ::rock::provider::RockProviderExternalBodyRegistration* bodies,
            const std::uint32_t bodyCount)
        {
            if (parentOwnerToken == 0 || scopeToken == 0 ||
                (!bodies && bodyCount != 0)) {
                return RegistrationResult::InvalidArgument;
            }
            if (bodyCount > kMaxBodies) {
                return RegistrationResult::CapacityFull;
            }

            for (std::uint32_t i = 0; i < bodyCount; ++i) {
                const auto& body = bodies[i];
                if (body.size != sizeof(
                        ::rock::provider::RockProviderExternalBodyRegistration) ||
                    body.ownerToken != scopeToken ||
                    body.bodyId == kInvalidBodyId ||
                    body.animationBoneCandidateCount >
                        ::rock::provider::ROCK_PROVIDER_MAX_ANATOMY_BONE_CANDIDATES_V1 ||
                    (body.bodyPartIndex != kInvalidIndex &&
                        body.bodyPartIndex >= 26u) ||
                    !std::isfinite(body.bodyPartDamageMultiplier) ||
                    body.bodyPartDamageMultiplier < 0.0f ||
                    batchContainsDuplicateBody(bodies, i, body.bodyId)) {
                    return RegistrationResult::InvalidArgument;
                }
                if (containsBodyOwnedByOtherScope(
                        body.bodyId,
                        parentOwnerToken,
                        scopeToken)) {
                    return RegistrationResult::OwnerConflict;
                }
            }

            const auto existingScopeBodyCount =
                countBodiesForScope(parentOwnerToken, scopeToken);
            if ((_bodyCount - existingScopeBodyCount) + bodyCount > kMaxBodies) {
                return RegistrationResult::CapacityFull;
            }

            const auto existingScopeIndex = findScopeIndex(
                parentOwnerToken,
                scopeToken);
            const auto availableScopeIndex = existingScopeIndex != kInvalidIndex ?
                existingScopeIndex :
                findAvailableScopeIndex();
            if (availableScopeIndex == kInvalidIndex) {
                return RegistrationResult::CapacityFull;
            }

            // A refresh preserves already-emitted evidence for this scope.
            if (existingScopeIndex == kInvalidIndex) {
                _scopes[availableScopeIndex] = ScopeSlot{
                    .parentOwnerToken = parentOwnerToken,
                    .scopeToken = scopeToken,
                };
            }
            clearScopeBodies(parentOwnerToken, scopeToken);
            for (std::uint32_t i = 0; i < bodyCount; ++i) {
                _bodies[_bodyCount++] = BodySlot{
                    .parentOwnerToken = parentOwnerToken,
                    .scopeToken = scopeToken,
                    .scopeIndex = availableScopeIndex,
                    .registration = bodies[i],
                };
            }
            publishAtomicBodyIndex();
            return RegistrationResult::Ok;
        }

        void clearOwner(const std::uint64_t parentOwnerToken)
        {
            if (parentOwnerToken == 0) {
                return;
            }
            removeBodies([&](const BodySlot& body) {
                return body.parentOwnerToken == parentOwnerToken;
            });
            removeContacts([&](const ContactSlot& contact) {
                return contact.record.parentOwnerToken == parentOwnerToken;
            });
            for (auto& scope : _scopes) {
                if (scope.parentOwnerToken == parentOwnerToken) {
                    scope = {};
                }
            }
            publishAtomicBodyIndex();
        }

        bool clearScope(
            const std::uint64_t parentOwnerToken,
            const std::uint64_t scopeToken)
        {
            if (parentOwnerToken == 0 || scopeToken == 0) {
                return false;
            }
            const auto scopeIndex = findScopeIndex(
                parentOwnerToken,
                scopeToken);
            const bool existed = scopeIndex != kInvalidIndex;
            clearScopeBodies(parentOwnerToken, scopeToken);
            removeContacts([&](const ContactSlot& contact) {
                return contact.record.parentOwnerToken == parentOwnerToken &&
                       contact.record.scopeToken == scopeToken;
            });
            if (scopeIndex != kInvalidIndex) {
                _scopes[scopeIndex] = {};
            }
            publishAtomicBodyIndex();
            return existed;
        }

        void clearAll()
        {
            _bodies = {};
            _contacts = {};
            _scopes = {};
            _bodyCount = 0;
            _contactCount = 0;
            _contactHead = 0;
            _nextContactSequence = 1;
            _impactEpisodes = {};
            _nextImpactId = 1;
            _nextEpisodeId = 1;
            publishAtomicBodyIndex();
        }

        [[nodiscard]] bool containsBody(const std::uint32_t bodyId) const
        {
            return findBody(bodyId) != nullptr;
        }

        /*
         * Physics contact callbacks must never wait behind provider
         * registration work. Registration publishes a sorted, atomic value
         * snapshot; callback readers either observe one complete generation
         * or conservatively report no match while a publication is in flight.
         */
        [[nodiscard]] bool containsBodyAtomic(const std::uint32_t bodyId) const noexcept
        {
            return findAtomicBodyPolicy(bodyId, nullptr);
        }

        [[nodiscard]] bool suppressesRockDynamicPushAtomic(
            const std::uint32_t bodyId) const noexcept
        {
            std::uint32_t policy = 0;
            if (!findAtomicBodyPolicy(bodyId, &policy)) {
                return false;
            }
            return (policy & static_cast<std::uint32_t>(
                                 ::rock::provider::RockProviderExternalBodyContactPolicy::SuppressRockDynamicPush)) != 0;
        }

        [[nodiscard]] bool suppressesRockDynamicPush(
            const std::uint32_t bodyId) const
        {
            const auto* body = findBody(bodyId);
            return body && hasPolicy(
                body->registration,
                ::rock::provider::RockProviderExternalBodyContactPolicy::SuppressRockDynamicPush);
        }

        [[nodiscard]] bool tryGetBody(
            const std::uint32_t bodyId,
            ::rock::provider::RockProviderExternalBodyRegistration& outBody) const
        {
            const auto* body = findBody(bodyId);
            if (!body) {
                return false;
            }
            outBody = body->registration;
            return true;
        }

        void recordHandContact(
            const bool isLeft,
            const std::uint32_t handBodyId,
            const std::uint32_t externalBodyId,
            const std::uint64_t frameIndex)
        {
            const auto* body = findBody(externalBodyId);
            if (!body || !hasAnyReportPolicy(body->registration)) {
                return;
            }

            ::rock::provider::RockProviderExternalContactV1 contact{};
            contact.sourceBodyId = handBodyId;
            contact.targetExternalBodyId = externalBodyId;
            contact.frameIndex = frameIndex;
            contact.sourceKind =
                ::rock::provider::RockProviderExternalSourceKind::Hand;
            contact.sourceHand = isLeft ?
                ::rock::provider::RockProviderHand::Left :
                ::rock::provider::RockProviderHand::Right;
            contact.quality =
                ::rock::provider::RockProviderExternalContactQuality::BodyPairOnly;
            recordContactV1(contact);
        }

        bool recordContactV1(
            ::rock::provider::RockProviderExternalContactV1 contact,
            const std::uint32_t worldGeneration = 0,
            const std::uint32_t skeletonGeneration = 0,
            const std::uint32_t providerGeneration = 0)
        {
            if (contact.size != sizeof(
                    ::rock::provider::RockProviderExternalContactV1) ||
                contact.sourceBodyId == kInvalidBodyId ||
                contact.targetExternalBodyId == kInvalidBodyId ||
                contact.sourceBodyId == contact.targetExternalBodyId) {
                return false;
            }

            const auto* body = findBody(contact.targetExternalBodyId);
            if (!body || !shouldReportContact(
                    body->registration,
                    contact.sourceKind)) {
                return false;
            }
            if (body->scopeIndex >= _scopes.size() ||
                _scopes[body->scopeIndex].scopeToken == 0) {
                return false;
            }

            contact.generation = body->registration.generation;
            contact.ownerToken = body->scopeToken;
            contact.targetRole = body->registration.role;
            contact.sequence = _nextContactSequence++;

            const auto episode = updateImpactEpisode(
                contact.sourceBodyId,
                contact.targetExternalBodyId,
                contact.frameIndex);

            ContactSlot slot{};
            slot.scopeIndex = body->scopeIndex;
            slot.legacy = contact;
            slot.record.parentOwnerToken = body->parentOwnerToken;
            slot.record.scopeToken = body->scopeToken;
            slot.record.sequence = contact.sequence;
            slot.record.frameIndex = contact.frameIndex;
            slot.record.sourceBodyId = contact.sourceBodyId;
            slot.record.targetExternalBodyId = contact.targetExternalBodyId;
            slot.record.bodyGeneration = contact.generation;
            slot.record.sourceKind = contact.sourceKind;
            slot.record.sourceHand = contact.sourceHand;
            slot.record.targetRole = contact.targetRole;
            slot.record.quality = contact.quality;
            slot.record.flags = contact.flags;
            std::copy_n(contact.sourceVelocityHavok, 3,
                slot.record.sourceVelocityHavok);
            std::copy_n(contact.contactPointHavok, 3,
                slot.record.contactPointHavok);
            std::copy_n(contact.contactNormalHavok, 3,
                slot.record.contactNormalHavok);
            slot.record.contactPointWeightSum =
                contact.contactPointWeightSum;
            slot.record.sourcePartKind = contact.sourcePartKind;
            slot.record.sourceRole = contact.sourceRole;
            slot.record.sourceSubRole = contact.sourceSubRole;
            slot.record.collisionGeneration = contact.collisionGeneration;
            slot.record.worldGeneration = worldGeneration;
            slot.record.skeletonGeneration = skeletonGeneration;
            slot.record.providerGeneration = providerGeneration;
            slot.record.impactId = _nextImpactId++;
            slot.record.episodeId = episode.episodeId;
            slot.record.episodeFlags = episode.flags;
            slot.record.sourceEndpointIndex = contact.sourceEndpointIndex;
            slot.record.manifoldPointCount = contact.manifoldPointCount;
            slot.record.selectedPointIndex = contact.selectedPointIndex;
            slot.record.nativeContactPointIndex = contact.nativeContactPointIndex;
            std::copy_n(&contact.manifoldPointsHavok[0][0], 16,
                &slot.record.manifoldPointsHavok[0][0]);
            std::copy_n(contact.manifoldSeparationsHavok, 4,
                slot.record.manifoldSeparationsHavok);
            std::copy_n(contact.manifoldImpulses, 4,
                slot.record.manifoldImpulses);
            std::copy_n(contact.sourceAngularVelocityHavok, 3,
                slot.record.sourceAngularVelocityHavok);
            std::copy_n(contact.targetVelocityHavok, 3,
                slot.record.targetVelocityHavok);
            std::copy_n(contact.targetAngularVelocityHavok, 3,
                slot.record.targetAngularVelocityHavok);
            std::copy_n(contact.sourceCenterOfMassHavok, 3,
                slot.record.sourceCenterOfMassHavok);
            std::copy_n(contact.targetCenterOfMassHavok, 3,
                slot.record.targetCenterOfMassHavok);
            std::copy_n(contact.sourceContactLocalGame, 3,
                slot.record.sourceContactLocalGame);
            slot.record.closingSpeedHavok = contact.closingSpeedHavok;
            slot.record.tangentSpeedHavok = contact.tangentSpeedHavok;
            slot.record.sourceSurfaceCoordinate = contact.sourceSurfaceCoordinate;
            slot.record.sourceSurfaceDamageCoefficient =
                contact.sourceSurfaceDamageCoefficient;
            slot.record.sourceWeaponGenerationKey =
                contact.sourceWeaponGenerationKey;
            slot.record.sourceGeometryKey = contact.sourceGeometryKey;
            slot.record.sourceWeaponFormId = contact.sourceWeaponFormId;
            slot.record.sourceDescriptorIndex = contact.sourceDescriptorIndex;
            slot.record.sourceSurfaceRegion = contact.sourceSurfaceRegion;
            slot.record.sourceSurfaceConfidencePermille =
                contact.sourceSurfaceConfidencePermille;
            slot.record.targetActorFormId = body->registration.actorFormId;
            slot.record.targetAnatomyFlags = body->registration.anatomyFlags;
            slot.record.targetAnimationBoneCandidateCount =
                body->registration.animationBoneCandidateCount;
            std::copy_n(body->registration.animationBoneCandidates,
                ::rock::provider::ROCK_PROVIDER_MAX_ANATOMY_BONE_CANDIDATES_V1,
                slot.record.targetAnimationBoneCandidates);
            slot.record.targetBodyPartIndex = body->registration.bodyPartIndex;
            slot.record.targetZone = body->registration.targetZone;
            slot.record.targetSide = body->registration.targetSide;
            slot.record.targetBodyPartDamageMultiplier =
                body->registration.bodyPartDamageMultiplier;
            slot.record.targetLimbActorValueFormId =
                body->registration.limbActorValueFormId;
            slot.record.targetNodeNameHash = body->registration.nodeNameHash;
            std::copy_n(body->registration.nodeName,
                ::rock::provider::ROCK_PROVIDER_MAX_EVIDENCE_NAME,
                slot.record.targetNodeName);
            if ((body->registration.anatomyFlags &
                    static_cast<std::uint32_t>(::rock::provider::RockProviderTargetAnatomyFlagV1::BodyPartValid)) != 0) {
                slot.record.flags |= static_cast<std::uint32_t>(
                    ::rock::provider::RockProviderExternalContactFlagV1::TargetAnatomyValid);
            }
            _scopes[body->scopeIndex].latestEmittedSequence =
                contact.sequence;

            if (_contactCount < kMaxContacts) {
                const auto index = (_contactHead + _contactCount) % kMaxContacts;
                _contacts[index] = slot;
                ++_contactCount;
            } else {
                const auto evictedScopeIndex =
                    _contacts[_contactHead].scopeIndex;
                if (evictedScopeIndex < _scopes.size() &&
                    _scopes[evictedScopeIndex].scopeToken != 0) {
                    ++_scopes[evictedScopeIndex].overwrittenCount;
                }
                _contacts[_contactHead] = slot;
                _contactHead = (_contactHead + 1) % kMaxContacts;
            }
            return true;
        }

        [[nodiscard]] std::uint32_t copyContactsForOwnerV1(
            const std::uint64_t parentOwnerToken,
            ::rock::provider::RockProviderExternalContactV1* outContacts,
            const std::uint32_t maxContacts) const
        {
            if (parentOwnerToken == 0 || !outContacts || maxContacts == 0) {
                return 0;
            }
            std::uint32_t copied = 0;
            for (std::uint32_t i = 0; i < _contactCount && copied < maxContacts; ++i) {
                const auto& slot = contactAt(i);
                if (slot.record.parentOwnerToken == parentOwnerToken) {
                    outContacts[copied++] = slot.legacy;
                }
            }
            return copied;
        }

        [[nodiscard]] std::uint32_t copyContactsSinceV1(
            const std::uint64_t parentOwnerToken,
            const std::uint64_t scopeToken,
            const std::uint64_t afterSequence,
            ::rock::provider::RockProviderExternalContactRecordV1* outContacts,
            const std::uint32_t maxContacts,
            ::rock::provider::RockProviderExternalContactStreamStateV1& outState) const
        {
            outState = {};
            outState.oldestRetainedSequence = oldestRetainedSequence(
                parentOwnerToken,
                scopeToken);
            outState.latestEmittedSequence = latestEmittedSequence(
                parentOwnerToken,
                scopeToken);
            outState.overwrittenCount = overwrittenContactCount(
                parentOwnerToken,
                scopeToken);
            const bool gapBeforeRetained =
                afterSequence != 0 &&
                outState.oldestRetainedSequence != 0 &&
                afterSequence < outState.oldestRetainedSequence &&
                outState.oldestRetainedSequence - afterSequence > 1;
            const bool allRequestedRecordsOverwritten =
                afterSequence != 0 &&
                outState.oldestRetainedSequence == 0 &&
                outState.latestEmittedSequence > afterSequence &&
                outState.overwrittenCount != 0;
            if (gapBeforeRetained || allRequestedRecordsOverwritten) {
                outState.flags |= static_cast<std::uint32_t>(
                    ::rock::provider::RockProviderExternalContactStreamFlagV1::GapBeforeFirstCopied);
            }
            if (outState.overwrittenCount != 0) {
                outState.flags |= static_cast<std::uint32_t>(
                    ::rock::provider::RockProviderExternalContactStreamFlagV1::RingOverwroteRecords);
            }

            if (!outContacts || maxContacts == 0) {
                return 0;
            }
            std::uint32_t copied = 0;
            for (std::uint32_t i = 0; i < _contactCount && copied < maxContacts; ++i) {
                const auto& record = contactAt(i).record;
                if (record.parentOwnerToken != parentOwnerToken ||
                    (scopeToken != 0 && record.scopeToken != scopeToken) ||
                    record.sequence <= afterSequence) {
                    continue;
                }
                outContacts[copied++] = record;
            }
            outState.copiedCount = copied;
            if (copied != 0) {
                outState.firstCopiedSequence = outContacts[0].sequence;
                outState.lastCopiedSequence = outContacts[copied - 1].sequence;
            }
            return copied;
        }

        [[nodiscard]] std::uint32_t bodyCount() const { return _bodyCount; }

    private:
        static constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;
        static constexpr std::uint32_t kInvalidIndex = UINT32_MAX;

        struct ScopeSlot
        {
            std::uint64_t parentOwnerToken{ 0 };
            std::uint64_t scopeToken{ 0 };
            std::uint64_t latestEmittedSequence{ 0 };
            std::uint64_t overwrittenCount{ 0 };
        };

        struct BodySlot
        {
            std::uint64_t parentOwnerToken{ 0 };
            std::uint64_t scopeToken{ 0 };
            std::uint32_t scopeIndex{ kInvalidIndex };
            ::rock::provider::RockProviderExternalBodyRegistration registration{};
        };

        struct ContactSlot
        {
            std::uint32_t scopeIndex{ kInvalidIndex };
            ::rock::provider::RockProviderExternalContactV1 legacy{};
            ::rock::provider::RockProviderExternalContactRecordV1 record{};
        };

        struct ImpactEpisodeEntry
        {
            std::uint32_t sourceBodyId{ kInvalidBodyId };
            std::uint32_t targetBodyId{ kInvalidBodyId };
            std::uint64_t lastFrameIndex{ 0 };
            std::uint64_t episodeId{ 0 };
        };

        struct ImpactEpisodeUpdate
        {
            std::uint64_t episodeId{ 0 };
            std::uint32_t flags{ 0 };
        };

        [[nodiscard]] ImpactEpisodeUpdate updateImpactEpisode(
            const std::uint32_t sourceBodyId,
            const std::uint32_t targetBodyId,
            const std::uint64_t frameIndex) noexcept
        {
            static constexpr std::uint64_t kEpisodeGapFrames = 4;
            ImpactEpisodeEntry* empty = nullptr;
            ImpactEpisodeEntry* oldest = &_impactEpisodes[0];
            for (auto& entry : _impactEpisodes) {
                if (entry.episodeId == 0) {
                    if (!empty) {
                        empty = &entry;
                    }
                    continue;
                }
                if (entry.lastFrameIndex < oldest->lastFrameIndex) {
                    oldest = &entry;
                }
                if (entry.sourceBodyId != sourceBodyId ||
                    entry.targetBodyId != targetBodyId) {
                    continue;
                }

                const bool continued = frameIndex >= entry.lastFrameIndex &&
                    frameIndex - entry.lastFrameIndex <= kEpisodeGapFrames;
                entry.lastFrameIndex = frameIndex;
                if (continued) {
                    return {
                        .episodeId = entry.episodeId,
                        .flags = static_cast<std::uint32_t>(
                            ::rock::provider::RockProviderImpactEpisodeFlagV1::Continued),
                    };
                }
                entry.episodeId = _nextEpisodeId++;
                return {
                    .episodeId = entry.episodeId,
                    .flags = static_cast<std::uint32_t>(
                        ::rock::provider::RockProviderImpactEpisodeFlagV1::Started),
                };
            }

            auto* entry = empty ? empty : oldest;
            *entry = {
                .sourceBodyId = sourceBodyId,
                .targetBodyId = targetBodyId,
                .lastFrameIndex = frameIndex,
                .episodeId = _nextEpisodeId++,
            };
            return {
                .episodeId = entry->episodeId,
                .flags = static_cast<std::uint32_t>(
                    ::rock::provider::RockProviderImpactEpisodeFlagV1::Started),
            };
        }

        struct AtomicBodyIndexSlot
        {
            std::atomic<std::uint32_t> bodyId{ kInvalidBodyId };
            std::atomic<std::uint32_t> contactPolicy{ 0 };
        };

        void publishAtomicBodyIndex() noexcept
        {
            struct BodyPolicy
            {
                std::uint32_t bodyId{ kInvalidBodyId };
                std::uint32_t contactPolicy{ 0 };
            };

            std::array<BodyPolicy, kMaxBodies> sorted{};
            for (std::uint32_t i = 0; i < _bodyCount; ++i) {
                sorted[i].bodyId = _bodies[i].registration.bodyId;
                sorted[i].contactPolicy = static_cast<std::uint32_t>(
                    _bodies[i].registration.contactPolicy);
            }
            std::sort(sorted.begin(), sorted.begin() + _bodyCount,
                [](const BodyPolicy& lhs, const BodyPolicy& rhs) {
                    return lhs.bodyId < rhs.bodyId;
                });

            const auto version = _atomicBodyIndexVersion.load(
                std::memory_order_relaxed);
            _atomicBodyIndexVersion.store(
                (version & ~1ull) + 1ull,
                std::memory_order_release);
            for (std::uint32_t i = 0; i < _bodyCount; ++i) {
                _atomicBodyIndex[i].contactPolicy.store(
                    sorted[i].contactPolicy,
                    std::memory_order_relaxed);
                _atomicBodyIndex[i].bodyId.store(
                    sorted[i].bodyId,
                    std::memory_order_relaxed);
            }
            for (std::uint32_t i = _bodyCount; i < kMaxBodies; ++i) {
                _atomicBodyIndex[i].contactPolicy.store(
                    0,
                    std::memory_order_relaxed);
                _atomicBodyIndex[i].bodyId.store(
                    kInvalidBodyId,
                    std::memory_order_relaxed);
            }
            _atomicBodyIndexCount.store(_bodyCount, std::memory_order_release);
            const auto publishingVersion = _atomicBodyIndexVersion.load(
                std::memory_order_relaxed);
            _atomicBodyIndexVersion.store(
                (publishingVersion | 1ull) + 1ull,
                std::memory_order_release);
        }

        [[nodiscard]] bool findAtomicBodyPolicy(
            const std::uint32_t bodyId,
            std::uint32_t* outPolicy) const noexcept
        {
            if (bodyId == kInvalidBodyId) {
                return false;
            }
            for (std::uint32_t attempt = 0; attempt < 4; ++attempt) {
                const auto startVersion = _atomicBodyIndexVersion.load(
                    std::memory_order_acquire);
                if ((startVersion & 1ull) != 0) {
                    continue;
                }

                std::uint32_t low = 0;
                std::uint32_t high = (std::min)(
                    _atomicBodyIndexCount.load(std::memory_order_acquire),
                    kMaxBodies);
                bool found = false;
                std::uint32_t policy = 0;
                while (low < high) {
                    const auto mid = low + ((high - low) / 2u);
                    const auto candidate = _atomicBodyIndex[mid].bodyId.load(
                        std::memory_order_relaxed);
                    if (candidate < bodyId) {
                        low = mid + 1u;
                    } else if (candidate > bodyId) {
                        high = mid;
                    } else {
                        policy = _atomicBodyIndex[mid].contactPolicy.load(
                            std::memory_order_relaxed);
                        found = true;
                        break;
                    }
                }

                const auto endVersion = _atomicBodyIndexVersion.load(
                    std::memory_order_acquire);
                if (startVersion != endVersion || (endVersion & 1ull) != 0) {
                    continue;
                }
                if (found && outPolicy) {
                    *outPolicy = policy;
                }
                return found;
            }
            return false;
        }

        [[nodiscard]] std::uint32_t findScopeIndex(
            const std::uint64_t parentOwnerToken,
            const std::uint64_t scopeToken) const
        {
            for (std::uint32_t index = 0; index < _scopes.size(); ++index) {
                if (_scopes[index].parentOwnerToken == parentOwnerToken &&
                    _scopes[index].scopeToken == scopeToken) {
                    return index;
                }
            }
            return kInvalidIndex;
        }

        [[nodiscard]] std::uint32_t findAvailableScopeIndex() const
        {
            for (std::uint32_t index = 0; index < _scopes.size(); ++index) {
                if (_scopes[index].scopeToken == 0) {
                    return index;
                }
            }
            return kInvalidIndex;
        }

        [[nodiscard]] static bool hasPolicy(
            const ::rock::provider::RockProviderExternalBodyRegistration& body,
            const ::rock::provider::RockProviderExternalBodyContactPolicy policy)
        {
            return (static_cast<std::uint32_t>(body.contactPolicy) &
                    static_cast<std::uint32_t>(policy)) != 0;
        }

        [[nodiscard]] static bool hasAnyReportPolicy(
            const ::rock::provider::RockProviderExternalBodyRegistration& body)
        {
            return hasPolicy(body,
                       ::rock::provider::RockProviderExternalBodyContactPolicy::ReportHandContacts) ||
                   hasPolicy(body,
                       ::rock::provider::RockProviderExternalBodyContactPolicy::ReportAllSourceKinds);
        }

        [[nodiscard]] static bool shouldReportContact(
            const ::rock::provider::RockProviderExternalBodyRegistration& body,
            const ::rock::provider::RockProviderExternalSourceKind sourceKind)
        {
            return sourceKind ==
                       ::rock::provider::RockProviderExternalSourceKind::Hand ?
                hasAnyReportPolicy(body) :
                hasPolicy(body,
                    ::rock::provider::RockProviderExternalBodyContactPolicy::ReportAllSourceKinds);
        }

        [[nodiscard]] const BodySlot* findBody(
            const std::uint32_t bodyId) const
        {
            for (std::uint32_t i = 0; i < _bodyCount; ++i) {
                if (_bodies[i].registration.bodyId == bodyId) {
                    return &_bodies[i];
                }
            }
            return nullptr;
        }

        [[nodiscard]] std::uint32_t countBodiesForScope(
            const std::uint64_t parentOwnerToken,
            const std::uint64_t scopeToken) const
        {
            std::uint32_t count = 0;
            for (std::uint32_t i = 0; i < _bodyCount; ++i) {
                if (_bodies[i].parentOwnerToken == parentOwnerToken &&
                    _bodies[i].scopeToken == scopeToken) {
                    ++count;
                }
            }
            return count;
        }

        [[nodiscard]] bool containsBodyOwnedByOtherScope(
            const std::uint32_t bodyId,
            const std::uint64_t parentOwnerToken,
            const std::uint64_t scopeToken) const
        {
            for (std::uint32_t i = 0; i < _bodyCount; ++i) {
                const auto& body = _bodies[i];
                if (body.registration.bodyId == bodyId &&
                    (body.parentOwnerToken != parentOwnerToken ||
                        body.scopeToken != scopeToken)) {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] static bool batchContainsDuplicateBody(
            const ::rock::provider::RockProviderExternalBodyRegistration* bodies,
            const std::uint32_t countBefore,
            const std::uint32_t bodyId)
        {
            for (std::uint32_t i = 0; i < countBefore; ++i) {
                if (bodies[i].bodyId == bodyId) {
                    return true;
                }
            }
            return false;
        }

        template <class Predicate>
        void removeBodies(Predicate&& shouldRemove)
        {
            std::uint32_t write = 0;
            for (std::uint32_t read = 0; read < _bodyCount; ++read) {
                if (!shouldRemove(_bodies[read])) {
                    _bodies[write++] = _bodies[read];
                }
            }
            for (std::uint32_t i = write; i < _bodyCount; ++i) {
                _bodies[i] = {};
            }
            _bodyCount = write;
        }

        template <class Predicate>
        void removeContacts(Predicate&& shouldRemove)
        {
            const auto originalCount = _contactCount;
            std::uint32_t write = 0;
            for (std::uint32_t read = 0; read < originalCount; ++read) {
                const auto& contact = contactAt(read);
                if (!shouldRemove(contact)) {
                    if (write != read) {
                        _contacts[(_contactHead + write) % kMaxContacts] =
                            contact;
                    }
                    ++write;
                }
            }
            for (std::uint32_t i = write; i < originalCount; ++i) {
                _contacts[(_contactHead + i) % kMaxContacts] = {};
            }
            _contactCount = write;
            if (_contactCount == 0) {
                _contactHead = 0;
            }
        }

        void clearScopeBodies(
            const std::uint64_t parentOwnerToken,
            const std::uint64_t scopeToken)
        {
            removeBodies([&](const BodySlot& body) {
                return body.parentOwnerToken == parentOwnerToken &&
                       body.scopeToken == scopeToken;
            });
        }

        [[nodiscard]] const ContactSlot& contactAt(
            const std::uint32_t logicalIndex) const
        {
            return _contacts[(_contactHead + logicalIndex) % kMaxContacts];
        }

        [[nodiscard]] std::uint64_t oldestRetainedSequence(
            const std::uint64_t parentOwnerToken,
            const std::uint64_t scopeToken) const
        {
            for (std::uint32_t index = 0; index < _contactCount; ++index) {
                const auto& record = contactAt(index).record;
                if (record.parentOwnerToken == parentOwnerToken &&
                    (scopeToken == 0 || record.scopeToken == scopeToken)) {
                    return record.sequence;
                }
            }
            return 0;
        }

        [[nodiscard]] std::uint64_t latestEmittedSequence(
            const std::uint64_t parentOwnerToken,
            const std::uint64_t scopeToken) const
        {
            std::uint64_t latest = 0;
            for (const auto& scope : _scopes) {
                if (scope.parentOwnerToken == parentOwnerToken &&
                    (scopeToken == 0 || scope.scopeToken == scopeToken)) {
                    latest = (std::max)(latest, scope.latestEmittedSequence);
                }
            }
            return latest;
        }

        [[nodiscard]] std::uint64_t overwrittenContactCount(
            const std::uint64_t parentOwnerToken,
            const std::uint64_t scopeToken) const
        {
            std::uint64_t count = 0;
            for (const auto& scope : _scopes) {
                if (scope.parentOwnerToken == parentOwnerToken &&
                    (scopeToken == 0 || scope.scopeToken == scopeToken)) {
                    count += scope.overwrittenCount;
                }
            }
            return count;
        }

        std::array<BodySlot, kMaxBodies> _bodies{};
        std::array<ContactSlot, kMaxContacts> _contacts{};
        std::array<ScopeSlot, kMaxScopes> _scopes{};
        std::uint32_t _bodyCount{ 0 };
        std::uint32_t _contactCount{ 0 };
        std::uint32_t _contactHead{ 0 };
        std::uint64_t _nextContactSequence{ 1 };
        std::array<ImpactEpisodeEntry, kMaxContacts> _impactEpisodes{};
        std::uint64_t _nextImpactId{ 1 };
        std::uint64_t _nextEpisodeId{ 1 };
        std::array<AtomicBodyIndexSlot, kMaxBodies> _atomicBodyIndex{};
        std::atomic<std::uint32_t> _atomicBodyIndexCount{ 0 };
        std::atomic<std::uint64_t> _atomicBodyIndexVersion{ 0 };
    };
}
