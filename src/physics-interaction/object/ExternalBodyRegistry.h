#pragma once

#include <algorithm>
#include <array>
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
        }

        [[nodiscard]] bool containsBody(const std::uint32_t bodyId) const
        {
            return findBody(bodyId) != nullptr;
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
    };
}
