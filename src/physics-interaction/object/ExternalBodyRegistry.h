#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

#include "api/ROCKProviderApi.h"

namespace frik::rock
{
    /*
     * PAPER-owned reload bodies need to participate in ROCK hand contact routing
     * without making ROCK understand reload phases again. This registry stores
     * body IDs, owner tokens, and contact policy as plain provider metadata, so
     * the contact listener can report hand-vs-external-body evidence while all
     * reload meaning remains on the PAPER side of the boundary.
     */
    class ExternalBodyRegistry
    {
    public:
        static constexpr std::uint32_t kMaxBodies = ::rock::provider::ROCK_PROVIDER_MAX_EXTERNAL_BODIES_V2;
        static constexpr std::uint32_t kMaxContacts = ::rock::provider::ROCK_PROVIDER_MAX_EXTERNAL_CONTACTS_V2;

        bool registerBodies(
            std::uint64_t ownerToken,
            const ::rock::provider::RockProviderExternalBodyRegistration* bodies,
            std::uint32_t bodyCount)
        {
            if (ownerToken == 0 || (!bodies && bodyCount != 0) || bodyCount > kMaxBodies) {
                return false;
            }

            /*
             * External providers refresh live body maps every frame, while ROCK's
             * contact listener records evidence before provider callbacks run.
             * Replacing body registrations must therefore preserve pending contact
             * evidence; only an explicit owner clear means "drop this session".
             */
            const auto existingOwnerBodyCount = countBodiesForOwner(ownerToken);
            if ((_bodyCount - existingOwnerBodyCount) + bodyCount > kMaxBodies) {
                return false;
            }

            for (std::uint32_t i = 0; i < bodyCount; ++i) {
                const auto& body = bodies[i];
                if (body.size != sizeof(::rock::provider::RockProviderExternalBodyRegistration) || body.ownerToken != ownerToken || body.bodyId == kInvalidBodyId ||
                    containsBodyOwnedByOther(body.bodyId, ownerToken) || batchContainsDuplicateBody(bodies, i, body.bodyId)) {
                    return false;
                }
            }

            clearOwnerBodies(ownerToken);
            for (std::uint32_t i = 0; i < bodyCount; ++i) {
                _bodies[_bodyCount++] = bodies[i];
            }

            return true;
        }

        void clearOwner(std::uint64_t ownerToken)
        {
            if (ownerToken == 0) {
                clearAll();
                return;
            }

            std::uint32_t write = 0;
            for (std::uint32_t read = 0; read < _bodyCount; ++read) {
                if (_bodies[read].ownerToken != ownerToken) {
                    _bodies[write++] = _bodies[read];
                }
            }
            clearBodyRange(write, _bodyCount);
            _bodyCount = write;

            write = 0;
            for (std::uint32_t read = 0; read < _contactCount; ++read) {
                if (_contacts[read].ownerToken != ownerToken) {
                    _contacts[write++] = _contacts[read];
                }
            }
            clearContactRange(write, _contactCount);
            _contactCount = write;
        }

        void clearAll()
        {
            clearBodyRange(0, _bodyCount);
            clearContactRange(0, _contactCount);
            _bodyCount = 0;
            _contactCount = 0;
            _nextContactSequence = 1;
        }

        [[nodiscard]] bool containsBody(std::uint32_t bodyId) const
        {
            return findBody(bodyId) != nullptr;
        }

        [[nodiscard]] bool suppressesRockDynamicPush(std::uint32_t bodyId) const
        {
            const auto* body = findBody(bodyId);
            return body && hasPolicy(*body, ::rock::provider::RockProviderExternalBodyContactPolicy::SuppressRockDynamicPush);
        }

        [[nodiscard]] bool tryGetBody(std::uint32_t bodyId, ::rock::provider::RockProviderExternalBodyRegistration& outBody) const
        {
            const auto* body = findBody(bodyId);
            if (!body) {
                return false;
            }

            outBody = *body;
            return true;
        }

        void recordHandContact(bool isLeft, std::uint32_t handBodyId, std::uint32_t externalBodyId, std::uint64_t frameIndex)
        {
            const auto* body = findBody(externalBodyId);
            if (!body || !hasAnyReportPolicy(*body)) {
                return;
            }

            ::rock::provider::RockProviderExternalContactV2 contact{};
            contact.sourceBodyId = handBodyId;
            contact.targetExternalBodyId = externalBodyId;
            contact.frameIndex = frameIndex;
            contact.sourceKind = ::rock::provider::RockProviderExternalSourceKind::Hand;
            contact.sourceHand = isLeft ? ::rock::provider::RockProviderHand::Left : ::rock::provider::RockProviderHand::Right;
            contact.quality = ::rock::provider::RockProviderExternalContactQuality::BodyPairOnly;
            recordContactV2(contact);
        }

        bool recordContactV2(::rock::provider::RockProviderExternalContactV2 contact)
        {
            if (contact.size != sizeof(::rock::provider::RockProviderExternalContactV2) || contact.sourceBodyId == kInvalidBodyId ||
                contact.targetExternalBodyId == kInvalidBodyId || contact.sourceBodyId == contact.targetExternalBodyId) {
                return false;
            }

            const auto* body = findBody(contact.targetExternalBodyId);
            if (!body || !shouldReportContact(*body, contact.sourceKind)) {
                return false;
            }

            contact.generation = body->generation;
            contact.ownerToken = body->ownerToken;
            contact.targetRole = body->role;
            if (contact.sequence == 0) {
                contact.sequence = _nextContactSequence++;
            }

            if (_contactCount < kMaxContacts) {
                _contacts[_contactCount++] = contact;
                return true;
            }

            for (std::uint32_t i = 1; i < kMaxContacts; ++i) {
                _contacts[i - 1] = _contacts[i];
            }
            _contacts.back() = contact;
            return true;
        }

        [[nodiscard]] std::uint32_t copyContacts(::rock::provider::RockProviderExternalContact* outContacts, std::uint32_t maxContacts) const
        {
            if (!outContacts || maxContacts == 0) {
                return 0;
            }

            std::uint32_t count = 0;
            std::array<::rock::provider::RockProviderExternalContact, kMaxContacts> latest{};
            for (std::uint32_t read = _contactCount; read > 0 && count < maxContacts; --read) {
                const std::uint32_t i = read - 1;
                const auto& source = _contacts[i];
                if (source.sourceKind != ::rock::provider::RockProviderExternalSourceKind::Hand) {
                    continue;
                }

                ::rock::provider::RockProviderExternalContact contact{};
                contact.handBodyId = source.sourceBodyId;
                contact.externalBodyId = source.targetExternalBodyId;
                contact.generation = source.generation;
                contact.ownerToken = source.ownerToken;
                contact.sequence = source.sequence;
                contact.hand = source.sourceHand;
                contact.role = source.targetRole;
                latest[count++] = contact;
            }

            for (std::uint32_t i = 0; i < count; ++i) {
                outContacts[i] = latest[count - 1 - i];
            }
            return count;
        }

        [[nodiscard]] std::uint32_t copyContactsV2(::rock::provider::RockProviderExternalContactV2* outContacts, std::uint32_t maxContacts) const
        {
            if (!outContacts || maxContacts == 0) {
                return 0;
            }

            const std::uint32_t count = (std::min)(maxContacts, _contactCount);
            const std::uint32_t start = _contactCount - count;
            for (std::uint32_t i = 0; i < count; ++i) {
                outContacts[i] = _contacts[start + i];
            }
            return count;
        }

        [[nodiscard]] std::uint32_t bodyCount() const { return _bodyCount; }

    private:
        static constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;

        [[nodiscard]] static bool hasPolicy(
            const ::rock::provider::RockProviderExternalBodyRegistration& body,
            ::rock::provider::RockProviderExternalBodyContactPolicy policy)
        {
            return (static_cast<std::uint32_t>(body.contactPolicy) & static_cast<std::uint32_t>(policy)) != 0;
        }

        [[nodiscard]] static bool hasAnyReportPolicy(const ::rock::provider::RockProviderExternalBodyRegistration& body)
        {
            return hasPolicy(body, ::rock::provider::RockProviderExternalBodyContactPolicy::ReportHandContacts) ||
                   hasPolicy(body, ::rock::provider::RockProviderExternalBodyContactPolicy::ReportAllSourceKinds);
        }

        [[nodiscard]] static bool shouldReportContact(
            const ::rock::provider::RockProviderExternalBodyRegistration& body,
            ::rock::provider::RockProviderExternalSourceKind sourceKind)
        {
            if (sourceKind == ::rock::provider::RockProviderExternalSourceKind::Hand) {
                return hasAnyReportPolicy(body);
            }

            return hasPolicy(body, ::rock::provider::RockProviderExternalBodyContactPolicy::ReportAllSourceKinds);
        }

        [[nodiscard]] const ::rock::provider::RockProviderExternalBodyRegistration* findBody(std::uint32_t bodyId) const
        {
            for (std::uint32_t i = 0; i < _bodyCount; ++i) {
                if (_bodies[i].bodyId == bodyId) {
                    return &_bodies[i];
                }
            }
            return nullptr;
        }

        [[nodiscard]] std::uint32_t countBodiesForOwner(std::uint64_t ownerToken) const
        {
            std::uint32_t count = 0;
            for (std::uint32_t i = 0; i < _bodyCount; ++i) {
                if (_bodies[i].ownerToken == ownerToken) {
                    ++count;
                }
            }
            return count;
        }

        [[nodiscard]] bool containsBodyOwnedByOther(std::uint32_t bodyId, std::uint64_t ownerToken) const
        {
            for (std::uint32_t i = 0; i < _bodyCount; ++i) {
                if (_bodies[i].bodyId == bodyId && _bodies[i].ownerToken != ownerToken) {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] static bool batchContainsDuplicateBody(
            const ::rock::provider::RockProviderExternalBodyRegistration* bodies,
            std::uint32_t countBefore,
            std::uint32_t bodyId)
        {
            for (std::uint32_t i = 0; i < countBefore; ++i) {
                if (bodies[i].bodyId == bodyId) {
                    return true;
                }
            }
            return false;
        }

        void clearOwnerBodies(std::uint64_t ownerToken)
        {
            std::uint32_t write = 0;
            for (std::uint32_t read = 0; read < _bodyCount; ++read) {
                if (_bodies[read].ownerToken != ownerToken) {
                    _bodies[write++] = _bodies[read];
                }
            }
            clearBodyRange(write, _bodyCount);
            _bodyCount = write;
        }

        void clearBodyRange(std::uint32_t begin, std::uint32_t end)
        {
            for (std::uint32_t i = begin; i < end && i < kMaxBodies; ++i) {
                _bodies[i] = {};
            }
        }

        void clearContactRange(std::uint32_t begin, std::uint32_t end)
        {
            for (std::uint32_t i = begin; i < end && i < kMaxContacts; ++i) {
                _contacts[i] = {};
            }
        }

        std::array<::rock::provider::RockProviderExternalBodyRegistration, kMaxBodies> _bodies{};
        std::array<::rock::provider::RockProviderExternalContactV2, kMaxContacts> _contacts{};
        std::uint32_t _bodyCount{ 0 };
        std::uint32_t _contactCount{ 0 };
        std::uint64_t _nextContactSequence{ 1 };
    };
}
