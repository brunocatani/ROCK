#include "physics-interaction/object/ExternalBodyRegistry.h"

#include <array>
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

namespace
{
    using namespace rock;
    using namespace rock::provider;

    RockProviderExternalBodyRegistration body(
        const std::uint64_t scopeToken,
        const std::uint32_t bodyId,
        const std::uint32_t generation = 1,
        const RockProviderExternalBodyContactPolicy policy =
            RockProviderExternalBodyContactPolicy::ReportAllSourceKinds)
    {
        RockProviderExternalBodyRegistration result{};
        result.bodyId = bodyId;
        result.ownerToken = scopeToken;
        result.generation = generation;
        result.role = RockProviderExternalBodyRole::ActorRagdollBone;
        result.contactPolicy = policy;
        return result;
    }

    RockProviderExternalContactV1 contact(
        const std::uint32_t sourceBodyId,
        const std::uint32_t targetBodyId,
        const std::uint64_t frameIndex,
        const RockProviderExternalSourceKind sourceKind =
            RockProviderExternalSourceKind::Hand)
    {
        RockProviderExternalContactV1 result{};
        result.sourceBodyId = sourceBodyId;
        result.targetExternalBodyId = targetBodyId;
        result.frameIndex = frameIndex;
        result.sourceKind = sourceKind;
        result.sourceHand = RockProviderHand::Left;
        result.quality = RockProviderExternalContactQuality::RawPoint;
        result.contactPointHavok[0] = 1.0f;
        result.contactNormalHavok[2] = 1.0f;
        result.sourceVelocityHavok[1] = 2.0f;
        result.contactPointWeightSum = 3.0f;
        result.sourcePartKind = 4;
        result.sourceRole = 5;
        result.sourceSubRole = 6;
        result.collisionGeneration = 7;
        return result;
    }

    void testValidationAndScopeOwnership()
    {
        using RegistrationResult = ExternalBodyRegistry::RegistrationResult;
        constexpr std::uint64_t ownerA = 0xA001;
        constexpr std::uint64_t ownerB = 0xA002;
        constexpr std::uint64_t scopeA = 0xB001;
        constexpr std::uint64_t scopeB = 0xB002;

        auto registry = std::make_unique<ExternalBodyRegistry>();
        assert(registry->registerBodiesForScopeDetailed(
                   0,
                   scopeA,
                   nullptr,
                   0) == RegistrationResult::InvalidArgument);
        assert(!registry->registerBodiesForScope(0, scopeA, nullptr, 0));
        assert(!registry->registerBodiesForScope(ownerA, 0, nullptr, 0));
        assert(registry->registerBodiesForScope(ownerA, scopeA, nullptr, 0));
        assert(registry->clearScope(ownerA, scopeA));
        assert(!registry->clearScope(ownerA, scopeA));

        auto valid = body(scopeA, 100);
        assert(registry->registerBodiesForScope(ownerA, scopeA, &valid, 1));
        assert(registry->containsBody(100));

        auto wrongScope = body(scopeB, 101);
        assert(!registry->registerBodiesForScope(ownerA, scopeA, &wrongScope, 1));
        assert(registry->containsBody(100));

        auto invalidBody = body(scopeB, 0x7FFF'FFFFu);
        assert(!registry->registerBodiesForScope(ownerA, scopeB, &invalidBody, 1));

        std::array duplicateBatch{ body(scopeB, 101), body(scopeB, 101) };
        assert(!registry->registerBodiesForScope(
            ownerA,
            scopeB,
            duplicateBatch.data(),
            static_cast<std::uint32_t>(duplicateBatch.size())));

        auto conflictingBody = body(scopeB, 100);
        assert(registry->registerBodiesForScopeDetailed(
                   ownerA,
                   scopeB,
                   &conflictingBody,
                   1) == RegistrationResult::OwnerConflict);
        assert(!registry->registerBodiesForScope(ownerA, scopeB, &conflictingBody, 1));
        assert(!registry->registerBodiesForScope(ownerB, scopeB, &conflictingBody, 1));

        auto validB = body(scopeB, 200);
        assert(registry->registerBodiesForScope(ownerB, scopeB, &validB, 1));
        assert(registry->bodyCount() == 2);

        registry->clearOwner(ownerA);
        assert(!registry->containsBody(100));
        assert(registry->containsBody(200));
        assert(registry->bodyCount() == 1);
    }

    void testContactDemultiplexingAndRefresh()
    {
        constexpr std::uint64_t owner = 0xA010;
        constexpr std::uint64_t otherOwner = 0xA011;
        constexpr std::uint64_t scopeA = 0xB010;
        constexpr std::uint64_t scopeB = 0xB011;
        constexpr std::uint64_t otherScope = 0xB012;

        auto registry = std::make_unique<ExternalBodyRegistry>();
        auto bodyA = body(scopeA, 300, 11);
        auto bodyB = body(scopeB, 301, 12);
        auto bodyOther = body(otherScope, 302, 13);
        assert(registry->registerBodiesForScope(owner, scopeA, &bodyA, 1));
        assert(registry->registerBodiesForScope(owner, scopeB, &bodyB, 1));
        assert(registry->registerBodiesForScope(otherOwner, otherScope, &bodyOther, 1));

        assert(registry->recordContactV1(contact(10, 300, 100), 21, 22, 23));
        assert(registry->recordContactV1(contact(11, 301, 101), 31, 32, 33));
        assert(registry->recordContactV1(contact(12, 302, 102), 41, 42, 43));

        std::array<RockProviderExternalContactRecordV1, 4> rows{};
        RockProviderExternalContactStreamStateV1 state{};
        assert(registry->copyContactsSinceV1(
                   owner,
                   scopeA,
                   0,
                   rows.data(),
                   static_cast<std::uint32_t>(rows.size()),
                   state) == 1);
        assert(rows[0].parentOwnerToken == owner);
        assert(rows[0].scopeToken == scopeA);
        assert(rows[0].bodyGeneration == 11);
        assert(rows[0].worldGeneration == 21);
        assert(rows[0].skeletonGeneration == 22);
        assert(rows[0].providerGeneration == 23);
        assert(rows[0].collisionGeneration == 7);
        assert(rows[0].contactPointHavok[0] == 1.0f);
        assert(rows[0].contactNormalHavok[2] == 1.0f);
        assert(rows[0].sourceVelocityHavok[1] == 2.0f);
        assert(rows[0].contactPointWeightSum == 3.0f);

        std::array<RockProviderExternalContactV1, 4> legacyRows{};
        assert(registry->copyContactsForOwnerV1(
                   owner,
                   legacyRows.data(),
                   static_cast<std::uint32_t>(legacyRows.size())) == 2);
        assert(legacyRows[0].ownerToken == scopeA);
        assert(legacyRows[1].ownerToken == scopeB);

        bodyA.generation = 14;
        assert(registry->registerBodiesForScope(owner, scopeA, &bodyA, 1));
        assert(registry->recordContactV1(contact(13, 300, 103), 51, 52, 53));
        rows = {};
        state = {};
        assert(registry->copyContactsSinceV1(
                   owner,
                   scopeA,
                   0,
                   rows.data(),
                   static_cast<std::uint32_t>(rows.size()),
                   state) == 2);
        assert(rows[0].bodyGeneration == 11);
        assert(rows[1].bodyGeneration == 14);
        assert(state.latestEmittedSequence == rows[1].sequence);

        assert(registry->clearScope(owner, scopeA));
        assert(!registry->containsBody(300));
        assert(registry->containsBody(301));
        rows = {};
        state = {};
        assert(registry->copyContactsSinceV1(
                   owner,
                   scopeA,
                   0,
                   rows.data(),
                   static_cast<std::uint32_t>(rows.size()),
                   state) == 0);
        assert(state.latestEmittedSequence == 0);
        assert(registry->copyContactsSinceV1(
                   owner,
                   scopeB,
                   0,
                   rows.data(),
                   static_cast<std::uint32_t>(rows.size()),
                   state) == 1);
    }

    void testContactPolicyAndScopedLossAccounting()
    {
        constexpr std::uint64_t owner = 0xA020;
        constexpr std::uint64_t scopeA = 0xB020;
        constexpr std::uint64_t scopeB = 0xB021;

        auto registry = std::make_unique<ExternalBodyRegistry>();
        auto handOnly = body(
            scopeA,
            400,
            1,
            RockProviderExternalBodyContactPolicy::ReportHandContacts);
        auto allSources = body(scopeB, 401);
        assert(registry->registerBodiesForScope(owner, scopeA, &handOnly, 1));
        assert(registry->registerBodiesForScope(owner, scopeB, &allSources, 1));
        assert(!registry->recordContactV1(contact(
            30,
            400,
            1,
            RockProviderExternalSourceKind::Weapon)));

        for (std::uint32_t index = 0;
             index < ExternalBodyRegistry::kMaxContacts;
             ++index) {
            assert(registry->recordContactV1(contact(30, 400, index + 1)));
        }
        assert(registry->recordContactV1(contact(31, 401, 1000)));
        assert(registry->recordContactV1(contact(32, 401, 1001)));

        auto rows = std::make_unique<std::array<
            RockProviderExternalContactRecordV1,
            ExternalBodyRegistry::kMaxContacts>>();
        RockProviderExternalContactStreamStateV1 stateA{};
        assert(registry->copyContactsSinceV1(
                   owner,
                   scopeA,
                   1,
                   rows->data(),
                   static_cast<std::uint32_t>(rows->size()),
                   stateA) == ExternalBodyRegistry::kMaxContacts - 2);
        assert(stateA.oldestRetainedSequence == 3);
        assert(stateA.latestEmittedSequence == ExternalBodyRegistry::kMaxContacts);
        assert(stateA.overwrittenCount == 2);
        assert((stateA.flags & static_cast<std::uint32_t>(
            RockProviderExternalContactStreamFlagV1::GapBeforeFirstCopied)) != 0);
        assert((stateA.flags & static_cast<std::uint32_t>(
            RockProviderExternalContactStreamFlagV1::RingOverwroteRecords)) != 0);

        RockProviderExternalContactStreamStateV1 stateB{};
        assert(registry->copyContactsSinceV1(
                   owner,
                   scopeB,
                   0,
                   rows->data(),
                   static_cast<std::uint32_t>(rows->size()),
                   stateB) == 2);
        assert(stateB.overwrittenCount == 0);
        assert(stateB.flags == 0);

        stateB = {};
        assert(registry->copyContactsSinceV1(
                   owner,
                   scopeB,
                   (std::numeric_limits<std::uint64_t>::max)(),
                   rows->data(),
                   static_cast<std::uint32_t>(rows->size()),
                   stateB) == 0);
        assert(stateB.flags == 0);
    }

    void testCapacityFailureIsTransactional()
    {
        using RegistrationResult = ExternalBodyRegistry::RegistrationResult;
        constexpr std::uint64_t owner = 0xA030;
        constexpr std::uint64_t scopeA = 0xB030;
        constexpr std::uint64_t scopeB = 0xB031;

        auto registry = std::make_unique<ExternalBodyRegistry>();
        std::vector<RockProviderExternalBodyRegistration> bodies;
        bodies.reserve(ExternalBodyRegistry::kMaxBodies - 1);
        for (std::uint32_t index = 0;
             index < ExternalBodyRegistry::kMaxBodies - 1;
             ++index) {
            bodies.push_back(body(scopeA, 1000 + index));
        }
        assert(registry->registerBodiesForScope(
            owner,
            scopeA,
            bodies.data(),
            static_cast<std::uint32_t>(bodies.size())));
        auto existingB = body(scopeB, 9000, 7);
        assert(registry->registerBodiesForScope(owner, scopeB, &existingB, 1));
        assert(registry->bodyCount() == ExternalBodyRegistry::kMaxBodies);

        std::array replacementB{ body(scopeB, 9001), body(scopeB, 9002) };
        assert(registry->registerBodiesForScopeDetailed(
                   owner,
                   scopeB,
                   replacementB.data(),
                   static_cast<std::uint32_t>(replacementB.size())) ==
               RegistrationResult::CapacityFull);
        assert(!registry->registerBodiesForScope(
            owner,
            scopeB,
            replacementB.data(),
            static_cast<std::uint32_t>(replacementB.size())));
        assert(registry->bodyCount() == ExternalBodyRegistry::kMaxBodies);
        assert(registry->containsBody(9000));
        assert(!registry->containsBody(9001));
        RockProviderExternalBodyRegistration retained{};
        assert(registry->tryGetBody(9000, retained));
        assert(retained.generation == 7);

        auto scopeRegistry = std::make_unique<ExternalBodyRegistry>();
        for (std::uint32_t index = 0;
             index < ExternalBodyRegistry::kMaxScopes;
             ++index) {
            assert(scopeRegistry->registerBodiesForScopeDetailed(
                       owner,
                       0xC000 + index,
                       nullptr,
                       0) == RegistrationResult::Ok);
        }
        assert(scopeRegistry->registerBodiesForScopeDetailed(
                   owner,
                   0xD000,
                   nullptr,
                   0) == RegistrationResult::CapacityFull);

        auto oneBody = body(scopeA, 42);
        assert(scopeRegistry->registerBodiesForScopeDetailed(
                   owner,
                   scopeA,
                   &oneBody,
                   ExternalBodyRegistry::kMaxBodies + 1) ==
               RegistrationResult::CapacityFull);
    }
}

int main()
{
    testValidationAndScopeOwnership();
    testContactDemultiplexingAndRefresh();
    testContactPolicyAndScopedLossAccounting();
    testCapacityFailureIsTransactional();
    return 0;
}
