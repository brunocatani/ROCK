#define ROCK_API_EXPORTS
/*
 * Build the DLL side of the public provider ABI.
 *
 * EXTERNAL BODIES: consumer-registered Havok bodies, their scopes, and the contact
 * streams they produce.
 *
 * The body lock joins the game thread and the physics thread. recordExternalContact
 * is reached from the contact callback, so keep every critical section here short
 * and free of allocation and logging.
 */
#include "api/detail/ProviderApiEntryPoints.h"
#include "api/detail/ProviderApiCore.h"
#include "api/detail/ProviderApiState.h"

#include <mutex>

namespace rock::provider::detail
{
    bool ROCK_PROVIDER_CALL apiRegisterExternalBodiesV1(
        std::uint64_t ownerToken,
        const RockProviderExternalBodyRegistration* bodies,
        std::uint32_t bodyCount)
    {
        std::scoped_lock lock(s_consumerMutex, s_externalBodyMutex);
        if (!consumerHasCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::ExternalBodies)) {
            return false;
        }
        return s_externalBodies.registerBodies(ownerToken, bodies, bodyCount);
    }

    void ROCK_PROVIDER_CALL apiClearExternalBodies(std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return;
        }
        std::scoped_lock lock(s_consumerMutex, s_externalBodyMutex);
        if (!consumerHasCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::ExternalBodies)) {
            return;
        }
        s_externalBodies.clearOwner(ownerToken);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRegisterExternalBodiesForScopeV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const RockProviderExternalBodyRegistration* bodies,
        const std::uint32_t bodyCount)
    {
        if (ownerToken == 0 || scopeToken == 0 ||
            (bodyCount != 0 && !bodies)) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_externalBodyMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::ExternalBodyScopes);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        switch (s_externalBodies.registerBodiesForScopeDetailed(
            ownerToken,
            scopeToken,
            bodies,
            bodyCount)) {
        case ExternalBodyRegistry::RegistrationResult::Ok:
            return RockProviderResultV1::Ok;
        case ExternalBodyRegistry::RegistrationResult::CapacityFull:
            return RockProviderResultV1::CapacityFull;
        case ExternalBodyRegistry::RegistrationResult::OwnerConflict:
            return RockProviderResultV1::OwnerConflict;
        case ExternalBodyRegistry::RegistrationResult::InvalidArgument:
        default:
            return RockProviderResultV1::InvalidArgument;
        }
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearExternalBodiesForScopeV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken)
    {
        if (ownerToken == 0 || scopeToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_externalBodyMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::ExternalBodyScopes);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        return s_externalBodies.clearScope(ownerToken, scopeToken) ?
            RockProviderResultV1::Ok :
            RockProviderResultV1::TargetUnavailable;
    }

    std::uint32_t ROCK_PROVIDER_CALL apiGetExternalContactSnapshotForOwnerV1(
        std::uint64_t ownerToken,
        RockProviderExternalContactV1* outContacts,
        std::uint32_t maxContacts)
    {
        if (ownerToken == 0 || !outContacts || maxContacts == 0) {
            return 0;
        }

        std::scoped_lock lock(s_consumerMutex, s_externalBodyMutex);
        if (!consumerHasCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::ExternalContacts)) {
            return 0;
        }
        return s_externalBodies.copyContactsForOwnerV1(ownerToken, outContacts, maxContacts);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopyExternalContactsSinceV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const std::uint64_t afterSequence,
        RockProviderExternalContactRecordV1* outContacts,
        const std::uint32_t maxContacts,
        RockProviderExternalContactStreamStateV1* outStreamState)
    {
        const auto entryResult = validateOutputEntry(
            ownerToken,
            outStreamState);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        if (maxContacts != 0 && !outContacts) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_externalBodyMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::ExternalBodyScopes);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        if (!consumerHasCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::ExternalContacts)) {
            return RockProviderResultV1::PermissionDenied;
        }
        (void)s_externalBodies.copyContactsSinceV1(
            ownerToken,
            scopeToken,
            afterSequence,
            outContacts,
            maxContacts,
            *outStreamState);
        return RockProviderResultV1::Ok;
    }
}

namespace rock::provider
{
    using namespace detail;

    bool isExternalBodyId(std::uint32_t bodyId)
    {
        std::scoped_lock lock(s_externalBodyMutex);
        return s_externalBodies.containsBody(bodyId);
    }

    bool isExternalBodyDynamicPushSuppressed(std::uint32_t bodyId)
    {
        std::scoped_lock lock(s_externalBodyMutex);
        return s_externalBodies.suppressesRockDynamicPush(bodyId);
    }

    bool recordExternalHandContact(bool isLeft, std::uint32_t handBodyId, std::uint32_t externalBodyId, std::uint64_t frameIndex)
    {
        std::scoped_lock lock(s_externalBodyMutex);
        if (!s_externalBodies.containsBody(externalBodyId)) {
            return false;
        }
        s_externalBodies.recordHandContact(isLeft, handBodyId, externalBodyId, frameIndex);
        return true;
    }

    bool recordExternalContact(
        const RockProviderExternalContactV1& contact,
        const std::uint32_t worldGeneration,
        const std::uint32_t skeletonGeneration,
        const std::uint32_t providerGeneration)
    {
        std::scoped_lock lock(s_externalBodyMutex);
        return s_externalBodies.recordContactV1(
            contact,
            worldGeneration,
            skeletonGeneration,
            providerGeneration);
    }

    std::uint32_t currentExternalBodyCount()
    {
        std::scoped_lock lock(s_externalBodyMutex);
        return s_externalBodies.bodyCount();
    }

}
