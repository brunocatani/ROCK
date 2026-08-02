#include "api/ProviderColliderVisualizationRuntime.h"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>

using namespace rock;
using namespace rock::provider;

int main()
{
    provider_collider_visualization::clearAll();
    assert(!provider_collider_visualization::hasOverride());

    RockProviderColliderVisualizationRequestV1 request{};
    request.weaponGenerationKey = 0x1234;
    request.bodyId = 17;
    request.partKind = static_cast<std::uint32_t>(
        RockProviderWeaponPartKindV1::Magazine);
    request.leaseFrames = 2;
    request.worldGeneration = 4;
    request.skeletonGeneration = 5;
    request.providerGeneration = 6;

    assert(provider_collider_visualization::set(10, request, 100) ==
           RockProviderResultV1::Ok);
    assert(provider_collider_visualization::hasOverride());

    provider_collider_visualization::Snapshot snapshot{};
    assert(provider_collider_visualization::copySnapshot(snapshot));
    assert(snapshot.ownerToken == 10);
    assert(snapshot.weaponGenerationKey == request.weaponGenerationKey);
    assert(snapshot.bodyId == request.bodyId);
    assert(snapshot.partKind == request.partKind);

    assert(provider_collider_visualization::set(11, request, 100) ==
           RockProviderResultV1::OwnerConflict);

    provider_collider_visualization::Invalidation invalidation{};
    provider_collider_visualization::prune(
        101,
        4,
        5,
        6,
        0x1234,
        true,
        invalidation);
    assert(invalidation.ownerToken == 0);
    assert(provider_collider_visualization::hasOverride());

    provider_collider_visualization::prune(
        102,
        4,
        5,
        6,
        0x1234,
        true,
        invalidation);
    assert(invalidation.ownerToken == 10);
    assert(invalidation.reason ==
           RockProviderSuppressionInvalidationReasonV1::Expired);
    assert(!provider_collider_visualization::hasOverride());

    request.leaseFrames =
        ROCK_PROVIDER_MAX_COLLIDER_VISUALIZATION_OVERRIDE_LEASE_FRAMES_V1 +
        1;
    assert(provider_collider_visualization::set(20, request, 200) ==
           RockProviderResultV1::Ok);
    provider_collider_visualization::prune(
        200 +
            ROCK_PROVIDER_MAX_COLLIDER_VISUALIZATION_OVERRIDE_LEASE_FRAMES_V1 -
            1,
        4,
        5,
        6,
        0x1234,
        true,
        invalidation);
    assert(invalidation.ownerToken == 0);
    provider_collider_visualization::prune(
        200 +
            ROCK_PROVIDER_MAX_COLLIDER_VISUALIZATION_OVERRIDE_LEASE_FRAMES_V1,
        4,
        5,
        6,
        0x1234,
        true,
        invalidation);
    assert(invalidation.ownerToken == 20);

    request.leaseFrames = 5;
    assert(provider_collider_visualization::set(30, request, 300) ==
           RockProviderResultV1::Ok);
    provider_collider_visualization::prune(
        300,
        4,
        5,
        6,
        0x1234,
        false,
        invalidation);
    assert(invalidation.ownerToken == 30);
    assert(invalidation.reason ==
           RockProviderSuppressionInvalidationReasonV1::GenerationChanged);

    assert(provider_collider_visualization::set(40, request, 400) ==
           RockProviderResultV1::Ok);
    provider_collider_visualization::clear(41);
    assert(provider_collider_visualization::hasOverride());
    provider_collider_visualization::clear(40);
    assert(!provider_collider_visualization::hasOverride());

    assert(provider_collider_visualization::set(50, request, 500) ==
           RockProviderResultV1::Ok);
    provider_collider_visualization::clearAll(
        invalidation,
        RockProviderSuppressionInvalidationReasonV1::ProviderLost);
    assert(invalidation.ownerToken == 50);
    assert(invalidation.reason ==
           RockProviderSuppressionInvalidationReasonV1::ProviderLost);
    assert(!provider_collider_visualization::hasOverride());

    return 0;
}
