#include "api/ROCKProviderApi.h"

#include <cassert>
#include <initializer_list>

namespace
{
    using namespace rock::provider;
    struct Reference { std::uint32_t formID; };
    Reference rightItem{ 0xFF001234 }, leftItem{ 0xFF005678 };
    constexpr std::uint64_t owner = 42;
    constexpr auto valid = static_cast<std::uint32_t>(RockProviderHandInteractionFlagV1::Valid);
    RockProviderHandInteractionStateV1 hands[2]{};
    RockProviderFrameSnapshot g_snapshot{};
    RockProviderForceGrabRequestV1 submitted{};
    RockProviderResultV1 readResult = RockProviderResultV1::Ok;
    RockProviderResultV1 g_admission = RockProviderResultV1::RequestQueued;
    bool snapshotAvailable = true;
    unsigned lookups = 0, queries = 0, requests = 0;

    Reference* resolve(std::uint32_t formId)
    {
        ++lookups;
        return formId == rightItem.formID ? &rightItem :
            formId == leftItem.formID ? &leftItem : nullptr;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL query(
        std::uint64_t token, RockProviderHand hand, RockProviderHandInteractionStateV1* out)
    {
        ++queries;
        assert(token == owner);
        *out = hands[hand == RockProviderHand::Left ? 1 : 0];
        return readResult;
    }

    bool ROCK_PROVIDER_CALL getSnapshot(RockProviderFrameSnapshot* out)
    {
        *out = g_snapshot;
        return snapshotAvailable;
    }

    bool ROCK_PROVIDER_CALL getLimits(RockProviderLimitsV1* out)
    {
        out->providerApiByteSize = sizeof(RockProviderApi);
        out->featureBits = static_cast<std::uint32_t>(RockProviderFeatureBitV1::InteractionCommandQueue) |
            static_cast<std::uint32_t>(RockProviderFeatureBitV1::ForceGrabCommand);
        return true;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL request(
        std::uint64_t token, const RockProviderForceGrabRequestV1* value, std::uint64_t* id)
    {
        ++requests;
        assert(token == owner);
        submitted = *value;
        *id = g_admission == RockProviderResultV1::RequestQueued ? 73 : 0;
        return g_admission;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL getCommand(
        std::uint64_t, std::uint64_t, RockProviderInteractionCommandResultV1*)
    {
        return RockProviderResultV1::RequestNotFound;
    }
}

int main()
{
    using namespace rock::provider;
    RockProviderApi table{};
    table.getHandInteractionStateV1 = &query;
    table.getFrameSnapshot = &getSnapshot;
    table.getProviderLimitsV1 = &getLimits;
    table.requestForceGrabV1 = &request;
    table.getInteractionCommandResultV1 = &getCommand;
    RockProviderApi::inst = &table;
    RockProviderApi::negotiatedTableByteSize = sizeof(table);
    RockProviderApi::negotiatedFeatureBits2 =
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::HandInteractionState);
    g_snapshot.worldGeneration = 3;
    g_snapshot.skeletonGeneration = 4;
    g_snapshot.providerGeneration = 5;

    RockHandItems<Reference> items{ owner, &resolve };
    hands[0].flags = hands[1].flags = valid;
    hands[0].targetKind = hands[1].targetKind = RockProviderBodyContactTargetKind::HeldObject;
    hands[0].targetFormId = rightItem.formID;
    hands[1].targetFormId = leftItem.formID;
    RockProviderResultV1 result{};
    for (auto phase : { RockProviderHandInteractionPhaseV1::Holding,
             RockProviderHandInteractionPhaseV1::Catching,
             RockProviderHandInteractionPhaseV1::StashCandidate,
             RockProviderHandInteractionPhaseV1::ConsumeCandidate }) {
        hands[0].phase = hands[1].phase = phase;
        assert(items.GetHeldItem(false, &result) == &rightItem);
        assert(result == RockProviderResultV1::Ok);
        assert(items.GetHeldItem(true) == &leftItem);
    }

    // The release event retains the old target identity without still holding it.
    auto releaseLookups = lookups;
    hands[0].phase = RockProviderHandInteractionPhaseV1::Releasing;
    assert(!items.GetHeldItem(false, &result) && result == RockProviderResultV1::Ok);
    assert(lookups == releaseLookups);
    hands[0].phase = RockProviderHandInteractionPhaseV1::Holding;

    // A failed query must not resolve even a populated output into an object.
    auto previousLookups = lookups;
    for (auto error : { RockProviderResultV1::PermissionDenied,
             RockProviderResultV1::OwnerNotRegistered, RockProviderResultV1::NotReady }) {
        readResult = error;
        assert(!items.GetHeldItem(false, &result));
        assert(result == error && lookups == previousLookups);
    }
    readResult = RockProviderResultV1::Ok;
    hands[0].flags = 0;
    assert(!items.GetHeldItem(false, &result) && result == RockProviderResultV1::NotReady);
    hands[0].flags = valid;
    hands[0].targetKind = RockProviderBodyContactTargetKind::DynamicProp;
    hands[0].phase = RockProviderHandInteractionPhaseV1::Selecting;
    assert(!items.GetHeldItem(false, &result) && result == RockProviderResultV1::Ok);
    assert(lookups == previousLookups);

    // An equipped weapon occupies the hand but its base form is not a held REFR.
    hands[0].phase = RockProviderHandInteractionPhaseV1::Holding;
    hands[0].targetKind = RockProviderBodyContactTargetKind::Weapon;
    hands[0].flags = valid | static_cast<std::uint32_t>(RockProviderHandInteractionFlagV1::FiringGrip) |
        static_cast<std::uint32_t>(RockProviderHandInteractionFlagV1::NativeWeaponCarry);
    assert(!items.GetHeldItem(false, &result) && result == RockProviderResultV1::Ok);
    assert(lookups == previousLookups);
    hands[0].targetKind = RockProviderBodyContactTargetKind::DynamicProp;
    hands[0].flags = valid;

    // Touch grips may have a world reference; global surfaces may have none.
    hands[0].flags |= static_cast<std::uint32_t>(RockProviderHandInteractionFlagV1::TouchGrab);
    hands[0].phase = RockProviderHandInteractionPhaseV1::Holding;
    assert(items.GetHeldItem(false) == &rightItem);
    hands[0].targetFormId = 0;
    hands[0].targetKind = RockProviderBodyContactTargetKind::WorldSurface;
    assert(!items.GetHeldItem(false, &result) && result == RockProviderResultV1::Ok);
    hands[0].targetFormId = 0xFF009999;
    assert(!items.GetHeldItem(false, &result) && result == RockProviderResultV1::TargetUnavailable);

    // Submission preserves the reference identity, current guards and command
    // ID without requiring a physics body ID or claiming synchronous success.
    auto grab = items.RequestGrabItem(true, &leftItem, 100.0f);
    assert(grab.result == RockProviderResultV1::RequestQueued && grab.commandId == 73);
    assert(submitted.hand == RockProviderHand::Left && submitted.targetFormId == leftItem.formID);
    assert(submitted.targetBodyId == 0x7FFFFFFF && submitted.targetRefr == 0);
    assert(submitted.worldGeneration == 3 && submitted.skeletonGeneration == 4 && submitted.providerGeneration == 5);
    assert(submitted.maxDistanceGame == 100.0f);
    g_snapshot.worldGeneration = 8;
    g_admission = RockProviderResultV1::HandBusy;
    grab = items.RequestGrabItem(false, &rightItem);
    assert(grab.result == RockProviderResultV1::HandBusy && grab.commandId == 0);
    assert(submitted.hand == RockProviderHand::Right && submitted.targetFormId == rightItem.formID);
    assert(submitted.worldGeneration == 8 && submitted.maxDistanceGame == 0.0f);

    auto previousRequests = requests;
    assert(items.RequestGrabItem(false, nullptr).result == RockProviderResultV1::InvalidArgument);
    snapshotAvailable = false;
    assert(items.RequestGrabItem(false, &rightItem).result == RockProviderResultV1::NotReady);
    snapshotAvailable = true;
    g_snapshot.skeletonGeneration = 0;
    assert(items.RequestGrabItem(false, &rightItem).result == RockProviderResultV1::NotReady);
    g_snapshot.skeletonGeneration = 4;
    assert(requests == previousRequests);

    // Negotiate table extent before touching a newer function slot.
    const auto previousQueries = queries;
    RockProviderApi::negotiatedTableByteSize = ROCK_PROVIDER_API_V1_HAND_INTERACTION_STATE_TABLE_BYTES - 1;
    assert(!items.GetHeldItem(false, &result) && result == RockProviderResultV1::UnsupportedVersion);
    assert(queries == previousQueries);
    RockProviderApi::negotiatedTableByteSize = ROCK_PROVIDER_API_V1_FORCE_GRAB_TABLE_BYTES - 1;
    assert(items.RequestGrabItem(false, &rightItem).result == RockProviderResultV1::UnsupportedVersion);
    assert(requests == previousRequests);
    RockProviderApi::negotiatedTableByteSize = sizeof(table);
    RockProviderApi::negotiatedFeatureBits2 = 0;
    assert(!items.GetHeldItem(false, &result) && result == RockProviderResultV1::UnsupportedVersion);
    assert(queries == previousQueries);
    table.requestForceGrabV1 = nullptr;
    assert(items.RequestGrabItem(false, &rightItem).result == RockProviderResultV1::UnsupportedVersion);
    assert(requests == previousRequests);
    RockProviderApi::inst = nullptr;
    assert(!items.GetHeldItem(false, &result) && result == RockProviderResultV1::NotReady);
    assert(items.RequestGrabItem(false, &rightItem).result == RockProviderResultV1::NotReady);
    return 0;
}
