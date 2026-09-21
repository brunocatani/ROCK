#include "ReferencesMarshalling.h"
#include <ROCK/Discovery.h>

namespace rock::api::references {
namespace {
using namespace boundary;
Status ROCK_CALL queryReferenceInteractionV1(std::uint64_t ownerToken, const ReferenceQueryV1* query, ReferenceInteractionV1* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        if (const auto s = checkInput(query); s != Status::Ok) return s;
        provider::RockProviderReferenceQueryV1 native_query{};
        convert(native_query, *query);
        provider::RockProviderReferenceInteractionV1 native_outState{};
        const auto result = provider::runtime::apiQueryReferenceInteractionV1(ownerToken, &native_query, &native_outState);
        convert(*outState, native_outState);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL queryPowerArmorTargetV1(std::uint64_t ownerToken, const ReferenceQueryV1* query, PowerArmorTargetV1* outTarget) noexcept {
    if (const auto s = checkOutput(outTarget); s != Status::Ok) return s;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        if (const auto s = checkInput(query); s != Status::Ok) return s;
        provider::RockProviderReferenceQueryV1 native_query{};
        convert(native_query, *query);
        provider::RockProviderPowerArmorTargetV1 native_outTarget{};
        const auto result = provider::runtime::apiQueryPowerArmorTargetV1(ownerToken, &native_query, &native_outTarget);
        convert(*outTarget, native_outTarget);
        return static_cast<Status>(result);
    });
}

#include "ReferencesEndpoints.inl"
}
const ApiV1& table() noexcept {
    static const ApiV1 value{
        &queryReferenceInteractionV1,
        &queryPowerArmorTargetV1,
        &getSample,
    };
    return value;
}
}
