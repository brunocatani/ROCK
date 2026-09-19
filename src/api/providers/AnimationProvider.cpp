#include "AnimationMarshalling.h"
#include <ROCK/Discovery.h>
#include "EventBoundary.h"

namespace rock::api::animation {
namespace {
using namespace boundary;
Status ROCK_CALL setNativeAnimationAuthorityV1(std::uint64_t ownerToken, const NativeAnimationAuthorityRequestV1* request) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        if (const auto s = checkInput(request); s != Status::Ok) return s;
        provider::RockProviderNativeAnimationAuthorityRequestV1 native_request{};
        convert(native_request, *request);
        const auto result = provider::runtime::apiSetNativeAnimationAuthorityV1(ownerToken, &native_request);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL clearNativeAnimationAuthorityV1(std::uint64_t ownerToken) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        const auto result = provider::runtime::apiClearNativeAnimationAuthorityV1(ownerToken);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL getNativeAnimationAuthorityStateV1(OwnerToken ownerToken, NativeAnimationAuthorityStateV1* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::RockProviderNativeAnimationAuthorityStateV1 native_outState{};
        const auto result = provider::runtime::apiGetNativeAnimationAuthorityStateV1(&native_outState);
        convert(*outState, native_outState);
        return result ? Status::Ok : Status::NotReady;
    });
}
Status ROCK_CALL setHandVisualAuthorityV1(std::uint64_t ownerToken, const HandVisualAuthorityRequestV1* request) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        if (const auto s = checkInput(request); s != Status::Ok) return s;
        provider::RockProviderHandVisualAuthorityRequestV1 native_request{};
        convert(native_request, *request);
        const auto result = provider::runtime::apiSetHandVisualAuthorityV1(ownerToken, &native_request);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL clearHandVisualAuthorityV1(std::uint64_t ownerToken, Hand hand) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        const auto result = provider::runtime::apiClearHandVisualAuthorityV1(ownerToken, static_cast<provider::RockProviderHand>(hand));
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL publishNativeAnimationRuntimeV1(std::uint64_t ownerToken, const NativeAnimationRuntimePublicationV1* publication) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        if (const auto s = checkInput(publication); s != Status::Ok) return s;
        provider::RockProviderNativeAnimationRuntimePublicationV1 native_publication{};
        convert(native_publication, *publication);
        const auto result = provider::runtime::apiPublishNativeAnimationRuntimeV1(ownerToken, &native_publication);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL clearNativeAnimationRuntimeV1(std::uint64_t ownerToken) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        const auto result = provider::runtime::apiClearNativeAnimationRuntimeV1(ownerToken);
        return static_cast<Status>(result);
    });
}

#include "AnimationEndpoints.inl"
}
const ApiV1& table() noexcept {
    static const ApiV1 value{
        &setNativeAnimationAuthorityV1,
        &clearNativeAnimationAuthorityV1,
        &getNativeAnimationAuthorityStateV1,
        &setHandVisualAuthorityV1,
        &clearHandVisualAuthorityV1,
        &publishNativeAnimationRuntimeV1,
        &clearNativeAnimationRuntimeV1,
        &getSample,
        &copyEvents,
    };
    return value;
}
}
