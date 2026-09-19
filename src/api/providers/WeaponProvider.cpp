#include "WeaponMarshalling.h"
#include <ROCK/Discovery.h>
#include "EventBoundary.h"

namespace rock::api::weapon {
namespace {
using namespace boundary;
Status ROCK_CALL getPrimaryHandV1(OwnerToken ownerToken, Hand* outValue) noexcept {
    if (!outValue) return Status::InvalidArgument;
    *outValue = {};
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        *outValue = static_cast<Hand>(provider::runtime::apiGetPrimaryHandV1());
        return Status::Ok;
    });
}
Status ROCK_CALL getOffhandHandV1(OwnerToken ownerToken, Hand* outValue) noexcept {
    if (!outValue) return Status::InvalidArgument;
    *outValue = {};
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        *outValue = static_cast<Hand>(provider::runtime::apiGetOffhandHandV1());
        return Status::Ok;
    });
}
Status ROCK_CALL queryEquippedWeaponClassificationV1(OwnerToken ownerToken, WeaponClassificationV1* outResult) noexcept {
    if (const auto s = checkOutput(outResult); s != Status::Ok) return s;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::RockProviderWeaponClassificationV1 native_outResult{};
        const auto result = provider::runtime::apiQueryEquippedWeaponClassificationV1(&native_outResult);
        convert(*outResult, native_outResult);
        return result ? Status::Ok : Status::NotReady;
    });
}
Status ROCK_CALL getWeaponEmitterCountV1(OwnerToken ownerToken, std::uint32_t* outValue) noexcept {
    if (!outValue) return Status::InvalidArgument;
    *outValue = {};
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        *outValue = static_cast<std::uint32_t>(provider::runtime::apiGetWeaponEmitterCountV1());
        return Status::Ok;
    });
}
Status ROCK_CALL copyWeaponEmittersV1(OwnerToken ownerToken, WeaponEmitterV1* outEmitters, std::uint32_t maxEmitters, std::uint32_t* outCopied) noexcept {
    if (!outCopied) return Status::InvalidArgument;
    *outCopied = {};
    if (maxEmitters > kMaxEmitters) return Status::CapacityFull;
    if (maxEmitters && !outEmitters) return Status::InvalidArgument;
    for (std::uint32_t i=0;i<maxEmitters;++i) if (const auto status=checkOutput(outEmitters+i); status!=Status::Ok) return status;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        std::array<provider::RockProviderWeaponEmitterV1, kMaxEmitters> native_outEmitters{};
        *outCopied = static_cast<std::uint32_t>(provider::runtime::apiCopyWeaponEmittersV1(native_outEmitters.data(), maxEmitters));
        for (std::uint32_t i=0; i<std::min<std::uint32_t>(maxEmitters, *outCopied); ++i) convert(outEmitters[i], native_outEmitters[i]);
        return Status::Ok;
    });
}
Status ROCK_CALL getEquippedWeaponGripStateV1(std::uint64_t ownerToken, EquippedWeaponGripStateV1* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::RockProviderEquippedWeaponGripStateV1 native_outState{};
        const auto result = provider::runtime::apiGetEquippedWeaponGripStateV1(ownerToken, &native_outState);
        convert(*outState, native_outState);
        return result ? Status::Ok : Status::NotReady;
    });
}
Status ROCK_CALL setEquippedWeaponHandlingAuthorityV1(std::uint64_t ownerToken, const EquippedWeaponHandlingRequestV1* request) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        if (const auto s = checkInput(request); s != Status::Ok) return s;
        provider::RockProviderEquippedWeaponHandlingRequestV1 native_request{};
        convert(native_request, *request);
        if (request->flags & ~0x3Fu) return Status::InvalidArgument;
        const auto result = provider::runtime::apiSetEquippedWeaponHandlingAuthorityV1(ownerToken, &native_request);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL clearEquippedWeaponHandlingAuthorityV1(std::uint64_t ownerToken) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        const auto result = provider::runtime::apiClearEquippedWeaponHandlingAuthorityV1(ownerToken);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL getEquippedWeaponHandlingStateV1(OwnerToken ownerToken, EquippedWeaponHandlingStateV1* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::RockProviderEquippedWeaponHandlingStateV1 native_outState{};
        const auto result = provider::runtime::apiGetEquippedWeaponHandlingStateV1(&native_outState);
        convert(*outState, native_outState);
        return result ? Status::Ok : Status::NotReady;
    });
}
Status ROCK_CALL getEquippedWeaponStateV1(std::uint64_t ownerToken, EquippedWeaponStateV1* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::RockProviderEquippedWeaponStateV1 native_outState{};
        const auto result = provider::runtime::apiGetEquippedWeaponStateV1(ownerToken, &native_outState);
        convert(*outState, native_outState);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL getScopeSightStateV1(std::uint64_t ownerToken, ScopeSightStateV1* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::RockProviderScopeSightStateV1 native_outState{};
        const auto result = provider::runtime::apiGetScopeSightStateV1(ownerToken, &native_outState);
        convert(*outState, native_outState);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL getWeaponCompositionStateV1(std::uint64_t ownerToken, WeaponCompositionStateV1* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::RockProviderWeaponCompositionStateV1 native_outState{};
        const auto result = provider::runtime::apiGetWeaponCompositionStateV1(ownerToken, &native_outState);
        convert(*outState, native_outState);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL copyWeaponCompositionEntriesV1(std::uint64_t ownerToken, WeaponCompositionEntryV1* outEntries, std::uint32_t maxEntries, std::uint32_t* outEntryCount) noexcept {
    if (maxEntries > kMaxCompositionEntries) return Status::CapacityFull;
    if (maxEntries && !outEntries) return Status::InvalidArgument;
    for (std::uint32_t i=0;i<maxEntries;++i) if (const auto status=checkOutput(outEntries+i); status!=Status::Ok) return status;
    if (!outEntryCount) return Status::InvalidArgument;
    *outEntryCount = {};
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        std::array<provider::RockProviderWeaponCompositionEntryV1, kMaxCompositionEntries> native_outEntries{};
        const auto result = provider::runtime::apiCopyWeaponCompositionEntriesV1(ownerToken, native_outEntries.data(), maxEntries, outEntryCount);
        for (std::uint32_t i=0; i<std::min<std::uint32_t>(maxEntries, *outEntryCount); ++i) convert(outEntries[i], native_outEntries[i]);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL getSelectedAuthoredGripPoseV1(std::uint64_t ownerToken, AuthoredGripPoseV1* outPose) noexcept {
    if (const auto s = checkOutput(outPose); s != Status::Ok) return s;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::RockProviderAuthoredGripPoseV1 native_outPose{};
        const auto result = provider::runtime::apiGetSelectedAuthoredGripPoseV1(ownerToken, &native_outPose);
        convert(*outPose, native_outPose);
        return static_cast<Status>(result);
    });
}

#include "WeaponEndpoints.inl"
}
const ApiV1& table() noexcept {
    static const ApiV1 value{
        &getPrimaryHandV1,
        &getOffhandHandV1,
        &queryEquippedWeaponClassificationV1,
        &getWeaponEmitterCountV1,
        &copyWeaponEmittersV1,
        &getEquippedWeaponGripStateV1,
        &setEquippedWeaponHandlingAuthorityV1,
        &clearEquippedWeaponHandlingAuthorityV1,
        &getEquippedWeaponHandlingStateV1,
        &getEquippedWeaponStateV1,
        &getScopeSightStateV1,
        &getWeaponCompositionStateV1,
        &copyWeaponCompositionEntriesV1,
        &getSelectedAuthoredGripPoseV1,
        &getSample,
        &copyEvents,
    };
    return value;
}
}
