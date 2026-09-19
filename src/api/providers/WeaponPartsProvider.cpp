#include "WeaponPartsMarshalling.h"
#include <ROCK/Discovery.h>
#include "EventBoundary.h"

namespace rock::api::weaponparts {
namespace {
using namespace boundary;
template<class T> Status validateSourceSelector(T& target) {
    constexpr auto byName=static_cast<std::uint32_t>(provider::RockProviderWeaponPartTargetFlagV1::MatchSourceName);
    constexpr auto byKey=static_cast<std::uint32_t>(provider::RockProviderWeaponPartTargetFlagV1::MatchSourceRoot);
    if (target.flags & byName) {
        if (!std::memchr(target.sourceName,0,sizeof(target.sourceName))) return Status::InvalidArgument;
        const auto node=provider::runtime::resolveSourceName(target.weaponGenerationKey,target.sourceName);
        if (!node) return Status::Ambiguous;
        if ((target.flags & byKey) && target.sourceRoot!=node) return Status::TargetInvalid;
        target.sourceRoot=node; target.flags=(target.flags & ~byName)|byKey;
    }
    return Status::Ok;
}
Status ROCK_CALL queryWeaponContactAtPoint(OwnerToken ownerToken, const WeaponContactQuery* query, WeaponContactResult* outResult) noexcept {
    if (const auto s = checkOutput(outResult); s != Status::Ok) return s;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::runtime::refreshSources();
        if (const auto s = checkInput(query); s != Status::Ok) return s;
        provider::RockProviderWeaponContactQuery native_query{};
        convert(native_query, *query);
        provider::RockProviderWeaponContactResult native_outResult{};
        const auto result = provider::runtime::apiQueryWeaponContactAtPoint(&native_query, &native_outResult);
        convert(*outResult, native_outResult);
        return result ? Status::Ok : Status::NotReady;
    });
}
Status ROCK_CALL getWeaponEvidenceDetailCountV1(OwnerToken ownerToken, std::uint32_t* outValue) noexcept {
    if (!outValue) return Status::InvalidArgument;
    *outValue = {};
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        *outValue = static_cast<std::uint32_t>(provider::runtime::apiGetWeaponEvidenceDetailCountV1());
        return Status::Ok;
    });
}
Status ROCK_CALL copyWeaponEvidenceDetailsV1(OwnerToken ownerToken, WeaponEvidenceDetailV1* outDetails, std::uint32_t maxDetails, std::uint32_t* outCopied) noexcept {
    if (!outCopied) return Status::InvalidArgument;
    *outCopied = {};
    if (maxDetails > kMaxEvidenceDetails) return Status::CapacityFull;
    if (maxDetails && !outDetails) return Status::InvalidArgument;
    for (std::uint32_t i=0;i<maxDetails;++i) if (const auto status=checkOutput(outDetails+i); status!=Status::Ok) return status;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::runtime::refreshSources();
        std::array<provider::RockProviderWeaponEvidenceDetailV1, kMaxEvidenceDetails> native_outDetails{};
        *outCopied = static_cast<std::uint32_t>(provider::runtime::apiCopyWeaponEvidenceDetailsV1(native_outDetails.data(), maxDetails));
        for (std::uint32_t i=0; i<std::min<std::uint32_t>(maxDetails, *outCopied); ++i) convert(outDetails[i], native_outDetails[i]);
        return Status::Ok;
    });
}
Status ROCK_CALL getWeaponEvidenceDetailPointCountV1(OwnerToken ownerToken, std::uint32_t bodyId, std::uint32_t* outValue) noexcept {
    if (!outValue) return Status::InvalidArgument;
    *outValue = {};
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        *outValue = static_cast<std::uint32_t>(provider::runtime::apiGetWeaponEvidenceDetailPointCountV1(bodyId));
        return Status::Ok;
    });
}
Status ROCK_CALL copyWeaponEvidenceDetailPointsV1(OwnerToken ownerToken, std::uint32_t bodyId, Point3* outPoints, std::uint32_t maxPoints, std::uint32_t* outCopied) noexcept {
    if (!outCopied) return Status::InvalidArgument;
    *outCopied = {};
    if (maxPoints > kMaxEvidencePoints) return Status::CapacityFull;
    if (maxPoints && !outPoints) return Status::InvalidArgument;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        std::array<provider::RockProviderPoint3, kMaxEvidencePoints> native_outPoints{};
        *outCopied = static_cast<std::uint32_t>(provider::runtime::apiCopyWeaponEvidenceDetailPointsV1(bodyId, native_outPoints.data(), maxPoints));
        for (std::uint32_t i=0; i<std::min<std::uint32_t>(maxPoints, *outCopied); ++i) convert(outPoints[i], native_outPoints[i]);
        return Status::Ok;
    });
}
Status ROCK_CALL setWeaponPartTargetsV1(std::uint64_t ownerToken, const WeaponPartTargetV1* targets, std::uint32_t targetCount) noexcept {
    if (targetCount > kMaxTargets) return Status::CapacityFull;
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        provider::runtime::refreshSources();
        if (targetCount && !targets) return Status::InvalidArgument;
        std::array<provider::RockProviderWeaponPartTargetV1, kMaxTargets> native_targets{};
        for (std::uint32_t i=0; i<targetCount; ++i) {
            if (const auto s = checkInput(targets+i); s != Status::Ok) return s;
            convert(native_targets[i], targets[i]);
            if (const auto status=validateSourceSelector(native_targets[i]); status!=Status::Ok) return status;
            if (targets[i].sourceKey && !native_targets[i].sourceRoot) return Status::GenerationMismatch;
        }
        const auto result = provider::runtime::apiSetWeaponPartTargetsV1(ownerToken, native_targets.data(), targetCount);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL clearWeaponPartTargetsV1(std::uint64_t ownerToken) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        const auto result = provider::runtime::apiClearWeaponPartTargetsV1(ownerToken);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL setWeaponPartDriveTargetsV1(std::uint64_t ownerToken, const WeaponPartDriveTargetV1* targets, std::uint32_t targetCount) noexcept {
    if (targetCount > kMaxDrives) return Status::CapacityFull;
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        provider::runtime::refreshSources();
        if (targetCount && !targets) return Status::InvalidArgument;
        std::array<provider::RockProviderWeaponPartDriveTargetV1, kMaxDrives> native_targets{};
        for (std::uint32_t i=0; i<targetCount; ++i) {
            if (const auto s = checkInput(targets+i); s != Status::Ok) return s;
            convert(native_targets[i], targets[i]);
            if (const auto status=validateSourceSelector(native_targets[i]); status!=Status::Ok) return status;
            if (targets[i].sourceKey && !native_targets[i].sourceRoot) return Status::GenerationMismatch;
        }
        const auto result = provider::runtime::apiSetWeaponPartDriveTargetsV1(ownerToken, native_targets.data(), targetCount);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL clearWeaponPartDriveTargetsV1(std::uint64_t ownerToken) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        const auto result = provider::runtime::apiClearWeaponPartDriveTargetsV1(ownerToken);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL getWeaponPartGripStateV1(OwnerToken ownerToken, Hand hand, WeaponPartGripStateV1* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    if (hand!=Hand::Right && hand!=Hand::Left) return Status::InvalidArgument;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::runtime::refreshSources();
        provider::RockProviderWeaponPartGripStateV1 native_outState{};
        const auto result = provider::runtime::apiGetWeaponPartGripStateV1(static_cast<provider::RockProviderHand>(hand), &native_outState);
        convert(*outState, native_outState);
        return result ? Status::Ok : Status::NotReady;
    });
}
Status ROCK_CALL queryWeaponPartTargetResolutionV1(std::uint64_t ownerToken, const WeaponPartResolutionQueryV1* query, WeaponPartResolutionResultV1* outResolution) noexcept {
    if (const auto s = checkOutput(outResolution); s != Status::Ok) return s;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::runtime::refreshSources();
        if (const auto s = checkInput(query); s != Status::Ok) return s;
        provider::RockProviderWeaponPartResolutionQueryV1 native_query{};
        convert(native_query, *query);
        if (query->sourceKey && !native_query.sourceRoot) return Status::GenerationMismatch;
        provider::RockProviderWeaponPartResolutionResultV1 native_outResolution{};
        const auto result = provider::runtime::apiQueryWeaponPartTargetResolutionV1(ownerToken, &native_query, &native_outResolution);
        convert(*outResolution, native_outResolution);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL copyWeaponPartPoseSnapshotV1(std::uint64_t ownerToken, WeaponPartPoseV1* outParts, std::uint32_t maxParts, std::uint32_t* outPartCount) noexcept {
    if (maxParts > kMaxPoses) return Status::CapacityFull;
    if (maxParts && !outParts) return Status::InvalidArgument;
    for (std::uint32_t i=0;i<maxParts;++i) if (const auto status=checkOutput(outParts+i); status!=Status::Ok) return status;
    if (!outPartCount) return Status::InvalidArgument;
    *outPartCount = {};
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::runtime::refreshSources();
        std::array<provider::RockProviderWeaponPartPoseV1, kMaxPoses> native_outParts{};
        const auto result = provider::runtime::apiCopyWeaponPartPoseSnapshotV1(ownerToken, native_outParts.data(), maxParts, outPartCount);
        for (std::uint32_t i=0; i<std::min<std::uint32_t>(maxParts, *outPartCount); ++i) convert(outParts[i], native_outParts[i]);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL copyWeaponPartDriveApplicationResultsV1(std::uint64_t ownerToken, WeaponPartDriveApplicationResultV1* outResults, std::uint32_t maxResults, std::uint32_t* outResultCount) noexcept {
    if (maxResults > kMaxDrives) return Status::CapacityFull;
    if (maxResults && !outResults) return Status::InvalidArgument;
    for (std::uint32_t i=0;i<maxResults;++i) if (const auto status=checkOutput(outResults+i); status!=Status::Ok) return status;
    if (!outResultCount) return Status::InvalidArgument;
    *outResultCount = {};
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::runtime::refreshSources();
        std::array<provider::RockProviderWeaponPartDriveApplicationResultV1, kMaxDrives> native_outResults{};
        const auto result = provider::runtime::apiCopyWeaponPartDriveApplicationResultsV1(ownerToken, native_outResults.data(), maxResults, outResultCount);
        for (std::uint32_t i=0; i<std::min<std::uint32_t>(maxResults, *outResultCount); ++i) convert(outResults[i], native_outResults[i]);
        return static_cast<Status>(result);
    });
}

#include "WeaponPartsEndpoints.inl"
}
const ApiV1& table() noexcept {
    static const ApiV1 value{
        &queryWeaponContactAtPoint,
        &getWeaponEvidenceDetailCountV1,
        &copyWeaponEvidenceDetailsV1,
        &getWeaponEvidenceDetailPointCountV1,
        &copyWeaponEvidenceDetailPointsV1,
        &setWeaponPartTargetsV1,
        &clearWeaponPartTargetsV1,
        &setWeaponPartDriveTargetsV1,
        &clearWeaponPartDriveTargetsV1,
        &getWeaponPartGripStateV1,
        &queryWeaponPartTargetResolutionV1,
        &copyWeaponPartPoseSnapshotV1,
        &copyWeaponPartDriveApplicationResultsV1,
        &getSample,
        &copySources,
        &copyEvents,
        &querySourcePose,
        &querySourcePath,
    };
    return value;
}
}
