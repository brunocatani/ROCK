#pragma once
#include "Abi.h"
#pragma pack(push,8)
namespace rock::api::storage_feedback
{
    inline constexpr char kExport[] = "ROCK_StorageFeedback_QueryV1";
    struct CandidateV1 {
        std::uint32_t size{sizeof(CandidateV1)}, version{1};
        Hand hand{Hand::None};
        std::uint32_t reference{}, expectedBase{}, worldGeneration{}, skeletonGeneration{}, providerGeneration{};
        std::uint64_t gripSequence{};
    };
    struct ApiV1 {
        std::uint32_t size{sizeof(ApiV1)}, version{1};
        // ROCK owner-thread calls, Grab Read|Control owner. Refresh each frame;
        // the lease expires after three frames and on exact hand/target loss.
        // Green, fast pulsing whole-object selection effect for a held item.
        // reference=0 means released ROCK's native-equipped expectedBase;
        // ROCK V2 accepts loose references only. Busy yields shared VATS color
        // to ordinary selection/native owners. No global config is changed.
        Status(ROCK_CALL* setCandidate)(OwnerToken,const CandidateV1*) noexcept;
        Status(ROCK_CALL* clearCandidate)(OwnerToken,Hand) noexcept;
    };
    using Query = const ApiV1*(ROCK_CALL*)(std::uint32_t major,std::uint32_t bytes) noexcept;
    static_assert(sizeof(CandidateV1)==40 && sizeof(ApiV1)==24);
}
#pragma pack(pop)
