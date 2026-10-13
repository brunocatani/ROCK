#pragma once
#include "Abi.h"
#pragma pack(push,8)
namespace rock::api::object_rest {
    inline constexpr char kExport[]="ROCK_ObjectRest_QueryV1";
    inline constexpr std::uint32_t kMaximumWatches=16;
    enum class State : std::uint32_t { Waiting, Watching, Resting };
    struct RequestV1 {
        std::uint32_t size{sizeof(RequestV1)},version{1};
        ReferenceKey reference{};
        std::uint32_t expectedBase{};
    };
    struct ApiV1 {
        std::uint32_t size{sizeof(ApiV1)},version{1};
        // Owner-thread only, Core owner with Grab Read|Control. Exact MISC
        // references only; no native pointer, global scan or collision rebuild.
        // RequestQueued admits a bounded watch. Poll with the same request;
        // Ok reports Watching or Resting. Resting acknowledges completion and
        // frees its watch. Retry CapacityFull/NotReady/TargetUnavailable later.
        // Held items cannot park. Two seconds of supported rest freezes the
        // native bodies and saves their final pose; ordinary grabbing wakes them.
        // Owner/lifecycle/scene loss cancels unfinished watches. Submit again
        // after regrab/drop or load. Completed native placement is not undone.
        Status(ROCK_CALL* watch)(OwnerToken,const RequestV1*,State*) noexcept;
        // reference=0 cancels this owner's unfinished watches; otherwise one.
        Status(ROCK_CALL* cancel)(OwnerToken,std::uint32_t reference) noexcept;
    };
    using Query=const ApiV1*(ROCK_CALL*)(std::uint32_t major,std::uint32_t bytes) noexcept;
    static_assert(sizeof(RequestV1)==32&&sizeof(ApiV1)==24);
}
#pragma pack(pop)
