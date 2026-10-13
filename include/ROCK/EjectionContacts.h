#pragma once
#include "Abi.h"
// Independent opt-in service. Existing provider tables and behavior are unchanged.
// Calls require the frame owner thread and Grab Read|Control permission.
#pragma pack(push,8)
namespace rock::api::ejection_contacts {
inline constexpr char kExport[]="ROCK_EjectionContacts_QueryV1";
struct RequestV1 {
    std::uint32_t size{sizeof(RequestV1)},version{1};
    ReferenceKey reference{};
    std::uint64_t weaponGenerationKey{};
    std::uint32_t expectedBase{},weaponForm{};
};
struct ApiV1 {
    std::uint32_t size{sizeof(ApiV1)},version{1};
    // One second of exact object/equipped-weapon pair exclusion after bodies
    // become available (up to 1.5s load wait). World/other objects still collide.
    // Repeating the same request observes it; it never extends the deadline.
    // Queued while waiting; Ok after activation, including completed requests.
    // An expired identical request stays completed and never renews the lease.
    // Capacity is eight references. Owner/generation/weapon changes revoke it.
    Status(ROCK_CALL* request)(OwnerToken,const RequestV1*) noexcept;
    // Cancels this owner's pending and active references; zero cancels all.
    Status(ROCK_CALL* cancel)(OwnerToken,std::uint32_t reference) noexcept;
};
using Query=const ApiV1*(ROCK_CALL*)(std::uint32_t major,std::uint32_t bytes) noexcept;
static_assert(sizeof(RequestV1)==48);
static_assert(offsetof(RequestV1,weaponGenerationKey)==32);
static_assert(sizeof(ApiV1)==24);
}
#pragma pack(pop)
