#pragma once
#include "EjectionContacts.h"
// Opt-in release clearance against the equipped weapon AND both physical
// hands. V1 retains its weapon-only behavior and unchanged table/records.
// RequestV1 remains the exact reference identity record (version == 1).
#pragma pack(push,8)
namespace rock::api::ejection_contacts::v2 {
inline constexpr char kExport[]="ROCK_EjectionContacts_QueryV2";
struct Api {
    std::uint32_t size{sizeof(Api)},version{2};
    // Same ownership, frame thread, identity, capacity, cancellation and
    // non-renewing one-second lifetime as V1. Includes exact live hand bodies;
    // preserves world/other objects, ray queries and grab acquisition.
    Status(ROCK_CALL* request)(OwnerToken,const RequestV1*) noexcept;
    Status(ROCK_CALL* cancel)(OwnerToken,std::uint32_t reference) noexcept;
};
using Query=const Api*(ROCK_CALL*)(std::uint32_t major,std::uint32_t bytes) noexcept;
static_assert(sizeof(Api)==24);
}
#pragma pack(pop)
