#pragma once
#include "Abi.h"
namespace rock::api::stash_conversion {
inline constexpr char kExport[]="ROCK_StashConversion_QueryV1";
enum class Decision : std::uint32_t { Pass=0, Reject=1, Consumed=2 };
// Called synchronously on the frame owner after physical detachment and before
// native inventory activation. A converter owns its transaction and must never
// return Pass for a recognized item it could not convert. No exceptions/reentry.
struct Result {Decision decision{Decision::Reject};std::uint32_t inventoryForm{},count{};};
using Convert=Result(ROCK_CALL*)(std::uint32_t reference,std::uint32_t base,void* context) noexcept;
struct ApiV1 {
    std::uint32_t size{sizeof(ApiV1)},version{1};
    // Grab Read|Control, frame-owner thread. One registration per owner, eight
    // total. Null callback unregisters. Context must outlive the registration.
    Status(ROCK_CALL* registerConverter)(OwnerToken,Convert,void*) noexcept;
};
using Query=const ApiV1*(ROCK_CALL*)(std::uint32_t,std::uint32_t) noexcept;
static_assert(sizeof(Result)==12&&sizeof(ApiV1)==16);
}
