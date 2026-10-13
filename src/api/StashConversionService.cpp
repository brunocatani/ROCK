#include "api/StashConversionService.h"
#include "api/ProviderRuntimeServices.h"
#include <array>
namespace rock::stash_conversion {
namespace {
struct Registration {api::OwnerToken owner{};api::storage::StashConverter callback{};void* context{};};
std::array<Registration,api::storage::kMaxConverters> registrations; // Frame-owner only; consumers are resident.
bool converting{};
api::Status authorizeConverter(api::OwnerToken owner) {
    return provider::runtime::authorize(owner,api::InterfaceId::Storage,static_cast<std::uint32_t>(api::storage::Permission::Callbacks),true);
}
}
api::Status ROCK_CALL registerConverter(api::OwnerToken owner,api::storage::StashConverter callback,void* context) noexcept {
    const auto status=authorizeConverter(owner);
    if(status!=api::Status::Ok)return status;
    if(converting)return api::Status::Busy;
    for(auto& entry:registrations)if(entry.owner==owner){entry=callback?Registration{owner,callback,context}:Registration{};return api::Status::Ok;}
    if(!callback)return api::Status::Ok;
    for(auto& entry:registrations)if(!entry.owner||authorizeConverter(entry.owner)!=api::Status::Ok){entry={owner,callback,context};return api::Status::Ok;}
    return api::Status::CapacityFull;
}
api::storage::StashResult convert(std::uint32_t reference,std::uint32_t base) noexcept {
    using Decision=api::storage::StashDecision;
    if(converting)return {Decision::Reject};
    for(auto& entry:registrations)if(entry.owner&&authorizeConverter(entry.owner)==api::Status::Ok) {
        converting=true;const auto result=entry.callback(reference,base,entry.context);converting=false;
        if(result.decision==Decision::Consumed||result.decision==Decision::Reject)return result;
        if(result.decision!=Decision::Pass)return {Decision::Reject};
    }
    return {Decision::Pass};
}
}
