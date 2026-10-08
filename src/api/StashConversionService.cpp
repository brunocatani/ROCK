#include "api/StashConversionService.h"
#include "api/ProviderRuntimeServices.h"
#include <array>
namespace rock::stash_conversion {
namespace {
struct Registration {api::OwnerToken owner{};api::stash_conversion::Convert callback{};void* context{};};
std::array<Registration,8> registrations; // Frame-owner only; consumers are resident.
bool converting{};
api::Status ROCK_CALL registerConverter(api::OwnerToken owner,api::stash_conversion::Convert callback,void* context) noexcept {
    const auto status=provider::runtime::authorize(owner,api::InterfaceId::Grab,3,true);
    if(status!=api::Status::Ok)return status;
    if(converting)return api::Status::Busy;
    for(auto& entry:registrations)if(entry.owner==owner){entry=callback?Registration{owner,callback,context}:Registration{};return api::Status::Ok;}
    if(!callback)return api::Status::Ok;
    for(auto& entry:registrations)if(!entry.owner||provider::runtime::authorize(entry.owner,api::InterfaceId::Grab,3,true)!=api::Status::Ok){entry={owner,callback,context};return api::Status::Ok;}
    return api::Status::CapacityFull;
}
}
api::stash_conversion::Result convert(std::uint32_t reference,std::uint32_t base) noexcept {
    using namespace api::stash_conversion;
    if(converting)return {Decision::Reject};
    for(auto& entry:registrations)if(entry.owner&&provider::runtime::authorize(entry.owner,api::InterfaceId::Grab,3,true)==api::Status::Ok) {
        converting=true;const auto result=entry.callback(reference,base,entry.context);converting=false;
        if(result.decision==Decision::Consumed||result.decision==Decision::Reject)return result;
        if(result.decision!=Decision::Pass)return {Decision::Reject};
    }
    return {Decision::Pass};
}
}
extern "C" __declspec(dllexport) const rock::api::stash_conversion::ApiV1* ROCK_CALL ROCK_StashConversion_QueryV1(std::uint32_t major,std::uint32_t bytes) noexcept {
    static const rock::api::stash_conversion::ApiV1 api{sizeof(api),1,&rock::stash_conversion::registerConverter};
    return major==1&&bytes==sizeof(api)?&api:nullptr;
}
