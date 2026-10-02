#include "AuthoredInteractionService.h"
#include "ProviderRuntimeServices.h"
#include "physics-interaction/native/NativePlayerCollisionFilter.h"
#include "physics-interaction/PhysicsLog.h"

namespace rock::authored_interaction {
namespace {
    std::array<Lease,2> leases;
    Lease* active(bool left) {
        auto& lease=leases[left?1:0];
        if(!lease.current(provider::runtime::sample()) ||
            provider::runtime::authorize(lease.owner,api::InterfaceId::Touch,2,true)!=api::Status::Ok)lease={};
        return lease.owner?&lease:nullptr;
    }
}
api::Status set(api::OwnerToken owner,const Request& request) {
    if(request.hand!=api::Hand::Left&&request.hand!=api::Hand::Right)return api::Status::InvalidArgument;
    if((request.flags&attached)&&!native_player_collision::installed())return api::Status::NotReady;
    auto& lease=leases[request.hand==api::Hand::Left?1:0];
    (void)active(request.hand==api::Hand::Left);
    const auto old=lease;
    const auto result=lease.set(owner,request,provider::runtime::sample());
    if(result==api::Status::Ok&&(old.owner!=owner||old.request.flags!=request.flags))
        ROCK_LOG_INFO(Hand,"Authored interaction hand={} owner={} ref={:08X} flags={} (indicator/reserve/attached)",static_cast<unsigned>(request.hand),owner,request.referenceFormId,request.flags);
    return result;
}
api::Status clear(api::OwnerToken owner,api::Hand hand) {
    if(hand!=api::Hand::Left&&hand!=api::Hand::Right)return api::Status::InvalidArgument;
    return leases[hand==api::Hand::Left?1:0].clear(owner,provider::runtime::sample());
}
bool reserves(bool left) {const auto* lease=active(left);return lease&&(lease->request.flags&reserve);}
bool isAttached(bool left) {const auto* lease=active(left);return lease&&(lease->request.flags&attached);}
std::uint32_t appendIndicators(std::span<RE::NiPoint3> output) {
    std::uint32_t count=0;
    for(bool left:{false,true})if(const auto* lease=active(left);lease&&(lease->request.flags&indicator)&&count<output.size()) {
        const auto& p=lease->request.contactWorldGame;output[count++]={p.x,p.y,p.z};
    }
    return count;
}
void reset() noexcept {leases={};}
}
