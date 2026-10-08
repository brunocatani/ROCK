#pragma once
#include <ROCK/EjectionContacts.h>
#include "physics-interaction/native/HavokPairCollisionFilter.h"
namespace rock {
struct EjectionContactRequests {
    struct Entry {api::OwnerToken owner{};api::ejection_contacts::RequestV1 request;float waiting{},age{};bool active{},cancelled{},finished{};};
    std::array<Entry,8> entries{};
    BasicHavokPairCollisionLeaseSet<4096,1> pairs;
    bool hadPairs{};
    api::Status request(api::OwnerToken owner,const api::ejection_contacts::RequestV1& request) {
        for(auto& e:entries)if(e.owner&&e.request.reference.referenceFormId==request.reference.referenceFormId) {
            if(e.owner!=owner&&!e.finished)return api::Status::OwnerConflict;
            if(e.owner!=owner)continue;
            const auto& a=e.request;const auto& b=request;
            const bool same=a.weaponGenerationKey==b.weaponGenerationKey&&a.weaponForm==b.weaponForm&&a.expectedBase==b.expectedBase&&
                a.reference.referenceNativeHandle==b.reference.referenceNativeHandle&&a.reference.worldGeneration==b.reference.worldGeneration&&
                a.reference.skeletonGeneration==b.reference.skeletonGeneration&&a.reference.providerGeneration==b.reference.providerGeneration;
            if(!same){if(e.finished){e={};break;}return api::Status::GenerationMismatch;}
            return e.cancelled?api::Status::TargetUnavailable:e.active||e.finished?api::Status::Ok:api::Status::RequestQueued;
        }
        for(auto& e:entries)if(!e.owner||e.finished){e={owner,request};return api::Status::RequestQueued;}
        return api::Status::CapacityFull;
    }
    void cancel(api::OwnerToken owner,std::uint32_t id) {for(auto& e:entries)if(e.owner==owner&&(!id||e.request.reference.referenceFormId==id)){if(e.finished)e={};else e.cancelled=true;}}
};
}
