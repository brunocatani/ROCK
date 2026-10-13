#pragma once
#include <ROCK/Collision.h>
#include <array>
namespace rock {
struct EjectionContactRequests {
    struct Entry {api::OwnerToken owner{};api::collision::EjectionClearanceRequest request;float waiting{},age{};bool active{},cancelled{},finished{},hands{};};
    std::array<Entry,api::collision::kMaxEjectionClearances> entries{};
    bool hadPairs{},handContactReported{};
    api::Status request(api::OwnerToken owner,const api::collision::EjectionClearanceRequest& request) {
        const bool hands=(request.flags&static_cast<std::uint32_t>(api::collision::EjectionClearanceFlag::IncludeHands))!=0;
        for(auto& e:entries)if(e.owner&&e.request.reference.referenceFormId==request.reference.referenceFormId) {
            if(e.owner!=owner&&!e.finished)return api::Status::OwnerConflict;
            if(e.owner!=owner)continue;
            const auto& a=e.request;const auto& b=request;
            const bool same=e.hands==hands&&a.weaponGenerationKey==b.weaponGenerationKey&&a.weaponFormId==b.weaponFormId&&a.expectedBase==b.expectedBase&&
                a.reference.referenceNativeHandle==b.reference.referenceNativeHandle&&a.reference.worldGeneration==b.reference.worldGeneration&&
                a.reference.skeletonGeneration==b.reference.skeletonGeneration&&a.reference.providerGeneration==b.reference.providerGeneration;
            if(!same){if(e.finished){e={};break;}return api::Status::GenerationMismatch;}
            return e.cancelled?api::Status::TargetUnavailable:e.active||e.finished?api::Status::Ok:api::Status::RequestQueued;
        }
        for(auto& e:entries)if(!e.owner||e.finished){e={owner,request};e.hands=hands;return api::Status::RequestQueued;}
        return api::Status::CapacityFull;
    }
    void cancel(api::OwnerToken owner,std::uint32_t id) {for(auto& e:entries)if(e.owner==owner&&(!id||e.request.reference.referenceFormId==id)){if(e.finished)e={};else e.cancelled=true;}}
};
}
