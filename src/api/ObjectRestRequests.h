#pragma once
#include <ROCK/ObjectRest.h>
#include <array>

namespace rock::object_rest_service {
    struct Watch {
        api::OwnerToken owner{};
        api::object_rest::RequestV1 request;
        api::object_rest::State state{api::object_rest::State::Waiting};
        api::Status status{api::Status::RequestQueued};
        bool cancelled{};
    };
    class Requests {
    public:
        std::array<Watch,api::object_rest::kMaximumWatches> entries{};
        api::Status watch(api::OwnerToken owner,const api::object_rest::RequestV1& request,api::object_rest::State& state) {
            state=api::object_rest::State::Waiting;
            for(auto& entry:entries)if(entry.owner && entry.request.reference.referenceFormId==request.reference.referenceFormId) {
                if(entry.owner!=owner)return api::Status::OwnerConflict;
                if(entry.request.expectedBase!=request.expectedBase || entry.request.reference.referenceNativeHandle!=request.reference.referenceNativeHandle)return api::Status::InvalidArgument;
                if(entry.request.reference.worldGeneration!=request.reference.worldGeneration || entry.request.reference.skeletonGeneration!=request.reference.skeletonGeneration ||
                   entry.request.reference.providerGeneration!=request.reference.providerGeneration)return api::Status::GenerationMismatch;
                if(entry.cancelled)return api::Status::Busy;
                state=entry.state;const auto status=entry.status;
                if(state==api::object_rest::State::Resting || (status!=api::Status::Ok && status!=api::Status::RequestQueued))entry={};
                return status;
            }
            for(auto& entry:entries)if(!entry.owner){entry={owner,request};return api::Status::RequestQueued;}
            return api::Status::CapacityFull;
        }
        void cancel(api::OwnerToken owner,std::uint32_t reference) {
            for(auto& entry:entries)if(entry.owner==owner && (!reference || entry.request.reference.referenceFormId==reference))entry.cancelled=true;
        }
    };
}
