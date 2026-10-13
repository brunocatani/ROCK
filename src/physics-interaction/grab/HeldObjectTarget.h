#pragma once
#include <ROCK/GrabV1_3.h>
#include <cmath>

namespace rock::held_object_target {
    // Per-Hand value state, read/written only by ROCK's frame owner. Physics
    // receives the resulting proxy target through the existing drive queue.
    struct Lease {
        api::OwnerToken owner{};
        std::uint64_t grab{},expires{};
        api::Transform target{};
        bool current(std::uint64_t identity,std::uint64_t frame) const noexcept {return owner&&grab==identity&&frame<=expires;}
        api::Status set(api::OwnerToken caller,std::uint64_t identity,std::uint64_t frame,
            const api::grab::v1_3::HeldObjectTarget& request) noexcept {
            if(!caller||!identity||request.leaseFrames<1||request.leaseFrames>2)return api::Status::InvalidArgument;
            if(current(identity,frame)&&owner!=caller)return api::Status::OwnerConflict;
            owner=caller;grab=identity;expires=frame+request.leaseFrames;target=request.rootWorld;return api::Status::Ok;
        }
        api::Status clear(api::OwnerToken caller,std::uint64_t identity,std::uint64_t frame) noexcept {
            if(current(identity,frame)&&owner!=caller)return api::Status::OwnerConflict;
            *this={};return api::Status::Ok;
        }
    };
}
