#pragma once
#include <ROCK/TouchV1_1.h>
#include <array>
#include <cmath>

namespace rock::authored_interaction {
    using Request=api::touch::v1_1::AuthoredHand;
    inline constexpr std::uint32_t indicator=1, reserve=2, attached=4;
    struct Lease {
        api::OwnerToken owner{};
        std::uint64_t expires{};
        Request request{};
        bool current(const api::Sample& now) const noexcept {
            const auto& s=request.sample;
            return owner && now.frameIndex<=expires && s.worldGeneration==now.worldGeneration &&
                s.skeletonGeneration==now.skeletonGeneration && s.providerGeneration==now.providerGeneration &&
                s.collisionGeneration==now.collisionGeneration;
        }
        api::Status set(api::OwnerToken caller,const Request& next,const api::Sample& now) noexcept {
            if(!caller || next.size!=sizeof(Request) || next.version!=1 || next.reserved ||
                (next.hand!=api::Hand::Right && next.hand!=api::Hand::Left) || !next.referenceFormId ||
                !next.flags || (next.flags&~7u) || ((next.flags&attached)&&!(next.flags&reserve)) ||
                next.leaseFrames<1 || next.leaseFrames>3 ||
                !std::isfinite(next.contactWorldGame.x) || !std::isfinite(next.contactWorldGame.y) || !std::isfinite(next.contactWorldGame.z))return api::Status::InvalidArgument;
            Lease candidate{caller,now.frameIndex+next.leaseFrames,next};
            if(!candidate.current(now) || next.sample.frameIndex>now.frameIndex || now.frameIndex-next.sample.frameIndex>1)return api::Status::GenerationMismatch;
            if(current(now)&&owner!=caller)return api::Status::OwnerConflict;
            *this=candidate;return api::Status::Ok;
        }
        api::Status clear(api::OwnerToken caller,const api::Sample& now) noexcept {
            if(current(now)&&owner!=caller)return api::Status::OwnerConflict;
            *this={};return api::Status::Ok;
        }
    };
}
