#pragma once
#include <ROCK/WeaponPartsV1_3.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <span>

namespace rock::weapon_part_points {
    using Point=api::weaponparts::v1_3::GrabPoint;
    inline constexpr std::size_t kCapacity=api::weaponparts::kMaxTargets;
    struct OwnedPoint {api::OwnerToken owner{};Point point;bool operating{};};
    inline bool valid(const Point& p) noexcept {
        return p.size==sizeof(Point)&&p.pointId&&p.groupId&&p.sourceKey&&p.weaponGenerationKey&&
            p.handMask&&!(p.handMask&~3u)&&std::isfinite(p.radiusGame)&&p.radiusGame>0&&p.radiusGame<=64&&
            std::all_of(std::begin(p.reserved),std::end(p.reserved),[](auto n){return n==0;})&&
            std::all_of(std::begin(p.pointInSource),std::end(p.pointInSource),[](auto v){return std::isfinite(v);});
    }
    struct Candidate {
        OwnedPoint rule;
        std::array<float,3> world{};
        std::size_t part{~std::size_t{0}};
        bool validWorld{};
    };
    struct Selection {std::size_t point;float distanceSquared{};};
    inline Selection nearest(std::span<const Candidate> points,api::Hand hand,const std::array<float,3>& probe,float probeRadius,
        std::span<const std::array<float,3>> extraProbes={},bool operatingOnly=false) noexcept {
        Selection result{points.size(),0};const auto mask=hand==api::Hand::Left?1u:hand==api::Hand::Right?2u:0u;
        if(!mask||!std::isfinite(probeRadius)||probeRadius<=0||!std::all_of(probe.begin(),probe.end(),[](auto v){return std::isfinite(v);}))return result;
        for(std::size_t i=0;i<points.size();++i){const auto& c=points[i];const auto& p=c.rule.point;
            if(!c.validWorld||c.part==~std::size_t{0}||!(p.handMask&mask)||c.rule.operating!=operatingOnly)continue;
            float distance=0;for(int k=0;k<3;++k){const auto d=c.world[k]-probe[k];distance+=d*d;}
            if(c.rule.operating)for(const auto& finger:extraProbes){
                float fingerDistance=0;for(int k=0;k<3;++k){const auto d=c.world[k]-finger[k];fingerDistance+=d*d;}
                if(std::isfinite(fingerDistance))distance=(std::min)(distance,fingerDistance);
            }
            const float radius=(std::min)(probeRadius,p.radiusGame);if(!std::isfinite(distance)||distance>radius*radius)continue;
            if(result.point<points.size()&&(distance>result.distanceSquared||
                (distance==result.distanceSquared&&p.pointId>=points[result.point].rule.point.pointId)))continue;
            result={i,distance};
        }
        return result;
    }
    // Protected by the provider's existing weapon-part mutex; snapshots are
    // copied for the owner frame and never contain retained engine pointers.
    class Registry {
    public:
        template<class GroupValid>api::Status replace(api::OwnerToken owner,std::span<const Point> points,GroupValid&& groupValid,bool operating=false) {
            if(!owner||points.empty()||points.size()>api::weaponparts::v1_3::kMaxGrabPoints)return api::Status::InvalidArgument;
            for(std::size_t i=0;i<points.size();++i){
                const auto& p=points[i];if(!valid(p))return api::Status::InvalidArgument;
                if(!groupValid(p))return api::Status::TargetUnavailable;
                for(std::size_t j=0;j<i;++j)if(points[j].groupId==p.groupId&&points[j].pointId==p.pointId)return api::Status::InvalidArgument;
            }
            const auto available=std::count_if(slots_.begin(),slots_.end(),[&](const auto& s){return !s.owner||s.owner==owner;});
            if(points.size()>std::size_t(available))return api::Status::CapacityFull;
            clear(owner);std::size_t i=0;for(auto& slot:slots_)if(!slot.owner){slot={owner,points[i++],operating};if(i==points.size())break;}
            return api::Status::Ok;
        }
        void clear(api::OwnerToken owner) noexcept {for(auto& slot:slots_)if(slot.owner==owner)slot={};}
        bool operatingFor(std::uint64_t generation) const noexcept {return std::any_of(slots_.begin(),slots_.end(),[&](const auto& s){return s.owner&&s.operating&&s.point.weaponGenerationKey==generation;});}
        std::size_t copy(std::span<OwnedPoint> out) const noexcept {std::size_t n=0;for(const auto& s:slots_)if(s.owner){if(n==out.size())break;out[n++]=s;}return n;}
    private:
        std::array<OwnedPoint,kCapacity> slots_{};
    };
}
