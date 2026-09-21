#include "physics-interaction/weapon/WeaponInteractionQuery.h"

#include <cassert>
#include <chrono>
#include <cstdio>
#include <random>

namespace
{
    using namespace rock;
    namespace query = weapon_interaction_query;

    struct Source
    {
        std::vector<TriangleData> triangles;
        WeaponTriangleIndex index;
        RE::NiTransform world = transform_math::makeIdentityTransform<RE::NiTransform>();
        RE::NiPoint3 min{-1,-1,0}, max{1,1,0};
        std::uint8_t priority = 0;
    };

    // The previous per-query calculation, with exhaustive surface distances
    // independent of the new prepared-part/index traversal.
    query::Selection reference(const std::vector<Source>& sources, const RE::NiPoint3& point, float radius)
    {
        query::Selection result;
        if (!query::finitePoint(point) || !std::isfinite(radius) || radius <= 0) return result;
        for (std::size_t i=0;i<sources.size();++i) {
            const auto& source=sources[i];
            if (source.triangles.empty() || !weapon_hierarchy::weaponTransformFinite(source.world) ||
                std::abs(source.world.scale)<=0.000001f || !query::finitePoint(source.min) || !query::finitePoint(source.max) ||
                source.min.x>source.max.x || source.min.y>source.max.y || source.min.z>source.max.z) continue;
            const float scale=std::abs(source.world.scale), localRadius=radius/scale;
            const auto local=weapon_collision_geometry_math::worldPointToLocal(source.world.rotate,
                source.world.translate,source.world.scale,point);
            if (!query::finitePoint(local)) continue;
            const float bounds=weapon_interaction_probe_math::pointAabbDistanceSquared(local,source.min,source.max);
            if (!std::isfinite(bounds) || !weapon_interaction_probe_math::isWithinProbeRadiusSquared(bounds,localRadius)) continue;
            ++result.boundsCandidates;
            float distance=(std::numeric_limits<float>::infinity)();
            for (const auto& triangle:source.triangles) {
                float d=(std::numeric_limits<float>::infinity)();
                (void)closestPointOnTriangleToPoint(local,triangle,d);
                if (std::isfinite(d) && d<=localRadius*localRadius) distance=(std::min)(distance,d);
            }
            if (!std::isfinite(distance) || !weapon_interaction_probe_math::isWithinProbeRadiusSquared(distance,localRadius)) continue;
            ++result.surfaceCandidates;
            const weapon_interaction_probe_math::ProbeCandidateRank rank{
                distance*(scale*scale),weapon_interaction_probe_math::aabbDiagonalSquared(source.min,source.max)*(scale*scale),source.priority};
            if (result.valid() && !weapon_interaction_probe_math::isBetterProbeCandidate(rank,result.rank)) continue;
            result.part=i;
            result.rank=rank;
        }
        return result;
    }

    struct Batch
    {
        std::array<query::Part,100> parts{};
        std::array<std::size_t,100> source{};
        std::size_t count=0;
        explicit Batch(const std::vector<Source>& sources)
        {
            assert(sources.size()<=parts.size());
            for (std::size_t i=0;i<sources.size();++i) {
                const auto& s=sources[i];
                if (query::prepare(s.triangles,s.index,s.world,s.min,s.max,s.priority,parts[count])) source[count++]=i;
            }
        }
        auto find(const RE::NiPoint3& point,float radius) const
        {
            auto result=query::find({parts.data(),count},point,radius);
            if (result.valid()) result.part=source[result.part];
            return result;
        }
    };

    void same(const query::Selection& a,const query::Selection& b)
    {
        assert(a.part==b.part && a.boundsCandidates==b.boundsCandidates && a.surfaceCandidates==b.surfaceCandidates);
        if (!a.valid()) return;
        assert(a.rank.distanceSquaredGame==b.rank.distanceSquaredGame);
        assert(a.rank.aabbDiagonalSquaredGame==b.rank.aabbDiagonalSquaredGame);
        assert(a.rank.semanticPriority==b.rank.semanticPriority);
    }

    std::vector<Source> makeSources(std::size_t count)
    {
        std::vector<Source> sources(count);
        for (std::size_t i=0;i<count;++i) {
            auto& source=sources[i];
            source.triangles={{{-1,-1,0},{1,-1,0},{-1,1,0}},{{1,-1,0},{1,1,0},{-1,1,0}}};
            source.index.build(source.triangles);
            source.world.translate={120000.0f,static_cast<float>(i)*1.5f,-76000.0f};
            source.world.scale=i%3==0?-1.5f:0.75f;
            source.priority=static_cast<std::uint8_t>(i%7);
        }
        return sources;
    }

    void parity()
    {
        auto sources=makeSources(68);
        sources[4].world.scale=0;
        sources[7].world.rotate.entry[1][2]=std::numeric_limits<float>::quiet_NaN();
        sources[9].min.x=2;
        sources[12].triangles.clear();
        sources[16].max.y=std::numeric_limits<float>::infinity();
        std::mt19937 random(71249);
        std::uniform_real_distribution<float> x(-5,5),y(-5,110),z(-4,4);
        for (int frame=0;frame<3;++frame) {
            const Batch batch(sources);
            for (int i=0;i<500;++i) {
                const RE::NiPoint3 point{120000+x(random),y(random),-76000+z(random)};
                const float radius=i%13==0?0.0f:static_cast<float>(i%6)+0.01f;
                same(reference(sources,point,radius),batch.find(point,radius));
            }
            same(reference(sources,{NAN,0,0},1),batch.find({NAN,0,0},1));
            same(reference(sources,{120000,0,-76000},NAN),batch.find({120000,0,-76000},NAN));
            // Later poses require a fresh batch; no previous-frame transform is reused.
            for (auto& source:sources) source.world.translate.y+=0.625f;
        }
        auto tied=makeSources(2);
        tied[1].world=tied[0].world;
        const auto p=tied[0].world.translate+RE::NiPoint3{0,0,1.5f};
        same(reference(tied,p,1.5f),Batch(tied).find(p,1.5f));
        assert(Batch(tied).find(p,1.5f).part==1); // Semantic priority breaks a geometric tie.
        tied[1].priority=tied[0].priority;
        assert(Batch(tied).find(p,1.5f).part==0); // Exact ties retain body-bank order.
        same(reference(tied,p,std::nextafter(1.5f,0.0f)),Batch(tied).find(p,std::nextafter(1.5f,0.0f)));
        tied[0].world.scale=0;
        same(reference(tied,p,2),Batch(tied).find(p,2));
    }

    void measure()
    {
        const auto sources=makeSources(68);
        const std::array<RE::NiPoint3,3> points{{{120000,2,-75999},{120000,30,-75999},{120000,80,-75999}}};
        std::size_t separateSum=0,sharedSum=0;
        const auto begin=std::chrono::steady_clock::now();
        for (int frame=0;frame<500;++frame) for (const auto& point:points) separateSum+=Batch(sources).find(point,3).part;
        const auto separate=std::chrono::steady_clock::now();
        for (int frame=0;frame<500;++frame) {
            const Batch batch(sources);
            for (const auto& point:points) sharedSum+=batch.find(point,3).part;
        }
        const auto shared=std::chrono::steady_clock::now();
        assert(separateSum==sharedSum);
        std::printf("68-part fixture, 500 three-query groups: repeated preparation %.3f ms / shared preparation %.3f ms; prepared parts per group 204 -> 68\n",
            std::chrono::duration<double,std::milli>(separate-begin).count(),
            std::chrono::duration<double,std::milli>(shared-separate).count());
    }
}

int main()
{
    parity();
    measure();
}
