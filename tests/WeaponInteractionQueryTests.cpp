#include "physics-interaction/weapon/WeaponInteractionQuery.h"
#include "physics-interaction/weapon/WeaponInteraction.h"
#include "physics-interaction/weapon/WeaponGripSelection.h"
#include "physics-interaction/weapon/FiringGripReattachZonePolicy.h"

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
            RE::NiPoint3 closestLocal{};
            for (const auto& triangle:source.triangles) {
                float d=(std::numeric_limits<float>::infinity)();
                const auto closest=closestPointOnTriangleToPoint(local,triangle,d);
                if (std::isfinite(d) && d<=localRadius*localRadius && d<distance) { distance=d; closestLocal=closest; }
            }
            if (!std::isfinite(distance) || !weapon_interaction_probe_math::isWithinProbeRadiusSquared(distance,localRadius)) continue;
            ++result.surfaceCandidates;
            const weapon_interaction_probe_math::ProbeCandidateRank rank{
                distance*(scale*scale),weapon_interaction_probe_math::aabbDiagonalSquared(source.min,source.max)*(scale*scale),source.priority};
            if (result.valid() && !weapon_interaction_probe_math::isBetterProbeCandidate(rank,result.rank)) continue;
            result.part=i;
            result.rank=rank;
            result.closestWorld=weapon_collision_geometry_math::localPointToWorld(source.world.rotate,source.world.translate,source.world.scale,closestLocal);
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
        assert((a.closestWorld-b.closestWorld).Length()<0.0001f);
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
    // A spherical seat admits the same physical displacement after the weapon
    // rotates/translates. Rear and side entry share one distance metric.
    namespace volume = rock::firing_grip_reattach_zone_policy;
    for (int i = 0; i < 80; ++i) {
        const float angle = i * 0.17f;
        const float distance = i * 0.125f;
        const volume::Vec3 seat{31, -18, 72};
        const auto result = volume::evaluateZone({seat,
            {seat.x + distance * std::cos(angle), seat.y + distance * std::sin(angle), seat.z}, 5.0f});
        assert(result.valid && std::abs(result.radialDistanceGameUnits - distance) < 0.00001f);
        if (std::abs(distance - 5.0f) > 0.001f) assert(result.inside == (distance < 5.0f));
    }
    // Hover state cannot resurrect a withdrawn bolt, survive a weapon identity
    // change, or prefer yesterday's distance to the current sampled positions.
    namespace selection = rock::weapon_grip_selection;
    const RE::NiPoint3 posePalm{1,0,0}, poseSeat{0,0,0}, poseNormal{0,0,1}, posePointing{0,1,0};
    const auto posePreference=selection::supportPosePreference(posePalm,poseSeat,poseNormal,poseNormal,posePointing,posePointing);
    assert(posePreference.valid && posePreference.credit>0);
    for (int i=0;i<30;++i) {
        const float angle=i*0.21f;
        const auto rotate=[&](const RE::NiPoint3& point) {
            return RE::NiPoint3{point.x*std::cos(angle)-point.y*std::sin(angle),point.x*std::sin(angle)+point.y*std::cos(angle),point.z};
        };
        const RE::NiPoint3 translation{130,-95,78};
        const auto rotated=selection::supportPosePreference(rotate(posePalm)+translation,rotate(poseSeat)+translation,
            rotate(poseNormal),rotate(poseNormal),rotate(posePointing),rotate(posePointing));
        assert(rotated.valid && std::abs(rotated.credit-posePreference.credit)<0.0001f);
    }
    const auto mirrored=selection::supportPosePreference({-1,0,0},poseSeat,{-1,0,0},{-1,0,0},posePointing,posePointing);
    assert(mirrored.valid && mirrored.credit==posePreference.credit);
    const auto acrossSlide=selection::supportPosePreference(posePalm,poseSeat,poseNormal,poseNormal,{1,0,0},posePointing);
    assert(acrossSlide.valid && acrossSlide.credit==0);
    const auto outsidePose=selection::supportPosePreference({selection::kSupportPoseOuterRadius+1,0,0},poseSeat,
        poseNormal,poseNormal,posePointing,posePointing);
    assert(outsidePose.valid && outsidePose.credit==0);
    assert(!selection::supportPosePreference(posePalm,poseSeat,{NAN,0,0},poseNormal,posePointing,posePointing).valid);
    // Hover must follow the current pose evidence; a previous support credit
    // cannot keep the hand on support after the wrist turns toward the slide.
    std::array<selection::Candidate,2> overlap{{
        {selection::Kind::Part,{},0.2f,2.0f,7,9,3,true},
        {selection::Kind::Support,{},2.0f,10.0f,0,0,0,true}}};
    selection::State overlapState{};
    overlap[1].poseCredit=posePreference.credit;
    assert(selection::select(overlapState,overlap,10,20,false).kind==selection::Kind::Support);
    overlap[1].poseCredit=acrossSlide.credit;
    assert(selection::select(overlapState,overlap,10,20,false).kind==selection::Kind::Part);
    std::array<selection::Candidate, 2> candidates{{
        {selection::Kind::Part, {}, 1.0f, 2.0f, 7, 9, 3, true},
        {selection::Kind::Firing, {}, 1.1f, 10.0f, 0, 0, 0, true}}};
    selection::State state{};
    assert(selection::select(state, candidates, 10, 20, false).kind == selection::Kind::Part);
    candidates[0].distance = 1.2f;
    assert(selection::select(state, candidates, 10, 20, false).kind == selection::Kind::Part);
    ++candidates[0].group;
    assert(selection::select(state, candidates, 10, 20, false).kind == selection::Kind::Firing);
    candidates[0].eligible = false;
    assert(selection::select(state, candidates, 10, 20, false).kind == selection::Kind::Firing);
    candidates[0].eligible = true;
    candidates[0].distance = 1.0f;
    assert(selection::select(state, candidates, 11, 20, false).kind == selection::Kind::Part);
    assert(selection::select(state, {}, 11, 20, false).kind == selection::Kind::None);
    assert(selection::select(state, candidates, 0, 20, false).kind == selection::Kind::None);
    // Rejecting the nearer receiver leaves the next eligible surface visible.
    const auto surfaceSources = makeSources(2);
    const Batch surfaceBatch(surfaceSources);
    const std::array<bool,2> excluded{{true,false}};
    const auto masked = query::find(std::span(surfaceBatch.parts).first(surfaceBatch.count),
        surfaceSources[0].world.translate, 3.0f, excluded);
    assert(masked.valid() && masked.part == 1);
    rock::WeaponInteractionContact contact;contact.valid=true;contact.weaponGenerationKey=123;
    contact.grabPoint.ownerToken=45;contact.grabPoint.groupId=1;contact.grabPoint.sourceKey=237;
    contact.grabPoint.pointId=7;contact.grabPoint.anchorAtPoint=true;contact.grabPoint.operating=true;
    contact.grabPoint.worldPoint={3,5,8};contact.grabPoint.attachmentWorld.scale=2;
    const auto routed=rock::routeWeaponInteraction(contact,{});
    assert(routed.grabPoint.pointId==7&&routed.grabPoint.ownerToken==45&&routed.grabPoint.sourceKey==237);
    assert(routed.grabPoint.anchorAtPoint&&routed.grabPoint.worldPoint.z==8&&routed.grabPoint.attachmentWorld.scale==2);
    assert(routed.grabPoint.operating);
    contact.valid=false;assert(!rock::routeWeaponInteraction(contact,{}).grabPoint.pointId);
    // An admitted point is also the palm seat; no projection to the rod body.
    auto hand=rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    hand.translate={10,20,30};const RE::NiPoint3 palm{11,22,33},knob{3,5,8};
    const auto seated=rock::weapon_two_handed_grip_math::alignHandFrameToGripPoint(hand,palm,knob);
    assert(seated.translate.x==2&&seated.translate.y==3&&seated.translate.z==5);
    for(int r=0;r<3;++r)for(int c=0;c<3;++c)assert(seated.rotate.entry[r][c]==hand.rotate.entry[r][c]);


    parity();
    const auto sources=makeSources(1);const Batch batch(sources);
    const auto probe= sources[0].world.translate;
    assert(query::find(std::span(batch.parts).first(batch.count),probe,2).valid());
    const std::array<bool,1> restricted{true};
    assert(!query::find(std::span(batch.parts).first(batch.count),probe,2,restricted).valid());
    measure();
}
