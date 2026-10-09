#include "api/WeaponPartGrabPoints.h"
#include <cassert>
#include <limits>
#include <iostream>
using namespace rock;
int main(){
    namespace p=weapon_part_points;using api::Status;
    p::Point a;a.pointId=1;a.groupId=7;a.weaponGenerationKey=10;a.sourceKey=11;a.pointInSource[0]=20;a.radiusGame=2;
    p::Registry registry;const auto group=[](const auto& p){return p.groupId==7&&p.weaponGenerationKey==10;};
    assert(registry.replace(100,std::span(&a,1),group)==Status::Ok);
    std::array<p::OwnedPoint,p::kCapacity> snapshot{};assert(registry.copy(snapshot)==1&&snapshot[0].owner==100);
    auto bad=a;bad.radiusGame=std::numeric_limits<float>::quiet_NaN();assert(registry.replace(100,std::span(&bad,1),group)==Status::InvalidArgument);
    bad=a;bad.groupId=8;assert(registry.replace(100,std::span(&bad,1),group)==Status::TargetUnavailable);
    assert(registry.copy(snapshot)==1&&snapshot[0].point.pointId==1); // rejection preserves the previous set
    std::array duplicate{a,a};assert(registry.replace(100,duplicate,group)==Status::InvalidArgument);
    auto b=a;b.pointId=2;b.pointInSource[0]=-20;b.handMask=1;
    std::array points{a,b};assert(registry.replace(100,points,group)==Status::Ok);
    std::array<p::Candidate,2> c{{{{100,a},{20,0,0},0,true},{{100,b},{-20,0,0},0,true}}};
    assert(p::nearest(c,api::Hand::Left,{0,0,0},12).point==c.size()); // long-part midpoint is not a grab location
    assert(p::nearest(c,api::Hand::Left,{19,0,0},12).point==0);
    assert(p::nearest(c,api::Hand::Left,{-19,0,0},12).point==1);
    assert(p::nearest(c,api::Hand::Right,{-19,0,0},12).point==c.size());
    c[0].world={2,0,0};c[1].world={-1,0,0};assert(p::nearest(c,api::Hand::Left,{0,0,0},12).point==1);
    c[0].world={0,40,0};c[1].world={0,-40,0}; // source rotated 90 degrees and scaled 2
    assert(p::nearest(c,api::Hand::Left,{0,39,0},12).point==0);
    c[0].world={10,40,0};assert(p::nearest(c,api::Hand::Left,{0,39,0},12).point==c.size());
    assert(p::nearest(c,api::Hand::Left,{10,39,0},12).point==0);
    c[0].validWorld=false;assert(p::nearest(c,api::Hand::Left,{10,39,0},12).point==c.size());
    c[0].validWorld=true;c[0].world={20,0,0};c[1].world={-20,0,0};
    const std::array<std::array<float,3>,2> fingers{{{19,0,0},{-19.5f,0,0}}};
    // 1.3 keeps palm-only acquisition even when finger probes are supplied.
    assert(p::nearest(c,api::Hand::Left,{0,0,0},12,fingers).point==c.size());
    for(auto& candidate:c)candidate.rule.operating=true;
    assert(p::nearest(c,api::Hand::Left,{0,0,0},12,fingers).point==1);
    assert(p::nearest(c,api::Hand::Right,{0,0,0},12,fingers).point==0);
    assert(p::nearest(c,api::Hand::Left,{20,0,0},12,{}).point==0); // palm remains available
    assert(p::nearest(c,api::Hand::Left,{0,0,0},12,{}).point==c.size()); // missing fingers do not invent a point
    const std::array<std::array<float,3>,2> distant{{{17,0,0},{-17,0,0}}};
    assert(p::nearest(c,api::Hand::Left,{0,0,0},12,distant).point==c.size()); // no radius inflation
    assert(registry.replace(100,points,group,true)==Status::Ok&&registry.operatingFor(10)&&!registry.operatingFor(11));
    assert(registry.replace(100,points,group)==Status::Ok&&!registry.operatingFor(10));
    // A 1.5 owner can acquire with either pinky anchor while 1.4 stays unchanged.
    assert(p::nearest(c,api::Hand::Left,{0,0,0},12,{},fingers).point==c.size());
    c[0].rule.pinky=true;c[1].rule.pinky=true;
    assert(p::nearest(c,api::Hand::Left,{0,0,0},12,{},fingers).point==1);
    assert(p::nearest(c,api::Hand::Right,{0,0,0},12,{},fingers).point==0);
    c[1].rule.pinky=false; // coexistence: the closer legacy point cannot use pinky.
    assert(p::nearest(c,api::Hand::Left,{0,0,0},12,{},fingers).point==0);
    assert(p::nearest(c,api::Hand::Left,{0,0,0},12,{},distant).point==c.size());
    assert(p::nearest(c,api::Hand::Left,{20,0,0},12,{}).point==0);
    assert(p::nearest(c,api::Hand::Left,{0,0,0},12,fingers).point==1);
    const std::array<std::array<float,3>,1> invalid{{{std::numeric_limits<float>::quiet_NaN(),0,0}}};
    assert(p::nearest(c,api::Hand::Left,{0,0,0},12,{},invalid).point==c.size());
    assert(registry.replace(100,points,group,true,true)==Status::Ok);
    assert(registry.copy(snapshot)==2&&snapshot[0].pinky&&snapshot[1].pinky);
    assert(registry.replace(101,std::span(&a,1),group,true)==Status::Ok);
    assert(registry.copy(snapshot)==3&&!snapshot[2].pinky); // old owner keeps its probe contract
    assert(registry.replace(100,points,group,true)==Status::Ok);
    assert(registry.copy(snapshot)==3&&!snapshot[0].pinky&&!snapshot[1].pinky);
    // A first-listed bolt body must not replace the authored knob surface.
    p::Candidate anchored;anchored.rule.anchored=true;
    p::offerPart(anchored,0,16.f,90);p::offerPart(anchored,1,.01f,91);
    assert(anchored.part==1);
    p::Candidate reordered;reordered.rule.anchored=true;
    p::offerPart(reordered,1,.01f,91);p::offerPart(reordered,0,16.f,90);
    assert(reordered.part==1);
    p::offerPart(anchored,3,std::numeric_limits<float>::quiet_NaN(),1);assert(anchored.part==1);
    p::offerPart(anchored,4,.01f,89);assert(anchored.part==4); // deterministic tie
    p::Candidate legacy;p::offerPart(legacy,0,16.f,90);p::offerPart(legacy,1,.01f,91);assert(legacy.part==0);
    assert(registry.replace(100,points,group,true,true,true)==Status::Ok);
    assert(registry.copy(snapshot)==3&&snapshot[0].anchored&&snapshot[1].anchored&&!snapshot[2].anchored);
    assert(registry.replace(100,points,group,true,true)==Status::Ok);
    assert(registry.copy(snapshot)==3&&!snapshot[0].anchored&&!snapshot[1].anchored);
    registry.clear(101);
    registry.clear(100);assert(registry.copy(snapshot)==0);
    std::cout<<"PASS exact-point distance, midpoint exclusion, nearest location, hand filtering, current source pose and atomic owner lifecycle\n";
}
