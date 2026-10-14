#include "api/WeaponPartGrabPoints.h"
#include <cassert>
#include <limits>
#include <iostream>
using namespace rock;
int main(){
    namespace p=weapon_part_points;using api::Status;
    p::Point a;a.pointId=1;a.groupId=7;a.weaponGenerationKey=10;a.sourceKey=11;a.pointInSource.x=20;a.radiusGame=2;
    p::Registry registry;const auto group=[](const auto& p){return p.groupId==7&&p.weaponGenerationKey==10;};
    assert(registry.replace(100,std::span(&a,1),group)==Status::Ok&&registry.any(10)&&!registry.any(11));
    std::array<p::OwnedPoint,p::kCapacity> snapshot{};assert(registry.copy(snapshot)==1&&snapshot[0].owner==100);
    auto bad=a;bad.radiusGame=std::numeric_limits<float>::quiet_NaN();assert(registry.replace(100,std::span(&bad,1),group)==Status::InvalidArgument);
    bad=a;bad.version=2;assert(registry.replace(100,std::span(&bad,1),group)==Status::InvalidArgument);
    bad=a;bad.groupId=8;assert(registry.replace(100,std::span(&bad,1),group)==Status::TargetUnavailable);
    assert(registry.copy(snapshot)==1&&snapshot[0].point.pointId==1); // rejection preserves the previous set
    std::array duplicate{a,a};assert(registry.replace(100,duplicate,group)==Status::InvalidArgument);
    auto b=a;b.pointId=2;b.pointInSource.x=-20;b.handMask=1;
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
    // Finger and pinky probes reach a point the palm cannot.
    const std::array<std::array<float,3>,2> fingers{{{19,0,0},{-19.5f,0,0}}};
    assert(p::nearest(c,api::Hand::Left,{0,0,0},12,fingers).point==1);
    assert(p::nearest(c,api::Hand::Right,{0,0,0},12,fingers).point==0);
    assert(p::nearest(c,api::Hand::Left,{0,0,0},12,{},fingers).point==1);
    assert(p::nearest(c,api::Hand::Right,{0,0,0},12,{},fingers).point==0);
    assert(p::nearest(c,api::Hand::Left,{20,0,0},12).point==0); // palm remains available
    const std::array<std::array<float,3>,2> distant{{{17,0,0},{-17,0,0}}};
    assert(p::nearest(c,api::Hand::Left,{0,0,0},12,distant,distant).point==c.size()); // no radius inflation
    const std::array<std::array<float,3>,1> invalid{{{std::numeric_limits<float>::quiet_NaN(),0,0}}};
    assert(p::nearest(c,api::Hand::Left,{0,0,0},12,invalid,invalid).point==c.size());
    // A point seats on the nearest surface of its group's parts, not the first listed.
    p::Candidate anchored;
    p::offerPart(anchored,0,16.f,90);p::offerPart(anchored,1,.01f,91);
    assert(anchored.part==1);
    p::Candidate reordered;
    p::offerPart(reordered,1,.01f,91);p::offerPart(reordered,0,16.f,90);
    assert(reordered.part==1);
    p::offerPart(anchored,3,std::numeric_limits<float>::quiet_NaN(),1);assert(anchored.part==1);
    p::offerPart(anchored,4,.01f,89);assert(anchored.part==4); // deterministic tie
    c[0].part=~std::size_t{0};assert(p::nearest(c,api::Hand::Right,{20,0,0},12).point==c.size()); // no surface, no acquisition
    assert(registry.replace(101,std::span(&a,1),group)==Status::Ok&&registry.copy(snapshot)==3);
    registry.clear(101);
    registry.clear(100);assert(registry.copy(snapshot)==0&&!registry.any(10));
    std::cout<<"PASS exact-point distance, finger probes, nearest surface, hand filtering and atomic owner lifecycle\n";
}
