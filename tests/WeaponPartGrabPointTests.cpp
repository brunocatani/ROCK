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
    registry.clear(100);assert(registry.copy(snapshot)==0);
    std::cout<<"PASS exact-point distance, midpoint exclusion, nearest location, hand filtering, current source pose and atomic owner lifecycle\n";
}
