#include "physics-interaction/weapon/WeaponPartDrivePose.h"
#include "api/WeaponPartGrabPoints.h"
#include <cassert>
#include <iostream>

namespace {
using namespace rock;
RE::NiTransform pose(float x,float y,float z){auto t=transform_math::makeIdentityTransform<RE::NiTransform>();t.translate={x,y,z};return t;}
struct Node {Node* parent{};RE::NiTransform local=pose(0,0,0),world=pose(0,0,0);};
void at(const RE::NiTransform& world,float x,float y,float z){assert(std::abs(world.translate.x-x)<.001f&&std::abs(world.translate.y-y)<.001f&&std::abs(world.translate.z-z)<.001f);}
}
int main(){
    Node scene,root{&scene},receiver{&root,pose(0,5,0)},bolt{&receiver,pose(0,10,0)},knob{&bolt,pose(2,0,0)};
    root.world=pose(100,200,300);RE::NiTransform world;
    weapon_drive_pose::Frame<Node> native(&root,root.world);assert(native.world(&knob,world));at(world,102,215,300);
    // Native animation has reset the bolt forward. Replay the active drive
    // without mutating its local or cached world; the acquired knob is rearward.
    weapon_drive_pose::Frame<Node> driven(&root,root.world);
    assert(driven.apply(&bolt,pose(0,-10,0),true,100));assert(driven.world(&knob,world));at(world,102,190,300);
    assert(bolt.local.translate.y==10&&receiver.local.translate.y==5);
    auto point=weapon_part_points::Point{};point.pointId=point.groupId=1;point.sourceKey=4;point.weaponGenerationKey=1;point.radiusGame=2;
    weapon_part_points::Candidate candidate{{7,point,true},{world.translate.x,world.translate.y,world.translate.z},0,true};
    const auto candidates=std::span(&candidate,1);
    assert(weapon_part_points::nearest(candidates,api::Hand::Right,{102,190,300},3,{},true).point==0);
    assert(weapon_part_points::nearest(candidates,api::Hand::Right,{102,215,300},3,{},true).point==1);
    // New hand can reacquire after releasing: the drive persists independently.
    assert(weapon_part_points::nearest(candidates,api::Hand::Left,{102,190,300},3,{},true).point==0);
    assert(!driven.apply(&bolt,pose(0,12,0),true,99));assert(driven.world(&knob,world));at(world,102,190,300);
    assert(driven.apply(&bolt,pose(0,-12,0),true,100));assert(driven.world(&knob,world));at(world,102,188,300);
    // Match the writer's ordering: moving a parent after a root-space child
    // moves that child's already-computed local, then a later child overrides it.
    assert(driven.apply(&receiver,pose(0,8,0),true,100));assert(driven.world(&knob,world));at(world,102,191,300);
    assert(driven.apply(&bolt,pose(0,-12,0),true,100));assert(driven.world(&knob,world));at(world,102,188,300);
    assert(driven.apply(&bolt,pose(0,2,0),false,101));assert(driven.world(&knob,world));at(world,102,210,300);
    auto moved=pose(500,600,700);moved.scale=2;moved.rotate.entry[0][0]=0;moved.rotate.entry[0][1]=1;moved.rotate.entry[1][0]=-1;moved.rotate.entry[1][1]=0;
    weapon_drive_pose::Frame<Node> turned(&root,moved);assert(turned.apply(&bolt,pose(0,-10,0),true,100));assert(turned.world(&knob,world));at(world,520,604,700);
    // An expired/revoked drive is omitted from the next snapshot. No cached
    // driven transform leaks across frames or survives a detached node.
    weapon_drive_pose::Frame<Node> expired(&root,moved);assert(expired.world(&knob,world));at(world,470,604,700);
    bolt.local=pose(0,-15,0);weapon_drive_pose::Frame<Node> restored(&root,moved);
    assert(!restored.contains(&bolt));assert(restored.apply(&bolt,pose(0,10,0),false,0));
    assert(restored.world(&knob,world));at(world,470,604,700);assert(bolt.local.translate.y==-15);
    Node detached;assert(!expired.world(&detached,world));
    std::cout<<"PASS held-open reacquisition, no native writes, ordered parent/root drives, priority, rotation/scale and frame-local cleanup\n";
}
