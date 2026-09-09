#include "physics-interaction/grab/SkinnedSurfaceMath.h"
#include "physics-interaction/grab/SegmentVisibilityPolicy.h"
#include "physics-interaction/object/RagdollComponentPolicy.h"
#include "physics-interaction/collision/PushContact.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <thread>

namespace
{
    struct Point { float x, y, z; };
    bool check(bool value, const char* message)
    {
        if (!value) std::printf("FAIL: %s\n", message);
        return value;
    }
    bool close(Point a, Point b)
    {
        return std::abs(a.x-b.x) < 0.0001f && std::abs(a.y-b.y) < 0.0001f && std::abs(a.z-b.z) < 0.0001f;
    }
}

int main()
{
    using namespace rock;
    bool ok = true;
    const skinned_surface_math::Transform identity{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    auto bone = identity;
    bone[0]=0; bone[1]=1; bone[4]=-1; bone[5]=0;
    bone[12]=10; bone[13]=20; bone[14]=30;
    auto bind = identity;
    bind[12]=-2;
    skinned_surface_math::Affine palette{};
    ok &= check(skinned_surface_math::worldFromSkin(bone,bind,{},palette), "posed palette valid");
    // A bind-space vertex two units beyond the bone origin follows the bent limb.
    ok &= check(close(skinned_surface_math::transform(palette,Point{4,0,0}), {10,22,30}), "bent limb follows native basis and bind translation");
    ok &= check(skinned_surface_math::worldFromSkin(bone,identity,{2,3,4},palette), "nonuniform bone scale valid");
    ok &= check(close(skinned_surface_math::transform(palette,Point{1,2,3}), {4,22,42}), "bone scales are applied before world rotation");
    ok &= check(skinned_surface_math::worldFromSkin(bone,identity,{},palette), "absent extra scale uses native uniform scale");
    ok &= check(close(skinned_surface_math::transform(palette,Point{1,0,0}), {10,21,30}), "dynamic pre-skin position requires bone rotation");
    auto farBone=identity;
    farBone[12]=100000;
    skinned_surface_math::Affine farPalette{};
    skinned_surface_math::worldFromSkin(farBone,identity,{},farPalette);
    Point blended{};
    ok &= check(skinned_surface_math::blendVertex<Point>({&farPalette,&farPalette,nullptr,nullptr},
        {0.50025f,0.50025f,0,-0.0005f},{0,0,0},{100000,0,0},blended) && close(blended,{100000,0,0}),
        "quantized skin weights do not displace a body far from world origin");
    ok &= check(skinned_surface_math::blendVertex<Point>({&farPalette,&farPalette,nullptr,nullptr},
        {0.999999f,0.000001f,0,0},{0,0,0},{100000,0,0},blended),
        "small positive skin influences remain part of the native blend");
    ok &= check(!skinned_surface_math::blendVertex<Point>({&farPalette,nullptr,nullptr,nullptr},
        {0.75f,0.25f,0,0},{0,0,0},{100000,0,0},blended),
        "missing weighted transform cannot substitute an unposed surface");
    bone[0] = std::numeric_limits<float>::quiet_NaN();
    ok &= check(!skinned_surface_math::worldFromSkin(bone,identity,{},palette), "invalid skin transform fails closed");

    std::array<segment_visibility::Segment, 3> segments{};
    segments[0] = {0,10,0xFFFFFFFF,2,0,{}};
    segments[1] = {0,4,0,0,0,{}};
    segments[2] = {12,6,0,0,1,{}};
    const std::array<std::uint32_t,1> roots{0};
    auto visible = segment_visibility::resolve(segments,roots,10);
    ok &= check(visible.valid && visible.contains(9), "whole visible segment overrides child split flags");
    segments[0].useChildren=1;
    visible=segment_visibility::resolve(segments,roots,10);
    ok &= check(visible.valid && visible.contains(3) && !visible.contains(4), "dismembered segment exposes only surviving child triangles");
    segments[1].useChildren=1;
    visible=segment_visibility::resolve(segments,roots,10);
    ok &= check(visible.valid && visible.count==0, "fully hidden body mesh has no selectable triangles");
    segments[0].children=3;
    ok &= check(!segment_visibility::resolve(segments,roots,10).valid, "invalid segment descendants fail closed");
    segments[0].children=0; segments[0].useChildren=0; segments[0].firstIndex=1;
    ok &= check(!segment_visibility::resolve(segments,roots,10).valid, "segment index offsets must identify whole triangles");

    using ragdoll::BodyNode;
    using ragdoll::Joint;
    std::array bodies{BodyNode{10,1,true}, BodyNode{20,2,true}, BodyNode{30,3,true},
        BodyNode{40,4,true}, BodyNode{50,4,true}, BodyNode{60,0,false}, BodyNode{70,0,false}};
    std::array joints{Joint{10,20,true}, Joint{20,30,true}, Joint{30,40,false}};
    auto component = ragdoll::connectedComponent(bodies,joints,20);
    ok &= check(component.valid && component.included[0] && component.included[1] && component.included[2], "joint-connected torso and limbs included");
    ok &= check(!component.included[3] && !component.included[4], "severed limb excluded despite shared system");
    component = ragdoll::connectedComponent(bodies,joints,40);
    ok &= check(component.included[3] && component.included[4] && !component.included[2], "detached shared-motion piece is independent");
    joints[1].enabled = false;
    component = ragdoll::connectedComponent(bodies,joints,30);
    ok &= check(component.included[2] && !component.included[0] && !component.included[1], "disabled joint immediately removes old component");
    joints[0] = Joint{30,60,true};
    component = ragdoll::connectedComponent(bodies,joints,30);
    ok &= check(component.fixedAttached && component.included[5] && !component.included[6], "fixed attachment does not connect all fixed-world motions");
    ok &= check(!ragdoll::connectedComponent(bodies,joints,99).valid, "missing primary rejected");
    ok &= check(!ragdoll::connectedComponent(bodies,joints,60).valid, "fixed primary rejected");
    joints[0] = Joint{ragdoll::kInvalidId,30,true};
    component = ragdoll::connectedComponent(bodies,joints,30);
    ok &= check(component.valid && !component.fixedAttached && !component.included[5], "deleted slot is not a connection");

    push_assist::ContactChannel channel;
    push_assist::Contact contact{}, sample{};
    ok &= check(!channel.consume(sample), "no phantom initial push");
    contact = {1,20,100,200,{3,4,5},true};
    ok &= check(channel.publish(contact) && channel.consume(sample), "contact handed off");
    ok &= check(sample.source==1 && sample.target==20 && sample.owner==200 && sample.point[1]==4 && sample.hasPoint,
        "push retains exact body identity and point");
    ok &= check(!channel.consume(sample), "contact consumed only once");
    channel.publish(contact); channel.clear();
    ok &= check(!channel.consume(sample), "world reset discards queued contact");
    std::atomic<bool> finished{false};
    std::thread writer([&] {
        for (std::uint32_t n=1;n<10000;++n) channel.publish({n,n+1,n+2,n+3,{float(n),float(n+1),float(n+2)},true});
        finished.store(true);
    });
    bool coherent = true;
    while (!finished.load()) {
        if (channel.consume(sample)) coherent = coherent && sample.target==sample.source+1 && sample.owner==sample.source+3 &&
            sample.point[0]==float(sample.source) && sample.point[2]==float(sample.source+2);
    }
    writer.join();
    ok &= check(coherent, "concurrent publication never mixes another limb with a contact point");
    return ok ? 0 : 1;
}
