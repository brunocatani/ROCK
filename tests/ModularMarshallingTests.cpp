#include "api/providers/AnimationMarshalling.h"
#include "api/providers/CollisionMarshalling.h"
#include "api/providers/HandsMarshalling.h"
#include "api/providers/ReferencesMarshalling.h"
#include <cassert>
#include <iterator>

namespace {
    template<class A,class B> void samePoint(const A& a,const B& b) {
        assert(a.x==b.x && a.y==b.y && a.z==b.z);
    }
    template<class A,class B> void samePose(const A& a,const B& b) {
        for(std::size_t i=0;i<9;++i)assert(a.rotate[i]==b.rotate[i]);
        for(std::size_t i=0;i<3;++i)assert(a.translate[i]==b.translate[i]);
        assert(a.scale==b.scale);
    }
    rock::provider::RockProviderTransform pose() {
        return {{0.36f,-0.48f,0.8f,0.8f,0.6f,0.0f,-0.48f,0.64f,0.6f},{11.0f,-22.0f,33.0f},1.25f};
    }
}
int main() {
    using namespace rock;
    using api::boundary::convert;
    provider::RockProviderPoint3 point{11,-22,33};
    api::Point3 publicPoint{}; convert(publicPoint,point); samePoint(point,publicPoint);
    provider::RockProviderPoint3 returnedPoint{}; convert(returnedPoint,publicPoint);samePoint(point,returnedPoint);
    const auto nativePose=pose();api::Transform publicPose{};convert(publicPose,nativePose);samePose(nativePose,publicPose);
    provider::RockProviderTransform returnedPose{};convert(returnedPose,publicPose);samePose(nativePose,returnedPose);
    provider::RockProviderBounds3 bounds{{-11,-22,-33},{44,55,66},1,0};
    api::Bounds3 publicBounds{};convert(publicBounds,bounds);
    samePoint(bounds.min,publicBounds.min);samePoint(bounds.max,publicBounds.max);assert(publicBounds.valid==1);
    provider::RockProviderBounds3 returnedBounds{};convert(returnedBounds,publicBounds);
    samePoint(bounds.min,returnedBounds.min);samePoint(bounds.max,returnedBounds.max);

    // Cover nested reads and writes used by separate feature providers.
    provider::RockProviderHandFrameV1 hand{};hand.transform=nativePose;
    api::hands::HandFrameV1 publicHand{};convert(publicHand,hand);samePose(hand.transform,publicHand.transform);
    api::collision::WorldRaycastRequestV1 ray{};ray.startGame={100,-20,30};ray.directionGame={1,2,3};
    provider::RockProviderWorldRaycastRequestV1 nativeRay{};convert(nativeRay,ray);
    samePoint(ray.startGame,nativeRay.startGame);samePoint(ray.directionGame,nativeRay.directionGame);
    api::animation::HandVisualAuthorityRequestV1 visual{};visual.worldTransform=publicPose;
    visual.fingerLocalTransformMask=0x7fff;
    for(std::size_t i=0;i<std::size(visual.fingerLocalTransforms);++i) {
        visual.fingerLocalTransforms[i]=publicPose;
        visual.fingerLocalTransforms[i].translate[0]+=static_cast<float>(i);
    }
    provider::RockProviderHandVisualAuthorityRequestV1 nativeVisual{};convert(nativeVisual,visual);
    samePose(visual.worldTransform,nativeVisual.worldTransform);
    for(std::size_t i=0;i<15;++i)samePose(visual.fingerLocalTransforms[i],nativeVisual.fingerLocalTransforms[i]);
    api::animation::HandVisualAuthorityRequestV1 returnedVisual{};convert(returnedVisual,nativeVisual);
    for(std::size_t i=0;i<15;++i)samePose(visual.fingerLocalTransforms[i],returnedVisual.fingerLocalTransforms[i]);

    provider::RockProviderPowerArmorTargetV1 target{};
    target.points[0].point=provider::RockProviderPowerArmorPointV1::RightArmorHand;
    target.points[1].point=provider::RockProviderPowerArmorPointV1::LeftArmorHand;
    for(auto& pointPose:target.points) {pointPose.valid=1;pointPose.world=nativePose;pointPose.frameLocal=nativePose;}
    api::references::PowerArmorTargetV1 publicTarget{};convert(publicTarget,target);
    assert(publicTarget.points[0].point==api::PowerArmorPointV1::RightArmorHand);
    assert(publicTarget.points[1].point==api::PowerArmorPointV1::LeftArmorHand);
    provider::RockProviderPowerArmorTargetV1 returnedTarget{};convert(returnedTarget,publicTarget);
    for(std::size_t i=0;i<2;++i) {
        assert(returnedTarget.points[i].point==target.points[i].point && returnedTarget.points[i].valid==1);
        samePose(returnedTarget.points[i].world,target.points[i].world);
        samePose(returnedTarget.points[i].frameLocal,target.points[i].frameLocal);
    }
}
