#include "api/providers/EventBoundary.h"
#include "api/OwnerBindingPolicy.h"
#include <cassert>

namespace {
    constexpr rock::api::OwnerToken testOwner=42, peerOwner=43;
    rock::provider::InterfaceBinding binding{1,1};
    bool revoked=false, revokeRequested=false;
    unsigned deliveries=0, peerDeliveries=0;
    rock::api::Status reentry=rock::api::Status::Ok;
}
// Native runtime ownership is the fixture boundary. The production access
// policy, exported Grab endpoint bodies and event dispatcher are exercised.
namespace rock::provider::runtime {
    api::Status authorize(std::uint64_t token,api::InterfaceId family,std::uint32_t permission,bool,OwnerAccess access) {
        if(events::inSynchronousCallback())return api::Status::Busy;
        if(token!=testOwner && token!=peerOwner)return api::Status::OwnerNotRegistered;
        if(family!=api::InterfaceId::Grab)return api::Status::PermissionDenied;
        return authorizeBinding(binding,token==testOwner && revoked,permission,access);
    }
    api::SampleV1 sample() {return {20,30,1,2,3,4};}
    void deferRevoke(std::uint64_t token) {assert(token==testOwner);revoked=true;revokeRequested=true;}
    void reportBoundaryFailure(std::uint64_t,api::InterfaceId) noexcept {assert(false);}
    RockProviderResultV1 apiRequestForceGrabV1(std::uint64_t,const RockProviderForceGrabRequestV1*,std::uint64_t*) {
        assert(false);return RockProviderResultV1::NotReady;
    }
}
namespace rock::api::grab {
    using namespace boundary;
    #include "api/providers/GrabEndpoints.inl"
}
namespace {
    void ROCK_CALL listener(const rock::api::grab::EventV1* event,void*) {
        ++deliveries;
        assert(event->hand==rock::api::Hand::Left && event->velocityGame[0]==11);
        reentry=rock::api::grab::clearEventCallback(testOwner);
    }
    void ROCK_CALL peerListener(const rock::api::grab::EventV1*,void*) {++peerDeliveries;}
    void ROCK_CALL failingListener(const rock::api::grab::EventV1*,void*) {throw 42;}
}
int main() {
    using namespace rock;
    using api::Status;
    provider::events::bind(testOwner,api::InterfaceId::Grab);
    provider::events::bind(peerOwner,api::InterfaceId::Grab);
    provider::RockProviderEventV1 change{};
    change.kind=provider::RockProviderEventKindV1::GrabStateChanged;
    change.frameIndex=20;change.worldGeneration=1;change.skeletonGeneration=2;change.providerGeneration=3;
    change.hand=provider::RockProviderHand::Left;change.formId=123;
    change.subjectSequence=99;
    change.result=static_cast<unsigned>(provider::RockProviderHandInteractionPhaseV1::Holding);
    change.data[0]=1; // A different target-kind value must never become the phase.
    change.data[1]=456;change.data[2]=0x55;
    provider::events::publish(change);
    api::grab::EventV1 copied[4]{};api::StreamV1 stream{};
    assert(api::grab::copyEvents(testOwner,0,copied,4,&stream)==Status::Ok && stream.copiedCount==1);
    assert(copied[0].phase==change.result && copied[0].phase!=change.data[0]);
    assert(copied[0].formId==123 && copied[0].primaryBodyId==456 && copied[0].flags==0x55);
    assert(copied[0].frameIndex==20 && copied[0].worldGeneration==1 && copied[0].subjectSequence==99);

    assert(api::grab::setEventCallback(testOwner,listener,nullptr)==Status::Ok);
    assert(api::grab::setEventCallback(peerOwner,peerListener,nullptr)==Status::Ok);
    GrabEventData released{};released.type=GrabEventType::Released;released.isLeft=true;
    released.formID=123;released.velocityGame[0]=11;released.velocityGame[1]=-22;released.velocityGame[2]=33;
    provider::events::publishGrab(released,{21,30,1,2,3,4});
    assert(deliveries==1 && peerDeliveries==1 && reentry==Status::Busy);
    assert(api::grab::copyEvents(testOwner,1,copied,4,&stream)==Status::Ok && stream.copiedCount==1);
    assert(copied[0].kind==static_cast<unsigned>(api::grab::EventKindV1::Released));
    assert(copied[0].velocityGame[0]==11 && copied[0].velocityGame[1]==-22 && copied[0].velocityGame[2]==33);

    assert(api::grab::setEventCallback(testOwner,failingListener,nullptr)==Status::Ok);
    provider::events::publishGrab(released,{22,30,1,2,3,4});
    assert(revokeRequested && revoked && peerDeliveries==2);
    assert(api::grab::setEventCallback(testOwner,listener,nullptr)==Status::OwnerRevoked);
    assert(api::grab::copyEvents(testOwner,0,copied,4,&stream)==Status::Ok && stream.copiedCount==3);
    assert(api::grab::clearEventCallback(testOwner)==Status::Ok); // Read/cleanup survives revocation.
    provider::events::publishGrab(released,{23,30,1,2,3,4});
    assert(deliveries==1 && peerDeliveries==3);
    assert(provider::authorizeBinding(binding,true,2)==Status::OwnerRevoked);
    assert(provider::authorizeBinding(binding,false,2)==Status::PermissionDenied);
    binding.permissions=0;revoked=false;
    assert(api::grab::setEventCallback(testOwner,listener,nullptr)==Status::PermissionDenied);
    assert(api::grab::setEventCallback(99,listener,nullptr)==Status::OwnerNotRegistered);
    assert(api::grab::setEventCallback(testOwner,nullptr,nullptr)==Status::InvalidArgument);
    provider::events::remove(testOwner);provider::events::remove(peerOwner);
}
