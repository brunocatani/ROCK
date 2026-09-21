#include "api/providers/EventBoundary.h"
#include "api/providers/GrabMarshalling.h"
#include "api/OwnerBindingPolicy.h"
#include <cassert>
#include <future>
#include <thread>

namespace {
    constexpr rock::api::OwnerToken testOwner=42, peerOwner=43;
    rock::provider::InterfaceBinding binding{1,1};
    bool revoked=false, revokeRequested=false;
    unsigned deliveries=0, peerDeliveries=0;
    rock::api::Status reentry=rock::api::Status::Ok;
    const auto ownerThread=std::this_thread::get_id();
    bool snapshotReady=true;
}
// Native runtime ownership is the fixture boundary. The production access
// policy, exported Grab endpoint bodies and event dispatcher are exercised.
namespace rock::provider::runtime {
    api::Status authorize(std::uint64_t token,api::InterfaceId family,std::uint32_t permission,bool requireThread,OwnerAccess access) {
        if(events::inSynchronousCallback())return api::Status::Busy;
        if(token!=testOwner && token!=peerOwner)return api::Status::OwnerNotRegistered;
        if(family!=api::InterfaceId::Grab)return api::Status::PermissionDenied;
        const auto status=authorizeBinding(binding,token==testOwner && revoked,permission,access);
        if(status!=api::Status::Ok)return status;
        return requireThread && std::this_thread::get_id()!=ownerThread?api::Status::WrongThread:api::Status::Ok;
    }
    api::SampleV1 sample() {return {20,30,1,2,3,4};}
    void deferRevoke(std::uint64_t token) {assert(token==testOwner);revoked=true;revokeRequested=true;}
    void reportBoundaryFailure(std::uint64_t,api::InterfaceId) noexcept {assert(false);}
    RockProviderResultV1 apiRequestForceGrabV1(std::uint64_t,const RockProviderForceGrabRequestV1*,std::uint64_t*) {
        assert(false);return RockProviderResultV1::NotReady;
    }
    RockProviderResultV1 apiGetHandInteractionStateV1(std::uint64_t,RockProviderHand hand,RockProviderHandInteractionStateV1* state) {
        if(!snapshotReady)return RockProviderResultV1::NotReady;
        state->hand=hand;
        state->flags=static_cast<std::uint32_t>(RockProviderHandInteractionFlagV1::Valid)|
            static_cast<std::uint32_t>(RockProviderHandInteractionFlagV1::LooseWeapon);
        state->phase=RockProviderHandInteractionPhaseV1::Holding;
        state->targetFormId=123;
        state->frameIndex=20;
        state->worldGeneration=1;state->skeletonGeneration=2;state->providerGeneration=3;
        return RockProviderResultV1::Ok;
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
    // The exported read must work from an F4SE task without admitting writes.
    binding.permissions=3;
    auto task=std::async(std::launch::async,[] {
        api::grab::HandInteractionStateV1 state{};
        for(const auto hand:{api::Hand::Left,api::Hand::Right}) {
            assert(api::grab::getHandInteractionStateV1(testOwner,hand,&state)==Status::Ok);
            assert(state.hand==hand && state.phase==api::grab::HandInteractionPhaseV1::Holding);
            assert(state.flags&static_cast<std::uint32_t>(api::grab::HandInteractionFlagV1::LooseWeapon));
            assert(state.targetFormId==123 && state.frameIndex==20 && state.providerGeneration==3);
        }
        assert(api::grab::getHandInteractionStateV1(999,api::Hand::Left,&state)==Status::OwnerNotRegistered);
        assert(state.flags==0 && state.targetFormId==0);
        api::grab::InventoryGrabRequestV1 request{};std::uint64_t command=99;
        assert(api::grab::requestInventoryGrab(testOwner,&request,&command)==Status::WrongThread);
        assert(command==0);
    });
    task.get();
    snapshotReady=false;
    api::grab::HandInteractionStateV1 unavailable{};
    unavailable.flags=0xFFFFFFFF;
    assert(api::grab::getHandInteractionStateV1(testOwner,api::Hand::Left,&unavailable)==Status::NotReady);
    assert(unavailable.flags==0);
    snapshotReady=true;
    binding.permissions=1;
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
