#include "api/providers/EventBoundary.h"
#include <ROCK/Grab.h>
#include "physics-interaction/grab/SavedGrabOffsetStore.h"
#include <cstring>
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
// policy, the published Grab table and event dispatcher are exercised.
namespace rock::provider::runtime {
    api::Status authorize(std::uint64_t token,api::InterfaceId family,std::uint32_t permission,bool requireThread,OwnerAccess access) {
        if(events::inSynchronousCallback())return api::Status::Busy;
        if(token!=testOwner && token!=peerOwner)return api::Status::OwnerNotRegistered;
        if(family!=api::InterfaceId::Grab)return api::Status::PermissionDenied;
        const auto status=authorizeBinding(binding,token==testOwner && revoked,permission,access);
        if(status!=api::Status::Ok)return status;
        return requireThread && std::this_thread::get_id()!=ownerThread?api::Status::WrongThread:api::Status::Ok;
    }
    api::Sample sample() {return {20,30,1,2,3,4};}
    void deferRevoke(std::uint64_t token) {assert(token==testOwner);revoked=true;revokeRequested=true;}
    void reportBoundaryFailure(std::uint64_t,api::InterfaceId) noexcept {assert(false);}
    api::Status apiRequestForceGrabV1(std::uint64_t,const RockProviderForceGrabRequestV1*,std::uint64_t*) {
        assert(false);return api::Status::NotReady;
    }
    api::Status apiGetHandInteractionStateV1(std::uint64_t,api::Hand hand,RockProviderHandInteractionStateV1* state) {
        if(!snapshotReady)return api::Status::NotReady;
        state->hand=hand;
        state->flags=static_cast<std::uint32_t>(api::grab::HandInteractionFlag::Valid)|
            static_cast<std::uint32_t>(api::grab::HandInteractionFlag::LooseWeapon);
        state->phase=api::grab::HandInteractionPhase::Holding;
        state->targetFormId=123;
        state->frameIndex=20;
        state->worldGeneration=1;state->skeletonGeneration=2;state->providerGeneration=3;
        return api::Status::Ok;
    }
    api::Status apiGetHandTargetDetailsV1(std::uint64_t owner,api::Hand hand,RockProviderHandTargetDetailsV1* details) {
        const auto result=apiGetHandInteractionStateV1(owner,hand,&details->handState);
        details->handState.releaseSequence=45;details->handState.reservedTargetIdentity=789;
        details->reference.referenceFormId=123;details->reference.referenceNativeHandle=987;
        std::strcpy(details->meshPartName,"Lid");
        return result;
    }
    api::Status apiRequestPowerArmorGrabV1(std::uint64_t,const RockProviderPowerArmorGrabRequestV1* request,std::uint64_t* command) {
        assert(request->target.referenceFormId==123 && request->target.referenceNativeHandle==987);
        assert(request->target.providerGeneration==3 && request->point==api::PowerArmorPoint::LeftArmorHand);
        *command=7;return api::Status::RequestQueued;
    }
    api::Status apiQueryPowerArmorTargetV1(std::uint64_t,const RockProviderReferenceQueryV1* query,RockProviderPowerArmorTargetV1* target) {
        assert(query->referenceFormId==123 && query->furnitureMarkerIndex==2);
        target->actorFormId=55;target->frameReference.referenceFormId=124;
        target->points[1].point=api::PowerArmorPoint::LeftArmorHand;target->points[1].valid=1;
        target->points[1].world.translate[0]=9;
        return api::Status::Ok;
    }
    // Grab calls this test does not reach.
    api::Status apiGetInteractionCommandResultV1(std::uint64_t,std::uint64_t,RockProviderInteractionCommandResultV1*) {assert(false);return api::Status::NotReady;}
    api::Status apiRequestForceReleaseV1(std::uint64_t,const RockProviderForceReleaseRequestV1*,std::uint64_t*) {assert(false);return api::Status::NotReady;}
    api::Status apiRequestThrownDropV1(std::uint64_t,const RockProviderThrownDropRequestV1*,std::uint64_t*) {assert(false);return api::Status::NotReady;}
    api::Status apiCancelInteractionCommandV1(std::uint64_t,std::uint64_t) {assert(false);return api::Status::NotReady;}
    api::Status apiAcquireOffhandReservationV1(std::uint64_t,const RockProviderOffhandReservationRequestV1*) {assert(false);return api::Status::NotReady;}
    api::Status apiRenewOffhandReservationV1(std::uint64_t,const RockProviderOffhandReservationRequestV1*) {assert(false);return api::Status::NotReady;}
    api::Status apiReleaseOffhandReservationV1(std::uint64_t) {assert(false);return api::Status::NotReady;}
    api::Status apiGetOffhandReservationStateV1(std::uint64_t,RockProviderOffhandReservationStateV1*) {assert(false);return api::Status::NotReady;}
    api::Status apiQueryReferenceInteractionV1(std::uint64_t,const RockProviderReferenceQueryV1*,RockProviderReferenceInteractionV1*) {assert(false);return api::Status::NotReady;}
    api::Status getHeldPlacementState(api::grab::HeldPlacementState&) {assert(false);return api::Status::NotReady;}
    api::Status submitHeldPlacementIntent(api::OwnerToken,const api::grab::HeldPlacementIntent&) {assert(false);return api::Status::NotReady;}
    api::Status clearHeldPlacementIntent(api::OwnerToken) {assert(false);return api::Status::NotReady;}
    api::Status setHeldObjectTarget(api::OwnerToken,const api::grab::HeldObjectTarget&) {assert(false);return api::Status::NotReady;}
    api::Status clearHeldObjectTarget(api::OwnerToken,api::Hand) {assert(false);return api::Status::NotReady;}
    api::Status getHeldObjectPose(api::OwnerToken,api::Hand,api::grab::HeldObjectPose&) {assert(false);return api::Status::NotReady;}
}
namespace rock::saved_grab_offset {
    api::Status setDefaultGrabPoses(api::OwnerToken,const api::grab::DefaultGrabPoses&) {assert(false);return api::Status::NotReady;}
    api::Status clearDefaultGrabPoses(api::OwnerToken,std::uint32_t) {assert(false);return api::Status::NotReady;}
}
namespace rock::api::grab { const Api& table() noexcept; }
namespace {
    const rock::api::grab::Api& grabApi=rock::api::grab::table();
}
namespace {
    void ROCK_CALL listener(const rock::api::grab::Event* event,void*) {
        ++deliveries;
        assert(event->hand==rock::api::Hand::Left && event->velocityGame[0]==11);
        reentry=grabApi.clearEventCallback(testOwner);
    }
    void ROCK_CALL peerListener(const rock::api::grab::Event*,void*) {++peerDeliveries;}
    void ROCK_CALL failingListener(const rock::api::grab::Event*,void*) {throw 42;}
}
int main() {
    using namespace rock;
    using api::Status;
    // The exported read must work from an F4SE task without admitting writes.
    binding.permissions=3;
    auto task=std::async(std::launch::async,[] {
        api::grab::HandInteractionState state{};
        for(const auto hand:{api::Hand::Left,api::Hand::Right}) {
            assert(grabApi.getHandInteractionState(testOwner,hand,&state)==Status::Ok);
            assert(state.hand==hand && state.phase==api::grab::HandInteractionPhase::Holding);
            assert(state.flags&static_cast<std::uint32_t>(api::grab::HandInteractionFlag::LooseWeapon));
            assert(state.targetFormId==123 && state.frameIndex==20 && state.providerGeneration==3);
        }
        assert(grabApi.getHandInteractionState(999,api::Hand::Left,&state)==Status::OwnerNotRegistered);
        assert(state.flags==0 && state.targetFormId==0);
        api::grab::InventoryGrabRequest request{};std::uint64_t command=99;
        assert(grabApi.requestInventoryGrab(testOwner,&request,&command)==Status::WrongThread);
        assert(command==0);
    });
    task.get();
    {
        // Values cross the Grab boundary field for field.
        api::grab::HandTargetDetails details{};
        assert(grabApi.getHandTargetDetails(testOwner,api::Hand::Right,&details)==Status::Ok);
        assert(details.referenceFormId==123 && details.referenceNativeHandle==987 && std::strcmp(details.meshPartName,"Lid")==0);
        assert(details.handState.releaseSequence==45 && details.handState.reservedTargetIdentity==789 && details.handState.hand==api::Hand::Right);
        api::grab::PowerArmorGrabRequest grab{};grab.target={123,987,1,2,3};grab.hand=api::Hand::Left;
        grab.point=api::PowerArmorPoint::LeftArmorHand;std::uint64_t command=0;
        assert(grabApi.requestPowerArmorGrab(testOwner,&grab,&command)==Status::RequestQueued && command==7);
        api::grab::ReferenceQuery query{};query.referenceFormId=123;query.furnitureMarkerIndex=2;
        api::grab::PowerArmorTarget target{};
        assert(grabApi.queryPowerArmorTarget(testOwner,&query,&target)==Status::Ok);
        assert(target.actorFormId==55 && target.frameReference.referenceFormId==124);
        assert(target.points[1].point==api::PowerArmorPoint::LeftArmorHand && target.points[1].valid==1 && target.points[1].world.translate[0]==9);
    }
    snapshotReady=false;
    api::grab::HandInteractionState unavailable{};
    unavailable.flags=0xFFFFFFFF;
    assert(grabApi.getHandInteractionState(testOwner,api::Hand::Left,&unavailable)==Status::NotReady);
    assert(unavailable.flags==0);
    snapshotReady=true;
    binding.permissions=1;
    provider::events::bind(testOwner,api::InterfaceId::Grab);
    provider::events::bind(peerOwner,api::InterfaceId::Grab);
    provider::RockProviderEventV1 change{};
    change.kind=provider::RockProviderEventKindV1::GrabStateChanged;
    change.frameIndex=20;change.worldGeneration=1;change.skeletonGeneration=2;change.providerGeneration=3;
    change.hand=api::Hand::Left;change.formId=123;
    change.subjectSequence=99;
    change.result=static_cast<unsigned>(api::grab::HandInteractionPhase::Holding);
    change.data[0]=1; // A different target-kind value must never become the phase.
    change.data[1]=456;change.data[2]=0x55;
    provider::events::publish(change);
    api::grab::Event copied[4]{};api::Stream stream{};
    assert(grabApi.copyEvents(testOwner,0,copied,4,&stream)==Status::Ok && stream.copiedCount==1);
    assert(copied[0].phase==change.result && copied[0].phase!=change.data[0]);
    assert(copied[0].formId==123 && copied[0].primaryBodyId==456 && copied[0].flags==0x55);
    assert(copied[0].frameIndex==20 && copied[0].worldGeneration==1 && copied[0].subjectSequence==99);

    assert(grabApi.setEventCallback(testOwner,listener,nullptr)==Status::Ok);
    assert(grabApi.setEventCallback(peerOwner,peerListener,nullptr)==Status::Ok);
    GrabEventData released{};released.type=GrabEventType::Released;released.isLeft=true;
    released.formID=123;released.velocityGame[0]=11;released.velocityGame[1]=-22;released.velocityGame[2]=33;
    provider::events::publishGrab(released,{21,30,1,2,3,4});
    assert(deliveries==1 && peerDeliveries==1 && reentry==Status::Busy);
    assert(grabApi.copyEvents(testOwner,1,copied,4,&stream)==Status::Ok && stream.copiedCount==1);
    assert(copied[0].kind==static_cast<unsigned>(api::grab::EventKind::Released));
    assert(copied[0].velocityGame[0]==11 && copied[0].velocityGame[1]==-22 && copied[0].velocityGame[2]==33);

    assert(grabApi.setEventCallback(testOwner,failingListener,nullptr)==Status::Ok);
    provider::events::publishGrab(released,{22,30,1,2,3,4});
    assert(revokeRequested && revoked && peerDeliveries==2);
    assert(grabApi.setEventCallback(testOwner,listener,nullptr)==Status::OwnerRevoked);
    assert(grabApi.copyEvents(testOwner,0,copied,4,&stream)==Status::Ok && stream.copiedCount==3);
    assert(grabApi.clearEventCallback(testOwner)==Status::Ok); // Read/cleanup survives revocation.
    provider::events::publishGrab(released,{23,30,1,2,3,4});
    assert(deliveries==1 && peerDeliveries==3);
    assert(provider::authorizeBinding(binding,true,2)==Status::OwnerRevoked);
    assert(provider::authorizeBinding(binding,false,2)==Status::PermissionDenied);
    binding.permissions=0;revoked=false;
    assert(grabApi.setEventCallback(testOwner,listener,nullptr)==Status::PermissionDenied);
    assert(grabApi.setEventCallback(99,listener,nullptr)==Status::OwnerNotRegistered);
    assert(grabApi.setEventCallback(testOwner,nullptr,nullptr)==Status::InvalidArgument);
    provider::events::remove(testOwner);provider::events::remove(peerOwner);
}
