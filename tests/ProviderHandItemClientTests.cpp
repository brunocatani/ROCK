#include "api/providers/GrabMarshalling.h"
#include "api/providers/HandsMarshalling.h"
#include <cassert>
int main() {
    using namespace rock;
    provider::RockProviderHandInteractionStateV1 state{};
    state.targetFormId=123; state.releaseSequence=45; state.gripSequence=46;
    state.targetSequence=47; state.reservedTargetIdentity=789;
    state.phase=provider::RockProviderHandInteractionPhaseV1::Holding;
    api::grab::HandInteractionStateV1 result{};
    api::boundary::convert(result,state);
    assert(result.targetFormId==123 && result.releaseSequence==45 && result.gripSequence==46);
    assert(result.targetSequence==47 && result.reservedTargetIdentity==789);
    provider::RockProviderHandTargetDetailsV1 details{};
    details.handState=state; details.reference.referenceFormId=123; details.reference.referenceNativeHandle=987;
    api::grab::HandTargetDetailsV1 target{};
    api::boundary::convert(target,details);
    assert(target.referenceNativeHandle==987 && target.handState.releaseSequence==45);
    api::grab::PowerArmorGrabRequestV1 request{}; request.target={123,987,1,2,3};
    provider::RockProviderPowerArmorGrabRequestV1 native{};
    api::boundary::convert(native,request);
    assert(native.target.referenceNativeHandle==987 && native.target.providerGeneration==3);
    provider::RockProviderHandFrameV1 frame{}; frame.flags=0xFFFFFFFF;
    api::hands::HandFrameV1 pose{}; api::boundary::convert(pose,frame);
    assert((pose.flags & ((1u<<2)|(1u<<3)))==0);
    pose.size=sizeof(pose)-1;
    assert(api::boundary::checkOutput(&pose)==api::Status::InvalidSize && pose.size==sizeof(pose)-1);
}
