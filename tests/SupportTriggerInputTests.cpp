#include "physics-interaction/input/SupportTriggerInput.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "api/ProviderRuntimeServices.h"
#include <cassert>

namespace {
    rock::api::Sample sample{10,1,1,2,3,4};
    rock::provider::RockProviderEquippedWeaponStateV1 weapon;
    rock::provider::RockProviderEquippedWeaponGripStateV1 grip;
    rock::input_remap_runtime::RawButtonState raw{true};
}
namespace rock::provider::runtime {
    std::uint64_t currentGameFrameIndex(){return ::sample.frameIndex;}
    api::Sample sample(){return ::sample;}
    api::Status apiGetEquippedWeaponStateV1(std::uint64_t,RockProviderEquippedWeaponStateV1* out){*out=::weapon;return api::Status::Ok;}
    bool apiGetEquippedWeaponGripStateV1(std::uint64_t,RockProviderEquippedWeaponGripStateV1* out){*out=::grip;return true;}
}
namespace rock::input_remap_runtime { RawButtonState peekRawButtonState(bool,int){return ::raw;} }
int main() {
    using namespace rock; using namespace api;
    namespace control = support_trigger_input;
    constexpr auto trigger = 1ull<<33, gripButton = 1ull<<2;
    ::weapon.weaponFormId=123; ::weapon.weaponGenerationKey=456; ::weapon.flags=1|512;
    ::grip.weaponGenerationKey=456;
    for (bool left : {true,false}) {
        control::reset();control::gameMask(true,0);control::gameMask(false,0);
        ::grip.flags=1|2|(left?0:4);raw={true,true};
        input::SupportTriggerRequest request;
        request.weaponFormId=123;request.weaponGenerationKey=456;request.sample=sample;request.hand=left?Hand::Left:Hand::Right;
        assert(control::set(1,request)==Status::Ok);
        input::SupportTriggerState state;control::read(state);auto before=state.pressSequence;
        assert(control::gameMask(left,trigger|gripButton)==trigger && control::gameMask(!left,trigger)==0);
        assert(control::set(2,request)==Status::Busy && control::clear(2)==Status::Busy);
        auto tick=[&]{++sample.frameIndex;request.sample=sample;assert(control::set(1,request)==Status::Ok);control::beginFrame(sample.frameIndex);control::read(state);};
        tick();assert(state.pressSequence==before); // entering support with trigger held
        raw={true,false};tick();raw={true,true};tick();assert(state.pressSequence==++before);
        tick();assert(state.pressSequence==before); // holding never repeats
        raw={};tick();raw={true,true};tick();assert(state.pressSequence==before); // UI/lost input
        raw={true,false};tick();raw={true,true};tick();assert(state.pressSequence==++before);
        control::clear(1);assert(control::gameMask(left,trigger)==trigger);assert(control::gameMask(left,0)==0);
        assert(control::set(1,request)==Status::Ok);::grip.flags^=4;
        control::beginFrame(sample.frameIndex);control::read(state);assert(!state.active); // handoff cancels and drains
        assert(control::gameMask(left,trigger)==trigger);control::gameMask(left,0);::grip.flags^=4;
        raw={true,false};assert(control::set(1,request)==Status::Ok);
        sample.frameIndex+=3;control::beginFrame(sample.frameIndex);control::read(state);assert(!state.active);
        assert(control::set(1,request)==Status::GenerationMismatch);request.sample=sample;
        assert(control::set(1,request)==Status::Ok);control::clearOwner(2);control::read(state);assert(state.active);
        control::clearOwner(1);control::read(state);assert(!state.active);
        request.hand=left?Hand::Right:Hand::Left;assert(control::set(1,request)==Status::GenerationMismatch);
    }
}
