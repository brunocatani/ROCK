#include "physics-interaction/input/PhysicalReloadInput.h"
#include "api/ProviderRuntimeServices.h"
#include <cassert>

namespace {
    rock::api::SampleV1 sample{10,1,1,2,3,4};
    rock::provider::RockProviderEquippedWeaponStateV1 weapon;
    rock::input_remap_runtime::RawButtonState raw{true};
}
namespace rock::provider::runtime {
    std::uint64_t currentGameFrameIndex(){return ::sample.frameIndex;}
    api::SampleV1 sample(){return ::sample;}
    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetEquippedWeaponStateV1(std::uint64_t,RockProviderEquippedWeaponStateV1* out){
        *out=::weapon;return RockProviderResultV1::Ok;
    }
}
namespace rock::input_remap_runtime {
    RawButtonState peekRawButtonState(bool,int){return ::raw;}
}
int main(){
    using namespace rock;
    using namespace api;
    namespace control=physical_reload_input;
    ::weapon.weaponFormId=123;::weapon.weaponGenerationKey=456;
    ::weapon.flags=static_cast<std::uint32_t>(weapon::EquippedWeaponStateFlagV1::Valid)|
        static_cast<std::uint32_t>(weapon::EquippedWeaponStateFlagV1::WeaponDrawn);
    input::v1_2::ReloadInputRequest request;
    request.weaponFormId=123;request.weaponGenerationKey=456;request.sample=sample;
    input::v1_2::ReloadInputState state;
    constexpr auto x=1ull<<7;
    assert(control::set(1,request)==Status::Ok);
    assert(control::gameMask(true,x)==x && control::gameMask(false,x)==0);
    assert(control::set(2,request)==Status::Busy);
    assert(control::clear(2)==Status::Busy);
    assert(control::observe(true,{true,true,true,false}));
    control::read(state);const auto first=state.pressSequence;
    assert(first!=0 && state.held);
    assert(control::set(1,request)==Status::Ok);
    control::read(state);assert(state.pressSequence==first);
    // Expiration drains both native raw input and ROCK's delayed gesture.
    sample.frameIndex=13;control::beginFrame(13);
    control::read(state);assert(!state.active);
    assert(control::gameMask(true,x)==x);
    assert(control::observe(true,{true,true,false,false}));
    assert(control::gameMask(true,0)==0);
    assert(control::observe(true,{true,false,false,true}));
    assert(!control::observe(true,{true}));
    // Acquiring a held button never fabricates a new press.
    request.sample=sample;raw={true,true};
    assert(control::set(1,request)==Status::Ok);
    control::observe(true,{true,true,true,false});
    control::read(state);assert(state.pressSequence==first);
    control::observe(true,{true,false,false,true});
    control::observe(true,{true,true,true,false});
    control::read(state);assert(state.pressSequence==first+1);
    // Blocking menus/tracking loss require release before a new gesture.
    control::observe(true,{});
    control::observe(true,{true,true,true,false});
    control::read(state);assert(state.pressSequence==first+1);
    control::observe(true,{true,false,false,true});
    ++sample.worldGeneration;control::beginFrame(sample.frameIndex);
    control::read(state);assert(!state.active);
    assert(control::set(1,request)==Status::GenerationMismatch);
    request.sample=sample;raw={true};
    assert(control::set(1,request)==Status::Ok);
    control::clearOwner(2);control::read(state);assert(state.active);
    control::clearOwner(1);control::read(state);assert(!state.active);
    raw={};assert(control::set(1,request)==Status::Ok);
    control::reset();
    assert(control::gameMask(true,x)==x);
    assert(control::gameMask(true,0)==0);
    assert(control::observe(true,{true,false,false,true}));
    assert(!control::observe(true,{true}));
    ::weapon.flags=0;assert(control::set(1,request)==Status::GenerationMismatch);
    request.leaseFrames=121;assert(control::set(1,request)==Status::InvalidArgument);
    request.leaseFrames=2;request.hand=Hand::None;assert(control::set(1,request)==Status::InvalidArgument);
    request.hand=Hand::Left;request.sample.frameIndex=99;assert(control::set(1,request)==Status::GenerationMismatch);
}
