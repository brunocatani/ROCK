#include "physics-interaction/input/PhysicalReloadInput.h"
#include "api/ProviderRuntimeServices.h"
#include "physics-interaction/input/ManualScopeInputPolicy.h"
#include <cassert>
#include "physics-interaction/input/NativeSecondaryTrigger.h"

namespace {
    rock::api::Sample sample{10,1,1,2,3,4};
    rock::provider::RockProviderEquippedWeaponStateV1 weapon;
    rock::input_remap_runtime::RawButtonState raw{true};
    rock::provider::RockProviderHand primary=rock::provider::RockProviderHand::Left;
}
namespace rock::provider::runtime {
    RockProviderHand ROCK_PROVIDER_CALL apiGetPrimaryHandV1(){return ::primary;}
    std::uint64_t currentGameFrameIndex(){return ::sample.frameIndex;}
    api::Sample sample(){return ::sample;}
    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetEquippedWeaponStateV1(std::uint64_t,RockProviderEquippedWeaponStateV1* out){
        *out=::weapon;return RockProviderResultV1::Ok;
    }
}
namespace rock::input_remap_runtime {
    RawButtonState peekRawButtonState(bool,int){return ::raw;}
}
int main(){
    {
        rock::native_secondary_trigger::Intent trigger;
        trigger.observe(true,true); // Do not fire the held Pip-Boy equip press.
        assert(!trigger.wantsStart());
        trigger.observe(false,true);
        trigger.observe(true,true);
        assert(trigger.wantsStart());
        trigger.observe(true,true); // Native rejected this still-held press.
        assert(trigger.wantsStart());
        trigger.accepted(true);
        assert(trigger.active() && !trigger.wantsStart());
        trigger.observe(true,true);
        assert(!trigger.wantsStart());
        assert(trigger.observe(false,true));
        trigger.observe(true,true);
        assert(trigger.wantsStart());
        trigger.observe(false,true); // Rejected short tap cannot fire later.
        assert(!trigger.wantsStart());
        trigger.observe(true,true);
        trigger.accepted(false); // Empty-trigger reload consumes this press.
        assert(!trigger.active() && !trigger.wantsStart());
        trigger.observe(true,true);
        assert(!trigger.wantsStart());
        trigger.cancel(); // Equipment change/menu/provider loss cancels it.
        trigger.observe(true,true);
        assert(!trigger.wantsStart());
        trigger.observe(false,true);
        trigger.observe(true,true);
        trigger.accepted(true);
        assert(trigger.observe(true,false)); // Stale input stops active fire.
        trigger.observe(true,true);
        assert(!trigger.wantsStart());
        trigger.observe(false,true);
        trigger.observe(true,true);
        assert(trigger.wantsStart());
    }
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
    // Production Input 1.3 lease and the same tap/hold policy used by runtime.
    // Test both physical firing hands; an empty chamber only gates the trigger.
    for(bool gameplayPriority:{false,true}) for(bool left:{true,false}) {
        const auto acquire=gameplayPriority?control::setGameplayGesture:control::setGesture;
        control::reset();raw={true};
        ::primary=left?provider::RockProviderHand::Left:provider::RockProviderHand::Right;
        ::weapon.flags=1|512;
        input::v1_3::ReloadGestureRequest gesture;gesture.hand=left?Hand::Left:Hand::Right;
        gesture.weaponFormId=123;gesture.weaponGenerationKey=456;gesture.sample=sample;gesture.flags=1;
        assert(acquire(1,gesture)==Status::Ok);
        control::read(state);assert(!state.active&&!state.owner); // never present shared scope routing as legacy exclusive capture
        assert(control::set(2,request)==Status::GenerationMismatch); // stale legacy sample
        input::v1_3::ReloadGestureState observed;
        control::readGesture(observed);auto before=observed.tapSequence;
        manual_scope_input_policy::RuntimeState scope;
        const auto update=[&](input_remap_runtime::RawButtonState button,float dt=.01f,bool nativeTarget=false) {
            ++sample.frameIndex;gesture.sample=sample;assert(acquire(1,gesture)==Status::Ok);
            control::beginFrame(sample.frameIndex);
            if(control::takeGestureChange())manual_scope_input_policy::blockUntilRelease(scope);
            if(control::observe(left,button))button={};
            manual_scope_input_policy::Input in;in.gameplayInputAllowed=in.weaponDrawn=true;in.firingHandIsLeft=left;in.deltaSeconds=dt;in.holdSeconds=.3f;
            in.leftButton=in.rightButton={true};in.nativeActivationTarget=nativeTarget&&control::nativeActivationPriority(left);
            (left?in.leftButton:in.rightButton)={button.available,button.held,button.pressed,button.released};
            const auto decision=manual_scope_input_policy::update(scope,in);
            assert(control::routeGesture(left,decision.dispatchReload,decision.scopeRequested));
            control::readGesture(observed);return decision;
        };
        update({true});assert(control::fireBlocked(left));
        const auto mask=control::gameMask(left,x|(1ull<<33));assert(mask==((gameplayPriority?0:x)|(1ull<<33)));
        assert(control::nativeActivationPriority(left)==gameplayPriority);
        update({true,true,true});assert(observed.tapSequence==before);
        update({true,false,false,true});assert(observed.tapSequence==++before);
        update({true});assert(observed.tapSequence==before);
        update({true,true,true});for(int i=0;i<4;++i)update({true,true},.1f);
        assert(observed.scopeHeld&&observed.tapSequence==before&&control::fireBlocked(left));
        gesture.flags=0;update({true,true});assert(observed.scopeHeld&&control::fireBlocked(left));
        // A held physical trigger remains drained after its gate clears.
        assert(control::gameMask(left,x|(1ull<<33))==((gameplayPriority?0:x)|(1ull<<33)));
        assert(control::gameMask(left,x)==(gameplayPriority?0:x)&&!control::fireBlocked(left));
        update({true,false,false,true});assert(!observed.scopeHeld&&observed.tapSequence==before);
        // Input/UI loss discards a pending tap and cannot replay on restoration.
        update({true,true,true});update({});update({true,true,true});update({true,false,false,true});update({true});
        assert(observed.tapSequence==before);
        update({true,true,true});update({true,false,false,true});assert(observed.tapSequence==++before);
        if(gameplayPriority&&!left) {
            scope={};
            // Native-first and raw-first delivery share the same full gesture.
            // Workbench/take use may remove its target or open a menu immediately.
            for(bool nativeFirst:{true,false}) {
                if(nativeFirst)manual_scope_input_policy::beginPrimaryActivateGesture(scope,true);
                const auto press=update({true,true,true},.01f,true);
                if(!nativeFirst)manual_scope_input_policy::beginPrimaryActivateGesture(scope,false);
                assert(!press.dispatchReload&&scope.primaryPressUsesNative);
                for(int i=0;i<4;++i)update({true,true},.1f,false);
                assert(!observed.scopeHeld&&observed.tapSequence==before);
                update({true,false,false,true});assert(observed.tapSequence==before);
                update({true});
            }
            // A target appearing during an existing reload/scope press cannot
            // steal it, and release after native activation cannot eject a mag.
            update({true,true,true});manual_scope_input_policy::beginPrimaryActivateGesture(scope,true);
            assert(!scope.primaryPressUsesNative);
            update({true,false,false,true},.01f,true);assert(observed.tapSequence==++before);
            update({true});
            manual_scope_input_policy::beginPrimaryActivateGesture(scope,true);
            update({true,true,true},.01f,true);update({});update({true,true,true});update({true,false,false,true});update({true});
            assert(observed.tapSequence==before);
        }
        // Switching acquisition modes mid-hold cannot inherit a reload tap.
        if(gameplayPriority) {
            update({true,true,true});raw={true,true};
            assert(control::setGesture(1,gesture)==Status::Ok&&!control::nativeActivationPriority(left));
            assert(control::takeGestureChange()&&control::gameMask(left,x)==x);
            update({true,true});update({true,false,false,true});update({true});raw={true};
            assert(observed.tapSequence==before);
            control::gameMask(left,0);
        }
        // Lease expiry requires a fresh physical release.
        update({true,true,true});sample.frameIndex+=3;control::beginFrame(sample.frameIndex);
        control::readGesture(observed);assert(!observed.active&&control::takeGestureChange());
        assert(control::observe(left,{true,true}));assert(control::observe(left,{true,false,false,true}));
        control::gameMask(left,0);raw={true};gesture.sample=sample;
        assert(acquire(1,gesture)==Status::Ok);
        ::primary=left?provider::RockProviderHand::Right:provider::RockProviderHand::Left;
        control::beginFrame(sample.frameIndex);control::readGesture(observed);assert(!observed.active);
        assert(acquire(1,gesture)==Status::HandUnavailable);
        control::reset();control::gameMask(left,0);control::observe(left,{true});
    }
}
