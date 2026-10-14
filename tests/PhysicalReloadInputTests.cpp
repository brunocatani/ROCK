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
    ::weapon.flags=static_cast<std::uint32_t>(provider::RockProviderEquippedWeaponStateFlagV1::Valid)|
        static_cast<std::uint32_t>(provider::RockProviderEquippedWeaponStateFlagV1::WeaponDrawn);
    constexpr auto x=1ull<<7;
    // The production reload gesture lease and the same tap/hold policy used
    // by runtime, without and with gameplay priority.
    // Test both physical firing hands; an empty chamber only gates the trigger.
    for(bool gameplayPriority:{false,true}) for(bool left:{true,false}) {
        const auto priority=gameplayPriority?static_cast<std::uint32_t>(input::ReloadGestureFlag::GameplayPriority):0u;
        const auto acquire=[&](OwnerToken owner,input::ReloadGestureRequest value){value.flags|=priority;return control::set(owner,value);};
        control::reset();raw={true};
        ::primary=left?provider::RockProviderHand::Left:provider::RockProviderHand::Right;
        ::weapon.flags=1|512;
        input::ReloadGestureRequest gesture;gesture.hand=left?Hand::Left:Hand::Right;
        gesture.weaponFormId=123;gesture.weaponGenerationKey=456;gesture.sample=sample;gesture.flags=1;
        assert(acquire(1,gesture)==Status::Ok);
        // One owner at a time.
        assert(acquire(2,gesture)==Status::Busy&&control::clear(2)==Status::Busy);
        input::ReloadGestureState observed;
        control::read(observed);auto before=observed.tapSequence;
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
            control::read(observed);return decision;
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
            assert(control::set(1,gesture)==Status::Ok&&!control::nativeActivationPriority(left));
            assert(control::takeGestureChange()&&control::gameMask(left,x)==x);
            update({true,true});update({true,false,false,true});update({true});raw={true};
            assert(observed.tapSequence==before);
            control::gameMask(left,0);
        }
        // Lease expiry requires a fresh physical release.
        update({true,true,true});sample.frameIndex+=3;control::beginFrame(sample.frameIndex);
        control::read(observed);assert(!observed.active&&control::takeGestureChange());
        assert(control::gameMask(left,x)==x); // The held Accept stays drained from the game.
        assert(control::observe(left,{true,true}));assert(control::observe(left,{true,false,false,true}));
        control::gameMask(left,0);raw={true};gesture.sample=sample;
        assert(acquire(1,gesture)==Status::Ok);
        ::primary=left?provider::RockProviderHand::Right:provider::RockProviderHand::Left;
        control::beginFrame(sample.frameIndex);control::read(observed);assert(!observed.active);
        assert(acquire(1,gesture)==Status::HandUnavailable);
        control::reset();control::gameMask(left,0);control::observe(left,{true});
    }
    // A world change ends a live lease, and a request on the stale sample is refused.
    {
        control::reset();raw={true};::primary=provider::RockProviderHand::Left;
        input::ReloadGestureRequest gesture;gesture.hand=Hand::Left;
        gesture.weaponFormId=123;gesture.weaponGenerationKey=456;gesture.sample=sample;
        assert(control::set(1,gesture)==Status::Ok);
        ++sample.worldGeneration;control::beginFrame(sample.frameIndex);
        input::ReloadGestureState observed;control::read(observed);assert(!observed.active);
        assert(control::set(1,gesture)==Status::GenerationMismatch);
        control::reset();control::gameMask(true,0);
    }
}
