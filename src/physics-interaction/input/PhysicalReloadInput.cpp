#include "physics-interaction/input/PhysicalReloadInput.h"
#include "api/ProviderRuntimeServices.h"
#include <atomic>
#include <mutex>
#include <utility>

namespace rock::physical_reload_input {
namespace {
    using namespace api;
    std::mutex mutex;
    input::ReloadGestureRequest request;
    struct LeaseState { OwnerToken owner{}; std::uint64_t expiresAfterFrame{}; Hand hand{Hand::None}; bool active{},held{}; } state;
    bool armed{};
    bool scopeHeld{},gestureChanged{},nativePriority{};
    std::uint64_t tapSequence{};
    std::uint32_t drainingHands{};
    std::atomic<std::uint32_t> activeMask{},drainMask{};
    std::atomic<std::uint32_t> fireMask{},fireDrainMask{};
    constexpr std::uint32_t handMask(Hand h){return h==Hand::Left?1u:2u;}
    void retire() {
        if(state.active)gestureChanged=true;
        // Unavailable input also needs drainage: menu/UI capture hides the
        // physical held level, and must not expose that hold when a lease ends.
        if(state.active&&(state.held||!armed)) {
            drainingHands|=handMask(state.hand);
            drainMask.fetch_or(handMask(state.hand),std::memory_order_release);
        }
        fireDrainMask.fetch_or(fireMask.exchange(0,std::memory_order_acq_rel),std::memory_order_release);
        activeMask.store(0,std::memory_order_release);state.active=false;state.owner=0;state.held=false;armed=false;scopeHeld=false;
    }
    bool current() {
        provider::RockProviderEquippedWeaponStateV1 weapon;
        const auto result=provider::runtime::apiGetEquippedWeaponStateV1(state.owner,&weapon);
        const auto sample=provider::runtime::sample();
        constexpr auto drawn=static_cast<std::uint32_t>(provider::RockProviderEquippedWeaponStateFlagV1::WeaponDrawn);
        constexpr auto valid=static_cast<std::uint32_t>(provider::RockProviderEquippedWeaponStateFlagV1::Valid);
        return result==provider::RockProviderResultV1::Ok&&(weapon.flags&(drawn|valid))==(drawn|valid)&&weapon.weaponFormId==request.weaponFormId&&
            weapon.weaponGenerationKey==request.weaponGenerationKey&&sample.worldGeneration==request.sample.worldGeneration&&
            sample.skeletonGeneration==request.sample.skeletonGeneration&&sample.providerGeneration==request.sample.providerGeneration;
    }
}
api::Status set(api::OwnerToken owner,const api::input::ReloadGestureRequest& value) {
    using api::Status;using api::Hand;using Flag=api::input::ReloadGestureFlag;
    constexpr auto blockFireFlag=static_cast<std::uint32_t>(Flag::BlockFire),priorityFlag=static_cast<std::uint32_t>(Flag::GameplayPriority);
    if(!owner||!value.weaponFormId||!value.weaponGenerationKey||!value.leaseFrames||value.leaseFrames>api::input::kMaxWeaponInputLeaseFrames||
        (value.hand!=Hand::Left&&value.hand!=Hand::Right)||(value.flags&~(blockFireFlag|priorityFlag)))return Status::InvalidArgument;
    if(static_cast<Hand>(provider::runtime::apiGetPrimaryHandV1())!=value.hand)return Status::HandUnavailable;
    const auto now=provider::runtime::currentGameFrameIndex();
    if(value.sample.frameIndex>now||now-value.sample.frameIndex>1)return Status::GenerationMismatch;
    const bool gameplayPriority=(value.flags&priorityFlag)!=0;
    std::scoped_lock lock(mutex);
    if(state.active&&state.owner!=owner)return Status::Busy;
    // Changing the hand, weapon or priority mode starts a new gesture.
    const bool continuing=state.active&&state.owner==owner&&request.weaponGenerationKey==value.weaponGenerationKey&&state.hand==value.hand&&nativePriority==gameplayPriority;
    const auto oldRequest=request;const auto oldState=state;
    request=value;state.owner=owner;
    if(!current()){request=oldRequest;state=oldState;return Status::GenerationMismatch;}
    if(!continuing) {
        retire();state.owner=owner;state.hand=value.hand;
        gestureChanged=true;
        const auto raw=input_remap_runtime::peekRawButtonState(value.hand==Hand::Left,7);
        armed=raw.available&&!raw.held;state.held=raw.held;
    }
    nativePriority=gameplayPriority;state.active=true;state.expiresAfterFrame=now+value.leaseFrames;
    const auto nextFire=(value.flags&blockFireFlag)?handMask(value.hand):0u;
    const auto oldFire=fireMask.exchange(nextFire,std::memory_order_acq_rel);
    fireDrainMask.fetch_or(oldFire&~nextFire,std::memory_order_release);
    // Gameplay-priority leases arbitrate Accept at the existing native event
    // hook. Masking it here would prevent workbenches/use/take from seeing it.
    activeMask.store(gameplayPriority?0u:handMask(value.hand),std::memory_order_release);
    return Status::Ok;
}
api::Status read(api::input::ReloadGestureState& out) {
    std::scoped_lock lock(mutex);out={};out.sample=provider::runtime::sample();out.tapSequence=tapSequence;
    if(state.active){out.active=1;out.owner=state.owner;out.hand=state.hand;out.weaponSlot=request.weaponSlot;out.held=state.held;out.scopeHeld=scopeHeld;out.expiresAfterFrame=state.expiresAfterFrame;}
    return api::Status::Ok;
}
bool gestureActive(bool left) {
    std::scoped_lock lock(mutex);return state.active&&state.hand==(left?api::Hand::Left:api::Hand::Right);
}
bool nativeActivationPriority(bool left) {
    std::scoped_lock lock(mutex);return state.active&&nativePriority&&state.hand==(left?api::Hand::Left:api::Hand::Right);
}
bool takeGestureChange(){std::scoped_lock lock(mutex);return std::exchange(gestureChanged,false);}
bool routeGesture(bool left,bool tap,bool scope) {
    std::scoped_lock lock(mutex);
    if(!state.active||state.hand!=(left?api::Hand::Left:api::Hand::Right))return false;
    scopeHeld=armed&&scope;
    if(armed&&tap)++tapSequence;
    return true;
}
bool fireBlocked(bool left) noexcept {
    return ((fireMask.load(std::memory_order_acquire)|fireDrainMask.load(std::memory_order_acquire))&(left?1u:2u))!=0;
}
api::Status clear(api::OwnerToken owner) {
    std::scoped_lock lock(mutex);
    if(state.active&&state.owner!=owner)return api::Status::Busy;
    retire();return api::Status::Ok;
}
void clearOwner(api::OwnerToken owner){std::scoped_lock lock(mutex);if(state.owner==owner)retire();}
void reset(){std::scoped_lock lock(mutex);retire();}
void beginFrame(std::uint64_t frame) {
    std::scoped_lock lock(mutex);
    if(state.active&&(frame>state.expiresAfterFrame||!current()||
        static_cast<api::Hand>(provider::runtime::apiGetPrimaryHandV1())!=state.hand))retire();
}
bool observe(bool left,const input_remap_runtime::RawButtonState& raw) {
    std::scoped_lock lock(mutex);
    const auto hand=left?api::Hand::Left:api::Hand::Right;
    const auto bit=handMask(hand);
    const bool draining=(drainingHands&bit)!=0;
    // The controller hook can see release before this frame consumes its edge.
    // Keep separate frame-side drainage so expiry cannot replay a scope/reload.
    if(draining&&raw.available&&!raw.held)drainingHands&=~bit;
    if(!state.active||state.hand!=hand)return draining;
    state.held=raw.held;
    if(!raw.available){armed=false;scopeHeld=false;return true;}
    if(!armed){if(!raw.held)armed=true;return true;}
    return draining;
}
std::uint64_t gameMask(bool left,std::uint64_t rawPressed) noexcept {
    constexpr std::uint64_t button=1ull<<7,trigger=1ull<<33;const auto bit=left?1u:2u;
    if(!(rawPressed&button))drainMask.fetch_and(~bit,std::memory_order_acq_rel);
    if(!(rawPressed&trigger))fireDrainMask.fetch_and(~bit,std::memory_order_acq_rel);
    const auto accept=((activeMask.load(std::memory_order_acquire)|drainMask.load(std::memory_order_acquire))&bit)?button:0;
    return accept|(fireBlocked(left)?trigger:0);
}
}
