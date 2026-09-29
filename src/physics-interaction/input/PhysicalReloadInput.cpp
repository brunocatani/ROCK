#include "physics-interaction/input/PhysicalReloadInput.h"
#include "api/ProviderRuntimeServices.h"
#include <atomic>
#include <mutex>

namespace rock::physical_reload_input {
namespace {
    using namespace api;
    std::mutex mutex;
    input::v1_2::ReloadInputRequest request;
    input::v1_2::ReloadInputState state;
    bool armed{};
    std::uint32_t drainingHands{};
    std::atomic<std::uint32_t> activeMask{},drainMask{};
    constexpr std::uint32_t handMask(Hand h){return h==Hand::Left?1u:2u;}
    void retire() {
        // Unavailable input also needs drainage: menu/UI capture hides the
        // physical held level, and must not expose that hold when a lease ends.
        if(state.active&&(state.held||!armed)) {
            drainingHands|=handMask(state.hand);
            drainMask.fetch_or(handMask(state.hand),std::memory_order_release);
        }
        activeMask.store(0,std::memory_order_release);state.active=0;state.owner=0;state.held=0;armed=false;
    }
    bool current() {
        provider::RockProviderEquippedWeaponStateV1 weapon;
        const auto result=provider::runtime::apiGetEquippedWeaponStateV1(state.owner,&weapon);
        const auto sample=provider::runtime::sample();
        constexpr auto drawn=static_cast<std::uint32_t>(api::weapon::EquippedWeaponStateFlagV1::WeaponDrawn);
        constexpr auto valid=static_cast<std::uint32_t>(api::weapon::EquippedWeaponStateFlagV1::Valid);
        return result==provider::RockProviderResultV1::Ok&&(weapon.flags&(drawn|valid))==(drawn|valid)&&weapon.weaponFormId==request.weaponFormId&&
            weapon.weaponGenerationKey==request.weaponGenerationKey&&sample.worldGeneration==request.sample.worldGeneration&&
            sample.skeletonGeneration==request.sample.skeletonGeneration&&sample.providerGeneration==request.sample.providerGeneration;
    }
}
api::Status set(api::OwnerToken owner,const api::input::v1_2::ReloadInputRequest& value) {
    using api::Status;using api::Hand;
    if(!owner||!value.weaponFormId||!value.weaponGenerationKey||!value.leaseFrames||value.leaseFrames>api::input::v1_2::kMaximumLeaseFrames||
        (value.hand!=Hand::Left&&value.hand!=Hand::Right))return Status::InvalidArgument;
    const auto now=provider::runtime::currentGameFrameIndex();
    if(value.sample.frameIndex>now||now-value.sample.frameIndex>1)return Status::GenerationMismatch;
    std::scoped_lock lock(mutex);
    if(state.active&&state.owner!=owner)return Status::Busy;
    const bool continuing=state.active&&state.owner==owner&&request.weaponGenerationKey==value.weaponGenerationKey&&state.hand==value.hand;
    const auto oldRequest=request;const auto oldState=state;
    request=value;state.owner=owner;
    if(!current()){request=oldRequest;state=oldState;return Status::GenerationMismatch;}
    if(!continuing) {
        retire();state.owner=owner;state.hand=value.hand;
        const auto raw=input_remap_runtime::peekRawButtonState(value.hand==Hand::Left,7);
        armed=raw.available&&!raw.held;state.held=raw.held;
    }
    state.active=1;state.sample=provider::runtime::sample();state.expiresAfterFrame=now+value.leaseFrames;
    activeMask.store(handMask(value.hand),std::memory_order_release);
    return Status::Ok;
}
api::Status clear(api::OwnerToken owner) {
    std::scoped_lock lock(mutex);
    if(state.active&&state.owner!=owner)return api::Status::Busy;
    retire();return api::Status::Ok;
}
api::Status read(api::input::v1_2::ReloadInputState& out) {
    std::scoped_lock lock(mutex);out=state;out.sample=provider::runtime::sample();return api::Status::Ok;
}
void clearOwner(api::OwnerToken owner){std::scoped_lock lock(mutex);if(state.owner==owner)retire();}
void reset(){std::scoped_lock lock(mutex);retire();}
void beginFrame(std::uint64_t frame) {
    std::scoped_lock lock(mutex);
    if(state.active&&(frame>state.expiresAfterFrame||!current()))retire();
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
    if(!raw.available){armed=false;return true;}
    if(!armed){if(!raw.held)armed=true;return true;}
    if(raw.pressed)++state.pressSequence;
    if(raw.released)++state.releaseSequence;
    return true;
}
std::uint64_t gameMask(bool left,std::uint64_t rawPressed) noexcept {
    constexpr std::uint64_t button=1ull<<7;const auto bit=left?1u:2u;
    if(!(rawPressed&button))drainMask.fetch_and(~bit,std::memory_order_acq_rel);
    return ((activeMask.load(std::memory_order_acquire)|drainMask.load(std::memory_order_acquire))&bit)?button:0;
}
}
