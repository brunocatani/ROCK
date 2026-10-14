#include "physics-interaction/input/SupportTriggerInput.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "api/ProviderRuntimeServices.h"
#include <atomic>
#include <mutex>

namespace rock::support_trigger_input {
namespace {
    // Mutable lease state belongs to the ROCK frame thread. The mutex also
    // serializes provider shutdown; controller hooks consume only these masks.
    std::mutex mutex;
    api::input::SupportTriggerRequest request;
    api::input::SupportTriggerState state;
    std::atomic_uint32_t activeMask{}, drainMask{};
    bool armed{};
    constexpr std::uint32_t bit(api::Hand hand) { return hand == api::Hand::Left ? 1u : 2u; }
    void retire() {
        drainMask.fetch_or(activeMask.exchange(0, std::memory_order_acq_rel), std::memory_order_release);
        state.active = 0; state.owner = 0; state.held = 0; state.hand = api::Hand::None; state.weaponSlot = 0; armed = false;
    }
    bool current(api::OwnerToken owner, const api::input::SupportTriggerRequest& value) {
        provider::RockProviderEquippedWeaponStateV1 equipped;
        provider::RockProviderEquippedWeaponGripStateV1 grip;
        const auto sample = provider::runtime::sample();
        constexpr auto drawn = static_cast<std::uint32_t>(provider::RockProviderEquippedWeaponStateFlagV1::WeaponDrawn);
        constexpr auto valid = static_cast<std::uint32_t>(provider::RockProviderEquippedWeaponStateFlagV1::Valid);
        using G = provider::RockProviderEquippedWeaponGripStateFlagV1;
        if (provider::runtime::apiGetEquippedWeaponStateV1(owner, &equipped) != provider::RockProviderResultV1::Ok ||
            (equipped.flags & (drawn | valid)) != (drawn | valid) ||
            !provider::runtime::apiGetEquippedWeaponGripStateV1(owner, &grip)) return false;
        const auto required = static_cast<std::uint32_t>(G::Valid) | static_cast<std::uint32_t>(G::TwoHandGripActive);
        const auto support = (grip.flags & static_cast<std::uint32_t>(G::FiringHandLeft)) ? api::Hand::Right : api::Hand::Left;
        return (grip.flags & required) == required && support == value.hand &&
            equipped.weaponFormId == value.weaponFormId && equipped.weaponGenerationKey == value.weaponGenerationKey &&
            grip.weaponGenerationKey == value.weaponGenerationKey &&
            sample.worldGeneration == value.sample.worldGeneration && sample.skeletonGeneration == value.sample.skeletonGeneration &&
            sample.providerGeneration == value.sample.providerGeneration;
    }
}
api::Status set(api::OwnerToken owner, const api::input::SupportTriggerRequest& value) {
    using api::Status;
    if (!owner || !value.weaponFormId || !value.weaponGenerationKey || !value.leaseFrames || value.leaseFrames > api::input::kMaxWeaponInputLeaseFrames ||
        (value.hand != api::Hand::Left && value.hand != api::Hand::Right)) return Status::InvalidArgument;
    const auto frame = provider::runtime::currentGameFrameIndex();
    if (value.sample.frameIndex > frame || frame - value.sample.frameIndex > 1) return Status::GenerationMismatch;
    std::scoped_lock lock(mutex);
    if (state.active && state.owner != owner) return Status::Busy;
    if (!current(owner, value)) return Status::GenerationMismatch;
    if (!state.active || state.hand != value.hand || request.weaponGenerationKey != value.weaponGenerationKey) {
        retire();
        const auto raw = input_remap_runtime::peekRawButtonState(value.hand == api::Hand::Left, 33);
        armed = raw.available && !raw.held;
        state.held = raw.available && raw.held;
    }
    request = value; state.owner = owner; state.hand = value.hand; state.weaponSlot = value.weaponSlot; state.active = 1;
    state.expiresAfterFrame = frame + value.leaseFrames; state.sample = provider::runtime::sample();
    activeMask.store(bit(value.hand), std::memory_order_release);
    return Status::Ok;
}
api::Status clear(api::OwnerToken owner) {
    std::scoped_lock lock(mutex);
    if (state.active && state.owner != owner) return api::Status::Busy;
    retire(); return api::Status::Ok;
}
api::Status read(api::input::SupportTriggerState& out) {
    std::scoped_lock lock(mutex); out = state; out.sample = provider::runtime::sample(); return api::Status::Ok;
}
void clearOwner(api::OwnerToken owner) { std::scoped_lock lock(mutex); if (state.owner == owner) retire(); }
void reset() { std::scoped_lock lock(mutex); retire(); }
void beginFrame(std::uint64_t frame) {
    std::scoped_lock lock(mutex);
    if (!state.active) return;
    if (frame > state.expiresAfterFrame || !current(state.owner, request)) { retire(); return; }
    const auto raw = input_remap_runtime::peekRawButtonState(state.hand == api::Hand::Left, 33);
    if (!raw.available) { armed = false; state.held = 0; return; }
    if (!armed) { if (!raw.held) armed = true; }
    else if (raw.held && !state.held) ++state.pressSequence;
    state.held = raw.held;
}
bool blocked(bool left) noexcept {
    return ((activeMask.load(std::memory_order_acquire) | drainMask.load(std::memory_order_acquire)) & (left ? 1u : 2u)) != 0;
}
std::uint64_t gameMask(bool left, std::uint64_t pressed) noexcept {
    constexpr auto trigger = 1ull << 33;
    if (!(pressed & trigger)) drainMask.fetch_and(~(left ? 1u : 2u), std::memory_order_acq_rel);
    return blocked(left) ? trigger : 0;
}
}
