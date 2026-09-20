#pragma once

#include "physics-interaction/weapon/EquippedWeaponTransitionPolicy.h"

#include <cstdint>

namespace rock::held_weapon_transfer
{
    enum class Role : std::uint8_t { Firing, Support, Paired };
    enum class Phase : std::uint8_t { Idle, AwaitEquip, AwaitGrip, Completing, Recovering, Terminal };
    enum class Outcome : std::uint8_t { None, Completed, Cancelled, Failed };

    struct Request
    {
        std::uint32_t reference{ 0 };
        std::uint64_t grab{ 0 };
        std::uint32_t world{ 0 };
        std::uint32_t skeleton{ 0 };
        bool isLeft{ false };
        Role role{ Role::Firing };
        bool retainOutgoing{ false };
        std::uint32_t previousForm{ 0 };
        std::uintptr_t previousInstance{ 0 };
    };

    // Main-thread logical ownership. Native placement and grip resources stay
    // with their existing owners and acknowledge this sequence exactly once.
    struct State
    {
        std::uint64_t sequence{ 0 };
        Request request{};
        Phase phase{ Phase::Idle };
        Outcome outcome{ Outcome::None };
        std::uint32_t targetForm{ 0 };
        std::uintptr_t targetInstance{ 0 };
        std::uintptr_t observedInstance{ 0 };
        bool identityBound{ false };
        std::uint32_t outgoingReference{ 0 };
        bool outgoingPending{ false };
        bool outgoingRemoved{ false };
        bool inventoryCommitted{ false };
        bool gripAcquired{ false };
        bool presentationAcquired{ false };

        [[nodiscard]] constexpr bool active() const noexcept
        {
            return phase != Phase::Idle && phase != Phase::Terminal;
        }
        [[nodiscard]] constexpr bool wantsEquip(bool left, std::uint32_t ref, std::uint64_t grab) const noexcept
        {
            return phase == Phase::AwaitEquip && request.isLeft == left && request.reference == ref && request.grab == grab;
        }
        [[nodiscard]] constexpr bool matchesTarget(std::uint32_t form, std::uintptr_t instance) const noexcept
        {
            if (identityBound) return targetForm != 0 && form == targetForm && instance == observedInstance;
            if (form == request.previousForm && instance == request.previousInstance) return false;
            return targetForm != 0 && equipped_weapon_transition_policy::matchesExpectedIdentity(
                form, instance, targetForm, targetInstance, request.previousForm, request.previousInstance);
        }
        [[nodiscard]] constexpr bool blocksFire() const noexcept
        {
            return phase == Phase::AwaitEquip || phase == Phase::AwaitGrip || phase == Phase::Recovering;
        }
        [[nodiscard]] constexpr bool ownsEmptySlot(std::uint32_t form) const noexcept
        {
            return phase == Phase::AwaitEquip && request.retainOutgoing && outgoingRemoved && form == 0;
        }
    };

    [[nodiscard]] inline constexpr bool sameMenuItem(std::uint32_t savedForm, std::uintptr_t savedInstance,
        std::uint32_t currentForm, std::uintptr_t currentInstance) noexcept
    {
        return savedForm != 0 && savedForm == currentForm && savedInstance == currentInstance;
    }

    [[nodiscard]] inline constexpr bool sourceCurrent(const State& state, bool held, std::uint32_t reference,
        std::uint64_t grab, std::uint32_t world, std::uint32_t skeleton) noexcept
    {
        return state.request.world == world && state.request.skeleton == skeleton &&
            (state.phase != Phase::AwaitEquip || (held && state.request.reference == reference && state.request.grab == grab));
    }

    [[nodiscard]] inline constexpr bool equippedSourceCurrent(const State& state, std::uint32_t form, std::uintptr_t instance) noexcept
    {
        if (state.phase != Phase::AwaitEquip) return true;
        return state.outgoingRemoved ? form == 0 : form == state.request.previousForm && instance == state.request.previousInstance;
    }

    inline constexpr bool admit(State& state, const Request& request) noexcept
    {
        if (state.active() || !request.reference || !request.grab) return false;
        const auto sequence = state.sequence + 1;
        state = State{ .sequence = sequence ? sequence : 1, .request = request, .phase = Phase::AwaitEquip };
        return true;
    }

    inline constexpr bool resumeMenu(State& state, std::uint32_t form, std::uintptr_t instance,
        const Request& destination) noexcept
    {
        if (!form) return false;
        if (state.active()) {
            if (!state.inventoryCommitted || !state.matchesTarget(form, instance)) return false;
            state.phase = Phase::AwaitGrip;
            state.gripAcquired = false;
            state.presentationAcquired = false;
        } else {
            const auto sequence = state.sequence + 1;
            state = { .sequence = sequence ? sequence : 1, .request = destination,
                .phase = Phase::AwaitGrip, .targetForm = form, .targetInstance = instance,
                .observedInstance = instance, .identityBound = true, .inventoryCommitted = true };
        }
        return true;
    }

    inline constexpr void finishIfReady(State& state) noexcept
    {
        if (state.phase != Phase::Completing || state.outgoingPending) return;
        if (state.outcome == Outcome::Completed && (!state.gripAcquired || !state.presentationAcquired)) return;
        state.phase = Phase::Terminal;
    }

    inline constexpr bool outgoingRemoved(State& state, std::uint32_t reference) noexcept
    {
        if (state.phase != Phase::AwaitEquip || !state.request.retainOutgoing || state.outgoingRemoved) return false;
        state.outgoingRemoved = true;
        state.outgoingPending = true;
        state.outgoingReference = reference;
        return true;
    }

    inline constexpr bool inventoryCommitted(State& state, std::uint32_t form, std::uintptr_t instance) noexcept
    {
        if (state.phase != Phase::AwaitEquip || !form) return false;
        state.inventoryCommitted = true;
        state.targetForm = form;
        state.targetInstance = instance;
        state.phase = Phase::AwaitGrip;
        return true;
    }

    inline constexpr void cancel(State& state, Outcome outcome = Outcome::Failed) noexcept
    {
        if (!state.active()) return;
        state.outcome = outcome;
        state.phase = state.inventoryCommitted ? Phase::Recovering : Phase::Completing;
        finishIfReady(state);
    }

    inline constexpr bool outgoingFinished(State& state, std::uint64_t sequence, bool succeeded) noexcept
    {
        if (!state.active() || sequence != state.sequence || !state.outgoingPending) return false;
        state.outgoingPending = false;
        // An outgoing rollback must not undo a successfully acquired incoming
        // weapon. The adapter reports its exact disposition independently.
        if (!succeeded && state.outcome != Outcome::Cancelled) state.outcome = Outcome::Failed;
        finishIfReady(state);
        return true;
    }

    inline constexpr bool acquireGrip(State& state, std::uint32_t form, std::uintptr_t instance, bool left, Role role) noexcept
    {
        if (state.phase != Phase::AwaitGrip || !state.matchesTarget(form, instance) ||
            left != state.request.isLeft || role != state.request.role) return false;
        state.gripAcquired = true;
        state.observedInstance = instance;
        state.identityBound = true;
        state.phase = Phase::Completing;
        if (state.outcome == Outcome::None) state.outcome = Outcome::Completed;
        return true;
    }

    inline constexpr void presentationAcquired(State& state) noexcept
    {
        if (!state.active() || !state.gripAcquired) return;
        state.presentationAcquired = true;
        finishIfReady(state);
    }

    inline constexpr void recovered(State& state) noexcept
    {
        if (state.phase != Phase::Recovering) return;
        state.inventoryCommitted = false;
        state.phase = Phase::Completing;
        finishIfReady(state);
    }
}
