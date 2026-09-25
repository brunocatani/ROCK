#pragma once

#include <array>
#include <cstdint>

namespace rock::physical_weapon_pair_policy
{
    struct CollisionOwner { std::uint32_t body{0x7FFFFFFFu}, hands{}; };
    // Native ownership is the initial view. Exact physical sessions override
    // that view per hand during conversion, including while a proxy is pending.
    constexpr std::array<CollisionOwner, 2> collisionOwners(CollisionOwner native,
        const std::array<CollisionOwner, 2>& physical) noexcept
    {
        std::array<CollisionOwner, 2> result{};
        for (unsigned hand = 0; hand < result.size(); ++hand) {
            const auto bit = 1u << hand;
            if (native.hands & bit) result[hand] = {native.body, bit};
            for (const auto source : physical) if (source.hands & bit) result[hand] = {source.body, bit};
        }
        return result;
    }
    enum class EntryAction { Wait, Cancel, RestoreOriginal, RestoreIncoming, Complete };
    struct EntryObservation
    {
        bool converted{}, incomingHeld{}, outgoingHeld{}, nativeOriginalPresent{}, placementPending{}, allReady{}, failed{};
    };
    constexpr EntryAction advance(EntryObservation state) noexcept
    {
        if (!state.incomingHeld) {
            if (!state.converted || state.nativeOriginalPresent) return EntryAction::Cancel;
            if (state.outgoingHeld) return EntryAction::RestoreOriginal;
            return state.placementPending ? EntryAction::Wait : EntryAction::Cancel;
        }
        if (state.failed && !state.converted) return EntryAction::Cancel;
        if (state.converted && !state.nativeOriginalPresent && !state.placementPending) {
            if (!state.outgoingHeld) return EntryAction::RestoreIncoming;
            if (state.failed) return EntryAction::RestoreOriginal;
        }
        return state.converted && !state.nativeOriginalPresent && !state.placementPending && state.allReady ?
            EntryAction::Complete : EntryAction::Wait;
    }
}
