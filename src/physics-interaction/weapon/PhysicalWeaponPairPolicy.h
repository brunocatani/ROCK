#pragma once

namespace rock::physical_weapon_pair_policy
{
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
