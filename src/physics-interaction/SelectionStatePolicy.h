#pragma once

#include "HandState.h"

namespace frik::rock::selection_state_policy
{
    inline HandState stateForSelection(bool isFarSelection) { return isFarSelection ? HandState::SelectedFar : HandState::SelectedClose; }

    inline bool canUpdateSelectionFromState(HandState state) { return state == HandState::Idle || state == HandState::SelectedClose || state == HandState::SelectedFar; }

    inline bool canProcessSelectedState(HandState state) { return state == HandState::SelectedClose || state == HandState::SelectedFar; }
}
