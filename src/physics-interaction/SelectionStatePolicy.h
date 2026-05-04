#pragma once

#include "HandInteractionStateMachine.h"

namespace frik::rock::selection_state_policy
{
    inline HandState stateForSelection(bool isFarSelection) { return isFarSelection ? HandState::SelectedFar : HandState::SelectedClose; }

    inline bool canUpdateSelectionFromState(HandState state) { return frik::rock::canUpdateSelectionFromState(state); }

    inline bool canProcessSelectedState(HandState state) { return frik::rock::canProcessSelectedState(state); }

    inline bool hasExclusiveObjectSelection(HandState state) { return frik::rock::hasExclusiveObjectSelection(state); }
}
