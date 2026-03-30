#pragma once

namespace rock::debug_controller_runtime
{
    void update(bool gameplayInputAllowed, float deltaSeconds);
    bool isPivotTuningActive();
    bool isSelectedPivotHandLeft();
}
