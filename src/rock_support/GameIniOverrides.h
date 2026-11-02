// Game INI settings ROCK requires: forced to ROCK's values and kept there.
#pragma once

namespace rock::game_ini_overrides
{
    // Install once on GameLoaded; setting objects and the hook live until exit.
    [[nodiscard]] bool install();
    // Main-thread enforcement and detection of writes that bypass native setters.
    void update();
}
