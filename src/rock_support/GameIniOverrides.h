#pragma once

namespace rock::game_ini_overrides
{
    // Install once on GameLoaded; setting objects and the hook live until exit.
    [[nodiscard]] bool install();
    // Main-thread enforcement and detection of writes that bypass native setters.
    void update();
}
