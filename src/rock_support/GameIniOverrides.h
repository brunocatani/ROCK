#pragma once

namespace rock
{
    struct RockConfigValues;
}

namespace rock::game_ini_overrides
{
    // Install once on GameLoaded; setting objects and the hook live until exit.
    [[nodiscard]] bool install(const RockConfigValues& config);
    // Main-thread config publication and detection of writes that bypass native setters.
    void update(const RockConfigValues& config);
}
