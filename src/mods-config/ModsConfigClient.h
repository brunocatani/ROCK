#pragma once

/*
 * ROCK's client of Mods Config FO4 VR's native API. ModsConfigFO4Api.h beside it is
 * Mods Config's own client header, copied as is, as Mods Config asks of every mod:
 * refresh it from Mods Config's src/api/ when its API grows, never edit it.
 *
 * The client connects under "ROCK", the modName of ROCK's config spec
 * (data/mod/ModsConfigFO4/Config/ROCK/spec.json). It gives the page's Setup Wizard
 * button its function, openSetupWizard: the menu steps aside (minimized, its edits
 * kept) while the wizard is up, and comes back with ROCK's values read again once it
 * closes. Without Mods Config nothing is called.
 *
 * Live preview: each change of a `live` setting becomes a preview value in
 * RockConfig, applied at the next FrameBegin without writing a file; its
 * Discard, a Save or the menu closing ends it. The modVersion state gives the
 * page its version line.
 *
 * While the menu is open and shown, its pointer reads the primary (right) trigger,
 * so the input remap's left-hand firing stops presenting the left trigger there.
 *
 * While the ray is on Grip Zone Marker Size or Firing Grip Reach (by their spec ids),
 * pointedPreview names the row, and ROCK draws it on the drawn weapon.
 *
 * Threads: Mods Config calls on the game's main thread, which is also ROCK's frame
 * thread; connect and update run there too.
 */
#include <cstdint>

namespace rock::mods_config_client
{
    // The row of ROCK's page the ray is on that ROCK shows in the world.
    enum class PointedPreview : std::uint8_t
    {
        None,
        GripZoneMarkerSize,
        FiringGripReach,
    };

    // Once ROCK has initialized: adds ROCK's entries and connects.
    void connect() noexcept;

    void update() noexcept;

    // Frame thread: the row the ray was on at this frame's FrameBegin.
    [[nodiscard]] PointedPreview pointedPreview() noexcept;
}
