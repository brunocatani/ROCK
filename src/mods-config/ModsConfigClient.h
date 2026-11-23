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
 * Threads: Mods Config calls on the game's main thread, which is also ROCK's frame
 * thread; connect and update run there too.
 */
namespace rock::mods_config_client
{
    // Once ROCK has initialized: adds ROCK's entries and connects.
    void connect() noexcept;

    // Every frame, after the setup wizard's update: restores the menu once the wizard it stepped aside for is gone.
    void update() noexcept;
}
