#pragma once

/*
 * ROCK's setup wizard: an RPS UI panel where the player picks the button and
 * gesture of each function and goes through ROCK's main options, one page
 * each. Nothing is written until the player saves on the review page; saving
 * goes through ROCK's settings store like any other change, so the bindings
 * and options apply at once.
 *
 * It opens from PALM's Config (the ROCK page) and from
 * Configuration::openSetupWizard, and by itself once per game start when this ROCK
 * brings a newer edition of it ([SetupWizard] ShownEdition) or a SteamVR binding to
 * offer, 10 seconds after the game is free (AutoOpenGate.h). While open it captures the
 * controllers through RPS UI, so the game and ROCK's gameplay ignore them, and
 * the function bindings stand aside so a captured button is the button pressed.
 *
 * This file is the game's host: RPS UI binding, placement, the automatic
 * opening and SteamVR. The pages themselves are SetupWizardPanel.h, which the
 * desktop preview (preview/SetupWizardPreview.cpp) runs unchanged.
 *
 * Threads: requestOpen from any thread; update and onSessionLoaded on the frame
 * thread; RPS UI calls the panel callbacks on its render thread and the input
 * callback on the game thread. The panel guards its own state.
 */
namespace rock::setup_wizard
{
    // Asks for the wizard on the next frame. False when RPS UI Framework is not loaded.
    bool requestOpen() noexcept;

    // Every frame: connects to RPS UI when needed, opens a requested
    // wizard, saves or closes it.
    void update() noexcept;

    // Any thread, from the game's load message: whether the session starting is a new game.
    void noteSessionStart(bool newGame) noexcept;

    // A game load or new game: closes the wizard and, once per game start, arms its
    // automatic opening (after the mirror on a new game).
    void onSessionLoaded() noexcept;
}
