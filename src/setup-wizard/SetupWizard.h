#pragma once

/*
 * ROCK's setup wizard: an RPS UI panel where the player picks the button and
 * gesture of each function and goes through ROCK's main options, one page
 * each. Nothing is written until the player saves on the review page; saving
 * goes through ROCK's settings store like any other change, so the bindings
 * and options apply at once.
 *
 * It opens from RobCo PALM's wheel (ROCK's "ROCK" section), from PALM's
 * Config, and from Configuration::openSetupWizard. While open it captures the
 * controllers through RPS UI, so the game and ROCK's gameplay ignore them, and
 * the function bindings stand aside so a captured button is the button pressed.
 *
 * Threads: requestOpen from any thread; update and onSessionLoaded on the frame
 * thread; RPS UI calls the panel callbacks on its render thread and the input
 * callback on the game thread. One mutex guards the wizard's state.
 */
namespace rock::setup_wizard
{
    // Asks for the wizard on the next frame. False when RPS UI Framework is not loaded.
    bool requestOpen() noexcept;

    // Every frame: connects to RPS UI and PALM when needed, opens a requested
    // wizard, saves or closes it.
    void update() noexcept;

    // A game load or new game: closes the wizard and republishes the wheel entry.
    void onSessionLoaded() noexcept;
}
