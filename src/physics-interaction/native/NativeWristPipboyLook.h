#pragma once

namespace rock::native_wrist_pipboy_look
{
    /*
     * The game's wrist Pip-Boy follows the player's gaze. Each frame
     * PlayerCharacter::Update (140EF19F0) calls 140EF67B0, which turns the
     * angles between the HMD and the left wand into a look factor through the
     * f{HMDTo}PipboyScale{Inner,Outer}Angle:VRPipboy settings. Above 0.01 the
     * factor plays the UIPipBoyHumLP hum; from 0.98 the Pip-Boy counts as
     * looked at (140EF8D80). Looked at, ActivateHandler ignores the Pip-Boy
     * hand's activate, and with bPipboyDisableWhenNotLooking:VRUI (default
     * on) 140EF6EC0 opens PipboyMenu, closing it again on a look away.
     *
     * FRIK opens its wrist Pip-Boy through that path: it zeroes the four
     * angles (every gaze counts as a look) and restores 20/5 degrees to
     * close. A closed FRIK Pip-Boy therefore still hums, blocks the off-hand
     * activate and opens and closes itself when the player looks at the
     * wrist, whatever FRIK's own look-to-open settings say.
     *
     * While FRIK manages the wrist Pip-Boy and has it closed, ROCK runs that
     * update with angles no gaze satisfies, so the native code itself reports
     * no look: the hum fades out and nothing opens. FRIK's open state, angles
     * forced open (FRIK's power-armor exit workaround opens with them without
     * marking itself open), projected and HMD modes, power armor and a
     * blocked FRIK Pip-Boy feature keep the native update unchanged.
     */
    [[nodiscard]] bool install() noexcept;
}
