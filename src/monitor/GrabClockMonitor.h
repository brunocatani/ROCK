#pragma once

namespace rock::provider
{
    struct RockProviderFrameSnapshot;
}

namespace rock::monitor
{
    /*
     * Embedded ROCK Monitor: a non-interactive PrismaUI WorldQuad dashboard
     * that renders the grab locomotion stutter channels (room/hand/held/body
     * clocks and the GrabClockDebugFeed stage samples) in real time, straight
     * from ROCK's internals with no provider-API hop. Presentation follows
     * ROCK Prober: right-hand-anchored panel, [PanelPose] INI at
     * My Games/Fallout4VR/ROCK_Config/ROCKMonitor.ini, hot-reloaded.
     */

    // Called once from the GameLoaded handler: loads the Monitor INI, acquires
    // the PrismaUI_F4 interfaces, and requests the panel view. Safe to call
    // when PrismaUI is absent; the monitor then stays dormant.
    void initialize();

    // Called on PostLoadGame/NewGame to re-request the view if needed.
    void onGameSessionReady();

    // Called at the tail of provider frame dispatch on the game thread with
    // the fully built frame snapshot. Samples engine nodes and the grab clock
    // feed, then schedules a coalesced UI push on the game task queue.
    void onProviderFrame(const ::rock::provider::RockProviderFrameSnapshot& snapshot);
}
