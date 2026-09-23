#pragma once

namespace rock::world_weapon_material_visibility
{
    // Install before the initial world attaches. Call update/suspend/reset only
    // on the game thread; native event callbacks enqueue copied IDs only.
    void install();
    void update(bool canCull);
    void suspend();
    void reset();
}
