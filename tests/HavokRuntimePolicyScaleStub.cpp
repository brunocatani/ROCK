#include "physics-interaction/native/PhysicsScale.h"

namespace rock::physics_scale
{
    /*
     * HavokRuntime policy tests compile the runtime facade without loading the
     * plugin PCH or FO4VR runtime globals. The production DLL links the real
     * PhysicsScale.cpp; this narrow stub only satisfies transform helper symbols
     * so the policy executable can validate body-id safety rules without touching
     * process-relative relocation state.
     */
    float havokToGame()
    {
        return kFallbackHavokToGame;
    }
}
