#pragma once

namespace rock::see_through_scopes
{
    [[nodiscard]] bool installLateCullingHook();

    void refreshRuntimeState();
    void resetRuntimeState();

    void updateFrame();
    void updateLateCulling();
}
