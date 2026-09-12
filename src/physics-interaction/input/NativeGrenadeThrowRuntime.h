#pragma once

namespace RE { class ButtonEvent; }

namespace rock::native_grenade_throw_runtime
{
    // Game/input dispatch thread only. No engine pointers survive a call.
    [[nodiscard]] float holdSeconds();
    [[nodiscard]] bool active();
    bool begin(RE::ButtonEvent& source);
    void release(RE::ButtonEvent& source);
    void cancel();
}
