#pragma once

namespace rock::native_shape_cast_safety
{
    /*
     * Installs the FO4VR-native world-cast context hook and the exact
     * collision-dispatch fail-closed boundary. The dispatcher guard remains
     * useful even if caller-context installation is unavailable.
     */
    [[nodiscard]] bool install() noexcept;
}
