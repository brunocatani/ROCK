#pragma once

namespace rock::native_collision_filter_safety
{
    /*
     * Installs the exact FO4VR collision-filter fault boundary. Only the
     * verified null-derived hknpWorld body-array read is suppressed.
     */
    [[nodiscard]] bool install() noexcept;
}
