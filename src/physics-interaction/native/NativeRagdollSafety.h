#pragma once

namespace rock::native_ragdoll_safety
{
    /*
     * Installs the exact FO4VR ragdoll-update fault boundary. Only the verified
     * null-derived constraint-array teardown read is suppressed.
     */
    [[nodiscard]] bool install() noexcept;
}
