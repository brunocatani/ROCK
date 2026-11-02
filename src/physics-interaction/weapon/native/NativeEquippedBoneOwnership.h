#pragma once

namespace rock::native_equipped_bone_ownership
{
    // Protect the native primary graph's binding from a second equipped
    // model with the same bone names. Runs at native binding, not per frame.
    bool install() noexcept;
}
