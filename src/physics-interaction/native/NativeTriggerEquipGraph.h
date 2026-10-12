#pragma once

namespace RE { class PlayerCharacter; }

namespace rock::native_trigger_equip_graph
{
    enum class Result { Ready, Pending, Unavailable };
    void install() noexcept;
    [[nodiscard]] bool installed() noexcept;
    // Game thread, after native item identity is committed and before draw.
    // Native AIProcess retains and releases all current/pending graph handles.
    // staged (optional) reports whether E8D090 staged a new handle set.
    [[nodiscard]] Result prepare(RE::PlayerCharacter* player, bool* staged = nullptr) noexcept;
}
