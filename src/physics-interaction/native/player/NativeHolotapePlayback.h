#pragma once

#include <cstdint>

namespace RE
{
    class TESBoundObject;
}

namespace rock::native_holotape_playback
{
    /*
     * Starts a holotape the way FO4VR does when a wand activates a holotape
     * in the world (140F283E0): PlayerCharacter::StopHolotape(nullptr) ends
     * the current tape, PlayerCharacter::currHolotape (+0xEB0) takes the new
     * one, then voice and scene tapes play through
     * PlayerCharacter::PlayHolotape(nullptr) and program and terminal tapes
     * (BGSNote::type 2 and 3, +0x100) go through
     * PipboyManager::PlayPipboyLoadHolotapeAnim, which shows them in the
     * Pip-Boy. ROCK calls this after the held tape has reached the inventory.
     */
    enum class PlayResult : std::uint8_t
    {
        Started = 0,
        Unavailable,
        NotHolotape,
        MissingPlayer,
        MissingPipboyManager,
    };

    // Load time: checks the entry of each native function called.
    [[nodiscard]] bool install() noexcept;

    [[nodiscard]] bool isHolotape(const RE::TESBoundObject* baseForm) noexcept;

    // Game thread.
    [[nodiscard]] PlayResult play(RE::TESBoundObject* baseForm) noexcept;

    [[nodiscard]] const char* playResultName(PlayResult result) noexcept;
}
