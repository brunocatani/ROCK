#include "physics-interaction/native/player/NativeHolotapePlayback.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/NativeCodeBytes.h"

#include <REL/Relocation.h>
#include "RE/Bethesda/PipboyManager.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"

#include <array>
#include <cstdint>

namespace rock::native_holotape_playback
{
    namespace
    {
        // FO4VR 1.2.72 entries; the prologue of each must be native or a
        // detour before ROCK calls it.
        constexpr std::uintptr_t kPlayHolotape = 0xF27E50;
        constexpr std::uintptr_t kStopHolotape = 0xF280F0;
        constexpr std::uintptr_t kPlayPipboyLoadHolotapeAnim = 0xC32D50;
        constexpr std::array<std::uint8_t, 16> kPlayHolotapePrologue{ 0x48, 0x89, 0x5C, 0x24, 0x08, 0x57, 0x48, 0x83,
            0xEC, 0x30, 0x48, 0x8B, 0xDA, 0x48, 0x8B, 0xF9 };
        constexpr std::array<std::uint8_t, 16> kStopHolotapePrologue{ 0x48, 0x89, 0x5C, 0x24, 0x10, 0x57, 0x48, 0x83,
            0xEC, 0x20, 0x48, 0x8B, 0xDA, 0x48, 0x8B, 0xF9 };
        constexpr std::array<std::uint8_t, 10> kPlayPipboyLoadHolotapeAnimPrologue{ 0x48, 0x89, 0x74, 0x24, 0x10, 0x57,
            0x48, 0x83, 0xEC, 0x20 };

        using PlayerHolotapeFn = void (*)(RE::PlayerCharacter*, RE::BGSNote*);
        using LoadHolotapeAnimFn = void (*)(RE::PipboyManager*, RE::BGSNote*, bool);

        PlayerHolotapeFn s_playHolotape = nullptr;
        PlayerHolotapeFn s_stopHolotape = nullptr;
        LoadHolotapeAnimFn s_loadHolotapeAnim = nullptr;
    }

    bool install() noexcept
    {
        if (!native_code::entryIsNativeOrDetoured(kPlayHolotape, kPlayHolotapePrologue) ||
            !native_code::entryIsNativeOrDetoured(kStopHolotape, kStopHolotapePrologue) ||
            !native_code::entryIsNativeOrDetoured(kPlayPipboyLoadHolotapeAnim, kPlayPipboyLoadHolotapeAnimPrologue)) {
            ROCK_LOG_ERROR(Init, "Holotape playback validation failed; trigger on a held holotape only stashes it");
            return false;
        }
        s_playHolotape = reinterpret_cast<PlayerHolotapeFn>(REL::Offset(kPlayHolotape).address());
        s_stopHolotape = reinterpret_cast<PlayerHolotapeFn>(REL::Offset(kStopHolotape).address());
        s_loadHolotapeAnim = reinterpret_cast<LoadHolotapeAnimFn>(REL::Offset(kPlayPipboyLoadHolotapeAnim).address());
        return true;
    }

    bool isHolotape(const RE::TESBoundObject* baseForm) noexcept
    {
        return baseForm && baseForm->Is(RE::ENUM_FORM_ID::kNOTE);
    }

    PlayResult play(RE::TESBoundObject* baseForm) noexcept
    {
        if (!s_playHolotape) {
            return PlayResult::Unavailable;
        }
        if (!isHolotape(baseForm)) {
            return PlayResult::NotHolotape;
        }
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player) {
            return PlayResult::MissingPlayer;
        }
        auto* note = static_cast<RE::BGSNote*>(baseForm);
        const bool shownInPipboy =
            note->type == RE::BGSNote::NOTE_TYPE::kProgram || note->type == RE::BGSNote::NOTE_TYPE::kTerminal;
        auto* pipboyManager = shownInPipboy ? RE::PipboyManager::GetSingleton() : nullptr;
        if (shownInPipboy && !pipboyManager) {
            return PlayResult::MissingPipboyManager;
        }

        s_stopHolotape(player, nullptr);
        player->currHolotape = note;
        if (shownInPipboy) {
            s_loadHolotapeAnim(pipboyManager, note, false);
        } else {
            s_playHolotape(player, nullptr);
        }
        return PlayResult::Started;
    }

    const char* playResultName(PlayResult result) noexcept
    {
        switch (result) {
        case PlayResult::Started:
            return "started";
        case PlayResult::Unavailable:
            return "unavailable";
        case PlayResult::NotHolotape:
            return "not-holotape";
        case PlayResult::MissingPlayer:
            return "missing-player";
        case PlayResult::MissingPipboyManager:
            return "missing-pipboy-manager";
        }
        return "unknown";
    }
}
