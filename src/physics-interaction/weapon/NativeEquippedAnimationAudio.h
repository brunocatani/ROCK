#pragma once

#include "RE/Bethesda/BSFixedString.h"
#include "RE/Bethesda/BSSoundHandle.h"
#include <array>
#include <cstdint>

namespace RE { class Actor; class NiAVObject; }

namespace rock::native_equipped_animation_audio
{
    // One equipped graph's native sound handles. The primary actor keeps its
    // own event sink; equal sound tags on two guns must never share a handle.
    class Channel
    {
        struct Voice {
            RE::BSFixedString payload;
            RE::BSSoundHandle handle{UINT32_MAX,false,0};
            std::uint64_t frame{};
            bool looping{};
        };
        std::array<Voice,64> _voices;
    public:
        Channel()=default;
        Channel(const Channel&)=delete;
        Channel& operator=(const Channel&)=delete;
        static bool validate() noexcept;
        void stop() noexcept;
        bool event(RE::Actor* player, RE::NiAVObject* model, const RE::BSFixedString& tag,
            const RE::BSFixedString& payload, std::uint64_t frame);
    };
}
