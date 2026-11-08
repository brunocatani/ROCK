#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

/*
 * Per-hand controller button layout.
 *
 * ROCK's features, its API consumers (PALM, PAPER, SCISSORS) and the
 * game all read the five legacy OpenVR buttons in Quest/Touch layout:
 * trigger (Axis1, bit 33), grab (Grip, bit 2), lower face A/X (bit 7), upper
 * face B/Y (ApplicationMenu, bit 1) and stick click (Axis0, bit 32). The
 * layout chooses, per hand, which physical input drives each of those
 * logical buttons. ROCK applies it to every IVRSystem controller-state read
 * before its own capture, so every reader in the process sees one consistent
 * layout and no feature keeps a private button table. The default layout is
 * the identity and leaves each read byte-for-byte unchanged.
 *
 * Stick axes (Axis0 x/y) and the Index finger-curl axes (Axis3/Axis4) always
 * stay physical. Pad regions split the Axis0 click by the touch position at
 * press time (Vive/WMR trackpads have no face buttons); the region is latched
 * until release so a sliding thumb never moves a held press between buttons.
 */
namespace rock::button_layout_policy
{
    enum class Slot : std::uint8_t
    {
        Trigger,
        Grab,
        LowerFace,
        UpperFace,
        StickClick,
        Count,
    };
    inline constexpr std::size_t kSlotCount = static_cast<std::size_t>(Slot::Count);

    enum class Source : std::uint8_t
    {
        None,
        Trigger,
        Grip,
        LowerFace,
        UpperFace,
        Stick,
        GripClick,
        Axis3Click,
        Axis4Click,
        PadUp,
        PadDown,
        PadLeft,
        PadRight,
        PadCenter,
        Count,
    };

    inline constexpr int kUpperFaceBit = 1;
    inline constexpr int kGripBit = 2;
    inline constexpr int kLowerFaceBit = 7;
    inline constexpr int kStickBit = 32;
    inline constexpr int kTriggerBit = 33;
    inline constexpr int kGripClickBit = 34;
    inline constexpr int kAxis3ClickBit = 35;
    inline constexpr int kAxis4ClickBit = 36;
    inline constexpr std::size_t kStickAxis = 0;
    inline constexpr std::size_t kTriggerAxis = 1;
    inline constexpr std::size_t kGripAxis = 2;
    // Normalized pad radius below which a click is the pad centre.
    inline constexpr float kPadCenterRadius = 0.4f;

    [[nodiscard]] constexpr std::uint64_t bit(int index) noexcept { return std::uint64_t{ 1 } << index; }

    struct HandLayout
    {
        std::array<Source, kSlotCount> sources{ Source::Trigger, Source::Grip, Source::LowerFace, Source::UpperFace,
            Source::Stick };

        [[nodiscard]] constexpr Source source(Slot slot) const noexcept
        {
            return sources[static_cast<std::size_t>(slot)];
        }

        [[nodiscard]] constexpr bool identity() const noexcept { return *this == HandLayout{}; }

        [[nodiscard]] constexpr bool usesPadRegions() const noexcept
        {
            for (const auto value : sources) {
                if (value >= Source::PadUp && value <= Source::PadCenter)
                    return true;
            }
            return false;
        }

        [[nodiscard]] constexpr bool uses(Source value) const noexcept
        {
            for (const auto entry : sources) {
                if (entry == value)
                    return true;
            }
            return false;
        }

        constexpr bool operator==(const HandLayout&) const = default;
    };

    // Lossless packing for one atomic publication per hand (8 bits per slot).
    [[nodiscard]] constexpr std::uint64_t pack(const HandLayout& layout) noexcept
    {
        std::uint64_t packed = 0;
        for (std::size_t i = 0; i < kSlotCount; ++i) {
            packed |= static_cast<std::uint64_t>(layout.sources[i]) << (i * 8);
        }
        return packed;
    }

    [[nodiscard]] constexpr HandLayout unpack(std::uint64_t packed) noexcept
    {
        HandLayout layout{};
        for (std::size_t i = 0; i < kSlotCount; ++i) {
            const auto value = static_cast<std::uint8_t>((packed >> (i * 8)) & 0xFF);
            layout.sources[i] =
                value < static_cast<std::uint8_t>(Source::Count) ? static_cast<Source>(value) : Source::None;
        }
        return layout;
    }

    [[nodiscard]] constexpr Source defaultSource(Slot slot) noexcept { return HandLayout{}.source(slot); }

    [[nodiscard]] constexpr char lower(char c) noexcept
    {
        return c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : c;
    }

    // Case-insensitive; '_', '-' and spaces are ignored so "pad_up", "Pad Up" and "padup" match.
    [[nodiscard]] constexpr bool tokenEquals(std::string_view text, std::string_view token) noexcept
    {
        std::size_t t = 0;
        for (const char c : text) {
            if (c == '_' || c == '-' || c == ' ' || c == '\t')
                continue;
            if (t >= token.size() || lower(c) != token[t])
                return false;
            ++t;
        }
        return t == token.size();
    }

    [[nodiscard]] constexpr std::optional<Source> parseSource(std::string_view text) noexcept
    {
        struct Alias
        {
            std::string_view token;
            Source source;
        };

        constexpr Alias kAliases[] = {
            { "none", Source::None },
            { "off", Source::None },
            { "disabled", Source::None },
            { "trigger", Source::Trigger },
            { "grip", Source::Grip },
            { "grab", Source::Grip },
            { "a", Source::LowerFace },
            { "x", Source::LowerFace },
            { "b", Source::UpperFace },
            { "y", Source::UpperFace },
            { "menu", Source::UpperFace },
            { "applicationmenu", Source::UpperFace },
            { "thumbstick", Source::Stick },
            { "stick", Source::Stick },
            { "joystick", Source::Stick },
            { "touchpad", Source::Stick },
            { "trackpad", Source::Stick },
            { "gripclick", Source::GripClick },
            { "axis2", Source::GripClick },
            { "axis3", Source::Axis3Click },
            { "bumper", Source::Axis3Click },
            { "axis4", Source::Axis4Click },
            { "padup", Source::PadUp },
            { "paddown", Source::PadDown },
            { "padleft", Source::PadLeft },
            { "padright", Source::PadRight },
            { "padcenter", Source::PadCenter },
            { "padcentre", Source::PadCenter },
        };
        for (const auto& alias : kAliases) {
            if (tokenEquals(text, alias.token))
                return alias.source;
        }
        return std::nullopt;
    }

    // Canonical settings spelling. The face buttons use the labels of the hand they belong to.
    [[nodiscard]] constexpr std::string_view sourceName(Source source, bool isLeft) noexcept
    {
        switch (source) {
        case Source::None:
            return "none";
        case Source::Trigger:
            return "trigger";
        case Source::Grip:
            return "grip";
        case Source::LowerFace:
            return isLeft ? "x" : "a";
        case Source::UpperFace:
            return isLeft ? "y" : "b";
        case Source::Stick:
            return "thumbstick";
        case Source::GripClick:
            return "grip_click";
        case Source::Axis3Click:
            return "axis3";
        case Source::Axis4Click:
            return "axis4";
        case Source::PadUp:
            return "pad_up";
        case Source::PadDown:
            return "pad_down";
        case Source::PadLeft:
            return "pad_left";
        case Source::PadRight:
            return "pad_right";
        case Source::PadCenter:
            return "pad_center";
        case Source::Count:
            break;
        }
        return "none";
    }
}
