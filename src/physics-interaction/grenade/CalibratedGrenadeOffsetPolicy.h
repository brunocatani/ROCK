#pragma once

#include "physics-interaction/grab/SavedGrabOffsetFormat.h"

#include <cstdint>

/*
 * Built-in proxy-local grab calibrations for ROCK's loose grenade flow.
 * These values were promoted from live SavedGrabOffsets captures so grenade
 * menu force-grabs do not depend on user-local JSON files. The policy stays
 * engine-free: runtime code classifies the reference, then selects one of
 * these immutable per-hand values at grab commit.
 */
namespace rock::calibrated_grenade_offset
{
    enum class Preset : std::uint8_t
    {
        None,
        GenericGrenade,
        Molotov
    };

    struct SelectionInput
    {
        bool enabled{ false };
        bool isGrenade{ false };
        bool isMolotov{ false };
    };

    [[nodiscard]] inline constexpr Preset selectPreset(const SelectionInput& input) noexcept
    {
        if (!input.enabled || !input.isGrenade) {
            return Preset::None;
        }
        return input.isMolotov ? Preset::Molotov : Preset::GenericGrenade;
    }

    namespace detail
    {
        inline constexpr saved_grab_offset::HandOffset kGenericGrenadeLeft{
            .present = true,
            .translateGame = { 1.3048561f, -1.5200334f, -0.36595982f },
            .rotate = { -0.5395594f, -0.6141064f, 0.5759767f,
                -0.75175023f, 0.04332781f, -0.6580229f,
                0.37914038f, -0.7880331f, -0.48503256f },
            .hasFingerPose = true,
            .fingerValues = { 0.52145356f, 0.87053704f, 0.819491f, 0.45910558f, 0.3f },
            .hasFingerJointValues = true,
            .fingerJointValues = { 0.7887265f, 0.75144297f, 0.7265873f,
                0.9442159f, 0.92562115f, 0.91446435f,
                0.91165674f, 0.882209f, 0.86454034f,
                0.7948792f, 0.72650564f, 0.6854815f,
                0.72670734f, 0.63560975f, 0.5809512f },
        };

        inline constexpr saved_grab_offset::HandOffset kGenericGrenadeRight{
            .present = true,
            .translateGame = { -0.27794087f, -1.4240625f, 0.1950554f },
            .rotate = { 0.72462f, -0.2999105f, 0.6204673f,
                -0.67950463f, -0.16083777f, 0.71582454f,
                -0.11488871f, -0.9403111f, -0.32033685f },
            .hasFingerPose = true,
            .fingerValues = { 0.53125703f, 0.9355881f, 0.78240657f, 0.4818801f, 0.3f },
            .hasFingerJointValues = true,
            .fingerJointValues = { 0.8290377f, 0.7988679f, 0.77875465f,
                0.9648811f, 0.9531748f, 0.9461511f,
                0.8786371f, 0.83818275f, 0.8139102f,
                0.7361541f, 0.64820546f, 0.5954363f,
                0.6386266f, 0.51816875f, 0.44589406f },
        };

        inline constexpr saved_grab_offset::HandOffset kMolotovLeft{
            .present = true,
            .translateGame = { -7.140895f, -7.976515f, -13.889911f },
            .rotate = { 0.12858178f, -0.9473355f, 0.29329658f,
                0.8972765f, -0.014815431f, -0.44122082f,
                0.42232937f, 0.319901f, 0.8481164f },
            .hasFingerPose = true,
            .fingerValues = { 0.51999456f, 0.47409388f, 0.50985265f, 0.47774673f, 0.5869753f },
            .hasFingerJointValues = true,
            .fingerJointValues = { 0.8510573f, 0.8247733f, 0.8072506f,
                0.7011796f, 0.6015729f, 0.5418088f,
                0.6847397f, 0.5796529f, 0.51660085f,
                0.6379697f, 0.5172929f, 0.44488686f,
                0.74753326f, 0.6633777f, 0.61288434f },
        };

        inline constexpr saved_grab_offset::HandOffset kMolotovRight{
            .present = true,
            .translateGame = { -6.608655f, -6.889555f, 9.963478f },
            .rotate = { -0.5364618f, 0.8368081f, -0.1093674f,
                0.58121526f, 0.4603092f, 0.6710475f,
                0.61188066f, 0.29642516f, -0.7333039f },
            .hasFingerPose = true,
            .fingerValues = { 0.5111174f, 0.58407485f, 0.8240613f, 0.6668065f, 0.5562322f },
            .hasFingerJointValues = true,
            .fingerJointValues = { 0.76196706f, 0.7199613f, 0.6919574f,
                0.86858237f, 0.8247765f, 0.79849297f,
                0.90636426f, 0.87515235f, 0.8564252f,
                0.87382984f, 0.8317731f, 0.80653906f,
                0.8318979f, 0.7758639f, 0.74224347f },
        };
    }

    [[nodiscard]] inline constexpr const saved_grab_offset::HandOffset* handOffsetForPreset(Preset preset, bool isLeft) noexcept
    {
        switch (preset) {
        case Preset::GenericGrenade:
            return isLeft ? &detail::kGenericGrenadeLeft : &detail::kGenericGrenadeRight;
        case Preset::Molotov:
            return isLeft ? &detail::kMolotovLeft : &detail::kMolotovRight;
        case Preset::None:
        default:
            return nullptr;
        }
    }
}
