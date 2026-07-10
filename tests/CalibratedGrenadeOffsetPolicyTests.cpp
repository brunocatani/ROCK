#include "physics-interaction/grenade/CalibratedGrenadeOffsetPolicy.h"

#include <bit>
#include <cstdint>
#include <cstdio>

namespace
{
    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectPreset(
        const char* label,
        rock::calibrated_grenade_offset::Preset actual,
        rock::calibrated_grenade_offset::Preset expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected preset %u got %u\n",
            label,
            static_cast<unsigned>(expected),
            static_cast<unsigned>(actual));
        return false;
    }

    std::uint64_t fingerprint(const rock::saved_grab_offset::HandOffset& offset)
    {
        std::uint64_t hash = 14695981039346656037ull;
        constexpr std::uint64_t prime = 1099511628211ull;
        auto mix = [&](float value) {
            hash ^= std::bit_cast<std::uint32_t>(value);
            hash *= prime;
        };

        for (const float value : offset.translateGame) {
            mix(value);
        }
        for (const float value : offset.rotate) {
            mix(value);
        }
        for (const float value : offset.fingerValues) {
            mix(value);
        }
        for (const float value : offset.fingerJointValues) {
            mix(value);
        }
        return hash;
    }

    bool expectFingerprint(
        const char* label,
        const rock::saved_grab_offset::HandOffset* offset,
        std::uint64_t expected)
    {
        if (!offset) {
            std::printf("%s expected a calibrated hand offset\n", label);
            return false;
        }
        bool ok = true;
        ok &= expectTrue(label, offset->present);
        ok &= expectTrue(label, offset->hasFingerPose);
        ok &= expectTrue(label, offset->hasFingerJointValues);
        const auto actual = fingerprint(*offset);
        if (actual != expected) {
            std::printf("%s expected fingerprint %llu got %llu\n",
                label,
                static_cast<unsigned long long>(expected),
                static_cast<unsigned long long>(actual));
            ok = false;
        }
        return ok;
    }
}

int main()
{
    using namespace rock::calibrated_grenade_offset;

    bool ok = true;
    ok &= expectPreset("disabled calibration selects no preset",
        selectPreset({ .enabled = false, .isGrenade = true, .isMolotov = true }),
        Preset::None);
    ok &= expectPreset("non-grenade fails closed even with Molotov evidence",
        selectPreset({ .enabled = true, .isGrenade = false, .isMolotov = true }),
        Preset::None);
    ok &= expectPreset("every non-Molotov grenade selects generic calibration",
        selectPreset({ .enabled = true, .isGrenade = true, .isMolotov = false }),
        Preset::GenericGrenade);
    ok &= expectPreset("Molotov classification takes preset priority",
        selectPreset({ .enabled = true, .isGrenade = true, .isMolotov = true }),
        Preset::Molotov);

    ok &= expectTrue("none preset has no hand offset", handOffsetForPreset(Preset::None, false) == nullptr);
    ok &= expectFingerprint("generic grenade left calibration",
        handOffsetForPreset(Preset::GenericGrenade, true),
        8184850932913033222ull);
    ok &= expectFingerprint("generic grenade right calibration",
        handOffsetForPreset(Preset::GenericGrenade, false),
        1410411899013989674ull);
    ok &= expectFingerprint("Molotov left calibration",
        handOffsetForPreset(Preset::Molotov, true),
        13877950579113736822ull);
    ok &= expectFingerprint("Molotov right calibration",
        handOffsetForPreset(Preset::Molotov, false),
        16560562437976390235ull);

    return ok ? 0 : 1;
}
