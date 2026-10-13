#pragma once
#include <array>
#include <cstdint>
#include "RE/NetImmerse/NiTransform.h"

namespace rock::gunstock_debug
{
    enum Pose : std::size_t { LeftController, RightController, LeftPhysical, PrimarySeat, SupportSeat, WeaponRendered,
        LeftAimReference, RightAimReference, LeftAimConfigured, RightAimConfigured, LeftAimDamped, RightAimDamped, PoseCount };
    struct AimError { bool valid{}; float yaw{},pitch{},angle{}; };
    AimError aimDirectionError(const RE::NiTransform& reference, const RE::NiPoint3& actualDirection);
    AimError aimError(const RE::NiTransform& reference, const RE::NiTransform& actual);
    // Live samples for the gunstock and controller-aim overlays. All poses are
    // world-space Ni transforms (stored rows, local vectors use the transpose).
    // Seat origins are grip points, not wrist origins. Bit N of valid
    // corresponds to Pose N; absent poses must never be read as identity.
    // Phase 1 is the grip-update output, phase 2 AfterWorldFinal. Controller
    // inputs are sampled once before the solve and shared by both phases.
    struct Sample {
        float aimYawDegrees{},aimPitchDegrees{};
        std::uint64_t frame{}, grip{}, mark{}, valid{};
        std::uint32_t form{}, firingLeft{}, supportActive{}, phase{}, gunstockAim{};
        std::array<RE::NiTransform, PoseCount> poses{};
        bool has(Pose pose) const { return (valid & (std::uint64_t{1} << pose)) != 0; }
    };
    bool setPose(Sample& sample, Pose pose, const RE::NiTransform& value);
}
