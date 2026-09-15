#pragma once

#include "RE/NetImmerse/NiTransform.h"
#include <cstdint>

namespace RE { class NiAVObject; class TESObjectREFR; }
namespace rock::authored_weapon_grip_library { struct LookupResult; struct WeaponVariantIdentity; }

namespace rock
{
    struct AuthoredPrimaryFiringGripFrameInput;
}

namespace rock::vanilla_weapon_alignment_telemetry
{
    enum class Phase { BeforeRockPreFrik, BeforeFrik, AfterFrik, AfterRock };
    enum class NativePhase { GraphEntry, GraphExit, PrimaryArmEntry, PrimaryArmExit, SupportArmEntry, SupportArmExit };

    // Main/game thread only. The worker receives formatted text, never scene
    // pointers. Skeleton destruction drains and joins it outside frame capture.
    void initialize();
    void shutdown();
    void capture(Phase phase, std::uint64_t schedulerSequence);
    // Native hooks may run on other threads: those calls are ignored before
    // accessing session state. No scene pointers leave the owning callback.
    void recordNative(NativePhase phase, const RE::NiAVObject* weapon = nullptr,
        const RE::NiAVObject* offset = nullptr) noexcept;
    void recordLooseGrab(RE::TESObjectREFR* ref, bool isLeft, std::uint64_t grabIdentity,
        const RE::NiTransform& handWorld) noexcept;
    void recordInput(const AuthoredPrimaryFiringGripFrameInput& input);
    void recordAuthoredSelection(const AuthoredPrimaryFiringGripFrameInput& input,
        const authored_weapon_grip_library::WeaponVariantIdentity& requested,
        const authored_weapon_grip_library::LookupResult& selected,
        const RE::NiPoint3& modelTranslation, bool compiledMinigunSeat) noexcept;
    // Boundary-only calls from the main-thread preharvest job. The writer
    // receives formatted values and never retains graph or pose pointers.
    void recordAuthoredPose(std::uint32_t formId, std::uint64_t captureSequence,
        const char* source, const char* label, const RE::NiTransform& pose) noexcept;
    void recordSolve(std::uint32_t formId, std::uint64_t captureSequence,
        const char* source, const RE::NiTransform& handInWeapon,
        const RE::NiTransform& trackedHand, const RE::NiTransform& solvedWeapon);
}
