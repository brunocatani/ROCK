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
    enum class Phase { BeforeRockPreFrik, BeforeFrik, AfterFrik, AfterWeaponSolve, AfterRock, AfterWorldFinal };
    enum class NativePhase { GraphEntry, GraphExit, PrimaryArmEntry, PrimaryArmExit, SupportArmEntry, SupportArmExit };

    // Borrowed only for the synchronous capture; the async logger receives
    // formatted values, never these references or the scene pointer.
    struct NativeAimCapture
    {
        const char* caller;
        std::uint32_t callerLine;
        std::uint32_t weaponFormId;
        std::uint64_t generation;
        std::uint64_t ownership;
        std::uint64_t instanceContent;
        const RE::NiAVObject* weapon;
        const RE::NiTransform& wandWorld;
        const RE::NiTransform& inputWorld;
        const RE::NiTransform& previousAim;
        const RE::NiTransform& nextAim;
        bool previousValid;
        bool identityChanged;
        bool cleanIntent;
        std::uint32_t intentSource;
        std::uint32_t gripState;
        bool authoredRefreshed;
        bool writeBlocked;
    };

    // Main/game thread only. The worker receives formatted text, never scene
    // pointers. Skeleton destruction drains and joins it outside frame capture.
    void initialize();
    void shutdown();
    void capture(Phase phase, std::uint64_t schedulerSequence);
    // Native hooks may run on other threads: those calls are ignored before
    // accessing session state. No scene pointers leave the owning callback.
    void recordNative(NativePhase phase, const RE::NiAVObject* weapon = nullptr,
        const RE::NiAVObject* offset = nullptr) noexcept;
    void recordNativeAimCapture(const NativeAimCapture& capture) noexcept;
    void recordLooseGrab(RE::TESObjectREFR* ref, bool isLeft, std::uint64_t grabIdentity,
        const RE::NiTransform& handWorld) noexcept;
    // Transition boundaries only, including modded weapons. Captures value
    // transforms synchronously; no scene pointers reach the async writer.
    void recordTransferPose(std::uint32_t refId, bool isLeft, const char* stage,
        const RE::NiTransform& weaponWorld, const RE::NiTransform& handWorld,
        const RE::NiTransform* proxyWorld = nullptr, const RE::NiTransform* desiredWeaponWorld = nullptr) noexcept;
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
