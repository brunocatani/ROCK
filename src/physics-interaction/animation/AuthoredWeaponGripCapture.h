#pragma once

#include "RE/NetImmerse/NiTransform.h"

#include <array>
#include <cstdint>

namespace RE
{
    class NiNode;
}

namespace rock::authored_weapon_grip_capture
{
    struct PrimaryFiringGripCaptureStatus
    {
        std::uint64_t captureSequence{ 0 };
        bool valid{ false };
    };

    enum class AuthoredSupportGripCaptureFailureReason : std::uint32_t
    {
        None = 0,
        SecondaryPassNotObserved,
        SourceTreeUnavailable,
        BoneCacheIncomplete,
        TopologyInvalid,
        PrimaryPoseUnavailable,
        AuthoredGraphPoseUnavailable,
        AuthoredHandHierarchyInvalid,
        SupportHandTransformInvalid,
        FingerTransformInvalid,
        ThreadMismatch,
        CaptureFault,
    };

    struct AuthoredSupportGripCaptureStatus
    {
        std::uint64_t captureSequence{ 0 };
        std::uint64_t secondaryPassSequence{ 0 };
        AuthoredSupportGripCaptureFailureReason failureReason{
            AuthoredSupportGripCaptureFailureReason::SecondaryPassNotObserved
        };
        std::uint16_t invalidOrMissingFingerMask{ 0 };
        bool valid{ false };
    };

    // Installs ROCK's validated UpdateFirstPersonArm grip capture and the one
    // shared native graph-output coordinator used by grip capture and provider
    // callbacks. Reload/bolt capture, application, WeaponFire, and
    // ReloadStateChange behavior remain animation-addon responsibilities.
    [[nodiscard]] bool installHook();
    [[nodiscard]] bool isHookInstalled();
    void setEnabled(bool enabled);
    void resetTransientState();

    [[nodiscard]] PrimaryFiringGripCaptureStatus
        queryPrimaryFiringGripCaptureStatus();
    [[nodiscard]] AuthoredSupportGripCaptureStatus
        queryAuthoredSupportGripCaptureStatus();
    [[nodiscard]] const char* authoredSupportGripCaptureFailureReasonName(
        AuthoredSupportGripCaptureFailureReason reason);
    [[nodiscard]] bool tryResolvePrimaryFiringGripAlignment(
        const RE::NiNode* expectedWeaponNode,
        const RE::NiTransform& liveWeaponWorld,
        const RE::NiTransform& trackedPrimaryHandWorld,
        RE::NiTransform& outWeaponWorld,
        RE::NiTransform& outCurrentAuthoredHandWorld,
        RE::NiTransform& outAuthoredPrimaryHandInWeapon,
        std::uint64_t& outCaptureSequence);
    [[nodiscard]] bool tryResolveAuthoredSupportGrip(
        const RE::NiNode* expectedWeaponNode,
        RE::NiTransform& outSupportHandInWeapon,
        std::array<RE::NiTransform, 15>& outFingerLocalTransforms,
        std::uint16_t& outFingerLocalTransformMask,
        std::uint64_t& outCaptureSequence);
}
