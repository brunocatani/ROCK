#pragma once

#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"

#include <array>
#include <cstdint>

namespace RE
{
    class NiNode;
    class TESObjectWEAP;
}

namespace rock
{
    class TwoHandedGrip;

    struct AuthoredPrimaryFiringGripFrameInput
    {
        RE::NiNode* weaponNode{ nullptr };
        const RE::TESObjectWEAP* weapon{ nullptr };
        std::uint64_t weaponOwnershipKey{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        bool runtimeInitialized{ false };
        bool visualAuthorityAvailable{ false };
        bool localSkeletonReady{ false };
        bool menuBlocking{ false };
        bool compatibilityBlocking{ false };
        bool weaponDrawn{ false };
        bool weaponVisible{ false };
        bool nativeReloadAuthorityActive{ false };
        bool conflictingWeaponTransformAuthorityActive{ false };
        bool weaponVisualReturnActive{ false };
        bool primaryHandHoldingObject{ false };
        bool rockFiringHandIsLeft{ false };
        bool inPowerArmor{ false };
    };

    // ROCK derives one generation-bound, modeler-authored primary grip and
    // inverts it onto hFRIK's live primary hand. The paired support relation
    // is captured while Bethesda's native right-primary topology is intact,
    // then republished as an ephemeral proximity candidate while ROCK's
    // physical left hand owns the weapon. Only acquisition can latch it, so
    // unrestricted dynamic grabs remain intact.
    class AuthoredPrimaryFiringGripRuntime
    {
    public:
        void update(
            const AuthoredPrimaryFiringGripFrameInput& input,
            TwoHandedGrip& weaponAuthority);
        void reset(const char* reason, TwoHandedGrip& weaponAuthority);

    private:
        struct StableAuthoredSupportGripSnapshot
        {
            // Non-owning identity witness only; never dereferenced. The value
            // transforms remain usable across ROCK's physical-left reparent
            // only while every weapon/canonical identity key still matches.
            RE::NiNode* weaponNodeIdentity{ nullptr };
            RE::NiTransform handWeaponLocal{};
            std::array<RE::NiTransform, 15> fingerLocalTransforms{};
            std::uint16_t fingerLocalTransformMask{ 0 };
            std::uint64_t weaponOwnershipKey{ 0 };
            std::uint64_t weaponGenerationKey{ 0 };
            std::uint64_t primaryGripCaptureSequence{ 0 };
            std::uint64_t supportCaptureSequence{ 0 };
            bool valid{ false };
        };

        void endSession(const char* reason);
        void clearStableAuthoredSupportGripSnapshot();

        // Non-owning identity witness only; never dereferenced. This catches
        // an equip before WeaponCollision has published the new generation.
        RE::NiNode* _weaponNodeIdentity{ nullptr };
        std::uint64_t _weaponOwnershipKey{ 0 };
        std::uint64_t _frikOffsetCacheRevision{ 0 };
        std::uint64_t _captureSequenceFloor{ 0 };
        std::uint64_t _supportCaptureSequenceFloor{ 0 };
        StableAuthoredSupportGripSnapshot _stableAuthoredSupportGrip{};
        authored_weapon_grip_library::FiringFingerPose _mirroredLeftFingerPose{};
        std::uint64_t _mirroredFingerPoseCaptureSequence{ 0 };
        bool _active{ false };
        bool _nativeReloadWasActive{ false };
        bool _sessionLogged{ false };
        bool _applyFailureLogged{ false };
        bool _canonicalPublishFailureLogged{ false };
        bool _libraryPublishFailureLogged{ false };
        bool _customFrikOffsetOverrideActive{ false };
        std::uint32_t _supportCaptureFailureReasonLogged{ 0 };
        std::uint16_t _supportCaptureFailureMaskLogged{ 0 };
        bool _supportCaptureFailureLogged{ false };
        bool _mirroredFingerPoseValid{ false };
        bool _fingerMirrorFailureLogged{ false };
    };
}
