#pragma once

#include <cstdint>

namespace RE
{
    class NiNode;
}

namespace rock
{
    class TwoHandedGrip;

    struct AuthoredPrimaryFiringGripFrameInput
    {
        RE::NiNode* weaponNode{ nullptr };
        std::uint64_t weaponOwnershipKey{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        bool enabled{ false };
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
        bool leftHandedMode{ false };
    };

    // ROCK derives one generation-bound, modeler-authored primary grip and
    // inverts it onto hFRIK's live primary hand. The paired support relation
    // is published to TwoHandedGrip as an ephemeral proximity candidate; only
    // acquisition can latch it, so unrestricted dynamic grabs remain intact.
    class AuthoredPrimaryFiringGripRuntime
    {
    public:
        void update(
            const AuthoredPrimaryFiringGripFrameInput& input,
            TwoHandedGrip& weaponAuthority);
        void reset(const char* reason);

    private:
        void endSession(const char* reason);

        // Non-owning identity witness only; never dereferenced. This catches
        // an equip before WeaponCollision has published the new generation.
        RE::NiNode* _weaponNodeIdentity{ nullptr };
        std::uint64_t _weaponOwnershipKey{ 0 };
        std::uint64_t _captureSequenceFloor{ 0 };
        std::uint64_t _supportCaptureSequenceFloor{ 0 };
        bool _active{ false };
        bool _nativeReloadWasActive{ false };
        bool _sessionLogged{ false };
        bool _applyFailureLogged{ false };
        std::uint32_t _supportCaptureFailureReasonLogged{ 0 };
        std::uint16_t _supportCaptureFailureMaskLogged{ 0 };
        bool _supportCaptureFailureLogged{ false };
    };
}
