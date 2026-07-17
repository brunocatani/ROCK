#pragma once

#include <cstdint>

namespace RE
{
    class NiNode;
}

namespace rock
{
    struct AuthoredPrimaryFiringGripFrameInput
    {
        RE::NiNode* weaponNode{ nullptr };
        std::uint64_t weaponOwnershipKey{ 0 };
        bool enabled{ false };
        bool runtimeInitialized{ false };
        bool visualAuthorityAvailable{ false };
        bool localSkeletonReady{ false };
        bool menuBlocking{ false };
        bool compatibilityBlocking{ false };
        bool weaponDrawn{ false };
        bool weaponVisible{ false };
        bool nativeReloadAuthorityActive{ false };
        bool manualWeaponAuthorityActive{ false };
        bool primaryHandHoldingObject{ false };
        bool leftHandedMode{ false };
    };

    // ROCK owns this tagged FRIK lease for the lifetime of one
    // PhysicsInteraction instance. It drives only Hand::Primary; neither the
    // visible weapon nor the support hand is ever written by this runtime.
    class AuthoredPrimaryFiringGripRuntime
    {
    public:
        void update(const AuthoredPrimaryFiringGripFrameInput& input);
        void reset(const char* reason);

    private:
        void clearAuthority(const char* reason);

        std::uint64_t _weaponOwnershipKey{ 0 };
        std::uint64_t _captureSequenceFloor{ 0 };
        bool _published{ false };
        bool _nativeReloadWasActive{ false };
        bool _sessionLogged{ false };
        bool _publishFailureLogged{ false };
        bool _clearFailureLogged{ false };
    };
}
