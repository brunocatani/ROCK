#pragma once

#include <cstdint>

namespace RE
{
    class NiAVObject;
}

namespace rock
{
    class WeaponVisualGraphRefreshCoordinator
    {
    public:
        struct UpdateInput
        {
            bool enabled{ false };
            bool visualAuthorityAvailable{ false };
            bool skeletonReady{ false };
            bool menuBlocking{ false };
            bool weaponDrawn{ false };
            RE::NiAVObject* weaponNode{ nullptr };
        };

        struct UpdateResult
        {
            bool refreshApplied{ false };
            bool deferWeaponCollision{ false };
            bool signatureChanged{ false };
            std::uint64_t signatureKey{ 0 };
        };

        void reset();
        UpdateResult update(const UpdateInput& input);

    private:
        std::uint64_t _lastSignatureKey{ 0 };
        std::uint64_t _lastConsumedWorkbenchRefreshEpoch{ 0 };
        std::uint64_t _pendingWorkbenchRefreshEpoch{ 0 };
        bool _hasLastSignature{ false };
        int _pendingApplyFrames{ 0 };
        int _postRefreshCollisionHoldFrames{ 0 };
    };
}
