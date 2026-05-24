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

        struct EquippedWeaponModSignature
        {
            std::uint64_t key{ 0 };
            std::uint32_t formID{ 0 };
            std::uintptr_t instanceDataAddress{ 0 };
            std::uintptr_t objectInstanceExtraAddress{ 0 };
            std::uint64_t objectIndexDataSignature{ 0 };
            std::uint32_t objectIndexDataCount{ 0 };
            std::uint32_t activeModCount{ 0 };
            std::uint32_t disabledModCount{ 0 };
            std::uintptr_t equippedDataAddress{ 0 };
            std::uintptr_t equippedObjectAddress{ 0 };
            bool hasEquippedWeapon{ false };
        };

    private:
        void scheduleRefresh(const EquippedWeaponModSignature& signature, const char* reason);
        bool tryApplyRefresh(const UpdateInput& input);

        EquippedWeaponModSignature _lastSignature{};
        EquippedWeaponModSignature _pendingSignature{};
        bool _hasLastSignature{ false };
        bool _pendingRefresh{ false };
        const char* _pendingReason{ "unknown" };
        int _pendingApplyFrames{ 0 };
        int _postRefreshCollisionHoldFrames{ 0 };
    };
}
