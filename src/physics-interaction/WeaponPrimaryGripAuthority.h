#pragma once

#include "WeaponReloadStageObserver.h"

#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiTransform.h"

#include <cstdint>

namespace frik::rock
{
    class WeaponPrimaryGripAuthority
    {
    public:
        void update(RE::NiNode* weaponNode, bool supportGripActive, const WeaponReloadRuntimeState& reloadState, bool authorityEnabled);
        void reset();
        bool getAppliedWeaponTransform(RE::NiTransform& outTransform) const;

    private:
        bool applyWeaponVisualAuthority(RE::NiNode* weaponNode, const RE::NiTransform& weaponWorldTarget);

        RE::NiNode* _activeWeaponNode{ nullptr };
        std::uint32_t _generation{ 0 };
        int _logCounter{ 0 };
        bool _authorityPublished{ false };
        RE::NiTransform _lastAppliedWeaponTransform{};
        bool _hasAppliedWeaponTransform{ false };
    };
}
