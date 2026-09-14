#pragma once

#include <cstdint>

#include "RE/Bethesda/BSPointerHandle.h"
#include "RE/NetImmerse/NiPoint.h"

#include "api/ROCKProviderApi.h"
#include "physics-interaction/grenade/LooseGrenadeRuntime.h"

namespace rock
{
    enum class PendingForceGrabCommitPhase : std::uint8_t
    {
        WaitingForReference = 0,
        WaitingForSettle = 1,
        AcquireAndCommitExactTarget = 2,
        WaitingForNativePlacement = 3,
        NativePlacementFailed = 4,
    };

    /*
     * Deferred state for provider world grabs and inventory transfers: both
     * target an object and must attach it to the hand, but freezing the
     * grab-authority relation on the same tick reads whatever hand transform
     * happens to exist that instant. This carries the request across the
     * settle delay (fForceGrabAttachSettleSeconds) so the eventual commit
     * reads a genuinely live hand pose. A saved grab offset, if any exists
     * for the target object+hand, is applied by grabSelectedObject itself at
     * commit time (SelectedObject::forcedArrival already makes it eligible,
     * same as an organic pull-catch/far-grab commit).
     */
    struct PendingForceGrabCommit
    {
        bool active{ false };
        bool isLeft{ false };
        PendingForceGrabCommitPhase phase{ PendingForceGrabCommitPhase::WaitingForSettle };

        RE::ObjectRefHandle targetHandle{};
        bool targetIsLooseThrowable{ false };
        bool inventoryTransfer{ false };
        // Internal B-hold draws share transfer/rollback, without an API owner.
        bool grenadeQuickDraw{ false };
        bool equippedWeaponTransfer{ false };
        std::uint32_t preferredBodyId{ 0x7FFF'FFFF };
        float maxDistanceGame{ 0.0f };
        bool hasSourcePointOverride{ false };
        RE::NiPoint3 sourcePointOverride{};
        float elapsedSettleSeconds{ 0.0f };
        float elapsedTotalSeconds{ 0.0f };
        float maxTotalSeconds{ 1.5f };

        // ProviderForceGrabCommand bookkeeping: pre-filled with request identity;
        // only .state/.failure/.targetBodyId are mutated when the commit resolves.
        provider::RockProviderInteractionCommandResultV1 providerResultTemplate{};

        [[nodiscard]] bool internallyOwned() const noexcept
        {
            return grenadeQuickDraw || equippedWeaponTransfer;
        }
    };
}
