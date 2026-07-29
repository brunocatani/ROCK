#pragma once

#include <cstdint>

#include "RE/Bethesda/BSPointerHandle.h"
#include "RE/NetImmerse/NiPoint.h"

#include "api/ROCKProviderApi.h"
#include "physics-interaction/grenade/LooseGrenadeRuntime.h"

namespace rock
{
    enum class PendingForceGrabCommitOrigin : std::uint8_t
    {
        LooseGrenadeMenuEquip = 0,
        ProviderForceGrabCommand = 1,
    };

    enum class PendingForceGrabCommitPhase : std::uint8_t
    {
        WaitingForReference = 0,
        WaitingForSettle = 1,
        AcquireAndCommitExactTarget = 2,
    };

    /*
     * Shared deferred state for the force-grab API (loose-grenade menu
     * auto-equip and the Provider SDK's force-grab command): both spawn or
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
        PendingForceGrabCommitOrigin origin{ PendingForceGrabCommitOrigin::LooseGrenadeMenuEquip };
        PendingForceGrabCommitPhase phase{ PendingForceGrabCommitPhase::WaitingForSettle };

        RE::ObjectRefHandle targetHandle{};
        bool targetIsLooseGrenade{ false };
        std::uint32_t preferredBodyId{ 0x7FFF'FFFF };
        float maxDistanceGame{ 0.0f };
        bool hasSourcePointOverride{ false };
        RE::NiPoint3 sourcePointOverride{};
        float elapsedSettleSeconds{ 0.0f };
        float elapsedTotalSeconds{ 0.0f };
        float maxTotalSeconds{ 1.5f };

        // LooseGrenadeMenuEquip bookkeeping.
        std::uint64_t grenadeRequestId{ 0 };
        loose_grenade_runtime::GrenadeRuntimeData grenadeRuntime{};

        // ProviderForceGrabCommand bookkeeping: pre-filled with request identity;
        // only .state/.failure/.targetBodyId are mutated when the commit resolves.
        provider::RockProviderInteractionCommandResultV1 providerResultTemplate{};
    };
}
