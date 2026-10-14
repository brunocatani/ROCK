#pragma once

#include <ROCK/Grab.h>

namespace rock::provider
{
    // ROCK-internal force-grab flag: an inventory grab reaches the queue as a
    // force grab that first drops one owned item. A consumer's
    // ForceGrabRequest can never carry it.
    inline constexpr std::uint32_t kForceGrabFromPlayerInventory = 1u << 1;

    struct QueuedInteractionCommand
    {
        std::uint64_t ownerToken{ 0 };
        std::uint64_t commandId{ 0 };
        api::grab::InteractionCommandKind kind{ api::grab::InteractionCommandKind::Unknown };
        api::grab::ForceGrabRequest forceGrab{};
        api::PowerArmorPoint powerArmorPoint{};
        std::uint32_t powerArmorReferenceNativeHandle = 0;
        api::grab::ForceReleaseRequest forceRelease{};
        api::grab::ThrownDropRequest thrownDrop{};
    };

    [[nodiscard]] bool dequeueInteractionCommand(QueuedInteractionCommand& outCommand);
    [[nodiscard]] bool isInteractionCommandActive(std::uint64_t ownerToken, std::uint64_t commandId);
    bool completeInteractionCommand(const api::grab::InteractionCommandResult& result);
    void clearInteractionCommandsForProviderLoss(api::grab::InteractionFailure failure);
}
