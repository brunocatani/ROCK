#define ROCK_API_EXPORTS
/*
 * Build the DLL side of the public provider ABI.
 *
 * INTERACTION COMMANDS: the consumer request queue for force grab, force release,
 * and thrown drop.
 *
 * Read in order: a consumer enqueues a request, the game thread dequeues it through
 * the internal bridge, and the result is stored back for the consumer to read.
 * Field access into the command union goes through one selector in
 * detail/ProviderCommandMarshal.h; do not hand-roll a second switch.
 */
#include "api/detail/ProviderInteractionCommands.h"

#include "api/detail/ProviderApiCore.h"
#include "api/detail/ProviderApiEntryPoints.h"
#include "api/detail/ProviderCommandMarshal.h"

#include <cmath>
#include <mutex>

#include "physics-interaction/api/InteractionCommandPolicy.h"
#include "physics-interaction/core/PhysicsInteraction.h"

namespace rock::provider::detail
{
    constexpr std::uint32_t kImplementedForceGrabFlagsV1 =
        static_cast<std::uint32_t>(RockProviderForceGrabFlagV1::UsePreferredGrabPointGame);
    constexpr std::uint32_t kImplementedForceReleaseFlagsV1 =
        static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::ImmediateCollisionRestore) |
        static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::RequireMatchingTarget) |
        static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::UseVelocityHavok);
    constexpr std::uint32_t kImplementedThrownDropFlagsV1 =
        static_cast<std::uint32_t>(RockProviderThrownDropFlagV1::ImmediateCollisionRestore) |
        static_cast<std::uint32_t>(RockProviderThrownDropFlagV1::RequireMatchingTarget) |
        static_cast<std::uint32_t>(RockProviderThrownDropFlagV1::UseVelocityHavok);

    RockProviderResultV1 validateInteractionCommandOwnerLocked(std::uint64_t ownerToken)
    {
        return validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::InteractionCommands);
    }


    std::uint64_t nextInteractionCommandId()
    {
        auto id = s_nextInteractionCommandId.fetch_add(1, std::memory_order_acq_rel);
        if (id == 0) {
            id = s_nextInteractionCommandId.fetch_add(1, std::memory_order_acq_rel);
        }
        return id;
    }


    void completeInteractionCommandLocked(
        const QueuedInteractionCommandV1& command,
        RockProviderInteractionCommandStateV1 state,
        RockProviderInteractionFailureV1 failure)
    {
        const auto result = makeCommandResult(command, state, failure);
        storeInteractionResultLocked(result);
        if (interaction_command_policy::isTerminal(result.state)) {
            s_forceGrabReservations.release(result.ownerToken, result.commandId);
        }
    }

    void clearInteractionCommandsForOwnerLocked(std::uint64_t ownerToken, RockProviderInteractionFailureV1 failure)
    {
        if (ownerToken == 0) {
            return;
        }

        for (auto& slot : s_interactionCommands) {
            if (slot.active && slot.command.ownerToken == ownerToken) {
                completeInteractionCommandLocked(slot.command, RockProviderInteractionCommandStateV1::Cancelled, failure);
                slot = {};
            }
        }

        for (auto& slot : s_interactionResults) {
            if (slot.active && slot.result.ownerToken == ownerToken) {
                slot = {};
            }
        }
        s_forceGrabReservations.clearOwner(ownerToken);
    }


    bool hasInteractionTargetIdentity(std::uint32_t targetFormId, std::uint32_t targetBodyId)
    {
        return targetFormId != 0 || targetBodyId != kProviderInvalidBodyId;
    }

    bool isFiniteVector3(const float values[3])
    {
        return std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]);
    }

    RockProviderResultV1 validateInteractionCommandProviderReady()
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return RockProviderResultV1::NotReady;
        }
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 enqueueInteractionCommand(QueuedInteractionCommandV1 command, std::uint64_t* outCommandId)
    {
        if (!outCommandId) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outCommandId = 0;

        const auto providerReadyResult = validateInteractionCommandProviderReady();
        if (providerReadyResult != RockProviderResultV1::Ok) {
            return providerReadyResult;
        }

        std::scoped_lock lock(s_consumerMutex, s_interactionCommandMutex);
        const auto ownerResult = validateInteractionCommandOwnerLocked(command.ownerToken);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        if (command.kind == RockProviderInteractionCommandKindV1::ForceGrab &&
            s_forceGrabReservations.isReserved(command.forceGrab.hand)) {
            return RockProviderResultV1::HandBusy;
        }

        for (auto& slot : s_interactionCommands) {
            if (!slot.active) {
                command.commandId = nextInteractionCommandId();
                if (command.kind == RockProviderInteractionCommandKindV1::ForceGrab &&
                    !s_forceGrabReservations.reserve(command.forceGrab.hand, command.ownerToken, command.commandId)) {
                    return RockProviderResultV1::HandBusy;
                }
                slot = InteractionCommandSlot{
                    .active = true,
                    .command = command,
                };
                *outCommandId = command.commandId;
                storeInteractionResultLocked(makeCommandResult(command, RockProviderInteractionCommandStateV1::Queued, RockProviderInteractionFailureV1::None));
                return RockProviderResultV1::RequestQueued;
            }
        }

        return RockProviderResultV1::CapacityFull;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRequestForceGrabV1(
        std::uint64_t ownerToken,
        const RockProviderForceGrabRequestV1* request,
        std::uint64_t* outCommandId)
    {
        const auto entryResult = validateEntry(request);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        if (!outCommandId || ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outCommandId = 0;
        if (request->hand != RockProviderHand::Right && request->hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }
        if (request->targetFormId == 0 || (request->flags & ~kImplementedForceGrabFlagsV1) != 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!std::isfinite(request->maxDistanceGame)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(RockProviderForceGrabFlagV1::UsePreferredGrabPointGame)) != 0 &&
            !isFiniteVector3(request->preferredGrabPointGame)) {
            return RockProviderResultV1::InvalidArgument;
        }

        QueuedInteractionCommandV1 command{};
        command.ownerToken = ownerToken;
        command.kind = RockProviderInteractionCommandKindV1::ForceGrab;
        command.forceGrab = *request;
        return enqueueInteractionCommand(command, outCommandId);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRequestForceReleaseV1(
        std::uint64_t ownerToken,
        const RockProviderForceReleaseRequestV1* request,
        std::uint64_t* outCommandId)
    {
        const auto entryResult = validateEntry(request);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        if (!outCommandId || ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outCommandId = 0;
        if (request->hand != RockProviderHand::Right && request->hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }
        if ((request->flags & ~kImplementedForceReleaseFlagsV1) != 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::RequireMatchingTarget)) != 0 &&
            !hasInteractionTargetIdentity(request->targetFormId, request->targetBodyId)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::UseVelocityHavok)) != 0 &&
            (!isFiniteVector3(request->linearVelocityHavok) || !isFiniteVector3(request->angularVelocityRadiansPerSecond))) {
            return RockProviderResultV1::InvalidArgument;
        }

        QueuedInteractionCommandV1 command{};
        command.ownerToken = ownerToken;
        command.kind = RockProviderInteractionCommandKindV1::ForceRelease;
        command.forceRelease = *request;
        return enqueueInteractionCommand(command, outCommandId);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRequestThrownDropV1(
        std::uint64_t ownerToken,
        const RockProviderThrownDropRequestV1* request,
        std::uint64_t* outCommandId)
    {
        const auto entryResult = validateEntry(request);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        if (!outCommandId || ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outCommandId = 0;
        if (request->hand != RockProviderHand::Right && request->hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }
        if ((request->flags & ~kImplementedThrownDropFlagsV1) != 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(RockProviderThrownDropFlagV1::RequireMatchingTarget)) != 0 &&
            !hasInteractionTargetIdentity(request->targetFormId, request->targetBodyId)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(RockProviderThrownDropFlagV1::UseVelocityHavok)) != 0 &&
            (!isFiniteVector3(request->linearVelocityHavok) || !isFiniteVector3(request->angularVelocityRadiansPerSecond))) {
            return RockProviderResultV1::InvalidArgument;
        }

        QueuedInteractionCommandV1 command{};
        command.ownerToken = ownerToken;
        command.kind = RockProviderInteractionCommandKindV1::ThrownDrop;
        command.thrownDrop = *request;
        return enqueueInteractionCommand(command, outCommandId);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetInteractionCommandResultV1(
        std::uint64_t ownerToken,
        std::uint64_t commandId,
        RockProviderInteractionCommandResultV1* outResult)
    {
        if (!outResult || ownerToken == 0 || commandId == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        const auto requestedSize = outResult->size;
        if (requestedSize <
            ROCK_PROVIDER_INTERACTION_COMMAND_RESULT_V1_PREFIX_SIZE) {
            return RockProviderResultV1::InvalidSize;
        }
        const auto copyResult = [outResult, requestedSize](
                                    RockProviderInteractionCommandResultV1 result) {
            const auto copySize = (std::min<std::size_t>)(
                requestedSize,
                sizeof(result));
            result.size = static_cast<std::uint32_t>(copySize);
            std::memcpy(outResult, &result, copySize);
        };

        std::scoped_lock lock(s_consumerMutex, s_interactionCommandMutex);
        const auto ownerResult = validateInteractionCommandOwnerLocked(ownerToken);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        for (const auto& slot : s_interactionResults) {
            if (slot.active && slot.result.ownerToken == ownerToken && slot.result.commandId == commandId) {
                copyResult(slot.result);
                return RockProviderResultV1::Ok;
            }
        }

        for (const auto& slot : s_interactionCommands) {
            if (slot.active && slot.command.ownerToken == ownerToken && slot.command.commandId == commandId) {
                copyResult(makeCommandResult(
                    slot.command,
                    RockProviderInteractionCommandStateV1::Queued,
                    RockProviderInteractionFailureV1::None));
                return RockProviderResultV1::Ok;
            }
        }

        return RockProviderResultV1::RequestNotFound;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiCancelInteractionCommandV1(
        const std::uint64_t ownerToken,
        const std::uint64_t commandId)
    {
        if (ownerToken == 0 || commandId == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_interactionCommandMutex);
        const auto ownerResult = validateInteractionCommandOwnerLocked(ownerToken);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        for (auto& slot : s_interactionCommands) {
            if (!slot.active || slot.command.ownerToken != ownerToken ||
                slot.command.commandId != commandId) {
                continue;
            }
            completeInteractionCommandLocked(
                slot.command,
                RockProviderInteractionCommandStateV1::Cancelled,
                RockProviderInteractionFailureV1::None);
            slot = {};
            return RockProviderResultV1::Ok;
        }
        for (const auto& slot : s_interactionResults) {
            if (!slot.active || slot.result.ownerToken != ownerToken ||
                slot.result.commandId != commandId) {
                continue;
            }
            return interaction_command_policy::isTerminal(slot.result.state) ?
                RockProviderResultV1::RequestNotFound :
                RockProviderResultV1::AlreadyCommitted;
        }
        return RockProviderResultV1::RequestNotFound;
    }


}

namespace rock::provider
{
    using namespace detail;

    bool dequeueInteractionCommandV1(QueuedInteractionCommandV1& outCommand)
    {
        std::scoped_lock lock(s_interactionCommandMutex);
        InteractionCommandSlot* oldestSlot = nullptr;
        for (auto& slot : s_interactionCommands) {
            if (slot.active && (!oldestSlot || slot.command.commandId < oldestSlot->command.commandId)) {
                oldestSlot = &slot;
            }
        }

        if (oldestSlot) {
            outCommand = oldestSlot->command;
            *oldestSlot = {};
            for (auto& resultSlot : s_interactionResults) {
                if (resultSlot.active &&
                    resultSlot.result.ownerToken == outCommand.ownerToken &&
                    resultSlot.result.commandId == outCommand.commandId) {
                    resultSlot.result.stage =
                        RockProviderCommandStageV1::Committed;
                    resultSlot.result.committedFrame = currentProviderFrameIndex();
                    break;
                }
            }
            return true;
        }
        return false;
    }

    bool isInteractionCommandActiveV1(std::uint64_t ownerToken, std::uint64_t commandId)
    {
        if (ownerToken == 0 || commandId == 0) {
            return false;
        }

        std::scoped_lock lock(s_interactionCommandMutex);
        // The reservation is the durable ownership record. Result slots are a
        // bounded polling history and may legitimately wrap while a deferred
        // physics commit is still alive.
        return s_forceGrabReservations.matches(ownerToken, commandId);
    }

    bool completeInteractionCommandV1(const RockProviderInteractionCommandResultV1& result)
    {
        std::scoped_lock lock(s_interactionCommandMutex);
        if (result.kind == RockProviderInteractionCommandKindV1::ForceGrab &&
            !s_forceGrabReservations.matches(result.ownerToken, result.commandId)) {
            return false;
        }
        storeInteractionResultLocked(result);
        if (interaction_command_policy::isTerminal(result.state)) {
            s_forceGrabReservations.release(result.ownerToken, result.commandId);
        }
        return true;
    }

    void clearInteractionCommandsForProviderLossV1(RockProviderInteractionFailureV1 failure)
    {
        std::scoped_lock lock(s_interactionCommandMutex);
        /*
         * Results remain Queued after dequeue while PhysicsInteraction owns the
         * deferred commit. Cancel those as well as slots still in the bounded
         * queue so consumers never observe an immortal command.
         */
        for (auto& slot : s_interactionResults) {
            if (slot.active && slot.result.state == RockProviderInteractionCommandStateV1::Queued) {
                auto terminal = slot.result;
                terminal.state =
                    RockProviderInteractionCommandStateV1::Cancelled;
                terminal.failure = failure;
                terminal.failureStage = failure;
                terminal.stage = RockProviderCommandStageV1::Terminal;
                terminal.frameIndex = currentProviderFrameIndex();
                storeInteractionResultLocked(terminal);
            }
        }
        s_interactionCommands = {};
        s_forceGrabReservations.clear();
    }


    void markInteractionCommandStageV1(
        const std::uint64_t ownerToken,
        const std::uint64_t commandId,
        const RockProviderCommandStageV1 stage)
    {
        if (ownerToken == 0 || commandId == 0 ||
            stage == RockProviderCommandStageV1::Unknown ||
            stage == RockProviderCommandStageV1::Terminal) {
            return;
        }
        std::scoped_lock lock(s_interactionCommandMutex);
        for (auto& slot : s_interactionResults) {
            if (!slot.active || slot.result.ownerToken != ownerToken ||
                slot.result.commandId != commandId ||
                interaction_command_policy::isTerminal(slot.result.state)) {
                continue;
            }
            slot.result.stage = stage;
            const auto frameIndex = currentProviderFrameIndex();
            if (stage == RockProviderCommandStageV1::Committed &&
                slot.result.committedFrame == 0) {
                slot.result.committedFrame = frameIndex;
            }
            if (stage == RockProviderCommandStageV1::Applied &&
                slot.result.appliedFrame == 0) {
                slot.result.appliedFrame = frameIndex;
            }
            return;
        }
    }
}
