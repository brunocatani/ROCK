#define ROCK_API_EXPORTS
/*
 * Build the DLL side of the public provider ABI.
 *
 * COMMAND MARSHALING: one field selector over the interaction command union, plus
 * result construction and storage.
 *
 * commandFields replaced six near-identical accessors that each switched over the
 * same union. Read a field through it. Do not add a seventh switch.
 */
#include "api/detail/ProviderCommandMarshal.h"

#include "physics-interaction/api/InteractionCommandPolicy.h"

namespace rock::provider::detail
{
    [[nodiscard]] CommandFields commandFields(
        const QueuedInteractionCommandV1& command)
    {
        const auto select = []<class Request>(const Request& request) {
            return CommandFields{
                .hand = request.hand,
                .targetFormId = request.targetFormId,
                .targetBodyId = request.targetBodyId,
                .worldGeneration = request.worldGeneration,
                .skeletonGeneration = request.skeletonGeneration,
                .providerGeneration = request.providerGeneration,
            };
        };

        switch (command.kind) {
        case RockProviderInteractionCommandKindV1::ForceGrab:
            return select(command.forceGrab);
        case RockProviderInteractionCommandKindV1::ForceRelease:
            return select(command.forceRelease);
        case RockProviderInteractionCommandKindV1::ThrownDrop:
            return select(command.thrownDrop);
        default:
            return {};
        }
    }

    RockProviderInteractionCommandResultV1 makeCommandResult(
        const QueuedInteractionCommandV1& command,
        RockProviderInteractionCommandStateV1 state,
        RockProviderInteractionFailureV1 failure)
    {
        RockProviderInteractionCommandResultV1 result{};
        result.size = sizeof(RockProviderInteractionCommandResultV1);
        result.version = ROCK_PROVIDER_API_VERSION;
        result.ownerToken = command.ownerToken;
        result.commandId = command.commandId;
        result.kind = command.kind;
        result.state = state;
        result.failure = failure;
        const auto fields = commandFields(command);
        result.hand = fields.hand;
        result.targetFormId = fields.targetFormId;
        result.targetBodyId = fields.targetBodyId;
        result.worldGeneration = fields.worldGeneration;
        result.skeletonGeneration = fields.skeletonGeneration;
        result.providerGeneration = fields.providerGeneration;
        result.stage = interaction_command_policy::isTerminal(state) ?
            RockProviderCommandStageV1::Terminal :
            RockProviderCommandStageV1::Queued;
        result.failureStage = failure;
        result.acceptedFrame = currentProviderFrameIndex();
        if (result.stage == RockProviderCommandStageV1::Terminal) {
            result.frameIndex = result.acceptedFrame;
        }
        return result;
    }

    void storeInteractionResultLocked(const RockProviderInteractionCommandResultV1& result)
    {
        for (auto& slot : s_interactionResults) {
            if (slot.active && slot.result.ownerToken == result.ownerToken && slot.result.commandId == result.commandId) {
                const bool wasTerminal =
                    interaction_command_policy::isTerminal(slot.result.state);
                const auto merged = interaction_command_policy::mergeResultHistory(
                    slot.result,
                    result,
                    currentProviderFrameIndex());
                slot.result = merged;
                if (!wasTerminal &&
                    interaction_command_policy::isTerminal(merged.state)) {
                    RockProviderEventV1 event{};
                    event.kind =
                        RockProviderEventKindV1::InteractionCommandTerminal;
                    event.ownerToken = merged.ownerToken;
                    event.hand = merged.hand;
                    event.subjectSequence = merged.commandId;
                    event.result = static_cast<std::uint32_t>(merged.state);
                    event.data[0] = static_cast<std::uint32_t>(merged.kind);
                    event.data[1] = static_cast<std::uint32_t>(merged.failure);
                    event.formId = merged.targetFormId;
                    publishProviderEvent(event);
                }
                return;
            }
        }

        /*
         * Queued results are live command state, not disposable polling
         * history. The bounded queue cannot produce more live commands than
         * this result table can hold, so rotate only through empty/terminal
         * slots and never make an in-flight command disappear from polling.
         */
        for (std::size_t offset = 0; offset < s_interactionResults.size(); ++offset) {
            const std::size_t index = (s_nextInteractionResultSlot + offset) % s_interactionResults.size();
            auto& slot = s_interactionResults[index];
            if (slot.active && !interaction_command_policy::isTerminal(slot.result.state)) {
                continue;
            }
            auto storedResult = result;
            if (interaction_command_policy::isTerminal(storedResult.state)) {
                storedResult.stage = RockProviderCommandStageV1::Terminal;
                if (storedResult.frameIndex == 0) {
                    storedResult.frameIndex = currentProviderFrameIndex();
                }
                if (storedResult.state ==
                        RockProviderInteractionCommandStateV1::Succeeded &&
                    storedResult.appliedFrame == 0) {
                    storedResult.appliedFrame = storedResult.frameIndex;
                }
                if (storedResult.failureStage ==
                    RockProviderInteractionFailureV1::None) {
                    storedResult.failureStage = storedResult.failure;
                }
            }
            slot = InteractionCommandResultSlot{
                .active = true,
                .result = storedResult,
            };
            if (interaction_command_policy::isTerminal(storedResult.state)) {
                RockProviderEventV1 event{};
                event.kind = RockProviderEventKindV1::InteractionCommandTerminal;
                event.ownerToken = storedResult.ownerToken;
                event.hand = storedResult.hand;
                event.subjectSequence = storedResult.commandId;
                event.result = static_cast<std::uint32_t>(storedResult.state);
                event.data[0] = static_cast<std::uint32_t>(storedResult.kind);
                event.data[1] = static_cast<std::uint32_t>(storedResult.failure);
                event.formId = storedResult.targetFormId;
                publishProviderEvent(event);
            }
            s_nextInteractionResultSlot = (index + 1) % s_interactionResults.size();
            return;
        }
    }
}
