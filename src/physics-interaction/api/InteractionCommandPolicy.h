#pragma once

#include "api/ROCKProviderApi.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace rock::provider::interaction_command_policy
{
    [[nodiscard]] inline constexpr bool isTerminal(RockProviderInteractionCommandStateV1 state) noexcept
    {
        return state == RockProviderInteractionCommandStateV1::Succeeded ||
               state == RockProviderInteractionCommandStateV1::Rejected ||
               state == RockProviderInteractionCommandStateV1::Cancelled;
    }

    [[nodiscard]] inline constexpr RockProviderInteractionCommandResultV1 mergeResultHistory(
        const RockProviderInteractionCommandResultV1& existing,
        const RockProviderInteractionCommandResultV1& update,
        const std::uint64_t currentFrame) noexcept
    {
        auto merged = update;

        // The first stored result is created at queue acceptance. Later
        // runtime stages must never reinterpret their own frame as acceptance.
        merged.acceptedFrame = existing.acceptedFrame;
        if (merged.committedFrame == 0) {
            merged.committedFrame = existing.committedFrame;
        }
        if (merged.appliedFrame == 0) {
            merged.appliedFrame = existing.appliedFrame;
        }

        if (isTerminal(merged.state)) {
            merged.stage = RockProviderCommandStageV1::Terminal;
            if (merged.frameIndex == 0) {
                merged.frameIndex = currentFrame;
            }
            if (merged.state == RockProviderInteractionCommandStateV1::Succeeded &&
                merged.appliedFrame == 0) {
                merged.appliedFrame = merged.frameIndex;
            }
            if (merged.failureStage == RockProviderInteractionFailureV1::None) {
                merged.failureStage = merged.failure;
            }
        }
        return merged;
    }

    struct ForceGrabReservation
    {
        std::uint64_t ownerToken{ 0 };
        std::uint64_t commandId{ 0 };
    };

    class ForceGrabReservations
    {
    public:
        [[nodiscard]] bool isReserved(RockProviderHand hand) const noexcept
        {
            const auto index = handIndex(hand);
            return index < _slots.size() && _slots[index].commandId != 0;
        }

        [[nodiscard]] bool reserve(RockProviderHand hand, std::uint64_t ownerToken, std::uint64_t commandId) noexcept
        {
            const auto index = handIndex(hand);
            if (index >= _slots.size() || ownerToken == 0 || commandId == 0 || _slots[index].commandId != 0) {
                return false;
            }
            _slots[index] = ForceGrabReservation{ .ownerToken = ownerToken, .commandId = commandId };
            return true;
        }

        [[nodiscard]] bool matches(std::uint64_t ownerToken, std::uint64_t commandId) const noexcept
        {
            for (const auto& slot : _slots) {
                if (slot.ownerToken == ownerToken && slot.commandId == commandId) {
                    return true;
                }
            }
            return false;
        }

        void release(std::uint64_t ownerToken, std::uint64_t commandId) noexcept
        {
            for (auto& slot : _slots) {
                if (slot.ownerToken == ownerToken && slot.commandId == commandId) {
                    slot = {};
                    return;
                }
            }
        }

        void clearOwner(std::uint64_t ownerToken) noexcept
        {
            for (auto& slot : _slots) {
                if (slot.ownerToken == ownerToken) {
                    slot = {};
                }
            }
        }

        void clear() noexcept { _slots = {}; }

    private:
        [[nodiscard]] static constexpr std::size_t handIndex(RockProviderHand hand) noexcept
        {
            return hand == RockProviderHand::Right ? 0u : hand == RockProviderHand::Left ? 1u : 2u;
        }

        std::array<ForceGrabReservation, 2> _slots{};
    };
}
