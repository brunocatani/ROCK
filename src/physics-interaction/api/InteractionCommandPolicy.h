#pragma once

#include "api/ProviderRuntimeTypes.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace rock::provider::interaction_command_policy
{
    [[nodiscard]] inline constexpr bool isTerminal(api::grab::InteractionCommandState state) noexcept
    {
        return state == api::grab::InteractionCommandState::Succeeded ||
               state == api::grab::InteractionCommandState::Rejected ||
               state == api::grab::InteractionCommandState::Cancelled;
    }

    [[nodiscard]] inline constexpr api::grab::InteractionCommandResult mergeResultHistory(
        const api::grab::InteractionCommandResult& existing,
        const api::grab::InteractionCommandResult& update,
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
            merged.stage = api::grab::CommandStage::Terminal;
            if (merged.frameIndex == 0) {
                merged.frameIndex = currentFrame;
            }
            if (merged.state == api::grab::InteractionCommandState::Succeeded &&
                merged.appliedFrame == 0) {
                merged.appliedFrame = merged.frameIndex;
            }
            if (merged.failureStage == api::grab::InteractionFailure::None) {
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
        [[nodiscard]] bool isReserved(api::Hand hand) const noexcept
        {
            if (hand == api::Hand::None) {
                return _slots[0].commandId != 0 || _slots[1].commandId != 0;
            }
            const auto index = handIndex(hand);
            return index < _slots.size() && _slots[index].commandId != 0;
        }

        [[nodiscard]] bool reserve(api::Hand hand, std::uint64_t ownerToken, std::uint64_t commandId) noexcept
        {
            // Auto-hand inventory draws reserve both queue slots until ROCK
            // chooses a physically free hand. No other request can overtake it.
            if (hand == api::Hand::None) {
                if (ownerToken == 0 || commandId == 0 || isReserved(hand)) {
                    return false;
                }
                _slots.fill(ForceGrabReservation{ .ownerToken = ownerToken, .commandId = commandId });
                return true;
            }
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
        [[nodiscard]] static constexpr std::size_t handIndex(api::Hand hand) noexcept
        {
            return hand == api::Hand::Right ? 0u : hand == api::Hand::Left ? 1u : 2u;
        }

        std::array<ForceGrabReservation, 2> _slots{};
    };
}
