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
