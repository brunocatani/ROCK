#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace rock::weapon_inventory_stack_selection_policy
{
    constexpr std::size_t kMaximumObservedStacks = 64;

    struct StackWitness
    {
        std::uintptr_t stackAddress{ 0 };
        std::uintptr_t instanceDataAddress{ 0 };
        std::uint32_t count{ 0 };
    };

    struct Snapshot
    {
        std::array<StackWitness, kMaximumObservedStacks> stacks{};
        std::size_t count{ 0 };
        bool complete{ true };
    };

    enum class Evidence : std::uint8_t
    {
        None = 0,
        ExactInstance,
        NewStack,
        ExistingStackCountIncrease,
        SolePostTransferStack,
        AmbiguousMutation,
        SnapshotCapacityExceeded,
    };

    struct Selection
    {
        std::size_t postIndex{ 0 };
        std::size_t mutationCandidateCount{ 0 };
        Evidence evidence{ Evidence::None };
        bool found{ false };
    };

    [[nodiscard]] inline constexpr const char* evidenceName(const Evidence evidence) noexcept
    {
        switch (evidence) {
        case Evidence::ExactInstance:
            return "exact-instance";
        case Evidence::NewStack:
            return "new-stack";
        case Evidence::ExistingStackCountIncrease:
            return "count-increase";
        case Evidence::SolePostTransferStack:
            return "sole-stack";
        case Evidence::AmbiguousMutation:
            return "ambiguous-mutation";
        case Evidence::SnapshotCapacityExceeded:
            return "snapshot-capacity-exceeded";
        case Evidence::None:
        default:
            return "none";
        }
    }

    [[nodiscard]] inline constexpr Selection selectTransferredStack(
        const Snapshot& before,
        const Snapshot& after,
        const std::uintptr_t expectedInstanceData) noexcept
    {
        Selection selection{};

        if (expectedInstanceData != 0) {
            std::size_t exactMatchCount = 0;
            for (std::size_t index = 0; index < after.count; ++index) {
                if (after.stacks[index].instanceDataAddress == expectedInstanceData) {
                    selection.postIndex = index;
                    ++exactMatchCount;
                }
            }
            if (exactMatchCount == 1) {
                selection.found = true;
                selection.evidence = Evidence::ExactInstance;
                return selection;
            }
            if (exactMatchCount > 1) {
                selection.evidence = Evidence::AmbiguousMutation;
                selection.mutationCandidateCount = exactMatchCount;
                return selection;
            }
        }

        if (!before.complete || !after.complete) {
            selection.evidence = Evidence::SnapshotCapacityExceeded;
            return selection;
        }

        Evidence soleMutationEvidence = Evidence::None;
        for (std::size_t afterIndex = 0; afterIndex < after.count; ++afterIndex) {
            const auto& afterStack = after.stacks[afterIndex];
            const StackWitness* prior = nullptr;
            for (std::size_t beforeIndex = 0; beforeIndex < before.count; ++beforeIndex) {
                if (before.stacks[beforeIndex].stackAddress == afterStack.stackAddress) {
                    prior = &before.stacks[beforeIndex];
                    break;
                }
            }
            if (!prior && afterStack.instanceDataAddress != 0) {
                const StackWitness* instancePrior = nullptr;
                std::size_t instancePriorCount = 0;
                for (std::size_t beforeIndex = 0; beforeIndex < before.count; ++beforeIndex) {
                    if (before.stacks[beforeIndex].instanceDataAddress ==
                        afterStack.instanceDataAddress) {
                        instancePrior = &before.stacks[beforeIndex];
                        ++instancePriorCount;
                    }
                }
                if (instancePriorCount == 1) {
                    prior = instancePrior;
                }
            }

            Evidence mutationEvidence = Evidence::None;
            if (!prior) {
                mutationEvidence = Evidence::NewStack;
            } else if (afterStack.count > prior->count) {
                mutationEvidence = Evidence::ExistingStackCountIncrease;
            }

            if (mutationEvidence != Evidence::None) {
                selection.postIndex = afterIndex;
                soleMutationEvidence = mutationEvidence;
                ++selection.mutationCandidateCount;
            }
        }

        if (selection.mutationCandidateCount == 1) {
            selection.found = true;
            selection.evidence = soleMutationEvidence;
            return selection;
        }
        if (selection.mutationCandidateCount > 1) {
            selection.evidence = Evidence::AmbiguousMutation;
            return selection;
        }

        // ActivateRef is known to complete the inventory acquisition before
        // returning on the main thread. If there is only one stack for the
        // base form, it is unambiguous even when the engine rebuilt that stack
        // without leaving an address/count delta that ROCK can observe.
        if (after.count == 1) {
            selection.found = true;
            selection.postIndex = 0;
            selection.evidence = Evidence::SolePostTransferStack;
        }
        return selection;
    }
}
