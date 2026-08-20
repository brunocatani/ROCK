#pragma once

#include "api/ROCKProviderApi.h"

#include <cstdint>

namespace rock::provider::detail
{
    enum class EntryThreadPolicy
    {
        AnyThread,
        AnimationOwner,
    };

    struct EntryValidationChecks
    {
        bool threadValid{ true };
        bool argumentPresent{ true };
        bool sizeValid{ true };
        bool versionValid{ true };
        bool semanticValid{ true };
    };

    [[nodiscard]] constexpr RockProviderResultV1 validateEntryOrder(
        const EntryValidationChecks& checks)
    {
        if (!checks.threadValid) {
            return RockProviderResultV1::WrongThread;
        }
        if (!checks.argumentPresent) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!checks.sizeValid) {
            return RockProviderResultV1::InvalidSize;
        }
        if (!checks.versionValid) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (!checks.semanticValid) {
            return RockProviderResultV1::InvalidArgument;
        }
        return RockProviderResultV1::Ok;
    }

    [[nodiscard]] bool entryThreadValid(EntryThreadPolicy policy);

    template <class Entry, class SemanticValidator>
    [[nodiscard]] RockProviderResultV1 validateEntry(
        const Entry* entry,
        const EntryThreadPolicy threadPolicy,
        SemanticValidator&& semanticValidator)
    {
        if (!entryThreadValid(threadPolicy)) {
            return RockProviderResultV1::WrongThread;
        }
        if (!entry) {
            return RockProviderResultV1::InvalidArgument;
        }

        const auto structuralResult = validateEntryOrder(
            EntryValidationChecks{
                .sizeValid = entry->size == sizeof(Entry),
                .versionValid = entry->version != 0 &&
                                entry->version <= ROCK_PROVIDER_API_VERSION,
            });
        if (structuralResult != RockProviderResultV1::Ok) {
            return structuralResult;
        }
        return semanticValidator(*entry) ?
            RockProviderResultV1::Ok :
            RockProviderResultV1::InvalidArgument;
    }

    template <class Entry>
    [[nodiscard]] RockProviderResultV1 validateEntry(
        const Entry* entry,
        const EntryThreadPolicy threadPolicy = EntryThreadPolicy::AnyThread)
    {
        return validateEntry(
            entry,
            threadPolicy,
            [](const Entry&) { return true; });
    }

    [[nodiscard]] inline RockProviderResultV1 validateOwnerEntry(
        const std::uint64_t ownerToken,
        const EntryThreadPolicy threadPolicy = EntryThreadPolicy::AnyThread)
    {
        return validateEntryOrder(EntryValidationChecks{
            .threadValid = entryThreadValid(threadPolicy),
            .semanticValid = ownerToken != 0,
        });
    }

    template <class Output>
    [[nodiscard]] RockProviderResultV1 validateOutputEntry(
        const std::uint64_t ownerToken,
        const Output* output,
        const EntryThreadPolicy threadPolicy = EntryThreadPolicy::AnyThread)
    {
        if (!entryThreadValid(threadPolicy)) {
            return RockProviderResultV1::WrongThread;
        }
        if (!output) {
            return RockProviderResultV1::InvalidArgument;
        }
        return validateEntryOrder(EntryValidationChecks{
            .sizeValid = output->size >= sizeof(Output),
            .semanticValid = ownerToken != 0,
        });
    }

    template <class Value>
    [[nodiscard]] RockProviderResultV1 validateArrayEntry(
        const std::uint64_t ownerToken,
        const Value* values,
        const std::uint32_t maxValues,
        const std::uint32_t* outValueCount,
        const EntryThreadPolicy threadPolicy = EntryThreadPolicy::AnyThread,
        const bool semanticValid = true)
    {
        return validateEntryOrder(EntryValidationChecks{
            .threadValid = entryThreadValid(threadPolicy),
            .argumentPresent = outValueCount != nullptr &&
                               (maxValues == 0 || values != nullptr),
            .semanticValid = ownerToken != 0 && semanticValid,
        });
    }
}
