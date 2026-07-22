#include "api/ProviderDebugOverlayRuntime.h"
#include "api/ProviderLeasePolicy.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>

namespace rock::provider_debug_overlay
{
    namespace
    {
        using namespace provider;

        struct PublisherSlot
        {
            std::uint64_t ownerToken{ 0 };
            std::array<RockProviderDebugOverlayLineV1,
                ROCK_PROVIDER_MAX_DEBUG_OVERLAY_LINES_PER_PUBLISHER_V1>
                lines{};
            std::array<RockProviderDebugOverlayTextV1,
                ROCK_PROVIDER_MAX_DEBUG_OVERLAY_TEXT_PER_PUBLISHER_V1>
                textEntries{};
            std::uint32_t lineCount{ 0 };
            std::uint32_t textCount{ 0 };
            std::uint64_t expiresAfterFrame{ 0 };
            std::uint32_t worldGeneration{ 0 };
            std::uint32_t skeletonGeneration{ 0 };
            std::uint32_t providerGeneration{ 0 };
        };

        std::array<PublisherSlot,
            ROCK_PROVIDER_MAX_DEBUG_OVERLAY_PUBLISHERS_V1>
            s_publishers{};
        std::mutex s_publisherMutex;
        std::atomic_bool s_hasContent{ false };

        [[nodiscard]] bool finitePoint(const float (&point)[3])
        {
            return std::isfinite(point[0]) &&
                   std::isfinite(point[1]) &&
                   std::isfinite(point[2]);
        }

        [[nodiscard]] bool validColor(const float (&color)[4])
        {
            for (const float component : color) {
                if (!std::isfinite(component) ||
                    component < 0.0f || component > 1.0f) {
                    return false;
                }
            }
            return true;
        }

        [[nodiscard]] bool validLine(
            const RockProviderDebugOverlayLineV1& line)
        {
            return line.size == sizeof(RockProviderDebugOverlayLineV1) &&
                   line.version > 0 &&
                   line.version <= ROCK_PROVIDER_API_VERSION &&
                   finitePoint(line.startGame) &&
                   finitePoint(line.endGame) &&
                   validColor(line.color);
        }

        [[nodiscard]] bool validText(
            const RockProviderDebugOverlayTextV1& entry)
        {
            constexpr std::uint32_t kImplementedFlags =
                static_cast<std::uint32_t>(
                    RockProviderDebugOverlayTextFlagV1::WorldAnchored);
            if (entry.size != sizeof(RockProviderDebugOverlayTextV1) ||
                entry.version == 0 ||
                entry.version > ROCK_PROVIDER_API_VERSION ||
                (entry.flags & ~kImplementedFlags) != 0 ||
                !std::memchr(
                    entry.text,
                    '\0',
                    ROCK_PROVIDER_DEBUG_OVERLAY_TEXT_CAPACITY_V1) ||
                !std::isfinite(entry.x) ||
                !std::isfinite(entry.y) ||
                !std::isfinite(entry.textSize) ||
                entry.textSize < 0.5f || entry.textSize > 8.0f ||
                !validColor(entry.color)) {
                return false;
            }
            const bool worldAnchored =
                (entry.flags & static_cast<std::uint32_t>(
                    RockProviderDebugOverlayTextFlagV1::WorldAnchored)) != 0;
            return !worldAnchored || finitePoint(entry.worldAnchorGame);
        }

        [[nodiscard]] PublisherSlot* findPublisherSlot(
            const std::uint64_t ownerToken)
        {
            PublisherSlot* available = nullptr;
            for (auto& slot : s_publishers) {
                if (slot.ownerToken == ownerToken) {
                    return std::addressof(slot);
                }
                if (slot.ownerToken == 0 && !available) {
                    available = std::addressof(slot);
                }
            }
            return available;
        }

        void publishContentStateLocked()
        {
            const bool hasContent = std::any_of(
                s_publishers.begin(),
                s_publishers.end(),
                [](const PublisherSlot& slot) {
                    return slot.lineCount > 0 || slot.textCount > 0;
                });
            s_hasContent.store(hasContent, std::memory_order_release);
        }
    }

    provider::RockProviderResultV1 publish(
        const std::uint64_t ownerToken,
        const provider::RockProviderDebugOverlayPublicationV1& publication,
        const std::uint64_t frameIndex)
    {
        using namespace provider;
        if (ownerToken == 0 ||
            publication.lineCount >
                ROCK_PROVIDER_MAX_DEBUG_OVERLAY_LINES_PER_PUBLISHER_V1 ||
            publication.textCount >
                ROCK_PROVIDER_MAX_DEBUG_OVERLAY_TEXT_PER_PUBLISHER_V1 ||
            publication.leaseFrames == 0 ||
            (publication.lineCount > 0 && !publication.lines) ||
            (publication.textCount > 0 && !publication.textEntries)) {
            return RockProviderResultV1::InvalidArgument;
        }
        for (std::uint32_t index = 0;
             index < publication.lineCount;
             ++index) {
            if (!validLine(publication.lines[index])) {
                return RockProviderResultV1::InvalidArgument;
            }
        }
        for (std::uint32_t index = 0;
             index < publication.textCount;
             ++index) {
            if (!validText(publication.textEntries[index])) {
                return RockProviderResultV1::InvalidArgument;
            }
        }

        std::scoped_lock lock(s_publisherMutex);
        auto* slot = findPublisherSlot(ownerToken);
        if (!slot) {
            return RockProviderResultV1::CapacityFull;
        }
        *slot = {};
        if (publication.lineCount == 0 && publication.textCount == 0) {
            publishContentStateLocked();
            return RockProviderResultV1::Ok;
        }
        slot->ownerToken = ownerToken;
        slot->lineCount = publication.lineCount;
        slot->textCount = publication.textCount;
        const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
            publication.leaseFrames,
            ROCK_PROVIDER_MAX_DEBUG_OVERLAY_PUBLICATION_LEASE_FRAMES_V1);
        slot->expiresAfterFrame = provider_lease_policy::exclusiveExpiryFrame(
            frameIndex,
            leaseFrames);
        slot->worldGeneration = publication.worldGeneration;
        slot->skeletonGeneration = publication.skeletonGeneration;
        slot->providerGeneration = publication.providerGeneration;
        if (publication.lineCount > 0) {
            std::copy_n(
                publication.lines,
                publication.lineCount,
                slot->lines.begin());
        }
        if (publication.textCount > 0) {
            std::copy_n(
                publication.textEntries,
                publication.textCount,
                slot->textEntries.begin());
        }
        publishContentStateLocked();
        return RockProviderResultV1::Ok;
    }

    void prune(
        const std::uint64_t frameIndex,
        const std::uint32_t worldGeneration,
        const std::uint32_t skeletonGeneration,
        const std::uint32_t providerGeneration,
        PruneResult& outResult)
    {
        outResult = {};
        std::scoped_lock lock(s_publisherMutex);
        for (auto& slot : s_publishers) {
            if (slot.ownerToken == 0) {
                continue;
            }
            const bool generationChanged =
                (slot.worldGeneration != 0 &&
                    slot.worldGeneration != worldGeneration) ||
                (slot.skeletonGeneration != 0 &&
                    slot.skeletonGeneration != skeletonGeneration) ||
                (slot.providerGeneration != 0 &&
                    slot.providerGeneration != providerGeneration);
            if (!generationChanged && provider_lease_policy::isActive(
                    frameIndex,
                    slot.expiresAfterFrame)) {
                continue;
            }

            auto& invalidated = outResult.publishers[outResult.count++];
            invalidated.ownerToken = slot.ownerToken;
            invalidated.reason = generationChanged ?
                RockProviderSuppressionInvalidationReasonV1::GenerationChanged :
                RockProviderSuppressionInvalidationReasonV1::Expired;
            slot = {};
        }
        if (outResult.count != 0) {
            publishContentStateLocked();
        }
    }

    void clear(const std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return;
        }
        std::scoped_lock lock(s_publisherMutex);
        for (auto& slot : s_publishers) {
            if (slot.ownerToken == ownerToken) {
                slot = {};
                publishContentStateLocked();
                return;
            }
        }
    }

    void clearAll(
        PruneResult& outResult,
        const RockProviderSuppressionInvalidationReasonV1 reason)
    {
        outResult = {};
        std::scoped_lock lock(s_publisherMutex);
        for (const auto& slot : s_publishers) {
            if (slot.ownerToken == 0) {
                continue;
            }
            auto& invalidated = outResult.publishers[outResult.count++];
            invalidated.ownerToken = slot.ownerToken;
            invalidated.reason = reason;
        }
        s_publishers = {};
        s_hasContent.store(false, std::memory_order_release);
    }

    void clearAll()
    {
        PruneResult ignored{};
        clearAll(
            ignored,
            RockProviderSuppressionInvalidationReasonV1::ExplicitClear);
    }

    bool hasContent()
    {
        return s_hasContent.load(std::memory_order_acquire);
    }

    void copySnapshot(Snapshot& outSnapshot)
    {
        // Counts own validity. Avoid clearing the fixed backing arrays every
        // frame; only the published prefixes are copied and observed.
        outSnapshot.lineCount = 0;
        outSnapshot.textCount = 0;
        std::scoped_lock lock(s_publisherMutex);
        for (const auto& slot : s_publishers) {
            const auto availableLines =
                static_cast<std::uint32_t>(outSnapshot.lines.size()) -
                outSnapshot.lineCount;
            const auto linesToCopy = (std::min)(
                slot.lineCount,
                availableLines);
            if (linesToCopy > 0) {
                std::copy_n(
                    slot.lines.begin(),
                    linesToCopy,
                    outSnapshot.lines.begin() + outSnapshot.lineCount);
            }
            outSnapshot.lineCount += linesToCopy;

            const auto availableText =
                static_cast<std::uint32_t>(outSnapshot.textEntries.size()) -
                outSnapshot.textCount;
            const auto textToCopy = (std::min)(
                slot.textCount,
                availableText);
            if (textToCopy > 0) {
                std::copy_n(
                    slot.textEntries.begin(),
                    textToCopy,
                    outSnapshot.textEntries.begin() + outSnapshot.textCount);
            }
            outSnapshot.textCount += textToCopy;
        }
    }
}
