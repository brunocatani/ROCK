#include "api/ProviderDebugOverlayRuntime.h"

#include <array>
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstring>

int main()
{
    using namespace rock;
    using namespace rock::provider;

    provider_debug_overlay::clearAll();
    assert(!provider_debug_overlay::hasContent());

    RockProviderDebugOverlayLineV1 line{};
    line.startGame[0] = 1.0f;
    line.startGame[1] = 2.0f;
    line.startGame[2] = 3.0f;
    line.endGame[0] = 4.0f;
    line.endGame[1] = 5.0f;
    line.endGame[2] = 6.0f;
    line.color[0] = 0.25f;
    line.color[1] = 0.50f;
    line.color[2] = 0.75f;

    RockProviderDebugOverlayTextV1 text{};
    std::memcpy(text.text, "overlay", sizeof("overlay"));
    text.textSize = 2.0f;

    RockProviderDebugOverlayPublicationV1 publication{};
    publication.leaseFrames = 2;
    publication.lines = &line;
    publication.lineCount = 1;
    publication.textEntries = &text;
    publication.textCount = 1;
    assert(provider_debug_overlay::publish(1, publication, 10) ==
           RockProviderResultV1::Ok);
    assert(provider_debug_overlay::hasContent());

    provider_debug_overlay::Snapshot snapshot{};
    provider_debug_overlay::copySnapshot(snapshot);
    assert(snapshot.lineCount == 1);
    assert(snapshot.textCount == 1);
    assert(snapshot.lines[0].startGame[1] == 2.0f);
    assert(std::strcmp(snapshot.textEntries[0].text, "overlay") == 0);

    line.color[0] = 1.5f;
    assert(provider_debug_overlay::publish(1, publication, 10) ==
           RockProviderResultV1::InvalidArgument);
    provider_debug_overlay::copySnapshot(snapshot);
    assert(snapshot.lineCount == 1);
    assert(snapshot.lines[0].color[0] == 0.25f);
    line.color[0] = 0.25f;

    publication.lineCount =
        ROCK_PROVIDER_MAX_DEBUG_OVERLAY_LINES_PER_PUBLISHER_V1 + 1;
    assert(provider_debug_overlay::publish(1, publication, 10) ==
           RockProviderResultV1::InvalidArgument);
    publication.lineCount = 1;

    RockProviderDebugOverlayTextV1 unterminatedText{};
    std::memset(
        unterminatedText.text,
        'x',
        sizeof(unterminatedText.text));
    publication.lineCount = 0;
    publication.lines = nullptr;
    publication.textEntries = &unterminatedText;
    publication.textCount = 1;
    assert(provider_debug_overlay::publish(2, publication, 10) ==
           RockProviderResultV1::InvalidArgument);

    publication.textEntries = nullptr;
    publication.textCount = 0;
    assert(provider_debug_overlay::publish(1, publication, 10) ==
           RockProviderResultV1::Ok);
    provider_debug_overlay::copySnapshot(snapshot);
    assert(snapshot.lineCount == 0);
    assert(snapshot.textCount == 0);

    std::array<RockProviderDebugOverlayLineV1,
        ROCK_PROVIDER_MAX_DEBUG_OVERLAY_LINES_PER_PUBLISHER_V1>
        fullLines{};
    for (std::size_t index = 0; index < fullLines.size(); ++index) {
        fullLines[index].startGame[0] = static_cast<float>(index);
        fullLines[index].endGame[0] = static_cast<float>(index + 1);
    }
    publication.lines = fullLines.data();
    publication.lineCount = static_cast<std::uint32_t>(fullLines.size());
    for (std::uint64_t owner = 1;
         owner <= ROCK_PROVIDER_MAX_DEBUG_OVERLAY_PUBLISHERS_V1;
         ++owner) {
        assert(provider_debug_overlay::publish(owner, publication, 20) ==
               RockProviderResultV1::Ok);
    }
    assert(provider_debug_overlay::publish(
               ROCK_PROVIDER_MAX_DEBUG_OVERLAY_PUBLISHERS_V1 + 1,
               publication,
               20) == RockProviderResultV1::CapacityFull);

    provider_debug_overlay::copySnapshot(snapshot);
    assert(snapshot.lineCount == ROCK_PROVIDER_MAX_DEBUG_OVERLAY_LINES_V1);
    provider_debug_overlay::clear(1);
    provider_debug_overlay::copySnapshot(snapshot);
    assert(snapshot.lineCount == ROCK_PROVIDER_MAX_DEBUG_OVERLAY_LINES_V1);
    provider_debug_overlay::clearAll();
    assert(!provider_debug_overlay::hasContent());
    provider_debug_overlay::copySnapshot(snapshot);
    assert(snapshot.lineCount == 0);
    assert(snapshot.textCount == 0);

    publication.lines = &line;
    publication.lineCount = 1;
    publication.leaseFrames = 2;
    publication.worldGeneration = 1;
    assert(provider_debug_overlay::publish(50, publication, 100) ==
           RockProviderResultV1::Ok);
    provider_debug_overlay::PruneResult pruneResult{};
    provider_debug_overlay::prune(101, 1, 0, 0, pruneResult);
    assert(pruneResult.count == 0);
    assert(provider_debug_overlay::hasContent());
    provider_debug_overlay::prune(102, 1, 0, 0, pruneResult);
    assert(pruneResult.count == 1);
    assert(pruneResult.publishers[0].ownerToken == 50);
    assert(pruneResult.publishers[0].reason ==
           RockProviderSuppressionInvalidationReasonV1::Expired);
    assert(!provider_debug_overlay::hasContent());

    publication.leaseFrames =
        ROCK_PROVIDER_MAX_DEBUG_OVERLAY_PUBLICATION_LEASE_FRAMES_V1 + 1;
    assert(provider_debug_overlay::publish(51, publication, 200) ==
           RockProviderResultV1::Ok);
    provider_debug_overlay::prune(
        200 + ROCK_PROVIDER_MAX_DEBUG_OVERLAY_PUBLICATION_LEASE_FRAMES_V1 - 1,
        1,
        0,
        0,
        pruneResult);
    assert(pruneResult.count == 0);
    provider_debug_overlay::prune(
        200 + ROCK_PROVIDER_MAX_DEBUG_OVERLAY_PUBLICATION_LEASE_FRAMES_V1,
        1,
        0,
        0,
        pruneResult);
    assert(pruneResult.count == 1);
    assert(pruneResult.publishers[0].reason ==
           RockProviderSuppressionInvalidationReasonV1::Expired);

    publication.leaseFrames = 2;
    assert(provider_debug_overlay::publish(52, publication, 300) ==
           RockProviderResultV1::Ok);
    provider_debug_overlay::prune(300, 2, 0, 0, pruneResult);
    assert(pruneResult.count == 1);
    assert(pruneResult.publishers[0].ownerToken == 52);
    assert(pruneResult.publishers[0].reason ==
           RockProviderSuppressionInvalidationReasonV1::GenerationChanged);
    assert(!provider_debug_overlay::hasContent());

    return 0;
}
