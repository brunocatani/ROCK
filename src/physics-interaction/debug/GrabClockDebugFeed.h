#pragma once

#include <cstdint>

/*
 * GrabClock debug feed (diagnostic, 2026-08-17 grab locomotion stutter):
 * in-memory, per-frame publication of the ANCHOR_CLOCK probe samples so the
 * embedded ROCK Monitor panel can render them in real time without any API
 * hop. The log-file probe stays authoritative for offline analysis; this
 * feed exists so the developer can visually correlate which channel moves
 * with the perceived stutter while playing.
 */
namespace rock::debug
{

    // Sampled on the physics thread at the +0x30 grab-authority flush
    // (first substep only). Room/held-node values are read from game-thread
    // node state at that moment; distances are in game units, -1 = unknown.
    struct RockGrabClockStagePhysicsV1
    {
        std::uint64_t writeCount{ 0 }; // monotonic; 0 = never published
        float roomPos[3]{};
        float roomYawDegrees{ 0.0f };
        std::uint32_t roomValid{ 0 };
        float heldNodePos[3]{};
        std::uint32_t heldNodeValid{ 0 };
        float heldVsLastWriteGu{ -1.0f };
    };

    // Sampled on the game thread inside Hand::updateHeldObject (producer),
    // after the anchor/body blend decision and after the node-ownership
    // write for this frame was either applied or skipped.
    struct RockGrabClockStageProducerV1
    {
        std::uint64_t writeCount{ 0 };
        std::uint64_t schedulerSequence{ 0 };
        float deltaSeconds{ 0.0f };
        float roomPos[3]{};
        float roomYawDegrees{ 0.0f };
        std::uint32_t roomValid{ 0 };
        float rawHandPos[3]{};
        float heldEntryPos[3]{};      // held node world before ROCK's write
        std::uint32_t heldEntryValid{ 0 };
        float entryVsLastWriteGu{ -1.0f }; // engine stomp meter
        float entryVsBodyGu{ -1.0f };      // what the node was stomped TO
        float bodyPos[3]{};           // physics-clock candidate
        std::uint32_t bodyValid{ 0 };
        float anchorPos[3]{};         // blended visual anchor actually used
        float bodyBlend{ 1.0f };      // 0 = pure render clock, 1 = pure body
        std::uint32_t anchorEngaged{ 0 };
        std::uint32_t nodeWriteApplied{ 0 };
    };

    // Sampled on the game thread in refreshGrabVisualAuthorityBeforeFrik,
    // immediately before the FRIK hand override is republished from
    // heldNode->world.
    struct RockGrabClockStagePreFrikV1
    {
        std::uint64_t writeCount{ 0 };
        std::uint64_t schedulerSequence{ 0 };
        float roomPos[3]{};
        float roomYawDegrees{ 0.0f };
        std::uint32_t roomValid{ 0 };
        float rawHandPos[3]{};
        std::uint32_t rawHandValid{ 0 };
        float rawVsProducerGu{ -1.0f };  // skeleton refresh between producer and FRIK
        float roomVsProducerGu{ -1.0f }; // root refresh between producer and FRIK
        float heldNodePos[3]{};
        float heldVsLastWriteGu{ -1.0f };
        float republishedHandPos[3]{};
    };

    struct RockGrabClockDebugHandV1
    {
        RockGrabClockStagePhysicsV1 physics{};
        RockGrabClockStageProducerV1 producer{};
        RockGrabClockStagePreFrikV1 preFrik{};
    };

    /*
     * Internal publishers, one per probe site. Writers use try_lock so the
     * physics thread can never block on the monitor's reader: a skipped
     * sample under contention is a lost display frame, not a correctness
     * problem. writeCount is assigned by the feed, not the caller.
     */
    void publishGrabClockPhysicsStage(bool isLeft, const RockGrabClockStagePhysicsV1& sample) noexcept;
    void publishGrabClockProducerStage(bool isLeft, const RockGrabClockStageProducerV1& sample) noexcept;
    void publishGrabClockPreFrikStage(bool isLeft, const RockGrabClockStagePreFrikV1& sample) noexcept;

    // Copies the latest published stage blocks for one hand into `out` for
    // the embedded monitor panel. Blocks are individually consistent;
    // cross-block skew of one frame is possible and acceptable for display.
    void copyGrabClockDebug(bool isLeft, RockGrabClockDebugHandV1& out) noexcept;
}
