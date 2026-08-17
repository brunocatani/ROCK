#define ROCK_API_EXPORTS
#include "physics-interaction/debug/GrabClockDebugFeed.h"

#include <mutex>

namespace rock::debug
{
    namespace
    {
        /*
         * One mutex per hand guards all three stage blocks. Writers are the
         * game thread (producer, pre-FRIK) and the physics thread (flush);
         * the only reader is the monitor's provider-frame callback on the
         * game thread. try_lock keeps every writer non-blocking; the reader
         * takes the lock outright, which can only contend with the physics
         * writer for the duration of a small POD copy.
         */
        struct HandFeedState
        {
            std::mutex mutex;
            std::uint64_t physicsWrites{ 0 };
            std::uint64_t producerWrites{ 0 };
            std::uint64_t preFrikWrites{ 0 };
            RockGrabClockStagePhysicsV1 physics{};
            RockGrabClockStageProducerV1 producer{};
            RockGrabClockStagePreFrikV1 preFrik{};
        };

        HandFeedState s_handFeeds[2]{};

        [[nodiscard]] HandFeedState& feedFor(const bool isLeft) noexcept
        {
            return s_handFeeds[isLeft ? 1 : 0];
        }
    }

    void publishGrabClockPhysicsStage(const bool isLeft, const RockGrabClockStagePhysicsV1& sample) noexcept
    {
        auto& feed = feedFor(isLeft);
        if (!feed.mutex.try_lock()) {
            return;
        }
        feed.physics = sample;
        feed.physics.writeCount = ++feed.physicsWrites;
        feed.mutex.unlock();
    }

    void publishGrabClockProducerStage(const bool isLeft, const RockGrabClockStageProducerV1& sample) noexcept
    {
        auto& feed = feedFor(isLeft);
        if (!feed.mutex.try_lock()) {
            return;
        }
        feed.producer = sample;
        feed.producer.writeCount = ++feed.producerWrites;
        feed.mutex.unlock();
    }

    void publishGrabClockPreFrikStage(const bool isLeft, const RockGrabClockStagePreFrikV1& sample) noexcept
    {
        auto& feed = feedFor(isLeft);
        if (!feed.mutex.try_lock()) {
            return;
        }
        feed.preFrik = sample;
        feed.preFrik.writeCount = ++feed.preFrikWrites;
        feed.mutex.unlock();
    }

    namespace
    {
        [[nodiscard]] bool copyGrabClockDebug(
            const std::uint32_t isLeft,
            RockGrabClockDebugHandV1* out)
        {
            if (!out ||
                out->structSize != sizeof(RockGrabClockDebugHandV1) ||
                out->version != ROCK_GRAB_CLOCK_DEBUG_VERSION ||
                isLeft > 1) {
                return false;
            }

            auto& feed = feedFor(isLeft != 0);
            std::scoped_lock lock(feed.mutex);
            out->physics = feed.physics;
            out->producer = feed.producer;
            out->preFrik = feed.preFrik;
            return true;
        }
    }
}

extern "C" __declspec(dllexport) bool __cdecl ROCKAPI_CopyGrabClockDebugV1(
    const std::uint32_t isLeft,
    rock::debug::RockGrabClockDebugHandV1* out)
{
    return rock::debug::copyGrabClockDebug(isLeft, out);
}
