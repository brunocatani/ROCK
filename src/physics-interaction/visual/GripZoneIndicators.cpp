#include "physics-interaction/visual/GripZoneIndicators.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/visual/WorldOverlayRenderer.h"
#include "physics-interaction/weapon/GripZoneIndicatorPolicy.h"
#include "rock_support/ImmutableSnapshotPool.h"

namespace rock::grip_zone_indicators
{
    namespace
    {
        immutable_snapshot::SnapshotPool<Frame, 4> frames;
        std::atomic<std::shared_ptr<const Frame>> published;
        std::atomic<std::uint64_t> currentFrame{ 0 };
        bool exhaustionReported{ false }; // Publication owner only.
    }

    void EndFrame(const std::uint64_t gameFrameIndex) noexcept
    {
        if (currentFrame.load(std::memory_order_acquire) != gameFrameIndex) Clear();
    }

    std::shared_ptr<const Frame> Snapshot() noexcept
    {
        auto frame = published.load(std::memory_order_acquire);
        if (!frame || !grip_zone_indicator_policy::isCurrentRenderFrame(
                frame->gameFrameIndex, currentFrame.load(std::memory_order_acquire))) {
            return {};
        }
        return frame;
    }

    void Clear() noexcept
    {
        currentFrame.store(0, std::memory_order_release);
        if (published.exchange({}, std::memory_order_acq_rel)) {
            world_overlay_renderer::NotifyPublication();
        }
    }

    void Publish(const Frame& frame)
    {
        currentFrame.store(frame.gameFrameIndex, std::memory_order_release);
        const auto count = (std::min)(frame.count, static_cast<std::uint32_t>(frame.positions.size()));
        if (frame.gameFrameIndex == 0 || count == 0 ||
            !std::isfinite(frame.diameterGameUnits) ||
            frame.diameterGameUnits < grip_zone_indicator_policy::kMinimumDiameterGameUnits ||
            frame.diameterGameUnits > grip_zone_indicator_policy::kMaximumDiameterGameUnits) {
            Clear();
            return;
        }

        auto next = frames.acquire();
        if (!next) {
            Clear();
            if (!exhaustionReported) {
                ROCK_LOG_WARN(Weapon, "Grip indicators: snapshot pool exhausted; current marker frame skipped");
                exhaustionReported = true;
            }
            return;
        }
        *next = {};
        next->gameFrameIndex = frame.gameFrameIndex;
        next->diameterGameUnits = frame.diameterGameUnits;
        for (std::uint32_t index = 0; index < count; ++index) {
            const auto& position = frame.positions[index];
            if (std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z)) {
                next->positions[next->count++] = position;
            }
        }
        if (next->count == 0) {
            Clear();
            return;
        }

        (void)world_overlay_renderer::EnsureInstalled();
        std::shared_ptr<const Frame> immutable = std::move(next);
        published.store(std::move(immutable), std::memory_order_release);
        exhaustionReported = false;
        world_overlay_renderer::NotifyPublication();
    }
}
