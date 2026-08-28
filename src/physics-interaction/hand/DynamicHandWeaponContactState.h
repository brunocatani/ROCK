#pragma once

#include "physics-interaction/hand/DynamicHandCollisionTelemetry.h"

#include "RE/NetImmerse/NiPoint.h"

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace rock::dynamic_hand_weapon_contact_state
{
    inline constexpr std::size_t kHandCount = 2;
    inline constexpr std::size_t kSlotsPerHand =
        dynamic_hand_collision_telemetry::kBodiesPerHand;
    inline constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFFu;
    inline constexpr float kContactRetentionSeconds = 0.050f;

    struct Record
    {
        bool valid{ false };
        bool isLeft{ false };
        std::uint8_t handSlot{ 0 };
        std::uint32_t handBodyId{ kInvalidBodyId };
        std::uint32_t weaponProxyBodyId{ kInvalidBodyId };
        std::uint32_t weaponBodyId{ kInvalidBodyId };
        std::uint32_t framesSinceContact{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        RE::NiPoint3 contactPointGame{};
        RE::NiPoint3 contactNormalGame{};
        float secondsSinceContact{
            (std::numeric_limits<float>::infinity)()
        };
        bool hasContactNormal{ false };
    };

    struct Collection
    {
        std::array<Record, kSlotsPerHand> records{};
        std::size_t count{ 0 };

        void add(const Record& record) noexcept
        {
            if (record.valid && count < records.size()) {
                records[count++] = record;
            }
        }
    };

    /*
     * Each exact hand-compound child owns one non-blocking publication slot.
     * Repeating manifolds refresh the slot; the main frame consumes only fresh,
     * generation-bound records.
     */
    class State
    {
    public:
        void advanceFrame(const float validDeltaSeconds) noexcept
        {
            _frame.fetch_add(1, std::memory_order_acq_rel);
            if (std::isfinite(validDeltaSeconds) && validDeltaSeconds > 0.0f) {
                const double now =
                    _elapsedSeconds.load(std::memory_order_acquire) +
                    validDeltaSeconds;
                _elapsedSeconds.store(now, std::memory_order_release);
            }
        }

        void clear() noexcept
        {
            _epoch.fetch_add(1, std::memory_order_acq_rel);
        }

        [[nodiscard]] bool record(
            const bool isLeft,
            const std::uint8_t handSlot,
            const std::uint32_t handBodyId,
            const std::uint32_t weaponProxyBodyId,
            const std::uint32_t weaponBodyId,
            const std::uint64_t weaponGenerationKey,
            const RE::NiPoint3& contactPointGame,
            const RE::NiPoint3* const contactNormalGame) noexcept
        {
            if (handSlot >= kSlotsPerHand ||
                handBodyId == kInvalidBodyId ||
                weaponProxyBodyId == kInvalidBodyId ||
                weaponBodyId == kInvalidBodyId ||
                weaponGenerationKey == 0 ||
                !finite(contactPointGame)) {
                return false;
            }

            auto& slot = _slots[isLeft ? 1u : 0u][handSlot];
            if (slot.writer.test_and_set(std::memory_order_acquire)) {
                return false;
            }

            const bool hasNormal =
                contactNormalGame && finite(*contactNormalGame) &&
                lengthSquared(*contactNormalGame) > 1.0e-6f;
            const RE::NiPoint3 normal =
                hasNormal ? *contactNormalGame : RE::NiPoint3{};
            std::uint32_t sequence =
                slot.sequence.load(std::memory_order_relaxed);
            if ((sequence & 1u) != 0) {
                ++sequence;
            }
            slot.sequence.store(sequence + 1, std::memory_order_release);
            slot.valid.store(0, std::memory_order_release);
            slot.epoch.store(
                _epoch.load(std::memory_order_acquire),
                std::memory_order_relaxed);
            slot.frame.store(
                _frame.load(std::memory_order_acquire),
                std::memory_order_relaxed);
            slot.seconds.store(
                _elapsedSeconds.load(std::memory_order_acquire),
                std::memory_order_relaxed);
            slot.handBodyId.store(handBodyId, std::memory_order_relaxed);
            slot.weaponProxyBodyId.store(
                weaponProxyBodyId,
                std::memory_order_relaxed);
            slot.weaponBodyId.store(weaponBodyId, std::memory_order_relaxed);
            slot.weaponGenerationKey.store(
                weaponGenerationKey,
                std::memory_order_relaxed);
            slot.pointX.store(contactPointGame.x, std::memory_order_relaxed);
            slot.pointY.store(contactPointGame.y, std::memory_order_relaxed);
            slot.pointZ.store(contactPointGame.z, std::memory_order_relaxed);
            slot.normalX.store(normal.x, std::memory_order_relaxed);
            slot.normalY.store(normal.y, std::memory_order_relaxed);
            slot.normalZ.store(normal.z, std::memory_order_relaxed);
            slot.hasNormal.store(
                hasNormal ? 1u : 0u,
                std::memory_order_relaxed);
            slot.valid.store(1, std::memory_order_release);
            slot.sequence.store(sequence + 2, std::memory_order_release);
            slot.writer.clear(std::memory_order_release);
            return true;
        }

        [[nodiscard]] Collection collectFresh(
            const bool isLeft,
            const std::uint32_t maximumAgeFrames,
            const float maximumAgeSeconds) const noexcept
        {
            Collection result{};
            const std::uint64_t currentEpoch =
                _epoch.load(std::memory_order_acquire);
            const std::uint32_t currentFrame =
                _frame.load(std::memory_order_acquire);
            const double nowSeconds =
                _elapsedSeconds.load(std::memory_order_acquire);

            for (std::size_t handSlot = 0;
                 handSlot < kSlotsPerHand;
                 ++handSlot) {
                const auto& slot = _slots[isLeft ? 1u : 0u][handSlot];
                for (int attempt = 0; attempt < 3; ++attempt) {
                    const std::uint32_t sequenceBefore =
                        slot.sequence.load(std::memory_order_acquire);
                    if ((sequenceBefore & 1u) != 0) {
                        continue;
                    }
                    if (slot.valid.load(std::memory_order_acquire) == 0) {
                        break;
                    }

                    const std::uint64_t contactEpoch =
                        slot.epoch.load(std::memory_order_relaxed);
                    const std::uint32_t contactFrame =
                        slot.frame.load(std::memory_order_relaxed);
                    const double contactSeconds =
                        slot.seconds.load(std::memory_order_relaxed);
                    Record record{};
                    record.valid = true;
                    record.isLeft = isLeft;
                    record.handSlot = static_cast<std::uint8_t>(handSlot);
                    record.handBodyId =
                        slot.handBodyId.load(std::memory_order_relaxed);
                    record.weaponProxyBodyId =
                        slot.weaponProxyBodyId.load(
                            std::memory_order_relaxed);
                    record.weaponBodyId =
                        slot.weaponBodyId.load(std::memory_order_relaxed);
                    record.weaponGenerationKey =
                        slot.weaponGenerationKey.load(
                            std::memory_order_relaxed);
                    record.framesSinceContact = currentFrame - contactFrame;
                    record.secondsSinceContact = static_cast<float>(
                        nowSeconds >= contactSeconds ?
                            nowSeconds - contactSeconds :
                            0.0);
                    record.contactPointGame = RE::NiPoint3{
                        slot.pointX.load(std::memory_order_relaxed),
                        slot.pointY.load(std::memory_order_relaxed),
                        slot.pointZ.load(std::memory_order_relaxed),
                    };
                    record.hasContactNormal =
                        slot.hasNormal.load(std::memory_order_relaxed) != 0;
                    record.contactNormalGame = RE::NiPoint3{
                        slot.normalX.load(std::memory_order_relaxed),
                        slot.normalY.load(std::memory_order_relaxed),
                        slot.normalZ.load(std::memory_order_relaxed),
                    };

                    const std::uint32_t sequenceAfter =
                        slot.sequence.load(std::memory_order_acquire);
                    if (sequenceBefore != sequenceAfter ||
                        (sequenceAfter & 1u) != 0) {
                        continue;
                    }
                    if (contactEpoch != currentEpoch ||
                        record.framesSinceContact > maximumAgeFrames ||
                        record.secondsSinceContact > maximumAgeSeconds ||
                        !finite(record.contactPointGame)) {
                        break;
                    }
                    if (record.hasContactNormal &&
                        (!finite(record.contactNormalGame) ||
                            lengthSquared(record.contactNormalGame) <=
                                1.0e-6f)) {
                        record.hasContactNormal = false;
                    }
                    result.add(record);
                    break;
                }
            }
            return result;
        }

    private:
        struct AtomicSlot
        {
            std::atomic_flag writer = ATOMIC_FLAG_INIT;
            std::atomic<std::uint32_t> sequence{ 0 };
            std::atomic<std::uint32_t> valid{ 0 };
            std::atomic<std::uint64_t> epoch{ 0 };
            std::atomic<std::uint32_t> frame{ 0 };
            std::atomic<double> seconds{ 0.0 };
            std::atomic<std::uint32_t> handBodyId{ kInvalidBodyId };
            std::atomic<std::uint32_t> weaponProxyBodyId{ kInvalidBodyId };
            std::atomic<std::uint32_t> weaponBodyId{ kInvalidBodyId };
            std::atomic<std::uint64_t> weaponGenerationKey{ 0 };
            std::atomic<float> pointX{ 0.0f };
            std::atomic<float> pointY{ 0.0f };
            std::atomic<float> pointZ{ 0.0f };
            std::atomic<float> normalX{ 0.0f };
            std::atomic<float> normalY{ 0.0f };
            std::atomic<float> normalZ{ 0.0f };
            std::atomic<std::uint32_t> hasNormal{ 0 };
        };

        [[nodiscard]] static bool finite(
            const RE::NiPoint3& value) noexcept
        {
            return std::isfinite(value.x) &&
                   std::isfinite(value.y) &&
                   std::isfinite(value.z);
        }

        [[nodiscard]] static float lengthSquared(
            const RE::NiPoint3& value) noexcept
        {
            return value.x * value.x +
                   value.y * value.y +
                   value.z * value.z;
        }

        std::array<std::array<AtomicSlot, kSlotsPerHand>, kHandCount>
            _slots{};
        std::atomic<std::uint64_t> _epoch{ 1 };
        std::atomic<std::uint32_t> _frame{ 0 };
        std::atomic<double> _elapsedSeconds{ 0.0 };
    };
}
