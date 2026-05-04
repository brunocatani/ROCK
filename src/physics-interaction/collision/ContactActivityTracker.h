#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>

namespace frik::rock::contact_activity_tracker
{
    /*
     * ROCK's semantic hand contacts answer "which generated hand part touched
     * this body recently"; this tracker answers the separate HIGGS-style
     * question "is this body pair a continuing contact or a newly active one?"
     * Keeping that state in a fixed slot table gives the hot contact callback a
     * stable, allocation-free path. Existing body-pair updates do not take the
     * insertion mutex; the mutex is only used when claiming or recycling slots.
     */

    inline constexpr std::uint32_t kInvalidBodyId = 0xFFFF'FFFFu;
    inline constexpr std::uint32_t kInvalidBodyIdLegacy = 0x7FFF'FFFFu;
    inline constexpr std::uint32_t kActiveContactFrames = 15;
    inline constexpr std::uint32_t kCleanupContactFrames = 100;
    inline constexpr std::size_t kMaxTrackedContactsPerHand = 512;
    inline constexpr std::uint64_t kFreeContactActivityKey = 0;
    inline constexpr std::uint64_t kReservedContactActivityKey = (std::numeric_limits<std::uint64_t>::max)();
    inline constexpr std::uint32_t kInvalidContactActivityFrame = (std::numeric_limits<std::uint32_t>::max)();

    struct ContactRegistrationResult
    {
        bool tracked = false;
        bool newlyActive = false;
        bool inserted = false;
        bool evictedStale = false;
        std::uint32_t frame = 0;
    };

    class ContactActivityTracker
    {
    public:
        void reset()
        {
            _frame.store(0, std::memory_order_release);
            clearBank(_right);
            clearBank(_left);
        }

        std::uint32_t advanceFrame()
        {
            const auto frame = _frame.fetch_add(1, std::memory_order_acq_rel) + 1;
            pruneStale(_right, frame);
            pruneStale(_left, frame);
            return frame;
        }

        std::uint32_t currentFrame() const
        {
            return _frame.load(std::memory_order_acquire);
        }

        ContactRegistrationResult registerHandContact(bool isLeft, std::uint32_t handBodyId, std::uint32_t targetBodyId)
        {
            if (!isTrackableBodyPair(handBodyId, targetBodyId)) {
                return {};
            }

            const auto frame = currentFrame();
            const auto key = makeBodyPairKey(handBodyId, targetBodyId);
            auto& bank = isLeft ? _left : _right;
            auto updateExistingSlot = [frame, key](auto* slot, ContactRegistrationResult& out) {
                const auto previousFrame = slot->lastFrame.exchange(frame, std::memory_order_acq_rel);
                if (slot->key.load(std::memory_order_acquire) != key) {
                    return false;
                }

                out = ContactRegistrationResult{
                    .tracked = true,
                    .newlyActive = !isFrameWithinActiveWindow(frame, previousFrame),
                    .inserted = false,
                    .evictedStale = false,
                    .frame = frame,
                };
                return true;
            };

            if (auto* slot = findSlot(bank, key)) {
                ContactRegistrationResult result{};
                if (updateExistingSlot(slot, result)) {
                    return result;
                }
            }

            std::scoped_lock lock(bank.insertMutex);
            if (auto* slot = findSlot(bank, key)) {
                ContactRegistrationResult result{};
                if (updateExistingSlot(slot, result)) {
                    return result;
                }
            }

            bool evictedStale = false;
            auto* slot = findClaimableSlot(bank, frame, evictedStale);
            if (!slot) {
                return ContactRegistrationResult{ .frame = frame };
            }

            slot->sourceBodyId.store(handBodyId, std::memory_order_relaxed);
            slot->targetBodyId.store(targetBodyId, std::memory_order_relaxed);
            slot->lastFrame.store(frame, std::memory_order_release);
            slot->key.store(key, std::memory_order_release);
            return ContactRegistrationResult{
                .tracked = true,
                .newlyActive = true,
                .inserted = true,
                .evictedStale = evictedStale,
                .frame = frame,
            };
        }

        bool isHandContactActive(bool isLeft, std::uint32_t handBodyId, std::uint32_t targetBodyId) const
        {
            const auto* slot = findSlot(isLeft ? _left : _right, makeBodyPairKey(handBodyId, targetBodyId));
            return slot && isFrameWithinActiveWindow(currentFrame(), slot->lastFrame.load(std::memory_order_acquire));
        }

        bool isHandContactTracked(bool isLeft, std::uint32_t handBodyId, std::uint32_t targetBodyId) const
        {
            return findSlot(isLeft ? _left : _right, makeBodyPairKey(handBodyId, targetBodyId)) != nullptr;
        }

        std::size_t trackedCount(bool isLeft) const
        {
            std::size_t count = 0;
            const auto& bank = isLeft ? _left : _right;
            for (const auto& slot : bank.slots) {
                const auto key = slot.key.load(std::memory_order_acquire);
                if (key != kFreeContactActivityKey && key != kReservedContactActivityKey) {
                    ++count;
                }
            }
            return count;
        }

    private:
        struct Slot
        {
            std::atomic<std::uint64_t> key{ kFreeContactActivityKey };
            std::atomic<std::uint32_t> sourceBodyId{ kInvalidBodyId };
            std::atomic<std::uint32_t> targetBodyId{ kInvalidBodyId };
            std::atomic<std::uint32_t> lastFrame{ kInvalidContactActivityFrame };
        };

        struct Bank
        {
            std::array<Slot, kMaxTrackedContactsPerHand> slots{};
            std::mutex insertMutex;
        };

        static constexpr bool isTrackableBodyId(std::uint32_t bodyId)
        {
            return bodyId != kInvalidBodyId && bodyId != kInvalidBodyIdLegacy;
        }

        static constexpr bool isTrackableBodyPair(std::uint32_t sourceBodyId, std::uint32_t targetBodyId)
        {
            return isTrackableBodyId(sourceBodyId) && isTrackableBodyId(targetBodyId) && sourceBodyId != targetBodyId;
        }

        static constexpr std::uint64_t makeBodyPairKey(std::uint32_t sourceBodyId, std::uint32_t targetBodyId)
        {
            return (static_cast<std::uint64_t>(sourceBodyId) << 32) | static_cast<std::uint64_t>(targetBodyId);
        }

        static constexpr std::uint32_t frameAge(std::uint32_t frame, std::uint32_t lastFrame)
        {
            if (lastFrame == kInvalidContactActivityFrame) {
                return kInvalidContactActivityFrame;
            }
            return frame - lastFrame;
        }

        static constexpr bool isFrameWithinActiveWindow(std::uint32_t frame, std::uint32_t lastFrame)
        {
            return frameAge(frame, lastFrame) <= kActiveContactFrames;
        }

        static constexpr bool isFrameBeyondCleanupWindow(std::uint32_t frame, std::uint32_t lastFrame)
        {
            return frameAge(frame, lastFrame) > kCleanupContactFrames;
        }

        static Slot* findSlot(Bank& bank, std::uint64_t key)
        {
            if (key == kFreeContactActivityKey || key == kReservedContactActivityKey) {
                return nullptr;
            }

            for (auto& slot : bank.slots) {
                if (slot.key.load(std::memory_order_acquire) == key) {
                    return &slot;
                }
            }
            return nullptr;
        }

        static const Slot* findSlot(const Bank& bank, std::uint64_t key)
        {
            if (key == kFreeContactActivityKey || key == kReservedContactActivityKey) {
                return nullptr;
            }

            for (const auto& slot : bank.slots) {
                if (slot.key.load(std::memory_order_acquire) == key) {
                    return &slot;
                }
            }
            return nullptr;
        }

        static void clearSlot(Slot& slot)
        {
            slot.sourceBodyId.store(kInvalidBodyId, std::memory_order_relaxed);
            slot.targetBodyId.store(kInvalidBodyId, std::memory_order_relaxed);
            slot.lastFrame.store(kInvalidContactActivityFrame, std::memory_order_release);
            slot.key.store(kFreeContactActivityKey, std::memory_order_release);
        }

        static void clearBank(Bank& bank)
        {
            std::scoped_lock lock(bank.insertMutex);
            for (auto& slot : bank.slots) {
                clearSlot(slot);
            }
        }

        static Slot* findClaimableSlot(Bank& bank, std::uint32_t frame, bool& evictedStale)
        {
            for (auto& slot : bank.slots) {
                if (slot.key.load(std::memory_order_acquire) == kFreeContactActivityKey) {
                    evictedStale = false;
                    return &slot;
                }
            }

            for (auto& slot : bank.slots) {
                auto key = slot.key.load(std::memory_order_acquire);
                if (key == kFreeContactActivityKey || key == kReservedContactActivityKey) {
                    continue;
                }

                const auto lastFrame = slot.lastFrame.load(std::memory_order_acquire);
                if (!isFrameBeyondCleanupWindow(frame, lastFrame)) {
                    continue;
                }

                auto expected = key;
                if (!slot.key.compare_exchange_strong(expected, kReservedContactActivityKey, std::memory_order_acq_rel)) {
                    continue;
                }

                if (isFrameBeyondCleanupWindow(frame, slot.lastFrame.load(std::memory_order_acquire))) {
                    slot.sourceBodyId.store(kInvalidBodyId, std::memory_order_relaxed);
                    slot.targetBodyId.store(kInvalidBodyId, std::memory_order_relaxed);
                    slot.lastFrame.store(kInvalidContactActivityFrame, std::memory_order_release);
                    evictedStale = true;
                    return &slot;
                }

                slot.key.store(key, std::memory_order_release);
            }

            return nullptr;
        }

        static void pruneStale(Bank& bank, std::uint32_t frame)
        {
            std::scoped_lock lock(bank.insertMutex);
            for (auto& slot : bank.slots) {
                auto key = slot.key.load(std::memory_order_acquire);
                if (key == kFreeContactActivityKey || key == kReservedContactActivityKey) {
                    continue;
                }

                if (!isFrameBeyondCleanupWindow(frame, slot.lastFrame.load(std::memory_order_acquire))) {
                    continue;
                }

                auto expected = key;
                if (!slot.key.compare_exchange_strong(expected, kReservedContactActivityKey, std::memory_order_acq_rel)) {
                    continue;
                }

                if (isFrameBeyondCleanupWindow(frame, slot.lastFrame.load(std::memory_order_acquire))) {
                    clearSlot(slot);
                } else {
                    slot.key.store(key, std::memory_order_release);
                }
            }
        }

        std::atomic<std::uint32_t> _frame{ 0 };
        Bank _right{};
        Bank _left{};
    };
}
