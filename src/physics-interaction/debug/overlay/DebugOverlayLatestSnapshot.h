#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace rock::debug_overlay_snapshot
{
    // Single-writer, multi-reader fixed snapshot exchange for runtime callbacks
    // that cannot allocate or wait on a mutex. The writer never mutates the
    // active slot or a slot retained by a reader. A reader increments the slot
    // count, then rechecks the active index before exposing the immutable value.
    template <class Snapshot, std::size_t Capacity>
    class LatestSnapshot
    {
        static_assert(Capacity >= 2);
        static_assert(Capacity <= 0xFF);
        static_assert(std::atomic<std::uint32_t>::is_always_lock_free);
        static_assert(std::atomic<std::uint64_t>::is_always_lock_free);

        static constexpr std::uint32_t kInvalidIndex = 0xFFFF'FFFFu;
        static constexpr std::uint64_t kIndexMask = 0xFFu;

        struct Slot
        {
            std::atomic<std::uint32_t> readers{ 0 };
            Snapshot snapshot{};
        };

    public:
        class ReadLease
        {
        public:
            ReadLease() noexcept = default;
            ReadLease(const ReadLease&) = delete;
            ReadLease& operator=(const ReadLease&) = delete;

            ReadLease(ReadLease&& other) noexcept :
                _owner(std::exchange(other._owner, nullptr)),
                _index(std::exchange(other._index, kInvalidIndex))
            {}

            ReadLease& operator=(ReadLease&& other) noexcept
            {
                if (this != &other) {
                    release();
                    _owner = std::exchange(other._owner, nullptr);
                    _index = std::exchange(other._index, kInvalidIndex);
                }
                return *this;
            }

            ~ReadLease() { release(); }

            [[nodiscard]] explicit operator bool() const noexcept
            {
                return _owner != nullptr && _index != kInvalidIndex;
            }

            [[nodiscard]] const Snapshot* get() const noexcept
            {
                return *this ? &_owner->_slots[_index].snapshot : nullptr;
            }

            [[nodiscard]] const Snapshot& operator*() const noexcept { return *get(); }
            [[nodiscard]] const Snapshot* operator->() const noexcept { return get(); }

        private:
            friend class LatestSnapshot;

            ReadLease(const LatestSnapshot* owner, std::uint32_t index) noexcept :
                _owner(owner),
                _index(index)
            {}

            void release() noexcept
            {
                if (_owner && _index != kInvalidIndex) {
                    _owner->_slots[_index].readers.fetch_sub(
                        1,
                        std::memory_order_release);
                }
                _owner = nullptr;
                _index = kInvalidIndex;
            }

            const LatestSnapshot* _owner{ nullptr };
            std::uint32_t _index{ kInvalidIndex };
        };

        class WriteLease
        {
        public:
            WriteLease() noexcept = default;
            WriteLease(const WriteLease&) = delete;
            WriteLease& operator=(const WriteLease&) = delete;

            WriteLease(WriteLease&& other) noexcept :
                _owner(std::exchange(other._owner, nullptr)),
                _index(std::exchange(other._index, kInvalidIndex))
            {}

            WriteLease& operator=(WriteLease&& other) noexcept
            {
                if (this != &other) {
                    _owner = std::exchange(other._owner, nullptr);
                    _index = std::exchange(other._index, kInvalidIndex);
                }
                return *this;
            }

            [[nodiscard]] explicit operator bool() const noexcept
            {
                return _owner != nullptr && _index != kInvalidIndex;
            }

            [[nodiscard]] Snapshot* get() noexcept
            {
                return *this ? &_owner->_slots[_index].snapshot : nullptr;
            }

            [[nodiscard]] Snapshot& operator*() noexcept { return *get(); }
            [[nodiscard]] Snapshot* operator->() noexcept { return get(); }

            void publish() noexcept
            {
                if (!*this) {
                    return;
                }
                _owner->publishIndex(_index);
                _owner = nullptr;
                _index = kInvalidIndex;
            }

        private:
            friend class LatestSnapshot;

            WriteLease(LatestSnapshot* owner, std::uint32_t index) noexcept :
                _owner(owner),
                _index(index)
            {}

            LatestSnapshot* _owner{ nullptr };
            std::uint32_t _index{ kInvalidIndex };
        };

        [[nodiscard]] WriteLease tryBeginWrite() noexcept
        {
            const auto active = activeIndex(
                _activeToken.load(std::memory_order_acquire));
            for (std::uint32_t index = 0; index < Capacity; ++index) {
                if (index != active &&
                    _slots[index].readers.load(std::memory_order_acquire) == 0) {
                    return WriteLease{ this, index };
                }
            }
            return {};
        }

        [[nodiscard]] ReadLease tryAcquire() const noexcept
        {
            for (std::size_t attempt = 0; attempt < Capacity; ++attempt) {
                const auto token = _activeToken.load(std::memory_order_acquire);
                const auto index = activeIndex(token);
                if (index >= Capacity) {
                    return {};
                }

                _slots[index].readers.fetch_add(1, std::memory_order_acquire);
                // Compare the full publication token, not only the slot. This
                // closes the ABA window where rapid writes cycle back to the
                // same index while a reader is between observation and retain.
                if (_activeToken.load(std::memory_order_acquire) == token) {
                    return ReadLease{ this, index };
                }
                _slots[index].readers.fetch_sub(1, std::memory_order_release);
            }
            return {};
        }

        void clear() noexcept
        {
            _activeToken.store(0, std::memory_order_release);
        }

    private:
        [[nodiscard]] static std::uint32_t activeIndex(
            std::uint64_t token) noexcept
        {
            if (token == 0) {
                return kInvalidIndex;
            }
            const auto encodedIndex =
                static_cast<std::uint32_t>(token & kIndexMask);
            return encodedIndex == 0 ? kInvalidIndex : encodedIndex - 1;
        }

        void publishIndex(std::uint32_t index) noexcept
        {
            auto sequence =
                _nextPublicationSequence.fetch_add(
                    1,
                    std::memory_order_relaxed) +
                1;
            if (sequence == 0) {
                sequence =
                    _nextPublicationSequence.fetch_add(
                        1,
                        std::memory_order_relaxed) +
                    1;
            }
            const auto token =
                (sequence << 8) | static_cast<std::uint64_t>(index + 1);
            _activeToken.store(token, std::memory_order_release);
        }

        mutable std::array<Slot, Capacity> _slots{};
        std::atomic<std::uint64_t> _activeToken{ 0 };
        std::atomic<std::uint64_t> _nextPublicationSequence{ 0 };
    };
}
