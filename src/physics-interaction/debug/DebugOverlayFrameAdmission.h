#pragma once

#include <atomic>
#include <cstdint>
#include <utility>

namespace rock::debug_overlay_frame_admission
{
    class FrameAdmission
    {
    public:
        class Lease
        {
        public:
            Lease() noexcept = default;
            Lease(const Lease&) = delete;
            Lease& operator=(const Lease&) = delete;

            Lease(Lease&& other) noexcept : _owner(std::exchange(other._owner, nullptr)) {}

            Lease& operator=(Lease&& other) noexcept
            {
                if (this != &other) {
                    release();
                    _owner = std::exchange(other._owner, nullptr);
                }
                return *this;
            }

            ~Lease() { release(); }

            [[nodiscard]] explicit operator bool() const noexcept { return _owner != nullptr; }

        private:
            friend class FrameAdmission;

            explicit Lease(FrameAdmission* owner) noexcept : _owner(owner) {}

            void release() noexcept
            {
                if (_owner) {
                    _owner->finishFrame();
                    _owner = nullptr;
                }
            }

            FrameAdmission* _owner{ nullptr };
        };

        [[nodiscard]] std::uint64_t publish() noexcept
        {
            return _publishedSerial.fetch_add(1, std::memory_order_release) + 1;
        }

        [[nodiscard]] std::uint64_t publishedSerial() const noexcept
        {
            return _publishedSerial.load(std::memory_order_acquire);
        }

        [[nodiscard]] Lease tryAcquire() noexcept
        {
            if (_drawInProgress.test_and_set(std::memory_order_acquire)) {
                return {};
            }

            const auto published = _publishedSerial.load(std::memory_order_acquire);
            if (published == 0) {
                finishFrame();
                return {};
            }

            auto previouslyRendered = _lastRenderedSerial.load(std::memory_order_relaxed);
            if (previouslyRendered == published ||
                !_lastRenderedSerial.compare_exchange_strong(
                    previouslyRendered,
                    published,
                    std::memory_order_acq_rel,
                    std::memory_order_relaxed)) {
                finishFrame();
                return {};
            }

            return Lease{ this };
        }

    private:
        void finishFrame() noexcept { _drawInProgress.clear(std::memory_order_release); }

        std::atomic<std::uint64_t> _publishedSerial{ 0 };
        std::atomic<std::uint64_t> _lastRenderedSerial{ 0 };
        std::atomic_flag _drawInProgress = ATOMIC_FLAG_INIT;
    };
}
