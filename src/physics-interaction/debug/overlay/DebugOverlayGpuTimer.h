#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>

#include <d3d11.h>
#include <wrl/client.h>

namespace rock::debug_overlay_gpu_timer
{
    struct TimerStats
    {
        double latestMicroseconds{ 0.0 };
        double averageMicroseconds{ 0.0 };
        std::uint64_t issuedSamples{ 0 };
        std::uint64_t completedSamples{ 0 };
        std::uint64_t pendingPolls{ 0 };
        std::uint64_t skippedBegins{ 0 };
        std::uint64_t disjointSamples{ 0 };
        std::uint64_t failedPolls{ 0 };
    };

    class TimestampQueryRing
    {
    public:
        static constexpr std::size_t kSlotCount = 4;

        class Scope
        {
        public:
            Scope() noexcept = default;
            Scope(const Scope&) = delete;
            Scope& operator=(const Scope&) = delete;
            Scope(Scope&& other) noexcept :
                _owner(std::exchange(other._owner, nullptr)),
                _context(std::exchange(other._context, nullptr)),
                _slot(std::exchange(other._slot, 0))
            {}
            Scope& operator=(Scope&& other) noexcept
            {
                if (this != &other) {
                    finish();
                    _owner = std::exchange(other._owner, nullptr);
                    _context = std::exchange(other._context, nullptr);
                    _slot = std::exchange(other._slot, 0);
                }
                return *this;
            }
            ~Scope() noexcept { finish(); }

            [[nodiscard]] explicit operator bool() const noexcept { return _owner != nullptr; }

        private:
            friend class TimestampQueryRing;

            Scope(TimestampQueryRing* owner, ID3D11DeviceContext* context, std::size_t slot) noexcept :
                _owner(owner), _context(context), _slot(slot)
            {}

            void finish() noexcept;

            TimestampQueryRing* _owner{ nullptr };
            ID3D11DeviceContext* _context{ nullptr };
            std::size_t _slot{ 0 };
        };

        TimestampQueryRing() = default;
        TimestampQueryRing(const TimestampQueryRing&) = delete;
        TimestampQueryRing& operator=(const TimestampQueryRing&) = delete;
        TimestampQueryRing(TimestampQueryRing&&) noexcept = default;
        TimestampQueryRing& operator=(TimestampQueryRing&&) noexcept = default;

        [[nodiscard]] bool initialize(ID3D11Device* device) noexcept;
        void reset() noexcept;
        [[nodiscard]] Scope begin(ID3D11DeviceContext* context) noexcept;
        [[nodiscard]] TimerStats stats() const noexcept;

    private:
        struct Slot
        {
            Microsoft::WRL::ComPtr<ID3D11Query> disjoint;
            Microsoft::WRL::ComPtr<ID3D11Query> start;
            Microsoft::WRL::ComPtr<ID3D11Query> end;
            bool pending{ false };
        };

        void pollOne(ID3D11DeviceContext* context) noexcept;
        void finish(ID3D11DeviceContext* context, std::size_t slot) noexcept;

        std::array<Slot, kSlotCount> _slots{};
        std::size_t _nextIssue{ 0 };
        std::size_t _nextPoll{ 0 };
        double _latestMicroseconds{ 0.0 };
        double _totalMicroseconds{ 0.0 };
        std::uint64_t _issuedSamples{ 0 };
        std::uint64_t _completedSamples{ 0 };
        std::uint64_t _pendingPolls{ 0 };
        std::uint64_t _skippedBegins{ 0 };
        std::uint64_t _disjointSamples{ 0 };
        std::uint64_t _failedPolls{ 0 };
        bool _ready{ false };
    };
}
