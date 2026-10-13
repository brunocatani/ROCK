#pragma once

#include <atomic>
#include <cstdint>

namespace rock::hand_touch
{
    struct Touch
    {
        std::uint32_t source = 0xFFFFFFFF;
        std::uint32_t target = 0xFFFFFFFF;
        std::uintptr_t world = 0;
    };

    // Native contact callbacks publish a hand's latest touch of a non-world
    // object; the game-frame contact resolver consumes it into the hand's
    // touch state. Single bounded attempt: contention drops one sample.
    // The sequence keeps source, target and world coherent.
    class TouchChannel
    {
    public:
        bool publish(const Touch& touch) noexcept
        {
            auto sequence = _sequence.load();
            if ((sequence & 1u) || !_sequence.compare_exchange_strong(sequence, sequence + 1)) return false;
            _source.store(touch.source);
            _target.store(touch.target);
            _world.store(touch.world);
            _sequence.store(sequence + 2);
            return true;
        }

        bool consume(Touch& touch) noexcept
        {
            const auto before = _sequence.load();
            if ((before & 1u) || before == _consumed) return false;
            touch = {_source.load(), _target.load(), _world.load()};
            if (before != _sequence.load()) return false;
            _consumed = before;
            return touch.source != 0xFFFFFFFF && touch.target != 0xFFFFFFFF;
        }

        void clear() noexcept { _consumed = _sequence.load(); }

    private:
        std::atomic<std::uint32_t> _sequence{0}, _source{0xFFFFFFFF}, _target{0xFFFFFFFF};
        std::atomic<std::uintptr_t> _world{0};
        std::uint32_t _consumed = 0; // game thread only
    };
}
