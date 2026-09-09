#pragma once

#include <array>
#include <atomic>
#include <cstdint>

namespace rock::push_assist
{
    struct Contact
    {
        std::uint32_t source = 0xFFFFFFFF;
        std::uint32_t target = 0xFFFFFFFF;
        std::uintptr_t world = 0;
        std::uintptr_t owner = 0;
        std::array<float, 3> point{};
        bool hasPoint = false;
    };

    // Native contact callbacks publish; the game-frame contact resolver consumes.
    // Single bounded attempt: contention drops assist, leaving native collision intact.
    // Atomic payload and sequence keep source, target, identity and point coherent.
    class ContactChannel
    {
    public:
        bool publish(const Contact& contact) noexcept
        {
            auto sequence = _sequence.load();
            if ((sequence & 1u) || !_sequence.compare_exchange_strong(sequence, sequence + 1)) return false;
            _source.store(contact.source);
            _target.store(contact.target);
            _world.store(contact.world);
            _owner.store(contact.owner);
            for (int i = 0; i < 3; ++i) _point[i].store(contact.point[i]);
            _hasPoint.store(contact.hasPoint);
            _sequence.store(sequence + 2);
            return true;
        }

        bool consume(Contact& contact) noexcept
        {
            const auto before = _sequence.load();
            if ((before & 1u) || before == _consumed) return false;
            contact = {_source.load(), _target.load(), _world.load(), _owner.load(),
                {_point[0].load(), _point[1].load(), _point[2].load()}, _hasPoint.load()};
            if (before != _sequence.load()) return false;
            _consumed = before;
            return contact.source != 0xFFFFFFFF && contact.target != 0xFFFFFFFF;
        }

        void clear() noexcept { _consumed = _sequence.load(); }

    private:
        std::atomic<std::uint32_t> _sequence{0}, _source{0xFFFFFFFF}, _target{0xFFFFFFFF};
        std::atomic<std::uintptr_t> _world{0}, _owner{0};
        std::array<std::atomic<float>, 3> _point{};
        std::atomic<bool> _hasPoint{false};
        std::uint32_t _consumed = 0; // game thread only
    };
}
