#pragma once
#include <cstdint>

namespace rock::equip_pickup
{
    struct Intent { std::uint32_t reference{},count{}; };
    inline thread_local Intent current;
    class Scope {
        Intent previous;
    public:
        Scope(std::uint32_t reference,std::uint32_t count) noexcept : previous(current) { current={reference,count}; }
        ~Scope() { current=previous; }
        Scope(const Scope&)=delete;
        Scope& operator=(const Scope&)=delete;
    };
    // Weapon isEquipPickup: the exact reference and count ROCK is acquiring
    // for an equip on this thread.
    inline bool matches(std::uint32_t reference,std::uint32_t count) noexcept
    { return reference && count && current.reference==reference && current.count==count; }
}
