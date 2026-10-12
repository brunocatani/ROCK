#pragma once

#include <array>
#include <cstdint>

namespace rock::queued_hand_input
{
    enum class Hand { None, Right, Left };
    struct Origin
    {
        Hand hand = Hand::None;
        std::uint64_t sequence = 0;
        std::uint32_t inputThread = 0;
        bool operator==(const Origin&) const = default;
    };

    // Private sidecar for native queue storage. Keys are non-owning numeric
    // identities and are never dereferenced. The native allocator invalidates
    // a reused entry; dispatch consumes once; movie destruction retires its
    // entries. Caller serializes access; no engine calls while holding it.
    class Records
    {
    public:
        static constexpr std::size_t capacity = 256;

        bool replace(std::uintptr_t entry, std::uintptr_t owner, Origin origin) noexcept
        {
            discard(entry);
            if (!entry || !owner || origin.hand == Hand::None || !origin.sequence) return false;
            for (auto& record : _records) {
                if (!record.entry) {
                    record = {entry, owner, origin};
                    return true;
                }
            }
            return false; // Never evict another live event to guess an owner.
        }

        Origin take(std::uintptr_t entry, std::uintptr_t owner) noexcept
        {
            for (auto& record : _records) {
                if (record.entry == entry && entry) {
                    const auto result = record.owner == owner ? record.origin : Origin{};
                    record = {};
                    return result;
                }
            }
            return {};
        }

        void discard(std::uintptr_t entry) noexcept
        {
            for (auto& record : _records) if (record.entry == entry) record = {};
        }

        void retire(std::uintptr_t owner) noexcept
        {
            for (auto& record : _records) if (record.owner == owner) record = {};
        }

    private:
        struct Record
        {
            std::uintptr_t entry = 0, owner = 0;
            Origin origin{};
        };
        std::array<Record, capacity> _records{};
    };
}
