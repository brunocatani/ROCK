#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace rock::havok_runtime
{
    // Owned by one synchronous, read-locked object scan. No cache entry may
    // survive that transaction or any native world mutation. Header identities
    // are re-read at every wrapper; only the copied body-ID list is reused.
    class PhysicsSystemBodyScanCache
    {
    public:
        struct Key
        {
            const void* system = nullptr;
            const void* instance = nullptr;
            const void* world = nullptr;
            const void* bodyArray = nullptr;
            std::int32_t bodyCount = 0;
            bool operator==(const Key&) const = default;
        };

        [[nodiscard]] std::span<const std::uint32_t> find(const Key& key, std::size_t count) const
        {
            for (std::size_t i = 0; i < _count; ++i) {
                if (_entries[i].key == key && _entries[i].ids.size() >= count)
                    return std::span<const std::uint32_t>(_entries[i].ids).first(count);
            }
            return {};
        }

        void remember(const Key& key, std::span<const std::uint32_t> ids)
        {
            if (ids.empty() || ids.size() > 4096) return;
            for (std::size_t i = 0; i < _count; ++i) {
                if (_entries[i].key == key) {
                    if (_entries[i].ids.size() < ids.size()) _entries[i].ids.assign(ids.begin(), ids.end());
                    return;
                }
            }
            // Capacity limits memoization only; the caller still scans every system.
            if (_count == _entries.size()) return;
            auto& entry = _entries[_count++];
            entry.key = key;
            entry.ids.assign(ids.begin(), ids.end());
        }

    private:
        struct Entry { Key key; std::vector<std::uint32_t> ids; };
        std::array<Entry, 32> _entries{};
        std::size_t _count = 0;
    };
}
