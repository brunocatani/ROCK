#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace rock::nearby_grab_damping
{
    // One restore invocation owns this value-only index. Positive body IDs are
    // hints and must be revalidated immediately before use. Absence is usable
    // only by the call that just completed the scan, never by a later restore.
    class MotionBodySearchBatch
    {
    public:
        static constexpr std::uint32_t invalidBody = 0x7FFF'FFFF;
        struct Result { std::uint32_t bodyId = invalidBody; bool freshAbsence = false; };

        void add(std::uint32_t motion)
        {
            if (_built || motion == 0 || _count == _entries.size()) return;
            for (std::size_t i = 0; i < _count; ++i) if (_entries[i].motion == motion) return;
            _entries[_count++] = { motion, invalidBody };
        }

        template <class Enumerate, class Validate>
        Result find(std::uint32_t motion, Enumerate&& enumerate, Validate&& validate)
        {
            if (!_built) {
                if (std::none_of(_entries.begin(), _entries.begin() + _count,
                        [&](const Entry& entry) { return entry.motion == motion; })) return {};
                _built = true;
                std::sort(_entries.begin(), _entries.begin() + _count,
                    [](const Entry& a, const Entry& b) { return a.motion < b.motion; });
            }
            auto* requested = lookup(motion);
            if (!requested) return {}; // Capacity overflow uses the caller's native search.
            if (requested->body != invalidBody && validate(requested->body, motion))
                return { requested->body, false };
            requested->body = invalidBody;

            // Stop for this restore, not for unrelated leases with valid preferred
            // bodies. Other observed IDs are only opportunistic positive hints.
            const bool completed = enumerate([&](std::uint32_t candidateMotion, std::uint32_t body) {
                if (candidateMotion == motion) {
                    if (!validate(body, motion)) return false;
                    requested->body = body;
                    return true;
                }
                if (auto* entry = lookup(candidateMotion); entry && entry->body == invalidBody)
                    entry->body = body;
                return false;
            });
            if (requested->body != invalidBody) return { requested->body, false };
            return { invalidBody, completed }; // Never cache absence between native mutations.
        }

    private:
        struct Entry { std::uint32_t motion = 0, body = invalidBody; };
        Entry* lookup(std::uint32_t motion)
        {
            const auto end = _entries.begin() + _count;
            const auto it = std::lower_bound(_entries.begin(), end, motion,
                [](const Entry& e, std::uint32_t key) { return e.motion < key; });
            return it != end && it->motion == motion ? &*it : nullptr;
        }
        std::array<Entry, 128> _entries{};
        std::size_t _count = 0;
        bool _built = false;
    };
}
