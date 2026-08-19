#pragma once

#include <array>
#include <cstddef>
#include <memory>

namespace rock::debug_overlay_snapshot
{
    // Single-producer pool for immutable snapshots published through shared_ptr.
    // The pool owns one reference to every allocated buffer. A use_count above
    // one means a buffer is still published or held by the compositor thread.
    template <class Snapshot, std::size_t Capacity>
    class SnapshotPool
    {
        static_assert(Capacity > 0);

    public:
        [[nodiscard]] std::shared_ptr<Snapshot> acquire()
        {
            for (std::size_t index = 0; index < _size; ++index) {
                if (_buffers[index].use_count() == 1) {
                    return _buffers[index];
                }
            }

            if (_size >= Capacity) {
                return {};
            }

            auto buffer = std::make_shared<Snapshot>();
            _buffers[_size++] = buffer;
            return buffer;
        }

        [[nodiscard]] std::size_t size() const noexcept { return _size; }
        [[nodiscard]] static constexpr std::size_t capacity() noexcept { return Capacity; }

    private:
        std::array<std::shared_ptr<Snapshot>, Capacity> _buffers{};
        std::size_t _size{ 0 };
    };
}
