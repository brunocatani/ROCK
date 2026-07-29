#include "physics-interaction/debug/DebugOverlayGpuTimer.h"

namespace rock::debug_overlay_gpu_timer
{
    void TimestampQueryRing::Scope::finish() noexcept
    {
        if (_owner && _context) {
            _owner->finish(_context, _slot);
        }
        _owner = nullptr;
        _context = nullptr;
        _slot = 0;
    }

    bool TimestampQueryRing::initialize(ID3D11Device* device) noexcept
    {
        reset();
        if (!device) {
            return false;
        }

        D3D11_QUERY_DESC disjointDesc{};
        disjointDesc.Query = D3D11_QUERY_TIMESTAMP_DISJOINT;
        D3D11_QUERY_DESC timestampDesc{};
        timestampDesc.Query = D3D11_QUERY_TIMESTAMP;
        for (auto& slot : _slots) {
            if (FAILED(device->CreateQuery(&disjointDesc, slot.disjoint.GetAddressOf())) ||
                FAILED(device->CreateQuery(&timestampDesc, slot.start.GetAddressOf())) ||
                FAILED(device->CreateQuery(&timestampDesc, slot.end.GetAddressOf()))) {
                reset();
                return false;
            }
        }

        _ready = true;
        return true;
    }

    void TimestampQueryRing::reset() noexcept
    {
        for (auto& slot : _slots) {
            slot.disjoint.Reset();
            slot.start.Reset();
            slot.end.Reset();
            slot.pending = false;
        }
        _nextIssue = 0;
        _nextPoll = 0;
        _latestMicroseconds = 0.0;
        _totalMicroseconds = 0.0;
        _issuedSamples = 0;
        _completedSamples = 0;
        _pendingPolls = 0;
        _skippedBegins = 0;
        _disjointSamples = 0;
        _failedPolls = 0;
        _ready = false;
    }

    void TimestampQueryRing::pollOne(ID3D11DeviceContext* context) noexcept
    {
        if (!_ready || !context) {
            return;
        }

        Slot* pending = nullptr;
        for (std::size_t offset = 0; offset < kSlotCount; ++offset) {
            const std::size_t index = (_nextPoll + offset) % kSlotCount;
            if (_slots[index].pending) {
                pending = &_slots[index];
                _nextPoll = (index + 1) % kSlotCount;
                break;
            }
        }
        if (!pending) {
            return;
        }

        constexpr UINT flags = D3D11_ASYNC_GETDATA_DONOTFLUSH;
        D3D11_QUERY_DATA_TIMESTAMP_DISJOINT disjoint{};
        const HRESULT disjointResult = context->GetData(pending->disjoint.Get(), &disjoint, sizeof(disjoint), flags);
        if (disjointResult == S_FALSE) {
            ++_pendingPolls;
            return;
        }
        if (disjointResult != S_OK) {
            pending->pending = false;
            ++_failedPolls;
            return;
        }

        UINT64 start = 0;
        UINT64 end = 0;
        const HRESULT startResult = context->GetData(pending->start.Get(), &start, sizeof(start), flags);
        const HRESULT endResult = context->GetData(pending->end.Get(), &end, sizeof(end), flags);
        if (startResult == S_FALSE || endResult == S_FALSE) {
            ++_pendingPolls;
            return;
        }
        if (startResult != S_OK || endResult != S_OK) {
            pending->pending = false;
            ++_failedPolls;
            return;
        }

        pending->pending = false;
        if (disjoint.Disjoint || disjoint.Frequency == 0 || end < start) {
            ++_disjointSamples;
            return;
        }

        _latestMicroseconds = static_cast<double>(end - start) * 1'000'000.0 / static_cast<double>(disjoint.Frequency);
        _totalMicroseconds += _latestMicroseconds;
        ++_completedSamples;
    }

    TimestampQueryRing::Scope TimestampQueryRing::begin(ID3D11DeviceContext* context) noexcept
    {
        if (!_ready || !context) {
            return {};
        }

        pollOne(context);
        for (std::size_t offset = 0; offset < kSlotCount; ++offset) {
            const std::size_t index = (_nextIssue + offset) % kSlotCount;
            auto& slot = _slots[index];
            if (slot.pending) {
                continue;
            }

            context->Begin(slot.disjoint.Get());
            context->End(slot.start.Get());
            slot.pending = true;
            _nextIssue = (index + 1) % kSlotCount;
            ++_issuedSamples;
            return Scope{ this, context, index };
        }

        ++_skippedBegins;
        return {};
    }

    void TimestampQueryRing::finish(ID3D11DeviceContext* context, std::size_t slot) noexcept
    {
        if (!_ready || !context || slot >= kSlotCount || !_slots[slot].pending) {
            return;
        }
        context->End(_slots[slot].end.Get());
        context->End(_slots[slot].disjoint.Get());
    }

    TimerStats TimestampQueryRing::stats() const noexcept
    {
        return TimerStats{
            _latestMicroseconds,
            _completedSamples == 0 ? 0.0 : _totalMicroseconds / static_cast<double>(_completedSamples),
            _issuedSamples,
            _completedSamples,
            _pendingPolls,
            _skippedBegins,
            _disjointSamples,
            _failedPolls
        };
    }
}
