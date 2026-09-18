#define NOMINMAX
#include <windows.h>

#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/performance/PerformanceProfiler.h"

#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <limits>

namespace
{
    bool profiling = true;
    std::array<std::uint64_t, 3> queries{};
    std::uint64_t apiFailures = 0;
    std::uint64_t rejected = 0;
    void reset() { queries = {}; apiFailures = 0; rejected = 0; }
}

namespace rock::performance_profiler
{
    // Exercise the real Windows range walker; substitute only its diagnostics
    // sink so this executable never loads the game or writes a production log.
    MemoryQuerySample beginMemoryQuery() noexcept
    {
        return { 0, Scope::UnattributedMemoryQueries, profiling };
    }
    void endMemoryQuery(MemoryQuerySample sample, MemoryQueryKind kind, bool succeeded) noexcept
    {
        if (!sample.active) return;
        ++queries[static_cast<std::size_t>(kind)];
        if (!succeeded) ++apiFailures;
    }
    void addCounter(Counter, std::uint64_t count) noexcept { if (profiling) rejected += count; }
}

int main()
{
    namespace memory = rock::native_memory;
    SYSTEM_INFO info{};
    GetSystemInfo(&info);
    const auto pageSize = static_cast<std::size_t>(info.dwPageSize);
    auto* pages = static_cast<std::uint8_t*>(VirtualAlloc(nullptr, pageSize * 3, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE));
    assert(pages);
    std::memset(pages, 0x5A, pageSize * 3);

    assert(memory::pointerLooksReadable(pages));
    assert(queries[0] == 0); // Address plausibility alone must never query pages.
    assert(memory::pointerRangeLooksReadable(pages, pageSize * 3));
    assert(queries[0] == 1); // One region, not one call per 4 KB page.

    reset();
    const std::uint32_t value = 0x12345678;
    assert(memory::tryWriteValue(reinterpret_cast<std::uint32_t*>(pages), value));
    std::uint32_t copied = 0;
    assert(memory::tryReadValue(reinterpret_cast<const std::uint32_t*>(pages), copied));
    assert(copied == value && queries[0] == 1 && queries[1] == 1 && rejected == 0);

    // The physics-system scanner now copies its IDs before visiting them.
    // Compare complete contents and actual OS-query counts with the old
    // range-check-plus-per-element-read pattern, including invalid-ID values.
    auto* bodyIds = reinterpret_cast<std::uint32_t*>(pages);
    std::array<std::uint32_t, 256> individualIds{};
    std::array<std::uint32_t, 256> batchIds{};
    for (std::size_t i = 0; i < individualIds.size(); ++i) {
        bodyIds[i] = i % 7 == 0 ? 0x7FFF'FFFFu : static_cast<std::uint32_t>(i * 3);
    }
    reset();
    assert(memory::pointerRangeLooksReadable(bodyIds, sizeof(individualIds)));
    for (std::size_t i = 0; i < individualIds.size(); ++i) {
        assert(memory::tryReadValue(bodyIds + i, individualIds[i]));
    }
    assert(queries[0] == 257);
    reset();
    assert(memory::guardedCopyFromMemory(bodyIds, batchIds.data(), sizeof(batchIds)));
    assert(batchIds == individualIds && queries[0] == 1 && rejected == 0);
    // Restore the fixture used by the remaining cross-page tests.
    std::memset(pages, 0x5A, pageSize);
    std::memcpy(pages, &value, sizeof(value));

    DWORD previous = 0;
    assert(VirtualProtect(pages + pageSize, pageSize, PAGE_READONLY, &previous));
    reset();
    std::array<std::uint8_t, 16> buffer{};
    assert(memory::guardedCopyFromMemory(pages + pageSize - 8, buffer.data(), buffer.size()));
    assert(queries[0] == 2 && buffer.front() == 0x5A && buffer.back() == 0x5A);
    assert(!memory::guardedCopyToMemory(pages + pageSize - 8, buffer.data(), buffer.size()));
    assert(queries[1] == 2 && rejected == 1 && apiFailures == 0);
    assert(pages[pageSize - 1] == 0x5A); // Refuse the entire copy before any write.

    assert(VirtualProtect(pages + pageSize * 2, pageSize, PAGE_NOACCESS, &previous));
    reset();
    assert(!memory::pointerRangeLooksReadable(pages + pageSize * 2, 1));
    assert(queries[0] == 1 && rejected == 1 && apiFailures == 0);
    batchIds.fill(0xABCDu);
    const auto untouchedIds = batchIds;
    assert(!memory::guardedCopyFromMemory(pages + pageSize * 2 - 16, batchIds.data(), sizeof(batchIds)));
    assert(batchIds == untouchedIds); // Reject the full scan before any IDs can be visited.
    assert(VirtualProtect(pages + pageSize * 2, pageSize, PAGE_READWRITE | PAGE_GUARD, &previous));
    assert(!memory::pointerRangeLooksReadable(pages + pageSize * 2, 1));
    MEMORY_BASIC_INFORMATION region{};
    assert(VirtualQuery(pages + pageSize * 2, &region, sizeof(region)) != 0);
    assert((region.Protect & PAGE_GUARD) != 0); // Validation must not consume the guard.

    reset();
    assert(!memory::pointerRangeLooksReadable(nullptr, 8));
    assert(!memory::pointerRangeLooksReadable(pages, 0));
    assert(!memory::pointerRangeLooksReadable(reinterpret_cast<void*>(0x10000), 1));
    assert(!memory::pointerRangeLooksReadable(reinterpret_cast<void*>((std::numeric_limits<std::uintptr_t>::max)() - 3), 8));
    assert(queries[0] == 0 && rejected == 4);

    reset();
    assert(!memory::pointerRangeLooksReadable(reinterpret_cast<void*>((std::numeric_limits<std::uintptr_t>::max)() - 4095), 8));
    assert(queries[0] == 1 && apiFailures == 1 && rejected == 1);

    reset();
    profiling = false;
    assert(memory::tryReadValue(reinterpret_cast<const std::uint32_t*>(pages), copied) && copied == value);
    assert(!memory::pointerRangeLooksWritable(pages + pageSize, 1));
    assert(queries[0] == 0 && queries[1] == 0 && rejected == 0);
    assert(VirtualFree(pages, 0, MEM_RELEASE));
    return 0;
}
