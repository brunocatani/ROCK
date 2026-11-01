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
    std::array<std::uint64_t, 2> queries{};
    std::uint64_t rejected = 0;
    std::uint64_t guardedReads = 0;
    std::uint_least32_t lastReadLine = 0;
    void reset() { queries = {}; rejected = 0; guardedReads = 0; lastReadLine = 0; }
}

namespace rock::performance_profiler
{
    // Exercise the real page probes and SEH copy against Windows pages; substitute
    // only the diagnostics sink so this executable never loads the game or writes a log.
    MemoryQuerySample beginMemoryQuery() noexcept
    {
        return { 0, Scope::UnattributedMemoryQueries, profiling };
    }
    void endMemoryQuery(MemoryQuerySample sample, MemoryQueryKind kind) noexcept
    {
        if (!sample.active) return;
        ++queries[static_cast<std::size_t>(kind)];
    }
    void addCounter(Counter, std::uint64_t count) noexcept { if (profiling) rejected += count; }
    void countGuardedRead(const std::source_location& site) noexcept
    {
        if (!profiling) return;
        ++guardedReads;
        lastReadLine = site.line();
    }
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
    assert(queries[0] == 1); // One check per call, however many pages it probes.

    // A write probe faults on read-only pages but never changes a byte.
    assert(memory::pointerRangeLooksWritable(pages + 1, pageSize * 3 - 1));
    for (std::size_t i = 0; i < pageSize * 3; ++i) {
        assert(pages[i] == 0x5A);
    }

    // Reserved but uncommitted memory (how allocator pools hold address space) is refused.
    auto* reserved = static_cast<std::uint8_t*>(VirtualAlloc(nullptr, pageSize, MEM_RESERVE, PAGE_READWRITE));
    assert(reserved);
    assert(!memory::pointerRangeLooksReadable(reserved, 1));
    assert(!memory::pointerRangeLooksWritable(reserved, 1));
    assert(VirtualFree(reserved, 0, MEM_RELEASE));

    // Guarded reads copy under SEH and never query page protection.
    reset();
    const std::uint32_t value = 0x12345678;
    assert(memory::tryWriteValue(reinterpret_cast<std::uint32_t*>(pages), value));
    std::uint32_t copied = 0;
    assert(memory::tryReadValue(reinterpret_cast<const std::uint32_t*>(pages), copied));
    assert(copied == value && queries[0] == 0 && queries[1] == 1 && rejected == 0 && guardedReads == 1);

    // The profiler attributes a read to its caller's line, through the wrappers.
    reset();
    const auto readLine = static_cast<std::uint_least32_t>(__LINE__ + 1);
    assert(memory::tryReadField(pages, 0, copied));
    assert(guardedReads == 1 && lastReadLine == readLine);

    std::array<std::uint32_t, 256> ids{};
    auto* sourceIds = reinterpret_cast<std::uint32_t*>(pages);
    for (std::size_t i = 0; i < ids.size(); ++i) {
        sourceIds[i] = static_cast<std::uint32_t>(i * 3);
    }
    reset();
    assert(memory::guardedCopyFromMemory(sourceIds, ids.data(), sizeof(ids)));
    for (std::size_t i = 0; i < ids.size(); ++i) {
        assert(ids[i] == i * 3);
    }
    assert(queries[0] == 0 && rejected == 0);

    // Restore the fixture used by the remaining cross-page tests.
    std::memset(pages, 0x5A, pageSize * 3);
    std::memcpy(pages, &value, sizeof(value));

    DWORD previous = 0;
    assert(VirtualProtect(pages + pageSize, pageSize, PAGE_READONLY, &previous));
    reset();
    std::array<std::uint8_t, 16> buffer{};
    assert(memory::guardedCopyFromMemory(pages + pageSize - 8, buffer.data(), buffer.size()));
    assert(buffer.front() == 0x5A && buffer.back() == 0x5A);
    std::array<std::uint8_t, 16> ones{};
    ones.fill(0x11);
    assert(!memory::guardedCopyToMemory(pages + pageSize - 8, ones.data(), ones.size()));
    assert(queries[1] == 1 && rejected == 1);
    assert(pages[pageSize - 8] == 0x5A && pages[pageSize - 1] == 0x5A); // Refuse the entire write before any byte lands.

    // A read that faults fails closed and later reads keep working.
    assert(VirtualProtect(pages + pageSize * 2, pageSize, PAGE_NOACCESS, &previous));
    reset();
    assert(!memory::pointerRangeLooksReadable(pages + pageSize * 2, 1));
    assert(queries[0] == 1 && rejected == 1);
    // A range that starts readable and runs into the inaccessible page is refused.
    assert(!memory::pointerRangeLooksReadable(pages + pageSize * 2 - 1, 2));
    reset();
    assert(!memory::guardedCopyFromMemory(pages + pageSize * 2 - 16, ids.data(), sizeof(ids)));
    assert(rejected == 1 && queries[0] == 0);
    std::uint32_t afterFault = 0;
    assert(memory::tryReadValue(reinterpret_cast<const std::uint32_t*>(pages), afterFault) && afterFault == value);

    // Null, low and wrapping addresses are refused without touching memory.
    reset();
    assert(!memory::tryReadValue(static_cast<const std::uint32_t*>(nullptr), copied));
    assert(!memory::tryReadValue(reinterpret_cast<const std::uint32_t*>(0x10000), copied));
    assert(!memory::guardedCopyFromMemory(reinterpret_cast<void*>((std::numeric_limits<std::uintptr_t>::max)() - 3), &copied, 8));
    assert(rejected == 3 && queries[0] == 0);

    assert(VirtualProtect(pages + pageSize * 2, pageSize, PAGE_READWRITE | PAGE_GUARD, &previous));
    // Standalone validation must not consume the guard: a probe that trips it puts it back.
    MEMORY_BASIC_INFORMATION region{};
    assert(!memory::pointerRangeLooksReadable(pages + pageSize * 2, 1));
    assert(VirtualQuery(pages + pageSize * 2, &region, sizeof(region)) != 0);
    assert((region.Protect & PAGE_GUARD) != 0);
    assert(!memory::pointerRangeLooksWritable(pages + pageSize * 2, 1));
    assert(VirtualQuery(pages + pageSize * 2, &region, sizeof(region)) != 0);
    assert((region.Protect & PAGE_GUARD) != 0);

    reset();
    assert(!memory::pointerRangeLooksReadable(nullptr, 8));
    assert(!memory::pointerRangeLooksReadable(pages, 0));
    assert(!memory::pointerRangeLooksReadable(reinterpret_cast<void*>(0x10000), 1));
    assert(!memory::pointerRangeLooksReadable(reinterpret_cast<void*>((std::numeric_limits<std::uintptr_t>::max)() - 3), 8));
    assert(queries[0] == 0 && rejected == 4);

    // Kernel and non-canonical addresses are refused without a probe.
    reset();
    assert(!memory::pointerRangeLooksReadable(reinterpret_cast<void*>((std::numeric_limits<std::uintptr_t>::max)() - 4095), 8));
    assert(!memory::pointerRangeLooksReadable(reinterpret_cast<void*>(0x0000'7FFF'FFFF'0000ull), 1));
    assert(queries[0] == 0 && rejected == 2);

    reset();
    profiling = false;
    assert(memory::tryReadValue(reinterpret_cast<const std::uint32_t*>(pages), copied) && copied == value);
    assert(!memory::pointerRangeLooksWritable(pages + pageSize, 1));
    assert(queries[0] == 0 && queries[1] == 0 && rejected == 0 && guardedReads == 0);
    assert(VirtualFree(pages, 0, MEM_RELEASE));
    return 0;
}
