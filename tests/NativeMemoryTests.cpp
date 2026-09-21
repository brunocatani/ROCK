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
    // Neighboring ragdoll fields use one guarded snapshot instead of three
    // page queries. Compare all decoded values with independent field reads.
    constexpr std::size_t recordCount = 32, recordStride = 0xA0;
    struct Fields { std::uint32_t flags{}, motion{}; std::uint64_t owner{}; };
    std::array<Fields, recordCount> expectedFields{}, batchFields{};
    for (std::size_t i = 0; i < recordCount; ++i) {
        const std::uint32_t flags = static_cast<std::uint32_t>(i * 17), motion = static_cast<std::uint32_t>(i + 91);
        const std::uint64_t owner = 0x12340000ull + i * 0x1000;
        std::memcpy(pages + i * recordStride + 0x40, &flags, sizeof(flags));
        std::memcpy(pages + i * recordStride + 0x68, &motion, sizeof(motion));
        std::memcpy(pages + i * recordStride + 0x88, &owner, sizeof(owner));
    }
    reset();
    for (std::size_t i = 0; i < recordCount; ++i) {
        assert(memory::tryReadField(pages + i * recordStride, 0x40, expectedFields[i].flags));
        assert(memory::tryReadField(pages + i * recordStride, 0x68, expectedFields[i].motion));
        assert(memory::tryReadField(pages + i * recordStride, 0x88, expectedFields[i].owner));
    }
    assert(queries[0] == recordCount * 3);
    reset();
    for (std::size_t i = 0; i < recordCount; ++i) {
        std::array<std::byte, 0x50> fields{};
        assert(memory::guardedCopyFromMemory(pages + i * recordStride + 0x40, fields.data(), fields.size()));
        std::memcpy(&batchFields[i].flags, fields.data(), sizeof(batchFields[i].flags));
        std::memcpy(&batchFields[i].motion, fields.data() + 0x28, sizeof(batchFields[i].motion));
        std::memcpy(&batchFields[i].owner, fields.data() + 0x48, sizeof(batchFields[i].owner));
        assert(batchFields[i].flags == expectedFields[i].flags && batchFields[i].motion == expectedFields[i].motion &&
            batchFields[i].owner == expectedFields[i].owner);
    }
    assert(queries[0] == recordCount && rejected == 0);
    const std::uint32_t changedMotion = 0x7FFFFFFF;
    std::memcpy(pages + 0x68, &changedMotion, sizeof(changedMotion));
    std::array<std::byte, 0x50> freshFields{};
    assert(memory::guardedCopyFromMemory(pages + 0x40, freshFields.data(), freshFields.size()));
    std::uint32_t freshMotion = 0;
    std::memcpy(&freshMotion, freshFields.data() + 0x28, sizeof(freshMotion));
    assert(freshMotion == changedMotion); // No stale data survives a later read.

    // Restore the fixture used by the remaining cross-page tests.
    std::memset(pages, 0x5A, pageSize * 3);
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
