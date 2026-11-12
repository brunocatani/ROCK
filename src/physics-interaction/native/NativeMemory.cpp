#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/performance/PerformanceProfiler.h"

#include <cstdint>
#include <cstring>
#include <intrin.h>
#include <windows.h>

namespace rock::native_memory
{
    namespace
    {
        // x64 Windows pages are 4 KB; large pages are multiples of it, so touching
        // every 4 KB boundary still touches every page of a range.
        constexpr std::uintptr_t kPageSize = 0x1000;
        // Exclusive end of x64 user space. Higher addresses are kernel or
        // non-canonical and are refused without touching memory.
        constexpr std::uintptr_t kUserSpaceEnd = 0x0000'7FFF'FFFF'0000;

        bool userRange(const void* ptr, std::size_t byteCount, std::uintptr_t& start, std::uintptr_t& last)
        {
            start = reinterpret_cast<std::uintptr_t>(ptr);
            if (!pointerLooksReadable(ptr) || byteCount == 0 || start >= kUserSpaceEnd ||
                byteCount > kUserSpaceEnd - start) {
                return false;
            }
            last = start + byteCount - 1;
            return true;
        }

        // A probe that lands on a guard page clears the guard as it faults. The
        // guard belongs to someone else (usually a thread stack's growth page), so
        // put it back. This is the only page query left, on a path that needs a
        // fault on a guard page and so practically never runs.
        void restoreGuard(const void* address) noexcept
        {
            MEMORY_BASIC_INFORMATION info{};
            if (VirtualQuery(address, &info, sizeof(info)) == 0 || info.State != MEM_COMMIT ||
                (info.Protect & PAGE_GUARD) != 0) {
                return;
            }
            DWORD previous = 0;
            VirtualProtect(const_cast<void*>(address), 1, info.Protect | PAGE_GUARD, &previous);
        }

#if defined(_MSC_VER)
        int probeFilter(const EXCEPTION_POINTERS* exception) noexcept
        {
            const auto* record = exception ? exception->ExceptionRecord : nullptr;
            if (record && record->ExceptionCode == STATUS_GUARD_PAGE_VIOLATION && record->NumberParameters >= 2) {
                restoreGuard(reinterpret_cast<const void*>(record->ExceptionInformation[1]));
            }
            return EXCEPTION_EXECUTE_HANDLER;
        }
#endif

        /*
         * Touch one byte in every page of [start, last] under SEH. Page-protection
         * queries were the previous design, but VirtualQuery walks the whole
         * region around an address to report its size: inside the game's own
         * memory pools (hundreds of MB when Buffout 4 does not replace the
         * allocator) one query cost 0.4-1.4 ms, measured on the bench at Boston
         * Common. A touch costs nanoseconds whatever the region size and still
         * answers "accessible right now". A write probe compare-exchanges a byte
         * with itself: lock cmpxchg always writes the destination back, so it
         * faults on a non-writable page and never changes the value, even if
         * another thread writes the byte at the same time.
         */
        bool probePages(std::uintptr_t start, std::uintptr_t last, bool write) noexcept
        {
#if defined(_MSC_VER)
            __try {
#endif
                for (auto address = start;; address = (address | (kPageSize - 1)) + 1) {
                    if (write) {
                        _InterlockedCompareExchange8(reinterpret_cast<volatile char*>(address), 0, 0);
                    } else {
                        static_cast<void>(*reinterpret_cast<const volatile std::uint8_t*>(address));
                    }
                    if ((address | (kPageSize - 1)) >= last) {
                        break;
                    }
                }
                return true;
#if defined(_MSC_VER)
            } __except (probeFilter(GetExceptionInformation())) {
                return false;
            }
#endif
        }

        bool rangeAccessible(const void* ptr, std::size_t byteCount, performance_profiler::MemoryQueryKind kind)
        {
            std::uintptr_t start = 0;
            std::uintptr_t last = 0;
            if (!userRange(ptr, byteCount, start, last)) {
                return false;
            }
            const auto sample = performance_profiler::beginMemoryQuery();
            const bool accessible = probePages(start, last, kind == performance_profiler::MemoryQueryKind::Write);
            performance_profiler::endMemoryQuery(sample, kind);
            return accessible;
        }

        bool sehCopy(void* target, const void* source, std::size_t byteCount) noexcept
        {
#if defined(_MSC_VER)
            __try {
                std::memcpy(target, source, byteCount);
                return true;
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
#else
            std::memcpy(target, source, byteCount);
            return true;
#endif
        }
    }

    bool pointerLooksReadable(const void* ptr) { return reinterpret_cast<std::uintptr_t>(ptr) > 0x10000; }

    bool pointerRangeLooksReadable(const void* ptr, std::size_t byteCount)
    {
        const bool readable = rangeAccessible(ptr, byteCount, performance_profiler::MemoryQueryKind::Read);
        if (!readable)
            performance_profiler::addCounter(performance_profiler::Counter::NativeReadRangeRejected);
        return readable;
    }

    bool pointerRangeLooksWritable(void* ptr, std::size_t byteCount)
    {
        const bool writable = rangeAccessible(ptr, byteCount, performance_profiler::MemoryQueryKind::Write);
        if (!writable)
            performance_profiler::addCounter(performance_profiler::Counter::NativeWriteRangeRejected);
        return writable;
    }

    bool guardedCopyFromMemory(const void* source, void* target, std::size_t byteCount,
        const std::source_location& site)
    {
        performance_profiler::countGuardedRead(site);
        const auto start = reinterpret_cast<std::uintptr_t>(source);
        if (!target || byteCount == 0 || !pointerLooksReadable(source) || start + byteCount < start ||
            !sehCopy(target, source, byteCount)) {
            performance_profiler::addCounter(performance_profiler::Counter::NativeReadRangeRejected);
            return false;
        }
        return true;
    }

    bool guardedCopyToMemory(void* target, const void* source, std::size_t byteCount)
    {
        // Probing every target page first refuses the whole write before any byte lands.
        if (!source || !target || byteCount == 0 || !pointerRangeLooksWritable(target, byteCount)) {
            return false;
        }

        return sehCopy(target, source, byteCount);
    }
}
