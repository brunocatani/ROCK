#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/performance/PerformanceProfiler.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <windows.h>

namespace rock::native_memory
{
    namespace
    {
        /*
         * VirtualQuery costs several microseconds in FO4VR's address space and
         * a frame performs hundreds of guarded field reads, almost all inside a
         * few heap regions; the OS queries were most of ROCK's frame time.
         * Guarded reads therefore reuse committed readable regions this thread
         * already queried, until the system tick (~16 ms) advances. A region
         * freed or re-protected meanwhile still fails closed: the copy runs
         * under SEH into a staging buffer, so the caller's target stays
         * untouched and the cache is dropped. Writes and standalone range
         * checks (callers that dereference without SEH) always query the OS.
         * Thread model: thread_local, holds addresses only, no shared state.
         */
        constexpr std::size_t kCachedRegionCount = 16;
        constexpr std::size_t kStagedReadBytes = 1024;

        struct ReadableRegionCache
        {
            std::array<std::uintptr_t, kCachedRegionCount> begin{};
            std::array<std::uintptr_t, kCachedRegionCount> end{};
            std::size_t count{ 0 };
            std::size_t next{ 0 };
            ULONGLONG tick{ 0 };
        };

        thread_local ReadableRegionCache t_readableRegions;

        ReadableRegionCache& currentReadableRegions() noexcept
        {
            auto& cache = t_readableRegions;
            const auto tick = GetTickCount64();
            if (cache.tick != tick) {
                cache.tick = tick;
                cache.count = 0;
                cache.next = 0;
            }
            return cache;
        }

        void dropReadableRegions() noexcept
        {
            t_readableRegions.count = 0;
            t_readableRegions.next = 0;
        }

        bool pageProtectAllowsRead(DWORD protect)
        {
            if ((protect & PAGE_GUARD) != 0 || (protect & PAGE_NOACCESS) != 0) {
                return false;
            }

            const DWORD baseProtect = protect & 0xFF;
            return baseProtect == PAGE_READONLY ||
                   baseProtect == PAGE_READWRITE ||
                   baseProtect == PAGE_WRITECOPY ||
                   baseProtect == PAGE_EXECUTE_READ ||
                   baseProtect == PAGE_EXECUTE_READWRITE ||
                   baseProtect == PAGE_EXECUTE_WRITECOPY;
        }

        bool pageProtectAllowsWrite(DWORD protect)
        {
            if ((protect & PAGE_GUARD) != 0 || (protect & PAGE_NOACCESS) != 0) {
                return false;
            }

            const DWORD baseProtect = protect & 0xFF;
            return baseProtect == PAGE_READWRITE ||
                   baseProtect == PAGE_WRITECOPY ||
                   baseProtect == PAGE_EXECUTE_READWRITE ||
                   baseProtect == PAGE_EXECUTE_WRITECOPY;
        }

        bool pointerRangeHasPageProtection(const void* ptr, std::size_t byteCount, bool (*allowsProtection)(DWORD),
            performance_profiler::MemoryQueryKind kind)
        {
            if (!ptr || byteCount == 0 || !pointerLooksReadable(ptr)) {
                return false;
            }

            const auto start = reinterpret_cast<std::uintptr_t>(ptr);
            const auto end = start + byteCount;
            if (end < start) {
                return false;
            }

            auto current = start;
            while (current < end) {
                MEMORY_BASIC_INFORMATION memoryInfo{};
                const auto sample = performance_profiler::beginMemoryQuery();
                const auto queried = VirtualQuery(reinterpret_cast<LPCVOID>(current), &memoryInfo, sizeof(memoryInfo));
                performance_profiler::endMemoryQuery(sample, kind, queried != 0);
                if (queried == 0) {
                    return false;
                }
                if (memoryInfo.State != MEM_COMMIT || !allowsProtection(memoryInfo.Protect)) {
                    return false;
                }

                const auto regionBase = reinterpret_cast<std::uintptr_t>(memoryInfo.BaseAddress);
                const auto regionEnd = regionBase + memoryInfo.RegionSize;
                if (regionEnd <= current || regionEnd < regionBase) {
                    return false;
                }

                current = (std::min)(regionEnd, end);
            }

            return true;
        }

        // Same walk as pointerRangeHasPageProtection, served from the region
        // cache where possible. Only committed readable regions are cached.
        bool cachedRangeLooksReadable(const void* ptr, std::size_t byteCount)
        {
            if (!ptr || byteCount == 0 || !pointerLooksReadable(ptr)) {
                return false;
            }

            const auto start = reinterpret_cast<std::uintptr_t>(ptr);
            const auto end = start + byteCount;
            if (end < start) {
                return false;
            }

            auto& cache = currentReadableRegions();
            auto current = start;
            while (current < end) {
                std::uintptr_t regionEnd = 0;
                for (std::size_t i = 0; i < cache.count; ++i) {
                    if (current >= cache.begin[i] && current < cache.end[i]) {
                        regionEnd = cache.end[i];
                        break;
                    }
                }

                if (regionEnd == 0) {
                    MEMORY_BASIC_INFORMATION memoryInfo{};
                    const auto sample = performance_profiler::beginMemoryQuery();
                    const auto queried = VirtualQuery(reinterpret_cast<LPCVOID>(current), &memoryInfo, sizeof(memoryInfo));
                    performance_profiler::endMemoryQuery(sample, performance_profiler::MemoryQueryKind::Read, queried != 0);
                    if (queried == 0 || memoryInfo.State != MEM_COMMIT || !pageProtectAllowsRead(memoryInfo.Protect)) {
                        return false;
                    }

                    const auto regionBase = reinterpret_cast<std::uintptr_t>(memoryInfo.BaseAddress);
                    regionEnd = regionBase + memoryInfo.RegionSize;
                    if (regionEnd <= current || regionEnd < regionBase) {
                        return false;
                    }

                    const auto slot = cache.next;
                    cache.begin[slot] = regionBase;
                    cache.end[slot] = regionEnd;
                    cache.next = (slot + 1) % kCachedRegionCount;
                    cache.count = (std::min)(cache.count + 1, kCachedRegionCount);
                }

                current = (std::min)(regionEnd, end);
            }

            return true;
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

    bool pointerLooksReadable(const void* ptr)
    {
        return reinterpret_cast<std::uintptr_t>(ptr) > 0x10000;
    }

    bool pointerRangeLooksReadable(const void* ptr, std::size_t byteCount)
    {
        const bool readable = pointerRangeHasPageProtection(ptr, byteCount, pageProtectAllowsRead, performance_profiler::MemoryQueryKind::Read);
        if (!readable) performance_profiler::addCounter(performance_profiler::Counter::NativeReadRangeRejected);
        return readable;
    }

    bool pointerRangeLooksWritable(void* ptr, std::size_t byteCount)
    {
        const bool writable = pointerRangeHasPageProtection(ptr, byteCount, pageProtectAllowsWrite, performance_profiler::MemoryQueryKind::Write);
        if (!writable) performance_profiler::addCounter(performance_profiler::Counter::NativeWriteRangeRejected);
        return writable;
    }

    bool guardedCopyFromMemory(const void* source, void* target, std::size_t byteCount)
    {
        if (!source || !target || byteCount == 0) {
            return false;
        }

        // Large copies are rare (mesh and table snapshots) and keep the exact
        // per-call query, since a stale region could fault mid-copy into target.
        if (byteCount > kStagedReadBytes) {
            return pointerRangeLooksReadable(source, byteCount) && sehCopy(target, source, byteCount);
        }

        if (!cachedRangeLooksReadable(source, byteCount)) {
            performance_profiler::addCounter(performance_profiler::Counter::NativeReadRangeRejected);
            return false;
        }

        std::array<std::byte, kStagedReadBytes> staged;
        if (!sehCopy(staged.data(), source, byteCount)) {
            dropReadableRegions();
            performance_profiler::addCounter(performance_profiler::Counter::NativeReadRangeRejected);
            return false;
        }
        std::memcpy(target, staged.data(), byteCount);
        return true;
    }

    bool guardedCopyToMemory(void* target, const void* source, std::size_t byteCount)
    {
        if (!source || !target || byteCount == 0 || !pointerRangeLooksWritable(target, byteCount)) {
            return false;
        }

        return sehCopy(target, source, byteCount);
    }
}
