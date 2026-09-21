#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/performance/PerformanceProfiler.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <windows.h>

namespace rock::native_memory
{
    namespace
    {
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
        if (!source || !target || byteCount == 0 || !pointerRangeLooksReadable(source, byteCount)) {
            return false;
        }

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

    bool guardedCopyToMemory(void* target, const void* source, std::size_t byteCount)
    {
        if (!source || !target || byteCount == 0 || !pointerRangeLooksWritable(target, byteCount)) {
            return false;
        }

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
