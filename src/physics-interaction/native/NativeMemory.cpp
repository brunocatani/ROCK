#include "physics-interaction/native/NativeMemory.h"

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

        bool pointerRangeHasPageProtection(const void* ptr, std::size_t byteCount, bool (*allowsProtection)(DWORD))
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
                if (VirtualQuery(reinterpret_cast<LPCVOID>(current), &memoryInfo, sizeof(memoryInfo)) == 0) {
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
        return pointerRangeHasPageProtection(ptr, byteCount, pageProtectAllowsRead);
    }

    bool pointerRangeLooksWritable(void* ptr, std::size_t byteCount)
    {
        return pointerRangeHasPageProtection(ptr, byteCount, pageProtectAllowsWrite);
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
