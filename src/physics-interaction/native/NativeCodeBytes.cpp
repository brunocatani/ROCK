#include "physics-interaction/native/NativeCodeBytes.h"

#include <RPS/Runtime/HookChain.h>

namespace rock::native_code
{
    bool entryIsNativeOrDetoured(const std::uintptr_t moduleOffset,
        const std::span<const std::uint8_t> nativePrologue) noexcept
    {
        return static_cast<bool>(
            RPS::Runtime::HookChain::resolveEntry(REL::Offset(moduleOffset).address(), nativePrologue));
    }
}
