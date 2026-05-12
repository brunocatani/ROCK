#include "physics-interaction/native/HavokTlsDiagnostics.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/NativeMemory.h"

#include <REL/Relocation.h>
#include <cstddef>
#include <windows.h>

namespace rock::havok_tls_diagnostics
{
    CommandQueueState readCurrentCommandQueueState()
    {
        CommandQueueState state{};

        static REL::Relocation<std::uint32_t*> tlsIndexStorage{ REL::Offset(offsets::kData_HavokTlsAllocKey) };
        auto* tlsIndexPtr = tlsIndexStorage.get();
        std::uint32_t tlsIndex = 0xFFFF'FFFFu;
        if (!tlsIndexPtr || !native_memory::tryReadValue(tlsIndexPtr, tlsIndex) || tlsIndex == TLS_OUT_OF_INDEXES) {
            return state;
        }

        state.tlsIndex = tlsIndex;
        SetLastError(ERROR_SUCCESS);
        auto* tlsBlock = TlsGetValue(tlsIndex);
        if (!tlsBlock && GetLastError() != ERROR_SUCCESS) {
            return state;
        }
        state.tlsBlock = tlsBlock;
        if (!tlsBlock) {
            return state;
        }

        const bool readCommandMode = native_memory::tryReadField(tlsBlock, static_cast<std::ptrdiff_t>(kCommandModeByteOffset), state.commandMode);
        const bool readPhysicsContext = native_memory::tryReadField(tlsBlock, static_cast<std::ptrdiff_t>(kPhysicsContextByteOffset), state.physicsContext);
        const bool readThreadCommandIndex =
            native_memory::tryReadField(tlsBlock, static_cast<std::ptrdiff_t>(kThreadCommandIndexOffset), state.threadCommandIndex);

        state.readable = readCommandMode && readPhysicsContext && readThreadCommandIndex;
        return state;
    }
}
