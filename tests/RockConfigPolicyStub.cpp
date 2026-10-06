#define NOMMNOSOUND
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "F4SE/Impl/PCH.h"

#ifdef near
#undef near
#endif
#ifdef far
#undef far
#endif
#ifdef MEM_RELEASE
#undef MEM_RELEASE
#endif
#ifdef MEM_COMMIT
#undef MEM_COMMIT
#endif
#ifdef MEM_RESERVE
#undef MEM_RESERVE
#endif
#ifdef PAGE_EXECUTE_READ
#undef PAGE_EXECUTE_READ
#endif
#ifdef PAGE_EXECUTE_READWRITE
#undef PAGE_EXECUTE_READWRITE
#endif
#ifdef MAX_PATH
#undef MAX_PATH
#endif

#include "RockConfig.h"

#include <cmath>
#include <cstddef>

namespace REL
{
    /*
     * Policy test executables validate pure ROCK math and state contracts. They
     * intentionally do not run inside Fallout 4 VR, so CommonLib relocation
     * singletons must not scan process modules or load the VR address library.
     * Production code still uses the real CommonLib definitions through the DLL
     * target; these inert definitions are linked only into standalone tests.
     */
    Module::Module() :
        _filename(L"Fallout4VR.exe"),
        _filePath(L"Fallout4VR.exe"),
        _runtime(Runtime::VR)
    {}

    IDDB::IDDB() = default;

    std::size_t IDDB::id2offset(std::uint64_t) const
    {
        return 0;
    }

#ifdef ENABLE_FALLOUT_VR
    bool IDDB::IsVRAddressLibraryAtLeastVersion(const char*, bool) const
    {
        return true;
    }
#endif
}

namespace rock
{
    /*
     * Policy executables exercise header-only math and state decisions without
     * loading the runtime INI watcher. Refactored grouped headers can still
     * instantiate the inline global config for default values, so tests provide
     * only the file-watch shutdown hook needed by RockConfig's destructor.
     */
    void RockConfig::stopFileWatch() {}
}
