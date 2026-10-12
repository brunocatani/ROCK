#include "physics-interaction/weapon/NativeModelAncestry.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include <Windows.h>
#include <cassert>
#include <cstddef>
#include <cstring>

namespace rock::performance_profiler
{
    // Only the diagnostic sink is substituted; ancestry and guarded native
    // reads execute their production implementations against Windows pages.
    MemoryQuerySample beginMemoryQuery() noexcept { return {}; }
    void endMemoryQuery(MemoryQuerySample, MemoryQueryKind, bool) noexcept {}
    void addCounter(Counter, std::uint64_t) noexcept {}
}

int main()
{
    using rock::native_model_ancestry::Relation;
    using rock::native_model_ancestry::within;
    SYSTEM_INFO system{};
    GetSystemInfo(&system);
    auto* capture = static_cast<std::byte*>(VirtualAlloc(nullptr, system.dwPageSize * 2,
        MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE));
    assert(capture);
    // Native NiAVObject record bytes, including +28 parent pointers; no engine
    // constructors, stand-in scene classes or test implementation of traversal.
    // Reproduce two disjoint equipped branches below a common scene. The two
    // model name fields intentionally have the same interned-string identity.
    auto* scene = capture;
    auto* rightAttachment = capture + 0x40;
    auto* rightModel = capture + 0x80;
    auto* leftAttachment = capture + 0xC0;
    auto* leftModel = capture + 0x100;
    auto* leftPart = capture + 0x140;
    const auto parent = [](std::byte* child, const void* value) { std::memcpy(child + 0x28, &value, sizeof(value)); };
    parent(rightAttachment, scene); parent(rightModel, rightAttachment);
    parent(leftAttachment, scene); parent(leftModel, leftAttachment); parent(leftPart, leftModel);
    const void* sameName = capture + 0x1C0;
    std::memcpy(rightModel + 0x10, &sameName, sizeof(sameName));
    std::memcpy(leftModel + 0x10, &sameName, sizeof(sameName));
    assert(within(leftPart, leftAttachment) == Relation::Inside);
    assert(within(leftPart, leftModel) == Relation::Inside);
    assert(within(rightModel, leftAttachment) == Relation::Outside);
    assert(within(leftModel, rightModel) == Relation::Outside);
    // Deleting a common ancestor would also destroy the surviving gun.
    assert(within(leftModel, scene) == Relation::Inside);
    // A queued cleanup must not interpret an inaccessible/recycled parent as
    // proof that a node belongs outside the protected subtree.
    DWORD previous = 0;
    assert(VirtualProtect(capture + system.dwPageSize, system.dwPageSize, PAGE_NOACCESS, &previous));
    parent(leftPart, capture + system.dwPageSize);
    assert(within(leftPart, leftModel) == Relation::Unreadable);
    // Corrupt parent cycles must terminate before any destructive call.
    parent(leftPart, leftPart);
    assert(within(leftPart, rightModel) == Relation::Unreadable);
    parent(leftPart, leftModel);
    assert(within(leftPart, leftModel) == Relation::Inside);
    assert(VirtualFree(capture, 0, MEM_RELEASE));
}
