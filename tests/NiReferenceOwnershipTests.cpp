#include "physics-interaction/native/NiReferenceOwnership.h"
#include <cassert>
#include <cstdint>

struct NativeObject {
    std::uint32_t count{},deleted{};
    void IncRefCount() { ++count; }
    void DecRefCount() { assert(count); if(--count==0) ++deleted; }
};
int main()
{
    using namespace rock::ni_reference_ownership;
    NativeObject system,node,collision;
    retain(&system); // local construction owner
    retain(&system); // native collision-object constructor's system edge
    retain(&collision); // generated body's owner, previously missing
    release(&system); // hand off to the native edge
    retain(&node); // generated body's owner, previously missing
    retain(&collision); // native node link
    retain(&node); release(&node); // transient resolver snapshot
    assert(node.count==1 && !node.deleted && collision.count==2);
    // Retirement must survive unlinking the node. The quarantine still reads
    // and clears the physics-system edge afterward; it cannot read a freed wrapper.
    release(&collision); release(&node);
    assert(node.deleted==1 && collision.count==1 && !collision.deleted);
    release(&system);
    assert(system.deleted==1 && !collision.deleted);
    NativeObject wide{0x10000};
    release(&wide);
    assert(wide.count==0xffff && !wide.deleted); // no Havok low-word sentinel
    release<NativeObject>(nullptr);
}
