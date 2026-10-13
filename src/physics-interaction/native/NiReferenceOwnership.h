#pragma once

namespace rock::ni_reference_ownership
{
    // NiObject construction starts at zero. Allocation is not an owning
    // reference; acquire one before any scene link or temporary NiPointer.
    template<class Object> void retain(Object* object)
    { if(object) object->IncRefCount(); }

    // NiRefObject uses the entire 32-bit count and DeleteThis at vslot 1.
    // Havok's packed 16-bit count and vslot 0 destructor are a different ABI.
    template<class Object> void release(Object* object)
    { if(object) object->DecRefCount(); }
}
