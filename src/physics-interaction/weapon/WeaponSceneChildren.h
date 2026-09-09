#pragma once

namespace rock
{
    // NiTObjectArray::size() counts populated entries; it is not an index
    // limit. Attachment replacement can leave holes before a live last slot.
    // Keep the physical slot index for scene identity, and let bounded callers
    // stop without treating empty slots as children.
    template <class Children, class Visitor>
    bool visitWeaponChildSlots(const Children& children, Visitor&& visitor)
    {
        for (decltype(children.capacity()) slot = 0; slot < children.capacity(); ++slot) {
            if (auto* child = children[slot].get(); child && !visitor(child, slot)) return false;
        }
        return true;
    }
}
