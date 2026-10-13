#pragma once
#include <cstddef>

namespace rock::storage_scene_lookup
{
    // FO4VR 1.2.72: NiNode RVA 1C18500 recursively dispatches +0x188;
    // NiAVObject RVA 1C233F0 compares the requested name and returns the node.
    // CommonLib's flat slot 0x2E resolves to RET at 1C23360 in VR and leaves
    // RAX undefined. Passing that value to VATS caused the stored-display CTD.
    inline constexpr std::size_t kGetObjectByNameSlot = 0x31;

    // Borrowed result. The storage-display owner retains the returned node
    // before installing its native highlight; null means no matching object.
    template<class SceneObject, class FixedString>
    [[nodiscard]] SceneObject* find(SceneObject* root, const FixedString& name)
    {
        if (!root) return nullptr;
        using Lookup = SceneObject* (*)(SceneObject*, const FixedString&);
        const auto* table = *reinterpret_cast<Lookup* const*>(root);
        const auto lookup = table ? table[kGetObjectByNameSlot] : nullptr;
        return lookup ? lookup(root, name) : nullptr;
    }
}
