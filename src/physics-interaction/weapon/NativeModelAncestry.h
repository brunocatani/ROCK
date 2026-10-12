#pragma once

#include "physics-interaction/native/NativeMemory.h"
#include <cstdint>

namespace rock::native_model_ancestry
{
    enum class Relation { Outside, Inside, Unreadable };

    // Read-only native scene ancestry. NiAVObject+28 is independently used by
    // 1C5E90 (detach) and 2DB2B0/its retirement callers. Unreadable/cyclic input
    // is not evidence that destructive work is outside a protected model.
    inline Relation within(const void* node, const void* root) noexcept
    {
        if (!root) return Relation::Outside;
        for (std::uint32_t depth = 0; node && depth < 128; ++depth) {
            if (node == root) return Relation::Inside;
            const void* parent = nullptr;
            if (!native_memory::tryReadField(node, 0x28, parent)) return Relation::Unreadable;
            node = parent;
        }
        return node ? Relation::Unreadable : Relation::Outside;
    }
}
