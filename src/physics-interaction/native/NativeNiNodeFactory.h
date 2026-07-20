#pragma once

#include "RE/NetImmerse/NiNode.h"

#include <cstdint>

namespace rock::native_scene
{
    /*
     * Creates a FO4VR NiNode on the scene-owner thread and returns one strong
     * reference. Both the node storage and its child array are allocated and
     * constructed by the engine, so the virtual engine destructor releases
     * them through the same allocator domain.
     *
     * Fails closed when the executable identity, native entry signatures, or
     * allocation cannot be validated. Callers must not retain borrowed scene
     * pointers beyond the lifetime protected by the returned NiPointer.
     */
    [[nodiscard]] RE::NiPointer<RE::NiNode> createEngineNiNode(std::uint16_t childCapacity) noexcept;
}
