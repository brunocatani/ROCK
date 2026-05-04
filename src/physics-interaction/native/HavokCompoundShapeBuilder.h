#pragma once

#include "RE/Havok/hknpShape.h"

#include <cstddef>
#include <span>

namespace frik::rock::havok_compound_shape_builder
{
    /*
     * PAPER reload bodies need one generated body for one authored part even
     * when the point cloud must be split into several convex children. Ghidra
     * verification on FO4VR shows Bethesda's own static-compound constructor
     * builds the required child storage, references, bounds, and tree data, so
     * ROCK exposes only the native construction path here instead of writing
     * hknpStaticCompoundShape internals by hand.
     */
    struct alignas(16) Vector4
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float w = 0.0f;
    };

    struct alignas(16) ChildTransform
    {
        Vector4 column0{ 1.0f, 0.0f, 0.0f, 0.0f };
        Vector4 column1{ 0.0f, 1.0f, 0.0f, 0.0f };
        Vector4 column2{ 0.0f, 0.0f, 1.0f, 0.0f };
        Vector4 translation{ 0.0f, 0.0f, 0.0f, 1.0f };
        Vector4 scale{ 1.0f, 1.0f, 1.0f, 1.0f };
        int scaleMode = 0;
    };

    struct CompoundChild
    {
        const RE::hknpShape* shape = nullptr;
        ChildTransform transform;
    };

    inline constexpr std::size_t kMaxStaticCompoundChildren = 0x7FFE;

    RE::hknpShape* buildStaticCompoundShape(std::span<const CompoundChild> children) noexcept;
}
