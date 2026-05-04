#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace rock::physics_shape_cast_math
{
    constexpr float kShapeCastTolerance = 0.001f;
    constexpr std::uint16_t kAnyMaterialId = 0xFFFF;

    struct alignas(16) ShapeCastVec4
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float w = 0.0f;
    };
    static_assert(sizeof(ShapeCastVec4) == 0x10);

    struct alignas(16) RuntimeShapeCastQuery
    {
        void* filterRef = nullptr;
        std::uint16_t materialId = kAnyMaterialId;
        std::uint16_t pad0A = 0;
        std::uint32_t collisionFilterInfo = 0;
        std::uint64_t reserved10 = 0;
        std::uint8_t reserved18 = 0;
        std::uint8_t pad19[7]{};
        void* shape = nullptr;
        std::uint64_t reserved28 = 0;
        ShapeCastVec4 start{};
        ShapeCastVec4 displacement{};
        ShapeCastVec4 inverseDisplacementAndSign{};
        float tolerance = kShapeCastTolerance;
        std::byte reserved64[0x1C]{};
    };

    static_assert(sizeof(RuntimeShapeCastQuery) == 0x80);
    static_assert(offsetof(RuntimeShapeCastQuery, filterRef) == 0x00);
    static_assert(offsetof(RuntimeShapeCastQuery, collisionFilterInfo) == 0x0C);
    static_assert(offsetof(RuntimeShapeCastQuery, shape) == 0x20);
    static_assert(offsetof(RuntimeShapeCastQuery, start) == 0x30);
    static_assert(offsetof(RuntimeShapeCastQuery, displacement) == 0x40);
    static_assert(offsetof(RuntimeShapeCastQuery, inverseDisplacementAndSign) == 0x50);
    static_assert(offsetof(RuntimeShapeCastQuery, tolerance) == 0x60);

    inline float inverseOrMax(float value)
    {
        if (value != 0.0f) {
            return 1.0f / value;
        }

        return std::numeric_limits<float>::max();
    }

    inline std::uint32_t buildPositiveSignMask(const ShapeCastVec4& displacement)
    {
        std::uint32_t mask = 0;
        if (displacement.x >= 0.0f) {
            mask |= 1u << 0;
        }
        if (displacement.y >= 0.0f) {
            mask |= 1u << 1;
        }
        if (displacement.z >= 0.0f) {
            mask |= 1u << 2;
        }
        return mask;
    }

    inline ShapeCastVec4 buildInverseDisplacementAndSign(const ShapeCastVec4& displacement)
    {
        return ShapeCastVec4{
            inverseOrMax(displacement.x),
            inverseOrMax(displacement.y),
            inverseOrMax(displacement.z),
            std::bit_cast<float>(0x3F000000u | buildPositiveSignMask(displacement)),
        };
    }

    inline RuntimeShapeCastQuery buildRuntimeShapeCastQuery(void* filterRef, void* shape, std::uint32_t collisionFilterInfo, const ShapeCastVec4& start,
        const ShapeCastVec4& displacement)
    {
        RuntimeShapeCastQuery query{};
        query.filterRef = filterRef;
        query.materialId = kAnyMaterialId;
        query.collisionFilterInfo = collisionFilterInfo;
        query.shape = shape;
        query.start = start;
        query.displacement = displacement;
        query.displacement.w = 1.0f;
        query.inverseDisplacementAndSign = buildInverseDisplacementAndSign(query.displacement);
        query.tolerance = kShapeCastTolerance;
        return query;
    }
}
