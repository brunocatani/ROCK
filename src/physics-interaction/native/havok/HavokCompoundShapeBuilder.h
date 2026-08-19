#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

namespace RE
{
    class hknpShape;
}

namespace rock::havok_compound_shape_builder
{
    [[nodiscard]] constexpr std::uint8_t compoundShapeKeyBitCount(
        std::size_t childCount) noexcept
    {
        std::uint8_t bits = 0;
        do {
            ++bits;
            childCount >>= 1;
        } while (childCount != 0);
        return bits;
    }

    [[nodiscard]] constexpr std::optional<std::uint16_t>
    decodeTopLevelCompoundInstanceId(
        const std::uint32_t shapeKey,
        const std::uint8_t bitCount) noexcept
    {
        if (shapeKey == 0xFFFF'FFFFu || bitCount == 0 || bitCount >= 32) {
            return std::nullopt;
        }
        return static_cast<std::uint16_t>(
            shapeKey >> (32u - bitCount));
    }

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

    struct DynamicCompoundUpdateResult
    {
        bool succeeded = false;
        std::size_t changedChildCount = 0;
    };

    struct HavokShapeRelease
    {
        void operator()(RE::hknpShape* shape) const noexcept;
    };

    /*
     * Owns one FO4VR hknpDynamicCompoundShape and the stable instance IDs
     * returned by its native constructor. Child shapes are referenced by the
     * compound itself. The retained instance records are non-owning templates
     * used to submit transform-only changes through the native
     * updateInstances contract without allocating in the physics callback.
     */
    class DynamicCompoundShape
    {
    public:
        DynamicCompoundShape() = default;
        DynamicCompoundShape(const DynamicCompoundShape&) = delete;
        DynamicCompoundShape& operator=(const DynamicCompoundShape&) = delete;
        DynamicCompoundShape(DynamicCompoundShape&&) noexcept = default;
        DynamicCompoundShape& operator=(DynamicCompoundShape&&) noexcept = default;
        ~DynamicCompoundShape() = default;

        [[nodiscard]] bool create(std::span<const CompoundChild> children) noexcept;
        [[nodiscard]] DynamicCompoundUpdateResult updateTransforms(std::span<const ChildTransform> transforms) noexcept;
        void reset() noexcept;

        [[nodiscard]] RE::hknpShape* get() const noexcept { return _shape.get(); }
        [[nodiscard]] std::size_t childCount() const noexcept { return _instances.size(); }
        /*
         * FO4VR hknpCompoundShape::getLeafShape (0x1416E2430) selects the
         * stable instance ID from the high shape-key bits. The dynamic
         * compound constructor (0x1416E2BE0) chooses bit_width(childCount),
         * with a one-bit minimum. Keep that native contract beside the IDs
         * returned by construction so contact callbacks can recover authored
         * child semantics without reading undocumented shape storage.
         */
        [[nodiscard]] std::optional<std::size_t> tryResolveChildIndex(
            std::uint32_t shapeKey) const noexcept;
        [[nodiscard]] std::uint8_t shapeKeyBitCount() const noexcept
        {
            return _shapeKeyBitCount;
        }
        [[nodiscard]] explicit operator bool() const noexcept { return _shape != nullptr; }

    private:
        struct alignas(16) ShapeInstanceStorage
        {
            std::array<std::byte, 0x80> bytes{};
        };

        std::unique_ptr<RE::hknpShape, HavokShapeRelease> _shape;
        std::vector<ShapeInstanceStorage> _instances;
        std::vector<std::int16_t> _instanceIds;
        std::vector<ChildTransform> _lastTransforms;
        std::vector<ShapeInstanceStorage> _updateInstances;
        std::vector<std::int16_t> _updateIds;
        std::uint8_t _shapeKeyBitCount = 0;
    };
}
