#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "physics-interaction/debug/DebugOverlayPolicy.h"

namespace rock::debug_overlay_shape
{
    struct Vertex
    {
        float x{ 0.0f };
        float y{ 0.0f };
        float z{ 0.0f };

        bool operator==(const Vertex&) const = default;
    };

    struct MeshData
    {
        std::vector<Vertex> vertices;
        std::vector<std::uint16_t> indices;
        bool valid{ false };
    };

    struct ShapeKey
    {
        std::uintptr_t shapeAddress{ 0 };
        std::uint64_t geometryFingerprint{ 0 };

        bool operator==(const ShapeKey&) const = default;
    };

    struct ShapeKeyHash
    {
        std::size_t operator()(const ShapeKey& key) const noexcept
        {
            std::size_t hash = 0;
            hash ^= std::hash<std::uintptr_t>{}(key.shapeAddress) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
            hash ^= std::hash<std::uint64_t>{}(key.geometryFingerprint) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
            return hash;
        }
    };

    struct RecipeSettings
    {
        float havokToGameScale{ 0.0f };
        std::uint32_t maxConvexSupportVertices{ 0 };
        std::uint32_t maxCompoundChildren{ 0 };
        std::uint32_t maxCompoundDepth{ 0 };
        bool useBoundsForHeavyConvex{ false };
    };

    struct ShapeRecipe
    {
        enum class Kind : std::uint8_t
        {
            Unsupported,
            Sphere,
            Capsule,
            ConvexVertices,
            Triangle,
            ScaledConvex,
            Compound
        };

        struct Child
        {
            std::array<float, 16> transform{
                1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f
            };
            std::array<float, 4> scale{ 1.0f, 1.0f, 1.0f, 0.0f };
            std::unique_ptr<ShapeRecipe> recipe;
        };

        Kind kind{ Kind::Unsupported };
        RecipeSettings settings{};
        float convexRadius{ 0.0f };
        std::array<float, 4> vertexA{};
        std::array<float, 4> vertexB{};
        std::array<float, 4> scale{ 1.0f, 1.0f, 1.0f, 0.0f };
        std::array<float, 4> translation{};
        std::vector<Vertex> vertices;
        std::unique_ptr<ShapeRecipe> inner;
        std::vector<Child> children;
        int shapeType{ -1 };
        bool canonicalUnitSphere{ false };
        bool valid{ false };
    };

    struct BuiltShape
    {
        MeshData mesh{};
        debug_overlay_policy::ShapeDecodeMode decodeMode{ debug_overlay_policy::ShapeDecodeMode::Unsupported };
        int shapeType{ -1 };
    };

    [[nodiscard]] BuiltShape buildMeshFromRecipe(const ShapeRecipe& recipe);
    [[nodiscard]] MeshData makeUnitBoxMesh();
    [[nodiscard]] std::size_t approximateGpuBytes(const MeshData& mesh) noexcept;
}
