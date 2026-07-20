#include "physics-interaction/debug/DebugOverlayShapeGeometry.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "physics-interaction/debug/DebugConvexHullMesh.h"

namespace rock::debug_overlay_shape
{
    namespace
    {
        Vertex subtract(const Vertex& left, const Vertex& right)
        {
            return Vertex{ left.x - right.x, left.y - right.y, left.z - right.z };
        }

        Vertex add(const Vertex& left, const Vertex& right)
        {
            return Vertex{ left.x + right.x, left.y + right.y, left.z + right.z };
        }

        Vertex multiply(const Vertex& value, float scalar)
        {
            return Vertex{ value.x * scalar, value.y * scalar, value.z * scalar };
        }

        float dot(const Vertex& left, const Vertex& right)
        {
            return left.x * right.x + left.y * right.y + left.z * right.z;
        }

        Vertex cross(const Vertex& left, const Vertex& right)
        {
            return Vertex{
                left.y * right.z - left.z * right.y,
                left.z * right.x - left.x * right.z,
                left.x * right.y - left.y * right.x
            };
        }

        float lengthSquared(const Vertex& value)
        {
            return dot(value, value);
        }

        Vertex normalized(const Vertex& value)
        {
            const float length = std::sqrt(lengthSquared(value));
            if (length < 1.0e-8f) {
                return Vertex{ 0.0f, 0.0f, 1.0f };
            }
            return Vertex{ value.x / length, value.y / length, value.z / length };
        }

        MeshData makeBoundsBox(const std::vector<Vertex>& vertices)
        {
            MeshData mesh{};
            if (vertices.empty()) {
                return mesh;
            }

            Vertex minimum = vertices.front();
            Vertex maximum = vertices.front();
            for (const auto& vertex : vertices) {
                minimum.x = (std::min)(minimum.x, vertex.x);
                minimum.y = (std::min)(minimum.y, vertex.y);
                minimum.z = (std::min)(minimum.z, vertex.z);
                maximum.x = (std::max)(maximum.x, vertex.x);
                maximum.y = (std::max)(maximum.y, vertex.y);
                maximum.z = (std::max)(maximum.z, vertex.z);
            }

            mesh.vertices = {
                { minimum.x, minimum.y, minimum.z },
                { maximum.x, minimum.y, minimum.z },
                { maximum.x, maximum.y, minimum.z },
                { minimum.x, maximum.y, minimum.z },
                { minimum.x, minimum.y, maximum.z },
                { maximum.x, minimum.y, maximum.z },
                { maximum.x, maximum.y, maximum.z },
                { minimum.x, maximum.y, maximum.z },
            };
            mesh.indices = {
                0, 1, 2, 0, 2, 3,
                4, 6, 5, 4, 7, 6,
                0, 4, 5, 0, 5, 1,
                1, 5, 6, 1, 6, 2,
                2, 6, 7, 2, 7, 3,
                3, 7, 4, 3, 4, 0,
            };
            mesh.valid = true;
            return mesh;
        }

        MeshData makeSphere(float havokRadius, float havokToGameScale)
        {
            MeshData mesh{};
            constexpr int segments = 12;
            constexpr int rings = 12;
            constexpr float pi = 3.14159265358979323846f;
            const float radius = havokRadius * havokToGameScale;

            for (int ring = 0; ring <= rings; ++ring) {
                const float v = ring / static_cast<float>(rings);
                const float theta = v * pi;
                for (int segment = 0; segment <= segments; ++segment) {
                    const float u = segment / static_cast<float>(segments);
                    const float phi = u * pi * 2.0f;
                    mesh.vertices.push_back(Vertex{
                        std::cos(phi) * std::sin(theta) * radius,
                        std::cos(theta) * radius,
                        std::sin(phi) * std::sin(theta) * radius
                    });
                }
            }

            for (int ring = 0; ring < rings; ++ring) {
                for (int segment = 0; segment < segments; ++segment) {
                    const int index = ring * (segments + 1) + segment;
                    mesh.indices.push_back(static_cast<std::uint16_t>(index));
                    mesh.indices.push_back(static_cast<std::uint16_t>(index + segments + 1));
                    mesh.indices.push_back(static_cast<std::uint16_t>(index + 1));
                    mesh.indices.push_back(static_cast<std::uint16_t>(index + 1));
                    mesh.indices.push_back(static_cast<std::uint16_t>(index + segments + 1));
                    mesh.indices.push_back(static_cast<std::uint16_t>(index + segments + 2));
                }
            }

            mesh.valid = true;
            return mesh;
        }

        MeshData makeCapsule(const ShapeRecipe& recipe)
        {
            MeshData mesh{};
            constexpr int segments = 12;
            constexpr int rings = 6;
            constexpr float pi = 3.14159265358979323846f;
            const float scale = recipe.settings.havokToGameScale;
            const float radius = recipe.convexRadius * scale;

            Vertex vertexA{ recipe.vertexA[0] * scale, recipe.vertexA[1] * scale, recipe.vertexA[2] * scale };
            Vertex vertexB{ recipe.vertexB[0] * scale, recipe.vertexB[1] * scale, recipe.vertexB[2] * scale };
            Vertex direction = subtract(vertexB, vertexA);
            const float capsuleLength = std::sqrt(lengthSquared(direction));
            if (capsuleLength < 0.001f) {
                return makeSphere(recipe.convexRadius, scale);
            }
            direction = multiply(direction, 1.0f / capsuleLength);

            Vertex perpendicular{ 0.0f, 0.0f, 1.0f };
            if (std::fabs(dot(direction, perpendicular)) > 0.999f) {
                perpendicular = Vertex{ 1.0f, 0.0f, 0.0f };
            }
            Vertex axis = normalized(cross(direction, perpendicular));
            perpendicular = normalized(cross(axis, direction));

            for (int segment = 0; segment <= segments; ++segment) {
                const float theta = segment / static_cast<float>(segments) * pi * 2.0f;
                for (int endpoint = 0; endpoint < 2; ++endpoint) {
                    mesh.vertices.push_back(add(
                        add(vertexA, multiply(direction, endpoint * capsuleLength)),
                        add(multiply(axis, std::cos(theta) * radius), multiply(perpendicular, std::sin(theta) * radius))));
                }
            }

            for (int segment = 0; segment < segments; ++segment) {
                const int index = segment * 2;
                mesh.indices.push_back(static_cast<std::uint16_t>(index));
                mesh.indices.push_back(static_cast<std::uint16_t>(index + 1));
                mesh.indices.push_back(static_cast<std::uint16_t>(index + 2));
                mesh.indices.push_back(static_cast<std::uint16_t>(index + 2));
                mesh.indices.push_back(static_cast<std::uint16_t>(index + 1));
                mesh.indices.push_back(static_cast<std::uint16_t>(index + 3));
            }

            const auto addCap = [&](const Vertex& center, bool upper) {
                const int base = static_cast<int>(mesh.vertices.size());
                for (int ring = 0; ring <= rings; ++ring) {
                    const float v = ring / static_cast<float>(rings);
                    const float theta = upper ? v * pi / 2.0f : pi / 2.0f + v * pi / 2.0f;
                    for (int segment = 0; segment <= segments; ++segment) {
                        const float u = segment / static_cast<float>(segments);
                        const float phi = u * pi * 2.0f;
                        const float x = std::cos(phi) * std::sin(theta);
                        const float y = std::cos(theta);
                        const float z = std::sin(phi) * std::sin(theta);
                        mesh.vertices.push_back(add(center,
                            add(add(multiply(axis, x * radius), multiply(direction, y * radius)), multiply(perpendicular, z * radius))));
                    }
                }

                for (int ring = 0; ring < rings; ++ring) {
                    for (int segment = 0; segment < segments; ++segment) {
                        const int index = base + ring * (segments + 1) + segment;
                        mesh.indices.push_back(static_cast<std::uint16_t>(index));
                        mesh.indices.push_back(static_cast<std::uint16_t>(index + segments + 1));
                        mesh.indices.push_back(static_cast<std::uint16_t>(index + 1));
                        mesh.indices.push_back(static_cast<std::uint16_t>(index + 1));
                        mesh.indices.push_back(static_cast<std::uint16_t>(index + segments + 1));
                        mesh.indices.push_back(static_cast<std::uint16_t>(index + segments + 2));
                    }
                }
            };

            addCap(vertexB, true);
            addCap(vertexA, false);
            mesh.valid = true;
            return mesh;
        }

        BuiltShape makeConvex(const ShapeRecipe& recipe)
        {
            BuiltShape result{};
            result.shapeType = recipe.shapeType;

            std::vector<debug_convex_hull_mesh::Vec3> rawVertices;
            rawVertices.reserve(recipe.vertices.size());
            for (const auto& vertex : recipe.vertices) {
                rawVertices.push_back(debug_convex_hull_mesh::Vec3{ vertex.x, vertex.y, vertex.z });
            }

            auto hullVertices = debug_convex_hull_mesh::deduplicateVertices(rawVertices);
            if (hullVertices.size() < 3) {
                return result;
            }

            result.decodeMode = debug_overlay_policy::chooseShapeDecodeMode(
                recipe.shapeType,
                static_cast<std::uint32_t>(hullVertices.size()),
                static_cast<int>(recipe.settings.maxConvexSupportVertices),
                recipe.settings.useBoundsForHeavyConvex);
            if (result.decodeMode == debug_overlay_policy::ShapeDecodeMode::Proxy) {
                std::vector<Vertex> boundsVertices;
                boundsVertices.reserve(hullVertices.size());
                for (const auto& vertex : hullVertices) {
                    boundsVertices.push_back(Vertex{ vertex.x, vertex.y, vertex.z });
                }
                result.mesh = makeBoundsBox(boundsVertices);
                return result;
            }

            if (recipe.convexRadius > 0.0f) {
                debug_convex_hull_mesh::inflateVerticesFromCentroid(
                    hullVertices, recipe.convexRadius * recipe.settings.havokToGameScale);
            }

            const auto triangles = debug_convex_hull_mesh::triangulateConvexHullFaces(hullVertices);
            if (triangles.empty() ||
                triangles.size() > debug_convex_hull_mesh::maxExpectedConvexHullTriangles(static_cast<std::uint32_t>(hullVertices.size()))) {
                if (!recipe.settings.useBoundsForHeavyConvex) {
                    result.decodeMode = debug_overlay_policy::ShapeDecodeMode::Unsupported;
                    return result;
                }
                std::vector<Vertex> boundsVertices;
                boundsVertices.reserve(hullVertices.size());
                for (const auto& vertex : hullVertices) {
                    boundsVertices.push_back(Vertex{ vertex.x, vertex.y, vertex.z });
                }
                result.mesh = makeBoundsBox(boundsVertices);
                result.decodeMode = debug_overlay_policy::ShapeDecodeMode::Proxy;
                return result;
            }

            result.mesh.indices.reserve(triangles.size() * 3);
            for (const auto& triangle : triangles) {
                result.mesh.indices.push_back(triangle[0]);
                result.mesh.indices.push_back(triangle[1]);
                result.mesh.indices.push_back(triangle[2]);
            }
            result.mesh.vertices.reserve(hullVertices.size());
            for (const auto& vertex : hullVertices) {
                result.mesh.vertices.push_back(Vertex{ vertex.x, vertex.y, vertex.z });
            }
            result.mesh.valid = !result.mesh.vertices.empty() && !result.mesh.indices.empty();
            result.decodeMode = result.mesh.valid ? debug_overlay_policy::ShapeDecodeMode::Detailed : debug_overlay_policy::ShapeDecodeMode::Unsupported;
            return result;
        }
    }

    BuiltShape buildMeshFromRecipe(const ShapeRecipe& recipe)
    {
        BuiltShape result{};
        result.shapeType = recipe.shapeType;
        if (!recipe.valid || !std::isfinite(recipe.settings.havokToGameScale) || recipe.settings.havokToGameScale <= 0.0f) {
            return result;
        }

        switch (recipe.kind) {
        case ShapeRecipe::Kind::Sphere:
            result.mesh = makeSphere(recipe.convexRadius, recipe.settings.havokToGameScale);
            result.decodeMode = debug_overlay_policy::ShapeDecodeMode::Detailed;
            return result;
        case ShapeRecipe::Kind::Capsule:
            result.mesh = makeCapsule(recipe);
            result.decodeMode = debug_overlay_policy::ShapeDecodeMode::Detailed;
            return result;
        case ShapeRecipe::Kind::ConvexVertices:
            return makeConvex(recipe);
        case ShapeRecipe::Kind::ScaledConvex:
            if (!recipe.inner) {
                return result;
            }
            result = buildMeshFromRecipe(*recipe.inner);
            if (!result.mesh.valid) {
                result.shapeType = recipe.shapeType;
                return result;
            }
            for (auto& vertex : result.mesh.vertices) {
                vertex.x *= recipe.scale[0];
                vertex.y *= recipe.scale[1];
                vertex.z *= recipe.scale[2];
            }
            result.shapeType = recipe.shapeType;
            return result;
        case ShapeRecipe::Kind::Unsupported:
        default:
            return result;
        }
    }

    MeshData makeUnitBoxMesh()
    {
        return makeBoundsBox(std::vector<Vertex>{
            { -0.5f, -0.5f, -0.5f },
            { 0.5f, 0.5f, 0.5f },
        });
    }

    std::size_t approximateGpuBytes(const MeshData& mesh) noexcept
    {
        return mesh.vertices.size() * sizeof(Vertex) + mesh.indices.size() * sizeof(std::uint16_t);
    }
}
