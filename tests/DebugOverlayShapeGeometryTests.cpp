#include <cmath>
#include <iostream>
#include <memory>
#include <utility>

#include "physics-interaction/debug/DebugOverlayShapeGeometry.h"

namespace
{
    bool expect(bool condition, const char* message)
    {
        if (!condition) {
            std::cerr << message << '\n';
            return false;
        }
        return true;
    }

    rock::debug_overlay_shape::ShapeRecipe baseRecipe()
    {
        rock::debug_overlay_shape::ShapeRecipe recipe{};
        recipe.settings.havokToGameScale = 10.0f;
        recipe.settings.maxConvexSupportVertices = 32;
        recipe.settings.useBoundsForHeavyConvex = true;
        recipe.valid = true;
        return recipe;
    }
}

int main()
{
    using namespace rock::debug_overlay_shape;
    bool passed = true;

    auto sphere = baseRecipe();
    sphere.kind = ShapeRecipe::Kind::Sphere;
    sphere.shapeType = 2;
    sphere.convexRadius = 0.5f;
    const auto sphereBuilt = buildMeshFromRecipe(sphere);
    passed &= expect(sphereBuilt.mesh.valid, "Sphere recipe did not build.");
    passed &= expect(sphereBuilt.mesh.vertices.size() == 169, "Sphere tessellation vertex count changed.");
    passed &= expect(sphereBuilt.mesh.indices.size() == 864, "Sphere tessellation index count changed.");
    passed &= expect(sphereBuilt.decodeMode == rock::debug_overlay_policy::ShapeDecodeMode::Detailed, "Sphere must remain detailed.");

    auto capsule = baseRecipe();
    capsule.kind = ShapeRecipe::Kind::Capsule;
    capsule.shapeType = 3;
    capsule.convexRadius = 0.25f;
    capsule.vertexA = { 0.0f, 0.0f, 0.0f, 0.0f };
    capsule.vertexB = { 0.0f, 0.0f, 1.0f, 0.0f };
    const auto capsuleBuilt = buildMeshFromRecipe(capsule);
    passed &= expect(capsuleBuilt.mesh.valid, "Capsule recipe did not build.");
    passed &= expect(capsuleBuilt.mesh.vertices.size() == 208, "Capsule tessellation vertex count changed.");
    passed &= expect(capsuleBuilt.mesh.indices.size() == 936, "Capsule tessellation index count changed.");

    auto tetrahedron = baseRecipe();
    tetrahedron.kind = ShapeRecipe::Kind::ConvexVertices;
    tetrahedron.shapeType = 1;
    tetrahedron.vertices = {
        { 0.0f, 0.0f, 0.0f },
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f },
    };
    const auto tetrahedronBuilt = buildMeshFromRecipe(tetrahedron);
    passed &= expect(tetrahedronBuilt.mesh.valid, "Convex recipe did not build.");
    passed &= expect(tetrahedronBuilt.decodeMode == rock::debug_overlay_policy::ShapeDecodeMode::Detailed, "Small convex hull must remain detailed.");
    passed &= expect(tetrahedronBuilt.mesh.vertices.size() == 4, "Convex recipe lost vertices.");
    passed &= expect(tetrahedronBuilt.mesh.indices.size() == 12, "Tetrahedron must contain four triangle faces.");

    auto heavyConvex = baseRecipe();
    heavyConvex.kind = ShapeRecipe::Kind::ConvexVertices;
    heavyConvex.shapeType = 1;
    heavyConvex.settings.maxConvexSupportVertices = 4;
    heavyConvex.vertices = {
        { -1.0f, -1.0f, -1.0f }, { 1.0f, -1.0f, -1.0f },
        { 1.0f, 1.0f, -1.0f }, { -1.0f, 1.0f, -1.0f },
        { -1.0f, -1.0f, 1.0f }, { 1.0f, -1.0f, 1.0f },
        { 1.0f, 1.0f, 1.0f }, { -1.0f, 1.0f, 1.0f },
    };
    const auto heavyBuilt = buildMeshFromRecipe(heavyConvex);
    passed &= expect(heavyBuilt.mesh.valid, "Heavy-convex bounds recipe did not build.");
    passed &= expect(heavyBuilt.decodeMode == rock::debug_overlay_policy::ShapeDecodeMode::Proxy, "Heavy convex must use its configured bounds mode.");
    passed &= expect(heavyBuilt.mesh.vertices.size() == 8 && heavyBuilt.mesh.indices.size() == 36, "Heavy-convex bounds topology changed.");

    auto scaled = baseRecipe();
    scaled.kind = ShapeRecipe::Kind::ScaledConvex;
    scaled.shapeType = 11;
    scaled.scale = { 2.0f, 3.0f, 4.0f, 0.0f };
    scaled.inner = std::make_unique<ShapeRecipe>(std::move(sphere));
    const auto scaledBuilt = buildMeshFromRecipe(scaled);
    passed &= expect(scaledBuilt.mesh.valid, "Scaled-convex recipe did not build.");
    passed &= expect(scaledBuilt.shapeType == 11, "Scaled-convex outer type was not retained.");
    passed &= expect(std::fabs(scaledBuilt.mesh.vertices[1].y - sphereBuilt.mesh.vertices[1].y * 3.0f) < 0.0001f,
        "Scaled-convex Y scale was not applied exactly once.");

    const auto unitBox = makeUnitBoxMesh();
    passed &= expect(unitBox.valid && unitBox.vertices.size() == 8 && unitBox.indices.size() == 36,
        "Canonical AABB proxy topology is invalid.");
    passed &= expect(approximateGpuBytes(unitBox) == 8 * sizeof(Vertex) + 36 * sizeof(std::uint16_t),
        "GPU byte estimate does not match uploaded topology.");

    return passed ? 0 : 1;
}
