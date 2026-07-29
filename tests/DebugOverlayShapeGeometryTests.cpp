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
        recipe.settings.maxCompoundChildren = 256;
        recipe.settings.maxCompoundDepth = 4;
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

    auto canonicalSphere = baseRecipe();
    canonicalSphere.kind = ShapeRecipe::Kind::Sphere;
    canonicalSphere.shapeType = 2;
    canonicalSphere.convexRadius = 0.125f;
    canonicalSphere.canonicalUnitSphere = true;
    const auto canonicalSphereBuilt = buildMeshFromRecipe(canonicalSphere);
    passed &= expect(canonicalSphereBuilt.mesh.valid && std::fabs(canonicalSphereBuilt.mesh.vertices[1].y - 1.0f) < 0.0001f,
        "Canonical direct-sphere geometry baked the source radius instead of using unit radius.");

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

    auto triangle = baseRecipe();
    triangle.kind = ShapeRecipe::Kind::Triangle;
    triangle.shapeType = 4;
    triangle.convexRadius = 0.25f;
    triangle.vertices = {
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f },
    };
    const auto triangleBuilt = buildMeshFromRecipe(triangle);
    passed &= expect(triangleBuilt.mesh.valid && triangleBuilt.mesh.vertices == triangle.vertices,
        "Triangle recipe changed its verified support vertices.");
    passed &= expect(triangleBuilt.mesh.indices == std::vector<std::uint16_t>{ 0, 1, 2, 2, 1, 0 },
        "Triangle recipe is not double-sided.");

    auto scaled = baseRecipe();
    scaled.kind = ShapeRecipe::Kind::ScaledConvex;
    scaled.shapeType = 11;
    scaled.scale = { 2.0f, 3.0f, 4.0f, 0.0f };
    scaled.translation = { 1.0f, 2.0f, 3.0f, 0.0f };
    scaled.inner = std::make_unique<ShapeRecipe>(std::move(sphere));
    const auto scaledBuilt = buildMeshFromRecipe(scaled);
    passed &= expect(scaledBuilt.mesh.valid, "Scaled-convex recipe did not build.");
    passed &= expect(scaledBuilt.shapeType == 11, "Scaled-convex outer type was not retained.");
    passed &= expect(std::fabs(scaledBuilt.mesh.vertices[1].y - (sphereBuilt.mesh.vertices[1].y * 3.0f + 20.0f)) < 0.0001f,
        "Scaled-convex Y scale and Havok-unit translation were not applied exactly once.");

    auto compound = baseRecipe();
    compound.kind = ShapeRecipe::Kind::Compound;
    compound.shapeType = 7;
    compound.children.emplace_back();
    auto& compoundChild = compound.children.back();
    compoundChild.transform = {
        0.0f, 1.0f, 0.0f, 0.0f,
        -1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        1.0f, 2.0f, 3.0f, 1.0f
    };
    compoundChild.scale = { 2.0f, 3.0f, 4.0f, 0.0f };
    compoundChild.recipe = std::make_unique<ShapeRecipe>(std::move(triangle));
    const auto compoundBuilt = buildMeshFromRecipe(compound);
    passed &= expect(compoundBuilt.mesh.valid && compoundBuilt.shapeType == 7,
        "Verified static-compound recipe did not build.");
    passed &= expect(std::fabs(compoundBuilt.mesh.vertices[0].x - 10.0f) < 0.0001f &&
                         std::fabs(compoundBuilt.mesh.vertices[0].y - 22.0f) < 0.0001f &&
                         std::fabs(compoundBuilt.mesh.vertices[0].z - 30.0f) < 0.0001f,
        "Compound child scale/rotation/translation order changed.");

    auto incompleteCompound = baseRecipe();
    incompleteCompound.kind = ShapeRecipe::Kind::Compound;
    incompleteCompound.shapeType = 8;
    incompleteCompound.children.emplace_back();
    incompleteCompound.children.back().recipe = std::make_unique<ShapeRecipe>();
    const auto incompleteBuilt = buildMeshFromRecipe(incompleteCompound);
    passed &= expect(!incompleteBuilt.mesh.valid && incompleteBuilt.decodeMode == rock::debug_overlay_policy::ShapeDecodeMode::Unsupported,
        "A compound with an unsupported active child produced a partial mesh instead of failing to AABB fallback.");

    const auto unitBox = makeUnitBoxMesh();
    passed &= expect(unitBox.valid && unitBox.vertices.size() == 8 && unitBox.indices.size() == 36,
        "Canonical AABB proxy topology is invalid.");
    passed &= expect(approximateGpuBytes(unitBox) == 8 * sizeof(Vertex) + 36 * sizeof(std::uint16_t),
        "GPU byte estimate does not match uploaded topology.");

    return passed ? 0 : 1;
}
