#include "physics-interaction/grab/SurfaceMeshGrabPolicy.h"

#include <cassert>
#include <cmath>
#include <limits>

namespace
{
    RE::NiTransform identityTransform()
    {
        RE::NiTransform transform{};
        transform.rotate.entry[0][0] = 1.0f;
        transform.rotate.entry[1][1] = 1.0f;
        transform.rotate.entry[2][2] = 1.0f;
        transform.scale = 1.0f;
        return transform;
    }

    bool nearlyEqual(const float lhs, const float rhs)
    {
        return std::abs(lhs - rhs) < 0.0001f;
    }
}

int main()
{
    namespace policy = rock::surface_mesh_grab_policy;

    auto handWorld = identityTransform();
    handWorld.translate = { 10.0f, 20.0f, 30.0f };
    const auto projected = policy::projectHandToMesh(
        policy::ProjectionInput{
            .handWorld = handWorld,
            .collisionPointWorld = { 2.0f, 4.0f, 6.0f },
            .collisionNormalWorld = { 0.0f, 0.0f, 1.0f },
            .meshPointWorld = { 2.0f, 4.0f, 3.0f },
            .meshNormalWorld = { 0.0f, 0.0f, 2.0f },
            .maximumProjectionDistanceGameUnits = 4.0f,
            .hasCollisionNormal = true,
            .hasMeshNormal = true,
        });
    assert(projected.valid);
    assert(nearlyEqual(projected.shellToMeshDistanceGameUnits, 3.0f));
    assert(nearlyEqual(projected.correctedHandWorld.translate.x, 10.0f));
    assert(nearlyEqual(projected.correctedHandWorld.translate.y, 20.0f));
    assert(nearlyEqual(projected.correctedHandWorld.translate.z, 27.0f));
    assert(nearlyEqual(projected.meshNormalWorld.z, 1.0f));
    assert(nearlyEqual(projected.correctedHandWorld.rotate.entry[0][0], 1.0f));

    const auto rejected = policy::projectHandToMesh(
        policy::ProjectionInput{
            .handWorld = handWorld,
            .collisionPointWorld = {},
            .meshPointWorld = { 0.0f, 0.0f, 5.0f },
            .maximumProjectionDistanceGameUnits = 4.0f,
        });
    assert(!rejected.valid);

    policy::ProjectionInput boundary{
        .handWorld = handWorld,
        .collisionPointWorld = {},
        .meshPointWorld = { 0.0f, 0.0f, 4.0f },
        .maximumProjectionDistanceGameUnits = 4.0f,
    };
    // Translation correction is in world space even for a rotated, scaled hand.
    boundary.handWorld.scale = 2.0f;
    boundary.handWorld.rotate.entry[0][0] = 0.0f;
    boundary.handWorld.rotate.entry[0][1] = -1.0f;
    boundary.handWorld.rotate.entry[1][0] = 1.0f;
    boundary.handWorld.rotate.entry[1][1] = 0.0f;
    const auto atBoundary = policy::projectHandToMesh(boundary);
    assert(atBoundary.valid);
    assert(nearlyEqual(atBoundary.correctedHandWorld.translate.z, 34.0f));
    assert(nearlyEqual(atBoundary.correctedHandWorld.scale, 2.0f));
    assert(nearlyEqual(atBoundary.correctedHandWorld.rotate.entry[0][1], -1.0f));
    assert(nearlyEqual(atBoundary.meshNormalWorld.z, 0.0f));
    boundary.meshPointWorld.z = std::nextafter(4.0f, 5.0f);
    assert(!policy::projectHandToMesh(boundary).valid);
    boundary.meshPointWorld.z = 4.0f;

    boundary.hasCollisionNormal = true;
    boundary.collisionNormalWorld = { 0.0f, 3.0f, 0.0f };
    assert(nearlyEqual(policy::projectHandToMesh(boundary).meshNormalWorld.y, 1.0f));
    boundary.hasMeshNormal = true;
    boundary.meshNormalWorld = { std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f };
    // Invalid normals become unavailable direction data; position remains usable.
    const auto invalidNormal = policy::projectHandToMesh(boundary);
    assert(invalidNormal.valid);
    assert(nearlyEqual(invalidNormal.meshNormalWorld.x, 0.0f));
    assert(nearlyEqual(invalidNormal.meshNormalWorld.y, 0.0f));
    boundary.handWorld.scale = 0.0f;
    assert(!policy::projectHandToMesh(boundary).valid);
    boundary.handWorld.scale = 2.0f;
    boundary.meshPointWorld.x = std::numeric_limits<float>::infinity();
    assert(!policy::projectHandToMesh(boundary).valid);

    return 0;
}
