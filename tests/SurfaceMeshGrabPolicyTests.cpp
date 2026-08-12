#include "physics-interaction/grab/SurfaceMeshGrabPolicy.h"

#ifdef NDEBUG
#    undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <vector>

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

    auto targetWorld = identityTransform();
    targetWorld.translate = { 10.0f, 0.0f, 0.0f };
    const std::vector<rock::TriangleData> triangles{
        {
            { 30.0f, 0.0f, 0.0f },
            { 31.0f, 0.0f, 0.0f },
            { 30.0f, 1.0f, 0.0f },
        },
        {
            { 11.0f, 0.0f, 0.0f },
            { 12.0f, 0.0f, 0.0f },
            { 11.0f, 1.0f, 0.0f },
        },
    };
    const auto patch = policy::buildTargetLocalPatch(
        triangles,
        targetWorld,
        { 11.25f, 0.25f, 0.0f },
        1);
    assert(patch.size() == 1);
    assert(nearlyEqual(patch[0].v0.x, 1.0f));
    assert(nearlyEqual(patch[0].v0.y, 0.0f));
    assert(nearlyEqual(patch[0].v0.z, 0.0f));

    return 0;
}
