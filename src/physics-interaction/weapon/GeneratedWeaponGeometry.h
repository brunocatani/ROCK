#pragma once

#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/weapon/WeaponTriangleIndex.h"

#include <memory>

namespace rock
{
    // Built before source publication, then shared as const by the source
    // cache, pending construction and body banks. No native object ownership.
    struct GeneratedWeaponMeshGeometry
    {
        std::vector<TriangleData> localTrianglesGame;
        std::vector<TriangleData> sourceLocalTrianglesGame;
    };

    // Built once per retained mesh after source admission, before publication.
    // Kept separate so rejected geometry never pays for acceleration storage.
    struct GeneratedWeaponMeshIndices
    {
        WeaponTriangleIndex localIndex;
        WeaponTriangleIndex sourceIndex;
    };

    struct GeneratedWeaponHullGeometry
    {
        std::vector<RE::NiPoint3> localPointsGame;
        std::vector<RE::NiPoint3> sourceLocalPointsGame;
        std::vector<std::vector<RE::NiPoint3>> childLocalPointCloudsGame;
        std::shared_ptr<const GeneratedWeaponMeshGeometry> mesh;
    };
}
