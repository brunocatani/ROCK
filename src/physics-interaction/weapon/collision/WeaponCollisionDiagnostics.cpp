#include "physics-interaction/weapon/WeaponCollisionInternal.h"

// Collision diagnostics: generated-recapture diagnostics and the weapon animation-node dump.

namespace rock
{
    void WeaponCollision::recordGeneratedRecaptureDiagnostic(
        const std::uint64_t equippedKey,
        const std::uint64_t identityKey,
        const std::uint64_t ownershipKey,
        const std::uint32_t weaponFormID,
        const std::vector<GeneratedHullSource>& sources)
    {
        if (equippedKey == 0 || identityKey == 0 || ownershipKey == 0 ||
            weaponFormID == 0 || sources.empty()) {
            return;
        }

        const auto pointDistance = [](const RE::NiPoint3& lhs, const RE::NiPoint3& rhs) {
            const float dx = lhs.x - rhs.x;
            const float dy = lhs.y - rhs.y;
            const float dz = lhs.z - rhs.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        };
        const auto captureCurrent = [&](const std::uint32_t comparisonSequence) {
            GeneratedRecaptureDiagnostic captured{};
            captured.valid = true;
            captured.equippedKey = equippedKey;
            captured.identityKey = identityKey;
            captured.ownershipKey = ownershipKey;
            captured.weaponFormID = weaponFormID;
            captured.comparisonSequence = comparisonSequence;
            captured.sources.reserve(sources.size());
            for (const auto& source : sources) {
                captured.sources.push_back(GeneratedRecaptureDiagnosticSource{
                    .sourceGroupId = source.sourceGroupId,
                    .sourceRootAddress = reinterpret_cast<std::uintptr_t>(source.sourceRoot),
                    .driveRootAddress = reinterpret_cast<std::uintptr_t>(source.driveRoot),
                    .sourceName = source.sourceName,
                    .weaponLocalCenter = source.localCenterGame,
                    .sourceLocalCenter = source.sourceLocalCenterGame,
                    .sourceLocalMin = source.sourceLocalMinGame,
                    .sourceLocalMax = source.sourceLocalMaxGame,
                    .mesh = source.geometry->mesh,
                    .sourceLocalPointCount = source.geometry->sourceLocalPointsGame.size(),
                    .sourceLocalTriangleCount = source.geometry->mesh->sourceLocalTrianglesGame.size(),
                    .sourceNodeScale = source.sourceNodeScale,
                });
            }
            _diagnostics.generatedRecapture = std::move(captured);
        };

        const bool sameExactIdentity =
            _diagnostics.generatedRecapture.valid &&
            _diagnostics.generatedRecapture.equippedKey == equippedKey &&
            _diagnostics.generatedRecapture.identityKey == identityKey &&
            _diagnostics.generatedRecapture.ownershipKey == ownershipKey &&
            _diagnostics.generatedRecapture.weaponFormID == weaponFormID;
        if (!sameExactIdentity) {
            captureCurrent(0);
            ROCK_LOG_DEBUG(Weapon,
                "Generated weapon recapture diagnostic baseline captured key={:016X} identity={:016X} ownership={:016X} formID={:08X} sources={}",
                equippedKey,
                identityKey,
                ownershipKey,
                weaponFormID,
                sources.size());
            return;
        }

        if (!_diagnostics.generatedRecapture.sawUndrawnInterval) {
            const auto comparisonSequence = _diagnostics.generatedRecapture.comparisonSequence;
            captureCurrent(comparisonSequence);
            return;
        }

        struct DriftRow
        {
            const GeneratedRecaptureDiagnosticSource* baseline{ nullptr };
            const GeneratedHullSource* current{ nullptr };
            float weaponCenterDeltaGame{ 0.0f };
            float sourceCenterDeltaGame{ 0.0f };
            float sourceBoundsDeltaGame{ 0.0f };
            float maximumSourceTriangleVertexDeltaGame{ 0.0f };
            bool sourcePointerStable{ false };
            bool rootPointerStable{ false };
            bool dedupPointCountStable{ false };
            bool triangleCountStable{ false };
            bool sourceGeometryStable{ false };
            bool sourceScaleStable{ false };
        };

        const bool traceEnabled = logger::isTraceEnabled();
        std::vector<DriftRow> driftRows;
        if (traceEnabled) driftRows.reserve(sources.size());
        bool allSourcesStable = true;
        struct MeshDelta
        {
            const GeneratedWeaponMeshGeometry* baseline = nullptr;
            const GeneratedWeaponMeshGeometry* current = nullptr;
            float maximum = 0.0f;
        };
        std::array<MeshDelta, MAX_WEAPON_BODIES> meshDeltas{};
        std::size_t meshDeltaCount = 0;
        std::size_t matchedSourceCount = 0;
        std::size_t sameSourcePointerCount = 0;
        std::size_t treeReplacementCount = 0;
        std::size_t hierarchyFrameDriftCount = 0;
        std::size_t sourceGeometryDriftCount = 0;
        float maximumWeaponCenterDeltaGame = 0.0f;
        float maximumSourceCenterDeltaGame = 0.0f;
        float maximumSourceTriangleVertexDeltaGame = 0.0f;
        const char* maximumWeaponCenterDeltaSource = "none";

        for (const auto& current : sources) {
            const GeneratedRecaptureDiagnosticSource* baseline = nullptr;
            if (current.sourceGroupId != 0) {
                const auto exact = std::find_if(
                    _diagnostics.generatedRecapture.sources.begin(),
                    _diagnostics.generatedRecapture.sources.end(),
                    [&](const GeneratedRecaptureDiagnosticSource& candidate) {
                        return candidate.sourceGroupId == current.sourceGroupId &&
                               candidate.sourceName == current.sourceName;
                    });
                if (exact != _diagnostics.generatedRecapture.sources.end()) {
                    baseline = &*exact;
                }
            }
            if (!baseline) {
                const auto sameName = std::find_if(
                    _diagnostics.generatedRecapture.sources.begin(),
                    _diagnostics.generatedRecapture.sources.end(),
                    [&](const GeneratedRecaptureDiagnosticSource& candidate) {
                        return candidate.sourceName == current.sourceName;
                    });
                if (sameName != _diagnostics.generatedRecapture.sources.end()) {
                    baseline = &*sameName;
                }
            }
            if (!baseline) {
                ++treeReplacementCount;
                continue;
            }

            ++matchedSourceCount;
            DriftRow row{};
            row.baseline = baseline;
            row.current = &current;
            row.weaponCenterDeltaGame = pointDistance(
                baseline->weaponLocalCenter,
                current.localCenterGame);
            row.sourceCenterDeltaGame = pointDistance(
                baseline->sourceLocalCenter,
                current.sourceLocalCenterGame);
            row.sourceBoundsDeltaGame = (std::max)(
                pointDistance(baseline->sourceLocalMin, current.sourceLocalMinGame),
                pointDistance(baseline->sourceLocalMax, current.sourceLocalMaxGame));
            row.sourcePointerStable =
                baseline->sourceGroupId != 0 &&
                baseline->sourceGroupId == current.sourceGroupId;
            row.rootPointerStable =
                baseline->sourceRootAddress ==
                    reinterpret_cast<std::uintptr_t>(current.sourceRoot) &&
                baseline->driveRootAddress ==
                    reinterpret_cast<std::uintptr_t>(current.driveRoot);
            row.triangleCountStable =
                baseline->sourceLocalTriangleCount ==
                    current.geometry->mesh->sourceLocalTrianglesGame.size();
            row.dedupPointCountStable =
                baseline->sourceLocalPointCount ==
                current.geometry->sourceLocalPointsGame.size();
            if (row.triangleCountStable &&
                baseline->mesh->sourceLocalTrianglesGame.size() == current.geometry->mesh->sourceLocalTrianglesGame.size()) {
                const auto deltaEnd = meshDeltas.begin() + meshDeltaCount;
                const auto cached = std::find_if(meshDeltas.begin(), deltaEnd, [&](const MeshDelta& delta) {
                    return delta.baseline == baseline->mesh.get() && delta.current == current.geometry->mesh.get();
                });
                if (cached != deltaEnd) {
                    row.maximumSourceTriangleVertexDeltaGame = cached->maximum;
                } else {
                    for (std::size_t triangleIndex = 0;
                         triangleIndex < current.geometry->mesh->sourceLocalTrianglesGame.size();
                         ++triangleIndex) {
                        const auto& authoritativeTriangle = baseline->mesh->sourceLocalTrianglesGame[triangleIndex];
                        const auto& currentTriangle = current.geometry->mesh->sourceLocalTrianglesGame[triangleIndex];
                        row.maximumSourceTriangleVertexDeltaGame = (std::max)({
                            row.maximumSourceTriangleVertexDeltaGame,
                            pointDistance(authoritativeTriangle.v0, currentTriangle.v0),
                            pointDistance(authoritativeTriangle.v1, currentTriangle.v1),
                            pointDistance(authoritativeTriangle.v2, currentTriangle.v2),
                        });
                    }
                    if (meshDeltaCount < meshDeltas.size()) {
                        meshDeltas[meshDeltaCount++] = { baseline->mesh.get(), current.geometry->mesh.get(),
                            row.maximumSourceTriangleVertexDeltaGame };
                    }
                }
            } else {
                row.maximumSourceTriangleVertexDeltaGame =
                    (std::numeric_limits<float>::infinity)();
            }
            row.sourceGeometryStable =
                row.triangleCountStable &&
                std::isfinite(row.maximumSourceTriangleVertexDeltaGame) &&
                row.maximumSourceTriangleVertexDeltaGame <=
                    GENERATED_RECAPTURE_SOURCE_CENTER_DRIFT_GAME;
            row.sourceScaleStable =
                std::isfinite(baseline->sourceNodeScale) &&
                std::isfinite(current.sourceNodeScale) &&
                std::abs(baseline->sourceNodeScale - current.sourceNodeScale) <= 0.001f;
            if (row.sourcePointerStable) {
                ++sameSourcePointerCount;
            }
            if (!row.sourcePointerStable || !row.rootPointerStable) {
                ++treeReplacementCount;
            }

            const bool sourceGeometryDrifted =
                !row.sourceGeometryStable ||
                !row.sourceScaleStable;
            if (sourceGeometryDrifted) {
                ++sourceGeometryDriftCount;
            } else if (row.sourcePointerStable && row.rootPointerStable &&
                       row.weaponCenterDeltaGame >
                           GENERATED_RECAPTURE_WEAPON_CENTER_DRIFT_GAME) {
                ++hierarchyFrameDriftCount;
            }

            maximumSourceCenterDeltaGame = (std::max)(
                maximumSourceCenterDeltaGame,
                row.sourceCenterDeltaGame);
            maximumSourceTriangleVertexDeltaGame = (std::max)(
                maximumSourceTriangleVertexDeltaGame,
                row.maximumSourceTriangleVertexDeltaGame);
            if (row.weaponCenterDeltaGame > maximumWeaponCenterDeltaGame) {
                maximumWeaponCenterDeltaGame = row.weaponCenterDeltaGame;
                maximumWeaponCenterDeltaSource = current.sourceName.c_str();
            }
            allSourcesStable = allSourcesStable && row.sourcePointerStable && row.rootPointerStable &&
                row.sourceGeometryStable && row.sourceScaleStable;
            if (traceEnabled) driftRows.push_back(row);
        }

        const std::size_t unmatchedBaselineCount =
            _diagnostics.generatedRecapture.sources.size() > matchedSourceCount ?
                _diagnostics.generatedRecapture.sources.size() - matchedSourceCount :
                0;
        treeReplacementCount += unmatchedBaselineCount;
        const char* classification = "stable";
        if (treeReplacementCount != 0 &&
            (hierarchyFrameDriftCount != 0 || sourceGeometryDriftCount != 0)) {
            classification = "mixed";
        } else if (treeReplacementCount != 0) {
            classification = "tree-replaced";
        } else if (sourceGeometryDriftCount != 0 && hierarchyFrameDriftCount != 0) {
            classification = "mixed";
        } else if (sourceGeometryDriftCount != 0) {
            classification = "source-geometry-drift";
        } else if (hierarchyFrameDriftCount != 0) {
            classification = "hierarchy-frame-drift";
        }

        const bool liveSourceContinuityValid =
            treeReplacementCount == 0 &&
            sourceGeometryDriftCount == 0 &&
            matchedSourceCount == sources.size() &&
            matchedSourceCount == _diagnostics.generatedRecapture.sources.size() &&
            allSourcesStable;

        ++_diagnostics.generatedRecapture.comparisonSequence;
        _diagnostics.generatedRecapture.sawUndrawnInterval = false;
        ROCK_LOG_INFO(Weapon,
            "Generated weapon post-undraw recapture diagnostic: sequence={} classification={} liveSourceContinuity={} key={:016X} identity={:016X} ownership={:016X} formID={:08X} baselineSources={} currentSources={} matched={} sameSourcePointers={} treeChanges={} hierarchyFrameDrift={} sourceGeometryDrift={} maxWeaponCenterDelta={:.3f} maxWeaponCenterSource='{}' maxSourceLocalCenterDelta={:.3f} maxSourceLocalTriangleDelta={:.3f}",
            _diagnostics.generatedRecapture.comparisonSequence,
            classification,
            liveSourceContinuityValid ? "yes" : "no",
            equippedKey,
            identityKey,
            ownershipKey,
            weaponFormID,
            _diagnostics.generatedRecapture.sources.size(),
            sources.size(),
            matchedSourceCount,
            sameSourcePointerCount,
            treeReplacementCount,
            hierarchyFrameDriftCount,
            sourceGeometryDriftCount,
            maximumWeaponCenterDeltaGame,
            maximumWeaponCenterDeltaSource,
            maximumSourceCenterDeltaGame,
            maximumSourceTriangleVertexDeltaGame);

        if (!traceEnabled) return;

        std::sort(
            driftRows.begin(),
            driftRows.end(),
            [](const DriftRow& lhs, const DriftRow& rhs) {
                return lhs.weaponCenterDeltaGame > rhs.weaponCenterDeltaGame;
            });
        std::size_t detailCount = 0;
        for (const auto& row : driftRows) {
            if (!row.baseline || !row.current ||
                (row.weaponCenterDeltaGame <=
                     GENERATED_RECAPTURE_WEAPON_CENTER_DRIFT_GAME &&
                    row.sourceCenterDeltaGame <=
                        GENERATED_RECAPTURE_SOURCE_CENTER_DRIFT_GAME &&
                    row.sourcePointerStable && row.rootPointerStable &&
                    row.sourceGeometryStable && row.sourceScaleStable &&
                    row.dedupPointCountStable)) {
                continue;
            }
            ROCK_LOG_TRACE(Weapon,
                "Generated weapon recapture drift[{}]: source='{}' sourcePointerStable={} rootPointersStable={} dedupPoints={}->{} triangles={}->{} sourceGeometryStable={} sourceScaleStable={} weaponCenterDelta={:.3f} sourceLocalCenterDelta={:.3f} sourceLocalBoundsDelta={:.3f} sourceLocalTriangleDelta={:.3f} weaponCenter=({:.3f},{:.3f},{:.3f})->({:.3f},{:.3f},{:.3f}) sourceLocalCenter=({:.3f},{:.3f},{:.3f})->({:.3f},{:.3f},{:.3f})",
                detailCount,
                row.current->sourceName,
                row.sourcePointerStable ? "yes" : "no",
                row.rootPointerStable ? "yes" : "no",
                row.baseline->sourceLocalPointCount,
                row.current->geometry->sourceLocalPointsGame.size(),
                row.baseline->sourceLocalTriangleCount,
                row.current->geometry->mesh->sourceLocalTrianglesGame.size(),
                row.sourceGeometryStable ? "yes" : "no",
                row.sourceScaleStable ? "yes" : "no",
                row.weaponCenterDeltaGame,
                row.sourceCenterDeltaGame,
                row.sourceBoundsDeltaGame,
                row.maximumSourceTriangleVertexDeltaGame,
                row.baseline->weaponLocalCenter.x,
                row.baseline->weaponLocalCenter.y,
                row.baseline->weaponLocalCenter.z,
                row.current->localCenterGame.x,
                row.current->localCenterGame.y,
                row.current->localCenterGame.z,
                row.baseline->sourceLocalCenter.x,
                row.baseline->sourceLocalCenter.y,
                row.baseline->sourceLocalCenter.z,
                row.current->sourceLocalCenterGame.x,
                row.current->sourceLocalCenterGame.y,
                row.current->sourceLocalCenterGame.z);
            if (++detailCount >= MAX_GENERATED_RECAPTURE_DETAIL_ROWS) {
                break;
            }
        }

        /*
         * The comparison is diagnostic only. Every generated shape preserves
         * native source-local geometry and both keyframed bodies and the live
         * dynamic compound resolve the current descendant-local transform on
         * every update. Applying a correction captured from one draw-animation
         * frame would become stale as the visible part finishes moving and
         * would separate collision from the rendered mesh.
         */
    }


    void WeaponCollision::maybeDumpWeaponAnimNodeDiagnostics(RE::NiAVObject* updateWeaponNode, std::uint64_t observedKey)
    {
        if (!g_rockConfig.rockDebugDumpWeaponAnimNodes) {
            _diagnostics.animNodeDumpFrameCounter = 0;
            _diagnostics.lastAnimNodeDumpKey = 0;
            return;
        }

        const bool generationChanged = observedKey != _diagnostics.lastAnimNodeDumpKey;
        const int intervalFrames = (std::max)(1, g_rockConfig.rockDebugWeaponAnimNodeDumpIntervalFrames);
        const bool intervalDue = ++_diagnostics.animNodeDumpFrameCounter >= intervalFrames;
        if (!generationChanged && !intervalDue) {
            return;
        }

        _diagnostics.animNodeDumpFrameCounter = 0;
        _diagnostics.lastAnimNodeDumpKey = observedKey;

        auto* firstPersonBoneTree = f4vr::getFirstPersonBoneTree();
        auto* gameFlattenedBoneTree = f4vr::getFlattenedBoneTree();
        auto* firstPersonSkeleton = f4vr::getFirstPersonSkeleton();
        auto* gameRootNode = f4vr::getRootNode();
        auto* weaponNode = f4vr::getWeaponNode();
        auto* player = f4vr::getPlayer();
        auto* playerNodes = player ? f4vr::getPlayerNodes() : nullptr;

        ROCK_LOG_INFO(Weapon,
            "WeaponAnimDump begin key={:016X} reason={} firstPersonSkeleton=0x{:X} firstPersonBoneTree=0x{:X} gameRootNode='{}'/0x{:X} gameFlattenedBoneTree=0x{:X} updateWeaponNode='{}'/0x{:X} getWeaponNode='{}'/0x{:X} WeaponLeftNode='{}'/0x{:X} primaryWeapontoWeaponNode='{}'/0x{:X} primaryWeaponOffsetNode='{}'/0x{:X}",
            observedKey,
            generationChanged ? "generation-change" : "interval",
            reinterpret_cast<std::uintptr_t>(firstPersonSkeleton),
            reinterpret_cast<std::uintptr_t>(firstPersonBoneTree),
            safeNodeName(gameRootNode),
            reinterpret_cast<std::uintptr_t>(gameRootNode),
            reinterpret_cast<std::uintptr_t>(gameFlattenedBoneTree),
            safeNodeName(updateWeaponNode),
            reinterpret_cast<std::uintptr_t>(updateWeaponNode),
            safeNodeName(weaponNode),
            reinterpret_cast<std::uintptr_t>(weaponNode),
            playerNodes ? safeNodeName(playerNodes->WeaponLeftNode) : "(null)",
            reinterpret_cast<std::uintptr_t>(playerNodes ? playerNodes->WeaponLeftNode : nullptr),
            playerNodes ? safeNodeName(playerNodes->primaryWeapontoWeaponNode) : "(null)",
            reinterpret_cast<std::uintptr_t>(playerNodes ? playerNodes->primaryWeapontoWeaponNode : nullptr),
            playerNodes ? safeNodeName(playerNodes->primaryWeaponOffsetNOde) : "(null)",
            reinterpret_cast<std::uintptr_t>(playerNodes ? playerNodes->primaryWeaponOffsetNOde : nullptr));

        for (const char* targetName : WEAPON_ANIM_NODE_DUMP_TARGETS) {
            auto matches = collectWeaponAnimNodeMatches(firstPersonSkeleton, targetName);
            ROCK_LOG_INFO(Weapon, "WeaponAnimDump target='{}' matches={}", targetName, matches.size());

            for (std::size_t matchIndex = 0; matchIndex < matches.size(); ++matchIndex) {
                auto* node = matches[matchIndex].node;
                if (!node) {
                    continue;
                }

                auto* niNode = node->IsNode();
                const auto childCount = niNode ? niNode->children.size() : 0;
                const auto stats = summarizeWeaponAnimNodeSubtree(node);
                const auto childNames = weaponAnimNodeImmediateChildNames(node);
                auto* parent = node->parent;

                ROCK_LOG_INFO(Weapon,
                    "WeaponAnimDump node target='{}' match={} path='{}' depth={} addr=0x{:X} name='{}' parent='{}'/0x{:X} children={} childNames='{}' flags=0x{:X} appCulled={} visible={} subtreeNodes={} niNodes={} triShapes={} visibleTriShapes={} hiddenFlags={} appCulledNodes={} subtreeMaxDepth={}",
                    targetName,
                    matchIndex,
                    matches[matchIndex].path,
                    matches[matchIndex].depth,
                    reinterpret_cast<std::uintptr_t>(node),
                    safeNodeName(node),
                    safeNodeName(parent),
                    reinterpret_cast<std::uintptr_t>(parent),
                    static_cast<std::size_t>(childCount),
                    childNames,
                    static_cast<std::uint32_t>(node->flags.flags),
                    node->GetAppCulled() ? "yes" : "no",
                    weaponVisualNodeVisible(node) ? "yes" : "no",
                    stats.nodeCount,
                    stats.niNodeCount,
                    stats.triShapeCount,
                    stats.visibleTriShapeCount,
                    stats.hiddenFlagCount,
                    stats.appCulledCount,
                    stats.maxDepth);

                ROCK_LOG_INFO(Weapon,
                    "WeaponAnimDump transform target='{}' match={} localT=({:.3f},{:.3f},{:.3f}) localScale={:.3f} localR=[{:.3f},{:.3f},{:.3f};{:.3f},{:.3f},{:.3f};{:.3f},{:.3f},{:.3f}] worldT=({:.3f},{:.3f},{:.3f}) worldScale={:.3f} worldR=[{:.3f},{:.3f},{:.3f};{:.3f},{:.3f},{:.3f};{:.3f},{:.3f},{:.3f}]",
                    targetName,
                    matchIndex,
                    node->local.translate.x,
                    node->local.translate.y,
                    node->local.translate.z,
                    node->local.scale,
                    node->local.rotate.entry[0][0],
                    node->local.rotate.entry[0][1],
                    node->local.rotate.entry[0][2],
                    node->local.rotate.entry[1][0],
                    node->local.rotate.entry[1][1],
                    node->local.rotate.entry[1][2],
                    node->local.rotate.entry[2][0],
                    node->local.rotate.entry[2][1],
                    node->local.rotate.entry[2][2],
                    node->world.translate.x,
                    node->world.translate.y,
                    node->world.translate.z,
                    node->world.scale,
                    node->world.rotate.entry[0][0],
                    node->world.rotate.entry[0][1],
                    node->world.rotate.entry[0][2],
                    node->world.rotate.entry[1][0],
                    node->world.rotate.entry[1][1],
                    node->world.rotate.entry[1][2],
                    node->world.rotate.entry[2][0],
                    node->world.rotate.entry[2][1],
                    node->world.rotate.entry[2][2]);
            }
        }

        // Debug-only authority map: collider generation still follows updateWeaponNode,
        // while these rows show whether the flat-root data has names the visual tree lost.
        const std::array<WeaponAnimNodeDumpRoot, 7> nodeRoots{ {
            { "firstPersonSkeleton", firstPersonSkeleton },
            { "firstPersonBoneTree.nodeChildren", static_cast<RE::NiAVObject*>(firstPersonBoneTree) },
            { "gameRootNode", gameRootNode },
            { "gameFlattenedBoneTree.nodeChildren", static_cast<RE::NiAVObject*>(gameFlattenedBoneTree) },
            { "PlayerNodes.primaryWeapontoWeaponNode", playerNodes ? playerNodes->primaryWeapontoWeaponNode : nullptr },
            { "PlayerNodes.primaryWeaponOffsetNode", playerNodes ? playerNodes->primaryWeaponOffsetNOde : nullptr },
            { "PlayerNodes.WeaponLeftNode", playerNodes ? playerNodes->WeaponLeftNode : nullptr },
        } };

        for (const auto& nodeRoot : nodeRoots) {
            logWeaponAnimNodeMapRoot(nodeRoot);
        }

        const std::array<WeaponAnimFlattenedRoot, 2> flatRoots{ {
            { "firstPersonBoneTree.transforms", firstPersonBoneTree },
            { "gameFlattenedBoneTree.transforms", gameFlattenedBoneTree },
        } };

        for (const auto& flatRoot : flatRoots) {
            logWeaponAnimFlattenedMapRoot(flatRoot);
        }

        ROCK_LOG_INFO(Weapon, "WeaponAnimDump end key={:016X}", observedKey);
    }
}
