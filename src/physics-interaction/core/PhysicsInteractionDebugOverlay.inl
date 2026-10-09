/*
 * Debug overlay publishing is split from the runtime frame loop because it is diagnostic fan-out over many subsystems, not interaction authority. The fragment remains in this translation unit so existing helper visibility and behavior stay unchanged.
 */
    void PhysicsInteraction::publishDebugBodyOverlay(const PhysicsFrameContext& context)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::DebugOverlayPublish);

        provider_collider_visualization::Snapshot colliderFocus{};
        if (provider_collider_visualization::copySnapshot(colliderFocus)) {
            debug::Install();
            debug::BodyOverlayFrame focusedFrame{};
            focusedFrame.world = context.hknpWorld;
            focusedFrame.gameFrameIndex =
                _frame.palmClockGameFrameIndex.load(std::memory_order_acquire);
            focusedFrame.drawRockBodies = true;
            focusedFrame.entries[0] = debug::BodyOverlayEntry{
                RE::hknpBodyId{ colliderFocus.bodyId },
                debug::BodyOverlayRole::FocusedWeaponPart
            };
            focusedFrame.count = 1;
            debug::PublishFrame(focusedFrame);
            return;
        }


        auto* hknp = context.hknpWorld;
        provider_debug_overlay::Snapshot* providerOverlay = nullptr;
        if (provider_debug_overlay::hasContent()) {
            // Lazily keep the aggregate buffer off the per-frame game stack.
            // This aggregate scratch remains on ROCK's update-owner thread;
            // copySnapshot synchronizes against publication and teardown.
            static provider_debug_overlay::Snapshot s_providerOverlay{};
            provider_debug_overlay::copySnapshot(s_providerOverlay);
            providerOverlay = &s_providerOverlay;
        }
        const bool drawProviderOverlay = providerOverlay &&
            (providerOverlay->lineCount > 0 || providerOverlay->textCount > 0);
        const auto skeletonBoneMode = skeleton_bone_debug_math::sanitizeDebugSkeletonBoneMode(g_rockConfig.rockDebugSkeletonBoneMode);
        const auto skeletonBoneSource = skeleton_bone_debug_math::sanitizeDebugSkeletonBoneSource(g_rockConfig.rockDebugSkeletonBoneSource);
        const auto visualization = debug_visualization_policy::resolve({
            .colliderMaster = g_rockConfig.rockDebugShowColliders,
            .targetColliders = g_rockConfig.rockDebugShowTargetColliders,
            .handColliders = g_rockConfig.rockDebugDrawHandColliders,
            .handBoneColliders = g_rockConfig.rockDebugDrawHandBoneColliders,
            .bodyBoneColliders = g_rockConfig.rockDebugDrawBodyBoneColliders,
            .dynamicHandColliders =
                g_rockConfig.rockDebugDrawDynamicHandColliders,
            .weaponColliders = g_rockConfig.rockDebugDrawWeaponColliders,
            .grabbedWeaponPartCollider =
                g_rockConfig.rockDebugDrawGrabbedWeaponPartCollider,
            .dynamicWeaponColliders =
                g_rockConfig.rockDebugDrawDynamicWeaponColliders,
            .handAxes = g_rockConfig.rockDebugShowHandAxes,
            .grabPivots = g_rockConfig.rockDebugShowGrabPivots,
            .fingerProbes = g_rockConfig.rockDebugShowGrabFingerProbes,
            .fingerSweptArc = g_rockConfig.rockDebugShowGrabFingerSweptArc,
            .fingerSweptArcText =
                g_rockConfig.rockDebugShowGrabFingerSweptArcText,
            .fingerSweptArcLiveSkeleton =
                g_rockConfig.rockDebugShowGrabFingerSweptArcLiveSkeleton,
            .palmVectors = g_rockConfig.rockDebugShowPalmVectors,
            .grabPockets = g_rockConfig.rockDebugDrawGrabPockets,
            .rootFlattenedFingerSkeleton =
                g_rockConfig.rockDebugShowRootFlattenedFingerSkeletonMarkers,
            .skeletonBones =
                g_rockConfig.rockDebugShowSkeletonBoneVisualizer &&
                skeletonBoneMode !=
                    skeleton_bone_debug_math::DebugSkeletonBoneMode::Off,
            .skeletonBoneAxes = g_rockConfig.rockDebugDrawSkeletonBoneAxes,
            .skeletonBoneLogging = g_rockConfig.rockDebugLogSkeletonBones,
            .skeletonBoneTruncationLogging =
                g_rockConfig.rockDebugLogSkeletonBoneTruncation,
            .grabPocketNormal = g_rockConfig.rockDebugShowGrabPocketNormal,
            .grabForceTorque = g_rockConfig.rockDebugDrawGrabForceTorque,
            .grabForceTorqueText =
                g_rockConfig.rockDebugDrawGrabForceTorqueText,
            .grabPivotSourceCollider =
                g_rockConfig.rockDebugDrawGrabPivotSourceCollider,
            .grabPivotSourceEvidence =
                g_rockConfig.rockDebugDrawGrabPivotSourceEvidence,
            .grabSupportFrame = g_rockConfig.rockDebugDrawGrabSupportFrame,
            .handBoneContacts = g_rockConfig.rockDebugDrawHandBoneContacts,
            .grabAuthorityProxy = g_rockConfig.rockDebugDrawGrabAuthorityProxy,
            .weaponAuthority = g_rockConfig.rockDebugDrawWeaponAuthority,
            .looseWeaponGripZones =
                g_rockConfig.rockDebugDrawLooseWeaponGripZones,
            .authoredGripActivationZones =
                g_rockConfig.rockDebugDrawAuthoredGripActivationZones,
            .nativeScopeActivation =
                g_rockConfig.rockEnableImmersiveScopes && g_rockConfig.rockDebugDrawNativeScopeActivation,
        });

        const bool drawTargetColliders = visualization.targetColliders;
        const bool drawHandColliders = visualization.handColliders;
        const bool drawHandBoneColliders = visualization.handBoneColliders;
        const bool drawBodyBoneColliders = visualization.bodyBoneColliders;
        const bool drawDynamicHandColliders =
            visualization.dynamicHandColliders;
        const bool drawWeaponColliders = visualization.weaponColliders;
        const bool drawGrabbedWeaponPartCollider =
            visualization.grabbedWeaponPartCollider;
        const bool drawDynamicWeaponColliders =
            visualization.dynamicWeaponColliders;
        const bool drawGrabAuthorityProxyCollider =
            visualization.grabAuthorityProxyCollider;
        const bool drawHandAxes = visualization.handAxes;
        const bool drawGrabPivots = visualization.grabPivots;
        const bool drawFingerProbes = visualization.fingerProbes;
        const bool drawFingerSweptArc = visualization.fingerSweptArc;
        const bool drawFingerSweptArcText =
            visualization.fingerSweptArcText;
        const bool drawFingerSweptArcLiveSkeleton =
            visualization.fingerSweptArcLiveSkeleton;
        const bool drawPalmVectors = visualization.palmVectors;
        const bool drawGrabPockets = visualization.grabPockets;
        const bool drawRootFlattenedFingerSkeleton =
            visualization.rootFlattenedFingerSkeleton;
        const bool drawSkeletonBones = visualization.skeletonBones;
        const bool drawGrabPocketNormal = visualization.grabPocketNormal;
        const bool drawGrabForceTorque = visualization.grabForceTorque;
        const bool drawGrabForceTorqueText =
            visualization.grabForceTorqueText;
        const bool drawGrabPivotSourceCollider =
            visualization.grabPivotSourceCollider;
        const bool drawGrabPivotSourceEvidence =
            visualization.grabPivotSourceEvidence;
        const bool drawGrabSupportFrame = visualization.grabSupportFrame;
        const bool drawHandBoneContacts = visualization.handBoneContacts;
        const bool drawGrabAuthorityProxy = visualization.grabAuthorityProxy;
        const bool drawPerformanceProfilerOverlay = performance_profiler::overlayTextEnabled();
        const bool drawWeaponAuthorityDebug =
            _twoHandedGrip.isGripping() && visualization.weaponAuthority;
        const bool drawLooseWeaponGripZones =
            visualization.looseWeaponGripZones;
        const bool drawAuthoredGripActivationZones =
            visualization.authoredGripActivationZones;
        const bool drawAuthoredSupportGripDebug =
            drawAuthoredGripActivationZones;
        const bool drawNativeScopeActivation =
            visualization.nativeScopeActivation;

        std::array<std::uint32_t, 2> grabbedWeaponPartColliderBodyIds{
            INVALID_BODY_ID,
            INVALID_BODY_ID,
        };
        std::uint32_t grabbedWeaponPartColliderCount = 0;
        const auto isGrabbedWeaponPartColliderBody =
            [&](const std::uint32_t bodyId) {
                for (std::uint32_t index = 0;
                     index < grabbedWeaponPartColliderCount;
                     ++index) {
                    if (grabbedWeaponPartColliderBodyIds[index] == bodyId) {
                        return true;
                    }
                }
                return false;
            };
        if (drawGrabbedWeaponPartCollider) {
            for (const bool isLeft : { false, true }) {
                HandGripReport report{};
                _twoHandedGrip.getHandGripReport(isLeft, report);
                if (!report.active ||
                    report.bodyId == INVALID_BODY_ID ||
                    !isProviderWeaponBodyCurrentV1(
                        report.weaponGenerationKey,
                        report.bodyId) ||
                    isGrabbedWeaponPartColliderBody(report.bodyId) ||
                    grabbedWeaponPartColliderCount >=
                        grabbedWeaponPartColliderBodyIds.size()) {
                    continue;
                }

                grabbedWeaponPartColliderBodyIds[
                    grabbedWeaponPartColliderCount++] = report.bodyId;
            }
        }
        const bool hasGrabbedWeaponPartCollider =
            grabbedWeaponPartColliderCount > 0;
        const bool drawAnyRockColliderBodies =
            drawHandColliders || drawHandBoneColliders ||
            drawBodyBoneColliders || drawDynamicHandColliders ||
            drawWeaponColliders || hasGrabbedWeaponPartCollider ||
            drawDynamicWeaponColliders ||
            drawGrabAuthorityProxyCollider || drawGrabPivotSourceCollider;
        if (!drawAnyRockColliderBodies && !drawTargetColliders && !drawHandAxes && !drawGrabPivots && !drawFingerProbes &&
            !drawFingerSweptArc && !drawPalmVectors && !drawGrabPockets && !drawRootFlattenedFingerSkeleton && !drawSkeletonBones && !drawGrabPocketNormal &&
            !drawHandBoneContacts && !drawGrabAuthorityProxy && !drawGrabForceTorque && !drawPerformanceProfilerOverlay &&
            !drawWeaponAuthorityDebug && !drawLooseWeaponGripZones && !drawNativeScopeActivation && !drawGrabSupportFrame &&
            !drawDynamicHandColliders && !drawDynamicWeaponColliders && !drawAuthoredSupportGripDebug && !drawProviderOverlay) {
            debug::ClearFrame();
            return;
        }

        debug::Install();

        debug::BodyOverlayFrame frame{};
        frame.world = hknp;
        frame.gameFrameIndex =
            _frame.palmClockGameFrameIndex.load(std::memory_order_acquire);
        frame.drawRockBodies = drawAnyRockColliderBodies;
        frame.drawTargetBodies = drawTargetColliders;
        frame.drawAxes = drawHandAxes || drawGrabAuthorityProxy || drawGrabForceTorque || drawNativeScopeActivation ||
            drawWeaponAuthorityDebug || drawAuthoredSupportGripDebug;
        frame.drawMarkers = drawGrabPivots || drawFingerProbes || drawFingerSweptArc || drawPalmVectors || drawGrabPockets || drawRootFlattenedFingerSkeleton ||
            drawGrabPocketNormal || drawGrabForceTorque || drawHandBoneContacts || drawGrabAuthorityProxy ||
            drawWeaponAuthorityDebug || drawLooseWeaponGripZones || drawNativeScopeActivation || drawGrabSupportFrame || drawDynamicHandColliders ||
            drawAuthoredSupportGripDebug;
        frame.drawSkeleton = drawSkeletonBones;
        frame.drawColoredLines = providerOverlay && providerOverlay->lineCount > 0;
        frame.drawText = drawGrabForceTorqueText || drawFingerSweptArcText || drawPerformanceProfilerOverlay ||
            drawDynamicHandColliders || drawDynamicWeaponColliders || drawNativeScopeActivation ||
            drawAuthoredSupportGripDebug || drawGrabAuthorityProxy ||
            (providerOverlay && providerOverlay->textCount > 0);
        if (providerOverlay) {
            frame.coloredLineEntries = providerOverlay->lines.data();
            frame.coloredLineCount = providerOverlay->lineCount;
        }
        for (std::uint32_t index = 0;
             providerOverlay && index < providerOverlay->textCount && frame.textCount < frame.textEntries.size();
             ++index) {
            const auto& source = providerOverlay->textEntries[index];
            auto& destination = frame.textEntries[frame.textCount++];
            std::snprintf(destination.text, sizeof(destination.text), "%s", source.text);
            destination.x = source.x;
            destination.y = source.y;
            destination.size = source.textSize;
            std::copy_n(source.color, 4, destination.color);
            destination.worldAnchor = RE::NiPoint3(
                source.worldAnchorGame[0],
                source.worldAnchorGame[1],
                source.worldAnchorGame[2]);
            destination.worldAnchored =
                (source.flags & static_cast<std::uint32_t>(
                    provider::RockProviderDebugOverlayTextFlagV1::WorldAnchored)) != 0;
        }
        const bool rightDisabled = context.right.disabled;
        const bool leftDisabled = context.left.disabled;

        auto addBody = [&](RE::hknpBodyId bodyId, debug::BodyOverlayRole role) {
            if (bodyId.value == INVALID_BODY_ID) {
                return;
            }

            for (std::uint32_t i = 0; i < frame.count; i++) {
                if (frame.entries[i].bodyId.value == bodyId.value && frame.entries[i].role == role) {
                    return;
                }
            }

            if (frame.count >= frame.entries.size()) {
                return;
            }

            auto& entry = frame.entries[frame.count++];
            entry.bodyId = bodyId;
            entry.role = role;
        };

        auto addAxisTransformWithBasis = [&](const RE::NiTransform& transform,
                                            debug::AxisOverlayRole role,
                                            const RE::NiPoint3& translationStart,
                                            bool drawTranslationLine,
                                            debug::AxisOverlayBasis basis) {
            if (!frame.drawAxes || frame.axisCount >= frame.axisEntries.size()) {
                return;
            }

            auto& entry = frame.axisEntries[frame.axisCount++];
            entry.source = debug::AxisOverlaySource::Transform;
            entry.role = role;
            entry.transform = transform;
            entry.translationStart = translationStart;
            entry.basis = basis;
            entry.drawTranslationLine = drawTranslationLine;
        };

        auto addAxisTransform = [&](const RE::NiTransform& transform, debug::AxisOverlayRole role, const RE::NiPoint3& translationStart, bool drawTranslationLine) {
            addAxisTransformWithBasis(transform, role, translationStart, drawTranslationLine, debug::AxisOverlayBasis::NiLocalVectorToWorld);
        };

        auto addStoredColumnAxisTransform =
            [&](const RE::NiTransform& transform, debug::AxisOverlayRole role, const RE::NiPoint3& translationStart, bool drawTranslationLine) {
                addAxisTransformWithBasis(transform, role, translationStart, drawTranslationLine, debug::AxisOverlayBasis::StoredColumns);
            };

        auto addAxisBody = [&](RE::hknpBodyId bodyId, debug::AxisOverlayRole role, const RE::NiPoint3& translationStart, bool drawTranslationLine) {
            if (!frame.drawAxes || bodyId.value == INVALID_BODY_ID || frame.axisCount >= frame.axisEntries.size()) {
                return;
            }

            auto& entry = frame.axisEntries[frame.axisCount++];
            entry.source = debug::AxisOverlaySource::Body;
            entry.role = role;
            entry.bodyId = bodyId;
            entry.translationStart = translationStart;
            entry.drawTranslationLine = drawTranslationLine;
        };

        auto addMarker = [&](debug::MarkerOverlayRole role, const RE::NiPoint3& position, const RE::NiPoint3& lineEnd, float size, bool drawPoint, bool drawLine) {
            if (!frame.drawMarkers || frame.markerCount >= frame.markerEntries.size()) {
                return;
            }

            auto& entry = frame.markerEntries[frame.markerCount++];
            entry.role = role;
            entry.position = position;
            entry.lineEnd = lineEnd;
            entry.size = size;
            entry.drawPoint = drawPoint;
            entry.drawLine = drawLine;
        };

        auto addMarkerPoint = [&](debug::MarkerOverlayRole role, const RE::NiPoint3& position, float size) {
            addMarker(role, position, position, size, true, false);
        };

        auto addMarkerRay = [&](debug::MarkerOverlayRole role, const RE::NiPoint3& start, const RE::NiPoint3& end, float startSize) {
            addMarker(role, start, end, startSize, true, true);
        };

        auto addMarkerLine = [&](debug::MarkerOverlayRole role, const RE::NiPoint3& start, const RE::NiPoint3& end) {
            addMarker(role, start, end, 0.0f, false, true);
        };

        auto addTextLineSized = [&](const RE::NiPoint3& worldAnchor, float size, const float color[4], const char* format, auto&&... args) {
            if (!frame.drawText || frame.textCount >= frame.textEntries.size()) {
                return;
            }

            auto& entry = frame.textEntries[frame.textCount++];
            entry.x = 20.0f;
            entry.y = 0.0f;
            entry.size = size;
            entry.color[0] = color[0];
            entry.color[1] = color[1];
            entry.color[2] = color[2];
            entry.color[3] = color[3];
            entry.worldAnchor = worldAnchor;
            entry.worldAnchored = true;
            std::snprintf(entry.text, sizeof(entry.text), format, std::forward<decltype(args)>(args)...);
        };

        auto addTextLine = [&](const RE::NiPoint3& worldAnchor, const float color[4], const char* format, auto&&... args) {
            addTextLineSized(worldAnchor, 3.0f, color, format, std::forward<decltype(args)>(args)...);
        };

        auto addScreenTextLine = [&](float x, float y, const float color[4], const char* text) {
            if (!frame.drawText || !text || frame.textCount >= frame.textEntries.size()) {
                return;
            }

            auto& entry = frame.textEntries[frame.textCount++];
            entry.x = x;
            entry.y = y;
            entry.size = 2.0f;
            entry.color[0] = color[0];
            entry.color[1] = color[1];
            entry.color[2] = color[2];
            entry.color[3] = color[3];
            entry.worldAnchored = false;
            std::snprintf(entry.text, sizeof(entry.text), "%s", text);
        };

        if (frame.drawRockBodies || frame.drawTargetBodies) {
            const float phaseLegendColor[4]{ 1.0f, 1.0f, 1.0f, 0.96f };
            char phaseLegend[128]{};
            std::snprintf(
                phaseLegend,
                sizeof(phaseLegend),
                "COLLIDER PHASE frame=%llu  TARGET=YELLOW(103%%)  PRE=MAGENTA(101.5%%)  POST=CYAN(100%%)",
                static_cast<unsigned long long>(frame.gameFrameIndex));
            addScreenTextLine(18.0f, 60.0f, phaseLegendColor, phaseLegend);
        }

        auto tryResolveBodyPosition = [&](std::uint32_t bodyId, RE::NiPoint3& outPosition) {
            if (!hknp || bodyId == INVALID_CONTACT_BODY_ID || bodyId == INVALID_BODY_ID) {
                return false;
            }
            RE::NiTransform bodyWorld{};
            if (!tryResolveLiveBodyWorldTransform(hknp, RE::hknpBodyId{ bodyId }, bodyWorld)) {
                return false;
            }
            outPosition = bodyWorld.translate;
            return true;
        };

        auto addSkeletonBone = [&](const DirectSkeletonBoneSnapshot& snapshot, std::size_t boneIndex, bool drawAxis) {
            if (!frame.drawSkeleton || boneIndex >= snapshot.bones.size() || frame.skeletonCount >= frame.skeletonEntries.size()) {
                return;
            }

            const auto& bone = snapshot.bones[boneIndex];
            auto& entry = frame.skeletonEntries[frame.skeletonCount++];
            entry.role = skeletonOverlayRoleForBone(bone.name);
            entry.transform = bone.world;
            entry.pointSize = g_rockConfig.rockDebugSkeletonBonePointSize;
            entry.axisLength = g_rockConfig.rockDebugSkeletonBoneAxisLength;
            entry.drawPoint = true;
            entry.drawAxis = drawAxis;
            entry.inPowerArmor = snapshot.inPowerArmor;
            if (bone.drawableParentSnapshotIndex >= 0 && static_cast<std::size_t>(bone.drawableParentSnapshotIndex) < snapshot.bones.size()) {
                entry.parentPosition = snapshot.bones[bone.drawableParentSnapshotIndex].world.translate;
                entry.hasParent = true;
            }
        };

        auto pointDistance = [](const RE::NiPoint3& lhs, const RE::NiPoint3& rhs) {
            const RE::NiPoint3 delta = lhs - rhs;
            return std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
        };

        if (drawPerformanceProfilerOverlay) {
            performance_profiler::OverlayLines profilerLines{};
            const auto profilerLineCount = performance_profiler::copyOverlayLines(profilerLines);
            const float profilerColor[4]{ 0.70f, 1.0f, 0.82f, 0.92f };
            if (profilerLineCount == 0) {
                addScreenTextLine(18.0f, 18.0f, profilerColor, "ROCK PERF warming");
            } else {
                for (std::uint32_t i = 0; i < profilerLineCount && i < profilerLines.size(); ++i) {
                    addScreenTextLine(18.0f, 18.0f + (14.0f * static_cast<float>(i)), profilerColor, profilerLines[i].data());
                }
            }
        }

        if (drawNativeScopeActivation) {
            constexpr float kScopeAxisGuideLengthGameUnits = 80.0f;
            constexpr float kScopeUpGuideLengthGameUnits = 28.0f;
            constexpr std::uint32_t kFreshScopeWriteMaxAgeFrames = 1;

            auto finitePoint = [](const RE::NiPoint3& point) {
                return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
            };
            auto normalizeScopeVector = [&](const RE::NiPoint3& vector, const RE::NiPoint3& fallback) {
                const float lengthSquared = vector.x * vector.x + vector.y * vector.y + vector.z * vector.z;
                if (std::isfinite(lengthSquared) && lengthSquared > 0.000001f) {
                    const float inverseLength = 1.0f / std::sqrt(lengthSquared);
                    return vector * inverseLength;
                }
                return fallback;
            };
            auto scopeForwardWorld = [&](const RE::NiTransform& transform) {
                return normalizeScopeVector(
                    transform_math::localVectorToWorld(transform, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }),
                    RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
            };
            auto scopeUpWorld = [&](const RE::NiTransform& transform) {
                return normalizeScopeVector(
                    transform_math::localVectorToWorld(transform, RE::NiPoint3{ 0.0f, 0.0f, 1.0f }),
                    RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
            };

            const auto* playerNodes = f4vr::getPlayerNodes();
            auto* scopeCamera = playerNodes ? playerNodes->primaryWeaponScopeCamera : nullptr;
            auto* scopeCameraParent = scopeCamera ? scopeCamera->parent : nullptr;
            const bool hmdFrameValid = context.hasHmdFrame && finitePoint(context.hmdPositionWorld) && finitePoint(context.hmdForwardWorld);

            RE::NiTransform liveCameraWorld{};
            const bool liveCameraValid = scopeCamera && finiteNiTransform(scopeCamera->world);
            if (liveCameraValid) {
                liveCameraWorld = scopeCamera->world;
            }

            RE::NiTransform parentComposedCameraWorld{};
            bool parentComposedCameraValid = false;
            if (scopeCamera && scopeCameraParent && finiteNiTransform(scopeCameraParent->world) && finiteNiTransform(scopeCamera->local)) {
                parentComposedCameraWorld = transform_math::composeTransforms(scopeCameraParent->world, scopeCamera->local);
                parentComposedCameraValid = finiteNiTransform(parentComposedCameraWorld);
            } else if (liveCameraValid && scopeCamera && !scopeCameraParent) {
                parentComposedCameraWorld = liveCameraWorld;
                parentComposedCameraValid = true;
            }

            auto* weaponNode = f4vr::getWeaponNode();
            const bool weaponWorldValid = weaponNode && finiteNiTransform(weaponNode->world);
            const WeaponCollision::NativeScopeSightAnchorSnapshot sightSnapshot = _weaponCollision.getNativeScopeSightAnchorSnapshot();
            const std::uint64_t publishedWeaponGeneration = _weaponCollision.getCurrentWeaponGenerationKey();
            const bool sightGenerationMatches = sightSnapshot.weaponGenerationKey != 0 &&
                sightSnapshot.weaponGenerationKey == publishedWeaponGeneration;
            const bool sightGeometryValid = weaponWorldValid && sightSnapshot.valid && sightGenerationMatches &&
                finitePoint(sightSnapshot.anchorWeaponLocal) && finitePoint(sightSnapshot.sightBoundsMinWeaponLocal) &&
                finitePoint(sightSnapshot.sightBoundsMaxWeaponLocal);
            const NativeScopeResolvedAnchorSnapshot resolvedAnchorSnapshot =
                _twoHandedGrip.getNativeScopeResolvedAnchorSnapshot();
            const native_scope_sight_anchor_policy::PublicationIdentity
                resolvedAnchorIdentity{
                    .weaponGenerationKey =
                        resolvedAnchorSnapshot.weaponGenerationKey,
                    .equippedWeaponOwnershipKey =
                        resolvedAnchorSnapshot.equippedWeaponOwnershipKey,
                    .weaponFormID = resolvedAnchorSnapshot.weaponFormID,
                };
            const native_scope_sight_anchor_policy::PublicationIdentity
                sightIdentity{
                    .weaponGenerationKey = sightSnapshot.weaponGenerationKey,
                    .equippedWeaponOwnershipKey =
                        sightSnapshot.equippedWeaponOwnershipKey,
                    .weaponFormID = sightSnapshot.weaponFormID,
                };
            const bool resolvedAnchorValid =
                weaponWorldValid &&
                resolvedAnchorSnapshot.valid &&
                native_scope_sight_anchor_policy::
                    matchesCurrentEquippedWeapon(
                        resolvedAnchorIdentity,
                        sightIdentity) &&
                sightGenerationMatches &&
                finitePoint(resolvedAnchorSnapshot.anchorWeaponLocal);
            RE::NiPoint3 resolvedAnchorWorld{};
            if (resolvedAnchorValid) {
                resolvedAnchorWorld = transform_math::localPointToWorld(
                    weaponNode->world,
                    resolvedAnchorSnapshot.anchorWeaponLocal);
            }
            const NativeScopeCameraTargetPreviewSnapshot targetPreviewSnapshot =
                _twoHandedGrip.getNativeScopeCameraTargetPreviewSnapshot();
            const native_scope_sight_anchor_policy::PublicationIdentity
                targetPreviewIdentity{
                    .weaponGenerationKey =
                        targetPreviewSnapshot.weaponGenerationKey,
                    .equippedWeaponOwnershipKey =
                        targetPreviewSnapshot.equippedWeaponOwnershipKey,
                    .weaponFormID = targetPreviewSnapshot.weaponFormID,
                };
            const bool targetPreviewValid =
                resolvedAnchorValid &&
                targetPreviewSnapshot.valid &&
                targetPreviewSnapshot.anchorSource ==
                    resolvedAnchorSnapshot.source &&
                native_scope_sight_anchor_policy::
                    matchesCurrentEquippedWeapon(
                        targetPreviewIdentity,
                        resolvedAnchorIdentity) &&
                finiteNiTransform(
                    targetPreviewSnapshot.cameraWeaponLocal);
            RE::NiTransform targetPreviewWorld{};
            if (targetPreviewValid) {
                targetPreviewWorld =
                    native_scope_camera_follow_math::
                        resolveRigidAnchorFrameWorld(
                            weaponNode->world,
                            targetPreviewSnapshot.cameraWeaponLocal);
            }
            const bool targetPreviewWorldValid =
                targetPreviewValid &&
                finiteNiTransform(targetPreviewWorld);

            RE::NiPoint3 sightAnchorWorld{};
            if (sightGeometryValid) {
                sightAnchorWorld = transform_math::localPointToWorld(weaponNode->world, sightSnapshot.anchorWeaponLocal);
                addMarkerPoint(debug::MarkerOverlayRole::NativeScopeSightBounds, sightAnchorWorld, 3.8f);

                const RE::NiPoint3& boundsMin = sightSnapshot.sightBoundsMinWeaponLocal;
                const RE::NiPoint3& boundsMax = sightSnapshot.sightBoundsMaxWeaponLocal;
                const std::array<RE::NiPoint3, 8> localCorners{
                    RE::NiPoint3{ boundsMin.x, boundsMin.y, boundsMin.z },
                    RE::NiPoint3{ boundsMax.x, boundsMin.y, boundsMin.z },
                    RE::NiPoint3{ boundsMax.x, boundsMax.y, boundsMin.z },
                    RE::NiPoint3{ boundsMin.x, boundsMax.y, boundsMin.z },
                    RE::NiPoint3{ boundsMin.x, boundsMin.y, boundsMax.z },
                    RE::NiPoint3{ boundsMax.x, boundsMin.y, boundsMax.z },
                    RE::NiPoint3{ boundsMax.x, boundsMax.y, boundsMax.z },
                    RE::NiPoint3{ boundsMin.x, boundsMax.y, boundsMax.z },
                };
                std::array<RE::NiPoint3, 8> worldCorners{};
                for (std::size_t cornerIndex = 0; cornerIndex < localCorners.size(); ++cornerIndex) {
                    worldCorners[cornerIndex] = transform_math::localPointToWorld(weaponNode->world, localCorners[cornerIndex]);
                }
                constexpr std::uint8_t kBoundsEdges[12][2]{
                    { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 0 },
                    { 4, 5 }, { 5, 6 }, { 6, 7 }, { 7, 4 },
                    { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 },
                };
                for (const auto& edge : kBoundsEdges) {
                    addMarkerLine(debug::MarkerOverlayRole::NativeScopeSightBounds, worldCorners[edge[0]], worldCorners[edge[1]]);
                }
            }

            const NativeScopeCameraDebugSnapshot writeSnapshot = _twoHandedGrip.getNativeScopeCameraDebugSnapshot();
            const NativeScopeActivationDebugSnapshot activationSnapshot = _twoHandedGrip.getNativeScopeActivationDebugSnapshot();
            const bool freshWrite = writeSnapshot.applySequence != 0 &&
                writeSnapshot.framesSinceApply <= kFreshScopeWriteMaxAgeFrames;
            RE::NiTransform rockTargetWorld{};
            bool rockTargetValid = false;
            bool targetFromResolvedPreview = false;
            bool targetFromRecordedWrite = false;
            if (targetPreviewWorldValid) {
                rockTargetWorld = targetPreviewWorld;
                rockTargetValid = true;
                targetFromResolvedPreview = true;
            } else if (freshWrite && writeSnapshot.targetValid && finiteNiTransform(writeSnapshot.targetCameraWorld)) {
                rockTargetWorld = writeSnapshot.targetCameraWorld;
                rockTargetValid = true;
                targetFromRecordedWrite = true;
            } else if (
                resolvedAnchorValid &&
                resolvedAnchorSnapshot.source !=
                    native_scope_sight_anchor_policy::AnchorSource::
                        FiringGripFallback &&
                (liveCameraValid || parentComposedCameraValid)) {
                rockTargetWorld = liveCameraValid ? liveCameraWorld : parentComposedCameraWorld;
                rockTargetWorld.translate = resolvedAnchorWorld;
                rockTargetValid = finiteNiTransform(rockTargetWorld);
            }
            const bool targetIsFallbackPreview =
                targetFromResolvedPreview &&
                targetPreviewSnapshot.anchorSource ==
                    native_scope_sight_anchor_policy::AnchorSource::
                        FiringGripFallback;

            const float liveColor[4]{ 1.0f, 0.12f, 0.08f, 1.0f };
            const float targetColor[4]{ 0.18f, 1.0f, 0.28f, 1.0f };
            const float anchorColor[4]{ 0.42f, 1.0f, 0.60f, 0.95f };
            const float hmdColor[4]{ 0.12f, 0.92f, 1.0f, 0.96f };

            if (liveCameraValid) {
                addAxisTransform(liveCameraWorld, debug::AxisOverlayRole::NativeScopeLiveCamera, liveCameraWorld.translate, false);
                addMarkerRay(
                    debug::MarkerOverlayRole::NativeScopeLiveCamera,
                    liveCameraWorld.translate,
                    liveCameraWorld.translate + scopeForwardWorld(liveCameraWorld) * kScopeAxisGuideLengthGameUnits,
                    4.2f);
                addTextLine(liveCameraWorld.translate, liveColor, "LIVE CAMERA (stored world)");
            }
            if (parentComposedCameraValid && scopeCameraParent) {
                addMarkerPoint(debug::MarkerOverlayRole::NativeScopeParentComposedCamera, parentComposedCameraWorld.translate, 3.2f);
                if (liveCameraValid) {
                    addMarkerLine(debug::MarkerOverlayRole::NativeScopeParentComposedCamera, liveCameraWorld.translate, parentComposedCameraWorld.translate);
                }
                addMarkerRay(
                    debug::MarkerOverlayRole::NativeScopeCameraParent,
                    scopeCameraParent->world.translate,
                    parentComposedCameraWorld.translate,
                    2.8f);
            }
            if (rockTargetValid) {
                const RE::NiPoint3 targetAimWorld =
                    rockTargetWorld.translate +
                    scopeForwardWorld(rockTargetWorld) *
                        kScopeAxisGuideLengthGameUnits;
                const RE::NiPoint3 targetUpWorld =
                    rockTargetWorld.translate +
                    scopeUpWorld(rockTargetWorld) *
                        kScopeUpGuideLengthGameUnits;
                addAxisTransform(rockTargetWorld, debug::AxisOverlayRole::NativeScopeRockTarget, rockTargetWorld.translate, false);
                addMarkerRay(
                    debug::MarkerOverlayRole::NativeScopeRockTarget,
                    rockTargetWorld.translate,
                    targetAimWorld,
                    4.0f);
                addMarkerPoint(
                    debug::MarkerOverlayRole::NativeScopeRockTarget,
                    targetAimWorld,
                    2.8f);
                addMarkerLine(
                    debug::MarkerOverlayRole::NativeScopeRockTarget,
                    rockTargetWorld.translate,
                    targetUpWorld);
                addMarkerPoint(
                    debug::MarkerOverlayRole::NativeScopeRockTarget,
                    targetUpWorld,
                    2.4f);
                addTextLine(
                    rockTargetWorld.translate,
                    targetColor,
                    targetIsFallbackPreview ?
                        "FALLBACK PREVIEW ORIGIN" :
                        (targetFromResolvedPreview ?
                                "ROCK PRE-ACTIVATION TARGET" :
                                (targetFromRecordedWrite ?
                                        "ROCK WRITE TARGET" :
                                        "ROCK GEOMETRY TARGET")));
                addTextLine(
                    targetAimWorld,
                    targetColor,
                    targetIsFallbackPreview ?
                        "FALLBACK AIM (+X)" :
                        "TARGET AIM (+X)");
                addTextLine(
                    targetUpWorld,
                    targetColor,
                    targetIsFallbackPreview ?
                        "FALLBACK UP (+Z)" :
                        "TARGET UP (+Z)");
                if (liveCameraValid) {
                    addMarkerLine(debug::MarkerOverlayRole::NativeScopeMismatch, liveCameraWorld.translate, rockTargetWorld.translate);
                }
            }
            if (freshWrite && writeSnapshot.captureValid && finiteNiTransform(writeSnapshot.cameraWorldBefore)) {
                addMarkerPoint(debug::MarkerOverlayRole::NativeScopePreWriteCamera, writeSnapshot.cameraWorldBefore.translate, 2.8f);
            }
            if (freshWrite && writeSnapshot.immediateReadbackValid && finiteNiTransform(writeSnapshot.immediateCameraWorldAfter)) {
                addMarkerPoint(debug::MarkerOverlayRole::NativeScopeImmediateReadback, writeSnapshot.immediateCameraWorldAfter.translate, 3.2f);
            }
            if (sightGeometryValid) {
                addTextLine(sightAnchorWorld, anchorColor, "GENERATED SIGHT REAR-CENTER");
            }
            if (resolvedAnchorValid &&
                resolvedAnchorSnapshot.source ==
                    native_scope_sight_anchor_policy::AnchorSource::
                        FiringGripFallback &&
                !targetIsFallbackPreview) {
                addMarkerPoint(
                    debug::MarkerOverlayRole::NativeScopeSightBounds,
                    resolvedAnchorWorld,
                    3.8f);
                addTextLine(
                    resolvedAnchorWorld,
                    anchorColor,
                    "FIRING GRIP FALLBACK ANCHOR");
            }
            if (hmdFrameValid) {
                const RE::NiPoint3 hmdForward = normalizeScopeVector(context.hmdForwardWorld, RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
                addMarkerRay(
                    debug::MarkerOverlayRole::NativeScopeHmd,
                    context.hmdPositionWorld,
                    context.hmdPositionWorld + hmdForward * 45.0f,
                    3.5f);
                addTextLine(context.hmdPositionWorld, hmdColor, "HMD");
            }

            const auto gripStateName = [](TwoHandedState state) {
                switch (state) {
                case TwoHandedState::Inactive:
                    return "Inactive";
                case TwoHandedState::Touching:
                    return "Touching";
                case TwoHandedState::Gripping:
                    return "Gripping";
                case TwoHandedState::PartCarry:
                    return "PartCarry";
                case TwoHandedState::PrimaryOnly:
                    return "PrimaryOnly";
                case TwoHandedState::SurfaceSupported:
                    return "SurfaceSupported";
                case TwoHandedState::EquippedDrop:
                    return "EquippedDrop";
                }
                return "Unknown";
            };
            const auto scopeWriteSourceName = [](NativeScopeCameraWriteSource source) {
                switch (source) {
                case NativeScopeCameraWriteSource::None:
                    return "none";
                case NativeScopeCameraWriteSource::PostFrikPresentationSync:
                    return "post-frik-presentation-sync";
                case NativeScopeCameraWriteSource::WeaponVisualAuthority:
                    return "weapon-visual-authority";
                }
                return "unknown";
            };
            const auto scopeAnchorSourceName = [](
                native_scope_sight_anchor_policy::AnchorSource source) {
                switch (source) {
                case native_scope_sight_anchor_policy::AnchorSource::None:
                    return "none";
                case native_scope_sight_anchor_policy::AnchorSource::
                    GeneratedSight:
                    return "generated-sight";
                case native_scope_sight_anchor_policy::AnchorSource::
                    FiringGripFallback:
                    return "firing-grip-fallback";
                }
                return "unknown";
            };
            const float panelColor[4]{ 0.96f, 0.98f, 1.0f, 0.98f };
            constexpr float panelX = 520.0f;
            float panelY = 18.0f;
            char panelLine[384]{};
            addScreenTextLine(panelX, panelY, panelColor,
                "NATIVE SCOPE: RED=stored live GREEN=exact preview/write ORANGE=immediate YELLOW=pre-write BLUE=parent/local CYAN=HMD");
            panelY += 14.0f;
            addScreenTextLine(panelX, panelY, panelColor,
                "GREEN tripod + long +X aim + short +Z up stay visible with ScopeMenu closed; activation samples HMD/weapon +Y.");
            panelY += 14.0f;

            std::snprintf(panelLine, sizeof(panelLine),
                "grip=%s ownsWeapon=%s scopeMenu=%s camera=%s parent=%s HMD=%s targetSource=%s",
                gripStateName(_twoHandedGrip.getState()),
                _twoHandedGrip.ownsWeaponTransform() ? "yes" : "no",
                _twoHandedGrip.isScopeMenuOpenThisFrame() ? "open" : "closed",
                liveCameraValid ? "yes" : "no",
                scopeCameraParent ? "yes" : "no",
                hmdFrameValid ? "yes" : "no",
                targetFromResolvedPreview ?
                    "pre-activation-preview" :
                    (targetFromRecordedWrite ?
                            "recorded-write" :
                            (rockTargetValid ?
                                    "anchor-only" :
                                    "none")));
            addScreenTextLine(panelX, panelY, panelColor, panelLine);
            panelY += 14.0f;

            std::snprintf(panelLine, sizeof(panelLine),
                "sight valid=%s generationMatch=%s bodies=%u anchorLocal=(%.2f,%.2f,%.2f)",
                sightGeometryValid ? "yes" : "no",
                sightGenerationMatches ? "yes" : "no",
                sightSnapshot.sightBodyCount,
                sightSnapshot.anchorWeaponLocal.x,
                sightSnapshot.anchorWeaponLocal.y,
                sightSnapshot.anchorWeaponLocal.z);
            addScreenTextLine(panelX, panelY, panelColor, panelLine);
            panelY += 14.0f;
            std::snprintf(panelLine, sizeof(panelLine),
                "resolved valid=%s source=%s anchorLocal=(%.2f,%.2f,%.2f)",
                resolvedAnchorValid ? "yes" : "no",
                scopeAnchorSourceName(resolvedAnchorSnapshot.source),
                resolvedAnchorSnapshot.anchorWeaponLocal.x,
                resolvedAnchorSnapshot.anchorWeaponLocal.y,
                resolvedAnchorSnapshot.anchorWeaponLocal.z);
            addScreenTextLine(panelX, panelY, panelColor, panelLine);
            panelY += 14.0f;
            std::snprintf(panelLine, sizeof(panelLine),
                "fallback tune: force=%s offset=(%.2f,%.2f,%.2f) rotation[pitch,yaw,roll]=(%.2f,%.2f,%.2f)",
                g_rockConfig.rockNativeScopeForceFiringGripFallback ? "yes" : "no",
                g_rockConfig.rockNativeScopeFiringGripFallbackOffsetXGameUnits,
                g_rockConfig.rockNativeScopeFiringGripFallbackOffsetYGameUnits,
                g_rockConfig.rockNativeScopeFiringGripFallbackOffsetZGameUnits,
                g_rockConfig.rockNativeScopeFiringGripFallbackPitchDegrees,
                g_rockConfig.rockNativeScopeFiringGripFallbackYawDegrees,
                g_rockConfig.rockNativeScopeFiringGripFallbackRollDegrees);
            addScreenTextLine(panelX, panelY, panelColor, panelLine);
            panelY += 14.0f;
            std::snprintf(panelLine, sizeof(panelLine),
                "preview: %s source=%s generation=%016llX scopeMenuIndependent=yes",
                targetPreviewWorldValid ? "ready" : "pending-calibration",
                scopeAnchorSourceName(targetPreviewSnapshot.anchorSource),
                static_cast<unsigned long long>(
                    targetPreviewSnapshot.weaponGenerationKey));
            addScreenTextLine(panelX, panelY, panelColor, panelLine);
            panelY += 14.0f;
            std::snprintf(panelLine, sizeof(panelLine),
                "generation: published=%016llX sight=%016llX",
                static_cast<unsigned long long>(publishedWeaponGeneration),
                static_cast<unsigned long long>(sightSnapshot.weaponGenerationKey));
            addScreenTextLine(panelX, panelY, panelColor, panelLine);
            panelY += 14.0f;

            if (writeSnapshot.applySequence == 0) {
                std::snprintf(panelLine, sizeof(panelLine), "write: never observed (no resolved-anchor camera write has applied while this diagnostic was enabled)");
                addScreenTextLine(panelX, panelY, panelColor, panelLine);
                panelY += 14.0f;
            } else {
                std::snprintf(panelLine, sizeof(panelLine),
                    "write seq=%llu age=%u source=%s generation=%016llX",
                    static_cast<unsigned long long>(writeSnapshot.applySequence),
                    writeSnapshot.framesSinceApply,
                    scopeWriteSourceName(writeSnapshot.writeSource),
                    static_cast<unsigned long long>(writeSnapshot.weaponGenerationKey));
                addScreenTextLine(panelX, panelY, panelColor, panelLine);
                panelY += 14.0f;
                std::snprintf(panelLine, sizeof(panelLine),
                    "stages: capture=%s target=%s applied=%s readback=%s anchorSource=%s",
                    writeSnapshot.captureValid ? "yes" : "no",
                    writeSnapshot.targetValid ? "yes" : "no",
                    writeSnapshot.writeApplied ? "yes" : "no",
                    writeSnapshot.immediateReadbackValid ? "yes" : "no",
                    scopeAnchorSourceName(writeSnapshot.anchorSource));
                addScreenTextLine(panelX, panelY, panelColor, panelLine);
                panelY += 14.0f;
            }

            if (activationSnapshot.publicationSequence == 0) {
                addScreenTextLine(panelX, panelY, panelColor, "scope input: no button/renderer sample observed");
                panelY += 14.0f;
            } else {
                std::snprintf(panelLine, sizeof(panelLine), "scope input seq=%llu button=%s renderer=%s/%s anchor=%s generation=%016llX",
                    static_cast<unsigned long long>(activationSnapshot.publicationSequence),
                    activationSnapshot.manualInputRequested ? "held" : "released",
                    activationSnapshot.rendererStateValid ? "valid" : "invalid",
                    activationSnapshot.rendererActive ? "active" : "inactive",
                    scopeAnchorSourceName(activationSnapshot.anchorSource),
                    static_cast<unsigned long long>(activationSnapshot.weaponGenerationKey));
                addScreenTextLine(panelX, panelY, panelColor, panelLine);
                panelY += 14.0f;
            }

            const float hmdToLive = hmdFrameValid && liveCameraValid ? pointDistance(context.hmdPositionWorld, liveCameraWorld.translate) : -1.0f;
            const float hmdToTarget = hmdFrameValid && rockTargetValid ? pointDistance(context.hmdPositionWorld, rockTargetWorld.translate) : -1.0f;
            const float liveToTarget = liveCameraValid && rockTargetValid ? pointDistance(liveCameraWorld.translate, rockTargetWorld.translate) : -1.0f;
            const float storedToComposed = liveCameraValid && parentComposedCameraValid ?
                pointDistance(liveCameraWorld.translate, parentComposedCameraWorld.translate) :
                -1.0f;
            std::snprintf(panelLine, sizeof(panelLine),
                "distance gu: HMD->live=%.2f HMD->target=%.2f live->target=%.2f storedWorld->parentLocal=%.2f",
                hmdToLive,
                hmdToTarget,
                liveToTarget,
                storedToComposed);
            addScreenTextLine(panelX, panelY, panelColor, panelLine);
            panelY += 14.0f;

            const RE::NiPoint3 livePosition = liveCameraValid ? liveCameraWorld.translate : RE::NiPoint3{};
            const RE::NiPoint3 targetPosition = rockTargetValid ? rockTargetWorld.translate : RE::NiPoint3{};
            std::snprintf(panelLine, sizeof(panelLine),
                "world pos: live=(%.1f,%.1f,%.1f) target=(%.1f,%.1f,%.1f) sightAnchor=(%.1f,%.1f,%.1f)",
                livePosition.x,
                livePosition.y,
                livePosition.z,
                targetPosition.x,
                targetPosition.y,
                targetPosition.z,
                sightAnchorWorld.x,
                sightAnchorWorld.y,
                sightAnchorWorld.z);
            addScreenTextLine(panelX, panelY, panelColor, panelLine);
        }

        if (frame.drawAxes) {
            if (!rightDisabled) {
                const RE::NiTransform& rawHand = context.right.rawHandWorld;
                if (drawHandAxes) {
                    addAxisTransform(rawHand, debug::AxisOverlayRole::RightHandRaw, rawHand.translate, false);
                    addAxisBody(_rightHand.getCollisionBodyId(), debug::AxisOverlayRole::RightHandBody, rawHand.translate, true);
                }
            }

            if (!leftDisabled) {
                const RE::NiTransform& rawHand = context.left.rawHandWorld;
                if (drawHandAxes) {
                    addAxisTransform(rawHand, debug::AxisOverlayRole::LeftHandRaw, rawHand.translate, false);
                    addAxisBody(_leftHand.getCollisionBodyId(), debug::AxisOverlayRole::LeftHandBody, rawHand.translate, true);
                }
            }
        }

        if (drawPalmVectors) {
            auto addPalmVectorDebug = [&](bool isLeft) {
                if ((isLeft && leftDisabled) || (!isLeft && rightDisabled)) {
                    return;
                }

                const auto& handInput = isLeft ? context.left : context.right;
                const RE::NiPoint3 grabAnchor = handInput.grabAnchorWorld;
                const RE::NiPoint3 closeSelectionDirection = handInput.closeSelectionDirectionWorld;
                const RE::NiPoint3 farSelectionDirection = handInput.farSelectionDirectionWorld;
                const float palmNormalLength = (std::max)(5.0f, selection_query_policy::kNearDetectionRangeGameUnits);
                const float pointingLength = (std::min)(90.0f, (std::max)(20.0f, selection_query_policy::kFarDetectionRangeGameUnits));

                addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabAnchor : debug::MarkerOverlayRole::RightGrabAnchor, grabAnchor, 2.0f);
                addMarkerRay(isLeft ? debug::MarkerOverlayRole::LeftPalmNormal : debug::MarkerOverlayRole::RightPalmNormal, grabAnchor,
                    grabAnchor + closeSelectionDirection * palmNormalLength, 1.6f);
                addMarkerRay(isLeft ? debug::MarkerOverlayRole::LeftPointing : debug::MarkerOverlayRole::RightPointing, grabAnchor,
                    grabAnchor + farSelectionDirection * pointingLength, 1.2f);
            };

            addPalmVectorDebug(false);
            addPalmVectorDebug(true);
        }

        if (drawGrabPockets) {
            auto addGrabPocketDebug = [&](bool isLeft) {
                if ((isLeft && leftDisabled) || (!isLeft && rightDisabled)) {
                    return;
                }

                const auto& handInput = isLeft ? context.left : context.right;
                const auto& hand = isLeft ? _leftHand : _rightHand;
                GrabAuthorityProxyDebugSnapshot snapshot{};
                if (!hand.getGrabAuthorityProxyDebugSnapshot(hknp, handInput.rawHandWorld, snapshot)) {
                    return;
                }
                const RE::NiTransform pocketBasisWorld =
                    makeGeneratedProxyAuthorityRelationFrame(snapshot.proxyTargetWorld);
                const auto palmPocket = grab_three_phase::buildGrabPocketFrameWithPalmCenter(pocketBasisWorld,
                    isLeft,
                    handInput.grabAnchorWorld,
                    g_rockConfig.rockGrabPocketDepthGameUnits,
                    g_rockConfig.rockGrabPocketRadiusGameUnits);
                if (palmPocket.valid) {
                    const auto palmCenterRole =
                        isLeft ? debug::MarkerOverlayRole::LeftPalmPocketCenter : debug::MarkerOverlayRole::RightPalmPocketCenter;
                    const auto palmRadiusRole =
                        isLeft ? debug::MarkerOverlayRole::LeftPalmPocketRadius : debug::MarkerOverlayRole::RightPalmPocketRadius;
                    addMarkerPoint(palmCenterRole, palmPocket.palmCenterWorld, 2.4f);
                    addMarkerLine(palmRadiusRole, palmPocket.palmCenterWorld, palmPocket.pocketCenterWorld);
                    addMarkerLine(palmRadiusRole,
                        palmPocket.pocketCenterWorld - palmPocket.fingerForwardWorld * palmPocket.pocketRadiusGameUnits,
                        palmPocket.pocketCenterWorld + palmPocket.fingerForwardWorld * palmPocket.pocketRadiusGameUnits);
                    addMarkerLine(palmRadiusRole,
                        palmPocket.pocketCenterWorld - palmPocket.crossPalmWorld * palmPocket.pocketRadiusGameUnits,
                        palmPocket.pocketCenterWorld + palmPocket.crossPalmWorld * palmPocket.pocketRadiusGameUnits);
                    constexpr float livePocketLabelColor[4]{ 1.0f, 0.08f, 0.58f, 0.96f };
                    RE::NiPoint3 pocketLabel = palmPocket.pocketCenterWorld;
                    pocketLabel.z += 3.0f;
                    addTextLineSized(
                        pocketLabel,
                        1.35f,
                        livePocketLabelColor,
                        "POCKET %s LIVE-PALM/PRE",
                        isLeft ? "L" : "R");
                }

                if (!handInput.hasPinchPocketWorld) {
                    return;
                }

                const RE::NiPoint3 pinchDetection = handInput.pinchDirectionWorld;
                const float directionLength =
                    (std::max)(g_rockConfig.rockGrabPinchMaxPocketDistanceGameUnits, selection_query_policy::kNearCastDistanceGameUnits);

                addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftPinchPocketCenter : debug::MarkerOverlayRole::RightPinchPocketCenter, handInput.pinchPocketWorld, 2.2f);
                addMarkerLine(isLeft ? debug::MarkerOverlayRole::LeftPinchPocketAxis : debug::MarkerOverlayRole::RightPinchPocketAxis, handInput.thumbPadWorld, handInput.indexPadWorld);
                addMarkerRay(isLeft ? debug::MarkerOverlayRole::LeftPinchDetectionDirection : debug::MarkerOverlayRole::RightPinchDetectionDirection,
                    handInput.pinchPocketWorld,
                    handInput.pinchPocketWorld + pinchDetection * directionLength,
                    1.4f);
            };

            addGrabPocketDebug(false);
            addGrabPocketDebug(true);
        }

        if (drawSkeletonBones) {
            DirectSkeletonBoneSnapshot snapshot{};
            if (s_directSkeletonBoneReader.capture(skeletonBoneMode, skeletonBoneSource, SkeletonBoneCaptureSpace::Rendered, snapshot)) {
                const std::size_t drawCap =
                    static_cast<std::size_t>(skeleton_bone_debug_math::sanitizeMaxSkeletonBonesDrawn(g_rockConfig.rockDebugMaxSkeletonBonesDrawn));
                const std::size_t axisCap =
                    static_cast<std::size_t>(skeleton_bone_debug_math::sanitizeMaxSkeletonAxesDrawn(g_rockConfig.rockDebugMaxSkeletonBoneAxesDrawn));
                std::size_t axesDrawn = 0;
                std::size_t skippedBones = 0;
                for (std::size_t i = 0; i < snapshot.bones.size(); ++i) {
                    if (i >= drawCap || frame.skeletonCount >= frame.skeletonEntries.size()) {
                        ++skippedBones;
                        continue;
                    }

                    const bool drawAxis =
                        visualization.skeletonBoneAxes &&
                        skeleton_bone_debug_math::shouldDrawSkeletonAxis(g_rockConfig.rockDebugSkeletonAxisBoneFilter, snapshot.bones[i].name, axesDrawn, axisCap);
                    if (drawAxis) {
                        ++axesDrawn;
                    }
                    addSkeletonBone(snapshot, i, drawAxis);
                }

                if (skippedBones > 0 &&
                    visualization.skeletonBoneTruncationLogging) {
                    ROCK_LOG_WARN(Hand,
                        "Direct skeleton overlay truncated: source={} mode={} total={} drawn={} skipped={} drawCap={} overlayBudget={}",
                        skeleton_bone_debug_math::snapshotSourceName(snapshot.source),
                        skeleton_bone_debug_math::modeName(snapshot.mode),
                        snapshot.bones.size(),
                        frame.skeletonCount,
                        skippedBones,
                        drawCap,
                        frame.skeletonEntries.size());
                }

                if (visualization.skeletonBoneLogging) {
                    const int interval = (std::max)(1, g_rockConfig.rockDebugSkeletonBoneLogIntervalFrames);
                    if (++s_directSkeletonBoneLogCounter >= static_cast<std::uint32_t>(interval)) {
                        s_directSkeletonBoneLogCounter = 0;
                        float vrScale = 0.0f;
                        if (auto* vrScaleSetting = f4vr::getIniSetting("fVrScale:VR")) {
                            vrScale = vrScaleSetting->GetFloat();
                        }
                        float rightHandScale = 0.0f;
                        float leftHandScale = 0.0f;
                        if (frik_visual_authority::isAvailable()) {
                            RE::NiTransform rightHandWorld{};
                            RE::NiTransform leftHandWorld{};
                            if (frik_visual_authority::tryGetPresentedHandWorldTransform(false, rightHandWorld)) {
                                rightHandScale = rightHandWorld.scale;
                            }
                            if (frik_visual_authority::tryGetPresentedHandWorldTransform(true, leftHandWorld)) {
                                leftHandScale = leftHandWorld.scale;
                            }
                        }

                        ROCK_LOG_DEBUG(Hand,
                            "Direct skeleton snapshot: source={} mode={} powerArmor={} skeleton={} tree={} total={} drawn={} axes={} skipped={} required={} missing={} vrScale={:.3f} rightHandScale={:.3f} leftHandScale={:.3f}",
                            skeleton_bone_debug_math::snapshotSourceName(snapshot.source),
                            skeleton_bone_debug_math::modeName(snapshot.mode),
                            snapshot.inPowerArmor,
                            reinterpret_cast<std::uintptr_t>(snapshot.skeleton),
                            reinterpret_cast<std::uintptr_t>(snapshot.boneTree),
                            snapshot.totalBoneCount,
                            frame.skeletonCount,
                            axesDrawn,
                            skippedBones,
                            snapshot.requiredResolvedCount,
                            snapshot.missingRequiredBones.size(),
                            vrScale,
                            rightHandScale,
                            leftHandScale);

                        for (const auto& bone : snapshot.bones) {
                            if (!skeletonLogFilterMatches(g_rockConfig.rockDebugSkeletonBoneLogFilter, bone.name)) {
                                continue;
                            }

                            const auto axes = skeleton_bone_debug_math::computeAxisEndpoints(bone.world, 1.0f);
                            ROCK_LOG_DEBUG(Hand,
                                "Direct skeleton bone {} parentTree={} parentDraw={} pos=({:.3f},{:.3f},{:.3f}) scale={:.3f} xAxisEnd=({:.3f},{:.3f},{:.3f}) yAxisEnd=({:.3f},{:.3f},{:.3f}) zAxisEnd=({:.3f},{:.3f},{:.3f})",
                                bone.name,
                                bone.parentTreeIndex,
                                bone.drawableParentSnapshotIndex,
                                bone.world.translate.x,
                                bone.world.translate.y,
                                bone.world.translate.z,
                                bone.world.scale,
                                axes.xEnd.x,
                                axes.xEnd.y,
                                axes.xEnd.z,
                                axes.yEnd.x,
                                axes.yEnd.y,
                                axes.yEnd.z,
                                axes.zEnd.x,
                                axes.zEnd.y,
                                axes.zEnd.z);
                        }
                    }
                } else {
                    s_directSkeletonBoneLogCounter = 0;
                }
            }
        } else {
            s_directSkeletonBoneReader.resetCache();
            s_directSkeletonBoneLogCounter = 0;
        }

        if (drawRootFlattenedFingerSkeleton) {
            auto addRootFlattenedFingerSkeletonDebug = [&](bool isLeft) {
                if ((isLeft && leftDisabled) || (!isLeft && rightDisabled)) {
                    return;
                }

                root_flattened_finger_skeleton_runtime::Snapshot snapshot{};
                if (!root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(isLeft, snapshot, nullptr, SkeletonBoneCaptureSpace::Rendered)) {
                    return;
                }

                const auto role = isLeft ? debug::MarkerOverlayRole::LeftRootFlattenedFingerSkeleton : debug::MarkerOverlayRole::RightRootFlattenedFingerSkeleton;
                const float markerSize = g_rockConfig.rockDebugRootFlattenedFingerSkeletonMarkerSize;
                for (const auto& finger : snapshot.fingers) {
                    if (!finger.valid) {
                        continue;
                    }
                    addMarkerPoint(role, finger.points[0], markerSize);
                    addMarkerPoint(role, finger.points[1], markerSize * 0.85f);
                    addMarkerPoint(role, finger.points[2], markerSize);
                    addMarkerLine(role, finger.points[0], finger.points[1]);
                    addMarkerLine(role, finger.points[1], finger.points[2]);
                }
            };

            addRootFlattenedFingerSkeletonDebug(false);
            addRootFlattenedFingerSkeletonDebug(true);
        }

        if (drawGrabPivots) {
            auto addGrabPivotDebug = [&](const Hand& hand) {
                if ((hand.isLeft() && leftDisabled) || (!hand.isLeft() && rightDisabled)) {
                    return;
                }

                GrabPivotDebugSnapshot snapshot{};
                if (!hand.getGrabPivotDebugSnapshot(hknp, snapshot)) {
                    return;
                }

                const bool isLeft = hand.isLeft();
                addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabPivotA : debug::MarkerOverlayRole::RightGrabPivotA, snapshot.handPivotWorld, 3.0f);
                addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabPivotB : debug::MarkerOverlayRole::RightGrabPivotB, snapshot.objectPivotWorld, 3.0f);
                addMarkerLine(isLeft ? debug::MarkerOverlayRole::LeftGrabPivotError : debug::MarkerOverlayRole::RightGrabPivotError, snapshot.handPivotWorld,
                    snapshot.objectPivotWorld);
            };

            addGrabPivotDebug(_rightHand);
            addGrabPivotDebug(_leftHand);
        }

        if (drawGrabPocketNormal) {
            auto addGrabPocketNormalDebug = [&](const Hand& hand) {
                if ((hand.isLeft() && leftDisabled) || (!hand.isLeft() && rightDisabled)) {
                    return;
                }

                GrabPocketNormalDebugSnapshot snapshot{};
                if (!hand.getGrabPocketNormalDebugSnapshot(hknp, snapshot)) {
                    return;
                }

                const bool isLeft = hand.isLeft();
                addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabSurfacePoint : debug::MarkerOverlayRole::RightGrabSurfacePoint, snapshot.contactPointWorld, 2.4f);
                addMarkerRay(isLeft ? debug::MarkerOverlayRole::LeftGrabSurfaceNormal : debug::MarkerOverlayRole::RightGrabSurfaceNormal, snapshot.contactPointWorld,
                    snapshot.normalEndWorld, 1.4f);
            };

            addGrabPocketNormalDebug(_rightHand);
            addGrabPocketNormalDebug(_leftHand);
        }

        if (drawGrabSupportFrame) {
            auto addGrabSupportFrameDebug = [&](const Hand& hand) {
                if ((hand.isLeft() && leftDisabled) || (!hand.isLeft() && rightDisabled)) {
                    return;
                }

                GrabSupportFrameDebugSnapshot snapshot{};
                if (!hand.getGrabSupportFrameDebugSnapshot(hknp, snapshot)) {
                    return;
                }

                const bool isLeft = hand.isLeft();
                const auto pivotRole = isLeft ? debug::MarkerOverlayRole::LeftGrabSupportFramePivot : debug::MarkerOverlayRole::RightGrabSupportFramePivot;
                const auto normalRole = isLeft ? debug::MarkerOverlayRole::LeftGrabSupportFrameNormal : debug::MarkerOverlayRole::RightGrabSupportFrameNormal;
                const auto axisRole = isLeft ? debug::MarkerOverlayRole::LeftGrabSupportFrameAxis : debug::MarkerOverlayRole::RightGrabSupportFrameAxis;
                const auto binormalRole = isLeft ? debug::MarkerOverlayRole::LeftGrabSupportFrameBinormal : debug::MarkerOverlayRole::RightGrabSupportFrameBinormal;
                const auto triangleRole = isLeft ? debug::MarkerOverlayRole::LeftGrabPivotSourceTriangle : debug::MarkerOverlayRole::RightGrabPivotSourceTriangle;

                if (snapshot.hasPivotTriangle) {
                    addMarkerLine(triangleRole, snapshot.pivotTriangleWorld[0], snapshot.pivotTriangleWorld[1]);
                    addMarkerLine(triangleRole, snapshot.pivotTriangleWorld[1], snapshot.pivotTriangleWorld[2]);
                    addMarkerLine(triangleRole, snapshot.pivotTriangleWorld[2], snapshot.pivotTriangleWorld[0]);
                }

                addMarkerPoint(pivotRole, snapshot.pivotWorld, 3.4f);
                if (snapshot.hasNormal) {
                    addMarkerRay(normalRole, snapshot.pivotWorld, snapshot.normalEndWorld, 1.8f);
                }
                if (snapshot.hasSupportAxis) {
                    const RE::NiPoint3 axisVector = snapshot.supportAxisEndWorld - snapshot.pivotWorld;
                    addMarkerLine(axisRole, snapshot.pivotWorld - axisVector, snapshot.supportAxisEndWorld);
                    addMarkerRay(axisRole, snapshot.pivotWorld, snapshot.supportAxisEndWorld, 1.4f);
                }
                if (snapshot.hasBinormal) {
                    const RE::NiPoint3 binormalVector = snapshot.binormalEndWorld - snapshot.pivotWorld;
                    addMarkerLine(binormalRole, snapshot.pivotWorld - binormalVector, snapshot.binormalEndWorld);
                    addMarkerRay(binormalRole, snapshot.pivotWorld, snapshot.binormalEndWorld, 1.2f);
                }
            };

            addGrabSupportFrameDebug(_rightHand);
            addGrabSupportFrameDebug(_leftHand);
        }

        if (drawGrabForceTorque) {
            auto addGrabForceTorqueDebug = [&](const Hand& hand) {
                if ((hand.isLeft() && leftDisabled) || (!hand.isLeft() && rightDisabled)) {
                    return;
                }

                const bool isLeft = hand.isLeft();
                const RE::NiTransform& rawHandWorld = isLeft ? context.left.rawHandWorld : context.right.rawHandWorld;
                GrabForceTorqueDebugSnapshot snapshot{};
                if (!hand.getGrabForceTorqueDebugSnapshot(hknp, rawHandWorld, snapshot)) {
                    return;
                }

                const float labelLiveColor[4]{ 0.80f, 1.0f, 0.95f, 0.90f };
                const float labelTargetColor[4]{ 1.0f, 0.90f, 0.18f, 0.92f };
                const float labelRelationColor[4]{ 0.55f, 0.92f, 1.0f, 0.86f };
                const float labelSolverColor[4]{ 1.0f, 0.36f, 0.18f, 0.96f };
                const RE::NiPoint3 labelLift{ 0.0f, 0.0f, 3.2f };
                auto addTriadLabel = [&](const RE::NiTransform& transform, const float color[4], const char* label) {
                    if (!drawGrabForceTorqueText) {
                        return;
                    }
                    addTextLineSized(transform.translate + labelLift, 1.85f, color, "%s", label);
                };

                if (drawGrabPivotSourceCollider) {
                    addBody(snapshot.pivotSourceBodyId,
                        isLeft ? debug::BodyOverlayRole::LeftGrabPivotSourceCollider : debug::BodyOverlayRole::RightGrabPivotSourceCollider);
                }

                addAxisTransform(snapshot.liveBodyWorld,
                    isLeft ? debug::AxisOverlayRole::LeftGrabForceTorqueLiveBody : debug::AxisOverlayRole::RightGrabForceTorqueLiveBody,
                    snapshot.livePivotWorld,
                    true);
                addTriadLabel(snapshot.liveBodyWorld, labelLiveColor, "LIVE BODY");
                addAxisTransform(snapshot.desiredBodyWorld,
                    isLeft ? debug::AxisOverlayRole::LeftGrabForceTorqueDesiredBody : debug::AxisOverlayRole::RightGrabForceTorqueDesiredBody,
                    snapshot.targetPivotWorld,
                    true);
                addTriadLabel(snapshot.desiredBodyWorld, labelTargetColor, "DESIRED BODY");
                if (snapshot.hasMotorConstraintFrames) {
                    addAxisTransform(snapshot.motorConstraintAWorld,
                        isLeft ? debug::AxisOverlayRole::LeftGrabMotorConstraintA : debug::AxisOverlayRole::RightGrabMotorConstraintA,
                        snapshot.liveBodyWorld.translate,
                        true);
                    addTriadLabel(snapshot.motorConstraintAWorld, labelRelationColor, "A FRAME");
                    addAxisTransform(snapshot.motorConstraintBWorld,
                        isLeft ? debug::AxisOverlayRole::LeftGrabMotorConstraintB : debug::AxisOverlayRole::RightGrabMotorConstraintB,
                        snapshot.liveBodyWorld.translate,
                        true);
                    addTriadLabel(snapshot.motorConstraintBWorld, labelRelationColor, "B FRAME");
                    addAxisTransform(snapshot.motorAtomTargetBodyWorld,
                        isLeft ? debug::AxisOverlayRole::LeftGrabMotorAtomTargetBody : debug::AxisOverlayRole::RightGrabMotorAtomTargetBody,
                        snapshot.motorAnchorAWorld,
                        true);
                    addTriadLabel(snapshot.motorAtomTargetBodyWorld, labelTargetColor, "ATOM ROW");
                    if (snapshot.hasMotorSolverEffectiveBody) {
                        addAxisTransform(snapshot.motorSolverEffectiveBodyWorld,
                            isLeft ? debug::AxisOverlayRole::LeftGrabMotorSolverEffectiveBody : debug::AxisOverlayRole::RightGrabMotorSolverEffectiveBody,
                            snapshot.motorAnchorAWorld,
                            true);
                        addTriadLabel(snapshot.motorSolverEffectiveBodyWorld, labelSolverColor, "SOLVER EFF");
                    }
                    if (snapshot.hasMotorRelationFrames) {
                        addAxisTransform(snapshot.motorRelationInputBodyWorld,
                            isLeft ? debug::AxisOverlayRole::LeftGrabMotorRelationInputBody : debug::AxisOverlayRole::RightGrabMotorRelationInputBody,
                            snapshot.motorAnchorAWorld,
                            true);
                        addTriadLabel(snapshot.motorRelationInputBodyWorld, labelRelationColor, "REL INPUT");
                        addAxisTransform(snapshot.motorRelationInverseBodyWorld,
                            isLeft ? debug::AxisOverlayRole::LeftGrabMotorRelationInverseBody : debug::AxisOverlayRole::RightGrabMotorRelationInverseBody,
                            snapshot.motorAnchorAWorld,
                            true);
                        addTriadLabel(snapshot.motorRelationInverseBodyWorld, labelRelationColor, "REL INV");
                    }
                }

                addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabForceTorqueTargetPivot : debug::MarkerOverlayRole::RightGrabForceTorqueTargetPivot,
                    snapshot.targetPivotWorld,
                    3.2f);
                addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabForceTorqueLivePivot : debug::MarkerOverlayRole::RightGrabForceTorqueLivePivot,
                    snapshot.livePivotWorld,
                    3.2f);
                addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabActivePivotBDesiredBody : debug::MarkerOverlayRole::RightGrabActivePivotBDesiredBody,
                    snapshot.activePivotBDesiredBodyWorld,
                    4.4f);
                addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabActivePivotBLiveBody : debug::MarkerOverlayRole::RightGrabActivePivotBLiveBody,
                    snapshot.activePivotBLiveBodyWorld,
                    4.4f);
                if (snapshot.hasActivePivotBVisualNode) {
                    addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabActivePivotBVisualNode : debug::MarkerOverlayRole::RightGrabActivePivotBVisualNode,
                        snapshot.activePivotBVisualNodeWorld,
                        4.0f);
                    addMarkerLine(isLeft ? debug::MarkerOverlayRole::LeftGrabActivePivotBVisualLock : debug::MarkerOverlayRole::RightGrabActivePivotBVisualLock,
                        snapshot.activePivotBLiveBodyWorld,
                        snapshot.activePivotBVisualNodeWorld);
                }
                addMarkerLine(isLeft ? debug::MarkerOverlayRole::LeftGrabForceTorqueCorrection : debug::MarkerOverlayRole::RightGrabForceTorqueCorrection,
                    snapshot.livePivotWorld,
                    snapshot.correctionEndWorld);
                addMarkerLine(isLeft ? debug::MarkerOverlayRole::LeftGrabForceTorqueLever : debug::MarkerOverlayRole::RightGrabForceTorqueLever,
                    snapshot.liveBodyWorld.translate,
                    snapshot.leverArmEndWorld);
                if (snapshot.hasTorqueAxis) {
                    addMarkerRay(isLeft ? debug::MarkerOverlayRole::LeftGrabForceTorqueAxis : debug::MarkerOverlayRole::RightGrabForceTorqueAxis,
                        snapshot.liveBodyWorld.translate,
                        snapshot.torqueAxisEndWorld,
                        1.4f);
                }
                if (snapshot.hasMotorConstraintFrames) {
                    addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabMotorAnchorA : debug::MarkerOverlayRole::RightGrabMotorAnchorA,
                        snapshot.motorAnchorAWorld,
                        3.8f);
                    addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabMotorAnchorB : debug::MarkerOverlayRole::RightGrabMotorAnchorB,
                        snapshot.motorAnchorBWorld,
                        3.4f);
                    addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabMotorAtomTargetPivot : debug::MarkerOverlayRole::RightGrabMotorAtomTargetPivot,
                        snapshot.motorAtomTargetPivotWorld,
                        3.2f);
                    if (snapshot.hasMotorRelationFrames) {
                        addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabMotorAtomTargetPivot : debug::MarkerOverlayRole::RightGrabMotorAtomTargetPivot,
                            snapshot.motorRelationPivotWorld,
                            2.4f);
                    }
                    addMarkerLine(isLeft ? debug::MarkerOverlayRole::LeftGrabMotorAnchorB : debug::MarkerOverlayRole::RightGrabMotorAnchorB,
                        snapshot.motorAnchorBWorld,
                        snapshot.motorAnchorAWorld);
                    if (snapshot.hasMotorAngularCommand) {
                        addMarkerRay(isLeft ? debug::MarkerOverlayRole::LeftGrabMotorAngularCommand : debug::MarkerOverlayRole::RightGrabMotorAngularCommand,
                            snapshot.liveBodyWorld.translate,
                            snapshot.motorAngularAxisEndWorld,
                            1.5f);
                    }
                    if (snapshot.hasMotorTargetBodyDelta) {
                        addMarkerLine(isLeft ? debug::MarkerOverlayRole::LeftGrabMotorTargetBodyDelta : debug::MarkerOverlayRole::RightGrabMotorTargetBodyDelta,
                            snapshot.desiredBodyWorld.translate,
                            snapshot.motorTargetBodyDeltaEndWorld);
                    }
                }

                if (drawGrabPivotSourceEvidence) {
                    const auto triangleRole =
                        isLeft ? debug::MarkerOverlayRole::LeftGrabPivotSourceTriangle : debug::MarkerOverlayRole::RightGrabPivotSourceTriangle;
                    if (snapshot.hasPivotTriangle) {
                        addMarkerLine(triangleRole, snapshot.pivotTriangleWorld[0], snapshot.pivotTriangleWorld[1]);
                        addMarkerLine(triangleRole, snapshot.pivotTriangleWorld[1], snapshot.pivotTriangleWorld[2]);
                        addMarkerLine(triangleRole, snapshot.pivotTriangleWorld[2], snapshot.pivotTriangleWorld[0]);
                    }
                    if (snapshot.hasMeshGripPoint) {
                        addMarkerPoint(
                            isLeft ? debug::MarkerOverlayRole::LeftGrabPivotSourceMeshPoint : debug::MarkerOverlayRole::RightGrabPivotSourceMeshPoint,
                            snapshot.meshGripPointWorld,
                            2.4f);
                    }
                    if (snapshot.hasVisualMeshGripPoint) {
                        addMarkerPoint(
                            isLeft ? debug::MarkerOverlayRole::LeftGrabPivotSourceVisualMeshPoint : debug::MarkerOverlayRole::RightGrabPivotSourceVisualMeshPoint,
                            snapshot.visualMeshGripPointWorld,
                            2.2f);
                        if (snapshot.hasMeshGripPoint) {
                            addMarkerLine(
                                isLeft ? debug::MarkerOverlayRole::LeftGrabPivotSourceBodyVisualLock : debug::MarkerOverlayRole::RightGrabPivotSourceBodyVisualLock,
                                snapshot.meshGripPointWorld,
                                snapshot.visualMeshGripPointWorld);
                        }
                    }
                    if (snapshot.hasCaptureMeshGripPoint) {
                        addMarkerPoint(
                            isLeft ? debug::MarkerOverlayRole::LeftGrabPivotSourceCapturePoint : debug::MarkerOverlayRole::RightGrabPivotSourceCapturePoint,
                            snapshot.captureMeshGripPointBodyWorld,
                            2.8f);
                        if (snapshot.hasMeshGripPoint && snapshot.gripPointMutatedAfterCapture) {
                            addMarkerLine(
                                isLeft ? debug::MarkerOverlayRole::LeftGrabPivotSourceCaptureMutation : debug::MarkerOverlayRole::RightGrabPivotSourceCaptureMutation,
                                snapshot.captureMeshGripPointBodyWorld,
                                snapshot.meshGripPointWorld);
                        }
                    }
                }

                if (drawGrabForceTorqueText) {
                    const float headerColor[4]{ 0.90f, 1.0f, 0.95f, 0.94f };
                    const float detailColor[4]{ 0.95f, 0.92f, 0.22f, 0.92f };
                    const RE::NiPoint3 labelAnchor = snapshot.targetPivotWorld + RE::NiPoint3{ 0.0f, 0.0f, 5.0f };
                    addTextLine(labelAnchor,
                        headerColor,
                        "GRAB track %.1f rot %.1f lever %.1f",
                        snapshot.pivotTrackingErrorGameUnits,
                        snapshot.rotationErrorDegrees,
                        snapshot.leverLengthGameUnits);
                    addTextLine(labelAnchor + RE::NiPoint3{ 0.0f, 0.0f, -3.5f },
                        detailColor,
                        "phase %s src %s body %u pbLock %.1f mut %.1f",
                        snapshot.acquisitionPhase,
                        snapshot.pivotAuthoritySource,
                        snapshot.pivotSourceBodyId.value,
                        snapshot.activePivotBVisualLockErrorGameUnits,
                        snapshot.captureGripLocalDeltaGameUnits);
                    addTextLine(labelAnchor + RE::NiPoint3{ 0.0f, 0.0f, -7.0f },
                        detailColor,
                        "cap shift %.1f rot %.1f gap %.1f>%.1f dot %.2f lev %.1f",
                        snapshot.captureFreezeBodyShiftGameUnits,
                        snapshot.captureFreezeBodyRotationDegrees,
                        snapshot.captureFreezePivotGapBeforeGameUnits,
                        snapshot.captureFreezePivotGapAfterGameUnits,
                        snapshot.captureFreezeShiftDot,
                        snapshot.captureFreezePivotLeverGameUnits);
                    if (snapshot.hasMotorConstraintFrames) {
                        addTextLine(labelAnchor + RE::NiPoint3{ 0.0f, 0.0f, -10.5f },
                            detailColor,
                            "atom %.2fgu %.1fdeg Aerr %.2fgu",
                            snapshot.motorTargetBodyDeltaGameUnits,
                            snapshot.motorTargetBodyDeltaDegrees,
                            snapshot.motorTransformBPivotToAnchorAGameUnits);
                        if (snapshot.hasMotorRelationFrames) {
                            addTextLine(labelAnchor + RE::NiPoint3{ 0.0f, 0.0f, -14.0f },
                                detailColor,
                                "relInv %.2fgu %.1fdeg atomRel %.1fdeg pivRel %.2fgu",
                                snapshot.motorRelationInverseBodyDeltaGameUnits,
                                snapshot.motorRelationInverseBodyDeltaDegrees,
                                snapshot.motorAtomToRelationInverseDeltaDegrees,
                                snapshot.motorTransformBRelationLocalDeltaGameUnits);
                        }
                        if (snapshot.hasMotorSolverEffectiveBody) {
                            addTextLine(labelAnchor + RE::NiPoint3{ 0.0f, 0.0f, -17.5f },
                                detailColor,
                                "solverEff %.2fgu %.1fdeg effAtom %.1fdeg",
                                snapshot.motorSolverEffectiveBodyDeltaGameUnits,
                                snapshot.motorSolverEffectiveBodyDeltaDegrees,
                                snapshot.motorSolverEffectiveToAtomDeltaDegrees);
                            addTextLine(labelAnchor + RE::NiPoint3{ 0.0f, 0.0f, -21.0f },
                                detailColor,
                                "solv targetA %.1f liveA %.1f renderA %.2fgu %.1fdeg",
                                snapshot.motorSolverEffectiveBodyDeltaDegrees,
                                snapshot.motorSolverEffectiveLiveABodyDeltaDegrees,
                                snapshot.motorTargetProxyToLiveProxyDeltaGameUnits,
                                snapshot.motorTargetProxyToLiveProxyDeltaDegrees);
                        }
                        const auto liveToA = grab_transform_telemetry::formatBasisBestMap(
                            "live>A",
                            grab_transform_telemetry::makeOrientationBasis(snapshot.liveBodyWorld),
                            grab_transform_telemetry::makeOrientationBasis(snapshot.motorConstraintAWorld));
                        const auto desiredToA = grab_transform_telemetry::formatBasisBestMap(
                            "des>A",
                            grab_transform_telemetry::makeOrientationBasis(snapshot.desiredBodyWorld),
                            grab_transform_telemetry::makeOrientationBasis(snapshot.motorConstraintAWorld));
                        addTextLine(labelAnchor + RE::NiPoint3{ 0.0f, 0.0f, -28.0f },
                            detailColor,
                            "%s",
                            liveToA.c_str());
                        addTextLine(labelAnchor + RE::NiPoint3{ 0.0f, 0.0f, -31.5f },
                            detailColor,
                            "%s",
                            desiredToA.c_str());
                        if (snapshot.hasMotorSolverEffectiveBody) {
                            const auto solverToA = grab_transform_telemetry::formatBasisBestMap(
                                "solv>A",
                                grab_transform_telemetry::makeOrientationBasis(snapshot.motorSolverEffectiveBodyWorld),
                                grab_transform_telemetry::makeOrientationBasis(snapshot.motorConstraintAWorld));
                            addTextLine(labelAnchor + RE::NiPoint3{ 0.0f, 0.0f, -35.0f },
                                detailColor,
                                "%s",
                                solverToA.c_str());
                        }
                    }
                }
            };

            addGrabForceTorqueDebug(_rightHand);
            addGrabForceTorqueDebug(_leftHand);
        }

        if (drawFingerSweptArc) {
            bool anySweepCapture = false;
            constexpr float kSweepHitTextColor[4] = { 1.0f, 0.90f, 0.05f, 1.0f };
            constexpr float kSweepMissTextColor[4] = { 1.0f, 0.05f, 0.05f, 1.0f };
            constexpr float kSweepOutOfReachTextColor[4] = { 0.08f, 0.35f, 1.0f, 1.0f };
            constexpr float kSweepOverOpenTextColor[4] = { 1.0f, 0.45f, 0.02f, 1.0f };
            constexpr float kSweepClosedLimitTextColor[4] = { 1.0f, 0.05f, 0.42f, 1.0f };
            constexpr float kSweepLegendTextColor[4] = { 0.92f, 0.92f, 0.92f, 0.96f };
            constexpr float kSweepAuthoredOpenTextColor[4] = { 1.0f, 0.72f, 0.08f, 1.0f };

            auto addFingerSweptArcDebug = [&](const Hand& hand) {
                if ((hand.isLeft() && leftDisabled) || (!hand.isLeft() && rightDisabled)) {
                    return;
                }

                grab_finger_pose_runtime::FingerSweepDebugSnapshot snapshot{};
                if (!hand.getGrabFingerSweepDebugSnapshot(snapshot) || !snapshot.valid || !snapshot.capture.valid || !std::isfinite(snapshot.objectWorld.scale) ||
                    std::abs(snapshot.objectWorld.scale) <= 0.000001f) {
                    return;
                }
                anySweepCapture = true;

                auto roleForProbe = [](grab_finger_pose_math::CalibratedFingerProbe probe) {
                    switch (probe) {
                    case grab_finger_pose_math::CalibratedFingerProbe::Outer:
                        return debug::MarkerOverlayRole::GrabFingerSweepOuter;
                    case grab_finger_pose_math::CalibratedFingerProbe::Inner:
                        return debug::MarkerOverlayRole::GrabFingerSweepInner;
                    case grab_finger_pose_math::CalibratedFingerProbe::Tip:
                    default:
                        return debug::MarkerOverlayRole::GrabFingerSweepTip;
                    }
                };
                auto roleForState = [](grab_finger_pose_runtime::FingerSweepDebugState state) {
                    switch (state) {
                    case grab_finger_pose_runtime::FingerSweepDebugState::MissFallback:
                        return debug::MarkerOverlayRole::GrabFingerSweepMiss;
                    case grab_finger_pose_runtime::FingerSweepDebugState::OutOfReach:
                        return debug::MarkerOverlayRole::GrabFingerSweepOutOfReach;
                    case grab_finger_pose_runtime::FingerSweepDebugState::OverOpen:
                        return debug::MarkerOverlayRole::GrabFingerSweepOverOpen;
                    case grab_finger_pose_runtime::FingerSweepDebugState::ClosedLimit:
                        return debug::MarkerOverlayRole::GrabFingerSweepClosedLimit;
                    case grab_finger_pose_runtime::FingerSweepDebugState::Hit:
                    default:
                        return debug::MarkerOverlayRole::GrabFingerSweepContact;
                    }
                };
                auto textColorForState = [&](grab_finger_pose_runtime::FingerSweepDebugState state) -> const float* {
                    switch (state) {
                    case grab_finger_pose_runtime::FingerSweepDebugState::MissFallback:
                        return kSweepMissTextColor;
                    case grab_finger_pose_runtime::FingerSweepDebugState::OutOfReach:
                        return kSweepOutOfReachTextColor;
                    case grab_finger_pose_runtime::FingerSweepDebugState::OverOpen:
                        return kSweepOverOpenTextColor;
                    case grab_finger_pose_runtime::FingerSweepDebugState::ClosedLimit:
                        return kSweepClosedLimitTextColor;
                    case grab_finger_pose_runtime::FingerSweepDebugState::Hit:
                    default:
                        return kSweepHitTextColor;
                    }
                };

                for (std::size_t fingerIndex = 0; fingerIndex < snapshot.capture.fingers.size(); ++fingerIndex) {
                    const auto& finger = snapshot.capture.fingers[fingerIndex];
                    if (!finger.valid) {
                        continue;
                    }

                    RE::NiPoint3 stateAnchorWorld = snapshot.objectWorld.translate;
                    bool stateAnchorValid = false;
                    RE::NiPoint3 pivotWorld{};
                    const bool pivotValid = finger.hasPivot;
                    if (pivotValid) {
                        pivotWorld = transform_math::localPointToWorld(snapshot.objectWorld, finger.pivotObjectLocal);
                        addMarkerPoint(debug::MarkerOverlayRole::GrabFingerSweepPivot, pivotWorld, 1.15f);
                        if (drawFingerSweptArcText) {
                            addTextLine(pivotWorld + RE::NiPoint3{ 0.0f, 0.0f, 0.8f }, kSweepLegendTextColor,
                                "%c %s PROXIMAL PIVOT", hand.isLeft() ? 'L' : 'R', grab_finger_pose_runtime::fingerSweepDebugFingerName(fingerIndex));
                        }
                    }
                    for (std::size_t probeIndex = 0; probeIndex < finger.probePointsObjectLocal.size(); ++probeIndex) {
                        const std::size_t pointCount = (std::min)(static_cast<std::size_t>(finger.probePointCount[probeIndex]), finger.probePointsObjectLocal[probeIndex].size());
                        if (pointCount == 0) {
                            continue;
                        }
                        const auto role = roleForProbe(finger.probeKind[probeIndex]);
                        RE::NiPoint3 previousWorld = transform_math::localPointToWorld(snapshot.objectWorld, finger.probePointsObjectLocal[probeIndex][0]);
                        if (pivotValid) {
                            // Radial spoke: this exposes the proximal rotation
                            // center that the colored distal trajectory alone
                            // cannot communicate.
                            addMarkerLine(role, pivotWorld, previousWorld);
                        }
                        addMarkerPoint(role, previousWorld, 0.55f);
                        if (finger.authoredOpenPointValid[probeIndex] != 0) {
                            const RE::NiPoint3 authoredOpenWorld =
                                transform_math::localPointToWorld(snapshot.objectWorld, finger.authoredOpenPointObjectLocal[probeIndex]);
                            addMarkerPoint(debug::MarkerOverlayRole::GrabFingerSweepAuthoredOpen, authoredOpenWorld, 0.62f);
                            if (drawFingerSweptArcText && finger.probeKind[probeIndex] == grab_finger_pose_math::CalibratedFingerProbe::Tip) {
                                addTextLine(authoredOpenWorld + RE::NiPoint3{ 0.0f, 0.0f, 0.7f }, kSweepAuthoredOpenTextColor, "1.0 AUTHORED OPEN");
                            }
                        }
                        if (drawFingerSweptArcText && finger.probeKind[probeIndex] == grab_finger_pose_math::CalibratedFingerProbe::Tip) {
                            addTextLine(previousWorld + RE::NiPoint3{ 0.0f, 0.0f, 0.7f }, kSweepLegendTextColor,
                                "%.2f SWEEP START", finger.probeStartOpenValue[probeIndex]);
                        }
                        for (std::size_t pointIndex = 1; pointIndex < pointCount; ++pointIndex) {
                            const RE::NiPoint3 pointWorld = transform_math::localPointToWorld(snapshot.objectWorld, finger.probePointsObjectLocal[probeIndex][pointIndex]);
                            addMarkerLine(role, previousWorld, pointWorld);
                            previousWorld = pointWorld;
                        }
                        if (finger.probeKind[probeIndex] == grab_finger_pose_math::CalibratedFingerProbe::Tip) {
                            stateAnchorWorld = previousWorld;
                            stateAnchorValid = true;
                        }
                    }

                    if (finger.hasContact) {
                        const RE::NiPoint3 contactCenterWorld = transform_math::localPointToWorld(snapshot.objectWorld, finger.contactCenterObjectLocal);
                        const float contactRadiusWorld = finger.contactRadiusObjectLocal * std::abs(snapshot.objectWorld.scale);
                        // A point marker is a three-axis diameter cross; using
                        // the captured radius exposes the exact contact sphere.
                        addMarkerPoint(debug::MarkerOverlayRole::GrabFingerSweepContact, contactCenterWorld, contactRadiusWorld);
                        stateAnchorWorld = contactCenterWorld;
                        stateAnchorValid = true;

                        if (finger.hasHitPoint) {
                            const RE::NiPoint3 hitPointWorld = transform_math::localPointToWorld(snapshot.objectWorld, finger.hitPointObjectLocal);
                            addMarkerLine(debug::MarkerOverlayRole::GrabFingerSweepContact, contactCenterWorld, hitPointWorld);
                            addMarkerPoint(debug::MarkerOverlayRole::GrabFingerSweepHitNormal, hitPointWorld, 0.8f);
                            if (finger.hasHitNormal) {
                                const RE::NiPoint3 normalWorld = grab_finger_pose_runtime::normalizedOrFallback(
                                    transform_math::localVectorToWorld(snapshot.objectWorld, finger.hitNormalObjectLocal), RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
                                addMarkerLine(debug::MarkerOverlayRole::GrabFingerSweepHitNormal, hitPointWorld, hitPointWorld + normalWorld * 4.0f);
                            }
                        }
                    }

                    if (stateAnchorValid && finger.state != grab_finger_pose_runtime::FingerSweepDebugState::Hit) {
                        addMarkerPoint(roleForState(finger.state), stateAnchorWorld, 1.6f);
                    }
                    if (drawFingerSweptArcText && stateAnchorValid) {
                        const char* selectedProbe = "NONE";
                        if (finger.selectedProbeIndex < finger.probeKind.size()) {
                            selectedProbe = grab_finger_pose_runtime::fingerSweepDebugProbeName(finger.probeKind[finger.selectedProbeIndex]);
                        }
                        const char* thumbLane = fingerIndex == 0 ? grab_finger_pose_math::thumbLaneName(finger.thumbLane) : "-";
                        addTextLine(stateAnchorWorld + RE::NiPoint3{ 0.0f, 0.0f, 1.8f + static_cast<float>(fingerIndex) * 0.6f }, textColorForState(finger.state),
                            "%c %s %s v=%.2f raw=%.2f range=%.2f->%.2f probe=%s lane=%s",
                            hand.isLeft() ? 'L' : 'R', grab_finger_pose_runtime::fingerSweepDebugFingerName(fingerIndex),
                            grab_finger_pose_runtime::fingerSweepDebugStateName(finger.state), finger.publishedValue, finger.rawCurveValue,
                            finger.probeStartOpenValue[0], finger.probeEndOpenValue[0], selectedProbe, thumbLane);
                    }
                }

                if (drawFingerSweptArcLiveSkeleton) {
                    root_flattened_finger_skeleton_runtime::Snapshot liveSkeleton{};
                    if (root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(hand.isLeft(), liveSkeleton, nullptr, SkeletonBoneCaptureSpace::Rendered)) {
                        for (const auto& finger : liveSkeleton.fingers) {
                            if (!finger.valid) {
                                continue;
                            }
                            addMarkerPoint(debug::MarkerOverlayRole::GrabFingerSweepLiveSkeleton, finger.points[0], 1.0f);
                            addMarkerPoint(debug::MarkerOverlayRole::GrabFingerSweepLiveSkeleton, finger.points[1], 0.8f);
                            addMarkerPoint(debug::MarkerOverlayRole::GrabFingerSweepLiveSkeleton, finger.points[2], 1.0f);
                            addMarkerLine(debug::MarkerOverlayRole::GrabFingerSweepLiveSkeleton, finger.points[0], finger.points[1]);
                            addMarkerLine(debug::MarkerOverlayRole::GrabFingerSweepLiveSkeleton, finger.points[1], finger.points[2]);
                        }
                    }
                }

                if (drawFingerSweptArcText) {
                    addTextLine(snapshot.objectWorld.translate + RE::NiPoint3{ 0.0f, 0.0f, 10.0f }, kSweepLegendTextColor,
                        "%c SWEEP tri=%u node=%u exact=%u | PIVOT white 1.0 gold TIP cyan OUT green IN purple CONTACT yellow LIVE gray", hand.isLeft() ? 'L' : 'R',
                        snapshot.capture.candidateTriangleCount, snapshot.capture.spatialNodeVisits, snapshot.capture.spatialTriangleTests);
                }
            };

            addFingerSweptArcDebug(_rightHand);
            addFingerSweptArcDebug(_leftHand);
            if (drawFingerSweptArcText && !anySweepCapture && (_rightHand.isHolding() || _leftHand.isHolding())) {
                addScreenTextLine(20.0f, 20.0f, kSweepMissTextColor, "SWEPT ARC: no captured regular solve; release and grab again (pinch is intentionally excluded)");
            }
        }

        if (drawFingerProbes) {
            auto addFingerProbeDebug = [&](const Hand& hand) {
                if ((hand.isLeft() && leftDisabled) || (!hand.isLeft() && rightDisabled)) {
                    return;
                }

                std::array<RE::NiPoint3, 5> starts{};
                std::array<RE::NiPoint3, 5> ends{};
                if (!hand.getGrabFingerProbeDebug(starts, ends)) {
                    return;
                }

                const auto role = hand.isLeft() ? debug::MarkerOverlayRole::LeftGrabFingerProbe : debug::MarkerOverlayRole::RightGrabFingerProbe;
                for (std::size_t i = 0; i < starts.size(); ++i) {
                    addMarkerLine(role, starts[i], ends[i]);
                }

                std::array<RE::NiPoint3, 5> padStarts{};
                std::array<RE::NiPoint3, 5> padEnds{};
                std::array<RE::NiPoint3, 5> padHits{};
                std::array<std::uint8_t, 5> padHitValid{};
                if (hand.getGrabFingerPadProbeDebug(padStarts, padEnds, padHits, padHitValid)) {
                    const auto padRole = hand.isLeft() ? debug::MarkerOverlayRole::LeftGrabFingerPadProbe : debug::MarkerOverlayRole::RightGrabFingerPadProbe;
                    for (std::size_t i = 0; i < padStarts.size(); ++i) {
                        const RE::NiPoint3 delta = padEnds[i] - padStarts[i];
                        const bool hasLine =
                            std::isfinite(padStarts[i].x) && std::isfinite(padStarts[i].y) && std::isfinite(padStarts[i].z) &&
                            std::isfinite(padEnds[i].x) && std::isfinite(padEnds[i].y) && std::isfinite(padEnds[i].z) &&
                            (delta.x * delta.x + delta.y * delta.y + delta.z * delta.z) > 0.000001f;
                        if (!hasLine) {
                            continue;
                        }
                        addMarkerLine(padRole, padStarts[i], padEnds[i]);
                        if (padHitValid[i]) {
                            addMarkerPoint(padRole, padHits[i], 1.4f);
                        }
                    }
                }

                std::array<RE::NiPoint3, 5> surfaceTargets{};
                std::array<std::uint8_t, 5> surfaceTargetValid{};
                if (hand.getGrabFingerSurfaceTargetDebug(surfaceTargets, surfaceTargetValid)) {
                    const auto targetRole = hand.isLeft() ? debug::MarkerOverlayRole::LeftGrabFingerSurfaceTarget : debug::MarkerOverlayRole::RightGrabFingerSurfaceTarget;
                    for (std::size_t i = 0; i < surfaceTargets.size(); ++i) {
                        if (surfaceTargetValid[i]) {
                            addMarkerPoint(targetRole, surfaceTargets[i], 2.0f);
                        }
                    }
                }
            };

            addFingerProbeDebug(_rightHand);
            addFingerProbeDebug(_leftHand);
        }

        if (drawHandBoneContacts) {
            auto addSemanticContactDebug = [&](const Hand& hand) {
                hand_semantic_contact_state::SemanticContactRecord contact{};
                if (!hand.getLastSemanticContact(contact) || contact.framesSinceContact >= 5) {
                    return;
                }

                RE::NiTransform sourceWorld{};
                if (!tryResolveLiveBodyWorldTransform(hknp, RE::hknpBodyId{ contact.handBodyId }, sourceWorld)) {
                    return;
                }

                RE::NiTransform targetWorld{};
                const bool hasTarget = tryResolveLiveBodyWorldTransform(hknp, RE::hknpBodyId{ contact.otherBodyId }, targetWorld);
                const auto role = hand.isLeft() ? debug::MarkerOverlayRole::LeftHandBoneContact : debug::MarkerOverlayRole::RightHandBoneContact;
                addMarkerPoint(role, sourceWorld.translate, 2.8f);
                if (hasTarget) {
                    addMarkerLine(role, sourceWorld.translate, targetWorld.translate);
                }
            };

            addSemanticContactDebug(_rightHand);
            addSemanticContactDebug(_leftHand);
        }

        const auto drawGripSphere = [&](const RE::NiPoint3& center, float radius, debug::MarkerOverlayRole role) {
            if (!std::isfinite(radius) || radius <= 0.0f) return;
            constexpr int segments = 24;
            for (int plane = 0; plane < 3; ++plane) {
                RE::NiPoint3 previous{};
                for (int i = 0; i <= segments; ++i) {
                    const float angle = i * 2.0f * std::numbers::pi_v<float> / segments;
                    const float a = radius * std::cos(angle), b = radius * std::sin(angle);
                    const auto point = center + (plane == 0 ? RE::NiPoint3{a,b,0} : plane == 1 ? RE::NiPoint3{a,0,b} : RE::NiPoint3{0,a,b});
                    if (i) addMarkerLine(role, previous, point);
                    previous = point;
                }
            }
        };
        if (drawAuthoredGripActivationZones) {
            for (bool isLeft : {true, false}) {
                const auto& snapshot = _twoHandedGrip.getWeaponGripSelectionDebug(isLeft);
                if (!snapshot.valid) continue;
                if (snapshot.pistolSupportContext && snapshot.supportPose.valid) {
                    const auto palmRing = [&](RE::NiPoint3 center, RE::NiPoint3 normal, debug::MarkerOverlayRole role) {
                        const float length=normal.Length();
                        if (!std::isfinite(length) || length<0.0001f) return;
                        normal=normal/length;
                        const auto cross=[](const RE::NiPoint3& a,const RE::NiPoint3& b) {
                            return RE::NiPoint3{a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
                        };
                        auto tangent=cross(normal,std::abs(normal.z)<0.9f ? RE::NiPoint3{0,0,1} : RE::NiPoint3{0,1,0});
                        tangent=tangent/tangent.Length();
                        const auto second=cross(normal,tangent);
                        addMarkerLine(role,center,center+normal*4.0f);
                        RE::NiPoint3 previous{};
                        for (int i=0;i<=16;++i) {
                            const float angle=i*2.0f*std::numbers::pi_v<float>/16;
                            const auto point=center+normal*2.0f+tangent*std::cos(angle)+second*std::sin(angle);
                            if (i) addMarkerLine(role,previous,point);
                            previous=point;
                        }
                    };
                    palmRing(snapshot.palm,snapshot.palmNormal,debug::MarkerOverlayRole::AuthoredSupportGripLiveSample);
                    palmRing(snapshot.supportSeat,snapshot.supportNormal,debug::MarkerOverlayRole::AuthoredSupportGripPalmSeat);
                    addMarkerLine(debug::MarkerOverlayRole::AuthoredSupportGripLiveSample,
                        snapshot.palm+snapshot.palmNormal*2.0f,snapshot.palm+snapshot.palmNormal*2.0f+snapshot.wristAxis*1.5f);
                    addMarkerLine(debug::MarkerOverlayRole::AuthoredSupportGripPalmSeat,
                        snapshot.supportSeat+snapshot.supportNormal*2.0f,snapshot.supportSeat+snapshot.supportNormal*2.0f+snapshot.supportWristAxis*1.5f);
                    drawGripSphere(snapshot.supportSeat,weapon_grip_selection::kSupportPoseOuterRadius,
                        debug::MarkerOverlayRole::AuthoredSupportGripPalmSeat);
                    const float poseColor[4]{0.3f,0.85f,1.0f,1.0f};
                    addTextLineSized(snapshot.supportSeat+RE::NiPoint3{0,0,7},1.7f,poseColor,
                        "PISTOL %s d=%.2f palm=%.2f wrist=%.2f credit=%.2f",
                        snapshot.supportPreferenceApplied && snapshot.supportPose.credit>0 ? "SUPPORT POSE MATCH" : "PROXIMITY",
                        snapshot.supportPose.distance,snapshot.supportPose.palmDot,snapshot.supportPose.wristDot,
                        snapshot.supportPreferenceApplied ? snapshot.supportPose.credit : 0.0f);
                }
                for (const auto& candidate : snapshot.candidates) {
                    if (candidate.kind == weapon_grip_selection::Kind::None) continue;
                    const bool selected = candidate.sameTarget(snapshot.selected);
                    const auto role = selected ? debug::MarkerOverlayRole::AuthoredGripActivationPass :
                        candidate.eligible ? debug::MarkerOverlayRole::AuthoredGripActivationAllowedRegion : debug::MarkerOverlayRole::AuthoredGripActivationFail;
                    drawGripSphere(candidate.world, candidate.radius, role);
                    addMarkerLine(role, snapshot.palm, candidate.world);
                    const float color[4]{selected ? 0.2f : 1.0f, candidate.eligible ? 1.0f : 0.2f, 0.2f, 1.0f};
                    addTextLineSized(candidate.world + RE::NiPoint3{0,0,isLeft ? 3.0f : 5.0f}, 1.7f, color,
                        "%s %s d=%.2f r=%.2f score=%.2f %s", isLeft ? "L" : "R", weapon_grip_selection::name(candidate.kind),
                        candidate.distance, candidate.radius, candidate.score(), selected ? "SELECTED" : candidate.eligible ? "ELIGIBLE" : "OUTSIDE");
                }
            }
        }

        if (drawWeaponAuthorityDebug) {
            TwoHandedGripDebugSnapshot snapshot{};
            if (_twoHandedGrip.getDebugAuthoritySnapshot(snapshot)) {
                addAxisTransform(snapshot.weaponWorld, debug::AxisOverlayRole::WeaponAuthority, snapshot.weaponWorld.translate, false);
                addAxisTransform(snapshot.rightRequestedHandWorld, debug::AxisOverlayRole::RightWeaponPrimaryGrip, snapshot.rightGripWorld, true);
                addAxisTransform(snapshot.leftRequestedHandWorld, debug::AxisOverlayRole::LeftWeaponSupportGrip, snapshot.leftGripWorld, true);
                addMarkerPoint(debug::MarkerOverlayRole::RightWeaponPrimaryGrip, snapshot.rightGripWorld, 3.0f);
                addMarkerPoint(debug::MarkerOverlayRole::LeftWeaponSupportGrip, snapshot.leftGripWorld, 3.0f);

                if (frik_visual_authority::isAvailable()) {
                    RE::NiTransform appliedRight{};
                    RE::NiTransform appliedLeft{};
                    const bool rightAppliedValid =
                        frik_visual_authority::tryGetPresentedHandWorldTransform(false, appliedRight);
                    const bool leftAppliedValid =
                        frik_visual_authority::tryGetPresentedHandWorldTransform(true, appliedLeft);
                    if (rightAppliedValid) {
                        addAxisTransform(appliedRight, debug::AxisOverlayRole::RightFrikAppliedHand, snapshot.rightRequestedHandWorld.translate, true);
                        addMarkerLine(debug::MarkerOverlayRole::RightWeaponAuthorityMismatch, snapshot.rightRequestedHandWorld.translate, appliedRight.translate);
                    }
                    if (leftAppliedValid) {
                        addAxisTransform(appliedLeft, debug::AxisOverlayRole::LeftFrikAppliedHand, snapshot.leftRequestedHandWorld.translate, true);
                        addMarkerLine(debug::MarkerOverlayRole::LeftWeaponAuthorityMismatch, snapshot.leftRequestedHandWorld.translate, appliedLeft.translate);
                    }

                    static std::uint32_t authorityMismatchLogCounter = 0;
                    if (rightAppliedValid && leftAppliedValid &&
                        ++authorityMismatchLogCounter >= 120) {
                        authorityMismatchLogCounter = 0;
                        ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip authority mismatch: right={:.2f}gu left={:.2f}gu",
                            pointDistance(snapshot.rightRequestedHandWorld.translate, appliedRight.translate),
                            pointDistance(snapshot.leftRequestedHandWorld.translate, appliedLeft.translate));
                    }
                }
            }
        }

        if (drawAuthoredGripActivationZones) {
            for (bool isLeft : {false, true}) {
                loose_weapon_grip_zone::AuthoredSupportDebug seat{};
                if (!loose_weapon_grip_zone::tryGetAuthoredSupportDebug(isLeft, seat)) continue;
                const auto role = seat.eligible ? debug::MarkerOverlayRole::AuthoredGripActivationPass : debug::MarkerOverlayRole::AuthoredGripActivationFail;
                drawGripSphere(seat.seatWorld, seat.radius, role);
                addMarkerLine(role, seat.seatWorld, seat.probeWorld);
            }
        }

        if (drawLooseWeaponGripZones) {
            /*
             * Loose-weapon grip zone: the FRIK-offset-projected firing grip on
             * a loosely held weapon, plus the palm-to-grip line the equip gate
             * measures. Marker grows while the palm is inside the equip
             * radius. Reuses the two-handed grip marker roles; the hand gate
             * keeps stale snapshots from drawing after release.
             */
            for (const bool zoneHandIsLeft : { false, true }) {
                const Hand& zoneHand = zoneHandIsLeft ? _leftHand : _rightHand;
                if (!zoneHand.isHoldingLooseWeapon()) {
                    continue;
                }
                loose_weapon_grip_zone::GripZoneDebug gripZone{};
                if (!loose_weapon_grip_zone::tryGetGripZoneDebug(zoneHandIsLeft, gripZone)) {
                    continue;
                }
                const auto markerRole =
                    zoneHandIsLeft ? debug::MarkerOverlayRole::LeftWeaponSupportGrip : debug::MarkerOverlayRole::RightWeaponPrimaryGrip;
                addMarkerPoint(markerRole, gripZone.gripWorld, gripZone.insideRadius ? 5.0f : 3.0f);
                if (gripZone.palmValid) {
                    addMarkerLine(markerRole, gripZone.palmWorld, gripZone.gripWorld);
                }
            }
        }

        if (drawGrabAuthorityProxy) {
            auto addGrabAuthorityAxisReference = [&](const Hand& hand, const RE::NiTransform& rawHandWorld) {
                if ((hand.isLeft() && leftDisabled) || (!hand.isLeft() && rightDisabled)) {
                    return;
                }

                const bool isLeft = hand.isLeft();
                RE::NiTransform palmAnchorTarget{};
                if (hand.tryGetPalmAnchorTarget(palmAnchorTarget)) {
                    addStoredColumnAxisTransform(
                        palmAnchorTarget,
                        isLeft ? debug::AxisOverlayRole::LeftGrabPalmGeneratedDirect : debug::AxisOverlayRole::RightGrabPalmGeneratedDirect,
                        rawHandWorld.translate,
                        true);
                }
            };

            auto addGrabAuthorityProxyClock = [&](Hand& hand, const RE::NiTransform& rawHandWorld) {
                if ((hand.isLeft() && leftDisabled) || (!hand.isLeft() && rightDisabled)) {
                    return;
                }

                GrabAuthorityProxyClockDebugSnapshot snapshot{};
                if (!hand.tryGetGrabAuthorityProxyClockDebugSnapshot(hknp, snapshot)) {
                    return;
                }

                const bool isLeft = hand.isLeft();
                if (drawGrabAuthorityProxyCollider) {
                    addBody(
                        snapshot.proxyBodyId,
                        isLeft ?
                            debug::BodyOverlayRole::LeftGrabAuthorityProxy :
                            debug::BodyOverlayRole::RightGrabAuthorityProxy);
                }

                RE::NiTransform currentPalmTarget{};
                const bool hasCurrentPalmTarget = hand.tryGetPalmAnchorTarget(currentPalmTarget);
                const RE::NiTransform currentPalmAuthority = hasCurrentPalmTarget ?
                    hand_bone_collider_geometry_math::generatedColliderFrameToGrabAuthorityFrame(currentPalmTarget) :
                    rawHandWorld;

                if (snapshot.hasQueuedTarget) {
                    addAxisTransform(
                        snapshot.queuedProxyTargetWorld,
                        isLeft ?
                            debug::AxisOverlayRole::LeftGrabAuthorityProxyTarget :
                            debug::AxisOverlayRole::RightGrabAuthorityProxyTarget,
                        currentPalmAuthority.translate,
                        true);
                    addMarkerPoint(
                        isLeft ?
                            debug::MarkerOverlayRole::LeftGrabAuthorityProxyTarget :
                            debug::MarkerOverlayRole::RightGrabAuthorityProxyTarget,
                        snapshot.queuedProxyTargetWorld.translate,
                        4.0f);
                    addMarkerLine(
                        isLeft ?
                            debug::MarkerOverlayRole::LeftGrabAuthorityProxyOffset :
                            debug::MarkerOverlayRole::RightGrabAuthorityProxyOffset,
                        currentPalmAuthority.translate,
                        snapshot.queuedProxyTargetWorld.translate);
                }
                if (snapshot.hasAppliedTarget) {
                    addAxisTransform(
                        snapshot.appliedProxyTargetWorld,
                        isLeft ?
                            debug::AxisOverlayRole::LeftGrabAuthorityProxyAppliedTarget :
                            debug::AxisOverlayRole::RightGrabAuthorityProxyAppliedTarget,
                        snapshot.appliedRawHandWorld.translate,
                        true);
                    addMarkerPoint(
                        isLeft ?
                            debug::MarkerOverlayRole::LeftGrabAuthorityProxyAppliedTarget :
                            debug::MarkerOverlayRole::RightGrabAuthorityProxyAppliedTarget,
                        snapshot.appliedProxyTargetWorld.translate,
                        3.0f);
                }
                if (snapshot.hasQueuedTarget && snapshot.hasAppliedTarget) {
                    addMarkerLine(
                        isLeft ?
                            debug::MarkerOverlayRole::LeftGrabAuthorityProxyClockDelta :
                            debug::MarkerOverlayRole::RightGrabAuthorityProxyClockDelta,
                        snapshot.appliedProxyTargetWorld.translate,
                        snapshot.queuedProxyTargetWorld.translate);
                }

                RE::NiTransform liveProxyWorld{};
                const bool liveProxyOk = tryResolveLiveBodyWorldTransform(
                    hknp,
                    snapshot.proxyBodyId,
                    liveProxyWorld);
                const float queuedToApplied =
                    snapshot.hasQueuedTarget && snapshot.hasAppliedTarget ?
                    (snapshot.queuedProxyTargetWorld.translate -
                        snapshot.appliedProxyTargetWorld.translate).Length() :
                    -1.0f;
                const float appliedToPre =
                    snapshot.hasAppliedTarget && liveProxyOk ?
                    (snapshot.appliedProxyTargetWorld.translate -
                        liveProxyWorld.translate).Length() :
                    -1.0f;
                constexpr float clockTextColor[4]{ 1.0f, 0.92f, 0.10f, 0.98f };
                const RE::NiPoint3 clockLabel = snapshot.hasQueuedTarget ?
                    snapshot.queuedProxyTargetWorld.translate :
                    snapshot.appliedProxyTargetWorld.translate;
                addTextLineSized(
                    clockLabel + RE::NiPoint3{ 0.0f, 0.0f, 6.0f },
                    1.45f,
                    clockTextColor,
                    "PROXY CLOCK %s Q=%llu F=%llu Q-A=%.2f A-PRE=%.2f",
                    isLeft ? "L" : "R",
                    static_cast<unsigned long long>(snapshot.queuedSequence),
                    static_cast<unsigned long long>(snapshot.flushSequence),
                    queuedToApplied,
                    appliedToPre);

            };

            addGrabAuthorityAxisReference(_rightHand, context.right.rawHandWorld);
            addGrabAuthorityAxisReference(_leftHand, context.left.rawHandWorld);

            addGrabAuthorityProxyClock(_rightHand, context.right.rawHandWorld);
            addGrabAuthorityProxyClock(_leftHand, context.left.rawHandWorld);

            const float proxyClockLegendColor[4]{ 1.0f, 1.0f, 1.0f, 0.98f };
            addScreenTextLine(
                18.0f,
                76.0f,
                proxyClockLegendColor,
                "GRAB PROXY CLOCK  QUEUED=YELLOW  APPLIED=ORANGE  PRE=MAGENTA  POST=CYAN");
        }

        if (frame.drawRockBodies) {
            if (drawHandColliders) {
                const auto addPalmBody = [&](const Hand& hand, debug::BodyOverlayRole role) {
                    const auto bodyId = hand.getCollisionBodyId();
                    addBody(
                        bodyId,
                        role);
                };
                addPalmBody(_rightHand, debug::BodyOverlayRole::RightHand);
                addPalmBody(_leftHand, debug::BodyOverlayRole::LeftHand);
            }

            /*
             * The dynamic hand/forearm twins are not part of any collider set,
             * so the overlay enumerates them explicitly behind the collider
             * master and their dedicated child switch. This view shows only
             * the arm-authority set, not the keyframed collider set.
             */
            if (drawDynamicHandColliders) {
                for (std::size_t twinIndex = 0; twinIndex < DynamicHandCollisionRuntime::kBodiesPerHand; ++twinIndex) {
                    addBody(
                        _dynamicHandCollision.proxyBodyIdForDebug(
                            false,
                            twinIndex),
                        debug::BodyOverlayRole::RightHand);

                    addBody(
                        _dynamicHandCollision.proxyBodyIdForDebug(
                            true,
                            twinIndex),
                        debug::BodyOverlayRole::LeftHand);
                }

                dynamic_hand_collision_telemetry::Snapshot telemetry{};
                if (_dynamicHandCollision.getTelemetrySnapshot(telemetry)) {
                    for (const auto& handSample : telemetry.hands) {
                        const auto& handInput = handSample.isLeft ? context.left : context.right;
                        const float handColor[4]{
                            handSample.isLeft ? 0.35f : 0.95f,
                            handSample.isLeft ? 0.82f : 0.52f,
                            1.0f,
                            0.96f,
                        };
                        RE::NiPoint3 labelAnchor = handInput.rawHandWorld.translate;
                        labelAnchor.z += 7.0f;
                        addTextLineSized(labelAnchor,
                            2.0f,
                            handColor,
                            "DHC %s C=%02X N=%u DEV=%.2f VIS=%.2f %s%s",
                            handSample.isLeft ? "L" : "R",
                            static_cast<unsigned int>(handSample.contactMask),
                            handSample.contactCount,
                            handSample.combinedContactDeviationGameUnits,
                            handSample.appliedVisualDeviationGameUnits,
                            handSample.visualActive ? "ACTIVE" : "IDLE",
                            handSample.ownedByStrongerSystem ? " OWNED" : "");
                        labelAnchor.z -= 3.0f;
                        addTextLineSized(labelAnchor,
                            1.7f,
                            handColor,
                            "ENTRY=%llu MASK=%02X SPEED=%.1f REC=%.3f",
                            static_cast<unsigned long long>(handSample.contactEntrySequence),
                            static_cast<unsigned int>(handSample.entryContactMask),
                            handSample.contactEntryApproachSpeedGameUnitsPerSecond,
                            handSample.teleportRecoverySecondsRemaining);
                        labelAnchor.z -= 2.5f;
                        addTextLineSized(labelAnchor,
                            1.55f,
                            handColor,
                            "SURF I/M/E/P=%llu/%llu/%llu/%llu",
                            static_cast<unsigned long long>(telemetry.surfaceImpulsePairSequence),
                            static_cast<unsigned long long>(telemetry.surfaceProcessedPairSequence),
                            static_cast<unsigned long long>(telemetry.surfaceEligiblePairSequence),
                            static_cast<unsigned long long>(telemetry.surfaceContactPublishSequence));

                        const auto requestedRole = handSample.isLeft ?
                            debug::MarkerOverlayRole::LeftDynamicHandRequestedDeviation :
                            debug::MarkerOverlayRole::RightDynamicHandRequestedDeviation;
                        for (const auto& twin : handSample.twins) {
                            if (!twin.physicsSampleValid || !twin.contactActive) {
                                continue;
                            }
                            addMarkerLine(requestedRole, twin.requestedTargetWorldGame, twin.liveBodyWorldGame);
                            addTextLineSized(twin.liveBodyWorldGame,
                                1.45f,
                                handColor,
                                "%s %s DEV=%.2f",
                                handSample.isLeft ? "L" : "R",
                                dynamic_hand_collision_telemetry::roleCode(twin.role),
                                twin.contactDeviationGameUnits);
                        }
                    }
                }
            }

            if (drawDynamicWeaponColliders) {
                addBody(
                    _dynamicWeaponCollision.proxyBodyIdForDebug(),
                    debug::BodyOverlayRole::DynamicWeaponProxy);

                DynamicWeaponCollisionRuntime::DebugSnapshot snapshot{};
                constexpr float proxyColor[4]{ 1.0f, 0.24f, 0.08f, 0.96f };
                if (_dynamicWeaponCollision.getDebugSnapshot(snapshot)) {
                    char panelLine[256]{};
                    std::snprintf(
                        panelLine,
                        sizeof(panelLine),
                        "DWC ACTIVE body=%u authority=%u constraint=%u contact=%s pivot=%.1fgu yield=%.1fdeg",
                        snapshot.bodyId,
                        snapshot.authorityBodyId,
                        snapshot.constraintId,
                        snapshot.contactActive ? "YES" : "NO",
                        snapshot.translationCorrectionGameUnits,
                        snapshot.rotationCorrectionDegrees);
                    addScreenTextLine(20.0f, 90.0f, proxyColor, panelLine);
                    RE::NiPoint3 labelAnchor =
                        snapshot.requestedWeaponWorld.translate;
                    labelAnchor.z += 8.0f;
                    addTextLineSized(
                        labelAnchor,
                        2.0f,
                        proxyColor,
                        "DWC COMPOUND contactBody=%u authorityBody=%u constraint=%u children=%u points=%llu contact=%s",
                        snapshot.bodyId,
                        snapshot.authorityBodyId,
                        snapshot.constraintId,
                        snapshot.compoundChildCount,
                        static_cast<unsigned long long>(snapshot.compoundPointCount),
                        snapshot.contactActive ? "YES" : "NO");
                    labelAnchor.z -= 3.0f;
                    addTextLineSized(
                        labelAnchor,
                        1.7f,
                        proxyColor,
                        "gripPivot=%.2fgu yield=%.2fdeg layer=%u retention=%.1fms visual=%s solve=%llu",
                        snapshot.translationCorrectionGameUnits,
                        snapshot.rotationCorrectionDegrees,
                        snapshot.otherLayer,
                        snapshot.contactRetentionSeconds * 1000.0f,
                        snapshot.visualCorrectionActive ? "ACTIVE" : "IDLE",
                        static_cast<unsigned long long>(
                            snapshot.solveSequence));
                    labelAnchor.z -= 3.0f;
                    addTextLineSized(
                        labelAnchor,
                        1.45f,
                        proxyColor,
                        "callbacks pair/obstacle/raw/manifold/admit=%llu/%llu/%llu/%llu/%llu",
                        static_cast<unsigned long long>(snapshot.proxyPairCallbackSequence),
                        static_cast<unsigned long long>(snapshot.obstacleCallbackSequence),
                        static_cast<unsigned long long>(snapshot.rawPointCallbackSequence),
                        static_cast<unsigned long long>(snapshot.processedManifoldCallbackSequence),
                        static_cast<unsigned long long>(snapshot.admittedContactSequence));
                    labelAnchor.z -= 3.0f;
                    addTextLineSized(
                        labelAnchor,
                        1.45f,
                        proxyColor,
                        "snapshot read/valid/id/contact/tele=%s/%s/%s/%s/%s",
                        snapshot.physicsSnapshotReadable ? "Y" : "N",
                        snapshot.physicsSnapshotValid ? "Y" : "N",
                        snapshot.physicsSnapshotIdentityCurrent ? "Y" : "N",
                        snapshot.physicsSnapshotContactActive ? "Y" : "N",
                        snapshot.physicsSnapshotTeleported ? "Y" : "N");
                } else {
                    addScreenTextLine(
                        20.0f,
                        90.0f,
                        proxyColor,
                        "DWC COMPOUND INACTIVE (enable dynamic collision + equip drawn weapon)");
                }
            }

            if (drawHandBoneColliders) {
                const std::uint32_t cap = static_cast<std::uint32_t>((std::clamp)(g_rockConfig.rockDebugMaxHandBoneBodiesDrawn, 0, 48));
                std::uint32_t drawn = 0;
                auto addHandBoneBodies = [&](const Hand& hand, debug::BodyOverlayRole anchorRole, debug::BodyOverlayRole segmentRole) {
                    const std::uint32_t count = hand.getHandColliderBodyCount();
                    for (std::uint32_t i = 0; i < count && drawn < cap; ++i) {
                        const std::uint32_t bodyId = hand.getHandColliderBodyIdAtomic(i);
                        if (bodyId == INVALID_BODY_ID) {
                            continue;
                        }
                        addBody(
                            RE::hknpBodyId{ bodyId },
                            i == 0 ? anchorRole : segmentRole);
                        ++drawn;
                    }
                };

                addHandBoneBodies(_rightHand, debug::BodyOverlayRole::RightHand, debug::BodyOverlayRole::RightHandSegment);
                addHandBoneBodies(_leftHand, debug::BodyOverlayRole::LeftHand, debug::BodyOverlayRole::LeftHandSegment);
            }

            if (drawBodyBoneColliders) {
                auto bodyOverlayRoleFor = [](skeleton_bone_debug_math::BoneColliderRole role) {
                    using skeleton_bone_debug_math::BoneColliderRole;
                    switch (role) {
                    case BoneColliderRole::TorsoSegment:
                        return debug::BodyOverlayRole::BodyTorsoSegment;
                    case BoneColliderRole::FootSegment:
                        return debug::BodyOverlayRole::BodyFootSegment;
                    case BoneColliderRole::LegSegment:
                        return debug::BodyOverlayRole::BodyLegSegment;
                    case BoneColliderRole::UpperArmSegment:
                    case BoneColliderRole::ForearmSegment:
                    case BoneColliderRole::HandSegment:
                    case BoneColliderRole::FingerSegment:
                        return debug::BodyOverlayRole::BodyArmSegment;
                    }
                    return debug::BodyOverlayRole::BodyTorsoSegment;
                };

                const std::uint32_t bodyCap = static_cast<std::uint32_t>((std::clamp)(g_rockConfig.rockDebugMaxBodyBoneBodiesDrawn, 0, 64));
                std::uint32_t bodyDrawn = 0;
                const std::uint32_t bodyCount = _bodyBoneColliders.getBodyCount();
                for (std::uint32_t i = 0; i < bodyCount && bodyDrawn < bodyCap; ++i) {
                    const std::uint32_t bodyId = _bodyBoneColliders.getBodyIdAtomic(i);
                    if (bodyId == INVALID_BODY_ID) {
                        continue;
                    }

                    BodyBoneColliderMetadata metadata{};
                    const auto role =
                        _bodyBoneColliders.tryGetBodyMetadataAtIndexAtomic(i, bodyId, metadata) ? bodyOverlayRoleFor(metadata.role) : debug::BodyOverlayRole::BodyTorsoSegment;
                    addBody(
                        RE::hknpBodyId{ bodyId },
                        role);
                    ++bodyDrawn;
                }
            }

            for (std::uint32_t index = 0;
                 index < grabbedWeaponPartColliderCount;
                 ++index) {
                const std::uint32_t bodyId =
                    grabbedWeaponPartColliderBodyIds[index];
                addBody(
                    RE::hknpBodyId{ bodyId },
                    debug::BodyOverlayRole::FocusedWeaponPart);
            }

            if (drawWeaponColliders) {
                const auto weaponSnapshot =
                    _weaponCollision.getWeaponBodySnapshotAtomic();
                const auto maximumWeaponBodies = static_cast<std::uint32_t>(
                    (std::max)(0, g_rockConfig.rockDebugMaxWeaponBodiesDrawn));
                for (std::uint32_t i = 0;
                     i < weaponSnapshot.count && i < maximumWeaponBodies;
                     ++i) {
                    if (isGrabbedWeaponPartColliderBody(
                            weaponSnapshot.bodyIds[i])) {
                        continue;
                    }
                    addBody(
                        RE::hknpBodyId{ weaponSnapshot.bodyIds[i] },
                        debug::BodyOverlayRole::Weapon);
                }
            }
        }

        if (frame.drawTargetBodies) {
            auto addHandTarget = [&](const Hand& hand) {
                if (hand.isHolding()) {
                    addBody(hand.getSavedObjectState().bodyId, debug::BodyOverlayRole::Target);
                    return;
                }
                if (hand.hasSelection()) {
                    addBody(hand.getSelection().bodyId, debug::BodyOverlayRole::Target);
                }
            };

            addHandTarget(_rightHand);
            addHandTarget(_leftHand);
        }

        debug::PublishFrame(frame);
    }
