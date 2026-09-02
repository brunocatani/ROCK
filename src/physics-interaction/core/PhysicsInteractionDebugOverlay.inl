/*
 * Debug overlay publishing is split from the runtime frame loop because it is diagnostic fan-out over many subsystems, not interaction authority. The fragment remains in this translation unit so existing helper visibility and behavior stay unchanged.
 */
    /*
     * OVERLAY-POINT probe: once per game frame, at the same frame phase where the
     * overlay publishes body IDs, sample every trajectory the player's eye can
     * compare: the current raw wand, the last APPLIED commanded proxy target plus
     * the wand sample it was built from, the live held-object body, the live hand
     * collider body, the live proxy body, and the current-frame stereo origin.
     * Consecutive lines decompose visible held-object stutter into its links
     * (object-vs-target, target-vs-wand, wand-vs-camera, camera-vs-world) in one
     * common time base. Bodies here reflect the last completed physics step - the
     * exact state this frame renders. Diagnostic only; rides
     * bDebugGrabFrameLogging like the HELD_POSTSOLVE probe and logs only while a
     * hand holds an object.
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
            .colliderPhaseDiagnostics =
                g_rockConfig.rockDebugDrawColliderPhaseDiagnostics,
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
            .handBoneContacts = g_rockConfig.rockDebugDrawHandBoneContacts,
            .grabAuthorityProxy = g_rockConfig.rockDebugDrawGrabAuthorityProxy,
            .videoSyncMarker = g_rockConfig.rockDebugVideoSyncMarker,
            .weaponAuthority = g_rockConfig.rockDebugDrawWeaponAuthority,
            .looseWeaponGripZones =
                g_rockConfig.rockDebugDrawLooseWeaponGripZones,
            .authoredGripActivationZones =
                g_rockConfig.rockDebugDrawAuthoredGripActivationZones,
            .nativeScopeActivation =
                g_rockConfig.rockDebugDrawNativeScopeActivation,
            .worldOriginDiagnostics =
                g_rockConfig.rockDebugWorldObjectOriginDiagnostics,
        });

        const bool drawTargetColliders = visualization.targetColliders;
        const bool drawColliderPhaseDiagnostics =
            visualization.colliderPhaseDiagnostics;
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
        const bool drawHandBoneContacts = visualization.handBoneContacts;
        const bool drawGrabAuthorityProxy = visualization.grabAuthorityProxy;
        const bool drawPerformanceProfilerOverlay = performance_profiler::overlayTextEnabled();
        const bool drawVideoSyncMarker = visualization.videoSyncMarker;
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
        const bool drawWorldOriginDiagnostics =
            visualization.worldOriginDiagnostics;

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
            drawGrabAuthorityProxyCollider;
        if (drawWorldOriginDiagnostics && !s_worldOriginDiagnosticsEnabledLogged) {
            ROCK_LOG_INFO(Hand,
                "World object origin diagnostics enabled: intervalFrames={} warnThresholdGameUnits={:.2f} visualSourceOrder=bodyOwnerNode>hitNode>visualNode>referenceRoot",
                g_rockConfig.rockDebugWorldObjectOriginLogIntervalFrames,
                g_rockConfig.rockDebugWorldObjectOriginMismatchWarnGameUnits);
            s_worldOriginDiagnosticsEnabledLogged = true;
        } else if (!drawWorldOriginDiagnostics) {
            s_worldOriginDiagnosticsEnabledLogged = false;
        }
        if (!drawAnyRockColliderBodies && !drawTargetColliders && !drawHandAxes && !drawGrabPivots &&
            !drawFingerSweptArc && !drawPalmVectors && !drawGrabPockets && !drawRootFlattenedFingerSkeleton && !drawSkeletonBones &&
            !drawHandBoneContacts && !drawGrabAuthorityProxy && !drawPerformanceProfilerOverlay &&
            !drawWeaponAuthorityDebug && !drawLooseWeaponGripZones && !drawNativeScopeActivation && !drawWorldOriginDiagnostics &&
            !drawDynamicHandColliders && !drawDynamicWeaponColliders && !drawAuthoredSupportGripDebug && !drawProviderOverlay && !drawVideoSyncMarker) {
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
        frame.drawColliderPhaseDiagnostics = drawColliderPhaseDiagnostics;
        frame.drawAxes = drawHandAxes || drawGrabAuthorityProxy || drawNativeScopeActivation ||
            drawWeaponAuthorityDebug || drawAuthoredSupportGripDebug;
        frame.drawMarkers = drawGrabPivots || drawFingerSweptArc || drawPalmVectors || drawGrabPockets || drawRootFlattenedFingerSkeleton ||
            drawHandBoneContacts || drawGrabAuthorityProxy ||
            drawWeaponAuthorityDebug || drawLooseWeaponGripZones || drawNativeScopeActivation || drawWorldOriginDiagnostics || drawDynamicHandColliders ||
            drawAuthoredSupportGripDebug;
        frame.drawSkeleton = drawSkeletonBones;
        frame.drawColoredLines = providerOverlay && providerOverlay->lineCount > 0;
        frame.drawText = drawFingerSweptArcText || drawPerformanceProfilerOverlay ||
            drawDynamicHandColliders || drawDynamicWeaponColliders || drawNativeScopeActivation ||
            drawAuthoredSupportGripDebug || drawVideoSyncMarker || drawGrabPockets ||
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
        RE::bhkWorld* originDiagnosticBhk = drawWorldOriginDiagnostics ? context.bhkWorld : nullptr;
        const bool rightDisabled = context.right.disabled;
        const bool leftDisabled = context.left.disabled;

        auto addBodyWithTarget = [&](RE::hknpBodyId bodyId,
                                     debug::BodyOverlayRole role,
                                     const RE::NiTransform* currentTarget) {
            if (bodyId.value == INVALID_BODY_ID) {
                return;
            }

            for (std::uint32_t i = 0; i < frame.count; i++) {
                if (frame.entries[i].bodyId.value == bodyId.value && frame.entries[i].role == role) {
                    if (drawColliderPhaseDiagnostics && currentTarget) {
                        frame.entries[i].currentTarget = *currentTarget;
                        frame.entries[i].hasCurrentTarget = true;
                    }
                    return;
                }
            }

            if (frame.count >= frame.entries.size()) {
                return;
            }

            auto& entry = frame.entries[frame.count++];
            entry.bodyId = bodyId;
            entry.role = role;
            if (drawColliderPhaseDiagnostics && currentTarget) {
                entry.currentTarget = *currentTarget;
                entry.hasCurrentTarget = true;
            }
        };

        auto addBody = [&](RE::hknpBodyId bodyId, debug::BodyOverlayRole role) {
            addBodyWithTarget(bodyId, role, nullptr);
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

        auto addWorldOriginDiagnostic =
            [&](const Hand& hand, bool held, RE::hknpBodyId bodyId, RE::TESObjectREFR* refr, RE::NiAVObject* hitNode, RE::NiAVObject* visualNode) {
                if (!drawWorldOriginDiagnostics || !originDiagnosticBhk || !hknp) {
                    return;
                }

                origin_diagnostics::TargetOriginSample sample{};
                if (!origin_diagnostics::sampleTarget(originDiagnosticBhk,
                        hknp,
                        bodyId,
                        refr,
                        hitNode,
                        visualNode,
                        g_rockConfig.rockDebugWorldObjectOriginMismatchWarnGameUnits,
                        sample)) {
                    return;
                }

                origin_diagnostics::logSampleIfNeeded(hand.handName(),
                    held,
                    sample,
                    static_cast<std::uint32_t>((std::max)(g_rockConfig.rockDebugWorldObjectOriginLogIntervalFrames, 1)));
                origin_diagnostics::publishMarkers(frame, sample);
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

        if (drawVideoSyncMarker) {
            // VIDEO-SYNC marker: a per-frame counter drawn in the VR view AND
            // logged on the same steady-clock microsecond base as
            // OVERLAY_POINT/RENDER_READ, so an external headset recording aligns
            // 1:1 with the probe lines (decode the on-screen counter per video
            // frame, join on seq/t). WORLD-anchored at the held object — the
            // screen-space overlay text path does not render visibly in the VR
            // view — with an HMD-forward fallback between grabs so alignment
            // coverage never drops. The log line emits regardless of whether an
            // anchor resolved, so the timeline stays gap-free.
            static std::uint32_t s_videoSyncFrameCounter = 0;
            ++s_videoSyncFrameCounter;
            const auto syncMicroseconds = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            RE::NiPoint3 syncAnchor{};
            bool syncAnchorValid = false;
            auto anchorAboveHeldObject = [&](const Hand& hand) {
                if (syncAnchorValid || !hand.isHolding() || !hknp) {
                    return;
                }
                RE::NiTransform heldWorld{};
                if (tryResolveLiveBodyWorldTransform(hknp, hand.getSavedObjectState().bodyId, heldWorld)) {
                    syncAnchor = heldWorld.translate;
                    syncAnchor.z += 10.0f;
                    syncAnchorValid = true;
                }
            };
            anchorAboveHeldObject(_rightHand);
            anchorAboveHeldObject(_leftHand);
            if (!syncAnchorValid && context.hasHmdFrame) {
                const RE::NiPoint3& forward = context.hmdForwardWorld;
                const float lengthSquared = forward.x * forward.x + forward.y * forward.y + forward.z * forward.z;
                if (std::isfinite(lengthSquared) && lengthSquared > 0.000001f) {
                    syncAnchor = context.hmdPositionWorld + forward * (60.0f / std::sqrt(lengthSquared));
                    syncAnchorValid = true;
                }
            }
            if (syncAnchorValid) {
                const float syncColor[4]{ 1.0f, 1.0f, 0.0f, 1.0f };
                addTextLineSized(syncAnchor, g_rockConfig.rockDebugVideoSyncMarkerSize, syncColor, "SYNC %07u", s_videoSyncFrameCounter);
            }
            ROCK_LOG_DEBUG(Hand, "VIDEO_SYNC: seq={} t={}us", s_videoSyncFrameCounter, syncMicroseconds);
        }

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

                const RE::NiPoint3 pinchAxis =
                    grab_pinch_pocket_policy::normalizeOrFallback(handInput.indexPadWorld - handInput.thumbPadWorld, RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
                const float axisBlend =
                    std::clamp(g_rockConfig.rockGrabPinchDetectionAxisBlend, 0.0f, 1.0f);
                const RE::NiPoint3 pinchDetection =
                    grab_pinch_pocket_policy::normalizeOrFallback(pinchAxis * axisBlend + handInput.pinchDirectionWorld * (1.0f - axisBlend), pinchAxis);
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
            if (s_directSkeletonBoneReader.capture(skeletonBoneMode, skeletonBoneSource, snapshot)) {
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
                            if (frik_visual_authority::
                                    tryGetHandWorldTransform(
                                        frik_visual_authority::Hand::Right,
                                        rightHandWorld)) {
                                rightHandScale = rightHandWorld.scale;
                            }
                            if (frik_visual_authority::
                                    tryGetHandWorldTransform(
                                        frik_visual_authority::Hand::Left,
                                        leftHandWorld)) {
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
                if (!root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(isLeft, snapshot)) {
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
                    if (root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(hand.isLeft(), liveSkeleton)) {
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

        if (drawAuthoredSupportGripDebug) {
            AuthoredSupportGripDebugSnapshot snapshot{};
            if (_twoHandedGrip.getAuthoredSupportGripDebugSnapshot(snapshot)) {
                // YELLOW cross: the final authored palm seat and center of the
                // touch-substitution radius. BLUE cross: the live palm touch
                // probe expressed in that same current Weapon frame. Wrist/
                // hand-bone origins no longer participate in eligibility.
                addMarkerPoint(
                    debug::MarkerOverlayRole::AuthoredSupportGripPalmSeat,
                    snapshot.authoredPalmSeatWorld,
                    3.0f);
                addMarkerPoint(
                    debug::MarkerOverlayRole::AuthoredSupportGripLiveSample,
                    snapshot.liveTouchProbeWorld,
                    2.5f);
                addMarkerLine(
                    debug::MarkerOverlayRole::AuthoredSupportGripPalmSeat,
                    snapshot.liveTouchProbeWorld,
                    snapshot.authoredPalmSeatWorld);

                constexpr float kSeatColor[4]{ 1.0f, 0.78f, 0.05f, 0.98f };
                constexpr float kCoordinateColor[4]{ 0.92f, 0.92f, 1.0f, 0.94f };
                const RE::NiPoint3 labelAnchor =
                    snapshot.authoredPalmSeatWorld +
                    RE::NiPoint3{ 0.0f, 0.0f, 4.0f };
                addTextLineSized(
                    labelAnchor,
                    2.1f,
                    kSeatColor,
                    "AUTHORED SUPPORT SEAT %s topology=%s d=%.2f r=%.2f %s",
                    snapshot.supportHandIsLeft ? "LEFT" : "RIGHT",
                    authored_weapon_grip_activation_policy::handTopologyName(
                        snapshot.handTopology),
                    snapshot.weaponRelativeDistanceGameUnits,
                    snapshot.touchRadiusGameUnits,
                    snapshot.insideTouchRadius ? "INSIDE" : "OUTSIDE");
                addTextLineSized(
                    labelAnchor + RE::NiPoint3{ 0.0f, 0.0f, -2.2f },
                    1.7f,
                    kCoordinateColor,
                    "seat Weapon=(%.2f, %.2f, %.2f)",
                    snapshot.authoredPalmSeatWeaponLocal.x,
                    snapshot.authoredPalmSeatWeaponLocal.y,
                    snapshot.authoredPalmSeatWeaponLocal.z);
                addTextLineSized(
                    labelAnchor + RE::NiPoint3{ 0.0f, 0.0f, -4.2f },
                    1.7f,
                    kCoordinateColor,
                    "touch Weapon=(%.2f, %.2f, %.2f) frameErr=%.4f",
                    snapshot.liveTouchProbeWeaponLocal.x,
                    snapshot.liveTouchProbeWeaponLocal.y,
                    snapshot.liveTouchProbeWeaponLocal.z,
                    snapshot.frameAgreementErrorGameUnits);

                if (drawAuthoredGripActivationZones) {
                    const float drawRadiusGameUnits = (std::min)(
                        snapshot.radialCapGameUnits,
                        12.0f);
                    const auto activationBoundary =
                        authored_weapon_grip_activation_policy::
                            resolveActivationBoundaryDimensions(
                                drawRadiusGameUnits);
                    if (snapshot.canonicalAxesValid &&
                        activationBoundary.valid) {
                        const auto axisEnd = [&](const RE::NiPoint3& axis) {
                            return RE::NiPoint3{
                                snapshot.authoredPalmSeatWorld.x +
                                    axis.x * drawRadiusGameUnits,
                                snapshot.authoredPalmSeatWorld.y +
                                    axis.y * drawRadiusGameUnits,
                                snapshot.authoredPalmSeatWorld.z +
                                    axis.z * drawRadiusGameUnits,
                            };
                        };
                        addMarkerLine(
                            debug::MarkerOverlayRole::AuthoredGripActivationSupportSideAxis,
                            snapshot.authoredPalmSeatWorld,
                            axisEnd(snapshot.supportSideAxisWorld));
                        addMarkerLine(
                            debug::MarkerOverlayRole::AuthoredGripActivationDownAxis,
                            snapshot.authoredPalmSeatWorld,
                            axisEnd(snapshot.downAxisWorld));
                        addMarkerLine(
                            debug::MarkerOverlayRole::AuthoredGripActivationReferenceAxis,
                            snapshot.authoredPalmSeatWorld,
                            axisEnd(snapshot.referenceAxisWorld));

                        const auto drawWireCone = [&](
                            const RE::NiPoint3& axis,
                            const RE::NiPoint3& tangentA,
                            const RE::NiPoint3& tangentB) {
                            constexpr std::size_t SegmentCount = 12;
                            std::array<RE::NiPoint3, SegmentCount> rim{};
                            for (std::size_t segment = 0;
                                 segment < SegmentCount;
                                 ++segment) {
                                const float angle =
                                    static_cast<float>(segment) *
                                    2.0f * std::numbers::pi_v<float> /
                                    static_cast<float>(SegmentCount);
                                const float radialA =
                                    std::cos(angle) *
                                    activationBoundary.rimRadiusGameUnits;
                                const float radialB =
                                    std::sin(angle) *
                                    activationBoundary.rimRadiusGameUnits;
                                rim[segment] = RE::NiPoint3{
                                    snapshot.authoredPalmSeatWorld.x +
                                        axis.x *
                                            activationBoundary.axialGameUnits +
                                        tangentA.x * radialA +
                                        tangentB.x * radialB,
                                    snapshot.authoredPalmSeatWorld.y +
                                        axis.y *
                                            activationBoundary.axialGameUnits +
                                        tangentA.y * radialA +
                                        tangentB.y * radialB,
                                    snapshot.authoredPalmSeatWorld.z +
                                        axis.z *
                                            activationBoundary.axialGameUnits +
                                        tangentA.z * radialA +
                                        tangentB.z * radialB,
                                };
                            }
                            for (std::size_t segment = 0;
                                 segment < SegmentCount;
                                 ++segment) {
                                addMarkerLine(
                                    debug::MarkerOverlayRole::AuthoredGripActivationAllowedRegion,
                                    rim[segment],
                                    rim[(segment + 1) % SegmentCount]);
                                if ((segment % (SegmentCount / 4)) == 0) {
                                    addMarkerLine(
                                        debug::MarkerOverlayRole::AuthoredGripActivationAllowedRegion,
                                        snapshot.authoredPalmSeatWorld,
                                        rim[segment]);
                                }
                            }
                        };

                        const auto drawWireSweptActivationRegion = [&]() {
                            const auto canonicalPoint = [&](
                                const float supportSideOffset,
                                const float downOffset,
                                const float referenceOffset) {
                                return RE::NiPoint3{
                                    snapshot.authoredPalmSeatWorld.x +
                                        snapshot.supportSideAxisWorld.x *
                                            supportSideOffset +
                                        snapshot.downAxisWorld.x * downOffset +
                                        snapshot.referenceAxisWorld.x *
                                            referenceOffset,
                                    snapshot.authoredPalmSeatWorld.y +
                                        snapshot.supportSideAxisWorld.y *
                                            supportSideOffset +
                                        snapshot.downAxisWorld.y * downOffset +
                                        snapshot.referenceAxisWorld.y *
                                            referenceOffset,
                                    snapshot.authoredPalmSeatWorld.z +
                                        snapshot.supportSideAxisWorld.z *
                                            supportSideOffset +
                                        snapshot.downAxisWorld.z * downOffset +
                                        snapshot.referenceAxisWorld.z *
                                            referenceOffset,
                                };
                            };
                            const auto addAllowedLine = [&](
                                const RE::NiPoint3& from,
                                const RE::NiPoint3& to) {
                                addMarkerLine(
                                    debug::MarkerOverlayRole::
                                        AuthoredGripActivationAllowedRegion,
                                    from,
                                    to);
                            };

                            constexpr std::size_t ArcSegmentCount = 12;
                            RE::NiPoint3 previousPositive{};
                            RE::NiPoint3 previousNegative{};
                            RE::NiPoint3 previousCenter{};
                            for (std::size_t segment = 0;
                                 segment <= ArcSegmentCount;
                                 ++segment) {
                                const float angle =
                                    static_cast<float>(segment) *
                                    (0.5f * std::numbers::pi_v<float>) /
                                    static_cast<float>(ArcSegmentCount);
                                const float supportSideScale = std::cos(angle);
                                const float downScale = std::sin(angle);
                                const auto center = canonicalPoint(
                                    supportSideScale * drawRadiusGameUnits,
                                    downScale * drawRadiusGameUnits,
                                    0.0f);
                                const auto positive = canonicalPoint(
                                    supportSideScale *
                                        activationBoundary.axialGameUnits,
                                    downScale *
                                        activationBoundary.axialGameUnits,
                                    activationBoundary.rimRadiusGameUnits);
                                const auto negative = canonicalPoint(
                                    supportSideScale *
                                        activationBoundary.axialGameUnits,
                                    downScale *
                                        activationBoundary.axialGameUnits,
                                    -activationBoundary.rimRadiusGameUnits);
                                if (segment > 0) {
                                    addAllowedLine(previousPositive, positive);
                                    addAllowedLine(previousNegative, negative);
                                    addAllowedLine(previousCenter, center);
                                }
                                if ((segment % (ArcSegmentCount / 4)) == 0) {
                                    addAllowedLine(
                                        snapshot.authoredPalmSeatWorld,
                                        positive);
                                    addAllowedLine(
                                        snapshot.authoredPalmSeatWorld,
                                        negative);
                                }
                                previousPositive = positive;
                                previousNegative = negative;
                                previousCenter = center;
                            }

                            const auto drawEndpointCap =
                                [&](const bool supportSideEndpoint) {
                                constexpr std::size_t SegmentCount = 12;
                                RE::NiPoint3 previous{};
                                for (std::size_t segment = 0;
                                     segment <= SegmentCount;
                                     ++segment) {
                                    const float angle =
                                        0.5f * std::numbers::pi_v<float> +
                                        static_cast<float>(segment) *
                                            std::numbers::pi_v<float> /
                                            static_cast<float>(SegmentCount);
                                    const float tangentOffset =
                                        std::cos(angle) *
                                        activationBoundary.rimRadiusGameUnits;
                                    const auto point = canonicalPoint(
                                        supportSideEndpoint ?
                                            activationBoundary.axialGameUnits :
                                            tangentOffset,
                                        supportSideEndpoint ?
                                            tangentOffset :
                                            activationBoundary.axialGameUnits,
                                        std::sin(angle) *
                                            activationBoundary.rimRadiusGameUnits);
                                    if (segment > 0) {
                                        addAllowedLine(previous, point);
                                    }
                                    if (segment == SegmentCount / 2) {
                                        addAllowedLine(
                                            snapshot.authoredPalmSeatWorld,
                                            point);
                                    }
                                    previous = point;
                                }
                            };
                            drawEndpointCap(true);
                            drawEndpointCap(false);
                        };
                        if (snapshot.weaponFamily ==
                                authored_weapon_grip_activation_policy::
                                    WeaponFamily::OneHandGun) {
                            drawWireCone(
                                snapshot.supportSideAxisWorld,
                                snapshot.downAxisWorld,
                                snapshot.referenceAxisWorld);
                        }
                        if (snapshot.weaponFamily ==
                            authored_weapon_grip_activation_policy::
                                WeaponFamily::TwoHandGun) {
                            drawWireSweptActivationRegion();
                        }
                    }

                    const auto liveVectorRole =
                        snapshot.activationSpatialPass ?
                        debug::MarkerOverlayRole::AuthoredGripActivationPass :
                        debug::MarkerOverlayRole::AuthoredGripActivationFail;
                    addMarkerLine(
                        liveVectorRole,
                        snapshot.authoredPalmSeatWorld,
                        snapshot.liveTouchProbeWorld);

                    for (std::size_t landmarkIndex = 0;
                         landmarkIndex <
                            AuthoredSupportGripDebugSnapshot::kPoseLandmarkCount;
                         ++landmarkIndex) {
                        const bool witnessValid =
                            (snapshot.poseSurfaceWitnessMask &
                                static_cast<std::uint8_t>(1u << landmarkIndex)) != 0;
                        const auto witnessRole = witnessValid ?
                            debug::MarkerOverlayRole::AuthoredGripActivationPass :
                            debug::MarkerOverlayRole::AuthoredGripActivationFail;
                        addMarkerPoint(
                            witnessRole,
                            snapshot.poseLandmarksWorld[landmarkIndex],
                            landmarkIndex == 0 ? 2.4f : 1.7f);
                        if (witnessValid) {
                            addMarkerLine(
                                witnessRole,
                                snapshot.poseLandmarksWorld[landmarkIndex],
                                snapshot.poseSurfaceWitnessWorld[landmarkIndex]);
                        }
                    }

                    constexpr float kPassColor[4]{ 0.25f, 1.0f, 0.12f, 0.98f };
                    constexpr float kFailColor[4]{ 1.0f, 0.18f, 0.08f, 0.98f };
                    const bool authoredActivationPass =
                        snapshot.activationSpatialPass;
                    const float* verdictColor =
                        authoredActivationPass ?
                        kPassColor : kFailColor;
                    addTextLineSized(
                        labelAnchor + RE::NiPoint3{ 0.0f, 0.0f, -6.4f },
                        1.75f,
                        verdictColor,
                        "AUTHORED ACTIVATION family=%s slot=%08X (%s) base=%08X",
                        authored_weapon_grip_activation_policy::weaponFamilyName(
                            snapshot.weaponFamily),
                        snapshot.effectiveEquipSlotFormID,
                        snapshot.effectiveEquipSlotUsesInstanceData ?
                            "INSTANCE" : "BASE",
                        snapshot.baseEquipSlotFormID);
                    addTextLineSized(
                        labelAnchor + RE::NiPoint3{ 0.0f, 0.0f, -8.4f },
                        1.65f,
                        verdictColor,
                        "d=%.2f cap=%.2f sideDot=%.3f downDot=%.3f arcDot=%.3f region=%s spatial=%s",
                        snapshot.weaponRelativeDistanceGameUnits,
                        snapshot.radialCapGameUnits,
                        snapshot.supportSideDot,
                        snapshot.downDot,
                        snapshot.sweptArcDot,
                        authored_weapon_grip_activation_policy::activationRegionName(
                            snapshot.selectedRegion),
                        snapshot.activationSpatialPass ? "PASS" : "FAIL");
                    addTextLineSized(
                        labelAnchor + RE::NiPoint3{ 0.0f, 0.0f, -10.4f },
                        1.65f,
                        kCoordinateColor,
                        "gates axes=%d class=%d radial=%d direction=%d topology=%d stable=%d",
                        snapshot.canonicalAxesValid ? 1 : 0,
                        snapshot.classifierSupported ? 1 : 0,
                        snapshot.radialPass ? 1 : 0,
                        snapshot.directionPass ? 1 : 0,
                        snapshot.topologyPass ? 1 : 0,
                        snapshot.directionUsedLastStableSample ? 1 : 0);
                    addTextLineSized(
                        labelAnchor + RE::NiPoint3{ 0.0f, 0.0f, -12.4f },
                        1.65f,
                        kCoordinateColor,
                        "collision diagnostic witnesses=%u/6 mask=%02X evidence=%s authored=%s current=%s",
                        static_cast<unsigned>(snapshot.poseSurfaceWitnessCount),
                        static_cast<unsigned>(snapshot.poseSurfaceWitnessMask),
                        snapshot.poseEvidencePass ? "PASS" : "FAIL",
                        authoredActivationPass ? "AUTHORED" : "DYNAMIC/NONE",
                        snapshot.currentSupportGripActive ?
                            (snapshot.currentAuthoredSupportGripActive ?
                                "AUTHORED" : "DYNAMIC") :
                            "IDLE");
                    addTextLineSized(
                        labelAnchor + RE::NiPoint3{ 0.0f, 0.0f, -14.4f },
                        1.65f,
                        kCoordinateColor,
                        "authoredOnly capability=%s reason=%s ready=%.3fs selection=%s/%s",
                        authored_support_grab_policy::capabilityName(
                            snapshot.authoredCapability),
                        authored_support_grab_policy::capabilityReasonName(
                            snapshot.authoredCapabilityReason),
                        snapshot.authoredCapabilityReadySeconds,
                        authored_support_grab_policy::selectionName(
                            snapshot.lastSelection),
                        authored_support_grab_policy::selectionReasonName(
                            snapshot.lastSelectionReason));
                }
            }
        }

        if (drawAuthoredGripActivationZones) {
            FiringGripReattachZoneDebugSnapshot snapshot{};
            if (_twoHandedGrip.getFiringGripReattachZoneDebugSnapshot(snapshot)) {
                namespace reattach_zone = firing_grip_reattach_zone_policy;
                // YELLOW cross: the captured firing grip point where both
                // reattach cylinders start. BLUE crosses: each evaluated palm.
                addMarkerPoint(
                    debug::MarkerOverlayRole::AuthoredSupportGripPalmSeat,
                    snapshot.gripWorld,
                    3.0f);

                const float drawReachGameUnits = (std::min)(
                    snapshot.reachGameUnits,
                    12.0f);
                const float drawRadiusGameUnits =
                    snapshot.cylinderRadiusGameUnits;
                const auto tryBuildCylinderBasis = [](
                    const RE::NiPoint3& axis,
                    RE::NiPoint3& outUnitAxis,
                    RE::NiPoint3& outTangentA,
                    RE::NiPoint3& outTangentB) {
                    const auto normalize = [](
                        const RE::NiPoint3& value,
                        RE::NiPoint3& outUnit) {
                        const float lengthSquared =
                            value.x * value.x +
                            value.y * value.y +
                            value.z * value.z;
                        if (!std::isfinite(lengthSquared) ||
                            lengthSquared <= 0.000001f) {
                            return false;
                        }
                        const float inverseLength =
                            1.0f / std::sqrt(lengthSquared);
                        outUnit = RE::NiPoint3{
                            value.x * inverseLength,
                            value.y * inverseLength,
                            value.z * inverseLength,
                        };
                        return true;
                    };
                    const auto cross = [](
                        const RE::NiPoint3& a,
                        const RE::NiPoint3& b) {
                        return RE::NiPoint3{
                            a.y * b.z - a.z * b.y,
                            a.z * b.x - a.x * b.z,
                            a.x * b.y - a.y * b.x,
                        };
                    };
                    if (!normalize(axis, outUnitAxis)) {
                        return false;
                    }
                    const RE::NiPoint3 reference =
                        std::abs(outUnitAxis.z) < 0.9f ?
                        RE::NiPoint3{ 0.0f, 0.0f, 1.0f } :
                        RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
                    if (!normalize(cross(reference, outUnitAxis), outTangentA)) {
                        return false;
                    }
                    return normalize(cross(outUnitAxis, outTangentA), outTangentB);
                };
                RE::NiPoint3 unitAxis{};
                RE::NiPoint3 tangentA{};
                RE::NiPoint3 tangentB{};
                if (drawReachGameUnits > 0.0f &&
                    std::isfinite(drawRadiusGameUnits) &&
                    drawRadiusGameUnits > 0.0f &&
                    tryBuildCylinderBasis(
                        snapshot.weaponLeftAxisWorld,
                        unitAxis,
                        tangentA,
                        tangentB)) {
                    const auto cylinderPoint = [&](
                        const float axial,
                        const float radialA,
                        const float radialB) {
                        return RE::NiPoint3{
                            snapshot.gripWorld.x + unitAxis.x * axial +
                                tangentA.x * radialA + tangentB.x * radialB,
                            snapshot.gripWorld.y + unitAxis.y * axial +
                                tangentA.y * radialA + tangentB.y * radialB,
                            snapshot.gripWorld.z + unitAxis.z * axial +
                                tangentA.z * radialA + tangentB.z * radialB,
                        };
                    };
                    constexpr std::size_t SegmentCount = 12;
                    const auto drawRing = [&](const float axial) {
                        RE::NiPoint3 previous =
                            cylinderPoint(axial, drawRadiusGameUnits, 0.0f);
                        for (std::size_t segment = 1;
                             segment <= SegmentCount;
                             ++segment) {
                            const float angle =
                                static_cast<float>(segment) *
                                2.0f * std::numbers::pi_v<float> /
                                static_cast<float>(SegmentCount);
                            const RE::NiPoint3 point = cylinderPoint(
                                axial,
                                std::cos(angle) * drawRadiusGameUnits,
                                std::sin(angle) * drawRadiusGameUnits);
                            addMarkerLine(
                                debug::MarkerOverlayRole::AuthoredGripActivationAllowedRegion,
                                previous,
                                point);
                            previous = point;
                        }
                    };
                    // One cylinder per side of the weapon, starting on the
                    // grip point: axis line, far ring, and four spokes.
                    const auto drawWireCylinder = [&](const float axisSign) {
                        addMarkerLine(
                            debug::MarkerOverlayRole::AuthoredGripActivationSupportSideAxis,
                            snapshot.gripWorld,
                            cylinderPoint(axisSign * drawReachGameUnits, 0.0f, 0.0f));
                        drawRing(axisSign * drawReachGameUnits);
                        for (std::size_t spoke = 0; spoke < 4; ++spoke) {
                            const float angle =
                                static_cast<float>(spoke) *
                                0.5f * std::numbers::pi_v<float>;
                            const float radialA =
                                std::cos(angle) * drawRadiusGameUnits;
                            const float radialB =
                                std::sin(angle) * drawRadiusGameUnits;
                            addMarkerLine(
                                debug::MarkerOverlayRole::AuthoredGripActivationAllowedRegion,
                                cylinderPoint(0.0f, radialA, radialB),
                                cylinderPoint(
                                    axisSign * drawReachGameUnits,
                                    radialA,
                                    radialB));
                        }
                    };
                    drawRing(0.0f);
                    drawWireCylinder(1.0f);
                    drawWireCylinder(-1.0f);
                }

                constexpr float kSeatColor[4]{ 1.0f, 0.78f, 0.05f, 0.98f };
                constexpr float kPassColor[4]{ 0.25f, 1.0f, 0.12f, 0.98f };
                constexpr float kFailColor[4]{ 1.0f, 0.18f, 0.08f, 0.98f };
                // Stack upward from the grip so the readout clears the
                // authored support seat labels below it.
                const RE::NiPoint3 labelAnchor =
                    snapshot.gripWorld + RE::NiPoint3{ 0.0f, 0.0f, 6.0f };
                addTextLineSized(
                    labelAnchor,
                    2.1f,
                    kSeatColor,
                    "FIRING REATTACH ZONE reach=%.2f radius=%.2f x2",
                    snapshot.reachGameUnits,
                    snapshot.cylinderRadiusGameUnits);
                float labelOffset = 2.2f;
                for (std::size_t handIndex = 0;
                     handIndex < snapshot.hands.size();
                     ++handIndex) {
                    const auto& hand = snapshot.hands[handIndex];
                    if (!hand.evaluated) {
                        continue;
                    }
                    const auto verdictRole = hand.inside ?
                        debug::MarkerOverlayRole::AuthoredGripActivationPass :
                        debug::MarkerOverlayRole::AuthoredGripActivationFail;
                    addMarkerPoint(
                        debug::MarkerOverlayRole::AuthoredSupportGripLiveSample,
                        hand.palmWorld,
                        2.5f);
                    addMarkerLine(verdictRole, snapshot.gripWorld, hand.palmWorld);
                    addTextLineSized(
                        labelAnchor + RE::NiPoint3{ 0.0f, 0.0f, labelOffset },
                        1.65f,
                        hand.inside ? kPassColor : kFailColor,
                        "%s palm along=%.2f perp=%.2f d=%.2f side=%s reach=%d radius=%d grab=%s %s",
                        handIndex == 0 ? "LEFT" : "RIGHT",
                        hand.alongAxisGameUnits,
                        hand.perpendicularDistanceGameUnits,
                        hand.radialDistanceGameUnits,
                        reattach_zone::sideName(hand.side),
                        hand.reachPass ? 1 : 0,
                        hand.radiusPass ? 1 : 0,
                        hand.gripHeld ? "HELD" : "OPEN",
                        hand.inside ? "INSIDE" : "OUTSIDE");
                    labelOffset += 2.0f;
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
                        frik_visual_authority::
                            tryGetHandWorldTransform(
                                frik_visual_authority::Hand::Right,
                                appliedRight);
                    const bool leftAppliedValid =
                        frik_visual_authority::
                            tryGetHandWorldTransform(
                                frik_visual_authority::Hand::Left,
                                appliedLeft);
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

            auto addGrabAuthorityProxyBody = [&](const Hand& hand, const RE::NiTransform& rawHandWorld) {
                if ((hand.isLeft() && leftDisabled) || (!hand.isLeft() && rightDisabled)) {
                    return;
                }

                GrabAuthorityProxyDebugSnapshot snapshot{};
                if (!hand.getGrabAuthorityProxyDebugSnapshot(hknp, rawHandWorld, snapshot)) {
                    return;
                }

                const bool isLeft = hand.isLeft();
                const RE::hknpBodyId proxyBodyId = hand.getGrabAuthorityProxyBodyId();
                if (drawGrabAuthorityProxyCollider && proxyBodyId.value != INVALID_BODY_ID) {
                    addBodyWithTarget(
                        proxyBodyId,
                        isLeft ?
                            debug::BodyOverlayRole::LeftGrabAuthorityProxy :
                            debug::BodyOverlayRole::RightGrabAuthorityProxy,
                        drawColliderPhaseDiagnostics ? &snapshot.proxyTargetWorld : nullptr);
                }
                addAxisTransform(
                    snapshot.proxyTargetWorld,
                    isLeft ?
                        debug::AxisOverlayRole::LeftGrabAuthorityProxyTarget :
                        debug::AxisOverlayRole::RightGrabAuthorityProxyTarget,
                    snapshot.palmAuthorityBaseWorld.translate,
                    true);
                addMarkerPoint(
                    isLeft ?
                        debug::MarkerOverlayRole::LeftGrabAuthorityProxyTarget :
                        debug::MarkerOverlayRole::RightGrabAuthorityProxyTarget,
                    snapshot.proxyTargetWorld.translate,
                    4.0f);
                addMarkerLine(
                    isLeft ?
                        debug::MarkerOverlayRole::LeftGrabAuthorityProxyOffset :
                        debug::MarkerOverlayRole::RightGrabAuthorityProxyOffset,
                    snapshot.palmAuthorityBaseWorld.translate,
                    snapshot.proxyTargetWorld.translate);
            };

            addGrabAuthorityAxisReference(_rightHand, context.right.rawHandWorld);
            addGrabAuthorityAxisReference(_leftHand, context.left.rawHandWorld);

            addGrabAuthorityProxyBody(_rightHand, context.right.rawHandWorld);
            addGrabAuthorityProxyBody(_leftHand, context.left.rawHandWorld);
        }

        if (frame.drawRockBodies) {
            if (drawHandColliders) {
                const auto addPalmBody = [&](const Hand& hand, debug::BodyOverlayRole role) {
                    const auto bodyId = hand.getCollisionBodyId();
                    RE::NiTransform currentTarget{};
                    const bool hasCurrentTarget =
                        drawColliderPhaseDiagnostics &&
                        hand.tryGetHandColliderTargetForDebug(
                            bodyId.value,
                            currentTarget);
                    addBodyWithTarget(
                        bodyId,
                        role,
                        hasCurrentTarget ? &currentTarget : nullptr);
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
                    RE::NiTransform rightTarget{};
                    const bool hasRightTarget =
                        drawColliderPhaseDiagnostics &&
                        _dynamicHandCollision.tryGetBodyTargetForDebug(
                            false,
                            twinIndex,
                            rightTarget);
                    addBodyWithTarget(
                        _dynamicHandCollision.proxyBodyIdForDebug(
                            false,
                            twinIndex),
                        debug::BodyOverlayRole::RightHand,
                        hasRightTarget ? &rightTarget : nullptr);

                    RE::NiTransform leftTarget{};
                    const bool hasLeftTarget =
                        drawColliderPhaseDiagnostics &&
                        _dynamicHandCollision.tryGetBodyTargetForDebug(
                            true,
                            twinIndex,
                            leftTarget);
                    addBodyWithTarget(
                        _dynamicHandCollision.proxyBodyIdForDebug(
                            true,
                            twinIndex),
                        debug::BodyOverlayRole::LeftHand,
                        hasLeftTarget ? &leftTarget : nullptr);
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
                        const auto residualRole = handSample.isLeft ?
                            debug::MarkerOverlayRole::LeftDynamicHandSolverResidual :
                            debug::MarkerOverlayRole::RightDynamicHandSolverResidual;
                        for (const auto& twin : handSample.twins) {
                            if (!twin.physicsSampleValid || !twin.contactActive) {
                                continue;
                            }
                            addMarkerLine(requestedRole, twin.requestedTargetWorldGame, twin.liveBodyWorldGame);
                            addMarkerLine(residualRole, twin.commandedTargetWorldGame, twin.liveBodyWorldGame);
                            addTextLineSized(twin.liveBodyWorldGame,
                                1.45f,
                                handColor,
                                "%s %s RES=%.2f GAP=%.2f IKx=%.2f V=%.1f",
                                handSample.isLeft ? "L" : "R",
                                dynamic_hand_collision_telemetry::roleCode(twin.role),
                                twin.solverResidualGameUnits,
                                twin.requestedGapGameUnits,
                                twin.handTargetResponseScale,
                                twin.approachSpeedGameUnitsPerSecond);
                        }
                    }
                }
            }

            if (drawDynamicWeaponColliders) {
                RE::NiTransform currentTarget{};
                const bool hasCurrentTarget =
                    drawColliderPhaseDiagnostics &&
                    _dynamicWeaponCollision.tryGetContactBodyTargetForDebug(
                        currentTarget);
                addBodyWithTarget(
                    _dynamicWeaponCollision.proxyBodyIdForDebug(),
                    debug::BodyOverlayRole::DynamicWeaponProxy,
                    hasCurrentTarget ? &currentTarget : nullptr);

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
                        "gripPivot=%.2fgu yield=%.2fdeg layer=%u grace=%u visual=%s solve=%llu",
                        snapshot.translationCorrectionGameUnits,
                        snapshot.rotationCorrectionDegrees,
                        snapshot.otherLayer,
                        snapshot.contactGraceSolves,
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
                        RE::NiTransform currentTarget{};
                        const bool hasCurrentTarget =
                            drawColliderPhaseDiagnostics &&
                            hand.tryGetHandColliderTargetForDebug(
                                bodyId,
                                currentTarget);
                        addBodyWithTarget(
                            RE::hknpBodyId{ bodyId },
                            i == 0 ? anchorRole : segmentRole,
                            hasCurrentTarget ? &currentTarget : nullptr);
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
                        _bodyBoneColliders.tryGetBodyMetadataAtomic(bodyId, metadata) ? bodyOverlayRoleFor(metadata.role) : debug::BodyOverlayRole::BodyTorsoSegment;
                    RE::NiTransform currentTarget{};
                    const bool hasCurrentTarget =
                        drawColliderPhaseDiagnostics &&
                        _bodyBoneColliders.tryGetBodyTargetForDebug(
                            bodyId,
                            currentTarget);
                    addBodyWithTarget(
                        RE::hknpBodyId{ bodyId },
                        role,
                        hasCurrentTarget ? &currentTarget : nullptr);
                    ++bodyDrawn;
                }
            }

            for (std::uint32_t index = 0;
                 index < grabbedWeaponPartColliderCount;
                 ++index) {
                const std::uint32_t bodyId =
                    grabbedWeaponPartColliderBodyIds[index];
                RE::NiTransform currentTarget{};
                const bool hasCurrentTarget =
                    drawColliderPhaseDiagnostics &&
                    _weaponCollision.tryGetBodyTargetForDebug(
                        bodyId,
                        currentTarget);
                addBodyWithTarget(
                    RE::hknpBodyId{ bodyId },
                    debug::BodyOverlayRole::FocusedWeaponPart,
                    hasCurrentTarget ? &currentTarget : nullptr);
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
                    RE::NiTransform currentTarget{};
                    const bool hasCurrentTarget =
                        drawColliderPhaseDiagnostics &&
                        _weaponCollision.tryGetBodyTargetForDebug(
                            weaponSnapshot.bodyIds[i],
                            currentTarget);
                    addBodyWithTarget(
                        RE::hknpBodyId{ weaponSnapshot.bodyIds[i] },
                        debug::BodyOverlayRole::Weapon,
                        hasCurrentTarget ? &currentTarget : nullptr);
                }
            }
        }

        if (frame.drawTargetBodies || drawWorldOriginDiagnostics) {
            auto addHandTarget = [&](const Hand& hand) {
                if (hand.isHolding()) {
                    const auto& savedState = hand.getSavedObjectState();
                    if (frame.drawTargetBodies) {
                        RE::NiTransform desiredBodyWorld{};
                        const bool hasCurrentTarget =
                            drawColliderPhaseDiagnostics &&
                            hand.tryGetHeldDesiredBodyWorld(hknp, desiredBodyWorld);
                        addBodyWithTarget(
                            savedState.bodyId,
                            debug::BodyOverlayRole::Target,
                            hasCurrentTarget ? &desiredBodyWorld : nullptr);
                    }
                    addWorldOriginDiagnostic(hand, true, savedState.bodyId, savedState.refr, nullptr, nullptr);
                    return;
                }
                if (hand.hasSelection()) {
                    const auto& selection = hand.getSelection();
                    if (frame.drawTargetBodies) {
                        addBody(selection.bodyId, debug::BodyOverlayRole::Target);
                    }
                    addWorldOriginDiagnostic(hand, false, selection.bodyId, selection.refr, selection.hitNode, selection.visualNode);
                }
            };

            addHandTarget(_rightHand);
            addHandTarget(_leftHand);
        }

        debug::PublishFrame(frame);
    }
