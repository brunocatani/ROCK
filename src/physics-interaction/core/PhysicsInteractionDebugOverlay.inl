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
    void PhysicsInteraction::logGrabOverlayPointProbe(const PhysicsFrameContext& context)
    {
        if (!g_rockConfig.rockDebugGrabFrameLogging) {
            return;
        }

        auto* hknp = context.hknpWorld;
        if (!hknp) {
            return;
        }

        const bool probeRight = _rightHand.isHolding() && !context.right.disabled;
        const bool probeLeft = _leftHand.isHolding() && !context.left.disabled;
        if (!probeRight && !probeLeft) {
            return;
        }

        RE::NiPoint3 stereoOrigin{};
        const bool stereoOk = debug::TryGetCurrentStereoOrigin(stereoOrigin);
        const auto probeMicroseconds =
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

        // Per-hand previous-frame rotations for the rotation-step fields. The
        // 2026-07-13 post-phase-lock session proved the visible stutter is NOT
        // translation (obj-cam jitter 0.118gu with the percept unchanged);
        // rotation was never sampled at the point of visibility. Probe-only
        // state; consecutive-frame validity gated on the time gap.
        struct RotationStepState
        {
            RE::NiMatrix3 wand{};
            RE::NiMatrix3 target{};
            RE::NiMatrix3 object{};
            RE::NiMatrix3 handBody{};
            RE::NiMatrix3 renderNode{};
            long long microseconds = 0;
            bool valid = false;
            bool renderNodeValid = false;
        };
        static RotationStepState s_rotationStepState[2]{};

        auto logHand = [&](Hand& hand, const RE::NiTransform& rawHandWorld) {
            GrabOverlayPointProbeSample sample{};
            if (!hand.tryGetGrabOverlayPointProbeSample(hknp, sample)) {
                s_rotationStepState[hand.isLeft() ? 1 : 0].valid = false;
                return;
            }

            RE::NiTransform objectWorld{};
            RE::NiTransform handBodyWorld{};
            RE::NiTransform proxyWorld{};
            const bool objectOk = tryResolveLiveBodyWorldTransform(hknp, sample.objectBodyId, objectWorld);
            const bool handBodyOk = tryResolveLiveBodyWorldTransform(hknp, hand.getCollisionBodyId(), handBodyWorld);
            const bool proxyOk = tryResolveLiveBodyWorldTransform(hknp, sample.proxyBodyId, proxyWorld);

            // Per-frame rotation steps (degrees) of each trajectory. A steadily
            // carried object should step like the wand; motor wobble shows as
            // rotObj >> rotWand with rotTgt clean (downstream) or rotTgt dirty
            // (upstream target noise).
            auto& rotationState = s_rotationStepState[hand.isLeft() ? 1 : 0];
            float rotStepWand = -1.0f;
            float rotStepTarget = -1.0f;
            float rotStepObject = -1.0f;
            float rotStepHandBody = -1.0f;
            if (rotationState.valid && objectOk && handBodyOk &&
                (probeMicroseconds - rotationState.microseconds) < 50000) {
                rotStepWand = grab_authority_source_clock::rotationDeltaDegrees(rawHandWorld.rotate, rotationState.wand);
                rotStepTarget = grab_authority_source_clock::rotationDeltaDegrees(sample.appliedProxyTargetWorld.rotate, rotationState.target);
                rotStepObject = grab_authority_source_clock::rotationDeltaDegrees(objectWorld.rotate, rotationState.object);
                rotStepHandBody = grab_authority_source_clock::rotationDeltaDegrees(handBodyWorld.rotate, rotationState.handBody);
            }
            // RENDER-READ probe fields: the scene-graph world transform the
            // renderer consumes for the held object, paired with the post-solve
            // body transform read in the same instant. Every prior stutter probe
            // measured the drive/physics side only; whether the engine's actual
            // render read matches it was never verified (2026-07-14 attempt
            // history, open lead #2). Node resolution reuses the origin-
            // diagnostics visual-source chain; fails closed to nodeOk=n when the
            // body, refr, or node is unavailable. Runs before the shared
            // rotation-state update so the step gap check still sees the
            // previous frame's timestamp.
            RE::NiPoint3 renderNodePosition{};
            const char* renderNodeSource = "none";
            float renderNodeBodyDistance = -1.0f;
            float rotStepRenderNode = -1.0f;
            float rotRenderNodeVsBody = -1.0f;
            bool renderNodeOk = false;
            {
                origin_diagnostics::TargetOriginSample renderSample{};
                const auto& savedState = hand.getSavedObjectState();
                if (origin_diagnostics::sampleTarget(context.bhkWorld, hknp, savedState.bodyId, savedState.refr, nullptr, nullptr, 0.0f, renderSample) &&
                    renderSample.visualSourceNode) {
                    renderNodeOk = true;
                    renderNodePosition = renderSample.visualSourceNode->world.translate;
                    renderNodeSource = origin_diagnostics::visualSourceKindName(renderSample.visualSourceKind);
                    if (objectOk) {
                        renderNodeBodyDistance = origin_diagnostics::distance(renderNodePosition, objectWorld.translate);
                        rotRenderNodeVsBody =
                            grab_authority_source_clock::rotationDeltaDegrees(renderSample.visualSourceNode->world.rotate, objectWorld.rotate);
                    }
                    if (rotationState.renderNodeValid && (probeMicroseconds - rotationState.microseconds) < 50000) {
                        rotStepRenderNode =
                            grab_authority_source_clock::rotationDeltaDegrees(renderSample.visualSourceNode->world.rotate, rotationState.renderNode);
                    }
                    rotationState.renderNode = renderSample.visualSourceNode->world.rotate;
                }
                rotationState.renderNodeValid = renderNodeOk;
            }

            rotationState.wand = rawHandWorld.rotate;
            rotationState.target = sample.appliedProxyTargetWorld.rotate;
            rotationState.object = objectWorld.rotate;
            rotationState.handBody = handBodyWorld.rotate;
            rotationState.microseconds = probeMicroseconds;
            rotationState.valid = objectOk && handBodyOk;

            ROCK_LOG_DEBUG(Hand,
                "{} OVERLAY_POINT: t={}us flushSeq={} wand=({:.3f},{:.3f},{:.3f}) appliedWand=({:.3f},{:.3f},{:.3f}) tgt=({:.3f},{:.3f},{:.3f}) objOk={} obj=({:.3f},{:.3f},{:.3f}) handOk={} handBody=({:.3f},{:.3f},{:.3f}) proxyOk={} proxy=({:.3f},{:.3f},{:.3f}) camOk={} cam=({:.3f},{:.3f},{:.3f}) rotWand={:.3f} rotTgt={:.3f} rotObj={:.3f} rotHand={:.3f} jag={:.4f}",
                hand.handName(),
                probeMicroseconds,
                sample.flushSequence,
                rawHandWorld.translate.x,
                rawHandWorld.translate.y,
                rawHandWorld.translate.z,
                sample.appliedRawHandWorld.translate.x,
                sample.appliedRawHandWorld.translate.y,
                sample.appliedRawHandWorld.translate.z,
                sample.appliedProxyTargetWorld.translate.x,
                sample.appliedProxyTargetWorld.translate.y,
                sample.appliedProxyTargetWorld.translate.z,
                objectOk ? "y" : "n",
                objectWorld.translate.x,
                objectWorld.translate.y,
                objectWorld.translate.z,
                handBodyOk ? "y" : "n",
                handBodyWorld.translate.x,
                handBodyWorld.translate.y,
                handBodyWorld.translate.z,
                proxyOk ? "y" : "n",
                proxyWorld.translate.x,
                proxyWorld.translate.y,
                proxyWorld.translate.z,
                stereoOk ? "y" : "n",
                stereoOrigin.x,
                stereoOrigin.y,
                stereoOrigin.z,
                rotStepWand,
                rotStepTarget,
                rotStepObject,
                rotStepHandBody,
                sample.jagCorrectionGameUnits);

            ROCK_LOG_DEBUG(Hand,
                "{} RENDER_READ: t={}us flushSeq={} nodeOk={} node=({:.3f},{:.3f},{:.3f}) nodeSrc={} body=({:.3f},{:.3f},{:.3f}) d={:.3f} rotNode={:.3f} rotNB={:.3f}",
                hand.handName(),
                probeMicroseconds,
                sample.flushSequence,
                renderNodeOk ? "y" : "n",
                renderNodePosition.x,
                renderNodePosition.y,
                renderNodePosition.z,
                renderNodeSource,
                objectWorld.translate.x,
                objectWorld.translate.y,
                objectWorld.translate.z,
                renderNodeBodyDistance,
                rotStepRenderNode,
                rotRenderNodeVsBody);

            /*
             * Anchor decision data. The jag correction needs the room anchor
             * the CAMERA follows; the first session's controller anchor
             * delivered only 6-15% of the measured artifact, consistent with
             * its delta and v_room*dt sharing the physics clock and cancelling.
             * Both candidates are logged every frame beside the camera (already
             * in OVERLAY_POINT above), so differencing them offline names the
             * anchor whose per-frame step actually tracks the camera's
             * staircase -- regardless of which one drove the correction.
             */
            ROCK_LOG_DEBUG(Hand,
                "{} JAG_ANCHOR: t={}us flushSeq={} active={} actorOk={} actor=({:.4f},{:.4f},{:.4f}) ccOk={} cc=({:.4f},{:.4f},{:.4f}) corr={:.4f}",
                hand.handName(),
                probeMicroseconds,
                sample.flushSequence,
                g_rockConfig.rockGrabLocomotionJagAnchor == 1 ? "controller" : "actor",
                sample.jagActorAnchorValid ? "y" : "n",
                sample.jagActorAnchorGameUnits.x,
                sample.jagActorAnchorGameUnits.y,
                sample.jagActorAnchorGameUnits.z,
                sample.jagControllerAnchorValid ? "y" : "n",
                sample.jagControllerAnchorGameUnits.x,
                sample.jagControllerAnchorGameUnits.y,
                sample.jagControllerAnchorGameUnits.z,
                sample.jagCorrectionGameUnits);
        };

        if (probeRight) {
            logHand(_rightHand, context.right.rawHandWorld);
        }
        if (probeLeft) {
            logHand(_leftHand, context.left.rawHandWorld);
        }
    }

    void PhysicsInteraction::publishDebugBodyOverlay(const PhysicsFrameContext& context)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::DebugOverlayPublish);

        logGrabOverlayPointProbe(context);

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
        const bool drawRockColliderBodies = g_rockConfig.rockDebugShowColliders;
        const bool drawGrabPivots = g_rockConfig.rockDebugShowGrabPivots;
        const bool drawFingerProbes = g_rockConfig.rockDebugShowGrabFingerProbes;
        const bool drawFingerSweptArc = g_rockConfig.rockDebugShowGrabFingerSweptArc;
        const bool drawFingerSweptArcText = drawFingerSweptArc && g_rockConfig.rockDebugShowGrabFingerSweptArcText;
        const bool drawFingerSweptArcLiveSkeleton = drawFingerSweptArc && g_rockConfig.rockDebugShowGrabFingerSweptArcLiveSkeleton;
        const bool drawPalmVectors = g_rockConfig.rockDebugShowPalmVectors;
        const bool drawGrabPockets = g_rockConfig.rockDebugDrawGrabPockets;
        const bool drawRootFlattenedFingerSkeleton = g_rockConfig.rockDebugShowRootFlattenedFingerSkeletonMarkers;
        const auto skeletonBoneMode = skeleton_bone_debug_math::sanitizeDebugSkeletonBoneMode(g_rockConfig.rockDebugSkeletonBoneMode);
        const auto skeletonBoneSource = skeleton_bone_debug_math::sanitizeDebugSkeletonBoneSource(g_rockConfig.rockDebugSkeletonBoneSource);
        const bool drawSkeletonBones =
            g_rockConfig.rockDebugShowSkeletonBoneVisualizer && skeletonBoneMode != skeleton_bone_debug_math::DebugSkeletonBoneMode::Off;
        const bool drawGrabPocketNormal = g_rockConfig.rockDebugShowGrabPocketNormal;
        const bool drawGrabContactPatch = g_rockConfig.rockDebugDrawGrabContactPatch;
        const bool drawGrabForceTorque = g_rockConfig.rockDebugDrawGrabForceTorque;
        const bool drawGrabForceTorqueText = drawGrabForceTorque && g_rockConfig.rockDebugDrawGrabForceTorqueText;
        const bool drawGrabPivotSourceCollider = drawGrabForceTorque && g_rockConfig.rockDebugDrawGrabPivotSourceCollider;
        const bool drawGrabPivotSourceEvidence = drawGrabForceTorque && g_rockConfig.rockDebugDrawGrabPivotSourceEvidence;
        const bool drawGrabSupportFrame = g_rockConfig.rockDebugDrawGrabSupportFrame;
        const bool drawHandBoneContacts = g_rockConfig.rockDebugDrawHandBoneContacts;
        const bool drawGrabAuthorityProxy = g_rockConfig.rockDebugDrawGrabAuthorityProxy;
        const bool drawDynamicHandColliders = g_rockConfig.rockDebugDrawDynamicHandColliders;
        const bool drawGrabTransformTelemetry = g_rockConfig.rockDebugGrabTransformTelemetry;
        const bool drawGrabTransformTelemetryAxes = drawGrabTransformTelemetry && g_rockConfig.rockDebugGrabTransformTelemetryAxes;
        const bool drawGrabTransformTelemetryText = drawGrabTransformTelemetry && g_rockConfig.rockDebugGrabTransformTelemetryText;
        const bool drawPerformanceProfilerOverlay = performance_profiler::overlayTextEnabled();
        const bool drawVideoSyncMarker = g_rockConfig.rockDebugVideoSyncMarker;
        const bool drawWeaponAuthorityDebug = _twoHandedGrip.isGripping() && (g_rockConfig.rockDebugShowHandAxes || drawGrabPivots);
        // Authored support-grip diagnostics remain available through the
        // existing weapon-authority debug controls without coupling normal
        // production behavior to a debug-overlay writer.
        const bool drawAuthoredSupportGripDebug = drawWeaponAuthorityDebug;
        const bool drawNativeScopeActivation = g_rockConfig.rockDebugDrawNativeScopeActivation;
        const bool drawWorldOriginDiagnostics = g_rockConfig.rockDebugWorldObjectOriginDiagnostics;
        const bool drawCustomCalibrationOffset = g_rockConfig.rockDebugCustomCalibrationOffset;
        if (drawWorldOriginDiagnostics && !s_worldOriginDiagnosticsEnabledLogged) {
            ROCK_LOG_INFO(Hand,
                "World object origin diagnostics enabled: intervalFrames={} warnThresholdGameUnits={:.2f} visualSourceOrder=bodyOwnerNode>hitNode>visualNode>referenceRoot",
                g_rockConfig.rockDebugWorldObjectOriginLogIntervalFrames,
                g_rockConfig.rockDebugWorldObjectOriginMismatchWarnGameUnits);
            s_worldOriginDiagnosticsEnabledLogged = true;
        } else if (!drawWorldOriginDiagnostics) {
            s_worldOriginDiagnosticsEnabledLogged = false;
        }
        if (!drawRockColliderBodies && !g_rockConfig.rockDebugShowTargetColliders && !g_rockConfig.rockDebugShowHandAxes && !drawGrabPivots && !drawFingerProbes &&
            !drawFingerSweptArc && !drawPalmVectors && !drawGrabPockets && !drawRootFlattenedFingerSkeleton && !drawSkeletonBones && !drawGrabPocketNormal &&
            !drawGrabContactPatch && !drawHandBoneContacts && !drawGrabAuthorityProxy && !drawGrabForceTorque && !drawGrabTransformTelemetry && !drawPerformanceProfilerOverlay &&
            !drawWeaponAuthorityDebug && !drawNativeScopeActivation && !drawGrabSupportFrame && !drawWorldOriginDiagnostics && !drawCustomCalibrationOffset &&
            !drawDynamicHandColliders && !drawAuthoredSupportGripDebug && !drawProviderOverlay && !drawVideoSyncMarker) {
            debug::ClearFrame();
            return;
        }

        debug::Install();

        debug::BodyOverlayFrame frame{};
        frame.world = hknp;
        frame.drawRockBodies = drawRockColliderBodies || drawGrabAuthorityProxy || drawGrabPivotSourceCollider || drawDynamicHandColliders;
        frame.drawTargetBodies = g_rockConfig.rockDebugShowTargetColliders;
        frame.drawAxes = g_rockConfig.rockDebugShowHandAxes || drawGrabTransformTelemetryAxes || drawGrabAuthorityProxy || drawGrabForceTorque ||
            drawCustomCalibrationOffset || drawNativeScopeActivation ||
            drawAuthoredSupportGripDebug;
        frame.drawMarkers = drawGrabPivots || drawFingerProbes || drawFingerSweptArc || drawPalmVectors || drawGrabPockets || drawRootFlattenedFingerSkeleton ||
            drawGrabPocketNormal || drawGrabContactPatch || drawGrabForceTorque || drawHandBoneContacts || drawGrabAuthorityProxy || drawGrabTransformTelemetryAxes ||
            drawWeaponAuthorityDebug || drawNativeScopeActivation || drawGrabSupportFrame || drawWorldOriginDiagnostics || drawDynamicHandColliders ||
            drawAuthoredSupportGripDebug;
        frame.drawSkeleton = drawSkeletonBones;
        frame.drawColoredLines = providerOverlay && providerOverlay->lineCount > 0;
        frame.drawText = drawGrabTransformTelemetryText || drawGrabForceTorqueText || drawFingerSweptArcText || drawPerformanceProfilerOverlay ||
            drawDynamicHandColliders || drawNativeScopeActivation ||
            drawAuthoredSupportGripDebug || drawVideoSyncMarker ||
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

        auto addBody = [&](RE::hknpBodyId bodyId, debug::BodyOverlayRole role) {
            if (bodyId.value == INVALID_BODY_ID || frame.count >= frame.entries.size()) {
                return;
            }

            for (std::uint32_t i = 0; i < frame.count; i++) {
                if (frame.entries[i].bodyId.value == bodyId.value && frame.entries[i].role == role) {
                    return;
                }
            }

            frame.entries[frame.count++] = debug::BodyOverlayEntry{ bodyId, role };
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

            if (activationSnapshot.evaluationSequence == 0) {
                addScreenTextLine(panelX, panelY, panelColor, "cone: no valid resolved-anchor evaluation observed");
                panelY += 14.0f;
            } else {
                std::snprintf(panelLine, sizeof(panelLine), "cone seq=%llu state=%s native=%s ROCK=%s anchor=%s generation=%016llX",
                    static_cast<unsigned long long>(activationSnapshot.evaluationSequence), activationSnapshot.nativeScopeAlreadyActive ? "exit" : "enter",
                    activationSnapshot.nativeGeometryDecision ? "inside" : "outside", activationSnapshot.rockGeometryDecision ? "inside" : "outside",
                    scopeAnchorSourceName(activationSnapshot.anchorSource),
                    static_cast<unsigned long long>(activationSnapshot.weaponGenerationKey));
                addScreenTextLine(panelX, panelY, panelColor, panelLine);
                panelY += 14.0f;

                const auto& sample = activationSnapshot.sample;
                const auto& thresholds = activationSnapshot.thresholds;
                const float hmdLimit = activationSnapshot.nativeScopeAlreadyActive ? thresholds.hmdExitDegrees : thresholds.hmdEnterDegrees;
                const float weaponLimit = (activationSnapshot.nativeScopeAlreadyActive ? thresholds.weaponExitDegrees : thresholds.weaponEnterDegrees) * sample.weaponAngleWidening;
                const float distanceLimit = activationSnapshot.nativeScopeAlreadyActive ? thresholds.distanceExitGameUnits : thresholds.distanceEnterGameUnits;
                std::snprintf(panelLine, sizeof(panelLine), "sample: HMD=%.2f/%.2f deg weapon=%.2f/%.2f deg distance=%.2f/%.2f gu widen=%.3f", sample.hmdAngleDegrees, hmdLimit,
                    sample.weaponAngleDegrees, weaponLimit, sample.distanceGameUnits, distanceLimit, sample.weaponAngleWidening);
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
                if (g_rockConfig.rockDebugShowHandAxes) {
                    addAxisTransform(rawHand, debug::AxisOverlayRole::RightHandRaw, rawHand.translate, false);
                    addAxisBody(_rightHand.getCollisionBodyId(), debug::AxisOverlayRole::RightHandBody, rawHand.translate, true);
                }
                if (drawCustomCalibrationOffset && _handBoneCache.isReady()) {
                    custom_oga::Frame customOGA{};
                    const RE::NiTransform handBoneWorld = _handBoneCache.getWorldTransform(false);
                    if (custom_oga::resolveLive(false, handBoneWorld, rawHand, customOGA) && customOGA.valid) {
                        addAxisTransform(customOGA.world, debug::AxisOverlayRole::RightCustomCalibrationOffset, customOGA.world.translate, false);
                    }
                }
            }

            if (!leftDisabled) {
                const RE::NiTransform& rawHand = context.left.rawHandWorld;
                if (g_rockConfig.rockDebugShowHandAxes) {
                    addAxisTransform(rawHand, debug::AxisOverlayRole::LeftHandRaw, rawHand.translate, false);
                    addAxisBody(_leftHand.getCollisionBodyId(), debug::AxisOverlayRole::LeftHandBody, rawHand.translate, true);
                }
                if (drawCustomCalibrationOffset && _handBoneCache.isReady()) {
                    custom_oga::Frame customOGA{};
                    const RE::NiTransform handBoneWorld = _handBoneCache.getWorldTransform(true);
                    if (custom_oga::resolveLive(true, handBoneWorld, rawHand, customOGA) && customOGA.valid) {
                        addAxisTransform(customOGA.world, debug::AxisOverlayRole::LeftCustomCalibrationOffset, customOGA.world.translate, false);
                    }
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
                const float palmNormalLength = (std::max)(5.0f, g_rockConfig.rockNearDetectionRange);
                const float pointingLength = (std::min)(90.0f, (std::max)(20.0f, g_rockConfig.rockFarDetectionRange));

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
                    (std::max)(g_rockConfig.rockGrabPinchMaxPocketDistanceGameUnits, g_rockConfig.rockNearCastDistanceGameUnits);

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
                        g_rockConfig.rockDebugDrawSkeletonBoneAxes &&
                        skeleton_bone_debug_math::shouldDrawSkeletonAxis(g_rockConfig.rockDebugSkeletonAxisBoneFilter, snapshot.bones[i].name, axesDrawn, axisCap);
                    if (drawAxis) {
                        ++axesDrawn;
                    }
                    addSkeletonBone(snapshot, i, drawAxis);
                }

                if (skippedBones > 0 && g_rockConfig.rockDebugLogSkeletonBoneTruncation) {
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

                if (g_rockConfig.rockDebugLogSkeletonBones) {
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
                            rightHandScale = frik_visual_authority::getHandWorldTransform(frik_visual_authority::Hand::Right).scale;
                            leftHandScale = frik_visual_authority::getHandWorldTransform(frik_visual_authority::Hand::Left).scale;
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

        if (drawGrabPivots || drawGrabContactPatch) {
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

        if (drawGrabPocketNormal || drawGrabContactPatch) {
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

        if (drawGrabContactPatch) {
            auto addGrabContactPatchDebug = [&](const Hand& hand) {
                if ((hand.isLeft() && leftDisabled) || (!hand.isLeft() && rightDisabled)) {
                    return;
                }

                GrabContactPatchDebugSnapshot snapshot{};
                if (!hand.getGrabContactPatchDebugSnapshot(hknp, snapshot)) {
                    return;
                }

                const bool isLeft = hand.isLeft();
                const auto role = isLeft ? debug::MarkerOverlayRole::LeftGrabContactPatchSample : debug::MarkerOverlayRole::RightGrabContactPatchSample;
                for (std::uint32_t i = 0; i < snapshot.sampleCount; ++i) {
                    addMarkerPoint(role, snapshot.samplePointsWorld[i], 1.8f);
                }
            };

            addGrabContactPatchDebug(_rightHand);
            addGrabContactPatchDebug(_leftHand);
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
                    if (snapshot.hasContactPatchPoint) {
                        const auto contactRole =
                            isLeft ? debug::MarkerOverlayRole::LeftGrabPivotSourceContactPoint : debug::MarkerOverlayRole::RightGrabPivotSourceContactPoint;
                        addMarkerPoint(contactRole, snapshot.contactPatchPointWorld, 2.5f);
                        for (std::uint32_t i = 0; i < snapshot.contactSampleCount; ++i) {
                            addMarkerLine(contactRole, snapshot.contactPatchPointWorld, snapshot.contactSamplePointsWorld[i]);
                            addMarkerPoint(contactRole, snapshot.contactSamplePointsWorld[i], 1.4f);
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
                        "phase %s src %s body %u pbLock %.1f mut %.1f rq %u",
                        snapshot.acquisitionPhase,
                        snapshot.pivotAuthoritySource,
                        snapshot.pivotSourceBodyId.value,
                        snapshot.activePivotBVisualLockErrorGameUnits,
                        snapshot.captureGripLocalDeltaGameUnits,
                        snapshot.seatedPivotReacquireCount);
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
                            addTextLine(labelAnchor + RE::NiPoint3{ 0.0f, 0.0f, -24.5f },
                                detailColor,
                                "physA %.2fgu %.1fdeg relCon %.0f/%.0f raw %.0e %.0e %.0e",
                                snapshot.motorPhysicsProxyToLiveProxyDeltaGameUnits,
                                snapshot.motorPhysicsProxyToLiveProxyDeltaDegrees,
                                snapshot.motorRelationToConstraintATargetDegrees,
                                snapshot.motorRelationToConstraintALiveDegrees,
                                snapshot.motorTransformARawMaxDelta,
                                snapshot.motorTransformBRawMaxDelta,
                                snapshot.motorTargetBRcaRawMaxDelta);
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
                    "AUTHORED SUPPORT SEAT %s%s d=%.2f r=%.2f %s",
                    snapshot.supportHandIsLeft ? "LEFT" : "RIGHT",
                    snapshot.mirroredForRightSupport ? " MIRRORED" : "",
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
                    const RE::NiTransform appliedRight = frik_visual_authority::getHandWorldTransform(frik_visual_authority::Hand::Right);
                    const RE::NiTransform appliedLeft = frik_visual_authority::getHandWorldTransform(frik_visual_authority::Hand::Left);
                    addAxisTransform(appliedRight, debug::AxisOverlayRole::RightFrikAppliedHand, snapshot.rightRequestedHandWorld.translate, true);
                    addAxisTransform(appliedLeft, debug::AxisOverlayRole::LeftFrikAppliedHand, snapshot.leftRequestedHandWorld.translate, true);
                    addMarkerLine(debug::MarkerOverlayRole::RightWeaponAuthorityMismatch, snapshot.rightRequestedHandWorld.translate, appliedRight.translate);
                    addMarkerLine(debug::MarkerOverlayRole::LeftWeaponAuthorityMismatch, snapshot.leftRequestedHandWorld.translate, appliedLeft.translate);

                    static std::uint32_t authorityMismatchLogCounter = 0;
                    if (++authorityMismatchLogCounter >= 120) {
                        authorityMismatchLogCounter = 0;
                        ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip authority mismatch: right={:.2f}gu left={:.2f}gu",
                            pointDistance(snapshot.rightRequestedHandWorld.translate, appliedRight.translate),
                            pointDistance(snapshot.leftRequestedHandWorld.translate, appliedLeft.translate));
                    }
                }
            }
        }

        if (drawGrabPivots) {
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

        if (drawGrabTransformTelemetry) {
            struct GrabAngularDeltaLogValue
            {
                bool valid = false;
                float angleDegrees = 0.0f;
                RE::NiPoint3 worldDegrees{};
                RE::NiPoint3 handLocalDegrees{};
                RE::NiPoint3 hmdLocalDegrees{};
            };

            auto vectorDot = [](const RE::NiPoint3& a, const RE::NiPoint3& b) {
                return a.x * b.x + a.y * b.y + a.z * b.z;
            };
            auto vectorCross = [](const RE::NiPoint3& a, const RE::NiPoint3& b) {
                return RE::NiPoint3{
                    a.y * b.z - a.z * b.y,
                    a.z * b.x - a.x * b.z,
                    a.x * b.y - a.y * b.x,
                };
            };
            auto vectorLength = [&](const RE::NiPoint3& value) {
                return std::sqrt((std::max)(0.0f, vectorDot(value, value)));
            };
            auto normalizeOr = [&](const RE::NiPoint3& value, const RE::NiPoint3& fallback) {
                const float length = vectorLength(value);
                if (!std::isfinite(length) || length <= 0.000001f) {
                    return fallback;
                }
                const float inverseLength = 1.0f / length;
                return RE::NiPoint3{ value.x * inverseLength, value.y * inverseLength, value.z * inverseLength };
            };
            auto projectToBasis = [&](const RE::NiPoint3& worldVector, const grab_transform_telemetry::OrientationBasis& basis) {
                return RE::NiPoint3{
                    vectorDot(worldVector, basis.x),
                    vectorDot(worldVector, basis.y),
                    vectorDot(worldVector, basis.z),
                };
            };
            auto makePlanarHmdBasis = [&]() {
                grab_transform_telemetry::OrientationBasis basis{};
                const RE::NiPoint3 worldUp{ 0.0f, 0.0f, 1.0f };
                RE::NiPoint3 forward = context.hasHmdFrame ? context.hmdForwardWorld : RE::NiPoint3{ 0.0f, 1.0f, 0.0f };
                forward.z = 0.0f;
                forward = normalizeOr(forward, RE::NiPoint3{ 0.0f, 1.0f, 0.0f });
                const RE::NiPoint3 right = normalizeOr(vectorCross(forward, worldUp), RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
                basis.x = right;
                basis.y = forward;
                basis.z = worldUp;
                return basis;
            };
            auto computeAngularDeltaLogValue = [&](const RE::NiTransform& previous,
                                                   const RE::NiTransform& current,
                                                   const grab_transform_telemetry::OrientationBasis& currentHandBasis,
                                                   const grab_transform_telemetry::OrientationBasis& hmdBasis,
                                                   bool enabled) {
                GrabAngularDeltaLogValue result{};
                if (!enabled) {
                    return result;
                }

                const auto previousBasis = grab_transform_telemetry::makeOrientationBasis(previous);
                const auto currentBasis = grab_transform_telemetry::makeOrientationBasis(current);
                const float angleDegrees = grab_transform_telemetry::orientationBasisMaxDeltaDegrees(previousBasis, currentBasis);
                if (!std::isfinite(angleDegrees) || angleDegrees <= 0.0001f) {
                    result.valid = true;
                    return result;
                }

                RE::NiPoint3 axisSum{};
                axisSum = axisSum + vectorCross(previousBasis.x, currentBasis.x);
                axisSum = axisSum + vectorCross(previousBasis.y, currentBasis.y);
                axisSum = axisSum + vectorCross(previousBasis.z, currentBasis.z);
                const RE::NiPoint3 axis = normalizeOr(axisSum, RE::NiPoint3{ 0.0f, 0.0f, 0.0f });
                if (vectorLength(axis) <= 0.000001f) {
                    return result;
                }

                result.valid = true;
                result.angleDegrees = angleDegrees;
                result.worldDegrees = RE::NiPoint3{
                    axis.x * angleDegrees,
                    axis.y * angleDegrees,
                    axis.z * angleDegrees,
                };
                result.handLocalDegrees = projectToBasis(result.worldDegrees, currentHandBasis);
                result.hmdLocalDegrees = projectToBasis(result.worldDegrees, hmdBasis);
                return result;
            };
            auto storePreviousAngularDeltaSample = [](GrabTransformTelemetryState& telemetryState, const grab_transform_telemetry::RuntimeSample& sample) {
                telemetryState.previousRawHandWorld = sample.rawHandWorld;
                telemetryState.previousPalmAnchorGrabAuthorityWorld = sample.palmAnchorGrabAuthorityWorld;
                telemetryState.previousProxyReadbackWorld = sample.proxyReadbackWorld;
                telemetryState.previousRawDesiredObjectWorld = sample.currentRawDesiredObjectWorld;
                telemetryState.previousHeldNodeWorld = sample.heldNodeWorld;
                telemetryState.previousHeldBodyWorld = sample.heldBodyWorld;
                telemetryState.previousNativeBodyWorld = sample.heldNativeBodyWorld;
                telemetryState.previousHasPalmAnchorGrabAuthority = sample.hasPalmAnchorGrabAuthority;
                telemetryState.previousHasProxyReadback = sample.hasProxyReadback;
                telemetryState.previousHasHeldNodeWorld = sample.hasHeldNodeWorld;
                telemetryState.previousHasHeldBodyWorld = sample.hasHeldBodyWorld;
                telemetryState.previousHasHeldNativeBodyWorld = sample.hasHeldNativeBodyWorld;
                telemetryState.hasPreviousAngularDeltaSample = true;
            };

            auto publishGrabTelemetry = [&](Hand& hand, bool isLeft) {
                auto& telemetryState = _grabTransformTelemetryStates[isLeft ? 1 : 0];
                if (!hand.isHolding()) {
                    telemetryState.active = false;
                    telemetryState.frame = 0;
                    telemetryState.logFrameCounter = 0;
                    telemetryState.hasPreviousAngularDeltaSample = false;
                    return;
                }

                if (!telemetryState.active) {
                    telemetryState.active = true;
                    telemetryState.session = _grabTransformTelemetryNextSession++;
                    telemetryState.frame = 0;
                    telemetryState.logFrameCounter = 0;
                    telemetryState.hasPreviousAngularDeltaSample = false;
                }

                ++telemetryState.frame;
                const grab_transform_telemetry::FrameStamp stamp{
                    .session = telemetryState.session,
                    .frame = telemetryState.frame,
                };

                const RE::NiTransform rawHandWorld = isLeft ? context.left.rawHandWorld : context.right.rawHandWorld;

                grab_transform_telemetry::RuntimeSample sample{};
                if (!hand.getGrabTransformTelemetrySnapshot(
                        hknp,
                        rawHandWorld,
                        sample)) {
                    return;
                }

                const auto hmdBasis = makePlanarHmdBasis();
                const bool hasPreviousAngularDeltaSample = telemetryState.hasPreviousAngularDeltaSample;
                const auto rawHandAngularDelta = computeAngularDeltaLogValue(
                    telemetryState.previousRawHandWorld,
                    sample.rawHandWorld,
                    sample.rawHandBasis,
                    hmdBasis,
                    hasPreviousAngularDeltaSample);
                const auto palmAuthorityAngularDelta = computeAngularDeltaLogValue(
                    telemetryState.previousPalmAnchorGrabAuthorityWorld,
                    sample.palmAnchorGrabAuthorityWorld,
                    sample.rawHandBasis,
                    hmdBasis,
                    hasPreviousAngularDeltaSample &&
                        telemetryState.previousHasPalmAnchorGrabAuthority &&
                        sample.hasPalmAnchorGrabAuthority);
                const auto proxyAngularDelta = computeAngularDeltaLogValue(
                    telemetryState.previousProxyReadbackWorld,
                    sample.proxyReadbackWorld,
                    sample.rawHandBasis,
                    hmdBasis,
                    hasPreviousAngularDeltaSample &&
                        telemetryState.previousHasProxyReadback &&
                        sample.hasProxyReadback);
                const auto rawDesiredObjectAngularDelta = computeAngularDeltaLogValue(
                    telemetryState.previousRawDesiredObjectWorld,
                    sample.currentRawDesiredObjectWorld,
                    sample.rawHandBasis,
                    hmdBasis,
                    hasPreviousAngularDeltaSample &&
                        sample.hasGrabStartFrames);
                const auto heldNodeAngularDelta = computeAngularDeltaLogValue(
                    telemetryState.previousHeldNodeWorld,
                    sample.heldNodeWorld,
                    sample.rawHandBasis,
                    hmdBasis,
                    hasPreviousAngularDeltaSample &&
                        telemetryState.previousHasHeldNodeWorld &&
                        sample.hasHeldNodeWorld);
                const auto heldBodyAngularDelta = computeAngularDeltaLogValue(
                    telemetryState.previousHeldBodyWorld,
                    sample.heldBodyWorld,
                    sample.rawHandBasis,
                    hmdBasis,
                    hasPreviousAngularDeltaSample &&
                        telemetryState.previousHasHeldBodyWorld &&
                        sample.hasHeldBodyWorld);
                const auto nativeBodyAngularDelta = computeAngularDeltaLogValue(
                    telemetryState.previousNativeBodyWorld,
                    sample.heldNativeBodyWorld,
                    sample.rawHandBasis,
                    hmdBasis,
                    hasPreviousAngularDeltaSample &&
                        telemetryState.previousHasHeldNativeBodyWorld &&
                        sample.hasHeldNativeBodyWorld);

                if (drawGrabTransformTelemetryAxes) {
                    const auto palmDebugBasis = grab_transform_telemetry_overlay::buildHandAttachedTextBasis(sample.rawHandWorld, isLeft);
                    const RE::NiPoint3 palmReference =
                        sample.hasPalmAnchorTarget ? sample.palmAnchorTargetWorld.translate : sample.rawHandWorld.translate;
                    auto withOverlayOrigin = [](RE::NiTransform transform, const RE::NiPoint3& origin) {
                        transform.translate = origin;
                        return transform;
                    };

                    if (sample.hasPalmAnchorTarget) {
                        /*
                         * These palm triads intentionally keep the generated
                         * collider frame, grab-authority frame, and live proxy
                         * readback separate. The generated collider frame uses
                         * the in-game verified native placement convention; the
                         * grab-authority frame is the explicit adapter handed
                         * to proxy/body-A grab math.
                         */
                        addStoredColumnAxisTransform(
                            withOverlayOrigin(sample.palmAnchorTargetWorld, palmReference - palmDebugBasis.panelRight * 8.0f),
                            isLeft ? debug::AxisOverlayRole::LeftGrabPalmGeneratedDirect : debug::AxisOverlayRole::RightGrabPalmGeneratedDirect,
                            palmReference,
                            true);
                    }
                    if (sample.hasPalmAnchorGrabAuthority) {
                        addAxisTransform(
                            withOverlayOrigin(sample.palmAnchorGrabAuthorityWorld, palmReference),
                            isLeft ? debug::AxisOverlayRole::LeftGrabPalmAuthorityFrame : debug::AxisOverlayRole::RightGrabPalmAuthorityFrame,
                            palmReference,
                            false);
                    }
                    if (sample.hasProxyReadback) {
                        addAxisTransform(
                            withOverlayOrigin(sample.proxyReadbackWorld, palmReference + palmDebugBasis.panelRight * 8.0f),
                            isLeft ? debug::AxisOverlayRole::LeftGrabProxyReadback : debug::AxisOverlayRole::RightGrabProxyReadback,
                            palmReference,
                            true);
                    }
                    if (sample.hasGrabStartFrames) {
                        addAxisTransform(sample.currentRawDesiredObjectWorld,
                            isLeft ? debug::AxisOverlayRole::LeftGrabDesiredObject : debug::AxisOverlayRole::RightGrabDesiredObject,
                            sample.rawHandWorld.translate,
                            true);
                        addAxisTransform(sample.heldNodeWorld,
                            isLeft ? debug::AxisOverlayRole::LeftGrabHeldNode : debug::AxisOverlayRole::RightGrabHeldNode,
                            sample.rawHandWorld.translate,
                            true);
                        addMarkerLine(isLeft ? debug::MarkerOverlayRole::LeftGrabHeldDesiredError : debug::MarkerOverlayRole::RightGrabHeldDesiredError,
                            sample.currentRawDesiredObjectWorld.translate,
                            sample.heldNodeWorld.translate);
                    }
                    if (sample.hasHeldRelativeHandTarget) {
                        addAxisTransform(sample.heldRelativeHandTargetWorld,
                            isLeft ? debug::AxisOverlayRole::LeftGrabHeldRelativeHandTarget : debug::AxisOverlayRole::RightGrabHeldRelativeHandTarget,
                            sample.rawHandWorld.translate,
                            true);
                        addMarkerLine(isLeft ? debug::MarkerOverlayRole::LeftGrabHeldRelativeHandTargetError : debug::MarkerOverlayRole::RightGrabHeldRelativeHandTargetError,
                            sample.rawHandWorld.translate,
                            sample.heldRelativeHandTargetWorld.translate);
                    }
                    addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabPivotA : debug::MarkerOverlayRole::RightGrabPivotA, sample.pivotAWorld, 3.0f);
                    addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabPivotB : debug::MarkerOverlayRole::RightGrabPivotB, sample.pivotBWorld, 3.0f);
                    addMarkerLine(isLeft ? debug::MarkerOverlayRole::LeftGrabPivotError : debug::MarkerOverlayRole::RightGrabPivotError, sample.pivotAWorld, sample.pivotBWorld);
                }

                if (drawGrabTransformTelemetryText) {
                    const float color[4] = { isLeft ? 1.0f : 0.55f, isLeft ? 0.55f : 1.0f, isLeft ? 0.95f : 1.0f, 0.94f };
                    const auto textBasis = grab_transform_telemetry_overlay::buildHandAttachedTextBasis(sample.rawHandWorld, isLeft);
                    const auto labelRole = isLeft ? debug::MarkerOverlayRole::LeftGrabTelemetryLabelAnchor : debug::MarkerOverlayRole::RightGrabTelemetryLabelAnchor;
                    addMarkerPoint(labelRole, textBasis.anchor, 2.6f);
                    addMarkerLine(labelRole, textBasis.hand, textBasis.anchor);

                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 0),
                        color,
                        "GTEL S%u F%llu H%s BODY%u HB%s OB%s",
                        stamp.session,
                        static_cast<unsigned long long>(stamp.frame),
                        grab_transform_telemetry::handLabel(isLeft),
                        sample.heldBodyId,
                        body_frame::bodyFrameSourceCode(sample.handBodySource),
                        body_frame::bodyFrameSourceCode(sample.heldBodySource));
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 1),
                        color,
                        "LIVE_HAND %.1f %.1f %.1f",
                        sample.rawHandWorld.translate.x,
                        sample.rawHandWorld.translate.y,
                        sample.rawHandWorld.translate.z);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 2),
                        color,
                        "PALM_BODY %s %.1f %.1f %.1f D %.2f %.2f",
                        body_frame::bodyFrameSourceCode(sample.handBodySource),
                        sample.handBodyWorld.translate.x,
                        sample.handBodyWorld.translate.y,
                        sample.handBodyWorld.translate.z,
                        sample.rawToHandBody.positionGameUnits,
                        sample.rawToHandBody.rotationDegrees);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 3),
                        color,
                        "VIS_NODE %.1f %.1f %.1f BODYNODE %.1f %.1f %.1f D %.2f %.2f",
                        sample.heldNodeWorld.translate.x,
                        sample.heldNodeWorld.translate.y,
                        sample.heldNodeWorld.translate.z,
                        sample.heldBodyDerivedNodeWorld.translate.x,
                        sample.heldBodyDerivedNodeWorld.translate.y,
                        sample.heldBodyDerivedNodeWorld.translate.z,
                        sample.bodyDerivedNodeToHeldNode.positionGameUnits,
                        sample.bodyDerivedNodeToHeldNode.rotationDegrees);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 4),
                        color,
                        "PIVERR NTV %.2f A %.1f %.1f %.1f B %.1f %.1f %.1f",
                        sample.pivotErrorGameUnits,
                        sample.pivotAWorld.x,
                        sample.pivotAWorld.y,
                        sample.pivotAWorld.z,
                        sample.pivotBWorld.x,
                        sample.pivotBWorld.y,
                        sample.pivotBWorld.z);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 5),
                        color,
                        "RAW_DESN %.1f %.1f %.1f D %.2f %.2f",
                        sample.currentRawDesiredObjectWorld.translate.x,
                        sample.currentRawDesiredObjectWorld.translate.y,
                        sample.currentRawDesiredObjectWorld.translate.z,
                        sample.heldNodeToRawDesiredObject.positionGameUnits,
                        sample.heldNodeToRawDesiredObject.rotationDegrees);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 6),
                        color,
                        "RAW_DESB %.1f %.1f %.1f D %.2f %.2f",
                        sample.currentRawDesiredBodyWorld.translate.x,
                        sample.currentRawDesiredBodyWorld.translate.y,
                        sample.currentRawDesiredBodyWorld.translate.z,
                        sample.heldBodyToRawDesiredBody.positionGameUnits,
                        sample.heldBodyToRawDesiredBody.rotationDegrees);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 7),
                        color,
                        "RAWHS_REV %.1f %.1f %.1f D %.2f %.2f",
                        sample.heldRelativeHandTargetWorld.translate.x,
                        sample.heldRelativeHandTargetWorld.translate.y,
                        sample.heldRelativeHandTargetWorld.translate.z,
                        sample.rawToHeldRelativeHandTarget.positionGameUnits,
                        sample.rawToHeldRelativeHandTarget.rotationDegrees);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 8),
                        color,
                        "BODY_ERR %.2f %.2f REL %.2f",
                        sample.bodyTargetNodeErr.positionGameUnits,
                        sample.bodyTargetNodeErr.rotationDegrees,
                        sample.relationPivotErr);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 9),
                        color,
                        "TBLOC %.1f %.1f %.1f DES %.1f %.1f %.1f E %.2f",
                        sample.constraintTransformBLocalGame.x,
                        sample.constraintTransformBLocalGame.y,
                        sample.constraintTransformBLocalGame.z,
                        sample.desiredTransformBLocalGame.x,
                        sample.desiredTransformBLocalGame.y,
                        sample.desiredTransformBLocalGame.z,
                        sample.transformBLocalDelta.distance);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 10),
                        color,
                        "ANGT CI %.2f HIG %.2f CF %.2f TBF %.2f EN%d",
                        sample.targetColumnsToConstraintInverseDegrees,
                        sample.targetToHiggsRelationDegrees,
                        sample.targetColumnsToConstraintForwardDegrees,
                        sample.transformBFrozenDeltaDegrees,
                        sample.ragdollMotorEnabled ? 1 : 0);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 11),
                        color,
                        "MOTOR AT %.3f AD %.2f AF %.0f LT %.3f LF %.0f M %.2f",
                        sample.angularMotorTau,
                        sample.angularMotorDamping,
                        sample.angularMotorMaxForce,
                        sample.linearMotorTau,
                        sample.linearMotorMaxForce,
                        sample.heldBodyMass);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 12),
                        color,
                        "AXIS RAW %.2f %.2f %.2f",
                        sample.rawToHeldRelativeHandTargetAxes.x,
                        sample.rawToHeldRelativeHandTargetAxes.y,
                        sample.rawToHeldRelativeHandTargetAxes.z);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 13),
                        color,
                        "NATIVE flat->palm %.2fgu %.2fdeg legacy->palm %.2f",
                        sample.nativeFlattenedHandToPalmAnchorTarget.positionGameUnits,
                        sample.nativeFlattenedHandToPalmAnchorTarget.rotationDegrees,
                        sample.legacyPalmPivotAToPalmAnchor.distance);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 14),
                        color,
                        "AUTH palm->auth %.2fgu %.2fdeg auth->proxy %.2fgu %.2fdeg",
                        sample.palmAnchorTargetToGrabAuthority.positionGameUnits,
                        sample.palmAnchorTargetToGrabAuthority.rotationDegrees,
                        sample.grabAuthorityToProxyReadback.positionGameUnits,
                        sample.grabAuthorityToProxyReadback.rotationDegrees);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 15),
                        color,
                        "LEGACY cfg->runtime %.2f cfg->proxy %.2f proxy%d",
                        sample.legacyPalmPivotAToRuntimePivotA.distance,
                        sample.legacyPalmPivotAToProxyReadback.distance,
                        sample.hasProxyReadback ? 1 : 0);
                }

                ++telemetryState.logFrameCounter;
                const std::uint64_t logInterval =
                    static_cast<std::uint64_t>((std::max)(1, g_rockConfig.rockDebugGrabTransformTelemetryLogIntervalFrames));
                if (telemetryState.frame == 1 || telemetryState.logFrameCounter >= logInterval) {
                    telemetryState.logFrameCounter = 0;
                    const auto prefix = grab_transform_telemetry::formatStampPrefix(stamp, isLeft, telemetryState.frame == 1 ? "start" : "held");
                    const char* phaseLabel = telemetryState.frame == 1 ? "START" : "HELD";
                    ROCK_LOG_INFO(Hand,
                        "GRAB TELEMETRY {} {} formID={:08X} heldBody={} handBody={} heldSource={} handSource={} heldMotion={} handMotion={} pivotFrame=NATIVE_BODY pivotA=({:.2f},{:.2f},{:.2f}) pivotB=({:.2f},{:.2f},{:.2f}) pivotErr={:.3f}",
                        prefix,
                        phaseLabel,
                        sample.heldFormId,
                        sample.heldBodyId,
                        sample.handBodyId,
                        body_frame::bodyFrameSourceCode(sample.heldBodySource),
                        body_frame::bodyFrameSourceCode(sample.handBodySource),
                        sample.heldMotionIndex,
                        sample.handMotionIndex,
                        sample.pivotAWorld.x,
                        sample.pivotAWorld.y,
                        sample.pivotAWorld.z,
                        sample.pivotBWorld.x,
                        sample.pivotBWorld.y,
                        sample.pivotBWorld.z,
                        sample.pivotErrorGameUnits);
                    ROCK_LOG_INFO(Hand,
                        "GRAB TELEMETRY {} {} liveHand=({:.2f},{:.2f},{:.2f}) handBody[{}]=({:.2f},{:.2f},{:.2f}) heldBody[{}]=({:.2f},{:.2f},{:.2f}) nativeBody[BODY]=({:.2f},{:.2f},{:.2f}) nativeToHeld={:.3f}gu/{:.3f}deg visualNode=({:.2f},{:.2f},{:.2f}) bodyDerivedNode=({:.2f},{:.2f},{:.2f}) bodyNodeToVisual={:.3f}gu/{:.3f}deg liveToHandBody={:.3f}gu/{:.3f}deg",
                        prefix,
                        phaseLabel,
                        sample.rawHandWorld.translate.x,
                        sample.rawHandWorld.translate.y,
                        sample.rawHandWorld.translate.z,
                        body_frame::bodyFrameSourceCode(sample.handBodySource),
                        sample.handBodyWorld.translate.x,
                        sample.handBodyWorld.translate.y,
                        sample.handBodyWorld.translate.z,
                        body_frame::bodyFrameSourceCode(sample.heldBodySource),
                        sample.heldBodyWorld.translate.x,
                        sample.heldBodyWorld.translate.y,
                        sample.heldBodyWorld.translate.z,
                        sample.heldNativeBodyWorld.translate.x,
                        sample.heldNativeBodyWorld.translate.y,
                        sample.heldNativeBodyWorld.translate.z,
                        sample.heldNativeBodyToHeldBody.positionGameUnits,
                        sample.heldNativeBodyToHeldBody.rotationDegrees,
                        sample.heldNodeWorld.translate.x,
                        sample.heldNodeWorld.translate.y,
                        sample.heldNodeWorld.translate.z,
                        sample.heldBodyDerivedNodeWorld.translate.x,
                        sample.heldBodyDerivedNodeWorld.translate.y,
                        sample.heldBodyDerivedNodeWorld.translate.z,
                        sample.bodyDerivedNodeToHeldNode.positionGameUnits,
                        sample.bodyDerivedNodeToHeldNode.rotationDegrees,
                        sample.rawToHandBody.positionGameUnits,
                        sample.rawToHandBody.rotationDegrees);
                    ROCK_LOG_INFO(Hand,
                        "GRAB TELEMETRY {} {} startLive=({:.2f},{:.2f},{:.2f}) startHandBody=({:.2f},{:.2f},{:.2f}) objectAtGrab=({:.2f},{:.2f},{:.2f}) desiredAtGrab=({:.2f},{:.2f},{:.2f}) startDrift={:.3f}gu/{:.3f}deg rawHS=({:.2f},{:.2f},{:.2f}) bodyRaw=({:.2f},{:.2f},{:.2f}) bodyLocal=({:.2f},{:.2f},{:.2f})",
                        prefix,
                        phaseLabel,
                        sample.liveHandWorldAtGrab.translate.x,
                        sample.liveHandWorldAtGrab.translate.y,
                        sample.liveHandWorldAtGrab.translate.z,
                        sample.handBodyWorldAtGrab.translate.x,
                        sample.handBodyWorldAtGrab.translate.y,
                        sample.handBodyWorldAtGrab.translate.z,
                        sample.objectNodeWorldAtGrab.translate.x,
                        sample.objectNodeWorldAtGrab.translate.y,
                        sample.objectNodeWorldAtGrab.translate.z,
                        sample.desiredObjectWorldAtGrab.translate.x,
                        sample.desiredObjectWorldAtGrab.translate.y,
                        sample.desiredObjectWorldAtGrab.translate.z,
                        sample.heldNodeToDesiredObjectAtGrab.positionGameUnits,
                        sample.heldNodeToDesiredObjectAtGrab.rotationDegrees,
                        sample.rawHandSpace.translate.x,
                        sample.rawHandSpace.translate.y,
                        sample.rawHandSpace.translate.z,
                        sample.handBodyToRawHandAtGrab.translate.x,
                        sample.handBodyToRawHandAtGrab.translate.y,
                        sample.handBodyToRawHandAtGrab.translate.z,
                        sample.bodyLocal.translate.x,
                        sample.bodyLocal.translate.y,
                        sample.bodyLocal.translate.z);
                    ROCK_LOG_INFO(Hand,
                        "GRAB TELEMETRY {} {} rawDesiredNode=({:.2f},{:.2f},{:.2f}) heldToRawDesiredNode={:.3f}gu/{:.3f}deg rawDesiredBody=({:.2f},{:.2f},{:.2f}) heldToRawDesiredBody={:.3f}gu/{:.3f}deg",
                        prefix,
                        phaseLabel,
                        sample.currentRawDesiredObjectWorld.translate.x,
                        sample.currentRawDesiredObjectWorld.translate.y,
                        sample.currentRawDesiredObjectWorld.translate.z,
                        sample.heldNodeToRawDesiredObject.positionGameUnits,
                        sample.heldNodeToRawDesiredObject.rotationDegrees,
                        sample.currentRawDesiredBodyWorld.translate.x,
                        sample.currentRawDesiredBodyWorld.translate.y,
                        sample.currentRawDesiredBodyWorld.translate.z,
                        sample.heldBodyToRawDesiredBody.positionGameUnits,
                        sample.heldBodyToRawDesiredBody.rotationDegrees);
                    ROCK_LOG_INFO(Hand,
                        "GRAB TELEMETRY {} {} invariants relationPivotErr={:.3f}gu rotationPreservedDeg={:.3f} bodyTargetNodeErr={:.3f}gu/{:.3f}deg normalAuthority={} authoredRotation={}",
                        prefix,
                        phaseLabel,
                        sample.relationPivotErr,
                        sample.rotationPreservedDeg,
                        sample.bodyTargetNodeErr.positionGameUnits,
                        sample.bodyTargetNodeErr.rotationDegrees,
                        sample.normalAuthority ? "true" : "false",
                        sample.authoredRotationAuthority ? "true" : "false");
                    ROCK_LOG_INFO(Hand,
                        "GRAB TELEMETRY {} {} rawHSRev=({:.2f},{:.2f},{:.2f}) liveToRawHSRev={:.3f}gu/{:.3f}deg",
                        prefix,
                        phaseLabel,
                        sample.heldRelativeHandTargetWorld.translate.x,
                        sample.heldRelativeHandTargetWorld.translate.y,
                        sample.heldRelativeHandTargetWorld.translate.z,
                        sample.rawToHeldRelativeHandTarget.positionGameUnits,
                        sample.rawToHeldRelativeHandTarget.rotationDegrees);
                    ROCK_LOG_INFO(Hand,
                        "GRAB TELEMETRY {} {} axisDots rawHSRev=({:.3f},{:.3f},{:.3f})",
                        prefix,
                        phaseLabel,
                        sample.rawToHeldRelativeHandTargetAxes.x,
                        sample.rawToHeldRelativeHandTargetAxes.y,
                        sample.rawToHeldRelativeHandTargetAxes.z);
                    const char* sideLabel = isLeft ? "left" : "right";
                    ROCK_LOG_INFO(Hand,
                        "GRAB BASIS {} {} side={} convention=niLocalVectorToWorld nativeFlattenedHand={} generatedPalm={} rootFinger={} {} {} rootBase={} rootTip={} rootPalmLine={} handBodyPalmLine={} generatedPalmLine={} rootOpenLine={} rootPalmNormal={} nativeToPalm={:.3f}gu/{:.3f}deg",
                        prefix,
                        phaseLabel,
                        sideLabel,
                        "yes",
                        sample.hasPalmAnchorTarget ? "yes" : "no",
                        sample.hasRootFingerLandmarks ? "yes" : "no",
                        grab_transform_telemetry::formatBasis("nativeFlattenedHand", sample.nativeFlattenedHandBasis),
                        grab_transform_telemetry::formatBasis("generatedPalm", sample.palmAnchorTargetBasis),
                        grab_transform_telemetry::formatVector3(sample.rootFingerBaseCenterWorld),
                        grab_transform_telemetry::formatVector3(sample.rootFingerTipCenterWorld),
                        grab_transform_telemetry::formatVector3(sample.rootFingerBaseLineWorld),
                        grab_transform_telemetry::formatVector3(sample.handBodyFingerBaseLineWorld),
                        grab_transform_telemetry::formatVector3(sample.palmAnchorFingerBaseLineWorld),
                        grab_transform_telemetry::formatVector3(sample.rootFingerOpenLineWorld),
                        grab_transform_telemetry::formatVector3(sample.rootPalmNormalWorld),
                        sample.nativeFlattenedHandToPalmAnchorTarget.positionGameUnits,
                        sample.nativeFlattenedHandToPalmAnchorTarget.rotationDegrees);
                    ROCK_LOG_INFO(Hand,
                        "GRAB BASIS FRAMECHAIN {} {} side={} generatedPalm={} grabAuthority={} proxyReadback={} nativeToPalm={:.3f}gu/{:.3f}deg nativeToAuthority={:.3f}gu/{:.3f}deg nativeToProxy={:.3f}gu/{:.3f}deg palmToAuthority={:.3f}gu/{:.3f}deg authorityToProxy={:.3f}gu/{:.3f}deg {} {} {}",
                        prefix,
                        phaseLabel,
                        sideLabel,
                        sample.hasPalmAnchorTarget ? "yes" : "no",
                        sample.hasPalmAnchorGrabAuthority ? "yes" : "no",
                        sample.hasProxyReadback ? "yes" : "no",
                        sample.nativeFlattenedHandToPalmAnchorTarget.positionGameUnits,
                        sample.nativeFlattenedHandToPalmAnchorTarget.rotationDegrees,
                        sample.nativeFlattenedHandToGrabAuthority.positionGameUnits,
                        sample.nativeFlattenedHandToGrabAuthority.rotationDegrees,
                        sample.nativeFlattenedHandToProxyReadback.positionGameUnits,
                        sample.nativeFlattenedHandToProxyReadback.rotationDegrees,
                        sample.palmAnchorTargetToGrabAuthority.positionGameUnits,
                        sample.palmAnchorTargetToGrabAuthority.rotationDegrees,
                        sample.grabAuthorityToProxyReadback.positionGameUnits,
                        sample.grabAuthorityToProxyReadback.rotationDegrees,
                        grab_transform_telemetry::formatBasis("generatedPalm", sample.palmAnchorTargetBasis),
                        grab_transform_telemetry::formatBasis("grabAuthority", sample.palmAnchorGrabAuthorityBasis),
                        grab_transform_telemetry::formatBasis("proxyReadback", sample.proxyReadbackBasis));
                    ROCK_LOG_INFO(Hand,
                        "GRAB BASIS LEGACY_PIVOT {} {} side={} legacyPresent={} legacyActive=no activeSource={} legacyPivotA={} runtimePivotA={} generatedPalm={} grabAuthority={} proxyReadback={} legacyToRuntime={:.3f}gu legacyToPalm={:.3f}gu legacyToAuthority={:.3f}gu legacyToProxy={:.3f}gu",
                        prefix,
                        phaseLabel,
                        sideLabel,
                        sample.hasLegacyPalmPivotAWorld ? "yes" : "no",
                        sample.runtimePivotSource,
                        grab_transform_telemetry::formatVector3(sample.legacyPalmPivotAWorld),
                        grab_transform_telemetry::formatVector3(sample.pivotAWorld),
                        sample.hasPalmAnchorTarget ? "yes" : "no",
                        sample.hasPalmAnchorGrabAuthority ? "yes" : "no",
                        sample.hasProxyReadback ? "yes" : "no",
                        sample.legacyPalmPivotAToRuntimePivotA.distance,
                        sample.legacyPalmPivotAToPalmAnchor.distance,
                        sample.legacyPalmPivotAToGrabAuthority.distance,
                        sample.legacyPalmPivotAToProxyReadback.distance);
                    ROCK_LOG_INFO(Hand,
                        "GRAB ANGULAR_DELTA {} {} side={} prev={} hmd={} raw={:.3f}deg world={} hand={} hmdLocal={} authority={:.3f}deg world={} hand={} hmdLocal={} proxy={:.3f}deg world={} hand={} hmdLocal={} rawDesired={:.3f}deg world={} hand={} hmdLocal={} heldNode={:.3f}deg world={} hand={} hmdLocal={} heldBody={:.3f}deg world={} hand={} hmdLocal={} nativeBody={:.3f}deg world={} hand={} hmdLocal={}",
                        prefix,
                        phaseLabel,
                        sideLabel,
                        hasPreviousAngularDeltaSample ? "yes" : "no",
                        context.hasHmdFrame ? "yes" : "no",
                        rawHandAngularDelta.valid ? rawHandAngularDelta.angleDegrees : -1.0f,
                        grab_transform_telemetry::formatVector3(rawHandAngularDelta.worldDegrees),
                        grab_transform_telemetry::formatVector3(rawHandAngularDelta.handLocalDegrees),
                        grab_transform_telemetry::formatVector3(rawHandAngularDelta.hmdLocalDegrees),
                        palmAuthorityAngularDelta.valid ? palmAuthorityAngularDelta.angleDegrees : -1.0f,
                        grab_transform_telemetry::formatVector3(palmAuthorityAngularDelta.worldDegrees),
                        grab_transform_telemetry::formatVector3(palmAuthorityAngularDelta.handLocalDegrees),
                        grab_transform_telemetry::formatVector3(palmAuthorityAngularDelta.hmdLocalDegrees),
                        proxyAngularDelta.valid ? proxyAngularDelta.angleDegrees : -1.0f,
                        grab_transform_telemetry::formatVector3(proxyAngularDelta.worldDegrees),
                        grab_transform_telemetry::formatVector3(proxyAngularDelta.handLocalDegrees),
                        grab_transform_telemetry::formatVector3(proxyAngularDelta.hmdLocalDegrees),
                        rawDesiredObjectAngularDelta.valid ? rawDesiredObjectAngularDelta.angleDegrees : -1.0f,
                        grab_transform_telemetry::formatVector3(rawDesiredObjectAngularDelta.worldDegrees),
                        grab_transform_telemetry::formatVector3(rawDesiredObjectAngularDelta.handLocalDegrees),
                        grab_transform_telemetry::formatVector3(rawDesiredObjectAngularDelta.hmdLocalDegrees),
                        heldNodeAngularDelta.valid ? heldNodeAngularDelta.angleDegrees : -1.0f,
                        grab_transform_telemetry::formatVector3(heldNodeAngularDelta.worldDegrees),
                        grab_transform_telemetry::formatVector3(heldNodeAngularDelta.handLocalDegrees),
                        grab_transform_telemetry::formatVector3(heldNodeAngularDelta.hmdLocalDegrees),
                        heldBodyAngularDelta.valid ? heldBodyAngularDelta.angleDegrees : -1.0f,
                        grab_transform_telemetry::formatVector3(heldBodyAngularDelta.worldDegrees),
                        grab_transform_telemetry::formatVector3(heldBodyAngularDelta.handLocalDegrees),
                        grab_transform_telemetry::formatVector3(heldBodyAngularDelta.hmdLocalDegrees),
                        nativeBodyAngularDelta.valid ? nativeBodyAngularDelta.angleDegrees : -1.0f,
                        grab_transform_telemetry::formatVector3(nativeBodyAngularDelta.worldDegrees),
                        grab_transform_telemetry::formatVector3(nativeBodyAngularDelta.handLocalDegrees),
                        grab_transform_telemetry::formatVector3(nativeBodyAngularDelta.hmdLocalDegrees));
                    ROCK_LOG_INFO(Hand,
                        "GRAB BASIS {} {} side={} objectFrames {} {} {} {} {} {}",
                        prefix,
                        phaseLabel,
                        sideLabel,
                        grab_transform_telemetry::formatBasis("objectAtGrab", sample.objectNodeWorldAtGrabBasis),
                        grab_transform_telemetry::formatBasis("desiredAtGrab", sample.desiredObjectWorldAtGrabBasis),
                        grab_transform_telemetry::formatBasis("heldNode", sample.heldNodeBasis),
                        grab_transform_telemetry::formatBasis("bodyDerivedNode", sample.heldBodyDerivedNodeBasis),
                        grab_transform_telemetry::formatBasis("heldBody", sample.heldBodyBasis),
                        grab_transform_telemetry::formatBasis("nativeBody", sample.heldNativeBodyBasis));
                    ROCK_LOG_INFO(Hand,
                        "GRAB BASIS {} {} side={} desiredFrames {} {} {}",
                        prefix,
                        phaseLabel,
                        sideLabel,
                        grab_transform_telemetry::formatBasis("rawDesiredObj", sample.currentRawDesiredObjectWorldBasis),
                        grab_transform_telemetry::formatBasis("rawDesiredBody", sample.currentRawDesiredBodyWorldBasis),
                        grab_transform_telemetry::formatBasis("rawHSReverseHand", sample.heldRelativeHandTargetBasis));
                    ROCK_LOG_INFO(Hand,
                        "GRAB BASIS DELTA {} {} side={} {} {} {} {} {} {} {} {}",
                        prefix,
                        phaseLabel,
                        sideLabel,
                        grab_transform_telemetry::formatBasisDelta("rawToPalmAnchor", sample.rawHandBasis, sample.palmAnchorTargetBasis),
                        grab_transform_telemetry::formatBasisDelta("rawToHandBody", sample.rawHandBasis, sample.handBodyBasis),
                        grab_transform_telemetry::formatBasisDelta("objectAtGrabToDesiredAtGrab", sample.objectNodeWorldAtGrabBasis, sample.desiredObjectWorldAtGrabBasis),
                        grab_transform_telemetry::formatBasisDelta("heldNodeToObjectAtGrab", sample.heldNodeBasis, sample.objectNodeWorldAtGrabBasis),
                        grab_transform_telemetry::formatBasisDelta("heldNodeToRawDesired", sample.heldNodeBasis, sample.currentRawDesiredObjectWorldBasis),
                        grab_transform_telemetry::formatBasisDelta("heldBodyToRawDesiredBody", sample.heldBodyBasis, sample.currentRawDesiredBodyWorldBasis),
                        grab_transform_telemetry::formatBasisDelta("nativeBodyToHeldBody", sample.heldNativeBodyBasis, sample.heldBodyBasis),
                        grab_transform_telemetry::formatBasisDelta("rawHandToHeldRelativeHand", sample.rawHandBasis, sample.heldRelativeHandTargetBasis));
                    const bool hasAuthorityAxisBasis = sample.hasProxyReadback || sample.hasPalmAnchorGrabAuthority;
                    if (hasAuthorityAxisBasis && sample.hasHeldBodyWorld && sample.hasGrabStartFrames) {
                        const auto& authorityBasis = sample.hasProxyReadback ? sample.proxyReadbackBasis : sample.palmAnchorGrabAuthorityBasis;
                        ROCK_LOG_INFO(Hand,
                            "GRAB BASIS AXISMAP {} {} side={} authority={} {} {} {} {}",
                            prefix,
                            phaseLabel,
                            sideLabel,
                            sample.hasProxyReadback ? "proxyReadback" : "grabAuthority",
                            grab_transform_telemetry::formatBasisCrossMap("heldBodyToAuthority", sample.heldBodyBasis, authorityBasis),
                            grab_transform_telemetry::formatBasisCrossMap("desiredBodyToAuthority", sample.currentRawDesiredBodyWorldBasis, authorityBasis),
                            grab_transform_telemetry::formatBasisCrossMap("heldBodyToDesiredBody", sample.heldBodyBasis, sample.currentRawDesiredBodyWorldBasis),
                            grab_transform_telemetry::formatBasisCrossMap("rawHandToAuthority", sample.rawHandBasis, authorityBasis));
                    }
                    if (sample.hasConstraintAngularTelemetry) {
                        ROCK_LOG_INFO(Hand,
                            "GRAB TELEMETRY {} {} transformBLocal=({:.2f},{:.2f},{:.2f}) desiredTransformBLocal=({:.2f},{:.2f},{:.2f}) pivotBRelationDelta={:.3f}gu targetErr(colsInv={:.3f}deg targetToHiggsRelation={:.3f}deg colsForward={:.3f}deg transformBFrozenDelta={:.3f}deg) ragEnabled={} angTau={:.3f} angDamping={:.3f} angForce={:.1f} linTau={:.3f} linForce={:.1f} mass={:.3f}",
                            prefix,
                            phaseLabel,
                            sample.constraintTransformBLocalGame.x,
                            sample.constraintTransformBLocalGame.y,
                            sample.constraintTransformBLocalGame.z,
                            sample.desiredTransformBLocalGame.x,
                            sample.desiredTransformBLocalGame.y,
                            sample.desiredTransformBLocalGame.z,
                            sample.transformBLocalDelta.distance,
                            sample.targetColumnsToConstraintInverseDegrees,
                            sample.targetToHiggsRelationDegrees,
                            sample.targetColumnsToConstraintForwardDegrees,
                            sample.transformBFrozenDeltaDegrees,
                            sample.ragdollMotorEnabled ? "yes" : "no",
                            sample.angularMotorTau,
                            sample.angularMotorDamping,
                            sample.angularMotorMaxForce,
                            sample.linearMotorTau,
                            sample.linearMotorMaxForce,
                            sample.heldBodyMass);
                    }
                }
                storePreviousAngularDeltaSample(telemetryState, sample);
            };

            publishGrabTelemetry(_rightHand, false);
            publishGrabTelemetry(_leftHand, true);
        } else {
            for (auto& state : _grabTransformTelemetryStates) {
                state.active = false;
                state.frame = 0;
                state.logFrameCounter = 0;
            }
        }

        if (frame.drawRockBodies) {
            if (drawGrabAuthorityProxy) {
                auto addGrabAuthorityAxisReference = [&](const Hand& hand, const RE::NiTransform& rawHandWorld) {
                    if ((hand.isLeft() && leftDisabled) || (!hand.isLeft() && rightDisabled)) {
                        return;
                    }

                    const bool isLeft = hand.isLeft();
                    if (!g_rockConfig.rockDebugShowHandAxes) {
                        addAxisTransform(rawHandWorld, isLeft ? debug::AxisOverlayRole::LeftHandRaw : debug::AxisOverlayRole::RightHandRaw, rawHandWorld.translate, false);
                    }

                    RE::NiTransform palmAnchorTarget{};
                    if (hand.tryGetPalmAnchorTarget(palmAnchorTarget)) {
                        addStoredColumnAxisTransform(
                            palmAnchorTarget,
                            isLeft ? debug::AxisOverlayRole::LeftGrabPalmGeneratedDirect : debug::AxisOverlayRole::RightGrabPalmGeneratedDirect,
                            rawHandWorld.translate,
                            true);
                    }
                };

                auto addGrabAuthorityProxyTarget = [&](const Hand& hand, const RE::NiTransform& rawHandWorld) {
                    if ((hand.isLeft() && leftDisabled) || (!hand.isLeft() && rightDisabled)) {
                        return;
                    }

                    GrabAuthorityProxyDebugSnapshot snapshot{};
                    if (!hand.getGrabAuthorityProxyDebugSnapshot(hknp, rawHandWorld, snapshot)) {
                        return;
                    }

                    const bool isLeft = hand.isLeft();
                    addAxisTransform(snapshot.proxyTargetWorld,
                        isLeft ? debug::AxisOverlayRole::LeftGrabAuthorityProxyTarget : debug::AxisOverlayRole::RightGrabAuthorityProxyTarget,
                        snapshot.palmAuthorityBaseWorld.translate,
                        true);
                    addMarkerPoint(
                        isLeft ? debug::MarkerOverlayRole::LeftGrabAuthorityProxyTarget : debug::MarkerOverlayRole::RightGrabAuthorityProxyTarget,
                        snapshot.proxyTargetWorld.translate,
                        3.4f);
                    addMarkerLine(
                        isLeft ? debug::MarkerOverlayRole::LeftGrabAuthorityProxyOffset : debug::MarkerOverlayRole::RightGrabAuthorityProxyOffset,
                        snapshot.palmAuthorityBaseWorld.translate,
                        snapshot.proxyTargetWorld.translate);

                };

                addGrabAuthorityAxisReference(_rightHand, context.right.rawHandWorld);
                addGrabAuthorityAxisReference(_leftHand, context.left.rawHandWorld);

                const RE::hknpBodyId rightPalm = _rightHand.getCollisionBodyId();
                const RE::hknpBodyId leftPalm = _leftHand.getCollisionBodyId();
                if (rightPalm.value != INVALID_BODY_ID) {
                    addBody(rightPalm, debug::BodyOverlayRole::RightHand);
                    addAxisBody(rightPalm, debug::AxisOverlayRole::RightGrabPalmAuthorityFrame, context.right.rawHandWorld.translate, true);
                }
                if (leftPalm.value != INVALID_BODY_ID) {
                    addBody(leftPalm, debug::BodyOverlayRole::LeftHand);
                    addAxisBody(leftPalm, debug::AxisOverlayRole::LeftGrabPalmAuthorityFrame, context.left.rawHandWorld.translate, true);
                }

                const RE::hknpBodyId rightProxy = _rightHand.getGrabAuthorityProxyBodyId();
                const RE::hknpBodyId leftProxy = _leftHand.getGrabAuthorityProxyBodyId();
                if (rightProxy.value != INVALID_BODY_ID) {
                    addBody(rightProxy, debug::BodyOverlayRole::RightGrabAuthorityProxy);
                    addAxisBody(rightProxy, debug::AxisOverlayRole::RightGrabProxyReadback, context.right.rawHandWorld.translate, true);
                }
                if (leftProxy.value != INVALID_BODY_ID) {
                    addBody(leftProxy, debug::BodyOverlayRole::LeftGrabAuthorityProxy);
                    addAxisBody(leftProxy, debug::AxisOverlayRole::LeftGrabProxyReadback, context.left.rawHandWorld.translate, true);
                }

                addGrabAuthorityProxyTarget(_rightHand, context.right.rawHandWorld);
                addGrabAuthorityProxyTarget(_leftHand, context.left.rawHandWorld);
            }

            if (debug_overlay_policy::shouldDrawHandBody(drawRockColliderBodies, g_rockConfig.rockDebugDrawHandColliders) &&
                !g_rockConfig.rockDebugDrawHandBoneColliders) {
                addBody(_rightHand.getCollisionBodyId(), debug::BodyOverlayRole::RightHand);
                addBody(_leftHand.getCollisionBodyId(), debug::BodyOverlayRole::LeftHand);
            }

            /*
             * The dynamic hand/forearm twins are not part of any collider set,
             * so the overlay enumerates them explicitly behind their OWN flag:
             * bDebugDrawDynamicHandColliders shows just this arm-authority set without the
             * full keyframed collider soup. Watching a twin stop while the raw
             * hand axes keep moving is the primary in-game validation view.
             */
            if (drawDynamicHandColliders) {
                for (std::size_t twinIndex = 0; twinIndex < DynamicHandCollisionRuntime::kBodiesPerHand; ++twinIndex) {
                    addBody(_dynamicHandCollision.proxyBodyIdForDebug(false, twinIndex), debug::BodyOverlayRole::RightHand);
                    addBody(_dynamicHandCollision.proxyBodyIdForDebug(true, twinIndex), debug::BodyOverlayRole::LeftHand);
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

            if (debug_overlay_policy::shouldDrawHandBody(drawRockColliderBodies, g_rockConfig.rockDebugDrawHandBoneColliders)) {
                const std::uint32_t cap = static_cast<std::uint32_t>((std::clamp)(g_rockConfig.rockDebugMaxHandBoneBodiesDrawn, 0, 48));
                std::uint32_t drawn = 0;
                auto addHandBoneBodies = [&](const Hand& hand, debug::BodyOverlayRole anchorRole, debug::BodyOverlayRole segmentRole) {
                    const std::uint32_t count = hand.getHandColliderBodyCount();
                    for (std::uint32_t i = 0; i < count && drawn < cap; ++i) {
                        const std::uint32_t bodyId = hand.getHandColliderBodyIdAtomic(i);
                        if (bodyId == INVALID_BODY_ID) {
                            continue;
                        }
                        addBody(RE::hknpBodyId{ bodyId }, i == 0 ? anchorRole : segmentRole);
                        ++drawn;
                    }
                };

                addHandBoneBodies(_rightHand, debug::BodyOverlayRole::RightHand, debug::BodyOverlayRole::RightHandSegment);
                addHandBoneBodies(_leftHand, debug::BodyOverlayRole::LeftHand, debug::BodyOverlayRole::LeftHandSegment);

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
                    addBody(RE::hknpBodyId{ bodyId }, role);
                    ++bodyDrawn;
                }
            }

            const auto weaponSnapshot = _weaponCollision.getWeaponBodySnapshotAtomic();
            for (std::uint32_t i = 0; i < weaponSnapshot.count; i++) {
                const bool drawNormalWeaponBody =
                    debug_overlay_policy::shouldDrawWeaponBody(drawRockColliderBodies, g_rockConfig.rockDebugDrawWeaponColliders, i, g_rockConfig.rockDebugMaxWeaponBodiesDrawn);
                if (drawNormalWeaponBody) {
                    addBody(RE::hknpBodyId{ weaponSnapshot.bodyIds[i] }, debug::BodyOverlayRole::Weapon);
                }
            }
        }

        if (frame.drawTargetBodies || drawWorldOriginDiagnostics) {
            auto addHandTarget = [&](const Hand& hand) {
                const auto& handInput = hand.isLeft() ? context.left : context.right;
                if (hand.isHolding()) {
                    const auto& savedState = hand.getSavedObjectState();
                    if (frame.drawTargetBodies) {
                        addBody(savedState.bodyId, debug::BodyOverlayRole::Target);
                        addAxisBody(savedState.bodyId, debug::AxisOverlayRole::TargetBody, handInput.rawHandWorld.translate, true);
                    }
                    addWorldOriginDiagnostic(hand, true, savedState.bodyId, savedState.refr, nullptr, nullptr);
                    return;
                }
                if (hand.hasSelection()) {
                    const auto& selection = hand.getSelection();
                    if (frame.drawTargetBodies) {
                        addBody(selection.bodyId, debug::BodyOverlayRole::Target);
                        addAxisBody(selection.bodyId, debug::AxisOverlayRole::TargetBody, handInput.rawHandWorld.translate, true);
                    }
                    addWorldOriginDiagnostic(hand, false, selection.bodyId, selection.refr, selection.hitNode, selection.visualNode);
                }
            };

            addHandTarget(_rightHand);
            addHandTarget(_leftHand);
        }

        debug::PublishFrame(frame);
    }
