/*
 * Debug overlay publishing is split from the runtime frame loop because it is diagnostic fan-out over many subsystems, not interaction authority. The fragment remains in this translation unit so existing helper visibility and behavior stay unchanged.
 */
    void PhysicsInteraction::publishDebugBodyOverlay(const PhysicsFrameContext& context)
    {
        auto* hknp = context.hknpWorld;
        const bool drawRockColliderBodies = g_rockConfig.rockDebugShowColliders;
        const bool drawGrabPivots = g_rockConfig.rockDebugShowGrabPivots;
        const bool drawFingerProbes = g_rockConfig.rockDebugShowGrabFingerProbes;
        const bool drawPalmVectors = g_rockConfig.rockDebugShowPalmVectors;
        const bool drawRootFlattenedFingerSkeleton = g_rockConfig.rockDebugShowRootFlattenedFingerSkeletonMarkers;
        const auto skeletonBoneMode = skeleton_bone_debug_math::sanitizeDebugSkeletonBoneMode(g_rockConfig.rockDebugSkeletonBoneMode);
        const auto skeletonBoneSource = skeleton_bone_debug_math::sanitizeDebugSkeletonBoneSource(g_rockConfig.rockDebugSkeletonBoneSource);
        const bool drawSkeletonBones =
            g_rockConfig.rockDebugShowSkeletonBoneVisualizer && skeletonBoneMode != skeleton_bone_debug_math::DebugSkeletonBoneMode::Off;
        const bool drawGrabSurfaceFrame = g_rockConfig.rockDebugShowGrabSurfaceFrame;
        const bool drawGrabContactPatch = g_rockConfig.rockDebugDrawGrabContactPatch;
        const bool drawHandBoneContacts = g_rockConfig.rockDebugDrawHandBoneContacts;
        const bool drawGrabTransformTelemetry = g_rockConfig.rockDebugGrabTransformTelemetry;
        const bool drawGrabTransformTelemetryAxes = drawGrabTransformTelemetry && g_rockConfig.rockDebugGrabTransformTelemetryAxes;
        const bool drawGrabTransformTelemetryText = drawGrabTransformTelemetry && g_rockConfig.rockDebugGrabTransformTelemetryText;
        const bool drawWeaponAuthorityDebug = _twoHandedGrip.isGripping() && (g_rockConfig.rockDebugShowHandAxes || drawGrabPivots);
        const bool drawWorldOriginDiagnostics = g_rockConfig.rockDebugWorldObjectOriginDiagnostics;
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
            !drawPalmVectors && !drawRootFlattenedFingerSkeleton && !drawSkeletonBones && !drawGrabSurfaceFrame && !drawGrabContactPatch && !drawHandBoneContacts &&
            !drawGrabTransformTelemetry && !drawWeaponAuthorityDebug && !drawWorldOriginDiagnostics) {
            debug::ClearFrame();
            return;
        }

        debug::Install();

        debug::BodyOverlayFrame frame{};
        frame.world = hknp;
        frame.drawRockBodies = drawRockColliderBodies;
        frame.drawTargetBodies = g_rockConfig.rockDebugShowTargetColliders;
        frame.drawAxes = g_rockConfig.rockDebugShowHandAxes || drawGrabTransformTelemetryAxes;
        frame.drawMarkers =
            drawGrabPivots || drawFingerProbes || drawPalmVectors || drawRootFlattenedFingerSkeleton || drawGrabSurfaceFrame || drawGrabContactPatch ||
            drawHandBoneContacts || drawGrabTransformTelemetryAxes || drawWeaponAuthorityDebug || drawWorldOriginDiagnostics;
        frame.drawSkeleton = drawSkeletonBones;
        frame.drawText = drawGrabTransformTelemetryText;
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

        auto addAxisTransform = [&](const RE::NiTransform& transform, debug::AxisOverlayRole role, const RE::NiPoint3& translationStart, bool drawTranslationLine) {
            if (!frame.drawAxes || frame.axisCount >= frame.axisEntries.size()) {
                return;
            }

            auto& entry = frame.axisEntries[frame.axisCount++];
            entry.source = debug::AxisOverlaySource::Transform;
            entry.role = role;
            entry.transform = transform;
            entry.translationStart = translationStart;
            entry.drawTranslationLine = drawTranslationLine;
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

        auto addTextLine = [&](const RE::NiPoint3& worldAnchor, const float color[4], const char* format, auto&&... args) {
            if (!frame.drawText || frame.textCount >= frame.textEntries.size()) {
                return;
            }

            auto& entry = frame.textEntries[frame.textCount++];
            entry.x = 20.0f;
            entry.y = 0.0f;
            entry.size = 3.0f;
            entry.color[0] = color[0];
            entry.color[1] = color[1];
            entry.color[2] = color[2];
            entry.color[3] = color[3];
            entry.worldAnchor = worldAnchor;
            entry.worldAnchored = true;
            std::snprintf(entry.text, sizeof(entry.text), format, std::forward<decltype(args)>(args)...);
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

        if (frame.drawAxes) {
            if (!rightDisabled) {
                const RE::NiTransform& rawHand = context.right.rawHandWorld;
                addAxisTransform(rawHand, debug::AxisOverlayRole::RightHandRaw, rawHand.translate, false);
                addAxisBody(_rightHand.getCollisionBodyId(), debug::AxisOverlayRole::RightHandBody, rawHand.translate, true);
            }

            if (!leftDisabled) {
                const RE::NiTransform& rawHand = context.left.rawHandWorld;
                addAxisTransform(rawHand, debug::AxisOverlayRole::LeftHandRaw, rawHand.translate, false);
                addAxisBody(_leftHand.getCollisionBodyId(), debug::AxisOverlayRole::LeftHandBody, rawHand.translate, true);
            }
        }

        if (drawPalmVectors) {
            auto addPalmVectorDebug = [&](bool isLeft) {
                if ((isLeft && leftDisabled) || (!isLeft && rightDisabled)) {
                    return;
                }

                const auto& handInput = isLeft ? context.left : context.right;
                const RE::NiPoint3 grabAnchor = handInput.grabAnchorWorld;
                const RE::NiPoint3 palmNormal = handInput.palmNormalWorld;
                const RE::NiPoint3 pointing = handInput.pointingWorld;
                const float palmNormalLength = (std::max)(5.0f, g_rockConfig.rockNearDetectionRange);
                const float pointingLength = (std::min)(90.0f, (std::max)(20.0f, g_rockConfig.rockFarDetectionRange));

                addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabAnchor : debug::MarkerOverlayRole::RightGrabAnchor, grabAnchor, 2.0f);
                addMarkerRay(isLeft ? debug::MarkerOverlayRole::LeftPalmNormal : debug::MarkerOverlayRole::RightPalmNormal, grabAnchor,
                    grabAnchor + palmNormal * palmNormalLength, 1.6f);
                addMarkerRay(isLeft ? debug::MarkerOverlayRole::LeftPointing : debug::MarkerOverlayRole::RightPointing, grabAnchor,
                    grabAnchor + pointing * pointingLength, 1.2f);
            };

            addPalmVectorDebug(false);
            addPalmVectorDebug(true);
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
                        if (frik::api::FRIKApi::inst) {
                            rightHandScale = frik::api::FRIKApi::inst->getHandWorldTransform(frik::api::FRIKApi::Hand::Right).scale;
                            leftHandScale = frik::api::FRIKApi::inst->getHandWorldTransform(frik::api::FRIKApi::Hand::Left).scale;
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

        if (drawGrabSurfaceFrame || drawGrabContactPatch) {
            auto addGrabSurfaceDebug = [&](const Hand& hand) {
                if ((hand.isLeft() && leftDisabled) || (!hand.isLeft() && rightDisabled)) {
                    return;
                }

                GrabSurfaceFrameDebugSnapshot snapshot{};
                if (!hand.getGrabSurfaceFrameDebugSnapshot(hknp, snapshot)) {
                    return;
                }

                const bool isLeft = hand.isLeft();
                addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabSurfacePoint : debug::MarkerOverlayRole::RightGrabSurfacePoint, snapshot.contactPointWorld, 2.4f);
                addMarkerRay(isLeft ? debug::MarkerOverlayRole::LeftGrabSurfaceNormal : debug::MarkerOverlayRole::RightGrabSurfaceNormal, snapshot.contactPointWorld,
                    snapshot.normalEndWorld, 1.4f);
                if (snapshot.hasTangent) {
                    addMarkerRay(isLeft ? debug::MarkerOverlayRole::LeftGrabSurfaceTangent : debug::MarkerOverlayRole::RightGrabSurfaceTangent, snapshot.contactPointWorld,
                        snapshot.tangentEndWorld, 1.0f);
                }
                if (snapshot.hasBitangent) {
                    addMarkerRay(isLeft ? debug::MarkerOverlayRole::LeftGrabSurfaceBitangent : debug::MarkerOverlayRole::RightGrabSurfaceBitangent, snapshot.contactPointWorld,
                        snapshot.bitangentEndWorld, 1.0f);
                }
            };

            addGrabSurfaceDebug(_rightHand);
            addGrabSurfaceDebug(_leftHand);
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

        if (drawWeaponAuthorityDebug) {
            TwoHandedGripDebugSnapshot snapshot{};
            if (_twoHandedGrip.getDebugAuthoritySnapshot(snapshot)) {
                addAxisTransform(snapshot.weaponWorld, debug::AxisOverlayRole::WeaponAuthority, snapshot.weaponWorld.translate, false);
                addAxisTransform(snapshot.rightRequestedHandWorld, debug::AxisOverlayRole::RightWeaponPrimaryGrip, snapshot.rightGripWorld, true);
                addAxisTransform(snapshot.leftRequestedHandWorld, debug::AxisOverlayRole::LeftWeaponSupportGrip, snapshot.leftGripWorld, true);
                addMarkerPoint(debug::MarkerOverlayRole::RightWeaponPrimaryGrip, snapshot.rightGripWorld, 3.0f);
                addMarkerPoint(debug::MarkerOverlayRole::LeftWeaponSupportGrip, snapshot.leftGripWorld, 3.0f);

                if (frik::api::FRIKApi::inst) {
                    const RE::NiTransform appliedRight = frik::api::FRIKApi::inst->getHandWorldTransform(frik::api::FRIKApi::Hand::Right);
                    const RE::NiTransform appliedLeft = frik::api::FRIKApi::inst->getHandWorldTransform(frik::api::FRIKApi::Hand::Left);
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

        if (drawGrabTransformTelemetry) {
            auto publishGrabTelemetry = [&](Hand& hand, bool isLeft) {
                auto& telemetryState = _grabTransformTelemetryStates[isLeft ? 1 : 0];
                if (!hand.isHolding()) {
                    telemetryState.active = false;
                    telemetryState.frame = 0;
                    telemetryState.logFrameCounter = 0;
                    return;
                }

                if (!telemetryState.active) {
                    telemetryState.active = true;
                    telemetryState.session = _grabTransformTelemetryNextSession++;
                    telemetryState.frame = 0;
                    telemetryState.logFrameCounter = 0;
                }

                ++telemetryState.frame;
                const grab_transform_telemetry::FrameStamp stamp{
                    .session = telemetryState.session,
                    .frame = telemetryState.frame,
                    .tickMs = GetTickCount64(),
                };

                const RE::NiTransform rawHandWorld = isLeft ? context.left.rawHandWorld : context.right.rawHandWorld;

                grab_transform_telemetry::RuntimeSample sample{};
                if (!hand.getGrabTransformTelemetrySnapshot(
                        hknp,
                        rawHandWorld,
                        g_rockConfig.rockGrabObjectVisualHandAuthorityEnabled,
                        sample)) {
                    return;
                }

                if (drawGrabTransformTelemetryAxes) {
                    if (sample.hasGrabStartFrames) {
                        addAxisTransform(sample.currentConstraintDesiredObjectWorld,
                            isLeft ? debug::AxisOverlayRole::LeftGrabDesiredObject : debug::AxisOverlayRole::RightGrabDesiredObject,
                            sample.rawHandWorld.translate,
                            true);
                        addAxisTransform(sample.heldNodeWorld,
                            isLeft ? debug::AxisOverlayRole::LeftGrabHeldNode : debug::AxisOverlayRole::RightGrabHeldNode,
                            sample.rawHandWorld.translate,
                            true);
                        addMarkerLine(isLeft ? debug::MarkerOverlayRole::LeftGrabHeldDesiredError : debug::MarkerOverlayRole::RightGrabHeldDesiredError,
                            sample.currentConstraintDesiredObjectWorld.translate,
                            sample.heldNodeWorld.translate);
                    }
                    if (sample.hasHiggsReverseTarget) {
                        addAxisTransform(sample.higgsReverseTargetWorld,
                            isLeft ? debug::AxisOverlayRole::LeftGrabHiggsReverse : debug::AxisOverlayRole::RightGrabHiggsReverse,
                            sample.rawHandWorld.translate,
                            true);
                        addMarkerLine(isLeft ? debug::MarkerOverlayRole::LeftGrabHiggsReverseError : debug::MarkerOverlayRole::RightGrabHiggsReverseError,
                            sample.rawHandWorld.translate,
                            sample.higgsReverseTargetWorld.translate);
                    }
                    if (sample.hasConstraintReverseTarget) {
                        addAxisTransform(sample.constraintReverseTargetWorld,
                            isLeft ? debug::AxisOverlayRole::LeftGrabConstraintReverse : debug::AxisOverlayRole::RightGrabConstraintReverse,
                            sample.rawHandWorld.translate,
                            true);
                        addMarkerLine(isLeft ? debug::MarkerOverlayRole::LeftGrabConstraintReverseError : debug::MarkerOverlayRole::RightGrabConstraintReverseError,
                            sample.handBodyWorld.translate,
                            sample.constraintReverseTargetWorld.translate);
                    }
                    if (sample.hasRockVisualTarget) {
                        addAxisTransform(sample.rockVisualTargetWorld,
                            isLeft ? debug::AxisOverlayRole::LeftGrabRockVisualTarget : debug::AxisOverlayRole::RightGrabRockVisualTarget,
                            sample.rawHandWorld.translate,
                            true);
                        addMarkerLine(isLeft ? debug::MarkerOverlayRole::LeftGrabRockVisualError : debug::MarkerOverlayRole::RightGrabRockVisualError,
                            sample.rawHandWorld.translate,
                            sample.rockVisualTargetWorld.translate);
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
                        "GTEL S%u F%llu MS%llu H%s AUTH%d BODY%u HB%s OB%s",
                        stamp.session,
                        static_cast<unsigned long long>(stamp.frame),
                        static_cast<unsigned long long>(stamp.tickMs),
                        grab_transform_telemetry::handLabel(isLeft),
                        sample.visualAuthorityEnabled ? 1 : 0,
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
                        "PIVERR LIVE %.2f A %.1f %.1f %.1f B %.1f %.1f %.1f",
                        sample.pivotErrorGameUnits,
                        sample.pivotAWorld.x,
                        sample.pivotAWorld.y,
                        sample.pivotAWorld.z,
                        sample.pivotBWorld.x,
                        sample.pivotBWorld.y,
                        sample.pivotBWorld.z);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 5),
                        color,
                        "CON_DESN %.1f %.1f %.1f D %.2f %.2f",
                        sample.currentConstraintDesiredObjectWorld.translate.x,
                        sample.currentConstraintDesiredObjectWorld.translate.y,
                        sample.currentConstraintDesiredObjectWorld.translate.z,
                        sample.heldNodeToConstraintDesiredObject.positionGameUnits,
                        sample.heldNodeToConstraintDesiredObject.rotationDegrees);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 6),
                        color,
                        "CON_DESB %.1f %.1f %.1f D %.2f %.2f",
                        sample.currentConstraintDesiredBodyWorld.translate.x,
                        sample.currentConstraintDesiredBodyWorld.translate.y,
                        sample.currentConstraintDesiredBodyWorld.translate.z,
                        sample.heldBodyToConstraintDesiredBody.positionGameUnits,
                        sample.heldBodyToConstraintDesiredBody.rotationDegrees);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 7),
                        color,
                        "RAWHS_REV %.1f %.1f %.1f D %.2f %.2f",
                        sample.higgsReverseTargetWorld.translate.x,
                        sample.higgsReverseTargetWorld.translate.y,
                        sample.higgsReverseTargetWorld.translate.z,
                        sample.rawToHiggsReverse.positionGameUnits,
                        sample.rawToHiggsReverse.rotationDegrees);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 8),
                        color,
                        "CONHS_REV %.1f %.1f %.1f D %.2f %.2f C %.2f",
                        sample.constraintReverseTargetWorld.translate.x,
                        sample.constraintReverseTargetWorld.translate.y,
                        sample.constraintReverseTargetWorld.translate.z,
                        sample.handBodyToConstraintReverse.positionGameUnits,
                        sample.handBodyToConstraintReverse.rotationDegrees,
                        sample.higgsToConstraintReverse.rotationDegrees);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 9),
                        color,
                        "AUTH_REV %.1f %.1f %.1f D %.2f %.2f H %.2f",
                        sample.rockVisualTargetWorld.translate.x,
                        sample.rockVisualTargetWorld.translate.y,
                        sample.rockVisualTargetWorld.translate.z,
                        sample.rawToRockVisual.positionGameUnits,
                        sample.rawToRockVisual.rotationDegrees,
                        sample.higgsToRockVisual.rotationDegrees);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 10),
                        color,
                        "TBLOC %.1f %.1f %.1f DES %.1f %.1f %.1f E %.2f",
                        sample.constraintTransformBLocalGame.x,
                        sample.constraintTransformBLocalGame.y,
                        sample.constraintTransformBLocalGame.z,
                        sample.desiredTransformBLocalGame.x,
                        sample.desiredTransformBLocalGame.y,
                        sample.desiredTransformBLocalGame.z,
                        sample.transformBLocalDelta.distance);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 11),
                        color,
                        "ANGT CI %.2f RI %.2f CF %.2f TB %.2f EN%d",
                        sample.targetColumnsToConstraintInverseDegrees,
                        sample.targetRowsToConstraintInverseDegrees,
                        sample.targetColumnsToConstraintForwardDegrees,
                        sample.targetColumnsToTransformBDegrees,
                        sample.ragdollMotorEnabled ? 1 : 0);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 12),
                        color,
                        "MOTOR AT %.3f AD %.2f AF %.0f LT %.3f LF %.0f M %.2f",
                        sample.angularMotorTau,
                        sample.angularMotorDamping,
                        sample.angularMotorMaxForce,
                        sample.linearMotorTau,
                        sample.linearMotorMaxForce,
                        sample.heldBodyMass);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 13),
                        color,
                        "AXIS RAW %.2f %.2f %.2f CON %.2f %.2f %.2f AUTH %.2f %.2f %.2f",
                        sample.rawToHiggsReverseAxes.x,
                        sample.rawToHiggsReverseAxes.y,
                        sample.rawToHiggsReverseAxes.z,
                        sample.rawToConstraintReverseAxes.x,
                        sample.rawToConstraintReverseAxes.y,
                        sample.rawToConstraintReverseAxes.z,
                        sample.rawToRockVisualAxes.x,
                        sample.rawToRockVisualAxes.y,
                        sample.rawToRockVisualAxes.z);
                }

                ++telemetryState.logFrameCounter;
                const std::uint64_t logInterval =
                    static_cast<std::uint64_t>((std::max)(1, g_rockConfig.rockDebugGrabTransformTelemetryLogIntervalFrames));
                if (telemetryState.frame == 1 || telemetryState.logFrameCounter >= logInterval) {
                    telemetryState.logFrameCounter = 0;
                    const auto prefix = grab_transform_telemetry::formatStampPrefix(stamp, isLeft, telemetryState.frame == 1 ? "start" : "held");
                    const char* phaseLabel = telemetryState.frame == 1 ? "START" : "HELD";
                    ROCK_LOG_INFO(Hand,
                        "GRAB TELEMETRY {} {} formID={:08X} heldBody={} handBody={} heldSource={} handSource={} heldMotion={} handMotion={} auth={} surfTarget={} pivotFrame=LIVE pivotA=({:.2f},{:.2f},{:.2f}) pivotB=({:.2f},{:.2f},{:.2f}) pivotErr={:.3f}",
                        prefix,
                        phaseLabel,
                        sample.heldFormId,
                        sample.heldBodyId,
                        sample.handBodyId,
                        body_frame::bodyFrameSourceCode(sample.heldBodySource),
                        body_frame::bodyFrameSourceCode(sample.handBodySource),
                        sample.heldMotionIndex,
                        sample.handMotionIndex,
                        sample.visualAuthorityEnabled ? "on" : "off",
                        sample.rockVisualTargetUsedSurfaceFrame ? "surface" : "reverse",
                        sample.pivotAWorld.x,
                        sample.pivotAWorld.y,
                        sample.pivotAWorld.z,
                        sample.pivotBWorld.x,
                        sample.pivotBWorld.y,
                        sample.pivotBWorld.z,
                        sample.pivotErrorGameUnits);
                    ROCK_LOG_INFO(Hand,
                        "GRAB TELEMETRY {} {} liveHand=({:.2f},{:.2f},{:.2f}) handBody[{}]=({:.2f},{:.2f},{:.2f}) heldBody[{}]=({:.2f},{:.2f},{:.2f}) visualNode=({:.2f},{:.2f},{:.2f}) bodyDerivedNode=({:.2f},{:.2f},{:.2f}) bodyNodeToVisual={:.3f}gu/{:.3f}deg liveToHandBody={:.3f}gu/{:.3f}deg",
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
                        "GRAB TELEMETRY {} {} startLive=({:.2f},{:.2f},{:.2f}) startHandBody=({:.2f},{:.2f},{:.2f}) objectAtGrab=({:.2f},{:.2f},{:.2f}) desiredAtGrab=({:.2f},{:.2f},{:.2f}) startDrift={:.3f}gu/{:.3f}deg rawHS=({:.2f},{:.2f},{:.2f}) conHS=({:.2f},{:.2f},{:.2f}) bodyRaw=({:.2f},{:.2f},{:.2f}) bodyLocal=({:.2f},{:.2f},{:.2f})",
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
                        sample.constraintHandSpace.translate.x,
                        sample.constraintHandSpace.translate.y,
                        sample.constraintHandSpace.translate.z,
                        sample.handBodyToRawHandAtGrab.translate.x,
                        sample.handBodyToRawHandAtGrab.translate.y,
                        sample.handBodyToRawHandAtGrab.translate.z,
                        sample.bodyLocal.translate.x,
                        sample.bodyLocal.translate.y,
                        sample.bodyLocal.translate.z);
                    ROCK_LOG_INFO(Hand,
                        "GRAB TELEMETRY {} {} rawDesired=({:.2f},{:.2f},{:.2f}) heldToRawDesired={:.3f}gu/{:.3f}deg conDesiredNode=({:.2f},{:.2f},{:.2f}) heldToConDesired={:.3f}gu/{:.3f}deg conDesiredBody=({:.2f},{:.2f},{:.2f}) bodyToConDesired={:.3f}gu/{:.3f}deg",
                        prefix,
                        phaseLabel,
                        sample.currentRawDesiredObjectWorld.translate.x,
                        sample.currentRawDesiredObjectWorld.translate.y,
                        sample.currentRawDesiredObjectWorld.translate.z,
                        sample.heldNodeToRawDesiredObject.positionGameUnits,
                        sample.heldNodeToRawDesiredObject.rotationDegrees,
                        sample.currentConstraintDesiredObjectWorld.translate.x,
                        sample.currentConstraintDesiredObjectWorld.translate.y,
                        sample.currentConstraintDesiredObjectWorld.translate.z,
                        sample.heldNodeToConstraintDesiredObject.positionGameUnits,
                        sample.heldNodeToConstraintDesiredObject.rotationDegrees,
                        sample.currentConstraintDesiredBodyWorld.translate.x,
                        sample.currentConstraintDesiredBodyWorld.translate.y,
                        sample.currentConstraintDesiredBodyWorld.translate.z,
                        sample.heldBodyToConstraintDesiredBody.positionGameUnits,
                        sample.heldBodyToConstraintDesiredBody.rotationDegrees);
                    ROCK_LOG_INFO(Hand,
                        "GRAB TELEMETRY {} {} rawHSRev=({:.2f},{:.2f},{:.2f}) liveToRawHSRev={:.3f}gu/{:.3f}deg conHSRev=({:.2f},{:.2f},{:.2f}) handBodyToConHSRev={:.3f}gu/{:.3f}deg rawToConRev={:.3f}gu/{:.3f}deg authRev=({:.2f},{:.2f},{:.2f}) liveToAuthRev={:.3f}gu/{:.3f}deg rawRevToAuth={:.3f}gu/{:.3f}deg",
                        prefix,
                        phaseLabel,
                        sample.higgsReverseTargetWorld.translate.x,
                        sample.higgsReverseTargetWorld.translate.y,
                        sample.higgsReverseTargetWorld.translate.z,
                        sample.rawToHiggsReverse.positionGameUnits,
                        sample.rawToHiggsReverse.rotationDegrees,
                        sample.constraintReverseTargetWorld.translate.x,
                        sample.constraintReverseTargetWorld.translate.y,
                        sample.constraintReverseTargetWorld.translate.z,
                        sample.handBodyToConstraintReverse.positionGameUnits,
                        sample.handBodyToConstraintReverse.rotationDegrees,
                        sample.higgsToConstraintReverse.positionGameUnits,
                        sample.higgsToConstraintReverse.rotationDegrees,
                        sample.rockVisualTargetWorld.translate.x,
                        sample.rockVisualTargetWorld.translate.y,
                        sample.rockVisualTargetWorld.translate.z,
                        sample.rawToRockVisual.positionGameUnits,
                        sample.rawToRockVisual.rotationDegrees,
                        sample.higgsToRockVisual.positionGameUnits,
                        sample.higgsToRockVisual.rotationDegrees);
                    ROCK_LOG_INFO(Hand,
                        "GRAB TELEMETRY {} {} axisDots rawHSRev=({:.3f},{:.3f},{:.3f}) conHSRev=({:.3f},{:.3f},{:.3f}) authRev=({:.3f},{:.3f},{:.3f})",
                        prefix,
                        phaseLabel,
                        sample.rawToHiggsReverseAxes.x,
                        sample.rawToHiggsReverseAxes.y,
                        sample.rawToHiggsReverseAxes.z,
                        sample.rawToConstraintReverseAxes.x,
                        sample.rawToConstraintReverseAxes.y,
                        sample.rawToConstraintReverseAxes.z,
                        sample.rawToRockVisualAxes.x,
                        sample.rawToRockVisualAxes.y,
                        sample.rawToRockVisualAxes.z);
                    if (sample.hasConstraintAngularTelemetry) {
                        ROCK_LOG_INFO(Hand,
                            "GRAB TELEMETRY {} {} transformBLocal=({:.2f},{:.2f},{:.2f}) desiredTransformBLocal=({:.2f},{:.2f},{:.2f}) transformBErr={:.3f}gu targetErr(colsInv={:.3f}deg rowsInv={:.3f}deg colsForward={:.3f}deg colsTransformB={:.3f}deg) ragEnabled={} angTau={:.3f} angDamping={:.3f} angForce={:.1f} linTau={:.3f} linForce={:.1f} mass={:.3f}",
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
                            sample.targetRowsToConstraintInverseDegrees,
                            sample.targetColumnsToConstraintForwardDegrees,
                            sample.targetColumnsToTransformBDegrees,
                            sample.ragdollMotorEnabled ? "yes" : "no",
                            sample.angularMotorTau,
                            sample.angularMotorDamping,
                            sample.angularMotorMaxForce,
                            sample.linearMotorTau,
                            sample.linearMotorMaxForce,
                            sample.heldBodyMass);
                    }
                }
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
            if (debug_overlay_policy::shouldDrawHandBody(drawRockColliderBodies, g_rockConfig.rockDebugDrawHandColliders) &&
                !g_rockConfig.rockDebugDrawHandBoneColliders) {
                addBody(_rightHand.getCollisionBodyId(), debug::BodyOverlayRole::RightHand);
                addBody(_leftHand.getCollisionBodyId(), debug::BodyOverlayRole::LeftHand);
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
            }

            const auto weaponCount = _weaponCollision.getWeaponBodyCount();
            for (std::uint32_t i = 0; i < weaponCount; i++) {
                const bool drawNormalWeaponBody =
                    debug_overlay_policy::shouldDrawWeaponBody(drawRockColliderBodies, g_rockConfig.rockDebugDrawWeaponColliders, i, g_rockConfig.rockDebugMaxWeaponBodiesDrawn);
                if (drawNormalWeaponBody) {
                    addBody(RE::hknpBodyId{ _weaponCollision.getWeaponBodyIdAtomic(i) }, debug::BodyOverlayRole::Weapon);
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
