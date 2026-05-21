PhysicsFrameContext PhysicsInteraction::buildFrameContext(RE::bhkWorld* bhk, RE::hknpWorld* hknp, float deltaSeconds)
{
    /*
     * Frame-context construction is separated from the main update loop so
     * lifecycle, collision, grab, weapon, and debug phases consume one coherent
     * snapshot of FRIK/FO4VR hand state. This keeps future frame inputs from
     * being added as scattered global reads throughout PhysicsInteraction::update().
     */
    PhysicsFrameContext frame{};
    frame.bhkWorld = bhk;
    frame.hknpWorld = hknp;
    frame.deltaSeconds = (deltaSeconds > 0.0f && deltaSeconds <= 0.1f) ? deltaSeconds : (1.0f / 90.0f);
    frame.worldReady = bhk && hknp;
    frame.menuBlocked = frik::api::FRIKApi::inst && frik::api::FRIKApi::inst->isAnyMenuOpen();
    frame.reloadBoundaryActive = false;

    if (auto* player = RE::PlayerCharacter::GetSingleton()) {
        (void)player;
        if (auto* playerNodes = f4vr::getPlayerNodes(); playerNodes && playerNodes->HmdNode) {
            frame.hmdPositionWorld = playerNodes->HmdNode->world.translate;
            const RE::NiPoint3 rawHmdForwardWorld = playerNodes->HmdNode->world.rotate.Transpose() * RE::NiPoint3(0.0f, 1.0f, 0.0f);
            frame.hasHmdFrame = selection_query_policy::tryNormalizeVectorForHmdCone(rawHmdForwardWorld, frame.hmdForwardWorld);
        }
    }

    auto buildHandInput = [&](bool isLeft, Hand& hand) {
        HandFrameInput input{};
        input.isLeft = isLeft;
        const bool rootHandReady = _handBoneCache.isReady();
        input.disabled = (isLeft ? s_leftHandDisabled.load(std::memory_order_acquire) : s_rightHandDisabled.load(std::memory_order_acquire)) || !rootHandReady;
        if (!rootHandReady) {
            return input;
        }

        input.rawHandWorld = getInteractionHandTransform(isLeft);
        input.handNode = getInteractionHandNode(isLeft);
        root_flattened_finger_skeleton_runtime::SemanticHandFrame semanticHandFrame{};
        if (!_handBoneCache.getSemanticHandFrame(isLeft, semanticHandFrame)) {
            input.disabled = true;
            return input;
        }

        input.grabAnchorWorld = semanticHandFrame.palmAnchorWorld.translate;
        if (frame.worldReady) {
            input.hasGrabAuthorityWorld = hand.tryComputeGrabAuthorityProxyFrameWorld(hknp, input.grabAuthorityWorld);
            input.grabAnchorWorld = input.hasGrabAuthorityWorld ? input.grabAuthorityWorld.translate : semanticHandFrame.palmAnchorWorld.translate;
        }
        input.palmNormalWorld =
            input.hasGrabAuthorityWorld ? computeGrabAuthorityPalmFaceWorld(input.grabAuthorityWorld) : semanticHandFrame.palmFaceWorld;
        input.pointingWorld = pointing_direction_math::applyFarGrabNormalReversal(
            root_flattened_finger_skeleton_runtime::transformSemanticHandFrameDirection(
                semanticHandFrame,
                g_rockConfig.rockPointingVectorHandspace),
            g_rockConfig.rockReverseFarGrabNormal);
        input.pinchDirectionWorld =
            input.hasGrabAuthorityWorld ? transformGrabAuthorityTuningDirection(input.grabAuthorityWorld, g_rockConfig.rockGrabPinchDetectionDirectionHandspace) :
                                          root_flattened_finger_skeleton_runtime::transformSemanticHandFrameDirection(
                                              semanticHandFrame, g_rockConfig.rockGrabPinchDetectionDirectionHandspace);
        if (g_rockConfig.rockDebugDrawGrabPockets) {
            root_flattened_finger_skeleton_runtime::Snapshot fingerSnapshot{};
            if (root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(isLeft, fingerSnapshot) &&
                fingerSnapshot.valid &&
                fingerSnapshot.fingers[0].valid &&
                fingerSnapshot.fingers[1].valid) {
                input.thumbPadWorld = (fingerSnapshot.fingers[0].points[1] + fingerSnapshot.fingers[0].points[2]) * 0.5f;
                input.indexPadWorld = (fingerSnapshot.fingers[1].points[1] + fingerSnapshot.fingers[1].points[2]) * 0.5f;
                input.pinchPocketWorld = (input.thumbPadWorld + input.indexPadWorld) * 0.5f;
                input.hasPinchPocketWorld = true;
            }
        }
        return input;
    };

    frame.right = buildHandInput(false, _rightHand);
    frame.left = buildHandInput(true, _leftHand);
    return frame;
}
