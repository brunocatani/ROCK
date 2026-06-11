PhysicsFrameContext PhysicsInteraction::buildFrameContext(RE::bhkWorld* bhk, RE::hknpWorld* hknp, float deltaSeconds)
{
    /*
     * Frame-context construction is separated from the main update loop so
     * lifecycle, collision, grab, weapon, and debug phases consume one coherent
     * snapshot of ROCK/FO4VR hand state. This keeps future frame inputs from
     * being added as scattered global reads throughout PhysicsInteraction::update().
     */
    PhysicsFrameContext frame{};
    frame.bhkWorld = bhk;
    frame.hknpWorld = hknp;
    frame.deltaSeconds = (deltaSeconds > 0.0f && deltaSeconds <= 0.1f) ? deltaSeconds : (1.0f / 90.0f);
    frame.worldReady = bhk && hknp;
    frame.menuBlocked = runtime_state::isPhysicsMenuBlocked();
    frame.reloadBoundaryActive = false;
    const auto locomotionAuthority = updateGrabLocomotionAuthorityBridge(frame.deltaSeconds, frame.worldReady);
    const RE::NiPoint3 locomotionAuthorityOffsetGame = fromGrabLocomotionAuthorityVec(locomotionAuthority.offsetGameUnits);
    const bool locomotionAuthorityBridged = locomotionAuthority.active && pointLength(locomotionAuthorityOffsetGame) > 0.0001f;

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
        input.unbridgedRawHandWorld = input.rawHandWorld;
        const bool holdingForLocomotionAuthority = hand.isHoldingAtomic();
        input.locomotionAuthorityOffsetGame = (locomotionAuthorityBridged && holdingForLocomotionAuthority) ? locomotionAuthorityOffsetGame : RE::NiPoint3{};
        input.locomotionAuthorityBridged = locomotionAuthorityBridged && holdingForLocomotionAuthority;
        if (input.locomotionAuthorityBridged) {
            input.rawHandWorld.translate = input.rawHandWorld.translate + input.locomotionAuthorityOffsetGame;
        }
        input.handNode = getInteractionHandNode(isLeft);
        input.grabAnchorWorld = input.rawHandWorld.translate;
        RE::NiTransform closeSelectionBasisWorld = input.rawHandWorld;
        if (frame.worldReady) {
            RE::NiTransform proxyFrameWorld{};
            if (hand.tryComputeGrabProxyLocalPalmPocketFrameWorld(hknp, proxyFrameWorld)) {
                input.grabAnchorWorld = proxyFrameWorld.translate;
                closeSelectionBasisWorld = makeGeneratedProxyAuthorityRelationFrame(proxyFrameWorld);
            }
        }
        input.palmNormalWorld = computePalmNormalFromHandBasis(closeSelectionBasisWorld, isLeft);
        input.pointingWorld = computePointingVectorFromHandBasis(input.rawHandWorld, isLeft);
        input.closeSelectionDirectionWorld = computeCloseSelectionDirectionFromHandBasis(closeSelectionBasisWorld, isLeft);
        input.farSelectionDirectionWorld = computeFarSelectionDirectionFromHandBasis(input.rawHandWorld, isLeft);
        input.pinchDirectionWorld = computePinchDetectionDirectionFromHandBasis(closeSelectionBasisWorld, isLeft);
        if (g_rockConfig.rockDebugDrawGrabPockets) {
            root_flattened_finger_skeleton_runtime::Snapshot fingerSnapshot{};
            if (root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(isLeft, fingerSnapshot) &&
                fingerSnapshot.valid &&
                fingerSnapshot.fingers[0].valid &&
                fingerSnapshot.fingers[1].valid) {
                input.thumbPadWorld = fingerSnapshot.fingers[0].points[2];
                input.indexPadWorld = fingerSnapshot.fingers[1].points[2];
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
