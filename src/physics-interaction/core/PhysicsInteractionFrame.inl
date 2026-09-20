PhysicsFrameContext PhysicsInteraction::buildFrameContext(RE::bhkWorld* bhk, RE::hknpWorld* hknp)
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
    /*
     * One central timing identity per frame: the runtime snapshot's timing is
     * authoritative and its legacy sanitized delta is copied without local
     * resanitization.
     */
    frame.timing = runtime_state::currentFrame().timing;
    frame.deltaSeconds = runtime_state::currentFrame().deltaSeconds;
    frame.worldReady = bhk && hknp;
    frame.menuBlocked = runtime_state::isPhysicsMenuBlocked();
    const auto animationAuthorityFlags =
        rock::provider::currentNativeAnimationAuthorityFlagsV1();
    frame.reloadBoundaryActive =
        (animationAuthorityFlags &
            (authored_weapon_grip_capture_policy::kArms |
                authored_weapon_grip_capture_policy::kHands)) != 0;

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
        const bool rootHandReady = _handBoneCache.isReady() &&
            frik_hand_world_authority::tryGetRawHandWorld(isLeft, input.rawHandWorld);
        input.disabled = (isLeft ? s_leftHandDisabled.load(std::memory_order_acquire) : s_rightHandDisabled.load(std::memory_order_acquire)) || !rootHandReady;
        if (!rootHandReady) {
            return input;
        }

        // The resolver intentionally exposes no scene node. Keep the validity
        // of this exact controller sample through every consumer's admission.
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
        input.pinchDirectionWorld = computePinchDetectionDirectionFromHandBasis(input.rawHandWorld, isLeft);
        if (g_rockConfig.rockDebugDrawGrabPockets) {
            grab_pinch_pocket_policy::FingerFrame pinchFrame{};
            if (hand.tryGetPinchFingerFrame(pinchFrame)) {
                input.thumbPadWorld = pinchFrame.thumbTip;
                input.indexPadWorld = pinchFrame.indexTip;
                input.pinchPocketWorld = pinchFrame.center;
                input.pinchDirectionWorld = grab_pinch_pocket_policy::detectionDirection(
                    pinchFrame, input.pinchDirectionWorld, g_rockConfig.rockGrabPinchDetectionAxisBlend);
                input.hasPinchPocketWorld = true;
            }
        }
        return input;
    };

    frame.right = buildHandInput(false, _rightHand);
    frame.left = buildHandInput(true, _leftHand);
    return frame;
}
