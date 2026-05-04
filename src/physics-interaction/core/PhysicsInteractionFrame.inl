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
        if (frame.worldReady) {
            input.grabAnchorWorld = hand.computeGrabPivotAWorld(hknp, input.rawHandWorld);
        }
        input.palmNormalWorld = computePalmNormalFromHandBasis(input.rawHandWorld, isLeft);
        input.pointingWorld = computePointingVectorFromHandBasis(input.rawHandWorld, isLeft);
        return input;
    };

    frame.right = buildHandInput(false, _rightHand);
    frame.left = buildHandInput(true, _leftHand);
    return frame;
}
