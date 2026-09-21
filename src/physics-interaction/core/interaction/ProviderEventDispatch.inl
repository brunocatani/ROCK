// Included in namespace rock. Event identity belongs to this live producer,
// not the last completed provider publication (which may belong to another world).
api::SampleV1 PhysicsInteraction::providerEventSample() const {
    return {provider::runtime::currentGameFrameIndex(), 0,
        _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire),
        _lifecycle.skeletonGenerationAtomic.load(std::memory_order_acquire),
        _lifecycle.providerGenerationAtomic.load(std::memory_order_acquire),
        _lifecycle.collisionGenerationAtomic.load(std::memory_order_acquire)};
}

    void PhysicsInteraction::dispatchPhysicsMessage(std::uint32_t msgType, bool isLeft, RE::TESObjectREFR*, std::uint32_t formID, std::uint32_t layer)
    {
        const auto sample=providerEventSample();
        provider::events::publishPhysics(msgType,isLeft,formID,layer,sample);
    }

    void PhysicsInteraction::dispatchGrabEvent(GrabEventData eventData)
    {
        eventData.size = sizeof(GrabEventData);
        eventData.version = ROCK_GRAB_EVENT_VERSION;
        if (eventData.refr && eventData.formID == 0) {
            eventData.formID = eventData.refr->GetFormID();
        }
        eventData.frameIndex = ++_grabEvents.frameCounter;

        handleGrabEventHaptics(eventData);

        const auto sample=providerEventSample();
        provider::events::publishGrab(eventData,sample);
    }
