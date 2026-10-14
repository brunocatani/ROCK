// Included in namespace rock. Event identity belongs to this live producer,
// not the last completed provider publication (which may belong to another world).
api::Sample PhysicsInteraction::providerEventSample() const {
    return {provider::runtime::currentGameFrameIndex(), 0,
        _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire),
        _lifecycle.skeletonGenerationAtomic.load(std::memory_order_acquire),
        _lifecycle.providerGenerationAtomic.load(std::memory_order_acquire),
        _lifecycle.collisionGenerationAtomic.load(std::memory_order_acquire)};
}

    void PhysicsInteraction::dispatchGrabEvent(GrabEventData eventData)
    {
        if (eventData.refr && eventData.formID == 0) {
            eventData.formID = eventData.refr->GetFormID();
        }
        eventData.frameIndex = ++_grabEvents.frameCounter;

        handleGrabEventHaptics(eventData);

        const auto sample=providerEventSample();
        provider::grab_callbacks::deliver(eventData,sample);
    }
