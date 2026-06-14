#define ROCK_API_EXPORTS
#include "ROCKProviderApi.h"

#include <array>
#include <atomic>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <mutex>

#include "physics-interaction/object/ExternalBodyRegistry.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/debug/DebugBodyOverlay.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "f4vr/F4VRUtils.h"

#ifdef DrawText
#undef DrawText
#endif

namespace
{
    using namespace rock::provider;
    using namespace rock;

    static_assert(static_cast<std::uint32_t>(RockProviderBodyZoneKind::LeftShoulder) ==
                  static_cast<std::uint32_t>(body_zone::BodyZoneKind::LeftShoulder));
    static_assert(static_cast<std::uint32_t>(RockProviderBodyZoneKind::RightShoulder) ==
                  static_cast<std::uint32_t>(body_zone::BodyZoneKind::RightShoulder));
    static_assert(static_cast<std::uint32_t>(RockProviderBodyZoneSide::Left) ==
                  static_cast<std::uint32_t>(body_zone::BodyZoneSide::Left));

    struct CallbackSlot
    {
        std::uint64_t token{ 0 };
        RockProviderFrameCallback callback{ nullptr };
        void* userData{ nullptr };
    };

    std::atomic<PhysicsInteraction*> s_physicsInteraction{ nullptr };
    std::atomic<std::uint64_t> s_nextFrameIndex{ 1 };
    std::atomic<std::uint64_t> s_nextCallbackToken{ 1 };
    std::mutex s_callbackMutex;
    std::array<CallbackSlot, 16> s_callbacks{};

    std::mutex s_snapshotMutex;
    RockProviderFrameSnapshot s_lastSnapshot{};
    bool s_hasSnapshot{ false };

    std::mutex s_externalBodyMutex;
    ExternalBodyRegistry s_externalBodies{};

    std::atomic<std::uint64_t> s_offhandReservationOwner{ 0 };
    std::atomic<std::uint32_t> s_offhandReservation{
        static_cast<std::uint32_t>(RockProviderOffhandReservation::Normal)
    };

    constexpr std::uint64_t kRockIssuedOwnerTokenNamespace = 0xA000'0000'0000'0000ull;
    constexpr std::uint64_t kRockIssuedOwnerTokenSequenceMask = 0x0FFF'FFFF'FFFF'FFFFull;
    constexpr std::uint32_t kImplementedConsumerCapabilitiesV1 =
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::FrameSnapshots) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::ExternalBodies) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::ExternalContacts) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::OffhandReservation) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::DiagnosticOverlay) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::DiagnosticInput);
    constexpr std::uint32_t kProviderFeatureBitsV1 =
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::FrameCallbacks) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::LifecycleFields) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::HandFrames) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::WeaponEvidence) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::BodyContacts) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::ExternalContacts) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::DiagnosticOverlay) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::DiagnosticInput) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::ConsumerRegistrationV1) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::OwnerFilteredExternalContactsV1);

    struct ConsumerSlot
    {
        std::uint64_t token{ 0 };
        std::uint32_t grantedCapabilities{ 0 };
        std::uint32_t providerGeneration{ 0 };
        char modName[64]{};
    };

    std::mutex s_consumerMutex;
    std::array<ConsumerSlot, ROCK_PROVIDER_MAX_CONSUMERS_V1> s_consumers{};
    std::atomic<std::uint64_t> s_nextConsumerTokenSequence{ 1 };

    std::uint32_t ROCK_PROVIDER_CALL apiGetVersion() { return ROCK_PROVIDER_API_VERSION; }

    const char* ROCK_PROVIDER_CALL apiGetModVersion()
    {
        static constexpr const char* version = "0.5.0";
        return version;
    }

    bool ROCK_PROVIDER_CALL apiIsProviderReady()
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        return pi && pi->isInitialized();
    }

    std::uint64_t ROCK_PROVIDER_CALL apiRegisterFrameCallback(RockProviderFrameCallback callback, void* userData)
    {
        if (!callback) {
            return 0;
        }

        std::scoped_lock lock(s_callbackMutex);
        for (auto& slot : s_callbacks) {
            if (!slot.callback) {
                slot.token = s_nextCallbackToken.fetch_add(1, std::memory_order_acq_rel);
                slot.callback = callback;
                slot.userData = userData;
                return slot.token;
            }
        }

        return 0;
    }

    bool ROCK_PROVIDER_CALL apiUnregisterFrameCallback(std::uint64_t callbackToken)
    {
        if (callbackToken == 0) {
            return false;
        }

        std::scoped_lock lock(s_callbackMutex);
        for (auto& slot : s_callbacks) {
            if (slot.token == callbackToken) {
                slot = {};
                return true;
            }
        }

        return false;
    }

    void clearCallbackSlot(std::uint64_t callbackToken)
    {
        if (callbackToken == 0) {
            return;
        }

        std::scoped_lock lock(s_callbackMutex);
        for (auto& slot : s_callbacks) {
            if (slot.token == callbackToken) {
                slot = {};
                return;
            }
        }
    }

    bool invokeFrameCallbackSafely(RockProviderFrameCallback callback, const RockProviderFrameSnapshot* snapshot, void* userData)
    {
        if (!callback) {
            return true;
        }

#if defined(_MSC_VER)
        __try {
            callback(snapshot, userData);
            return true;
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            return false;
        }
#else
        callback(snapshot, userData);
        return true;
#endif
    }

    bool ROCK_PROVIDER_CALL apiGetFrameSnapshot(RockProviderFrameSnapshot* outSnapshot)
    {
        if (!outSnapshot || outSnapshot->size < ROCK_PROVIDER_FRAME_SNAPSHOT_V1_SIZE) {
            return false;
        }

        const auto requestedSize = outSnapshot->size;
        std::scoped_lock lock(s_snapshotMutex);
        if (!s_hasSnapshot) {
            return false;
        }

        const auto copySize = (std::min<std::size_t>)(requestedSize, sizeof(RockProviderFrameSnapshot));
        std::memcpy(outSnapshot, &s_lastSnapshot, copySize);
        outSnapshot->size = static_cast<std::uint32_t>(copySize);
        return true;
    }

    RockProviderHand ROCK_PROVIDER_CALL apiGetPrimaryHandV1()
    {
        return f4vr::isLeftHandedMode() ? RockProviderHand::Left : RockProviderHand::Right;
    }

    RockProviderHand ROCK_PROVIDER_CALL apiGetOffhandHandV1()
    {
        return f4vr::isLeftHandedMode() ? RockProviderHand::Right : RockProviderHand::Left;
    }

    bool ROCK_PROVIDER_CALL apiGetHandFrameV1(RockProviderHand hand, RockProviderHandFrameV1* outFrame)
    {
        /*
         * Hand frames expose ROCK's hand authority as a value snapshot instead
         * of a NiNode lookup. Consumers need the same primary/offhand mapping,
         * body id, and root-flattened transform that ROCK drives each frame,
         * while ROCK deliberately does not promise a live scene node for that
         * authority surface.
         */
        if (!outFrame || outFrame->size != sizeof(RockProviderHandFrameV1)) {
            return false;
        }

        if (hand != RockProviderHand::Right && hand != RockProviderHand::Left) {
            return false;
        }

        RockProviderFrameSnapshot snapshot{};
        {
            std::scoped_lock lock(s_snapshotMutex);
            if (!s_hasSnapshot) {
                return false;
            }
            snapshot = s_lastSnapshot;
        }

        if (snapshot.providerReady == 0) {
            return false;
        }

        const bool isLeft = hand == RockProviderHand::Left;
        RockProviderHandFrameV1 frame{};
        frame.hand = hand;
        frame.flags = static_cast<std::uint32_t>(RockProviderHandFrameFlagV1::Valid) |
                      static_cast<std::uint32_t>(RockProviderHandFrameFlagV1::RootFlattenedAuthority);
        if (isLeft) {
            frame.flags |= static_cast<std::uint32_t>(RockProviderHandFrameFlagV1::Left);
        }
        if (hand == apiGetPrimaryHandV1()) {
            frame.flags |= static_cast<std::uint32_t>(RockProviderHandFrameFlagV1::Primary);
        }
        if (hand == apiGetOffhandHandV1()) {
            frame.flags |= static_cast<std::uint32_t>(RockProviderHandFrameFlagV1::Offhand);
        }

        frame.transform = isLeft ? snapshot.leftHandTransform : snapshot.rightHandTransform;
        frame.bodyId = isLeft ? snapshot.leftHandBodyId : snapshot.rightHandBodyId;
        frame.state = isLeft ? snapshot.leftHandState : snapshot.rightHandState;
        *outFrame = frame;
        return true;
    }

    std::size_t boundedStringLength(const char* value, std::size_t capacity)
    {
        for (std::size_t i = 0; i < capacity; ++i) {
            if (value[i] == '\0') {
                return i;
            }
        }
        return capacity;
    }

    bool modNameEquals(const ConsumerSlot& slot, const char* modName, std::size_t modNameLength)
    {
        return slot.token != 0 &&
               boundedStringLength(slot.modName, sizeof(slot.modName)) == modNameLength &&
               std::memcmp(slot.modName, modName, modNameLength) == 0;
    }

    ConsumerSlot* findConsumerSlotLocked(std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return nullptr;
        }

        for (auto& slot : s_consumers) {
            if (slot.token == ownerToken) {
                return &slot;
            }
        }
        return nullptr;
    }

    std::uint32_t currentProviderGenerationForRegistration()
    {
        std::scoped_lock lock(s_snapshotMutex);
        return s_hasSnapshot ? s_lastSnapshot.providerGeneration : 0;
    }

    std::uint64_t nextConsumerToken()
    {
        const auto sequence = s_nextConsumerTokenSequence.fetch_add(1, std::memory_order_acq_rel);
        return kRockIssuedOwnerTokenNamespace | (sequence & kRockIssuedOwnerTokenSequenceMask);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRegisterConsumerV1(
        const RockProviderConsumerRegistrationV1* registration,
        RockProviderConsumerHandleV1* outHandle)
    {
        if (!registration || !outHandle) {
            return RockProviderResultV1::InvalidArgument;
        }

        if (registration->size != sizeof(RockProviderConsumerRegistrationV1) || outHandle->size != sizeof(RockProviderConsumerHandleV1)) {
            return RockProviderResultV1::InvalidSize;
        }

        if (registration->version == 0 || registration->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }

        const auto modNameLength = boundedStringLength(registration->modName, sizeof(registration->modName));
        if (modNameLength == 0 || modNameLength >= sizeof(registration->modName)) {
            return RockProviderResultV1::InvalidArgument;
        }

        const auto grantedCapabilities = registration->requestedCapabilities & kImplementedConsumerCapabilitiesV1;
        const auto providerGeneration = currentProviderGenerationForRegistration();

        std::scoped_lock lock(s_consumerMutex);
        for (const auto& slot : s_consumers) {
            if (modNameEquals(slot, registration->modName, modNameLength)) {
                return RockProviderResultV1::OwnerConflict;
            }
        }

        for (auto& slot : s_consumers) {
            if (slot.token != 0) {
                continue;
            }

            slot = {};
            slot.token = nextConsumerToken();
            slot.grantedCapabilities = grantedCapabilities;
            slot.providerGeneration = providerGeneration;
            std::memcpy(slot.modName, registration->modName, modNameLength);

            *outHandle = {};
            outHandle->size = sizeof(RockProviderConsumerHandleV1);
            outHandle->version = ROCK_PROVIDER_API_VERSION;
            outHandle->ownerToken = slot.token;
            outHandle->grantedCapabilities = slot.grantedCapabilities;
            outHandle->providerGeneration = slot.providerGeneration;
            return RockProviderResultV1::Ok;
        }

        return RockProviderResultV1::CapacityFull;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiUnregisterConsumerV1(std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        {
            std::scoped_lock lock(s_consumerMutex);
            auto* slot = findConsumerSlotLocked(ownerToken);
            if (!slot) {
                return RockProviderResultV1::OwnerNotRegistered;
            }
            *slot = {};
        }

        {
            std::scoped_lock lock(s_externalBodyMutex);
            s_externalBodies.clearOwner(ownerToken);
        }

        if (s_offhandReservationOwner.load(std::memory_order_acquire) == ownerToken) {
            s_offhandReservation.store(static_cast<std::uint32_t>(RockProviderOffhandReservation::Normal), std::memory_order_release);
            s_offhandReservationOwner.store(0, std::memory_order_release);
        }

        (void)rock::input_remap_runtime::setExternalDiagnosticInputSuppression(ownerToken, 0);
        return RockProviderResultV1::Ok;
    }

    std::uint32_t ROCK_PROVIDER_CALL apiGetGrantedCapabilitiesV1(std::uint64_t ownerToken)
    {
        std::scoped_lock lock(s_consumerMutex);
        auto* slot = findConsumerSlotLocked(ownerToken);
        return slot ? slot->grantedCapabilities : 0;
    }

    bool ROCK_PROVIDER_CALL apiGetProviderLimitsV1(RockProviderLimitsV1* outLimits)
    {
        if (!outLimits || outLimits->size != sizeof(RockProviderLimitsV1)) {
            return false;
        }

        *outLimits = {};
        outLimits->size = sizeof(RockProviderLimitsV1);
        outLimits->version = ROCK_PROVIDER_API_VERSION;
        outLimits->featureBits = kProviderFeatureBitsV1;
        outLimits->maxFrameCallbacks = ROCK_PROVIDER_MAX_FRAME_CALLBACKS_V1;
        outLimits->maxConsumers = ROCK_PROVIDER_MAX_CONSUMERS_V1;
        outLimits->maxExternalBodies = ROCK_PROVIDER_MAX_EXTERNAL_BODIES_V1;
        outLimits->maxExternalContacts = ROCK_PROVIDER_MAX_EXTERNAL_CONTACTS_V1;
        outLimits->maxBodyContacts = ROCK_PROVIDER_MAX_BODY_CONTACTS_V1;
        outLimits->maxWeaponBodies = ROCK_PROVIDER_MAX_WEAPON_BODIES;
        outLimits->maxInteractionCommands = 0;
        return true;
    }

    bool ROCK_PROVIDER_CALL apiQueryWeaponContactAtPoint(
        const RockProviderWeaponContactQuery* query,
        RockProviderWeaponContactResult* outResult)
    {
        if (!query || !outResult ||
            query->size != sizeof(RockProviderWeaponContactQuery) ||
            outResult->size != sizeof(RockProviderWeaponContactResult)) {
            return false;
        }

        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return false;
        }

        return pi->queryProviderWeaponContactAtPoint(*query, *outResult);
    }

    std::uint32_t ROCK_PROVIDER_CALL apiGetWeaponEvidenceDescriptors(
        RockProviderWeaponEvidenceDescriptor* outDescriptors,
        std::uint32_t maxDescriptors)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return 0;
        }

        return pi->copyProviderWeaponEvidenceDescriptors(outDescriptors, maxDescriptors);
    }

    std::uint32_t ROCK_PROVIDER_CALL apiGetWeaponEvidenceDetailCountV1()
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return 0;
        }

        return pi->getProviderWeaponEvidenceDetailCountV1();
    }

    std::uint32_t ROCK_PROVIDER_CALL apiCopyWeaponEvidenceDetailsV1(
        RockProviderWeaponEvidenceDetailV1* outDetails,
        std::uint32_t maxDetails)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return 0;
        }

        return pi->copyProviderWeaponEvidenceDetailsV1(outDetails, maxDetails);
    }

    std::uint32_t ROCK_PROVIDER_CALL apiGetWeaponEvidenceDetailPointCountV1(std::uint32_t bodyId)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return 0;
        }

        return pi->getProviderWeaponEvidenceDetailPointCountV1(bodyId);
    }

    std::uint32_t ROCK_PROVIDER_CALL apiCopyWeaponEvidenceDetailPointsV1(
        std::uint32_t bodyId,
        RockProviderPoint3* outPoints,
        std::uint32_t maxPoints)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return 0;
        }

        return pi->copyProviderWeaponEvidenceDetailPointsV1(bodyId, outPoints, maxPoints);
    }

    std::uint32_t ROCK_PROVIDER_CALL apiGetBodyContactSnapshotV1(
        RockProviderBodyContactV1* outContacts,
        std::uint32_t maxContacts)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return 0;
        }

        return pi->copyProviderBodyContacts(outContacts, maxContacts);
    }

    RE::NiPoint3 providerPoint(const float source[3])
    {
        return RE::NiPoint3(source[0], source[1], source[2]);
    }

    RE::NiTransform providerTransform(const RockProviderTransform& source)
    {
        RE::NiTransform transform{};
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                transform.rotate.entry[row][column] = source.rotate[static_cast<std::size_t>(row * 3 + column)];
            }
        }
        transform.translate = providerPoint(source.translate);
        transform.scale = source.scale;
        return transform;
    }

    bool diagnosticOverlayFlagSet(std::uint32_t flags, RockProviderDiagnosticOverlayFlagsV1 flag)
    {
        return (flags & static_cast<std::uint32_t>(flag)) != 0;
    }

    bool ROCK_PROVIDER_CALL apiPublishDiagnosticOverlay(const RockProviderDiagnosticOverlayFrameV1* frame)
    {
        if (!frame || frame->size != sizeof(RockProviderDiagnosticOverlayFrameV1) || frame->ownerToken == 0 || frame->hknpWorld == 0) {
            return false;
        }

        /*
         * External diagnostic drawing is intentionally exposed as a small value
         * ABI instead of sharing ROCK's renderer internals with consumers. ROCK
         * remains the only owner of the OpenVR/D3D hook; PAPER and future tools
         * publish axes, markers, and note text as frame data.
         */
        debug::Install();

        debug::BodyOverlayFrame overlay{};
        overlay.world = reinterpret_cast<RE::hknpWorld*>(frame->hknpWorld);
        overlay.drawAxes = diagnosticOverlayFlagSet(frame->flags, RockProviderDiagnosticOverlayFlagsV1::DrawAxes);
        overlay.drawMarkers = diagnosticOverlayFlagSet(frame->flags, RockProviderDiagnosticOverlayFlagsV1::DrawMarkers);
        overlay.drawText = diagnosticOverlayFlagSet(frame->flags, RockProviderDiagnosticOverlayFlagsV1::DrawScreenText);

        const auto axisCount = (std::min)(frame->axisCount, ROCK_PROVIDER_MAX_DIAGNOSTIC_AXES_V1);
        for (std::uint32_t i = 0; overlay.drawAxes && i < axisCount && overlay.axisCount < overlay.axisEntries.size(); ++i) {
            const auto& source = frame->axes[i];
            if (source.size != sizeof(RockProviderDiagnosticOverlayAxisV1)) {
                continue;
            }

            auto& target = overlay.axisEntries[overlay.axisCount++];
            target.source = debug::AxisOverlaySource::Transform;
            target.role = debug::AxisOverlayRole::TargetBody;
            target.transform = providerTransform(source.transform);
            target.translationStart = providerPoint(source.translationStart);
            target.drawTranslationLine = source.drawTranslationLine != 0;
        }

        const auto markerCount = (std::min)(frame->markerCount, ROCK_PROVIDER_MAX_DIAGNOSTIC_MARKERS_V1);
        for (std::uint32_t i = 0; overlay.drawMarkers && i < markerCount && overlay.markerCount < overlay.markerEntries.size(); ++i) {
            const auto& source = frame->markers[i];
            if (source.size != sizeof(RockProviderDiagnosticOverlayMarkerV1)) {
                continue;
            }

            auto& target = overlay.markerEntries[overlay.markerCount++];
            target.role = debug::MarkerOverlayRole::TargetVisualOrigin;
            target.position = providerPoint(source.position);
            target.lineEnd = providerPoint(source.lineEnd);
            target.size = source.sizeGame;
            target.drawPoint = source.drawPoint != 0;
            target.drawLine = source.drawLine != 0;
        }

        const auto textCount = (std::min)(frame->textCount, ROCK_PROVIDER_MAX_DIAGNOSTIC_TEXT_V1);
        for (std::uint32_t i = 0; overlay.drawText && i < textCount && overlay.textCount < overlay.textEntries.size(); ++i) {
            const auto& source = frame->texts[i];
            if (source.size != sizeof(RockProviderDiagnosticOverlayTextV1)) {
                continue;
            }

            auto& target = overlay.textEntries[overlay.textCount++];
            std::snprintf(target.text, sizeof(target.text), "%s", source.text);
            target.x = source.x;
            target.y = source.y;
            target.size = source.sizeScale;
            std::copy(std::begin(source.color), std::end(source.color), std::begin(target.color));
            target.worldAnchor = providerPoint(source.worldAnchor);
            target.worldAnchored = source.worldAnchored != 0;
        }

        debug::PublishFrame(overlay);
        return true;
    }

    bool ROCK_PROVIDER_CALL apiGetDiagnosticInputSnapshotV1(std::uint64_t ownerToken, RockProviderDiagnosticInputSnapshotV1* outSnapshot)
    {
        if (ownerToken == 0 || !outSnapshot || outSnapshot->size != sizeof(RockProviderDiagnosticInputSnapshotV1)) {
            return false;
        }

        const auto snapshot = rock::input_remap_runtime::consumeDiagnosticInputSnapshot();
        outSnapshot->version = ROCK_PROVIDER_API_VERSION;
        outSnapshot->ownerToken = ownerToken;
        outSnapshot->sequence = snapshot.sequence;
        outSnapshot->flags = snapshot.flags;
        outSnapshot->rightThumbstickX = snapshot.rightThumbstickX;
        outSnapshot->rightThumbstickY = snapshot.rightThumbstickY;
        outSnapshot->primaryTriggerAxisX = snapshot.primaryTriggerAxisX;
        return true;
    }

    bool ROCK_PROVIDER_CALL apiSetDiagnosticInputSuppressionV1(std::uint64_t ownerToken, std::uint32_t suppressionFlags)
    {
        return rock::input_remap_runtime::setExternalDiagnosticInputSuppression(ownerToken, suppressionFlags);
    }

    bool ROCK_PROVIDER_CALL apiRegisterExternalBodiesV1(
        std::uint64_t ownerToken,
        const RockProviderExternalBodyRegistration* bodies,
        std::uint32_t bodyCount)
    {
        std::scoped_lock lock(s_externalBodyMutex);
        return s_externalBodies.registerBodies(ownerToken, bodies, bodyCount);
    }

    void ROCK_PROVIDER_CALL apiClearExternalBodies(std::uint64_t ownerToken)
    {
        std::scoped_lock lock(s_externalBodyMutex);
        s_externalBodies.clearOwner(ownerToken);
    }

    std::uint32_t ROCK_PROVIDER_CALL apiGetExternalContactSnapshotV1(
        RockProviderExternalContactV1* outContacts,
        std::uint32_t maxContacts)
    {
        std::scoped_lock lock(s_externalBodyMutex);
        return s_externalBodies.copyContactsV1(outContacts, maxContacts);
    }

    std::uint32_t ROCK_PROVIDER_CALL apiGetExternalContactSnapshotForOwnerV1(
        std::uint64_t ownerToken,
        RockProviderExternalContactV1* outContacts,
        std::uint32_t maxContacts)
    {
        if (ownerToken == 0 || !outContacts || maxContacts == 0) {
            return 0;
        }

        std::scoped_lock lock(s_consumerMutex, s_externalBodyMutex);
        if (!findConsumerSlotLocked(ownerToken)) {
            return 0;
        }
        return s_externalBodies.copyContactsForOwnerV1(ownerToken, outContacts, maxContacts);
    }

    bool ROCK_PROVIDER_CALL apiSetOffhandInteractionReservation(std::uint64_t ownerToken, RockProviderOffhandReservation reservation)
    {
        if (ownerToken == 0) {
            return false;
        }

        if (reservation == RockProviderOffhandReservation::Normal) {
            const auto currentOwner = s_offhandReservationOwner.load(std::memory_order_acquire);
            if (currentOwner == ownerToken || currentOwner == 0) {
                s_offhandReservation.store(static_cast<std::uint32_t>(RockProviderOffhandReservation::Normal), std::memory_order_release);
                s_offhandReservationOwner.store(0, std::memory_order_release);
            }
            return true;
        }

        s_offhandReservationOwner.store(ownerToken, std::memory_order_release);
        s_offhandReservation.store(static_cast<std::uint32_t>(reservation), std::memory_order_release);
        return true;
    }

    constexpr RockProviderApi ROCK_PROVIDER_API_FUNCTION_TABLE{
        .getVersion = &apiGetVersion,
        .getModVersion = &apiGetModVersion,
        .isProviderReady = &apiIsProviderReady,
        .registerFrameCallback = &apiRegisterFrameCallback,
        .unregisterFrameCallback = &apiUnregisterFrameCallback,
        .getFrameSnapshot = &apiGetFrameSnapshot,
        .queryWeaponContactAtPoint = &apiQueryWeaponContactAtPoint,
        .getWeaponEvidenceDescriptors = &apiGetWeaponEvidenceDescriptors,
        .clearExternalBodies = &apiClearExternalBodies,
        .setOffhandInteractionReservation = &apiSetOffhandInteractionReservation,
        .registerExternalBodiesV1 = &apiRegisterExternalBodiesV1,
        .getExternalContactSnapshotV1 = &apiGetExternalContactSnapshotV1,
        .getWeaponEvidenceDetailCountV1 = &apiGetWeaponEvidenceDetailCountV1,
        .copyWeaponEvidenceDetailsV1 = &apiCopyWeaponEvidenceDetailsV1,
        .getWeaponEvidenceDetailPointCountV1 = &apiGetWeaponEvidenceDetailPointCountV1,
        .copyWeaponEvidenceDetailPointsV1 = &apiCopyWeaponEvidenceDetailPointsV1,
        .publishDiagnosticOverlay = &apiPublishDiagnosticOverlay,
        .getDiagnosticInputSnapshotV1 = &apiGetDiagnosticInputSnapshotV1,
        .setDiagnosticInputSuppressionV1 = &apiSetDiagnosticInputSuppressionV1,
        .getBodyContactSnapshotV1 = &apiGetBodyContactSnapshotV1,
        .getPrimaryHandV1 = &apiGetPrimaryHandV1,
        .getOffhandHandV1 = &apiGetOffhandHandV1,
        .getHandFrameV1 = &apiGetHandFrameV1,
        .registerConsumerV1 = &apiRegisterConsumerV1,
        .unregisterConsumerV1 = &apiUnregisterConsumerV1,
        .getGrantedCapabilitiesV1 = &apiGetGrantedCapabilitiesV1,
        .getProviderLimitsV1 = &apiGetProviderLimitsV1,
        .getExternalContactSnapshotForOwnerV1 = &apiGetExternalContactSnapshotForOwnerV1,
    };
}

namespace rock::provider
{
    ROCK_PROVIDER_API const RockProviderApi* ROCK_PROVIDER_CALL ROCKAPI_GetProviderApi()
    {
        return &ROCK_PROVIDER_API_FUNCTION_TABLE;
    }

    void setPhysicsInteractionInstance(rock::PhysicsInteraction* pi)
    {
        s_physicsInteraction.store(pi, std::memory_order_release);
    }

    void dispatchFrameCallbacks(rock::PhysicsInteraction& pi)
    {
        RockProviderFrameSnapshot snapshot{};
        snapshot.frameIndex = s_nextFrameIndex.fetch_add(1, std::memory_order_acq_rel);
        pi.fillProviderFrameSnapshot(snapshot);
        snapshot.externalBodyCount = currentExternalBodyCount();

        {
            std::scoped_lock lock(s_snapshotMutex);
            s_lastSnapshot = snapshot;
            s_hasSnapshot = true;
        }

        std::array<CallbackSlot, 16> callbacks{};
        {
            std::scoped_lock lock(s_callbackMutex);
            callbacks = s_callbacks;
        }

        for (const auto& slot : callbacks) {
            if (slot.callback) {
                bool callbackHealthy = true;
                try {
                    callbackHealthy = invokeFrameCallbackSafely(slot.callback, &snapshot, slot.userData);
                } catch (...) {
                    callbackHealthy = false;
                }

                if (!callbackHealthy) {
                    logger::error("ROCK provider frame callback token {} faulted; unregistering the callback.", slot.token);
                    clearCallbackSlot(slot.token);
                }
            }
        }
    }

    void clearExternalBodiesForProviderLoss()
    {
        {
            std::scoped_lock lock(s_externalBodyMutex);
            s_externalBodies.clearAll();
        }
        s_offhandReservation.store(static_cast<std::uint32_t>(RockProviderOffhandReservation::Normal), std::memory_order_release);
        s_offhandReservationOwner.store(0, std::memory_order_release);
    }

    bool isExternalBodyId(std::uint32_t bodyId)
    {
        std::scoped_lock lock(s_externalBodyMutex);
        return s_externalBodies.containsBody(bodyId);
    }

    bool isExternalBodyDynamicPushSuppressed(std::uint32_t bodyId)
    {
        std::scoped_lock lock(s_externalBodyMutex);
        return s_externalBodies.suppressesRockDynamicPush(bodyId);
    }

    bool recordExternalHandContact(bool isLeft, std::uint32_t handBodyId, std::uint32_t externalBodyId, std::uint64_t frameIndex)
    {
        std::scoped_lock lock(s_externalBodyMutex);
        if (!s_externalBodies.containsBody(externalBodyId)) {
            return false;
        }
        s_externalBodies.recordHandContact(isLeft, handBodyId, externalBodyId, frameIndex);
        return true;
    }

    bool recordExternalContact(const RockProviderExternalContactV1& contact)
    {
        std::scoped_lock lock(s_externalBodyMutex);
        return s_externalBodies.recordContactV1(contact);
    }

    RockProviderOffhandReservation currentOffhandReservation()
    {
        return static_cast<RockProviderOffhandReservation>(s_offhandReservation.load(std::memory_order_acquire));
    }

    std::uint32_t currentExternalBodyCount()
    {
        std::scoped_lock lock(s_externalBodyMutex);
        return s_externalBodies.bodyCount();
    }
}
