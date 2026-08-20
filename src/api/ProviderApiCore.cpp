#define ROCK_API_EXPORTS
// Build the DLL side of the public provider ABI.
#include "api/detail/ProviderApiCore.h"

#include "api/detail/ProviderApiEntryPoints.h"
#include "api/detail/ProviderTransformMath.h"

#include <algorithm>
#include <cstring>
#include <mutex>

#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"

namespace rock::provider::detail
{
    using namespace rock;

    constexpr std::uint64_t kRockIssuedOwnerTokenNamespace = 0xA000'0000'0000'0000ull;
    constexpr std::uint64_t kRockIssuedOwnerTokenSequenceMask = 0x0FFF'FFFF'FFFF'FFFFull;
    constexpr std::uint32_t kImplementedConsumerCapabilitiesV1 =
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::FrameSnapshots) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::ExternalBodies) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::ExternalContacts) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::OffhandReservation) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::InteractionCommands) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::HandInputSuppression) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::WeaponPartInteraction) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::NativeAnimationAuthority) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::AnimationPhases) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::EquippedWeaponGripState) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::HandVisualAuthority) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::NativeAnimationRuntimeProvider) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::EquippedWeaponHandlingAuthority) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::DebugOverlayPublication) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::ProviderEvents) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::HandInteractionState) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::ExternalBodyScopes) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::WeaponPartObservability) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::WeaponComposition) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::PoseReadback) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::SemanticHandContacts) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::PlayerColliderDescriptors) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::ScopeSightState) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::InputObservability) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::TouchGrabTargets) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::WorldRaycasts) |
        static_cast<std::uint32_t>(
            RockProviderConsumerCapabilityV1::ColliderVisualizationOverride);

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
                slot.ownerToken = 0;
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

#if defined(_MSC_VER)
    int captureFrameCallbackException(
        EXCEPTION_POINTERS* exception,
        FrameCallbackInvocationResult* result) noexcept
    {
        if (result) {
            result->healthy = false;
            if (exception && exception->ExceptionRecord) {
                result->exceptionCode =
                    exception->ExceptionRecord->ExceptionCode;
                result->exceptionAddress =
                    reinterpret_cast<std::uintptr_t>(
                        exception->ExceptionRecord->ExceptionAddress);
            }
        }
        return EXCEPTION_EXECUTE_HANDLER;
    }
#endif

    FrameCallbackInvocationResult invokeFrameCallbackSafely(
        RockProviderFrameCallback callback,
        const RockProviderFrameSnapshot* snapshot,
        void* userData)
    {
        FrameCallbackInvocationResult result{};
        if (!callback) {
            return result;
        }

#if defined(_MSC_VER)
        __try {
            callback(snapshot, userData);
        } __except (captureFrameCallbackException(
            GetExceptionInformation(),
            &result)) {
        }
#else
        callback(snapshot, userData);
#endif
        return result;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRegisterFrameCallbackForOwnerV1(
        const std::uint64_t ownerToken,
        RockProviderFrameCallback callback,
        void* userData,
        std::uint64_t* outCallbackToken)
    {
        if (ownerToken == 0 || !callback || !outCallbackToken) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outCallbackToken = 0;

        std::scoped_lock lock(s_consumerMutex, s_callbackMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::FrameSnapshots);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        for (auto& slot : s_callbacks) {
            if (!slot.callback) {
                auto token = s_nextCallbackToken.fetch_add(
                    1,
                    std::memory_order_acq_rel);
                if (token == 0) {
                    token = s_nextCallbackToken.fetch_add(
                        1,
                        std::memory_order_acq_rel);
                }
                slot = CallbackSlot{
                    .token = token,
                    .ownerToken = ownerToken,
                    .callback = callback,
                    .userData = userData,
                };
                *outCallbackToken = token;
                return RockProviderResultV1::Ok;
            }
        }
        return RockProviderResultV1::CapacityFull;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiUnregisterFrameCallbackForOwnerV1(
        const std::uint64_t ownerToken,
        const std::uint64_t callbackToken)
    {
        if (ownerToken == 0 || callbackToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_callbackMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::FrameSnapshots);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        for (auto& slot : s_callbacks) {
            if (slot.token != callbackToken) {
                continue;
            }
            if (slot.ownerToken != ownerToken) {
                return RockProviderResultV1::OwnerConflict;
            }
            slot = {};
            return RockProviderResultV1::Ok;
        }
        return RockProviderResultV1::TargetUnavailable;
    }

    [[nodiscard]] bool claimOrValidateAnimationOwnerThread()
    {
        const auto currentThread =
            static_cast<std::uint32_t>(GetCurrentThreadId());
        std::uint32_t expected = 0;
        if (s_animationOwnerThreadId.compare_exchange_strong(
                expected,
                currentThread,
                std::memory_order_acq_rel)) {
            return true;
        }
        return expected == currentThread;
    }

    [[nodiscard]] bool onAnimationOwnerThread()
    {
        const auto ownerThread =
            s_animationOwnerThreadId.load(std::memory_order_acquire);
        return ownerThread != 0 && ownerThread ==
            static_cast<std::uint32_t>(GetCurrentThreadId());
    }

    [[nodiscard]] bool entryThreadValid(const EntryThreadPolicy policy)
    {
        return policy == EntryThreadPolicy::AnyThread ||
               onAnimationOwnerThread();
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
        // ROCK's firing role is the only primary/offhand authority. Native
        // Fallout/FRIK handedness never changes ROCK controller identity.
        const bool primaryIsLeft =
            s_equippedWeaponFiringHandIsLeft.load(std::memory_order_acquire);
        return primaryIsLeft ? RockProviderHand::Left : RockProviderHand::Right;
    }

    RockProviderHand ROCK_PROVIDER_CALL apiGetOffhandHandV1()
    {
        return apiGetPrimaryHandV1() == RockProviderHand::Left ? RockProviderHand::Right : RockProviderHand::Left;
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
        if (!outFrame || outFrame->size < 112) {
            return false;
        }
        const auto requestedSize = outFrame->size;

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
        if (hand == snapshot.primaryHand) {
            frame.flags |= static_cast<std::uint32_t>(RockProviderHandFrameFlagV1::Primary);
        }
        if (hand == snapshot.offhandHand) {
            frame.flags |= static_cast<std::uint32_t>(RockProviderHandFrameFlagV1::Offhand);
        }

        frame.transform = isLeft ? snapshot.leftHandTransform : snapshot.rightHandTransform;
        frame.bodyId = isLeft ? snapshot.leftHandBodyId : snapshot.rightHandBodyId;
        frame.state = isLeft ? snapshot.leftHandState : snapshot.rightHandState;
        frame.frameIndex = snapshot.frameIndex;
        frame.worldGeneration = snapshot.worldGeneration;
        frame.skeletonGeneration = snapshot.skeletonGeneration;
        frame.providerGeneration = snapshot.providerGeneration;
        frame.collisionGeneration = snapshot.collisionGeneration;
        frame.stateSequence = snapshot.stateSequence;
        const auto copySize = (std::min<std::size_t>)(
            requestedSize,
            sizeof(frame));
        std::memcpy(outFrame, &frame, copySize);
        outFrame->size = static_cast<std::uint32_t>(copySize);
        return true;
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

    bool consumerHasCapabilityLocked(std::uint64_t ownerToken, RockProviderConsumerCapabilityV1 capability)
    {
        const auto* slot = findConsumerSlotLocked(ownerToken);
        return slot && hasConsumerCapabilityV1(slot->grantedCapabilities, capability);
    }

    [[nodiscard]] RockProviderTransform toProviderTransform(
        const RE::NiTransform& source)
    {
        RockProviderTransform target{};
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                target.rotate[row * 3 + column] =
                    source.rotate.entry[row][column];
            }
        }
        target.translate[0] = source.translate.x;
        target.translate[1] = source.translate.y;
        target.translate[2] = source.translate.z;
        target.scale = source.scale;
        return target;
    }

    bool ROCK_PROVIDER_CALL apiGetPresentedHandFrameV1(
        const RockProviderHand hand,
        RockProviderHandFrameV1* outFrame)
    {
        if (!outFrame ||
            outFrame->size < 112 ||
            (hand != RockProviderHand::Right &&
                hand != RockProviderHand::Left) ||
            !onAnimationOwnerThread() ||
            !apiIsProviderReady() ||
            !frik_visual_authority::isAvailable() ||
            !frik_visual_authority::isSkeletonReadyHint()) {
            return false;
        }

        const auto presentedWorld =
            frik_visual_authority::getHandWorldTransform(
                toVisualHand(hand));
        const auto providerTransform =
            toProviderTransform(presentedWorld);
        if (!finiteProviderTransform(providerTransform)) {
            return false;
        }

        const auto requestedSize = outFrame->size;
        RockProviderFrameSnapshot snapshot{};
        {
            std::scoped_lock lock(s_snapshotMutex);
            if (s_hasSnapshot) {
                snapshot = s_lastSnapshot;
            }
        }
        RockProviderHandFrameV1 frame{};
        frame.hand = hand;
        frame.flags = static_cast<std::uint32_t>(
                          RockProviderHandFrameFlagV1::Valid) |
                      static_cast<std::uint32_t>(
                          RockProviderHandFrameFlagV1::PresentedVisual);
        if (hand == RockProviderHand::Left) {
            frame.flags |= static_cast<std::uint32_t>(
                RockProviderHandFrameFlagV1::Left);
        }
        if (hand == snapshot.primaryHand) {
            frame.flags |= static_cast<std::uint32_t>(
                RockProviderHandFrameFlagV1::Primary);
        }
        if (hand == snapshot.offhandHand) {
            frame.flags |= static_cast<std::uint32_t>(
                RockProviderHandFrameFlagV1::Offhand);
        }
        frame.transform = providerTransform;
        frame.frameIndex = snapshot.frameIndex;
        frame.worldGeneration = snapshot.worldGeneration;
        frame.skeletonGeneration = snapshot.skeletonGeneration;
        frame.providerGeneration = snapshot.providerGeneration;
        frame.collisionGeneration = snapshot.collisionGeneration;
        frame.stateSequence = snapshot.stateSequence;
        const auto copySize = (std::min<std::size_t>)(
            requestedSize,
            sizeof(frame));
        std::memcpy(outFrame, &frame, copySize);
        outFrame->size = static_cast<std::uint32_t>(copySize);
        return true;
    }

    std::uint32_t currentProviderGenerationForRegistration()
    {
        return s_generationStateAvailable.load(std::memory_order_acquire) ?
            s_currentProviderGeneration.load(std::memory_order_acquire) :
            0;
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
        const auto entryResult = validateEntry(registration);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        if (!outHandle) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (outHandle->size != sizeof(RockProviderConsumerHandleV1)) {
            return RockProviderResultV1::InvalidSize;
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

    std::uint32_t ROCK_PROVIDER_CALL apiGetGrantedCapabilitiesV1(std::uint64_t ownerToken)
    {
        std::scoped_lock lock(s_consumerMutex);
        auto* slot = findConsumerSlotLocked(ownerToken);
        return slot ? slot->grantedCapabilities : 0;
    }

    RockProviderResultV1 validateReadCapability(
        const std::uint64_t ownerToken,
        const RockProviderConsumerCapabilityV1 capability)
    {
        std::scoped_lock lock(s_consumerMutex);
        return validateRegisteredOwnerCapabilityLocked(ownerToken, capability);
    }
}
