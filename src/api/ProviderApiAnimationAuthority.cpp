#define ROCK_API_EXPORTS
/*
 * Build the DLL side of the public provider ABI.
 *
 * ANIMATION AUTHORITY: native animation authority, the animation phase callbacks,
 * the FRIK hand visual authority bridge, native-anim runtime publication, and the
 * equipped grip state readers.
 *
 * The phase callback invoker uses MSVC SEH. Keep it free of C++ unwinding objects.
 *
 * Most entry points here must run on the claimed animation owner thread. That check
 * comes FIRST, before any structural or semantic check, per the shared entry
 * validation order.
 */
#include "api/detail/ProviderAnimationAuthority.h"
#include "api/detail/ProviderOwnerLifecycle.h"

#include "api/detail/ProviderApiCore.h"
#include "api/detail/ProviderApiEntryPoints.h"
#include "api/detail/ProviderTransformMath.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <mutex>

#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "RockConfig.h"

namespace rock::provider::detail
{
    using namespace rock;

    bool invokeAnimationPhaseCallbackSafely(
        RockProviderAnimationPhaseCallbackV1 callback,
        const RockProviderAnimationPhaseContextV1* context,
        void* userData)
    {
        if (!callback) {
            return true;
        }

#if defined(_MSC_VER)
        __try {
            callback(context, userData);
            return true;
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            return false;
        }
#else
        callback(context, userData);
        return true;
#endif
    }

    [[nodiscard]] HandVisualAuthoritySlot* findHandVisualAuthoritySlotLocked(
        const std::uint64_t ownerToken,
        const RockProviderHand hand)
    {
        HandVisualAuthoritySlot* available = nullptr;
        for (auto& slot : s_handVisualAuthoritySlots) {
            if (slot.ownerToken == ownerToken && slot.hand == hand) {
                return &slot;
            }
            if (slot.ownerToken == 0 && !available) {
                available = &slot;
            }
        }
        return available;
    }


    [[nodiscard]] bool clearHandVisualAuthoritySlotLocked(
        HandVisualAuthoritySlot& slot,
        const bool releaseSlot)
    {
        if (slot.ownerToken == 0) {
            return true;
        }

        const auto hand = toVisualHand(slot.hand);
        const auto worldFlag =
            static_cast<std::uint32_t>(RockProviderHandVisualAuthorityFlagV1::WorldTransform);
        const auto fingerFlag =
            static_cast<std::uint32_t>(RockProviderHandVisualAuthorityFlagV1::FingerLocalTransforms);
        bool cleared = true;
        if ((slot.publishedFlags & fingerFlag) != 0) {
            cleared = frik_visual_authority::clearHandPose(slot.tag, hand) && cleared;
        }
        if ((slot.publishedFlags & worldFlag) != 0) {
            cleared = frik_visual_authority::clearExternalHandWorldTransform(slot.tag, hand) && cleared;
        }

        if (cleared || !frik_visual_authority::isSkeletonReadyHint() || releaseSlot) {
            slot = {};
            return true;
        }
        return false;
    }

    [[nodiscard]] bool clearHandVisualAuthorityForOwner(
        const std::uint64_t ownerToken,
        const RockProviderHand hand,
        const bool releaseSlots)
    {
        bool cleared = true;
        std::scoped_lock lock(s_handVisualAuthorityMutex);
        for (auto& slot : s_handVisualAuthoritySlots) {
            if (slot.ownerToken != ownerToken ||
                (hand != RockProviderHand::None && slot.hand != hand)) {
                continue;
            }
            cleared = clearHandVisualAuthoritySlotLocked(slot, releaseSlots) &&
                cleared;
        }
        return cleared;
    }

    void pruneHandVisualAuthorityLocked(const std::uint64_t frameIndex)
    {
        for (auto& slot : s_handVisualAuthoritySlots) {
            if (slot.ownerToken == 0) {
                continue;
            }
            const bool generationChanged = generationGuardsStale(
                slot.worldGeneration,
                slot.skeletonGeneration,
                slot.providerGeneration);
            if (!generationChanged && provider_lease_policy::isActive(
                    frameIndex,
                    slot.expiresAfterFrame)) {
                continue;
            }

            const auto ownerToken = slot.ownerToken;
            (void)clearHandVisualAuthoritySlotLocked(slot, true);
            publishAuthorityLostEvent(
                ownerToken,
                RockProviderAuthorityKindV1::HandVisual,
                static_cast<std::uint32_t>(
                    generationChanged ?
                        RockProviderSuppressionInvalidationReasonV1::GenerationChanged :
                        RockProviderSuppressionInvalidationReasonV1::Expired));
        }
    }

    void clearAnimationPhaseCallbacksForOwnerLocked(const std::uint64_t ownerToken)
    {
        for (auto& slot : s_animationPhaseCallbacks) {
            if (slot.ownerToken == ownerToken) {
                slot = {};
            }
        }
    }

    void clearNativeAnimationRuntimePublicationLocked()
    {
        s_nativeAnimationRuntimeProviderOwner = 0;
        s_nativeAnimationRuntimePublication = {};
        s_nativeAnimationRuntimeExpiresAfterFrame = 0;
        s_hasNativeAnimationRuntimePublication = false;
    }

    void clearNativeAnimationRuntimePublicationForOwner(const std::uint64_t ownerToken)
    {
        std::scoped_lock lock(s_nativeAnimationRuntimePublicationMutex);
        if (s_nativeAnimationRuntimeProviderOwner == ownerToken) {
            clearNativeAnimationRuntimePublicationLocked();
        }
    }


    void pruneExpiredNativeAnimationRuntimePublicationLocked(
        const std::uint64_t frameIndex)
    {
        (void)pruneExpiredSlots(
            1,
            frameIndex,
            [](const std::size_t) {
                return s_hasNativeAnimationRuntimePublication;
            },
            [](const std::size_t) {
                return generationGuardsStale(
                    s_nativeAnimationRuntimePublication.worldGeneration,
                    s_nativeAnimationRuntimePublication.skeletonGeneration,
                    s_nativeAnimationRuntimePublication.providerGeneration);
            },
            [](const std::size_t) {
                return s_nativeAnimationRuntimeExpiresAfterFrame;
            },
            [](const std::size_t, const auto reason) {
                const auto ownerToken = s_nativeAnimationRuntimeProviderOwner;
                clearNativeAnimationRuntimePublicationLocked();
                publishAuthorityLostEvent(
                    ownerToken,
                    RockProviderAuthorityKindV1::NativeAnimationRuntime,
                    static_cast<std::uint32_t>(reason));
            });
    }


    void publishNativeAnimationAuthorityAggregateLocked()
    {
        std::uint32_t flags = 0;
        std::uint32_t ownerCount = 0;
        for (const auto& slot : s_nativeAnimationAuthoritySlots) {
            if (!slot.active) {
                continue;
            }
            flags |= slot.flags;
            ++ownerCount;
        }
        s_nativeAnimationAuthorityFlags.store(flags, std::memory_order_release);
        s_nativeAnimationAuthorityOwnerCount.store(ownerCount, std::memory_order_release);
    }

    void pruneExpiredNativeAnimationAuthorityLocked(std::uint64_t frameIndex)
    {
        const bool changed = pruneExpiredSlots(
            s_nativeAnimationAuthoritySlots.size(),
            frameIndex,
            [](const std::size_t index) {
                return s_nativeAnimationAuthoritySlots[index].active;
            },
            [](const std::size_t index) {
                const auto& slot = s_nativeAnimationAuthoritySlots[index];
                return generationGuardsStale(
                    slot.worldGeneration,
                    slot.skeletonGeneration,
                    slot.providerGeneration);
            },
            [](const std::size_t index) {
                return s_nativeAnimationAuthoritySlots[index].expiresAtFrame;
            },
            [](const std::size_t index, const auto reason) {
                auto& slot = s_nativeAnimationAuthoritySlots[index];
                publishAuthorityLostEvent(
                    slot.ownerToken,
                    RockProviderAuthorityKindV1::NativeAnimation,
                    static_cast<std::uint32_t>(reason));
                slot = {};
            });
        if (changed) {
            publishNativeAnimationAuthorityAggregateLocked();
        }
    }

    void clearNativeAnimationAuthorityForOwnerLocked(
        std::uint64_t ownerToken,
        const bool publishAggregate)
    {
        bool changed = false;
        for (auto& slot : s_nativeAnimationAuthoritySlots) {
            if (slot.active && slot.ownerToken == ownerToken) {
                slot = {};
                changed = true;
            }
        }
        if (changed && publishAggregate) {
            publishNativeAnimationAuthorityAggregateLocked();
        }
    }


    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetNativeAnimationAuthorityV1(
        std::uint64_t ownerToken,
        const RockProviderNativeAnimationAuthorityRequestV1* request)
    {
        const auto entryResult = validateEntry(request);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        constexpr auto implementedFlags = static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityFlagV1::ReloadPose);
        if (request->flags == 0 ||
            (request->flags & ~implementedFlags) != 0 ||
            request->leaseFrames == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        const auto generationResult = validateGenerationGuards(
            request->worldGeneration,
            request->skeletonGeneration,
            request->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }
        if (!apiIsProviderReady()) {
            return RockProviderResultV1::NotReady;
        }

        const auto frameIndex = currentProviderFrameIndex();
        const auto boundedLeaseFrames = provider_lease_policy::clampLeaseFrames(
            request->leaseFrames,
            ROCK_PROVIDER_MAX_NATIVE_ANIMATION_AUTHORITY_LEASE_FRAMES_V1);
        const auto expiresAtFrame = provider_lease_policy::exclusiveExpiryFrame(
            frameIndex,
            boundedLeaseFrames);

        std::scoped_lock lock(s_consumerMutex, s_nativeAnimationAuthorityMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::NativeAnimationAuthority);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        pruneExpiredNativeAnimationAuthorityLocked(frameIndex);
        NativeAnimationAuthoritySlot* available = nullptr;
        for (auto& slot : s_nativeAnimationAuthoritySlots) {
            if (slot.active && slot.ownerToken == ownerToken) {
                available = &slot;
                break;
            }
            if (!slot.active && !available) {
                available = &slot;
            }
        }
        if (!available) {
            return RockProviderResultV1::CapacityFull;
        }

        *available = NativeAnimationAuthoritySlot{
            .active = true,
            .ownerToken = ownerToken,
            .flags = request->flags,
            .expiresAtFrame = expiresAtFrame,
            .worldGeneration = request->worldGeneration,
            .skeletonGeneration = request->skeletonGeneration,
            .providerGeneration = request->providerGeneration,
        };
        publishNativeAnimationAuthorityAggregateLocked();
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearNativeAnimationAuthorityV1(std::uint64_t ownerToken)
    {
        const auto entryResult = validateOwnerEntry(ownerToken);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }

        std::scoped_lock lock(s_consumerMutex, s_nativeAnimationAuthorityMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::NativeAnimationAuthority);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        clearNativeAnimationAuthorityForOwnerLocked(ownerToken);
        return RockProviderResultV1::Ok;
    }

    bool ROCK_PROVIDER_CALL apiGetNativeAnimationAuthorityStateV1(
        RockProviderNativeAnimationAuthorityStateV1* outState)
    {
        if (!outState || outState->size != sizeof(RockProviderNativeAnimationAuthorityStateV1)) {
            return false;
        }

        RockProviderNativeAnimationRuntimePublicationV1 publication{};
        bool hasRuntimeProvider = false;
        {
            std::scoped_lock lock(s_nativeAnimationRuntimePublicationMutex);
            pruneExpiredNativeAnimationRuntimePublicationLocked(
                currentProviderFrameIndex());
            hasRuntimeProvider = s_hasNativeAnimationRuntimePublication;
            if (hasRuntimeProvider) {
                publication = s_nativeAnimationRuntimePublication;
            }
        }
        *outState = {};
        outState->size = sizeof(RockProviderNativeAnimationAuthorityStateV1);
        outState->version = ROCK_PROVIDER_API_VERSION;
        outState->activeFlags = s_nativeAnimationAuthorityFlags.load(std::memory_order_acquire);
        outState->statusFlags = hasRuntimeProvider ?
            publication.statusFlags |
                static_cast<std::uint32_t>(
                    RockProviderNativeAnimationAuthorityStatusFlagV1::RuntimeProviderAvailable) :
            0;
        outState->activeOwnerCount = s_nativeAnimationAuthorityOwnerCount.load(std::memory_order_acquire);
        outState->capturedTransformCount = hasRuntimeProvider ?
            publication.capturedTransformCount : 0;
        outState->captureSequence = hasRuntimeProvider ?
            publication.captureSequence : 0;
        return true;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRegisterAnimationPhaseCallbackV1(
        const std::uint64_t ownerToken,
        RockProviderAnimationPhaseCallbackV1 callback,
        void* userData,
        std::uint64_t* outCallbackToken)
    {
        if (!callback || !outCallbackToken) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outCallbackToken = 0;

        std::scoped_lock lock(s_consumerMutex, s_animationPhaseCallbackMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::AnimationPhases);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        AnimationPhaseCallbackSlot* available = nullptr;
        for (auto& slot : s_animationPhaseCallbacks) {
            if (slot.ownerToken == ownerToken) {
                return RockProviderResultV1::OwnerConflict;
            }
            if (!slot.callback && !available) {
                available = &slot;
            }
        }
        if (!available) {
            return RockProviderResultV1::CapacityFull;
        }

        auto token = s_nextAnimationPhaseCallbackToken.fetch_add(
            1,
            std::memory_order_acq_rel);
        if (token == 0) {
            token = s_nextAnimationPhaseCallbackToken.fetch_add(
                1,
                std::memory_order_acq_rel);
        }
        *available = AnimationPhaseCallbackSlot{
            .token = token,
            .ownerToken = ownerToken,
            .callback = callback,
            .userData = userData,
        };
        *outCallbackToken = token;
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiUnregisterAnimationPhaseCallbackV1(
        const std::uint64_t ownerToken,
        const std::uint64_t callbackToken)
    {
        if (ownerToken == 0 || callbackToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        std::scoped_lock lock(s_consumerMutex, s_animationPhaseCallbackMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::AnimationPhases);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        for (auto& slot : s_animationPhaseCallbacks) {
            if (slot.token == callbackToken && slot.ownerToken == ownerToken) {
                slot = {};
                return RockProviderResultV1::Ok;
            }
        }
        return RockProviderResultV1::TargetUnavailable;
    }

    bool ROCK_PROVIDER_CALL apiGetEquippedWeaponGripStateV1(
        const std::uint64_t ownerToken,
        RockProviderEquippedWeaponGripStateV1* outState)
    {
        if (!outState ||
            outState->size != sizeof(RockProviderEquippedWeaponGripStateV1)) {
            return false;
        }
        if (!onAnimationOwnerThread()) {
            return false;
        }
        {
            std::scoped_lock lock(s_consumerMutex);
            if (!consumerHasCapabilityLocked(
                    ownerToken,
                    RockProviderConsumerCapabilityV1::EquippedWeaponGripState)) {
                return false;
            }
        }

        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        return pi && pi->isInitialized() &&
               pi->queryProviderEquippedWeaponGripStateV1(*outState);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetHandVisualAuthorityV1(
        const std::uint64_t ownerToken,
        const RockProviderHandVisualAuthorityRequestV1* request)
    {
        const auto entryResult = validateEntry(
            request,
            EntryThreadPolicy::AnimationOwner);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (request->hand != RockProviderHand::Right &&
            request->hand != RockProviderHand::Left) {
            return RockProviderResultV1::InvalidArgument;
        }

        constexpr std::uint32_t worldFlag =
            static_cast<std::uint32_t>(RockProviderHandVisualAuthorityFlagV1::WorldTransform);
        constexpr std::uint32_t fingerFlag =
            static_cast<std::uint32_t>(RockProviderHandVisualAuthorityFlagV1::FingerLocalTransforms);
        constexpr std::uint32_t implementedFlags = worldFlag | fingerFlag;
        if (request->flags == 0 || (request->flags & ~implementedFlags) != 0 ||
            request->priority < -10000 || request->priority > 10000 ||
            request->leaseFrames == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & worldFlag) != 0 &&
            !finiteProviderTransform(request->worldTransform)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & fingerFlag) != 0) {
            if (request->fingerLocalTransformMask == 0 ||
                (request->fingerLocalTransformMask &
                    ~ROCK_PROVIDER_ALL_FINGER_LOCAL_TRANSFORMS_V1) != 0) {
                return RockProviderResultV1::InvalidArgument;
            }
            for (std::size_t index = 0; index < 15; ++index) {
                const auto bit = static_cast<std::uint16_t>(1u << index);
                if ((request->fingerLocalTransformMask & bit) != 0 &&
                    !finiteProviderTransform(request->fingerLocalTransforms[index])) {
                    return RockProviderResultV1::InvalidArgument;
                }
            }
        }

        const auto generationResult = validateGenerationGuards(
            request->worldGeneration,
            request->skeletonGeneration,
            request->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }

        {
            std::scoped_lock lock(s_consumerMutex);
            const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::HandVisualAuthority);
            if (ownerResult != RockProviderResultV1::Ok) {
                return ownerResult;
            }
        }
        if (!frik_visual_authority::isAvailable() ||
            !frik_visual_authority::isSkeletonReadyHint()) {
            return RockProviderResultV1::NotReady;
        }
        if ((request->flags & worldFlag) != 0 &&
            !frik_visual_authority::
                isExternalHandWorldSchedulerReady()) {
            return RockProviderResultV1::NotReady;
        }

        std::scoped_lock lock(s_handVisualAuthorityMutex);
        auto* slot = findHandVisualAuthoritySlotLocked(ownerToken, request->hand);
        if (!slot) {
            return RockProviderResultV1::CapacityFull;
        }
        if (slot->ownerToken != 0 && slot->publishedFlags != request->flags &&
            !clearHandVisualAuthoritySlotLocked(*slot, false)) {
            return RockProviderResultV1::TargetUnavailable;
        }
        if (slot->ownerToken == 0) {
            slot->ownerToken = ownerToken;
            slot->hand = request->hand;
            const int length = std::snprintf(
                slot->tag,
                sizeof(slot->tag),
                "ROCK_API_%016llX",
                static_cast<unsigned long long>(ownerToken));
            if (length <= 0 || static_cast<std::size_t>(length) >= sizeof(slot->tag)) {
                *slot = {};
                return RockProviderResultV1::InvalidArgument;
            }
        }

        const auto hand = toVisualHand(request->hand);
        bool published = true;
        if ((request->flags & fingerFlag) != 0) {
            frik_visual_authority::FingerLocalTransformOverride fingerLocals{};
            fingerLocals.enabledMask = request->fingerLocalTransformMask;
            for (std::size_t index = 0; index < 15; ++index) {
                const auto bit = static_cast<std::uint16_t>(1u << index);
                if ((fingerLocals.enabledMask & bit) != 0) {
                    fingerLocals.localTransforms[index] =
                        toNiTransform(request->fingerLocalTransforms[index]);
                }
            }
            published = frik_visual_authority::setHandPoseCustomWithPriority(
                            slot->tag,
                            hand,
                            frik_visual_authority::HandPoseData{},
                            request->priority) &&
                        frik_visual_authority::setHandPoseCustomLocalTransformsWithPriority(
                            slot->tag,
                            hand,
                            &fingerLocals,
                            request->priority);
        }
        if (published && (request->flags & worldFlag) != 0) {
            published = frik_visual_authority::publishExternalHandWorldTransform(
                slot->tag,
                hand,
                toNiTransform(request->worldTransform),
                request->priority);
        }
        if (!published) {
            slot->publishedFlags = request->flags;
            (void)clearHandVisualAuthoritySlotLocked(*slot, true);
            return RockProviderResultV1::TargetUnavailable;
        }

        slot->publishedFlags = request->flags;
        const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
            request->leaseFrames,
            ROCK_PROVIDER_MAX_HAND_VISUAL_AUTHORITY_LEASE_FRAMES_V1);
        slot->expiresAfterFrame = provider_lease_policy::exclusiveExpiryFrame(
            currentProviderFrameIndex(),
            leaseFrames);
        slot->worldGeneration = request->worldGeneration;
        slot->skeletonGeneration = request->skeletonGeneration;
        slot->providerGeneration = request->providerGeneration;
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearHandVisualAuthorityV1(
        const std::uint64_t ownerToken,
        const RockProviderHand hand)
    {
        const auto entryResult = validateOwnerEntry(
            ownerToken,
            EntryThreadPolicy::AnimationOwner);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        if (hand != RockProviderHand::None &&
            hand != RockProviderHand::Right &&
            hand != RockProviderHand::Left) {
            return RockProviderResultV1::InvalidArgument;
        }
        {
            std::scoped_lock lock(s_consumerMutex);
            const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::HandVisualAuthority);
            if (ownerResult != RockProviderResultV1::Ok) {
                return ownerResult;
            }
        }

        return clearHandVisualAuthorityForOwner(ownerToken, hand, false) ?
            RockProviderResultV1::Ok :
            RockProviderResultV1::TargetUnavailable;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiPublishNativeAnimationRuntimeV1(
        const std::uint64_t ownerToken,
        const RockProviderNativeAnimationRuntimePublicationV1* publication)
    {
        const auto entryResult = validateEntry(publication);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        constexpr std::uint32_t implementedStatusFlags =
            static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityStatusFlagV1::HookInstalled) |
            static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityStatusFlagV1::RuntimeEnabled) |
            static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityStatusFlagV1::CaptureValid) |
            static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityStatusFlagV1::LocalReloadTestLeaseActive) |
            static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityStatusFlagV1::HookInstallFailed) |
            static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityStatusFlagV1::ThreadMismatch) |
            static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityStatusFlagV1::CaptureFault);
        if ((publication->statusFlags & ~implementedStatusFlags) != 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (publication->leaseFrames == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        const auto generationResult = validateGenerationGuards(
            publication->worldGeneration,
            publication->skeletonGeneration,
            publication->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }

        std::scoped_lock lock(
            s_consumerMutex,
            s_nativeAnimationRuntimePublicationMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::NativeAnimationRuntimeProvider);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        if (s_nativeAnimationRuntimeProviderOwner != 0 &&
            s_nativeAnimationRuntimeProviderOwner != ownerToken) {
            return RockProviderResultV1::OwnerConflict;
        }

        s_nativeAnimationRuntimeProviderOwner = ownerToken;
        s_nativeAnimationRuntimePublication = *publication;
        s_nativeAnimationRuntimePublication.size =
            sizeof(RockProviderNativeAnimationRuntimePublicationV1);
        s_nativeAnimationRuntimePublication.version = ROCK_PROVIDER_API_VERSION;
        const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
            publication->leaseFrames,
            ROCK_PROVIDER_MAX_NATIVE_ANIMATION_RUNTIME_LEASE_FRAMES_V1);
        s_nativeAnimationRuntimePublication.leaseFrames = leaseFrames;
        s_nativeAnimationRuntimeExpiresAfterFrame =
            provider_lease_policy::exclusiveExpiryFrame(
                currentProviderFrameIndex(),
                leaseFrames);
        s_hasNativeAnimationRuntimePublication = true;
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearNativeAnimationRuntimeV1(
        const std::uint64_t ownerToken)
    {
        const auto entryResult = validateOwnerEntry(ownerToken);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        std::scoped_lock lock(
            s_consumerMutex,
            s_nativeAnimationRuntimePublicationMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::NativeAnimationRuntimeProvider);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        if (s_nativeAnimationRuntimeProviderOwner != 0 &&
            s_nativeAnimationRuntimeProviderOwner != ownerToken) {
            return RockProviderResultV1::OwnerConflict;
        }
        s_nativeAnimationRuntimeProviderOwner = 0;
        s_nativeAnimationRuntimePublication = {};
        s_nativeAnimationRuntimeExpiresAfterFrame = 0;
        s_hasNativeAnimationRuntimePublication = false;
        return RockProviderResultV1::Ok;
    }

}

namespace rock::provider
{
    using namespace detail;

    void dispatchAnimationPhaseCallbacksV1(
        const RockProviderAnimationPhaseV1 phase,
        const float deltaSeconds)
    {
        if (!claimOrValidateAnimationOwnerThread()) {
            if (!s_animationThreadMismatchLogged.exchange(
                    true,
                    std::memory_order_acq_rel)) {
                logger::error(
                    "ROCK provider animation phases observed multiple threads; callback dispatch is disabled for the mismatching thread.");
            }
            return;
        }

        std::uint64_t phaseFrameIndex =
            s_activeAnimationPhaseFrameIndex.load(std::memory_order_acquire);
        const bool startingAnimationFrame = phaseFrameIndex == 0;
        if (phaseFrameIndex == 0) {
            phaseFrameIndex = s_nextAnimationPhaseFrameIndex.fetch_add(
                1,
                std::memory_order_acq_rel);
            if (phaseFrameIndex == 0) {
                phaseFrameIndex = s_nextAnimationPhaseFrameIndex.fetch_add(
                    1,
                    std::memory_order_acq_rel);
            }
            s_activeAnimationPhaseFrameIndex.store(
                phaseFrameIndex,
                std::memory_order_release);
        }

        if (startingAnimationFrame) {
            std::scoped_lock lock(s_handVisualAuthorityMutex);
            pruneHandVisualAuthorityLocked(currentProviderFrameIndex());
        }

        RockProviderAnimationPhaseContextV1 context{};
        context.phase = phase;
        context.frameIndex = phaseFrameIndex;
        context.deltaSeconds =
            std::isfinite(deltaSeconds) && deltaSeconds > 0.0f &&
                    deltaSeconds <= 0.1f ?
                deltaSeconds :
                (1.0f / 90.0f);
        context.activeNativeAnimationAuthorityFlags =
            s_nativeAnimationAuthorityFlags.load(std::memory_order_acquire);

        const auto& runtime = runtime_state::currentFrame();
        if (g_rockConfig.rockEnabled) {
            context.flags |= static_cast<std::uint32_t>(
                RockProviderAnimationPhaseContextFlagV1::RockEnabled);
        }
        if (apiIsProviderReady()) {
            context.flags |= static_cast<std::uint32_t>(
                RockProviderAnimationPhaseContextFlagV1::ProviderReady);
        }
        if (runtime.localSkeletonReady) {
            context.flags |= static_cast<std::uint32_t>(
                RockProviderAnimationPhaseContextFlagV1::SkeletonReady);
        }
        if (runtime.localMenuBlocking) {
            context.flags |= static_cast<std::uint32_t>(
                RockProviderAnimationPhaseContextFlagV1::MenuBlocking);
        }
        if (runtime.compatibilityConfigBlocking) {
            context.flags |= static_cast<std::uint32_t>(
                RockProviderAnimationPhaseContextFlagV1::ConfigBlocking);
        }
        if (runtime.visualAuthorityAvailable && runtime.localSkeletonReady &&
            !runtime.localMenuBlocking &&
            !runtime.compatibilityConfigBlocking) {
            context.flags |= static_cast<std::uint32_t>(
                RockProviderAnimationPhaseContextFlagV1::VisualWritesAllowed);
            if (frik_visual_authority::
                    isExternalHandWorldSchedulerReady()) {
                context.flags |= static_cast<std::uint32_t>(
                    RockProviderAnimationPhaseContextFlagV1::
                        WorldTransformWritesAllowed);
            }
        }
        {
            std::scoped_lock lock(s_snapshotMutex);
            if (s_hasSnapshot) {
                context.worldGeneration = s_lastSnapshot.worldGeneration;
                context.skeletonGeneration = s_lastSnapshot.skeletonGeneration;
                context.providerGeneration = s_lastSnapshot.providerGeneration;
            }
        }

        for (std::size_t index = 0;
             index < s_animationPhaseCallbacks.size();
             ++index) {
            AnimationPhaseCallbackSlot slot{};
            {
                std::scoped_lock lock(s_animationPhaseCallbackMutex);
                slot = s_animationPhaseCallbacks[index];
            }
            if (!slot.callback) {
                continue;
            }

            bool callbackHealthy = true;
            try {
                callbackHealthy = invokeAnimationPhaseCallbackSafely(
                    slot.callback,
                    &context,
                    slot.userData);
            } catch (...) {
                callbackHealthy = false;
            }
            if (callbackHealthy) {
                continue;
            }

            logger::error(
                "ROCK provider animation phase callback token {} owner {:016X} faulted; releasing its animation publications.",
                slot.token,
                slot.ownerToken);
            clearOwnerStateAfterCallbackFault(slot.ownerToken);
        }

        if (phase == RockProviderAnimationPhaseV1::Complete) {
            s_activeAnimationPhaseFrameIndex.store(0, std::memory_order_release);
        }
    }

    std::uint32_t currentNativeAnimationAuthorityFlagsV1()
    {
        return s_nativeAnimationAuthorityFlags.load(std::memory_order_acquire);
    }

    void refreshNativeAnimationAuthorityLeasesV1()
    {
        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_nativeAnimationAuthorityMutex);
        pruneExpiredNativeAnimationAuthorityLocked(frameIndex);
    }
}
