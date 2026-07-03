#define ROCK_API_EXPORTS
#include "ROCKProviderApi.h"

#include <array>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string_view>

#include "physics-interaction/object/ExternalBodyRegistry.h"
#include "physics-interaction/api/InteractionCommandQueue.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/weapon/WeaponPartGripReportPolicy.h"
#include "physics-interaction/weapon/WeaponPartRuntime.h"
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
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartGripKindV1::FiringGrip) ==
                  static_cast<std::uint32_t>(weapon_part_grip_report_policy::HandGripKind::FiringGrip));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartGripKindV1::SupportFullAuthority) ==
                  static_cast<std::uint32_t>(weapon_part_grip_report_policy::HandGripKind::SupportFullAuthority));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartGripKindV1::SupportVisualOnly) ==
                  static_cast<std::uint32_t>(weapon_part_grip_report_policy::HandGripKind::SupportVisualOnly));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartGripKindV1::PartCarry) ==
                  static_cast<std::uint32_t>(weapon_part_grip_report_policy::HandGripKind::PartCarry));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartGripKindV1::AttachOnly) ==
                  static_cast<std::uint32_t>(weapon_part_grip_report_policy::HandGripKind::AttachOnly));

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
    // Published together with the frame snapshot; indexed [right, left].
    std::array<RockProviderWeaponPartGripStateV1, 2> s_lastPartGripStates{};

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
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::InteractionCommands) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::HandInputSuppression) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::WeaponPartInteraction);
    constexpr std::uint32_t kProviderFeatureBitsV1 =
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::FrameCallbacks) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::LifecycleFields) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::HandFrames) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::WeaponEvidence) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::BodyContacts) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::ExternalContacts) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::ConsumerRegistrationV1) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::OwnerFilteredExternalContactsV1) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::InteractionCommandQueue) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::ForceGrabCommand) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::ForceReleaseCommand) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::ThrownDropCommand) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::HandInputSuppression) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::WeaponPartInteraction) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::WeaponPartGripState);
    constexpr std::uint32_t kImplementedForceGrabFlagsV1 =
        static_cast<std::uint32_t>(RockProviderForceGrabFlagV1::UsePreferredGrabPointGame);
    constexpr std::uint32_t kImplementedForceReleaseFlagsV1 =
        static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::ImmediateCollisionRestore) |
        static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::RequireMatchingTarget) |
        static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::UseVelocityHavok);
    constexpr std::uint32_t kImplementedThrownDropFlagsV1 =
        static_cast<std::uint32_t>(RockProviderThrownDropFlagV1::ImmediateCollisionRestore) |
        static_cast<std::uint32_t>(RockProviderThrownDropFlagV1::RequireMatchingTarget) |
        static_cast<std::uint32_t>(RockProviderThrownDropFlagV1::UseVelocityHavok);
    constexpr std::uint32_t kImplementedHandInputSuppressionFlagsV1 =
        static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::SuppressConfigModeChord) |
        static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::SuppressOpenVrGameInput);
    constexpr std::uint32_t kImplementedWeaponPartTargetFlagsV1 =
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchBodyId) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceRoot) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceName) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchPartKind) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchReloadRole) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSupportRole) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSocketRole) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchActionRole);
    constexpr std::uint32_t kImplementedWeaponPartDriveMatcherFlagsV1 =
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchBodyId) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceRoot) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceName);
    constexpr std::uint32_t kProviderInvalidBodyId = 0x7FFF'FFFFu;

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

    struct InteractionCommandSlot
    {
        bool active{ false };
        QueuedInteractionCommandV1 command{};
    };

    struct InteractionCommandResultSlot
    {
        bool active{ false };
        RockProviderInteractionCommandResultV1 result{};
    };

    std::mutex s_interactionCommandMutex;
    std::array<InteractionCommandSlot, ROCK_PROVIDER_MAX_INTERACTION_COMMANDS_V1> s_interactionCommands{};
    std::array<InteractionCommandResultSlot, ROCK_PROVIDER_MAX_COMPLETED_INTERACTION_COMMANDS_V1> s_interactionResults{};
    std::size_t s_nextInteractionResultSlot{ 0 };
    std::atomic<std::uint64_t> s_nextInteractionCommandId{ 1 };

    struct HandInputSuppressionSlot
    {
        bool active{ false };
        std::uint64_t ownerToken{ 0 };
        RockProviderHand hand{ RockProviderHand::None };
        std::uint32_t flags{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
    };

    std::mutex s_handInputSuppressionMutex;
    std::array<HandInputSuppressionSlot, ROCK_PROVIDER_MAX_HAND_INPUT_SUPPRESSIONS_V1> s_handInputSuppressions{};

    struct WeaponPartTargetSlot
    {
        bool active{ false };
        std::uint64_t ownerToken{ 0 };
        RockProviderWeaponPartTargetV1 target{};
    };

    struct WeaponPartDriveSlot
    {
        bool active{ false };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
        RockProviderWeaponPartDriveTargetV1 target{};
    };

    std::mutex s_weaponPartMutex;
    std::array<WeaponPartTargetSlot, ROCK_PROVIDER_MAX_WEAPON_PART_TARGETS_V1> s_weaponPartTargets{};
    std::array<WeaponPartDriveSlot, ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1> s_weaponPartDrives{};

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

    bool ROCK_PROVIDER_CALL apiGetWeaponPartGripStateV1(RockProviderHand hand, RockProviderWeaponPartGripStateV1* outState)
    {
        if (!outState || outState->size != sizeof(RockProviderWeaponPartGripStateV1)) {
            return false;
        }
        if (hand != RockProviderHand::Right && hand != RockProviderHand::Left) {
            return false;
        }

        std::scoped_lock lock(s_snapshotMutex);
        if (!s_hasSnapshot || s_lastSnapshot.providerReady == 0) {
            return false;
        }
        *outState = s_lastPartGripStates[hand == RockProviderHand::Left ? 1u : 0u];
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

    bool consumerHasCapabilityLocked(std::uint64_t ownerToken, RockProviderConsumerCapabilityV1 capability)
    {
        const auto* slot = findConsumerSlotLocked(ownerToken);
        return slot && hasConsumerCapabilityV1(slot->grantedCapabilities, capability);
    }

    std::uint64_t currentProviderFrameIndex()
    {
        const auto nextFrameIndex = s_nextFrameIndex.load(std::memory_order_acquire);
        return nextFrameIndex > 0 ? nextFrameIndex - 1 : 0;
    }

    void pruneExpiredHandInputSuppressionsLocked(std::uint64_t frameIndex)
    {
        for (auto& slot : s_handInputSuppressions) {
            if (slot.active && slot.expiresAfterFrame < frameIndex) {
                slot = {};
            }
        }
    }

    void clearHandInputSuppressionsForOwnerLocked(std::uint64_t ownerToken, RockProviderHand hand)
    {
        for (auto& slot : s_handInputSuppressions) {
            if (!slot.active || slot.ownerToken != ownerToken) {
                continue;
            }
            if (hand == RockProviderHand::None || slot.hand == hand) {
                slot = {};
            }
        }
    }

    void clearWeaponPartTargetsForOwnerLocked(std::uint64_t ownerToken)
    {
        for (auto& slot : s_weaponPartTargets) {
            if (slot.active && slot.ownerToken == ownerToken) {
                slot = {};
            }
        }
    }

    void clearWeaponPartDrivesForOwnerLocked(std::uint64_t ownerToken)
    {
        for (auto& slot : s_weaponPartDrives) {
            if (slot.active && slot.ownerToken == ownerToken) {
                slot = {};
            }
        }
    }

    void pruneExpiredWeaponPartDrivesLocked(std::uint64_t frameIndex)
    {
        for (auto& slot : s_weaponPartDrives) {
            if (slot.active && slot.expiresAfterFrame < frameIndex) {
                slot = {};
            }
        }
    }

    bool hasValidWeaponPartMatcher(std::uint32_t flags, std::uint32_t bodyId, std::uintptr_t sourceRoot, const char* sourceName)
    {
        if ((flags & ~kImplementedWeaponPartTargetFlagsV1) != 0 || (flags & kImplementedWeaponPartTargetFlagsV1) == 0) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchBodyId)) != 0 && bodyId == kProviderInvalidBodyId) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceRoot)) != 0 && sourceRoot == 0) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceName)) != 0 &&
            boundedStringLength(sourceName, ROCK_PROVIDER_MAX_EVIDENCE_NAME) == 0) {
            return false;
        }
        return true;
    }

    bool hasConcreteWeaponPartDriveMatcher(std::uint32_t flags, std::uint32_t bodyId, std::uintptr_t sourceRoot, const char* sourceName)
    {
        if ((flags & ~kImplementedWeaponPartDriveMatcherFlagsV1) != 0 || (flags & kImplementedWeaponPartDriveMatcherFlagsV1) == 0) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchBodyId)) != 0 && bodyId == kProviderInvalidBodyId) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceRoot)) != 0 && sourceRoot == 0) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceName)) != 0 &&
            boundedStringLength(sourceName, ROCK_PROVIDER_MAX_EVIDENCE_NAME) == 0) {
            return false;
        }
        return ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceRoot)) != 0 && sourceRoot != 0) ||
               ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchBodyId)) != 0 && bodyId != kProviderInvalidBodyId) ||
               ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceName)) != 0 &&
                   boundedStringLength(sourceName, ROCK_PROVIDER_MAX_EVIDENCE_NAME) != 0);
    }

    bool isValidWeaponPartKindValue(std::uint32_t value)
    {
        return value <= static_cast<std::uint32_t>(WeaponPartKind::Other);
    }

    bool isValidWeaponReloadRoleValue(std::uint32_t value)
    {
        return value <= static_cast<std::uint32_t>(WeaponReloadRole::Receiver);
    }

    bool isValidWeaponSupportRoleValue(std::uint32_t value)
    {
        return value <= static_cast<std::uint32_t>(WeaponSupportGripRole::ReceiverSupport);
    }

    bool isValidWeaponSocketRoleValue(std::uint32_t value)
    {
        return value <= static_cast<std::uint32_t>(WeaponSocketRole::LoadingGate);
    }

    bool isValidWeaponActionRoleValue(std::uint32_t value)
    {
        return value <= static_cast<std::uint32_t>(WeaponActionRole::Latch);
    }

    bool hasValidWeaponPartTargetSemantics(const RockProviderWeaponPartTargetV1& target)
    {
        const auto flags = target.flags;
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchPartKind)) != 0 &&
            !isValidWeaponPartKindValue(target.partKind)) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchReloadRole)) != 0 &&
            !isValidWeaponReloadRoleValue(target.reloadRole)) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSupportRole)) != 0 &&
            !isValidWeaponSupportRoleValue(target.supportRole)) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSocketRole)) != 0 &&
            !isValidWeaponSocketRoleValue(target.socketRole)) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchActionRole)) != 0 &&
            !isValidWeaponActionRoleValue(target.actionRole)) {
            return false;
        }
        return true;
    }

    std::size_t availableWeaponPartTargetSlotsForOwnerLocked(std::uint64_t ownerToken)
    {
        std::size_t available = 0;
        for (const auto& slot : s_weaponPartTargets) {
            if (!slot.active || slot.ownerToken == ownerToken) {
                ++available;
            }
        }
        return available;
    }

    std::size_t availableWeaponPartDriveSlotsForOwnerLocked(std::uint64_t ownerToken)
    {
        std::size_t available = 0;
        for (const auto& slot : s_weaponPartDrives) {
            if (!slot.active || slot.ownerToken == ownerToken) {
                ++available;
            }
        }
        return available;
    }

    bool isValidWeaponPartGrabMode(RockProviderWeaponPartGrabModeV1 mode)
    {
        return mode == RockProviderWeaponPartGrabModeV1::FullTwoHandAuthority ||
               mode == RockProviderWeaponPartGrabModeV1::AttachOnly;
    }

    bool isValidWeaponPartDriveSpace(RockProviderWeaponPartDriveSpaceV1 space)
    {
        return space == RockProviderWeaponPartDriveSpaceV1::WeaponRootLocal ||
               space == RockProviderWeaponPartDriveSpaceV1::SourceParentLocal;
    }

    bool isFiniteProviderTransform(const RockProviderTransform& transform)
    {
        for (float value : transform.rotate) {
            if (!std::isfinite(value)) {
                return false;
            }
        }
        return std::isfinite(transform.translate[0]) &&
               std::isfinite(transform.translate[1]) &&
               std::isfinite(transform.translate[2]) &&
               std::isfinite(transform.scale) &&
               std::abs(transform.scale) > 0.0001f;
    }

    weapon_part_runtime::GrabMode toRuntimeGrabMode(RockProviderWeaponPartGrabModeV1 mode)
    {
        switch (mode) {
        case RockProviderWeaponPartGrabModeV1::FullTwoHandAuthority:
            return weapon_part_runtime::GrabMode::FullTwoHandAuthority;
        case RockProviderWeaponPartGrabModeV1::AttachOnly:
            return weapon_part_runtime::GrabMode::AttachOnly;
        case RockProviderWeaponPartGrabModeV1::None:
        default:
            return weapon_part_runtime::GrabMode::None;
        }
    }

    RockProviderWeaponPartGrabModeV1 fromRuntimeGrabMode(weapon_part_runtime::GrabMode mode)
    {
        switch (mode) {
        case weapon_part_runtime::GrabMode::FullTwoHandAuthority:
            return RockProviderWeaponPartGrabModeV1::FullTwoHandAuthority;
        case weapon_part_runtime::GrabMode::AttachOnly:
            return RockProviderWeaponPartGrabModeV1::AttachOnly;
        case weapon_part_runtime::GrabMode::None:
        default:
            return RockProviderWeaponPartGrabModeV1::None;
        }
    }

    weapon_part_runtime::Target toRuntimeTarget(const WeaponPartTargetSlot& slot)
    {
        weapon_part_runtime::Target target{};
        if (!slot.active) {
            return target;
        }

        target.active = true;
        target.ownerToken = slot.ownerToken;
        target.weaponGenerationKey = slot.target.weaponGenerationKey;
        target.flags = slot.target.flags;
        target.grabMode = toRuntimeGrabMode(slot.target.grabMode);
        target.bodyId = slot.target.bodyId;
        target.sourceRoot = slot.target.sourceRoot;
        std::memcpy(target.sourceName.data(), slot.target.sourceName, target.sourceName.size());
        target.sourceName[target.sourceName.size() - 1] = '\0';
        target.partKind = static_cast<WeaponPartKind>(slot.target.partKind);
        target.reloadRole = static_cast<WeaponReloadRole>(slot.target.reloadRole);
        target.supportRole = static_cast<WeaponSupportGripRole>(slot.target.supportRole);
        target.socketRole = static_cast<WeaponSocketRole>(slot.target.socketRole);
        target.actionRole = static_cast<WeaponActionRole>(slot.target.actionRole);
        target.groupId = slot.target.groupId;
        target.priority = slot.target.priority;
        return target;
    }

    RockProviderResultV1 validateRegisteredOwnerCapabilityLocked(
        std::uint64_t ownerToken,
        RockProviderConsumerCapabilityV1 capability)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        if (!findConsumerSlotLocked(ownerToken)) {
            return RockProviderResultV1::OwnerNotRegistered;
        }
        if (!consumerHasCapabilityLocked(ownerToken, capability)) {
            return RockProviderResultV1::PermissionDenied;
        }
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 validateInteractionCommandOwnerLocked(std::uint64_t ownerToken)
    {
        return validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::InteractionCommands);
    }

    RockProviderResultV1 validateGenerationGuards(
        std::uint32_t worldGeneration,
        std::uint32_t skeletonGeneration,
        std::uint32_t providerGeneration)
    {
        if (worldGeneration == 0 && skeletonGeneration == 0 && providerGeneration == 0) {
            return RockProviderResultV1::Ok;
        }

        std::scoped_lock lock(s_snapshotMutex);
        if (!s_hasSnapshot) {
            return RockProviderResultV1::NotReady;
        }
        if (worldGeneration != 0 && worldGeneration != s_lastSnapshot.worldGeneration) {
            return RockProviderResultV1::WorldNotReady;
        }
        if (skeletonGeneration != 0 && skeletonGeneration != s_lastSnapshot.skeletonGeneration) {
            return RockProviderResultV1::NotReady;
        }
        if (providerGeneration != 0 && providerGeneration != s_lastSnapshot.providerGeneration) {
            return RockProviderResultV1::NotReady;
        }
        return RockProviderResultV1::Ok;
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

    std::uint64_t nextInteractionCommandId()
    {
        auto id = s_nextInteractionCommandId.fetch_add(1, std::memory_order_acq_rel);
        if (id == 0) {
            id = s_nextInteractionCommandId.fetch_add(1, std::memory_order_acq_rel);
        }
        return id;
    }

    RockProviderHand commandHand(const QueuedInteractionCommandV1& command)
    {
        switch (command.kind) {
        case RockProviderInteractionCommandKindV1::ForceGrab:
            return command.forceGrab.hand;
        case RockProviderInteractionCommandKindV1::ForceRelease:
            return command.forceRelease.hand;
        case RockProviderInteractionCommandKindV1::ThrownDrop:
            return command.thrownDrop.hand;
        default:
            return RockProviderHand::None;
        }
    }

    std::uintptr_t commandTargetRefr(const QueuedInteractionCommandV1& command)
    {
        switch (command.kind) {
        case RockProviderInteractionCommandKindV1::ForceGrab:
            return command.forceGrab.targetRefr;
        case RockProviderInteractionCommandKindV1::ForceRelease:
            return command.forceRelease.targetRefr;
        case RockProviderInteractionCommandKindV1::ThrownDrop:
            return command.thrownDrop.targetRefr;
        default:
            return 0;
        }
    }

    std::uint32_t commandTargetFormId(const QueuedInteractionCommandV1& command)
    {
        switch (command.kind) {
        case RockProviderInteractionCommandKindV1::ForceGrab:
            return command.forceGrab.targetFormId;
        case RockProviderInteractionCommandKindV1::ForceRelease:
            return command.forceRelease.targetFormId;
        case RockProviderInteractionCommandKindV1::ThrownDrop:
            return command.thrownDrop.targetFormId;
        default:
            return 0;
        }
    }

    std::uint32_t commandTargetBodyId(const QueuedInteractionCommandV1& command)
    {
        switch (command.kind) {
        case RockProviderInteractionCommandKindV1::ForceGrab:
            return command.forceGrab.targetBodyId;
        case RockProviderInteractionCommandKindV1::ForceRelease:
            return command.forceRelease.targetBodyId;
        case RockProviderInteractionCommandKindV1::ThrownDrop:
            return command.thrownDrop.targetBodyId;
        default:
            return kProviderInvalidBodyId;
        }
    }

    std::uint32_t commandWorldGeneration(const QueuedInteractionCommandV1& command)
    {
        switch (command.kind) {
        case RockProviderInteractionCommandKindV1::ForceGrab:
            return command.forceGrab.worldGeneration;
        case RockProviderInteractionCommandKindV1::ForceRelease:
            return command.forceRelease.worldGeneration;
        case RockProviderInteractionCommandKindV1::ThrownDrop:
            return command.thrownDrop.worldGeneration;
        default:
            return 0;
        }
    }

    std::uint32_t commandSkeletonGeneration(const QueuedInteractionCommandV1& command)
    {
        switch (command.kind) {
        case RockProviderInteractionCommandKindV1::ForceGrab:
            return command.forceGrab.skeletonGeneration;
        case RockProviderInteractionCommandKindV1::ForceRelease:
            return command.forceRelease.skeletonGeneration;
        case RockProviderInteractionCommandKindV1::ThrownDrop:
            return command.thrownDrop.skeletonGeneration;
        default:
            return 0;
        }
    }

    std::uint32_t commandProviderGeneration(const QueuedInteractionCommandV1& command)
    {
        switch (command.kind) {
        case RockProviderInteractionCommandKindV1::ForceGrab:
            return command.forceGrab.providerGeneration;
        case RockProviderInteractionCommandKindV1::ForceRelease:
            return command.forceRelease.providerGeneration;
        case RockProviderInteractionCommandKindV1::ThrownDrop:
            return command.thrownDrop.providerGeneration;
        default:
            return 0;
        }
    }

    RockProviderInteractionCommandResultV1 makeCommandResult(
        const QueuedInteractionCommandV1& command,
        RockProviderInteractionCommandStateV1 state,
        RockProviderInteractionFailureV1 failure)
    {
        RockProviderInteractionCommandResultV1 result{};
        result.size = sizeof(RockProviderInteractionCommandResultV1);
        result.version = ROCK_PROVIDER_API_VERSION;
        result.ownerToken = command.ownerToken;
        result.commandId = command.commandId;
        result.kind = command.kind;
        result.state = state;
        result.failure = failure;
        result.hand = commandHand(command);
        result.targetRefr = commandTargetRefr(command);
        result.targetFormId = commandTargetFormId(command);
        result.targetBodyId = commandTargetBodyId(command);
        result.worldGeneration = commandWorldGeneration(command);
        result.skeletonGeneration = commandSkeletonGeneration(command);
        result.providerGeneration = commandProviderGeneration(command);
        return result;
    }

    void storeInteractionResultLocked(const RockProviderInteractionCommandResultV1& result)
    {
        for (auto& slot : s_interactionResults) {
            if (slot.active && slot.result.ownerToken == result.ownerToken && slot.result.commandId == result.commandId) {
                slot.result = result;
                return;
            }
        }

        s_interactionResults[s_nextInteractionResultSlot] = InteractionCommandResultSlot{
            .active = true,
            .result = result,
        };
        s_nextInteractionResultSlot = (s_nextInteractionResultSlot + 1) % s_interactionResults.size();
    }

    void completeInteractionCommandLocked(
        const QueuedInteractionCommandV1& command,
        RockProviderInteractionCommandStateV1 state,
        RockProviderInteractionFailureV1 failure)
    {
        storeInteractionResultLocked(makeCommandResult(command, state, failure));
    }

    void clearInteractionCommandsForOwnerLocked(std::uint64_t ownerToken, RockProviderInteractionFailureV1 failure)
    {
        if (ownerToken == 0) {
            return;
        }

        for (auto& slot : s_interactionCommands) {
            if (slot.active && slot.command.ownerToken == ownerToken) {
                completeInteractionCommandLocked(slot.command, RockProviderInteractionCommandStateV1::Cancelled, failure);
                slot = {};
            }
        }

        for (auto& slot : s_interactionResults) {
            if (slot.active && slot.result.ownerToken == ownerToken) {
                slot = {};
            }
        }
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
            std::scoped_lock lock(s_consumerMutex, s_interactionCommandMutex, s_handInputSuppressionMutex, s_weaponPartMutex);
            auto* slot = findConsumerSlotLocked(ownerToken);
            if (!slot) {
                return RockProviderResultV1::OwnerNotRegistered;
            }
            *slot = {};
            clearInteractionCommandsForOwnerLocked(ownerToken, RockProviderInteractionFailureV1::OwnerNotRegistered);
            clearHandInputSuppressionsForOwnerLocked(ownerToken, RockProviderHand::None);
            clearWeaponPartTargetsForOwnerLocked(ownerToken);
            clearWeaponPartDrivesForOwnerLocked(ownerToken);
        }

        {
            std::scoped_lock lock(s_externalBodyMutex);
            s_externalBodies.clearOwner(ownerToken);
        }

        if (s_offhandReservationOwner.load(std::memory_order_acquire) == ownerToken) {
            s_offhandReservation.store(static_cast<std::uint32_t>(RockProviderOffhandReservation::Normal), std::memory_order_release);
            s_offhandReservationOwner.store(0, std::memory_order_release);
        }

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
        outLimits->maxInteractionCommands = ROCK_PROVIDER_MAX_INTERACTION_COMMANDS_V1;
        outLimits->maxCompletedInteractionCommands = ROCK_PROVIDER_MAX_COMPLETED_INTERACTION_COMMANDS_V1;
        outLimits->providerApiByteSize = static_cast<std::uint32_t>(sizeof(RockProviderApi));
        return true;
    }

    bool hasInteractionTargetIdentity(std::uintptr_t targetRefr, std::uint32_t targetFormId, std::uint32_t targetBodyId)
    {
        return targetRefr != 0 || targetFormId != 0 || targetBodyId != kProviderInvalidBodyId;
    }

    bool isFiniteVector3(const float values[3])
    {
        return std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]);
    }

    RockProviderResultV1 validateInteractionCommandProviderReady()
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return RockProviderResultV1::NotReady;
        }
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 enqueueInteractionCommand(QueuedInteractionCommandV1 command, std::uint64_t* outCommandId)
    {
        if (!outCommandId) {
            return RockProviderResultV1::InvalidArgument;
        }

        const auto providerReadyResult = validateInteractionCommandProviderReady();
        if (providerReadyResult != RockProviderResultV1::Ok) {
            return providerReadyResult;
        }

        std::scoped_lock lock(s_consumerMutex, s_interactionCommandMutex);
        const auto ownerResult = validateInteractionCommandOwnerLocked(command.ownerToken);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        for (auto& slot : s_interactionCommands) {
            if (!slot.active) {
                command.commandId = nextInteractionCommandId();
                slot = InteractionCommandSlot{
                    .active = true,
                    .command = command,
                };
                *outCommandId = command.commandId;
                storeInteractionResultLocked(makeCommandResult(command, RockProviderInteractionCommandStateV1::Queued, RockProviderInteractionFailureV1::None));
                return RockProviderResultV1::RequestQueued;
            }
        }

        return RockProviderResultV1::CapacityFull;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRequestForceGrabV1(
        std::uint64_t ownerToken,
        const RockProviderForceGrabRequestV1* request,
        std::uint64_t* outCommandId)
    {
        if (!request || !outCommandId || ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outCommandId = 0;

        if (request->size != sizeof(RockProviderForceGrabRequestV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (request->hand != RockProviderHand::Right && request->hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }
        if (request->targetFormId == 0 || (request->flags & ~kImplementedForceGrabFlagsV1) != 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!std::isfinite(request->maxDistanceGame)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(RockProviderForceGrabFlagV1::UsePreferredGrabPointGame)) != 0 &&
            !isFiniteVector3(request->preferredGrabPointGame)) {
            return RockProviderResultV1::InvalidArgument;
        }

        QueuedInteractionCommandV1 command{};
        command.ownerToken = ownerToken;
        command.kind = RockProviderInteractionCommandKindV1::ForceGrab;
        command.forceGrab = *request;
        return enqueueInteractionCommand(command, outCommandId);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRequestForceReleaseV1(
        std::uint64_t ownerToken,
        const RockProviderForceReleaseRequestV1* request,
        std::uint64_t* outCommandId)
    {
        if (!request || !outCommandId || ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outCommandId = 0;

        if (request->size != sizeof(RockProviderForceReleaseRequestV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (request->hand != RockProviderHand::Right && request->hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }
        if ((request->flags & ~kImplementedForceReleaseFlagsV1) != 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::RequireMatchingTarget)) != 0 &&
            !hasInteractionTargetIdentity(request->targetRefr, request->targetFormId, request->targetBodyId)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::UseVelocityHavok)) != 0 &&
            (!isFiniteVector3(request->linearVelocityHavok) || !isFiniteVector3(request->angularVelocityRadiansPerSecond))) {
            return RockProviderResultV1::InvalidArgument;
        }

        QueuedInteractionCommandV1 command{};
        command.ownerToken = ownerToken;
        command.kind = RockProviderInteractionCommandKindV1::ForceRelease;
        command.forceRelease = *request;
        return enqueueInteractionCommand(command, outCommandId);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRequestThrownDropV1(
        std::uint64_t ownerToken,
        const RockProviderThrownDropRequestV1* request,
        std::uint64_t* outCommandId)
    {
        if (!request || !outCommandId || ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outCommandId = 0;

        if (request->size != sizeof(RockProviderThrownDropRequestV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (request->hand != RockProviderHand::Right && request->hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }
        if ((request->flags & ~kImplementedThrownDropFlagsV1) != 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(RockProviderThrownDropFlagV1::RequireMatchingTarget)) != 0 &&
            !hasInteractionTargetIdentity(request->targetRefr, request->targetFormId, request->targetBodyId)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(RockProviderThrownDropFlagV1::UseVelocityHavok)) != 0 &&
            (!isFiniteVector3(request->linearVelocityHavok) || !isFiniteVector3(request->angularVelocityRadiansPerSecond))) {
            return RockProviderResultV1::InvalidArgument;
        }

        QueuedInteractionCommandV1 command{};
        command.ownerToken = ownerToken;
        command.kind = RockProviderInteractionCommandKindV1::ThrownDrop;
        command.thrownDrop = *request;
        return enqueueInteractionCommand(command, outCommandId);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetInteractionCommandResultV1(
        std::uint64_t ownerToken,
        std::uint64_t commandId,
        RockProviderInteractionCommandResultV1* outResult)
    {
        if (!outResult || ownerToken == 0 || commandId == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (outResult->size != sizeof(RockProviderInteractionCommandResultV1)) {
            return RockProviderResultV1::InvalidSize;
        }

        std::scoped_lock lock(s_consumerMutex, s_interactionCommandMutex);
        if (!findConsumerSlotLocked(ownerToken)) {
            return RockProviderResultV1::OwnerNotRegistered;
        }

        for (const auto& slot : s_interactionResults) {
            if (slot.active && slot.result.ownerToken == ownerToken && slot.result.commandId == commandId) {
                *outResult = slot.result;
                return RockProviderResultV1::Ok;
            }
        }

        for (const auto& slot : s_interactionCommands) {
            if (slot.active && slot.command.ownerToken == ownerToken && slot.command.commandId == commandId) {
                *outResult = makeCommandResult(slot.command, RockProviderInteractionCommandStateV1::Queued, RockProviderInteractionFailureV1::None);
                return RockProviderResultV1::Ok;
            }
        }

        return RockProviderResultV1::RequestNotFound;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetHandInputSuppressionV1(
        std::uint64_t ownerToken,
        const RockProviderHandInputSuppressionRequestV1* request)
    {
        if (!request || ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (request->size != sizeof(RockProviderHandInputSuppressionRequestV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (request->hand != RockProviderHand::Right && request->hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }
        if (request->flags == 0 || (request->flags & ~kImplementedHandInputSuppressionFlagsV1) != 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (request->leaseFrames == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        const auto generationResult = validateGenerationGuards(
            request->worldGeneration,
            request->skeletonGeneration,
            request->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }

        const auto leaseFrames = (std::min)(request->leaseFrames, ROCK_PROVIDER_MAX_HAND_INPUT_SUPPRESSION_LEASE_FRAMES_V1);
        const auto frameIndex = currentProviderFrameIndex();
        const auto expiresAfterFrame = frameIndex + leaseFrames;

        std::scoped_lock lock(s_consumerMutex, s_handInputSuppressionMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::HandInputSuppression);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        pruneExpiredHandInputSuppressionsLocked(frameIndex);
        for (auto& slot : s_handInputSuppressions) {
            if (slot.active && slot.ownerToken == ownerToken && slot.hand == request->hand) {
                slot.flags = request->flags;
                slot.expiresAfterFrame = expiresAfterFrame;
                return RockProviderResultV1::Ok;
            }
        }

        for (auto& slot : s_handInputSuppressions) {
            if (!slot.active) {
                slot = HandInputSuppressionSlot{
                    .active = true,
                    .ownerToken = ownerToken,
                    .hand = request->hand,
                    .flags = request->flags,
                    .expiresAfterFrame = expiresAfterFrame,
                };
                return RockProviderResultV1::Ok;
            }
        }

        return RockProviderResultV1::CapacityFull;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearHandInputSuppressionV1(
        std::uint64_t ownerToken,
        RockProviderHand hand)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (hand != RockProviderHand::None && hand != RockProviderHand::Right && hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }

        std::scoped_lock lock(s_consumerMutex, s_handInputSuppressionMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::HandInputSuppression);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        clearHandInputSuppressionsForOwnerLocked(ownerToken, hand);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetWeaponPartTargetsV1(
        std::uint64_t ownerToken,
        const RockProviderWeaponPartTargetV1* targets,
        std::uint32_t targetCount)
    {
        if (ownerToken == 0 || (targetCount > 0 && !targets)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (targetCount > ROCK_PROVIDER_MAX_WEAPON_PART_TARGETS_V1) {
            return RockProviderResultV1::CapacityFull;
        }

        std::scoped_lock lock(s_consumerMutex, s_weaponPartMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::WeaponPartInteraction);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        for (std::uint32_t i = 0; i < targetCount; ++i) {
            const auto& target = targets[i];
            if (target.size != sizeof(RockProviderWeaponPartTargetV1)) {
                return RockProviderResultV1::InvalidSize;
            }
            if (target.version == 0 || target.version > ROCK_PROVIDER_API_VERSION) {
                return RockProviderResultV1::UnsupportedVersion;
            }
            if (!isValidWeaponPartGrabMode(target.grabMode) ||
                !hasValidWeaponPartMatcher(target.flags, target.bodyId, target.sourceRoot, target.sourceName) ||
                !hasValidWeaponPartTargetSemantics(target)) {
                return RockProviderResultV1::InvalidArgument;
            }
        }

        if (targetCount > availableWeaponPartTargetSlotsForOwnerLocked(ownerToken)) {
            return RockProviderResultV1::CapacityFull;
        }

        clearWeaponPartTargetsForOwnerLocked(ownerToken);
        for (std::uint32_t i = 0; i < targetCount; ++i) {
            bool stored = false;
            for (auto& slot : s_weaponPartTargets) {
                if (!slot.active) {
                    slot.active = true;
                    slot.ownerToken = ownerToken;
                    slot.target = targets[i];
                    slot.target.sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME - 1] = '\0';
                    stored = true;
                    break;
                }
            }
            if (!stored) {
                clearWeaponPartTargetsForOwnerLocked(ownerToken);
                return RockProviderResultV1::CapacityFull;
            }
        }
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearWeaponPartTargetsV1(std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        std::scoped_lock lock(s_consumerMutex, s_weaponPartMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::WeaponPartInteraction);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        clearWeaponPartTargetsForOwnerLocked(ownerToken);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetWeaponPartDriveTargetsV1(
        std::uint64_t ownerToken,
        const RockProviderWeaponPartDriveTargetV1* targets,
        std::uint32_t targetCount)
    {
        if (ownerToken == 0 || (targetCount > 0 && !targets)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (targetCount > ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1) {
            return RockProviderResultV1::CapacityFull;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_consumerMutex, s_weaponPartMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::WeaponPartInteraction);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        for (std::uint32_t i = 0; i < targetCount; ++i) {
            const auto& target = targets[i];
            if (target.size != sizeof(RockProviderWeaponPartDriveTargetV1)) {
                return RockProviderResultV1::InvalidSize;
            }
            if (target.version == 0 || target.version > ROCK_PROVIDER_API_VERSION) {
                return RockProviderResultV1::UnsupportedVersion;
            }
            if (!isValidWeaponPartDriveSpace(target.driveSpace) ||
                target.leaseFrames == 0 ||
                !isFiniteProviderTransform(target.targetTransform) ||
                !hasConcreteWeaponPartDriveMatcher(target.flags, target.bodyId, target.sourceRoot, target.sourceName)) {
                return RockProviderResultV1::InvalidArgument;
            }
        }

        pruneExpiredWeaponPartDrivesLocked(frameIndex);
        if (targetCount > availableWeaponPartDriveSlotsForOwnerLocked(ownerToken)) {
            return RockProviderResultV1::CapacityFull;
        }
        clearWeaponPartDrivesForOwnerLocked(ownerToken);
        for (std::uint32_t i = 0; i < targetCount; ++i) {
            const auto leaseFrames = (std::min)(targets[i].leaseFrames, ROCK_PROVIDER_MAX_WEAPON_PART_DRIVE_LEASE_FRAMES_V1);
            const auto expiresAfterFrame = frameIndex + leaseFrames;
            bool stored = false;
            for (auto& slot : s_weaponPartDrives) {
                if (!slot.active) {
                    slot.active = true;
                    slot.ownerToken = ownerToken;
                    slot.expiresAfterFrame = expiresAfterFrame;
                    slot.target = targets[i];
                    slot.target.sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME - 1] = '\0';
                    stored = true;
                    break;
                }
            }
            if (!stored) {
                clearWeaponPartDrivesForOwnerLocked(ownerToken);
                return RockProviderResultV1::CapacityFull;
            }
        }
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearWeaponPartDriveTargetsV1(std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        std::scoped_lock lock(s_consumerMutex, s_weaponPartMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::WeaponPartInteraction);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        clearWeaponPartDrivesForOwnerLocked(ownerToken);
        return RockProviderResultV1::Ok;
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

    bool ROCK_PROVIDER_CALL apiQueryEquippedWeaponClassificationV1(RockProviderWeaponClassificationV1* outResult)
    {
        if (!outResult || outResult->size != sizeof(RockProviderWeaponClassificationV1)) {
            return false;
        }

        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return false;
        }

        return pi->queryProviderEquippedWeaponClassificationV1(*outResult);
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
        .clearExternalBodies = &apiClearExternalBodies,
        .setOffhandInteractionReservation = &apiSetOffhandInteractionReservation,
        .registerExternalBodiesV1 = &apiRegisterExternalBodiesV1,
        .getWeaponEvidenceDetailCountV1 = &apiGetWeaponEvidenceDetailCountV1,
        .copyWeaponEvidenceDetailsV1 = &apiCopyWeaponEvidenceDetailsV1,
        .getWeaponEvidenceDetailPointCountV1 = &apiGetWeaponEvidenceDetailPointCountV1,
        .copyWeaponEvidenceDetailPointsV1 = &apiCopyWeaponEvidenceDetailPointsV1,
        .getBodyContactSnapshotV1 = &apiGetBodyContactSnapshotV1,
        .getPrimaryHandV1 = &apiGetPrimaryHandV1,
        .getOffhandHandV1 = &apiGetOffhandHandV1,
        .getHandFrameV1 = &apiGetHandFrameV1,
        .registerConsumerV1 = &apiRegisterConsumerV1,
        .unregisterConsumerV1 = &apiUnregisterConsumerV1,
        .getGrantedCapabilitiesV1 = &apiGetGrantedCapabilitiesV1,
        .getProviderLimitsV1 = &apiGetProviderLimitsV1,
        .getExternalContactSnapshotForOwnerV1 = &apiGetExternalContactSnapshotForOwnerV1,
        .requestForceGrabV1 = &apiRequestForceGrabV1,
        .getInteractionCommandResultV1 = &apiGetInteractionCommandResultV1,
        .requestForceReleaseV1 = &apiRequestForceReleaseV1,
        .requestThrownDropV1 = &apiRequestThrownDropV1,
        .setHandInputSuppressionV1 = &apiSetHandInputSuppressionV1,
        .clearHandInputSuppressionV1 = &apiClearHandInputSuppressionV1,
        .setWeaponPartTargetsV1 = &apiSetWeaponPartTargetsV1,
        .clearWeaponPartTargetsV1 = &apiClearWeaponPartTargetsV1,
        .setWeaponPartDriveTargetsV1 = &apiSetWeaponPartDriveTargetsV1,
        .clearWeaponPartDriveTargetsV1 = &apiClearWeaponPartDriveTargetsV1,
        .queryEquippedWeaponClassificationV1 = &apiQueryEquippedWeaponClassificationV1,
        .getWeaponPartGripStateV1 = &apiGetWeaponPartGripStateV1,
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

        std::array<RockProviderWeaponPartGripStateV1, 2> partGripStates{};
        pi.fillProviderWeaponPartGripStates(partGripStates);

        {
            std::scoped_lock lock(s_snapshotMutex);
            s_lastSnapshot = snapshot;
            s_hasSnapshot = true;
            s_lastPartGripStates = partGripStates;
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
        clearInteractionCommandsForProviderLossV1(RockProviderInteractionFailureV1::ProviderNotReady);
        {
            std::scoped_lock lock(s_handInputSuppressionMutex);
            s_handInputSuppressions = {};
        }
        {
            std::scoped_lock lock(s_weaponPartMutex);
            s_weaponPartTargets = {};
            s_weaponPartDrives = {};
        }
        s_offhandReservation.store(static_cast<std::uint32_t>(RockProviderOffhandReservation::Normal), std::memory_order_release);
        s_offhandReservationOwner.store(0, std::memory_order_release);
    }

    bool dequeueInteractionCommandV1(QueuedInteractionCommandV1& outCommand)
    {
        std::scoped_lock lock(s_interactionCommandMutex);
        InteractionCommandSlot* oldestSlot = nullptr;
        for (auto& slot : s_interactionCommands) {
            if (slot.active && (!oldestSlot || slot.command.commandId < oldestSlot->command.commandId)) {
                oldestSlot = &slot;
            }
        }

        if (oldestSlot) {
            outCommand = oldestSlot->command;
            *oldestSlot = {};
            return true;
        }
        return false;
    }

    void completeInteractionCommandV1(const RockProviderInteractionCommandResultV1& result)
    {
        std::scoped_lock lock(s_interactionCommandMutex);
        storeInteractionResultLocked(result);
    }

    void clearInteractionCommandsForProviderLossV1(RockProviderInteractionFailureV1 failure)
    {
        std::scoped_lock lock(s_interactionCommandMutex);
        for (auto& slot : s_interactionCommands) {
            if (slot.active) {
                completeInteractionCommandLocked(slot.command, RockProviderInteractionCommandStateV1::Cancelled, failure);
                slot = {};
            }
        }
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

    std::uint32_t currentHandInputSuppressionFlagsV1(RockProviderHand hand)
    {
        if (hand != RockProviderHand::Right && hand != RockProviderHand::Left) {
            return 0;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::uint32_t flags = 0;
        std::scoped_lock lock(s_handInputSuppressionMutex);
        pruneExpiredHandInputSuppressionsLocked(frameIndex);
        for (const auto& slot : s_handInputSuppressions) {
            if (slot.active && slot.hand == hand) {
                flags |= slot.flags;
            }
        }
        return flags;
    }

    bool resolveWeaponPartTargetV1(
        const RockProviderWeaponPartTargetQueryV1& query,
        RockProviderWeaponPartTargetResolutionV1& outResolution)
    {
        outResolution = {};
        std::array<weapon_part_runtime::Target, ROCK_PROVIDER_MAX_WEAPON_PART_TARGETS_V1> runtimeTargets{};
        {
            std::scoped_lock lock(s_weaponPartMutex);
            for (std::size_t i = 0; i < s_weaponPartTargets.size(); ++i) {
                runtimeTargets[i] = toRuntimeTarget(s_weaponPartTargets[i]);
            }
        }

        const weapon_part_runtime::Contact contact{
            .weaponGenerationKey = query.weaponGenerationKey,
            .bodyId = query.bodyId,
            .sourceRoot = query.sourceRoot,
            .sourceName = std::string_view(query.sourceName, boundedStringLength(query.sourceName, ROCK_PROVIDER_MAX_EVIDENCE_NAME)),
            .partKind = static_cast<WeaponPartKind>(query.partKind),
            .reloadRole = static_cast<WeaponReloadRole>(query.reloadRole),
            .supportRole = static_cast<WeaponSupportGripRole>(query.supportRole),
            .socketRole = static_cast<WeaponSocketRole>(query.socketRole),
            .actionRole = static_cast<WeaponActionRole>(query.actionRole),
        };
        const auto resolution = weapon_part_runtime::resolveTarget(runtimeTargets, contact);
        outResolution.whitelistActive = resolution.whitelistActive ? 1u : 0u;
        outResolution.matched = resolution.matched ? 1u : 0u;
        outResolution.grabMode = fromRuntimeGrabMode(resolution.grabMode);
        outResolution.groupId = resolution.groupId;
        outResolution.ownerToken = resolution.ownerToken;
        return resolution.whitelistActive;
    }

    std::uint32_t copyWeaponPartDriveTargetsV1(
        RockProviderWeaponPartDriveTargetV1* outTargets,
        std::uint32_t maxTargets)
    {
        if (!outTargets || maxTargets == 0) {
            return 0;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::uint32_t copied = 0;
        std::scoped_lock lock(s_weaponPartMutex);
        pruneExpiredWeaponPartDrivesLocked(frameIndex);
        for (const auto& slot : s_weaponPartDrives) {
            if (!slot.active) {
                continue;
            }
            if (copied >= maxTargets) {
                break;
            }
            outTargets[copied] = slot.target;
            outTargets[copied].sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME - 1] = '\0';
            ++copied;
        }
        return copied;
    }

    std::uint32_t currentExternalBodyCount()
    {
        std::scoped_lock lock(s_externalBodyMutex);
        return s_externalBodies.bodyCount();
    }
}
