#include "physics-interaction/grab/NearbyGrabDamping.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/object/ObjectDetection.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsShapeCast.h"

#include "RE/Havok/hknpAllHitsCollector.h"
#include "RE/Havok/hknpBodyCinfo.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Havok/hknpWorld.h"
#include "REL/Relocation.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <condition_variable>
#include <cmath>
#include <cstring>
#include <mutex>
#include <unordered_set>
#include <utility>
#include <windows.h>

namespace rock::nearby_grab_damping
{
    namespace
    {
        /*
         * Nearby damping mutates native hknp motion-property IDs, and the verified
         * FO4VR add/set paths can notify world systems. The registry therefore
         * reserves ownership under a ROCK mutex, releases it for native calls, and
         * commits or retries afterward so body churn cannot strand damped IDs.
         */
        inline constexpr std::uint16_t kInvalidMotionPropertiesId = INVALID_MOTION_PROPERTIES_ID;
        inline constexpr std::size_t kMaxCachedDampedProperties = 128;

        struct MotionPropertiesRecord
        {
            std::array<std::uint8_t, offsets::kMotionProperties_RecordSize> bytes{};
        };
        static_assert(sizeof(MotionPropertiesRecord) == offsets::kMotionProperties_RecordSize);

        struct BodyMotionState
        {
            std::uint32_t motionId = 0;
            std::uint16_t motionPropertiesId = kInvalidMotionPropertiesId;
        };

        struct MotionDampingLeaseEntry
        {
            RE::hknpWorld* world = nullptr;
            std::uint32_t motionId = 0;
            std::uint32_t representativeBodyId = INVALID_BODY_ID;
            std::uint16_t originalMotionPropertiesId = kInvalidMotionPropertiesId;
            std::uint16_t dampedMotionPropertiesId = kInvalidMotionPropertiesId;
            float originalLinearDamping = 0.0f;
            float originalAngularDamping = 0.0f;
            std::vector<std::uintptr_t> ownerTokens;
            bool restoreInProgress = false;
        };

        struct DampedPropertyCacheEntry
        {
            RE::hknpWorld* world = nullptr;
            MotionPropertiesRecord record{};
            std::uint16_t motionPropertiesId = kInvalidMotionPropertiesId;
            bool creationInProgress = false;
        };

        struct MotionDampingInFlightEntry
        {
            RE::hknpWorld* world = nullptr;
            std::uint32_t motionId = 0;
        };

        struct LeaseReleaseResult
        {
            bool released = false;
            bool finalLease = false;
            bool restoredOriginal = false;
            bool alreadyOriginal = false;
            bool skippedExternalChange = false;
            bool restoreFailed = false;
            bool motionGone = false;
        };

        struct LeaseRestoreAttempt
        {
            LeaseReleaseResult result{};
            bool completed = false;
        };

        std::mutex g_motionDampingMutex;
        std::condition_variable g_motionDampingChanged;
        std::vector<MotionDampingLeaseEntry> g_motionDampingLeases;
        std::vector<MotionDampingInFlightEntry> g_motionDampingInFlight;
        std::vector<DampedPropertyCacheEntry> g_dampedPropertyCache;
        std::atomic<std::uintptr_t> g_nextLeaseToken{ 1 };

        bool guardedCopyFromMemory(const void* source, void* target, std::size_t byteCount)
        {
            return native_memory::guardedCopyFromMemory(source, target, byteCount);
        }

        template <class T>
        bool tryReadValue(const void* address, T& out)
        {
            return native_memory::tryReadValue(reinterpret_cast<const T*>(address), out);
        }

        float readRecordFloat(const MotionPropertiesRecord& record, std::uintptr_t offset)
        {
            float value = 0.0f;
            if (offset + sizeof(float) <= record.bytes.size()) {
                std::memcpy(&value, record.bytes.data() + offset, sizeof(value));
            }
            return value;
        }

        void writeRecordFloat(MotionPropertiesRecord& record, std::uintptr_t offset, float value)
        {
            if (offset + sizeof(float) <= record.bytes.size()) {
                std::memcpy(record.bytes.data() + offset, &value, sizeof(value));
            }
        }

        std::uintptr_t nextLeaseToken()
        {
            auto token = g_nextLeaseToken.fetch_add(1, std::memory_order_relaxed);
            if (token == 0) {
                token = g_nextLeaseToken.fetch_add(1, std::memory_order_relaxed);
            }
            return token;
        }

        bool containsBodyId(const std::vector<std::uint32_t>& bodyIds, std::uint32_t bodyId)
        {
            return std::find(bodyIds.begin(), bodyIds.end(), bodyId) != bodyIds.end();
        }

        bool leaseContainsOwner(const MotionDampingLeaseEntry& lease, std::uintptr_t ownerToken)
        {
            return std::find(lease.ownerTokens.begin(), lease.ownerTokens.end(), ownerToken) != lease.ownerTokens.end();
        }

        auto findLeaseLocked(RE::hknpWorld* world, std::uint32_t motionId)
        {
            return std::find_if(g_motionDampingLeases.begin(), g_motionDampingLeases.end(), [&](const auto& lease) {
                return lease.world == world && lease.motionId == motionId;
            });
        }

        auto findInFlightMotionLocked(RE::hknpWorld* world, std::uint32_t motionId)
        {
            return std::find_if(g_motionDampingInFlight.begin(), g_motionDampingInFlight.end(), [&](const auto& entry) {
                return entry.world == world && entry.motionId == motionId;
            });
        }

        void fillSavedMotionFromLease(const MotionDampingLeaseEntry& lease,
            std::uint32_t representativeBodyId,
            std::uintptr_t ownerToken,
            SavedNearbyMotionDamping& outSaved)
        {
            outSaved = SavedNearbyMotionDamping{
                .representativeBodyId = representativeBodyId,
                .motionId = lease.motionId,
                .originalLinearDamping = lease.originalLinearDamping,
                .originalAngularDamping = lease.originalAngularDamping,
                .originalMotionPropertiesId = lease.originalMotionPropertiesId,
                .dampedMotionPropertiesId = lease.dampedMotionPropertiesId,
                .leaseToken = ownerToken,
                .active = true,
            };
        }

        bool reserveMotionDampingWriteOrAttachExisting(RE::hknpWorld* world,
            std::uint32_t bodyId,
            std::uint32_t motionId,
            std::uintptr_t ownerToken,
            SavedNearbyMotionDamping& outSaved,
            bool& outReservedNativeWrite)
        {
            outReservedNativeWrite = false;
            std::unique_lock lock(g_motionDampingMutex);
            for (;;) {
                auto leaseIt = findLeaseLocked(world, motionId);
                if (leaseIt != g_motionDampingLeases.end()) {
                    if (leaseIt->restoreInProgress) {
                        g_motionDampingChanged.wait(lock);
                        continue;
                    }

                    leaseIt->representativeBodyId = bodyId;
                    if (!leaseContainsOwner(*leaseIt, ownerToken)) {
                        leaseIt->ownerTokens.push_back(ownerToken);
                    }
                    fillSavedMotionFromLease(*leaseIt, bodyId, ownerToken, outSaved);
                    return true;
                }

                if (findInFlightMotionLocked(world, motionId) == g_motionDampingInFlight.end()) {
                    g_motionDampingInFlight.push_back(MotionDampingInFlightEntry{
                        .world = world,
                        .motionId = motionId,
                    });
                    outReservedNativeWrite = true;
                    return true;
                }

                g_motionDampingChanged.wait(lock);
            }
        }

        void clearInFlightMotionLocked(RE::hknpWorld* world, std::uint32_t motionId)
        {
            g_motionDampingInFlight.erase(
                std::remove_if(g_motionDampingInFlight.begin(),
                    g_motionDampingInFlight.end(),
                    [&](const MotionDampingInFlightEntry& entry) {
                        return entry.world == world && entry.motionId == motionId;
                    }),
                g_motionDampingInFlight.end());
        }

        void abortMotionDampingWrite(RE::hknpWorld* world, std::uint32_t motionId)
        {
            {
                std::lock_guard lock(g_motionDampingMutex);
                clearInFlightMotionLocked(world, motionId);
            }
            g_motionDampingChanged.notify_all();
        }

        void commitMotionDampingWrite(MotionDampingLeaseEntry lease)
        {
            {
                std::lock_guard lock(g_motionDampingMutex);
                clearInFlightMotionLocked(lease.world, lease.motionId);
                g_motionDampingLeases.push_back(std::move(lease));
            }
            g_motionDampingChanged.notify_all();
        }

        bool worldHasActiveLeaseLocked(RE::hknpWorld* world)
        {
            return std::any_of(g_motionDampingLeases.begin(), g_motionDampingLeases.end(), [&](const auto& lease) {
                return lease.world == world;
            });
        }

        void pruneDampedPropertyCacheLocked()
        {
            if (g_dampedPropertyCache.size() <= kMaxCachedDampedProperties) {
                return;
            }

            g_dampedPropertyCache.erase(
                std::remove_if(g_dampedPropertyCache.begin(),
                    g_dampedPropertyCache.end(),
                    [](const DampedPropertyCacheEntry& entry) {
                        return !entry.creationInProgress && !worldHasActiveLeaseLocked(entry.world);
                    }),
                g_dampedPropertyCache.end());
        }

        void* getMotionPropertiesLibrary(RE::hknpWorld* world)
        {
            if (!world) {
                return nullptr;
            }

            void* library = nullptr;
            const auto worldAddress = reinterpret_cast<std::uintptr_t>(world);
            if (!tryReadValue(reinterpret_cast<const void* const*>(worldAddress + offsets::kHknpWorld_MotionPropertiesLibraryPtr), library)) {
                return nullptr;
            }
            return library;
        }

        bool tryReadMotionPropertiesRecord(RE::hknpWorld* world, std::uint16_t motionPropertiesId, MotionPropertiesRecord& outRecord)
        {
            auto* library = getMotionPropertiesLibrary(world);
            if (!library || motionPropertiesId == kInvalidMotionPropertiesId) {
                return false;
            }

            const auto libraryAddress = reinterpret_cast<std::uintptr_t>(library);
            void* entries = nullptr;
            std::int32_t count = 0;
            if (!tryReadValue(reinterpret_cast<const void* const*>(libraryAddress + offsets::kMotionPropertiesLibrary_Entries), entries) ||
                !tryReadValue(reinterpret_cast<const std::int32_t*>(libraryAddress + offsets::kMotionPropertiesLibrary_Count), count)) {
                return false;
            }

            if (!entries || count <= 0 || count > 4096 || motionPropertiesId >= static_cast<std::uint16_t>(count)) {
                return false;
            }

            const auto entryAddress = reinterpret_cast<std::uintptr_t>(entries) +
                                      static_cast<std::uintptr_t>(motionPropertiesId) * offsets::kMotionProperties_RecordSize;
            return guardedCopyFromMemory(reinterpret_cast<const void*>(entryAddress), &outRecord, sizeof(outRecord));
        }

        bool addMotionPropertiesEntry(RE::hknpWorld* world, const MotionPropertiesRecord& record, std::uint16_t& outMotionPropertiesId)
        {
            outMotionPropertiesId = kInvalidMotionPropertiesId;

            auto* library = getMotionPropertiesLibrary(world);
            if (!library) {
                return false;
            }

            using AddEntry_t = std::uint16_t* (*)(void*, std::uint16_t*, const MotionPropertiesRecord*);
            static REL::Relocation<AddEntry_t> addEntry{ REL::Offset(offsets::kFunc_MotionPropertiesLibrary_AddEntry) };

            std::uint16_t addedId = kInvalidMotionPropertiesId;
            auto* result = addEntry(library, &addedId, &record);
            if (!result || *result == kInvalidMotionPropertiesId) {
                return false;
            }

            outMotionPropertiesId = *result;
            return true;
        }

        bool tryReuseCachedDampedPropertyLocked(RE::hknpWorld* world, const MotionPropertiesRecord& record, std::uint16_t& outMotionPropertiesId)
        {
            for (auto it = g_dampedPropertyCache.begin(); it != g_dampedPropertyCache.end();) {
                if (it->creationInProgress || it->world != world || it->record.bytes != record.bytes) {
                    ++it;
                    continue;
                }

                MotionPropertiesRecord liveRecord{};
                if (tryReadMotionPropertiesRecord(world, it->motionPropertiesId, liveRecord) && liveRecord.bytes == record.bytes) {
                    outMotionPropertiesId = it->motionPropertiesId;
                    return true;
                }

                it = g_dampedPropertyCache.erase(it);
            }

            return false;
        }

        auto findInProgressDampedPropertyLocked(RE::hknpWorld* world, const MotionPropertiesRecord& record)
        {
            return std::find_if(g_dampedPropertyCache.begin(), g_dampedPropertyCache.end(), [&](const DampedPropertyCacheEntry& entry) {
                return entry.creationInProgress && entry.world == world && entry.record.bytes == record.bytes;
            });
        }

        bool getOrCreateDampedPropertyId(RE::hknpWorld* world, const MotionPropertiesRecord& record, std::uint16_t& outMotionPropertiesId)
        {
            outMotionPropertiesId = kInvalidMotionPropertiesId;

            {
                std::unique_lock lock(g_motionDampingMutex);
                for (;;) {
                    if (tryReuseCachedDampedPropertyLocked(world, record, outMotionPropertiesId)) {
                        return true;
                    }

                    if (findInProgressDampedPropertyLocked(world, record) == g_dampedPropertyCache.end()) {
                        g_dampedPropertyCache.push_back(DampedPropertyCacheEntry{
                            .world = world,
                            .record = record,
                            .motionPropertiesId = kInvalidMotionPropertiesId,
                            .creationInProgress = true,
                        });
                        break;
                    }

                    g_motionDampingChanged.wait(lock);
                }
            }

            std::uint16_t addedMotionPropertiesId = kInvalidMotionPropertiesId;
            bool created = addMotionPropertiesEntry(world, record, addedMotionPropertiesId);
            if (created) {
                MotionPropertiesRecord liveRecord{};
                created = tryReadMotionPropertiesRecord(world, addedMotionPropertiesId, liveRecord) && liveRecord.bytes == record.bytes;
            }

            {
                std::lock_guard lock(g_motionDampingMutex);
                auto pendingIt = findInProgressDampedPropertyLocked(world, record);
                if (pendingIt != g_dampedPropertyCache.end()) {
                    if (created) {
                        pendingIt->motionPropertiesId = addedMotionPropertiesId;
                        pendingIt->creationInProgress = false;
                    } else {
                        g_dampedPropertyCache.erase(pendingIt);
                    }
                }

                if (created) {
                    pruneDampedPropertyCacheLocked();
                }
            }
            g_motionDampingChanged.notify_all();

            if (!created) {
                return false;
            }

            outMotionPropertiesId = addedMotionPropertiesId;
            return true;
        }

        bool tryReadCurrentBodyMotionState(RE::hknpWorld* world, std::uint32_t bodyId, BodyMotionState& outState)
        {
            outState = {};
            if (!world || bodyId == INVALID_BODY_ID) {
                return false;
            }

            auto* body = havok_runtime::getBody(world, RE::hknpBodyId{ bodyId });
            if (!body || body->motionIndex == 0) {
                return false;
            }

            auto* motion = havok_runtime::getMotion(world, body->motionIndex);
            if (!motion) {
                return false;
            }

            outState.motionId = body->motionIndex;
            outState.motionPropertiesId = motion->motionPropertiesId;
            return outState.motionPropertiesId != kInvalidMotionPropertiesId;
        }

        bool tryReadWorldBodyHighWaterMark(RE::hknpWorld* world, std::uint32_t& outHighWaterMark)
        {
            outHighWaterMark = 0;
            if (!world) {
                return false;
            }

            const auto worldAddress = reinterpret_cast<std::uintptr_t>(world);
            if (!tryReadValue(reinterpret_cast<const std::uint32_t*>(worldAddress + havok_runtime::kHknpWorldBodyHighWaterMarkOffset), outHighWaterMark)) {
                return false;
            }

            return outHighWaterMark <= 0x000F'FFFF;
        }

        bool tryFindLiveBodyForMotion(RE::hknpWorld* world,
            std::uint32_t preferredBodyId,
            std::uint32_t motionId,
            std::uint32_t& outBodyId,
            BodyMotionState& outState,
            bool& outSearchCompleted)
        {
            outBodyId = INVALID_BODY_ID;
            outState = {};
            outSearchCompleted = false;
            if (!world || motionId == 0) {
                return false;
            }

            if (tryReadCurrentBodyMotionState(world, preferredBodyId, outState) && outState.motionId == motionId) {
                outBodyId = preferredBodyId;
                return true;
            }

            std::uint32_t highWaterMark = 0;
            if (!tryReadWorldBodyHighWaterMark(world, highWaterMark)) {
                return false;
            }
            outSearchCompleted = true;

            for (std::uint32_t bodyId = 0; bodyId <= highWaterMark; ++bodyId) {
                if (!havok_runtime::bodySlotCanBeRead(bodyId, highWaterMark)) {
                    continue;
                }

                BodyMotionState candidate{};
                if (tryReadCurrentBodyMotionState(world, bodyId, candidate) && candidate.motionId == motionId) {
                    outBodyId = bodyId;
                    outState = candidate;
                    return true;
                }
            }

            return false;
        }

        bool setBodyMotionPropertiesVerified(
            RE::hknpWorld* world,
            std::uint32_t bodyId,
            std::uint32_t expectedMotionId,
            std::uint16_t motionPropertiesId)
        {
            if (!world || bodyId == INVALID_BODY_ID || motionPropertiesId == kInvalidMotionPropertiesId) {
                return false;
            }

            using SetBodyMotionProperties_t = void (*)(RE::hknpWorld*, RE::hknpBodyId, RE::hknpMotionPropertiesId_Handle);
            static REL::Relocation<SetBodyMotionProperties_t> setBodyMotionProperties{ REL::Offset(offsets::kFunc_SetBodyMotionProperties) };

            RE::hknpMotionPropertiesId_Handle handle{};
            handle.value = motionPropertiesId;
            setBodyMotionProperties(world, RE::hknpBodyId{ bodyId }, handle);

            BodyMotionState after{};
            return tryReadCurrentBodyMotionState(world, bodyId, after) &&
                   after.motionId == expectedMotionId &&
                   after.motionPropertiesId == motionPropertiesId;
        }

        bool acquireMotionDampingLease(RE::hknpWorld* world,
            std::uint32_t bodyId,
            std::uint32_t requestedMotionId,
            float requestedLinearDamping,
            float requestedAngularDamping,
            std::uintptr_t ownerToken,
            SavedNearbyMotionDamping& outSaved)
        {
            outSaved = {};
            if (!world || bodyId == INVALID_BODY_ID || requestedMotionId == 0 || ownerToken == 0) {
                return false;
            }

            bool reservedNativeWrite = false;
            if (!reserveMotionDampingWriteOrAttachExisting(world, bodyId, requestedMotionId, ownerToken, outSaved, reservedNativeWrite)) {
                return false;
            }
            if (!reservedNativeWrite) {
                return true;
            }

            BodyMotionState current{};
            if (!tryReadCurrentBodyMotionState(world, bodyId, current) || current.motionId != requestedMotionId) {
                abortMotionDampingWrite(world, requestedMotionId);
                return false;
            }

            MotionPropertiesRecord originalRecord{};
            if (!tryReadMotionPropertiesRecord(world, current.motionPropertiesId, originalRecord)) {
                abortMotionDampingWrite(world, requestedMotionId);
                return false;
            }

            const float originalLinearDamping = readRecordFloat(originalRecord, offsets::kMotionProperties_LinearDamping);
            const float originalAngularDamping = readRecordFloat(originalRecord, offsets::kMotionProperties_AngularDamping);
            if (!std::isfinite(originalLinearDamping) || !std::isfinite(originalAngularDamping)) {
                abortMotionDampingWrite(world, requestedMotionId);
                return false;
            }

            const float targetLinearDamping = targetHknpDampingCoefficient(originalLinearDamping, requestedLinearDamping);
            const float targetAngularDamping = targetHknpDampingCoefficient(originalAngularDamping, requestedAngularDamping);
            if (targetLinearDamping == originalLinearDamping && targetAngularDamping == originalAngularDamping) {
                abortMotionDampingWrite(world, requestedMotionId);
                return false;
            }

            auto dampedRecord = originalRecord;
            writeRecordFloat(dampedRecord, offsets::kMotionProperties_LinearDamping, targetLinearDamping);
            writeRecordFloat(dampedRecord, offsets::kMotionProperties_AngularDamping, targetAngularDamping);

            std::uint16_t dampedMotionPropertiesId = kInvalidMotionPropertiesId;
            if (!getOrCreateDampedPropertyId(world, dampedRecord, dampedMotionPropertiesId)) {
                abortMotionDampingWrite(world, requestedMotionId);
                return false;
            }

            if (!setBodyMotionPropertiesVerified(world, bodyId, current.motionId, dampedMotionPropertiesId)) {
                abortMotionDampingWrite(world, requestedMotionId);
                return false;
            }

            commitMotionDampingWrite(MotionDampingLeaseEntry{
                .world = world,
                .motionId = current.motionId,
                .representativeBodyId = bodyId,
                .originalMotionPropertiesId = current.motionPropertiesId,
                .dampedMotionPropertiesId = dampedMotionPropertiesId,
                .originalLinearDamping = originalLinearDamping,
                .originalAngularDamping = originalAngularDamping,
                .ownerTokens = { ownerToken },
                .restoreInProgress = false,
            });

            outSaved = SavedNearbyMotionDamping{
                .representativeBodyId = bodyId,
                .motionId = current.motionId,
                .originalLinearDamping = originalLinearDamping,
                .originalAngularDamping = originalAngularDamping,
                .originalMotionPropertiesId = current.motionPropertiesId,
                .dampedMotionPropertiesId = dampedMotionPropertiesId,
                .leaseToken = ownerToken,
                .active = true,
            };
            return true;
        }

        LeaseRestoreAttempt attemptFinalLeaseRestore(RE::hknpWorld* world, const MotionDampingLeaseEntry& lease)
        {
            LeaseRestoreAttempt attempt{};
            attempt.result.finalLease = true;

            std::uint32_t restoreBodyId = INVALID_BODY_ID;
            BodyMotionState current{};
            bool bodySearchCompleted = false;
            const bool bodyFound = tryFindLiveBodyForMotion(world, lease.representativeBodyId, lease.motionId, restoreBodyId, current, bodySearchCompleted);
            const auto decision = decideFinalLeaseRestore(FinalLeaseRestoreDecisionInput{
                .bodyFound = bodyFound,
                .bodySearchCompleted = world && bodySearchCompleted,
                .leasedMotionId = lease.motionId,
                .currentMotionId = current.motionId,
                .currentMotionPropertiesId = current.motionPropertiesId,
                .originalMotionPropertiesId = lease.originalMotionPropertiesId,
                .dampedMotionPropertiesId = lease.dampedMotionPropertiesId,
            });

            if (decision == FinalLeaseRestoreDecision::CompleteMotionGone) {
                attempt.result.motionGone = true;
                attempt.completed = true;
            } else if (decision == FinalLeaseRestoreDecision::CompleteAlreadyOriginal) {
                attempt.result.alreadyOriginal = true;
                attempt.completed = true;
            } else if (decision == FinalLeaseRestoreDecision::CompleteExternalChange) {
                attempt.result.skippedExternalChange = true;
                attempt.completed = true;
            } else if (decision == FinalLeaseRestoreDecision::WriteOriginal &&
                       setBodyMotionPropertiesVerified(world, restoreBodyId, lease.motionId, lease.originalMotionPropertiesId)) {
                attempt.result.restoredOriginal = true;
                attempt.completed = true;
            } else {
                attempt.result.restoreFailed = true;
            }

            return attempt;
        }

        void commitFinalLeaseRestoreAttempt(const MotionDampingLeaseEntry& lease, bool completed)
        {
            {
                std::lock_guard lock(g_motionDampingMutex);
                auto leaseIt = findLeaseLocked(lease.world, lease.motionId);
                if (leaseIt != g_motionDampingLeases.end()) {
                    if (completed && leaseIt->ownerTokens.empty()) {
                        g_motionDampingLeases.erase(leaseIt);
                    } else {
                        leaseIt->restoreInProgress = false;
                    }
                }
            }
            g_motionDampingChanged.notify_all();
        }

        LeaseReleaseResult retryPendingFinalLeaseRestores(RE::hknpWorld* world)
        {
            LeaseReleaseResult aggregate{};
            if (!world) {
                return aggregate;
            }

            for (;;) {
                MotionDampingLeaseEntry leaseSnapshot{};
                {
                    std::lock_guard lock(g_motionDampingMutex);
                    auto leaseIt = std::find_if(g_motionDampingLeases.begin(), g_motionDampingLeases.end(), [&](const MotionDampingLeaseEntry& lease) {
                        return lease.world == world && lease.ownerTokens.empty() && !lease.restoreInProgress;
                    });
                    if (leaseIt == g_motionDampingLeases.end()) {
                        return aggregate;
                    }

                    leaseIt->restoreInProgress = true;
                    leaseSnapshot = *leaseIt;
                }

                const auto attempt = attemptFinalLeaseRestore(world, leaseSnapshot);
                commitFinalLeaseRestoreAttempt(leaseSnapshot, attempt.completed);
                aggregate.finalLease = aggregate.finalLease || attempt.result.finalLease;
                aggregate.restoredOriginal = aggregate.restoredOriginal || attempt.result.restoredOriginal;
                aggregate.alreadyOriginal = aggregate.alreadyOriginal || attempt.result.alreadyOriginal;
                aggregate.skippedExternalChange = aggregate.skippedExternalChange || attempt.result.skippedExternalChange;
                aggregate.restoreFailed = aggregate.restoreFailed || attempt.result.restoreFailed;
                aggregate.motionGone = aggregate.motionGone || attempt.result.motionGone;

                if (!attempt.completed) {
                    return aggregate;
                }
            }
        }

        LeaseReleaseResult releaseMotionDampingLease(RE::hknpWorld* world, SavedNearbyMotionDamping& motionState)
        {
            LeaseReleaseResult result{};
            if (!world || !motionState.active || motionState.leaseToken == 0 || motionState.motionId == 0) {
                return result;
            }

            MotionDampingLeaseEntry leaseSnapshot{};
            bool shouldRestoreFinalLease = false;
            {
                std::unique_lock lock(g_motionDampingMutex);
                auto leaseIt = findLeaseLocked(world, motionState.motionId);
                if (leaseIt == g_motionDampingLeases.end()) {
                    motionState.active = false;
                    return result;
                }

                while (leaseIt->restoreInProgress) {
                    g_motionDampingChanged.wait(lock);
                    leaseIt = findLeaseLocked(world, motionState.motionId);
                    if (leaseIt == g_motionDampingLeases.end()) {
                        motionState.active = false;
                        return result;
                    }
                }

                auto ownerIt = std::find(leaseIt->ownerTokens.begin(), leaseIt->ownerTokens.end(), motionState.leaseToken);
                if (ownerIt == leaseIt->ownerTokens.end()) {
                    motionState.active = false;
                    return result;
                }

                leaseIt->ownerTokens.erase(ownerIt);
                result.released = true;

                if (!leaseIt->ownerTokens.empty()) {
                    motionState.active = false;
                    return result;
                }

                leaseIt->restoreInProgress = true;
                leaseSnapshot = *leaseIt;
                shouldRestoreFinalLease = true;
            }

            if (shouldRestoreFinalLease) {
                const auto attempt = attemptFinalLeaseRestore(world, leaseSnapshot);
                result.finalLease = attempt.result.finalLease;
                result.restoredOriginal = attempt.result.restoredOriginal;
                result.alreadyOriginal = attempt.result.alreadyOriginal;
                result.skippedExternalChange = attempt.result.skippedExternalChange;
                result.restoreFailed = attempt.result.restoreFailed;
                result.motionGone = attempt.result.motionGone;
                commitFinalLeaseRestoreAttempt(leaseSnapshot, attempt.completed);
            }

            motionState.active = false;
            return result;
        }

        void appendUniqueMotionRecords(const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
            PureDampingCandidateSet& candidates,
            std::unordered_set<std::uint32_t>& seenMotionIds)
        {
            for (const auto* record : bodySet.uniqueAcceptedMotionRecords()) {
                if (!record || record->bodyId == INVALID_BODY_ID || record->motionId == 0) {
                    continue;
                }

                if (!seenMotionIds.insert(record->motionId).second) {
                    candidates.add(PureDampingCandidate{ .bodyId = record->bodyId, .motionId = record->motionId, .accepted = true });
                    continue;
                }

                candidates.add(PureDampingCandidate{ .bodyId = record->bodyId, .motionId = record->motionId, .accepted = true });
            }
        }
    }

    NearbyGrabDampingState beginNearbyGrabDamping(RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        RE::TESObjectREFR* heldRef,
        const std::vector<std::uint32_t>& heldBodyIds,
        RE::NiPoint3 centerGame,
        float radiusGame,
        float durationSeconds,
        float linearDamping,
        float angularDamping,
        const object_physics_body_set::BodySetScanOptions& baseOptions)
    {
        NearbyGrabDampingState state;
        const float radius = sanitizeRadius(radiusGame);
        const float duration = sanitizeDuration(durationSeconds);
        const float safeLinearDamping = sanitizeHknpDampingCoefficient(linearDamping);
        const float safeAngularDamping = sanitizeHknpDampingCoefficient(angularDamping);

        if (!runtimeDampingWritesVerified()) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                1000,
                "Nearby grab damping disabled: runtime motion damping write path is not verified radius={:.1f} duration={:.2f}s",
                radius,
                duration);
            return state;
        }

        if (!shouldBeginRuntimeNearbyDamping(true, true, radius, duration)) {
            return state;
        }

        if (!bhkWorld || !hknpWorld) {
            return state;
        }

        retryPendingFinalLeaseRestores(hknpWorld);

        state.world = hknpWorld;
        state.linearDamping = safeLinearDamping;
        state.angularDamping = safeAngularDamping;
        state.remainingSeconds = duration;

        RE::hknpAllHitsCollector collector;
        physics_shape_cast::SphereCastDiagnostics diagnostics;
        if (!physics_shape_cast::castSelectionSphere(
                hknpWorld,
                physics_shape_cast::SphereCastInput{ .startGame = centerGame,
                    .directionGame = RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
                    .distanceGame = 1.0f,
                    .radiusGame = radius,
                    .collisionFilterInfo = physics_shape_cast::kSelectionQueryCollisionFilterInfo },
                 collector,
                 &diagnostics)) {
            ROCK_LOG_DEBUG(Hand, "Nearby grab damping skipped: sphere query failed radius={:.1f}", radius);
            state.clear();
            return state;
        }

        PureDampingCandidateSet candidates;
        std::unordered_set<RE::TESObjectREFR*> scannedRefs;
        std::unordered_set<std::uint32_t> seenMotionIds;
        std::uint32_t rejectedHeldBodies = 0;
        std::uint32_t rejectedRefs = 0;

        auto* hits = collector.hits._data;
        for (int i = 0; i < collector.hits._size; ++i) {
            const auto bodyId = hits[i].hitBodyInfo.m_bodyId;
            if (bodyId.value == INVALID_BODY_ID || containsBodyId(heldBodyIds, bodyId.value)) {
                ++rejectedHeldBodies;
                continue;
            }

            auto* ref = resolveBodyToRef(bhkWorld, hknpWorld, bodyId);
            if (!ref || ref == heldRef) {
                ++rejectedRefs;
                continue;
            }
            if (!scannedRefs.insert(ref).second) {
                continue;
            }

            auto options = baseOptions;
            options.mode = physics_body_classifier::InteractionMode::PassivePush;
            options.heldBySameHand = &heldBodyIds;
            auto bodySet = object_physics_body_set::scanObjectPhysicsBodySet(bhkWorld, hknpWorld, ref, options);
            appendUniqueMotionRecords(bodySet, candidates, seenMotionIds);
        }

        state.leaseToken = nextLeaseToken();
        std::uint32_t dampingWriteSkips = 0;
        for (const auto bodyId : candidates.uniqueAcceptedMotionBodyIds()) {
            auto* body = havok_runtime::getBody(hknpWorld, RE::hknpBodyId{ bodyId });
            if (!body || bodyId == INVALID_BODY_ID) {
                continue;
            }

            SavedNearbyMotionDamping savedMotion{};
            if (acquireMotionDampingLease(
                    hknpWorld, bodyId, body->motionIndex, state.linearDamping, state.angularDamping, state.leaseToken, savedMotion)) {
                state.motions.push_back(savedMotion);
            } else {
                ++dampingWriteSkips;
            }
        }

        state.active = !state.motions.empty();
        ROCK_LOG_DEBUG(Hand,
            "Nearby grab damping begin: hits={} refs={} motions={} writeSkips={} dupMotionSkips={} rejectHeld={} rejectRef={} radius={:.1f} duration={:.2f}s linearDamp={:.2f} angularDamp={:.2f}",
            diagnostics.hitCount,
            scannedRefs.size(),
            state.motions.size(),
            dampingWriteSkips,
            candidates.duplicateMotionSkips(),
            rejectedHeldBodies,
            rejectedRefs,
            radius,
            duration,
            state.linearDamping,
            state.angularDamping);

        if (!state.active) {
            state.clear();
        }

        return state;
    }

    void tickNearbyGrabDamping(RE::hknpWorld* world, NearbyGrabDampingState& state, float deltaTime)
    {
        auto* targetWorld = world ? world : state.world;
        if (targetWorld) {
            retryPendingFinalLeaseRestores(targetWorld);
        }
        if (!targetWorld || !state.active) {
            return;
        }

        if (advanceTimer(state, deltaTime)) {
            restoreNearbyGrabDamping(targetWorld, state);
        }
    }

    void restoreNearbyGrabDamping(RE::hknpWorld* world, NearbyGrabDampingState& state)
    {
        auto* targetWorld = world ? world : state.world;
        if (targetWorld) {
            retryPendingFinalLeaseRestores(targetWorld);
        }

        if (state.motions.empty() && !state.active) {
            state.clear();
            return;
        }

        std::uint32_t released = 0;
        std::uint32_t finalLeases = 0;
        std::uint32_t restored = 0;
        std::uint32_t alreadyOriginal = 0;
        std::uint32_t skippedExternal = 0;
        std::uint32_t restoreFailed = 0;
        std::uint32_t motionGone = 0;
        if (targetWorld) {
            for (auto& motionState : state.motions) {
                if (motionState.active) {
                    const auto result = releaseMotionDampingLease(targetWorld, motionState);
                    if (result.released) {
                        ++released;
                    }
                    if (result.finalLease) {
                        ++finalLeases;
                    }
                    if (result.restoredOriginal) {
                        ++restored;
                    }
                    if (result.alreadyOriginal) {
                        ++alreadyOriginal;
                    }
                    if (result.skippedExternalChange) {
                        ++skippedExternal;
                    }
                    if (result.restoreFailed) {
                        ++restoreFailed;
                    }
                    if (result.motionGone) {
                        ++motionGone;
                    }
                }
            }
        } else {
            restoreFailed = static_cast<std::uint32_t>(state.motions.size());
        }

        ROCK_LOG_DEBUG(Hand,
            "Nearby grab damping restored: released={} final={} restored={} alreadyOriginal={} skippedExternal={} motionGone={} restoreFailed={}",
            released,
            finalLeases,
            restored,
            alreadyOriginal,
            skippedExternal,
            motionGone,
            restoreFailed);
        state.clear();
    }
}
