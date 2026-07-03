#include "physics-interaction/weapon/WeaponPartMotionLearner.h"

#include "physics-interaction/PhysicsLog.h"

#include <algorithm>
#include <cstring>

namespace rock
{
    namespace
    {
        std::string_view slotName(const std::array<char, WeaponPartMotionLearner::kMaxSourceName>& name)
        {
            std::size_t length = 0;
            while (length < name.size() && name[length] != '\0') {
                ++length;
            }
            return std::string_view(name.data(), length);
        }

        void copySlotName(std::array<char, WeaponPartMotionLearner::kMaxSourceName>& target, std::string_view source)
        {
            target = {};
            const auto count = (std::min)(source.size(), target.size() - 1);
            std::memcpy(target.data(), source.data(), count);
        }

        bool slotMatches(
            std::uint32_t slotFormId,
            const std::array<char, WeaponPartMotionLearner::kMaxSourceName>& name,
            std::uint32_t weaponFormId,
            std::string_view sourceName)
        {
            return slotFormId == weaponFormId && slotName(name) == sourceName;
        }
    }

    void WeaponPartMotionLearner::observe(const Observation& observation)
    {
        if (observation.weaponFormId == 0 || observation.sourceName.empty()) {
            return;
        }
        ++_observationCounter;

        auto* recorder = acquireRecorderSlot(observation.weaponFormId, observation.sourceName);
        if (!recorder) {
            return;
        }
        recorder->lastSeenCounter = _observationCounter;

        const auto result = weapon_part_motion_path::step(
            recorder->state,
            recorder->buffer.data(),
            observation.pose,
            observation.trusted);
        if (result == weapon_part_motion_path::StepResult::RecordingComplete) {
            storeCompletedPath(*recorder);
        }
    }

    const weapon_part_motion_path::MotionPath* WeaponPartMotionLearner::findPath(
        std::uint32_t weaponFormId,
        std::string_view sourceName) const
    {
        for (const auto& slot : _paths) {
            if (slot.used && slotMatches(slot.weaponFormId, slot.sourceName, weaponFormId, sourceName)) {
                // lastUseCounter is an eviction hint, not behavior; keeping
                // this accessor const outweighs refreshing it on reads.
                return &slot.path;
            }
        }
        return nullptr;
    }

    void WeaponPartMotionLearner::reset()
    {
        _paths = {};
        _recorders = {};
        _observationCounter = 0;
    }

    WeaponPartMotionLearner::RecorderSlot* WeaponPartMotionLearner::acquireRecorderSlot(
        std::uint32_t weaponFormId,
        std::string_view sourceName)
    {
        RecorderSlot* freeSlot = nullptr;
        RecorderSlot* staleSlot = nullptr;
        for (auto& slot : _recorders) {
            if (slot.used && slotMatches(slot.weaponFormId, slot.sourceName, weaponFormId, sourceName)) {
                return &slot;
            }
            if (!slot.used) {
                freeSlot = freeSlot ? freeSlot : &slot;
            } else if (_observationCounter - slot.lastSeenCounter > kRecorderStaleObservationAge &&
                       (!staleSlot || slot.lastSeenCounter < staleSlot->lastSeenCounter)) {
                staleSlot = &slot;
            }
        }

        auto* claimed = freeSlot ? freeSlot : staleSlot;
        if (!claimed) {
            return nullptr;
        }
        claimed->used = true;
        claimed->weaponFormId = weaponFormId;
        copySlotName(claimed->sourceName, sourceName);
        claimed->lastSeenCounter = _observationCounter;
        claimed->state = {};
        return claimed;
    }

    void WeaponPartMotionLearner::storeCompletedPath(const RecorderSlot& recorder)
    {
        weapon_part_motion_path::MotionPath candidate{};
        if (!weapon_part_motion_path::buildPathFromRecording(recorder.buffer.data(), recorder.state.sampleCount, candidate)) {
            return;
        }

        PathSlot* target = nullptr;
        for (auto& slot : _paths) {
            if (slot.used && slotMatches(slot.weaponFormId, slot.sourceName, recorder.weaponFormId, slotName(recorder.sourceName))) {
                target = &slot;
                break;
            }
        }
        if (target) {
            if (!weapon_part_motion_path::shouldReplacePath(target->path, candidate)) {
                return;
            }
        } else {
            for (auto& slot : _paths) {
                if (!slot.used) {
                    target = &slot;
                    break;
                }
                if (!target || slot.lastUseCounter < target->lastUseCounter) {
                    target = &slot;
                }
            }
        }
        if (!target) {
            return;
        }

        const bool replaced = target->used;
        target->used = true;
        target->weaponFormId = recorder.weaponFormId;
        target->sourceName = recorder.sourceName;
        target->lastUseCounter = _observationCounter;
        target->path = candidate;

        // Once per completed stroke, never per frame.
        ROCK_LOG_INFO(Weapon,
            "WeaponPartMotionLearner: {} motion path for part '{}' on weapon {:08X} (stroke arc {:.2f} game units, {} keys, {} raw samples)",
            replaced ? "updated" : "learned",
            slotName(recorder.sourceName),
            recorder.weaponFormId,
            candidate.totalArcLength,
            weapon_part_motion_path::kResampledKeyCount,
            recorder.state.sampleCount);
    }
}
