#pragma once

/*
 * Native hknp contacts and visual hand authority are separate systems. FO4VR
 * can tell ROCK that generated hand/weapon bodies touched a wall, table, actor,
 * weapon, or held object, but keyframed bodies will not move the rendered FRIK
 * hand by themselves. This cache is the boundary between those systems: the
 * physics callback records compact, bounded contact evidence, and frame-time
 * consumers decide how to use it for visual stops, haptics, debug, or future
 * body interaction without adding another native listener.
 */

#include "RE/NetImmerse/NiPoint.h"

#include <array>
#include <cstdint>
#include <mutex>

namespace rock::contact_evidence
{
    inline constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFFu;
    inline constexpr std::uint32_t kUnknownLayer = 0xFFFF'FFFFu;
    inline constexpr std::uint32_t kUnknownFilterInfo = 0xFFFF'FFFFu;
    inline constexpr std::size_t kMaxNativeContactEvidenceRecords = 128;

    enum class NativeContactEndpointKind : std::uint8_t
    {
        Unknown = 0,
        RightHand,
        LeftHand,
        Weapon,
        RightHeldObject,
        LeftHeldObject,
        External,
        WorldSurface,
        DynamicProp,
        Actor,
        QueryOnly,
    };

    enum class NativeContactQuality : std::uint8_t
    {
        BodyPairOnly = 0,
        RawPoint,
    };

    struct NativeContactEvidenceRecord
    {
        bool valid = false;
        std::uint64_t sequence = 0;
        std::uint32_t frame = 0;
        std::uint32_t sourceBodyId = kInvalidBodyId;
        std::uint32_t targetBodyId = kInvalidBodyId;
        std::uint32_t sourceLayer = kUnknownLayer;
        std::uint32_t targetLayer = kUnknownLayer;
        std::uint32_t sourceFilterInfo = kUnknownFilterInfo;
        std::uint32_t targetFilterInfo = kUnknownFilterInfo;
        NativeContactEndpointKind sourceKind = NativeContactEndpointKind::Unknown;
        NativeContactEndpointKind targetKind = NativeContactEndpointKind::Unknown;
        NativeContactQuality quality = NativeContactQuality::BodyPairOnly;
        bool sourceIsLeft = false;
        bool targetIsLeft = false;
        std::uint32_t sourceRole = 0;
        std::uint32_t sourcePartKind = 0;
        std::uint32_t sourceSubRole = 0;
        RE::NiPoint3 contactPointGame{};
        RE::NiPoint3 contactNormalGame{};
        RE::NiPoint3 sourceVelocityGame{};
        float contactPointWeightSum = 0.0f;
    };

    struct NativeContactEvidenceSnapshot
    {
        std::array<NativeContactEvidenceRecord, kMaxNativeContactEvidenceRecords> records{};
        std::uint32_t count = 0;
        std::uint32_t currentFrame = 0;
    };

    inline constexpr bool isValidBodyId(std::uint32_t bodyId)
    {
        return bodyId != kInvalidBodyId && bodyId != 0xFFFF'FFFFu;
    }

    inline constexpr bool isFrameFresh(std::uint32_t currentFrame, std::uint32_t recordFrame, std::uint32_t maxAgeFrames)
    {
        return static_cast<std::uint32_t>(currentFrame - recordFrame) <= maxAgeFrames;
    }

    inline constexpr bool isHandSourceFor(const NativeContactEvidenceRecord& record, bool isLeft)
    {
        const auto expectedKind = isLeft ? NativeContactEndpointKind::LeftHand : NativeContactEndpointKind::RightHand;
        return record.sourceKind == expectedKind || (record.sourceKind == NativeContactEndpointKind::Unknown && record.sourceIsLeft == isLeft);
    }

    class NativeContactEvidenceCache
    {
    public:
        void reset()
        {
            std::scoped_lock lock(_mutex);
            _records = {};
            _nextSlot = 0;
            _sequence = 0;
        }

        void record(NativeContactEvidenceRecord record)
        {
            if (!isValidBodyId(record.sourceBodyId) || !isValidBodyId(record.targetBodyId) || record.sourceBodyId == record.targetBodyId) {
                return;
            }

            std::scoped_lock lock(_mutex);
            record.valid = true;
            record.sequence = ++_sequence;
            _records[_nextSlot] = record;
            _nextSlot = (_nextSlot + 1) % _records.size();
        }

        std::uint32_t invalidateHand(bool isLeft)
        {
            std::uint32_t invalidated = 0;
            std::scoped_lock lock(_mutex);
            for (auto& record : _records) {
                if (record.valid && isHandSourceFor(record, isLeft)) {
                    record.valid = false;
                    ++invalidated;
                }
            }
            return invalidated;
        }

        void snapshot(NativeContactEvidenceSnapshot& outSnapshot, std::uint32_t currentFrame) const
        {
            outSnapshot = {};
            outSnapshot.currentFrame = currentFrame;

            std::scoped_lock lock(_mutex);
            for (const auto& record : _records) {
                if (!record.valid || outSnapshot.count >= outSnapshot.records.size()) {
                    continue;
                }
                outSnapshot.records[outSnapshot.count++] = record;
            }
        }

    private:
        mutable std::mutex _mutex;
        std::array<NativeContactEvidenceRecord, kMaxNativeContactEvidenceRecords> _records{};
        std::size_t _nextSlot = 0;
        std::uint64_t _sequence = 0;
    };
}
