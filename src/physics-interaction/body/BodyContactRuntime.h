#pragma once

#include "physics-interaction/collision/ContactPipelinePolicy.h"
#include "physics-interaction/debug/SkeletonBoneDebugMath.h"

#include "RE/NetImmerse/NiPoint.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>

namespace rock::body_contact_runtime
{
    /*
     * Body collider contacts are intentionally kept as an internal runtime
     * cache first. The public provider ABI can stay stable while shoulder stash,
     * holsters, backpack zones, and body-zone tuning get a concrete source of
     * recent body contact evidence.
     */
    inline constexpr std::size_t kMaxBodyContactRecords = 128;
    inline constexpr std::uint32_t kInvalidBodyContactId = 0xFFFF'FFFFu;

    struct BodyContactRecord
    {
        std::uint32_t frame = 0;
        std::uint32_t bodyId = kInvalidBodyContactId;
        std::uint32_t targetBodyId = kInvalidBodyContactId;
        std::uint32_t bodyLayer = contact_pipeline_policy::kUnknownLayer;
        std::uint32_t targetLayer = contact_pipeline_policy::kUnknownLayer;
        skeleton_bone_debug_math::BoneColliderRole role = skeleton_bone_debug_math::BoneColliderRole::TorsoSegment;
        body_zone::BodyZoneKind zone = body_zone::BodyZoneKind::Unknown;
        body_zone::BodyZoneSide side = body_zone::BodyZoneSide::Center;
        std::uint32_t descriptorIndex = 0;
        contact_pipeline_policy::ContactEndpointKind targetKind = contact_pipeline_policy::ContactEndpointKind::Unknown;
        skeleton_bone_debug_math::BoneColliderRole targetRole = skeleton_bone_debug_math::BoneColliderRole::TorsoSegment;
        body_zone::BodyZoneKind targetZone = body_zone::BodyZoneKind::Unknown;
        body_zone::BodyZoneSide targetSide = body_zone::BodyZoneSide::Center;
        std::uint32_t targetDescriptorIndex = 0;
        RE::NiPoint3 contactPointGame{};
        bool inPowerArmor = false;
        bool targetInPowerArmor = false;
        bool hasContactPointGame = false;
    };

    class BodyContactRuntime
    {
    public:
        void reset()
        {
            std::scoped_lock lock(_mutex);
            _writeIndex = 0;
            _recordCount = 0;
            _records = {};
        }

        void record(const BodyContactRecord& record)
        {
            if (!contact_pipeline_policy::isValidBodyId(record.bodyId) ||
                !contact_pipeline_policy::isValidBodyId(record.targetBodyId) ||
                record.bodyId == record.targetBodyId) {
                return;
            }

            std::scoped_lock lock(_mutex);
            _records[_writeIndex] = record;
            _writeIndex = (_writeIndex + 1) % _records.size();
            if (_recordCount < _records.size()) {
                ++_recordCount;
            }
        }

        std::size_t snapshot(BodyContactRecord* outRecords, std::size_t maxRecords) const
        {
            if (!outRecords || maxRecords == 0) {
                return 0;
            }

            std::scoped_lock lock(_mutex);
            const auto count = (_recordCount < maxRecords) ? _recordCount : maxRecords;
            const auto start = (_writeIndex + _records.size() - count) % _records.size();
            for (std::size_t i = 0; i < count; ++i) {
                outRecords[i] = _records[(start + i) % _records.size()];
            }
            return count;
        }

        std::size_t recordCount() const
        {
            std::scoped_lock lock(_mutex);
            return _recordCount;
        }

    private:
        mutable std::mutex _mutex;
        std::array<BodyContactRecord, kMaxBodyContactRecords> _records{};
        std::size_t _writeIndex = 0;
        std::size_t _recordCount = 0;
    };
}
