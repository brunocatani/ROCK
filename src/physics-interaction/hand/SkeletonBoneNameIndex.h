#pragma once

#include "physics-interaction/hand/HandSkeleton.h"

#include <algorithm>
#include <numeric>
#include <span>

namespace rock
{
    // Main-thread topology cache. Only indices and an opaque reader identity
    // survive a bind; live transforms always come from the supplied snapshot.
    class SkeletonBoneNameIndex
    {
    public:
        struct View
        {
            const DirectSkeletonBoneSnapshot& snapshot;
            std::span<const std::size_t> indices;

            [[nodiscard]] const DirectSkeletonBoneEntry* find(std::string_view name) const
            {
                if (!snapshot.valid) return nullptr;
                const auto it = std::lower_bound(indices.begin(), indices.end(), name,
                    [&](std::size_t index, std::string_view key) { return snapshot.bones[index].name < key; });
                return it != indices.end() && snapshot.bones[*it].name == name ? &snapshot.bones[*it] : nullptr;
            }
        };

        // The returned view is borrowed for this operation, until the next bind.
        [[nodiscard]] View bind(const DirectSkeletonBoneSnapshot& snapshot)
        {
            if (!snapshot.valid) {
                _owner = nullptr;
                _indices.clear();
                return { snapshot, _indices };
            }
            if (!snapshot.topologyOwner || _owner != snapshot.topologyOwner ||
                _revision != snapshot.topologyRevision || _payload != snapshot.payload || _indices.size() != snapshot.bones.size()) {
                _indices.resize(snapshot.bones.size());
                std::iota(_indices.begin(), _indices.end(), std::size_t{ 0 });
                std::sort(_indices.begin(), _indices.end(), [&](std::size_t a, std::size_t b) {
                    const auto& aName = snapshot.bones[a].name;
                    const auto& bName = snapshot.bones[b].name;
                    // The previous emplace-based map kept the first duplicate.
                    return aName == bName ? a < b : aName < bName;
                });
                _owner = snapshot.topologyOwner;
                _revision = snapshot.topologyRevision;
                _payload = snapshot.payload;
            }
            return { snapshot, _indices };
        }

    private:
        const void* _owner = nullptr; // Identity only; never dereferenced.
        std::uint64_t _revision = 0;
        SkeletonBoneCapturePayload _payload = SkeletonBoneCapturePayload::FlattenedTransforms;
        std::vector<std::size_t> _indices;
    };
}
