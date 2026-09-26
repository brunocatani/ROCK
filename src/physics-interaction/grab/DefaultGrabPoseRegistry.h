#pragma once
#include <ROCK/GrabV1_2.h>
#include "SavedGrabOffsetFormat.h"
#include <algorithm>
#include <array>
#include <cmath>

namespace rock::saved_grab_offset
{
    // Value-only policy. SavedGrabOffsetStore owns this registry and serializes
    // every operation with its existing mutex, including consumer fault cleanup.
    class DefaultGrabPoseRegistry
    {
    public:
        static constexpr std::size_t capacity = 64;
        using Request = api::grab::v1_2::DefaultGrabPoses;
        using Pose = api::grab::v1_2::DefaultHandPose;

        api::Status set(api::OwnerToken owner, const Request& request) noexcept
        {
            using api::Status;
            if (request.size != sizeof(Request)) return Status::InvalidSize;
            if (!owner || !request.baseFormId || !(request.left.flags | request.right.flags) ||
                !valid(request.left) || !valid(request.right)) return Status::InvalidArgument;
            Entry* free{};
            for (auto& entry : _entries) {
                if (entry.owner && entry.request.baseFormId == request.baseFormId) {
                    if (entry.owner != owner) return Status::OwnerConflict;
                    entry.request = request;
                    return Status::Ok;
                }
                if (!entry.owner && !free) free = &entry;
            }
            if (!free) return Status::CapacityFull;
            *free = {owner, request};
            return Status::Ok;
        }
        api::Status clear(api::OwnerToken owner, std::uint32_t form) noexcept
        {
            if (!owner || !form) return api::Status::InvalidArgument;
            for (auto& entry : _entries) {
                if (entry.owner && entry.request.baseFormId == form) {
                    if (entry.owner != owner) return api::Status::OwnerConflict;
                    entry = {};
                    break;
                }
            }
            return api::Status::Ok;
        }
        void clearOwner(api::OwnerToken owner) noexcept
        {
            for (auto& entry : _entries) if (entry.owner == owner) entry = {};
        }
        bool load(std::uint32_t form, bool left, HandOffset& out) const noexcept
        {
            for (const auto& entry : _entries) {
                if (!entry.owner || entry.request.baseFormId != form) continue;
                const auto& pose = left ? entry.request.left : entry.request.right;
                if (!(pose.flags & 1)) return false;
                out = {};
                out.present = true;
                std::copy_n(pose.translateGame, 3, out.translateGame);
                std::copy_n(pose.rotate, 9, out.rotate);
                out.hasFingerPose = (pose.flags & 2) != 0;
                out.hasFingerJointValues = (pose.flags & 4) != 0;
                std::copy_n(pose.fingerValues, 5, out.fingerValues);
                std::copy_n(pose.fingerJointValues, 15, out.fingerJointValues);
                return true;
            }
            return false;
        }
    private:
        static bool valid(const Pose& pose) noexcept
        {
            if (pose.flags & ~7u) return false;
            if (!pose.flags) return true;
            if (!(pose.flags & 1) || ((pose.flags & 4) && !(pose.flags & 2))) return false;
            for (auto value : pose.translateGame) if (!std::isfinite(value)) return false;
            for (auto value : pose.rotate) if (!std::isfinite(value)) return false;
            const auto* r = pose.rotate;
            for (int row = 0; row < 3; ++row) {
                for (int other = row; other < 3; ++other) {
                    float dot{};
                    for (int col = 0; col < 3; ++col) dot += r[row*3+col] * r[other*3+col];
                    if (std::abs(dot - (row == other ? 1.f : 0.f)) > 0.005f) return false;
                }
            }
            const float determinant = r[0]*(r[4]*r[8]-r[5]*r[7]) -
                r[1]*(r[3]*r[8]-r[5]*r[6]) + r[2]*(r[3]*r[7]-r[4]*r[6]);
            if (std::abs(determinant - 1.f) > 0.005f) return false;
            if (pose.flags & 2) {
                for (int i = 0; i < 5; ++i)
                    if (!std::isfinite(pose.fingerValues[i]) || pose.fingerValues[i] < 0 ||
                        pose.fingerValues[i] > (i == 0 ? 2.f : 1.f)) return false;
            }
            if (pose.flags & 4) {
                for (int i = 0; i < 15; ++i)
                    if (!std::isfinite(pose.fingerJointValues[i]) || pose.fingerJointValues[i] < 0 ||
                        pose.fingerJointValues[i] > (i < 3 ? 2.f : 1.f)) return false;
            }
            return true;
        }
        struct Entry { api::OwnerToken owner{}; Request request{}; };
        std::array<Entry, capacity> _entries{};
    };
}
