// Released object drive (Grab API driveReleasedObject): a client glides a
// loose reference no hand holds onto a pose by setting its bodies' velocities
// for one physics step per call. The object stays dynamic and collidable; a
// client that stops calling simply lets it go. Used to settle a set-down
// object onto the spot it belongs (ROCK Interactive Objects' pre-war props).
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Havok/hknpBody.h"
#include "api/ProviderRuntimeServices.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/native/havok/HavokRuntime.h"
#include "physics-interaction/native/havok/PhysicsScale.h"

#include <array>
#include <cmath>

namespace rock
{
    namespace
    {
        constexpr std::size_t kMaxDriveBodies = 16;
        constexpr int kMaxDriveDepth = 8;

        bool finitePose(const api::Transform& t)
        {
            for (const float v : t.rotate)
                if (!std::isfinite(v))
                    return false;
            for (const float v : t.translate)
                if (!std::isfinite(v))
                    return false;
            return std::isfinite(t.scale) && t.scale > 0.0f;
        }

        RE::NiTransform unpack(const api::Transform& t)
        {
            RE::NiTransform out{};
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    out.rotate.entry[r][c] = t.rotate[r * 3 + c];
            out.translate = { t.translate[0], t.translate[1], t.translate[2] };
            out.scale = t.scale;
            return out;
        }

        // Bodies of the reference's scene, bounded; duplicates removed.
        void collectBodies(RE::NiAVObject* node, int depth, std::array<std::uint32_t, kMaxDriveBodies>& ids,
            std::size_t& count)
        {
            if (!node || depth <= 0)
                return;
            struct Sink
            {
                std::array<std::uint32_t, kMaxDriveBodies>* ids;
                std::size_t* count;
            } sink{ &ids, &count };
            havok_runtime::forEachPhysicsSystemBodyId(node->collisionObject.get(), nullptr, 64,
                [](std::uint32_t id, void* data) {
                    auto& s = *static_cast<Sink*>(data);
                    for (std::size_t i = 0; i < *s.count; ++i)
                        if ((*s.ids)[i] == id)
                            return true;
                    if (*s.count == s.ids->size())
                        return false;
                    (*s.ids)[(*s.count)++] = id;
                    return true;
                },
                &sink);
            if (auto* branch = node->IsNode())
                for (const auto& child : branch->children)
                    collectBodies(child.get(), depth - 1, ids, count);
        }

        // Axis * angle of a rotation matrix in the column-vector convention
        // (world = m * v); ROCK's stored Ni rows are its transpose.
        RE::NiPoint3 rotationVector(const RE::NiMatrix3& m, float& angle)
        {
            const float trace = m.entry[0][0] + m.entry[1][1] + m.entry[2][2];
            angle = std::acos(std::clamp((trace - 1.0f) * 0.5f, -1.0f, 1.0f));
            RE::NiPoint3 axis{ m.entry[2][1] - m.entry[1][2], m.entry[0][2] - m.entry[2][0], m.entry[1][0] - m.entry[0][1] };
            const float length = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
            if (angle < 1.0e-5f || length < 1.0e-6f)
                return {};
            // Near a half turn the skew part vanishes; any axis turns there.
            const float k = angle / length;
            return { axis.x * k, axis.y * k, axis.z * k };
        }
    }

    api::Status PhysicsInteraction::driveReleasedObject(const api::grab::ReleasedObjectDrive& request,
        api::grab::ReleasedObjectProgress& progress)
    {
        if (!request.reference.referenceFormId || !request.expectedBase || !finitePose(request.rootWorld) ||
            !std::isfinite(request.approachSeconds) || request.approachSeconds < 0.05f || request.approachSeconds > 2.0f)
            return api::Status::InvalidArgument;
        const auto sample = provider::runtime::sample();
        auto* world = _lifecycle.cachedHknpWorld;
        if (!isProviderReady() || !physicsWritesAllowedForWorld(world))
            return api::Status::NotReady;
        if (request.reference.worldGeneration != sample.worldGeneration ||
            request.reference.skeletonGeneration != sample.skeletonGeneration ||
            request.reference.providerGeneration != sample.providerGeneration)
            return api::Status::GenerationMismatch;
        auto* ref = RE::TESForm::GetFormByID<RE::TESObjectREFR>(request.reference.referenceFormId);
        auto* root = ref && !ref->IsDeleted() && !ref->IsDisabled() ? ref->Get3D() : nullptr;
        if (!root)
            return api::Status::TargetUnavailable;
        auto* base = ref->GetObjectReference();
        if (!base || base->formID != request.expectedBase ||
            (request.reference.referenceNativeHandle &&
                ref->GetHandle().native_handle() != request.reference.referenceNativeHandle))
            return api::Status::TargetInvalid;
        for (const auto* hand : { &_rightHand, &_leftHand })
            if (hand->isHolding() && hand->getHeldRef() == ref)
                return api::Status::Busy;

        const auto target = unpack(request.rootWorld);
        const auto& current = root->world;
        // delta maps the root's current pose onto the target (world space).
        const auto delta = transform_math::composeTransforms(target, transform_math::invertTransform(current));
        float angle = 0.0f;
        const auto spin = rotationVector(transform_math::transposeRotation(delta.rotate), angle);
        const RE::NiPoint3 gap{ target.translate.x - current.translate.x, target.translate.y - current.translate.y,
            target.translate.z - current.translate.z };
        progress.distanceGame = std::sqrt(gap.x * gap.x + gap.y * gap.y + gap.z * gap.z);
        progress.angleRadians = angle;

        std::array<std::uint32_t, kMaxDriveBodies> ids{};
        std::size_t count = 0;
        collectBodies(root, kMaxDriveDepth, ids, count);
        const float inverseTau = 1.0f / request.approachSeconds;
        const float toGame = physics_scale::havokToGame();
        const float toHavok = physics_scale::gameToHavok();
        std::array<std::uint32_t, kMaxDriveBodies> motions{};
        std::size_t motionCount = 0, driven = 0;
        for (std::size_t i = 0; i < count; ++i) {
            const auto* body = havok_runtime::getBody(world, RE::hknpBodyId{ ids[i] });
            if (!body || !body_frame::hasUsableMotionIndex(body->motionIndex))
                continue;
            bool seen = false;
            for (std::size_t m = 0; m < motionCount; ++m)
                seen |= motions[m] == body->motionIndex;
            auto* motion = seen ? nullptr : havok_runtime::getMotion(world, body->motionIndex);
            if (!motion)
                continue;
            motions[motionCount++] = body->motionIndex;
            // The centre of mass moves where delta takes it.
            const RE::NiPoint3 com{ motion->position.x * toGame, motion->position.y * toGame, motion->position.z * toGame };
            const auto moved = transform_math::localPointToWorld(delta, com);
            const RE::hkVector4f linear{ (moved.x - com.x) * inverseTau * toHavok, (moved.y - com.y) * inverseTau * toHavok,
                (moved.z - com.z) * inverseTau * toHavok, 0.0f };
            const RE::hkVector4f angular{ spin.x * inverseTau, spin.y * inverseTau, spin.z * inverseTau, 0.0f };
            havok_runtime::activateBody(world, ids[i]);
            if (havok_runtime::setBodyVelocityDeferred(world, ids[i], linear, angular))
                ++driven;
        }
        return driven ? api::Status::Ok : api::Status::TargetInvalid;
    }
}
