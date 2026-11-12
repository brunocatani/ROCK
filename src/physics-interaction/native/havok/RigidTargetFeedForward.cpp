#include "physics-interaction/native/havok/RigidTargetFeedForward.h"

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/native/havok/BethesdaPhysicsBody.h"
#include "physics-interaction/native/havok/GeneratedKeyframedBodyDrive.h"
#include "physics-interaction/native/havok/HavokRuntime.h"

#include <cmath>

namespace rock::rigid_target_feed_forward
{
    namespace
    {
        [[nodiscard]] bool finiteTransform(const RE::NiTransform& value)
        {
            for (int row = 0; row < 3; ++row) {
                if (!std::isfinite(value.rotate.entry[row].x) || !std::isfinite(value.rotate.entry[row].y) ||
                    !std::isfinite(value.rotate.entry[row].z))
                    return false;
            }
            return std::isfinite(value.translate.x) && std::isfinite(value.translate.y) &&
                std::isfinite(value.translate.z) && std::isfinite(value.scale);
        }

        // Generated bodies store column axes; TransformMath composes row axes.
        [[nodiscard]] RE::NiTransform swapAxisStorage(RE::NiTransform value)
        {
            value.rotate = transform_math::transposeRotation(value.rotate);
            return value;
        }

        bool addBodyVelocity(RE::hknpWorld* world, RE::hknpBodyId bodyId, const RE::NiPoint3& linearDelta,
            const RE::NiPoint3& angularDelta)
        {
            const auto snapshot = havok_runtime::snapshotBody(world, bodyId);
            if (!snapshot.valid || !snapshot.motion || !snapshot.collisionObject)
                return false;
            const auto q = snapshot.motion->orientation;
            const float quaternion[4]{ q.x, q.y, q.z, q.w };
            const float lengthSquared = quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
                quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3];
            if (!std::isfinite(lengthSquared) || std::abs(lengthSquared - 1.0f) > 0.01f)
                return false;
            // The motion stores angular velocity in its local frame; the native
            // setter takes world angular velocity. havokQuaternionToNiRows
            // returns the generated-body matrix: transpose to Ni's stored axes.
            const auto rotation =
                transform_math::transposeRotation(transform_math::havokQuaternionToNiRows<RE::NiMatrix3>(quaternion));
            const auto localAngular = snapshot.motion->angularVelocity;
            const auto worldAngular = transform_math::rotateLocalVectorToWorld(rotation,
                RE::NiPoint3{ localAngular.x, localAngular.y, localAngular.z });
            const auto linear = snapshot.motion->linearVelocity;
            alignas(16) const float linearWorld[4]{ linear.x + linearDelta.x, linear.y + linearDelta.y,
                linear.z + linearDelta.z, 0.0f };
            alignas(16) const float angularWorld[4]{ worldAngular.x + angularDelta.x, worldAngular.y + angularDelta.y,
                worldAngular.z + angularDelta.z, 0.0f };
            if (!havok_runtime::isFinite3(linearWorld) || !havok_runtime::isFinite3(angularWorld))
                return false;
            return setCollisionObjectVelocity(snapshot.collisionObject, linearWorld, angularWorld);
        }
    }

    void reseed(State& state, const RE::NiTransform& authorityTarget)
    {
        if (!finiteTransform(authorityTarget))
            return;
        state.previousTarget = swapAxisStorage(authorityTarget);
        state.previousTarget.scale = 1.0f;
        state.hasPreviousTarget = true;
    }

    Outcome advance(RE::hknpWorld* world, RE::hknpBodyId bodyId, State& state, const RE::NiTransform& authorityTarget,
        float driveDeltaSeconds, bool feedAllowed, float maxLinearVelocityHavok, float maxAngularVelocityRadians)
    {
        if (!world || !finiteTransform(authorityTarget)) {
            reset(state);
            return Outcome::Failed;
        }
        auto target = swapAxisStorage(authorityTarget);
        target.scale = 1.0f;
        if (!state.hasPreviousTarget) {
            state.previousTarget = target;
            state.hasPreviousTarget = true;
            return Outcome::Seeded;
        }
        auto motion = transform_math::composeTransforms(target, transform_math::invertTransform(state.previousTarget));
        motion.scale = 1.0f;
        state.previousTarget = target;

        RE::NiTransform body{};
        if (!havok_runtime::tryResolveLiveBodyWorldTransform(world, bodyId, body) || !finiteTransform(body))
            return Outcome::Failed;
        const auto moved = swapAxisStorage(transform_math::composeTransforms(motion, swapAxisStorage(body)));
        alignas(16) float linearHavok[4]{};
        alignas(16) float angularWorld[4]{};
        if (!computeHardKeyFrameVelocity(world, bodyId, moved, driveDeltaSeconds, linearHavok, angularWorld))
            return Outcome::Failed;

        const RE::NiPoint3 linear{ linearHavok[0], linearHavok[1], linearHavok[2] };
        const RE::NiPoint3 angular{ angularWorld[0], angularWorld[1], angularWorld[2] };
        if (!(linear.Length() <= maxLinearVelocityHavok) || !(angular.Length() <= maxAngularVelocityRadians))
            return Outcome::Withheld;
        const RE::NiPoint3 linearDelta = linear - state.previousLinearHavok;
        const RE::NiPoint3 angularDelta = angular - state.previousAngular;
        state.previousLinearHavok = linear;
        state.previousAngular = angular;
        if (!feedAllowed)
            return Outcome::Withheld;
        return addBodyVelocity(world, bodyId, linearDelta, angularDelta) ? Outcome::Fed : Outcome::Failed;
    }
}
