#include "HeldPlayerSpaceRegistry.h"

#include "HavokOffsets.h"
#include "HavokRuntime.h"
#include "HeldPlayerSpaceMath.h"
#include "PhysicsUtils.h"

#include "RE/Havok/hknpMotion.h"
#include "RE/Havok/hknpWorld.h"
#include "REL/Relocation.h"

namespace frik::rock::held_player_space_registry
{
    namespace
    {
        PureVec3 toPure(const RE::NiPoint3& value) { return PureVec3{ .x = value.x, .y = value.y, .z = value.z }; }

        PureVec3 scaled(PureVec3 value, float scale)
        {
            return PureVec3{ .x = value.x * scale, .y = value.y * scale, .z = value.z * scale };
        }

        PureVec3 removePreviousPlayerVelocity(PureVec3 currentVelocity, PureVec3 previousPlayerVelocity)
        {
            return PureVec3{
                .x = currentVelocity.x - previousPlayerVelocity.x,
                .y = currentVelocity.y - previousPlayerVelocity.y,
                .z = currentVelocity.z - previousPlayerVelocity.z,
            };
        }

        void setBodyVelocityDeferred(RE::hknpWorld* world, std::uint32_t bodyId, const PureVec3& linear, const PureVec3& angular)
        {
            havok_runtime::setBodyVelocityDeferred(
                world,
                bodyId,
                RE::hkVector4f{ linear.x, linear.y, linear.z, 0.0f },
                RE::hkVector4f{ angular.x, angular.y, angular.z, 0.0f });
        }

        void setBodyTransformDeferred(RE::hknpWorld* world, std::uint32_t bodyId, const RE::NiTransform& transform)
        {
            using SetTransformDeferred_t = void (*)(void*, std::uint32_t, const float*, int);
            static REL::Relocation<SetTransformDeferred_t> setBodyTransform{ REL::Offset(offsets::kFunc_SetBodyTransformDeferred) };
            RE::hkTransformf hkTransform;
            hkTransform.rotation = niRotToHkTransformRotation(transform.rotate);
            const float scale = gameToHavokScale();
            hkTransform.translation = RE::NiPoint4(transform.translate.x * scale, transform.translate.y * scale, transform.translate.z * scale, 0.0f);
            setBodyTransform(world, bodyId, reinterpret_cast<const float*>(&hkTransform), 1);
        }
    }

    RuntimeHeldPlayerSpaceResult applyCentralPlayerSpaceVelocity(RE::hknpWorld* world,
        const std::vector<std::uint32_t>& bodyIds,
        const RE::NiPoint3& currentPlayerVelocityHavok,
        const RE::NiPoint3& previousPlayerVelocityHavok,
        float residualVelocityKeep,
        bool enabled,
        bool warp,
        const RE::NiTransform* previousPlayerSpaceWorld,
        const RE::NiTransform* currentPlayerSpaceWorld)
    {
        RuntimeHeldPlayerSpaceResult result{};
        if (!world || !enabled || bodyIds.empty()) {
            return result;
        }

        HeldPlayerSpaceRegistry registry;
        registry.beginFrame(toPure(currentPlayerVelocityHavok), toPure(previousPlayerVelocityHavok), residualVelocityKeep);
        for (const auto bodyId : bodyIds) {
            if (bodyId == kInvalidBodyId) {
                continue;
            }
            auto* body = havok_runtime::getBody(world, RE::hknpBodyId{ bodyId });
            if (!body) {
                continue;
            }
            const auto motionIndex = body->motionIndex;
            registry.registerBody(bodyId, motionIndex);
        }

        result.registeredBodies = static_cast<std::uint32_t>(registry.registeredBodyCount());
        for (const auto& registration : registry.registrations()) {
            auto* motion = havok_runtime::getMotion(world, registration.motionIndex);
            if (!motion) {
                continue;
            }

            const HeldMotionSample sample{
                .bodyId = registration.bodyId,
                .motionIndex = registration.motionIndex,
                .linearVelocity = PureVec3{ .x = motion->linearVelocity.x, .y = motion->linearVelocity.y, .z = motion->linearVelocity.z },
                .angularVelocity = PureVec3{ .x = motion->angularVelocity.x, .y = motion->angularVelocity.y, .z = motion->angularVelocity.z },
            };
            const auto write = registry.solveBodyVelocity(sample);
            if (write.duplicateMotion) {
                ++result.duplicateMotionSkips;
                continue;
            }
            if (!write.shouldWrite) {
                continue;
            }

            if (warp) {
                if (!previousPlayerSpaceWorld || !currentPlayerSpaceWorld) {
                    continue;
                }

                RE::NiTransform bodyWorld{};
                if (!tryGetBodyWorldTransform(world, RE::hknpBodyId{ registration.bodyId }, bodyWorld)) {
                    continue;
                }

                const RE::NiTransform warpedBodyWorld =
                    held_player_space_math::warpBodyWorldThroughPlayerSpace(*previousPlayerSpaceWorld, *currentPlayerSpaceWorld, bodyWorld);
                const PureVec3 localLinear = removePreviousPlayerVelocity(sample.linearVelocity, toPure(previousPlayerVelocityHavok));
                setBodyTransformDeferred(world, registration.bodyId, warpedBodyWorld);
                setBodyVelocityDeferred(world, registration.bodyId, scaled(localLinear, residualVelocityKeep), scaled(sample.angularVelocity, residualVelocityKeep));
                ++result.transformsWarped;
                ++result.motionsWritten;
                continue;
            }

            setBodyVelocityDeferred(world, write.bodyId, write.linearVelocity, write.angularVelocity);
            ++result.motionsWritten;
        }

        registry.recordWriter(WriterKind::ConstraintTarget);
        registry.recordWriter(WriterKind::PlayerSpaceCentral);
        result.writerMask = registry.writerMask();
        return result;
    }
}
