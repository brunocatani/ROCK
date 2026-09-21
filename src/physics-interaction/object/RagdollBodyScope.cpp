#include "physics-interaction/object/RagdollBodyScope.h"

#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/HavokWorldLock.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/object/PhysicsBodyClassifier.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/performance/PerformanceProfiler.h"

#include <cstring>

namespace rock::ragdoll
{
    Component readComponent(RE::hknpWorld* world, std::uint32_t primary)
    {
        performance_profiler::ScopedTimer timer(performance_profiler::Scope::RagdollComponentRead);
        Component result{};
        // Live ABI witnesses: 141565CD0 appends system constraint IDs at +30;
        // 1415653F0 destroys them. 141546C60/141546D10 toggle slot+16 bit 4.
        static const bool verified = [] {
            constexpr std::array<std::uint8_t, 4> arrayCode{0x4C, 0x8D, 0x79, 0x30};
            constexpr std::array<std::uint8_t, 6> flagCode{0x0F, 0xB6, 0x46, 0x16, 0xA8, 0x04};
            std::array<std::uint8_t, 4> a{};
            std::array<std::uint8_t, 6> b{};
            const auto base = REL::Module::get().base();
            const bool valid = native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(base + 0x1565CEF), a.data(), a.size()) &&
                native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(base + 0x1546CC4), b.data(), b.size()) &&
                a == arrayCode && b == flagCode;
            if (!valid) ROCK_LOG_ERROR(Hand, "Ragdoll native topology ABI mismatch; body grabs disabled");
            return valid;
        }();
        if (!verified || !world || primary == kInvalidId) { result.reason = "world-or-abi"; return result; }
        havok_world_lock::ScopedWorldReadLock lock(world);
        if (!havok_runtime::bodySlotLooksReadable(world, RE::hknpBodyId{primary})) { result.reason = "primary-slot"; return result; }
        auto* collision = havok_runtime::getCollisionObjectFromBody(world, RE::hknpBodyId{primary});
        auto* system = havok_runtime::getPhysicsSystemFromCollisionObject(collision);
        auto* instance = havok_runtime::getPhysicsSystemInstance(system);
        RE::hknpWorld* instanceWorld = nullptr;
        const std::uint32_t *ids = nullptr, *constraintIds = nullptr;
        std::int32_t bodyCount = 0, constraintCount = 0;
        // Snapshot only the existing verified fields, while the world read lock
        // fixes membership. Every refresh copies live bytes and retains SEH/page
        // validation; no pointer, page permission or topology cache crosses frames.
        std::array<std::byte, 0x3C - 0x18> instanceFields{};
        if (!instance || !native_memory::guardedCopyFromMemory(
                static_cast<const char*>(instance) + 0x18, instanceFields.data(), instanceFields.size())) {
            result.reason = "system-header"; return result;
        }
        std::memcpy(&instanceWorld, instanceFields.data(), sizeof(instanceWorld));
        std::memcpy(&ids, instanceFields.data() + 0x20 - 0x18, sizeof(ids));
        std::memcpy(&bodyCount, instanceFields.data() + 0x28 - 0x18, sizeof(bodyCount));
        std::memcpy(&constraintIds, instanceFields.data() + 0x30 - 0x18, sizeof(constraintIds));
        std::memcpy(&constraintCount, instanceFields.data() + 0x38 - 0x18, sizeof(constraintCount));
        if (instanceWorld != world || bodyCount <= 0 || bodyCount > kMaxBodies ||
            constraintCount < 0 || constraintCount > kMaxConstraints ||
            !ids || (constraintCount > 0 && !constraintIds)) {
            result.reason = "system-arrays-or-budget"; return result;
        }
        performance_profiler::observeValue(performance_profiler::ValueMetric::RagdollSystemBodies, bodyCount);
        performance_profiler::observeValue(performance_profiler::ValueMetric::RagdollSystemConstraints, constraintCount);
        std::array<std::uint32_t, kMaxBodies> bodyIds{};
        std::array<std::uint32_t, kMaxConstraints> jointIds{};
        if (!native_memory::guardedCopyFromMemory(ids, bodyIds.data(), bodyCount * sizeof(bodyIds[0]))) {
            result.reason = "body-id-array"; return result;
        }
        if (constraintCount > 0 && !native_memory::guardedCopyFromMemory(
                constraintIds, jointIds.data(), constraintCount * sizeof(jointIds[0]))) {
            result.reason = "constraint-id-array"; return result;
        }
        std::array<BodyNode, kMaxBodies> nodes{};
        std::array<RE::NiCollisionObject*, kMaxBodies> wrappers{};
        for (std::int32_t i = 0; i < bodyCount; ++i) {
            nodes[i].id = bodyIds[i];
            if (nodes[i].id == kInvalidId) continue;
            const RE::hknpBodyId id{nodes[i].id};
            std::uint32_t flags = 0;
            RE::NiCollisionObject* wrapper = nullptr;
            const auto* body = havok_runtime::getBody(world, id);
            std::array<std::byte, offsets::kBody_CollisionObjectBackPointer + sizeof(wrapper) - 0x40> bodyFields{};
            if (!body || !native_memory::guardedCopyFromMemory(
                    reinterpret_cast<const char*>(body) + 0x40, bodyFields.data(), bodyFields.size())) {
                result.reason = "body-slot"; return result;
            }
            std::memcpy(&flags, bodyFields.data(), sizeof(flags));
            std::memcpy(&nodes[i].motion, bodyFields.data() + 0x68 - 0x40, sizeof(nodes[i].motion));
            std::memcpy(&wrapper, bodyFields.data() + offsets::kBody_CollisionObjectBackPointer - 0x40, sizeof(wrapper));
            nodes[i].dynamic = physics_body_classifier::motionTypeFromBodyFlags(flags) == physics_body_classifier::BodyMotionType::Dynamic;
            wrappers[i] = wrapper;
        }
        const char* slots = nullptr;
        std::uint32_t slotLimit = 0;
        std::array<std::byte, offsets::kHknpWorld_ConstraintCount + sizeof(slotLimit) - offsets::kHknpWorld_ConstraintArrayPtr> slotFields{};
        if (!native_memory::guardedCopyFromMemory(
                reinterpret_cast<const char*>(world) + offsets::kHknpWorld_ConstraintArrayPtr,
                slotFields.data(), slotFields.size())) {
            result.reason = "constraint-storage"; return result;
        }
        std::memcpy(&slots, slotFields.data(), sizeof(slots));
        std::memcpy(&slotLimit, slotFields.data() + offsets::kHknpWorld_ConstraintCount - offsets::kHknpWorld_ConstraintArrayPtr, sizeof(slotLimit));
        std::array<Joint, kMaxConstraints> joints{};
        for (std::int32_t i = 0; i < constraintCount; ++i) {
            const auto id = jointIds[i];
            if (id == kInvalidId) continue;
            if (!slots || id >= slotLimit) { result.reason = "constraint-id-bound"; return result; }
            const auto* slot = slots + std::size_t(id) * 0x38;
            std::uint8_t flags = 0;
            std::array<std::byte, 0x17> jointFields{};
            if (!native_memory::guardedCopyFromMemory(slot, jointFields.data(), jointFields.size())) {
                result.reason = "constraint-slot"; return result;
            }
            std::memcpy(&joints[i].a, jointFields.data(), sizeof(joints[i].a));
            std::memcpy(&joints[i].b, jointFields.data() + 4, sizeof(joints[i].b));
            std::memcpy(&flags, jointFields.data() + 0x16, sizeof(flags));
            joints[i].enabled = joints[i].a != kInvalidId && joints[i].b != kInvalidId && (flags & 4) == 0;
            // A native fixed-world endpoint can lie outside the system's body array.
            if (joints[i].enabled) {
                for (auto endpoint : {joints[i].a, joints[i].b}) {
                    bool found = false;
                    for (int j = 0; j < bodyCount; ++j) found = found || nodes[j].id == endpoint;
                    if (!found && endpoint != 0) { result.reason = "joint-outside-system"; return result; }
                }
            }
        }
        const auto membership = connectedComponent({nodes.data(), std::size_t(bodyCount)},
            {joints.data(), std::size_t(constraintCount)}, primary);
        if (!membership.valid) { result.reason = "primary-not-dynamic"; return result; }
        result.fixedAttached = membership.fixedAttached;
        for (int i = 0; i < bodyCount; ++i) {
            if (!membership.included[i] || !nodes[i].dynamic) continue;
            // Detached/unconnected bodies still participate in topology checks,
            // but only the component being held needs its scene owner resolved.
            RE::hknpWorld* ownerWorld = nullptr;
            RE::hknpBodyId ownerId{kInvalidId};
            auto* wrapper = wrappers[i];
            auto* node = wrapper && havok_runtime::tryResolveCollisionObjectBody(wrapper, ownerWorld, ownerId) &&
                ownerWorld == world && ownerId.value == nodes[i].id ?
                    havok_runtime::getOwnerNodeFromCollisionObject(wrapper) : nullptr;
            if (!node) { result.reason = "connected-body-owner"; return result; }
            result.bodies[result.count++] = BodyOwner{ nodes[i], wrapper, node };
            for (int j = 0; j < constraintCount; ++j) {
                if (joints[j].enabled && ((joints[j].a == nodes[i].id && joints[j].b == 0) ||
                    (joints[j].b == nodes[i].id && joints[j].a == 0))) result.fixedAttached = true;
            }
        }
        performance_profiler::observeValue(performance_profiler::ValueMetric::RagdollConnectedBodies, result.count);
        result.valid = true;
        result.reason = "native-joint-component";
        return result;
    }
}
