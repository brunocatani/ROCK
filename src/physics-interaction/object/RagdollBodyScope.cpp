#include "physics-interaction/object/RagdollBodyScope.h"

#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/HavokWorldLock.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/object/PhysicsBodyClassifier.h"
#include "physics-interaction/PhysicsLog.h"

namespace rock::ragdoll
{
    Component readComponent(RE::hknpWorld* world, std::uint32_t primary)
    {
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
        if (!instance || !native_memory::tryReadField(instance, 0x18, instanceWorld) || instanceWorld != world ||
            !native_memory::tryReadField(instance, 0x20, ids) ||
            !native_memory::tryReadField(instance, 0x28, bodyCount) ||
            !native_memory::tryReadField(instance, 0x30, constraintIds) ||
            !native_memory::tryReadField(instance, 0x38, constraintCount) ||
            bodyCount <= 0 || bodyCount > kMaxBodies || constraintCount < 0 || constraintCount > kMaxConstraints ||
            !ids || (constraintCount > 0 && !constraintIds)) {
            result.reason = "system-arrays-or-budget"; return result;
        }
        std::array<BodyNode, kMaxBodies> nodes{};
        std::array<BodyOwner, kMaxBodies> owners{};
        for (std::int32_t i = 0; i < bodyCount; ++i) {
            if (!native_memory::tryReadValue(ids + i, nodes[i].id)) { result.reason = "body-id-array"; return result; }
            if (nodes[i].id == kInvalidId) continue;
            const RE::hknpBodyId id{nodes[i].id};
            std::uint32_t flags = 0;
            RE::NiCollisionObject* wrapper = nullptr;
            const auto* body = havok_runtime::bodySlotLooksReadable(world, id) ? havok_runtime::getBody(world, id) : nullptr;
            if (!body || !native_memory::tryReadField(body, 0x40, flags) ||
                !native_memory::tryReadField(body, 0x68, nodes[i].motion) ||
                !native_memory::tryReadField(body, offsets::kBody_CollisionObjectBackPointer, wrapper)) {
                result.reason = "body-slot"; return result;
            }
            nodes[i].dynamic = physics_body_classifier::motionTypeFromBodyFlags(flags) == physics_body_classifier::BodyMotionType::Dynamic;
            RE::hknpWorld* ownerWorld = nullptr;
            RE::hknpBodyId ownerId{kInvalidId};
            if (wrapper && havok_runtime::tryResolveCollisionObjectBody(wrapper, ownerWorld, ownerId) && ownerWorld == world && ownerId.value == id.value) {
                owners[i].collision = wrapper;
                owners[i].node = havok_runtime::getOwnerNodeFromCollisionObject(wrapper);
            }
            owners[i].body = nodes[i];
        }
        const char* slots = nullptr;
        std::uint32_t slotLimit = 0;
        if (!native_memory::tryReadField(world, offsets::kHknpWorld_ConstraintArrayPtr, slots) ||
            !native_memory::tryReadField(world, offsets::kHknpWorld_ConstraintCount, slotLimit)) {
            result.reason = "constraint-storage"; return result;
        }
        std::array<Joint, kMaxConstraints> joints{};
        for (std::int32_t i = 0; i < constraintCount; ++i) {
            std::uint32_t id = kInvalidId;
            if (!native_memory::tryReadValue(constraintIds + i, id)) { result.reason = "constraint-id-array"; return result; }
            if (id == kInvalidId) continue;
            if (!slots || id >= slotLimit) { result.reason = "constraint-id-bound"; return result; }
            const auto* slot = slots + std::size_t(id) * 0x38;
            std::uint8_t flags = 0;
            if (!native_memory::tryReadField(slot, 0, joints[i].a) ||
                !native_memory::tryReadField(slot, 4, joints[i].b) ||
                !native_memory::tryReadField(slot, 0x16, flags)) { result.reason = "constraint-slot"; return result; }
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
            if (!owners[i].collision || !owners[i].node) { result.reason = "connected-body-owner"; return result; }
            result.bodies[result.count++] = owners[i];
            for (int j = 0; j < constraintCount; ++j) {
                if (joints[j].enabled && ((joints[j].a == nodes[i].id && joints[j].b == 0) ||
                    (joints[j].b == nodes[i].id && joints[j].a == 0))) result.fixedAttached = true;
            }
        }
        result.valid = true;
        result.reason = "native-joint-component";
        return result;
    }
}
