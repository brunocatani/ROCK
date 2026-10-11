#include "physics-interaction/actor/NpcCollisionRuntime.h"

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/native/GeneratedKeyframedBodyDrive.h"
#include "physics-interaction/native/HavokRefCount.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/HavokWorldLock.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/PhysicsCallbackQuiescenceGate.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/ProcessLists.h"
#include "RE/Havok/hknpBody.h"
#include "RE/Havok/hknpShape.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string_view>
#include <Windows.h>

namespace rock
{
    namespace
    {
        constexpr float kDiscoverySeconds = 0.25f;
        constexpr float kReachGameUnits = 240.0f;
        constexpr float kRetainGameUnits = 320.0f;
        constexpr std::uint32_t kProxyFilter = (0x000Fu << 16) | collision_layer_policy::ROCK_LAYER_NPC_SUPPLEMENT;
        constexpr std::uint32_t kDisabledProxyFilter = kProxyFilter | 0x4000u;

        struct Sample
        {
            npc_collision::SourceState state{};
            RE::NiTransform pose{};
            RE::hknpMaterialId material{};
            const char* stage = "body-slot";
        };

        bool readSource(RE::hknpWorld* world, std::uint32_t id, Sample& sample)
        {
            sample = {};
            const auto identity = havok_runtime::snapshotBodyIdentity(world, RE::hknpBodyId{id});
            std::array<std::byte, sizeof(RE::hknpBody)> bytes{};
            if (!identity.valid || !native_memory::guardedCopyFromMemory(identity.body, bytes.data(), bytes.size())) return false;
            const auto read = [&]<class T>(std::size_t offset, T& value) {
                std::memcpy(&value, bytes.data() + offset, sizeof(value));
            };
            auto& state = sample.state;
            read(offsetof(RE::hknpBody, bodyId), state.identity.bodyId);
            read(offsetof(RE::hknpBody, motionIndex), state.identity.motionId);
            read(offsetof(RE::hknpBody, userData), state.identity.collisionObject);
            read(offsetof(RE::hknpBody, shape), state.identity.shape);
            read(offsetof(RE::hknpBody, flags), state.flags);
            read(offsetof(RE::hknpBody, collisionFilterInfo), state.filter);
            state.identity.group = state.filter >> 16;
            read(offsetof(RE::hknpBody, deactivationIslandId), state.presence);
            std::uint16_t material = 0;
            read(offsetof(RE::hknpBody, materialId), material);
            sample.material = RE::hknpMaterialId{material};
            sample.stage = "body-identity";
            if (!native_memory::tryReadField(identity.collisionObject, offsets::kCollisionObject_OwnerNode, state.identity.ownerNode)) return false;
            if (state.identity.bodyId != id || !state.identity.valid() ||
                state.identity.collisionObject != reinterpret_cast<std::uintptr_t>(identity.collisionObject)) return false;
            sample.stage = "owner-world";
            RE::hknpWorld* ownerWorld = nullptr;
            RE::hknpBodyId ownerId{};
            if (!havok_runtime::tryResolveCollisionObjectBody(identity.collisionObject, ownerWorld, ownerId) ||
                ownerWorld != world || ownerId.value != id) return false;
            std::array<float, 16> pose{};
            std::memcpy(pose.data(), bytes.data(), sizeof(pose));
            sample.stage = "body-pose";
            for (std::size_t i = 0; i < 16; ++i) {
                // Rotation W lanes carry packed native data, not float axes.
                if ((i & 3u) != 3u && !std::isfinite(pose[i])) return false;
            }
            sample.pose.rotate = havokRotationBlocksToNiMatrix(pose.data());
            sample.pose.translate = {pose[12] * havokToGameScale(), pose[13] * havokToGameScale(), pose[14] * havokToGameScale()};
            sample.pose.scale = 1.0f;
            if (transform_math::storedRotationOrthonormalityError(sample.pose.rotate) > 0.001 ||
                !std::isfinite(sample.pose.translate.x) || !std::isfinite(sample.pose.translate.y) ||
                !std::isfinite(sample.pose.translate.z)) return false;
            sample.stage = "complete";
            return true;
        }

        // Source shape survives releasing the read lock and creating the wrapper.
        // BethesdaPhysicsBody then retains its own reference in the system data.
        struct ShapeLease
        {
            RE::hknpShape* shape = nullptr;
            ~ShapeLease() { havok_ref_count::release(shape); }
            ShapeLease() = default;
            ShapeLease(const ShapeLease&) = delete;
            ShapeLease& operator=(const ShapeLease&) = delete;
            bool retain(std::uintptr_t identity)
            {
                auto* candidate = reinterpret_cast<RE::hknpShape*>(identity);
                std::uintptr_t vtable = 0;
                std::uint32_t refs = 0;
                if ((identity & 7u) != 0 || !native_memory::tryReadField(candidate, 0, vtable) || !vtable ||
                    !native_memory::tryReadField(candidate, 8, refs) || (refs >> 16) == 0 || (refs & 0xFFFFu) == 0) return false;
                shape = candidate;
                havok_ref_count::addRef(shape);
                return true;
            }
        };

        RE::hkTransformf targetTransform(const RE::NiTransform& pose)
        {
            RE::hkTransformf target{};
            target.rotation = niRotToHkTransformRotation(pose.rotate);
            target.translation = RE::NiPoint4(pose.translate.x * gameToHavokScale(),
                pose.translate.y * gameToHavokScale(), pose.translate.z * gameToHavokScale(), 0.0f);
            return target;
        }

        bool withinRetainRadius(const RE::NiPoint3& position, const RE::NiPoint3& origin)
        {
            const auto delta = position - origin;
            const float squared = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
            return std::isfinite(squared) && squared <= kRetainGameUnits * kRetainGameUnits;
        }
    }

    void NpcCollisionRuntime::disable(Limb& limb)
    {
        if (!limb.collisionEnabled) return;
        limb.body.setCollisionFilterInfo(kDisabledProxyFilter, 0);
        limb.collisionEnabled = false;
    }

    void NpcCollisionRuntime::retire(Limb& limb, bool worldLive)
    {
        if (limb.body.isValid()) {
            const auto live = worldLive ? havok_runtime::snapshotBodyIdentity(_world, limb.body.getBodyId()) :
                havok_runtime::BodyIdentitySnapshot{};
            if (live.valid && live.collisionObject == limb.body.getCollisionObject()) limb.body.retireDeferred(_bhkWorld);
            else {
                if (worldLive) ROCK_LOG_WARN(Physics, "NPC supplemental collider ownership lost at retirement: body={}", limb.body.getBodyId().value);
                limb.body.reset();
            }
        }
        limb.collisionEnabled = false;
    }

    void NpcCollisionRuntime::release(ActorSlot& actor, bool worldLive)
    {
        for (auto& limb : actor.limbs) {
            retire(limb, worldLive);
            limb.source = {};
            limb.failed = false;
            limb.seen = false;
        }
        actor.handle = {};
        actor.formId = 0;
        actor.root = 0;
    }

    void NpcCollisionRuntime::clear(RE::hknpWorld* liveWorld, void* liveBhkWorld)
    {
        if (!_world) return;
        auto mutation = _gate ? _gate->pauseForMutation() : PhysicsCallbackQuiescenceGate::MutationLease{};
        const bool worldLive = liveWorld && liveWorld == _world && liveBhkWorld && liveBhkWorld == _bhkWorld;
        for (auto& actor : _actors) release(actor, worldLive);
        _world = nullptr;
        _bhkWorld = nullptr;
        _discoverySeconds = 0.0f;
        _mainFrameAgeSeconds = 0.0f;
        if (_lastPooledCount) ROCK_LOG_INFO(Physics, "NPC supplemental colliders cleared: pooled={} worldLive={}", _lastPooledCount, worldLive);
        _lastActiveCount = _lastPooledCount = 0;
    }

    void NpcCollisionRuntime::scan(ActorSlot& slot, RE::NiAVObject* root)
    {
        for (auto& limb : slot.limbs) limb.seen = false;
        std::array<RE::NiAVObject*, 512> nodes{};
        std::size_t queued = 1, cursor = 0;
        nodes[0] = root;
        bool truncated = false;
        std::size_t unreadable = 0;
        const char* failureStage = "none";
        {
            havok_world_lock::ScopedWorldReadLock readLock(_world);
            while (cursor < queued) {
                auto* node = nodes[cursor++];
                if (auto* collision = node->collisionObject.get()) {
                    RE::hknpWorld* ownerWorld = nullptr;
                    RE::hknpBodyId bodyId{};
                    Sample sample;
                    const bool resolved = havok_runtime::tryResolveCollisionObjectBody(collision, ownerWorld, bodyId) && ownerWorld == _world;
                    const bool readable = resolved && readSource(_world, bodyId.value, sample);
                    if (resolved && !readable) {
                        ++unreadable;
                        failureStage = sample.stage;
                    }
                    if (readable && sample.state.animatedLimb()) {
                        Limb* found = nullptr;
                        Limb* empty = nullptr;
                        for (auto& limb : slot.limbs) {
                            if (limb.source == sample.state.identity) { found = &limb; break; }
                            if (!limb.source.valid() && !empty) empty = &limb;
                        }
                        if (!found) {
                            found = empty;
                            if (found) found->source = sample.state.identity;
                        }
                        if (found) found->seen = true;
                        else truncated = true;
                    }
                }
                if (auto* branch = node->IsNode()) {
                    const auto& children = branch->GetRuntimeData().children;
                    const auto limit = (std::min)(std::size_t(children.size()), nodes.size());
                    truncated |= children.size() > limit;
                    for (decltype(children.size()) i = 0; i < limit; ++i) {
                        if (auto* child = children[i].get()) {
                            if (queued == nodes.size()) { truncated = true; break; }
                            nodes[queued++] = child;
                        }
                    }
                }
            }
        }
        for (auto& limb : slot.limbs) {
            if (!limb.seen) {
                retire(limb, true);
                limb.source = {};
                limb.failed = false;
            }
        }
        if (truncated) ROCK_LOG_SAMPLE_WARN(Physics, 5000,
            "NPC supplemental collision discovery truncated: actor={:08X} nodes={} capacity={}", slot.formId, cursor, kBodyCapacity);
        if (unreadable) ROCK_LOG_SAMPLE_WARN(Physics, 5000,
            "NPC supplemental collision discovery rejected sources: actor={:08X} count={} stage={}", slot.formId, unreadable, failureStage);
    }

    void NpcCollisionRuntime::discover(const PhysicsFrameContext& frame)
    {
        struct Candidate { RE::ActorHandle handle{}; std::uint32_t formId = 0; float distance = 0; };
        std::array<Candidate, kActorCapacity> candidates{};
        std::size_t count = 0;
        auto* processes = RE::ProcessLists::GetSingleton();
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!processes || !player) {
            for (auto& slot : _actors) release(slot, true);
            return;
        }
        const auto& handles = processes->highActorHandles;
        const auto limit = (std::min)(handles.size(), 256u);
        for (decltype(handles.size()) i = 0; i < limit; ++i) {
            auto actor = handles[i].get();
            if (!actor || actor.get() == player || actor->IsDeleted() || actor->IsDisabled() || actor->IsDead(false) || !actor->Get3D()) continue;
            const auto delta = actor->GetPosition() - frame.hmdPositionWorld;
            const float distance = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
            bool retained = false;
            for (const auto& slot : _actors) retained |= slot.formId == actor->GetFormID() && slot.handle == handles[i];
            const float reach = retained ? kRetainGameUnits : kReachGameUnits;
            if (!std::isfinite(distance) || distance > reach * reach) continue;
            std::size_t insert = 0;
            while (insert < count && candidates[insert].distance <= distance) ++insert;
            if (insert == candidates.size()) continue;
            if (count < candidates.size()) ++count;
            for (std::size_t j = count - 1; j > insert; --j) candidates[j] = candidates[j - 1];
            candidates[insert] = {handles[i], actor->GetFormID(), distance};
        }
        if (handles.size() > limit) ROCK_LOG_SAMPLE_WARN(Physics, 5000,
            "NPC supplemental collision actor scan truncated: actors={} scanned={}", handles.size(), limit);
        for (auto& slot : _actors) {
            bool wanted = false;
            for (std::size_t i = 0; i < count; ++i) wanted |= slot.formId == candidates[i].formId && slot.handle == candidates[i].handle;
            if (!wanted) release(slot, true);
        }
        for (std::size_t i = 0; i < count; ++i) {
            ActorSlot* found = nullptr;
            for (auto& slot : _actors) if (slot.formId == candidates[i].formId && slot.handle == candidates[i].handle) found = &slot;
            if (!found) for (auto& slot : _actors) if (!slot.formId) { found = &slot; break; }
            if (!found) continue;
            auto actor = candidates[i].handle.get();
            auto* root = actor ? actor->Get3D() : nullptr;
            if (!root) continue;
            if (found->root && found->root != reinterpret_cast<std::uintptr_t>(root)) release(*found, true);
            const bool firstScan = !found->formId;
            found->handle = candidates[i].handle;
            found->formId = candidates[i].formId;
            found->root = reinterpret_cast<std::uintptr_t>(root);
            scan(*found, root);
            if (firstScan) {
                std::size_t sources = 0;
                for (const auto& limb : found->limbs) sources += limb.source.valid();
                ROCK_LOG_INFO(Physics, "NPC supplemental source discovery: actor={:08X} limbs={}", found->formId, sources);
            }
        }
    }

    void NpcCollisionRuntime::update(const PhysicsFrameContext& frame, bool enabled)
    {
        if (!_companionChecked) {
            _companionChecked = true;
            _scissorsLoaded = GetModuleHandleW(L"SCISSORS.dll") != nullptr;
            if (_scissorsLoaded) ROCK_LOG_INFO(Physics, "NPC supplemental colliders yield to loaded SCISSORS");
        }
        if (!enabled || _scissorsLoaded || !frame.worldReady || !frame.hasHmdFrame || frame.menuBlocked) {
            clear(frame.hknpWorld, frame.bhkWorld);
            return;
        }
        auto mutation = _gate ? _gate->pauseForMutation() : PhysicsCallbackQuiescenceGate::MutationLease{};
        if (_world != frame.hknpWorld || _bhkWorld != frame.bhkWorld) {
            clear(nullptr, nullptr);
            _world = frame.hknpWorld;
            _bhkWorld = frame.bhkWorld;
        }
        _mainFrameAgeSeconds = 0.0f;
        if (_driveFailures || _staleFrames) {
            ROCK_LOG_WARN(Physics, "NPC supplemental collision fail-closed: driveFailures={} staleFrameSuppressions={}", _driveFailures, _staleFrames);
            _driveFailures = _staleFrames = 0;
        }
        _discoverySeconds -= std::isfinite(frame.deltaSeconds) ? (std::max)(0.0f, frame.deltaSeconds) : 0.0f;
        if (_discoverySeconds <= 0.0f) {
            discover(frame);
            _discoverySeconds = kDiscoverySeconds;
        }
        std::size_t createsRemaining = 8;
        for (auto& slot : _actors) {
            if (!slot.formId) continue;
            auto actor = slot.handle.get();
            if (!actor || actor->GetFormID() != slot.formId || actor->IsDeleted() || actor->IsDisabled() || actor->IsDead(false) ||
                reinterpret_cast<std::uintptr_t>(actor->Get3D()) != slot.root || !withinRetainRadius(actor->GetPosition(), frame.hmdPositionWorld)) {
                release(slot, true);
                continue;
            }
            std::size_t created = 0, retired = 0;
            std::array<Sample, kBodyCapacity> samples{};
            std::array<bool, kBodyCapacity> readable{};
            std::array<ShapeLease, kBodyCapacity> shapes{};
            std::size_t availableCreates = createsRemaining;
            {
                havok_world_lock::ScopedWorldReadLock readLock(_world);
                for (std::size_t i = 0; i < slot.limbs.size(); ++i) {
                    auto& limb = slot.limbs[i];
                    if (!limb.source.valid()) continue;
                    auto& sample = samples[i];
                    readable[i] = readSource(_world, limb.source.bodyId, sample);
                    if (readable[i] && sample.state.canDrive(limb.source) && !limb.failed && !limb.body.isValid() && availableCreates) {
                        if (!shapes[i].retain(sample.state.identity.shape)) {
                            limb.failed = true;
                            sample.stage = "shape-lifetime";
                        } else {
                            --availableCreates;
                        }
                    }
                }
            }
            for (std::size_t i = 0; i < slot.limbs.size(); ++i) {
                auto& limb = slot.limbs[i];
                if (!limb.source.valid()) continue;
                const auto& sample = samples[i];
                if (!readable[i] || sample.state.identity != limb.source || !sample.state.animatedLimb() || limb.failed) {
                    if (sample.stage == std::string_view{"shape-lifetime"}) {
                        ROCK_LOG_WARN(Physics, "NPC supplemental source rejected: actor={:08X} body={} stage=shape-lifetime",
                            slot.formId, limb.source.bodyId);
                    }
                    retired += limb.body.isValid();
                    retire(limb, true);
                    if (!readable[i] || sample.state.identity != limb.source) {
                        ROCK_LOG_SAMPLE_WARN(Physics, 2000, "NPC supplemental source lost: actor={:08X} body={} stage={}",
                            slot.formId, limb.source.bodyId, readable[i] ? "identity-changed" : sample.stage);
                        limb.source = {};
                        limb.failed = false;
                    }
                    continue;
                }
                if (!sample.state.needsSupplement()) {
                    // Keep the bounded, exact-owner pool across animation
                    // transitions. Collision is disabled by the pre-collision
                    // recheck; repeated transitions do not accumulate retired
                    // Bethesda collision-wrapper tombstones.
                    continue;
                }
                if (limb.body.isValid() || !shapes[i].shape) continue;
                --createsRemaining;
                if (!limb.body.create(_world, _bhkWorld, shapes[i].shape, kDisabledProxyFilter, sample.material,
                        BethesdaMotionType::Keyframed, "ROCK_NpcSupplement") ||
                    !placeGeneratedKeyframedBodyImmediately(limb.body, sample.pose)) {
                    retire(limb, true);
                    limb.failed = true;
                    ROCK_LOG_WARN(Physics, "NPC supplemental collider creation failed: actor={:08X} source={}", slot.formId, limb.source.bodyId);
                    continue;
                }
                // Publish collision only after the first pre-collision source
                // recheck. Native ownership may change while creation runs.
                ++created;
            }
            if (created || retired) ROCK_LOG_INFO(Physics, "NPC supplemental colliders: actor={:08X} created={} retired={}", slot.formId, created, retired);
        }
        std::size_t active = 0, pooled = 0;
        for (const auto& actor : _actors) for (const auto& limb : actor.limbs) {
            pooled += limb.body.isValid();
            active += limb.collisionEnabled;
        }
        if (active != _lastActiveCount || pooled != _lastPooledCount) {
            ROCK_LOG_SAMPLE_INFO(Physics, 1000, "NPC supplemental collision state: active={} pooled={}", active, pooled);
            _lastActiveCount = active;
            _lastPooledCount = pooled;
        }
    }

    void NpcCollisionRuntime::drive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!world || world != _world || timing.phase != havok_physics_timing::PhysicsStepPhase::SubstepPreCollide) return;
        float delta = 0.0f;
        const bool timed = havok_physics_timing::tryGetDriveDeltaSeconds(timing, delta);
        if (timed) _mainFrameAgeSeconds += delta;
        const bool fresh = timed && !generated_keyframed_body_drive_math::sourceIsStale(_mainFrameAgeSeconds);
        for (auto& actor : _actors) for (auto& limb : actor.limbs) {
            if (!limb.body.isValid()) continue;
            const auto proxy = havok_runtime::snapshotBodyIdentity(world, limb.body.getBodyId());
            if (!proxy.valid || proxy.collisionObject != limb.body.getCollisionObject()) {
                limb.body.reset(); // An unrelated/recycled slot must never be written.
                limb.failed = true;
                limb.collisionEnabled = false;
                ++_driveFailures;
                continue;
            }
            Sample sample;
            if (!fresh || limb.failed || !readSource(world, limb.source.bodyId, sample) || !sample.state.canDrive(limb.source)) {
                if (!fresh && limb.collisionEnabled) ++_staleFrames;
                disable(limb);
                continue;
            }
            // Read the native animated body immediately before collision. This
            // uses its actual per-step pose, not player/skeleton callback order
            // or a stored actor/bone pointer. Native keyframe drive supplies the
            // moving obstacle velocity and handles discontinuous placement.
            const bool driven = limb.collisionEnabled ?
                limb.body.driveToKeyFrame(targetTransform(sample.pose), delta) :
                placeGeneratedKeyframedBodyImmediately(limb.body, sample.pose);
            if (!driven) {
                disable(limb);
                limb.failed = true;
                ++_driveFailures;
                continue;
            }
            if (!limb.collisionEnabled) {
                limb.body.setCollisionFilterInfo(kProxyFilter, 0);
                limb.collisionEnabled = true;
            }
        }
    }
}
