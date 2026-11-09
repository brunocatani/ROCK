#include "physics-interaction/weapon/equip/LodgedWeapons.h"

#include <cmath>

#include "api/NativeInteractionService.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/native/havok/BethesdaPhysicsBody.h"
#include "physics-interaction/native/havok/GeneratedKeyframedBodyDrive.h"
#include "physics-interaction/native/havok/HavokRuntime.h"

namespace rock
{
    namespace
    {
        constexpr std::uint32_t kInvalidBody = 0x7FFF'FFFFu;
        constexpr std::uint32_t kGroupMask = 0xFFFF'0000u;

        bool finite(const RE::NiTransform& value)
        {
            for (int row = 0; row < 3; ++row)
                if (!std::isfinite(value.rotate.entry[row].x) || !std::isfinite(value.rotate.entry[row].y) ||
                    !std::isfinite(value.rotate.entry[row].z))
                    return false;
            return std::isfinite(value.translate.x) && std::isfinite(value.translate.y) &&
                std::isfinite(value.translate.z);
        }

        // The body, if it is still the one with this collision object.
        bool sameBody(RE::hknpWorld* world, std::uint32_t id, std::uintptr_t collisionObject)
        {
            if (id == kInvalidBody || !collisionObject)
                return false;
            const auto live = havok_runtime::snapshotBody(world, RE::hknpBodyId{ id });
            return live.valid && live.body && reinterpret_cast<std::uintptr_t>(live.collisionObject) == collisionObject;
        }

        bool bodyWorld(RE::hknpWorld* world, std::uint32_t id, RE::NiTransform& out)
        {
            return havok_runtime::tryGetBodyArrayWorldTransform(world, RE::hknpBodyId{ id }, out) && finite(out);
        }
    }

    bool LodgedWeapons::capture(RE::hknpWorld* world, std::uint32_t slot, Capture& out)
    {
        out = {};
        api::native_interaction::BodyIdentity target{};
        if (!world || !native_interaction_service::weaponLodge(slot, target))
            return false;
        const auto live = havok_runtime::snapshotBody(world, RE::hknpBodyId{ target.body });
        if (!live.valid || !live.body || !live.ownerNode ||
            reinterpret_cast<std::uintptr_t>(live.collisionObject) != target.collisionObject ||
            reinterpret_cast<std::uintptr_t>(live.ownerNode) != target.ownerNode) {
            ROCK_LOG_INFO(Weapon, "Lodged weapon refused: slot={} target={} is no longer that body", slot, target.body);
            return false;
        }
        // Without a group the filter cannot keep the weapon off him.
        const auto group = live.collisionFilterInfo & kGroupMask;
        if (!group) {
            ROCK_LOG_INFO(Weapon, "Lodged weapon refused: slot={} target={} has no collision group", slot, target.body);
            return false;
        }
        out.valid = true;
        out.slot = slot;
        out.body = target.body;
        out.collisionObject = target.collisionObject;
        out.group = group;
        out.bone.reset(live.ownerNode);
        return true;
    }

    bool LodgedWeapons::anchor(Entry& entry, RE::hknpWorld* world, std::span<const PlacedBody> placed, bool usePlaced)
    {
        RE::NiTransform target{};
        if (!bodyWorld(world, entry.target.body, target))
            return false;
        const auto inverse = transform_math::invertTransform(target);
        for (std::uint32_t i = 0; i < entry.count; ++i) {
            auto& body = entry.bodies[i];
            if (!body.drive)
                continue;
            RE::NiTransform at{};
            if (usePlaced) {
                at = placed[i].placed;
            } else if (!bodyWorld(world, body.id, at)) {
                return false;
            }
            body.inTarget = transform_math::composeTransforms(inverse, at);
            if (!finite(body.inTarget))
                return false;
        }
        return true;
    }

    bool LodgedWeapons::begin(RE::hknpWorld* world, const RE::ObjectRefHandle& reference, std::uint32_t form,
        const Capture& target, std::span<const PlacedBody> bodies)
    {
        if (!world || !target.valid || bodies.empty() || bodies.size() > kMaxBodies || !reference)
            return false;
        Entry* entry = nullptr;
        for (auto& candidate : _entries)
            if (candidate.state.load(std::memory_order_acquire) != State::Free && candidate.form == form) {
                end(candidate, candidate.world == world ? world : nullptr, "placement-restarted", false);
            }
        for (auto& candidate : _entries)
            if (candidate.state.load(std::memory_order_acquire) == State::Free) {
                entry = &candidate;
                break;
            }
        if (!entry) {
            ROCK_LOG_WARN(Weapon, "Lodged weapon refused: ref={:08X} all {} lodges in use", form, kMaxLodges);
            return false;
        }
        entry->world = world;
        entry->reference = reference;
        entry->form = form;
        entry->target = target;
        entry->count = 0;
        entry->failure.store(Failure::None, std::memory_order_relaxed);
        entry->offSeat.store(0, std::memory_order_relaxed);
        bool drives = false;
        for (const auto& placed : bodies) {
            const auto live = havok_runtime::snapshotBody(world, RE::hknpBodyId{ placed.id });
            if (!live.valid || !live.body || !live.collisionObject)
                return false;
            auto& body = entry->bodies[entry->count++];
            body = {};
            body.id = placed.id;
            body.collisionObject = reinterpret_cast<std::uintptr_t>(live.collisionObject);
            body.filter = live.collisionFilterInfo;
            body.drive = placed.drive;
            body.lastSeat = placed.placed;
            drives = drives || placed.drive;
        }
        if (!drives || !anchor(*entry, world, bodies, true)) {
            ROCK_LOG_WARN(Weapon, "Lodged weapon refused: ref={:08X} target={} pose unavailable", form, target.body);
            entry->count = 0;
            return false;
        }
        for (std::uint32_t i = 0; i < entry->count; ++i) {
            const auto& body = entry->bodies[i];
            havok_runtime::setFilterInfo(world, RE::hknpBodyId{ body.id }, (body.filter & ~kGroupMask) | target.group);
        }
        entry->state.store(State::Driving, std::memory_order_release);
        ROCK_LOG_INFO(Weapon, "Lodged weapon: ref={:08X} slot={} target={} bone={} group={:04X} bodies={}", form,
            target.slot, target.body, target.bone && target.bone->name.c_str() ? target.bone->name.c_str() : "",
            target.group >> 16, entry->count);
        return true;
    }

    const char* LodgedWeapons::follow(Entry& entry, RE::hknpWorld* world)
    {
        // The bone's current body: an actor's limb changes bodies when his
        // ragdoll is added, removed or replaced by the engine's. Returns the
        // stage that failed, or nullptr.
        auto* bone = entry.target.bone.get();
        auto* collisionObject = bone ? bone->collisionObject.get() : nullptr;
        if (!collisionObject)
            return bone ? "bone-without-collision" : "bone-gone";
        RE::hknpWorld* bodyWorldPtr = nullptr;
        RE::hknpBodyId id{ kInvalidBody };
        if (!havok_runtime::tryResolveCollisionObjectBody(collisionObject, bodyWorldPtr, id))
            return "bone-body-unresolved";
        if (bodyWorldPtr != world)
            return "bone-body-other-world";
        const auto live = havok_runtime::snapshotBody(world, id);
        const auto group = live.collisionFilterInfo & kGroupMask;
        if (!live.valid || !live.body || reinterpret_cast<RE::NiCollisionObject*>(live.collisionObject) != collisionObject)
            return "bone-body-unreadable";
        if (!group)
            return "bone-body-without-group";
        const auto previous = entry.target.body;
        entry.target.body = id.value;
        entry.target.collisionObject = reinterpret_cast<std::uintptr_t>(collisionObject);
        // Re-seat where the weapon is now on the new body.
        if (!anchor(entry, world, {}, false))
            return "re-seat-pose";
        if (group != entry.target.group) {
            entry.target.group = group;
            for (std::uint32_t i = 0; i < entry.count; ++i) {
                const auto& body = entry.bodies[i];
                havok_runtime::setFilterInfo(world, RE::hknpBodyId{ body.id }, (body.filter & ~kGroupMask) | group);
            }
        }
        ROCK_LOG_INFO(Weapon, "Lodged weapon followed its bone: ref={:08X} bone={} body {}->{} group={:04X}",
            entry.form, bone->name.c_str() ? bone->name.c_str() : "", previous, id.value, group >> 16);
        return nullptr;
    }

    void LodgedWeapons::end(Entry& entry, RE::hknpWorld* liveWorld, const char* reason, bool dropsLoose)
    {
        entry.state.store(State::Free, std::memory_order_release);
        std::uint32_t restored = 0;
        if (liveWorld && entry.world == liveWorld) {
            // A weapon that falls loose leaves with the body it was in, never
            // with the drive's last catch-up velocity (which can be large when
            // that body was knocked away).
            RE::hkVector4f carried{};
            if (dropsLoose && sameBody(liveWorld, entry.target.body, entry.target.collisionObject)) {
                const auto target = havok_runtime::snapshotBody(liveWorld, RE::hknpBodyId{ entry.target.body });
                if (target.motion && std::isfinite(target.motion->linearVelocity.x) &&
                    std::isfinite(target.motion->linearVelocity.y) && std::isfinite(target.motion->linearVelocity.z))
                    carried = RE::hkVector4f{ target.motion->linearVelocity.x, target.motion->linearVelocity.y,
                        target.motion->linearVelocity.z, 0.0f };
            }
            for (std::uint32_t i = 0; i < entry.count; ++i) {
                const auto& body = entry.bodies[i];
                // Only the same body gets its filter back; a reused id is not ours.
                if (sameBody(liveWorld, body.id, body.collisionObject) &&
                    havok_runtime::setFilterInfo(liveWorld, RE::hknpBodyId{ body.id }, body.filter)) {
                    if (dropsLoose && body.drive)
                        havok_runtime::setBodyVelocityDeferred(liveWorld, body.id, carried, RE::hkVector4f{});
                    havok_runtime::activateBody(liveWorld, body.id);
                    ++restored;
                }
            }
        }
        ROCK_LOG_INFO(Weapon, "Lodged weapon released: ref={:08X} reason={} restored={}/{}", entry.form, reason,
            restored, entry.count);
        entry.world = nullptr;
        entry.reference = {};
        entry.form = 0;
        entry.target = {};
        entry.count = 0;
        entry.failure.store(Failure::None, std::memory_order_relaxed);
        entry.offSeat.store(0, std::memory_order_relaxed);
        entry.lostPending = false;
    }

    void LodgedWeapons::update(RE::hknpWorld* world, std::span<const RE::TESObjectREFR* const> handReferences)
    {
        for (auto& entry : _entries) {
            const auto state = entry.state.load(std::memory_order_acquire);
            if (state == State::Free)
                continue;
            if (entry.world != world) {
                end(entry, nullptr, "world-changed", false);
                continue;
            }
            const auto reference = entry.reference.get();
            if (!reference || reference->IsDeleted() || reference->IsDisabled() || !reference->Get3D()) {
                // Picked up, equipped, unloaded: its bodies are not ours to touch.
                end(entry, nullptr, "reference-gone", false);
                continue;
            }
            bool inHand = false;
            for (const auto* held : handReferences)
                inHand = inHand || held == reference.get();
            if (inHand) {
                // The hand drives it now, with the weapon's own collision.
                end(entry, world, "taken-by-hand", false);
                continue;
            }
            switch (entry.failure.load(std::memory_order_acquire)) {
            case Failure::None:
                break;
            case Failure::TargetLost: {
                // The engine swaps a limb's body when his ragdoll changes and
                // adds the new one a few frames later; the step holds the
                // weapon where it was meanwhile.
                const auto now = std::chrono::steady_clock::now();
                if (!entry.lostPending) {
                    entry.lostPending = true;
                    entry.lostSince = now;
                }
                if (const char* stage = follow(entry, world)) {
                    if (now - entry.lostSince < kFollowGrace)
                        break;
                    const auto* bone = entry.target.bone.get();
                    ROCK_LOG_INFO(Weapon, "Lodged weapon target gone: ref={:08X} body={} bone={} lost={} follow={}",
                        entry.form, entry.target.body, bone && bone->name.c_str() ? bone->name.c_str() : "",
                        entry.lost.load(std::memory_order_relaxed) == 1 ? "identity" : "pose", stage);
                    end(entry, world, "target-gone", true);
                } else {
                    entry.lostPending = false;
                    entry.offSeat.store(0, std::memory_order_relaxed);
                    entry.failure.store(Failure::None, std::memory_order_release);
                }
                break;
            }
            case Failure::BodyLost:
                end(entry, world, "weapon-body-changed", true);
                break;
            case Failure::PushedOff: {
                // Off by how far, and how fast his body moved: a push, or a
                // body the drive could not keep up with.
                float targetSpeed = -1.0f;
                if (sameBody(world, entry.target.body, entry.target.collisionObject)) {
                    const auto target = havok_runtime::snapshotBody(world, RE::hknpBodyId{ entry.target.body });
                    if (target.motion) {
                        const auto& v = target.motion->linearVelocity;
                        targetSpeed = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
                    }
                }
                ROCK_LOG_INFO(Weapon, "Lodged weapon pushed off: ref={:08X} off={:.1f}gu for {:.3f}s targetSpeed={:.2f}m/s",
                    entry.form, entry.offSeatGame.load(std::memory_order_relaxed),
                    entry.offSeat.load(std::memory_order_relaxed), targetSpeed);
                end(entry, world, "pushed-off", true);
                break;
            }
            }
        }
    }

    void LodgedWeapons::step(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing) noexcept
    {
        float delta = 0.0f;
        if (!world || !havok_physics_timing::tryGetDriveDeltaSeconds(timing, delta))
            return;
        // Drives one weapon body to a world pose for this substep.
        const auto drive = [&](const Body& body, const RE::NiTransform& seat) {
            alignas(16) float linear[4]{};
            alignas(16) float angular[4]{};
            const auto live = havok_runtime::snapshotBody(world, RE::hknpBodyId{ body.id });
            if (live.collisionObject &&
                computeHardKeyFrameVelocity(world, RE::hknpBodyId{ body.id }, seat, delta, linear, angular))
                setCollisionObjectVelocity(live.collisionObject, linear, angular);
        };
        for (auto& entry : _entries) {
            if (entry.state.load(std::memory_order_acquire) != State::Driving || entry.world != world)
                continue;
            const auto failure = entry.failure.load(std::memory_order_acquire);
            if (failure == Failure::TargetLost) {
                // Hold it where it last sat while the frame thread looks for
                // the bone's new body.
                for (std::uint32_t i = 0; i < entry.count; ++i)
                    if (entry.bodies[i].drive && sameBody(world, entry.bodies[i].id, entry.bodies[i].collisionObject))
                        drive(entry.bodies[i], entry.bodies[i].lastSeat);
                continue;
            }
            if (failure != Failure::None)
                continue;
            RE::NiTransform target{};
            const bool same = sameBody(world, entry.target.body, entry.target.collisionObject);
            if (!same || !bodyWorld(world, entry.target.body, target)) {
                entry.lost.store(same ? 2 : 1, std::memory_order_relaxed);
                entry.failure.store(Failure::TargetLost, std::memory_order_release);
                continue;
            }
            float worst = 0.0f;
            for (std::uint32_t i = 0; i < entry.count; ++i) {
                auto& body = entry.bodies[i];
                if (!sameBody(world, body.id, body.collisionObject)) {
                    entry.failure.store(Failure::BodyLost, std::memory_order_release);
                    break;
                }
                if (!body.drive)
                    continue;
                RE::NiTransform current{};
                if (!bodyWorld(world, body.id, current))
                    continue;
                const auto seat = transform_math::composeTransforms(target, body.inTarget);
                const auto error = seat.translate - current.translate;
                worst = (std::max)(worst, error.Length());
                body.lastSeat = seat;
                drive(body, seat);
            }
            // Pushed off its seat (a wall, a floor under a corpse, a hit) for
            // long enough: it comes out instead of fighting the push. Only
            // this thread writes the off-seat clock.
            if (worst > kBreakGame) {
                const float off = entry.offSeat.load(std::memory_order_relaxed) + delta;
                entry.offSeat.store(off, std::memory_order_relaxed);
                entry.offSeatGame.store(worst, std::memory_order_relaxed);
                if (off >= kBreakSeconds)
                    entry.failure.store(Failure::PushedOff, std::memory_order_release);
            } else {
                entry.offSeat.store(0.0f, std::memory_order_relaxed);
            }
        }
    }

    void LodgedWeapons::clear(RE::hknpWorld* liveWorld)
    {
        for (auto& entry : _entries)
            if (entry.state.load(std::memory_order_acquire) != State::Free)
                end(entry, liveWorld, "cleared", false);
    }

    bool LodgedWeapons::lodged(std::uint32_t form) const noexcept
    {
        for (const auto& entry : _entries)
            if (entry.state.load(std::memory_order_acquire) == State::Driving && entry.form == form)
                return true;
        return false;
    }
}
