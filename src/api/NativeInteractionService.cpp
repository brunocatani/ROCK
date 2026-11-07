#include <algorithm>
#include <cstring>
#include <limits>
#include <span>
#include "NativeInteractionState.h"
#include "ProviderRuntimeServices.h"
#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/native/havok/HavokRuntime.h"
#include "physics-interaction/native/havok/PhysicsUtils.h"
#include "physics-interaction/native/player/NativePlayerCollisionFilter.h"
#include "physics-interaction/weapon/collision/WeaponCollision.h"
#include "providers/Boundary.h"

namespace rock::native_interaction_service
{
    using api::Status;
    namespace ni = api::native_interaction;

    namespace detail
    {
        State& state()
        {
            static State value;
            return value;
        }

        thread_local const CallContext* current = nullptr;

        Extension* extension(Owner owner)
        {
            for (auto& slot : state().extensions)
                if (owner && slot.owner == owner)
                    return &slot;
            return nullptr;
        }

        Status access(Owner owner, bool structural, bool write)
        {
            if (!current || current->owner != owner || !extension(owner))
                return Status::WrongThread;
            const auto phase = current->frame.phase;
            const bool retire = phase == ni::Phase::Retire || phase == ni::Phase::WeaponRetire;
            const auto index = static_cast<unsigned>(extension(owner) - state().extensions.data());
            if (!retire && (state().faultedExtensions.load(std::memory_order_acquire) & (1u << index)))
                return Status::OwnerRevoked;
            const bool mutation = retire || phase == ni::Phase::Update || phase == ni::Phase::WeaponUpdate ||
                phase == ni::Phase::Extraction;
            if ((structural && !mutation) || (write && !mutation && phase != ni::Phase::PreCollide))
                return Status::WrongThread;
            if (!current->frame.world && !retire)
                return Status::WorldNotReady;
            return Status::Ok;
        }
    }

    using namespace detail;

    namespace
    {
        ni::Frame makeFrame(const PhysicsFrameContext& context, ni::Phase phase)
        {
            ni::Frame result{};
            result.phase = phase;
            result.sample = provider::runtime::sample();
            result.sample.frameIndex = provider::runtime::currentGameFrameIndex();
            result.world = reinterpret_cast<std::uintptr_t>(context.hknpWorld);
            result.bhkWorld = reinterpret_cast<std::uintptr_t>(context.bhkWorld);
            result.hmdPosition = toApi(context.hmdPositionWorld);
            result.deltaSeconds = context.deltaSeconds;
            result.gameToHavok = gameToHavokScale();
            result.havokToGame = havokToGameScale();
            result.worldReady = context.worldReady;
            result.menuBlocked = context.menuBlocked;
            result.hmdValid = context.hasHmdFrame;
            return result;
        }

        ni::Frame stepFrame(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
        {
            auto frame = state().frame;
            frame.phase = ni::Phase::PreCollide;
            frame.world = reinterpret_cast<std::uintptr_t>(world);
            frame.substepSeconds = timing.substepDeltaSeconds;
            frame.timingValid = timing.valid;
            frame.timingFallback = timing.usedFallback;
            return frame;
        }

        // Keep SEH outside the RAII scope, so quiescence and callback context
        // unwind normally when an extension faults in native code.
        bool invokeProtected(void (*operation)(void*), void* context)
        {
#if defined(_MSC_VER)
            __try {
                operation(context);
                return true;
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
#else
            operation(context);
            return true;
#endif
        }

        bool disabled(const Extension& slot)
        {
            const auto& service = state();
            if (service.faultedExtensions.load(std::memory_order_acquire) &
                (1u << static_cast<unsigned>(&slot - service.extensions.data())))
                return true;
            if (service.removeAll.load(std::memory_order_acquire))
                return true;
            for (const auto& pending : service.pendingRemoval)
                if (pending.load(std::memory_order_acquire) == slot.owner)
                    return true;
            return false;
        }

        template <class F>
        bool invoke(Extension& slot, const ni::Frame& frame, F&& operation)
        {
            const bool cleanup = frame.phase == ni::Phase::Retire || frame.phase == ni::Phase::WeaponRetire;
            if (!cleanup && disabled(slot))
                return false;
            ContextScope scope(slot.owner, frame);
            const auto status = guarded(slot.owner, [&] {
                if (invokeProtected([](void* value) { (*static_cast<std::remove_reference_t<F>*>(value))(); },
                        &operation))
                    return Status::Ok;
                state().faultedExtensions.fetch_or(1u << static_cast<unsigned>(&slot - state().extensions.data()),
                    std::memory_order_release);
                return Status::InternalError;
            });
            return status == Status::Ok;
        }

        void combineMasks(RE::hknpWorld* world = nullptr)
        {
            std::uint64_t mask = 0;
            for (const auto& slot : state().extensions)
                if (slot.owner)
                    mask |= slot.obstacleLayers;
            const auto previous = state().obstacleLayers.exchange(mask, std::memory_order_acq_rel);
            if (previous != mask && world && state().refreshLayers)
                state().refreshLayers(state().retireContext, world);
        }

        // Publishes the bodies every extension lets pass static bodies. The
        // filter rebuilds the caches of those that changed; a null world clears.
        void combineStaticPass(RE::hknpWorld* world)
        {
            std::array<std::uint32_t, ni::kMaxStaticContactPassBodies> bodies{};
            std::size_t count = 0;
            if (world)
                for (const auto& slot : state().extensions)
                    if (slot.owner)
                        for (std::uint32_t i = 0; i < slot.staticPassCount && count < bodies.size(); ++i)
                            bodies[count++] = slot.staticPass[i];
            native_player_collision::publishStaticPass(world, std::span(bodies).first(count));
        }

        // The slot's proxy must be retired before its driver goes when the
        // driver controls it or owns a contact exclusion on it.
        bool needsWeaponRetire(std::uint32_t slot)
        {
            const auto& weapon = state().weapons[slot];
            return weapon.command.result == ni::DriveResult::Controlled || weapon.excludedTarget != ni::kInvalidBody;
        }

        void retireDrivenWeapons(void* bhk)
        {
            auto& service = state();
            if (!service.driver || !service.retireWeapon)
                return;
            for (std::uint32_t slot = 0; slot < service.weapons.size(); ++slot)
                if (needsWeaponRetire(slot))
                    service.retireWeapon(service.retireContext, slot, bhk);
        }

        Status registerExtension(Owner owner, const ni::Callbacks* callbacks) noexcept
        {
            return guarded(owner, [&] {
                if (!callbacks)
                    return Status::InvalidArgument;
                if (callbacks->size != sizeof(*callbacks))
                    return Status::InvalidSize;
                if (callbacks->version != 1)
                    return Status::UnsupportedVersion;
                if (current)
                    return Status::Busy;
                if (callbacks->weaponUpdate && (!callbacks->weaponRetire || !callbacks->weaponPreCollide))
                    return Status::InvalidArgument;
                if (callbacks->extractSelection && (!callbacks->classifySelection || !callbacks->validateSelection))
                    return Status::InvalidArgument;
                if (auto s = provider::runtime::authorize(owner, ni::kInterfaceId, 3, true); s != Status::Ok)
                    return s;
                auto& service = state();
                if (!service.rockGate)
                    return Status::NotReady;
                auto rock = service.rockGate->pauseForMutation();
                auto mutation = service.gate.pauseForMutation();
                if (extension(owner))
                    return Status::OwnerConflict;
                if (callbacks->weaponUpdate && service.driver)
                    return Status::OwnerConflict;
                for (auto& slot : service.extensions)
                    if (!slot.owner) {
                        slot.owner = owner;
                        slot.callbacks = *callbacks;
                        slot.obstacleLayers = 0;
                        service.registeredOwners[&slot - service.extensions.data()].store(owner,
                            std::memory_order_release);
                        service.faultedExtensions.fetch_and(
                            ~(1u << static_cast<unsigned>(&slot - service.extensions.data())),
                            std::memory_order_release);
                        if (callbacks->weaponUpdate)
                            service.driver = owner;
                        ROCK_LOG_INFO(Physics,
                            "Native interaction extension registered: owner={} weaponDriver={} selection={}", owner,
                            callbacks->weaponUpdate != nullptr, callbacks->classifySelection != nullptr);
                        return Status::Ok;
                    }
                return Status::CapacityFull;
            });
        }

        Status unregisterExtension(Owner owner) noexcept
        {
            return guarded(owner, [&] {
                if (current)
                    return Status::Busy;
                if (auto s = provider::runtime::authorize(owner, ni::kInterfaceId, 0, true); s != Status::Ok)
                    return s;
                clearOwner(owner);
                return Status::Ok;
            });
        }

        Status setDynamicObstacleLayers(Owner owner, std::uint64_t mask) noexcept
        {
            return guarded(owner, [&] {
                if (auto s = access(owner, true, true); s != Status::Ok)
                    return s;
                // Only optional obstacle pairs are delegated. Native
                // actor/controller and generated self-collision policy stays core.
                if (mask & ~collision_layer_policy::kOptionalObstacleLayers)
                    return Status::InvalidArgument;
                extension(owner)->obstacleLayers = mask;
                combineMasks(reinterpret_cast<RE::hknpWorld*>(current->frame.world));
                return Status::Ok;
            });
        }

        Status setStaticContactPass(Owner owner, const std::uint32_t* bodies, std::uint32_t count) noexcept
        {
            return guarded(owner, [&] {
                if (auto s = access(owner, true, true); s != Status::Ok)
                    return s;
                if (current->frame.phase != ni::Phase::Update)
                    return Status::WrongThread;
                if (count && !bodies)
                    return Status::InvalidArgument;
                auto* self = extension(owner);
                std::uint64_t others = 0;
                for (const auto& slot : state().extensions)
                    if (slot.owner && &slot != self)
                        others += slot.staticPassCount;
                if (others + count > ni::kMaxStaticContactPassBodies)
                    return Status::CapacityFull;
                std::copy_n(bodies, count, self->staticPass.begin());
                self->staticPassCount = count;
                combineStaticPass(reinterpret_cast<RE::hknpWorld*>(current->frame.world));
                return Status::Ok;
            });
        }

        Status copyWeaponSources(Owner owner, std::uint32_t slot, ni::WeaponSource* output, std::uint32_t capacity,
            std::uint32_t* count) noexcept
        {
            return guarded(owner, [&] {
                if (!count)
                    return Status::InvalidArgument;
                *count = 0;
                if (capacity > ni::kMaxWeaponSources)
                    return Status::CapacityFull;
                if (capacity && !output)
                    return Status::InvalidArgument;
                for (std::uint32_t i = 0; i < capacity; ++i) {
                    if (output[i].size != sizeof(output[i]))
                        return Status::InvalidSize;
                    if (output[i].version != 1)
                        return Status::UnsupportedVersion;
                    output[i] = {};
                }
                if (auto s = access(owner); s != Status::Ok)
                    return s;
                if (auto s = api::boundary::checkWeaponSlot(slot); s != Status::Ok)
                    return s;
                const auto& service = state();
                const auto& weapon = service.weapons[slot];
                if (service.driver != owner || !weapon.collision || current->frame.phase != ni::Phase::WeaponUpdate)
                    return Status::WrongThread;
                std::uint32_t total = 0;
                for (const auto& source : weapon.collision->getProfileEvidenceDescriptors()) {
                    if (!source.valid || source.weaponGenerationKey != weapon.generation)
                        continue;
                    if (source.sourceName.size() >= sizeof(ni::WeaponSource::name)) {
                        for (std::uint32_t i = 0; i < capacity; ++i)
                            output[i] = {};
                        return Status::TargetInvalid;
                    }
                    if (total < capacity) {
                        auto& value = output[total];
                        value.body = source.bodyId;
                        value.generation = source.weaponGenerationKey;
                        std::snprintf(value.name, sizeof(value.name), "%s", source.sourceName.c_str());
                    }
                    ++total;
                }
                *count = total;
                return Status::Ok;
            });
        }

        Status getSourceGeometry(Owner owner, std::uint32_t slot, std::uint32_t body, const api::Point3* direction,
            ni::SourceGeometry* output) noexcept
        {
            return guarded(owner, [&] {
                if (!output)
                    return Status::InvalidArgument;
                if (output->size != sizeof(*output))
                    return Status::InvalidSize;
                if (output->version != 1)
                    return Status::UnsupportedVersion;
                *output = {};
                if (auto s = access(owner); s != Status::Ok)
                    return s;
                if (auto s = api::boundary::checkWeaponSlot(slot); s != Status::Ok)
                    return s;
                const auto& service = state();
                const auto& weapon = service.weapons[slot];
                if (service.driver != owner || !weapon.collision || !weapon.weaponNode ||
                    current->frame.phase != ni::Phase::WeaponUpdate)
                    return Status::WrongThread;
                if (!finite(toApi(weapon.weaponNode->world)))
                    return Status::TargetInvalid;
                WeaponCollision::SupportGripEvidenceView view{};
                if (!weapon.collision->tryGetSupportGripEvidenceView(body, weapon.weaponNode, view) ||
                    !view.sourceNodeCurrent || view.weaponGenerationKey != weapon.generation)
                    return Status::TargetInvalid;
                output->generation = weapon.generation;
                output->sourceInWeapon = toApi(transform_math::composeTransforms(
                    transform_math::invertTransform(weapon.weaponNode->world), view.localToWorld));
                if (!finite(output->sourceInWeapon))
                    return Status::TargetInvalid;
                if (direction) {
                    if (!std::isfinite(direction->x) || !std::isfinite(direction->y) || !std::isfinite(direction->z) ||
                        view.localTriangles.empty() || view.localTriangles.size() > 10000)
                        return Status::TargetInvalid;
                    constexpr float largest = (std::numeric_limits<float>::max)();
                    api::Point3 low{ largest, largest, largest }, high{ -largest, -largest, -largest };
                    float support = -largest;
                    for (const auto& triangle : view.localTriangles)
                        for (const auto& p : { triangle.v0, triangle.v1, triangle.v2 }) {
                            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                                return Status::TargetInvalid;
                            low.x = (std::min)(low.x, p.x);
                            low.y = (std::min)(low.y, p.y);
                            low.z = (std::min)(low.z, p.z);
                            high.x = (std::max)(high.x, p.x);
                            high.y = (std::max)(high.y, p.y);
                            high.z = (std::max)(high.z, p.z);
                            const float projection = p.x * direction->x + p.y * direction->y + p.z * direction->z;
                            if (projection > support) {
                                support = projection;
                                output->supportPointLocal = toApi(p);
                            }
                        }
                    output->boundsLocal = { low, high, 1, 0 };
                    output->triangleCount = static_cast<std::uint32_t>(view.localTriangles.size());
                }
                output->valid = 1;
                return Status::Ok;
            });
        }
    }

    void initialize(PhysicsCallbackQuiescenceGate* gate, RetireWeapon retire, RefreshLayers refresh, void* context)
    {
        state().rockGate = gate;
        state().retireWeapon = retire;
        state().refreshLayers = refresh;
        state().retireContext = context;
        state().gate.resumeCallbacks();
    }

    void detach(PhysicsCallbackQuiescenceGate* gate)
    {
        auto& service = state();
        if (service.rockGate != gate)
            return;
        service.gate.pauseAndWait();
        service.rockGate = nullptr;
        service.retireWeapon = nullptr;
        service.refreshLayers = nullptr;
        service.retireContext = nullptr;
        service.frame = {};
        service.resetWeapons();
    }

    void update(const PhysicsFrameContext& context)
    {
        auto& service = state();
        if (!service.rockGate)
            return;
        auto rock = service.rockGate->pauseForMutation();
        auto mutation = service.gate.pauseForMutation();
        service.frame = makeFrame(context, ni::Phase::Update);
        const bool removeAll = service.removeAll.exchange(false, std::memory_order_acq_rel);
        for (auto& pending : service.pendingRemoval)
            if (const auto owner = pending.exchange(0, std::memory_order_acq_rel))
                clearOwner(owner);
        if (removeAll)
            for (auto& slot : service.extensions)
                if (slot.owner)
                    clearOwner(slot.owner);
        const auto faults = service.faultedExtensions.exchange(0, std::memory_order_acq_rel);
        for (std::size_t i = 0; i < service.extensions.size(); ++i) {
            auto& slot = service.extensions[i];
            if (!slot.owner)
                continue;
            if (faults & (1u << i)) {
                ROCK_LOG_ERROR(Physics, "Native interaction extension faulted: owner={}", slot.owner);
                clearOwner(slot.owner);
                continue;
            }
            if (slot.callbacks.update)
                invoke(slot, service.frame, [&] { slot.callbacks.update(&service.frame, slot.callbacks.context); });
        }
    }

    void preCollide(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (timing.phase != havok_physics_timing::PhysicsStepPhase::SubstepPreCollide)
            return;
        auto lease = state().gate.tryEnterCallback();
        if (!lease || reinterpret_cast<std::uintptr_t>(world) != state().frame.world)
            return;
        const auto frame = stepFrame(world, timing);
        for (auto& slot : state().extensions)
            if (slot.owner && slot.callbacks.preCollide)
                invoke(slot, frame, [&] { slot.callbacks.preCollide(&frame, slot.callbacks.context); });
    }

    std::uint64_t dynamicObstacleLayers() { return state().obstacleLayers.load(std::memory_order_acquire); }

    void clear(RE::hknpWorld* world, void* bhk)
    {
        auto& service = state();
        if (!service.rockGate)
            return;
        auto rock = service.rockGate->pauseForMutation();
        auto mutation = service.gate.pauseForMutation();
        retireDrivenWeapons(bhk);
        auto frame = service.frame;
        frame.phase = ni::Phase::Retire;
        frame.world = reinterpret_cast<std::uintptr_t>(world);
        frame.bhkWorld = reinterpret_cast<std::uintptr_t>(bhk);
        for (auto& slot : service.extensions)
            if (slot.owner) {
                if (slot.callbacks.retire)
                    invoke(slot, frame, [&] { slot.callbacks.retire(&frame, slot.callbacks.context); });
                releaseWeaponResources(slot.owner, world);
                slot.obstacleLayers = 0;
                slot.staticPassCount = 0;
            }
        service.resetWeapons();
        combineMasks(world);
        combineStaticPass(world);
        service.frame = {};
    }

    Status canUnregisterOwner(Owner owner)
    {
        if (!owner)
            return Status::Ok;
        for (const auto& value : state().registeredOwners)
            if (value.load(std::memory_order_acquire) == owner)
                return current ? Status::Busy : (provider::runtime::onOwnerThread() ? Status::Ok : Status::WrongThread);
        return Status::Ok;
    }

    void clearOwner(Owner owner)
    {
        if (!owner)
            return;
        auto& service = state();
        if (current || !provider::runtime::onOwnerThread()) {
            for (auto& pending : service.pendingRemoval) {
                auto value = pending.load(std::memory_order_acquire);
                if (value == owner ||
                    (!value && pending.compare_exchange_strong(value, owner, std::memory_order_acq_rel)))
                    return;
            }
            service.removeAll.store(true, std::memory_order_release);
            return;
        }
        auto* slot = extension(owner);
        if (!slot)
            return;
        if (!service.rockGate) {
            service.gate.pauseAndWait();
            releaseWeaponResources(owner, nullptr);
            service.registeredOwners[slot - service.extensions.data()].store(0, std::memory_order_release);
            slot->owner = 0;
            slot->callbacks = {};
            slot->obstacleLayers = 0;
            slot->staticPassCount = 0;
            if (service.driver == owner) {
                service.driver = 0;
                service.resetWeapons();
            }
            combineMasks();
            combineStaticPass(nullptr);
            return;
        }
        auto rock = service.rockGate->pauseForMutation();
        auto mutation = service.gate.pauseForMutation();
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto* cell = player ? player->GetParentCell() : nullptr;
        auto* bhk = cell ? cell->GetbhkWorld() : nullptr;
        auto* world = bhk ? havok_runtime::getHknpWorldFromBhk(bhk) : nullptr;
        if (reinterpret_cast<std::uintptr_t>(world) != service.frame.world) {
            world = nullptr;
            bhk = nullptr;
        }
        if (service.driver == owner)
            retireDrivenWeapons(bhk);
        auto frame = service.frame;
        frame.phase = ni::Phase::Retire;
        frame.world = reinterpret_cast<std::uintptr_t>(world);
        frame.bhkWorld = reinterpret_cast<std::uintptr_t>(bhk);
        if (slot->callbacks.retire)
            invoke(*slot, frame, [&] { slot->callbacks.retire(&frame, slot->callbacks.context); });
        releaseWeaponResources(owner, world);
        service.registeredOwners[slot - service.extensions.data()].store(0, std::memory_order_release);
        slot->owner = 0;
        slot->callbacks = {};
        slot->obstacleLayers = 0;
        slot->staticPassCount = 0;
        if (service.driver == owner) {
            service.driver = 0;
            service.resetWeapons();
        }
        combineMasks(world);
        combineStaticPass(world);
    }

    bool updateWeapon(std::uint32_t index, const PhysicsFrameContext& context, const WeaponCollision& collision,
        RE::NiNode* node, BethesdaPhysicsBody& body, const RE::NiPoint3& center, std::uint64_t generation,
        RE::NiTransform& requested, bool support, const RE::NiPoint3* grip)
    {
        auto& service = state();
        auto* slot = extension(service.driver);
        if (index >= service.weapons.size() || !slot || !slot->callbacks.weaponUpdate)
            return true;
        auto rock = service.rockGate->pauseForMutation();
        auto mutation = service.gate.pauseForMutation();
        auto& weapon = service.weapons[index];
        ni::WeaponFrame frame{};
        frame.frame = makeFrame(context, ni::Phase::WeaponUpdate);
        frame.weaponSlot = index;
        frame.body = body.getBodyId().value;
        const auto composition = collision.getWeaponCompositionSnapshot();
        frame.generation = generation;
        frame.form = composition.weaponGenerationKey == generation ? composition.weaponFormId : 0;
        frame.supportOwnsPose = support;
        frame.requestedWeapon = toApi(requested);
        frame.centerWeaponLocal = toApi(center);
        frame.primaryGripValid = grip != nullptr;
        if (grip)
            frame.primaryGripLocal = toApi(*grip);
        frame.recoveryDistanceGame = g_rockConfig.rockWeaponCollisionGripRecoveryDistanceGameUnits;
        weapon.body = frame.body;
        weapon.generation = generation;
        weapon.collision = &collision;
        weapon.weaponNode = node;
        ni::WeaponCommand command{};
        command.requestedWeapon = frame.requestedWeapon;
        command.presentation = frame.requestedWeapon;
        const bool ok = invoke(*slot, frame.frame,
            [&] { slot->callbacks.weaponUpdate(&frame, &command, slot->callbacks.context); });
        weapon.collision = nullptr;
        weapon.weaponNode = nullptr;
        if (!ok || disabled(*slot) || command.size != sizeof(command) || command.version != 1 ||
            (command.result != ni::DriveResult::Unchanged && command.result != ni::DriveResult::Controlled) ||
            (command.result == ni::DriveResult::Unchanged && weapon.excludedTarget != ni::kInvalidBody) ||
            (command.result == ni::DriveResult::Controlled &&
                (!finite(command.requestedWeapon) || !finite(command.presentation))))
            return false;
        weapon.command = command;
        if (command.result == ni::DriveResult::Controlled)
            requested = fromApi(command.requestedWeapon);
        return true;
    }

    bool weaponControlled(std::uint32_t slot)
    {
        return slot < api::kMaxWeaponSlots && state().weapons[slot].command.result == ni::DriveResult::Controlled;
    }

    RE::NiTransform weaponPresentation(std::uint32_t slot)
    {
        return fromApi(state().weapons[slot].command.presentation);
    }

    bool preCollideWeapon(std::uint32_t index, RE::hknpWorld* world,
        const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (index >= api::kMaxWeaponSlots)
            return true;
        auto lease = state().gate.tryEnterCallback();
        if (!lease)
            return !weaponControlled(index);
        if (reinterpret_cast<std::uintptr_t>(world) != state().frame.world)
            return !weaponControlled(index);
        auto* slot = extension(state().driver);
        if (!slot || !slot->callbacks.weaponPreCollide)
            return true;
        const auto frame = stepFrame(world, timing);
        std::uint32_t result = 0;
        return invoke(*slot, frame,
                   [&] { result = slot->callbacks.weaponPreCollide(&frame, index, slot->callbacks.context); }) &&
            result != 0;
    }

    void weaponContact(std::uint32_t index, RE::hknpWorld* world, std::uint32_t weapon, std::uint32_t target,
        std::uint32_t layer, const RE::NiPoint3& point, const RE::NiPoint3& normal, std::uint32_t pointCount)
    {
        if (index >= api::kMaxWeaponSlots)
            return;
        auto lease = state().gate.tryEnterCallback();
        if (!lease)
            return;
        auto* slot = extension(state().driver);
        if (!slot || !slot->callbacks.weaponContact || state().weapons[index].body != weapon ||
            state().frame.world != reinterpret_cast<std::uintptr_t>(world))
            return;
        ni::Contact contact{};
        contact.frame = state().frame;
        contact.frame.phase = ni::Phase::WeaponContact;
        contact.weaponSlot = index;
        contact.weapon = weapon;
        contact.target = target;
        contact.targetLayer = layer;
        contact.pointWorld = toApi(point);
        contact.pointCount = pointCount;
        const float length = normal.x * normal.x + normal.y * normal.y + normal.z * normal.z;
        if (pointCount > 0 && pointCount <= 4 && std::isfinite(length) && length > 0.000001f) {
            const float inverse = 1.0f / std::sqrt(length);
            contact.normalValid = 1;
            contact.normalWorld = { normal.x * inverse, normal.y * inverse, normal.z * inverse };
        }
        invoke(*slot, contact.frame, [&] { slot->callbacks.weaponContact(&contact, slot->callbacks.context); });
    }

    void retireWeapon(std::uint32_t index, RE::hknpWorld* world, void* bhk)
    {
        auto& service = state();
        auto* slot = extension(service.driver);
        if (index >= service.weapons.size() || !slot || !service.rockGate)
            return;
        auto rock = service.rockGate->pauseForMutation();
        auto mutation = service.gate.pauseForMutation();
        auto frame = service.frame;
        frame.phase = ni::Phase::WeaponRetire;
        frame.world = reinterpret_cast<std::uintptr_t>(world);
        frame.bhkWorld = reinterpret_cast<std::uintptr_t>(bhk);
        if (slot->callbacks.weaponRetire)
            invoke(*slot, frame, [&] { slot->callbacks.weaponRetire(&frame, index, slot->callbacks.context); });
        // The parent retires its tool body immediately after this returns.
        releaseWeaponResources(slot->owner, index, world);
        service.weapons[index].command = {};
        service.weapons[index].body = ni::kInvalidBody;
    }

    void handContact(RE::hknpWorld* world, bool left, ni::HandPart part, std::uint32_t handBody, std::uint32_t target,
        std::uint32_t layer, const RE::NiPoint3& point, const RE::NiPoint3& normal, std::uint32_t pointCount)
    {
        // Only layers some extension opted into; the rest of the hand's
        // contacts (world, clutter, its own weapon) never leave ROCK.
        if (layer >= 64 || !(state().obstacleLayers.load(std::memory_order_acquire) & (std::uint64_t{ 1 } << layer)))
            return;
        auto lease = state().gate.tryEnterCallback();
        if (!lease)
            return;
        if (state().frame.world != reinterpret_cast<std::uintptr_t>(world))
            return;
        ni::HandContact contact{};
        contact.frame = state().frame;
        contact.frame.phase = ni::Phase::HandContact;
        contact.hand = left ? 1u : 0u;
        contact.part = part;
        contact.handBody = handBody;
        contact.target = target;
        contact.targetLayer = layer;
        contact.pointWorld = toApi(point);
        contact.pointCount = pointCount;
        const float length = normal.x * normal.x + normal.y * normal.y + normal.z * normal.z;
        if (pointCount > 0 && pointCount <= 4 && std::isfinite(length) && length > 0.000001f) {
            const float inverse = 1.0f / std::sqrt(length);
            contact.normalValid = 1;
            contact.normalWorld = { normal.x * inverse, normal.y * inverse, normal.z * inverse };
        }
        for (auto& slot : state().extensions)
            if (slot.owner && slot.callbacks.handContact)
                invoke(slot, contact.frame, [&] { slot.callbacks.handContact(&contact, slot.callbacks.context); });
    }

    ni::SelectionReply classify(const ni::SelectionQuery& query)
    {
        ni::SelectionReply result{};
        auto& service = state();
        if (!service.rockGate)
            return result;
        auto rock = service.rockGate->pauseForMutation();
        auto mutation = service.gate.pauseForMutation();
        auto frame = service.frame;
        frame.phase = ni::Phase::Selection;
        frame.world = query.world;
        for (auto& slot : service.extensions)
            if (slot.owner && slot.callbacks.classifySelection) {
                ni::SelectionReply candidate{};
                if (!invoke(slot, frame,
                        [&] { slot.callbacks.classifySelection(&query, &candidate, slot.callbacks.context); }))
                    continue;
                if (candidate.size == sizeof(candidate) && candidate.version == 1 &&
                    candidate.kind != ni::SelectionKind::Unhandled &&
                    static_cast<unsigned>(candidate.kind) <= static_cast<unsigned>(ni::kLastSelectionKind) &&
                    candidate.reason[63] == 0) {
                    candidate.owner = slot.owner;
                    return candidate;
                }
            }
        return result;
    }

    bool validateSelection(const ni::SelectionQuery& query, const ni::SelectionReply& selection)
    {
        auto& service = state();
        auto* slot = extension(selection.owner);
        if (!service.rockGate || !slot || !slot->callbacks.validateSelection)
            return false;
        auto rock = service.rockGate->pauseForMutation();
        auto mutation = service.gate.pauseForMutation();
        auto frame = service.frame;
        frame.phase = ni::Phase::Selection;
        frame.world = query.world;
        std::uint32_t valid = 0;
        return invoke(*slot, frame,
                   [&] { valid = slot->callbacks.validateSelection(&query, &selection, slot->callbacks.context); }) &&
            valid == 1;
    }

    Status extract(const ni::SelectionQuery& query, const ni::SelectionReply& selection, ni::ExtractedReference& result)
    {
        auto& service = state();
        auto* slot = extension(selection.owner);
        if (!service.rockGate || !slot || !slot->callbacks.extractSelection)
            return Status::OwnerNotRegistered;
        auto rock = service.rockGate->pauseForMutation();
        auto mutation = service.gate.pauseForMutation();
        auto frame = service.frame;
        frame.phase = ni::Phase::Extraction;
        frame.world = query.world;
        Status status = Status::InternalError;
        if (!invoke(*slot, frame, [&] {
                status = slot->callbacks.extractSelection(&query, &selection, &result, slot->callbacks.context);
            }))
            return Status::InternalError;
        if (status == Status::Ok &&
            (result.size != sizeof(result) || result.version != 1 || !result.nativeHandle || !result.referenceForm ||
                !result.itemForm))
            return Status::TargetInvalid;
        return status;
    }

    ni::ProximityResult proximityTarget(std::uintptr_t reference, float radiusGame)
    {
        auto& service = state();
        ni::ProximityResult result{};
        if (!service.rockGate)
            return result;
        auto rock = service.rockGate->pauseForMutation();
        auto mutation = service.gate.pauseForMutation();
        auto frame = service.frame;
        frame.phase = ni::Phase::Gameplay;
        ni::ProximityQuery query{};
        query.reference = reference;
        query.radiusGame = radiusGame;
        for (auto& slot : service.extensions)
            if (slot.owner && slot.callbacks.proximityTarget) {
                ni::ProximityResult candidate{};
                if (invoke(slot, frame,
                        [&] { slot.callbacks.proximityTarget(&query, &candidate, slot.callbacks.context); }) &&
                    candidate.size == sizeof(candidate) && candidate.version == 1 && candidate.handled)
                    return candidate;
            }
        return result;
    }

    std::uint32_t impactEffect(std::uintptr_t source, std::uintptr_t target, float damage)
    {
        auto& service = state();
        if (!service.rockGate || !target || !std::isfinite(damage) || damage <= 0)
            return 0;
        auto rock = service.rockGate->pauseForMutation();
        auto mutation = service.gate.pauseForMutation();
        auto frame = service.frame;
        frame.phase = ni::Phase::Gameplay;
        ni::ImpactQuery query{};
        query.source = source;
        query.target = target;
        query.authoredDamage = damage;
        for (auto& slot : service.extensions)
            if (slot.owner && slot.callbacks.impactEffect) {
                ni::ImpactResult candidate{};
                if (!invoke(slot, frame,
                        [&] { slot.callbacks.impactEffect(&query, &candidate, slot.callbacks.context); }))
                    return 0;
                if (candidate.size != sizeof(candidate) || candidate.version != 1)
                    return 0;
                if (candidate.handled)
                    return candidate.targetForm;
            }
        return 0;
    }

    const ni::Api& table() noexcept
    {
        static const ni::Api api{ &registerExtension, &unregisterExtension, &setDynamicObstacleLayers, &activateWeapon,
            &setWeaponContactExclusion, &clearWeaponContactExclusion, &hasWeaponContactExclusion, &copyWeaponSources,
            &getSourceGeometry, &setStaticContactPass };
        return api;
    }
}
