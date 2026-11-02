#include "physics-interaction/weapon/secondary/NativeSecondaryMelee.h"
#include <REL/Relocation.h>
#include <Windows.h>
#include <array>
#include <atomic>
#include <cstring>
#include <optional>
#include <stdexcept>
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/NetImmerse/NiNode.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/NativeMemory.h"
#include "rock_support/Fo4VrRuntime.h"

namespace rock::native_secondary_melee
{
    namespace
    {
        // FO4VR 1.2.72, verified in ghidra_fo4vr and checked live at install.
        constexpr std::uintptr_t kAttach =
            0xF0AC60; // (actor, node): add the subtree's bodies, keyframed, with the melee callback
        constexpr std::uintptr_t kDetach = 0xF0AD90; // (actor, node, bhkWorld or null): unregister and remove them
        constexpr std::uintptr_t kWorldOfCell = 0x39B070; // (cell) -> its bhkWorld
        constexpr std::uintptr_t kBodyId = 0x1E07EE0; // (collision object, out) -> body id, 7FFFFFFF when none
        constexpr std::ptrdiff_t kParentCell = 0xB8;
        constexpr std::ptrdiff_t kCollisionObject = 0x100;
        constexpr std::ptrdiff_t kAsNpCollisionSlot = 0xD8; // the attach visitor's own collision-object test
        // PlayerCharacter byte; bit 2 is the weapon-collision switch the swing
        // gate 140EFE880 writes (EFEC56/EFEC6B) and 140F0AE70 applies every
        // frame to the index-0 melee nodes (F075BD..F075FA) as filter bit 14.
        constexpr std::ptrdiff_t kMeleeCollisionFlags = 0x12A4;
        constexpr std::uint8_t kWeaponCollisionLive = 1u << 2;
        constexpr std::uint32_t kInvalidBody = 0x7FFFFFFF;

        // 140EFF000's indexed calls: weapon (twice), object impact, actor hit.
        // Each receives 1459444C8 loaded just before it; only the call is relayed.
        struct Relay
        {
            std::uintptr_t site;
            std::uintptr_t target;
        };

        constexpr std::array<Relay, 4> kRelays{ {
            { 0xEFF043, 0xE50DA0 },
            { 0xEFF063, 0xE50DA0 },
            { 0xEFF8E2, 0xE56FA0 },
            { 0xEFF8F3, 0xE56B10 },
        } };
        // PlayerCharacter vtable 142D80F80: slot 172 moves the player's havok
        // to another world (detaching its melee nodes in the old one first),
        // slot 282 detaches all of them.
        constexpr std::uintptr_t kWorldChangeEntry = 0x2D814E0, kWorldChangeFunction = 0xF0A800;
        constexpr std::uintptr_t kDetachAllEntry = 0x2D81850, kDetachAllFunction = 0xF29720;

        using Attach = void (*)(RE::PlayerCharacter*, RE::NiAVObject*);
        using Detach = void (*)(RE::PlayerCharacter*, RE::NiAVObject*, void*);
        using WorldOfCell = void* (*)(const void*);
        using BodyId = std::uint32_t* (*)(const void*, std::uint32_t*);
        using AsNpCollision = const void* (*)(const void*);
        using ReadItem = void* (*)(void*, void*, std::uint32_t);
        using ObjectImpact = void (*)(void*, std::uint32_t, void*, void*, void*, void*, void*);
        using ActorHit = void (*)(void*, std::uint32_t, void*, void*);
        using WorldChange = void (*)(RE::PlayerCharacter*);
        using DetachAll = void (*)(RE::PlayerCharacter*, void*, std::uint32_t, std::uint8_t, std::uint8_t);

        bool s_installed = false; // Load owner; immutable after install.
        ReadItem s_readItem = nullptr;
        ObjectImpact s_objectImpact = nullptr;
        ActorHit s_actorHit = nullptr;
        WorldChange s_worldChange = nullptr;
        DetachAll s_detachAll = nullptr;

        // Set only by ImpactRoute on the thread running the native callback.
        thread_local bool t_routed = false;

        /*
         * Frame owner (s_owner): the attached model, retained so a detach never
         * reaches a freed node, and the world it was attached in. The body ids
         * are published to the contact-callback threads; they are cleared
         * before every detach, so a reused id is never routed afterwards.
         */
        std::atomic<DWORD> s_owner{ 0 }; // Claimed by the first present().
        RE::NiPointer<RE::NiAVObject> s_model;
        void* s_world = nullptr;
        std::array<std::atomic<std::uint32_t>, 16> s_bodies{};
        std::atomic<std::uint32_t> s_bodyCount{ 0 };
        // Weapon-collision switch last applied to the bodies; unknown after attach.
        std::optional<bool> s_appliedLive;

        void* readItem(void* actor, void* out, std::uint32_t index)
        {
            return s_readItem(actor, out, t_routed ? 1u : index);
        }

        void objectImpact(void* actor, std::uint32_t index, void* a, void* b, void* c, void* d, void* e)
        {
            s_objectImpact(actor, t_routed ? 1u : index, a, b, c, d, e);
        }

        void actorHit(void* actor, std::uint32_t index, void* target, void* d)
        {
            s_actorHit(actor, t_routed ? 1u : index, target, d);
        }

        bool onOwner() noexcept
        {
            const auto owner = s_owner.load(std::memory_order_acquire);
            return owner && owner == GetCurrentThreadId();
        }

        void* worldOf(const RE::PlayerCharacter* player) noexcept
        {
            const void* cell = nullptr;
            if (!player || !native_memory::tryReadField(player, kParentCell, cell) || !cell)
                return nullptr;
            return reinterpret_cast<WorldOfCell>(REL::Offset(kWorldOfCell).address())(cell);
        }

        void clearBodies() noexcept
        {
            s_bodyCount.store(0, std::memory_order_release);
            s_appliedLive.reset();
            for (auto& body : s_bodies)
                body.store(kInvalidBody, std::memory_order_relaxed);
        }

        // Why a subtree registered no body: nodes walked, collision objects,
        // native-physics ones, ones with a body id.
        struct BodyCensus
        {
            std::uint32_t nodes = 0, collisionObjects = 0, nativePhysics = 0, withBody = 0;
        };

        // The ids the attach registered: every native collision object in the
        // model subtree, tested the way the attach visitor tests it.
        std::uint32_t collectBodies(RE::NiAVObject* root, BodyCensus& census) noexcept
        {
            std::array<RE::NiAVObject*, 256> pending{};
            std::size_t queued = 0;
            std::uint32_t count = 0;
            pending[queued++] = root;
            for (std::size_t i = 0; i < queued; ++i) {
                auto* object = pending[i];
                ++census.nodes;
                const void* collision = nullptr;
                const void* vtable = nullptr;
                AsNpCollision asNp = nullptr;
                if (native_memory::tryReadField(object, kCollisionObject, collision) && collision) {
                    ++census.collisionObjects;
                    if (native_memory::tryReadField(collision, 0, vtable) && vtable &&
                        native_memory::tryReadField(vtable, kAsNpCollisionSlot, asNp) && asNp && asNp(collision)) {
                        ++census.nativePhysics;
                        std::uint32_t id = kInvalidBody;
                        reinterpret_cast<BodyId>(REL::Offset(kBodyId).address())(collision, &id);
                        if (id != kInvalidBody)
                            ++census.withBody;
                        if (id != kInvalidBody && count < s_bodies.size())
                            s_bodies[count++].store(id, std::memory_order_relaxed);
                    }
                }
                if (auto* node = object->IsNode()) {
                    for (const auto& child : node->GetRuntimeData().children) {
                        if (child && queued < pending.size())
                            pending[queued++] = child.get();
                    }
                }
            }
            s_bodyCount.store(count, std::memory_order_release);
            return count;
        }

        /*
         * The native keeps melee bodies no-collide (filter bit 14) except while
         * a swing is live: they raise melee events without colliding with
         * character controllers, shell casings or the player's own projectiles.
         * It applies the player's weapon-collision switch only to the index-0
         * nodes, so the index-1 bodies collided with all of those (walking into
         * an NPC was a hit, pistol bullets stopped in the gun's own hull). The
         * same switch is applied to index 1 when it changes or after an attach;
         * nothing else writes bit 14 on these bodies.
         */
        void applyWeaponCollisionSwitch(RE::PlayerCharacter* player, void* world) noexcept
        {
            std::uint8_t flags = 0;
            if (!native_memory::tryReadField(player, kMeleeCollisionFlags, flags))
                return;
            const bool live = (flags & kWeaponCollisionLive) != 0;
            if (s_appliedLive == live)
                return;
            const auto count = s_bodyCount.load(std::memory_order_acquire);
            auto* hknp = world ? havok_runtime::getHknpWorldFromBhk(static_cast<RE::bhkWorld*>(world)) : nullptr;
            if (!count || !hknp)
                return;
            constexpr auto noCollide = collision_suppression_registry::kSuppressionNoCollideBit;
            for (std::uint32_t i = 0; i < count && i < s_bodies.size(); ++i) {
                const auto id = s_bodies[i].load(std::memory_order_relaxed);
                std::uint32_t filter = 0;
                if (id == kInvalidBody || !body_collision::tryReadFilterInfo(hknp, RE::hknpBodyId{ id }, filter))
                    continue;
                const auto applied = live ? (filter & ~noCollide) : (filter | noCollide);
                if (applied != filter)
                    body_collision::setFilterInfo(hknp, RE::hknpBodyId{ id }, applied);
            }
            s_appliedLive = live;
        }

        void detachCurrent(RE::PlayerCharacter* player, const char* reason) noexcept
        {
            if (!s_model)
                return;
            clearBodies();
            if (player && s_world)
                reinterpret_cast<Detach>(REL::Offset(kDetach).address())(player, s_model.get(), s_world);
            ROCK_LOG_INFO(Weapon, "AKIMBO second melee native collision detached model={:#x} reason={}",
                reinterpret_cast<std::uintptr_t>(s_model.get()), reason);
            s_model.reset();
            s_world = nullptr;
        }

        void worldChange(RE::PlayerCharacter* player)
        {
            // As the native does for its own melee nodes: detach in the old
            // world before the move; present() attaches in the new one.
            if (s_model && player == f4vr::getPlayer() && onOwner() && worldOf(player) != s_world)
                detachCurrent(player, "world-change");
            s_worldChange(player);
        }

        void detachAll(RE::PlayerCharacter* player, void* a, std::uint32_t b, std::uint8_t c, std::uint8_t d)
        {
            if (s_model && player == f4vr::getPlayer() && onOwner())
                detachCurrent(player, "player-detach");
            s_detachAll(player, a, b, c, d);
        }

        bool callMatches(std::uintptr_t site, std::uintptr_t target) noexcept
        {
            std::array<std::uint8_t, 5> live{};
            const auto address = REL::Offset(site).address();
            std::int32_t displacement = 0;
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address), live.data(),
                    live.size()) ||
                live[0] != 0xE8)
                return false;
            std::memcpy(&displacement, live.data() + 1, sizeof(displacement));
            return address + 5 + displacement == REL::Offset(target).address();
        }

        bool entryMatches(std::uintptr_t entry, std::uintptr_t function) noexcept
        {
            std::uintptr_t live = 0;
            return native_memory::tryReadValue(reinterpret_cast<const std::uintptr_t*>(REL::Offset(entry).address()),
                       live) &&
                live == REL::Offset(function).address();
        }
    }

    bool install() noexcept
    {
        if (s_installed)
            return true;
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72)
            return false;
        for (const auto& relay : kRelays) {
            if (!callMatches(relay.site, relay.target)) {
                ROCK_LOG_ERROR(Init, "Second melee routing refused: call at {:#x} is not to {:#x}", relay.site,
                    relay.target);
                return false;
            }
        }
        if (!entryMatches(kWorldChangeEntry, kWorldChangeFunction) ||
            !entryMatches(kDetachAllEntry, kDetachAllFunction)) {
            ROCK_LOG_ERROR(Init, "Second melee routing refused: PlayerCharacter world/detach vtable entries differ");
            return false;
        }
        std::array<std::array<std::uint8_t, 5>, kRelays.size()> originals{};
        for (std::size_t i = 0; i < kRelays.size(); ++i) {
            (void)native_memory::guardedCopyFromMemory(
                reinterpret_cast<const void*>(REL::Offset(kRelays[i].site).address()), originals[i].data(),
                originals[i].size());
        }
        const std::array<std::uintptr_t, kRelays.size()> hooks{ reinterpret_cast<std::uintptr_t>(&readItem),
            reinterpret_cast<std::uintptr_t>(&readItem), reinterpret_cast<std::uintptr_t>(&objectImpact),
            reinterpret_cast<std::uintptr_t>(&actorHit) };
        try {
            auto& trampoline = F4SE::GetTrampoline();
            if (trampoline.free_size() < kRelays.size() * 14)
                throw std::runtime_error("trampoline space");
            s_readItem = reinterpret_cast<ReadItem>(REL::Offset(0xE50DA0).address());
            s_objectImpact = reinterpret_cast<ObjectImpact>(REL::Offset(0xE56FA0).address());
            s_actorHit = reinterpret_cast<ActorHit>(REL::Offset(0xE56B10).address());
            for (std::size_t i = 0; i < kRelays.size(); ++i)
                (void)trampoline.write_call<5>(REL::Offset(kRelays[i].site).address(), hooks[i]);
            if (!FlushInstructionCache(GetCurrentProcess(), nullptr, 0))
                throw std::runtime_error("instruction cache");
        } catch (...) {
            for (std::size_t i = 0; i < kRelays.size(); ++i)
                REL::safe_write(REL::Offset(kRelays[i].site).address(), originals[i].data(), originals[i].size());
            FlushInstructionCache(GetCurrentProcess(), nullptr, 0);
            ROCK_LOG_ERROR(Init, "Second melee routing install failed; original calls restored");
            return false;
        }
        s_worldChange = reinterpret_cast<WorldChange>(REL::Offset(kWorldChangeFunction).address());
        s_detachAll = reinterpret_cast<DetachAll>(REL::Offset(kDetachAllFunction).address());
        const auto worldHook = reinterpret_cast<std::uintptr_t>(&worldChange);
        const auto detachHook = reinterpret_cast<std::uintptr_t>(&detachAll);
        REL::safe_write(REL::Offset(kWorldChangeEntry).address(), &worldHook, sizeof(worldHook));
        REL::safe_write(REL::Offset(kDetachAllEntry).address(), &detachHook, sizeof(detachHook));
        clearBodies();
        s_installed = true;
        ROCK_LOG_INFO(Init,
            "Second melee native routing installed: four indexed melee calls, player world-change and detach");
        return true;
    }

    void present(RE::NiAVObject* model) noexcept
    {
        if (!s_installed)
            return;
        DWORD absent = 0;
        (void)s_owner.compare_exchange_strong(absent, GetCurrentThreadId(), std::memory_order_acq_rel);
        if (!onOwner())
            return;
        auto* player = f4vr::getPlayer();
        void* world = worldOf(player);
        if (s_model && s_model.get() == model && s_world == world) {
            applyWeaponCollisionSwitch(player, world);
            return;
        }
        if (s_model && s_world != world) {
            // The world-change hook detaches before every move it makes. Never
            // reach a previous world that may already be gone.
            ROCK_LOG_WARN(Weapon, "AKIMBO second melee native collision lost its world without a detach model={:#x}",
                reinterpret_cast<std::uintptr_t>(s_model.get()));
            clearBodies();
            s_model.reset();
            s_world = nullptr;
        }
        detachCurrent(player, model ? "model-changed" : "not-presented");
        if (!model || !player || !world)
            return;
        reinterpret_cast<Attach>(REL::Offset(kAttach).address())(player, model);
        s_model.reset(model);
        s_world = world;
        BodyCensus census{};
        const auto bodies = collectBodies(model, census);
        if (bodies) {
            ROCK_LOG_INFO(Weapon, "AKIMBO second melee native collision attached model={:#x} bodies={}",
                reinterpret_cast<std::uintptr_t>(model), bodies);
            applyWeaponCollisionSwitch(player, world);
        } else {
            ROCK_LOG_WARN(Weapon,
                "AKIMBO second melee has no native collision bodies model={:#x} name='{}' nodes={} collisionObjects={} "
                "nativePhysics={} withBody={}; native hits unavailable",
                reinterpret_cast<std::uintptr_t>(model), model->name.c_str() ? model->name.c_str() : "", census.nodes,
                census.collisionObjects, census.nativePhysics, census.withBody);
        }
    }

    void beforeModelRetired(const RE::NiAVObject* root) noexcept
    {
        if (!s_installed || !root || s_model.get() != root)
            return;
        if (!onOwner()) {
            // Never call into the physics world off the frame owner; stop
            // routing now and let the frame owner forget the model.
            clearBodies();
            ROCK_LOG_WARN(Weapon, "AKIMBO second melee model retired off the frame thread; routing stopped model={:#x}",
                reinterpret_cast<std::uintptr_t>(root));
            return;
        }
        detachCurrent(f4vr::getPlayer(), "model-retired");
    }

    ImpactRoute::ImpactRoute(const void* contactEvent, const void* collisionEvent) noexcept : _previous(t_routed)
    {
        const auto count = s_bodyCount.load(std::memory_order_acquire);
        if (!count || !contactEvent || !collisionEvent)
            return;
        // Same event layout the existing partner decode reads: the callback's
        // own body index at +20, the pair's body ids at +08.
        std::uint32_t ownIndex = 0;
        std::uint32_t ownBody = kInvalidBody;
        if (!native_memory::guardedCopyFromMemory(static_cast<const char*>(contactEvent) + 0x20, &ownIndex,
                sizeof(ownIndex)) ||
            ownIndex > 1 ||
            !native_memory::guardedCopyFromMemory(static_cast<const char*>(collisionEvent) + 0x08 +
                    ownIndex * sizeof(ownBody),
                &ownBody, sizeof(ownBody)))
            return;
        for (std::uint32_t i = 0; i < count && i < s_bodies.size(); ++i) {
            if (s_bodies[i].load(std::memory_order_relaxed) == ownBody) {
                _routed = true;
                break;
            }
        }
        t_routed = _routed;
    }

    ImpactRoute::~ImpactRoute() { t_routed = _previous; }

    bool routedContact() noexcept { return t_routed; }
}
