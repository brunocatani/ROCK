#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <span>

#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiTransform.h"
#include "physics-interaction/native/havok/HavokPhysicsTiming.h"

namespace rock
{
    /*
     * Loose weapons left lodged in a body (NativeInteraction WeaponCommand
     * lodged): a blade the player let go of while its driver held it in a limb.
     *
     * The weapon stays an ordinary dynamic loose reference. Its native bodies
     * take the target body's collision group, so the engine filter
     * (0x141E115B0) skips every pair with that actor: inside one group a
     * non-biped row never collides, which covers his limbs and his character
     * controller, while hands, selection casts and the world (other groups)
     * still meet it. Each physics substep a velocity drive carries it on the
     * target body's pose. Nothing is constrained to an engine body, so the
     * engine may remove that body at any time; the drive only reads it by id
     * and identity and stops when either changes.
     *
     * A lodge ends when a hand starts to take, pull or holds the weapon (its
     * update runs before the loose weapon pool admits a held weapon, which
     * records the filter it finds), when its reference or bodies go away, when
     * it is pushed kBreakGame off its seat for kBreakSeconds, or when the
     * target body goes and its bone has no new body to follow within
     * kFollowGrace (the weapon is held where it was meanwhile). Ending restores
     * each body's original filter: the weapon is an ordinary loose weapon
     * again. Saves keep only the native state, so a reloaded weapon is loose.
     *
     * Threads: begin, update and clear run on the frame thread with the
     * physics-step callbacks paused (PhysicsCallbackQuiescenceGate); step runs
     * on the physics thread and writes only the atomic failure and off-seat
     * counters.
     */
    class LodgedWeapons
    {
    public:
        static constexpr std::size_t kMaxLodges = 8;
        static constexpr std::size_t kMaxBodies = 8;
        static constexpr float kBreakGame = 8.0f;
        static constexpr float kBreakSeconds = 0.1f;
        // How long the weapon waits, held still, for a lost limb's bone to
        // get its new body.
        static constexpr std::chrono::milliseconds kFollowGrace{ 500 };

        // The target as the driver named it, checked live at the drop.
        struct Capture
        {
            bool valid{ false };
            std::uint32_t slot{ 0 };
            std::uint32_t body{ 0x7FFF'FFFFu };
            std::uintptr_t collisionObject{ 0 };
            std::uint32_t group{ 0 };
            RE::NiPointer<RE::NiAVObject> bone{};
        };

        // One placed native body of the dropped weapon. drive: one body per
        // motion (bodies sharing a motion move together). placed: where the
        // drop placed it, world.
        struct PlacedBody
        {
            std::uint32_t id{ 0x7FFF'FFFFu };
            bool drive{ false };
            RE::NiTransform placed{};
        };

        // Frame thread: the slot's lodge mark, its target body live in world.
        [[nodiscard]] static bool capture(RE::hknpWorld* world, std::uint32_t slot, Capture& out);

        // Frame thread, step callbacks paused: the dropped reference's native
        // bodies were just placed; the lodge takes them over. Repeating it for
        // the same reference (a placement restart) replaces its lodge.
        bool begin(RE::hknpWorld* world, const RE::ObjectRefHandle& reference, std::uint32_t form,
            const Capture& target, std::span<const PlacedBody> bodies);

        // Frame thread, step callbacks paused. handReferences: what each hand
        // has selected, is pulling or holds (null when nothing).
        void update(RE::hknpWorld* world, std::span<const RE::TESObjectREFR* const> handReferences);

        // Physics thread, each substep before collision.
        void step(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing) noexcept;

        // Frame thread, step callbacks paused: ends every lodge. A live world
        // gets its filters back; a lost one only forgets them.
        void clear(RE::hknpWorld* liveWorld);

        [[nodiscard]] bool lodged(std::uint32_t form) const noexcept;

    private:
        enum class State : std::uint8_t
        {
            Free,
            Driving
        };
        enum class Failure : std::uint8_t
        {
            None,
            TargetLost,
            BodyLost,
            PushedOff
        };

        struct Body
        {
            std::uint32_t id{ 0x7FFF'FFFFu };
            std::uintptr_t collisionObject{ 0 };
            std::uint32_t filter{ 0 };
            bool drive{ false };
            RE::NiTransform inTarget{};
            // Where the last good substep put it, world; held there while the
            // target is lost (physics thread only).
            RE::NiTransform lastSeat{};
        };

        struct Entry
        {
            std::atomic<State> state{ State::Free };
            std::atomic<Failure> failure{ Failure::None };
            // Seconds the weapon has been kBreakGame off its seat, and by how far.
            std::atomic<float> offSeat{ 0.0f };
            std::atomic<float> offSeatGame{ 0.0f };
            // Why the step lost the target: 1 another body, 2 no pose. Since
            // when the frame thread has been looking for the bone's new body
            // (frame thread only).
            std::atomic<std::uint8_t> lost{ 0 };
            bool lostPending{ false };
            std::chrono::steady_clock::time_point lostSince{};
            RE::hknpWorld* world{ nullptr };
            RE::ObjectRefHandle reference{};
            std::uint32_t form{ 0 };
            Capture target{};
            std::array<Body, kMaxBodies> bodies{};
            std::uint32_t count{ 0 };
        };

        bool anchor(Entry& entry, RE::hknpWorld* world, std::span<const PlacedBody> placed, bool usePlaced);
        // The stage that failed, or nullptr when the lodge rides the bone's body.
        const char* follow(Entry& entry, RE::hknpWorld* world);
        // dropsLoose: the weapon falls on its own afterwards (not taken by a hand).
        void end(Entry& entry, RE::hknpWorld* liveWorld, const char* reason, bool dropsLoose);

        std::array<Entry, kMaxLodges> _entries{};
    };
}
