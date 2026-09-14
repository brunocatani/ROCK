#pragma once

#include <cmath>
#include <cstdint>

#include "RE/NetImmerse/NiTransform.h"

/*
 * FRIK's weapon presentation for a Weapon node ROCK does not hold (FRIK API
 * v2.3).
 *
 * ROCK runs in FRIK's AfterArmSolve phase. At that point the Weapon node
 * carries the re-glue local from FRIK's hand-target update; FRIK writes its
 * stored per-weapon offset local later, in its weapon-position pass. Every
 * ROCK read of the Weapon subtree in between would miss that offset. ROCK
 * latches the local FRIK wrote after its pass, applies it to the node for
 * ROCK's own frame, and restores the re-glue local before FRIK continues, so
 * FRIK's pass sees exactly what it would have seen without ROCK. When ROCK's
 * write block is engaged at the end of its frame the node keeps its pose,
 * since FRIK leaves the node alone.
 */
namespace rock::frik_weapon_presentation_policy
{
    struct NodeIdentity
    {
        // Identities only; never dereferenced.
        std::uintptr_t node = 0;
        std::uintptr_t parent = 0;
        // The equipped model under the Weapon node, rebuilt on any weapon or mod change.
        std::uintptr_t modelRoot = 0;
        bool inPowerArmor = false;

        [[nodiscard]] constexpr bool valid() const noexcept
        {
            return node != 0 && parent != 0 && modelRoot != 0;
        }

        // The same equipped weapon, whatever hand it currently hangs under.
        [[nodiscard]] constexpr bool sameWeapon(const NodeIdentity& other) const noexcept
        {
            return node == other.node && modelRoot == other.modelRoot && inPowerArmor == other.inPowerArmor;
        }

        [[nodiscard]] constexpr bool operator==(const NodeIdentity&) const noexcept = default;
    };

    struct OffsetLatch
    {
        NodeIdentity identity{};
        RE::NiTransform local{};
        bool valid = false;
    };

    [[nodiscard]] inline bool isFiniteTransform(const RE::NiTransform& transform) noexcept
    {
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(transform.rotate.entry[row][column])) {
                    return false;
                }
            }
        }
        return std::isfinite(transform.translate.x) && std::isfinite(transform.translate.y) &&
               std::isfinite(transform.translate.z) && std::isfinite(transform.scale);
    }

    struct CaptureInput
    {
        NodeIdentity identity{};
        // The Weapon node's local after FRIK's weapon pass.
        RE::NiTransform local{};
        bool nodeVisible = false;
        // ROCK held FRIK's weapon-node write block through FRIK's weapon pass.
        bool writeBlockHeld = false;
    };

    /*
     * After FRIK's weapon pass. FRIK wrote the node only if it was visible and
     * not blocked, so only then is its local the one FRIK renders. Otherwise
     * the latch is kept for the same equipped weapon (a grip or a left carry
     * leaves the offset unchanged) and dropped for any other.
     */
    [[nodiscard]] inline OffsetLatch captureOffsetLatch(const OffsetLatch& previous, const CaptureInput& input) noexcept
    {
        if (!input.identity.valid()) {
            return {};
        }
        if (!input.writeBlockHeld && input.nodeVisible) {
            // FRIK wrote the node: its local replaces the latch, and an unusable one leaves none.
            return isFiniteTransform(input.local) ? OffsetLatch{ .identity = input.identity, .local = input.local, .valid = true } : OffsetLatch{};
        }
        return previous.valid && previous.identity.sameWeapon(input.identity) ? previous : OffsetLatch{};
    }

    struct PresentInput
    {
        NodeIdentity identity{};
        bool nodeVisible = false;
        // ROCK held the write block when FRIK re-glued the node this frame, so the live node is ROCK's pose.
        bool rockOwnsLivePose = false;
    };

    /*
     * Start of ROCK's frame. Only a node FRIK re-glued this frame is presented,
     * and only with a latch taken under the same parent for the same model:
     * the local is relative to that hand, and a swapped model is a new weapon.
     */
    [[nodiscard]] constexpr bool shouldPresent(const OffsetLatch& latch, const PresentInput& input) noexcept
    {
        return !input.rockOwnsLivePose && input.nodeVisible && latch.valid && input.identity.valid() &&
               latch.identity == input.identity;
    }

    /*
     * End of ROCK's frame, once the write block is settled for FRIK's pass.
     * With the block engaged FRIK leaves the node alone, so its current pose,
     * presented or written over, is the one to keep. With the block released
     * FRIK reads and rewrites the node next, so it gets its re-glue local back
     * whatever the node now holds.
     */
    [[nodiscard]] constexpr bool shouldRestore(const bool presented, const bool writeBlockEngaged) noexcept
    {
        return presented && !writeBlockEngaged;
    }
}
