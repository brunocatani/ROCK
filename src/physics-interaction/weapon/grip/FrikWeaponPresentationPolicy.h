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
 *
 * A latch can only be captured on a frame FRIK wrote. ROCK's authored primary
 * alignment writes the node and holds the write block from the equip frame
 * on, so for such a weapon FRIK never writes and nothing is ever captured;
 * the node would be read at the glue for the whole hold (the position-only
 * alignment inherits that rotation: every weapon rendered at the vanilla
 * attach, pipe rifles pointing up). ROCK keeps its own copy of FRIK's offset
 * table, so the stored offset stands in for the latch whenever no captured
 * one matches; presenting it every frame under the block gives ROCK's
 * readers the pose FRIK 0.78 wrote before each ROCK frame.
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

    /*
     * FRIK's stored per-weapon offset from ROCK's copy of the offset table,
     * resolved once per node identity and table revision (a miss is kept the
     * same way, so the name lookup does not repeat every frame).
     */
    struct SynthesizedOffset
    {
        NodeIdentity identity{};
        std::uint64_t offsetTableRevision = 0;
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
        // ROCK owns the weapon pose this frame (two-hand authority, part
        // carry, left carry, a return blend): the node holds ROCK's own solve,
        // which FRIK restores behind its glue, and FRIK's offset must not
        // replace it. The write block alone does not own the pose: the
        // authored alignment and the one-hand recoil re-apply themselves on
        // top of FRIK's pose every frame, as they did under FRIK 0.78.
        bool rockOwnsPose = false;
        // The node hangs under the hand FRIK's stored offset is authored for
        // (the game's primary hand), so that offset is a valid local here.
        bool underPrimaryHand = false;
    };

    enum class PresentSource : std::uint8_t
    {
        None,
        // The local FRIK last wrote for this node, parent and model.
        CapturedLatch,
        // FRIK's stored offset from ROCK's copy of the offset table.
        SynthesizedOffset,
    };

    /*
     * Start of ROCK's frame: which local stands in for FRIK's weapon pass on
     * the node ROCK is about to read. The captured latch is FRIK's own last
     * write and wins whenever it matches the node, parent and model (a
     * swapped model is a new weapon, a reparented node a different local).
     * Without one (a new weapon before FRIK's first write, or a write block
     * held since before it, so FRIK never wrote) the stored offset of the
     * current table revision is presented under the primary hand. Hidden
     * equip frames still feed authored alignment and native aim capture, so
     * they need the same baseline. Visibility gates capturing a FRIK write,
     * not presenting a known offset. ROCK-owned poses are left intact.
     */
    [[nodiscard]] constexpr PresentSource selectPresentation(
        const OffsetLatch& latch,
        const SynthesizedOffset& synthesized,
        const std::uint64_t offsetTableRevision,
        const PresentInput& input) noexcept
    {
        if (input.rockOwnsPose || !input.identity.valid()) {
            return PresentSource::None;
        }
        if (latch.valid && latch.identity == input.identity) {
            return PresentSource::CapturedLatch;
        }
        if (input.underPrimaryHand && synthesized.valid && synthesized.identity == input.identity &&
            synthesized.offsetTableRevision == offsetTableRevision) {
            return PresentSource::SynthesizedOffset;
        }
        return PresentSource::None;
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
