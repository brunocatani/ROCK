#pragma once

#include <cstdint>
#include <string_view>

namespace RE
{
    class hknpWorld;
    class NiAVObject;
    class TESObjectREFR;
}

/*
 * Loose ammunition with count-based 3D.
 *
 * FO4VR builds a loose ammo reference from one model holding an "AmmoSingle"
 * and an "AmmoMultiple" branch. When the ammo's AMMO_DATA flag 0x04 is set,
 * TESAmmo's 3D setup (0x1402F2F00) reads the reference's extra count and
 * detaches "AmmoMultiple" when the count is exactly 1 and "AmmoSingle"
 * otherwise. The flag offset (TESAmmo+0x16C, AMMO_DATA+0x0C) is the byte that
 * function tests; the AMMO DNAM records of the vanilla rounds carry the same bit.
 *
 * The model's physics system holds one body per branch plus a body on the
 * root node (a ~1 game-unit cube of 0.01 mass) with no constraint between
 * them. The root body owns no geometry, yet it drives the reference's root
 * node and therefore the reference position. A grab must drive the visible
 * branch body and carry the root body with it as one rigid object; holding
 * the root body leaves the visible round behind, and leaving the root body
 * behind makes the reference reload at the wrong place.
 *
 * The visible branch sits at the identity under the root in every model this
 * targets, so the rigid rest relation is "root node frame == branch node frame".
 *
 * Both bodies come from one physics system, so enumerating the system from the
 * root's collision object returns the branch body too. Which node owns a body
 * is therefore read from each node's own collision object (resolveBodies),
 * never from body-scan order. The root body is never a selection target; the
 * selection finds the visible branch body instead. Outside a grab the root body
 * is kept on the branch by RootBodySync (CountBasedAmmoRootSync.h).
 */
namespace rock::count_based_ammo
{
    enum class Variant : std::uint8_t
    {
        None,
        Single,
        Multiple,
    };

    inline constexpr std::uint8_t kCountBased3DFlag = 0x04;

    // Saved-grab-offset key component: one round and a box are different
    // objects to hold, so each keeps its own saved pose.
    [[nodiscard]] constexpr std::string_view variantKey(Variant variant) noexcept
    {
        switch (variant) {
        case Variant::Single:
            return "single";
        case Variant::Multiple:
            return "multiple";
        case Variant::None:
        default:
            return {};
        }
    }

    struct Assembly
    {
        Variant variant = Variant::None;
        // Non-owning; valid for the current frame only. Never retain.
        RE::NiAVObject* root = nullptr;
        RE::NiAVObject* variantNode = nullptr;

        [[nodiscard]] bool valid() const noexcept
        {
            return variant != Variant::None && root && variantNode && root != variantNode;
        }
    };

    struct AssemblyBodies
    {
        std::uint32_t root = kNoBody;
        std::uint32_t branch = kNoBody;

        static constexpr std::uint32_t kNoBody = 0x7FFF'FFFF;

        [[nodiscard]] bool valid() const noexcept { return root != kNoBody && branch != kNoBody && root != branch; }
    };

    // Frame thread only. Returns an invalid assembly for anything that is not
    // count-based loose ammo with exactly one visible branch attached.
    [[nodiscard]] Assembly resolve(RE::TESObjectREFR* refr);

    // True only for references whose base is count-based ammo; a cheap gate
    // before resolve().
    [[nodiscard]] bool isCountBasedAmmo(RE::TESObjectREFR* refr);

    // The root body and the visible branch body of the assembly in this world,
    // each read from its own node's collision object.
    [[nodiscard]] AssemblyBodies resolveBodies(RE::hknpWorld* world, const Assembly& assembly);

    // True when hitNode is the geometry-less root of a resolved assembly.
    [[nodiscard]] bool isRootBodyNode(RE::TESObjectREFR* refr, const RE::NiAVObject* hitNode);
}
