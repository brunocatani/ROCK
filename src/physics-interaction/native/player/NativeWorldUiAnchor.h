#pragma once

#include "RE/NetImmerse/NiNode.h"

#include <array>
#include <cstdint>
#include <optional>

/*
 * Shared placement for the native world-space HUD models (the quick loot
 * panel and the wand activate prompts) that ROCK moves off the wands. A model
 * keeps its root and the local transform the game writes into it; ROCK moves
 * the root under a carrier node it owns and poses the carrier so the root
 * lands where it should. Game thread only.
 */
namespace rock::native_world_ui_anchor
{
    // A model's visible face in its root's local space, read from its mesh:
    // unit vectors along the text's right and up and out of the face toward
    // the viewer.
    struct FaceFrame
    {
        std::array<float, 3> right;
        std::array<float, 3> up;
        std::array<float, 3> toViewer;
    };

    /*
     * World pose for a model root at the reference behind an ObjectRefHandle
     * value. The face's centre (`faceCenter`, root-local) stands on the
     * reference's near side at its bound centre height; the face is upright,
     * its text level, turned about the vertical toward the eye; its size is a
     * fixed multiple of `handScale`, the root's world scale on the hand,
     * whatever the distance. Empty when the handle no longer resolves.
     */
    [[nodiscard]] std::optional<RE::NiTransform> uprightPoseAt(std::uint32_t handleValue, const RE::NiPoint3& eye,
        const RE::NiPoint3& eyeRight, const FaceFrame& face, float handScale, const RE::NiPoint3& faceCenter);

    // The root's world bound centre in its own local space; zero without a bound.
    [[nodiscard]] RE::NiPoint3 boundCenterInRootSpace(const RE::NiAVObject& root);

    // Distance from a node to the reference's bound centre (its origin when it
    // has no bound); empty when the handle no longer resolves.
    [[nodiscard]] std::optional<float> distanceToReference(std::uint32_t handleValue, const RE::NiNode* from);

    // Re-parents a model root; nothing happens when it has no parent or is
    // already under `to`.
    void moveTo(RE::NiAVObject* root, RE::NiNode* to);

    // Creates the carrier once and keeps it under `parent`, re-parenting it
    // when its old parent is gone. False (logged once through `failed`) when
    // the engine node cannot be created.
    bool attachCarrier(RE::NiPointer<RE::NiNode>& carrier, bool& failed, RE::NiNode* parent, const char* name);

    // Carrier local transform that puts `root` (with its own local transform)
    // at `rootWorld` under `parent`: parent^-1 * rootWorld * rootLocal^-1.
    [[nodiscard]] RE::NiTransform carrierLocalFor(const RE::NiTransform& rootWorld, const RE::NiTransform& rootLocal,
        const RE::NiTransform& parentWorld);
}
