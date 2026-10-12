#pragma once

#include "physics-interaction/weapon/WeaponAuthority.h"
#include "RE/NetImmerse/NiNode.h"

namespace rock::weapon_scene_presentation
{
    inline constexpr std::size_t WEAPON_PRESENTATION_MAX_DEPTH = 128;
    inline constexpr std::size_t WEAPON_PRESENTATION_MAX_OBJECTS = 8192;

    inline bool isFiniteRotation(const RE::NiMatrix3& rotation)
    {
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(rotation.entry[row][column])) {
                    return false;
                }
            }
        }
        return true;
    }

    inline bool isFiniteTransform(const RE::NiTransform& transform)
    {
        return isFiniteRotation(transform.rotate) && std::isfinite(transform.translate.x) && std::isfinite(transform.translate.y) &&
               std::isfinite(transform.translate.z) && std::isfinite(transform.scale);
    }

    inline bool isInvertibleTransform(const RE::NiTransform& transform)
    {
        return isFiniteTransform(transform) &&
            std::abs(transform.scale) > 0.0001f;
    }

    inline bool validateWeaponPresentationSubtree(
        RE::NiAVObject* object,
        const RE::NiTransform& presentationWorldDelta,
        const std::size_t depth,
        std::size_t& objectCount)
    {
        if (!object || depth > WEAPON_PRESENTATION_MAX_DEPTH ||
            objectCount >= WEAPON_PRESENTATION_MAX_OBJECTS ||
            !isFiniteTransform(object->world)) {
            return false;
        }
        ++objectCount;

        const RE::NiTransform reframedWorld =
            weapon_visual_authority_math::applyPresentationWorldDelta(
                presentationWorldDelta,
                object->world);
        if (!isFiniteTransform(reframedWorld)) {
            return false;
        }

        auto* node = object->IsNode();
        if (!node) {
            return true;
        }
        for (const auto& child : node->children) {
            if (child && !validateWeaponPresentationSubtree(
                             child.get(),
                             presentationWorldDelta,
                             depth + 1,
                             objectCount)) {
                return false;
            }
        }
        return true;
    }

    inline void applyWeaponPresentationDeltaToDescendants(
        RE::NiNode* parent,
        const RE::NiTransform& presentationWorldDelta)
    {
        for (const auto& child : parent->children) {
            if (!child) {
                continue;
            }
            child->world =
                weapon_visual_authority_math::applyPresentationWorldDelta(
                    presentationWorldDelta,
                    child->world);
            if (auto* childNode = child->IsNode()) {
                applyWeaponPresentationDeltaToDescendants(
                    childNode,
                    presentationWorldDelta);
            }
        }
    }

    inline bool tryResolveWeaponRootLocal(
        const RE::NiNode* parent,
        const RE::NiTransform& weaponWorld,
        RE::NiTransform& outWeaponLocal)
    {
        if (!isFiniteTransform(weaponWorld)) {
            return false;
        }

        outWeaponLocal = weaponWorld;
        if (parent) {
            if (!isInvertibleTransform(parent->world)) {
                return false;
            }
            outWeaponLocal =
                weapon_visual_authority_math::worldTargetToParentLocal(
                    parent->world,
                    weaponWorld);
        }
        return isFiniteTransform(outWeaponLocal);
    }

    inline bool restoreWeaponRootPreservingPresentedDescendants(
        RE::NiNode* weaponNode,
        const RE::NiTransform& weaponWorld)
    {
        if (!weaponNode) {
            return false;
        }

        RE::NiTransform weaponLocal{};
        if (!tryResolveWeaponRootLocal(
                weaponNode->parent,
                weaponWorld,
                weaponLocal)) {
            return false;
        }
        weaponNode->local = weaponLocal;
        weaponNode->world = weaponWorld;
        return true;
    }

    inline bool moveWeaponPresentationRigidly(
        RE::NiNode* weaponNode,
        const RE::NiTransform& solvedWeaponWorld)
    {
        if (!weaponNode || !isInvertibleTransform(weaponNode->world) ||
            !isFiniteTransform(solvedWeaponWorld)) {
            return false;
        }

        RE::NiTransform weaponLocal{};
        if (!tryResolveWeaponRootLocal(
                weaponNode->parent,
                solvedWeaponWorld,
                weaponLocal)) {
            return false;
        }

        const RE::NiTransform oldWeaponWorld = weaponNode->world;
        const RE::NiTransform presentationWorldDelta =
            weapon_visual_authority_math::makePresentationWorldDelta(
                oldWeaponWorld,
                solvedWeaponWorld);
        if (!isFiniteTransform(presentationWorldDelta)) {
            return false;
        }

        // Validate the entire bounded tree before the first write so an
        // invalid modded node cannot leave a partially moved presentation.
        std::size_t objectCount = 0;
        if (!validateWeaponPresentationSubtree(
                weaponNode,
                presentationWorldDelta,
                0,
                objectCount)) {
            return false;
        }

        weaponNode->local = weaponLocal;
        weaponNode->world = solvedWeaponWorld;
        applyWeaponPresentationDeltaToDescendants(
            weaponNode,
            presentationWorldDelta);
        return true;
    }
}
