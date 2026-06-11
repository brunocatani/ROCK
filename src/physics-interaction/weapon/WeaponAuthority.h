#pragma once

/*
 * Weapon authority helpers are grouped here because lifecycle, visual authority, muzzle authority, and visual composition all decide final weapon ownership.
 */


// ---- WeaponAuthorityLifecyclePolicy.h ----

#include <array>
#include <cstdint>
#include <cstddef>
#include <string_view>

namespace rock::weapon_authority_lifecycle_policy
{
    /*
     * Game-stopping menus and world interruptions must make every ROCK-owned
     * authority yield before returning to the game. ROCK keeps FRIK weapon
     * authority, input blocking, and left-hand collision suppression in separate
     * systems, so the interruption decision is centralized here to keep all
     * three lifetimes in lockstep.
     */
    inline bool shouldClearWeaponAuthorityForUpdateInterruption(bool menuOpen, bool rockDisabled, bool skeletonUnavailable)
    {
        return menuOpen || rockDisabled || skeletonUnavailable;
    }

    inline bool isWeaponContactGenerationCurrent(std::uint64_t contactGenerationKey, std::uint64_t currentGenerationKey)
    {
        /*
         * Zero means the older path could not report a source generation. Treat
         * it as usable so legacy non-generated contacts do not get dropped, but
         * reject explicit mismatches because that is the melee/modded-weapon
         * failure mode: a contact from one generated source tree drives a grip
         * solve on another weapon tree after a swap or root change.
         */
        return contactGenerationKey == 0 || currentGenerationKey == 0 || contactGenerationKey == currentGenerationKey;
    }

}

// ---- WeaponVisualAuthorityMath.h ----

#include "physics-interaction/TransformMath.h"

#include <array>

namespace rock::weapon_visual_authority_math
{
    /*
     * ROCK-owned equipped weapon authority must write the same final visual
     * weapon frame that later drives generated weapon collision bodies. Keeping
     * world-target to parent-local conversion as pure math avoids duplicating
     * node write convention code across two-handed grip, one-handed mesh grip,
     * and debug verification paths. Two-handed support also needs a locked hand
     * frame: controllers guide the weapon solve, while visible hands are
     * recomposed from stored weapon-local frames so the mesh contact point cannot
     * slide along the gun.
     */

    enum class TwoHandedExternalAuthorityStep
    {
        ApplyWeaponVisual,
        PublishHandPose,
        ApplyLockedHandVisual
    };

    enum class LockedHandRole
    {
        Primary,
        Support
    };

    inline constexpr std::array<TwoHandedExternalAuthorityStep, 3> twoHandedExternalAuthorityOrder()
    {
        /*
         * The order is part of the cross-mod contract with FRIK. The weapon must
         * be in its final ROCK-owned frame before hand targets are composed, and
         * the hand pose must be published before FRIK applies/finalizes the
         * external wrist target so right-hand fingers inherit the pivoted wrist.
         */
        return {
            TwoHandedExternalAuthorityStep::ApplyWeaponVisual,
            TwoHandedExternalAuthorityStep::PublishHandPose,
            TwoHandedExternalAuthorityStep::ApplyLockedHandVisual
        };
    }

    inline constexpr int authorityOrderIndex(const TwoHandedExternalAuthorityStep step)
    {
        const auto order = twoHandedExternalAuthorityOrder();
        for (int index = 0; index < static_cast<int>(order.size()); ++index) {
            if (order[static_cast<std::size_t>(index)] == step) {
                return index;
            }
        }
        return -1;
    }

    inline constexpr bool handPosePrecedesLockedHandAuthority()
    {
        return authorityOrderIndex(TwoHandedExternalAuthorityStep::PublishHandPose) <
               authorityOrderIndex(TwoHandedExternalAuthorityStep::ApplyLockedHandVisual);
    }

    inline constexpr bool weaponVisualPrecedesLockedHandAuthority()
    {
        return authorityOrderIndex(TwoHandedExternalAuthorityStep::ApplyWeaponVisual) <
               authorityOrderIndex(TwoHandedExternalAuthorityStep::ApplyLockedHandVisual);
    }

    inline constexpr bool shouldPublishTwoHandedGripPose(const LockedHandRole role)
    {
        /*
         * The primary/right hand already has FRIK's native weapon grip pose. ROCK
         * only needs to move that wrist with the final weapon frame. Publishing a
         * primary mesh pose would replace the user's tuned FRIK grip pose, while
         * the support hand still needs ROCK's mesh/contact pose.
         */
        return role == LockedHandRole::Support;
    }

    inline constexpr bool shouldUseMeshGripFrameRotationAtGrabStart(const LockedHandRole)
    {
        /*
         * Mesh semantics may choose the grip point, but locked hand authority
         * must preserve the wrist rotation already produced by FRIK/INI/native
         * weapon setup at the moment support grip starts. Replacing the right
         * wrist with the mesh-derived grip frame is what causes the visible
         * hand to snap up or down on grab activation.
         */
        return false;
    }

    inline constexpr bool shouldSelectMeshGripPointAtGrabStart(const LockedHandRole role)
    {
        /*
         * The support hand is allowed to select the mesh contact it actually
         * touched. The primary/right hand is not reselected from weapon part
         * names or bounds when support grip starts; the current FRIK-configured
         * hand-to-weapon relationship is already the correct primary grip.
         */
        return role == LockedHandRole::Support;
    }

    template <class Transform>
    inline Transform worldTargetToParentLocal(const Transform& parentWorld, const Transform& targetWorld)
    {
        return transform_math::composeTransforms(transform_math::invertTransform(parentWorld), targetWorld);
    }

    template <class Transform>
    inline Transform weaponLocalFrameToWorld(const Transform& weaponWorld, const Transform& weaponLocalFrame)
    {
        return transform_math::composeTransforms(weaponWorld, weaponLocalFrame);
    }
}

// ---- WeaponMuzzleAuthorityMath.h ----

namespace rock::weapon_muzzle_authority_math
{
    /*
     * ROCK writes the final equipped weapon transform after FRIK's normal weapon
     * update has already run. FRIK's proven muzzle fix copies the projectile node
     * world transform into the fire node so flashes/projectiles originate at the
     * current barrel tip. Keeping the transform rule here gives ROCK the same
     * final-owner behavior without depending on FRIK's weapon adjuster pass.
     */
    template <class Transform>
    [[nodiscard]] Transform fireNodeLocalFromProjectileWorld(const Transform& projectileWorld)
    {
        return projectileWorld;
    }
}

// ---- WeaponVisualCompositionPolicy.h ----

#include <cstdint>
#include <string_view>

namespace rock::weapon_visual_composition_policy
{
    /*
     * Weapon mod swaps can expose the weapon node before the edited part's
     * renderer data has settled. ROCK therefore separates runtime visual
     * composition from transforms: geometry/visibility identity participates in
     * rebuild decisions, while ordinary animation and weapon motion do not.
     */
    inline constexpr std::uint64_t kWeaponVisualCompositionOffset = 1469598103934665603ull;
    inline constexpr std::uint64_t kWeaponVisualCompositionPrime = 1099511628211ull;

    struct VisualRecord
    {
        std::uintptr_t nodeAddress{ 0 };
        std::uintptr_t parentAddress{ 0 };
        std::string_view name{};
        std::uint32_t depth{ 0 };
        std::uint32_t childIndex{ 0 };
        std::uint32_t childCount{ 0 };
        bool visible{ false };
        bool triShape{ false };
        std::uintptr_t rendererData{ 0 };
        std::uintptr_t skinInstance{ 0 };
        std::uintptr_t vertexBlock{ 0 };
        std::uintptr_t triangleBlock{ 0 };
        std::uint64_t vertexDesc{ 0 };
        std::uint32_t numTriangles{ 0 };
        std::uint32_t numVertices{ 0 };
        std::uint32_t geometryType{ 0 };
    };

    inline void mixValue(std::uint64_t& key, std::uint64_t value)
    {
        key ^= value;
        key *= kWeaponVisualCompositionPrime;
    }

    inline void mixString(std::uint64_t& key, std::string_view value)
    {
        for (const char ch : value) {
            mixValue(key, static_cast<unsigned char>(ch));
        }
        mixValue(key, value.size());
    }

    inline void mixVisualRecord(std::uint64_t& key, const VisualRecord& record)
    {
        mixValue(key, record.nodeAddress);
        mixValue(key, record.parentAddress);
        mixString(key, record.name);
        mixValue(key, record.depth);
        mixValue(key, record.childIndex);
        mixValue(key, record.childCount);
        mixValue(key, record.visible ? 1ULL : 0ULL);
        mixValue(key, record.triShape ? 1ULL : 0ULL);
        mixValue(key, record.rendererData);
        mixValue(key, record.skinInstance);
        mixValue(key, record.vertexBlock);
        mixValue(key, record.triangleBlock);
        mixValue(key, record.vertexDesc);
        mixValue(key, record.numTriangles);
        mixValue(key, record.numVertices);
        mixValue(key, record.geometryType);
    }

}

// ---- WeaponGeneratedSourceCompletenessPolicy.h ----

#include "physics-interaction/weapon/WeaponTypes.h"

#include <cstdint>

namespace rock::weapon_generated_source_completeness_policy
{
    /*
     * FO4VR weapon attachments can become extractable after the first visible
     * weapon root has already stabilized. A spawned modded firearm may therefore
     * produce a partial generated source set, then gain a stock or attachment
     * only after an unrelated mod swap forces a rebuild. ROCK tracks source-set
     * completeness separately from visual identity so firearms can self-enrich
     * when late parts appear while existing bodies remain live until a better
     * replacement set is ready.
     */
    struct GeneratedSourceCompleteness
    {
        std::uint64_t signature{ 0 };
        std::uint64_t geometryHash{ 0 };
        std::uint64_t boundsExtentScore{ 0 };
        std::size_t sourceCount{ 0 };
        std::size_t pointCount{ 0 };
        std::size_t childClusterCount{ 0 };
        std::uint32_t semanticPartMask{ 0 };
        std::uint32_t gameplayCriticalCount{ 0 };
        std::size_t durableSourceCount{ 0 };
        std::size_t durableChildClusterCount{ 0 };
        std::size_t durablePointCount{ 0 };
        std::uint64_t durableBoundsExtentScore{ 0 };
        std::uint64_t durableGeometryHash{ 0 };
        std::size_t transientReloadSourceCount{ 0 };
        std::uint32_t missingRequiredPackageCoverageMask{ 0 };
        bool firearmLikePackage{ false };
        bool hasRequiredFrontCoverage{ false };
        bool hasRequiredRearCoverage{ false };
    };

    inline std::uint32_t partMask(WeaponPartKind kind)
    {
        const auto index = static_cast<std::uint32_t>(kind);
        if (index >= 31) {
            return 0;
        }
        return 1u << index;
    }

    inline bool hasPart(const GeneratedSourceCompleteness& completeness, WeaponPartKind kind)
    {
        return (completeness.semanticPartMask & partMask(kind)) != 0;
    }

    inline constexpr std::uint32_t kMissingFrontPackageCoverage = 1u << 0;
    inline constexpr std::uint32_t kMissingRearPackageCoverage = 1u << 1;

    inline bool hasFrontPackageCoverage(const GeneratedSourceCompleteness& completeness)
    {
        return hasPart(completeness, WeaponPartKind::Barrel) ||
               hasPart(completeness, WeaponPartKind::Handguard) ||
               hasPart(completeness, WeaponPartKind::Foregrip) ||
               hasPart(completeness, WeaponPartKind::Pump);
    }

    inline bool hasCompactRearPackageCoverage(const GeneratedSourceCompleteness& completeness)
    {
        return hasPart(completeness, WeaponPartKind::Stock) ||
               hasPart(completeness, WeaponPartKind::Grip);
    }

    inline bool hasLongGunRearPackageCoverage(const GeneratedSourceCompleteness& completeness)
    {
        return hasPart(completeness, WeaponPartKind::Stock);
    }

    inline bool hasFirearmActionPackageEvidence(const GeneratedSourceCompleteness& completeness)
    {
        return hasPart(completeness, WeaponPartKind::Magazine) ||
               hasPart(completeness, WeaponPartKind::Magwell) ||
               hasPart(completeness, WeaponPartKind::Bolt) ||
               hasPart(completeness, WeaponPartKind::Slide) ||
               hasPart(completeness, WeaponPartKind::ChargingHandle) ||
               hasPart(completeness, WeaponPartKind::BreakAction) ||
               hasPart(completeness, WeaponPartKind::Cylinder) ||
               hasPart(completeness, WeaponPartKind::Chamber) ||
               hasPart(completeness, WeaponPartKind::LaserCell) ||
               hasPart(completeness, WeaponPartKind::Lever);
    }

    inline bool isFirearmLikeSourcePackage(const GeneratedSourceCompleteness& completeness)
    {
        /*
         * This is retained as telemetry for logs and debug visualizers only.
         * CommonLib/WEAP classification can be unavailable or Unknown while the
         * visible source package clearly has receiver/action/magazine evidence,
         * but the result no longer gates collision publication.
         */
        if (completeness.sourceCount == 0 || !hasPart(completeness, WeaponPartKind::Receiver)) {
            return false;
        }

        const bool actionOrSocketEvidence = hasFirearmActionPackageEvidence(completeness);
        const bool authoredLongGunCoverage = hasFrontPackageCoverage(completeness) && hasCompactRearPackageCoverage(completeness);
        const bool sightedWeaponPackage = hasPart(completeness, WeaponPartKind::Sight) && (actionOrSocketEvidence || authoredLongGunCoverage);
        return actionOrSocketEvidence || sightedWeaponPackage || authoredLongGunCoverage;
    }

    inline bool hasLongGunPackageEvidence(const GeneratedSourceCompleteness& completeness)
    {
        /*
         * Retained as package telemetry. It used to drive front/rear coherence
         * blocking, but modded weapons can expose valid visible parts without
         * those names. Collision generation now consumes the visible mesh set.
         */
        return hasPart(completeness, WeaponPartKind::Stock) ||
               hasPart(completeness, WeaponPartKind::Handguard) ||
               hasPart(completeness, WeaponPartKind::Foregrip) ||
               hasPart(completeness, WeaponPartKind::Pump) ||
               hasPart(completeness, WeaponPartKind::Bolt) ||
               hasPart(completeness, WeaponPartKind::ChargingHandle) ||
               hasPart(completeness, WeaponPartKind::Lever);
    }

    inline bool hasRequiredRearPackageCoverage(const GeneratedSourceCompleteness& completeness)
    {
        return hasLongGunPackageEvidence(completeness) ?
            hasLongGunRearPackageCoverage(completeness) :
            hasCompactRearPackageCoverage(completeness);
    }

    inline std::uint32_t missingRequiredPackageCoverageMask(const GeneratedSourceCompleteness& completeness)
    {
        /*
         * Collision generation is visibility-driven: if a mesh is present,
         * buildable, and survives geometry dedupe, it gets collision. The old
         * front/rear firearm package gate guessed that a stable source package
         * was incomplete when names like stock/barrel were absent; modded FO4VR
         * weapons routinely violate those lexical assumptions, so the gate
         * blocked exactly the last-applied parts we were trying to recover.
         * Keep the package evidence fields as telemetry only.
         */
        (void)completeness;
        return 0;
    }

    inline GeneratedSourceCompleteness withDerivedPackageCoverage(GeneratedSourceCompleteness completeness)
    {
        completeness.firearmLikePackage = isFirearmLikeSourcePackage(completeness);
        completeness.hasRequiredFrontCoverage = hasFrontPackageCoverage(completeness);
        completeness.hasRequiredRearCoverage = hasRequiredRearPackageCoverage(completeness);
        completeness.missingRequiredPackageCoverageMask = missingRequiredPackageCoverageMask(completeness);
        return completeness;
    }

    inline std::uint32_t permanentGameplayCriticalPartMask()
    {
        /*
         * Reload ammo displays are intentionally excluded from this permanence
         * mask. They can appear/disappear during action animation, while stock,
         * receiver, barrel/support, magazine/socket, and action components are
         * persistent weapon structure for collision and support-grip routing.
         */
        return partMask(WeaponPartKind::Receiver) |
               partMask(WeaponPartKind::Barrel) |
               partMask(WeaponPartKind::Handguard) |
               partMask(WeaponPartKind::Foregrip) |
               partMask(WeaponPartKind::Pump) |
               partMask(WeaponPartKind::Stock) |
               partMask(WeaponPartKind::Grip) |
               partMask(WeaponPartKind::Magazine) |
               partMask(WeaponPartKind::Magwell) |
               partMask(WeaponPartKind::Bolt) |
               partMask(WeaponPartKind::Slide) |
               partMask(WeaponPartKind::ChargingHandle) |
               partMask(WeaponPartKind::BreakAction) |
               partMask(WeaponPartKind::Cylinder) |
               partMask(WeaponPartKind::Chamber) |
               partMask(WeaponPartKind::LaserCell) |
               partMask(WeaponPartKind::Lever);
    }

    inline bool isTransientReloadPart(WeaponPartKind kind)
    {
        return kind == WeaponPartKind::Shell ||
               kind == WeaponPartKind::Round ||
               kind == WeaponPartKind::CosmeticAmmo;
    }

    inline std::uint64_t makeGeneratedWeaponBodySetKey(
        std::uint64_t equippedWeaponKey,
        const GeneratedSourceCompleteness& sourceCompleteness,
        std::uint64_t bodySetEpoch)
    {
        if (equippedWeaponKey == 0 || sourceCompleteness.signature == 0 || bodySetEpoch == 0) {
            return 0;
        }

        std::uint64_t key = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
        weapon_visual_composition_policy::mixString(key, "ROCKGeneratedWeaponBodySetV1");
        weapon_visual_composition_policy::mixValue(key, equippedWeaponKey);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.signature);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.geometryHash);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.boundsExtentScore);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.sourceCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.pointCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.childClusterCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.semanticPartMask);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.gameplayCriticalCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.durableSourceCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.durableChildClusterCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.durablePointCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.durableBoundsExtentScore);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.durableGeometryHash);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.transientReloadSourceCount);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.missingRequiredPackageCoverageMask);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.firearmLikePackage ? 1ULL : 0ULL);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.hasRequiredFrontCoverage ? 1ULL : 0ULL);
        weapon_visual_composition_policy::mixValue(key, sourceCompleteness.hasRequiredRearCoverage ? 1ULL : 0ULL);
        weapon_visual_composition_policy::mixValue(key, bodySetEpoch);
        return key;
    }
}

// ---- WeaponGenerationIdentityPolicy.h ----

namespace rock::weapon_generation_identity_policy
{
    /*
     * Generated collision rebuild authority comes from the equipped weapon
     * instance, not from reload-time visual tree churn. Visual composition is
     * still collected when a rebuild is already allowed, but it must not make a
     * stable equipped weapon look like a new collider lifecycle.
     */
    struct EquippedWeaponGenerationIdentity
    {
        std::uint32_t formID{ 0 };
        std::uintptr_t formAddress{ 0 };
        std::uintptr_t instanceDataAddress{ 0 };
        std::uintptr_t instanceKeywordDataAddress{ 0 };
        std::uint64_t instanceContentKey{ 0 };
        std::uintptr_t objectInstanceExtraAddress{ 0 };
        std::uint64_t objectIndexDataSignature{ 0 };
        std::uint32_t objectIndexDataCount{ 0 };
        std::uint32_t activeModCount{ 0 };
        std::uint32_t disabledModCount{ 0 };
        std::uintptr_t equippedDataAddress{ 0 };
        std::uintptr_t equippedObjectAddress{ 0 };
        std::string_view displayName{};
        bool hasEquippedWeapon{ false };
    };

    inline std::uint64_t makeEquippedWeaponIdentityKey(const EquippedWeaponGenerationIdentity& identity)
    {
        if (!identity.hasEquippedWeapon) {
            return 0;
        }

        std::uint64_t key = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
        weapon_visual_composition_policy::mixString(key, "ROCKWeaponGenerationIdentityV1");
        weapon_visual_composition_policy::mixValue(key, identity.hasEquippedWeapon ? 1ULL : 0ULL);
        weapon_visual_composition_policy::mixValue(key, identity.formID);
        weapon_visual_composition_policy::mixValue(key, identity.formAddress);
        weapon_visual_composition_policy::mixValue(key, identity.instanceDataAddress);
        weapon_visual_composition_policy::mixValue(key, identity.instanceKeywordDataAddress);
        weapon_visual_composition_policy::mixValue(key, identity.instanceContentKey);
        weapon_visual_composition_policy::mixValue(key, identity.objectIndexDataSignature);
        weapon_visual_composition_policy::mixValue(key, identity.objectIndexDataCount);
        weapon_visual_composition_policy::mixValue(key, identity.activeModCount);
        weapon_visual_composition_policy::mixValue(key, identity.disabledModCount);
        weapon_visual_composition_policy::mixValue(key, identity.equippedDataAddress);
        weapon_visual_composition_policy::mixValue(key, identity.equippedObjectAddress);
        weapon_visual_composition_policy::mixString(key, identity.displayName);
        return key;
    }

    inline std::uint64_t makeEquippedWeaponGenerationKey(
        std::uint64_t visualCompositionKey,
        const EquippedWeaponGenerationIdentity& identity)
    {
        (void)visualCompositionKey;
        return makeEquippedWeaponIdentityKey(identity);
    }
}
