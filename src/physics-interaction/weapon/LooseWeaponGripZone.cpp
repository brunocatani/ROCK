#include "physics-interaction/weapon/LooseWeaponGripZone.h"

#include <array>
#include <cmath>

#include "RockConfig.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/grab/FrikWeaponOffsetCache.h"
#include "physics-interaction/weapon/TwoHandedGrip.h"

#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/NetImmerse/NiNode.h"

#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"

namespace rock::loose_weapon_grip_zone
{
    namespace
    {
        struct HandZoneState
        {
            bool valid{ false };
            bool palmValid{ false };
            bool insideRadius{ false };
            float insideSettledSeconds{ 0.0f };
            float palmToGripDistance{ 0.0f };
            RE::NiPoint3 gripWorld{};
            RE::NiPoint3 palmWorld{};
            const char* reason{ "notEvaluated" };
        };

        std::array<HandZoneState, 2> s_handStates{};

        std::size_t handIndex(const bool isLeft) { return isLeft ? 0u : 1u; }

        bool isFinitePoint(const RE::NiPoint3& point)
        {
            return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
        }

        bool isUsableWorldTransform(const RE::NiTransform& transform)
        {
            return isFinitePoint(transform.translate) && std::isfinite(transform.scale) && std::fabs(transform.scale) > 0.0001f;
        }

        float pointDistance(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            const float dx = lhs.x - rhs.x;
            const float dy = lhs.y - rhs.y;
            const float dz = lhs.z - rhs.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        /*
         * Project the FRIK offset onto the held world model: build the root
         * world the weapon would have if attached to the hand at the offset
         * (same math as the loose-weapon primary attach frame, including the
         * root-scale override), express the live palm in that hypothetical
         * weapon space, then map the point onto the actual held root. Grenades
         * and other throwables resolve through the throwable offset store
         * upstream and never reach the equip path, so only the primary weapon
         * offset is consulted here.
         */
        bool tryResolveGripWorld(RE::TESObjectREFR* heldRef, const RE::NiPoint3& palmWorld, HandZoneState& state)
        {
            auto* baseForm = heldRef->GetObjectReference();
            const auto* weapon = baseForm ? baseForm->As<RE::TESObjectWEAP>() : nullptr;
            if (!weapon) {
                state.reason = "missingWeaponForm";
                return false;
            }

            auto* looseRoot = heldRef->Get3D();
            if (!looseRoot || !isUsableWorldTransform(looseRoot->world)) {
                state.reason = "missingWeaponRoot";
                return false;
            }

            const auto lookup = frik_weapon_offset_cache::findPrimaryWeaponOffset(weapon, looseRoot);
            if (!lookup.found) {
                state.reason = lookup.reason;
                return false;
            }

            auto* weaponNode = f4vr::getWeaponNode();
            auto* attachParent = weaponNode ? weaponNode->parent : nullptr;
            if (!attachParent || !isUsableWorldTransform(attachParent->world)) {
                state.reason = "missingPrimaryWeaponParent";
                return false;
            }

            RE::NiTransform attachedRootWorld = transform_math::composeTransforms(attachParent->world, lookup.offset);
            attachedRootWorld.scale = looseRoot->world.scale;
            if (!isUsableWorldTransform(attachedRootWorld)) {
                state.reason = "nonFiniteAttachedRoot";
                return false;
            }

            const RE::NiPoint3 palmInWeaponLocal = transform_math::worldPointToLocal(attachedRootWorld, palmWorld);
            state.gripWorld = transform_math::localPointToWorld(looseRoot->world, palmInWeaponLocal);
            if (!isFinitePoint(state.gripWorld)) {
                state.reason = "nonFiniteGripPoint";
                return false;
            }

            state.reason = lookup.reason;
            return true;
        }
    }

    void updateHeldLooseWeapon(const bool isLeft, const bool holdingLooseWeapon, RE::TESObjectREFR* heldRef, const bool heldSettled, const float dt)
    {
        auto& state = s_handStates[handIndex(isLeft)];
        if (!holdingLooseWeapon || !heldRef) {
            state = {};
            return;
        }

        HandZoneState next{};
        next.insideSettledSeconds = state.insideSettledSeconds;

        RE::NiTransform palmHandWorld{};
        next.palmValid = TwoHandedGrip::tryCaptureRootFlattenedPalmWorld(isLeft, next.palmWorld, palmHandWorld) && isFinitePoint(next.palmWorld);
        if (!next.palmValid) {
            next.reason = "missingPalm";
            next.insideSettledSeconds = 0.0f;
            state = next;
            return;
        }

        next.valid = tryResolveGripWorld(heldRef, next.palmWorld, next);
        if (!next.valid) {
            next.insideSettledSeconds = 0.0f;
            if (state.valid || state.reason != next.reason) {
                ROCK_LOG_INFO(Hand,
                    "{} hand loose weapon grip zone unavailable: reason={} formID={:08X}",
                    isLeft ? "left" : "right",
                    next.reason,
                    heldRef->GetFormID());
            }
            state = next;
            return;
        }

        next.palmToGripDistance = pointDistance(next.palmWorld, next.gripWorld);
        next.insideRadius = next.palmToGripDistance <= g_rockConfig.rockGrabbedWeaponGripZoneEquipRadius;
        if (next.insideRadius && heldSettled) {
            next.insideSettledSeconds += (std::max)(0.0f, dt);
        } else {
            next.insideSettledSeconds = 0.0f;
        }

        if (next.insideRadius != state.insideRadius) {
            ROCK_LOG_INFO(Hand,
                "{} hand loose weapon grip zone {}: palmDist={:.2f}gu radius={:.2f}gu heldSettled={} offsetSource={}",
                isLeft ? "left" : "right",
                next.insideRadius ? "entered" : "exited",
                next.palmToGripDistance,
                g_rockConfig.rockGrabbedWeaponGripZoneEquipRadius,
                heldSettled ? "yes" : "no",
                next.reason);
        }

        state = next;
    }

    bool isGripZoneEquipSettled(const bool isLeft)
    {
        const auto& state = s_handStates[handIndex(isLeft)];
        return state.valid &&
               state.insideRadius &&
               state.insideSettledSeconds >= g_rockConfig.rockGrabbedWeaponGripZoneEquipSettleSeconds;
    }

    bool tryGetGripZoneDebug(const bool isLeft, GripZoneDebug& out)
    {
        const auto& state = s_handStates[handIndex(isLeft)];
        if (!state.valid) {
            return false;
        }
        out.valid = state.valid;
        out.palmValid = state.palmValid;
        out.insideRadius = state.insideRadius;
        out.gripWorld = state.gripWorld;
        out.palmWorld = state.palmWorld;
        out.palmToGripDistance = state.palmToGripDistance;
        return true;
    }
}
