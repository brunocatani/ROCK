#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "api/ProviderRuntimeServices.h"
#include "physics-interaction/native/NativeAmmoCounterVisibility.h"
#include "physics-interaction/visual/AmmoCounters.h"
#include "physics-interaction/weapon/NativeSecondaryPresentation.h"

// Ammo counters: one per firing weapon on the palm side of the hand holding
// its firing grip, from this frame's publication (hand roles, per-slot rounds).

namespace rock
{
    namespace
    {
        ammo_counter_pose::Point toPoint(const RE::NiPoint3& value) noexcept
        {
            return { value.x, value.y, value.z };
        }

        RE::NiPoint3 toNiPoint(const ammo_counter_pose::Point& value) noexcept
        {
            return { value[0], value[1], value[2] };
        }
    }

    void PhysicsInteraction::publishAmmoCounters()
    {
        std::array<ammo_counters::HandCounter, 2> hands{};
        std::array<bool, 2> anchored{};
        const auto& runtime = runtime_state::currentFrame();
        provider::Publication publication{};
        auto* nodes = f4vr::getPlayerNodes();
        if (_lifecycle.initialized.load(std::memory_order_acquire) && runtime.localSkeletonReady &&
            !runtime.localMenuBlocking && runtime.weaponDrawn && nodes && nodes->HmdNode &&
            provider::runtime::copyPublication(publication) && publication.frameIndex == runtime.frameIndex) {
            const auto head = toPoint(nodes->HmdNode->world.translate);
            using WeaponFlag = api::weapon::WeaponStateFlag;
            constexpr auto required = static_cast<std::uint32_t>(WeaponFlag::AmmoKnown) |
                static_cast<std::uint32_t>(WeaponFlag::WeaponDrawn);
            for (const bool isLeft : { false, true }) {
                const auto index = isLeft ? 1u : 0u;
                const auto& role = publication.handRoles[index];
                if (role.role != api::hands::HandRole::Firing) continue;
                api::weapon::WeaponState state{};
                if (provider::runtime::copyPublishedWeaponState(state, role.weaponSlot) != api::Status::Ok ||
                    (state.flags & required) != required) {
                    continue;
                }
                // The slot's presented weapon and the rendered wrist, both
                // final after AfterWorldFinal.
                const RE::NiAVObject* weapon = role.weaponSlot == 0 ?
                    static_cast<RE::NiAVObject*>(resolveEquippedWeaponInteractionNode()) :
                    native_secondary_presentation::presentedModel();
                RE::NiTransform hand{};
                if (!weapon || !finiteNiTransform(weapon->world) ||
                    !frik_visual_authority::tryGetBoneWorldTransform(isLeft ? "LArm_Hand" : "RArm_Hand", hand)) {
                    continue;
                }
                const auto& weaponWorld = weapon->world;
                const auto palmInWeapon = transform_math::worldPointToLocal(
                    weaponWorld, computeGrabPalmPointWorldFromHandBasis(hand, isLeft));
                const auto& anchor = _ammoCounterAnchors[index].update(
                    { role.weaponSlot, state.weaponFormId, state.weaponGenerationKey }, toPoint(palmInWeapon));
                anchored[index] = true;
                const auto palm = transform_math::localPointToWorld(weaponWorld, toNiPoint(anchor));
                const auto weaponRight = transform_math::rotateLocalVectorToWorld(weaponWorld.rotate, RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
                const auto weaponUp = transform_math::rotateLocalVectorToWorld(weaponWorld.rotate, RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
                auto& counter = hands[index];
                counter.pose = ammo_counter_pose::resolve(toPoint(palm), toPoint(weaponRight), toPoint(weaponUp), isLeft, head);
                counter.shown = counter.pose.valid;
                counter.loaded = state.loadedRounds;
                counter.reserve = state.reserveRounds;
            }
        }
        // A hand that stops firing recaptures its palm on its next weapon.
        for (std::size_t index = 0; index < anchored.size(); ++index) {
            if (!anchored[index]) _ammoCounterAnchors[index].clear();
        }
        // The counters replace the vanilla one only while RPS UI shows them.
        const bool replacing = ammo_counters::update(hands) && native_ammo_counter_visibility::install();
        native_ammo_counter_visibility::replace(replacing);
    }
}
