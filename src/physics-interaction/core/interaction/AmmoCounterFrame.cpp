#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "api/ProviderRuntimeServices.h"
#include "physics-interaction/native/NativeAmmoCounterVisibility.h"
#include "physics-interaction/visual/AmmoCounters.h"

// Ammo counters: one per firing weapon on the back of the hand holding its
// firing grip, from this frame's publication (hand roles, per-slot rounds).

namespace rock
{
    namespace
    {
        ammo_counter_pose::Point toPoint(const RE::NiPoint3& value) noexcept
        {
            return { value.x, value.y, value.z };
        }
    }

    void PhysicsInteraction::publishAmmoCounters()
    {
        std::array<ammo_counters::HandCounter, 2> hands{};
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
                const auto& role = publication.handRoles[isLeft ? 1 : 0];
                if (role.role != api::hands::HandRole::Firing) continue;
                api::weapon::WeaponState state{};
                if (provider::runtime::copyPublishedWeaponState(state, role.weaponSlot) != api::Status::Ok ||
                    (state.flags & required) != required) {
                    continue;
                }
                // The rendered wrist, final after AfterWorldFinal.
                RE::NiTransform hand{};
                if (!frik_visual_authority::tryGetBoneWorldTransform(isLeft ? "LArm_Hand" : "RArm_Hand", hand)) continue;
                const auto fingers = transformHandspaceDirection(hand, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, isLeft);
                const auto backOfHand = computePalmNormalFromHandBasis(hand, isLeft) * -1.0f;
                auto& counter = hands[isLeft ? 1 : 0];
                counter.pose = ammo_counter_pose::resolve(toPoint(hand.translate), toPoint(fingers), toPoint(backOfHand), head);
                counter.shown = counter.pose.valid;
                counter.loaded = state.loadedRounds;
                counter.reserve = state.reserveRounds;
            }
        }
        // The counters replace the vanilla one only while RPS UI shows them.
        const bool replacing = ammo_counters::update(hands) && native_ammo_counter_visibility::install();
        native_ammo_counter_visibility::replace(replacing);
    }
}
