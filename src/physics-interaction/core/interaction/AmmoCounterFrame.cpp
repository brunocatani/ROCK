// Ammo counters: one per held weapon on the hand that holds it, from this
// frame's publication (hand roles, per-slot rounds). Minimal and Immersive sit
// at the weapon's firing grip on its palm side; Vanilla on the back of the hand.
#include "api/ProviderRuntimeServices.h"
#include "api/ProviderStatePolicy.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/visual/AmmoCounters.h"
#include "physics-interaction/visual/FrikHandWorldAuthority.h"
#include "physics-interaction/weapon/authored/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/authored/HoldingGripRecords.h"
#include "physics-interaction/weapon/classification/WeaponInstanceClassification.h"
#include "physics-interaction/weapon/grip/WeaponGripSeat.h"
#include "physics-interaction/weapon/native/NativeAmmoCounterVisibility.h"
#include "physics-interaction/weapon/secondary/NativeSecondaryPresentation.h"
#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace rock
{
    namespace
    {
        ammo_counter_pose::Point toPoint(const RE::NiPoint3& value) noexcept { return { value.x, value.y, value.z }; }

        ammo_counter_pose::Frame toFrame(const RE::NiTransform& world) noexcept
        {
            return {
                .x = toPoint(transform_math::rotateLocalVectorToWorld(world.rotate, RE::NiPoint3{ 1.0f, 0.0f, 0.0f })),
                .y = toPoint(transform_math::rotateLocalVectorToWorld(world.rotate, RE::NiPoint3{ 0.0f, 1.0f, 0.0f })),
                .z = toPoint(transform_math::rotateLocalVectorToWorld(world.rotate, RE::NiPoint3{ 0.0f, 0.0f, 1.0f })),
                .origin = toPoint(world.translate),
                .scale = world.scale,
            };
        }

        /*
         * The game's HUD color and opacity (Fallout4Prefs.ini [Interface]),
         * which the HUD styles follow like the vanilla counter. The settings
         * live in the game's static INI collections for the whole process, so
         * their pointers are looked up once; the values are read every frame
         * because the game's settings menu changes them live.
         */
        struct HudLook
        {
            std::array<float, 3> color{ 18.0f / 255.0f, 1.0f, 21.0f / 255.0f };
            float opacity{ 1.0f };
        };

        HudLook readHudLook() noexcept
        {
            static bool looked = false;
            static std::array<RE::Setting*, 3> colors{};
            static RE::Setting* opacity = nullptr;
            if (!looked) {
                looked = true;
                colors = { f4vr::getIniSetting("iHUDColorR:Interface"), f4vr::getIniSetting("iHUDColorG:Interface"),
                    f4vr::getIniSetting("iHUDColorB:Interface") };
                opacity = f4vr::getIniSetting("fHUDOpacity:Interface");
            }
            HudLook look{};
            for (std::size_t i = 0; i < colors.size(); ++i) {
                if (colors[i] && colors[i]->GetType() == RE::Setting::SETTING_TYPE::kInt) {
                    look.color[i] = static_cast<float>(std::clamp(colors[i]->GetInt(), 0, 255)) / 255.0f;
                }
            }
            if (opacity && opacity->GetType() == RE::Setting::SETTING_TYPE::kFloat &&
                std::isfinite(opacity->GetFloat())) {
                look.opacity = std::clamp(opacity->GetFloat(), 0.0f, 1.0f);
            }
            return look;
        }

        /*
         * The vanilla counter's screen: the ammo:0 quad of the wand HUD mesh
         * under the primary wand (FRIK's _primaryWand.nif puts it on the back
         * of the hand). The mesh is attached once and lives with the wand, so
         * the node is found by a bounded search at most once a second while
         * missing and kept while it still hangs under the wand. The holder is
         * never destroyed: releasing game nodes during process exit is unsafe.
         */
        constexpr char kVanillaAmmoQuadName[] = "ammo:0";
        constexpr std::uint32_t kVanillaQuadSearchDepth = 6;
        constexpr std::uint32_t kVanillaQuadSearchIntervalFrames = 90;
        auto& s_vanillaAmmoQuad = *new RE::NiPointer<RE::NiAVObject>{};

        RE::NiAVObject* findNamed(RE::NiAVObject* object, const std::uint32_t remainingDepth) noexcept
        {
            if (!object)
                return nullptr;
            if (const char* name = object->name.c_str(); name && std::strcmp(name, kVanillaAmmoQuadName) == 0)
                return object;
            auto* node = remainingDepth > 0 ? object->IsNode() : nullptr;
            if (!node)
                return nullptr;
            for (const auto& child : node->children) {
                if (auto* found = findNamed(child.get(), remainingDepth - 1))
                    return found;
            }
            return nullptr;
        }

        RE::NiAVObject* vanillaAmmoQuad(RE::NiNode* wand) noexcept
        {
            if (!wand)
                return nullptr;
            if (s_vanillaAmmoQuad) {
                const RE::NiAVObject* ancestor = s_vanillaAmmoQuad->parent;
                for (std::uint32_t hop = 0; ancestor && ancestor != wand && hop < kVanillaQuadSearchDepth; ++hop) {
                    ancestor = ancestor->parent;
                }
                if (ancestor == wand)
                    return s_vanillaAmmoQuad.get();
                s_vanillaAmmoQuad.reset();
            }
            static std::uint32_t framesUntilSearch = 0;
            static bool reportedMissing = false;
            if (framesUntilSearch > 0) {
                --framesUntilSearch;
                return nullptr;
            }
            framesUntilSearch = kVanillaQuadSearchIntervalFrames;
            if (auto* found = findNamed(wand, kVanillaQuadSearchDepth)) {
                s_vanillaAmmoQuad.reset(found);
                reportedMissing = false;
                ROCK_LOG_INFO(Weapon, "Ammo counter: vanilla wand ammo screen '{}' found", kVanillaAmmoQuadName);
                return found;
            }
            if (!reportedMissing) {
                reportedMissing = true;
                ROCK_LOG_WARN(Weapon, "Ammo counter: no '{}' under the primary wand; the Vanilla style stays hidden",
                    kVanillaAmmoQuadName);
            }
            return nullptr;
        }
    }

    void PhysicsInteraction::publishAmmoCounters()
    {
        std::array<ammo_counters::HandCounter, 2> hands{};
        const auto style = static_cast<ammo_counters::Style>(g_rockConfig.rockAmmoCounterStyle);
        const bool hudStyle = style != ammo_counters::Style::Minimal;
        const auto hudLook = hudStyle ? readHudLook() : HudLook{};
        const auto& runtime = runtime_state::currentFrame();
        provider::Publication publication{};
        auto* nodes = f4vr::getPlayerNodes();
        if (_lifecycle.initialized.load(std::memory_order_acquire) && runtime.localSkeletonReady &&
            !runtime.localMenuBlocking && runtime.weaponDrawn && nodes && nodes->HmdNode &&
            provider::runtime::copyPublication(publication) && publication.frameIndex == runtime.frameIndex) {
            const auto head = toPoint(nodes->HmdNode->world.translate);
            using WeaponFlag = api::weapon::WeaponStateFlag;
            constexpr auto required =
                static_cast<std::uint32_t>(WeaponFlag::AmmoKnown) | static_cast<std::uint32_t>(WeaponFlag::WeaponDrawn);
            for (std::uint32_t slot = 0; slot < api::kMaxWeaponSlots; ++slot) {
                api::weapon::WeaponState state{};
                if (provider::runtime::copyPublishedWeaponState(state, slot) != api::Status::Ok)
                    continue;
                // Slot 0's hands are applied at read time, as getWeaponState
                // does; slot 1 carries its own in the record.
                if (slot == 0) {
                    provider_state_policy::WeaponHandling handling{};
                    if (queryProviderWeaponHandling(handling))
                        provider_state_policy::applyHandling(state, handling);
                }
                if ((state.flags & required) != required ||
                    (state.firingHand != api::Hand::Right && state.firingHand != api::Hand::Left)) {
                    continue;
                }
                // handRoles: [0] right, [1] left.
                const auto holds = [&](std::size_t hand, bool firingOnly) {
                    const auto& role = publication.handRoles[hand];
                    return role.weaponSlot == slot &&
                        (firingOnly ? role.role == api::hands::HandRole::Firing
                                    : role.role != api::hands::HandRole::None);
                };
                if (!holds(0, false) && !holds(1, false))
                    continue;
                // The counter sits at the weapon's firing grip on the palm side
                // of the hand that holds it: the firing-grip hand, else the only
                // hand still on the weapon. A detached firing grip keeps its
                // firingHand record, so with akimbo both weapons could name the
                // same hand and one counter would overwrite the other.
                bool isLeft = state.firingHand == api::Hand::Left;
                if (holds(1, true) || holds(0, true)) {
                    isLeft = holds(1, true);
                } else if (holds(1, false) != holds(0, false)) {
                    isLeft = holds(1, false);
                }
                const auto index = isLeft ? 1u : 0u;
                // The slot's firing seat in its weapon's frame, final after
                // AfterWorldFinal. It needs no hand on the weapon.
                RE::NiTransform seat{}, weaponWorld{};
                const auto hidden = [&](const char* stage) {
                    ROCK_LOG_SAMPLE_INFO(Weapon, 2000, "Ammo counter hidden slot={} stage={} firingHand={} form={:08X}",
                        slot, stage, isLeft ? "left" : "right", state.weaponFormId);
                };
                if (style == ammo_counters::Style::Vanilla) {
                    // The back of the holding hand: the wand HUD's ammo screen,
                    // mirrored onto the left hand. It needs no weapon seat.
                    auto* quad = vanillaAmmoQuad(nodes->primaryWandNode);
                    if (!quad) {
                        hidden("vanilla-screen");
                        continue;
                    }
                    // The drawn hands: the counter mirrors onto the hand the
                    // player sees, whatever ROCK's interaction frames hold.
                    ammo_counter_pose::Frame rightBone{}, leftBone{};
                    if (isLeft) {
                        RE::NiTransform rightHand{}, leftHand{};
                        if (!frik_hand_world_authority::tryGetPresentedHandWorld(false, rightHand) ||
                            !frik_hand_world_authority::tryGetPresentedHandWorld(true, leftHand)) {
                            hidden("hand-bones");
                            continue;
                        }
                        rightBone = toFrame(rightHand);
                        leftBone = toFrame(leftHand);
                    }
                    auto& counter = hands[index];
                    counter.pose = ammo_counter_pose::resolveVanilla(toFrame(quad->world), isLeft, rightBone, leftBone,
                        head, hudLook.opacity);
                    counter.shown = counter.pose.valid;
                    counter.loaded = state.loadedRounds;
                    counter.reserve = state.reserveRounds;
                    counter.chamber = provider::chamberMarker(slot);
                    if (!counter.shown)
                        hidden("vanilla-pose");
                    continue;
                }
                if (slot == 0) {
                    // The same authored seat the firing grip places the hand on.
                    auto* node = resolveEquippedWeaponInteractionNode();
                    auto* weaponForm = currentEquippedWeaponForm();
                    if (!node || !weaponForm) {
                        hidden("weapon-node");
                        continue;
                    }
                    const bool powerArmor = f4vr::isInPowerArmor();
                    const auto native = weapon_grip_seat::resolveNative({
                        .weapon = weaponForm,
                        .weaponRoot = node,
                        .variant = authored_weapon_grip_library::identifyWeaponVariant(node),
                        .keywordFlags = weapon_instance_classification::computeWeaponKeywordFlags(weaponForm,
                            currentEquippedWeaponInstanceData(weaponForm)),
                        .inPowerArmor = powerArmor,
                    });
                    weapon_grip_seat::FiringSeat firing{};
                    if (!weapon_grip_seat::firingSeat(native,
                            holding_grips::find(weaponForm, node, isLeft, state.weaponGenerationKey, powerArmor),
                            isLeft, _weaponCollision.holdingMirrorOffsets().plane(false), firing)) {
                        hidden(native.modelRegistered ? "authored-seat" : "model-frame");
                        continue;
                    }
                    seat = firing.handWeaponLocal;
                    weaponWorld = node->world;
                } else if (!native_secondary_presentation::tryGetFiringSeat(seat, weaponWorld)) {
                    hidden("second-weapon-placement");
                    continue;
                }
                if (!finiteNiTransform(weaponWorld) || !finiteNiTransform(seat)) {
                    hidden("non-finite");
                    continue;
                }
                const auto palmInWeapon = computeGrabPalmPointWorldFromHandBasis(seat, isLeft);
                const auto palm = transform_math::localPointToWorld(weaponWorld, palmInWeapon);
                const auto weaponRight =
                    transform_math::rotateLocalVectorToWorld(weaponWorld.rotate, RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
                const auto weaponUp =
                    transform_math::rotateLocalVectorToWorld(weaponWorld.rotate, RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
                auto& counter = hands[index];
                counter.pose = style == ammo_counters::Style::Immersive
                    ? ammo_counter_pose::resolveImmersive(toPoint(palm), toPoint(weaponRight), toPoint(weaponUp),
                          isLeft, head)
                    : ammo_counter_pose::resolve(toPoint(palm), toPoint(weaponRight), toPoint(weaponUp), isLeft, head);
                counter.shown = counter.pose.valid;
                counter.loaded = state.loadedRounds;
                counter.reserve = state.reserveRounds;
                counter.chamber = provider::chamberMarker(slot);
            }
        }
        // The counters replace the vanilla one only while RPS UI shows them.
        const bool replacing =
            ammo_counters::update(hands, style, hudLook.color) && native_ammo_counter_visibility::install();
        native_ammo_counter_visibility::replace(replacing);
    }
}
