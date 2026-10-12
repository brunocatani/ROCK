#include "physics-interaction/weapon/NativeSecondaryPresentation.h"

#include "physics-interaction/weapon/NativeHandEquipment.h"
#include "physics-interaction/weapon/NativeIdleGripPreharvest.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/HoldingGripRecords.h"
#include "physics-interaction/weapon/WeaponAimBasis.h"
#include "physics-interaction/weapon/NativeSecondaryPose.h"
#include "physics-interaction/weapon/NativeSecondaryPoseTransport.h"
#include "physics-interaction/weapon/WeaponScenePresentation.h"
#include "physics-interaction/weapon/NativeWeaponModelGeometry.h"
#include "physics-interaction/weapon/NativeEquippedModelLifetime.h"
#include "physics-interaction/weapon/NativeSecondaryActions.h"
#include "physics-interaction/weapon/EquippedWeaponVisualState.h"
#include "physics-interaction/weapon/WeaponGripCalibration.h"
#include "physics-interaction/hand/HandFingerMirrorMath.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/grab/SkinnedBoneOwner.h"
#include "physics-interaction/PhysicsLog.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RE/NetImmerse/NiNode.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include <Windows.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <string_view>

namespace rock::native_secondary_presentation
{
    namespace
    {
        constexpr const char* kTag = "ROCK.NativeSecondaryFiring";
        constexpr int kPriority = 100;
        struct State
        {
            // Numeric identities only. Every scene write reacquires the native
            // equipment and biped record; no previous frame pointer is followed.
            std::uintptr_t root = 0, attachment = 0, instance = 0, data = 0;
            std::uintptr_t hiddenBody = 0, observedBody = 0;
            std::uint32_t form = 0;
            RE::NiTransform originalLocal{}, targetWorld{};
            RE::NiTransform controllerWorld{};
            float modelScale = 1.0f;
            bool ownsLocal = false, claimed = false, ready = false;
            // Physical hand of the index-1 weapon this frame, and the hand
            // whose FRIK publications are currently claimed.
            bool left = true, claimedLeft = true;
            bool geometryChecked = false, geometryReady = false, bodyWasVisible = false;
            bool traceFrame = false;
            std::uint64_t frame = 0;
            std::uint32_t samples = 0, nativeRepairs = 0;
            std::uint32_t traceNativePass = 0, tracedFrames = 0;
            float maxOverwriteDistance = 0, maxOverwriteAngle = 0;
            std::string_view reason{};
        };
        State s_state{}; // FRIK frame thread; native callbacks validate the same thread.
        std::atomic<DWORD> s_thread{0};
        bool s_updatedThisFrame = false;

        bool onFrameThread() noexcept
        {
            const auto owner = s_thread.load(std::memory_order_acquire);
            return owner != 0 && owner == GetCurrentThreadId();
        }

        bool usable(const RE::NiTransform& transform) noexcept
        {
            return holding_grip_math::valid(holding_grip_math::fromNative(transform));
        }

        bool matches(const native_hand_equipment::EquippedView& view) noexcept
        {
            return view.weapon && view.weapon->formID == s_state.form &&
                reinterpret_cast<std::uintptr_t>(view.root) == s_state.root &&
                reinterpret_cast<std::uintptr_t>(view.instance) == s_state.instance && view.data == s_state.data;
        }

        RE::NiNode* currentAttachment(const native_hand_equipment::EquippedView& view) noexcept
        {
            if (!matches(view) || !s_state.attachment) return nullptr;
            // Reacquire through the current model's ancestry, never through a
            // retained pointer. The native resolver established this identity
            // when the equipment/model changed; no per-frame tree search.
            auto* node = view.root;
            for (unsigned depth = 0; node && depth < 48; ++depth, node = node->parent) {
                if (reinterpret_cast<std::uintptr_t>(node) == s_state.attachment) return node->IsNode();
            }
            return nullptr;
        }

        void traceTransform(const char* phase, const char* label, const RE::NiTransform& pose)
        {
            const auto inController = transform_math::composeTransforms(
                transform_math::invertTransform(s_state.controllerWorld), pose);
            ROCK_LOG_INFO(Weapon,
                "AKIMBO secondary frame frame={} phase={} pass={} weapon={:08X} pose={} worldT=({:.4f},{:.4f},{:.4f}) scale={:.6f} "
                "wandT=({:.4f},{:.4f},{:.4f}) wandR=({:.6f},{:.6f},{:.6f};{:.6f},{:.6f},{:.6f};{:.6f},{:.6f},{:.6f})",
                s_state.frame, phase, s_state.traceNativePass, s_state.form, label,
                pose.translate.x, pose.translate.y, pose.translate.z, pose.scale,
                inController.translate.x, inController.translate.y, inController.translate.z,
                inController.rotate.entry[0][0], inController.rotate.entry[0][1], inController.rotate.entry[0][2],
                inController.rotate.entry[1][0], inController.rotate.entry[1][1], inController.rotate.entry[1][2],
                inController.rotate.entry[2][0], inController.rotate.entry[2][1], inController.rotate.entry[2][2]);
        }

        void traceNode(const char* phase, const char* label, const RE::NiAVObject* node)
        {
            RE::NiTransform world{};
            if (node && native_memory::tryReadValue(&node->world, world)) {
                traceTransform(phase, label, world);
                if (!usable(world)) {
                    const auto quality=native_secondary_pose_transport::inspect(world);
                    ROCK_LOG_INFO(Weapon,"AKIMBO pose quality frame={} phase={} name='{}' finite={} scale={} determinant={:.9f} orthogonalityError={:.9f}",
                        s_state.frame,phase,label,quality.finite,world.scale,quality.determinant,quality.orthogonalityError);
                }
            } else if (node) {
                ROCK_LOG_INFO(Weapon,"AKIMBO pose quality frame={} phase={} name='{}' readable=false",s_state.frame,phase,label);
            }
        }

        void traceGeometry(const char* phase) noexcept
        {
            if (!s_state.ready || !s_state.traceFrame) return;
            try {
                const auto view = native_hand_equipment::equippedViews()[1];
                auto* attachment = currentAttachment(view);
                if (!attachment) return;
                std::array<RE::NiAVObject*, 512> pending{};
                std::array<const RE::NiTransform*, 8> seenBoneWorlds{};
                pending[0] = attachment;
                std::size_t count = 1, shapes = 0, bones = 0;
                for (std::size_t i = 0; i < count && shapes < 8; ++i) {
                    auto* object = pending[i];
                    if (auto* node = object->IsNode()) {
                        for (const auto& child : node->GetRuntimeData().children) {
                            if (!child) continue;
                            if (count == pending.size()) break;
                            pending[count++] = child.get();
                        }
                    }
                    auto* geometry = object->IsGeometry();
                    if (!geometry) geometry = object->IsTriShape();
                    if (!geometry) continue;
                    ++shapes;
                    const char* name = geometry->name.c_str() ? geometry->name.c_str() : "<unnamed-shape>";
                    traceNode(phase, name, geometry);
                    traceTransform(phase, "geometry-previous-world", geometry->previousWorld);
                    auto* skin = geometry->GetRuntimeData().skinInstance.get();
                    const RE::NiTransform** worlds = nullptr;
                    RE::NiAVObject* skinRoot = nullptr;
                    std::uint32_t worldCount = 0;
                    // Same verified palette layout as MeshGrab, not the stale
                    // CommonLib count labels. No binding or bone is modified.
                    const bool readable = skin && nativeSkinLayoutVerified() &&
                        native_memory::tryReadField(skin, 0x28, worlds) &&
                        native_memory::tryReadField(skin, 0x38, worldCount) &&
                        native_memory::tryReadField(skin, 0x48, skinRoot) && worlds && worldCount <= 512;
                    ROCK_LOG_INFO(Weapon,
                        "AKIMBO secondary geometry frame={} phase={} pass={} name='{}' node={:#x} skin={:#x} paletteReadable={} bones={} skinRoot={:#x} flags={:#x} culled={} fade={}",
                        s_state.frame, phase, s_state.traceNativePass, name, reinterpret_cast<std::uintptr_t>(geometry),
                        reinterpret_cast<std::uintptr_t>(skin), readable, worldCount, reinterpret_cast<std::uintptr_t>(skinRoot),
                        geometry->GetFlags(),geometry->GetAppCulled(),geometry->fadeAmount);
                    if (!readable) continue;
                    for (std::uint32_t b = 0; b < worldCount && bones < seenBoneWorlds.size(); ++b) {
                        const RE::NiTransform* world = nullptr;
                        RE::NiTransform pose{};
                        if (!native_memory::tryReadValue(worlds + b, world) || !world ||
                            std::find(seenBoneWorlds.begin(), seenBoneWorlds.begin() + bones, world) != seenBoneWorlds.begin() + bones ||
                            !native_memory::tryReadValue(world, pose) || !usable(pose)) continue;
                        seenBoneWorlds[bones++] = world;
                        auto* owner = resolveFlattenedSkinBoneOwner(skinRoot, world);
                        const char* ownerName = owner && owner->name.c_str() ? owner->name.c_str() : "<node-world-or-unresolved>";
                        ROCK_LOG_INFO(Weapon,
                            "AKIMBO secondary skin binding frame={} phase={} pass={} shape='{}' bone={} worldPtr={:#x} flattenedOwner={:#x} ownerName='{}'",
                            s_state.frame, phase, s_state.traceNativePass, name, b, reinterpret_cast<std::uintptr_t>(world),
                            reinterpret_cast<std::uintptr_t>(owner), ownerName);
                        traceTransform(phase, "skin-bone-world", pose);
                    }
                }
                ROCK_LOG_INFO(Weapon, "AKIMBO secondary geometry census frame={} phase={} pass={} attachment={:#x} model={:#x} nodes={} shapes={} sampledBones={}",
                    s_state.frame, phase, s_state.traceNativePass, s_state.attachment, s_state.root, count, shapes, bones);
                if (std::string_view(phase) == "world-final") {
                    const auto primary = native_hand_equipment::equippedViews()[0];
                    const auto primaryGeometry = native_weapon_model_geometry::inspect(primary.root);
                    auto* primaryParent = primary.root ? primary.root->parent : nullptr;
                    ROCK_LOG_INFO(Weapon,
                        "AKIMBO rendered visibility frame={} rightWeapon={:08X} rightRoot={:#x} rightParent={:#x} rightShapes={} rightCulled={} rightParentCulled={} leftCulled={} leftParentCulled={} leftBody={:#x} leftBodyCulled={}",
                        s_state.frame, primary.weapon ? primary.weapon->formID : 0,
                        reinterpret_cast<std::uintptr_t>(primary.root), reinterpret_cast<std::uintptr_t>(primaryParent),
                        primaryGeometry.shapes, primary.root ? primary.root->GetAppCulled() : true,
                        primaryParent ? primaryParent->GetAppCulled() : true, view.root->GetAppCulled(), attachment->GetAppCulled(),
                        reinterpret_cast<std::uintptr_t>(view.bodyRoot),view.bodyRoot?view.bodyRoot->GetAppCulled():true);
                }
            } catch (...) {
                OutputDebugStringA("ROCK: secondary geometry trace failed\n");
            }
        }

        // A bounded sample of the same frame at every owner boundary. The
        // late native pass can temporarily rebuild the first-person tree in
        // another origin; its difference from a world target is not motion.
        void tracePlacement(const char* phase) noexcept
        {
            if (!s_state.traceFrame || !s_state.ready ||
                s_state.frame != runtime_state::currentFrame().frameIndex) return;
            try {
                const auto view = native_hand_equipment::equippedViews()[1];
                if (!matches(view) || !view.root || !view.root->parent) return;
                const auto* nodes = f4vr::getPlayerNodes();
                auto* parent = view.root->parent;
                traceTransform(phase, "target", s_state.targetWorld);
                traceNode(phase, "model", view.root);
                traceNode(phase, "attachment", parent);
                traceNode(phase, "native-equipped-attachment", currentAttachment(view));
                traceNode(phase, "attachment-parent", parent->parent);
                traceNode(phase, "first-person-root", f4vr::getFirstPersonSkeleton());
                traceNode(phase, "body-root", f4vr::getWorldRootNode());
                traceNode(phase, "wand-live", nodes ? (s_state.left ? nodes->SecondaryWandNode : nodes->primaryWandNode) : nullptr);
                const RE::NiAVObject* fireNode = nullptr;
                if (native_memory::tryReadField(reinterpret_cast<const void*>(view.data), 0x30, fireNode))
                    traceNode(phase, "native-fire-node", fireNode);
                RE::NiTransform renderedHand{};
                if (frik_visual_authority::tryGetPresentedHandWorldTransform(s_state.left, renderedHand))
                    traceTransform(phase, "body-hand", renderedHand);
                const auto& local = view.root->local;
                const auto& attach = parent->local;
                ROCK_LOG_INFO(Weapon,
                    "AKIMBO secondary local frame={} phase={} pass={} model={:#x} modelName='{}' attachment={:#x} attachmentName='{}' "
                    "modelT=({:.4f},{:.4f},{:.4f}) modelR=({:.6f},{:.6f},{:.6f};{:.6f},{:.6f},{:.6f};{:.6f},{:.6f},{:.6f}) "
                    "attachmentT=({:.4f},{:.4f},{:.4f}) attachmentR=({:.6f},{:.6f},{:.6f};{:.6f},{:.6f},{:.6f};{:.6f},{:.6f},{:.6f})",
                    s_state.frame, phase, s_state.traceNativePass, s_state.root,
                    view.root->name.c_str() ? view.root->name.c_str() : "<unnamed>", reinterpret_cast<std::uintptr_t>(parent),
                    parent->name.c_str() ? parent->name.c_str() : "<unnamed>",
                    local.translate.x, local.translate.y, local.translate.z,
                    local.rotate.entry[0][0], local.rotate.entry[0][1], local.rotate.entry[0][2],
                    local.rotate.entry[1][0], local.rotate.entry[1][1], local.rotate.entry[1][2],
                    local.rotate.entry[2][0], local.rotate.entry[2][1], local.rotate.entry[2][2],
                    attach.translate.x, attach.translate.y, attach.translate.z,
                    attach.rotate.entry[0][0], attach.rotate.entry[0][1], attach.rotate.entry[0][2],
                    attach.rotate.entry[1][0], attach.rotate.entry[1][1], attach.rotate.entry[1][2],
                    attach.rotate.entry[2][0], attach.rotate.entry[2][1], attach.rotate.entry[2][2]);
            } catch (...) {
                OutputDebugStringA("ROCK: secondary placement frame trace failed\n");
            }
        }

        frik_visual_authority::Hand frikHand(bool left) noexcept
        {
            return left ? frik_visual_authority::Hand::Left : frik_visual_authority::Hand::Right;
        }

        void clearClaims() noexcept
        {
            if (!s_state.claimed) return;
            (void)frik_visual_authority::clearHandWorld(kTag, frikHand(s_state.claimedLeft));
            (void)frik_visual_authority::clearHandPose(kTag, frikHand(s_state.claimedLeft));
            (void)frik_visual_authority::blockOffHandWeaponGripping(kTag, false);
            s_state.claimed = false;
        }

        void restoreBody(const native_hand_equipment::EquippedView& view) noexcept
        {
            // A removed model is the native teardown owner's responsibility.
            // Restore only a reacquired, still-current model, never follow an
            // address retained from the previous equipment or skeleton.
            if (view.weapon && view.weapon->formID == s_state.form && view.data == s_state.data &&
                reinterpret_cast<std::uintptr_t>(view.instance) == s_state.instance && view.bodyRoot &&
                reinterpret_cast<std::uintptr_t>(view.bodyRoot) == s_state.hiddenBody)
                equipped_weapon_visual_state::setLocallyVisible(view.bodyRoot, s_state.bodyWasVisible);
            s_state.hiddenBody = 0;
        }

        void hideBody(const native_hand_equipment::EquippedView& view) noexcept
        {
            if (!matches(view) || !s_state.ready || !s_state.geometryReady || !view.bodyRoot ||
                view.bodyRoot == view.root) return;
            const auto identity = reinterpret_cast<std::uintptr_t>(view.bodyRoot);
            if (s_state.hiddenBody != identity) {
                restoreBody(view);
                s_state.hiddenBody = identity;
                s_state.bodyWasVisible = equipped_weapon_visual_state::isLocallyVisible(view.bodyRoot);
            }
            // FRIK hides the primary body gun only. Own this exact secondary
            // model's cull state while the complete first-person gun is placed.
            // Use the native setter so replacement geometry caches are dirtied.
            equipped_weapon_visual_state::setLocallyVisible(view.bodyRoot, false);
        }

        void release() noexcept
        {
            native_secondary_actions::suspend();
            clearClaims();
            const auto view = native_hand_equipment::equippedViews()[1];
            restoreBody(view);
            // Only restore the still-equipped object that we changed. A removed
            // biped node may already be destroyed or reused by the engine.
            if (auto* attachment = currentAttachment(view); s_state.ownsLocal && attachment) {
                attachment->local = s_state.originalLocal;
                f4vr::updateTransformsDown(attachment, true);
            }
            s_state = {};
        }

        void waitFor(const native_hand_equipment::EquippedView& view, const char* reason)
        {
            native_secondary_actions::suspend();
            s_state.ready = false;
            s_state.traceFrame = false;
            clearClaims();
            restoreBody(view);
            if (auto* attachment = currentAttachment(view); s_state.ownsLocal && attachment) {
                attachment->local = s_state.originalLocal;
                f4vr::updateTransformsDown(attachment, true);
                s_state.ownsLocal = false;
            }
            if (s_state.reason == reason) return;
            s_state.reason = reason;
            ROCK_LOG_INFO(Weapon, "AKIMBO secondary presentation waiting weapon={:08X} root={:#x} reason={}",
                view.weapon ? view.weapon->formID : 0, reinterpret_cast<std::uintptr_t>(view.root), reason);
        }

        bool writeTarget(bool nativePass) noexcept
        {
            if (!s_state.ready || s_state.frame != runtime_state::currentFrame().frameIndex) return false;
            const auto view = native_hand_equipment::equippedViews()[1];
            auto* attachment = currentAttachment(view);
            if (!attachment || !attachment->parent || !usable(attachment->parent->world)) return false;
            if (nativePass && usable(attachment->world)) {
                const auto difference = attachment->world.translate - s_state.targetWorld.translate;
                s_state.maxOverwriteDistance = (std::max)(s_state.maxOverwriteDistance,
                    std::sqrt(difference.x * difference.x + difference.y * difference.y + difference.z * difference.z));
                float trace = 0;
                for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
                    trace += attachment->world.rotate.entry[i][j] * s_state.targetWorld.rotate.entry[i][j];
                s_state.maxOverwriteAngle = (std::max)(s_state.maxOverwriteAngle,
                    std::acos(std::clamp((trace - 1.0f) * 0.5f, -1.0f, 1.0f)) * 57.2957795f);
                ++s_state.nativeRepairs;
            }
            // Native animation/OMOD output owns descendants' evaluated worlds.
            // Rebuilding them from locals discards that output. Use the same
            // rigid transport as the primary equipped-weapon presentation.
            const auto previousWorld=attachment->world;
            const bool moved = weapon_scene_presentation::moveWeaponPresentationRigidly(attachment, s_state.targetWorld);
            if (moved) {
                native_secondary_actions::carryPresentation(previousWorld,s_state.targetWorld);
                native_secondary_actions::present(attachment);
                hideBody(view);
            }
            return moved;
        }
    }

    void beginFrame() noexcept
    {
        DWORD absent = 0;
        (void)s_thread.compare_exchange_strong(absent, GetCurrentThreadId(), std::memory_order_acq_rel);
        if (onFrameThread()) {
            s_state.ready = false;
            s_updatedThisFrame = false;
        }
    }

    void update() noexcept
    {
        if (!onFrameThread()) return;
        s_updatedThisFrame = true;
        native_equipped_model_lifetime::updateVisibility();
        try {
            const auto& frame = runtime_state::currentFrame();
            const auto view = native_hand_equipment::equippedViews()[1];
            if (!view.weapon || !frame.visualAuthorityAvailable || !frame.localSkeletonReady ||
                frame.localMenuBlocking || frame.compatibilityConfigBlocking || !frame.weaponDrawn) {
                release();
                return;
            }
            if (!matches(view)) {
                release();
                s_state.form = view.weapon->formID;
                s_state.root = reinterpret_cast<std::uintptr_t>(view.root);
                s_state.instance = reinterpret_cast<std::uintptr_t>(view.instance);
                s_state.data = view.data;
            }
            if (!currentAttachment(view)) {
                auto* attachment = native_hand_equipment::resolveAttachment(1);
                if (attachment && attachment != native_hand_equipment::resolveAttachment(0))
                    s_state.attachment = reinterpret_cast<std::uintptr_t>(attachment);
            }
            if ((!s_state.geometryChecked && view.root) ||
                reinterpret_cast<std::uintptr_t>(view.bodyRoot) != s_state.observedBody) {
                const auto first = native_weapon_model_geometry::inspect(view.root);
                const auto body = native_weapon_model_geometry::inspect(view.bodyRoot);
                s_state.geometryChecked = view.root != nullptr;
                s_state.geometryReady = first.complete && first.shapes > 0;
                s_state.observedBody = reinterpret_cast<std::uintptr_t>(view.bodyRoot);
                ROCK_LOG_INFO(Weapon,
                    "AKIMBO secondary model views weapon={:08X} instance={:#x} data={:#x} firstRoot={:#x} firstObjects={} firstShapes={} firstComplete={} firstCulled={} bodyRoot={:#x} bodyObjects={} bodyShapes={} bodyComplete={} bodyCulled={}",
                    s_state.form, s_state.instance, s_state.data, s_state.root, first.objects, first.shapes, first.complete,
                    view.root ? view.root->GetAppCulled() : true, s_state.observedBody,
                    body.objects, body.shapes, body.complete, view.bodyRoot ? view.bodyRoot->GetAppCulled() : true);
            }
            if (!s_state.geometryReady) {
                waitFor(view, "native-first-person-geometry");
                return;
            }
            auto* attachment = currentAttachment(view);
            if (!attachment || !attachment->parent || !usable(attachment->world) || !usable(attachment->local)) {
                waitFor(view, "native-equipped-attachment");
                return;
            }
            if (!s_state.ownsLocal) {
                s_state.originalLocal = attachment->local;
                s_state.modelScale = attachment->world.scale;
                s_state.ownsLocal = true;
            }

            // This is an inventory-owned native model. The existing off-screen
            // idle sampler supplies animation data only; it never fires or owns
            // a loose weapon, and retains the exact native item instance itself.
            native_idle_grip_preharvest::observeInventoryDisplay(view.weapon, attachment, view.instance);
            // The equip slot follows the weapon: index 1 is presented in the
            // hand that holds it. A hand change moves the FRIK claims.
            const bool left = native_hand_equipment::secondaryHandIsLeft();
            if (s_state.claimed && s_state.claimedLeft != left) clearClaims();
            s_state.left = left;
            const bool powerArmor = f4vr::isInPowerArmor();
            const auto authored = authored_weapon_grip_library::find(view.weapon, attachment, powerArmor);
            const auto* handOverride = holding_grips::find(view.weapon, attachment, left, view.data, powerArmor);
            RE::NiTransform seat{};
            authored_weapon_grip_library::FiringFingerPose fingers{};
            if (handOverride) {
                seat = handOverride->wrist;
                fingers = handOverride->fingers;
            } else {
                if (!authored.found || !authored_weapon_grip_library::isNativeIdleAuthority(authored.source) ||
                    !authored.rightFiringFingerPose.complete()) {
                    waitFor(view, "own-authored-firing-grip");
                    return;
                }
                if (left) {
                    seat = holding_grip_math::toNative(holding_grip_math::mirroredHandSeat(
                        holding_grip_math::fromNative(authored.rightHandWeaponLocal)));
                    if (!hand_finger_mirror_math::mirrorFingerLocalsAcrossHands<RE::NiTransform>(
                            authored.rightFiringFingerPose.localTransforms, fingers.localTransforms)) {
                        waitFor(view, "finger-pose");
                        return;
                    }
                    fingers.enabledMask = authored.rightFiringFingerPose.enabledMask;
                } else {
                    // The right hand uses the native idle seat as captured.
                    seat = authored.rightHandWeaponLocal;
                    fingers = authored.rightFiringFingerPose;
                }
            }
            if (left) seat = weapon_grip_calibration::shiftedHand(seat, g_rockConfig.rockLeftFiringGripOffsetGameUnits, true);
            RE::NiTransform physicalHand{};
            const auto* nodes = f4vr::getPlayerNodes();
            auto* wand = nodes ? (left ? nodes->SecondaryWandNode : nodes->primaryWandNode) : nullptr;
            if (!usable(seat) || !wand ||
                !frik_hand_world_authority::hasCalibratedRawHandFrame(left) ||
                !frik_hand_world_authority::tryGetRawHandWorld(left, physicalHand) || !usable(physicalHand)) {
                waitFor(view, "tracked-hand");
                return;
            }
            RE::NiTransform target{}, hand{};
            const auto palm = authoredHandspaceToRawHandspaceForHand(computeGrabLegacyPalmPivotAHandspacePosition(left), left);
            if (!native_secondary_pose::solve(wand->world, physicalHand, seat,
                    palm, s_state.modelScale, target, hand)) {
                waitFor(view, "invalid-placement");
                return;
            }
            native_secondary_actions::applyRecoil(target,hand,wand->world);
            frik_visual_authority::FingerLocalTransformOverride fingerOverride{};
            fingerOverride.enabledMask = fingers.enabledMask;
            std::copy(fingers.localTransforms.begin(), fingers.localTransforms.end(), fingerOverride.localTransforms);
            // Mark before publication so a partial provider failure is unwound.
            s_state.claimed = true;
            s_state.claimedLeft = left;
            if (!frik_visual_authority::setAuthoredFingerPose(kTag, frikHand(left), fingerOverride, kPriority)) {
                waitFor(view, "authored-finger-provider");
                return;
            }
            if (!frik_visual_authority::blockOffHandWeaponGripping(kTag, true)) {
                waitFor(view, "offhand-grip-provider");
                return;
            }
            if (!frik_visual_authority::publishHandWorld(kTag, frikHand(left), hand, kPriority)) {
                waitFor(view, "hand-world-provider");
                return;
            }
            s_state.targetWorld = target;
            s_state.controllerWorld = wand->world;
            s_state.frame = frame.frameIndex;
            s_state.ready = true;
            s_state.traceFrame = s_state.samples == 0 && s_state.tracedFrames < 4;
            if (s_state.traceFrame) ++s_state.tracedFrames;
            s_state.traceNativePass = 0;
            tracePlacement("solve-before");
            if (s_state.traceFrame) {
                traceTransform("solve-input", "wand-sampled", s_state.controllerWorld);
                traceTransform("solve-input", "physical-hand", physicalHand);
                traceTransform("solve-input", "authored-hand-target", hand);
            }
            if (!writeTarget(false)) {
                waitFor(view, "model-parent-changed");
                return;
            }
            tracePlacement("solve-after");
            native_secondary_actions::update(attachment,frame.deltaSeconds);
            if (s_state.reason != "ready") {
                s_state.reason = "ready";
                ROCK_LOG_INFO(Weapon,
                    "AKIMBO secondary presentation ready weapon={:08X} hand={} model={:#x} attachment={:#x} attachmentName='{}' instance={:#x} data={:#x} grip={} capture={} handInWeapon=({:.3f},{:.3f},{:.3f})",
                    s_state.form, left ? "left" : "right", s_state.root, s_state.attachment, attachment->name.c_str() ? attachment->name.c_str() : "<unnamed>", s_state.instance, s_state.data,
                    handOverride ? "authored-hand-override" : left ? "own-native-idle-mirrored" : "own-native-idle", authored.captureSequence,
                    seat.translate.x, seat.translate.y, seat.translate.z);
            }
            if (++s_state.samples == 120) {
                ROCK_LOG_INFO(Weapon,
                    "AKIMBO secondary presentation continuity weapon={:08X} frames={} nativePasses={} maxNativeOverwriteGame={:.4f} maxNativeOverwriteDegrees={:.3f} reference=controller-relative",
                    s_state.form, s_state.samples, s_state.nativeRepairs, s_state.maxOverwriteDistance, s_state.maxOverwriteAngle);
                s_state.samples = s_state.nativeRepairs = 0;
                s_state.maxOverwriteDistance = s_state.maxOverwriteAngle = 0;
            }
        } catch (...) {
            release();
            OutputDebugStringA("ROCK: secondary equipped presentation failed\n");
        }
    }

    void afterWeaponPosition() noexcept
    {
        if (!onFrameThread()) return;
        tracePlacement("frik-weapon-before");
        (void)writeTarget(false);
        tracePlacement("frik-weapon-after");
    }

    void afterWorldFinal() noexcept
    {
        if (onFrameThread()) {
            if (s_state.ready) hideBody(native_hand_equipment::equippedViews()[1]);
            if (s_state.ready) native_secondary_actions::present(currentAttachment(native_hand_equipment::equippedViews()[1]));
            tracePlacement("world-final");
            traceGeometry("world-final");
        }
    }

    bool beforeNativeArm(const RE::PlayerCharacter* player, const RE::NiNode* weapon, const RE::NiNode* offset) noexcept
    {
        if (!onFrameThread() || player != f4vr::getPlayer() || !s_state.ready) return false;
        const auto* nodes = f4vr::getPlayerNodes();
        const bool secondaryPass = nodes && weapon && offset &&
            offset == nodes->SecondaryMeleeWeaponOffsetNode2;
        if (s_state.traceFrame) {
            ++s_state.traceNativePass;
            tracePlacement("native-entry");
            try {
                traceNode("native-entry", "native-input-weapon", weapon);
                traceNode("native-entry", "native-input-offset", offset);
                ROCK_LOG_INFO(Weapon, "AKIMBO secondary native pass frame={} pass={} selected={} inputWeapon={:#x} inputOffset={:#x}",
                    s_state.frame, s_state.traceNativePass, secondaryPass,
                    reinterpret_cast<std::uintptr_t>(weapon), reinterpret_cast<std::uintptr_t>(offset));
            } catch (...) {
                OutputDebugStringA("ROCK: secondary native input trace failed\n");
            }
        }
        return secondaryPass;
    }

    void afterNativeArm(const RE::PlayerCharacter* player, bool secondaryPass) noexcept
    {
        if (!onFrameThread() || player != f4vr::getPlayer()) return;
        if (!secondaryPass) {
            // The primary arm pass leaves the left subtree near the origin.
            // A world target there becomes a ~79,000-unit model-local offset
            // in the captured run. Let the left native pass place it first.
            tracePlacement("native-unrelated-no-write");
            return;
        }
        tracePlacement("native-exit-before-repair");
        (void)writeTarget(true);
        tracePlacement("native-exit-after-repair");
        traceGeometry("native-exit-after-repair");
    }

    void endFrame(bool skeletonUpdated) noexcept
    {
        if (onFrameThread() && (!skeletonUpdated || !s_updatedThisFrame)) release();
    }
}
