// The devbench tool's state of hands and weapons: plain values copied on the
// frame thread, which the tool formats later on devbench's listener thread;
// and its hand action, one hand in detail, read and formatted at FrameEnd.
#include "devbench/RockDevBenchState.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/object/GrabTargetKind.h"
#include "physics-interaction/weapon/equip/HeldWeaponEquipStatePolicy.h"
#include "physics-interaction/weapon/equip/HeldWeaponTransferPolicy.h"
#include "rock_support/Fo4VrRuntime.h"

#include <format>

#include <nlohmann/json.hpp>

namespace rock
{
    namespace
    {
        using json = nlohmann::json;

        json pointJson(const RE::NiPoint3& point) { return json::array({ point.x, point.y, point.z }); }

        /**
         * A world transform: translation in game units, the rotation matrix by rows, and scale.
         */
        json transformJson(const RE::NiTransform& transform)
        {
            json rows = json::array();
            for (const auto& row : transform.rotate.entry)
                rows.push_back(json::array({ row.x, row.y, row.z }));
            return { { "pos", pointJson(transform.translate) }, { "rot", rows }, { "scale", transform.scale } };
        }

        json formJson(RE::TESObjectREFR* ref)
        {
            if (!ref)
                return nullptr;
            const char* name = ref->GetDisplayFullName();
            auto* base = ref->GetObjectReference();
            return { { "formId", std::format("{:08X}", ref->GetFormID()) },
                { "baseFormId", base ? json(std::format("{:08X}", base->GetFormID())) : json(nullptr) },
                { "name", name ? name : "" } };
        }

        const char* twoHandedStateName(TwoHandedState state) noexcept
        {
            switch (state) {
            case TwoHandedState::Inactive:
                return "Inactive";
            case TwoHandedState::Touching:
                return "Touching";
            case TwoHandedState::Gripping:
                return "Gripping";
            case TwoHandedState::PartCarry:
                return "PartCarry";
            case TwoHandedState::PrimaryOnly:
                return "PrimaryOnly";
            case TwoHandedState::SurfaceSupported:
                return "SurfaceSupported";
            }
            return "Unknown";
        }

        const char* handName(equipped_weapon_roster::Hand hand) noexcept
        {
            switch (hand) {
            case equipped_weapon_roster::Hand::Left:
                return "left";
            case equipped_weapon_roster::Hand::Right:
                return "right";
            case equipped_weapon_roster::Hand::None:
                break;
            }
            return "none";
        }

        const char* heldTransferPhaseName(held_weapon_transfer::Phase phase) noexcept
        {
            switch (phase) {
            case held_weapon_transfer::Phase::Idle:
                return "Idle";
            case held_weapon_transfer::Phase::AwaitEquip:
                return "AwaitEquip";
            case held_weapon_transfer::Phase::AwaitGrip:
                return "AwaitGrip";
            case held_weapon_transfer::Phase::Completing:
                return "Completing";
            case held_weapon_transfer::Phase::Recovering:
                return "Recovering";
            case held_weapon_transfer::Phase::Terminal:
                return "Terminal";
            }
            return "Unknown";
        }

        std::string referenceName(RE::TESObjectREFR* ref)
        {
            const char* name = ref ? ref->GetDisplayFullName() : nullptr;
            return name ? std::string(name) : std::string();
        }

        /**
         * The weapon in a native equipment index, or none.
         */
        void captureWeapon(std::uint32_t index, devbench::WeaponState& out)
        {
            const auto* item = f4vr::getEquippedWeaponItem(index);
            auto* form = item ? item->item.object : nullptr;
            auto* weapon = form && form->formType == RE::ENUM_FORM_ID::kWEAP ? form->As<RE::TESObjectWEAP>() : nullptr;
            if (!weapon)
                return;
            out.formId = weapon->GetFormID();
            out.name = std::string(RE::TESFullName::GetFullName(*weapon, false));
        }
    }

    /**
     * Fills the interaction part of the devbench state: what each hand does and holds, and which equipped weapon
     * each hand fires. The held kinds follow the test-bench telemetry's: ROCK's own grab wins over the equipped
     * weapon's grips.
     */
    void PhysicsInteraction::captureDevBenchState(devbench::RockState& state) const
    {
        const auto& runtime = runtime_state::currentFrame();
        state.interaction = true;
        state.playerAvailable = runtime.playerAvailable;
        state.weaponDrawn = runtime.weaponDrawn;
        state.menuBlocking = runtime.localMenuBlocking;
        state.inputMenuBlocking = runtime.inputMenuBlocking;
        state.loading = runtime.localLoadingMenuOpen;

        const auto roster = equippedWeaponRoster();
        for (const bool isLeft : { true, false }) {
            const Hand& hand = isLeft ? _leftHand : _rightHand;
            auto& out = state.hands[isLeft ? 0 : 1];
            out.state = handStateName(hand.getState());
            const auto grips = roster.slotZeroGrips(isLeft);
            if (hand.isHolding()) {
                out.held = hand.isHoldingLooseWeapon() ? "loose-weapon" : "object";
                if (auto* ref = hand.getHeldRef()) {
                    out.heldFormId = ref->GetFormID();
                    out.heldName = referenceName(ref);
                }
            } else if (grips.firingGripActive || roster.slotOneHolds(isLeft)) {
                out.held = "equipped-weapon";
            } else if (grips.partGripActive) {
                out.held = "weapon-part";
            }
            if (hand.hasSelection()) {
                const auto& selection = hand.getSelection();
                if (selection.refr) {
                    out.selectedFormId = selection.refr->GetFormID();
                    out.selectedName = referenceName(selection.refr);
                }
                out.selectedKind = grab_target::name(selection.targetKind);
                out.selectionFar = selection.isFarSelection;
            }
        }

        state.twoHandedGrip = twoHandedStateName(_twoHandedGrip.getState());
        const auto& facts = roster.facts();
        captureWeapon(0, state.weapons[0]);
        if (state.weapons[0].formId && facts.slot0.holdsHand && !facts.slot0.firingGripDetached)
            state.weapons[0].hand = handName(facts.slot0.firingHand);
        if (facts.slot1.equipped) {
            captureWeapon(1, state.weapons[1]);
            if (facts.slot1.holdsHand)
                state.weapons[1].hand = handName(facts.slot1.hand);
        }
    }

    /**
     * One hand for the devbench hand action, read at FrameEnd after the frame's grab work: its poses from the same
     * frame context the debug overlay draws from, its selection, and while it holds, the held object, constraint,
     * pivot and force solve, palm authority, finger curls and grab clock. Positions are world, in game units.
     */
    json PhysicsInteraction::describeHandForDevBench(bool isLeft)
    {
        const Hand& hand = isLeft ? _leftHand : _rightHand;
        json out = {
            { "hand", isLeft ? "left" : "right" },
            { "state", handStateName(hand.getState()) },
            { "holding", hand.isHolding() },
        };

        auto* bhk = getPlayerBhkWorld();
        auto* hknp = bhk ? getHknpWorld(bhk) : nullptr;
        const bool worldReady =
            bhk && hknp && bhk == _lifecycle.cachedBhkWorld && hknp == _lifecycle.cachedHknpWorld;
        out["worldReady"] = worldReady;
        if (worldReady) {
            const auto context = buildFrameContext(bhk, hknp);
            const auto& input = isLeft ? context.left : context.right;
            out["poses"] = {
                { "disabled", input.disabled },
                { "rawHand", transformJson(input.rawHandWorld) },
                { "grabAnchor", pointJson(input.grabAnchorWorld) },
                { "palmNormal", pointJson(input.palmNormalWorld) },
                { "pointing", pointJson(input.pointingWorld) },
            };
        }

        if (hand.hasSelection()) {
            const auto& selection = hand.getSelection();
            json selected = formJson(selection.refr);
            if (selected.is_null())
                selected = json::object();
            selected["kind"] = grab_target::name(selection.targetKind);
            selected["far"] = selection.isFarSelection;
            selected["distance"] = selection.distance;
            selected["bodyId"] = selection.bodyId.value;
            if (selection.hasHitPoint)
                selected["hitPoint"] = pointJson(selection.hitPointWorld);
            if (selection.hasHitNormal)
                selected["hitNormal"] = pointJson(selection.hitNormalWorld);
            if (selection.hasSelectionScore)
                selected["score"] = selection.selectionScore;
            if (selection.hasHmdConeDot)
                selected["hmdConeDot"] = selection.hmdConeDot;
            selected["forcedArrival"] = selection.forcedArrival;
            selected["pinchCloseFallback"] = selection.pinchCloseSelectionFallback;
            out["selection"] = std::move(selected);
        } else {
            out["selection"] = nullptr;
        }

        if (!hand.isHolding()) {
            out["grab"] = nullptr;
            return out;
        }

        const auto& saved = hand.getSavedObjectState();
        json heldBodies = json::array();
        for (const auto bodyId : hand.getHeldBodyIds())
            heldBodies.push_back(bodyId);
        json grab = {
            { "object", formJson(hand.getHeldRef()) },
            { "kind", grab_target::name(saved.targetKind) },
            { "looseWeapon", hand.isHoldingLooseWeapon() },
            { "liveLimb", saved.liveLimb },
            { "actorBodies", saved.actorBodies },
            { "bodyId", saved.bodyId.value },
            { "heldBodyIds", std::move(heldBodies) },
            { "inverseMass", saved.inverseMass },
            // The trace= of this grab's lines in ROCK.log.
            { "trace", hand.heldGrabIdentity() },
        };

        const auto& constraint = hand.getActiveConstraint();
        grab["constraint"] = constraint.isValid()
            ? json{ { "id", constraint.constraintId },
                  { "angularAuthority", grabAngularAuthorityName(constraint.angularAuthority) },
                  { "tau", constraint.currentTau }, { "maxForce", constraint.currentMaxForce },
                  { "targetMaxForce", constraint.targetMaxForce } }
            : json(nullptr);

        const auto clock = hand.getGrabClockTelemetry();
        grab["clock"] = { { "physicsHz", clock.physicsHz }, { "sourceIntervalSeconds", clock.sourceIntervalSeconds } };

        Hand::GrabFingerPoseSnapshot fingers{};
        grab["fingerCurls"] = hand.tryGetLiveGrabFingerPoseSnapshot(fingers)
            ? json{ { "thumb", fingers.values[0] }, { "index", fingers.values[1] }, { "middle", fingers.values[2] },
                  { "ring", fingers.values[3] }, { "pinky", fingers.values[4] } }
            : json(nullptr);

        if (worldReady) {
            RE::NiTransform root;
            grab["heldRoot"] = hand.readHeldObjectRoot(hknp, root) ? transformJson(root) : json(nullptr);

            GrabPivotDebugSnapshot pivot{};
            grab["pivot"] = hand.getGrabPivotDebugSnapshot(hknp, pivot)
                ? json{ { "handPivot", pointJson(pivot.handPivotWorld) },
                      { "objectPivot", pointJson(pivot.objectPivotWorld) },
                      { "handBody", pointJson(pivot.handBodyWorld) },
                      { "objectBody", pointJson(pivot.objectBodyWorld) },
                      { "errorGameUnits", pivot.pivotErrorGameUnits } }
                : json(nullptr);

            GrabForceTorqueDebugSnapshot solve{};
            grab["solve"] = hand.getGrabForceTorqueDebugSnapshot(hknp, solve)
                ? json{ { "acquisitionPhase", solve.acquisitionPhase },
                      { "pivotAuthoritySource", solve.pivotAuthoritySource },
                      { "grabPointMode", solve.activeGrabPointMode },
                      { "authorityFrameSource", solve.authorityFrameSource },
                      { "capturePivotAuthoritySource", solve.capturePivotAuthoritySource },
                      { "captureGrabPointMode", solve.captureGrabPointMode },
                      { "pivotErrorGameUnits", solve.pivotErrorGameUnits },
                      { "pivotTrackingErrorGameUnits", solve.pivotTrackingErrorGameUnits },
                      { "rotationErrorDegrees", solve.rotationErrorDegrees },
                      { "bodyVisualMeshLockErrorGameUnits", solve.bodyVisualMeshLockErrorGameUnits },
                      { "captureGripLocalDeltaGameUnits", solve.captureGripLocalDeltaGameUnits },
                      { "pocketDistanceGameUnits", solve.pocketDistanceGameUnits },
                      { "selectionDistanceGameUnits", solve.selectionDistanceGameUnits },
                      { "leverGameUnits", solve.leverLengthGameUnits },
                      { "positionConfidence", solve.positionConfidence },
                      { "liveBody", transformJson(solve.liveBodyWorld) },
                      { "desiredBody", transformJson(solve.desiredBodyWorld) },
                      { "targetPivot", pointJson(solve.targetPivotWorld) },
                      { "livePivot", pointJson(solve.livePivotWorld) },
                      { "meshGripPoint", pointJson(solve.meshGripPointWorld) } }
                : json(nullptr);

            GrabSupportFrameDebugSnapshot support{};
            grab["supportFrame"] = hand.getGrabSupportFrameDebugSnapshot(hknp, support)
                ? json{ { "pivot", pointJson(support.pivotWorld) },
                      { "pivotAuthoritySource", support.pivotAuthoritySource },
                      { "grabPointMode", support.activeGrabPointMode },
                      { "positionOnlyPivot", support.positionOnlyPivot },
                      { "normalTrusted", support.normalTrusted } }
                : json(nullptr);

            GrabAuthorityProxyDebugSnapshot proxy{};
            grab["palmAuthority"] = hand.getGrabAuthorityProxyDebugSnapshot(hknp, proxy)
                ? json{ { "palmBase", transformJson(proxy.palmAuthorityBaseWorld) },
                      { "proxyTarget", transformJson(proxy.proxyTargetWorld) },
                      { "localOffsetGameUnits", pointJson(proxy.localOffsetGameUnits) } }
                : json(nullptr);
        }
        out["grab"] = std::move(grab);
        return out;
    }

    /**
     * The equipped weapons for the devbench weapon action, read at FrameEnd: the native weapon state, the roster's
     * per-slot facts (which hand fires, which part grips hold, pending grip reservations), each weapon with its
     * generated collision and scope eligibility, the two-handed grip session, and the equip transition's hand-pose
     * handoff.
     */
    json PhysicsInteraction::describeWeaponsForDevBench() const
    {
        const auto& runtime = runtime_state::currentFrame();
        const auto roster = equippedWeaponRoster();
        const auto& facts = roster.facts();
        const auto partGripJson = [](const equipped_weapon_roster::PartGripFacts& grip) {
            return json{ { "active", grip.active }, { "attachOnly", grip.attachOnly } };
        };
        const auto weaponJson = [this](std::uint32_t index, const WeaponCollision& collision) {
            devbench::WeaponState weapon;
            captureWeapon(index, weapon);
            if (!weapon.formId)
                return json(nullptr);
            return json{
                { "formId", std::format("{:08X}", weapon.formId) },
                { "name", weapon.name },
                { "scopeEligible", isScopeEligible(static_cast<std::int32_t>(index)) },
                { "collision",
                    { { "bodies", collision.getWeaponBodyCount() },
                        { "observedFormId", std::format("{:08X}", collision.getCurrentObservedEquippedWeaponFormID()) },
                        { "generationKey", std::format("{:016X}", collision.getCurrentWeaponGenerationKey()) } } },
            };
        };

        const auto& grip = _twoHandedGrip;
        const auto& transition = _equipped.transition;
        return {
            { "weaponDrawn", runtime.weaponDrawn },
            { "nativeWeaponState",
                _equipped.tracedNativeWeaponState == 0xFFFFFFFFu
                    ? json(nullptr)
                    : json(held_weapon_equip_state_policy::nativeWeaponStateName(_equipped.tracedNativeWeaponState)) },
            { "inventoryEquipPending", _inventoryEquip.active() },
            { "primary", weaponJson(0, _weaponCollision) },
            { "secondary", weaponJson(1, _secondaryWeaponCollision) },
            { "roster",
                {
                    { "slot0",
                        { { "holdsHand", facts.slot0.holdsHand },
                            { "sessionFiringHand", handName(facts.slot0.sessionFiringHand) },
                            { "firingHand", handName(facts.slot0.firingHand) },
                            { "firingGripConfirmed", facts.slot0.firingGripConfirmed },
                            { "firingGripDetached", facts.slot0.firingGripDetached },
                            { "partGrips",
                                { { "left", partGripJson(facts.slot0.partGrips[1]) },
                                    { "right", partGripJson(facts.slot0.partGrips[0]) } } } } },
                    { "slot1",
                        { { "formId", std::format("{:08X}", facts.slot1.formId) },
                            { "equipped", facts.slot1.equipped }, { "holdsHand", facts.slot1.holdsHand },
                            { "hand", handName(facts.slot1.hand) } } },
                    { "pending",
                        { { "bothHands", facts.pending.bothHands },
                            { "supportHand", handName(facts.pending.supportHand) },
                            { "firingHand", handName(facts.pending.firingHand) } } },
                } },
            { "twoHandedGrip",
                {
                    { "state", twoHandedStateName(grip.getState()) },
                    { "firingHand", grip.isFiringHandLeft() ? "left" : "right" },
                    { "gripping", grip.isGripping() },
                    { "firingGripOccupied", grip.isFiringGripOccupied() },
                    { "partCarry", grip.isPartCarryActive() },
                    { "primaryOnly", grip.isPrimaryOnlyActive() },
                    { "handsFreeRetained", grip.isHandsFreeRetained() },
                    { "liveGripPresenting", grip.isLiveGripPresenting() },
                    { "visualReturn", grip.isWeaponVisualReturnActive() },
                    { "gunstockAim", grip.isGunstockAimActive() },
                    { "partGrips",
                        { { "left",
                              { { "gripping", grip.isHandPartGripping(true) },
                                  { "carry", grip.isHandPartCarryGripping(true) } } },
                            { "right",
                                { { "gripping", grip.isHandPartGripping(false) },
                                    { "carry", grip.isHandPartCarryGripping(false) } } } } },
                } },
            { "transition",
                {
                    { "heldTransferPhase", heldTransferPhaseName(transition.heldTransfer().phase) },
                    { "handPoseHandoff", transition.isHandPoseHandoffActive() },
                    { "handPoseHandoffHand",
                        transition.isHandPoseHandoffActive()
                            ? json(transition.handPoseHandoffIsLeft() ? "left" : "right")
                            : json(nullptr) },
                    { "pairedHandoff", transition.hasPairedHandPoseHandoff() },
                    { "bridgeWeaponFormId", std::format("{:08X}", transition.bridgeWeaponBaseFormID()) },
                    { "bridgeModelPresented", transition.presentedBridgeModel() != nullptr },
                } },
        };
    }
}
