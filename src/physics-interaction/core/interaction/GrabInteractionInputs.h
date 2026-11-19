// Inputs several grab stages share: the claim owner of a hand, the far-selection
// view cone, shoulder stash detector settings and probes, held body ids and the
// packed held-impact pair, a held body's mass, and one grab button's state.
#pragma once

#include "RockConfig.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/grab/GrabEvent.h"
#include "physics-interaction/grab/GrabMassPolicy.h"
#include "physics-interaction/hand/HandSelection.h"
#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/native/havok/HavokRuntime.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/stash/ShoulderStashDetector.h"
#include "rock_support/VRControllers.h"

#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Havok/hknpWorld.h"

#include <cstdint>
#include <string_view>

namespace rock
{
    namespace physics_interaction_internal
    {
        constexpr PhysicsObjectClaimOwner claimOwnerForHand(bool isLeft)
        {
            return isLeft ? PhysicsObjectClaimOwner::LeftHand : PhysicsObjectClaimOwner::RightHand;
        }

        inline FarSelectionHmdConeGate makeFarSelectionHmdConeGate(const PhysicsFrameContext& frame)
        {
            FarSelectionHmdConeGate gate{};
            gate.enabled = selection_query_policy::kFarSelectionHmdConeEnabled;
            gate.hasHmdFrame = frame.hasHmdFrame;
            gate.hmdPositionWorld = frame.hmdPositionWorld;
            gate.hmdForwardWorld = frame.hmdForwardWorld;
            gate.minDot = selection_query_policy::farSelectionHmdConeMinDot(
                selection_query_policy::kFarSelectionHmdConeHalfAngleDegrees);
            return gate;
        }

        inline shoulder_stash::DetectorConfig makeShoulderStashDetectorConfig()
        {
            shoulder_stash::DetectorConfig config{};
            config.enabled = g_rockConfig.rockShoulderStashEnabled;
            config.useBodyZoneColliders = g_rockConfig.rockShoulderStashUseBodyZoneColliders;
            config.useHmdBackVolume = g_rockConfig.rockShoulderStashUseHmdBackVolume;
            config.enterPaddingGameUnits = g_rockConfig.rockShoulderStashEnterPaddingGameUnits;
            config.exitPaddingGameUnits = g_rockConfig.rockShoulderStashExitPaddingGameUnits;
            config.recentContactSeconds = g_rockConfig.rockShoulderStashRecentContactSeconds;
            config.sustainedContactMissSeconds = g_rockConfig.rockShoulderStashSustainedContactMissSeconds;
            config.hmdBackRightOffsetGameUnits = g_rockConfig.rockShoulderStashHmdBackRightOffsetGameUnits;
            config.hmdBackLeftOffsetGameUnits = g_rockConfig.rockShoulderStashHmdBackLeftOffsetGameUnits;
            config.hmdBackRadiusGameUnits = g_rockConfig.rockShoulderStashHmdBackRadiusGameUnits;
            config.hmdBackEnterPaddingGameUnits = g_rockConfig.rockShoulderStashHmdBackEnterPaddingGameUnits;
            config.hmdBackExitPaddingGameUnits = g_rockConfig.rockShoulderStashHmdBackExitPaddingGameUnits;
            config.hmdBackMinBehindGameUnits = g_rockConfig.rockShoulderStashHmdBackMinBehindGameUnits;
            return config;
        }

        inline shoulder_stash::Probe makeShoulderStashHmdProbe(const HandFrameInput& handInput)
        {
            shoulder_stash::Probe probe{};
            probe.pointGame = handInput.rawHandWorld.translate;
            return probe;
        }

        inline std::string_view shoulderStashItemName(RE::TESBoundObject* baseForm)
        {
            if (!baseForm) {
                return {};
            }

            return RE::TESFullName::GetFullName(*baseForm, false);
        }

        inline constexpr std::uint32_t kInvalidAtomicBodyId = 0xFFFF'FFFFu;
        inline constexpr std::uint64_t kInvalidHeldImpactPair = 0xFFFF'FFFF'FFFF'FFFFull;

        inline bool isInvalidGrabBodyId(std::uint32_t bodyId)
        {
            return bodyId == kInvalidAtomicBodyId || bodyId == ROCK_GRAB_EVENT_INVALID_BODY_ID ||
                bodyId == object_physics_body_set::INVALID_BODY_ID;
        }

        inline bool unpackHeldImpactPair(std::uint64_t packedPair, std::uint32_t& heldBodyId,
            std::uint32_t& otherBodyId)
        {
            if (packedPair == kInvalidHeldImpactPair) {
                return false;
            }
            heldBodyId = static_cast<std::uint32_t>(packedPair >> 32);
            otherBodyId = static_cast<std::uint32_t>(packedPair & 0xFFFF'FFFFu);
            return !isInvalidGrabBodyId(heldBodyId) && !isInvalidGrabBodyId(otherBodyId);
        }

        inline float readGrabEventBodyMass(RE::hknpWorld* world, std::uint32_t bodyId)
        {
            if (!world || isInvalidGrabBodyId(bodyId)) {
                return 0.0f;
            }

            auto* motion = havok_runtime::getBodyMotion(world, RE::hknpBodyId{ bodyId });
            if (!motion) {
                return 0.0f;
            }

            const auto packedInvMass = static_cast<std::int16_t>(motion->packedInverseInertia[3]);
            if (packedInvMass == 0) {
                return 0.0f;
            }
            return grab_mass_policy::massFromInverseMass(unpackBfloat16(packedInvMass));
        }

        struct GrabButtonState
        {
            bool held{ false };
            bool pressed{ false };
            bool released{ false };
            bool syntheticPressed{ false };
        };

        inline GrabButtonState readGrabButtonState(bool isLeft, int buttonId)
        {
            if (!input_remap_policy::isAllowedGrabButtonId(buttonId)) {
                return {};
            }

            const auto rawState = input_remap_runtime::consumeRawButtonState(isLeft, buttonId);
            if (rawState.available) {
                return GrabButtonState{ .held = rawState.held,
                    .pressed = rawState.pressed,
                    .released = rawState.released };
            }

            const auto vrHand = isLeft ? vrcf::Hand::Left : vrcf::Hand::Right;
            return GrabButtonState{
                .held = vrcf::VRControllers.isPressHeldDown(vrHand, buttonId),
                .pressed = vrcf::VRControllers.isPressed(vrHand, buttonId),
                .released = vrcf::VRControllers.isReleased(vrHand, buttonId),
            };
        }
    }
}
