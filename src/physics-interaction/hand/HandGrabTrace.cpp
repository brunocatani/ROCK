#include "physics-interaction/hand/HandGrabTrace.h"

#include "RockConfig.h"
#include "physics-interaction/grab/GrabNodeInfoMath.h"
#include "physics-interaction/native/CharacterControllerRuntime.h"
#include "physics-interaction/native/PhysicsScale.h"
#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>

namespace rock::hand_grab_detail
{
    grab_authority_source_clock::ControllerRootFrameSample samplePlayerControllerRootFrame() noexcept
    {
        const auto controller = character_controller_runtime::samplePlayerCharacterControllerPositionHavok();
        return grab_authority_source_clock::ControllerRootFrameSample{
            .positionHavok = controller.positionHavok,
            .controllerIdentity = controller.controllerIdentity,
            .controllerVtable = controller.controllerVtable,
            .physicsScaleRevision = physics_scale::revision(),
            .valid = controller.valid,
        };
    }
    
    std::uint64_t nextGrabTimelineTraceId() noexcept
    {
        static std::atomic<std::uint64_t> nextTraceId{ 1 };
        return nextTraceId.fetch_add(1, std::memory_order_relaxed);
    }
    
    bool grabTimelineTraceEnabled() noexcept
    {
        return g_rockConfig.rockDebugGrabTimelineTrace;
    }
    
    bool shouldLogGrabTimelineSequence(std::uint64_t sequence) noexcept
    {
        const auto interval = static_cast<std::uint64_t>(
            (std::max)(1, g_rockConfig.rockDebugGrabTimelineTraceIntervalFrames));
        return sequence <= 16u || (sequence % interval) == 0u;
    }
    
    const char* releaseDispositionName(GrabReleaseDisposition disposition) noexcept
    {
        switch (disposition) {
        case GrabReleaseDisposition::PhysicalDrop:
            return "physical-drop";
        case GrabReleaseDisposition::PendingInventoryTransfer:
            return "pending-inventory-transfer";
        case GrabReleaseDisposition::TransferToInventory:
            return "transfer-to-inventory";
        case GrabReleaseDisposition::PendingConsumeTransfer:
            return "pending-consume-transfer";
        case GrabReleaseDisposition::OwnershipHandoff:
            return "ownership-handoff";
        }
        return "unknown";
    }
    
    const char* nodeDebugName(const RE::NiAVObject* node) noexcept
    {
        if (!node) {
            return "(null)";
        }
    
        const char* name = node->name.c_str();
        return name ? name : "(unnamed)";
    }
    
    void logGrabNodeInfo(const char* handName,
        bool isLeft,
        const RE::NiAVObject* parentNode,
        const RE::NiAVObject* authoredGrabNode,
        const RE::NiTransform& desiredObjectWorld,
        const RE::NiTransform& handWorldTransform,
        const RE::NiPoint3& grabPivotAWorld,
        const char* grabPointMode)
    {
        if (!g_rockConfig.rockPrintGrabNodeInfo) {
            return;
        }
    
        const RE::NiTransform grabNodeLocal =
            grab_node_info_math::computeGrabNodeLocalTransformForCurrentGrab(desiredObjectWorld, handWorldTransform, grabPivotAWorld);
        const auto nifskopeEulerDegrees = grab_node_info_math::nifskopeMatrixToEulerDegrees(grabNodeLocal.rotate);
        const char* configuredNodeName = isLeft ? g_rockConfig.rockGrabNodeNameLeft.c_str() : g_rockConfig.rockGrabNodeNameRight.c_str();
    
        ROCK_LOG_INFO(Hand, "{} ROCK GRAB NODE INFO BEGIN", handName);
        ROCK_LOG_INFO(Hand, "Parent: {}", nodeDebugName(parentNode));
        ROCK_LOG_INFO(Hand, "Name: {}", configuredNodeName);
        ROCK_LOG_INFO(Hand,
            "Translation: {:.4f} {:.4f} {:.4f}",
            grabNodeLocal.translate.x,
            grabNodeLocal.translate.y,
            grabNodeLocal.translate.z);
        ROCK_LOG_INFO(Hand,
            "Rotation: {:.4f} {:.4f} {:.4f}",
            nifskopeEulerDegrees.x,
            nifskopeEulerDegrees.y,
            nifskopeEulerDegrees.z);
        ROCK_LOG_INFO(Hand,
            "Source: authoredNode={} authoredName={} pointMode={}",
            authoredGrabNode ? "yes" : "no",
            authoredGrabNode ? nodeDebugName(authoredGrabNode) : "none",
            grabPointMode ? grabPointMode : "unknown");
        ROCK_LOG_INFO(Hand, "{} ROCK GRAB NODE INFO END", handName);
    }
    
    AnchorClockRoomSample sampleAnchorClockRoom()
    {
        AnchorClockRoomSample sample{};
        if (const auto* playerNodes = f4vr::getPlayerNodes(); playerNodes && playerNodes->roomnode) {
            const auto& world = playerNodes->roomnode->world;
            sample.position = world.translate;
            sample.yawDegrees = std::atan2(world.rotate.entry[1][0], world.rotate.entry[0][0]) * 57.295779513f;
            sample.valid = true;
        }
        return sample;
    }
    
    void assignGrabClockFeedVec(float (&out)[3], const RE::NiPoint3& in) noexcept
    {
        out[0] = in.x;
        out[1] = in.y;
        out[2] = in.z;
    }
    
    void logRuntimeScaleIfChanged(bool isLeft, const char* handName, const RE::NiTransform& handWorldTransform, const RE::NiAVObject* collidableNode)
    {
        struct RuntimeScaleLogState
        {
            bool initialized = false;
            float handScale = 0.0f;
            float collidableScale = 0.0f;
            float vrScale = 0.0f;
        };
    
        static std::array<RuntimeScaleLogState, 2> s_scaleStates{};
        auto& state = s_scaleStates[isLeft ? 1 : 0];
        auto* vrScaleSetting = f4vr::getIniSetting("fVrScale:VR");
        const float vrScale = vrScaleSetting ? vrScaleSetting->GetFloat() : -1.0f;
        const float collidableScale = collidableNode ? collidableNode->world.scale : -1.0f;
        auto changedEnough = [](float a, float b) {
            return std::fabs(a - b) > 0.01f;
        };
    
        if (!state.initialized || changedEnough(state.handScale, handWorldTransform.scale) || changedEnough(state.collidableScale, collidableScale) ||
            changedEnough(state.vrScale, vrScale)) {
            ROCK_LOG_DEBUG(Hand,
                "Runtime scale {}: handScale={:.3f} collidableScale={:.3f} vrScale={:.3f} previous=({:.3f},{:.3f},{:.3f})",
                handName,
                handWorldTransform.scale,
                collidableScale,
                vrScale,
                state.handScale,
                state.collidableScale,
                state.vrScale);
            state.initialized = true;
            state.handScale = handWorldTransform.scale;
            state.collidableScale = collidableScale;
            state.vrScale = vrScale;
        }
    }
    
    
}
