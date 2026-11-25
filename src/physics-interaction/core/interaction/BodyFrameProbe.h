// Temporary measurement: for every body with a motion that is ROCK's own or near the right
// hand, the BODY blocks (axes, W centre of mass), the MOTION pose and the packed quaternion
// at +0x80, so the two frames can be compared.
#pragma once

#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/native/havok/HavokRuntime.h"
#include "physics-interaction/PhysicsLog.h"

#include <array>
#include <chrono>
#include <cstring>

namespace rock::body_frame_probe
{
    struct Sample
    {
        bool valid = false;
        std::array<float, 16> body{};
        std::array<std::uint16_t, 4> packed{};
        RE::NiPoint3 motionPosition{};
        std::array<float, 4> motionOrientation{};
        std::uint32_t layer = 0;
        std::uint32_t motionIndex = 0;
    };

    inline Sample read(RE::hknpWorld* world, std::uint32_t id)
    {
        Sample out{};
        const auto* body = havok_runtime::getBody(world, RE::hknpBodyId{ id });
        if (!body || !body_frame::hasUsableMotionIndex(body->motionIndex))
            return out;
        const auto* motion = havok_runtime::getMotion(world, body->motionIndex);
        if (!motion)
            return out;
        std::memcpy(out.body.data(), body, sizeof(out.body));
        std::memcpy(out.packed.data(), reinterpret_cast<const std::uint8_t*>(body) + 0x80, sizeof(out.packed));
        out.motionPosition = { motion->position.x, motion->position.y, motion->position.z };
        out.motionOrientation = { motion->orientation.x, motion->orientation.y, motion->orientation.z,
            motion->orientation.w };
        out.layer = body->collisionFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK;
        out.motionIndex = body->motionIndex;
        out.valid = true;
        return out;
    }

    inline void log(const char* stage, RE::hknpWorld* world, const RE::NiPoint3* nearGame, std::uint32_t rightProxy,
        std::uint32_t leftProxy, std::chrono::steady_clock::time_point& next)
    {
        const auto now = std::chrono::steady_clock::now();
        if (!world || now < next)
            return;
        next = now + std::chrono::milliseconds(1500);
        const float scale = physics_scale::havokToGame();
        for (std::uint32_t id = 0; havok_runtime::bodySlotLooksReadable(world, RE::hknpBodyId{ id }); ++id) {
            const auto s = read(world, id);
            if (!s.valid)
                continue;
            const bool proxy = id == rightProxy || id == leftProxy;
            const bool own = proxy || collision_layer_policy::isRockGeneratedColliderLayer(s.layer);
            if (!own) {
                if (!nearGame)
                    continue;
                const RE::NiPoint3 at{ s.body[12] * scale, s.body[13] * scale, s.body[14] * scale };
                if ((at - *nearGame).Length() > 200.0f)
                    continue;
            }
            ROCK_LOG_INFO(Physics,
                "FRAMEPROBE stage={} id={} layer={} own={} motion={} B=({:.5f},{:.5f},{:.5f},{:.5f};{:.5f},{:.5f},{:.5f},"
                "{:.5f};{:.5f},{:.5f},{:.5f},{:.5f}) P=({:.5f},{:.5f},{:.5f}) M=({:.5f},{:.5f},{:.5f}) "
                "Q=({:.5f},{:.5f},{:.5f},{:.5f}) K=({},{},{},{}) scale={:.3f}",
                stage, id, s.layer, proxy ? "proxy" : own ? "yes" : "no", s.motionIndex, s.body[0], s.body[1], s.body[2],
                s.body[3], s.body[4], s.body[5], s.body[6], s.body[7], s.body[8], s.body[9], s.body[10], s.body[11],
                s.body[12], s.body[13], s.body[14], s.motionPosition.x, s.motionPosition.y, s.motionPosition.z,
                s.motionOrientation[0], s.motionOrientation[1], s.motionOrientation[2], s.motionOrientation[3],
                s.packed[0], s.packed[1], s.packed[2], s.packed[3], scale);
        }
    }
}
