#include "physics-interaction/native/player/NativeWorldUiAnchor.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/native/scene/NativeNiNodeFactory.h"

namespace rock::native_world_ui_anchor
{
    namespace
    {
        // At an object a model keeps one world size: this multiple of the
        // size it has on the hand, whatever the player's distance.
        constexpr float kObjectScale = 2.0f;
        // The model stands at most this fraction of the way from the
        // reference's centre to the player, so a large bound cannot bring it
        // into the player's face.
        constexpr float kMaxApproachFraction = 0.5f;

        RE::NiPoint3 normalized(const RE::NiPoint3& v)
        {
            const float length = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
            return length > 0.0001f ? RE::NiPoint3{ v.x / length, v.y / length, v.z / length } : RE::NiPoint3{};
        }

        RE::NiPoint3 cross(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
        }

        RE::NiPointer<RE::TESObjectREFR> resolve(const std::uint32_t handleValue)
        {
            if (!handleValue) {
                return {};
            }
            RE::ObjectRefHandle handle{};
            static_assert(sizeof(handle) == sizeof(handleValue));
            std::memcpy(&handle, &handleValue, sizeof(handle));
            return handle.get();
        }

        RE::NiPoint3 anchorCenter(RE::TESObjectREFR& ref, float& radius)
        {
            const auto* root = ref.Get3D();
            const bool bounded = root && root->worldBound.fRadius > 0.0f;
            radius = bounded ? root->worldBound.fRadius : 0.0f;
            return bounded ? root->worldBound.center : ref.data.location;
        }
    }

    std::optional<RE::NiTransform> uprightPoseAt(const std::uint32_t handleValue, const RE::NiPoint3& eye,
        const RE::NiPoint3& eyeRight, const FaceFrame& face, const float handScale, const RE::NiPoint3& faceCenter)
    {
        const auto ref = resolve(handleValue);
        if (!ref) {
            return std::nullopt;
        }
        float radius = 0.0f;
        const RE::NiPoint3 center = anchorCenter(*ref, radius);

        RE::NiPoint3 toEye{ eye.x - center.x, eye.y - center.y, 0.0f };
        const float horizontal = std::sqrt(toEye.x * toEye.x + toEye.y * toEye.y);
        const RE::NiPoint3 kWorldUp{ 0.0f, 0.0f, 1.0f };
        if (horizontal > 0.01f) {
            toEye = { toEye.x / horizontal, toEye.y / horizontal, 0.0f };
        } else {
            // Eye straight above the anchor: face the way the eye looks.
            toEye = normalized(cross(RE::NiPoint3{ eyeRight.x, eyeRight.y, 0.0f }, kWorldUp));
            if (toEye.x == 0.0f && toEye.y == 0.0f) {
                return std::nullopt;
            }
        }
        const float approach = (std::min)(radius, horizontal * kMaxApproachFraction);
        const RE::NiPoint3 position{ center.x + toEye.x * approach, center.y + toEye.y * approach, center.z };
        // Upright: the face turns about the vertical only; right is as the viewer sees it.
        const RE::NiPoint3 right = cross(kWorldUp, toEye);

        // Each root-local axis maps to the world through the face frame:
        // row i = right_i * worldRight + up_i * worldUp + toViewer_i * worldToEye.
        RE::NiTransform pose = transform_math::makeIdentityTransform<RE::NiTransform>();
        for (std::size_t row = 0; row < 3; ++row) {
            const float r = face.right[row];
            const float u = face.up[row];
            const float v = face.toViewer[row];
            pose.rotate.entry[row][0] = r * right.x + u * kWorldUp.x + v * toEye.x;
            pose.rotate.entry[row][1] = r * right.y + u * kWorldUp.y + v * toEye.y;
            pose.rotate.entry[row][2] = r * right.z + u * kWorldUp.z + v * toEye.z;
        }
        pose.scale = handScale * kObjectScale;
        // The face's centre, not the root's origin, stands at the anchor.
        const float centerLocal[3]{ faceCenter.x, faceCenter.y, faceCenter.z };
        RE::NiPoint3 offset{};
        for (int row = 0; row < 3; ++row) {
            offset.x += centerLocal[row] * pose.rotate.entry[row][0];
            offset.y += centerLocal[row] * pose.rotate.entry[row][1];
            offset.z += centerLocal[row] * pose.rotate.entry[row][2];
        }
        pose.translate = { position.x - offset.x * pose.scale, position.y - offset.y * pose.scale,
            position.z - offset.z * pose.scale };
        return pose;
    }

    RE::NiPoint3 boundCenterInRootSpace(const RE::NiAVObject& root)
    {
        const float scale = root.world.scale;
        if (!(root.worldBound.fRadius > 0.0f) || !(scale > 0.0001f)) {
            return {};
        }
        const RE::NiPoint3 d{ root.worldBound.center.x - root.world.translate.x,
            root.worldBound.center.y - root.world.translate.y, root.worldBound.center.z - root.world.translate.z };
        // Rows are the root's local axes in world space.
        const auto& r = root.world.rotate.entry;
        return { (r[0][0] * d.x + r[0][1] * d.y + r[0][2] * d.z) / scale,
            (r[1][0] * d.x + r[1][1] * d.y + r[1][2] * d.z) / scale,
            (r[2][0] * d.x + r[2][1] * d.y + r[2][2] * d.z) / scale };
    }

    std::optional<float> distanceToReference(const std::uint32_t handleValue, const RE::NiNode* from)
    {
        const auto ref = resolve(handleValue);
        if (!ref || !from) {
            return std::nullopt;
        }
        float radius = 0.0f;
        const RE::NiPoint3 target = anchorCenter(*ref, radius);
        const float dx = target.x - from->world.translate.x;
        const float dy = target.y - from->world.translate.y;
        const float dz = target.z - from->world.translate.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    void moveTo(RE::NiAVObject* root, RE::NiNode* to)
    {
        auto* const from = root->parent;
        if (!from || from == to) {
            return;
        }
        RE::NiPointer<RE::NiAVObject> held;
        from->DetachChild(root, held);
        if (held) {
            to->AttachChild(held.get(), true);
        }
    }

    bool attachCarrier(RE::NiPointer<RE::NiNode>& carrier, bool& failed, RE::NiNode* parent, const char* name)
    {
        if (!carrier) {
            if (failed) {
                return false;
            }
            carrier = native_scene::createEngineNiNode(1);
            if (!carrier) {
                failed = true;
                ROCK_LOG_ERROR(Hand, "World UI: the {} carrier node could not be created; that model stays on its wand",
                    name);
                return false;
            }
        }
        if (carrier->parent != parent) {
            if (auto* oldParent = carrier->parent) {
                RE::NiPointer<RE::NiAVObject> detached;
                oldParent->DetachChild(carrier.get(), detached);
            }
            parent->AttachChild(carrier.get(), true);
        }
        return true;
    }

    RE::NiTransform carrierLocalFor(const RE::NiTransform& rootWorld, const RE::NiTransform& rootLocal,
        const RE::NiTransform& parentWorld)
    {
        return transform_math::composeTransforms(transform_math::invertTransform(parentWorld),
            transform_math::composeTransforms(rootWorld, transform_math::invertTransform(rootLocal)));
    }
}
