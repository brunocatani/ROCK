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
        /*
         * A model reads at its hand size when the eye is this far from it
         * (about 60 cm, a hand held up to read). Further away it is scaled up
         * with distance so it keeps that apparent size, up to kMaxViewScale.
         * Game units.
         */
        constexpr float kHandReadingDistance = 45.0f;
        constexpr float kMaxViewScale = 4.0f;
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

    std::optional<RE::NiTransform> poseFacingEye(const std::uint32_t handleValue, const RE::NiPoint3& eye,
        const RE::NiPoint3& eyeRight, const float handScale)
    {
        const auto ref = resolve(handleValue);
        if (!ref) {
            return std::nullopt;
        }
        float radius = 0.0f;
        const RE::NiPoint3 center = anchorCenter(*ref, radius);

        RE::NiPoint3 toEye{ eye.x - center.x, eye.y - center.y, 0.0f };
        const float horizontal = std::sqrt(toEye.x * toEye.x + toEye.y * toEye.y);
        toEye = normalized(toEye);
        const float approach = (std::min)(radius, horizontal * kMaxApproachFraction);
        const RE::NiPoint3 position{ center.x + toEye.x * approach, center.y + toEye.y * approach, center.z };

        const RE::NiPoint3 view{ position.x - eye.x, position.y - eye.y, position.z - eye.z };
        const float distance = std::sqrt(view.x * view.x + view.y * view.y + view.z * view.z);
        if (distance < 1.0f) {
            return std::nullopt;
        }
        const RE::NiPoint3 forward = normalized(view);
        const RE::NiPoint3 kWorldUp{ 0.0f, 0.0f, 1.0f };
        RE::NiPoint3 right = cross(forward, kWorldUp);
        right = right.x * right.x + right.y * right.y + right.z * right.z > 0.0001f ? normalized(right) : eyeRight;
        const RE::NiPoint3 up = cross(right, forward);

        RE::NiTransform pose = transform_math::makeIdentityTransform<RE::NiTransform>();
        pose.rotate.entry[0][0] = right.x;
        pose.rotate.entry[0][1] = right.y;
        pose.rotate.entry[0][2] = right.z;
        pose.rotate.entry[1][0] = forward.x;
        pose.rotate.entry[1][1] = forward.y;
        pose.rotate.entry[1][2] = forward.z;
        pose.rotate.entry[2][0] = up.x;
        pose.rotate.entry[2][1] = up.y;
        pose.rotate.entry[2][2] = up.z;
        pose.translate = position;
        pose.scale = handScale * std::clamp(distance / kHandReadingDistance, 1.0f, kMaxViewScale);
        return pose;
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
