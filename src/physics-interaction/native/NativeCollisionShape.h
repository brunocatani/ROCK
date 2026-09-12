#pragma once

#include <cstdint>

namespace RE
{
    class NiAVObject;
    class hknpShape;
}

namespace rock::native_scene
{
    enum class CollisionShapeQueryStage : std::uint8_t
    {
        Node,
        CollisionObject,
        NativeDispatch,
        PhysicsSystem,
        Shape,
        Complete,
        Count,
    };

    struct CollisionShapeQueryResult
    {
        RE::hknpShape* shape{ nullptr };
        CollisionShapeQueryStage stage{ CollisionShapeQueryStage::Node };
    };

    // Game-thread scene inspection only. The caller must keep the scene/template
    // alive; the returned shape is borrowed for that inspection, never retained.
    [[nodiscard]] CollisionShapeQueryResult queryCollisionShape(RE::NiAVObject* node);
}
