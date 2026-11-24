// The hkTransformf handed to native Havok calls. RE::NiTransform holds two rotation
// layouts in ROCK and nothing in its type tells them apart, so a HavokTransform is
// built only through the constructor function that names the source's layout.
#pragma once

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/native/havok/PhysicsScale.h"

#include <RE/Bethesda/bhkCharacterController.h>
#include <RE/NetImmerse/NiTransform.h>

namespace rock
{
    class HavokTransform
    {
    public:
        // Local axes are the matrix rows: scene nodes, TransformMath results and
        // body-array (BODY) readback.
        [[nodiscard]] static HavokTransform fromRowAxes(const RE::NiTransform& source)
        {
            return HavokTransform(transform_math::niStoredAxesToHknpBodyColumns(source.rotate), source.translate);
        }

        // Local axes are the matrix columns: generated collider frames, drive targets
        // and motion (MOTION) readback.
        [[nodiscard]] static HavokTransform fromColumnAxes(const RE::NiTransform& source)
        {
            return HavokTransform(transform_math::niRowsToHavokColumns(source.rotate), source.translate);
        }

        [[nodiscard]] const RE::hkTransformf& native() const noexcept { return _native; }

    private:
        HavokTransform(const RE::NiMatrix3& nativeRotation, const RE::NiPoint3& translateGame)
        {
            const float scale = physics_scale::gameToHavok();
            _native.rotation = nativeRotation;
            _native.translation =
                RE::NiPoint4(translateGame.x * scale, translateGame.y * scale, translateGame.z * scale, 0.0f);
        }

        RE::hkTransformf _native{};
    };
}
