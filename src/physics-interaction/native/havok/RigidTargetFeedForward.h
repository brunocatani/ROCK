// Rigid target feed-forward for a dynamic body held by a grab constraint to a
// keyframed authority (the equipped-weapon body).
//
// The constraint holds such a body at its grip pivot while its mass sits
// elsewhere. Every change of the authority's velocity reached the body as an
// impulse at that pivot: through the centre of mass it only moved the body,
// across it it also turned it, so the far end swung back and recovered on
// every uneven frame while the grip stayed put (stick strafing, walking
// sideways in the room, a sideways hand move). The presentation shows the body
// pose, so the swing was on screen.
//
// Each physics substep advance() turns the authority's own commanded motion
// since the previous substep into the rigid velocity it implies at the body
// (the engine's hard-keyframe velocity for the body moved by that motion) and
// adds the change of that velocity to the body. In free space the body then
// moves rigidly with its authority and the constraint carries nothing; in
// contact the caller withholds the feed, the history still advances, and the
// constraint and the solver keep the contact response as before.
#pragma once

#include "RE/Havok/hknpBodyId.h"
#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

#include <cstdint>

namespace RE
{
    class hknpWorld;
}

namespace rock::rigid_target_feed_forward
{
    // Physics-thread state of one fed body. Reset when the body is created,
    // placed or re-pivoted, or when its authority is replaced.
    struct State
    {
        RE::NiTransform previousTarget{};
        RE::NiPoint3 previousLinearHavok{};
        RE::NiPoint3 previousAngular{};
        bool hasPreviousTarget = false;
    };

    inline void reset(State& state) { state = {}; }

    // The authority was placed rather than moved: compare the next substep with
    // this target, keeping the body's velocity history.
    void reseed(State& state, const RE::NiTransform& authorityTarget);

    enum class Outcome : std::uint8_t
    {
        // First target after a reset: nothing to compare against yet.
        Seeded,
        // The velocity change was added to the body.
        Fed,
        // History advanced, the body was left to its constraint.
        Withheld,
        // The body or the native velocity could not be read or written.
        Failed,
    };

    // authorityTarget: the authority body's commanded world target for this
    // substep, in generated-body storage (column axes), as the drive passes it
    // to the engine. feedAllowed false (contact, an extension owning the body,
    // a placement this substep) withholds the feed. A motion past the given
    // limits is a jump, not motion: the feed is withheld and the velocity
    // history is kept, so the following substeps add no spurious change.
    Outcome advance(RE::hknpWorld* world, RE::hknpBodyId bodyId, State& state, const RE::NiTransform& authorityTarget,
        float driveDeltaSeconds, bool feedAllowed, float maxLinearVelocityHavok, float maxAngularVelocityRadians);
}
