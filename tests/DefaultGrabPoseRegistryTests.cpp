#include "physics-interaction/grab/DefaultGrabPoseRegistry.h"
#include <cassert>
#include <limits>
int main()
{
    using namespace rock;
    using api::Status;
    using Registry = saved_grab_offset::DefaultGrabPoseRegistry;
    Registry registry;
    Registry::Request request;
    request.baseFormId = 100;
    request.left.flags = 7;
    request.right.flags = 1;
    request.left.translateGame[0] = -5;
    request.right.translateGame[0] = 8;
    request.left.fingerValues[0] = 1.01219594f;
    request.left.fingerJointValues[2] = 1.01219594f;
    assert(registry.set(1, request) == Status::Ok);
    saved_grab_offset::HandOffset left, right;
    assert(registry.load(100, true, left) && registry.load(100, false, right));
    assert(left.translateGame[0] == -5 && right.translateGame[0] == 8);
    assert(left.hasFingerPose && left.hasFingerJointValues && left.fingerJointValues[2] > 1);
    assert(!right.hasFingerPose && !right.hasFingerJointValues);
    assert(registry.set(2, request) == Status::OwnerConflict);
    assert(registry.clear(2, 100) == Status::OwnerConflict);
    auto invalid = request;
    --invalid.size;
    assert(registry.set(1, invalid) == Status::InvalidSize);
    invalid = request; invalid.left.rotate[0] = -1;
    assert(registry.set(1, invalid) == Status::InvalidArgument); // reflection
    invalid = request; invalid.left.rotate[4] = 2;
    assert(registry.set(1, invalid) == Status::InvalidArgument); // scaled rotation
    invalid = request; invalid.right.translateGame[1] = std::numeric_limits<float>::quiet_NaN();
    assert(registry.set(1, invalid) == Status::InvalidArgument);
    invalid = request; invalid.left.fingerValues[1] = 1.01f;
    assert(registry.set(1, invalid) == Status::InvalidArgument);
    invalid = request; invalid.left.fingerJointValues[3] = -0.01f;
    assert(registry.set(1, invalid) == Status::InvalidArgument);
    invalid = request; invalid.left.flags = 5;
    assert(registry.set(1, invalid) == Status::InvalidArgument);
    invalid = request; invalid.left.flags = 8;
    assert(registry.set(1, invalid) == Status::InvalidArgument);
    invalid = request; invalid.left.flags = invalid.right.flags = 0;
    assert(registry.set(1, invalid) == Status::InvalidArgument);
    assert(registry.load(100, true, left) && left.translateGame[0] == -5); // rejected updates are atomic
    request.left.flags = 0;
    assert(registry.set(1, request) == Status::Ok);
    assert(!registry.load(100, true, left) && registry.load(100, false, right));
    registry.clearOwner(2);
    assert(registry.load(100, false, right));
    registry.clearOwner(1);
    assert(!registry.load(100, false, right));
    assert(registry.set(2, request) == Status::Ok); // revoked owner's claim is reusable
    assert(registry.clear(2, 100) == Status::Ok && registry.clear(2, 100) == Status::Ok);
    for (unsigned i = 0; i < Registry::capacity; ++i) {
        request.baseFormId = 100 + i;
        assert(registry.set(i % 2 + 1, request) == Status::Ok);
    }
    request.baseFormId = 1000;
    assert(registry.set(3, request) == Status::CapacityFull);
    registry.clearOwner(1);
    assert(registry.set(3, request) == Status::Ok);
    assert(registry.load(101, false, right)); // another owner's entries survive cleanup
}
