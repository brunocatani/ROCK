// Exercise the production service boundary with a completed-pose owner and
// a separately advancing control snapshot. No live engine is needed.
#include "api/ProviderStatePolicy.h"
#include <cassert>
#include <cmath>
#include <mutex>

namespace rock::frik_visual_authority {
    bool ready = true;
    bool isAvailable() { return ready; }
    bool isSkeletonReadyHint() { return ready; }
}
namespace rock::provider {
    struct PhysicsInteraction {
        RockProviderPresentedHandPoseV1 captured{};
        bool isInitialized() const { return true; }
        bool queryProviderPresentedHandPoseV1(RockProviderHand hand, RockProviderPresentedHandPoseV1& out) {
            if (!captured.frameIndex || hand != captured.hand) return false;
            out = captured;
            return true;
        }
    } instance;
    struct Access { PhysicsInteraction* get() const { return &instance; } };
    struct InstanceAccess { Access borrow() const { return {}; } } s_physicsInteraction;
    std::mutex s_snapshotMutex;
    bool s_hasSnapshot = true;
    RockProviderFrameSnapshot s_lastSnapshot{};
    bool ownerThread = true;
    bool onAnimationOwnerThread() { return ownerThread; }
    bool apiIsProviderReady() { return frik_visual_authority::ready; }
    bool finiteProviderTransform(const RockProviderTransform& pose) { return std::isfinite(pose.scale); }
    template<class Output, class Query>
    RockProviderResultV1 queryPhysicsInteractionValueV1(std::uint64_t owner,
        RockProviderConsumerCapabilityV1, Output* output, Query query, bool) {
        provider_state_policy::clearQueryOutput(output);
        if (!owner || !output) return RockProviderResultV1::InvalidArgument;
        if (!ownerThread) return RockProviderResultV1::WrongThread;
        if (!apiIsProviderReady()) return RockProviderResultV1::NotReady;
        return query(instance, *output) ? RockProviderResultV1::Ok : RockProviderResultV1::TargetUnavailable;
    }
#include "api/services/HandsService.inl"
}

int main()
{
    using namespace rock::provider;
    s_lastSnapshot.frameIndex = 10;
    s_lastSnapshot.providerReady = 1;
    RockProviderPresentedHandPoseV1 pose{};
    RockProviderHandFrameV1 frame{};
    assert(apiGetPresentedHandPoseV1(1, RockProviderHand::Right, &pose) == RockProviderResultV1::TargetUnavailable);
    assert(!apiGetPresentedHandFrameV1(RockProviderHand::Right, &frame));

    instance.captured.hand = RockProviderHand::Right;
    instance.captured.frameIndex = 10;
    instance.captured.presentationSequence = 10;
    instance.captured.worldGeneration = 2;
    instance.captured.skeletonGeneration = 3;
    instance.captured.providerGeneration = 4;
    instance.captured.handWorld.translate[0] = 100;
    instance.captured.fingerLocalTransformMask = 1;
    instance.captured.fingerLocalTransforms[0].translate[0] = 5;
    // BeforeRock/AfterRock/Complete have advanced to 11; the completed pose
    // remains 10 until world final. Neither service may relabel it as 11.
    s_lastSnapshot.frameIndex = 11;
    assert(apiGetPresentedHandPoseV1(1, RockProviderHand::Right, &pose) == RockProviderResultV1::Ok);
    assert(pose.frameIndex == 10 && pose.presentationSequence == 10 && pose.handWorld.translate[0] == 100);
    assert(pose.fingerLocalTransforms[0].translate[0] == 5);
    assert(apiGetPresentedHandFrameV1(RockProviderHand::Right, &frame));
    assert(frame.frameIndex == 10 && frame.skeletonGeneration == 3 && frame.transform.translate[0] == 100);

    instance.captured.frameIndex = instance.captured.presentationSequence = 11;
    instance.captured.handWorld.translate[0] = 200;
    assert(apiGetPresentedHandPoseV1(1, RockProviderHand::Right, &pose) == RockProviderResultV1::Ok);
    assert(pose.frameIndex == 11 && pose.presentationSequence == 11 && pose.handWorld.translate[0] == 200);
    assert(!apiGetPresentedHandFrameV1(RockProviderHand::Left, &frame));
    ownerThread = false;
    assert(!apiGetPresentedHandFrameV1(RockProviderHand::Right, &frame));
    assert(frame.flags == 0);
    ownerThread = true;
    rock::frik_visual_authority::ready = false;
    assert(!apiGetPresentedHandFrameV1(RockProviderHand::Right, &frame));
    assert(apiGetPresentedHandPoseV1(1, RockProviderHand::Right, &pose) == RockProviderResultV1::NotReady);
}
