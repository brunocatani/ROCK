// Exercise the production service boundary with a completed-pose owner and
// a separately advancing control snapshot. No live engine is needed.
#include "api/ProviderStatePolicy.h"
#include <ROCK/Hands.h>
#include <algorithm>
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
        api::hands::PresentedHandPose captured{};
        RockProviderFrameSnapshot metadata{};
        bool isInitialized() const { return true; }
        bool queryProviderPresentedHandPose(api::Hand hand, api::hands::PresentedHandPose& out, RockProviderFrameSnapshot* frame = nullptr) {
            if (!captured.frameIndex || hand != captured.hand) return false;
            out = captured;
            if (frame) *frame = metadata;
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
    using rock::api::Hand;
    using rock::api::Status;
    using Flag = rock::api::hands::HandFrameFlag;
    s_lastSnapshot.frameIndex = 10;
    s_lastSnapshot.providerReady = 1;
    rock::api::hands::PresentedHandPose pose{};
    rock::api::hands::HandFrame frame{};
    assert(apiGetPresentedHandPose(1, Hand::Right, pose) == Status::TargetUnavailable);
    assert(apiGetPresentedHandFrame(Hand::Right, frame) == Status::NotReady);

    instance.captured.hand = Hand::Right;
    instance.captured.frameIndex = 10;
    instance.captured.presentationSequence = 10;
    instance.captured.worldGeneration = 2;
    instance.captured.skeletonGeneration = 3;
    instance.captured.providerGeneration = 4;
    instance.captured.handWorld.translate[0] = 100;
    instance.captured.fingerLocalTransformMask = 1;
    instance.captured.fingerLocalTransforms[0].translate[0] = 5;
    instance.metadata.stateSequence = 42;
    instance.metadata.collisionGeneration = 7;
    // BeforeRock/AfterRock/Complete have advanced to 11; the completed pose
    // remains 10 until world final. Neither service may relabel it as 11.
    s_lastSnapshot.frameIndex = 11;
    s_lastSnapshot.stateSequence = 43;
    s_lastSnapshot.collisionGeneration = 8;
    assert(apiGetPresentedHandPose(1, Hand::Right, pose) == Status::Ok);
    assert(pose.frameIndex == 10 && pose.presentationSequence == 10 && pose.handWorld.translate[0] == 100);
    assert(pose.fingerLocalTransforms[0].translate[0] == 5);
    assert(apiGetPresentedHandFrame(Hand::Right, frame) == Status::Ok);
    assert(frame.sample.frameIndex == 10 && frame.sample.skeletonGeneration == 3 && frame.transform.translate[0] == 100);
    assert(frame.sample.publicationSequence == 42 && frame.sample.collisionGeneration == 7);
    assert((frame.flags & static_cast<std::uint32_t>(Flag::PresentedVisual)) != 0);
    assert((frame.flags & static_cast<std::uint32_t>(Flag::Left)) == 0);

    instance.captured.frameIndex = instance.captured.presentationSequence = 11;
    instance.captured.handWorld.translate[0] = 200;
    instance.metadata = s_lastSnapshot;
    assert(apiGetPresentedHandPose(1, Hand::Right, pose) == Status::Ok);
    assert(pose.frameIndex == 11 && pose.presentationSequence == 11 && pose.handWorld.translate[0] == 200);
    frame = {};
    assert(apiGetPresentedHandFrame(Hand::Right, frame) == Status::Ok);
    assert(frame.sample.frameIndex == 11 && frame.sample.publicationSequence == 43 && frame.sample.collisionGeneration == 8);
    frame = {};
    assert(apiGetPresentedHandFrame(Hand::Left, frame) == Status::NotReady && frame.flags == 0);
    instance.captured.handWorld.scale = std::nanf("");
    assert(apiGetPresentedHandFrame(Hand::Right, frame) == Status::NotReady && frame.flags == 0);
    instance.captured.handWorld.scale = 1.0f;
    ownerThread = false;
    assert(apiGetPresentedHandFrame(Hand::Right, frame) == Status::WrongThread && frame.flags == 0);
    ownerThread = true;
    rock::frik_visual_authority::ready = false;
    assert(apiGetPresentedHandFrame(Hand::Right, frame) == Status::NotReady);
    assert(apiGetPresentedHandPose(1, Hand::Right, pose) == Status::NotReady);
    assert(apiGetPresentedHandPose(1, Hand::None, pose) == Status::HandUnavailable);
}
