#include "api/ProviderLeasePolicy.h"
#include "api/ProviderFrameClock.h"

#include <cassert>
#include <cstdint>
#include <limits>

int main()
{
    using namespace rock::provider_lease_policy;

    assert(clampLeaseFrames(1, 120) == 1);
    assert(clampLeaseFrames(120, 120) == 120);
    assert(clampLeaseFrames(121, 120) == 120);

    const auto oneFrameExpiry = exclusiveExpiryFrame(100, 1);
    assert(oneFrameExpiry == 101);
    assert(isActive(100, oneFrameExpiry));
    assert(!isActive(101, oneFrameExpiry));
    assert(remainingFrames(100, oneFrameExpiry) == 1);
    assert(remainingFrames(101, oneFrameExpiry) == 0);

    const auto maximumExpiry = exclusiveExpiryFrame(100, 120);
    assert(isActive(219, maximumExpiry));
    assert(!isActive(220, maximumExpiry));
    assert(remainingFrames(100, maximumExpiry) == 120);

    constexpr auto maximumFrame =
        (std::numeric_limits<std::uint64_t>::max)();
    assert(exclusiveExpiryFrame(maximumFrame - 1, 120) == maximumFrame);
    assert(isActive(maximumFrame - 1, maximumFrame));
    assert(!isActive(maximumFrame, maximumFrame));
    assert(!isActive(0, 0));

    // A default one-frame drive published by the end-of-frame callback must
    // reach the following update. Public timestamps already name that update,
    // but its lease retires only at the following provider publication.
    rock::provider::ProviderFrameClock clock;
    clock.beginFrame(100);
    assert(clock.publishFrame() == 100);
    const auto driveExpiry = exclusiveExpiryFrame(clock.leaseBoundary(), 1);
    clock.beginFrame(101);
    assert(clock.current() == 101);
    assert(isActive(clock.leaseBoundary(), driveExpiry));
    assert(clock.publishFrame() == 101);
    assert(!isActive(clock.leaseBoundary(), driveExpiry));

    // Missing provider frames and provider recreation never invent a second
    // counter or rewind timestamps; a second lifecycle publication can reuse
    // this frame identity without consuming another lease frame.
    clock.beginFrame(120);
    assert(clock.current() == 120 && clock.leaseBoundary() == 101);
    assert(clock.publishFrame() == 120);
    assert(clock.publishFrame() == 120);

    // Native graph output observes the last measured timing between game-loop
    // entries. It cannot preempt BeforeRock or consume a one-frame drive.
    using Phase=rock::api::core::AnimationPhaseV1;
    rock::provider::ProviderFrameClock phases;
    assert(phases.beginPhase(Phase::NativeGraphOutput,0)==0);
    assert(phases.current()==0 && phases.leaseBoundary()==0);
    assert(phases.beginPhase(Phase::BeforeRock,40)==40);
    assert(phases.publishFrame()==40);
    assert(phases.beginPhase(Phase::Complete,40)==40);
    const auto pendingDrive=exclusiveExpiryFrame(phases.leaseBoundary(),1);
    assert(phases.beginPhase(Phase::NativeGraphOutput,40)==40);
    assert(phases.beginPhase(Phase::NativeGraphOutput,40)==40);
    assert(phases.beginPhase(Phase::BeforeRock,41)==41);
    assert(phases.current()==41 && phases.leaseBoundary()==40);
    assert(isActive(phases.leaseBoundary(),pendingDrive));
    assert(phases.beginPhase(Phase::AfterRock,41)==41);
    assert(phases.publishFrame()==41);
    assert(!isActive(phases.leaseBoundary(),pendingDrive));
    assert(phases.beginPhase(Phase::Complete,41)==41);
    assert(phases.beginPhase(Phase::Presented,41)==41);
    assert(phases.current()==41 && phases.leaseBoundary()==41);
    // An observation does not advance the clock even if called with a later
    // sequence; the next measured BeforeRock owns that transition.
    assert(phases.beginPhase(Phase::NativeGraphOutput,42)==42);
    assert(phases.current()==41 && phases.leaseBoundary()==41);
    assert(phases.beginPhase(Phase::BeforeRock,42)==42);
    assert(phases.publishFrame()==42);

    // FRIK 2.3 can initialize the provider during FrameBegin, before BeforeRock.
    // Those events need the new frame while the previous publication's drive
    // still remains available to the upcoming physics/animation update.
    const auto earlyFrameLease = exclusiveExpiryFrame(phases.leaseBoundary(), 1);
    phases.beginFrame(43);
    assert(phases.current() == 43 && phases.leaseBoundary() == 42);
    assert(isActive(phases.leaseBoundary(), earlyFrameLease));
    assert(phases.beginPhase(Phase::NativeGraphOutput, 43) == 43);
    assert(phases.beginPhase(Phase::BeforeRock, 43) == 43);
    assert(phases.leaseBoundary() == 42);
    assert(phases.publishFrame() == 43);
    assert(!isActive(phases.leaseBoundary(), earlyFrameLease));

    return 0;
}
