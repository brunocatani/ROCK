#include "physics-interaction/consume/ImmersiveAidPolicy.h"
#include "physics-interaction/consume/ImmersiveAidPose.h"
#include "physics-interaction/consume/ImmersiveAidContact.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

namespace
{
    bool check(bool condition, const char* label)
    {
        if (!condition) {
            std::printf("FAILED: %s\n", label);
        }
        return condition;
    }

    rock::game_frame_timing_policy::GameFrameTiming frame(std::uint64_t sequence, float dt = 0.01f)
    {
        return { .sequence = sequence, .deltaSeconds = dt, .valid = true };
    }
}

int main()
{
    using namespace rock::immersive_aid;
    using Zone = rock::body_zone::BodyZoneKind;
    bool ok = true;
    constexpr std::array ids{ 0x23736u, 0x19C0D9u, 0x55F10u, 0x33779u, 0x58AA7u,
        0x3377Du, 0x58AACu, 0x58AAAu, 0x58AA8u, 0xEC4F7u };
    for (const auto id : ids) {
        ok &= check(classify("Fallout4.esm", id) != Injector::None, "injectable ALCH identity");
        ok &= check(classify("Other.esm", id) == Injector::None, "same local ID in another plugin stays oral");
    }
    ok &= check(classify("Fallout4.esm", 0x3736) == Injector::None, "truncated saved key is not a Stimpak form");
    ok &= check(classify("Fallout4.esm", 0x23742) == Injector::None, "RadAway stays oral");
    ok &= check(classify("Fallout4.esm", 0x459C5) == Injector::None, "Addictol stays oral");
    ok &= check(classify("Fallout4.esm", 0x58AF2) == Injector::None, "syringer poison excluded");
    ok &= check(usesStimpakPose(Injector::Stimpak) && usesStimpakPose(Injector::MedX), "shared Stimpak and Med-X pose");
    ok &= check(!usesStimpakPose(Injector::Psycho) && !usesStimpakPose(Injector::Serum), "different injector shapes retain their grips");
    ok &= check(!eligibleBodyZone(Zone::LeftHand, true) && !eligibleBodyZone(Zone::RightHand, false), "holding hand excluded");
    ok &= check(eligibleBodyZone(Zone::RightHand, true) && eligibleBodyZone(Zone::LeftHand, false), "opposite hand allowed");
    ok &= check(eligibleBodyZone(Zone::LeftForearmLower, true) && eligibleBodyZone(Zone::RightForearmLower, false), "suppressed forearm remains observable");
    ok &= check(eligibleBodyZone(Zone::Chest, true) && eligibleBodyZone(Zone::LeftThigh, false), "torso and legs allowed");
    ok &= check(!eligibleBodyZone(Zone::Unknown, false), "unknown body zone rejected");

    for (const int hz : { 30, 60, 72, 90, 120, 144 }) {
        ContactState state{};
        int commits = 0;
        for (std::uint64_t sequence = 1; sequence <= static_cast<std::uint64_t>(hz * 2); ++sequence) {
            if (advanceContact(state, true, 1, 10, frame(sequence, 1.0f / hz))) {
                ++commits;
                ok &= check(state.dwellSeconds >= kContactSeconds && state.dwellSeconds < kContactSeconds + 1.01 / hz,
                    "half-second dwell is independent of frame rate");
            }
        }
        ok &= check(commits == 1, "sustained contact commits once");
    }

    ContactState state{};
    for (std::uint64_t i = 1; i <= 41; ++i) {
        ok &= check(!advanceContact(state, true, 1, 1, frame(i)), "brief contact cannot consume");
    }
    const double beforeDuplicate = state.dwellSeconds;
    ok &= check(!advanceContact(state, true, 1, 1, frame(41)) && state.dwellSeconds == beforeDuplicate,
        "duplicate frame cannot advance dwell");
    ok &= check(!advanceContact(state, false, 1, 1, frame(42)) && !state.touching, "contact loss resets dwell");
    ok &= check(!advanceContact(state, true, 1, 1, frame(43)) && state.dwellSeconds == 0, "reentry starts at zero");
    static_cast<void>(advanceContact(state, true, 1, 1, frame(44)));
    ok &= check(!advanceContact(state, true, 2, 1, frame(45)) && state.dwellSeconds == 0, "new grab resets dwell");
    static_cast<void>(advanceContact(state, true, 2, 1, frame(46)));
    ok &= check(!advanceContact(state, true, 2, 2, frame(47)) && state.dwellSeconds == 0, "collider rebuild resets dwell");
    static_cast<void>(advanceContact(state, true, 2, 2, frame(48)));
    ok &= check(!advanceContact(state, true, 2, 2, frame(50)) && state.dwellSeconds == 0, "unobserved frame cannot count as contact");
    auto paused = frame(51);
    paused.menuPaused = true;
    ok &= check(!advanceContact(state, true, 2, 2, paused) && !state.touching, "menu pause cancels gesture");
    auto hitch = frame(52);
    hitch.discontinuity = true;
    ok &= check(!advanceContact(state, true, 2, 2, hitch) && !state.touching, "hitch cancels gesture");
    ok &= check(!advanceContact(state, true, 2, 2, frame(53, std::numeric_limits<float>::quiet_NaN())), "invalid delta rejected");
    ok &= check(!advanceContact(state, true, 0, 2, frame(54)), "missing grab rejected");

    const std::array mesh{ rock::GrabLocalTriangle{ { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } } };
    const RE::NiPoint3 minimum{ 0, 0, 0 }, maximum{ 1, 1, 0 };
    ok &= check(capsuleTouchesMesh(mesh, minimum, maximum, { 0.2f, 0.2f, -1 }, { 0.2f, 0.2f, 1 }, 0.01f),
        "thin capsule crossing a triangle face counts as contact");
    ok &= check(capsuleTouchesMesh(mesh, minimum, maximum, { 0, 0, 0.1f }, { 0, 0, 1 }, 0.11f),
        "small needle vertex touching a capsule cap counts");
    ok &= check(!capsuleTouchesMesh(mesh, minimum, maximum, { 0.9f, 0.9f, -1 }, { 0.9f, 0.9f, 1 }, 0.05f),
        "overlapping bounding boxes alone cannot trigger injection");
    ok &= check(!capsuleTouchesMesh(mesh, minimum, maximum, { 5, 5, -1 }, { 5, 5, 1 }, 0.1f), "distant body zone rejected");
    ok &= check(!capsuleTouchesMesh(mesh, minimum, maximum, { 0, 0, 0 }, { 0, 0, 1 },
        std::numeric_limits<float>::quiet_NaN()), "invalid body radius rejected");
    ok &= check(!capsuleTouchesMesh({}, minimum, maximum, { 0, 0, 0 }, { 0, 0, 1 }, 1), "missing mesh rejected");

    for (const bool left : { false, true }) {
        const auto& pose = stimpakPose(left);
        ok &= check(pose.present && pose.hasFingerPose && pose.hasFingerJointValues, "authored object and complete fingers retained");
        for (int row = 0; row < 3; ++row) {
            for (int other = 0; other < 3; ++other) {
                float dot = 0;
                for (int col = 0; col < 3; ++col) {
                    dot += pose.rotate[row * 3 + col] * pose.rotate[other * 3 + col];
                }
                ok &= check(std::abs(dot - (row == other ? 1.0f : 0.0f)) < 0.001f, "authored rotation remains orthonormal");
            }
        }
    }
    ok &= check(stimpakPose(true).translateGame[2] > 0 && stimpakPose(false).translateGame[2] < 0,
        "left and right select their separate captures");
    return ok ? 0 : 1;
}
