#include "physics-interaction/weapon/collision/WeaponIntentStabilityPolicy.h"
#include "physics-interaction/weapon/presentation/PresentationTracePolicy.h"

#include <cmath>
#include <cstdio>
#include <limits>

namespace
{
    using namespace rock;

    bool expectTrue(const char* label, const bool actual)
    {
        if (actual) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, const bool actual)
    {
        if (!actual) {
            return true;
        }
        std::printf("%s expected false\n", label);
        return false;
    }

    bool expectEqual(const char* label, const unsigned actual, const unsigned expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected %u got %u\n", label, expected, actual);
        return false;
    }

    RE::NiTransform identityTransform()
    {
        RE::NiTransform result{};
        result.rotate.MakeIdentity();
        result.translate = RE::NiPoint3{ 0.0f, 0.0f, 0.0f };
        result.scale = 1.0f;
        return result;
    }

    RE::NiTransform translated(const RE::NiTransform& base, const float x, const float y, const float z)
    {
        RE::NiTransform result = base;
        result.translate = RE::NiPoint3{ base.translate.x + x, base.translate.y + y, base.translate.z + z };
        return result;
    }

    presentation_trace_policy::FrameRecord makeTwoHandCorrectionRecord()
    {
        presentation_trace_policy::FrameRecord record{};
        record.valid = true;
        record.dwcPublishRequested = true;
        for (auto& hand : record.hands) {
            hand.collisionRequested = true;
            hand.collisionTargetValid = true;
            hand.collisionApplied = true;
            hand.collisionAuthorityLive = true;
            hand.publicationReady = true;
        }
        record.dwcWeaponPublished = true;
        record.dwcHandGroupPublished = true;
        return record;
    }

    bool testGroupCommitInvariant()
    {
        bool ok = true;
        auto record = makeTwoHandCorrectionRecord();
        ok &= expectFalse("complete group does not violate group commit", presentation_trace_policy::violatesGroupCommit(record));

        // The shipped defect: the weapon takes the correction anyway.
        record.dwcHandGroupPublished = false;
        record.hands[0].collisionApplied = false;
        ok &= expectTrue("weapon write with failed hands is a group-commit violation", presentation_trace_policy::violatesGroupCommit(record));

        // A rolled-back transaction writes neither side and is not a violation.
        record.dwcWeaponPublished = false;
        ok &= expectFalse("rolled-back group is not a group-commit violation", presentation_trace_policy::violatesGroupCommit(record));

        // Nothing was requested, so nothing can be violated.
        auto idle = presentation_trace_policy::FrameRecord{};
        idle.valid = true;
        idle.dwcWeaponPublished = true;
        ok &= expectFalse("no requested correction is never a violation", presentation_trace_policy::violatesGroupCommit(idle));
        return ok;
    }

    bool testGroupAtomicityInvariant()
    {
        bool ok = true;
        auto record = makeTwoHandCorrectionRecord();
        ok &= expectFalse("both hands published is atomic", presentation_trace_policy::violatesGroupAtomicity(record));

        record.hands[1].collisionApplied = false;
        ok &= expectTrue("one hand of a requested pair is a partial stage", presentation_trace_policy::violatesGroupAtomicity(record));

        // Both requested hands failed: incomplete, but not split.
        record.hands[0].collisionApplied = false;
        ok &= expectFalse("a wholly failed group is not a partial stage", presentation_trace_policy::violatesGroupAtomicity(record));

        // A single attached hand cannot split.
        auto oneHand = makeTwoHandCorrectionRecord();
        oneHand.hands[0].collisionRequested = false;
        oneHand.hands[0].collisionApplied = false;
        ok &= expectFalse("a one-hand group cannot be a partial stage", presentation_trace_policy::violatesGroupAtomicity(oneHand));
        return ok;
    }

    bool testPublicationReadinessInvariant()
    {
        bool ok = true;
        auto record = makeTwoHandCorrectionRecord();
        ok &= expectFalse("ready hands satisfy the readiness invariant", presentation_trace_policy::violatesPublicationReadiness(record));

        record.hands[0].publicationReady = false;
        ok &= expectTrue("a claim on an unready hand violates readiness", presentation_trace_policy::violatesPublicationReadiness(record));

        // An unready hand that was never claimed is irrelevant.
        record.hands[0].collisionRequested = false;
        ok &= expectFalse("an unready but unclaimed hand is not a violation", presentation_trace_policy::violatesPublicationReadiness(record));
        return ok;
    }

    bool testDownstreamOrderingInvariant()
    {
        bool ok = true;
        presentation_trace_policy::FrameRecord record{};
        record.valid = true;
        ok &= expectFalse("no collider sample means no ordering claim", presentation_trace_policy::violatesDownstreamOrdering(record));

        record.weaponSampledAtColliderPublication = true;
        record.lateWriterTranslationGameUnits = 0.0f;
        record.lateWriterRotationDegrees = 0.0f;
        ok &= expectFalse("an unmoved weapon satisfies downstream ordering", presentation_trace_policy::violatesDownstreamOrdering(record));

        record.lateWriterTranslationGameUnits = presentation_trace_policy::kLateWriterTranslationToleranceGameUnits + 0.5f;
        ok &= expectTrue("a late translation write violates downstream ordering", presentation_trace_policy::violatesDownstreamOrdering(record));

        record.lateWriterTranslationGameUnits = 0.0f;
        record.lateWriterRotationDegrees = presentation_trace_policy::kLateWriterRotationToleranceDegrees + 1.0f;
        ok &= expectTrue("a late rotation write violates downstream ordering", presentation_trace_policy::violatesDownstreamOrdering(record));
        return ok;
    }

    bool testIntentStabilityCountsSteadyFrames()
    {
        bool ok = true;
        weapon_intent_stability_policy::State state{};
        const RE::NiTransform driver = identityTransform();
        const RE::NiTransform weapon = translated(driver, 3.0f, 0.0f, 0.0f);

        // The first frame of a generation establishes the baseline only.
        auto sample = weapon_intent_stability_policy::update(state, true, driver, true, weapon, 0x1234u);
        ok &= expectTrue("first generation frame is a valid sample", sample.valid);
        ok &= expectEqual("first generation frame counts no stable frame", sample.stableFrameCount, 0u);

        for (unsigned frame = 1; frame <= 3; ++frame) {
            // Move the driver and the weapon together: the local relation holds.
            const RE::NiTransform movedDriver = translated(driver, static_cast<float>(frame) * 10.0f, 0.0f, 0.0f);
            const RE::NiTransform movedWeapon = translated(movedDriver, 3.0f, 0.0f, 0.0f);
            sample = weapon_intent_stability_policy::update(state, true, movedDriver, true, movedWeapon, 0x1234u);
            ok &= expectEqual("player motion alone keeps the weapon intent stable", sample.stableFrameCount, frame);
        }
        return ok;
    }

    bool testIntentStabilityRejectsAttachFlightAndGenerationEdge()
    {
        bool ok = true;
        weapon_intent_stability_policy::State state{};
        const RE::NiTransform driver = identityTransform();

        (void)weapon_intent_stability_policy::update(state, true, driver, true, translated(driver, 3.0f, 0.0f, 0.0f), 0x1234u);
        auto sample = weapon_intent_stability_policy::update(state, true, driver, true, translated(driver, 3.0f, 0.0f, 0.0f), 0x1234u);
        ok &= expectEqual("a still weapon accumulates stability", sample.stableFrameCount, 1u);

        // An equip flight moves the weapon inside the driver's frame.
        sample = weapon_intent_stability_policy::update(state, true, driver, true, translated(driver, 500.0f, 0.0f, 0.0f), 0x1234u);
        ok &= expectEqual("attach flight resets the stable count", sample.stableFrameCount, 0u);

        // Rebuild the count, then change generation.
        (void)weapon_intent_stability_policy::update(state, true, driver, true, translated(driver, 500.0f, 0.0f, 0.0f), 0x1234u);
        sample = weapon_intent_stability_policy::update(state, true, driver, true, translated(driver, 500.0f, 0.0f, 0.0f), 0x9999u);
        ok &= expectTrue("a new generation is a valid sample", sample.valid);
        ok &= expectEqual("a new generation restarts the stable count", sample.stableFrameCount, 0u);

        // Missing inputs fail closed and drop the accumulated confidence.
        (void)weapon_intent_stability_policy::update(state, true, driver, true, translated(driver, 500.0f, 0.0f, 0.0f), 0x9999u);
        sample = weapon_intent_stability_policy::update(state, false, driver, true, translated(driver, 500.0f, 0.0f, 0.0f), 0x9999u);
        ok &= expectFalse("a missing driver yields no sample", sample.valid);
        sample = weapon_intent_stability_policy::update(state, true, driver, true, translated(driver, 500.0f, 0.0f, 0.0f), 0x9999u);
        ok &= expectEqual("a missing driver restarts the stable count", sample.stableFrameCount, 0u);

        RE::NiTransform brokenWeapon = translated(driver, 3.0f, 0.0f, 0.0f);
        brokenWeapon.translate.x = std::numeric_limits<float>::quiet_NaN();
        sample = weapon_intent_stability_policy::update(state, true, driver, true, brokenWeapon, 0x9999u);
        ok &= expectFalse("a non-finite weapon world yields no sample", sample.valid);
        return ok;
    }
}

int main()
{
    bool ok = true;
    ok &= testGroupCommitInvariant();
    ok &= testGroupAtomicityInvariant();
    ok &= testPublicationReadinessInvariant();
    ok &= testDownstreamOrderingInvariant();
    ok &= testIntentStabilityCountsSteadyFrames();
    ok &= testIntentStabilityRejectsAttachFlightAndGenerationEdge();
    return ok ? 0 : 1;
}
