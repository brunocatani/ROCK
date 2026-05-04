#include <cstdio>
#include <cstdint>

#include "physics-interaction/PhysicsWorldOriginDiagnostics.h"

namespace
{
    bool expectFloat(const char* label, float actual, float expected)
    {
        constexpr float kEpsilon = 0.0001f;
        const float delta = actual >= expected ? actual - expected : expected - actual;
        if (delta <= kEpsilon) {
            return true;
        }

        std::printf("%s expected %.4f got %.4f\n", label, expected, actual);
        return false;
    }

    bool expectCandidateKind(
        const char* label,
        frik::rock::origin_diagnostics::OriginCandidateKind actual,
        frik::rock::origin_diagnostics::OriginCandidateKind expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected %s got %s\n",
            label,
            frik::rock::origin_diagnostics::candidateKindName(expected),
            frik::rock::origin_diagnostics::candidateKindName(actual));
        return false;
    }

    bool expectU32(const char* label, std::uint32_t actual, std::uint32_t expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected %u got %u\n", label, expected, actual);
        return false;
    }
}

int main()
{
    namespace origin_diagnostics = frik::rock::origin_diagnostics;

    bool ok = true;

    {
        const RE::NiPoint3 raw{ 100.0f, 200.0f, -50.0f };
        const RE::NiPoint3 visual{ 100.0f, 200.0f, -50.0f };
        const RE::NiPoint3 zero{};

        const auto result = origin_diagnostics::evaluateOriginCandidates(raw, visual, zero, zero, zero, 5.0f);
        ok &= expectCandidateKind("zero origin candidate", result.best.kind, origin_diagnostics::OriginCandidateKind::Raw);
        ok &= expectFloat("zero origin best distance", result.best.distanceToVisual, 0.0f);
        ok &= expectFloat("zero origin warning", result.exceedsWarningThreshold ? 1.0f : 0.0f, 0.0f);
        ok &= expectU32("zero origin candidate count", result.count, 9);
    }

    {
        const RE::NiPoint3 raw{ 100.0f, 200.0f, -50.0f };
        const RE::NiPoint3 bhkOrigin{ 4096.0f, -8192.0f, 0.0f };
        const RE::NiPoint3 visual = origin_diagnostics::addPoint(raw, bhkOrigin);
        const RE::NiPoint3 hknpOrigin{ 128.0f, 128.0f, 0.0f };

        const auto result = origin_diagnostics::evaluateOriginCandidates(raw, visual, bhkOrigin, hknpOrigin, hknpOrigin, 5.0f);
        ok &= expectCandidateKind("bhk add candidate", result.best.kind, origin_diagnostics::OriginCandidateKind::BhkStoredOriginAdd);
        ok &= expectFloat("bhk add best distance", result.best.distanceToVisual, 0.0f);
    }

    {
        const RE::NiPoint3 raw{ -300.0f, 600.0f, 90.0f };
        const RE::NiPoint3 hknpOrigin{ -8192.0f, 4096.0f, 0.0f };
        const RE::NiPoint3 visual = origin_diagnostics::subtractPoint(raw, hknpOrigin);
        const RE::NiPoint3 bhkOrigin{ 0.0f, 0.0f, 0.0f };

        const auto result = origin_diagnostics::evaluateOriginCandidates(raw, visual, bhkOrigin, hknpOrigin, hknpOrigin, 5.0f);
        ok &= expectCandidateKind("hknp subtract candidate", result.best.kind, origin_diagnostics::OriginCandidateKind::HknpBroadphaseMinShiftSubtract);
        ok &= expectFloat("hknp subtract best distance", result.best.distanceToVisual, 0.0f);
    }

    {
        const RE::NiPoint3 raw{ 0.0f, 0.0f, 0.0f };
        const RE::NiPoint3 visual{ 50.0f, 0.0f, 0.0f };
        const RE::NiPoint3 origin{ 10.0f, 0.0f, 0.0f };

        const auto result = origin_diagnostics::evaluateOriginCandidates(raw, visual, origin, origin, origin, 5.0f);
        ok &= expectCandidateKind("mismatch keeps closest candidate", result.best.kind, origin_diagnostics::OriginCandidateKind::BhkStoredOriginAdd);
        ok &= expectFloat("mismatch best distance", result.best.distanceToVisual, 40.0f);
        ok &= expectFloat("mismatch warning", result.exceedsWarningThreshold ? 1.0f : 0.0f, 1.0f);
    }

    {
        const RE::NiPoint3 raw{ 10.0f, 10.0f, 10.0f };
        const RE::NiPoint3 bhkOrigin{ 0.0f, 0.0f, 0.0f };
        const RE::NiPoint3 hknpMinShift{ 256.0f, -128.0f, 0.0f };
        const RE::NiPoint3 hknpMaxShift{ -1024.0f, 512.0f, 0.0f };
        const RE::NiPoint3 visual = origin_diagnostics::addPoint(raw, hknpMaxShift);

        const auto result = origin_diagnostics::evaluateOriginCandidates(raw, visual, bhkOrigin, hknpMinShift, hknpMaxShift, 5.0f);
        ok &= expectCandidateKind("hknp max-shift add candidate", result.best.kind, origin_diagnostics::OriginCandidateKind::HknpBroadphaseMaxShiftAdd);
        ok &= expectFloat("hknp max-shift add distance", result.best.distanceToVisual, 0.0f);
    }

    {
        using origin_diagnostics::BodyPositionSource;
        using origin_diagnostics::VisualSourceKind;

        ok &= expectFloat("motion index 0 uses body transform",
            origin_diagnostics::chooseBodyPositionSource(0) == BodyPositionSource::BodyTransform ? 1.0f : 0.0f,
            1.0f);
        ok &= expectFloat("valid motion index uses body transform for collider origin",
            origin_diagnostics::chooseBodyPositionSource(42) == BodyPositionSource::BodyTransform ? 1.0f : 0.0f,
            1.0f);
        ok &= expectFloat("free motion index uses body transform",
            origin_diagnostics::chooseBodyPositionSource(0x7FFF'FFFF) == BodyPositionSource::BodyTransform ? 1.0f : 0.0f,
            1.0f);

        ok &= expectFloat("body owner visual source wins",
            origin_diagnostics::chooseVisualSourceKind(true, true, true, true) == VisualSourceKind::BodyOwnerNode ? 1.0f : 0.0f,
            1.0f);
        ok &= expectFloat("hit visual source wins over root",
            origin_diagnostics::chooseVisualSourceKind(false, true, false, true) == VisualSourceKind::HitNode ? 1.0f : 0.0f,
            1.0f);
        ok &= expectFloat("root visual source fallback",
            origin_diagnostics::chooseVisualSourceKind(false, false, false, true) == VisualSourceKind::ReferenceRoot ? 1.0f : 0.0f,
            1.0f);
    }

    return ok ? 0 : 1;
}
