#include "physics-interaction/grab/GrabCore.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

namespace
{
    RE::NiTransform identityTransform()
    {
        return rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    }

    RE::NiMatrix3 zRotationDegrees(float degrees)
    {
        const float radians = degrees * (3.14159265358979323846f / 180.0f);
        const float halfAngle = radians * 0.5f;
        const float quaternion[4]{ 0.0f, 0.0f, std::sin(halfAngle), std::cos(halfAngle) };
        return rock::transform_math::havokQuaternionToNiRows<RE::NiMatrix3>(quaternion);
    }

    bool expectNear(const char* label, float actual, float expected, float epsilon)
    {
        const float delta = actual > expected ? actual - expected : expected - actual;
        if (delta <= epsilon) {
            return true;
        }
        std::printf("%s expected %.4f got %.4f\n", label, expected, actual);
        return false;
    }

    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, bool value)
    {
        if (!value) {
            return true;
        }
        std::printf("%s expected false\n", label);
        return false;
    }

    bool expectPointNear(const char* label, const RE::NiPoint3& actual, const RE::NiPoint3& expected, float epsilon)
    {
        bool ok = true;
        ok &= expectNear(label, actual.x, expected.x, epsilon);
        ok &= expectNear(label, actual.y, expected.y, epsilon);
        ok &= expectNear(label, actual.z, expected.z, epsilon);
        return ok;
    }

    bool expectRotationNear(const char* label, const RE::NiMatrix3& actual, const RE::NiMatrix3& expected, float epsilonDegrees)
    {
        return expectNear(label, rock::grab_authority_experiment::rotationDeltaDegrees(actual, expected), 0.0f, epsilonDegrees);
    }
}

int main()
{
    namespace authority = rock::grab_authority_frame_math;
    namespace experiment = rock::grab_authority_experiment;

    bool ok = true;

    {
        ok &= expectTrue("mode 51 is accepted", experiment::isKnownModeId(51));
        ok &= expectTrue(
            "unknown INI mode fails closed",
            experiment::modeFromId(99) == experiment::Mode::CurrentHybridBaseline);
        ok &= expectTrue(
            "unified blend mode uses body-local pivot B",
            experiment::usesBodyLocalPivotBTruth(experiment::Mode::UnifiedAuthorityLocalAAndBodyLocalBWithBlend));
        ok &= expectTrue(
            "unified blend mode uses authority-local pivot A",
            experiment::usesAuthorityLocalPivotAFreeze(experiment::Mode::UnifiedAuthorityLocalAAndBodyLocalBWithBlend));
    }

    {
        RE::NiTransform rawHandWorld = identityTransform();
        rawHandWorld.translate = RE::NiPoint3{ 1.0f, 2.0f, 3.0f };

        RE::NiTransform proxyWorld = identityTransform();
        proxyWorld.translate = RE::NiPoint3{ 10.0f, 20.0f, 30.0f };
        proxyWorld.rotate = zRotationDegrees(90.0f);

        auto policy = experiment::Policy{};
        auto evaluated = experiment::makeAuthorityFrame(rawHandWorld, proxyWorld, policy, 0.0f, false);
        ok &= expectPointNear("mode 0 keeps proxy translation", evaluated.frame.translate, proxyWorld.translate, 0.001f);
        ok &= expectRotationNear("mode 0 keeps raw rotation", evaluated.frame.rotate, rawHandWorld.rotate, 0.01f);
        ok &= expectNear("mode 0 has no blend", evaluated.blendWeight, 0.0f, 0.001f);

        policy.mode = experiment::Mode::RawRawAuthorityFrame;
        evaluated = experiment::makeAuthorityFrame(rawHandWorld, proxyWorld, policy, 0.0f, false);
        ok &= expectPointNear("mode 10 uses raw translation", evaluated.frame.translate, rawHandWorld.translate, 0.001f);
        ok &= expectRotationNear("mode 10 uses raw rotation", evaluated.frame.rotate, rawHandWorld.rotate, 0.01f);

        policy.mode = experiment::Mode::ProxyProxyAuthorityFrame;
        evaluated = experiment::makeAuthorityFrame(rawHandWorld, proxyWorld, policy, 0.0f, false);
        ok &= expectPointNear("mode 11 uses proxy translation", evaluated.frame.translate, proxyWorld.translate, 0.001f);
        ok &= expectRotationNear("mode 11 uses proxy rotation", evaluated.frame.rotate, proxyWorld.rotate, 0.01f);

        policy.mode = experiment::Mode::HybridSmoothBlendOnMismatch;
        policy.mismatchBlendStartDegrees = 0.0f;
        policy.mismatchBlendFullDegrees = 90.0f;
        policy.mismatchBlendMaxWeight = 1.0f;
        evaluated = experiment::makeAuthorityFrame(rawHandWorld, proxyWorld, policy, 0.0f, false);
        ok &= expectNear("mode 21 reaches full blend", evaluated.blendWeight, 1.0f, 0.001f);
        ok &= expectRotationNear("mode 21 blends to proxy rotation", evaluated.frame.rotate, proxyWorld.rotate, 0.01f);

        policy.mode = experiment::Mode::HybridStartupSmoothBlendOnMismatch;
        policy.startupBlendMaxSeconds = 0.35f;
        evaluated = experiment::makeAuthorityFrame(rawHandWorld, proxyWorld, policy, 0.5f, false);
        ok &= expectNear("mode 23 stops after startup window", evaluated.blendWeight, 0.0f, 0.001f);
        ok &= expectRotationNear("mode 23 returns to raw rotation after startup", evaluated.frame.rotate, rawHandWorld.rotate, 0.01f);

        policy.mode = experiment::Mode::HybridBlendUntilTouchHeld;
        evaluated = experiment::makeAuthorityFrame(rawHandWorld, proxyWorld, policy, 0.0f, true);
        ok &= expectNear("mode 24 stops at TouchHeld", evaluated.blendWeight, 0.0f, 0.001f);
        ok &= expectRotationNear("mode 24 returns to raw rotation at TouchHeld", evaluated.frame.rotate, rawHandWorld.rotate, 0.01f);
    }

    {
        using Candidate = authority::GrabAuthorityPivotCandidate<RE::NiPoint3>;
        const std::array<Candidate, 5> candidates{
            Candidate{
                .valid = true,
                .source = authority::GrabAuthorityPivotSource::CollisionFallback,
                .pointWorld = RE::NiPoint3{ 50.0f, 0.0f, 0.0f },
                .bodyId = 7,
                .reason = "collision",
            },
            Candidate{
                .valid = true,
                .source = authority::GrabAuthorityPivotSource::SelectionMeshSnap,
                .pointWorld = RE::NiPoint3{ 40.0f, 0.0f, 0.0f },
                .bodyId = 7,
                .reason = "selection",
            },
            Candidate{
                .valid = true,
                .source = authority::GrabAuthorityPivotSource::PalmPocketMesh,
                .pointWorld = RE::NiPoint3{ 30.0f, 0.0f, 0.0f },
                .bodyId = 7,
                .reason = "palm",
            },
            Candidate{
                .valid = true,
                .source = authority::GrabAuthorityPivotSource::GripSupportModel,
                .pointWorld = RE::NiPoint3{ 20.0f, 0.0f, 0.0f },
                .bodyId = 7,
                .reason = "support",
            },
            Candidate{
                .valid = true,
                .source = authority::GrabAuthorityPivotSource::PinchPocket,
                .pointWorld = RE::NiPoint3{ 10.0f, 0.0f, 0.0f },
                .bodyId = 7,
                .reason = "pinch",
            },
        };

        const auto resolved = authority::resolveGrabAuthorityPivot<RE::NiPoint3>(candidates, true);
        ok &= expectTrue("resolver accepts best candidate", resolved.valid);
        ok &= expectTrue("pinch pocket has highest priority", resolved.source == authority::GrabAuthorityPivotSource::PinchPocket);
        ok &= expectPointNear("pinch point selected", resolved.pointWorld, RE::NiPoint3{ 10.0f, 0.0f, 0.0f }, 0.001f);
    }

    {
        using Candidate = authority::GrabAuthorityPivotCandidate<RE::NiPoint3>;
        const std::array<Candidate, 1> candidates{
            Candidate{
                .valid = true,
                .source = authority::GrabAuthorityPivotSource::CollisionFallback,
                .pointWorld = RE::NiPoint3{ 50.0f, 0.0f, 0.0f },
                .bodyId = 7,
                .reason = "collision",
            },
        };

        const auto disabled = authority::resolveGrabAuthorityPivot<RE::NiPoint3>(candidates, false);
        ok &= expectFalse("collision fallback is rejected when disabled", disabled.valid);

        const auto enabled = authority::resolveGrabAuthorityPivot<RE::NiPoint3>(candidates, true);
        ok &= expectTrue("collision fallback is accepted when enabled", enabled.valid);
        ok &= expectTrue("collision fallback source selected", enabled.source == authority::GrabAuthorityPivotSource::CollisionFallback);
    }

    {
        RE::NiTransform rawHandWorld = identityTransform();
        rawHandWorld.translate = RE::NiPoint3{ 0.0f, 0.0f, 0.0f };

        RE::NiTransform proxyWorld = identityTransform();
        proxyWorld.translate = RE::NiPoint3{ 3.0f, 0.0f, 0.0f };

        RE::NiTransform objectWorld = identityTransform();
        objectWorld.translate = RE::NiPoint3{ 10.0f, 0.0f, 0.0f };

        RE::NiTransform bodyWorld = identityTransform();
        bodyWorld.translate = RE::NiPoint3{ 12.0f, 0.0f, 0.0f };

        const RE::NiPoint3 pivotAWorld{ 5.0f, 0.0f, 0.0f };
        const RE::NiPoint3 gripPointWorld{ 11.0f, 0.0f, 0.0f };

        const auto frozen = authority::freezeGrabAuthorityFrame<RE::NiTransform>(
            authority::GrabAuthorityFrameFreezeInput<RE::NiTransform>{
                .rawHandWorld = rawHandWorld,
                .proxyWorld = proxyWorld,
                .rawRotationProxyFrameWorld = proxyWorld,
                .objectWorld = objectWorld,
                .bodyWorld = bodyWorld,
                .constraintBodyWorld = bodyWorld,
                .pivotAWorld = pivotAWorld,
                .gripPointWorld = gripPointWorld,
                .visualNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
                .source = authority::GrabAuthorityPivotSource::PalmPocketMesh,
                .visualNormalValid = true,
            });

        ok &= expectTrue("freeze creates a valid authority frame", frozen.valid);
        ok &= expectPointNear("body-local pivot B is frozen from selected world point",
            frozen.pivotBBodyLocalGame,
            RE::NiPoint3{ -1.0f, 0.0f, 0.0f },
            0.001f);
        ok &= expectPointNear("constraint-local pivot B matches selected world point",
            frozen.pivotBConstraintLocalGame,
            RE::NiPoint3{ -1.0f, 0.0f, 0.0f },
            0.001f);
        ok &= expectPointNear("desired object shifts selected point to pivot A",
            frozen.desiredCapturedNodeWorld.translate,
            RE::NiPoint3{ 4.0f, 0.0f, 0.0f },
            0.001f);
        ok &= expectPointNear("desired body preserves body-local relation",
            frozen.desiredBodyWorld.translate,
            RE::NiPoint3{ 6.0f, 0.0f, 0.0f },
            0.001f);
        ok &= expectPointNear("pivot A is preserved as a non-origin proxy-local point",
            frozen.pivotAAuthorityBodyLocalGame,
            RE::NiPoint3{ 2.0f, 0.0f, 0.0f },
            0.001f);

        const RE::NiTransform recomposedDesiredBody =
            rock::transform_math::composeTransforms(proxyWorld, frozen.desiredBodyInAuthorityFrameSpace);
        ok &= expectPointNear("frozen body relation recomposes through proxy frame",
            recomposedDesiredBody.translate,
            frozen.desiredBodyWorld.translate,
            0.001f);

        const RE::NiPoint3 targetGripPoint =
            rock::transform_math::localPointToWorld(frozen.desiredBodyWorld, frozen.pivotBConstraintLocalGame);
        ok &= expectPointNear("frozen pivot B reaches pivot A through desired body",
            targetGripPoint,
            pivotAWorld,
            0.001f);
    }

    {
        RE::NiTransform rawHandWorld = identityTransform();
        RE::NiTransform proxyWorld = identityTransform();
        proxyWorld.rotate = zRotationDegrees(90.0f);

        RE::NiTransform authorityWorld = proxyWorld;
        authorityWorld.rotate = rawHandWorld.rotate;

        RE::NiTransform objectWorld = identityTransform();
        objectWorld.translate = RE::NiPoint3{ 10.0f, 0.0f, 0.0f };

        RE::NiTransform bodyWorld = identityTransform();
        bodyWorld.translate = RE::NiPoint3{ 12.0f, 0.0f, 0.0f };

        const RE::NiPoint3 pivotAWorld{ 2.0f, 0.0f, 0.0f };
        const RE::NiPoint3 gripPointWorld{ 11.0f, 0.0f, 0.0f };

        const auto frozen = authority::freezeGrabAuthorityFrame<RE::NiTransform>(
            authority::GrabAuthorityFrameFreezeInput<RE::NiTransform>{
                .rawHandWorld = rawHandWorld,
                .proxyWorld = proxyWorld,
                .rawRotationProxyFrameWorld = authorityWorld,
                .objectWorld = objectWorld,
                .bodyWorld = bodyWorld,
                .constraintBodyWorld = bodyWorld,
                .pivotAWorld = pivotAWorld,
                .gripPointWorld = gripPointWorld,
                .source = authority::GrabAuthorityPivotSource::PalmPocketMesh,
            });

        ok &= expectTrue("freeze keeps authority-local pivot A", frozen.valid);
        ok &= expectPointNear(
            "authority-local pivot A follows authority frame",
            frozen.pivotAAuthorityFrameLocalGame,
            RE::NiPoint3{ 2.0f, 0.0f, 0.0f },
            0.001f);
        ok &= expectFalse(
            "proxy-local and authority-local pivot A differ under raw/proxy mismatch",
            frozen.pivotAAuthorityBodyLocalGame == frozen.pivotAAuthorityFrameLocalGame);
    }

    {
        RE::NiTransform invalidHand = identityTransform();
        invalidHand.rotate.entry[0][0] = std::numeric_limits<float>::infinity();

        RE::NiTransform frame = identityTransform();
        frame.translate = RE::NiPoint3{ 5.0f, 0.0f, 0.0f };

        RE::NiTransform objectWorld = identityTransform();
        objectWorld.translate = RE::NiPoint3{ 10.0f, 0.0f, 0.0f };

        RE::NiTransform bodyWorld = identityTransform();
        bodyWorld.translate = RE::NiPoint3{ 12.0f, 0.0f, 0.0f };

        const auto frozen = authority::freezeGrabAuthorityFrame<RE::NiTransform>(
            authority::GrabAuthorityFrameFreezeInput<RE::NiTransform>{
                .rawHandWorld = invalidHand,
                .proxyWorld = frame,
                .rawRotationProxyFrameWorld = frame,
                .objectWorld = objectWorld,
                .bodyWorld = bodyWorld,
                .constraintBodyWorld = bodyWorld,
                .pivotAWorld = frame.translate,
                .gripPointWorld = RE::NiPoint3{ 11.0f, 0.0f, 0.0f },
                .source = authority::GrabAuthorityPivotSource::PalmPocketMesh,
            });
        ok &= expectFalse("freeze rejects non-finite authority rotation", frozen.valid);
    }

    return ok ? 0 : 1;
}
