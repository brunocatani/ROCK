#include "physics-interaction/grab/GrabContact.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>

namespace
{
    struct Vec3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    constexpr float kEpsilon = 0.001f;

    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectEqual(const char* label, std::size_t actual, std::size_t expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected %zu got %zu\n", label, expected, actual);
        return false;
    }

    bool expectNear(const char* label, float actual, float expected)
    {
        const float delta = std::fabs(actual - expected);
        if (delta <= kEpsilon) {
            return true;
        }
        std::printf("%s expected %.4f got %.4f\n", label, expected, actual);
        return false;
    }

    bool expectPointNear(const char* label, const Vec3& actual, const Vec3& expected)
    {
        bool ok = true;
        ok &= expectNear((std::string(label) + ".x").c_str(), actual.x, expected.x);
        ok &= expectNear((std::string(label) + ".y").c_str(), actual.y, expected.y);
        ok &= expectNear((std::string(label) + ".z").c_str(), actual.z, expected.z);
        return ok;
    }

    rock::grab_contact_patch_math::GrabContactPatchSample<Vec3> sample(
        Vec3 point,
        Vec3 normal = Vec3{ 0.0f, 0.0f, -1.0f })
    {
        rock::grab_contact_patch_math::GrabContactPatchSample<Vec3> out{};
        out.bodyId = 42;
        out.point = point;
        out.normal = normal;
        out.accepted = true;
        return out;
    }

    rock::grab_contact_patch_math::GrabContactPatchSample<Vec3> surfaceSample(
        Vec3 point,
        std::uintptr_t surfaceKey,
        std::uint32_t triangleIndex,
        Vec3 normal = Vec3{ 0.0f, 0.0f, -1.0f })
    {
        auto out = sample(point, normal);
        out.surfaceKey = surfaceKey;
        out.triangleIndex = triangleIndex;
        out.sourceKindId = 1;
        out.hasTriangle = true;
        out.exactBodyHit = true;
        return out;
    }

    rock::grab_support_model_math::GripSupportSample<Vec3> supportSample(
        Vec3 point,
        rock::grab_support_model_math::GripSupportRole role,
        Vec3 normal = Vec3{ 0.0f, 0.0f, -1.0f })
    {
        rock::grab_support_model_math::GripSupportSample<Vec3> out{};
        out.point = point;
        out.normal = normal;
        out.role = role;
        out.valid = true;
        return out;
    }

    auto cluster(const std::vector<rock::grab_contact_patch_math::GrabContactPatchSample<Vec3>>& samples)
    {
        return rock::grab_contact_patch_math::filterContactPatchSameSurfaceCluster(samples,
            Vec3{ 0.0f, 0.0f, 0.0f },
            Vec3{ 0.0f, 0.0f, 1.0f },
            2.5f,
            1.75f,
            8.0f,
            35.0f);
    }
}

int main()
{
    /*
     * Contact patch samples must describe one seated surface near the selected
     * pivot. These tests protect the in-game corner case where one palm probe
     * wraps to the opposite face of a tray/pistol/shovel and would otherwise
     * stretch the fitted patch into a false rotation request.
     */
    bool ok = true;

    {
        std::array<Vec3, rock::grab_contact_patch_math::kContactPatchProbePatternSampleCount> offsets{};
        const auto count = rock::grab_contact_patch_math::buildContactPatchProbeOffsets(offsets,
            Vec3{ 1.0f, 0.0f, 0.0f },
            Vec3{ 0.0f, 1.0f, 0.0f },
            3.0f);

        ok &= expectEqual("expanded probe pattern count", count, 9);
        ok &= expectPointNear("expanded probe center", offsets[0], Vec3{ 0.0f, 0.0f, 0.0f });
        ok &= expectPointNear("expanded probe tangent positive", offsets[1], Vec3{ 3.0f, 0.0f, 0.0f });
        ok &= expectPointNear("expanded probe tangent negative", offsets[2], Vec3{ -3.0f, 0.0f, 0.0f });
        ok &= expectPointNear("expanded probe bitangent positive", offsets[3], Vec3{ 0.0f, 3.0f, 0.0f });
        ok &= expectPointNear("expanded probe bitangent negative", offsets[4], Vec3{ 0.0f, -3.0f, 0.0f });
        ok &= expectPointNear("expanded probe diagonal positive positive", offsets[5], Vec3{ 3.0f, 3.0f, 0.0f });
        ok &= expectPointNear("expanded probe diagonal positive negative", offsets[6], Vec3{ 3.0f, -3.0f, 0.0f });
        ok &= expectPointNear("expanded probe diagonal negative positive", offsets[7], Vec3{ -3.0f, 3.0f, 0.0f });
        ok &= expectPointNear("expanded probe diagonal negative negative", offsets[8], Vec3{ -3.0f, -3.0f, 0.0f });
    }

    {
        const auto defaultProbe = rock::grab_contact_patch_math::computeContactPatchProbeGeometry(3.0f, 2.0f, 0.0f, 12.0f, 24.0f);
        ok &= expectNear("default probe spacing remains configured", defaultProbe.spacingGameUnits, 3.0f);
        ok &= expectNear("default probe radius remains configured", defaultProbe.radiusGameUnits, 2.0f);

        const auto smallProbe = rock::grab_contact_patch_math::computeContactPatchProbeGeometry(3.0f, 2.0f, 6.0f, 12.0f, 24.0f);
        ok &= expectNear("small-object probe spacing scales down", smallProbe.spacingGameUnits, 1.5f);
        ok &= expectNear("small-object probe radius keeps enough cast thickness", smallProbe.radiusGameUnits, 1.2f);

        const auto longProbe = rock::grab_contact_patch_math::computeContactPatchProbeGeometry(3.0f, 2.0f, 72.0f, 12.0f, 24.0f);
        ok &= expectNear("long-object probe spacing scales down", longProbe.spacingGameUnits, 1.65f);
        ok &= expectNear("long-object probe radius keeps enough cast thickness", longProbe.radiusGameUnits, 1.2f);
    }

    {
        using namespace rock::grab_contact_patch_math;
        const auto result = dedupeContactPatchSamples<Vec3>({
            surfaceSample(Vec3{ 0.0f, 0.0f, 0.0f }, 100, 7),
            surfaceSample(Vec3{ 0.2f, 0.0f, 0.0f }, 100, 7),
            surfaceSample(Vec3{ 2.0f, 0.0f, 0.0f }, 100, 8),
        });

        ok &= expectEqual("duplicate triangle raw count", result.rawAcceptedCount, 3);
        ok &= expectEqual("duplicate triangle deduped count", result.samples.size(), 2);
        ok &= expectEqual("duplicate triangle rejected count", result.duplicateRejectedCount, 1);
    }

    {
        using namespace rock::grab_contact_patch_math;
        const auto result = dedupeContactPatchSamples<Vec3>({
            sample(Vec3{ 0.0f, 0.0f, 0.0f }),
            sample(Vec3{ 0.25f, 0.10f, 0.0f }),
            sample(Vec3{ 2.0f, 0.0f, 0.0f }),
        },
            0.5f,
            0.98f);

        ok &= expectEqual("fallback point dedupe count", result.samples.size(), 2);
        ok &= expectEqual("fallback point duplicate rejected", result.duplicateRejectedCount, 1);
    }

    {
        using namespace rock::grab_contact_patch_math;
        const std::vector samples{
            surfaceSample(Vec3{ -1.0f, 0.0f, 0.0f }, 100, 1),
            surfaceSample(Vec3{ 1.0f, 0.0f, 0.0f }, 100, 2),
            surfaceSample(Vec3{ 0.0f, 1.0f, 0.0f }, 100, 3),
        };
        const auto quality = classifyContactPatchQuality(samples);

        ok &= expectTrue("three adjacent triangles form patch", quality.kind == SurfacePatchQualityKind::Patch);
        ok &= expectTrue("patch quality reports broad surface support", quality.broadSurfaceSupport);
        ok &= expectEqual("patch unique triangles", quality.uniqueTriangleCount, 3);
    }

    {
        using namespace rock::grab_contact_patch_math;
        const std::vector samples{
            surfaceSample(Vec3{ -1.0f, 0.0f, 0.0f }, 100, 1),
            surfaceSample(Vec3{ 1.0f, 0.0f, 0.0f }, 100, 2),
        };
        const auto quality = classifyContactPatchQuality(samples);

        ok &= expectTrue("two same-face samples are line quality", quality.kind == SurfacePatchQualityKind::Line);
        ok &= expectTrue("line quality is not broad support", !quality.broadSurfaceSupport);
    }

    {
        using namespace rock::grab_contact_patch_math;
        const std::vector samples{
            surfaceSample(Vec3{ -1.0f, 0.0f, 0.0f }, 100, 1),
            surfaceSample(Vec3{ 1.0f, 0.0f, 0.0f }, 101, 2),
            surfaceSample(Vec3{ 0.0f, 1.0f, 0.0f }, 100, 3),
        };
        const auto quality = classifyContactPatchQuality(samples);

        ok &= expectTrue("split-surface samples are downgraded", quality.kind == SurfacePatchQualityKind::Line);
        ok &= expectTrue("split-surface samples cannot claim broad support", !quality.broadSurfaceSupport);
        ok &= expectEqual("split-surface unique surface count", quality.uniqueSurfaceCount, 2);
    }

    {
        const auto result = cluster({
            sample(Vec3{ 0.0f, 0.0f, 0.0f }),
            sample(Vec3{ 1.5f, 0.0f, 0.2f }),
            sample(Vec3{ -1.2f, 0.4f, -0.1f }),
            sample(Vec3{ 0.5f, 0.0f, 7.0f }),
            sample(Vec3{ -0.5f, 0.0f, 8.5f }),
        });

        ok &= expectTrue("depth outlier cluster valid", result.valid);
        ok &= expectEqual("depth outlier accepted count", result.samples.size(), 3);
        ok &= expectEqual("depth outlier raw count", result.rawAcceptedCount, 5);
        ok &= expectEqual("depth outlier rejected count", result.clusterRejectedCount, 2);
        ok &= expectPointNear("depth outlier first survivor", result.samples.front().point, Vec3{ 0.0f, 0.0f, 0.0f });
    }

    {
        const auto result = cluster({
            sample(Vec3{ 0.0f, 0.0f, 0.0f }),
            sample(Vec3{ 2.0f, 0.0f, 0.1f }),
            sample(Vec3{ -1.5f, 1.5f, -0.2f }),
            sample(Vec3{ 12.0f, 0.0f, 0.1f }),
        });

        ok &= expectTrue("lateral outlier cluster valid", result.valid);
        ok &= expectEqual("lateral outlier accepted count", result.samples.size(), 3);
        ok &= expectEqual("lateral outlier rejected count", result.clusterRejectedCount, 1);
    }

    {
        const auto result = cluster({
            sample(Vec3{ 0.0f, 0.0f, 0.0f }, Vec3{ 0.0f, 0.0f, -1.0f }),
            sample(Vec3{ 1.0f, 0.0f, 0.1f }, Vec3{ 0.0f, 0.0f, -1.0f }),
            sample(Vec3{ -1.0f, 0.0f, -0.1f }, Vec3{ 0.0f, 0.0f, -1.0f }),
            sample(Vec3{ 0.5f, 0.5f, 0.0f }, Vec3{ 1.0f, 0.0f, 0.0f }),
        });

        ok &= expectTrue("normal outlier cluster valid", result.valid);
        ok &= expectEqual("normal outlier accepted count", result.samples.size(), 3);
        ok &= expectEqual("normal outlier rejected count", result.clusterRejectedCount, 1);
    }

    {
        const auto result = cluster({
            sample(Vec3{ 0.0f, 0.0f, 9.0f }),
            sample(Vec3{ 1.0f, 0.0f, 10.0f }),
        });

        ok &= expectTrue("all outliers rejected", !result.valid);
        ok &= expectEqual("all outliers rejected count", result.clusterRejectedCount, 0);
        ok &= expectEqual("all outliers anchor rejected", result.anchorRejectedCount, 2);
    }

    {
        using namespace rock::grab_support_model_math;
        const std::array samples{
            supportSample(Vec3{ -1.0f, 0.0f, 0.0f }, GripSupportRole::ThumbPad, Vec3{ -1.0f, 0.0f, 0.0f }),
            supportSample(Vec3{ 1.0f, 0.0f, 0.0f }, GripSupportRole::IndexPad, Vec3{ 1.0f, 0.0f, 0.0f }),
        };
        const auto model = buildGripSupportModel(GripSupportModelInput<Vec3>{
            .anchorPoint = Vec3{ 0.0f, 0.0f, 0.0f },
            .anchorNormal = Vec3{ 0.0f, 0.0f, -1.0f },
            .palmNormal = Vec3{ 0.0f, 0.0f, 1.0f },
            .acrossPalmAxis = Vec3{ 1.0f, 0.0f, 0.0f },
            .fingerAxis = Vec3{ 0.0f, 1.0f, 0.0f },
            .pinchAxis = Vec3{ 1.0f, 0.0f, 0.0f },
            .samples = samples.data(),
            .sampleCount = samples.size(),
            .longObjectLeverGameUnits = 4.0f,
            .smallObjectReferenceLeverGameUnits = 12.0f,
            .longObjectReferenceLeverGameUnits = 24.0f,
            .maxPivotShiftGameUnits = 4.0f,
            .minOpposedSpanGameUnits = 0.75f,
            .pinchSeat = true,
        });

        ok &= expectTrue("pinch support model is valid", model.valid);
        ok &= expectTrue("pinch support authors pivot", model.canAuthorPivot);
        ok &= expectTrue("pinch support kind", model.kind == GripSupportKind::OpposedPinch);
        ok &= expectPointNear("pinch support midpoint", model.pivotPoint, Vec3{ 0.0f, 0.0f, 0.0f });
    }

    {
        using namespace rock::grab_support_model_math;
        const std::array samples{
            supportSample(Vec3{ -1.2f, 0.0f, 0.0f }, GripSupportRole::AcrossNegative, Vec3{ -1.0f, 0.0f, 0.0f }),
            supportSample(Vec3{ 1.2f, 0.0f, 0.0f }, GripSupportRole::AcrossPositive, Vec3{ 1.0f, 0.0f, 0.0f }),
        };
        const auto model = buildGripSupportModel(GripSupportModelInput<Vec3>{
            .anchorPoint = Vec3{ 0.0f, 0.0f, 0.0f },
            .anchorNormal = Vec3{ 0.0f, 0.0f, -1.0f },
            .palmNormal = Vec3{ 0.0f, 0.0f, 1.0f },
            .acrossPalmAxis = Vec3{ 1.0f, 0.0f, 0.0f },
            .fingerAxis = Vec3{ 0.0f, 1.0f, 0.0f },
            .pinchAxis = Vec3{ 1.0f, 0.0f, 0.0f },
            .objectLongAxis = Vec3{ 0.0f, 1.0f, 0.0f },
            .samples = samples.data(),
            .sampleCount = samples.size(),
            .longObjectLeverGameUnits = 72.0f,
            .objectLongAxisSpanGameUnits = 80.0f,
            .smallObjectReferenceLeverGameUnits = 12.0f,
            .longObjectReferenceLeverGameUnits = 24.0f,
            .maxPivotShiftGameUnits = 4.0f,
            .minOpposedSpanGameUnits = 0.75f,
        });

        ok &= expectTrue("long handle support kind", model.kind == GripSupportKind::LongHandleAxis);
        ok &= expectTrue("long handle support authors pivot only from opposed side support", model.canAuthorPivot);
        ok &= expectPointNear("long handle support centerline pivot", model.pivotPoint, Vec3{ 0.0f, 0.0f, 0.0f });
        ok &= expectPointNear("long handle keeps object long axis", model.supportAxis, Vec3{ 0.0f, 1.0f, 0.0f });
    }

    {
        using namespace rock::grab_support_model_math;
        const std::array samples{
            supportSample(Vec3{ 0.0f, -10.0f, 0.0f }, GripSupportRole::ContactPatch),
            supportSample(Vec3{ 0.0f, 10.0f, 0.0f }, GripSupportRole::ContactPatch),
        };
        const auto model = buildGripSupportModel(GripSupportModelInput<Vec3>{
            .anchorPoint = Vec3{ 0.0f, 0.0f, 0.0f },
            .anchorNormal = Vec3{ 0.0f, 0.0f, -1.0f },
            .palmNormal = Vec3{ 0.0f, 0.0f, 1.0f },
            .acrossPalmAxis = Vec3{ 1.0f, 0.0f, 0.0f },
            .fingerAxis = Vec3{ 0.0f, 1.0f, 0.0f },
            .objectLongAxis = Vec3{ 0.0f, 1.0f, 0.0f },
            .samples = samples.data(),
            .sampleCount = samples.size(),
            .longObjectLeverGameUnits = 72.0f,
            .objectLongAxisSpanGameUnits = 80.0f,
            .smallObjectReferenceLeverGameUnits = 12.0f,
            .longObjectReferenceLeverGameUnits = 24.0f,
            .maxPivotShiftGameUnits = 4.0f,
            .minOpposedSpanGameUnits = 0.75f,
        });

        ok &= expectTrue("long same-face line reports long-handle metadata", model.kind == GripSupportKind::LongHandleAxis);
        ok &= expectTrue("long same-face line cannot author pivot", !model.canAuthorPivot);
    }

    {
        using namespace rock::grab_support_model_math;
        const std::array samples{
            supportSample(Vec3{ -1.2f, 0.0f, 0.0f }, GripSupportRole::AcrossNegative, Vec3{ 0.0f, 0.0f, -1.0f }),
            supportSample(Vec3{ 1.2f, 0.0f, 0.0f }, GripSupportRole::AcrossPositive, Vec3{ 0.0f, 0.0f, -1.0f }),
        };
        const auto model = buildGripSupportModel(GripSupportModelInput<Vec3>{
            .anchorPoint = Vec3{ 0.0f, 0.0f, 0.0f },
            .anchorNormal = Vec3{ 0.0f, 0.0f, -1.0f },
            .palmNormal = Vec3{ 0.0f, 0.0f, 1.0f },
            .acrossPalmAxis = Vec3{ 1.0f, 0.0f, 0.0f },
            .fingerAxis = Vec3{ 0.0f, 1.0f, 0.0f },
            .pinchAxis = Vec3{ 1.0f, 0.0f, 0.0f },
            .samples = samples.data(),
            .sampleCount = samples.size(),
            .longObjectLeverGameUnits = 4.0f,
            .smallObjectReferenceLeverGameUnits = 12.0f,
            .longObjectReferenceLeverGameUnits = 24.0f,
            .maxPivotShiftGameUnits = 4.0f,
            .minOpposedSpanGameUnits = 0.75f,
            .pinchSeat = true,
        });

        ok &= expectTrue("opposed roles with same face normals cannot author pivot", !model.canAuthorPivot);
        ok &= expectTrue("opposed roles with same face normals are not pinch", model.kind != GripSupportKind::OpposedPinch);
    }

    {
        using namespace rock::grab_support_model_math;
        const std::array samples{
            supportSample(Vec3{ -1.0f, -1.0f, 0.0f }, GripSupportRole::ContactPatch),
            supportSample(Vec3{ 1.0f, -1.0f, 0.0f }, GripSupportRole::ContactPatch),
            supportSample(Vec3{ -1.0f, 1.0f, 0.0f }, GripSupportRole::ContactPatch),
            supportSample(Vec3{ 1.0f, 1.0f, 0.0f }, GripSupportRole::ContactPatch),
        };
        const auto model = buildGripSupportModel(GripSupportModelInput<Vec3>{
            .anchorPoint = Vec3{ 0.0f, 0.0f, 0.0f },
            .anchorNormal = Vec3{ 0.0f, 0.0f, -1.0f },
            .palmNormal = Vec3{ 0.0f, 0.0f, 1.0f },
            .acrossPalmAxis = Vec3{ 1.0f, 0.0f, 0.0f },
            .fingerAxis = Vec3{ 0.0f, 1.0f, 0.0f },
            .samples = samples.data(),
            .sampleCount = samples.size(),
            .longObjectLeverGameUnits = 10.0f,
            .smallObjectReferenceLeverGameUnits = 12.0f,
            .longObjectReferenceLeverGameUnits = 24.0f,
            .maxPivotShiftGameUnits = 4.0f,
            .minOpposedSpanGameUnits = 0.75f,
        });

        ok &= expectTrue("broad same-face support stays evidence-only", !model.canAuthorPivot);
        ok &= expectTrue("broad same-face support is not palm wrap", model.kind != GripSupportKind::PalmWrap);
    }

    {
        using namespace rock::grab_support_model_math;
        const std::array samples{
            supportSample(Vec3{ 8.0f, 0.0f, 0.0f }, GripSupportRole::AcrossPositive),
            supportSample(Vec3{ 10.0f, 0.0f, 0.0f }, GripSupportRole::AcrossNegative),
        };
        const auto model = buildGripSupportModel(GripSupportModelInput<Vec3>{
            .anchorPoint = Vec3{ 0.0f, 0.0f, 0.0f },
            .anchorNormal = Vec3{ 0.0f, 0.0f, -1.0f },
            .palmNormal = Vec3{ 0.0f, 0.0f, 1.0f },
            .acrossPalmAxis = Vec3{ 1.0f, 0.0f, 0.0f },
            .fingerAxis = Vec3{ 0.0f, 1.0f, 0.0f },
            .pinchAxis = Vec3{ 1.0f, 0.0f, 0.0f },
            .samples = samples.data(),
            .sampleCount = samples.size(),
            .longObjectLeverGameUnits = 4.0f,
            .smallObjectReferenceLeverGameUnits = 12.0f,
            .longObjectReferenceLeverGameUnits = 24.0f,
            .maxPivotShiftGameUnits = 2.0f,
            .minOpposedSpanGameUnits = 0.75f,
            .pinchSeat = true,
        });

        ok &= expectTrue("distant opposed support remains evidence only", !model.canAuthorPivot);
        ok &= expectTrue("distant opposed support does not become pinch", model.kind != GripSupportKind::OpposedPinch);
    }

    return ok ? 0 : 1;
}
