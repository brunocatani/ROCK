#include "physics-interaction/grab/GrabContact.h"

#include <array>
#include <cmath>
#include <cstdio>
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

    return ok ? 0 : 1;
}
