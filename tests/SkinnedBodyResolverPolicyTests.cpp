#include "physics-interaction/object/SkinnedBodyResolver.h"

#include <cstdio>
#include <string_view>

namespace
{
    bool expectBody(const char* label, std::uint32_t actual, std::uint32_t expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected body=%u got body=%u\n", label, expected, actual);
        return false;
    }

    bool expectSource(
        const char* label,
        rock::skinned_body_resolver::ResolutionSource actual,
        rock::skinned_body_resolver::ResolutionSource expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected source=%s got source=%s\n",
            label,
            rock::skinned_body_resolver::sourceName(expected),
            rock::skinned_body_resolver::sourceName(actual));
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

    bool expectReason(const char* label, const char* actual, std::string_view expected)
    {
        if (actual && std::string_view(actual) == expected) {
            return true;
        }
        std::printf("%s expected reason=%.*s got reason=%s\n", label, static_cast<int>(expected.size()), expected.data(), actual ? actual : "null");
        return false;
    }
}

int main()
{
    using namespace rock::skinned_body_resolver;
    using rock::grab_target::Kind;

    bool ok = true;

    const auto weightedDeadActor = resolvePrimaryBody(ResolutionInput{
        .targetKind = Kind::DeadActorBody,
        .surfaceOwnerBodyId = 202u,
        .selectedBodyId = 101u,
        .nearestBodyId = 303u,
        .surfaceOwnerUsable = true,
        .selectedUsable = true,
        .nearestUsable = true,
        .surfaceIsSkinned = true,
        .hasSkinInfluences = true,
    });
    ok &= expectBody("weighted dead actor uses bone owner", weightedDeadActor.bodyId, 202u);
    ok &= expectSource("weighted dead actor source", weightedDeadActor.source, ResolutionSource::WeightedSkinOwner);
    ok &= expectTrue("weighted dead actor records skin use", weightedDeadActor.usedSkinInfluences);
    ok &= expectTrue("dead actor relaxes strict authority", weightedDeadActor.relaxedMechanicalAuthority);
    ok &= expectReason("weighted dead actor reason", weightedDeadActor.reason, "weightedSkinOwner");

    const auto authoredWins = resolvePrimaryBody(ResolutionInput{
        .targetKind = Kind::DeadActorBody,
        .authoredBodyId = 404u,
        .surfaceOwnerBodyId = 202u,
        .selectedBodyId = 101u,
        .nearestBodyId = 303u,
        .authoredUsable = true,
        .surfaceOwnerUsable = true,
        .selectedUsable = true,
        .nearestUsable = true,
        .surfaceIsSkinned = true,
        .hasSkinInfluences = true,
    });
    ok &= expectBody("authored node wins over skin owner", authoredWins.bodyId, 404u);
    ok &= expectSource("authored node source", authoredWins.source, ResolutionSource::AuthoredNode);

    const auto positionOnlySkinned = resolvePrimaryBody(ResolutionInput{
        .targetKind = Kind::DeadActorBody,
        .surfaceOwnerBodyId = 202u,
        .selectedBodyId = 101u,
        .nearestBodyId = 303u,
        .surfaceOwnerUsable = true,
        .selectedUsable = true,
        .nearestUsable = true,
        .surfaceIsSkinned = true,
        .hasSkinInfluences = false,
    });
    ok &= expectBody("position-only skinned uses triangle owner", positionOnlySkinned.bodyId, 202u);
    ok &= expectSource("position-only skinned source", positionOnlySkinned.source, ResolutionSource::TriangleOwner);
    ok &= expectFalse("position-only skinned does not claim weights", positionOnlySkinned.usedSkinInfluences);

    const auto selectedFallback = resolvePrimaryBody(ResolutionInput{
        .targetKind = Kind::LooseObject,
        .surfaceOwnerBodyId = 202u,
        .selectedBodyId = 101u,
        .nearestBodyId = 303u,
        .surfaceOwnerUsable = false,
        .selectedUsable = true,
        .nearestUsable = true,
    });
    ok &= expectBody("selected fallback body", selectedFallback.bodyId, 101u);
    ok &= expectSource("selected fallback source", selectedFallback.source, ResolutionSource::SelectedBody);

    const auto nearestFallback = resolvePrimaryBody(ResolutionInput{
        .targetKind = Kind::LooseObject,
        .selectedBodyId = 101u,
        .nearestBodyId = 303u,
        .selectedUsable = false,
        .nearestUsable = true,
    });
    ok &= expectBody("nearest fallback body", nearestFallback.bodyId, 303u);
    ok &= expectSource("nearest fallback source", nearestFallback.source, ResolutionSource::NearestAccepted);

    return ok ? 0 : 1;
}
