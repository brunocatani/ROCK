#include <cstdio>
#include <string_view>

#include "physics-interaction/contact/ContactTargetIdentity.h"

namespace
{
    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }

        std::printf("%s expected true\n", label);
        return false;
    }
}

int main()
{
    using namespace rock::contact_target_identity;

    bool ok = true;
    ok &= expectTrue("horizontal normal classifies as wall-like", classifySurfaceHint(RE::NiPoint3{ 1.0f, 0.0f, 0.0f }) == SurfaceHint::WallLike);
    ok &= expectTrue("up normal classifies as floor-like", classifySurfaceHint(RE::NiPoint3{ 0.0f, 0.0f, 1.0f }) == SurfaceHint::FloorLike);
    ok &= expectTrue("down normal classifies as ceiling-like", classifySurfaceHint(RE::NiPoint3{ 0.0f, 0.0f, -1.0f }) == SurfaceHint::CeilingLike);
    ok &= expectTrue("diagonal normal classifies as slope-like", classifySurfaceHint(RE::NiPoint3{ 0.0f, 0.70f, 0.70f }) == SurfaceHint::SlopeLike);
    ok &= expectTrue("zero normal classifies as unknown", classifySurfaceHint(RE::NiPoint3{}) == SurfaceHint::Unknown);

    ok &= expectTrue("surface hint name is stable", std::string_view(surfaceHintName(SurfaceHint::WallLike)) == "WallLike");
    ok &= expectTrue("body resolved status name is stable", std::string_view(resolutionStatusName(ContactTargetResolutionStatus::BodyResolved)) == "BodyResolved");
    ok &= expectTrue("resolution status name is stable", std::string_view(resolutionStatusName(ContactTargetResolutionStatus::ResolvedReference)) == "ResolvedReference");
    ok &= expectTrue("endpoint kind name is stable", std::string_view(endpointKindName(rock::contact_evidence::NativeContactEndpointKind::WorldSurface)) == "WorldSurface");

    ContactTargetIdentity identity{};
    ok &= expectTrue("default identity starts invalid", !identity.valid);
    ok &= expectTrue("default identity has no resolved ref", !identity.hasResolvedReference);
    ok &= expectTrue("default identity has no rich text", !identity.hasRichText);
    ok &= expectTrue("default identity target layer is unknown", identity.layer == kUnknownLayer);
    ok &= expectTrue("default identity filter is unknown", identity.filterInfo == kUnknownFilterInfo);

    ContactTargetResolutionOptions options{};
    ok &= expectTrue("reference resolution is opt-in", !options.resolveReference);
    ok &= expectTrue("rich text resolution is opt-in", !options.includeRichText);

    return ok ? 0 : 1;
}
