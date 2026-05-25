#include "physics-interaction/grab/GrabContact.h"

#include <cstdio>
#include <limits>
#include <string_view>

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
        if (actual && actual == expected) {
            return true;
        }
        std::printf("%s expected reason %.*s got %s\n",
            label,
            static_cast<int>(expected.size()),
            expected.data(),
            actual ? actual : "(null)");
        return false;
    }

    rock::grab_pivot_authority_policy::MeshBackedPatchPivotAuthorityInput validInput()
    {
        rock::grab_pivot_authority_policy::MeshBackedPatchPivotAuthorityInput input{};
        input.baselineValid = true;
        input.patchComparable = true;
        input.patchValid = true;
        input.patchMeshSnapped = true;
        input.patchBroadSurfaceSupport = true;
        input.patchUniqueSampleCount = 3;
        input.patchUniqueTriangleCount = 3;
        input.selectedPivotToPocketGameUnits = 9.0f;
        input.patchPivotToPocketGameUnits = 3.0f;
        input.selectedScore = 9.0f;
        input.patchScore = 2.5f;
        input.patchAuthorityDeltaGameUnits = 4.0f;
        input.patchSelectionDeltaGameUnits = 5.0f;
        input.patchSpanGameUnits = 2.0f;
        input.patchAreaTwiceGameUnitsSq = 4.0f;
        input.probeSpacingGameUnits = 3.0f;
        input.meshSnapMaxDistanceGameUnits = 6.0f;
        input.alignmentMaxSelectionDeltaGameUnits = 8.0f;
        return input;
    }
}

int main()
{
    using namespace rock::grab_pivot_authority_policy;

    bool ok = true;

    {
        const auto decision = chooseMeshBackedPatchPivotAuthority(validInput());
        ok &= expectTrue("mesh-snapped broad patch can own position pivot", decision.acceptPatchPivot);
        ok &= expectTrue("accepted patch remains position-only", decision.positionOnlyAuthority);
        ok &= expectReason("accepted reason", decision.reason, "SurfacePatchPositionPivot");
    }

    {
        auto input = validInput();
        input.patchMeshSnapped = false;
        const auto decision = chooseMeshBackedPatchPivotAuthority(input);
        ok &= expectFalse("unsnapped patch cannot own pivot", decision.acceptPatchPivot);
        ok &= expectReason("unsnapped reason", decision.reason, "patchNotMeshSnappedForAuthority");
    }

    {
        auto input = validInput();
        input.patchBroadSurfaceSupport = false;
        input.patchUniqueSampleCount = 2;
        input.patchUniqueTriangleCount = 1;
        const auto decision = chooseMeshBackedPatchPivotAuthority(input);
        ok &= expectFalse("weak patch support cannot own pivot", decision.acceptPatchPivot);
        ok &= expectReason("weak support reason", decision.reason, "patchSurfaceSupportTooWeak");
    }

    {
        auto input = validInput();
        input.patchSpanGameUnits = 0.25f;
        const auto decision = chooseMeshBackedPatchPivotAuthority(input);
        ok &= expectFalse("tiny patch support cannot own pivot", decision.acceptPatchPivot);
        ok &= expectReason("tiny support reason", decision.reason, "patchSurfaceSupportTooSmall");
    }

    {
        auto input = validInput();
        input.patchAuthorityDeltaGameUnits = 14.0f;
        const auto decision = chooseMeshBackedPatchPivotAuthority(input);
        ok &= expectFalse("distant patch cannot own pivot", decision.acceptPatchPivot);
        ok &= expectReason("distant reason", decision.reason, "patchTooFarFromSelectedAuthority");
    }

    {
        auto input = validInput();
        input.patchSelectionDeltaGameUnits = 12.5f;
        const auto decision = chooseMeshBackedPatchPivotAuthority(input);
        ok &= expectFalse("selection-incoherent patch cannot own pivot", decision.acceptPatchPivot);
        ok &= expectReason("selection reason", decision.reason, "patchSelectionNotCoherent");
    }

    {
        auto input = validInput();
        input.patchSelectionDeltaGameUnits = (std::numeric_limits<float>::infinity)();
        const auto decision = chooseMeshBackedPatchPivotAuthority(input);
        ok &= expectFalse("non-finite selection delta cannot own pivot", decision.acceptPatchPivot);
        ok &= expectReason("non-finite selection reason", decision.reason, "patchSelectionNotCoherent");
    }

    {
        auto input = validInput();
        input.selectedPivotToPocketGameUnits = 4.0f;
        input.patchPivotToPocketGameUnits = 3.25f;
        const auto decision = chooseMeshBackedPatchPivotAuthority(input);
        ok &= expectFalse("small seat improvement is not enough", decision.acceptPatchPivot);
        ok &= expectReason("small seat improvement reason", decision.reason, "patchSeatImprovementTooSmall");
    }

    {
        auto input = validInput();
        input.selectedScore = 4.0f;
        input.patchScore = 3.25f;
        const auto decision = chooseMeshBackedPatchPivotAuthority(input);
        ok &= expectFalse("small score improvement is not enough", decision.acceptPatchPivot);
        ok &= expectReason("small score improvement reason", decision.reason, "patchScoreImprovementTooSmall");
    }

    return ok ? 0 : 1;
}
