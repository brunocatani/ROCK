#include "physics-interaction/hand/HandSelection.h"
#include "physics-interaction/hand/HandSelection.h"

#include <cstdio>
#include <cstdint>

namespace RE
{
    class NiAVObject;
}

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

    bool expectUint32(const char* label, std::uint32_t actual, std::uint32_t expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected 0x%08X got 0x%08X\n", label, expected, actual);
        return false;
    }
}

int main()
{
    using namespace frik::rock::selection_highlight_policy;

    bool ok = true;

    auto* hitNode = reinterpret_cast<RE::NiAVObject*>(static_cast<std::uintptr_t>(0x1000));
    auto* visualNode = reinterpret_cast<RE::NiAVObject*>(static_cast<std::uintptr_t>(0x2000));
    auto* referenceRoot = reinterpret_cast<RE::NiAVObject*>(static_cast<std::uintptr_t>(0x3000));

    ok &= expectTrue("selection highlight prefers reference root", chooseVatsHighlightTargetNode(hitNode, visualNode, referenceRoot) == referenceRoot);
    ok &= expectTrue("selection highlight keeps reference root without hit node", chooseVatsHighlightTargetNode(nullptr, visualNode, referenceRoot) == referenceRoot);
    ok &= expectTrue("selection highlight falls back to visual node", chooseVatsHighlightTargetNode(nullptr, visualNode, nullptr) == visualNode);
    ok &= expectTrue("selection highlight falls back to hit node", chooseVatsHighlightTargetNode(hitNode, nullptr, nullptr) == hitNode);
    ok &= expectTrue("selection highlight reports no node", chooseVatsHighlightTargetNode(nullptr, nullptr, nullptr) == nullptr);

    ok &= expectFalse("selection highlight does not enable VATS image space", kVatsHighlightEnableImageSpaceEffect);
    ok &= expectTrue("selection highlight uses object rollover render state", kVatsHighlightUseObjectRolloverState);
    ok &= expectTrue("selection highlight applies to far ray selection", shouldUseVatsHighlightForSelection(true));
    ok &= expectFalse("selection highlight skips close selection", shouldUseVatsHighlightForSelection(false));

    ok &= expectTrue("selection highlight starts only for enabled new ref", shouldStartVatsHighlight(true, true, false));
    ok &= expectFalse("selection highlight blocks disabled", shouldStartVatsHighlight(false, true, false));
    ok &= expectFalse("selection highlight blocks missing target", shouldStartVatsHighlight(true, false, false));
    ok &= expectFalse("selection highlight keeps existing target stable", shouldStartVatsHighlight(true, true, true));

    ok &= expectTrue("selection highlight refreshes selected ref at interval", shouldRefreshVatsHighlightAttempt(true, true, 10, 10));
    ok &= expectFalse("selection highlight refresh blocks disabled", shouldRefreshVatsHighlightAttempt(false, true, 10, 10));
    ok &= expectFalse("selection highlight refresh blocks missing ref", shouldRefreshVatsHighlightAttempt(true, false, 10, 10));
    ok &= expectFalse("selection highlight refresh throttles selected ref", shouldRefreshVatsHighlightAttempt(true, true, 9, 10));

    using frik::rock::selection_query_policy::shouldKeepSelectionAfterMiss;
    ok &= expectTrue("selection miss grace keeps far target briefly", shouldKeepSelectionAfterMiss(true, 14, 15, 100.0f, 50.0f));
    ok &= expectFalse("selection miss clears far target after grace", shouldKeepSelectionAfterMiss(true, 15, 15, 10.0f, 500.0f));
    ok &= expectTrue("selection miss keeps close target inside hysteresis", shouldKeepSelectionAfterMiss(false, 15, 15, 10.0f, 50.0f));
    ok &= expectFalse("selection miss clears close target outside hysteresis", shouldKeepSelectionAfterMiss(false, 15, 15, 60.0f, 50.0f));

    return ok ? 0 : 1;
}
