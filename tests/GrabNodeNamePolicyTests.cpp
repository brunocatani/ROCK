#include "physics-interaction/grab/GrabNodeNamePolicy.h"

#include <cstdio>

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
}

int main()
{
    using namespace rock::grab_node_name_policy;

    bool ok = true;
    ok &= expectTrue("default blacklist contains right marker", listContainsName(kDefaultGrabNodeNameBlacklist, "ROCK:GrabR"));
    ok &= expectTrue("default blacklist contains left marker", listContainsName(kDefaultGrabNodeNameBlacklist, "ROCK:GrabL"));
    ok &= expectTrue("list parser trims whitespace", listContainsName("  ROCK:GrabR ; CustomMarker | Other ", "CustomMarker"));
    ok &= expectFalse("list parser does not use substring matching", listContainsName("ROCK:GrabRight", "ROCK:GrabR"));
    ok &= expectTrue("right hand rejects left authored marker", isOppositeHandGrabNodeName("ROCK:GrabL", false));
    ok &= expectTrue("left hand rejects right authored marker", isOppositeHandGrabNodeName("ROCK:GrabR", true));
    ok &= expectFalse("left hand accepts left authored marker", isOppositeHandGrabNodeName("ROCK:GrabL", true));
    ok &= expectFalse("foreign namespaced grab node falls back before becoming an asset contract", sanitizeConfiguredGrabNodeName("FOREIGN:GrabR", false) == "FOREIGN:GrabR");

    return ok ? 0 : 1;
}
