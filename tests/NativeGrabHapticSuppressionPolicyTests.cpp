#include "physics-interaction/native/NativeGrabHapticSuppressionPolicy.h"

#include <cstdio>
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

    bool expectEqual(const char* label, std::string_view actual, std::string_view expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected '%.*s' got '%.*s'\n",
            label,
            static_cast<int>(expected.size()),
            expected.data(),
            static_cast<int>(actual.size()),
            actual.data());
        return false;
    }
}

int main()
{
    using namespace rock::native_grab_haptic_suppression;

    bool ok = true;

    ok &= expectEqual("rollover rumble setting name", kSuppressedIniSettings[0], "bRumbleOnRollover:VRInterface");
    ok &= expectEqual("hover intensity setting name", kSuppressedIniSettings[1], "fVRInputHoverRumbleIntensity:VRWand");
    ok &= expectEqual("hover duration setting name", kSuppressedIniSettings[2], "fVRInputHoverRumbleDuration:VRWand");

    ok &= expectTrue("enabled ROCK suppresses native hover haptics",
        shouldSuppressNativeGrabHoverHaptics(RuntimeInput{
            .rockEnabled = true,
            .suppressionEnabled = true,
        }));

    ok &= expectFalse("disabled ROCK does not suppress native hover haptics",
        shouldSuppressNativeGrabHoverHaptics(RuntimeInput{
            .rockEnabled = false,
            .suppressionEnabled = true,
        }));

    ok &= expectFalse("disabled feature does not suppress native hover haptics",
        shouldSuppressNativeGrabHoverHaptics(RuntimeInput{
            .rockEnabled = true,
            .suppressionEnabled = false,
        }));

    ok &= expectTrue("applied suppression restores when feature turns off",
        shouldRestoreNativeGrabHoverHaptics(true,
            RuntimeInput{
                .rockEnabled = true,
                .suppressionEnabled = false,
            }));

    ok &= expectFalse("unapplied suppression has nothing to restore",
        shouldRestoreNativeGrabHoverHaptics(false,
            RuntimeInput{
                .rockEnabled = true,
                .suppressionEnabled = false,
            }));

    ok &= expectFalse("active suppression should not restore",
        shouldRestoreNativeGrabHoverHaptics(true,
            RuntimeInput{
                .rockEnabled = true,
                .suppressionEnabled = true,
            }));

    return ok ? 0 : 1;
}
