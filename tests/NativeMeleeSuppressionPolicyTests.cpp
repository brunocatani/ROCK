#include "physics-interaction/NativeMeleeSuppressionPolicy.h"

#include <cstdio>
#include <limits>
#include <string_view>
#include <type_traits>

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

    template <class Enum>
        requires std::is_enum_v<Enum>
    bool expectEqual(const char* label, Enum actual, Enum expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected %d got %d\n", label, static_cast<int>(expected), static_cast<int>(actual));
        return false;
    }
}

int main()
{
    using namespace rock::native_melee_suppression;

    bool ok = true;

    ok &= expectEqual("velocity check setting", kVelocityCheckSetting, "bMeleeVelocityCheck:VRInput");
    ok &= expectEqual("linear threshold setting", kLinearVelocityThresholdSetting, "fMeleeLinearVelocityThreshold:VRInput");
    ok &= expectEqual("angular threshold setting", kAngularVelocityThresholdSetting, "fMeleeAngularVelocityThreshold:VRInput");
    ok &= expectTrue("suppression threshold is unreachable during ordinary play", kSuppressedVelocityThreshold >= 1.0e9f);
    const float quietNan = std::numeric_limits<float>::quiet_NaN();
    ok &= expectTrue("exact restore compares identical NaN payloads by bits", sameRuntimeFloatBits(quietNan, quietNan));
    ok &= expectFalse("exact restore preserves signed zero", sameRuntimeFloatBits(0.0f, -0.0f));

    const NativeMeleeRuntimeSettingPolicyInput activeRuntime{
        .hooksInstalled = true,
        .rockEnabled = true,
        .suppressionEnabled = true,
        .fullSuppression = true,
    };
    ok &= expectTrue("fully enabled runtime suppression owns VRInput settings", shouldSuppressNativeMeleeRuntimeSettings(activeRuntime));
    ok &= expectFalse("active runtime suppression does not restore", shouldRestoreNativeMeleeRuntimeSettings(true, activeRuntime));

    auto disabledMasterRuntime = activeRuntime;
    disabledMasterRuntime.suppressionEnabled = false;
    ok &= expectFalse("master fallback disables runtime setting suppression", shouldSuppressNativeMeleeRuntimeSettings(disabledMasterRuntime));
    ok &= expectTrue("master fallback restores owned runtime settings", shouldRestoreNativeMeleeRuntimeSettings(true, disabledMasterRuntime));
    ok &= expectFalse("master fallback has nothing to restore before ownership", shouldRestoreNativeMeleeRuntimeSettings(false, disabledMasterRuntime));

    auto disabledRockRuntime = activeRuntime;
    disabledRockRuntime.rockEnabled = false;
    ok &= expectTrue("disabling ROCK restores owned runtime settings", shouldRestoreNativeMeleeRuntimeSettings(true, disabledRockRuntime));

    auto partialRuntime = activeRuntime;
    partialRuntime.fullSuppression = false;
    ok &= expectTrue("partial mode restores full-suppression runtime settings", shouldRestoreNativeMeleeRuntimeSettings(true, partialRuntime));

    auto missingHooksRuntime = activeRuntime;
    missingHooksRuntime.hooksInstalled = false;
    ok &= expectTrue("missing hooks restore owned runtime settings", shouldRestoreNativeMeleeRuntimeSettings(true, missingHooksRuntime));

    const NativeMeleePolicyInput masterFallbackHookInput{
        .rockEnabled = true,
        .suppressionEnabled = false,
        .fullSuppression = true,
        .suppressWeaponSwing = true,
        .suppressHitFrame = true,
        .actorIsPlayer = true,
        .physicalSwingActive = true,
    };
    ok &= expectEqual("master fallback passes WeaponSwing through",
        evaluateNativeMeleeSuppression(NativeMeleeEvent::WeaponSwing, masterFallbackHookInput).action,
        NativeMeleeSuppressionAction::CallNative);
    ok &= expectEqual("master fallback passes HitFrame through",
        evaluateNativeMeleeSuppression(NativeMeleeEvent::HitFrame, masterFallbackHookInput).action,
        NativeMeleeSuppressionAction::CallNative);

    ok &= expectEqual("master fallback passes RightStick gate through",
        evaluateNativeMeleeInputGate(NativeMeleeInputGatePolicyInput{
            .rockEnabled = true,
            .suppressionEnabled = false,
            .fullSuppression = true,
            .inputEvent = NativeMeleeInputEvent::RightStick,
        }).action,
        NativeMeleeInputGateAction::CallNative);

    ok &= expectEqual("master fallback passes VR melee impact through",
        evaluateNativeMeleeImpactSuppression(NativeMeleeImpactPolicyInput{
            .rockEnabled = true,
            .suppressionEnabled = false,
            .fullSuppression = true,
            .actorIsPlayer = true,
        }).action,
        NativeMeleeImpactAction::CallNative);

    return ok ? 0 : 1;
}
