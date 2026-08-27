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
        .suppressionEnabled = true,
    };
    ok &= expectTrue("enabled suppression owns VRInput settings", shouldSuppressNativeMeleeRuntimeSettings(activeRuntime));
    ok &= expectFalse("active runtime suppression does not restore", shouldRestoreNativeMeleeRuntimeSettings(true, activeRuntime));

    auto disabledMasterRuntime = activeRuntime;
    disabledMasterRuntime.suppressionEnabled = false;
    ok &= expectFalse("disabled suppression leaves runtime settings native", shouldSuppressNativeMeleeRuntimeSettings(disabledMasterRuntime));
    ok &= expectTrue("disabled suppression restores owned runtime settings", shouldRestoreNativeMeleeRuntimeSettings(true, disabledMasterRuntime));
    ok &= expectFalse("disabled suppression has nothing to restore before ownership", shouldRestoreNativeMeleeRuntimeSettings(false, disabledMasterRuntime));

    auto missingHooksRuntime = activeRuntime;
    missingHooksRuntime.hooksInstalled = false;
    ok &= expectTrue("missing hooks restore owned runtime settings", shouldRestoreNativeMeleeRuntimeSettings(true, missingHooksRuntime));

    const NativeMeleePolicyInput nativePlayer{
        .suppressionActive = false,
        .actorIsPlayer = true,
    };
    ok &= expectEqual("disabled suppression passes WeaponSwing through",
        evaluateNativeMeleeSuppression(NativeMeleeEvent::WeaponSwing, nativePlayer).action,
        NativeMeleeSuppressionAction::CallNative);
    ok &= expectEqual("disabled suppression passes HitFrame through",
        evaluateNativeMeleeSuppression(NativeMeleeEvent::HitFrame, nativePlayer).action,
        NativeMeleeSuppressionAction::CallNative);

    ok &= expectEqual("disabled suppression passes RightStick gate through",
        evaluateNativeMeleeInputGate(NativeMeleeInputGatePolicyInput{
            .suppressionActive = false,
            .inputEvent = NativeMeleeInputEvent::RightStick,
        }).action,
        NativeMeleeInputGateAction::CallNative);

    ok &= expectEqual("disabled suppression passes VR melee impact through",
        evaluateNativeMeleeImpactSuppression(NativeMeleeImpactPolicyInput{
            .suppressionActive = false,
            .actorIsPlayer = true,
        }).action,
        NativeMeleeImpactAction::CallNative);

    const NativeMeleePolicyInput suppressedPlayer{
        .suppressionActive = true,
        .actorIsPlayer = true,
    };
    ok &= expectEqual("enabled suppression handles player WeaponSwing",
        evaluateNativeMeleeSuppression(NativeMeleeEvent::WeaponSwing, suppressedPlayer).action,
        NativeMeleeSuppressionAction::ReturnHandled);
    ok &= expectEqual("enabled suppression handles player HitFrame",
        evaluateNativeMeleeSuppression(NativeMeleeEvent::HitFrame, suppressedPlayer).action,
        NativeMeleeSuppressionAction::ReturnHandled);
    ok &= expectEqual("enabled suppression blocks RightStick melee gate",
        evaluateNativeMeleeInputGate(NativeMeleeInputGatePolicyInput{
            .suppressionActive = true,
            .inputEvent = NativeMeleeInputEvent::RightStick,
        }).action,
        NativeMeleeInputGateAction::ReturnFalse);
    ok &= expectEqual("enabled suppression leaves primary attack input native",
        evaluateNativeMeleeInputGate(NativeMeleeInputGatePolicyInput{
            .suppressionActive = true,
            .inputEvent = NativeMeleeInputEvent::PrimaryAttack,
        }).action,
        NativeMeleeInputGateAction::CallNative);
    ok &= expectEqual("enabled suppression blocks player VR melee impact",
        evaluateNativeMeleeImpactSuppression(NativeMeleeImpactPolicyInput{
            .suppressionActive = true,
            .actorIsPlayer = true,
        }).action,
        NativeMeleeImpactAction::Suppress);

    const NativeMeleePolicyInput suppressedNpc{
        .suppressionActive = true,
        .actorIsPlayer = false,
    };
    ok &= expectEqual("enabled suppression preserves NPC WeaponSwing",
        evaluateNativeMeleeSuppression(NativeMeleeEvent::WeaponSwing, suppressedNpc).action,
        NativeMeleeSuppressionAction::CallNative);
    ok &= expectEqual("enabled suppression preserves NPC VR melee impact",
        evaluateNativeMeleeImpactSuppression(NativeMeleeImpactPolicyInput{
            .suppressionActive = true,
            .actorIsPlayer = false,
        }).action,
        NativeMeleeImpactAction::CallNative);

    return ok ? 0 : 1;
}
