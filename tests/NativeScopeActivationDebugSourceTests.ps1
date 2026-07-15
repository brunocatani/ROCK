param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path)) {
        $failures.Add("$RelativePath`: missing file for text check")
        return
    }

    $text = Get-Content -Raw -LiteralPath $path
    if ($text -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/RockConfig.h' 'rockDebugDrawNativeScopeActivation\s*=\s*false' `
    'Native-scope visualization must have a dedicated opt-in runtime gate.'
Require-Text 'src/RockConfig.cpp' 'rockDebugDrawNativeScopeActivation\s*=\s*false[\s\S]*bDebugDrawNativeScopeActivation' `
    'The diagnostic gate must reset fail-closed and load from ROCK.ini.'
Require-Text 'data/config/ROCK.ini' 'bDebugDrawNativeScopeActivation\s*=\s*false' `
    'The development config template must keep the native-scope diagnostic disabled by default.'
Require-Text 'data/mod/ROCK_Config/ROCK.ini' 'bDebugDrawNativeScopeActivation\s*=\s*false' `
    'The packaged config template must keep the native-scope diagnostic disabled by default.'
Require-Text 'src/RockConfig.h' 'rockNativeScopeOverlayOffsetXGameUnits[\s\S]*rockNativeScopeOverlayOffsetYGameUnits[\s\S]*rockNativeScopeOverlayOffsetZGameUnits[\s\S]*rockNativeScopeOverlayPitchDegrees[\s\S]*rockNativeScopeOverlayYawDegrees[\s\S]*rockNativeScopeOverlayRollDegrees' `
    'Native scope overlay placement must expose three model-local position and three rotation tuning values.'
Require-Text 'src/RockConfig.cpp' 'NATIVE_SCOPES_SECTION\s*=\s*"NativeScopes"[\s\S]*fNativeScopeOverlayOffsetXGameUnits[\s\S]*fNativeScopeOverlayOffsetYGameUnits[\s\S]*fNativeScopeOverlayOffsetZGameUnits[\s\S]*fNativeScopeOverlayPitchDegrees[\s\S]*fNativeScopeOverlayYawDegrees[\s\S]*fNativeScopeOverlayRollDegrees' `
    'Native scope overlay tuning must load from its independent NativeScopes INI section.'
foreach ($configPath in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Require-Text $configPath '\[NativeScopes\][\s\S]*fNativeScopeOverlayOffsetXGameUnits\s*=\s*0\.0[\s\S]*fNativeScopeOverlayOffsetYGameUnits\s*=\s*0\.0[\s\S]*fNativeScopeOverlayOffsetZGameUnits\s*=\s*0\.0[\s\S]*fNativeScopeOverlayPitchDegrees\s*=\s*0\.0[\s\S]*fNativeScopeOverlayYawDegrees\s*=\s*0\.0[\s\S]*fNativeScopeOverlayRollDegrees\s*=\s*0\.0' `
        'Native scope overlay template tuning must default to a neutral additive transform.'
}

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' 'enum class NativeScopeCameraWriteSource[\s\S]*PreNativeGameUpdate[\s\S]*WeaponVisualAuthority[\s\S]*struct NativeScopeCameraDebugSnapshot[\s\S]*writeSource[\s\S]*usedSightAnchor[\s\S]*usedLastRenderedRockWeaponFrame[\s\S]*cameraWorldBefore[\s\S]*targetCameraWorld[\s\S]*immediateCameraWorldAfter' `
    'The handoff diagnostic must retain value snapshots for pre-write, target, and immediate readback stages.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'NativeScopeCameraFollowResult applyNativeScopeCameraFollow[\s\S]*result\.targetCameraWorld\s*=\s*targetCameraWorld[\s\S]*scopeCamera->local\s*=\s*targetCameraLocal[\s\S]*immediateCameraWorld\s*=\s*scopeCamera->world' `
    'The diagnostic must observe the stored camera world immediately after the real native-camera write.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'void TwoHandedGrip::prepareNativeScopeCameraForGameUpdate[\s\S]*_nativeScopeSightAnchorWeaponNode\s*!=\s*weaponNode[\s\S]*shouldUseLastRenderedRockWeaponFrame[\s\S]*activationWeaponWorld[\s\S]*applyNativeScopeCameraFollow\(capture,\s*activationWeaponWorld,\s*&_nativeScopeSightAnchorWeaponLocal\)[\s\S]*NativeScopeCameraWriteSource::PreNativeGameUpdate' `
    'Pre-native scope detection must use the last rendered ROCK weapon frame during matching two-hand ownership and still anchor it to generated Sight geometry.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' 'NativeScopeOverlayPendingHandoff[\s\S]*nativeCameraWorldBefore[\s\S]*correctedCameraWorld[\s\S]*NativeScopeOverlayCalibrationState[\s\S]*scopeParentIdentity[\s\S]*scopeModelRootIdentity[\s\S]*scopeModelRootLocal[\s\S]*scopeModelRootCalibrationInCameraLocal[\s\S]*nativeScopeParentLocal[\s\S]*lastAppliedScopeParentLocal' `
    'The native overlay handoff must retain the native camera value, generation-keyed node identities, model orientation calibration, and rollback state.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'captureNativeScopeOverlayCalibration[\s\S]*ScopeParentNode[\s\S]*find1StChildNode\(scopeParent,\s*"world_scope\.nif"\)[\s\S]*captureModelRootCalibrationInCameraLocal[\s\S]*applyNativeScopeOverlayTarget[\s\S]*makeModelRootFineTuneLocal[\s\S]*resolveScopeModelRootWorld[\s\S]*resolveScopeParentWorldForModelRoot[\s\S]*worldTargetToParentLocal[\s\S]*updateTransformsDown\(scopeParent,\s*true\)' `
    'The rendered world-scope hierarchy must preserve native model orientation, apply INI tuning, and compensate the live NIF root transform at the generated sight.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'tryGetComposedNodeWorld\(scopeModelRoot,\s*immediateScopeModelRootWorld\)[\s\S]*areTransformsNearlyEqual\(immediateScopeModelRootWorld,\s*targetScopeModelRootWorld,\s*0\.01f\)' `
    'Every overlay write must immediately verify that the live world-scope model root reached its calibrated and tuned target.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'clearNativeScopeOverlayAuthority[\s\S]*lastAppliedScopeParentLocal[\s\S]*nativeScopeParentLocal[\s\S]*finalizeNativeScopeOverlayAfterGameUpdate' `
    'ScopeParent authority must restore the captured native local only while ROCK still owns the last applied transform.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'void PhysicsInteraction::prepareNativeScopeCameraForGameUpdate\(\)[\s\S]*_twoHandedGrip\.prepareNativeScopeCameraForGameUpdate\([\s\S]*_weaponCollision\.getCurrentWeaponGenerationKey\(\)' `
    'The main-loop boundary must consume only the current weapon node and prior completed generation snapshot.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'void PhysicsInteraction::finalizeNativeScopeOverlayAfterGameUpdate\(\)[\s\S]*_twoHandedGrip\.finalizeNativeScopeOverlayAfterGameUpdate\([\s\S]*_weaponCollision\.getCurrentWeaponGenerationKey\(\)' `
    'The post-native boundary must retain the same current weapon-generation guard as the camera handoff.'
Require-Text 'src/ROCKMain.cpp' 'prepareNativeScopeCameraForGameUpdate\(\);[\s\S]*s_originalGameLoopFunc\(rcx\);[\s\S]*finalizeNativeScopeOverlayAfterGameUpdate\(\);[\s\S]*onFrameUpdate\(\);' `
    'The camera must publish before native consumption, ScopeParent after native ownership, and both before ROCK post-frame authority.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'NativeScopeCameraWriteSource::WeaponVisualAuthority[\s\S]*scopeCameraFollow[\s\S]*scopeCameraResult[\s\S]*sightAnchorWeaponLocal\s*!=\s*nullptr' `
    'Later weapon authority writes must remain source-aware and report whether they consumed generated sight geometry.'

$overlay = 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl'
Require-Text $overlay 'drawNativeScopeActivation\s*=\s*g_rockConfig\.rockDebugDrawNativeScopeActivation' `
    'Overlay publication must be independently gated by the native-scope diagnostic setting.'
Require-Text $overlay 'primaryWeaponScopeCamera[\s\S]*scopeCamera->world[\s\S]*composeTransforms\(scopeCameraParent->world,\s*scopeCamera->local\)' `
    'The overlay must distinguish the engine node stored world from its parent/local recomposition.'
Require-Text $overlay 'getNativeScopeSightAnchorSnapshot\(\)[\s\S]*sightBoundsMinWeaponLocal[\s\S]*kBoundsEdges[\s\S]*NativeScopeSightBounds' `
    'The generated Sight union and rear-center anchor must be visible in weapon world space.'
Require-Text $overlay 'getNativeScopeCameraDebugSnapshot\(\)[\s\S]*NativeScopePreWriteCamera[\s\S]*NativeScopeImmediateReadback' `
    'The overlay must expose the recorded pre-write and immediate-readback handoff stages.'
Require-Text $overlay 'writeSnapshot\.usedSightAnchor' `
    'The overlay must report whether the actual authority write consumed generated sight geometry.'
Require-Text $overlay 'writeSnapshot\.usedLastRenderedRockWeaponFrame' `
    'The overlay must report whether pre-native scope detection consumed ROCKs last rendered weapon frame.'
Require-Text $overlay 'scopeWriteSourceName[\s\S]*pre-native-game-update[\s\S]*weapon-visual-authority[\s\S]*writeSnapshot\.writeSource' `
    'The in-game panel must distinguish the pre-native baseline from a later weapon-authority write.'
Require-Text $overlay 'hmdPositionWorld[\s\S]*NativeScopeHmd[\s\S]*HMD->live[\s\S]*HMD->target' `
    'The headset relationship to the live and intended activation anchors must be visible and quantified.'
Require-Text $overlay 'not verified engine cone thresholds' `
    'Axis guides must explicitly avoid misrepresenting an unverified engine activation predicate.'

if ($failures.Count -gt 0) {
    Write-Host 'NativeScopeActivationDebugSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'NativeScopeActivationDebugSourceTests passed.' -ForegroundColor Green
