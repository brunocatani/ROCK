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

function Reject-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    if ((Test-Path -LiteralPath $path) -and ((Get-Content -Raw -LiteralPath $path) -match $Pattern)) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/RockConfig.h' 'rockDebugDrawNativeScopeActivation\s*=\s*false' `
    'Native-scope visualization must have a dedicated opt-in runtime gate.'
Require-Text 'src/RockConfig.cpp' 'rockDebugDrawNativeScopeActivation\s*=\s*false[\s\S]*bDebugDrawNativeScopeActivation' `
    'The diagnostic gate must reset fail-closed and load from ROCK.ini.'
Require-Text 'src/RockConfig.h' 'rockAutoActivateScope\s*=\s*false[\s\S]*rockManualScopeHoldSeconds\s*=\s*0\.30f' `
    'Manual firing-hand hold activation must be the native-scope default.'
Require-Text 'src/RockConfig.cpp' 'bAutoActivateScope[\s\S]*fManualScopeHoldSeconds' `
    'Native-scope activation mode and hold threshold must load from ROCK.ini.'
Require-Text 'src/RockConfig.h' 'rockNativeScopeForceFiringGripFallback\s*=\s*false[\s\S]*rockNativeScopeFiringGripFallbackOffsetXGameUnits\s*=\s*0\.0f[\s\S]*rockNativeScopeFiringGripFallbackOffsetYGameUnits\s*=\s*0\.0f[\s\S]*rockNativeScopeFiringGripFallbackOffsetZGameUnits\s*=\s*0\.0f[\s\S]*rockNativeScopeFiringGripFallbackPitchDegrees\s*=\s*0\.0f[\s\S]*rockNativeScopeFiringGripFallbackYawDegrees\s*=\s*0\.0f[\s\S]*rockNativeScopeFiringGripFallbackRollDegrees\s*=\s*0\.0f' `
    'Missing-optic fallback must expose one force switch and a neutral six-degree-of-freedom Weapon-local firing-grip frame.'
Require-Text 'src/RockConfig.cpp' 'bNativeScopeForceFiringGripFallback[\s\S]*fNativeScopeFiringGripFallbackOffsetXGameUnits[\s\S]*fNativeScopeFiringGripFallbackOffsetYGameUnits[\s\S]*fNativeScopeFiringGripFallbackOffsetZGameUnits[\s\S]*fNativeScopeFiringGripFallbackPitchDegrees[\s\S]*fNativeScopeFiringGripFallbackYawDegrees[\s\S]*fNativeScopeFiringGripFallbackRollDegrees' `
    'Firing-grip fallback position and rotation controls must load from the NativeScopes section.'
Require-Text 'data/config/ROCK.ini' 'bDebugDrawNativeScopeActivation\s*=\s*false' `
    'The development config template must keep the native-scope diagnostic disabled by default.'
Require-Text 'data/mod/ROCK_Config/ROCK.ini' 'bDebugDrawNativeScopeActivation\s*=\s*false' `
    'The packaged config template must keep the native-scope diagnostic disabled by default.'
Require-Text 'src/RockConfig.h' 'rockNativeScopeOverlayOffsetXGameUnits[\s\S]*rockNativeScopeOverlayOffsetYGameUnits[\s\S]*rockNativeScopeOverlayOffsetZGameUnits[\s\S]*rockNativeScopeOverlayPitchDegrees[\s\S]*rockNativeScopeOverlayYawDegrees[\s\S]*rockNativeScopeOverlayRollDegrees' `
    'Native scope overlay placement must expose three model-local position and three rotation tuning values.'
Require-Text 'src/RockConfig.cpp' 'NATIVE_SCOPES_SECTION\s*=\s*"NativeScopes"[\s\S]*fNativeScopeOverlayOffsetXGameUnits[\s\S]*fNativeScopeOverlayOffsetYGameUnits[\s\S]*fNativeScopeOverlayOffsetZGameUnits[\s\S]*fNativeScopeOverlayPitchDegrees[\s\S]*fNativeScopeOverlayYawDegrees[\s\S]*fNativeScopeOverlayRollDegrees' `
    'Native scope overlay tuning must load from its independent NativeScopes INI section.'
foreach ($configPath in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Require-Text $configPath '\[NativeScopes\][\s\S]*bAutoActivateScope\s*=\s*false[\s\S]*fManualScopeHoldSeconds\s*=\s*0\.30' `
        'Native scope templates must default to manual A/X hold activation.'
    Require-Text $configPath '\[NativeScopes\][\s\S]*bNativeScopeForceFiringGripFallback\s*=\s*false[\s\S]*fNativeScopeFiringGripFallbackOffsetXGameUnits\s*=\s*0\.0[\s\S]*fNativeScopeFiringGripFallbackOffsetYGameUnits\s*=\s*0\.0[\s\S]*fNativeScopeFiringGripFallbackOffsetZGameUnits\s*=\s*0\.0[\s\S]*fNativeScopeFiringGripFallbackPitchDegrees\s*=\s*0\.0[\s\S]*fNativeScopeFiringGripFallbackYawDegrees\s*=\s*0\.0[\s\S]*fNativeScopeFiringGripFallbackRollDegrees\s*=\s*0\.0' `
        'Native scope templates must expose a neutral, opt-in-force six-degree-of-freedom firing-grip fallback.'
    Require-Text $configPath '\[NativeScopes\][\s\S]*fNativeScopeOverlayOffsetXGameUnits\s*=\s*0\.0[\s\S]*fNativeScopeOverlayOffsetYGameUnits\s*=\s*0\.0[\s\S]*fNativeScopeOverlayOffsetZGameUnits\s*=\s*0\.0[\s\S]*fNativeScopeOverlayPitchDegrees\s*=\s*0\.0[\s\S]*fNativeScopeOverlayYawDegrees\s*=\s*0\.0[\s\S]*fNativeScopeOverlayRollDegrees\s*=\s*0\.0' `
        'Native scope overlay template tuning must default to a neutral additive transform.'
}

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' 'enum class NativeScopeCameraWriteSource[\s\S]*PostFrikPresentationSync[\s\S]*WeaponVisualAuthority[\s\S]*struct NativeScopeActivationDebugSnapshot' `
    'Native scope activation diagnostics must retain the verified cone decision.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' 'struct NativeScopeRigidFrameState[\s\S]*weaponGenerationKey[\s\S]*nativeCameraWeaponLocal[\s\S]*cameraWeaponLocal' `
    'Native scope presentation must retain immutable native calibration separately from its generation-bound tuned target.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' 'struct NativeScopeCameraTargetPreviewSnapshot[\s\S]*equippedWeaponOwnershipKey[\s\S]*anchorSource[\s\S]*cameraWeaponLocal[\s\S]*valid' `
    'The diagnostic path must expose a pointer-free, identity-bound copy of the exact retained camera target.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' 'getNativeScopeCameraTargetPreviewSnapshot[\s\S]*_nativeScopeRigidFrame\.valid[\s\S]*_nativeScopeAnchorValid[\s\S]*_nativeScopeRigidFrame\.cameraWeaponLocal[\s\S]*\.valid\s*=\s*valid' `
    'The exact preview must fail closed unless the retained target and resolved anchor still describe one weapon generation.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'captureNativeScopeRigidFrame[\s\S]*captureRigidAnchorFrameWeaponLocal[\s\S]*nativeCameraWeaponLocal[\s\S]*rebuildNativeScopeRigidFrameTarget[\s\S]*synchronizeNativeScopePresentationAfterFrikUpdate[\s\S]*resolveRigidAnchorFrameWorld[\s\S]*NativeScopeCameraWriteSource::PostFrikPresentationSync' `
    'Post-FRIK presentation must derive fallback tuning from one immutable native frame instead of recapturing or compounding it per hand mode.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'rebuildNativeScopeRigidFrameTarget[\s\S]*nativeCameraWeaponLocal[\s\S]*FiringGripFallback[\s\S]*applyWeaponLocalRotationOffset[\s\S]*cameraWeaponLocal' `
    'Only the firing-grip fallback may add the configured weapon-axis camera rotation to immutable native calibration.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'tryResolveNativeScopeGeometryDecision[\s\S]*_nativeScopeAnchorGenerationKey\s*!=\s*currentWeaponGenerationKey[\s\S]*native_scope_activation_geometry::sample[\s\S]*native_scope_activation_geometry::isInsideCone' `
    'Native entry and exit must use the exact generation-matched resolved anchor and final weapon transform.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'refreshNativeScopeAnchor[\s\S]*rockNativeScopeForceFiringGripFallback[\s\S]*hasRightFiringHandCanonicalFrame[\s\S]*_primaryGripConfidence[\s\S]*native_scope_sight_anchor_policy::resolve[\s\S]*FiringGripFallback' `
    'Malformed optics must resolve from the generation-bound canonical or active firing grip, with an explicit forced-override path.'
Reject-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'followWeaponWorldChange\s*\(' `
    'The controller-relative rigid-delta fallback must stay removed.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'isInsideCone[\s\S]*stabilizeExitDecision[\s\S]*kNativeScopeExitConfirmationFrames' `
    'Native scope exit must reject transient outside samples without changing the cone or immediate entry decision.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'captureNativeScopeOverlayCalibration[\s\S]*ScopeParentNode[\s\S]*find1StChildNode\(scopeParent,\s*"world_scope\.nif"\)[\s\S]*captureModelRootCalibrationInCameraLocal[\s\S]*applyNativeScopeOverlayTarget[\s\S]*makeModelRootFineTuneLocal[\s\S]*resolveScopeModelRootWorld[\s\S]*resolveScopeParentWorldForModelRoot[\s\S]*worldTargetToParentLocal[\s\S]*updateTransformsDown\(scopeParent,\s*true\)' `
    'The rendered world-scope hierarchy must preserve native model orientation, apply INI tuning, and compensate the live NIF root transform at the generated sight.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'tryGetComposedNodeWorld\(scopeModelRoot,\s*immediateScopeModelRootWorld\)[\s\S]*areTransformsNearlyEqual\(immediateScopeModelRootWorld,\s*targetScopeModelRootWorld,\s*0\.01f\)' `
    'Every overlay write must immediately verify that the live world-scope model root reached its calibrated and tuned target.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'clearNativeScopeOverlayAuthority[\s\S]*lastAppliedScopeParentLocal[\s\S]*nativeScopeParentLocal[\s\S]*_nativeScopeOverlayCalibration\s*=\s*\{\}' `
    'ScopeParent authority must restore the captured native local only while ROCK still owns the last applied transform.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'tryReadNativeScopeRequestState[\s\S]*kSetting_HmdScopeOffsetY[\s\S]*kSetting_HmdScopeAngleEnterDegrees[\s\S]*kSetting_WeaponScopeAngleExitDegrees[\s\S]*kSetting_ScopeWeaponAngleExponent[\s\S]*tryResolveNativeScopeGeometryDecision' `
    'The replacement cone must preserve the live native settings and current enter/exit state.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'tryGetManualScopeDirectTransitionTarget[\s\S]*getNativeScopeResolvedAnchorSnapshot[\s\S]*resolvedAnchor\.valid[\s\S]*matchesCurrentEquippedWeapon\([\s\S]*resolvedIdentity,[\s\S]*currentIdentity' `
    'Unflagged scopes may transition only after either generated geometry or the firing-grip fallback resolved for the current equipped instance.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionProvider.inl' 'queryProviderScopeSightStateV1[\s\S]*resolvedAnchorMatchesPublication[\s\S]*resolvedAnchor\.anchorWeaponLocal[\s\S]*Flag::AnchorValid[\s\S]*Flag::BoundsValid' `
    'The V1 scope readback must expose the selected fallback anchor without claiming generated sight bounds exist.'
Require-Text 'src/ROCKMain.cpp' 'hookNativeScopeGeometryDecision[\s\S]*callBytes\[0\]\s*!=\s*0xE8[\s\S]*decodedTarget\s*!=\s*expectedTarget[\s\S]*kExpectedNativeDecisionTest[\s\S]*write_call<5>\(callSiteAddress,\s*&onNativeScopeGeometryDecision\)[\s\S]*kRockDecisionTest[\s\S]*REL::safe_write' `
    'The exact verified geometry call site and original target must be validated before patching.'
Require-Text 'src/ROCKMain.cpp' 'bool onNativeScopeGeometryDecision[\s\S]*finalGeometryDecision\s*=\s*nativeGeometryDecision[\s\S]*nativeForceDecision[\s\S]*!g_rockConfig\.rockAutoActivateScope[\s\S]*isManualScopeActivationRequested[\s\S]*tryResolveNativeScopeGeometryDecision[\s\S]*s_originalNativeScopeStateTransition\(player,\s*finalGeometryDecision\)[\s\S]*manualScopeDecisionApplied\s*\?\s*true\s*:\s*finalGeometryDecision' `
    'The hook must preserve Bethesda force priority, use held input instead of the cone in manual mode, and bypass cone-derived approach fade.'
Require-Text 'src/ROCKMain.cpp' 'configureNativeWorldScopeForManualTarget[\s\S]*kFunc_NativeWorldScopeConfigure[\s\S]*kData_NativeWorldScopeSingleton[\s\S]*kData_NativeWorldScopePrimaryVtable[\s\S]*driveManualScopeTransitionFallback[\s\S]*nativeForceDecision[\s\S]*tryGetManualScopeDirectTransitionTarget[\s\S]*isManualScopeActivationRequested\(\)[\s\S]*configureNativeWorldScopeForManualTarget[\s\S]*s_originalNativeScopeStateTransition\(player,\s*true\)[\s\S]*s_originalNativeScopeStateTransition\(player,\s*false\)' `
    'Manual hold must validate and configure native WSScope before directly transitioning an unflagged magnified scope.'
Require-Text 'src/physics-interaction/weapon/ManualScopeTargetPolicy.h' 'modelPath\.empty\(\)[\s\S]*recordName,\s*"scope"[\s\S]*modelPath,\s*"scope"' `
    'The cheap unflagged-scope path must require explicit scope identity and a physical model instead of promoting generic sights.'
Require-Text 'src/physics-interaction/weapon/ManualScopeTargetPolicy.h' 'StructuralMarkerEvidence[\s\S]*ScopeAiming[\s\S]*ScopeViewParts[\s\S]*ScopeFade[\s\S]*hasMagnifiedScopeStructure[\s\S]*isValidNativeOverlayIndex' `
    'Scopes whose OMOD names are generic must be recognized from the native magnified-scope NIF contract and a bounded overlay index.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'loadCompleteOmodModelTemplate[\s\S]*resolveEquippedManualScopeTarget[\s\S]*collectManualScopeStructuralMarkers[\s\S]*nativeScopeOverlayIndex' `
    'The equipped scope target must resolve structural NIF evidence and publish its ZOOM overlay with the collider generation.'
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' 'consumeRawButtonState\(true,\s*input_remap_policy::kOpenVrAcceptButtonId\)[\s\S]*consumeRawButtonState\(false,\s*input_remap_policy::kOpenVrAcceptButtonId\)[\s\S]*manual_scope_input_policy::update[\s\S]*decision\.scopeRequested[\s\S]*decision\.dispatchReload' `
    'Manual scope and release-time reload must share one physical firing-hand A/X gesture classifier.'
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' 'shouldDeferFiringHandActivateForManualScope\(inputEvent\)[\s\S]*markInputEventStopped\(inputEvent\)[\s\S]*return;' `
    'Primary-wand press-time reload must be deferred while manual scope classifies the hold.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kHookSite_NativeScopeGeometryDecision\s*=\s*0xEF851F[\s\S]*kPatchSite_NativeScopePostDecisionTest\s*=\s*0xEF8528[\s\S]*kFunc_NativeScopeStateTransition\s*=\s*0xEFAA60[\s\S]*kFunc_NativeWorldScopeConfigure\s*=\s*0xC8DC60[\s\S]*kData_NativeWorldScopeSingleton\s*=\s*0x5ACBF58[\s\S]*kData_NativeWorldScopePrimaryVtable\s*=\s*0x2D68718[\s\S]*kPlayerCharacter_NativeScopeForceDecisionMask\s*=\s*0x08' `
    'The production hook must retain the independently raw-disassembly-verified FO4VR scope boundary constants.'
Require-Text 'src/ROCKMain.cpp' 's_originalGameLoopFunc\(rcx\);[\s\S]*synchronizeNativeScopePresentationAfterFrikUpdate\(\);[\s\S]*onFrameUpdate\(\);' `
    'Presentation must synchronize after hFRIK and before ROCK final weapon authority.'
Reject-Text 'src/ROCKMain.cpp' 'prepareNativeScopeCameraForGameUpdate|finalizeNativeScopeOverlayAfterGameUpdate' `
    'The disproven pre/post displaced-call scope handoff must not remain.'

$overlay = 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl'
Require-Text $overlay 'drawNativeScopeActivation\s*=\s*g_rockConfig\.rockDebugDrawNativeScopeActivation' `
    'Overlay publication must be independently gated by the native-scope diagnostic setting.'
Require-Text $overlay 'primaryWeaponScopeCamera[\s\S]*scopeCamera->world[\s\S]*composeTransforms\(scopeCameraParent->world,\s*scopeCamera->local\)' `
    'The overlay must distinguish the engine node stored world from its parent/local recomposition.'
Require-Text $overlay 'getNativeScopeSightAnchorSnapshot\(\)[\s\S]*sightBoundsMinWeaponLocal[\s\S]*kBoundsEdges[\s\S]*NativeScopeSightBounds' `
    'The generated Sight union and rear-center anchor must be visible in weapon world space.'
Require-Text $overlay 'getNativeScopeCameraDebugSnapshot\(\)[\s\S]*NativeScopePreWriteCamera[\s\S]*NativeScopeImmediateReadback' `
    'The overlay must expose the recorded pre-write and immediate-readback handoff stages.'
Require-Text $overlay 'getNativeScopeResolvedAnchorSnapshot[\s\S]*FIRING GRIP FALLBACK ANCHOR[\s\S]*scopeAnchorSourceName[\s\S]*writeSnapshot\.anchorSource' `
    'The overlay must expose the selected generated-sight or firing-grip anchor used by the actual camera write.'
Require-Text $overlay 'getNativeScopeCameraTargetPreviewSnapshot\(\)[\s\S]*matchesCurrentEquippedWeapon\([\s\S]*resolveRigidAnchorFrameWorld\([\s\S]*targetPreviewSnapshot\.cameraWeaponLocal[\s\S]*targetFromResolvedPreview\s*=\s*true' `
    'The pre-activation visualizer must resolve the same identity-bound weapon-local target used by the camera writer.'
Require-Text $overlay 'FALLBACK PREVIEW ORIGIN[\s\S]*FALLBACK AIM \(\+X\)[\s\S]*FALLBACK UP \(\+Z\)[\s\S]*scopeMenuIndependent=yes' `
    'Fallback tuning must show a persistent origin, pointing direction, and roll-readable up guide while ScopeMenu is closed.'
Reject-Text $overlay 'applyWeaponLocalRotationOffset\s*\(' `
    'The visualizer must consume the retained production target instead of duplicating fallback rotation math.'
Require-Text $overlay 'fallback tune:[\s\S]*rockNativeScopeFiringGripFallbackOffsetXGameUnits[\s\S]*rockNativeScopeFiringGripFallbackPitchDegrees[\s\S]*rockNativeScopeFiringGripFallbackYawDegrees[\s\S]*rockNativeScopeFiringGripFallbackRollDegrees' `
    'The in-game diagnostic panel must expose the active firing-grip fallback position and rotation tuning.'
Require-Text $overlay 'scopeWriteSourceName[\s\S]*post-frik-presentation-sync[\s\S]*weapon-visual-authority[\s\S]*writeSnapshot\.writeSource' `
    'The in-game panel must distinguish presentation synchronization from final weapon authority.'
Require-Text $overlay 'hmdPositionWorld[\s\S]*NativeScopeHmd[\s\S]*HMD->live[\s\S]*HMD->target' `
    'The headset relationship to the live and intended activation anchors must be visible and quantified.'
Require-Text $overlay 'getNativeScopeActivationDebugSnapshot[\s\S]*nativeGeometryDecision[\s\S]*rockGeometryDecision[\s\S]*hmdAngleDegrees[\s\S]*weaponAngleDegrees[\s\S]*weaponAngleWidening' `
    'The verified native-cone sample, limits, and replacement decision must be visible at runtime.'

if ($failures.Count -gt 0) {
    Write-Host 'NativeScopeActivationDebugSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'NativeScopeActivationDebugSourceTests passed.' -ForegroundColor Green
