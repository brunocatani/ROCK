param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source {
    param([string]$Path)
    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        $failures.Add("Missing source file: $Path")
        return ''
    }
    return Get-Content -Raw -LiteralPath $fullPath
}

function Require-Pattern {
    param([string]$Path, [string]$Pattern, [string]$Message)
    if ((Read-Source $Path) -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Pattern {
    param([string]$Path, [string]$Pattern, [string]$Message)
    if ((Read-Source $Path) -match $Pattern) {
        $failures.Add($Message)
    }
}

$main = 'src/ROCKMain.cpp'
$offsets = 'src/physics-interaction/native/HavokOffsets.h'
$interactionHeader = 'src/physics-interaction/core/PhysicsInteraction.h'
$interaction = 'src/physics-interaction/core/PhysicsInteraction.cpp'
$frame = 'src/physics-interaction/core/PhysicsInteractionFrame.inl'
$frameContext = 'src/physics-interaction/core/PhysicsFrameContext.h'
$handSkeleton = 'src/physics-interaction/hand/HandSkeleton.h'
$weaponHeader = 'src/physics-interaction/weapon/TwoHandedGrip.h'
$weapon = 'src/physics-interaction/weapon/TwoHandedGrip.cpp'
$dynamicHandHeader = 'src/physics-interaction/hand/DynamicHandCollision.h'
$dynamicHand = 'src/physics-interaction/hand/DynamicHandCollision.cpp'

# Loader compatibility and executable-layout identity are separate domains.
Reject-Pattern $main `
    'RuntimeVersion\s*\(\s*\)[\s\S]{0,160}(RUNTIME_LATEST_VR|RUNTIME_VR_1_2_72)|(RUNTIME_LATEST_VR|RUNTIME_VR_1_2_72)[\s\S]{0,160}RuntimeVersion\s*\(' `
    'QueryInterface::RuntimeVersion must never be compared with a VR executable-version constant.'
Require-Pattern $main `
    'F4SEPlugin_Query[\s\S]*REL::Module::IsVR\(\)[\s\S]*F4SEPlugin_Load' `
    'Query must reject non-VR module identity without treating the F4SE compatibility value as the executable version.'
Require-Pattern $main `
    'F4SEPlugin_Load[\s\S]*F4SE::Init\(a_f4se, false\);[\s\S]{0,500}!REL::Module::IsVR\(\)[\s\S]{0,180}REL::Module::get\(\)\.version\(\)\s*!=\s*F4SE::RUNTIME_VR_1_2_72[\s\S]{0,500}Register F4SE messaging listener' `
    'Load must enforce exact FO4VR 1.2.72 module identity before any hook or listener installation.'

# Both writes target one independently verified FO4VR callsite and preserve
# separate inner/native and outer/FRIK chain pointers.
Require-Pattern $offsets `
    'kHookSite_MainLoop\s*=\s*0xD8405E[\s\S]{0,180}kFunc_MainLoopDisplacedTarget\s*=\s*0xD3C820' `
    'The main-loop hook must retain the verified FO4VR callsite and displaced native target identity.'
Require-Pattern $main `
    'hookMainLoop\(\)[\s\S]{0,1600}kFunc_MainLoopDisplacedTarget[\s\S]{0,700}decodeRelativeCallTarget[\s\S]{0,800}write_call<5>[\s\S]{0,300}s_originalGameLoopFunc' `
    'The original ROCK hook must validate the live E8 target before retaining the displaced native call.'
Require-Pattern $main `
    's_originalGameLoopFunc\s*=\s*nullptr[\s\S]{0,180}s_frikOuterGameLoopFunc\s*=\s*nullptr' `
    'The displaced native call and saved FRIK outer chain must have distinct function pointers.'
Require-Pattern $main `
    'tryHookFrikOuterGameLoop\(\)[\s\S]*decodeCommonLibAbsoluteJumpTarget[\s\S]*isAddressOwnedByModule\(terminalTarget, L"FRIK\.dll"\)[\s\S]*write_call<5>[\s\S]*s_frikOuterGameLoopFunc' `
    'The outer wrapper must resolve a CommonLib thunk, verify the terminal FRIK.dll owner, and preserve the exact prior chain.'
Require-Pattern $main `
    'onPreFrikGameFrameUpdateHook[\s\S]{0,900}refreshExternalHandWorldTransformsBeforeFrik[\s\S]{0,300}s_frikOuterGameLoopFunc\(rcx\)' `
    'Pre-FRIK providers must refresh before invoking the saved FRIK outer chain.'
Require-Pattern $main `
    'onGameFrameUpdateHook[\s\S]{0,180}\(void\)tryHookFrikOuterGameLoop\(\)[\s\S]{0,180}s_originalGameLoopFunc\(rcx\)' `
    'The existing inner hook must lazily acquire the late FRIK wrapper without changing its native displaced-call order.'

# Scheduler generation is captured once by PhysicsInteraction and stamps both
# post providers; only the immediately succeeding pre phase may consume it.
Require-Pattern $interactionHeader `
    'refreshExternalHandWorldTransformsBeforeFrik[\s\S]*_currentPreFrikSchedulerSequence' `
    'PhysicsInteraction must own the pre-FRIK scheduler generation across pre/post phases.'
Require-Pattern $frameContext `
    'preFrikSchedulerSequence' `
    'The coherent frame context must carry the scheduler generation used for post publication.'
Require-Pattern $frame `
    'frame\.preFrikSchedulerSequence\s*=\s*_currentPreFrikSchedulerSequence' `
    'Frame construction must snapshot the current pre-FRIK scheduler generation.'
Require-Pattern $interaction `
    'refreshExternalHandWorldTransformsBeforeFrik[\s\S]{0,2600}tryReconstructCalibratedHand[\s\S]{0,1800}refreshContactVisualAuthorityBeforeFrik[\s\S]{0,1000}refreshWeaponCollisionHandAuthorityBeforeFrik' `
    'The pre phase must reconstruct clean hand input, refresh contact authority, then refresh the higher-priority weapon authority.'

# Calibration cannot learn one presentation-contaminated frame.
Require-Pattern $handSkeleton `
    'persistentAuthorityFell[\s\S]{0,500}coherentCandidateSamples\s*=\s*0[\s\S]*calibrationRelationsCoherent[\s\S]*kRequiredStableCalibrationSamples[\s\S]*tryReconstructCalibratedHand' `
    'Controller calibration must quarantine the falling edge, require stable samples, and expose root-free reconstruction.'

# Weapon targets are reconstructed from current firing-wand motion and reject
# stale scheduler, generation, or firing-role state.
Require-Pattern $weaponHeader `
    'PreFrikWeaponHandAuthority[\s\S]*firingWandToHandLocal[\s\S]*weaponGenerationKey[\s\S]*sourceSchedulerSequence[\s\S]*firingHandIsLeft' `
    'Weapon pre-FRIK state must retain its wand-local target and every identity generation.'
Require-Pattern $weapon `
    'refreshWeaponCollisionHandAuthorityBeforeFrik[\s\S]*source\.weaponGenerationKey\s*==\s*currentWeaponGenerationKey[\s\S]*source\.firingHandIsLeft\s*==\s*firingHandIsLeft[\s\S]*isImmediateSuccessor[\s\S]*reconstructTargetWorld[\s\S]*WEAPON_COLLISION_HAND_TAG' `
    'Weapon hand authority must fail closed on stale identity and reconstruct through the current firing wand before FRIK.'
Require-Pattern $weapon `
    'applyWeaponCollisionResolvedAuthority[\s\S]*captureDriverToTargetLocal[\s\S]*sourceSchedulerSequence[\s\S]*preFrikSource\.valid' `
    'Successful post-solve weapon publication must capture the next pre-FRIK source generation.'

# Ordinary contact transport is scheduler-fresh and contact-normal safe, while
# fixed-surface latches remain explicitly outside this provider.
Require-Pattern $dynamicHandHeader `
    'PreFrikContactAuthority[\s\S]*sourceRawHandWorld[\s\S]*appliedDeviationWorldGame[\s\S]*sourceSchedulerSequence[\s\S]*sourceGameFrameIndex[\s\S]*solveSequence' `
    'DHC pre-FRIK state must retain the source raw hand, applied correction, and diagnostic generations.'
Require-Pattern $dynamicHand `
    'refreshContactVisualAuthorityBeforeFrik[\s\S]*surfaceLatch\.active[\s\S]*preFrikContactAuthority\s*=\s*\{\}[\s\S]*isImmediateSuccessor[\s\S]*transportContactTarget[\s\S]*dynamicHandTag' `
    'DHC must skip fixed-surface latches and transport only an immediately preceding ordinary contact target.'
Require-Pattern $dynamicHand `
    'applyExternalHandWorldTransform\([\s\S]{0,300}dynamicHandTag[\s\S]{0,1000}source\.sourceRawHandWorld\s*=\s*handInput\.rawHandWorld[\s\S]*source\.sourceSchedulerSequence\s*=[\s\S]*frame\.preFrikSchedulerSequence' `
    'Successful post contact publication must capture its clean raw source and scheduler generation.'
Require-Pattern $dynamicHand `
    'clearVisual\([\s\S]{0,220}preFrikContactAuthority\s*=\s*\{\}' `
    'Every ordinary DHC visual clear must invalidate retained pre-FRIK source state.'

if ($failures.Count -gt 0) {
    Write-Host 'Pre-FRIK hand authority source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Pre-FRIK hand authority source boundary passed.'
