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
$handHeader = 'src/physics-interaction/hand/Hand.h'
$visualBridge = 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h'
$equipHeader = 'src/physics-interaction/weapon/EquipVisualBridge.h'
$equip = 'src/physics-interaction/weapon/EquipVisualBridge.cpp'
$transitionHeader = 'src/physics-interaction/weapon/EquippedWeaponTransitionCoordinator.h'
$weaponHeader = 'src/physics-interaction/weapon/TwoHandedGrip.h'
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
    'refreshExternalHandWorldTransformsBeforeFrik[\s\S]*tryReconstructCalibratedHand[\s\S]*refreshGrabVisualAuthorityBeforeFrik[\s\S]*refreshHandVisualAuthorityBeforeFrik[\s\S]*refreshContactVisualAuthorityBeforeFrik[\s\S]*refreshRetainedHandVisualAuthoritiesBeforeFrik[\s\S]*refreshWeaponCollisionHandAuthorityBeforeFrik' `
    'The pre phase must reconstruct clean hand input and refresh every retained hand-world provider before FRIK.'

# Authority is latched once per scheduler generation; provider publications or
# clears later in the same ROCK frame cannot flap the physics input source.
Require-Pattern $interactionHeader `
    '_persistentFrikHandInputIsolationActive[\s\S]*_persistentFrikHandInputIsolationSequence[\s\S]*_currentPreFrikSchedulerSequence' `
    'PhysicsInteraction must retain the authority value and the scheduler generation that sampled it.'
Require-Pattern $interaction `
    '_persistentFrikHandInputIsolationSequence\[handIndex\][\s\S]{0,1200}hasPublishedExternalHandWorldTransform[\s\S]{0,1200}persistentWorldAuthorityPublished\s*=\s*_persistentFrikHandInputIsolationActive\[handIndex\]' `
    'Hand input isolation must sample publication state only at a scheduler-generation edge.'
Require-Pattern $visualBridge `
    'hasPublishedExternalHandWorldTransform\(const char\* tag, Hand hand\)[\s\S]{0,300}findTrackedHandWorldPublication' `
    'Pre-FRIK providers must query their exact tag so one owner cannot resurrect another owner''s cleared claim.'

# Calibration cannot learn one presentation-contaminated frame.
Require-Pattern $handSkeleton `
    'persistentAuthorityFell[\s\S]{0,500}coherentCandidateSamples\s*=\s*0[\s\S]*calibrationRelationsCoherent[\s\S]*kRequiredStableCalibrationSamples[\s\S]*tryReconstructCalibratedHand' `
    'Controller calibration must quarantine the falling edge, require stable samples, and expose root-free reconstruction.'
Require-Pattern $handSkeleton `
    'if\s*\(!sourceSkeleton\s*\|\|\s*!sourceBoneTree\)[\s\S]{0,220}if\s*\(!persistentWorldAuthorityPublished\)[\s\S]{0,220}!hasRootFlattenedHand[\s\S]*reconstructedHandWorld' `
    'A committed persistent controller reconstruction must not require the deferred root-flattened hand to exist.'

# Regular grabs follow the current held scene object, never player/controller
# locomotion. Returns follow the clean current raw hand.
Require-Pattern $handHeader `
    'PreFrikGrabVisualAuthority[\s\S]*NiPointer<RE::NiAVObject>\s+heldNode[\s\S]*heldNodeToHandLocal[\s\S]*heldBodyId[\s\S]*constraintId[\s\S]*sourceSchedulerSequence' `
    'Regular grab pre-FRIK state must own the held node and bind it to exact body, constraint, and scheduler identities.'

# Equip handoff follows whichever retained weapon graph is currently visible.
Require-Pattern $equipHeader `
    '_preFrikHandWorldAnchor[\s\S]*_preFrikAnchorToHandLocal[\s\S]*_preFrikSourceSchedulerSequence[\s\S]*_preFrikHandWorldAuthorityValid' `
    'Equip handoff must own an anchor-local hand target and scheduler generation.'
Require-Pattern $equip `
    'publishHandWorldHandoff[\s\S]*captureDriverToTargetLocal[\s\S]*refreshHandVisualAuthorityBeforeFrik[\s\S]*isImmediateSuccessor[\s\S]*reconstructTargetWorld' `
    'Equip handoff must capture post authority and reconstruct it from the current retained weapon graph before FRIK.'
Require-Pattern $transitionHeader `
    'refreshHandVisualAuthorityBeforeFrik[\s\S]{0,180}_bridge\.refreshHandVisualAuthorityBeforeFrik' `
    'The transition coordinator must expose the equip bridge pre-FRIK provider to PhysicsInteraction.'

# Weapon targets are reconstructed from each physical hand's current FRIK
# driver and reject stale scheduler, generation, or firing-role state.
Require-Pattern $weaponHeader `
    'PreFrikWeaponHandAuthority[\s\S]*driverToHandLocal[\s\S]*weaponGenerationKey[\s\S]*sourceSchedulerSequence[\s\S]*firingHandIsLeft' `
    'Weapon pre-FRIK state must retain its physical-driver-local target and every identity generation.'
Require-Pattern $weaponHeader `
    'RetainedHandAuthorityKind[\s\S]*PrimaryGrip[\s\S]*SupportGrip[\s\S]*GunstockAlignment[\s\S]*Return[\s\S]*PreFrikRetainedHandAuthority' `
    'Every transportable retained weapon-hand role must have explicit pre-FRIK source state.'

# Ordinary contact transport is scheduler-fresh and contact-normal safe, while
# fixed-surface latches remain explicitly outside this provider.
Require-Pattern $dynamicHandHeader `
    'PreFrikContactAuthority[\s\S]*sourceRawHandWorld[\s\S]*sourceSolvedHandWorld[\s\S]*appliedDeviationWorldGame[\s\S]*sourceSchedulerSequence[\s\S]*sourceGameFrameIndex[\s\S]*solveSequence' `
    'DHC pre-FRIK state must retain the raw and rigid solved hand transforms plus diagnostic generations.'
Require-Pattern $dynamicHand `
    'refreshContactVisualAuthorityBeforeFrik[\s\S]*surfaceLatch\.active[\s\S]*preFrikContactAuthority\s*=\s*\{\}[\s\S]*isImmediateSuccessor[\s\S]*transportRigidContactTarget[\s\S]*dynamicHandTag' `
    'DHC must skip fixed-surface latches and transport only an immediately preceding rigid contact target.'
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
