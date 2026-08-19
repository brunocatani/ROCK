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
$interactionHeader = 'src/physics-interaction/core/PhysicsInteraction.h'
$frameContext = 'src/physics-interaction/core/PhysicsFrameContext.h'
$handHeader = 'src/physics-interaction/hand/Hand.h'
$visualBridge = 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h'

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

# Authority is latched once per scheduler generation; provider publications or
# clears later in the same ROCK frame cannot flap the physics input source.
Require-Pattern $interactionHeader `
    '_persistentFrikHandInputIsolationActive[\s\S]*_persistentFrikHandInputIsolationSequence[\s\S]*_currentPreFrikSchedulerSequence' `
    'PhysicsInteraction must retain the authority value and the scheduler generation that sampled it.'
Require-Pattern $visualBridge `
    'hasPublishedExternalHandWorldTransform\(const char\* tag, Hand hand\)[\s\S]{0,300}findTrackedHandWorldPublication' `
    'Pre-FRIK providers must query their exact tag so one owner cannot resurrect another owner''s cleared claim.'

# Calibration cannot learn one presentation-contaminated frame.

# Regular grabs follow the current held scene object, never player/controller
# locomotion. Returns follow the clean current raw hand.
Require-Pattern $handHeader `
    'PreFrikGrabVisualAuthority[\s\S]*NiPointer<RE::NiAVObject>\s+heldNode[\s\S]*heldNodeToHandLocal[\s\S]*heldBodyId[\s\S]*constraintId[\s\S]*sourceSchedulerSequence' `
    'Regular grab pre-FRIK state must own the held node and bind it to exact body, constraint, and scheduler identities.'

# Equip handoff follows whichever retained weapon graph is currently visible.

# Weapon targets are reconstructed from each physical hand's current FRIK
# driver and reject stale scheduler, generation, or firing-role state.

# Ordinary contact transport is scheduler-fresh and contact-normal safe, while
# fixed-surface latches remain explicitly outside this provider.

if ($failures.Count -gt 0) {
    Write-Host 'Pre-FRIK hand authority source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Pre-FRIK hand authority source boundary passed.'
