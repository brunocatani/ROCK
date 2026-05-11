param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()
$Root = (Resolve-Path -LiteralPath $Root).Path

function Read-Text {
    param([string]$Path)
    return Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
}

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if ((Read-Text $Path) -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if ((Read-Text $Path) -match $Pattern) {
        $failures.Add($Message)
    }
}

$profilerFiles = @(
    'src/physics-interaction/performance/PerformanceProfiler.h',
    'src/physics-interaction/performance/PerformanceProfiler.cpp'
)

Require-Text 'src/physics-interaction/performance/PerformanceProfiler.h' 'enum class Scope[\s\S]*FrameUpdate[\s\S]*HandColliderUpdate[\s\S]*BodyColliderUpdate[\s\S]*GeneratedColliderPhysicsFlush[\s\S]*WeaponCollision[\s\S]*SelectionCasts[\s\S]*SoftContact[\s\S]*DebugOverlayPublish[\s\S]*DebugOverlayRender[\s\S]*ContactResolve[\s\S]*NativeContactCallback' 'Performance profiler scope enum must cover the planned ROCK hot-path boundaries.'
Require-Text 'src/ROCKMain.cpp' 'performance_profiler::refreshSettings\(\s*g_rockConfig\.rockPerformanceProfilerEnabled' 'Frame loop must refresh profiler settings from ROCK.ini.'
Require-Text 'src/ROCKMain.cpp' 'performance_profiler::FrameScope profilerFrame;' 'Frame loop must time the full ROCK frame update boundary.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'Scope::HandColliderUpdate' 'Hand collider update must be instrumented.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'Scope::BodyColliderUpdate' 'Body collider update must be instrumented.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'Scope::GeneratedColliderPhysicsFlush' 'Generated collider physics flush must be instrumented.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'Scope::WeaponCollision' 'Weapon collision update must be instrumented.'
Require-Text 'src/physics-interaction/native/PhysicsShapeCast.cpp' 'Scope::SelectionCasts' 'Selection shape casts must be instrumented.'
Require-Text 'src/physics-interaction/contact/SoftContactRuntime.cpp' 'Scope::SoftContact' 'Soft contact runtime must be instrumented.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'Scope::DebugOverlayPublish' 'Debug overlay publication must be instrumented.'
Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.cpp' 'Scope::DebugOverlayRender' 'Debug overlay render path must be instrumented.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionContacts.inl' 'Scope::ContactResolve' 'Frame-thread contact resolution must be instrumented.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionContacts.inl' 'Scope::NativeContactCallback' 'Native contact callback bridge must be instrumented.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionContacts.inl' 'onContactCallbackSeh[\s\S]{0,220}__try[\s\S]{0,220}onContactCallbackUnsafe' 'Native contact callback SEH must be isolated from C++ local object unwinding so MSVC can compile it.'

Require-Text 'src/physics-interaction/performance/PerformanceProfiler.cpp' 'if \(!s_settings\.enabled\.load\([\s\S]*?return;[\s\S]*?_startTicks = queryPerformanceTicks\(\);' 'ScopedTimer must return before high-resolution sampling when the profiler is disabled.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.cpp' 'void refreshSettings\(bool enabled[\s\S]*if \(!enabled\)[\s\S]*s_settings\.enabled\.store\(false' 'Disabling the profiler must clear the enabled state through refreshSettings.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.cpp' 'void addEventCount\(Scope scope' 'Profiler must support counters in addition to timed scopes.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'copyOverlayLines' 'Profiler overlay text must use the profiler snapshot instead of measuring during draw text creation.'

$allSourceFiles = Get-ChildItem -LiteralPath (Join-Path $Root 'src') -Recurse -File |
    Where-Object { $_.Extension -in @('.cpp', '.h', '.hpp', '.inl') }
foreach ($file in $allSourceFiles) {
    $relative = $file.FullName.Substring($Root.Length).TrimStart('\', '/')
    $text = Get-Content -Raw -LiteralPath $file.FullName
    if ($text -match 'QueryPerformanceCounter|QueryPerformanceFrequency') {
        $normalized = $relative -replace '/', '\'
        if ($normalized -ne 'src\physics-interaction\performance\PerformanceProfiler.cpp') {
            $failures.Add("$relative must not use QueryPerformanceCounter or QueryPerformanceFrequency; profiler timing must stay isolated.")
        }
    }
}

foreach ($file in $profilerFiles) {
    Reject-Text $file 'GetTickCount64|std::chrono::|timeGetTime|\bSleep\s*\(' "$file must not introduce alternate wall-clock APIs."
}

if ($failures.Count -gt 0) {
    Write-Host 'Performance profiler source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Performance profiler source boundary passed.'
