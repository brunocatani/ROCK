param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source([string]$RelativePath) {
    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        $failures.Add("Missing source file: $RelativePath")
        return ''
    }
    return Get-Content -LiteralPath $path -Raw
}

function Require-Text(
    [string]$RelativePath,
    [string]$Pattern,
    [string]$Message
) {
    if ((Read-Source $RelativePath) -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Reject-Text(
    [string]$RelativePath,
    [string]$Pattern,
    [string]$Message
) {
    if ((Read-Source $RelativePath) -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

$sceneWriterPath = 'src/physics-interaction/native/SceneWriterProbe.cpp'
$sceneWriter = Read-Source $sceneWriterPath
$hookStart = $sceneWriter.IndexOf('void __fastcall onSceneTransformWriter')
$installStart = $sceneWriter.IndexOf('    bool install()', $hookStart)
if ($hookStart -lt 0 -or $installStart -le $hookStart) {
    $failures.Add("$sceneWriterPath`: Could not isolate the scene-writer hook body.")
} else {
    $hook = $sceneWriter.Substring($hookStart, $installStart - $hookStart)
    foreach ($forbidden in @(
        'g_rockConfig',
        'ROCK_LOG_',
        'fmt::',
        'spdlog',
        'GetCurrentThreadId',
        'readMotionCenter',
        'MotionSample'
    )) {
        if ($hook.Contains($forbidden)) {
            $failures.Add("$sceneWriterPath`: The hook body must not contain $forbidden.")
        }
    }
}

Require-Text $sceneWriterPath `
    'HookConfigSlot[\s\S]*std::atomic<bool>\s+syncEnabled[\s\S]*std::atomic<float>\s+fullGapGameUnits[\s\S]*publishHookConfig[\s\S]*tryReadHookConfig' `
    'Hook configuration must be published atomically by the game thread and consumed only through a coherent snapshot.'
Require-Text $sceneWriterPath `
    'refreshHeldPresentationConfig[\s\S]*publishHookConfig\(\)[\s\S]*!g_rockConfig\.rockGrabHeldScenePoseSync[\s\S]*invalidateHeldAnchor' `
    'Every held frame must publish hot-reloaded configuration and invalidate retained anchors when scene sync is disabled.'
Require-Text 'src/physics-interaction/hand/grab/HandGrabHeldUpdate.cpp' `
    'refreshHeldPresentationConfig\(_isLeft\)' `
    'The held-object game-thread update must refresh immutable scene-writer configuration every frame.'
Require-Text $sceneWriterPath `
    'std::atomic<bool>\s+active[\s\S]*s_activeHookReaders[\s\S]*NiPointer<RE::NiAVObject>\s+s_roomNodeOwners[\s\S]*s_retiredRoomNodeOwners[\s\S]*HookReaderGuard' `
    'Hook readers must be counted and every raw room-node snapshot must have a strong game-thread owner and retirement path.'
Require-Text $sceneWriterPath `
    'static_assert\(std::atomic<bool>::is_always_lock_free\)[\s\S]*std::atomic<float>::is_always_lock_free[\s\S]*std::atomic<const RE::NiAVObject\*>::is_always_lock_free' `
    'Every atomic type used by the unknown-thread hook must be build-verified as lock-free.'
Require-Text $sceneWriterPath `
    'bool registerHeldTarget[\s\S]*clearHandSlot\(slot\)[\s\S]*s_activeHookReaders[\s\S]*return false[\s\S]*slot\.active\.store\(true' `
    'Registration must deactivate the old slot and retry instead of replacing a node owner under an active reader.'
Require-Text 'src/ROCKMain.cpp' `
    'onGameFrameUpdateHook[\s\S]{0,300}s_originalGameLoopFunc\(rcx\)[\s\S]{0,160}scene_writer_probe::serviceGameThread\(\)' `
    'The game thread must release drained reader-retired node owners even when no later grab occurs.'
Require-Text 'src/physics-interaction/hand/grab/HandGrabHeldUpdate.cpp' `
    'if \(scene_writer_probe::registerHeldTarget[\s\S]{0,180}_sceneWriterProbeRegisteredTraceId\s*=\s*_grabFrame\.traceId' `
    'A failed scene-writer registration must remain unacknowledged so the game thread retries it.'
Require-Text 'src/ROCKMain.cpp' `
    'scene_writer_probe::install\(\)[\s\S]{0,250}logger::critical[\s\S]{0,180}return false' `
    'The default-on held scene presentation boundary must fail plugin load when its verified hook cannot install.'
Reject-Text 'src/physics-interaction/hand/grab/HandGrabVisualAuthority.cpp' `
    'ROCK_LOG_INFO\(Hand,[\s\S]{0,120}ANCHOR_CLOCK\s+stage=preFrik' `
    'Pre-hFRIK grab authority must not emit an unbounded information log each frame.'

Require-Text 'src/api/ProviderApiAnimationAuthority.cpp' `
    'isExternalHandWorldSchedulerReady\(\)[\s\S]{0,300}RockProviderResultV1::NotReady[\s\S]*VisualWritesAllowed[\s\S]{0,400}isExternalHandWorldSchedulerReady\(\)[\s\S]{0,300}WorldTransformWritesAllowed' `
    'Provider world-transform requests and their separate capability flag must fail closed when the pre-hFRIK scheduler is unavailable without disabling finger-only poses.'
Require-Text 'src/api/ROCKProviderApi.h' `
    'next skeleton frame[\s\S]*republish[\s\S]*next skeleton frame[\s\S]*NotReady' `
    'Provider V1 must document deferred publication, moving-target refresh, clear latency, and scheduler readiness.'

Reject-Text 'src/physics-interaction/weapon/two_handed/TwoHandedGripGunstock.cpp' `
    'recoilPrecompensated|recoilProbeApplied|immediately read|synchronously inside' `
    'Gunstock must not retain same-call deferred-publication readback or recoil precompensation.'
Require-Text 'src/physics-interaction/weapon/two_handed/TwoHandedGripGunstock.cpp' `
    'Publication is deferred[\s\S]{0,300}shared recoil transaction[\s\S]{0,300}publishExternalHandWorldTransform' `
    'Gunstock must submit its target to the shared deferred recoil transaction.'
Require-Text 'src/physics-interaction/weapon/two_handed/TwoHandedGripHandAuthority.cpp' `
    'tryGetPublishedExternalHandWorldWinner[\s\S]{0,300}weaponPresentationFollowsRole[\s\S]*handWorldPublicationSequence' `
    'Weapon recoil and independent restore must validate typed winner role and exact publication sequence.'
Require-Text 'src/physics-interaction/weapon/two_handed/TwoHandedGrip.cpp' `
    'void TwoHandedGrip::reset\(\)[\s\S]*_scopeDeferredHandAuthorityClears\s*=\s*\{\}' `
    'Lifecycle reset must remove every pending deferred scope-hand clear mask.'

if ($failures.Count -gt 0) {
    Write-Host 'Deferred hand-authority safety source tests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Deferred hand-authority safety source tests passed.' -ForegroundColor Green
