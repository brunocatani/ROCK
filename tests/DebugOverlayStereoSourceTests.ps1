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
    if (-not (Test-Path -LiteralPath $path)) {
        return
    }

    $text = Get-Content -Raw -LiteralPath $path
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

$overlay = 'src/physics-interaction/debug/DebugBodyOverlay.cpp'

# Verified FO4VR stereo layout: current-frame eye origins are the +0x2590/+0x25A0 pair.
Require-Text $overlay 'kRootStereoSlot0OriginOffset\s*=\s*0x2590' `
    'Left-eye origin must read the verified current-frame slot +0x2590.'
Require-Text $overlay 'kRootStereoSlot1OriginOffset\s*=\s*0x25A0' `
    'Right-eye origin must read the verified current-frame slot +0x25A0.'
Require-Text $overlay 'kRootStereoRecordsDataOffset\s*=\s*0x25D0' `
    'Stereo-record pointer must be the named +0x25D0 constant.'
Require-Text $overlay 'kStereoRecordStride\s*=\s*0x210' `
    'Stereo record stride must be the named 0x210 constant.'
Require-Text $overlay 'kStereoRecordCompositeOffset\s*=\s*0xD0' `
    'Composite matrix offset must be the named +0xD0 constant.'
Require-Text $overlay 'static_assert\(kStereoSlot1CompositeOffset\s*==\s*0x2E0\)' `
    'Right-eye composite offset must be derived from stride and asserted as 0x2E0.'

# The previous-frame right-eye origin (+0x25C0) caused the right-eye stick-locomotion stutter.
# Reject code usages (pointer arithmetic or constant definitions); the rationale comment may name them.
Reject-Text $overlay '\+\s*0x25C0\s*\)' `
    'The overlay must not read +0x25C0; it is the previous-frame right-eye origin and lags stick locomotion by one frame.'
Reject-Text $overlay '=\s*0x25C0' `
    'No stereo constant may be defined as 0x25C0; the current-frame right-eye origin is +0x25A0.'
Reject-Text $overlay '\+\s*0x25B0\s*\)' `
    'The overlay must not read +0x25B0; it is the previous-frame left-eye origin.'
Reject-Text $overlay '=\s*0x25B0' `
    'No stereo constant may be defined as 0x25B0; the current-frame left-eye origin is +0x2590.'

# Fail-closed staged capture: guarded reads, validation, deepest-stage rate-limited diagnostics.
Require-Text $overlay 'ReadProcessMemory\(GetCurrentProcess\(\)' `
    'Stereo state must be read through guarded ReadProcessMemory, not raw dereferences.'
Require-Text $overlay 'isStereoPlausiblePointer\(runtimeRoot\)' `
    'The runtime root pointer must pass a plausibility gate before use.'
Require-Text $overlay 'isStereoPlausiblePointer\(rootFields\.recordsData\)' `
    'The stereo-record pointer must pass a plausibility gate before use.'
Require-Text $overlay 'validateStereoVector3\(rootFields\.slot0Origin\)\s*\|\|\s*!validateStereoVector3\(rootFields\.slot1Origin\)' `
    'Both eye origins must be validated before the snapshot is accepted.'
Require-Text $overlay 'validateStereoMatrix\(composite0\)\s*\|\|\s*!validateStereoMatrix\(composite1\)' `
    'Both composite matrices must be validated before the snapshot is accepted.'
Require-Text $overlay 'reportStereoCaptureFailure\(deepestStage' `
    'Capture failures must report the deepest successful stage.'
Require-Text $overlay 'stageChanged\s*\|\|\s*intervalElapsed' `
    'Stereo capture diagnostics must be rate-limited.'
Require-Text $overlay 'static_assert\(offsetof\(RootStereoFields,\s*slot1Origin\)\s*==\s*0x10\)' `
    'The contiguous root stereo layout must be enforced with static asserts.'

# Raw unguarded reads of the stereo state must not come back.
Reject-Text $overlay 'reinterpret_cast<std::uintptr_t\*>\(skyVR' `
    'The unchecked skyVR pointer walk must not return; use the staged fail-closed capture.'

if ($failures.Count -gt 0) {
    Write-Host 'DebugOverlayStereoSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'DebugOverlayStereoSourceTests passed.' -ForegroundColor Green
