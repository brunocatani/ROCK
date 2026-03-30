param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        $failures.Add($Message)
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

Reject-Text 'src/api/FRIKApi.h' '#include\s+<Windows\.h>' 'FRIK public API header must not include Windows.h or add Windows macro state to consumers.'
Reject-Text 'src/api/ROCKApi.h' '#include\s+<Windows\.h>' 'ROCK public API header must not include Windows.h or add Windows macro state to consumers.'
Reject-Text 'src/api/ROCKProviderApi.h' '#include\s+<Windows\.h>' 'ROCK provider API header must not include Windows.h or add Windows macro state to consumers.'
Require-Text 'src/api/FRIKApi.h' '#pragma push_macro\("MEM_COMMIT"\)[\s\S]*#undef MEM_COMMIT[\s\S]*#include "RE/NetImmerse/NiPoint.h"[\s\S]*#pragma pop_macro\("MEM_COMMIT"\)' 'FRIK public API header must scope pre-existing Windows macro cleanup around CommonLib includes and restore consumer macro state.'
Require-Text 'src/api/ROCKApi.h' '#pragma push_macro\("MEM_COMMIT"\)[\s\S]*#undef MEM_COMMIT[\s\S]*#include "RE/NetImmerse/NiPoint.h"[\s\S]*#pragma pop_macro\("MEM_COMMIT"\)' 'ROCK public API header must scope pre-existing Windows macro cleanup around CommonLib includes and restore consumer macro state.'
Reject-Text 'src/api/ROCKProviderApi.h' '#undef\s+(near|far|MEM_RELEASE|MEM_COMMIT|MEM_RESERVE|PAGE_EXECUTE_READ|PAGE_EXECUTE_READWRITE|MAX_PATH)' 'ROCK provider API header does not include CommonLib headers and must not mutate consumer Windows macro state.'
Require-Text 'src/api/ROCKApi.cpp' 'ROCK_ASSERT_API_ENUM\(GrabEventType,\s*GrabEventType,\s*PullCatchAttempt\)' 'ROCK public grab event enum values must be statically guarded against internal enum drift.'
Require-Text 'src/api/ROCKApi.cpp' 'ROCK_ASSERT_API_ENUM\(GrabEventSourceKind,\s*GrabEventSourceKind,\s*PulledObject\)' 'ROCK public grab source enum values must be statically guarded against internal enum drift.'
Require-Text 'src/api/ROCKApi.cpp' 'kGrabEventFlagSuppressHaptic\s*==\s*rock::ROCK_GRAB_EVENT_FLAG_SUPPRESS_HAPTIC' 'ROCK public grab event flag constants must be statically guarded against internal flag drift.'
Require-Text 'src/api/ROCKApi.cpp' 'ROCK_ASSERT_GRAB_EVENT_OFFSET\(reservedFloat\)' 'ROCK public grab event payload must statically guard every field offset, not only sample offsets.'
Require-Text 'src/api/ROCKApi.h' 'kGrabEventFlagMassValid' 'ROCK public grab event flags must expose payload validity to API consumers.'

if ($failures.Count -gt 0) {
    Write-Host 'API header macro boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'API header macro boundary passed.'
