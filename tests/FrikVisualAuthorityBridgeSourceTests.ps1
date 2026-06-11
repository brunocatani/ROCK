param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()
$workspaceRoot = Split-Path -Parent $Root

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-ExternalText {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $workspaceRoot $Path)
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

function Require-ExternalText {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $workspaceRoot $Path)
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h' 'g_cachedHandPosePublications' `
    'FRIK scalar hand-pose publication must retain a bounded cache to avoid duplicate hot-path provider calls.'
Require-Text 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h' 'getHandPoseSetTagState\(tag,\s*hand\)\s*==\s*HandPoseTagState::Active' `
    'Cached FRIK hand-pose skips must verify the tag is still active before suppressing a publish.'
Require-Text 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h' 'sameHandPoseData\(entry->pose,\s*handPose\)' `
    'Cached FRIK hand-pose skips must compare the full hand-pose payload.'
Require-Text 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h' 'clearHandPose\([^)]*\)[\s\S]*invalidateCachedHandPosePublication\(tag,\s*hand\)' `
    'Clearing a FRIK hand-pose tag must invalidate ROCK cached publication state.'

Reject-ExternalText 'hFRIK/src/skeleton/HandPose.cpp' 'logger::info\("Hand pose:' `
    'hFRIK must not info-log hand-pose override set/clear operations from the runtime hot path.'
Require-ExternalText 'hFRIK/src/skeleton/HandPose.cpp' 'logger::debug\("Hand pose:' `
    'hFRIK hand-pose stack transition diagnostics should remain debug-only.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}
