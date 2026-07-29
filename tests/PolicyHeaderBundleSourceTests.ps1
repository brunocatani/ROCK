param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Path {
    param(
        [string]$RelativePath,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path)) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Reject-Path {
    param(
        [string]$RelativePath,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    if (Test-Path -LiteralPath $path) {
        $failures.Add("$RelativePath`: $Message")
    }
}

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

function Reject-RepositoryText {
    param(
        [string]$Pattern,
        [string]$Message
    )

    $paths = @()
    foreach ($relativeRoot in @('src', 'tests')) {
        $searchRoot = Join-Path $Root $relativeRoot
        if (Test-Path -LiteralPath $searchRoot) {
            $paths += Get-ChildItem -LiteralPath $searchRoot -Recurse -File -Include '*.h', '*.cpp', '*.inl', '*.ps1'
        }
    }

    foreach ($path in $paths) {
        $text = Get-Content -Raw -LiteralPath $path.FullName
        if ($text -match $Pattern) {
            $relative = $path.FullName
            $rootPrefix = $Root.TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar
            if ($relative.StartsWith($rootPrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
                $relative = $relative.Substring($rootPrefix.Length)
            }
            $failures.Add("$relative`: $Message")
        }
    }
}

Require-Path 'src/physics-interaction/feedback/HapticPolicy.h' `
    'Grab and shoulder-stash haptic policies should share one small feedback policy header.'
Require-Text 'src/physics-interaction/feedback/HapticPolicy.h' 'namespace\s+rock::grab_haptic_policy' `
    'Bundled haptic policy must preserve the grab haptic namespace.'
Require-Text 'src/physics-interaction/feedback/HapticPolicy.h' 'namespace\s+rock::shoulder_stash_haptic_policy' `
    'Bundled haptic policy must preserve the shoulder-stash haptic namespace.'
Require-Text 'src/physics-interaction/feedback/HapticPolicy.h' 'namespace\s+rock::haptic_policy_detail' `
    'Bundled haptic policy should share finite/intensity sanitizers instead of duplicating them.'

Require-Path 'src/physics-interaction/stash/ShoulderStashPolicy.h' `
    'Tiny shoulder-stash policy surfaces should be bundled behind one stash policy header.'
Require-Text 'src/physics-interaction/stash/ShoulderStashPolicy.h' 'namespace\s+rock::shoulder_stash_notification_policy' `
    'Bundled shoulder-stash policy must preserve notification formatting helpers.'
Require-Text 'src/physics-interaction/stash/ShoulderStashPolicy.h' 'enum\s+class\s+EligibilityReason' `
    'Bundled shoulder-stash policy must preserve eligibility declarations.'
Require-Text 'src/physics-interaction/stash/ShoulderStashPolicy.h' '#include\s+"physics-interaction/feedback/HapticPolicy.h"' `
    'Bundled shoulder-stash policy should re-export shoulder-stash haptic policy through the shared feedback header.'
Require-Text 'src/physics-interaction/stash/ShoulderStashPolicy.h' 'namespace\s+rock\s*\{[\s\S]*struct\s+SavedObjectState' `
    'Bundled shoulder-stash policy should forward-declare saved grab state instead of including heavy grab constraint headers.'
Reject-Text 'src/physics-interaction/stash/ShoulderStashPolicy.h' 'GrabConstraint\.h' `
    'Bundled shoulder-stash policy must not pull heavy grab/Havok constraint headers into haptic and notification users.'

Require-Path 'src/physics-interaction/debug/DebugMath.h' `
    'Tiny debug axis and pivot math helpers should be bundled behind one debug math header.'
Require-Text 'src/physics-interaction/debug/DebugMath.h' 'namespace\s+rock::debug_axis_math' `
    'Bundled debug math must preserve axis helpers.'
Require-Text 'src/physics-interaction/debug/DebugMath.h' 'namespace\s+rock::debug_pivot_math' `
    'Bundled debug math must preserve pivot helpers.'

Reject-Path 'src/physics-interaction/grab/GrabHapticPolicy.h' `
    'Superseded grab haptic policy shim should not remain as a duplicate include path.'
Reject-Path 'src/physics-interaction/stash/ShoulderStashHapticPolicy.h' `
    'Superseded shoulder-stash haptic policy shim should not remain as a duplicate include path.'
Reject-Path 'src/physics-interaction/stash/ShoulderStashNotificationPolicy.h' `
    'Superseded shoulder-stash notification policy shim should not remain as a duplicate include path.'
Reject-Path 'src/physics-interaction/stash/ShoulderStashEligibility.h' `
    'Superseded shoulder-stash eligibility policy shim should not remain as a duplicate include path.'
Reject-Path 'src/physics-interaction/debug/DebugAxisMath.h' `
    'Superseded debug axis math shim should not remain as a duplicate include path.'
Reject-Path 'src/physics-interaction/debug/DebugPivotMath.h' `
    'Superseded debug pivot math shim should not remain as a duplicate include path.'

Reject-RepositoryText '#include\s+"physics-interaction/(grab/GrabHapticPolicy|stash/ShoulderStashHapticPolicy|stash/ShoulderStashNotificationPolicy|stash/ShoulderStashEligibility|debug/DebugAxisMath|debug/DebugPivotMath)\.h"' `
    'Code must include the bundled policy headers instead of superseded tiny policy paths.'

if ($failures.Count -gt 0) {
    Write-Host 'PolicyHeaderBundleSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'PolicyHeaderBundleSourceTests passed.' -ForegroundColor Green
