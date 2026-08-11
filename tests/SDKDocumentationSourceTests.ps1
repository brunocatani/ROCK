param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()
$headerPath = Join-Path $Root 'SDK/ROCK/include/ROCKProviderApi.h'
$apiIndexPath = Join-Path $Root 'SDK/ROCK/docs/ApiIndex.md'
$capabilitiesPath = Join-Path $Root 'SDK/ROCK/docs/DiscoveryAndCapabilities.md'
$gettingStartedPath = Join-Path $Root 'SDK/ROCK/docs/GettingStarted.md'
$examplesRoot = Join-Path $Root 'SDK/ROCK/examples'

foreach ($requiredPath in @(
        $headerPath,
        $apiIndexPath,
        $capabilitiesPath,
        $gettingStartedPath,
        (Join-Path $examplesRoot 'README.md'),
        (Join-Path $examplesRoot 'CMakeLists.txt'),
        (Join-Path $examplesRoot 'common/ExampleRuntime.cpp'),
        (Join-Path $examplesRoot 'cmake/VerifyFo4VrLoader.cmake'),
        (Join-Path $examplesRoot 'mods/HandStateMonitor.cpp'),
        (Join-Path $examplesRoot 'mods/WeaponInspector.cpp'),
        (Join-Path $examplesRoot 'mods/SurfaceClimber.cpp'),
        (Join-Path $examplesRoot 'mods/ContactVisualizer.cpp'))) {
    if (-not (Test-Path -LiteralPath $requiredPath)) {
        $failures.Add("Missing required SDK artifact: $requiredPath")
    }
}

if ((Test-Path -LiteralPath $headerPath) -and
    (Test-Path -LiteralPath $apiIndexPath)) {
    $headerText = Get-Content -Raw -LiteralPath $headerPath
    $apiIndexText = Get-Content -Raw -LiteralPath $apiIndexPath
    $headerFunctions = @(
        [regex]::Matches(
            $headerText,
            'ROCK_PROVIDER_CALL\s*\*\s*([A-Za-z_][A-Za-z0-9_]*)') |
            ForEach-Object { $_.Groups[1].Value })
    $documentedRows = @(
        [regex]::Matches(
            $apiIndexText,
            '(?m)^\|\s*(\d+)\s*\|\s*`([A-Za-z_][A-Za-z0-9_]*)`\s*\|'))

    if ($headerFunctions.Count -ne 90) {
        $failures.Add("Expected the V1 header to expose 90 function slots; found $($headerFunctions.Count)")
    }
    if ($documentedRows.Count -ne $headerFunctions.Count) {
        $failures.Add("API index documents $($documentedRows.Count) slots; header exposes $($headerFunctions.Count)")
    } else {
        for ($index = 0; $index -lt $headerFunctions.Count; ++$index) {
            $documentedIndex = [int]$documentedRows[$index].Groups[1].Value
            $documentedName = $documentedRows[$index].Groups[2].Value
            if ($documentedIndex -ne $index -or
                $documentedName -ne $headerFunctions[$index]) {
                $failures.Add(
                    "API index mismatch at slot $index`: expected '$($headerFunctions[$index])', documented '$documentedName' at $documentedIndex")
                break
            }
        }
    }
}

if (Test-Path -LiteralPath $capabilitiesPath) {
    $capabilitiesText = Get-Content -Raw -LiteralPath $capabilitiesPath
    $requiredCapabilities = @(
        'FrameSnapshots', 'ExternalBodies', 'ExternalContacts',
        'OffhandReservation', 'InteractionCommands', 'HandInputSuppression',
        'WeaponPartInteraction', 'NativeAnimationAuthority', 'AnimationPhases',
        'EquippedWeaponGripState', 'HandVisualAuthority',
        'NativeAnimationRuntimeProvider', 'EquippedWeaponHandlingAuthority',
        'DebugOverlayPublication', 'ProviderEvents', 'HandInteractionState',
        'ExternalBodyScopes', 'WeaponPartObservability', 'WeaponComposition',
        'PoseReadback', 'SemanticHandContacts', 'PlayerColliderDescriptors',
        'ScopeSightState', 'InputObservability', 'TouchGrabTargets',
        'WorldRaycasts', 'ColliderVisualizationOverride')
    foreach ($capability in $requiredCapabilities) {
        if ($capabilitiesText -notmatch [regex]::Escape("``$capability``")) {
            $failures.Add("Capability documentation is missing '$capability'")
        }
    }
}

if (Test-Path -LiteralPath $gettingStartedPath) {
    $gettingStartedText = Get-Content -Raw -LiteralPath $gettingStartedPath
    foreach ($code in 0..6) {
        if ($gettingStartedText -notmatch "(?m)^\|\s*$code\s*\|") {
            $failures.Add("Getting started documentation is missing initialize result code $code")
        }
    }
}

if (Test-Path -LiteralPath $examplesRoot) {
    $publicTextFiles = Get-ChildItem -LiteralPath (Join-Path $Root 'SDK/ROCK') -Recurse -File |
        Where-Object { $_.Extension -in @('.md', '.cpp', '.h', '.cmake', '.txt') }
    foreach ($file in $publicTextFiles) {
        $text = Get-Content -Raw -LiteralPath $file.FullName
        if ($text -match '(?i)\bPAPER(?:_Toolkit)?\b|\bSCISSORS\b|\bROCK_Addons\b') {
            $failures.Add("Public SDK artifact names an internal/reference project: $($file.FullName)")
        }
        if ($text -match '(?i)\b82\s+(?:callables|functions|slots)\b') {
            $failures.Add("Public SDK artifact contains the stale 82-call count: $($file.FullName)")
        }
    }
}

$sdkDocumentation = Get-ChildItem -LiteralPath (Join-Path $Root 'SDK/ROCK') -Recurse -File |
    Where-Object { $_.Extension -eq '.md' } |
    ForEach-Object { Get-Content -Raw -LiteralPath $_.FullName }
$combinedDocumentation = $sdkDocumentation -join "`n"
foreach ($invalidIdentifier in @(
        'RockProviderExternalBodyRole::Interactive',
        'RockProviderExternalBodyContactPolicy::All',
        'RockProviderHandInputSuppressionFlagV1::SuppressGrab',
        'request.originGame')) {
    if ($combinedDocumentation -match [regex]::Escape($invalidIdentifier)) {
        $failures.Add("SDK documentation uses nonexistent identifier '$invalidIdentifier'")
    }
}

if ($failures.Count -ne 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'ROCK SDK documentation/source checks passed.'
