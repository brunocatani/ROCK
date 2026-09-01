param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

# Functional regression test for the affected-test selector. It executes
# tools/Invoke-RockTests.ps1 with -PlanOnly -Json and asserts on the produced
# plan, never on the selector's source text.

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()
$selectorPath = Join-Path $Root 'tools/Invoke-RockTests.ps1'

function Get-TestPlan {
    param([Parameter(Mandatory)][string]$ChangedPath)

    $json = & $selectorPath -ChangedFile $ChangedPath -PlanOnly -Json
    return $json | ConvertFrom-Json
}

function Require-Contains {
    param(
        [object[]]$Values,
        [string]$Expected,
        [string]$Message
    )

    if ($Values -notcontains $Expected) {
        $failures.Add($Message)
    }
}

function Reject-Contains {
    param(
        [object[]]$Values,
        [string]$Rejected,
        [string]$Message
    )

    if ($Values -contains $Rejected) {
        $failures.Add($Message)
    }
}

if (-not (Test-Path -LiteralPath $selectorPath -PathType Leaf)) {
    $failures.Add('Affected-test selector must exist under tools/.')
} else {
    $focusedPlan = Get-TestPlan 'src/physics-interaction/input/GrabInputIntentPolicy.h'
    if ($focusedPlan.Mode -ne 'affected') {
        $failures.Add('A focused policy-header change must remain in affected-test mode.')
    }
    Require-Contains $focusedPlan.PolicyTargets 'ROCKGrabInputIntentPolicyTests' 'A changed policy header must select its consuming C++ policy test.'
    Reject-Contains $focusedPlan.PolicyTargets 'ROCKWeaponInteractionPolicyTests' 'An unrelated C++ policy target must not be built for a focused input-policy change.'
    Require-Contains $focusedPlan.PolicyBuildTargets 'ROCKGrabInputIntentPolicyTests' 'A focused policy plan must build its exact CMake target.'

    $sourceOnlyPlan = Get-TestPlan 'src/physics-interaction/native/PhysicsShapeCast.cpp'
    if ($sourceOnlyPlan.Mode -ne 'affected') {
        $failures.Add('A production source change covered by the repository contract scan must stay in affected-test mode.')
    }
    Require-Contains $sourceOnlyPlan.SourceTests 'RepoContractSourceTests' 'A production source change must select the repository contract checks.'
    if (@($sourceOnlyPlan.PolicyTargets).Count -ne 0) {
        $failures.Add('A production source-only change must not build unrelated header-only policy tests.')
    }

    $sharedPlan = Get-TestPlan 'src/RockConfig.h'
    $fullPlan = Get-TestPlan 'CMakeLists.txt'
    if ($sharedPlan.Mode -ne 'affected') {
        $failures.Add('A shared policy-support dependency should expand C++ targets without forcing the full suite.')
    }
    if (@($sharedPlan.PolicyTargets).Count -ne @($fullPlan.PolicyTargets).Count) {
        $failures.Add('A shared policy-support dependency must select every C++ policy target.')
    }
    if (@($sharedPlan.PolicyBuildTargets).Count -ne 1 -or
        @($sharedPlan.PolicyBuildTargets) -notcontains 'ROCKPolicyTestBinaries') {
        $failures.Add('All-policy plans must use the single aggregate CMake build target.')
    }
    if ($fullPlan.Mode -ne 'full') {
        $failures.Add('CMake changes must fail closed to the complete regression suite.')
    }
    if (@($fullPlan.SourceTests).Count -lt 1) {
        $failures.Add('The complete regression plan must include the repository contract checks.')
    }

    $selectorPlan = Get-TestPlan 'tools/Invoke-RockTests.ps1'
    Require-Contains $selectorPlan.SourceTests 'AffectedTestSelectionSourceTests' 'Changes to the selector must select its own regression test.'

    $unmappedPath = '{0}{1}' -f 'too', 'ls/UnmappedRuntimeTool.ps1'
    $unmappedPlan = Get-TestPlan $unmappedPath
    if ($unmappedPlan.Mode -ne 'full') {
        $failures.Add('Unmapped relevant files must fail closed to the complete regression suite.')
    }
}

if ($failures.Count -gt 0) {
    Write-Host 'AffectedTestSelectionSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure" -ForegroundColor Red
    }
    exit 1
}

Write-Host 'AffectedTestSelectionSourceTests passed.' -ForegroundColor Green
