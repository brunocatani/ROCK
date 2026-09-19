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
    param([Parameter(Mandatory)][string]$ChangedPath, [string]$TestRoot = $Root)

    $json = & $selectorPath -Root $TestRoot -ChangedFile $ChangedPath -PlanOnly -Json
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
    Require-Contains $sourceOnlyPlan.BehavioralCoverageUnverified 'src/physics-interaction/native/PhysicsShapeCast.cpp' 'A source scan must not be reported as behavioral coverage.'
    if (@($focusedPlan.SdkBuildTargets).Count -ne 0) {
        $failures.Add('A focused input-policy change must not build the SDK examples.')
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
    Require-Contains $fullPlan.SdkBuildTargets 'ROCKSDKExamplePlugins' 'Full validation must explicitly include the separate SDK build lane.'
    $apiPlan = Get-TestPlan 'src/api/Discovery.cpp'
    Require-Contains $apiPlan.SdkBuildTargets 'ROCKSDKExamplePlugins' 'A public API change must build SDK consumers.'
    Require-Contains $apiPlan.SdkTests 'RpsSdkRockContractTests' 'A public API change must select SDK publication contracts.'
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

# Exercise the catalog consumer independently of the live suite's target count
# and naming. CMake itself owns parsing single/multiline target registrations.
$fixture = Join-Path ([System.IO.Path]::GetTempPath()) ('rock-selector-' + [guid]::NewGuid())
try {
    foreach ($directory in @('tests', 'src', 'cmake', 'build-tests')) {
        $null = New-Item -ItemType Directory -Path (Join-Path $fixture $directory) -Force
    }
    Set-Content (Join-Path $fixture 'CMakeLists.txt') 'project(SelectorFixture)'
    Set-Content (Join-Path $fixture 'cmake/RockTestRegistration.cmake') '# fixture registration'
    Set-Content (Join-Path $fixture 'tests/DifferentFilename.cpp') '#include "Shared.h"'
    Set-Content (Join-Path $fixture 'tests/Second.cpp') '// second compilation unit'
    Set-Content (Join-Path $fixture 'src/Shared.h') '// behavioral dependency'
    Set-Content (Join-Path $fixture 'src/CycleA.h') @('#include "CycleB.h"', '#include "Shared.h"')
    Set-Content (Join-Path $fixture 'src/CycleB.h') '#include "CycleA.h"'
    Set-Content (Join-Path $fixture 'tests/CycleA.cpp') '#include "CycleA.h"'
    Set-Content (Join-Path $fixture 'tests/CycleB.cpp') '#include "CycleB.h"'
    Set-Content (Join-Path $fixture 'src/Uncovered.cpp') '// has no registered consumer'
    $catalog = @{
        CMakeHash = (Get-FileHash (Join-Path $fixture 'CMakeLists.txt')).Hash
        RegistrationHash = (Get-FileHash (Join-Path $fixture 'cmake/RockTestRegistration.cmake')).Hash
        Targets = @{
            ACycleEntry = @((Join-Path $fixture 'tests/CycleA.cpp'))
            NumericsContract = @((Join-Path $fixture 'tests/DifferentFilename.cpp'), (Join-Path $fixture 'tests/Second.cpp'))
            ZCycleEntry = @((Join-Path $fixture 'tests/CycleB.cpp'))
        }
    }
    $catalog | ConvertTo-Json -Depth 5 | Set-Content (Join-Path $fixture 'build-tests/RockTestCatalog.json')
    foreach ($changed in @('src/Shared.h', 'tests/Second.cpp')) {
        $fixturePlan = Get-TestPlan $changed $fixture
        Require-Contains $fixturePlan.PolicyTargets 'NumericsContract' 'Catalog targets must support multiple sources and unrelated target/source names.'
        if ($fixturePlan.Mode -ne 'affected') { $failures.Add('A mapped fixture input must remain focused.') }
    }
    $cyclePlan = Get-TestPlan 'src/Shared.h' $fixture
    Require-Contains $cyclePlan.PolicyTargets 'ACycleEntry' 'The first entry into an include cycle must retain all dependencies.'
    Require-Contains $cyclePlan.PolicyTargets 'ZCycleEntry' 'A later entry into an include cycle must not reuse an incomplete cached closure.'
    $deletedPlan = Get-TestPlan 'tests/Deleted.cpp' $fixture
    if ($deletedPlan.Mode -ne 'full') { $failures.Add('Deleted inputs must select broad validation.') }
    $uncoveredPlan = Get-TestPlan 'src/Uncovered.cpp' $fixture
    Require-Contains $uncoveredPlan.BehavioralCoverageUnverified 'src/Uncovered.cpp' 'Unknown production coverage must be explicit even after broad selection.'
    Add-Content (Join-Path $fixture 'CMakeLists.txt') '# new registration'
    $staleRejected = $false
    try { $null = Get-TestPlan 'src/Shared.h' $fixture } catch { $staleRejected = $true }
    if (-not $staleRejected) { $failures.Add('A stale catalog must not produce a misleading plan.') }
} finally {
    $resolvedFixture = [System.IO.Path]::GetFullPath($fixture)
    $tempRoot = [System.IO.Path]::GetFullPath([System.IO.Path]::GetTempPath()).TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar
    if (-not $resolvedFixture.StartsWith($tempRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Fixture escaped temporary root: $resolvedFixture"
    }
    Remove-Item -LiteralPath $resolvedFixture -Recurse -Force
}

if ($failures.Count -gt 0) {
    Write-Host 'AffectedTestSelectionSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure" -ForegroundColor Red
    }
    exit 1
}

Write-Host 'AffectedTestSelectionSourceTests passed.' -ForegroundColor Green
