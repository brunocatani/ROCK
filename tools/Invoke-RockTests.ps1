<#
.SYNOPSIS
Builds and runs only ROCK tests affected by the current change.

.DESCRIPTION
With a dirty worktree, selects from staged, unstaged, and untracked files.
With a clean worktree, selects from the latest commit. C++ tests are mapped
through their local include closure; PowerShell source tests are mapped through
repository paths in their syntax tree. Shared, deleted, unclassified, and build
infrastructure inputs fail closed to broader coverage.

.PARAMETER BaseRef
Selects committed changes from BaseRef through HEAD, plus worktree changes.

.PARAMETER ChangedFile
Uses explicit repository-relative paths instead of Git change discovery.

.PARAMETER All
Builds and runs the complete policy and source-boundary suite.

.PARAMETER PlanOnly
Prints the selection without building or running tests.

.PARAMETER Json
Emits the plan as JSON. Valid only with PlanOnly and intended for regression tooling.

.EXAMPLE
pwsh -NoProfile -File tools/Invoke-RockTests.ps1

.EXAMPLE
pwsh -NoProfile -File tools/Invoke-RockTests.ps1 -BaseRef HEAD~3

.EXAMPLE
pwsh -NoProfile -File tools/Invoke-RockTests.ps1 -All
#>
[CmdletBinding()]
param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path,
    [string]$BaseRef,
    [string[]]$ChangedFile,
    [switch]$All,
    [switch]$PlanOnly,
    [switch]$Json
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$script:RepoRoot = (Resolve-Path -LiteralPath $Root).Path
$script:BuildDirectory = Join-Path $script:RepoRoot 'build-tests'
$script:IncludeCache = [System.Collections.Generic.Dictionary[string, object]]::new(
    [System.StringComparer]::OrdinalIgnoreCase)

function Normalize-RelativePath {
    param([Parameter(Mandatory)][string]$Path)

    $candidate = $Path.Trim().Trim('"').Replace('\', '/')
    if ([string]::IsNullOrWhiteSpace($candidate)) {
        return $null
    }

    if ([System.IO.Path]::IsPathRooted($candidate)) {
        $rootWithSeparator = $script:RepoRoot.TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar
        $fullPath = [System.IO.Path]::GetFullPath($candidate)
        if (-not $fullPath.StartsWith($rootWithSeparator, [System.StringComparison]::OrdinalIgnoreCase) -and
            -not $fullPath.Equals($script:RepoRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
            throw "Path is outside the ROCK repository: $Path"
        }
        $candidate = $fullPath.Substring($script:RepoRoot.Length).TrimStart('\', '/')
    }

    while ($candidate.StartsWith('./', [System.StringComparison]::Ordinal)) {
        $candidate = $candidate.Substring(2)
    }

    return $candidate.TrimStart('/').Replace('\', '/')
}

function Invoke-GitLines {
    param([Parameter(Mandatory)][string[]]$Arguments)

    $combinedOutput = @(& git -C $script:RepoRoot @Arguments 2>&1)
    $errorOutput = @(
        $combinedOutput |
            Where-Object { $_ -is [System.Management.Automation.ErrorRecord] } |
            ForEach-Object { "$_" }
    )
    $output = @(
        $combinedOutput |
            Where-Object { $_ -isnot [System.Management.Automation.ErrorRecord] } |
            ForEach-Object { "$_" }
    )
    if ($LASTEXITCODE -ne 0) {
        $diagnostics = @($output + $errorOutput) -join [Environment]::NewLine
        throw "git $($Arguments -join ' ') failed: $diagnostics"
    }

    return @($output | Where-Object { -not [string]::IsNullOrWhiteSpace($_) })
}

function Test-GitRef {
    param([Parameter(Mandatory)][string]$Ref)

    & git -C $script:RepoRoot rev-parse --verify --quiet $Ref *> $null
    return $LASTEXITCODE -eq 0
}

function Get-ChangedFiles {
    $paths = [System.Collections.Generic.HashSet[string]]::new(
        [System.StringComparer]::OrdinalIgnoreCase)

    if ($ChangedFile -and $ChangedFile.Count -gt 0) {
        foreach ($path in $ChangedFile) {
            $normalized = Normalize-RelativePath $path
            if ($normalized) {
                $null = $paths.Add($normalized)
            }
        }
        return @($paths | Sort-Object)
    }

    $worktreePaths = @(
        Invoke-GitLines @('-c', 'core.quotepath=false', 'diff', '--name-only',
            '--diff-filter=ACDMRTUXB', '--no-renames', 'HEAD', '--')
    )
    $untrackedPaths = @(
        Invoke-GitLines @('-c', 'core.quotepath=false', 'ls-files', '--others',
            '--exclude-standard')
    )

    if ($BaseRef) {
        $commitRef = $BaseRef + '^{commit}'
        if (-not (Test-GitRef $commitRef)) {
            throw "BaseRef is not a valid commit: $BaseRef"
        }
        $range = $BaseRef + '..HEAD'
        foreach ($path in Invoke-GitLines @('-c', 'core.quotepath=false', 'diff',
                '--name-only', '--diff-filter=ACDMRTUXB', '--no-renames', $range, '--')) {
            $null = $paths.Add((Normalize-RelativePath $path))
        }
    } elseif ($worktreePaths.Count -eq 0 -and $untrackedPaths.Count -eq 0) {
        if ((Test-GitRef 'HEAD^{commit}') -and (Test-GitRef 'HEAD^{commit}^')) {
            foreach ($path in Invoke-GitLines @('-c', 'core.quotepath=false', 'diff',
                    '--name-only', '--diff-filter=ACDMRTUXB', '--no-renames', 'HEAD^', 'HEAD', '--')) {
                $null = $paths.Add((Normalize-RelativePath $path))
            }
        }
    }

    foreach ($path in $worktreePaths + $untrackedPaths) {
        $null = $paths.Add((Normalize-RelativePath $path))
    }

    return @($paths | Sort-Object)
}

function Test-CatalogCurrent {
    $catalogPath = Join-Path $script:BuildDirectory 'RockTestCatalog.json'
    if (-not (Test-Path -LiteralPath $catalogPath)) { return $false }
    try {
        $catalog = Get-Content -Raw -LiteralPath $catalogPath | ConvertFrom-Json
        return $catalog.CMakeHash -eq (Get-FileHash (Join-Path $script:RepoRoot 'CMakeLists.txt')).Hash -and
            $catalog.RegistrationHash -eq (Get-FileHash (Join-Path $script:RepoRoot 'cmake/RockTestRegistration.cmake')).Hash
    } catch {
        return $false
    }
}

function Get-PolicyTargetDefinitions {
    $catalog = Get-Content -Raw -LiteralPath (Join-Path $script:BuildDirectory 'RockTestCatalog.json') | ConvertFrom-Json
    $definitions = [ordered]@{}
    foreach ($target in $catalog.Targets.PSObject.Properties) {
        $sources = @($target.Value | ForEach-Object { Normalize-RelativePath $_ })
        if ($sources.Count -eq 0) {
            throw "No source files registered for policy target $($target.Name)."
        }
        $definitions[$target.Name] = $sources
    }
    if ($definitions.Count -eq 0) { throw 'CMake registered no policy tests.' }
    return $definitions
}

function Resolve-LocalInclude {
    param(
        [Parameter(Mandatory)][string]$IncludingFile,
        [Parameter(Mandatory)][string]$Include
    )

    $includingFullPath = Join-Path $script:RepoRoot $IncludingFile
    $candidates = @(
        (Join-Path (Split-Path $includingFullPath -Parent) $Include),
        (Join-Path (Join-Path $script:RepoRoot 'src') $Include),
        (Join-Path (Join-Path $script:RepoRoot 'tests') $Include),
        (Join-Path $script:RepoRoot $Include)
    )

    foreach ($candidate in $candidates) {
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            return Normalize-RelativePath ([System.IO.Path]::GetFullPath($candidate))
        }
    }

    return $null
}

function Get-LocalIncludeClosure {
    param([Parameter(Mandatory)][string]$RelativePath)

    # Cache direct edges only. A recursive closure cached while an include cycle
    # is still being visited can silently omit dependencies for later targets.
    $dependencies = [System.Collections.Generic.HashSet[string]]::new(
        [System.StringComparer]::OrdinalIgnoreCase)
    $pending = [System.Collections.Generic.Stack[string]]::new()
    $pending.Push((Normalize-RelativePath $RelativePath))
    while ($pending.Count -gt 0) {
        $current = $pending.Pop()
        if (-not $dependencies.Add($current)) { continue }
        if (-not $script:IncludeCache.ContainsKey($current)) {
            $includes = [System.Collections.Generic.List[string]]::new()
            $fullPath = Join-Path $script:RepoRoot $current
            if (Test-Path -LiteralPath $fullPath -PathType Leaf) {
                $content = Get-Content -Raw -LiteralPath $fullPath
                foreach ($match in [regex]::Matches($content, '(?m)^\s*#\s*include\s*[<"]([^>"]+)[>"]')) {
                    $resolved = Resolve-LocalInclude $current $match.Groups[1].Value
                    if ($resolved) { $includes.Add($resolved) }
                }
            }
            $script:IncludeCache[$current] = @($includes)
        }
        foreach ($included in $script:IncludeCache[$current]) { $pending.Push($included) }
    }
    return @($dependencies | Sort-Object)
}

function Get-SourceTestDependencies {
    param([Parameter(Mandatory)][string]$RelativePath)

    $tokens = $null
    $parseErrors = $null
    $fullPath = Join-Path $script:RepoRoot $RelativePath
    $ast = [System.Management.Automation.Language.Parser]::ParseFile(
        $fullPath,
        [ref]$tokens,
        [ref]$parseErrors)
    if ($parseErrors.Count -gt 0) {
        throw "Cannot analyze $RelativePath because it has PowerShell parse errors."
    }

    $dependencies = [System.Collections.Generic.HashSet[string]]::new(
        [System.StringComparer]::OrdinalIgnoreCase)
    $stringNodes = $ast.FindAll(
        {
            param($node)
            return $node -is [System.Management.Automation.Language.StringConstantExpressionAst] -or
                $node -is [System.Management.Automation.Language.ExpandableStringExpressionAst]
        },
        $true)

    foreach ($node in $stringNodes) {
        $value = $node.Value.Trim().Replace('\', '/')
        if ($value -match '(?i)^(src|tests|data|sdk|public|tools|cmake)(/.*)?$' -or
            $value -match '(?i)^(CMakeLists\.txt|CMakePresets\.json|CMakeUserPresets\.json(?:\.template)?|vcpkg\.json)$') {
            $null = $dependencies.Add($value.TrimStart('./'))
        }
    }

    if ($dependencies.Count -eq 0) {
        throw "No repository input paths could be inferred for $RelativePath."
    }

    return @($dependencies | Sort-Object)
}

function Test-ChangedPathMatchesDependency {
    param(
        [Parameter(Mandatory)][string]$ChangedPath,
        [Parameter(Mandatory)][string]$Dependency
    )

    $changed = $ChangedPath.Replace('\', '/')
    $inputPath = $Dependency.Replace('\', '/').TrimEnd('/')
    if ($inputPath.Contains('*') -or $inputPath.Contains('?')) {
        $wildcard = [System.Management.Automation.WildcardPattern]::new(
            $inputPath,
            [System.Management.Automation.WildcardOptions]::IgnoreCase)
        return $wildcard.IsMatch($changed)
    }
    if ($changed.Equals($inputPath, [System.StringComparison]::OrdinalIgnoreCase)) {
        return $true
    }

    $fullInputPath = Join-Path $script:RepoRoot $inputPath
    $isDirectory = $inputPath -match '(?i)^(src|tests|data|sdk|public|tools|cmake)$' -or
        (Test-Path -LiteralPath $fullInputPath -PathType Container)
    return $isDirectory -and
        $changed.StartsWith($inputPath + '/', [System.StringComparison]::OrdinalIgnoreCase)
}

function Test-FullSuiteInfrastructurePath {
    param([Parameter(Mandatory)][string]$Path)

    return $Path -match '(?i)^(CMakeLists\.txt|CMakePresets\.json|CMakeUserPresets\.json(?:\.template)?|vcpkg\.json|\.gitmodules)$' -or
        $Path -match '(?i)^(cmake|external)/'
}

function Invoke-CheckedCommand {
    param(
        [Parameter(Mandatory)][string]$FilePath,
        [Parameter(Mandatory)][string[]]$Arguments
    )

    Push-Location -LiteralPath $script:RepoRoot
    try {
        & $FilePath @Arguments | Out-Host
        if ($LASTEXITCODE -ne 0) {
            throw "$FilePath failed with exit code $LASTEXITCODE."
        }
    } finally {
        Pop-Location
    }
}

if ($All -and ($BaseRef -or ($ChangedFile -and $ChangedFile.Count -gt 0))) {
    throw '-All cannot be combined with -BaseRef or -ChangedFile.'
}
if ($BaseRef -and $ChangedFile -and $ChangedFile.Count -gt 0) {
    throw '-BaseRef cannot be combined with -ChangedFile.'
}
if ($Json -and -not $PlanOnly) {
    throw '-Json is available only with -PlanOnly.'
}

# PlanOnly remains read-only. Refuse a stale catalog rather than presenting
# an incomplete target list; an executing run refreshes it before selection.
$catalogConfigured = $false
if (-not (Test-CatalogCurrent)) {
    if ($PlanOnly) {
        throw 'The CMake test catalog is missing or stale. Run cmake --preset custom-tests before requesting a plan.'
    }
    Invoke-CheckedCommand 'cmake' @('--preset', 'custom-tests')
    $catalogConfigured = $true
}
$policyDefinitions = Get-PolicyTargetDefinitions
$allPolicyTargets = @($policyDefinitions.Keys | Sort-Object)
$sourceTestPaths = @(
    Get-ChildItem (Join-Path $script:RepoRoot 'tests') -Filter '*SourceTests.ps1' |
        ForEach-Object { Normalize-RelativePath $_.FullName } |
        Sort-Object
)
$allSourceTests = @(
    $sourceTestPaths |
        ForEach-Object { [System.IO.Path]::GetFileNameWithoutExtension($_) } |
        Sort-Object
)

if ($All) {
    $changedFiles = @()
} else {
    $changedFiles = @(Get-ChangedFiles)
}
$selectedPolicy = [System.Collections.Generic.HashSet[string]]::new(
    [System.StringComparer]::OrdinalIgnoreCase)
$selectedSource = [System.Collections.Generic.HashSet[string]]::new(
    [System.StringComparer]::OrdinalIgnoreCase)
$behaviorMappedChanges = [System.Collections.Generic.HashSet[string]]::new(
    [System.StringComparer]::OrdinalIgnoreCase)
$mappedChanges = [System.Collections.Generic.HashSet[string]]::new(
    [System.StringComparer]::OrdinalIgnoreCase)
$fullSuiteReason = $null
$allPolicyReason = $null

if ($All) {
    $fullSuiteReason = 'Full suite explicitly requested.'
} elseif ($changedFiles.Count -eq 0) {
    $fullSuiteReason = 'No changed files could be determined safely.'
} else {
    $infrastructureChanges = @($changedFiles | Where-Object { Test-FullSuiteInfrastructurePath $_ })
    if ($infrastructureChanges.Count -gt 0) {
        $fullSuiteReason = 'Build or test infrastructure changed: ' + ($infrastructureChanges -join ', ')
    } else {
        $missingRelevantChanges = @(
            $changedFiles |
                Where-Object {
                    $_ -match '(?i)^(src|tests|data|sdk|public|tools)/' -and
                    -not (Test-Path -LiteralPath (Join-Path $script:RepoRoot $_))
                }
        )
        if ($missingRelevantChanges.Count -gt 0) {
            $fullSuiteReason = 'Deleted or unavailable relevant inputs require the full suite: ' +
                ($missingRelevantChanges -join ', ')
        }
    }
}

if (-not $fullSuiteReason) {
    $commonDependencies = [System.Collections.Generic.HashSet[string]]::new(
        [System.StringComparer]::OrdinalIgnoreCase)
    foreach ($supportSource in @('tests/CommonLibPolicyPCH.h', 'tests/RockConfigPolicyStub.cpp')) {
        foreach ($dependency in Get-LocalIncludeClosure $supportSource) {
            $null = $commonDependencies.Add($dependency)
        }
    }
    $commonChanges = @($changedFiles | Where-Object { $commonDependencies.Contains($_) })
    if ($commonChanges.Count -gt 0) {
        $allPolicyReason = 'Shared policy-test support changed: ' + ($commonChanges -join ', ')
        foreach ($changed in $commonChanges) {
            $null = $mappedChanges.Add($changed)
            $null = $behaviorMappedChanges.Add($changed)
        }
        foreach ($target in $allPolicyTargets) {
            $null = $selectedPolicy.Add($target)
        }
    } else {
        foreach ($target in $allPolicyTargets) {
            $targetDependencies = [System.Collections.Generic.HashSet[string]]::new(
                [System.StringComparer]::OrdinalIgnoreCase)
            foreach ($targetSource in $policyDefinitions[$target]) {
                foreach ($dependency in Get-LocalIncludeClosure $targetSource) {
                    $null = $targetDependencies.Add($dependency)
                }
            }
            foreach ($changed in $changedFiles) {
                if ($targetDependencies.Contains($changed)) {
                    $null = $selectedPolicy.Add($target)
                    $null = $mappedChanges.Add($changed)
                    $null = $behaviorMappedChanges.Add($changed)
                }
            }
        }
    }

    try {
        foreach ($sourceTestPath in $sourceTestPaths) {
            $testName = [System.IO.Path]::GetFileNameWithoutExtension($sourceTestPath)
            $dependencies = @(Get-SourceTestDependencies $sourceTestPath)
            foreach ($changed in $changedFiles) {
                $affected = $changed.Equals(
                    $sourceTestPath,
                    [System.StringComparison]::OrdinalIgnoreCase)
                if (-not $affected) {
                    foreach ($dependency in $dependencies) {
                        if (Test-ChangedPathMatchesDependency $changed $dependency) {
                            $affected = $true
                            break
                        }
                    }
                }
                if ($affected) {
                    $null = $selectedSource.Add($testName)
                    $null = $mappedChanges.Add($changed)
                }
            }
        }
    } catch {
        $fullSuiteReason = $_.Exception.Message
    }

    if (-not $fullSuiteReason) {
        $unmappedRelevantChanges = @(
            $changedFiles |
                Where-Object {
                    $_ -match '(?i)^(src|tests|data|sdk|public|tools)/' -and
                    -not $mappedChanges.Contains($_)
                }
        )
        if ($unmappedRelevantChanges.Count -gt 0) {
            $fullSuiteReason = 'Relevant changes could not be mapped safely: ' +
                ($unmappedRelevantChanges -join ', ')
        }
    }
}

if ($fullSuiteReason) {
    foreach ($target in $allPolicyTargets) {
        $null = $selectedPolicy.Add($target)
    }
    foreach ($testName in $allSourceTests) {
        $null = $selectedSource.Add($testName)
    }
}

$policyTargets = @($selectedPolicy | Sort-Object)
$sourceTests = @($selectedSource | Sort-Object)
$sdkSelected = [bool]($fullSuiteReason -or @($changedFiles | Where-Object {
    $_ -match '(?i)^(src/api|public|sdk)/'
}).Count -gt 0)
$sdkBuildTargets = if ($sdkSelected) { @('ROCKSDKExamplePlugins') } else { @() }
$sdkTests = if ($sdkSelected) { @('RpsSdkRockContractTests', 'ROCKSDKExampleBehaviorTests') } else { @() }
$testNames = @($policyTargets + $sourceTests + $sdkTests | Sort-Object -Unique)
$uncoveredBehavior = if ($fullSuiteReason) {
    # Full selection is conservative; it cannot establish integration coverage.
    @($changedFiles | Where-Object { $_ -match '(?i)^src/.*\.(cpp|h|inl)$' })
} else {
    @($changedFiles | Where-Object {
        $_ -match '(?i)^src/.*\.(cpp|h|inl)$' -and -not $behaviorMappedChanges.Contains($_)
    })
}
if ($policyTargets.Count -eq $allPolicyTargets.Count) {
    $policyBuildTargets = @('ROCKPolicyTestBinaries')
} else {
    $policyBuildTargets = @($policyTargets)
}
$hasInfrastructureChange = @(
    $changedFiles | Where-Object { Test-FullSuiteInfrastructurePath $_ }
).Count -gt 0
$hasSourceRegistrationChange = @(
    $changedFiles | Where-Object { $_ -match '(?i)^tests/.*SourceTests\.ps1$' }
).Count -gt 0
$configureRequired = -not (Test-Path (Join-Path $script:BuildDirectory 'ROCK.sln')) -or
    -not (Test-Path (Join-Path $script:BuildDirectory 'CTestTestfile.cmake')) -or
    $hasInfrastructureChange -or
    $hasSourceRegistrationChange

$plan = [pscustomobject]@{
    Mode = if ($fullSuiteReason) { 'full' } else { 'affected' }
    ChangedFiles = @($changedFiles)
    FullSuiteReason = $fullSuiteReason
    AllPolicyReason = $allPolicyReason
    PolicyTargets = @($policyTargets)
    PolicyBuildTargets = @($policyBuildTargets)
    SdkBuildTargets = @($sdkBuildTargets)
    SdkTests = @($sdkTests)
    BehavioralCoverageUnverified = @($uncoveredBehavior)
    SourceTests = @($sourceTests)
    TestNames = @($testNames)
    ConfigureRequired = [bool]$configureRequired
}

if ($Json) {
    Write-Output ($plan | ConvertTo-Json -Depth 5 -Compress)
} else {
    Write-Host "ROCK test mode: $($plan.Mode)"
    if ($changedFiles.Count -gt 0) {
        Write-Host "Changed files ($($changedFiles.Count)): $($changedFiles -join ', ')"
    }
    if ($fullSuiteReason) {
        Write-Host $fullSuiteReason
    } elseif ($allPolicyReason) {
        Write-Host $allPolicyReason
    }
    Write-Host "Policy binaries selected: $($policyTargets.Count)/$($allPolicyTargets.Count)"
    Write-Host "SDK example builds selected: $sdkSelected"
    if ($uncoveredBehavior.Count -gt 0) {
        Write-Host "Behavioral coverage unverified: $($uncoveredBehavior -join ', '). Source checks and broad selection do not establish runtime coverage."
    }
    Write-Host "Source-boundary tests selected: $($sourceTests.Count)/$($allSourceTests.Count)"
}

if ($PlanOnly) {
    return
}

if ($configureRequired -and -not $catalogConfigured) {
    Invoke-CheckedCommand 'cmake' @('--preset', 'custom-tests')
}

$buildTargets = @($policyBuildTargets + $sdkBuildTargets)
if ($buildTargets.Count -gt 0) {
    $buildArguments = @(
        '--build',
        $script:BuildDirectory,
        '--config',
        'Release',
        '--target'
    )
    $buildArguments += $buildTargets
    $buildArguments += @('--', '/m:1', '/p:CL_MPCount=2')
    Invoke-CheckedCommand 'cmake' $buildArguments
}

if ($testNames.Count -eq 0) {
    Write-Host 'No registered tests consume the changed files.'
    return
}

$ctestArguments = @(
    '--test-dir',
    $script:BuildDirectory,
    '-C',
    'Release',
    '--output-on-failure',
    '-j',
    '4',
    '--no-tests=error'
)
if (-not $fullSuiteReason) {
    $escapedNames = @($testNames | ForEach-Object { [regex]::Escape($_) })
    $testRegex = '^(' + ($escapedNames -join '|') + ')$'
    $ctestArguments += @('-R', $testRegex)
}
Invoke-CheckedCommand 'ctest' $ctestArguments
