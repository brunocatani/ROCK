$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$repoRoot = Split-Path -Parent $PSScriptRoot
$workspaceRoot = Split-Path -Parent $repoRoot
$rockHeaderPath = Join-Path $repoRoot 'src/api/FRIKApi.h'
$hfrikHeaderPath = Join-Path $workspaceRoot 'hFRIK/src/api/FRIKApi.h'

if (-not (Test-Path -LiteralPath $hfrikHeaderPath)) {
    throw "hFRIK API header not found: $hfrikHeaderPath"
}

$rockHeader = Get-Content -LiteralPath $rockHeaderPath -Raw
$hfrikHeader = Get-Content -LiteralPath $hfrikHeaderPath -Raw

function Get-ApiVersion {
    param([string] $Text)

    $match = [regex]::Match($Text, 'FRIK_API_VERSION\s*=\s*(\d+)')
    if (-not $match.Success) {
        throw 'FRIK_API_VERSION not found'
    }
    return [int] $match.Groups[1].Value
}

function Get-HandPoseValues {
    param([string] $Text)

    $match = [regex]::Match($Text, 'enum\s+class\s+HandPoseKind[^{]*\{(?<body>.*?)\};', [Text.RegularExpressions.RegexOptions]::Singleline)
    if (-not $match.Success) {
        throw 'HandPoseKind enum not found'
    }

    $values = @{}
    foreach ($entry in [regex]::Matches($match.Groups['body'].Value, '([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(\d+)')) {
        $values[$entry.Groups[1].Value] = [int] $entry.Groups[2].Value
    }
    return $values
}

function Get-FunctionPointerNames {
    param([string] $Text)

    $names = New-Object System.Collections.Generic.List[string]
    foreach ($entry in [regex]::Matches($Text, 'FRIK_CALL\s*\*\s*([A-Za-z_][A-Za-z0-9_]*)')) {
        $names.Add($entry.Groups[1].Value)
    }
    return [string[]] $names
}

function Assert-SequenceEqual {
    param(
        [string] $Name,
        [string[]] $Actual,
        [string[]] $Expected
    )

    if ($Actual.Count -ne $Expected.Count) {
        throw "$Name count mismatch. ROCK=$($Actual.Count) hFRIK=$($Expected.Count)"
    }

    for ($i = 0; $i -lt $Actual.Count; $i++) {
        if ($Actual[$i] -ne $Expected[$i]) {
            throw "$Name mismatch at index $i. ROCK='$($Actual[$i])' hFRIK='$($Expected[$i])'"
        }
    }
}

$rockVersion = Get-ApiVersion $rockHeader
$hfrikVersion = Get-ApiVersion $hfrikHeader
if ($rockVersion -ne 5 -or $hfrikVersion -ne 5) {
    throw "Expected FRIK API version 5. ROCK=$rockVersion hFRIK=$hfrikVersion"
}

$rockPoseValues = Get-HandPoseValues $rockHeader
$hfrikPoseValues = Get-HandPoseValues $hfrikHeader
foreach ($poseName in @('Unset', 'Custom', 'Open', 'Pointing', 'HoldingWeapon', 'OffhandGrip', 'Attaboy', 'ThumbsUp', 'Fist', 'HoldingGun', 'HoldingMelee')) {
    if (-not $rockPoseValues.ContainsKey($poseName) -or -not $hfrikPoseValues.ContainsKey($poseName)) {
        throw "HandPoseKind value missing: $poseName"
    }
    if ($rockPoseValues[$poseName] -ne $hfrikPoseValues[$poseName]) {
        throw "HandPoseKind.$poseName mismatch. ROCK=$($rockPoseValues[$poseName]) hFRIK=$($hfrikPoseValues[$poseName])"
    }
}

Assert-SequenceEqual 'FRIKApi function pointer order' (Get-FunctionPointerNames $rockHeader) (Get-FunctionPointerNames $hfrikHeader)

