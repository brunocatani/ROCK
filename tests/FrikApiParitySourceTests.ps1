$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$repoRoot = Split-Path -Parent $PSScriptRoot
$workspaceRoot = Split-Path -Parent $repoRoot
$rockHeaderPath = Join-Path $repoRoot 'src/api/FRIKApiV2.h'
$hfrikHeaderPath = Join-Path $workspaceRoot 'hFRIK/src/api/FRIKApiV2.h'

if (-not (Test-Path -LiteralPath $hfrikHeaderPath)) {
    throw "hFRIK API header not found: $hfrikHeaderPath"
}

$rockHeader = Get-Content -LiteralPath $rockHeaderPath -Raw
$hfrikHeader = Get-Content -LiteralPath $hfrikHeaderPath -Raw

function Get-ApiVersion {
    param([string] $Text)

    $match = [regex]::Match($Text, 'FRIK_API_V2_VERSION\s*=\s*(\d+)')
    if (-not $match.Success) {
        throw 'FRIK_API_V2_VERSION not found'
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

function Get-FunctionPointerDeclarations {
    param([string] $Text)

    $names = New-Object System.Collections.Generic.List[string]
    $pattern = '(?ms)^\s*(?<return>[A-Za-z_][A-Za-z0-9_:<>\s\*&]*?)\(\s*FRIK_CALL\s*\*\s*(?<name>[A-Za-z_][A-Za-z0-9_]*)\s*\)\s*\((?<parameters>.*?)\)\s*;'
    foreach ($entry in [regex]::Matches($Text, $pattern)) {
        $declaration = '{0}({1})({2})' -f $entry.Groups['return'].Value, $entry.Groups['name'].Value, $entry.Groups['parameters'].Value
        $names.Add(($declaration -replace '\s+', ''))
    }
    return [string[]] $names
}

function Get-EnumSchema {
    param(
        [string] $Text,
        [string] $EnumName
    )

    $pattern = 'enum\s+class\s+' + [regex]::Escape($EnumName) + '\s*:\s*(?<underlying>[^\s{]+)\s*\{(?<body>.*?)\};'
    $match = [regex]::Match($Text, $pattern, [Text.RegularExpressions.RegexOptions]::Singleline)
    if (-not $match.Success) {
        throw "Enum not found: $EnumName"
    }

    $body = [regex]::Replace($match.Groups['body'].Value, '/\*.*?\*/', '', [Text.RegularExpressions.RegexOptions]::Singleline)
    $body = [regex]::Replace($body, '//.*?(\r?\n|$)', '')
    $entries = foreach ($entry in $body.Split(',')) {
        $normalized = $entry.Trim() -replace '\s+', ''
        if ($normalized) {
            $normalized
        }
    }
    return '{0}:{1}' -f ($match.Groups['underlying'].Value -replace '\s+', ''), ($entries -join ',')
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
if ($rockVersion -ne 1 -or $hfrikVersion -ne 1) {
    throw "Expected FRIK API V2 version 1. ROCK=$rockVersion hFRIK=$hfrikVersion"
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

foreach ($enumName in @('Hand', 'HandPoseKind', 'HandPoseTagState', 'Feature', 'RecoilDelivery', 'RecoilHandMask', 'LifecycleEvent')) {
    $rockSchema = Get-EnumSchema $rockHeader $enumName
    $hfrikSchema = Get-EnumSchema $hfrikHeader $enumName
    if ($rockSchema -ne $hfrikSchema) {
        throw "$enumName ABI mismatch. ROCK='$rockSchema' hFRIK='$hfrikSchema'"
    }
}

Assert-SequenceEqual 'FRIKApiV2 function pointer ABI' (Get-FunctionPointerDeclarations $rockHeader) (Get-FunctionPointerDeclarations $hfrikHeader)

