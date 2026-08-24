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
    $text = Get-Content -Raw -LiteralPath $path
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/RockConfig.cpp' `
    'constexpr auto LOGGING_SECTION\s*=\s*"Logging"' `
    'Normal log settings must have a consumer-facing section.'
Require-Text 'src/RockConfig.cpp' `
    'GetLongValue\(LOGGING_SECTION,\s*"iLogLevel",\s*rockLogLevel\)' `
    'The normal ROCK.log threshold must load from [Logging].'
Reject-Text 'src/RockConfig.cpp' `
    'GetLongValue\(DEBUG_SECTION,\s*"iLogLevel"' `
    'The normal ROCK.log threshold must not retain a hidden [Debug] loading path.'

$configPath = 'data/config/ROCK_example.ini'
$text = Get-Content -Raw -LiteralPath (Join-Path $Root $configPath)
$loggingMatch = [regex]::Match($text, '(?ms)^\[Logging\]\s*(?<body>.*?)(?=^\[[^\]]+\]|\z)')
if (!$loggingMatch.Success) {
    $failures.Add("$configPath`: Missing [Logging] section.")
} elseif ($loggingMatch.Groups['body'].Value -notmatch '(?m)^iLogLevel\s*=\s*1\s*$') {
    $failures.Add("$configPath`: iLogLevel must be present under [Logging].")
}

$debugMatch = [regex]::Match($text, '(?ms)^\[Debug\]\s*(?<body>.*?)(?=^\[[^\]]+\]|\z)')
if (!$debugMatch.Success) {
    $failures.Add("$configPath`: Missing [Debug] section.")
} elseif ($debugMatch.Groups['body'].Value -match '(?m)^iLogLevel\s*=') {
    $failures.Add("$configPath`: iLogLevel must not remain under [Debug].")
}

$consumerHeaderIndex = $text.IndexOf('; CONSUMER OPTIONS', [System.StringComparison]::Ordinal)
$loggingSectionIndex = $text.IndexOf('[Logging]', [System.StringComparison]::Ordinal)
$firstFeatureSectionIndex = $text.IndexOf('[RealisticWeapons]', [System.StringComparison]::Ordinal)
if ($consumerHeaderIndex -lt 0 -or
    $loggingSectionIndex -le $consumerHeaderIndex -or
    $firstFeatureSectionIndex -le $loggingSectionIndex) {
    $failures.Add("$configPath`: [Logging] must be the first consumer section.")
}

if ($failures.Count -gt 0) {
    Write-Host 'LoggingConfigSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure" -ForegroundColor Red
    }
    exit 1
}

Write-Host 'LoggingConfigSourceTests passed.' -ForegroundColor Green
