param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source {
    param([string]$RelativePath)

    return Get-Content -Raw -LiteralPath (Join-Path $Root $RelativePath)
}

function Require-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $text = Read-Source $RelativePath
    if ($text -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Reject-FunctionBodyText {
    param(
        [string]$RelativePath,
        [string]$FunctionName,
        [string]$Pattern,
        [string]$Message
    )

    $text = Read-Source $RelativePath
    $start = $text.IndexOf($FunctionName)
    if ($start -lt 0) {
        $failures.Add("$RelativePath`: missing function $FunctionName")
        return
    }

    $brace = $text.IndexOf('{', $start)
    if ($brace -lt 0) {
        $failures.Add("$RelativePath`: missing body for $FunctionName")
        return
    }

    $depth = 0
    for ($i = $brace; $i -lt $text.Length; $i++) {
        if ($text[$i] -eq '{') {
            $depth++
        } elseif ($text[$i] -eq '}') {
            $depth--
            if ($depth -eq 0) {
                $body = $text.Substring($brace, $i - $brace + 1)
                if ($body -match $Pattern) {
                    $failures.Add("$RelativePath`: $Message")
                }
                return
            }
        }
    }

    $failures.Add("$RelativePath`: unterminated body for $FunctionName")
}

Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'hasDynamicMovableStaticBodyEvidence' `
    'Movable static selection must have an explicit dynamic-body evidence helper.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'baseForm->Is\(RE::ENUM_FORM_ID::kMSTT\)[\s\S]*hasDynamicMovableStaticBodyEvidence\(hknpWorld,\s*bodyId\)' `
    'MSTT selection must be accepted only when the selected body evidence is dynamic.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'dynamic-mstt-body' `
    'Dynamic MSTT selection needs a stable classification reason for runtime diagnostics.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'motionType == physics_body_classifier::BodyMotionType::Dynamic' `
    'Dynamic MSTT evidence must use resolved hknp motion type, not just form type.'
Reject-FunctionBodyText 'src/physics-interaction/object/ObjectDetection.cpp' 'isLooseGrabbableBaseType' 'RE::ENUM_FORM_ID::kMSTT' `
    'MSTT must not be added to the unconditional loose-form whitelist; static MSTT stays blocked.'

if ($failures.Count -gt 0) {
    Write-Host 'Dynamic movable static grab source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Dynamic movable static grab source boundary passed.'
