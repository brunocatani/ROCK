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

Require-Text 'src/physics-interaction/consume/MouthConsumePolicy.cpp' `
    'return baseForm && baseForm->Is\(RE::ENUM_FORM_ID::kALCH\);' `
    'Mouth consume eligibility must be ALCH-only.'
Reject-Text 'src/physics-interaction/consume/MouthConsumePolicy.cpp' `
    'RE::ENUM_FORM_ID::k\(INGR|BOOK\)' `
    'Mouth consume eligibility must not include ingredients or books.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'if \(consumeEligibility\.eligible\) \{' `
    'Mouth detector evaluation must be gated by consume eligibility.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'if \(consumeEligibility\.eligible && consumeDecision\.candidate\)' `
    'Mouth candidate haptics must be gated by consume eligibility.'
Require-Text 'src/physics-interaction/consume/MouthConsumeTransfer.cpp' `
    'transferToPlayerConsume\(ConsumeInput input\)[\s\S]{0,300}untransferredRef\s*=\s*std::move\(input\.heldRef\)[\s\S]*ActivateRef\([\s\S]{0,500}untransferredRef\.reset\(\);[\s\S]*EquipObject\(' `
    'Mouth consume must relinquish ROCK reference ownership before its native inventory-use dispatch.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'transferToPlayerConsume\([\s\S]{0,300}releaseOutcome\.takeRetainedReference\(\)[\s\S]{0,800}postConsumeRef\s*=\s*consumeResult\.untransferredRef\.get\(\)' `
    'Mouth consume failure recovery must use the transfer-owned reference pin.'

if ($failures.Count -gt 0) {
    Write-Host 'MouthConsumeSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'MouthConsumeSourceTests passed.' -ForegroundColor Green
