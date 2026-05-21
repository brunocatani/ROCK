param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        $failures.Add($Message)
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text `
    'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' `
    'formHasSeeThroughScopesSourceFile' `
    'STS overlay suppression must be gated by each OMOD source-file chain, not only by global plugin detection.'

Require-Text `
    'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' `
    'form\.sourceFiles\.array' `
    'STS overlay suppression must inspect TESForm source files so vanilla/non-STS scopes keep native overlays.'

Require-Text `
    'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' `
    'formOwnedByLoadedSeeThroughScopesPlugin[\s\S]*IsFormInMod' `
    'STS OMOD classification must fall back to loaded 3dscopes file ownership when sourceFiles are incomplete.'

Require-Text `
    'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' `
    'if\s*\(\s*!formIsSeeThroughScopesAttachmentMod\(\*omod\)\s*\)\s*\{\s*continue;\s*\}' `
    'The OMOD overlay patch loop must skip records that were not sourced from a 3dscopes plugin.'

Require-Text `
    'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' `
    'resolveEquippedScopeRoute[\s\S]*GetIndexData\(\)[\s\S]*TESForm::GetFormByID<RE::BGSMod::Attachment::Mod>' `
    'Runtime STS routing must inspect active equipped-weapon OMOD index data.'

Require-Text `
    'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' `
    'chooseEquippedScopeRoute\(\s*\{[\s\S]*activeStsScopeMods[\s\S]*activeNativeScopeMods' `
    'Equipped scope routing must use the STS-first/native-fallback policy.'

Require-Text `
    'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' `
    'alignReticle\(\s*const\s+EquippedScopeRouteSnapshot&\s+equippedScope\s*\)[\s\S]*equippedScope\.route\s*!=\s*EquippedScopeRoute::StsPreferred' `
    'STS reticle alignment must be gated by the equipped STS-preferred route.'

Require-Text `
    'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' `
    'keepScopeMeshVisible\(\s*const\s+EquippedScopeRouteSnapshot&\s+equippedScope\s*\)[\s\S]*equippedScope\.route\s*!=\s*EquippedScopeRoute::StsPreferred' `
    'STS scope mesh visibility must be gated by the equipped STS-preferred route.'

Require-Text `
    'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' `
    'restoreScopeMeshBaselineIfPresent' `
    'Native fallback must restore ROCK-forced STS scope mesh visibility before yielding to native scope behavior.'

Require-Text `
    'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' `
    'See-Through Scopes OMOD records' `
    'The overlay patch log should identify that only STS-sourced OMOD records are patched.'

Require-Text `
    'data/config/ROCK.ini' `
    'Active equipped STS scopes are preferred; native scope overlays stay enabled for non-STS scopes\.' `
    'ROCK.ini must document STS-preferred/native-fallback scope behavior.'

Require-Text `
    'src/physics-interaction/weapon/SeeThroughScopesPolicy.h' `
    'enum\s+class\s+EquippedScopeRoute[\s\S]*NativeFallback[\s\S]*StsPreferred[\s\S]*chooseEquippedScopeRoute' `
    'Pure STS scope policy must expose STS-preferred/native-fallback routing.'

Require-Text `
    'CMakeLists.txt' `
    'ROCKSeeThroughScopesPolicyTests' `
    'STS route policy tests must be part of the policy test build.'

if ($failures.Count -gt 0) {
    Write-Host 'See-through scopes compatibility source boundary failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'See-through scopes compatibility source boundary passed.' -ForegroundColor Green
