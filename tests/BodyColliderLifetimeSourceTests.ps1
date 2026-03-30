$ErrorActionPreference = 'Stop'

<#
Generated body colliders own shape references separately from their hknp body handles. Normal
shutdown destroys bodies through the live world, while stale-world shutdown can only abandon body
handles; both paths still have to release ROCK-owned shape refs.
#>

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '..')
$path = Join-Path $repoRoot 'src/physics-interaction/body/BodyBoneColliderSet.cpp'
$content = Get-Content -LiteralPath $path -Raw
$failures = New-Object System.Collections.Generic.List[string]

function Require-Pattern {
    param(
        [string]$Pattern,
        [string]$Message
    )

    if ($content -notmatch $Pattern) {
        $script:failures.Add($Message)
    }
}

function Reject-Pattern {
    param(
        [string]$Pattern,
        [string]$Message
    )

    if ($content -match $Pattern) {
        $script:failures.Add($Message)
    }
}

Require-Pattern 'void\s+BodyBoneColliderSet::destroy\(void\*\s+bhkWorld\)[\s\S]*instance\.body\.destroy\([\s\S]*clearInstance\(instance,\s*true\);' `
    'Body collider normal destroy must release ROCK-owned shape refs after destroying bodies.'
Require-Pattern 'void\s+BodyBoneColliderSet::reset\(\)[\s\S]*clearInstance\(instance,\s*true\);' `
    'Body collider stale-world reset must release ROCK-owned shape refs even when body handles are abandoned.'
Reject-Pattern 'void\s+BodyBoneColliderSet::reset\(\)[\s\S]*clearInstance\(instance,\s*false\);' `
    'Body collider stale-world reset must not silently drop owned shape refs.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'ROCK body collider lifetime source boundary passed.'
