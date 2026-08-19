param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source {
    param([string]$Path)
    Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
}

function Require-Pattern {
    param([string]$Path, [string]$Text, [string]$Pattern, [string]$Message)
    if ($Text -notmatch $Pattern) {
        $failures.Add("$Path`: $Message")
    }
}

function Reject-Pattern {
    param([string]$Path, [string]$Text, [string]$Pattern, [string]$Message)
    if ($Text -match $Pattern) {
        $failures.Add("$Path`: $Message")
    }
}






if ($failures.Count -gt 0) {
    Write-Host 'Authored weapon-pose authority source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Authored weapon-pose authority source boundary passed.'
