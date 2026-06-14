param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Path {
    param(
        [string]$RelativePath,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path)) {
        $failures.Add("$RelativePath`: $Message")
    }
}

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

function Require-FilesEqual {
    param(
        [string]$LeftRelativePath,
        [string]$RightRelativePath,
        [string]$Message
    )

    $leftPath = Join-Path $Root $LeftRelativePath
    $rightPath = Join-Path $Root $RightRelativePath
    if (-not (Test-Path -LiteralPath $leftPath) -or -not (Test-Path -LiteralPath $rightPath)) {
        $failures.Add("$LeftRelativePath / $RightRelativePath`: missing file for sync check")
        return
    }

    $left = Get-Content -Raw -LiteralPath $leftPath
    $right = Get-Content -Raw -LiteralPath $rightPath
    if ($left -cne $right) {
        $failures.Add("$LeftRelativePath / $RightRelativePath`: $Message")
    }
}

function Get-ProviderFunctionNames {
    param([string]$Text)

    $names = [System.Collections.Generic.List[string]]::new()
    foreach ($entry in [regex]::Matches($Text, 'ROCK_PROVIDER_CALL\s*\*\s*([A-Za-z_][A-Za-z0-9_]*)')) {
        $names.Add($entry.Groups[1].Value)
    }
    return [string[]]$names
}

function Require-SequenceEqual {
    param(
        [string]$Name,
        [string[]]$Actual,
        [string[]]$Expected
    )

    if ($Actual.Count -ne $Expected.Count) {
        $failures.Add("$Name`: expected $($Expected.Count) entries, found $($Actual.Count)")
        return
    }

    for ($i = 0; $i -lt $Expected.Count; ++$i) {
        if ($Actual[$i] -ne $Expected[$i]) {
            $failures.Add("$Name`: mismatch at index $i; expected '$($Expected[$i])', found '$($Actual[$i])'")
            return
        }
    }
}

Require-Path 'SDK/ROCK/include/ROCKProviderApi.h' 'Public SDK must ship the provider header.'
Require-Path 'SDK/ROCK/include/ROCKApi.h' 'Public SDK must ship the API alias header.'
Require-Path 'SDK/ROCK/docs/PublicApi.md' 'Public API documentation must be packaged with the SDK.'
Require-Path 'SDK/ROCK/docs/VersionMatrix.md' 'API version matrix must be packaged with the SDK.'
Require-Path 'SDK/ROCK/examples/MinimalProviderConsumer.cpp' 'A minimal provider consumer example must be packaged with the SDK.'

Require-FilesEqual 'src/api/ROCKProviderApi.h' 'SDK/ROCK/include/ROCKProviderApi.h' `
    'SDK provider header must stay byte-for-byte synced with the source ABI header.'
Require-FilesEqual 'src/api/ROCKApi.h' 'SDK/ROCK/include/ROCKApi.h' `
    'SDK API alias header must stay byte-for-byte synced with the source ABI header.'

Require-Text 'src/api/ROCKProviderApi.h' 'ROCK_PROVIDER_API_VERSION\s*=\s*1' `
    'Provider API version must be v1.'
Require-Text 'src/api/ROCKApi.h' 'ROCK_API_VERSION\s*=\s*rock::provider::ROCK_PROVIDER_API_VERSION' `
    'ROCKApi alias must use the same version constant as the provider API.'
Require-Text 'src/api/ROCKProviderApi.h' 'enum\s+class\s+RockProviderResultV1' `
    'v1 must expose explicit result codes.'
Require-Text 'src/api/ROCKProviderApi.h' 'struct\s+RockProviderConsumerRegistrationV1' `
    'v1 must expose consumer registration.'
Require-Text 'src/api/ROCKProviderApi.h' 'struct\s+RockProviderLimitsV1' `
    'v1 must expose provider limits.'
Require-Text 'src/api/ROCKProviderApi.h' 'ROCK_PROVIDER_MAX_CONSUMERS_V1\s*=\s*64' `
    'Public consumer registry capacity must be an explicit SDK limit.'
Require-Text 'src/api/ROCKApi.cpp' 'ROCKAPI_GetProviderApi\(\)' `
    'ROCKAPI_GetApi must return the same table as ROCKAPI_GetProviderApi.'
Require-Text 'SDK/ROCK/docs/PublicApi.md' 'rock::provider::ROCK_PROVIDER_API_VERSION' `
    'Public API initialization sample must use the namespaced v1 version constant.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'kRockIssuedOwnerTokenNamespace\s*=\s*0xA000''0000''0000''0000ull' `
    'Registered public owner tokens must be ROCK-issued and namespaced.'
Require-Text 'src/api/ROCKProviderApi.cpp' 's_externalBodies\.clearOwner\(ownerToken\)' `
    'Unregistering a consumer must release that owner external-body state.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'setExternalDiagnosticInputSuppression\(ownerToken,\s*0\)' `
    'Unregistering a consumer must release diagnostic input suppression.'
Require-Text 'src/physics-interaction/object/ExternalBodyRegistry.h' 'copyContactsForOwnerV1' `
    'Owner-filtered contact polling must be implemented in the external-body registry.'
Require-Text 'SDK/ROCK/docs/PublicApi.md' 'ROCK v1 does not expose public force-grab or force-release commands' `
    'Docs must state that command APIs are not public until the safe queue exists.'
Require-Text 'README.md' 'API v1 includes ROCK-issued owner tokens' `
    'README must point consumers to the v1 API.'
Require-Text 'cmake/package.cmake' 'SDK/ROCK' `
    'Release packaging must include the SDK directory.'
Require-Text 'cmake/package.cmake' 'src/api/ROCKProviderApi\.h' `
    'Release packaging must copy the source provider ABI header into the SDK include directory.'
Require-Text 'cmake/package.cmake' 'src/api/ROCKApi\.h' `
    'Release packaging must copy the API alias header into the SDK include directory.'

Reject-Text 'src/api/ROCKProviderApi.h' 'requestForceGrab|requestForceRelease' `
    'Public force-grab/release functions must not be exported until a real queued implementation exists.'
Reject-Text 'src/api/ROCKProviderApi.cpp' 'apiRequestForceGrab|apiRequestForceRelease' `
    'Provider glue must not expose fake immediate command stubs.'
Reject-Text 'src/api/ROCKProviderApi.h' 'getWeaponEvidenceDescriptors|RockProviderWeaponEvidenceDescriptor|getExternalContactSnapshotV1' `
    'Public API must not expose redundant shallow weapon evidence or unowned contact snapshots.'

$providerHeader = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/api/ROCKProviderApi.h')
$expectedProviderFunctions = [string[]]@(
    'getVersion',
    'getModVersion',
    'isProviderReady',
    'registerFrameCallback',
    'unregisterFrameCallback',
    'getFrameSnapshot',
    'queryWeaponContactAtPoint',
    'clearExternalBodies',
    'setOffhandInteractionReservation',
    'registerExternalBodiesV1',
    'getWeaponEvidenceDetailCountV1',
    'copyWeaponEvidenceDetailsV1',
    'getWeaponEvidenceDetailPointCountV1',
    'copyWeaponEvidenceDetailPointsV1',
    'publishDiagnosticOverlay',
    'getDiagnosticInputSnapshotV1',
    'setDiagnosticInputSuppressionV1',
    'getBodyContactSnapshotV1',
    'getPrimaryHandV1',
    'getOffhandHandV1',
    'getHandFrameV1',
    'registerConsumerV1',
    'unregisterConsumerV1',
    'getGrantedCapabilitiesV1',
    'getProviderLimitsV1',
    'getExternalContactSnapshotForOwnerV1'
)
Require-SequenceEqual 'ROCKProviderApi function pointer order' (Get-ProviderFunctionNames $providerHeader) $expectedProviderFunctions

if ($failures.Count -gt 0) {
    Write-Host 'PublicApiLaunchSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'PublicApiLaunchSourceTests passed.' -ForegroundColor Green
