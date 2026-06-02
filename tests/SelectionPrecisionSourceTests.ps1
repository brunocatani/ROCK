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

Require-Text 'src/physics-interaction/hand/HandSelection.h' 'ShapeCastCandidateScoringInput' `
    'Selection precision must expose a pure scoring policy for compiled regression tests.'
Require-Text 'src/physics-interaction/hand/HandSelection.h' 'scoreShapeCastCandidate' `
    'Selection precision must score candidates from lateral, depth, and surface evidence.'
Require-Text 'src/physics-interaction/hand/HandSelection.h' 'shouldKeepCurrentCloseSelectionAgainstCandidate' `
    'Close-selection stickiness must use candidate confidence before falling back to distance.'
Require-Text 'src/physics-interaction/hand/HandSelection.h' 'sanitizeSelectionAimAngleDegrees' `
    'Fixed selection aim angles must be sanitized by the selection policy.'
Require-Text 'src/RockConfig.cpp' 'iCloseSelectionAngleDegrees' `
    'Runtime config loading must read the close selection aim angle.'
Require-Text 'src/RockConfig.cpp' 'iFarSelectionAngleDegrees' `
    'Runtime config loading must read the far selection aim angle.'
Require-Text 'data/config/ROCK.ini' 'iCloseSelectionAngleDegrees\s*=\s*0[\s\S]*iFarSelectionAngleDegrees\s*=\s*0' `
    'Packaged ROCK.ini must expose separate close and far fixed selection angles.'

Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'std::array<RankedSelectionCandidate,\s*selection_query_policy::kMaxShapeCastPrecisionCandidates>' `
    'Shape-cast selection must keep a bounded top-candidate shortlist instead of one mutable winner.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'insertRankedSelectionCandidate' `
    'Shape-cast selection must insert accepted hits through the bounded precision ranking path.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'scoreShapeCastCandidate' `
    'Runtime shape-cast selection must use the same scoring policy covered by compiled tests.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'selectionScore' `
    'SelectedObject telemetry must retain the confidence score used for hand stickiness.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'promotedCloseScore' `
    'Far hits promoted to close selection must recompute confidence with close-selection scales.'

Require-Text 'src/physics-interaction/hand/Hand.cpp' 'shouldKeepCurrentCloseSelectionAgainstCandidate' `
    'Hand selection switching must use confidence-scored stickiness for close-object clutter.'
Require-Text 'CMakeLists.txt' 'ROCKSelectionPrecisionPolicyTests' `
    'Selection precision policy tests must be part of ROCKPolicyTestBinaries.'

if ($failures.Count -gt 0) {
    Write-Host 'Selection precision source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Selection precision source boundary passed.'
