param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Read-Text {
    param([string]$Path)
    return Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
}

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if ((Read-Text $Path) -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if ((Read-Text $Path) -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text 'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' 'kLateCullingHookCallSite\s*=\s*0xD84EE4' 'STS compatibility must use the late-culling hook site only.'
Require-Text 'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' 'GetFormArray<RE::BGSMod::Attachment::Mod>' 'STS compatibility must patch loaded OMOD scope overlay properties through TESDataHandler forms.'
Require-Text 'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' 'restoreOverlayPatch' 'STS compatibility must be able to restore cached native overlay property data when disabled.'
Require-Text 'src/physics-interaction/weapon/SeeThroughScopesPolicy.h' 'kNativeScopeOverlayTarget\s*=\s*48' 'STS compatibility must keep the native scope overlay target documented as a named policy constant.'
Require-Text 'src/RockConfig.h' 'bool rockSeeThroughScopesCompatibilityEnabled = true;' 'ROCK config must expose the factory STS compatibility gate.'
Require-Text 'src/RockConfig.h' 'bool rockSeeThroughScopesReticleAlignmentEnabled = true;' 'ROCK config must expose the reticle-alignment gate.'
Require-Text 'src/RockConfig.h' 'float rockSeeThroughScopesReticleOffsetXGameUnits = 0\.372727f;' 'ROCK config must seed reticle X offset from the tuned BetterScopes value.'
Require-Text 'src/RockConfig.h' 'float rockSeeThroughScopesReticleOffsetZGameUnits = -0\.149692f;' 'ROCK config must seed reticle Z offset from the tuned BetterScopes value.'
Require-Text 'src/RockConfig.h' 'float rockSeeThroughScopesLookDotThreshold = 0\.98f;' 'ROCK config must seed the scope look threshold from the tuned BetterScopes value.'
Require-Text 'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' 'isDLLModLoaded\("FO4VR_better_scopes\.dll"\)' 'STS compatibility must warn if the old BetterScopes DLL is loaded.'
Require-Text 'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' 'restoreReticleBaselineIfPresent\(\);[\s\S]*readNodeLocalTranslate\(reticleNode, reticleBaselineLocal, "ReticleNode baseline capture"\)[\s\S]*s_state\.reticleNode\.reset\(reticleNode\);' 'STS compatibility must restore a previous tracked reticle before switching to a new one.'
Require-Text 'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' 'rockSeeThroughScopesReticleOffsetXGameUnits' 'Reticle alignment must read ROCK config tuning instead of policy constants at runtime.'
Require-Text 'src/physics-interaction/native/NativeMemory.h' 'bool pointerRangeLooksWritable\(void\* ptr, std::size_t byteCount\);' 'Native memory helpers must expose a writable-page check for guarded runtime node mutation.'
Require-Text 'src/physics-interaction/native/NativeMemory.h' 'bool tryWriteValue\(T\* address, const T& value\)' 'Native memory helpers must expose guarded typed writes for native scene fields.'
Require-Text 'src/physics-interaction/native/NativeMemory.cpp' 'guardedCopyToMemory' 'Native memory helpers must protect writes with the same SEH boundary used for guarded reads.'
Require-Text 'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' 'nativeNodeStorageWritable\(RE::NiAVObject\* node, const char\* context\)' 'STS runtime node mutation must first reject stale or non-writable native scene objects.'
Require-Text 'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' 'ROCK_LOG_SAMPLE_WARN\(\s*Scope' 'STS stale-node diagnostics must be sampled instead of logged every frame.'
Require-Text 'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' 'writeNodeFlags\(scopeNormal, scopeNormalFlags, "ScopeNormal visibility"\)' 'STS scope visibility must use guarded native flag writes.'
Require-Text 'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' 'writeNodeLocalTranslate\(reticleNode, s_state\.reticleBaselineLocal \+ reticleOffset, "ReticleNode aligned offset"\)' 'STS reticle alignment must use guarded native transform writes.'
Require-Text 'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' 'readNodeWorldTransform\(reticleNode, reticleWorld, "ReticleNode world transform"\)' 'STS reticle alignment must read reticle world transforms through guarded native reads.'
Require-Text 'data/config/ROCK.ini' '(?m)^\s*bSeeThroughScopesCompatibilityEnabled\s*=\s*true\s*$' 'Packaged ROCK.ini must enable STS compatibility by default.'
Require-Text 'data/config/ROCK.ini' '(?m)^\s*bSeeThroughScopesReticleAlignmentEnabled\s*=\s*true\s*$' 'Packaged ROCK.ini must enable STS reticle alignment by default.'
Require-Text 'data/config/ROCK.ini' '(?m)^\s*fSeeThroughScopesReticleOffsetXGameUnits\s*=\s*0\.372727\s*$' 'Packaged ROCK.ini must expose the tuned STS reticle X offset.'
Require-Text 'data/config/ROCK.ini' '(?m)^\s*fSeeThroughScopesReticleOffsetZGameUnits\s*=\s*-0\.149692\s*$' 'Packaged ROCK.ini must expose the tuned STS reticle Z offset.'
Require-Text 'data/config/ROCK.ini' '(?m)^\s*fSeeThroughScopesLookDotThreshold\s*=\s*0\.98\s*$' 'Packaged ROCK.ini must expose the tuned STS look threshold.'
Require-Text 'tests/SeeThroughScopesPolicyTests.cpp' '3dscopes-wmsr\.esp' 'Policy tests must cover non-replacer 3dscopes plugins.'

Reject-Text 'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' '0x2804c26|hookEyeFOV|EyeFOV|scopeZoom|ZoomValues' 'ROCK STS integration must not port BetterScopes FOV/zoom behavior.'
Reject-Text 'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' 'F4VRBody|EnableFRIKScopeStateMessages|Dispatch\([^;]*15' 'ROCK STS integration must not dispatch BetterScopes scope-state messages to FRIK.'
Reject-Text 'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' 'WeaponOffset|update1stPersonArm|handleStaticGripping' 'ROCK STS integration must not port BetterScopes weapon-offset/static-grip manipulation.'
Reject-Text 'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' '->flags\.flags\s*[|&]?=' 'STS late-culling hook must not directly mutate native node flags.'
Reject-Text 'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' 'reticleNode->local\.translate\s*=' 'STS reticle alignment must not directly mutate native reticle transforms.'
Reject-Text 'src/physics-interaction/weapon/SeeThroughScopesCompatibility.cpp' 'reticleNode->world\.translate|reticleNode->parent->world|cameraNode->world|hmdNode->world' 'STS reticle alignment must not directly dereference native scene transforms that can go stale during graph rebuilds.'
Reject-Text 'src/ROCKMain.cpp' 'processPendingConfigReload\(\);\s*[\r\n]+\s*see_through_scopes::refreshRuntimeState\(\);' 'STS plugin detection must not rescan loaded plugins every frame.'

if ($failures.Count -gt 0) {
    foreach ($failure in $failures) {
        Write-Error $failure
    }
    exit 1
}

Write-Host 'See-Through Scopes source-boundary tests passed.'
