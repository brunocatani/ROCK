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

function Reject-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $text = Read-Source $RelativePath
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Require-Order {
    param(
        [string]$RelativePath,
        [string]$First,
        [string]$Second,
        [string]$Message
    )

    $text = Read-Source $RelativePath
    $firstIndex = $text.IndexOf($First)
    $secondIndex = if ($firstIndex -ge 0) { $text.IndexOf($Second, $firstIndex + $First.Length) } else { -1 }
    if ($firstIndex -lt 0 -or $secondIndex -lt 0 -or $firstIndex -ge $secondIndex) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'if\s*\(isFarSelection\)\s*\{\s*auto equipmentSelection\s*=\s*actor_equipment_grab::resolveFarActorEquipmentSelection' 'Dead actor clothing/equipment resolution must stay gated to far selection only.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'close-dead-actor-body' 'Close dead-actor selection must remain a blocked body target, not a clothing strip.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'result\.targetKind\s*!=\s*grab_target::Kind::ActorEquipment' 'Far actor equipment must not be promoted into the close-grab path.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' '!actor_equipment_grab::nodeContainsNode\(root3D,\s*hitNode,\s*64\)' 'Detached gore acceptance for NPC refs must reject hits still owned by the actor root tree.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'grab_target::isDetachedGoreLayer\(layer\)' 'Detached gore must be based on the configured dead-biped collision layer.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'BodyMotionType::Dynamic' 'Detached gore must require dynamic body evidence instead of static actor body evidence.'

Reject-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'resolveFarActorEquipmentSelection\(ref,\s*nullptr' 'Actor equipment resolution needs the hit node and must not run from the legacy ref-only grabbable path.'

Require-Text 'src/physics-interaction/actor/ActorEquipmentGrab.cpp' 'actor->GetBiped\(\)\.get\(\)' 'Far actor equipment resolution must use the actor biped runtime.'
Require-Text 'src/physics-interaction/actor/ActorEquipmentGrab.cpp' 'biped->object\[i\]' 'Far actor equipment resolution must inspect biped slots.'
Require-Text 'src/physics-interaction/actor/ActorEquipmentGrab.cpp' 'BIPED_OBJECT::kEditorCount' 'Far actor clothing resolution must stay inside authored armor/clothing slots, not weapon/grenade utility slots.'
Require-Text 'src/physics-interaction/actor/ActorEquipmentGrab.cpp' 'item->Is\(RE::ENUM_FORM_ID::kARMO\)' 'Dead actor clothing pull must not strip equipped weapons or utility biped entries.'
Require-Text 'src/physics-interaction/actor/ActorEquipmentGrab.cpp' 'partClone\.get\(\)' 'Far actor equipment resolution must bind selection to the visible biped clone.'
Require-Text 'src/physics-interaction/actor/ActorEquipmentGrab.cpp' 'GetRuntimeData\(\)\.skinInstance\.get\(\)' 'Skinned clothing selection must inspect geometry skin instances.'
Require-Text 'src/physics-interaction/actor/ActorEquipmentGrab.cpp' 'skinInstance->bonesData' 'Skinned clothing selection must inspect skin bone data.'
Require-Text 'src/physics-interaction/actor/ActorEquipmentGrab.cpp' 'skinInstance->bonesCount' 'Skinned clothing selection must guard skin bone counts.'
Require-Text 'src/physics-interaction/actor/ActorEquipmentGrab.cpp' 'collectDownstreamNodesNoCollision' 'Skinned clothing selection must use a downstream node set without crossing collision nodes.'
Require-Text 'src/physics-interaction/actor/ActorEquipmentGrab.cpp' 'stack->IsEquipped\(\)' 'Dropped clothing must be confirmed against the equipped inventory stack.'
Require-Text 'src/physics-interaction/actor/ActorEquipmentGrab.cpp' 'equippedMatchCount' 'Equipped stack matching must track ambiguous duplicate stacks.'
Require-Text 'src/physics-interaction/actor/ActorEquipmentGrab.cpp' 'preferredInstanceData\s*&&\s*equippedMatchCount\s*!=\s*1' 'Modded/legendary clothing with preferred instance data must not silently fall back when duplicate equipped stacks exist.'
Require-Text 'src/physics-interaction/actor/ActorEquipmentGrab.cpp' 'RemoveItemData\s+removeData' 'The actor-equipment executor must drop through TESObjectREFR::RemoveItemData.'
Require-Text 'src/physics-interaction/actor/ActorEquipmentGrab.cpp' 'ITEM_REMOVE_REASON::KDropping' 'Actor equipment removal must use the dropping reason.'
Require-Text 'src/physics-interaction/actor/ActorEquipmentGrab.cpp' 'removeData\.stackData\.push_back\(stack\.stackIndex\)' 'Actor equipment removal must target the verified equipped stack index.'
Require-Text 'src/physics-interaction/actor/ActorEquipmentGrab.cpp' 'actorRef->RemoveItem\(removeData\)' 'Actor equipment removal must go through the actor ref, not direct inventory mutation.'
Require-Text 'src/physics-interaction/actor/ActorEquipmentGrab.cpp' 'result\.droppedRef\s*=\s*result\.handle\.get\(\)' 'Actor equipment drop must resolve the spawned reference before handoff.'

Require-Order 'src/physics-interaction/core/PhysicsInteraction.cpp' 'dropFarActorEquipmentSelection' 'selectedObjectInteractionBlocked' 'Actor equipment must be dropped and staged before the generic actor-target blocker runs.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'beginActorEquipmentDropHandoff' 'Dropped actor equipment must enter a bounded spawned-item handoff instead of requiring same-frame physics bodies.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'advanceActorEquipmentDropHandoff' 'Actor-equipment dropped refs must be polled until their loose-object bodies are ready.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'actorEquipmentDropHandoffReady\s*&&\s*grabInput\.held' 'A held grip must continue into far pull after the actor-equipment handoff resolves.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'replaceFarActorEquipmentSelectionWithDroppedObject\(' 'Core grab input must not fail actor equipment pull only because the dropped ref lacks bodies in the drop frame.'

Require-Text 'src/physics-interaction/hand/Hand.h' 'struct ActorEquipmentDropHandoff' 'Actor-equipment drop timing must be explicit hand lifecycle state.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::beginActorEquipmentDropHandoff' 'Hand runtime must arm a spawned-item handoff after RemoveItem succeeds.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::advanceActorEquipmentDropHandoff' 'Hand runtime must advance the spawned-item handoff with bounded retries.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'hasPendingActorEquipmentDropHandoff\(\)[\s\S]*refreshSelectionHighlight\(_currentSelection\)[\s\S]*return;' 'Selection refresh must hold actor-equipment selection stable while waiting for the dropped ref.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::replaceFarActorEquipmentSelectionWithDroppedObject' 'Hand runtime must expose a bounded actor-equipment-to-loose-object handoff.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'replacement\.targetKind\s*=\s*grab_target::Kind::LooseObject' 'Actor-equipment handoff must replace the selection with a loose object.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' '_cachedFarCandidate\s*=\s*replacement' 'Actor-equipment handoff must refresh the far candidate used by dynamic pull.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'scanOptions\.requireSameResolvedRef\s*=\s*true' 'Actor-equipment handoff scans must reject bodies that resolve to a different reference.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'scanOptions\.allowUnresolvedRefBodies\s*=\s*true' 'Actor-equipment handoff scans must still accept freshly spawned bodies before native ref resolution catches up.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'selection\.targetKind\s*==\s*grab_target::Kind::ActorEquipment' 'Actor-equipment highlight must keep a separate visual-node-first path.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'playSelectionHighlightCandidate\(selection\.refr,\s*selection\.visualNode' 'Actor-equipment highlight should target the clothing visual before falling back to the actor root.'

Require-Text 'src/physics-interaction/object/GrabTargetKind.h' 'return isPhysicalRockObject\(kind\)' 'Actor-driven targets must stay out of direct dynamic pull and active grab.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'targetKind == grab_target::Kind::ActorEquipment' 'The selected-object guard must still block actor equipment from generic interaction after the drop pre-pass.'

if ($failures.Count -gt 0) {
    Write-Host 'Actor equipment grab source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Actor equipment grab source boundary passed.'
