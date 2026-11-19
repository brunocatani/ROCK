// Everything the PhysicsInteraction implementation files in core/interaction/
// share: the topic headers below hold the helpers (physics_interaction_internal),
// and the include list after them is what those files have always received from
// here. A using-directive makes the helpers visible unqualified.
// Include only from PhysicsInteraction implementation files.
#pragma once
#include "physics-interaction/core/interaction/WorkbenchExitMenuSink.h"
#include "physics-interaction/core/interaction/GrabInteractionInputs.h"
#include "physics-interaction/core/interaction/PalmClockDiagnostics.h"
#include "physics-interaction/core/interaction/EquippedWeaponAccess.h"
#include "physics-interaction/core/interaction/ProviderRecordConversion.h"

#include "api/NativeInteractionService.h"
#include "api/StorageInteractionService.h"

#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/native/havok/NativeImpactAudio.h"

#include "api/ProviderRuntime.h"
#include "api/ProviderStatePolicy.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>

#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/api/InteractionCommandQueue.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/collision/ContactPipelinePolicy.h"
#include "physics-interaction/collision/ContactSignalSubscriptionPolicy.h"
#include "physics-interaction/consume/MouthConsumeDetector.h"
#include "physics-interaction/consume/MouthConsumePolicy.h"
#include "physics-interaction/consume/MouthConsumeTransfer.h"
#include "physics-interaction/core/PhysicsHooks.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/debug/DebugBodyOverlay.h"
#include "physics-interaction/debug/DebugVisualizationPolicy.h"
#include "physics-interaction/feedback/FeedbackHaptics.h"
#include "physics-interaction/feedback/HapticPolicy.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/GrabEvent.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/grab/GrabMassPolicy.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/grab/GrabTelemetry.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/grab/HeldMassMovement.h"
#include "physics-interaction/hand/DynamicHandCollisionPolicy.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/hand/HandSelection.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/input/GrabInputIntentPolicy.h"
#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/native/havok/BethesdaPhysicsBody.h"
#include "physics-interaction/native/havok/BodyCollisionControl.h"
#include "physics-interaction/native/havok/HavokOffsets.h"
#include "physics-interaction/native/havok/HavokRuntime.h"
#include "physics-interaction/native/havok/PhysicsRecursiveWrappers.h"
#include "physics-interaction/native/havok/PhysicsScale.h"
#include "physics-interaction/native/havok/PhysicsUtils.h"
#include "physics-interaction/native/player/CharacterControllerRuntime.h"
#include "physics-interaction/native/query/PhysicsRayCast.h"
#include "physics-interaction/native/scene/HeldScenePresentation.h"
#include "physics-interaction/native/scene/ReferenceInteraction.h"
#include "physics-interaction/native/scene/SceneRelation.h"
#include "physics-interaction/object/CarInteractionPolicy.h"
#include "physics-interaction/object/ObjectDetection.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/stash/ShoulderStashDetector.h"
#include "physics-interaction/stash/ShoulderStashPolicy.h"
#include "physics-interaction/stash/ShoulderStashTransfer.h"
#include "physics-interaction/timing/RockGameTiming.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/authored/NativeIdleGripPreharvest.h"
#include "physics-interaction/weapon/collision/DynamicWeaponCollisionPolicy.h"
#include "physics-interaction/weapon/equip/HeldWeaponEquipStatePolicy.h"
#include "physics-interaction/weapon/equip/WeaponEquipTransfer.h"
#include "physics-interaction/weapon/equip/WeaponTransitionAnimationAcceleration.h"
#include "physics-interaction/weapon/grip/LooseWeaponGripZone.h"
#include "physics-interaction/weapon/grip/WeaponAuthority.h"
#include "physics-interaction/weapon/grip/WeaponSupport.h"
#include "physics-interaction/weapon/handling/EquippedWeaponHandlingRuntime.h"
#include "physics-interaction/weapon/native/HeldWeaponInstantTransition.h"
#include "physics-interaction/weapon/native/NativeEquippedWeaponDraw.h"
#include "physics-interaction/weapon/parts/WeaponInteraction.h"
#include "physics-interaction/weapon/scope/NativeScopeSightAnchorPolicy.h"

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/ActorValueInfo.h"
#include "RE/Bethesda/BSHavok.h"
#include "RE/Bethesda/Events.h"
#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Bethesda/UI.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Havok/hknpWorld.h"

#include <windows.h>
#include "ROCKMain.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "rock_support/Fo4VrActorStatePolicy.h"
#include "rock_support/Fo4VrRuntime.h"
#include "rock_support/VRControllers.h"

namespace rock
{
    using namespace physics_interaction_internal;
}
