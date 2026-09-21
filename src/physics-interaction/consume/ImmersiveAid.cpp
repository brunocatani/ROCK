#include "physics-interaction/consume/ImmersiveAid.h"

#include "physics-interaction/body/BodyBoneColliderSet.h"
#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/consume/ImmersiveAidContact.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/PhysicsLog.h"

#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESFile.h"

#include <limits>

namespace rock::immersive_aid
{
    Injector classify(RE::TESBoundObject* base)
    {
        if (!base || !base->Is(RE::ENUM_FORM_ID::kALCH)) {
            return Injector::None;
        }
        const auto* file = base->GetFile(0);
        // These are full-master forms. Preserve all 24 local bits rather than
        // using the historical saved-offset key's truncated local identity.
        return file ? classify(file->GetFilename(), base->GetFormID() & 0x00FF'FFFFu) : Injector::None;
    }

    mouth_consume::Decision evaluate(RE::hknpWorld* world,
        const BodyBoneColliderSet& bodyColliders, const Hand& hand, bool isLeft,
        std::uint64_t collisionGeneration, const game_frame_timing_policy::GameFrameTiming& timing,
        mouth_consume::RuntimeState& state)
    {
        mouth_consume::Decision decision{};
        std::span<const GrabLocalTriangle> triangles;
        RE::NiTransform meshWorld{};
        const HeldContactMeshCache* meshCache = nullptr;
        // Full mesh coverage, bounded work; never subsample away the needle.
        constexpr std::size_t kMaxContactTriangles = HeldContactMeshCache::maximumTriangles;
        if (!world || !bodyColliders.hasBodies() || bodyColliders.isRebuildPendingAtomic() ||
            !hand.getHeldBodyContactMesh(world, triangles, meshWorld, meshCache) || triangles.size() > kMaxContactTriangles ||
            !timing.valid || timing.discontinuity || timing.menuPaused) {
            if (triangles.size() > kMaxContactTriangles) {
                ROCK_LOG_SAMPLE_WARN(Hand, 5000, "Immersive aid contact unavailable: mesh has {} triangles (limit {})",
                    triangles.size(), kMaxContactTriangles);
            }
            mouth_consume::resetRuntime(state);
            return decision;
        }

        if (!meshCache || !meshCache->valid) {
            mouth_consume::resetRuntime(state);
            return decision;
        }
        const auto& minimum = meshCache->minimum;
        const auto& maximum = meshCache->maximum;

        auto zone = body_zone::BodyZoneKind::Unknown;
        const auto count = (std::min)(bodyColliders.getBodyCount(), static_cast<std::uint32_t>(kBodyBoneColliderBodyCount));
        for (std::uint32_t index = 0; index < count; ++index) {
            BodyBoneColliderMetadata metadata{};
            const auto bodyId = bodyColliders.getBodyIdAtomic(index);
            RE::NiTransform bodyWorld{};
            if (!bodyColliders.tryGetBodyMetadataAtIndexAtomic(index, bodyId, metadata) || !metadata.valid ||
                !eligibleBodyZone(metadata.zone, isLeft) ||
                !std::isfinite(metadata.lengthGameUnits) || metadata.lengthGameUnits <= 0.0f ||
                !std::isfinite(metadata.radiusGameUnits) || metadata.radiusGameUnits <= 0.0f ||
                !tryGetBodyWorldTransform(world, RE::hknpBodyId{ bodyId }, bodyWorld)) {
                continue;
            }
            // Read geometry even for suppressed bodies: no filter mutation or
            // stale contact callback is needed to observe sustained overlap.
            const float halfLength = metadata.lengthGameUnits * 0.5f;
            const auto a = transform_math::worldPointToLocal(meshWorld,
                transform_math::localPointToWorld(bodyWorld, RE::NiPoint3{ -halfLength, 0.0f, 0.0f }));
            const auto b = transform_math::worldPointToLocal(meshWorld,
                transform_math::localPointToWorld(bodyWorld, RE::NiPoint3{ halfLength, 0.0f, 0.0f }));
            const float radius = metadata.radiusGameUnits * std::abs(bodyWorld.scale / meshWorld.scale);
            if (capsuleTouchesMesh(triangles, minimum, maximum, a, b, radius, meshCache->indexed ? &meshCache->index : nullptr)) {
                zone = metadata.zone;
                decision.mouthCenterGame = bodyWorld.translate;
                break;
            }
        }

        const bool wasCandidate = state.candidate;
        decision.confirmedForCommit = advanceContact(state.injection,
            zone != body_zone::BodyZoneKind::Unknown, hand.heldGrabIdentity(), collisionGeneration, timing);
        decision.candidate = state.injection.touching;
        decision.enteredCandidate = decision.candidate && !wasCandidate;
        decision.changedCandidate = decision.candidate != wasCandidate;
        decision.confidence = static_cast<float>(std::clamp(state.injection.dwellSeconds / kContactSeconds, 0.0, 1.0));
        state.candidate = decision.candidate;
        state.confirmed = decision.confirmedForCommit;
        state.dwellSeconds = static_cast<float>(state.injection.dwellSeconds);
        if (decision.changedCandidate) {
            ROCK_LOG_SAMPLE_INFO(Hand, 1000, "{} hand immersive aid contact: active={} zone={} grab={}",
                isLeft ? "Left" : "Right", decision.candidate, body_zone::bodyZoneName(zone), hand.heldGrabIdentity());
        }
        return decision;
    }
}
