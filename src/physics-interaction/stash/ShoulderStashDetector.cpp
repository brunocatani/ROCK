#include "physics-interaction/stash/ShoulderStashDetector.h"

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/native/PhysicsUtils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace rock::shoulder_stash
{
    namespace
    {
        const RE::NiPoint3 kWorldUp{ 0.0f, 0.0f, 1.0f };
        const RE::NiPoint3 kWorldForward{ 0.0f, 1.0f, 0.0f };

        struct Candidate
        {
            bool valid = false;
            body_zone::BodyZoneKind zone = body_zone::BodyZoneKind::Unknown;
            EvidenceSource source = EvidenceSource::None;
            std::uint32_t shoulderBodyId = kInvalidBodyId;
            std::uint32_t heldBodyId = kInvalidBodyId;
            RE::NiPoint3 nearestPointGame{};
            float distanceGameUnits = (std::numeric_limits<float>::max)();
            float radiusGameUnits = 0.0f;
            float confidence = 0.0f;
        };

        struct ShoulderCapsule
        {
            bool valid = false;
            body_zone::BodyZoneKind zone = body_zone::BodyZoneKind::Unknown;
            std::uint32_t bodyId = kInvalidBodyId;
            RE::NiPoint3 startGame{};
            RE::NiPoint3 endGame{};
            float radiusGameUnits = 0.0f;
        };

        struct BodyCandidateSearchResult
        {
            Candidate candidate{};
        };

        [[nodiscard]] bool heldBodyContains(const std::vector<std::uint32_t>* heldBodyIds, std::uint32_t bodyId) noexcept
        {
            if (!heldBodyIds || bodyId == kInvalidBodyId || bodyId == body_contact_runtime::kInvalidBodyContactId) {
                return false;
            }

            return std::find(heldBodyIds->begin(), heldBodyIds->end(), bodyId) != heldBodyIds->end();
        }

        [[nodiscard]] bool validStashBodyId(std::uint32_t bodyId) noexcept
        {
            return bodyId != kInvalidBodyId && bodyId != body_contact_runtime::kInvalidBodyContactId;
        }

        [[nodiscard]] bool contactIsRecent(std::uint32_t currentFrame, std::uint32_t recordFrame, int recentFrames) noexcept
        {
            if (recentFrames < 0) {
                return false;
            }
            if (recordFrame > currentFrame) {
                return false;
            }
            return (currentFrame - recordFrame) <= static_cast<std::uint32_t>(recentFrames);
        }

        [[nodiscard]] float candidateConfidence(float distanceGameUnits, float radiusGameUnits, float thresholdGameUnits, bool sameSide) noexcept
        {
            if (thresholdGameUnits <= 0.0001f) {
                return 0.0f;
            }

            const float normalized = std::clamp(1.0f - (distanceGameUnits / thresholdGameUnits), 0.0f, 1.0f);
            const float coreBonus = distanceGameUnits <= radiusGameUnits ? 0.20f : 0.0f;
            const float sideScale = sameSide ? 1.0f : 0.82f;
            return std::clamp((normalized + coreBonus) * sideScale, 0.0f, 1.0f);
        }

        void considerCandidate(Candidate& best, const Candidate& candidate) noexcept
        {
            if (!candidate.valid) {
                return;
            }
            if (!best.valid ||
                candidate.confidence > best.confidence ||
                (candidate.confidence == best.confidence && candidate.distanceGameUnits < best.distanceGameUnits)) {
                best = candidate;
            }
        }

        [[nodiscard]] const Probe& hmdKinematicProbe(const DetectorInput& input) noexcept
        {
            return input.config.useHmdBackVolume && input.hasHmdProbe && finitePoint(input.hmdProbe.pointGame) ? input.hmdProbe : input.probe;
        }

        [[nodiscard]] bool tryBuildShoulderCapsule(
            const DetectorInput& input,
            std::uint32_t shoulderBodyId,
            body_zone::BodyZoneKind expectedZone,
            ShoulderCapsule& outCapsule)
        {
            outCapsule = {};
            if (!input.world || !input.bodyColliders || !validStashBodyId(shoulderBodyId) || !isShoulderZone(expectedZone)) {
                return false;
            }

            BodyBoneColliderMetadata metadata{};
            if (!input.bodyColliders->tryGetBodyMetadataAtomic(shoulderBodyId, metadata) ||
                !metadata.valid ||
                metadata.zone != expectedZone ||
                !isShoulderZone(metadata.zone)) {
                return false;
            }

            RE::NiTransform bodyWorld{};
            if (!tryGetBodyWorldTransform(input.world, RE::hknpBodyId{ shoulderBodyId }, bodyWorld)) {
                return false;
            }

            const float halfLength = (std::max)(metadata.lengthGameUnits * 0.5f, 0.5f);
            const float radius = (std::max)(metadata.radiusGameUnits, 0.1f);
            outCapsule.valid = true;
            outCapsule.zone = metadata.zone;
            outCapsule.bodyId = shoulderBodyId;
            outCapsule.startGame = transform_math::localPointToWorld(bodyWorld, RE::NiPoint3{ -halfLength, 0.0f, 0.0f });
            outCapsule.endGame = transform_math::localPointToWorld(bodyWorld, RE::NiPoint3{ halfLength, 0.0f, 0.0f });
            outCapsule.radiusGameUnits = radius;
            return true;
        }

        /*
         * Sustained stash contact is anchored to the held body because the grab
         * pivot can move away from the back while another point on the object
         * remains pressed into the shoulder collider. Widening the shoulder
         * capsule would make the gesture imprecise; reprojecting the original
         * contact point keeps the decision tied to the actual object surface.
         */
        void captureSustainedContactAnchor(const DetectorInput& input, const Candidate& contact, RuntimeState& runtime)
        {
            if (!contact.valid ||
                !input.world ||
                !validStashBodyId(contact.shoulderBodyId) ||
                !validStashBodyId(contact.heldBodyId) ||
                !finitePoint(contact.nearestPointGame)) {
                return;
            }

            RE::NiTransform heldWorld{};
            if (!tryGetBodyWorldTransform(input.world, RE::hknpBodyId{ contact.heldBodyId }, heldWorld)) {
                return;
            }

            runtime.sustainedZone = contact.zone;
            runtime.sustainedShoulderBodyId = contact.shoulderBodyId;
            runtime.sustainedHeldBodyId = contact.heldBodyId;
            runtime.sustainedHeldBodyLocalPointGame = transform_math::worldPointToLocal(heldWorld, contact.nearestPointGame);
            runtime.sustainedPointGame = contact.nearestPointGame;
            runtime.sustainedMissFrames = 0;
            runtime.hasSustainedContactAnchor = true;
            runtime.hasSustainedPointGame = true;
        }

        [[nodiscard]] Candidate makeSustainedMissToleranceCandidate(const DetectorInput& input, RuntimeState& runtime)
        {
            ++runtime.sustainedMissFrames;
            if (!runtime.hasSustainedPointGame ||
                !sustainedContactMissWithinTolerance(runtime.sustainedMissFrames, input.config.sustainedContactMissFrames)) {
                clearSustainedContact(runtime);
                return {};
            }

            Candidate candidate{};
            candidate.valid = true;
            candidate.zone = runtime.sustainedZone;
            candidate.source = EvidenceSource::BodyZoneSustainedContact;
            candidate.shoulderBodyId = runtime.sustainedShoulderBodyId;
            candidate.heldBodyId = runtime.sustainedHeldBodyId;
            candidate.nearestPointGame = runtime.sustainedPointGame;
            candidate.distanceGameUnits = 0.0f;
            candidate.radiusGameUnits = 0.0f;
            candidate.confidence = isSameSideShoulder(input.isLeftHand, candidate.zone) ? 0.70f : 0.58f;
            return candidate;
        }

        [[nodiscard]] Candidate findSustainedContactCandidate(const DetectorInput& input, RuntimeState& runtime)
        {
            if (!runtime.hasSustainedContactAnchor) {
                return {};
            }

            if (!heldBodyContains(input.heldBodyIds, runtime.sustainedHeldBodyId) ||
                !isShoulderZone(runtime.sustainedZone) ||
                !validStashBodyId(runtime.sustainedShoulderBodyId)) {
                clearSustainedContact(runtime);
                return {};
            }

            if (!input.world) {
                return makeSustainedMissToleranceCandidate(input, runtime);
            }

            RE::NiTransform heldWorld{};
            if (!tryGetBodyWorldTransform(input.world, RE::hknpBodyId{ runtime.sustainedHeldBodyId }, heldWorld)) {
                return makeSustainedMissToleranceCandidate(input, runtime);
            }

            ShoulderCapsule shoulder{};
            if (!tryBuildShoulderCapsule(input, runtime.sustainedShoulderBodyId, runtime.sustainedZone, shoulder)) {
                return makeSustainedMissToleranceCandidate(input, runtime);
            }

            const auto anchorWorld = transform_math::localPointToWorld(heldWorld, runtime.sustainedHeldBodyLocalPointGame);
            if (!finitePoint(anchorWorld)) {
                return makeSustainedMissToleranceCandidate(input, runtime);
            }

            const float padding = (std::max)(0.0f, input.config.exitPaddingGameUnits);
            const float threshold = shoulder.radiusGameUnits + padding;
            const float distance = pointSegmentDistance(anchorWorld, shoulder.startGame, shoulder.endGame);
            if (!std::isfinite(distance) || distance > threshold) {
                clearSustainedContact(runtime);
                return {};
            }

            runtime.sustainedMissFrames = 0;
            runtime.sustainedPointGame = anchorWorld;
            runtime.hasSustainedPointGame = true;

            Candidate candidate{};
            candidate.valid = true;
            candidate.zone = shoulder.zone;
            candidate.source = EvidenceSource::BodyZoneSustainedContact;
            candidate.shoulderBodyId = shoulder.bodyId;
            candidate.heldBodyId = runtime.sustainedHeldBodyId;
            candidate.nearestPointGame = anchorWorld;
            candidate.distanceGameUnits = distance;
            candidate.radiusGameUnits = shoulder.radiusGameUnits;
            candidate.confidence = candidateConfidence(distance, shoulder.radiusGameUnits, threshold, isSameSideShoulder(input.isLeftHand, shoulder.zone)) * 0.96f;
            return candidate;
        }

        [[nodiscard]] BodyCandidateSearchResult findBodyColliderCandidate(const DetectorInput& input, const RuntimeState& runtime)
        {
            /*
             * Body-zone evidence is retained as a backup for shoulder stash and
             * as the precise authority future holsters need. It no longer vetoes
             * the HMD-relative back volume; the caller only selects this result
             * when the HMD path has no candidate.
             */
            BodyCandidateSearchResult result{};
            if (!input.config.useBodyZoneColliders || !input.world || !input.bodyColliders || !input.bodyColliders->hasBodies()) {
                return result;
            }

            const auto bodyCount = input.bodyColliders->getBodyCount();
            for (std::uint32_t i = 0; i < bodyCount; ++i) {
                const auto bodyId = input.bodyColliders->getBodyIdAtomic(i);
                BodyBoneColliderMetadata metadata{};
                if (!input.bodyColliders->tryGetBodyMetadataAtomic(bodyId, metadata) || !metadata.valid || !isShoulderZone(metadata.zone)) {
                    continue;
                }

                RE::NiTransform bodyWorld{};
                if (!tryGetBodyWorldTransform(input.world, RE::hknpBodyId{ bodyId }, bodyWorld)) {
                    continue;
                }

                const float halfLength = (std::max)(metadata.lengthGameUnits * 0.5f, 0.5f);
                const float radius = (std::max)(metadata.radiusGameUnits, 0.1f);
                const RE::NiPoint3 start =
                    transform_math::localPointToWorld(bodyWorld, RE::NiPoint3{ -halfLength, 0.0f, 0.0f });
                const RE::NiPoint3 end =
                    transform_math::localPointToWorld(bodyWorld, RE::NiPoint3{ halfLength, 0.0f, 0.0f });

                RE::NiPoint3 nearest{};
                const float distance = pointSegmentDistance(input.probe.pointGame, start, end, &nearest);
                if (!std::isfinite(distance)) {
                    continue;
                }

                const bool continuing = runtime.candidate && runtime.zone == metadata.zone && runtime.shoulderBodyId == bodyId;
                const float padding = continuing ? input.config.exitPaddingGameUnits : input.config.enterPaddingGameUnits;
                const float threshold = radius + (std::max)(0.0f, padding);
                if (distance > threshold) {
                    continue;
                }

                Candidate candidate{};
                candidate.valid = true;
                candidate.zone = metadata.zone;
                candidate.source = EvidenceSource::BodyZoneCollider;
                candidate.shoulderBodyId = bodyId;
                candidate.nearestPointGame = nearest;
                candidate.distanceGameUnits = distance;
                candidate.radiusGameUnits = radius;
                candidate.confidence = candidateConfidence(distance, radius, threshold, isSameSideShoulder(input.isLeftHand, metadata.zone));
                considerCandidate(result.candidate, candidate);
            }

            return result;
        }

        [[nodiscard]] Candidate findContactCandidate(const DetectorInput& input)
        {
            Candidate best{};
            if (!input.bodyContacts || !input.heldBodyIds) {
                return best;
            }

            std::array<body_contact_runtime::BodyContactRecord, body_contact_runtime::kMaxBodyContactRecords> records{};
            const auto count = input.bodyContacts->snapshot(records.data(), records.size());
            for (std::size_t i = 0; i < count; ++i) {
                const auto& record = records[i];
                if (!contactIsRecent(input.contactFrame, record.frame, input.config.recentContactFrames) || !isShoulderZone(record.zone)) {
                    continue;
                }
                if (!heldBodyContains(input.heldBodyIds, record.targetBodyId)) {
                    continue;
                }

                Candidate candidate{};
                candidate.valid = true;
                candidate.zone = record.zone;
                candidate.source = EvidenceSource::BodyZoneContact;
                candidate.shoulderBodyId = record.bodyId;
                candidate.heldBodyId = record.targetBodyId;
                candidate.nearestPointGame = record.hasContactPointGame ? record.contactPointGame : input.probe.pointGame;
                candidate.distanceGameUnits = 0.0f;
                candidate.radiusGameUnits = 0.0f;
                candidate.confidence = isSameSideShoulder(input.isLeftHand, record.zone) ? 1.0f : 0.88f;
                considerCandidate(best, candidate);
            }

            return best;
        }

        [[nodiscard]] Candidate findHmdBackVolumeCandidate(
            const DetectorInput& input,
            const RuntimeState& runtime)
        {
            Candidate best{};
            if (!input.config.useHmdBackVolume || !input.hasHmdFrame || !finitePoint(input.hmdPositionWorld) || !finitePoint(input.hmdForwardWorld)) {
                return best;
            }

            const Probe& hmdProbe = hmdKinematicProbe(input);
            if (!finitePoint(hmdProbe.pointGame)) {
                return best;
            }

            const RE::NiPoint3 forward = normalizeOr(input.hmdForwardWorld, kWorldForward);
            RE::NiPoint3 right = normalizeOr(cross(forward, kWorldUp), RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
            if (lengthSquared(right) <= 0.000001f) {
                right = RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
            }

            auto considerHmdBackSide = [&](bool leftSide) {
                const auto offset = leftSide ? input.config.hmdBackLeftOffsetGameUnits : input.config.hmdBackRightOffsetGameUnits;
                const auto center =
                    add(add(add(input.hmdPositionWorld, mul(right, offset.x)), mul(forward, offset.y)), mul(kWorldUp, offset.z));
                const float radius = (std::max)(input.config.hmdBackRadiusGameUnits, 0.1f);
                const body_zone::BodyZoneKind zone = leftSide ? body_zone::BodyZoneKind::LeftShoulder : body_zone::BodyZoneKind::RightShoulder;
                const bool continuing = runtime.candidate && runtime.zone == zone && runtime.source == EvidenceSource::HmdBackVolume;
                const float padding = continuing ? input.config.exitPaddingGameUnits : input.config.enterPaddingGameUnits;
                const float threshold = radius + (std::max)(0.0f, padding);
                const float distance = length(sub(hmdProbe.pointGame, center));
                if (!std::isfinite(distance) || distance > threshold) {
                    return;
                }

                Candidate candidate{};
                candidate.valid = true;
                candidate.zone = zone;
                candidate.source = EvidenceSource::HmdBackVolume;
                candidate.shoulderBodyId = kInvalidBodyId;
                candidate.nearestPointGame = center;
                candidate.distanceGameUnits = distance;
                candidate.radiusGameUnits = radius;
                candidate.confidence = candidateConfidence(distance, radius, threshold, isSameSideShoulder(input.isLeftHand, zone));
                considerCandidate(best, candidate);
            };

            considerHmdBackSide(true);
            considerHmdBackSide(false);
            return best;
        }

        [[nodiscard]] float resolvedProbeSpeed(const DetectorInput& input, const RuntimeState& runtime) noexcept
        {
            const Probe& probe = hmdKinematicProbe(input);
            if (probe.hasVelocity) {
                return probeSpeed(probe);
            }
            if (!runtime.hasLastProbePoint || input.deltaSeconds <= 0.000001f) {
                return 0.0f;
            }
            return length(sub(probe.pointGame, runtime.lastProbePointGame)) / input.deltaSeconds;
        }
    }

    void resetRuntime(RuntimeState& state) noexcept
    {
        state = {};
    }

    Decision evaluate(const DetectorInput& input, RuntimeState& runtime)
    {
        Decision decision{};
        const float speed = resolvedProbeSpeed(input, runtime);
        decision.speedGameUnitsPerSecond = speed;

        auto updateProbeHistory = [&]() {
            const Probe& probe = hmdKinematicProbe(input);
            runtime.lastProbePointGame = probe.pointGame;
            runtime.hasLastProbePoint = finitePoint(probe.pointGame);
        };

        if (!input.config.enabled || !finitePoint(input.probe.pointGame)) {
            resetRuntime(runtime);
            updateProbeHistory();
            return decision;
        }

        if (std::isfinite(input.config.maxSpeedGameUnitsPerSecond) &&
            input.config.maxSpeedGameUnitsPerSecond > 0.0f &&
            speed > input.config.maxSpeedGameUnitsPerSecond) {
            runtime.candidate = false;
            runtime.confirmed = false;
            runtime.zone = body_zone::BodyZoneKind::Unknown;
            runtime.source = EvidenceSource::None;
            runtime.shoulderBodyId = kInvalidBodyId;
            runtime.dwellSeconds = 0.0f;
            clearSustainedContact(runtime);
            updateProbeHistory();
            return decision;
        }

        Candidate best = findHmdBackVolumeCandidate(input, runtime);
        const BodyCandidateSearchResult bodySearch = findBodyColliderCandidate(input, runtime);
        Candidate bodyBackup = bodySearch.candidate;
        const Candidate contact = findContactCandidate(input);
        if (contact.valid) {
            captureSustainedContactAnchor(input, contact, runtime);
            if (bodyBackup.valid && bodyBackup.zone == contact.zone) {
                bodyBackup.source = EvidenceSource::BodyZoneColliderAndContact;
                bodyBackup.heldBodyId = contact.heldBodyId;
                bodyBackup.confidence = (std::max)(bodyBackup.confidence, contact.confidence);
                bodyBackup.nearestPointGame = contact.nearestPointGame;
                bodyBackup.distanceGameUnits = (std::min)(bodyBackup.distanceGameUnits, contact.distanceGameUnits);
            } else {
                considerCandidate(bodyBackup, contact);
            }
        } else {
            considerCandidate(bodyBackup, findSustainedContactCandidate(input, runtime));
        }
        if (!best.valid) {
            best = bodyBackup;
        }

        if (!best.valid) {
            runtime.candidate = false;
            runtime.confirmed = false;
            runtime.zone = body_zone::BodyZoneKind::Unknown;
            runtime.source = EvidenceSource::None;
            runtime.shoulderBodyId = kInvalidBodyId;
            runtime.dwellSeconds = 0.0f;
            updateProbeHistory();
            return decision;
        }

        const bool sameCandidate =
            runtime.candidate &&
            runtime.zone == best.zone &&
            runtime.source == best.source &&
            runtime.shoulderBodyId == best.shoulderBodyId;
        const bool sameDwellCandidate =
            runtime.candidate &&
            shoulderStashDwellIdentityMatches(
                runtime.zone,
                runtime.source,
                runtime.shoulderBodyId,
                best.zone,
                best.source,
                best.shoulderBodyId);
        const bool wasCandidate = runtime.candidate;

        runtime.candidate = true;
        runtime.zone = best.zone;
        runtime.source = best.source;
        runtime.shoulderBodyId = best.shoulderBodyId;
        runtime.dwellSeconds = sameDwellCandidate ? runtime.dwellSeconds + (std::max)(0.0f, input.deltaSeconds) : 0.0f;
        runtime.confirmed =
            best.source == EvidenceSource::BodyZoneColliderAndContact ||
            best.source == EvidenceSource::BodyZoneContact ||
            best.source == EvidenceSource::BodyZoneSustainedContact ||
            runtime.dwellSeconds >= (std::max)(0.0f, input.config.minDwellSeconds);

        decision.candidate = true;
        decision.confirmedForCommit = runtime.confirmed;
        decision.enteredCandidate = !wasCandidate;
        decision.changedCandidate = !sameCandidate;
        decision.zone = best.zone;
        decision.source = best.source;
        decision.shoulderBodyId = best.shoulderBodyId;
        decision.nearestPointGame = best.nearestPointGame;
        decision.distanceGameUnits = best.distanceGameUnits;
        decision.confidence = best.confidence;
        decision.speedGameUnitsPerSecond = speed;

        updateProbeHistory();
        return decision;
    }
}
