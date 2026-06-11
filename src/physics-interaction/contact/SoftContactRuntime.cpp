#include "physics-interaction/contact/SoftContactRuntime.h"

#include "RockConfig.h"
#include "RockUtils.h"
#include "physics-interaction/contact/ContactTargetIdentity.h"
#include "physics-interaction/contact/SoftContactWorldPolicy.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/native/PhysicsShapeCast.h"
#include "physics-interaction/native/PhysicsUtils.h"

#include "RE/Havok/hknpAllHitsCollector.h"
#include "RE/Havok/hknpCollisionResult.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <vrcf/VRControllersManager.h>

namespace rock
{
    namespace
    {
        using soft_contact_math::CapsuleContact;
        using soft_contact_math::ContactKind;
        using soft_contact_math::ContactState;

        constexpr const char* RIGHT_SOFT_CONTACT_TAG = "ROCK_SoftContact_Right";
        constexpr const char* LEFT_SOFT_CONTACT_TAG = "ROCK_SoftContact_Left";
        constexpr float kCorrectionClearDistance = 0.001f;
        constexpr float kWorldProbeMinSweepDistanceGameUnits = 0.05f;
        constexpr float kWorldProbeRestQueryDistanceGameUnits = 1.0f;
        constexpr float kWorldProbeRestQueryCooldownSeconds = 0.08f;
        constexpr float kDefaultWorldContactPaddingGameUnits = 0.35f;
        constexpr float kDefaultWorldPostReleaseReentryMinApproachDistanceGameUnits = 0.025f;
        constexpr float kDefaultWorldCachedPlaneMaxTangentDriftGameUnits = 10.0f;
        constexpr float kDefaultWorldCachedPlaneMaxClearDistanceGameUnits = 18.0f;
        constexpr std::uint32_t kNativeWorldContactMaxAgeFrames = 2;
        constexpr std::uint32_t kWorldProbeIdRightBase = 0x5000u;
        constexpr std::uint32_t kWorldProbeIdLeftBase = 0x6000u;
        constexpr float kWorldContactDuplicateNormalDot = 0.92f;
        constexpr std::uint32_t kWorldContactSolverPasses = 3;

        enum class CandidateSource : std::uint8_t
        {
            QueryWorld = 0,
            CachedWorldPlane,
            NativeWorld,
        };

        struct Candidate
        {
            bool valid = false;
            bool suppressed = false;
            ContactKind kind = ContactKind::None;
            CandidateSource source = CandidateSource::QueryWorld;
            CapsuleContact contact{};
            float approachSpeedGameUnits = 0.0f;
            contact_target_identity::ContactTargetIdentity targetIdentity{};
        };

        struct CandidateManifold
        {
            std::array<Candidate, SoftContactRuntime::kMaxWorldContactManifoldContactsPerHand> contacts{};
            std::uint32_t count = 0;
        };

        struct CachedWorldPlaneContactsResult
        {
            CandidateManifold manifold{};
            bool hadCachedPlane = false;
            bool leftContact = false;
            std::array<RE::NiPoint3, SoftContactRuntime::kMaxWorldContactManifoldContactsPerHand> releasedNormals{};
            std::uint32_t releasedNormalCount = 0;
        };

        struct WorldContactProbe
        {
            bool valid = false;
            RE::NiPoint3 position{};
            float radius = 0.0f;
            std::uint32_t id = 0;
            std::size_t stateIndex = 0;
        };

        contact_target_identity::ContactTargetResolutionOptions contactTargetResolutionOptions()
        {
            /*
             * Contact solving only needs cheap body/filter/motion identity.
             * Ref traversal and string copies are diagnostics for mapping world
             * contacts, so keep them behind the explicit identity log toggle.
             */
            const bool richDiagnostics = g_rockConfig.rockDebugContactTargetIdentityLogging;
            return contact_target_identity::ContactTargetResolutionOptions{
                .resolveReference = richDiagnostics,
                .includeRichText = richDiagnostics,
            };
        }

        const char* softContactTag(bool isLeft)
        {
            return isLeft ? LEFT_SOFT_CONTACT_TAG : RIGHT_SOFT_CONTACT_TAG;
        }

        std::size_t handIndex(bool isLeft)
        {
            return isLeft ? 1u : 0u;
        }

        std::uint32_t makeWorldProbeId(bool isLeft, std::uint32_t ordinal)
        {
            return (isLeft ? kWorldProbeIdLeftBase : kWorldProbeIdRightBase) + ordinal;
        }

        void addWorldContactProbe(
            std::array<WorldContactProbe, SoftContactRuntime::kMaxWorldContactProbesPerHand>& outProbes,
            std::uint32_t& outCount,
            const RE::NiPoint3& position,
            float radius,
            bool isLeft)
        {
            if (outCount >= outProbes.size() || !soft_contact_math::isFinite(position) || !std::isfinite(radius) || radius <= 0.0f) {
                return;
            }

            auto& probe = outProbes[outCount];
            probe.valid = true;
            probe.position = position;
            probe.radius = radius;
            probe.id = makeWorldProbeId(isLeft, outCount);
            probe.stateIndex = outCount;
            ++outCount;
        }

        void buildWorldContactProbes(
            bool isLeft,
            const HandFrameInput& handInput,
            std::array<WorldContactProbe, SoftContactRuntime::kMaxWorldContactProbesPerHand>& outProbes,
            std::uint32_t& outCount)
        {
            outProbes = {};
            outCount = 0;
            if (handInput.disabled) {
                return;
            }

            addWorldContactProbe(outProbes, outCount, handInput.rawHandWorld.translate, 3.2f, isLeft);
            addWorldContactProbe(outProbes, outCount, handInput.grabAnchorWorld, 2.4f, isLeft);

            root_flattened_finger_skeleton_runtime::Snapshot snapshot{};
            if (!root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(isLeft, snapshot)) {
                return;
            }

            for (const auto& chain : snapshot.fingers) {
                if (!chain.valid) {
                    continue;
                }
                addWorldContactProbe(outProbes, outCount, chain.points[2], 1.15f, isLeft);
            }
        }

        int candidateSourcePriority(CandidateSource source)
        {
            switch (source) {
            case CandidateSource::NativeWorld:
                return 30;
            case CandidateSource::CachedWorldPlane:
                return 20;
            case CandidateSource::QueryWorld:
                return 10;
            default:
                return 0;
            }
        }

        float maxWorldCorrection();
        RE::NiPoint3 correctionForCandidate(const Candidate& candidate);

        float candidateResponseStrength(const Candidate& candidate)
        {
            return soft_contact_math::length(correctionForCandidate(candidate));
        }

        bool candidateRanksBefore(const Candidate& lhs, const Candidate& rhs)
        {
            if (!lhs.valid) {
                return false;
            }
            if (!rhs.valid) {
                return true;
            }

            const int lhsSourcePriority = candidateSourcePriority(lhs.source);
            const int rhsSourcePriority = candidateSourcePriority(rhs.source);
            if (lhsSourcePriority != rhsSourcePriority) {
                return lhsSourcePriority > rhsSourcePriority;
            }

            return soft_contact_math::preferStrongerContactResponse(
                candidateResponseStrength(lhs),
                lhs.contact.penetration,
                candidateResponseStrength(rhs),
                rhs.contact.penetration);
        }

        bool candidatesShareContactDirection(const Candidate& lhs, const Candidate& rhs)
        {
            if (!lhs.valid || !rhs.valid || !soft_contact_math::isFinite(lhs.contact.normal) || !soft_contact_math::isFinite(rhs.contact.normal)) {
                return false;
            }

            const auto lhsNormal = soft_contact_math::normalizeOr(lhs.contact.normal, RE::NiPoint3{});
            const auto rhsNormal = soft_contact_math::normalizeOr(rhs.contact.normal, RE::NiPoint3{});
            return soft_contact_math::dot(lhsNormal, rhsNormal) >= kWorldContactDuplicateNormalDot;
        }

        void addCandidateToManifold(CandidateManifold& manifold, const Candidate& candidate)
        {
            if (!candidate.valid) {
                return;
            }

            for (std::uint32_t i = 0; i < manifold.count && i < manifold.contacts.size(); ++i) {
                auto& existing = manifold.contacts[i];
                if (!candidatesShareContactDirection(existing, candidate)) {
                    continue;
                }

                if (candidateRanksBefore(candidate, existing)) {
                    existing = candidate;
                }
                return;
            }

            if (manifold.count < manifold.contacts.size()) {
                manifold.contacts[manifold.count++] = candidate;
                return;
            }

            std::uint32_t weakestIndex = 0;
            for (std::uint32_t i = 1; i < manifold.count && i < manifold.contacts.size(); ++i) {
                if (candidateRanksBefore(manifold.contacts[weakestIndex], manifold.contacts[i])) {
                    weakestIndex = i;
                }
            }
            if (candidateRanksBefore(candidate, manifold.contacts[weakestIndex])) {
                manifold.contacts[weakestIndex] = candidate;
            }
        }

        void addCandidatesToManifold(CandidateManifold& target, const CandidateManifold& source)
        {
            for (std::uint32_t i = 0; i < source.count && i < source.contacts.size(); ++i) {
                addCandidateToManifold(target, source.contacts[i]);
            }
        }

        void sortCandidateManifold(CandidateManifold& manifold)
        {
            std::sort(manifold.contacts.begin(), manifold.contacts.begin() + manifold.count, candidateRanksBefore);
        }

        bool hasWorldContact(const CandidateManifold& manifold)
        {
            return manifold.count > 0;
        }

        float maxApproachSpeed(const CandidateManifold& manifold)
        {
            float approachSpeed = 0.0f;
            for (std::uint32_t i = 0; i < manifold.count && i < manifold.contacts.size(); ++i) {
                const float candidateSpeed = manifold.contacts[i].approachSpeedGameUnits;
                if (std::isfinite(candidateSpeed)) {
                    approachSpeed = std::max(approachSpeed, candidateSpeed);
                }
            }
            return approachSpeed;
        }

        bool anyPenetratingContact(const CandidateManifold& manifold, float threshold)
        {
            for (std::uint32_t i = 0; i < manifold.count && i < manifold.contacts.size(); ++i) {
                const float penetration = manifold.contacts[i].contact.penetration;
                if (std::isfinite(penetration) && penetration >= threshold) {
                    return true;
                }
            }
            return false;
        }

        void clearWorldContactState(auto& handState)
        {
            for (auto& probe : handState.worldProbes) {
                probe = {};
            }
            for (auto& cachedPlane : handState.cachedWorldPlanes) {
                cachedPlane = {};
            }
            handState.worldHaptic = {};
        }

        float worldContactPadding()
        {
            return soft_contact_math::sanitizeNonNegative(
                g_rockConfig.rockSoftContactWorldContactPaddingGameUnits,
                kDefaultWorldContactPaddingGameUnits);
        }

        float worldQueryRadiusPadding(float contactPadding)
        {
            return soft_contact_math::effectiveQueryPadding(
                g_rockConfig.rockSoftContactWorldRadiusPaddingGameUnits,
                contactPadding,
                1.5f,
                kDefaultWorldContactPaddingGameUnits);
        }

        float worldPostReleaseReentryMinApproachDistance()
        {
            return soft_contact_math::sanitizeNonNegative(
                g_rockConfig.rockSoftContactWorldPostReleaseReentryMinApproachDistanceGameUnits,
                kDefaultWorldPostReleaseReentryMinApproachDistanceGameUnits);
        }

        float worldCachedPlaneMaxTangentDrift()
        {
            return soft_contact_math::sanitizePositive(
                g_rockConfig.rockSoftContactWorldCachedPlaneMaxTangentDriftGameUnits,
                kDefaultWorldCachedPlaneMaxTangentDriftGameUnits);
        }

        float worldCachedPlaneMaxClearDistance()
        {
            return soft_contact_math::sanitizePositive(
                g_rockConfig.rockSoftContactWorldCachedPlaneMaxClearDistanceGameUnits,
                kDefaultWorldCachedPlaneMaxClearDistanceGameUnits);
        }

        float maxWorldCorrection()
        {
            return soft_contact_math::sanitizePositive(g_rockConfig.rockSoftContactWorldMaxCorrectionGameUnits, 18.0f);
        }

        float softContactReleaseLerpMinTime()
        {
            return std::clamp(std::isfinite(g_rockConfig.rockSoftContactWorldReleaseLerpTimeMin) ? g_rockConfig.rockSoftContactWorldReleaseLerpTimeMin : 0.06f,
                0.0f,
                0.5f);
        }

        float softContactReleaseLerpMaxTime()
        {
            return std::clamp(std::isfinite(g_rockConfig.rockSoftContactWorldReleaseLerpTimeMax) ? g_rockConfig.rockSoftContactWorldReleaseLerpTimeMax : 0.12f,
                softContactReleaseLerpMinTime(),
                0.5f);
        }

        float softContactReleaseLerpMinDistance()
        {
            return soft_contact_math::sanitizeNonNegative(g_rockConfig.rockSoftContactWorldReleaseLerpMinDistance, 0.5f);
        }

        float softContactReleaseLerpMaxDistance()
        {
            return std::max(
                softContactReleaseLerpMinDistance(),
                soft_contact_math::sanitizeNonNegative(g_rockConfig.rockSoftContactWorldReleaseLerpMaxDistance, 18.0f));
        }

        bool isFiniteTransform(const RE::NiTransform& transform)
        {
            if (!soft_contact_math::isFinite(transform.translate) || !std::isfinite(transform.scale)) {
                return false;
            }

            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(transform.rotate.entry[row][column])) {
                        return false;
                    }
                }
            }
            return true;
        }

        float responseScaleForCandidate(const Candidate& candidate)
        {
            return candidate.contact.penetration > 0.0f ? 1.0f : 0.0f;
        }

        RE::NiPoint3 correctionForCandidate(const Candidate& candidate)
        {
            return soft_contact_math::projectTrackedMagnetCorrection(
                candidate.contact.normal,
                candidate.contact.penetration,
                maxWorldCorrection());
        }

        RE::NiPoint3 correctionForManifold(const CandidateManifold& manifold)
        {
            std::array<CapsuleContact, SoftContactRuntime::kMaxWorldContactManifoldContactsPerHand> contacts{};
            std::uint32_t count = 0;
            for (std::uint32_t i = 0; i < manifold.count && i < manifold.contacts.size(); ++i) {
                const auto& candidate = manifold.contacts[i];
                if (!candidate.valid || !candidate.contact.active || count >= contacts.size()) {
                    continue;
                }
                contacts[count++] = candidate.contact;
            }

            return soft_contact_math::projectTrackedMagnetPlaneSetCorrection(
                contacts,
                count,
                maxWorldCorrection(),
                kWorldContactSolverPasses);
        }

        std::uint32_t resolveWorldHitFilterInfo(RE::hknpWorld* world, const RE::hknpCollisionResult& hit)
        {
            const std::uint32_t shapeFilterInfo = hit.hitBodyInfo.m_shapeCollisionFilterInfo.storage;
            if (soft_contact_world_policy::acceptsWorldSurfaceFilterInfo(shapeFilterInfo)) {
                return shapeFilterInfo;
            }

            const auto bodyId = hit.hitBodyInfo.m_bodyId;
            if (bodyId.value == soft_contact_world_policy::kInvalidWorldBodyId || !bodySlotLooksReadable(world, bodyId)) {
                return shapeFilterInfo;
            }

            auto* body = havok_runtime::getBody(world, bodyId);
            return body ? body->collisionFilterInfo : shapeFilterInfo;
        }

        const char* candidateSourceName(CandidateSource source)
        {
            switch (source) {
            case CandidateSource::QueryWorld:
                return "QueryWorld";
            case CandidateSource::CachedWorldPlane:
                return "CachedWorldPlane";
            case CandidateSource::NativeWorld:
                return "NativeWorld";
            default:
                return "Unknown";
            }
        }

        SoftContactDebugSource debugSourceForCandidateSource(CandidateSource source)
        {
            switch (source) {
            case CandidateSource::QueryWorld:
                return SoftContactDebugSource::QueryWorld;
            case CandidateSource::CachedWorldPlane:
                return SoftContactDebugSource::CachedWorldPlane;
            case CandidateSource::NativeWorld:
                return SoftContactDebugSource::NativeWorld;
            default:
                return SoftContactDebugSource::Unknown;
            }
        }

        Candidate makeWorldStaticCandidate(RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            const RE::hknpCollisionResult& hit,
            const WorldContactProbe& probe,
            const RE::NiPoint3& previousProbePosition,
            const RE::NiPoint3& fallbackNormal,
            float radiusPadding,
            float skin,
            float deltaSeconds)
        {
            Candidate candidate{};
            const auto hitBodyId = hit.hitBodyInfo.m_bodyId;
            if (hitBodyId.value == soft_contact_world_policy::kInvalidWorldBodyId) {
                return candidate;
            }

            const RE::NiPoint3 hitPoint = hkVectorToNiPoint(hit.position);
            const RE::NiPoint3 sweepDelta = soft_contact_math::sub(probe.position, previousProbePosition);
            const RE::NiPoint3 rawHitNormal(hit.normal.x, hit.normal.y, hit.normal.z);
            const RE::NiPoint3 castFallbackNormal = soft_contact_math::normalizeOr(soft_contact_math::negate(sweepDelta), fallbackNormal);
            const RE::NiPoint3 surfaceNormal =
                soft_contact_math::orientNormalTowardPoint(hitPoint, rawHitNormal, probe.position, castFallbackNormal);

            const float effectiveProbeRadius =
                probe.radius + soft_contact_math::sanitizeNonNegative(radiusPadding, 0.0f);
            auto contact = soft_contact_math::solvePointPlaneContact(
                probe.position,
                hitPoint,
                surfaceNormal,
                effectiveProbeRadius,
                skin,
                probe.id,
                hitBodyId.value);
            if (!contact.active) {
                return candidate;
            }

            const float dt = std::clamp(std::isfinite(deltaSeconds) ? deltaSeconds : (1.0f / 90.0f), 1.0f / 240.0f, 0.1f);
            const RE::NiPoint3 velocity = soft_contact_math::mul(sweepDelta, 1.0f / dt);
            candidate.valid = true;
            candidate.kind = ContactKind::WorldStatic;
            candidate.source = CandidateSource::QueryWorld;
            candidate.contact = contact;
            candidate.approachSpeedGameUnits = std::max(0.0f, -soft_contact_math::dot(velocity, contact.normal));
            candidate.targetIdentity = contact_target_identity::resolveContactTarget(
                bhkWorld,
                hknpWorld,
                hitBodyId.value,
                contact_evidence::NativeContactEndpointKind::WorldSurface,
                &hitPoint,
                &surfaceNormal,
                contactTargetResolutionOptions());
            return candidate;
        }

        const WorldContactProbe* findClosestWorldProbe(
            const std::array<WorldContactProbe, SoftContactRuntime::kMaxWorldContactProbesPerHand>& probes,
            std::uint32_t probeCount,
            const RE::NiPoint3& point)
        {
            const WorldContactProbe* best = nullptr;
            float bestDistanceSquared = std::numeric_limits<float>::max();
            if (!soft_contact_math::isFinite(point)) {
                return nullptr;
            }

            for (std::uint32_t i = 0; i < probeCount && i < probes.size(); ++i) {
                const auto& probe = probes[i];
                if (!probe.valid) {
                    continue;
                }
                const float distanceSquared = soft_contact_math::lengthSquared(soft_contact_math::sub(probe.position, point));
                if (!std::isfinite(distanceSquared) || distanceSquared >= bestDistanceSquared) {
                    continue;
                }
                best = &probe;
                bestDistanceSquared = distanceSquared;
            }
            return best;
        }

        Candidate makeNativeWorldStaticCandidate(
            RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            const contact_evidence::NativeContactEvidenceRecord& evidence,
            const WorldContactProbe& probe,
            auto& handState,
            const RE::NiPoint3& fallbackNormal,
            float contactPadding,
            float skin,
            float deltaSeconds)
        {
            Candidate candidate{};
            if (evidence.quality != contact_evidence::NativeContactQuality::RawPoint ||
                !soft_contact_math::isFinite(evidence.contactPointGame) ||
                !soft_contact_math::isFinite(evidence.contactNormalGame)) {
                return candidate;
            }

            const RE::NiPoint3 surfaceNormal = soft_contact_math::orientNormalTowardPoint(
                evidence.contactPointGame,
                evidence.contactNormalGame,
                probe.position,
                fallbackNormal);
            const float effectiveProbeRadius = probe.radius + soft_contact_math::sanitizeNonNegative(contactPadding, 0.0f);
            auto contact = soft_contact_math::solvePointPlaneContact(
                probe.position,
                evidence.contactPointGame,
                surfaceNormal,
                effectiveProbeRadius,
                skin,
                probe.id,
                evidence.targetBodyId);
            if (!contact.active) {
                return candidate;
            }

            float approachSpeed = 0.0f;
            if (soft_contact_math::isFinite(evidence.sourceVelocityGame)) {
                approachSpeed = std::max(0.0f, -soft_contact_math::dot(evidence.sourceVelocityGame, contact.normal));
            }
            if (approachSpeed <= 0.0f && probe.stateIndex < handState.worldProbes.size() && handState.worldProbes[probe.stateIndex].valid) {
                const float dt = std::clamp(std::isfinite(deltaSeconds) ? deltaSeconds : (1.0f / 90.0f), 1.0f / 240.0f, 0.1f);
                const auto sweepDelta = soft_contact_math::sub(probe.position, handState.worldProbes[probe.stateIndex].previous);
                const auto velocity = soft_contact_math::mul(sweepDelta, 1.0f / dt);
                approachSpeed = std::max(0.0f, -soft_contact_math::dot(velocity, contact.normal));
            }

            candidate.valid = true;
            candidate.kind = ContactKind::WorldStatic;
            candidate.source = CandidateSource::NativeWorld;
            candidate.contact = contact;
            candidate.approachSpeedGameUnits = approachSpeed;
            candidate.targetIdentity = contact_target_identity::resolveContactTarget(
                bhkWorld,
                hknpWorld,
                evidence.targetBodyId,
                evidence.targetKind,
                &evidence.contactPointGame,
                &surfaceNormal,
                contactTargetResolutionOptions());
            if (candidate.targetIdentity.filterInfo == contact_target_identity::kUnknownFilterInfo &&
                evidence.targetFilterInfo != contact_evidence::kUnknownFilterInfo) {
                candidate.targetIdentity.filterInfo = evidence.targetFilterInfo;
                candidate.targetIdentity.layer = evidence.targetLayer;
            }
            return candidate;
        }

        void collectNativeWorldStaticContacts(
            CandidateManifold& manifold,
            RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            const contact_evidence::NativeContactEvidenceSnapshot& evidenceSnapshot,
            auto& handState,
            bool isLeft,
            const std::array<WorldContactProbe, SoftContactRuntime::kMaxWorldContactProbesPerHand>& probes,
            std::uint32_t probeCount,
            const RE::NiPoint3& fallbackNormal,
            float contactPadding,
            float skin,
            float deltaSeconds)
        {
            using contact_evidence::NativeContactEndpointKind;
            for (std::uint32_t i = 0; i < evidenceSnapshot.count && i < evidenceSnapshot.records.size(); ++i) {
                const auto& evidence = evidenceSnapshot.records[i];
                if (!evidence.valid ||
                    !contact_evidence::isFrameFresh(evidenceSnapshot.currentFrame, evidence.frame, kNativeWorldContactMaxAgeFrames) ||
                    evidence.targetKind != NativeContactEndpointKind::WorldSurface ||
                    evidence.quality != contact_evidence::NativeContactQuality::RawPoint ||
                    evidence.sourceIsLeft != isLeft) {
                    continue;
                }

                const bool sourceIsExpectedHand =
                    (!isLeft && evidence.sourceKind == NativeContactEndpointKind::RightHand) ||
                    (isLeft && evidence.sourceKind == NativeContactEndpointKind::LeftHand);
                if (!sourceIsExpectedHand) {
                    continue;
                }

                const auto* probe = findClosestWorldProbe(probes, probeCount, evidence.contactPointGame);
                if (!probe) {
                    continue;
                }

                addCandidateToManifold(manifold,
                    makeNativeWorldStaticCandidate(bhkWorld, hknpWorld, evidence, *probe, handState, fallbackNormal, contactPadding, skin, deltaSeconds));
            }
        }

        void addReleasedCachedPlaneNormal(CachedWorldPlaneContactsResult& result, const RE::NiPoint3& normal)
        {
            result.leftContact = true;
            if (result.releasedNormalCount >= result.releasedNormals.size() || !soft_contact_math::isFinite(normal)) {
                return;
            }
            result.releasedNormals[result.releasedNormalCount++] = normal;
        }

        CachedWorldPlaneContactsResult solveCachedWorldPlaneContacts(auto& handState,
            const std::array<WorldContactProbe, SoftContactRuntime::kMaxWorldContactProbesPerHand>& probes,
            std::uint32_t probeCount,
            float contactPadding,
            float skin,
            float maxTangentDrift,
            float maxClearDistance,
            float deltaSeconds)
        {
            CachedWorldPlaneContactsResult result{};
            for (auto& cachedPlane : handState.cachedWorldPlanes) {
                if (!cachedPlane.active) {
                    continue;
                }
                result.hadCachedPlane = true;

                const WorldContactProbe* probe = nullptr;
                for (std::uint32_t i = 0; i < probeCount && i < probes.size(); ++i) {
                    if (probes[i].valid && probes[i].id == cachedPlane.probeId) {
                        probe = &probes[i];
                        break;
                    }
                }

                auto releaseCachedPlane = [&]() {
                    addReleasedCachedPlaneNormal(result, cachedPlane.normal);
                    cachedPlane = {};
                };

                if (!probe) {
                    releaseCachedPlane();
                    continue;
                }

                /*
                 * Tangent drift protects sliding along a surface, but it does
                 * not catch the raw hand moving deep through the cached plane.
                 * This radial limit is the hard escape hatch for all directions.
                 */
                if (!soft_contact_math::withinClearDistanceLimit(probe->position, cachedPlane.surfacePoint, maxClearDistance)) {
                    releaseCachedPlane();
                    continue;
                }

                const float effectiveProbeRadius = probe->radius + soft_contact_math::sanitizeNonNegative(contactPadding, 0.0f);
                auto contact = soft_contact_math::solvePointPlaneContact(
                    probe->position,
                    cachedPlane.surfacePoint,
                    cachedPlane.normal,
                    effectiveProbeRadius,
                    skin,
                    probe->id,
                    cachedPlane.bodyId);

                /*
                 * This is cached evidence, not a lock. The current tracked probe
                 * must still be penetrating the cached plane this frame; otherwise
                 * the external visual authority is released immediately.
                 */
                if (!contact.active) {
                    releaseCachedPlane();
                    continue;
                }

                if (!soft_contact_math::withinTangentDriftLimit(
                        contact.targetPoint,
                        cachedPlane.surfacePoint,
                        cachedPlane.normal,
                        maxTangentDrift)) {
                    releaseCachedPlane();
                    continue;
                }

                float approachSpeed = cachedPlane.approachSpeedGameUnits;
                if (probe->stateIndex < handState.worldProbes.size() && handState.worldProbes[probe->stateIndex].valid) {
                    const float dt = std::clamp(std::isfinite(deltaSeconds) ? deltaSeconds : (1.0f / 90.0f), 1.0f / 240.0f, 0.1f);
                    const auto sweepDelta = soft_contact_math::sub(probe->position, handState.worldProbes[probe->stateIndex].previous);
                    const auto velocity = soft_contact_math::mul(sweepDelta, 1.0f / dt);
                    approachSpeed = std::max(0.0f, -soft_contact_math::dot(velocity, contact.normal));
                }

                Candidate candidate{};
                candidate.valid = true;
                candidate.kind = ContactKind::WorldStatic;
                candidate.source = CandidateSource::CachedWorldPlane;
                candidate.contact = contact;
                candidate.approachSpeedGameUnits = approachSpeed;
                candidate.targetIdentity = cachedPlane.targetIdentity;
                addCandidateToManifold(result.manifold, candidate);
            }
            return result;
        }

        bool normalsShareDirection(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            if (!soft_contact_math::isFinite(lhs) || !soft_contact_math::isFinite(rhs)) {
                return false;
            }
            const auto lhsNormal = soft_contact_math::normalizeOr(lhs, RE::NiPoint3{});
            const auto rhsNormal = soft_contact_math::normalizeOr(rhs, RE::NiPoint3{});
            return soft_contact_math::dot(lhsNormal, rhsNormal) >= kWorldContactDuplicateNormalDot;
        }

        bool shouldAcceptPostReleaseQueryCandidate(
            const CachedWorldPlaneContactsResult& cachedPlaneContacts,
            const RE::NiPoint3& sweepDelta,
            const Candidate& candidate,
            float reentryMinApproachDistance)
        {
            if (!cachedPlaneContacts.leftContact || !candidate.valid) {
                return true;
            }

            for (std::uint32_t i = 0; i < cachedPlaneContacts.releasedNormalCount && i < cachedPlaneContacts.releasedNormals.size(); ++i) {
                const auto& releasedNormal = cachedPlaneContacts.releasedNormals[i];
                if (!normalsShareDirection(candidate.contact.normal, releasedNormal)) {
                    continue;
                }

                return soft_contact_math::shouldAllowPostReleaseReentrySweep(
                    true,
                    sweepDelta,
                    releasedNormal,
                    reentryMinApproachDistance);
            }

            return true;
        }

        void collectWorldProbeCastContacts(CandidateManifold& manifold,
            RE::bhkWorld* bhkWorld,
            RE::hknpWorld* world,
            const WorldContactProbe& probe,
            const RE::NiPoint3& start,
            const RE::NiPoint3& displacement,
            float distance,
            const RE::NiPoint3& previousPosition,
            const RE::NiPoint3& fallbackNormal,
            float queryRadiusPadding,
            float contactPadding,
            float skin,
            const CachedWorldPlaneContactsResult& cachedPlaneContacts,
            float reentryMinApproachDistance,
            float deltaSeconds)
        {
            if (!world || !probe.valid || !std::isfinite(distance) || distance <= 0.001f) {
                return;
            }

            RE::hknpAllHitsCollector collector;
            if (!physics_shape_cast::castSelectionSphere(
                    world,
                    physics_shape_cast::SphereCastInput{
                        .startGame = start,
                        .directionGame = displacement,
                        .distanceGame = distance,
                        .radiusGame = probe.radius + queryRadiusPadding,
                        .collisionFilterInfo = g_rockConfig.rockSoftContactWorldShapeCastFilterInfo },
                    collector,
                    nullptr)) {
                return;
            }

            auto* hits = collector.hits._data;
            const int hitCount = collector.hits._size;
            for (int hitIndex = 0; hitIndex < hitCount; ++hitIndex) {
                const auto& hit = hits[hitIndex];
                const std::uint32_t filterInfo = resolveWorldHitFilterInfo(world, hit);
                if (!soft_contact_world_policy::acceptsWorldSurfaceFilterInfo(filterInfo)) {
                    continue;
                }

                const auto candidate =
                    makeWorldStaticCandidate(bhkWorld, world, hit, probe, previousPosition, fallbackNormal, contactPadding, skin, deltaSeconds);
                if (shouldAcceptPostReleaseQueryCandidate(cachedPlaneContacts, displacement, candidate, reentryMinApproachDistance)) {
                    addCandidateToManifold(manifold, candidate);
                }
            }
        }

        void storeWorldProbePositions(auto& handState,
            const std::array<WorldContactProbe, SoftContactRuntime::kMaxWorldContactProbesPerHand>& probes,
            std::uint32_t probeCount)
        {
            for (std::size_t i = 0; i < handState.worldProbes.size(); ++i) {
                if (i < probeCount && probes[i].valid) {
                    handState.worldProbes[i].valid = true;
                    handState.worldProbes[i].previous = probes[i].position;
                } else {
                    handState.worldProbes[i] = {};
                }
            }
        }

        void storeCachedWorldPlanes(auto& handState, const CandidateManifold& manifold)
        {
            for (auto& cachedPlane : handState.cachedWorldPlanes) {
                cachedPlane = {};
            }

            std::uint32_t cachedCount = 0;
            for (std::uint32_t i = 0; i < manifold.count && i < manifold.contacts.size() && cachedCount < handState.cachedWorldPlanes.size(); ++i) {
                const auto& candidate = manifold.contacts[i];
                if (!candidate.valid || candidate.kind != ContactKind::WorldStatic) {
                    continue;
                }

                auto& cachedPlane = handState.cachedWorldPlanes[cachedCount++];
                cachedPlane.active = true;
                cachedPlane.bodyId = candidate.contact.targetId;
                cachedPlane.probeId = candidate.contact.movableId;
                cachedPlane.surfacePoint = candidate.contact.targetPoint;
                cachedPlane.normal = candidate.contact.normal;
                cachedPlane.approachSpeedGameUnits = candidate.approachSpeedGameUnits;
                cachedPlane.targetIdentity = candidate.targetIdentity;
            }
        }

        CandidateManifold solveWorldStaticContact(RE::bhkWorld* bhkWorld,
            RE::hknpWorld* world,
            const contact_evidence::NativeContactEvidenceSnapshot& nativeContactEvidence,
            auto& handState,
            bool isLeft,
            const HandFrameInput& handInput,
            const RE::NiPoint3& fallbackNormal,
            float deltaSeconds)
        {
            CandidateManifold manifold{};
            if (!g_rockConfig.rockSoftContactWorldEnabled || !world || handInput.disabled) {
                clearWorldContactState(handState);
                return manifold;
            }

            std::array<WorldContactProbe, SoftContactRuntime::kMaxWorldContactProbesPerHand> probes{};
            std::uint32_t probeCount = 0;
            buildWorldContactProbes(isLeft, handInput, probes, probeCount);
            if (probeCount == 0) {
                clearWorldContactState(handState);
                return manifold;
            }

            const float contactPadding = worldContactPadding();
            const float queryRadiusPadding = worldQueryRadiusPadding(contactPadding);
            const float skin =
                soft_contact_math::sanitizeNonNegative(g_rockConfig.rockSoftContactWorldSkinGameUnits, 0.5f);
            const float reentryMinApproachDistance = worldPostReleaseReentryMinApproachDistance();
            const float maxTangentDrift = worldCachedPlaneMaxTangentDrift();
            const float maxClearDistance = worldCachedPlaneMaxClearDistance();
            const float dt = std::clamp(std::isfinite(deltaSeconds) ? deltaSeconds : (1.0f / 90.0f), 0.0f, 0.1f);

            const auto cachedPlaneContacts = solveCachedWorldPlaneContacts(
                handState,
                probes,
                probeCount,
                contactPadding,
                skin,
                maxTangentDrift,
                maxClearDistance,
                deltaSeconds);
            addCandidatesToManifold(manifold, cachedPlaneContacts.manifold);

            collectNativeWorldStaticContacts(
                manifold,
                bhkWorld,
                world,
                nativeContactEvidence,
                handState,
                isLeft,
                probes,
                probeCount,
                fallbackNormal,
                contactPadding,
                skin,
                deltaSeconds);
            const bool hadContactBeforeQuery = hasWorldContact(manifold);
            const bool releasedCachedPlaneThisFrame =
                cachedPlaneContacts.hadCachedPlane && cachedPlaneContacts.leftContact;

            if (manifold.count < manifold.contacts.size()) {
                for (std::uint32_t i = 0; i < probeCount && i < probes.size(); ++i) {
                    const auto& probe = probes[i];
                    if (!probe.valid || probe.stateIndex >= handState.worldProbes.size()) {
                        continue;
                    }

                    auto& previousState = handState.worldProbes[probe.stateIndex];
                    previousState.restQueryCooldownSeconds =
                        std::max(0.0f, previousState.restQueryCooldownSeconds - dt);

                    const RE::NiPoint3 sweepDelta = previousState.valid ?
                                                      soft_contact_math::sub(probe.position, previousState.previous) :
                                                      RE::NiPoint3{};
                    const float sweepDistance = previousState.valid ? soft_contact_math::length(sweepDelta) : 0.0f;
                    const bool canSweep =
                        previousState.valid &&
                        std::isfinite(sweepDistance) &&
                        sweepDistance > kWorldProbeMinSweepDistanceGameUnits;
                    if (canSweep) {
                        collectWorldProbeCastContacts(manifold,
                            bhkWorld,
                            world,
                            probe,
                            previousState.previous,
                            sweepDelta,
                            sweepDistance,
                            previousState.previous,
                            fallbackNormal,
                            queryRadiusPadding,
                            contactPadding,
                            skin,
                            cachedPlaneContacts,
                            reentryMinApproachDistance,
                            deltaSeconds);
                    }

                    if (canSweep || releasedCachedPlaneThisFrame || hadContactBeforeQuery || previousState.restQueryCooldownSeconds > 0.0f) {
                        continue;
                    }

                    previousState.restQueryCooldownSeconds = kWorldProbeRestQueryCooldownSeconds;
                    const RE::NiPoint3 restDirection = soft_contact_math::normalizeOr(fallbackNormal, RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
                    const RE::NiPoint3 previousPosition = previousState.valid ? previousState.previous : probe.position;
                    collectWorldProbeCastContacts(manifold,
                        bhkWorld,
                        world,
                        probe,
                        probe.position,
                        restDirection,
                        kWorldProbeRestQueryDistanceGameUnits,
                        previousPosition,
                        fallbackNormal,
                        queryRadiusPadding,
                        contactPadding,
                        skin,
                        cachedPlaneContacts,
                        reentryMinApproachDistance,
                        deltaSeconds);
                }
            }

            storeWorldProbePositions(handState, probes, probeCount);
            sortCandidateManifold(manifold);
            storeCachedWorldPlanes(handState, manifold);

            return manifold;
        }

        void updateWorldContactHaptics(
            auto& handState,
            bool isLeft,
            bool active,
            float approachSpeedGameUnits,
            float deltaSeconds)
        {
            soft_contact_math::HapticEdgeConfig config{};
            config.enabled = g_rockConfig.rockSoftContactWorldHapticsEnabled;
            config.baseIntensity = g_rockConfig.rockSoftContactWorldHapticBaseIntensity;
            config.maxIntensity = g_rockConfig.rockSoftContactWorldHapticMaxIntensity;
            config.speedScale = g_rockConfig.rockSoftContactWorldHapticSpeedScale;
            config.minApproachSpeed = g_rockConfig.rockSoftContactWorldHapticMinApproachSpeedGameUnits;
            config.cooldownSeconds = g_rockConfig.rockSoftContactWorldHapticCooldownSeconds;

            const auto decision = soft_contact_math::updateHapticEdge(handState.worldHaptic, active, approachSpeedGameUnits, deltaSeconds, config);
            const float duration = std::clamp(
                std::isfinite(g_rockConfig.rockSoftContactWorldHapticDurationSeconds) ? g_rockConfig.rockSoftContactWorldHapticDurationSeconds : 0.035f,
                0.0f,
                0.2f);
            if (decision.fire && duration > 0.0f && decision.intensity > 0.0f) {
                f4cf::vrcf::VRControllers.triggerHaptic(
                    isLeft ? f4cf::vrcf::Hand::Left : f4cf::vrcf::Hand::Right,
                    duration,
                    decision.intensity);
            }
        }

        void logContactTargetIdentity(bool isLeft, const Candidate& candidate)
        {
            if (!g_rockConfig.rockDebugContactTargetIdentityLogging || candidate.kind != ContactKind::WorldStatic) {
                return;
            }

            const auto& identity = candidate.targetIdentity;
            const auto sampleMilliseconds = std::max(1, g_rockConfig.rockDebugContactTargetIdentitySampleMilliseconds);
            const auto bodyId = contact_evidence::isValidBodyId(identity.bodyId) ? identity.bodyId : candidate.contact.targetId;
            const auto layer = identity.layer;
            const auto filterInfo = identity.filterInfo;
            const auto motionIndex = identity.motionIndex;
            const auto status = identity.status;
            const auto surfaceHint = identity.surfaceHint;
            const char* formType = identity.formType.empty() ? "???" : identity.formType.c_str();
            const char* displayName = identity.displayName.empty() ? "(unnamed)" : identity.displayName.c_str();
            const char* refEditorId = identity.refEditorId.empty() ? "(none)" : identity.refEditorId.c_str();
            const char* baseEditorId = identity.baseEditorId.empty() ? "(none)" : identity.baseEditorId.c_str();
            const auto& point = candidate.contact.targetPoint;
            const auto& normal = candidate.contact.normal;

            ROCK_LOG_SAMPLE_DEBUG(ContactTarget,
                sampleMilliseconds,
                "Contact target identity: hand={} source={} status={} endpoint={} body={} layer={} filter=0x{:08X} motion={} ref={:08X} base={:08X} type={} name='{}' refEditor='{}' baseEditor='{}' surface={} point=({:.2f},{:.2f},{:.2f}) normal=({:.3f},{:.3f},{:.3f})",
                isLeft ? "Left" : "Right",
                candidateSourceName(candidate.source),
                contact_target_identity::resolutionStatusName(status),
                contact_target_identity::endpointKindName(identity.endpointKind),
                bodyId,
                layer,
                filterInfo,
                motionIndex,
                identity.refFormId,
                identity.baseFormId,
                formType,
                displayName,
                refEditorId,
                baseEditorId,
                contact_target_identity::surfaceHintName(surfaceHint),
                point.x,
                point.y,
                point.z,
                normal.x,
                normal.y,
                normal.z);
        }

        void addDebugContact(SoftContactDebugSnapshot& snapshot, bool isLeft, ContactState state, const Candidate& candidate, const RE::NiPoint3& correction)
        {
            if (!candidate.valid || snapshot.contactCount >= snapshot.contacts.size()) {
                return;
            }

            auto& entry = snapshot.contacts[snapshot.contactCount++];
            entry.valid = true;
            entry.isLeft = isLeft;
            entry.suppressed = candidate.suppressed;
            entry.kind = candidate.kind;
            entry.source = debugSourceForCandidateSource(candidate.source);
            entry.state = state;
            entry.point = candidate.contact.movablePoint;
            entry.normalEnd = soft_contact_math::add(candidate.contact.movablePoint, soft_contact_math::mul(candidate.contact.normal, 8.0f));
            entry.correctionEnd = soft_contact_math::add(candidate.contact.movablePoint, correction);
            entry.penetration = candidate.contact.penetration;
            entry.responseScale = responseScaleForCandidate(candidate);
            entry.maxCorrection = maxWorldCorrection();
            entry.correctionLength = soft_contact_math::length(correction);
            entry.movableId = candidate.contact.movableId;
            entry.targetId = candidate.contact.targetId;
            entry.targetLayer = candidate.targetIdentity.layer;
            entry.targetFilterInfo = candidate.targetIdentity.filterInfo;
            entry.targetRefFormId = candidate.targetIdentity.refFormId;
            entry.targetBaseFormId = candidate.targetIdentity.baseFormId;
            entry.surfaceHint = candidate.targetIdentity.surfaceHint;
        }

        void beginReleaseBlend(auto& handState, const RE::NiTransform& rawHandWorld)
        {
            handState.releaseBlend = {};
            if (!g_rockConfig.rockSoftContactWorldReleaseLerpEnabled ||
                !handState.externalTransformActive ||
                !isFiniteTransform(handState.lastAppliedWorld) ||
                !isFiniteTransform(rawHandWorld)) {
                return;
            }

            const float distance = hand_visual_lerp_math::distanceGameUnits(handState.lastAppliedWorld.translate, rawHandWorld.translate);
            const float duration = hand_visual_lerp_math::computeDistanceMappedDurationGameUnits(
                distance,
                softContactReleaseLerpMinTime(),
                softContactReleaseLerpMaxTime(),
                softContactReleaseLerpMinDistance(),
                softContactReleaseLerpMaxDistance());
            if (duration <= 0.0f) {
                return;
            }

            handState.releaseBlend.active = true;
            handState.releaseBlend.startWorld = handState.lastAppliedWorld;
            handState.releaseBlend.elapsedSeconds = 0.0f;
            handState.releaseBlend.durationSeconds = duration;
        }

        enum class ReleaseBlendStep : std::uint8_t
        {
            Inactive,
            Applied,
            Finished,
            Failed
        };

        ReleaseBlendStep applyReleaseBlend(auto& handState, bool isLeft, const RE::NiTransform& rawHandWorld, float deltaSeconds)
        {
            if (!handState.releaseBlend.active) {
                beginReleaseBlend(handState, rawHandWorld);
            }
            if (!handState.releaseBlend.active) {
                return ReleaseBlendStep::Inactive;
            }

            auto& release = handState.releaseBlend;
            release.elapsedSeconds = hand_visual_lerp_math::advanceTimedBlendElapsed(
                release.elapsedSeconds,
                std::clamp(std::isfinite(deltaSeconds) ? deltaSeconds : (1.0f / 90.0f), 0.0f, 0.1f),
                release.durationSeconds);
            const auto blend = hand_visual_lerp_math::blendTransformOverDuration(
                release.startWorld,
                rawHandWorld,
                release.elapsedSeconds,
                release.durationSeconds);
            if (!frik_visual_authority::applyExternalHandWorldTransform(
                    softContactTag(isLeft),
                    handFromBool(isLeft),
                    blend.transform,
                    g_rockConfig.rockSoftContactVisualPriority)) {
                return ReleaseBlendStep::Failed;
            }

            handState.lastAppliedWorld = blend.transform;
            handState.externalTransformActive = true;
            handState.correction = soft_contact_math::sub(blend.transform.translate, rawHandWorld.translate);
            handState.state = ContactState::Inactive;
            return blend.reachedTarget ? ReleaseBlendStep::Finished : ReleaseBlendStep::Applied;
        }
    }

    void SoftContactRuntime::reset()
    {
        clearAllHands();
        _debugSnapshot = {};
        _wasEnabled = false;
        _logCounter = 0;
    }

    void SoftContactRuntime::clearHandForStrongerOwner(bool isLeft, const char* reason)
    {
        auto& handState = _hands[handIndex(isLeft)];
        const bool hadVisualAuthority =
            handState.externalTransformActive ||
            handState.state != ContactState::Inactive ||
            soft_contact_math::length(handState.correction) > kCorrectionClearDistance;
        clearHand(isLeft);
        _hands[handIndex(isLeft)].state = ContactState::Suppressed;
        if (hadVisualAuthority) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                500,
                "{} soft contact cleared for stronger hand owner ({})",
                isLeft ? "Left" : "Right",
                reason ? reason : "unknown");
        }
    }

    void SoftContactRuntime::clearHand(bool isLeft)
    {
        auto& handState = _hands[handIndex(isLeft)];
        (void)frik_visual_authority::clearExternalHandWorldTransform(softContactTag(isLeft), handFromBool(isLeft));
        handState = {};
    }

    void SoftContactRuntime::clearAllHands()
    {
        clearHand(false);
        clearHand(true);
    }

    bool SoftContactRuntime::getDebugSnapshot(SoftContactDebugSnapshot& outSnapshot) const
    {
        outSnapshot = _debugSnapshot;
        return outSnapshot.contactCount > 0 ||
               outSnapshot.rightState != ContactState::Inactive ||
               outSnapshot.leftState != ContactState::Inactive;
    }

    void SoftContactRuntime::update(const PhysicsFrameContext& frame,
        const Hand& rightHand,
        const Hand& leftHand,
        bool rightHandWeaponEquipped,
        bool leftSupportGripActive,
        const contact_evidence::NativeContactEvidenceSnapshot& nativeContactEvidence)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::SoftContact);

        _debugSnapshot = {};
        _debugSnapshot.rightState = _hands[0].state;
        _debugSnapshot.leftState = _hands[1].state;

        if (!g_rockConfig.rockSoftContactWorldEnabled || !frame.worldReady || frame.menuBlocked) {
            if (_wasEnabled) {
                clearAllHands();
            }
            _debugSnapshot = {};
            _wasEnabled = false;
            return;
        }
        _wasEnabled = true;

        if (++_logCounter >= 360) {
            _logCounter = 0;
            const float loggedWorldContactPadding = worldContactPadding();
            const float loggedWorldQueryPadding = worldQueryRadiusPadding(loggedWorldContactPadding);
            ROCK_LOG_DEBUG(Hand,
                "SoftContact active: worldOnly=yes worldHaptics={} worldMaxCorrection={:.2f} worldQueryPadding={:.2f} worldContactPadding={:.2f} worldPostReleaseReentryMinApproach={:.3f} worldCachedPlaneMaxTangentDrift={:.2f} worldCachedPlaneMaxClearDistance={:.2f} releaseLerp={} releaseLerpTime={:.2f}-{:.2f}s priority={}",
                g_rockConfig.rockSoftContactWorldHapticsEnabled ? "yes" : "no",
                maxWorldCorrection(),
                loggedWorldQueryPadding,
                loggedWorldContactPadding,
                worldPostReleaseReentryMinApproachDistance(),
                worldCachedPlaneMaxTangentDrift(),
                worldCachedPlaneMaxClearDistance(),
                g_rockConfig.rockSoftContactWorldReleaseLerpEnabled ? "yes" : "no",
                softContactReleaseLerpMinTime(),
                softContactReleaseLerpMaxTime(),
                g_rockConfig.rockSoftContactVisualPriority);
        }

        auto solveForHand = [&](bool isLeft, const HandFrameInput& handInput, const Hand& hand) {
            auto& handState = _hands[handIndex(isLeft)];
            /*
             * ROCK soft contact is production world-only visual authority. It
             * still yields to held objects and weapon owners because those
             * systems own the hand transform when active. Pull/locked-selection are
             * also grab ownership states: they can transition into a capture in
             * the same frame, so stale visual contact must not survive there and
             * poison the raw tracked hand frame used by grab setup.
             */
            const bool ownedByGrabFlow =
                suppressesGeneratedHandContactEvidence(hand.getState());
            const bool ownedByStrongerSystem =
                ownedByGrabFlow ||
                (!isLeft && rightHandWeaponEquipped) ||
                (isLeft && leftSupportGripActive);
            if (ownedByStrongerSystem) {
                clearHand(isLeft);
                handState.state = ContactState::Suppressed;
                return;
            }

            CandidateManifold manifold{};
            const RE::NiPoint3 fallbackNormal = soft_contact_math::normalizeOr(handInput.palmNormalWorld, RE::NiPoint3(0.0f, 0.0f, 1.0f));

            if (!handInput.disabled) {
                manifold = solveWorldStaticContact(
                    frame.bhkWorld,
                    frame.hknpWorld,
                    nativeContactEvidence,
                    _hands[handIndex(isLeft)],
                    isLeft,
                    handInput,
                    fallbackNormal,
                    frame.deltaSeconds);
            }

            if (!frik_visual_authority::isAvailable() || handInput.disabled) {
                clearHand(isLeft);
                updateWorldContactHaptics(handState, isLeft, false, 0.0f, frame.deltaSeconds);
                return;
            }

            if (hasWorldContact(manifold)) {
                for (std::uint32_t i = 0; i < manifold.count && i < manifold.contacts.size(); ++i) {
                    logContactTargetIdentity(isLeft, manifold.contacts[i]);
                }
                updateWorldContactHaptics(handState, isLeft, true, maxApproachSpeed(manifold), frame.deltaSeconds);
                handState.releaseBlend = {};
                handState.correction = correctionForManifold(manifold);
                handState.lastContactKind = ContactKind::WorldStatic;
                handState.state = anyPenetratingContact(manifold, 0.5f) ? ContactState::Penetrating : ContactState::Touching;

                RE::NiTransform target = handInput.rawHandWorld;
                target.translate = soft_contact_math::add(target.translate, handState.correction);
                if (frik_visual_authority::applyExternalHandWorldTransform(softContactTag(isLeft), handFromBool(isLeft), target, g_rockConfig.rockSoftContactVisualPriority)) {
                    handState.lastAppliedWorld = target;
                    handState.externalTransformActive = true;
                } else {
                    ROCK_LOG_SAMPLE_WARN(Hand,
                        1000,
                        "{} soft contact external transform apply failed; clearing visual contact state",
                        isLeft ? "Left" : "Right");
                    clearHand(isLeft);
                    handState.state = ContactState::Inactive;
                    return;
                }
                for (std::uint32_t i = 0; i < manifold.count && i < manifold.contacts.size(); ++i) {
                    addDebugContact(_debugSnapshot, isLeft, handState.state, manifold.contacts[i], handState.correction);
                }
                return;
            }

            updateWorldContactHaptics(handState, isLeft, false, 0.0f, frame.deltaSeconds);
            if (handState.externalTransformActive || handState.releaseBlend.active || soft_contact_math::length(handState.correction) > kCorrectionClearDistance) {
                const auto releaseStep = applyReleaseBlend(handState, isLeft, handInput.rawHandWorld, frame.deltaSeconds);
                if (releaseStep == ReleaseBlendStep::Applied) {
                    return;
                }
                if (releaseStep == ReleaseBlendStep::Failed) {
                    ROCK_LOG_SAMPLE_WARN(Hand,
                        1000,
                        "{} soft contact release blend apply failed; clearing visual contact state",
                        isLeft ? "Left" : "Right");
                }
                clearHand(isLeft);
                return;
            }

            handState.state = ContactState::Inactive;
        };

        solveForHand(false, frame.right, rightHand);
        solveForHand(true, frame.left, leftHand);

        _debugSnapshot.rightState = _hands[0].state;
        _debugSnapshot.leftState = _hands[1].state;
    }
}
