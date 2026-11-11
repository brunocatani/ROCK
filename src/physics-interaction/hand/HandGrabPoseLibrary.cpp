// Library grab poses: the pose an object without an authored offset borrows
// from the authored objects shaped like it (GrabPoseLibrary.h). This file
// reads the object's mesh into the library's frame, lets the finger solve pick
// among the borrowed candidates, and keeps the result per base form.
#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/hand/HandGrabInternal.h"

#include "RockConfig.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/grab/GrabFingerCapsuleSweep.h"
#include "physics-interaction/grab/GrabPoseLibrary.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/object/CountBasedAmmo.h"
#include "physics-interaction/visual/FrikHandWorldAuthority.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>

namespace rock
{
    using namespace hand_grab_internal;

    namespace
    {
        namespace library = grab_pose_library;
        namespace sweep = grab_finger_capsule_sweep;

        // A mesh this large is not a hand-held prop; describing it would stall the grab frame.
        constexpr std::size_t kMaxLibraryMeshTriangles = 120'000;
        // Candidates tried with the finger solve, best signature match first:
        // eight exemplars at four headings each. Two exemplars left objects
        // with a finger through them when a nearby exemplar held them cleanly.
        // A mesh too large for one finger index rebuilds it per pose, so it
        // tries four exemplars.
        constexpr std::size_t kMaxTestedCandidates = 32;
        constexpr std::size_t kMaxTestedCandidatesLargeMesh = 16;

        /*
         * How well the hand holds the object in a candidate pose, lower is
         * better. A finger whose open pose starts inside the object and never
         * clears it means the object sits through the hand; a finger that
         * closes without meeting the surface holds nothing; the thumb matters
         * most; surface behind the palm is the object passing through the hand.
         * The signature order and the exemplar's own heading break ties.
         */
        float candidateScore(const sweep::Result& fingers, float behindPalmFraction, std::uint32_t rank,
            std::uint32_t heading)
        {
            int contacts = 0;
            int blocked = 0;
            for (const auto& finger : fingers.fingers) {
                contacts += finger.contact ? 1 : 0;
                blocked += finger.blocked ? 1 : 0;
            }
            return 2.0f * static_cast<float>(blocked) + static_cast<float>(5 - contacts) +
                (fingers.fingers[0].contact ? 0.0f : 1.5f) + 30.0f * behindPalmFraction +
                0.35f * static_cast<float>(rank) + (heading == 0 ? 0.0f : 0.5f);
        }

        // Share of the surface behind the palm plane within the hand's outline (palm frame, game units).
        float behindPalmFraction(const library::Pose& pose, const std::vector<library::Vec3>& samples)
        {
            if (samples.empty()) {
                return 0.0f;
            }
            std::size_t behind = 0;
            for (const auto& sample : samples) {
                const auto p = library::detail::add(library::detail::applyRotation(pose.rotate, sample), pose.translate);
                if (p[1] > 1.2 && std::abs(p[2]) < 4.5 && p[0] > -8.0 && p[0] < 4.0) {
                    ++behind;
                }
            }
            return static_cast<float>(behind) / static_cast<float>(samples.size());
        }

        // A library pose as the saved-offset transform: rows are the object's axes in the palm frame.
        RE::NiTransform objectInPalm(const library::Pose& pose)
        {
            RE::NiTransform out = makeIdentityTransform();
            out.translate = { static_cast<float>(pose.translate[0]), static_cast<float>(pose.translate[1]),
                static_cast<float>(pose.translate[2]) };
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    out.rotate.entry[row][column] = static_cast<float>(pose.rotate[column * 3 + row]);
                }
            }
            return out;
        }
    }

    bool Hand::findLibraryGrabPose(RE::TESObjectREFR* refr, saved_grab_offset::HandOffset& out) const
    {
        auto* baseForm = refr ? refr->GetObjectReference() : nullptr;
        auto* root = refr ? refr->Get3D() : nullptr;
        if (!baseForm || !root) {
            return false;
        }
        const auto ammoVariant = static_cast<std::uint8_t>(count_based_ammo::resolve(refr).variant);
        for (const auto& entry : _libraryGrabPoses) {
            if (entry.baseFormId == baseForm->GetFormID() && entry.ammoVariant == ammoVariant &&
                std::abs(entry.scale - root->world.scale) < 0.001f) {
                if (entry.found) {
                    out = entry.offset;
                }
                return entry.found;
            }
        }
        return false;
    }

    void Hand::prepareLibraryGrabPose(RE::TESObjectREFR* refr, RE::NiAVObject* rootNode,
        const std::vector<TriangleData>& worldTriangles, bool rigidObject)
    {
        auto* baseForm = refr ? refr->GetObjectReference() : nullptr;
        if (!baseForm || !rootNode || refr->Get3D() != rootNode || worldTriangles.empty() ||
            !grab_three_phase::isFinite(rootNode->world) || !(rootNode->world.scale > 0.0001f)) {
            return;
        }
        // A root-relative pose says nothing about an object whose bodies move
        // against each other, and guns and melee seat through their grips.
        const auto* weapon = looseWeaponFormFromRef(refr);
        if (!rigidObject || !saved_grab_offset::participatesInSavedGrabOffsets(weapon != nullptr,
                                isThrowableLooseWeapon(weapon))) {
            return;
        }
        const std::uint32_t baseFormId = baseForm->GetFormID();
        const float rootScale = rootNode->world.scale;
        const auto ammoVariant = static_cast<std::uint8_t>(count_based_ammo::resolve(refr).variant);
        for (const auto& entry : _libraryGrabPoses) {
            if (entry.baseFormId == baseFormId && entry.ammoVariant == ammoVariant &&
                std::abs(entry.scale - rootScale) < 0.001f) {
                return;
            }
        }
        if (hasOwnAuthoredGrabOffset(refr)) {
            return;
        }

        // The live hand: the finger rig, the palm frame the pose is expressed
        // in and the knuckles the fingers hang from. Without them nothing is
        // solved or remembered, and the next grab tries again.
        RE::NiTransform rawHand{};
        RE::NiTransform proxyWorld{};
        sweep::FrikFingerRig rig{};
        root_flattened_finger_skeleton_runtime::Snapshot skeleton{};
        if (!frik_hand_world_authority::tryGetRawHandWorld(_isLeft, rawHand) ||
            !tryComputeGrabProxyFrameForRawHand(rawHand, proxyWorld) || !sweep::resolveFrikFingerRig(_isLeft, rig) ||
            !root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(_isLeft, skeleton)) {
            return;
        }

        const auto started = std::chrono::steady_clock::now();
        LibraryGrabPose solved{};
        solved.baseFormId = baseFormId;
        solved.scale = rootScale;
        solved.ammoVariant = ammoVariant;
        const auto remember = [&]() {
            if (_libraryGrabPoses.size() < kMaxLibraryGrabPoses) {
                _libraryGrabPoses.push_back(solved);
            } else {
                _libraryGrabPoses[_libraryGrabPoseNext] = solved;
                _libraryGrabPoseNext = (_libraryGrabPoseNext + 1) % kMaxLibraryGrabPoses;
            }
        };
        const char* objectName = refr->GetDisplayFullName() ? refr->GetDisplayFullName() : "";
        if (worldTriangles.size() > kMaxLibraryMeshTriangles) {
            ROCK_LOG_INFO(Hand, "{} hand library grab pose: none for '{}' {:08X}, mesh too large ({} triangles)",
                handName(), objectName, baseFormId, worldTriangles.size());
            remember();
            return;
        }

        // The mesh in the root's frame: in game units for the library, unscaled for the finger index.
        RE::NiTransform rootUnscaled = rootNode->world;
        rootUnscaled.scale = 1.0f;
        std::vector<GrabLocalTriangle> gameTriangles;
        std::vector<GrabLocalTriangle> localTriangles;
        gameTriangles.reserve(worldTriangles.size());
        localTriangles.reserve(worldTriangles.size());
        const float inverseScale = 1.0f / rootScale;
        for (const auto& triangle : worldTriangles) {
            const GrabLocalTriangle game{
                transform_math::worldPointToLocal(rootUnscaled, triangle.v0),
                transform_math::worldPointToLocal(rootUnscaled, triangle.v1),
                transform_math::worldPointToLocal(rootUnscaled, triangle.v2),
            };
            gameTriangles.push_back(game);
            localTriangles.push_back(GrabLocalTriangle{ game.v0 * inverseScale, game.v1 * inverseScale,
                game.v2 * inverseScale });
        }

        const auto shape = library::describeShape(gameTriangles, 1.0);
        const auto samples = library::sampleSurface(gameTriangles, 1.0);
        const auto candidates = library::candidates(shape, samples, _isLeft);
        if (candidates.empty()) {
            ROCK_LOG_INFO(Hand, "{} hand library grab pose: none for '{}' {:08X}, its mesh has no measurable shape "
                                "(triangles={})",
                handName(), objectName, baseFormId, worldTriangles.size());
            remember();
            return;
        }

        // The finger solve's index holds a bounded number of triangles: the
        // whole mesh when it fits, otherwise those nearest the palm per pose.
        grab_finger_pose_runtime::FingerPoseTriangleSpatialIndex index{};
        const bool wholeMesh = localTriangles.size() <= grab_finger_pose_runtime::kMaxFingerPoseCandidateTriangles;
        if (wholeMesh && !index.buildFromLocalTriangles(localTriangles)) {
            return;
        }
        // Nearness is each triangle's bounding sphere, computed once: an exact
        // closest point per triangle per pose was most of a 9 ms solve on a
        // 21,000-triangle mesh.
        struct TriangleSphere
        {
            RE::NiPoint3 center{};
            float radius = 0.0f;
        };
        std::vector<TriangleSphere> triangleSpheres;
        if (!wholeMesh) {
            triangleSpheres.reserve(localTriangles.size());
            for (const auto& t : localTriangles) {
                TriangleSphere sphere{};
                sphere.center = (t.v0 + t.v1 + t.v2) * (1.0f / 3.0f);
                for (const auto& vertex : { t.v0, t.v1, t.v2 }) {
                    const RE::NiPoint3 d = vertex - sphere.center;
                    sphere.radius = (std::max)(sphere.radius, std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z));
                }
                triangleSpheres.push_back(sphere);
            }
        }
        std::vector<std::pair<float, std::uint32_t>> nearestScratch;
        std::vector<GrabLocalTriangle> nearestTriangles;
        const auto indexNear = [&](const RE::NiPoint3& centerLocal) {
            nearestScratch.clear();
            nearestScratch.reserve(localTriangles.size());
            for (std::uint32_t i = 0; i < triangleSpheres.size(); ++i) {
                const RE::NiPoint3 d = triangleSpheres[i].center - centerLocal;
                nearestScratch.emplace_back(
                    std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z) - triangleSpheres[i].radius, i);
            }
            const std::size_t keep = grab_finger_pose_runtime::kMaxFingerPoseCandidateTriangles;
            std::nth_element(nearestScratch.begin(), nearestScratch.begin() + static_cast<std::ptrdiff_t>(keep),
                nearestScratch.end());
            nearestTriangles.clear();
            nearestTriangles.reserve(keep);
            for (std::size_t i = 0; i < keep; ++i) {
                nearestTriangles.push_back(localTriangles[nearestScratch[i].second]);
            }
            return index.buildFromLocalTriangles(nearestTriangles);
        };

        const RE::NiTransform handBone = sweep::resolveHandBoneWorld(rawHand, skeleton, rig);
        const auto tips = sweep::tipLengths(skeleton);
        const sweep::Options options{ .minValue = g_rockConfig.rockGrabFingerMinValue };

        const library::Candidate* best = nullptr;
        sweep::Result bestFingers{};
        float bestScore = 0.0f;
        const std::size_t tested =
            (std::min)(candidates.size(), wholeMesh ? kMaxTestedCandidates : kMaxTestedCandidatesLargeMesh);
        for (std::size_t i = 0; i < tested; ++i) {
            const auto& candidate = candidates[i];
            RE::NiTransform objectWorld = grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorld,
                objectInPalm(candidate.pose));
            objectWorld.scale = rootScale;
            if (!grab_three_phase::isFinite(objectWorld)) {
                continue;
            }
            if (!wholeMesh &&
                !indexNear(transform_math::worldPointToLocal(objectWorld, proxyWorld.translate))) {
                continue;
            }
            const auto fingers = sweep::solve(rig, handBone, tips, index, objectWorld, options);
            const float score =
                candidateScore(fingers, behindPalmFraction(candidate.pose, samples), candidate.rank, candidate.heading);
            if (!best || score < bestScore) {
                best = &candidate;
                bestFingers = fingers;
                bestScore = score;
            }
        }
        if (!best) {
            return;
        }

        const auto objectProxyLocal = objectInPalm(best->pose);
        solved.found = true;
        solved.offset.present = true;
        solved.offset.translateGame[0] = objectProxyLocal.translate.x;
        solved.offset.translateGame[1] = objectProxyLocal.translate.y;
        solved.offset.translateGame[2] = objectProxyLocal.translate.z;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                solved.offset.rotate[row * 3 + column] = objectProxyLocal.rotate.entry[row][column];
            }
        }
        // The fingers as the solve closed them on this mesh in this pose.
        solved.offset.hasFingerPose = true;
        int contacts = 0;
        int blocked = 0;
        for (std::size_t finger = 0; finger < 5; ++finger) {
            solved.offset.fingerValues[finger] = std::clamp(bestFingers.values[finger], 0.0f, 1.0f);
            contacts += bestFingers.fingers[finger].contact ? 1 : 0;
            blocked += bestFingers.fingers[finger].blocked ? 1 : 0;
        }
        remember();

        const float milliseconds = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - started)
                                       .count();
        ROCK_LOG_INFO(Hand,
            "{} hand library grab pose: '{}' {:08X} held like '{}' (match {} of {}, distance {:.2f}, heading {}) "
            "fingers on mesh {} blocked {} score {:.2f} triangles={} tested={} {:.1f}ms",
            handName(), objectName, baseFormId, best->exemplar->name, best->rank + 1,
            (candidates.back().rank + 1), best->signatureDistance, best->heading, contacts, blocked, bestScore,
            worldTriangles.size(), tested, milliseconds);
    }
}
