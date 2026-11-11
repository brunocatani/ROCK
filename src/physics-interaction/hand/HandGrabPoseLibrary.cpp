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
#include <memory>
#include <string>
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

    /*
     * One library pose solve, resumable. Everything in it is relative to the
     * hand as captured at its start (palm frame, knuckles, finger rig), so it
     * can run a little each frame while the hand points at the object and the
     * hand moves: the solved pose is the object's place in the palm. It holds
     * no engine pointers. Frame thread only.
     */
    struct Hand::LibraryGrabPoseJob
    {
        std::uint32_t baseFormId = 0;
        float scale = 1.0f;
        std::uint8_t ammoVariant = 0;
        std::string objectName;
        std::size_t triangleCount = 0;

        RE::NiTransform proxyWorld{};
        RE::NiTransform handBone{};
        sweep::FrikFingerRig rig{};
        std::array<float, 5> tips{};
        sweep::Options options{};

        // The mesh in the object's frame, unscaled, for the finger index.
        std::vector<GrabLocalTriangle> localTriangles;
        // Each triangle's bounding sphere: an exact closest point per triangle
        // per pose was most of a 9 ms solve on a 21,000-triangle mesh.
        struct TriangleSphere
        {
            RE::NiPoint3 center{};
            float radius = 0.0f;
        };
        std::vector<TriangleSphere> triangleSpheres;
        std::vector<std::pair<float, std::uint32_t>> nearestScratch;
        std::vector<GrabLocalTriangle> nearestTriangles;
        // The finger solve's index holds a bounded number of triangles: the
        // whole mesh when it fits, otherwise those nearest the palm per pose.
        grab_finger_pose_runtime::FingerPoseTriangleSpatialIndex index{};
        bool wholeMesh = true;

        std::vector<library::Vec3> samples;
        std::vector<library::Candidate> candidates;
        std::size_t next = 0;
        std::size_t limit = 0;
        int best = -1;
        sweep::Result bestFingers{};
        float bestScore = 0.0f;
        const char* noneReason = nullptr;

        float milliseconds = 0.0f;
        std::uint32_t frames = 0;

        [[nodiscard]] bool matches(std::uint32_t formId, float objectScale, std::uint8_t variant) const
        {
            return baseFormId == formId && ammoVariant == variant && std::abs(scale - objectScale) < 0.001f;
        }
        [[nodiscard]] bool finished() const { return next >= limit; }

        // Reads the mesh: its shape, its surface samples and the poses to try.
        void begin(const RE::NiTransform& objectFrameWorld, const std::vector<TriangleData>& worldTriangles)
        {
            const auto started = std::chrono::steady_clock::now();
            triangleCount = worldTriangles.size();
            if (triangleCount > kMaxLibraryMeshTriangles) {
                noneReason = "mesh too large";
                return;
            }
            RE::NiTransform frameUnscaled = objectFrameWorld;
            frameUnscaled.scale = 1.0f;
            const float inverseScale = 1.0f / scale;
            std::vector<GrabLocalTriangle> gameTriangles;
            gameTriangles.reserve(triangleCount);
            localTriangles.reserve(triangleCount);
            for (const auto& triangle : worldTriangles) {
                const GrabLocalTriangle game{
                    transform_math::worldPointToLocal(frameUnscaled, triangle.v0),
                    transform_math::worldPointToLocal(frameUnscaled, triangle.v1),
                    transform_math::worldPointToLocal(frameUnscaled, triangle.v2),
                };
                gameTriangles.push_back(game);
                localTriangles.push_back(GrabLocalTriangle{ game.v0 * inverseScale, game.v1 * inverseScale,
                    game.v2 * inverseScale });
            }

            const auto shape = library::describeShape(gameTriangles, 1.0);
            samples = library::sampleSurface(gameTriangles, 1.0);
            candidates = library::candidates(shape, samples, isLeft);
            wholeMesh = localTriangles.size() <= grab_finger_pose_runtime::kMaxFingerPoseCandidateTriangles;
            if (candidates.empty() || (wholeMesh && !index.buildFromLocalTriangles(localTriangles))) {
                candidates.clear();
                noneReason = "its mesh has no measurable shape";
                return;
            }
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
            limit = (std::min)(candidates.size(), wholeMesh ? kMaxTestedCandidates : kMaxTestedCandidatesLargeMesh);
            milliseconds +=
                std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - started).count();
        }

        // Tries the next pose with the finger solve.
        void step()
        {
            if (finished()) {
                return;
            }
            const auto started = std::chrono::steady_clock::now();
            const std::size_t candidateIndex = next++;
            const auto& candidate = candidates[candidateIndex];
            RE::NiTransform objectWorld = grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorld,
                objectInPalm(candidate.pose));
            objectWorld.scale = scale;
            if (grab_three_phase::isFinite(objectWorld) &&
                (wholeMesh || indexNear(transform_math::worldPointToLocal(objectWorld, proxyWorld.translate)))) {
                const auto fingers = sweep::solve(rig, handBone, tips, index, objectWorld, options);
                const float score = candidateScore(fingers, behindPalmFraction(candidate.pose, samples),
                    candidate.rank, candidate.heading);
                if (best < 0 || score < bestScore) {
                    best = static_cast<int>(candidateIndex);
                    bestFingers = fingers;
                    bestScore = score;
                }
            }
            milliseconds +=
                std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - started).count();
        }

        bool isLeft = false;

    private:
        bool indexNear(const RE::NiPoint3& centerLocal)
        {
            nearestScratch.clear();
            nearestScratch.reserve(triangleSpheres.size());
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
        }
    };

    bool Hand::libraryGrabPoseKnown(std::uint32_t baseFormId, float scale, std::uint8_t ammoVariant) const
    {
        for (const auto& entry : _libraryGrabPoses) {
            if (entry.baseFormId == baseFormId && entry.ammoVariant == ammoVariant &&
                std::abs(entry.scale - scale) < 0.001f) {
                return true;
            }
        }
        return false;
    }

    bool Hand::startLibraryGrabPoseJob(RE::TESObjectREFR* refr, const RE::NiTransform& objectFrameWorld,
        std::uint8_t ammoVariant, const std::vector<TriangleData>& worldTriangles)
    {
        // The live hand: the finger rig, the palm frame the pose is expressed
        // in and the knuckles the fingers hang from. Without them nothing is
        // solved or remembered, and a later frame or grab tries again.
        RE::NiTransform rawHand{};
        auto job = std::make_shared<LibraryGrabPoseJob>();
        root_flattened_finger_skeleton_runtime::Snapshot skeleton{};
        if (!frik_hand_world_authority::tryGetRawHandWorld(_isLeft, rawHand) ||
            !tryComputeGrabProxyFrameForRawHand(rawHand, job->proxyWorld) ||
            !sweep::resolveFrikFingerRig(_isLeft, job->rig) ||
            !root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(_isLeft, skeleton)) {
            return false;
        }
        job->handBone = sweep::resolveHandBoneWorld(rawHand, skeleton, job->rig);
        job->tips = sweep::tipLengths(skeleton);
        job->options = sweep::Options{ .minValue = g_rockConfig.rockGrabFingerMinValue };
        job->isLeft = _isLeft;
        job->baseFormId = refr->GetObjectReference()->GetFormID();
        job->scale = objectFrameWorld.scale;
        job->ammoVariant = ammoVariant;
        job->objectName = refr->GetDisplayFullName() ? refr->GetDisplayFullName() : "";
        job->begin(objectFrameWorld, worldTriangles);
        job->frames = 1;
        _libraryGrabPoseJob = std::move(job);
        return true;
    }

    void Hand::finishLibraryGrabPoseJob(float grabFrameMilliseconds)
    {
        const auto job = std::move(_libraryGrabPoseJob);
        _libraryGrabPoseJob.reset();
        if (!job) {
            return;
        }
        LibraryGrabPose solved{};
        solved.baseFormId = job->baseFormId;
        solved.scale = job->scale;
        solved.ammoVariant = job->ammoVariant;
        int contacts = 0;
        int blocked = 0;
        if (job->best >= 0) {
            const auto& chosen = job->candidates[static_cast<std::size_t>(job->best)];
            const auto objectProxyLocal = objectInPalm(chosen.pose);
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
            for (std::size_t finger = 0; finger < 5; ++finger) {
                solved.offset.fingerValues[finger] = std::clamp(job->bestFingers.values[finger], 0.0f, 1.0f);
                contacts += job->bestFingers.fingers[finger].contact ? 1 : 0;
                blocked += job->bestFingers.fingers[finger].blocked ? 1 : 0;
            }
        }
        if (_libraryGrabPoses.size() < kMaxLibraryGrabPoses) {
            _libraryGrabPoses.push_back(solved);
        } else {
            _libraryGrabPoses[_libraryGrabPoseNext] = solved;
            _libraryGrabPoseNext = (_libraryGrabPoseNext + 1) % kMaxLibraryGrabPoses;
        }

        if (!solved.found) {
            ROCK_LOG_INFO(Hand, "{} hand library grab pose: none for '{}' {:08X}, {} (triangles={})", handName(),
                job->objectName, job->baseFormId, job->noneReason ? job->noneReason : "no pose could be tried",
                job->triangleCount);
            return;
        }
        const auto& chosen = job->candidates[static_cast<std::size_t>(job->best)];
        ROCK_LOG_INFO(Hand,
            "{} hand library grab pose: '{}' {:08X} held like '{}' (match {} of {}, distance {:.2f}, heading {}) "
            "fingers on mesh {} blocked {} score {:.2f} triangles={} tested={} solve={:.1f}ms over {} frames, "
            "{:.2f}ms on the grab frame",
            handName(), job->objectName, job->baseFormId, chosen.exemplar->name, chosen.rank + 1,
            job->candidates.back().rank + 1, chosen.signatureDistance, chosen.heading, contacts, blocked,
            job->bestScore, job->triangleCount, job->limit, job->milliseconds, job->frames, grabFrameMilliseconds);
    }

    /*
     * Runs the selected object's library pose solve ahead of the grab: the mesh
     * is read on the frame the object is first selected, and the poses are
     * tried within a small time budget on the frames after it, so a pull or
     * grab finds the pose ready instead of paying for the whole solve on its
     * own frame. Count-based ammo is read in its visible branch's frame, which
     * is the root's frame once the grab restores the assembly.
     */
    void Hand::prewarmLibraryGrabPose()
    {
        constexpr float kFrameBudgetMilliseconds = 0.3f;

        auto* refr = (hasSelection() && !isHolding()) ? _currentSelection.refr : nullptr;
        if (!refr || _currentSelection.targetKind != grab_target::Kind::LooseObject) {
            _libraryPrewarmRef = nullptr;
            return;
        }
        if (refr == _libraryPrewarmRef) {
            if (_libraryGrabPoseJob && !_libraryGrabPoseJob->finished()) {
                const auto started = std::chrono::steady_clock::now();
                ++_libraryGrabPoseJob->frames;
                do {
                    _libraryGrabPoseJob->step();
                } while (!_libraryGrabPoseJob->finished() &&
                         std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - started).count() <
                             kFrameBudgetMilliseconds);
            }
            return;
        }

        // A newly selected object: decide once whether it needs a solve.
        _libraryPrewarmRef = refr;
        auto* baseForm = refr->GetObjectReference();
        auto* root = refr->Get3D();
        if (!baseForm || !root) {
            return;
        }
        const auto* weapon = looseWeaponFormFromRef(refr);
        if (!saved_grab_offset::participatesInSavedGrabOffsets(weapon != nullptr, isThrowableLooseWeapon(weapon))) {
            return;
        }
        const auto assembly = count_based_ammo::resolve(refr);
        auto* frameNode = assembly.valid() ? assembly.variantNode : root;
        if (!frameNode || !grab_three_phase::isFinite(frameNode->world) || !(frameNode->world.scale > 0.0001f)) {
            return;
        }
        const auto ammoVariant = static_cast<std::uint8_t>(assembly.variant);
        const std::uint32_t baseFormId = baseForm->GetFormID();
        if (libraryGrabPoseKnown(baseFormId, frameNode->world.scale, ammoVariant) ||
            (_libraryGrabPoseJob && _libraryGrabPoseJob->matches(baseFormId, frameNode->world.scale, ammoVariant)) ||
            hasOwnAuthoredGrabOffset(refr)) {
            return;
        }

        auto* meshNode = _currentSelection.visualNode ? _currentSelection.visualNode : root;
        std::vector<TriangleData> worldTriangles;
        std::vector<GrabSurfaceTriangleData> surfaceTriangles;
        MeshExtractionStats stats;
        extractAllSurfaceTriangles(meshNode, worldTriangles, surfaceTriangles,
            (std::max)(1, g_rockConfig.rockObjectPhysicsTreeMaxDepth), &stats, false);
        if (worldTriangles.empty() ||
            !startLibraryGrabPoseJob(refr, frameNode->world, ammoVariant, worldTriangles)) {
            // Nothing to read yet, or no live hand: look again next frame.
            _libraryPrewarmRef = nullptr;
        }
    }

    void Hand::prepareLibraryGrabPose(RE::TESObjectREFR* refr, RE::NiAVObject* rootNode,
        const std::vector<TriangleData>& worldTriangles, bool rigidObject)
    {
        auto* baseForm = refr ? refr->GetObjectReference() : nullptr;
        if (!baseForm || !rootNode || refr->Get3D() != rootNode || worldTriangles.empty() ||
            !grab_three_phase::isFinite(rootNode->world) || !(rootNode->world.scale > 0.0001f)) {
            return;
        }
        const std::uint32_t baseFormId = baseForm->GetFormID();
        const float rootScale = rootNode->world.scale;
        const auto ammoVariant = static_cast<std::uint8_t>(count_based_ammo::resolve(refr).variant);
        const bool jobMatches = _libraryGrabPoseJob && _libraryGrabPoseJob->matches(baseFormId, rootScale, ammoVariant);
        // A root-relative pose says nothing about an object whose bodies move
        // against each other, and guns and melee seat through their grips.
        const auto* weapon = looseWeaponFormFromRef(refr);
        if (!rigidObject || !saved_grab_offset::participatesInSavedGrabOffsets(weapon != nullptr,
                                isThrowableLooseWeapon(weapon)) ||
            libraryGrabPoseKnown(baseFormId, rootScale, ammoVariant) || hasOwnAuthoredGrabOffset(refr)) {
            if (jobMatches) {
                _libraryGrabPoseJob.reset();
            }
            return;
        }

        const auto started = std::chrono::steady_clock::now();
        if (!jobMatches && !startLibraryGrabPoseJob(refr, rootNode->world, ammoVariant, worldTriangles)) {
            return;
        }
        while (!_libraryGrabPoseJob->finished()) {
            _libraryGrabPoseJob->step();
        }
        finishLibraryGrabPoseJob(
            std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - started).count());
    }
}

