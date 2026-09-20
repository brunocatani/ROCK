#include "physics-interaction/performance/PhysicsStepProfile.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/performance/ContactPairProfile.h"
#include <F4SE/Logger.h>

#include <cassert>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <thread>

namespace F4SE::log
{
    std::optional<std::filesystem::path> log_directory()
    {
        return std::filesystem::path{ ROCK_PROFILER_TEST_DIR };
    }
}

namespace
{
    void verifyOutput()
    {
        // Registered before the writer is constructed: its destructor drains and
        // flushes first. No sleeps, background polling or production log access.
        std::ifstream file(std::filesystem::path{ ROCK_PROFILER_TEST_DIR } / "ROCK_Profiler.log");
        assert(file);
        const std::string text(std::istreambuf_iterator<char>{ file }, {});
        const auto requireScope = [&](const char* name, int samples) {
            const auto start = text.find(std::string("Profiler ") + name + ":");
            assert(start != std::string::npos);
            const auto line = text.substr(start, text.find('\n', start) - start);
            assert(line.find("samples=" + std::to_string(samples) + " ") != std::string::npos);
        };
        requireScope("nativePhysicsUpdate", 30);
        requireScope("nativePhysicsCollideInterval", 60);
        requireScope("nativePhysicsSolveInterval", 60);
        requireScope("runtimePreparation", 30);
        requireScope("grabFingerIndexBuild", 1);
        requireScope("grabContactPatch", 1);
        requireScope("meshPointQuery", 2);
        requireScope("meshDirectionalQuery", 1);
        requireScope("grabMeshQueryIndexBuild", 1);
        requireScope("grabTriangleSelection", 2);
        requireScope("nativeImpactListener", 30);
        requireScope("nativeImpactDispatch", 30);
        requireScope("nativeImpactConsumer", 30);
        requireScope("nativeImpactPlayPair", 30);
        assert(text.find("contactPairs=1 nativeWeaponSelfFilter=1") != std::string::npos);
        assert(text.find("otherMeleeForwarded=0 nativeWeaponSelfRejected=30") != std::string::npos);
        assert(text.find("Profiler counter nativeWeaponSelfPairsRejected: count=30") != std::string::npos);
        assert(text.find("bodies=42/700 layers=5/51 shapeKeys=0xFFFFFFFF/0xFFFFFFFF frames=1-30 layerChanged=false simulationInput=30 simulationNative=0 simulationKept=0 manifolds=30 impulses=0 playerMeleeDropped=30") != std::string::npos);
        assert(text.find("bodies=42/700 layers=5/51 shapeKeys=0xFFFFFFFF/0x12000000 frames=1-30 layerChanged=false simulationInput=0 simulationNative=0 simulationKept=0 manifolds=30") != std::string::npos);
        assert(text.find("bodies=888/999") == std::string::npos);
        assert(text.find("Profiler value meshPointQueryTriangleTests: avg=64.00 max=64 samples=2") != std::string::npos);
        assert(text.find("Profiler value grabTriangleSelectionTests: avg=32.00 max=64 samples=2") != std::string::npos);
        assert(text.find("Profiler value meshStaticVerticesTransformed: avg=3.00 max=3 samples=1") != std::string::npos);
        assert(text.find("Profiler value meshPointQueryTriangles: avg=64.00 max=64 samples=2") != std::string::npos);
        assert(text.find("Profiler value meshDirectionalQueryTriangles: avg=64.00 max=64 samples=1") != std::string::npos);
        assert(text.find("Profiler counter grabAcquisitionPeerHeld: count=1") != std::string::npos);
        assert(text.find("Profiler value physicsCompletedSubsteps: avg=2.00 max=2 samples=30") != std::string::npos);
        assert(text.find("Profiler memory runtimePreparation: readQueries=30") != std::string::npos);
        assert(text.find("Profiler grabSurfaceResolution:") == std::string::npos);
        assert(text.find("Profiler grabAcquisition:") == std::string::npos);
        assert(text.find("schema=3") != std::string::npos);
    }
}

int main()
{
    using namespace rock::performance_profiler;
    std::filesystem::create_directories(ROCK_PROFILER_TEST_DIR);
    std::atexit(verifyOutput);
    assert(beginInterval().startTicks == 0);
    refreshSettings(true, 30, 0, false);
    beginFrame();
    observeContactPair({.world=1, .bodyA=888, .bodyB=999}, ContactStage::Manifold);
    auto stale = beginInterval();
    refreshSettings(false, 30, 0, false);
    refreshSettings(true, 30, 0, false);
    assert(!endInterval(Scope::GrabSurfaceResolution, stale));
    assert(stale.startTicks == 0);
    stale = beginInterval();
    refreshSettings(true, 60, 0, false);
    assert(!endInterval(Scope::GrabSurfaceResolution, stale));
    refreshSettings(true, 30, 0, false);

    PhysicsStepProfile interrupted;
    interrupted.beginUpdate();
    interrupted.beginCollide();
    refreshSettings(false, 30, 0, false);
    refreshSettings(true, 30, 0, false);
    interrupted.endCollide();
    interrupted.beginSolve();
    interrupted.endSolve();
    interrupted.endUpdate();

    // Intervals can cross callbacks/threads without carrying a TLS scope pointer.
    auto crossThread = beginInterval();
    std::thread worker([&] { assert(endInterval(Scope::GrabFingerIndexBuild, crossThread)); });
    worker.join();
    assert(!endInterval(Scope::GrabFingerIndexBuild, crossThread));

    PhysicsStepProfile profile;
    profile.endUpdate();
    profile.beginUpdate();
    profile.beginCollide();
    profile.reset();
    profile.endCollide();
    profile.endUpdate();

    {
        ScopedTimer acquisitionStage(Scope::GrabContactPatch);
        std::vector<rock::GrabSurfaceTriangleData> triangles(64);
        for (auto& surface : triangles) {
            surface.triangle = { {0, 0, 0}, {4, 0, 0}, {0, 4, 0} };
        }
        rock::GrabSurfaceQueryIndex index;
        index.build(triangles);
        assert(index.nearest(triangles, {1, 1, 1}, 4).size() == 4);
        assert(index.nearest(triangles, {1, 1, 1}, 4).size() == 4);
        std::array<RE::NiPoint3, 3> vertices{{ {0, 0, 0}, {4, 0, 0}, {0, 4, 0} }};
        std::array<std::uint16_t, 6> indices{0, 1, 2, 0, 2, 1};
        rock::TriShapeRawGeometry geometry;
        geometry.numVertices = 3;
        geometry.numTriangles = 2;
        geometry.triangles = indices.data();
        std::vector<rock::TriangleData> extracted;
        assert(rock::appendStaticMeshTriangles(geometry, reinterpret_cast<const std::uint8_t*>(vertices.data()),
            sizeof(RE::NiPoint3), 0, true, {}, RE::NiTransform{}, nullptr, extracted, nullptr, nullptr) == 2);
        rock::GrabSurfaceHit hit{};
        assert(rock::findClosestGrabSurfaceHitToPoint(triangles, {1, 1, 1}, {0, 0, 1}, 2.0f, 45.0f, hit));
        assert(hit.valid && hit.triangleIndex == 0 && hit.position.z == 0.0f);
        assert(rock::findClosestGrabSurfaceHitToPointPositionOnly(triangles, {1, 1, 1}, {0, 0, 1}, 2.0f, hit, &index));
        assert(hit.valid && hit.triangleIndex == 0 && hit.position.z == 0.0f);
        assert(rock::findClosestGrabSurfaceHit(triangles, {1, 1, 1}, {0, 0, -1}, 1.0f, 1.0f, hit));
        assert(hit.valid && hit.triangleIndex == 0 && hit.position.z == 0.0f);
        assert(beginMemoryQuery().scope == Scope::GrabContactPatch);
        addCounter(Counter::GrabAcquisitionPeerHeld);
    }

    for (int frame = 0; frame < 30; ++frame) {
        beginFrame();
        observeContactPair({.world=123, .bodyA=42, .bodyB=700}, ContactStage::SimulationInput);
        observeContactPair({.world=123, .bodyA=42, .bodyB=700, .layerA=5, .layerB=51}, ContactStage::NativeWeaponSelfRejected);
        addCounter(Counter::NativeWeaponSelfPairsRejected);
        observeContactPair({.world=123, .bodyA=700, .bodyB=42, .shapeA=0x12000000, .layerA=51, .layerB=5}, ContactStage::Manifold);
        observeContactPair({.world=123, .bodyA=700, .bodyB=42, .layerA=51, .layerB=5}, ContactStage::PlayerMeleeDropped);
        {
            ScopedTimer listener(Scope::NativeImpactListener);
            ScopedTimer dispatch(Scope::NativeImpactDispatch);
            ScopedTimer consumer(Scope::NativeImpactConsumer);
            ScopedTimer pair(Scope::NativeImpactPlayPair);
        }
        {
            ScopedTimer outer(Scope::RuntimePreparation);
            profile.beginUpdate();
            for (int substep = 0; substep < 2; ++substep) {
                profile.beginCollide();
                auto memory = beginMemoryQuery();
                assert(memory.scope == Scope::RuntimePreparation);
                if (substep == 0) endMemoryQuery(memory, MemoryQueryKind::Read, true);
                profile.endCollide();
                profile.beginSolve();
                // Consecutive high-resolution clock calls are sufficient; this
                // tests sample ownership/counts rather than a timing threshold.
                for (int i = 0; i < 10; ++i) (void)beginInterval();
                profile.endSolve();
                profile.endSolve();
            }
            profile.endUpdate();
            profile.endUpdate();
        }
        endFrame();
    }
    refreshSettings(false, 30, 0, false);
    auto disabled = beginInterval();
    assert(!endInterval(Scope::GrabAcquisition, disabled));
    return 0;
}
