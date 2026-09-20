#include "physics-interaction/performance/PhysicsStepProfile.h"
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
    for (int frame = 0; frame < 30; ++frame) {
        beginFrame();
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
