#include "physics-interaction/performance/PerformanceProfiler.h"
#include "F4SE/Logger.h"

#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{
    std::filesystem::path logDirectory;
    using namespace rock::performance_profiler;

    void query(MemoryQueryKind kind, Scope expectedScope, bool succeeded = true)
    {
        const auto sample = beginMemoryQuery();
        assert(sample.active && sample.scope == expectedScope);
        endMemoryQuery(sample, kind, succeeded);
    }

    std::vector<std::uint64_t> counts(const std::string& log, const char* scope, const char* field)
    {
        std::vector<std::uint64_t> values;
        std::istringstream input(log);
        std::string line;
        const auto label = std::string("Profiler memory ") + scope + ":";
        const auto key = std::string(field) + "=";
        while (std::getline(input, line)) {
            if (line.find(label) == std::string::npos) continue;
            const auto start = line.find(key);
            assert(start != std::string::npos);
            values.push_back(std::stoull(line.substr(start + key.size())));
        }
        return values;
    }
}

namespace F4SE::log
{
    // The real profiler writer is exercised, but only in this test's build
    // directory. REL's inert module implementation comes from policy support.
    std::optional<std::filesystem::path> log_directory() { return logDirectory; }
}

int main()
{
    using namespace rock::performance_profiler;
    logDirectory = std::filesystem::current_path() /
        ("profiler-test-" + std::to_string(GetCurrentProcessId()) + "-" + std::to_string(GetTickCount64())) /
        "Fallout4VR" / "F4SE";
    std::filesystem::create_directories(logDirectory);

    assert(!beginMemoryQuery().active);
    refreshSettings(true, 30, 0, false);
    query(MemoryQueryKind::Execute, Scope::UnattributedMemoryQueries, false);
    for (int frame = 0; frame < 30; ++frame) {
        FrameScope frameScope;
        if (frame != 0) continue;
        ScopedTimer outer(Scope::HandFrameResolve);
        for (int i = 0; i < 3; ++i) query(MemoryQueryKind::Read, Scope::HandFrameResolve);
        {
            ScopedTimer inner(Scope::HandBoneCapture);
            for (int i = 0; i < 2; ++i) query(MemoryQueryKind::Write, Scope::HandBoneCapture);
            inner.stop();
            query(MemoryQueryKind::Read, Scope::HandFrameResolve);
        }
        query(MemoryQueryKind::Read, Scope::HandFrameResolve);
        const auto concurrentQueries = [] {
            ScopedTimer timer(Scope::NativePlayerPairFilter);
            for (int i = 0; i < 65; ++i) query(MemoryQueryKind::Read, Scope::NativePlayerPairFilter);
        };
        std::thread a(concurrentQueries), b(concurrentQueries);
        a.join();
        b.join();
    }

    refreshSettings(false, 30, 0, false);
    assert(!beginMemoryQuery().active);
    refreshSettings(true, 30, 0, false);
    for (int frame = 0; frame < 30; ++frame) {
        FrameScope frameScope;
        if (frame == 0) {
            ScopedTimer timer(Scope::HandFrameResolve);
            query(MemoryQueryKind::Read, Scope::HandFrameResolve);
        }
    }

    std::string output;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    do {
        // The writer flushes on later windows. Keep delivering empty frames as
        // the game does, rather than requiring an extra production flush API.
        { FrameScope idleFrame; }
        std::ifstream input(logDirectory / "ROCK_0.9_Profiler.log");
        output.assign(std::istreambuf_iterator<char>(input), {});
        if (counts(output, "handFrameResolve", "readQueries").size() == 2) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while (std::chrono::steady_clock::now() < deadline);

    assert((counts(output, "handFrameResolve", "readQueries") == std::vector<std::uint64_t>{5, 1}));
    assert((counts(output, "handBoneCapture", "writeQueries") == std::vector<std::uint64_t>{2}));
    assert(counts(output, "frame", "readQueries").empty()); // Children are not counted again in their parent.
    assert((counts(output, "nativePlayerPairFilter", "readQueries") == std::vector<std::uint64_t>{130}));
    assert((counts(output, "nativePlayerPairFilter", "timedQueries") == std::vector<std::uint64_t>{4}));
    assert((counts(output, "unattributedMemoryQueries", "executeQueries") == std::vector<std::uint64_t>{1}));
    assert((counts(output, "unattributedMemoryQueries", "apiFailures") == std::vector<std::uint64_t>{1}));
    refreshSettings(false, 30, 0, false);
    return 0;
}
