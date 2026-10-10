#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <filesystem>
#include <semaphore>
#include <thread>
#include "RE/NetImmerse/NiTransform.h"

namespace rock::gunstock_debug
{
    enum Pose : std::size_t { LeftController, RightController, LeftDriver, RightDriver,
        LeftPhysical, RightPhysical, WeaponBefore, WeaponAfter, PrimarySeat, SupportSeat,
        Camera, Scope, WeaponRendered, LeftRendered, RightRendered, ScopeRendered, PoseCount };
    // All poses are world-space Ni transforms (stored rows, local vectors use
    // the transpose). Seat origins are grip points, not wrist origins. Bit N
    // of valid corresponds to Pose N; absent poses must never be read as identity.
    // Phase 1 is the grip-update output, phase 2 AfterWorldFinal. Controller
    // inputs are sampled once before the solve and shared by both phase rows.
    struct Sample {
        std::uint64_t frame{}, tick{}, generation{}, ownership{}, grip{}, mark{}, valid{};
        std::uint32_t form{}, state{}, firingLeft{}, supportActive{}, authored{}, part{}, omod{}, attachPoint{}, phase{};
        float dt{};
        char source[128]{};
        std::array<RE::NiTransform, PoseCount> poses{};
        bool has(Pose pose) const { return (valid & (std::uint64_t{1} << pose)) != 0; }
    };
    bool setPose(Sample& sample, Pose pose, const RE::NiTransform& value);

    // One game-update producer, one file-writer consumer. Producer only copies
    // values into a bounded SPSC ring and signals; it never waits or touches files.
    // Owner must stop submitting before destruction, which drains and joins.
    class Recorder {
    public:
        explicit Recorder(std::filesystem::path directory);
        ~Recorder();
        bool submit(const Sample& sample);
        std::uint64_t dropped() const { return _dropped.load(); }
        std::uint64_t written() const { return _written.load(); }
        bool failed() const { return _failed.load(); }
    private:
        void run() noexcept;
        static constexpr std::size_t capacity = 256;
        std::filesystem::path _directory;
        std::array<Sample, capacity> _queue{};
        std::atomic<std::uint64_t> _write{}, _read{}, _dropped{}, _written{};
        std::atomic<bool> _stop{}, _failed{};
        std::counting_semaphore<capacity + 1> _ready{0};
        std::thread _worker;
    };
}
