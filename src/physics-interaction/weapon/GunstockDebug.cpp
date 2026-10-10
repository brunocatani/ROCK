#include "physics-interaction/weapon/GunstockDebug.h"
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <locale>

namespace rock::gunstock_debug
{
    bool setPose(Sample& sample, Pose pose, const RE::NiTransform& value)
    {
        if (!std::isfinite(value.scale) || value.scale <= 0 ||
            !std::isfinite(value.translate.x) || !std::isfinite(value.translate.y) || !std::isfinite(value.translate.z)) return false;
        for (int row = 0; row < 3; ++row) for (int column = 0; column < 3; ++column)
            if (!std::isfinite(value.rotate.entry[row][column])) return false;
        sample.poses[pose] = value;
        sample.valid |= std::uint64_t{1} << pose;
        return true;
    }

    Recorder::Recorder(std::filesystem::path directory) : _directory(std::move(directory)), _worker([this] { run(); }) {}
    Recorder::~Recorder()
    {
        _stop.store(true, std::memory_order_release);
        _ready.release();
        _worker.join();
    }
    bool Recorder::submit(const Sample& sample)
    {
        if (_failed.load(std::memory_order_relaxed)) { ++_dropped; return false; }
        const auto write = _write.load(std::memory_order_relaxed);
        if (write - _read.load(std::memory_order_acquire) >= capacity) { ++_dropped; return false; }
        _queue[write % capacity] = sample;
        _write.store(write + 1, std::memory_order_release);
        _ready.release();
        return true;
    }
    void Recorder::run() noexcept
    {
        try {
            std::ofstream file;
            for (;;) {
                _ready.acquire();
                const auto read = _read.load(std::memory_order_relaxed);
                if (read == _write.load(std::memory_order_acquire)) {
                    if (_stop.load(std::memory_order_acquire)) break;
                    continue;
                }
                const Sample sample = _queue[read % capacity];
                _read.store(read + 1, std::memory_order_release);
                if (!file.is_open()) {
                    std::filesystem::create_directories(_directory);
                    const auto stamp = std::chrono::system_clock::now().time_since_epoch().count();
                    file.open(_directory / ("gunstock-" + std::to_string(stamp) + ".csv"));
                    file.imbue(std::locale::classic());
                    file << std::setprecision(9);
                    file << "frame,tick_ms,generation,ownership,grip,mark,valid,form,state,firing_left,support_active,authored,part,omod,attach_point,phase,gunstock_aim,dt,source";
                    constexpr const char* names[]{"left_controller","right_controller","left_driver","right_driver","left_physical","right_physical","weapon_before","weapon_after","primary_seat_world","support_seat_world","camera_root","scope_world","weapon_rendered","left_rendered","right_rendered","scope_rendered"};
                    for (auto name : names) {
                        for (int i = 0; i < 9; ++i) file << ',' << name << "_r" << i;
                        file << ',' << name << "_x," << name << "_y," << name << "_z," << name << "_scale";
                    }
                    file << '\n';
                }
                file << sample.frame << ',' << sample.tick << ',' << sample.generation << ',' << sample.ownership << ','
                     << sample.grip << ',' << sample.mark << ',' << sample.valid << ',' << sample.form << ',' << sample.state << ','
                     << sample.firingLeft << ',' << sample.supportActive << ',' << sample.authored << ',' << sample.part << ','
                     << sample.omod << ',' << sample.attachPoint << ',' << sample.phase << ',' << sample.gunstockAim << ',' << sample.dt << ",\"";
                for (char c : sample.source) { if (!c) break; if (c == '"') file << '"'; file << (c == '\n' || c == '\r' ? ' ' : c); }
                file << '"';
                for (const auto& pose : sample.poses) {
                    for (int row = 0; row < 3; ++row) for (int column = 0; column < 3; ++column)
                        file << ',' << pose.rotate.entry[row][column];
                    file << ',' << pose.translate.x << ',' << pose.translate.y << ',' << pose.translate.z << ',' << pose.scale;
                }
                file << '\n';
                file.flush();
                if (!file) { _failed.store(true); return; }
                ++_written;
            }
        } catch (...) { _failed.store(true); }
    }
}
