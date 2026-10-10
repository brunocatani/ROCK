#include "physics-interaction/weapon/GunstockDebug.h"
#include <chrono>
#include "physics-interaction/weapon/HoldingGripAdapter.h"
#include <nlohmann/json.hpp>
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

    Recorder::Recorder(std::filesystem::path directory,std::filesystem::path holdingDirectory) : _directory(std::move(directory)), _holdingDirectory(std::move(holdingDirectory)), _queue(std::make_unique<std::array<Sample,capacity>>()), _worker([this] { run(); }) {}
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
        (*_queue)[write % capacity] = sample;
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
                const Sample sample = (*_queue)[read % capacity];
                _read.store(read + 1, std::memory_order_release);
                if(sample.phase==2&&sample.holding.requested&&sample.holding.ready){try{
                    using namespace holding_grip_math;using J=nlohmann::json;
                    const auto pose=[](const RE::NiTransform& t){const auto p=fromNative(t);return J{{"rotate",p.r},{"translate",p.t},{"scale",p.s}};};
                    const auto& c=sample.holding;J j{{"format","rock-holding-grip-capture"},{"version",1},{"space","weapon-local"},{"fingerSpace","parent-local"},{"matrixConvention","active-row-major"},{"units","game"},{"weapon",c.weapon},{"gripNode",c.gripNode},{"frame",sample.frame},{"generation",sample.generation},{"holdingHand",sample.firingLeft?"left":"right"},{"rightBoneInWand",pose(c.rightBoneInWand)},{"leftBoneInWand",pose(c.leftBoneInWand)},{"nativeRight",pose(c.nativeRight)},{"palmRight",c.palmRight},{"palmLeft",c.palmLeft},{"leftGripOffset",c.leftOffset},{"leftGripBaselineY",0.0f},{"leftGripMirror","geometry-plane-hand-z"},{"mirrorPlaneX",c.mirrorPlaneX},{"rightSupportOffset",c.rightSupportOffset},{"rightSupportMirror","geometry-plane-hand-z"},{"rightSupportBaselineX",0.0f}};
                    J fingers=J::array();for(const auto& f:c.nativeFingers)fingers.push_back(pose(f));if(c.nativeFingersValid)j["nativeFingers"]=fingers;j["powerArmor"]=c.powerArmor;j["gameLeftHanded"]=c.gameLeftHanded;j["phase"]="AfterWorldFinal";j["supportActive"]=sample.supportActive!=0;j["gunstockAim"]=sample.gunstockAim!=0;if(c.hasSupport){j["nativeSupport"]=pose(c.nativeSupport);J sf=J::array();for(const auto& f:c.supportFingers)sf.push_back(pose(f));j["supportFingers"]=sf;}
                    if(sample.has(WeaponRendered))for(int h=0;h<2;++h){const auto id=h==0?LeftRendered:RightRendered;if(!sample.has(id))continue;const auto relative=compose(inverse(fromNative(sample.poses[WeaponRendered])),fromNative(sample.poses[id]));J hand{{"wrist",pose(toNative(relative))}};if(c.fingerValid[h]){hand["fingers"]=J::array();for(const auto& f:c.renderedFingers[h])hand["fingers"].push_back(pose(f));}j[h==0?"renderedLeft":"renderedRight"]=hand;}
                    const auto& directory=_holdingDirectory;std::filesystem::create_directories(directory);std::ofstream capture(directory/(std::to_string(sample.form)+"-"+std::to_string(sample.tick)+"-"+std::to_string(sample.frame)+".json"));capture<<j.dump(2);capture.flush();if(!capture)_holdingFailed.store(true);
                }catch(const std::exception&){_holdingFailed.store(true);}}
                if(sample.holding.captureOnly)continue;
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
