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
    AimError aimDirectionError(const RE::NiTransform& reference,const RE::NiPoint3& actualDirection) {
        using namespace holding_grip_math;
        auto frame=fromNative(reference);frame.t={};frame.s=1;
        auto local=rotate(inverse(frame),{actualDirection.x,actualDirection.y,actualDirection.z});
        const float length=std::sqrt(local[0]*local[0]+local[1]*local[1]+local[2]*local[2]);
        if(!std::isfinite(length)||length<.000001f)return {};
        for(auto& v:local)v/=length;
        constexpr float degrees=57.295779513f;
        return {true,std::atan2(local[0],local[1])*degrees,
            std::atan2(local[2],std::hypot(local[0],local[1]))*degrees,
            std::acos(std::clamp(local[1],-1.f,1.f))*degrees};
    }
    AimError aimError(const RE::NiTransform& reference,const RE::NiTransform& actual) {
        return aimDirectionError(reference,{actual.rotate.entry[1][0],actual.rotate.entry[1][1],actual.rotate.entry[1][2]});
    }
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

    Recorder::Recorder(std::filesystem::path directory,std::filesystem::path holdingDirectory,std::filesystem::path aimDirectory) : _directory(std::move(directory)), _holdingDirectory(std::move(holdingDirectory)), _aimDirectory(std::move(aimDirectory)), _queue(std::make_unique<std::array<Sample,capacity>>()), _worker([this] { run(); }) {}
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
                if(sample.phase==2&&sample.aimRequested){try{
                    using namespace holding_grip_math;using J=nlohmann::json;
                    const auto error=[](AimError e)->J {if(!e.valid)return nullptr;return J{{"yawDegrees",e.yaw},{"pitchDegrees",e.pitch},{"angleDegrees",e.angle}};};
                    J frames=J::object();for(std::size_t i=0;i<PoseCount;++i)if(sample.has(static_cast<gunstock_debug::Pose>(i))){const auto p=fromNative(sample.poses[i]);frames[poseNames[i]]=J{{"rotate",p.r},{"translate",p.t},{"scale",p.s}};}
                    J errors=J::object();const auto active=sample.firingLeft?LeftAimDamped:RightAimDamped;
                    if(sample.has(active)&&sample.has(WeaponRendered))errors["activePredictedToRendered"]=error(aimError(sample.poses[active],sample.poses[WeaponRendered]));
                    if(sample.has(LeftAimReference)&&sample.has(RightAimReference))errors["rightReferenceToLeftReference"]=error(aimError(sample.poses[RightAimReference],sample.poses[LeftAimReference]));
                    if(sample.has(ProjectileRendered)&&sample.has(WeaponRendered))errors["barrelToProjectile"]=error(aimError(sample.poses[WeaponRendered],sample.poses[ProjectileRendered]));
                    if(sample.aimReferenceValid)for(const auto id:{LeftAimConfigured,RightAimConfigured})if(sample.has(id)){const auto& t=sample.poses[id].translate;errors[id==LeftAimConfigured?"leftConfiguredToTarget":"rightConfiguredToTarget"]=error(aimDirectionError(sample.poses[id],{sample.aimReferenceTarget[0]-t.x,sample.aimReferenceTarget[1]-t.y,sample.aimReferenceTarget[2]-t.z}));}
                    J j{{"format","rock-controller-aim-capture"},{"version",1},{"space","world"},{"matrixConvention","active-row-major"},{"units","game"},{"phase","AfterWorldFinal"},{"frame",sample.frame},{"weaponFormId",sample.form},{"generation",sample.generation},{"firingHand",sample.firingLeft?"left":"right"},{"supportActive",sample.supportActive!=0},{"gunstockAim",sample.gunstockAim!=0},{"leftAimYawDegrees",sample.aimYawDegrees},{"leftAimPitchDegrees",sample.aimPitchDegrees},{"referenceMeaning","59-degree controller basis; physical intended direction is established by the user"},{"trackingSource","game wand frames through the FRIK tracked-hand provider"},{"frames",frames},{"errors",errors}};
                    if(sample.aimReferenceValid)j["rightCapturedTarget"]=J{{"position",sample.aimReferenceTarget},{"frame",sample.aimReferenceFrame},{"distanceGameUnits",500}};
                    std::filesystem::create_directories(_aimDirectory);std::ofstream capture(_aimDirectory/(std::to_string(sample.form)+"-"+std::to_string(sample.tick)+"-"+std::to_string(sample.frame)+".json"));capture<<j.dump(2);capture.flush();if(!capture)_aimFailed.store(true);else{_aimFailed.store(false);++_aimWritten;}
                }catch(const std::exception&){_aimFailed.store(true);}}
                if(sample.holding.captureOnly)continue;
                if (!file.is_open()) {
                    std::filesystem::create_directories(_directory);
                    const auto stamp = std::chrono::system_clock::now().time_since_epoch().count();
                    file.open(_directory / ("gunstock-" + std::to_string(stamp) + ".csv"));
                    file.imbue(std::locale::classic());
                    file << std::setprecision(9);
                    file << "frame,tick_ms,generation,ownership,grip,mark,valid,form,state,firing_left,support_active,authored,part,omod,attach_point,phase,gunstock_aim,dt,source";
                    const auto& names=poseNames;
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
