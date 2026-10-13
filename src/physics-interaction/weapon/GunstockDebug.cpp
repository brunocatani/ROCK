#include "physics-interaction/weapon/GunstockDebug.h"
#include "physics-interaction/weapon/HoldingGripAdapter.h"
#include <algorithm>
#include <cmath>

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
}
