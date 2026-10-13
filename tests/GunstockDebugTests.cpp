#include "physics-interaction/weapon/GunstockDebug.h"
#include "physics-interaction/TransformMath.h"
#include <cmath>
#include <iostream>
#include <limits>

int main()
{
    using namespace rock::gunstock_debug;
    Sample sample{};
    auto pose = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    pose.translate = {17, -23, 61};
    if (!setPose(sample, LeftController, pose) || !sample.has(LeftController)) return 2;
    pose.translate.x = (std::numeric_limits<float>::quiet_NaN)();
    if (setPose(sample, RightController, pose) || sample.has(RightController)) return 3;
    auto identity=rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    auto yaw=identity;yaw.rotate.entry[0][0]=0;yaw.rotate.entry[1][1]=0;
    yaw.rotate.entry[0][1]=-1;yaw.rotate.entry[1][0]=1;
    const auto error=aimError(identity,yaw);
    if(!error.valid||std::abs(error.yaw-90)>0.001f||std::abs(error.pitch)>0.001f||std::abs(error.angle-90)>0.001f)return 4;
    if(aimDirectionError(identity,{0,0,0}).valid)return 5;
    std::cout << "Verified invalid-pose rejection and aim error angles.\n";
    return 0;
}
