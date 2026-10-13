#pragma once
#include "physics-interaction/weapon/HoldingGripMath.h"
#include "RE/NetImmerse/NiTransform.h"
namespace rock::holding_grip_math {
inline Pose fromNative(const RE::NiTransform& p){Pose o;for(int i=0;i<3;++i)for(int j=0;j<3;++j)o.r[i*3+j]=p.rotate.entry[j][i];o.t={p.translate.x,p.translate.y,p.translate.z};o.s=p.scale;return o;}
inline RE::NiTransform toNative(const Pose& p){RE::NiTransform o;for(int i=0;i<3;++i)for(int j=0;j<3;++j)o.rotate.entry[j][i]=p.r[i*3+j];o.translate={p.t[0],p.t[1],p.t[2]};o.scale=p.s;return o;}
}
