#pragma once
#include <array>
#include <cmath>
#include <algorithm>
#include <span>
namespace rock::holding_grip_math {
using V=std::array<float,3>;
struct Pose { std::array<float,9> r{1,0,0,0,1,0,0,0,1};V t{};float s{1}; };
inline V add(V a,V b){for(int i=0;i<3;++i)a[i]+=b[i];return a;}
inline V sub(V a,V b){for(int i=0;i<3;++i)a[i]-=b[i];return a;}
inline V rotate(const Pose& p,V v){V o{};for(int i=0;i<3;++i)for(int j=0;j<3;++j)o[i]+=p.r[i*3+j]*v[j];return o;}
inline V point(const Pose& p,V v){for(auto& x:v)x*=p.s;return add(p.t,rotate(p,v));}
inline Pose compose(const Pose& a,const Pose& b){Pose o;o.r={};for(int i=0;i<3;++i)for(int j=0;j<3;++j)for(int k=0;k<3;++k)o.r[i*3+j]+=a.r[i*3+k]*b.r[k*3+j];o.t=point(a,b.t);o.s=a.s*b.s;return o;}
inline Pose inverse(const Pose& p){Pose o;for(int i=0;i<3;++i)for(int j=0;j<3;++j)o.r[i*3+j]=p.r[j*3+i];o.s=1/p.s;V v=p.t;for(auto& x:v)x=-x*o.s;o.t=rotate(o,v);return o;}
inline Pose reflected(Pose p,int axis){p.t[axis]=-p.t[axis];for(int i=0;i<3;++i)for(int j=0;j<3;++j)if((i==axis)!=(j==axis))p.r[i*3+j]=-p.r[i*3+j];return p;}
inline Pose finger(Pose p){return reflected(p,2);}
// Broad longitudinal surfaces dominate small asymmetric controls. Median
// weighting avoids moving the plane toward a single side attachment.
struct PlaneEvidence { float center{}, weight{}; };
inline PlaneEvidence planeEvidence(V low, V high, int axis = 0) {
    const int a=(axis+1)%3,b=(axis+2)%3;
    const float weight=(high[a]-low[a])*(high[b]-low[b]);
    if(!std::isfinite(weight)||weight<=0||high[axis]<=low[axis])return {};
    return {(low[axis]+high[axis])*.5f,weight};
}
// The weapon-local plane a left seat mirrors across. A firearm mirrors its
// lateral X. Melee models carry the edge, head or striking face on X and the
// flat side on Z (knife, machete, axe, knuckles), so they mirror Z; mirroring
// X would turn the edge to the back.
struct MirrorPlane {
    int axis{0};
    float offset{0};
    bool operator==(const MirrorPlane&) const = default;
};
inline int mirrorAxis(bool melee){return melee?2:0;}
// One model's geometry plane on each mirror axis. The holder's weapon kind
// picks the axis, so a seat never waits for generated collision to decide it.
struct MirrorOffsets {
    float x{0}, z{0};
    MirrorPlane plane(bool melee) const { return melee ? MirrorPlane{2,z} : MirrorPlane{0,x}; }
};
inline float geometryPlane(std::span<PlaneEvidence> evidence) {
    std::sort(evidence.begin(),evidence.end(),[](const auto& a,const auto& b){return a.center<b.center;});
    float total=0;for(const auto& e:evidence)total+=e.weight;
    if(total<=0)return 0;
    float sum=0;for(const auto& e:evidence){sum+=e.weight;if(sum>=total*.5f)return e.center;}
    return 0;
}
// Mirror the weapon-local plane axis and hand-local Z together. Both
// reflections produce a proper wrist rotation; mirrored finger locals then
// preserve every contact. Controller bases belong to weapon aiming, not this
// authored mesh relation.
inline Pose mirroredHandSeat(Pose right, MirrorPlane plane = {}) {
    right.t[plane.axis] = 2 * plane.offset - right.t[plane.axis];
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            if ((row == plane.axis) != (column == 2)) right.r[row * 3 + column] *= -1;
        }
    }
    return right;
}
inline bool valid(const Pose& p){if(!std::isfinite(p.s)||p.s<=.0001f)return false;for(auto x:p.t)if(!std::isfinite(x))return false;for(auto x:p.r)if(!std::isfinite(x))return false;for(int i=0;i<3;++i)for(int j=0;j<3;++j){float dot=0;for(int k=0;k<3;++k)dot+=p.r[i*3+k]*p.r[j*3+k];if(std::abs(dot-(i==j?1.f:0.f))>.002f)return false;}const auto& m=p.r;return std::abs(m[0]*(m[4]*m[8]-m[5]*m[7])-m[1]*(m[3]*m[8]-m[5]*m[6])+m[2]*(m[3]*m[7]-m[4]*m[6])-1)<.002f;}
}
