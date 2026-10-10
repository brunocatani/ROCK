#include "physics-interaction/weapon/HoldingGripMath.h"
#include <cassert>
#include <iostream>
using namespace rock::holding_grip_math;
void near(V a,V b){for(int i=0;i<3;++i)assert(std::abs(a[i]-b[i])<.0001f);}
int main(){
 Pose right;right.r={0,-1,0,1,0,0,0,0,1};right.t={2,7,-3};right.s=1.2f;
 Pose br; br.r={1,0,0,0,0,-1,0,1,0};Pose bl;bl.r={0,0,1,0,1,0,-1,0,0};
 V palm{6,-2,.2f},leftPalm{6,-2,-.2f};const auto grip=point(right,palm);
 const auto left=leftFiring(right,br,bl,grip);assert(valid(left));near(point(inverse(left),grip),leftPalm);
 auto expected=reflected(compose(br,inverse(right)),0);auto actual=compose(bl,inverse(left));for(int i=0;i<9;++i)assert(std::abs(expected.r[i]-actual.r[i])<.0001f);
 auto f=finger(finger(right));near(f.t,right.t);for(int i=0;i<9;++i)assert(f.r[i]==right.r[i]);assert(f.s==right.s);
 auto support=rightSupport(right,bl,br,leftPalm,palm);auto desired=point(right,leftPalm);desired[0]=-desired[0];near(point(support,palm),desired);assert(valid(support));
 near(point(compose(right,inverse(right)),{1,2,3}),{1,2,3});Pose bad;bad.r[0]=-1;assert(!valid(bad));
 std::cout<<"PASS holding grip orientation, independent palm placement, scale and exact finger reflection\n";
}
