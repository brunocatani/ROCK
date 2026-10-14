#include "physics-interaction/weapon/HoldingGripMath.h"
#include <cassert>
#include <iostream>
using namespace rock::holding_grip_math;
void near(V a,V b){for(int i=0;i<3;++i)assert(std::abs(a[i]-b[i])<.0001f);}
int main(){
 Pose right;right.r={0,-1,0,1,0,0,0,0,1};right.t={2,7,-3};right.s=1.2f;
 Pose br; br.r={1,0,0,0,0,-1,0,1,0};Pose bl;bl.r={0,0,1,0,1,0,-1,0,0};
 V palm{6,-2,.2f},leftPalm{6,-2,-.2f};const auto grip=point(right,palm);
 auto f=finger(finger(right));near(f.t,right.t);for(int i=0;i<9;++i)assert(f.r[i]==right.r[i]);assert(f.s==right.s);

 // A thumb/finger contact expressed through the mirrored wrist and local
 // joint chain must be the weapon-space reflection of the original contact.
 for (const auto& source : {right, compose(br,right), compose(bl,right)}) {
  const auto seat=mirroredHandSeat(source);assert(valid(seat));
  const auto restored=mirroredHandSeat(seat);near(restored.t,source.t);
  for(int i=0;i<9;++i)assert(restored.r[i]==source.r[i]);
  Pose joint=br;joint.t={1.2f,-.8f,.4f};
  for(V contact : {V{6,-2,.2f},V{3,1,-.9f},V{0,0,0}}) {
   auto original=point(compose(source,joint),contact);original[0]=-original[0];
   contact[2]=-contact[2];
   near(point(compose(seat,finger(joint)),contact),original);
  }
  // The same involution preserves left-support to right-support contact.
  auto supportContact=point(seat,leftPalm);supportContact[0]=-supportContact[0];
  near(point(mirroredHandSeat(seat),palm),supportContact);
  auto mirroredGrip=point(source,palm);mirroredGrip[0]=-mirroredGrip[0];
  near(point(seat,leftPalm),mirroredGrip);
 }
 // Shifting the weapon's modeling origin must not alter mirrored contact.
 for(float center:{-.8f,0.f,.379f,2.f}) {
  const auto seat=mirroredHandSeat(right,center);auto expected=point(right,palm);expected[0]=2*center-expected[0];
  near(point(seat,leftPalm),expected);
  auto shifted=right;shifted.t[0]+=3.2f;auto mirroredShift=mirroredHandSeat(shifted,center+3.2f);
  auto expectedShift=seat.t;expectedShift[0]+=3.2f;near(mirroredShift.t,expectedShift);
 }
 std::array<PlaneEvidence,3> planes{planeEvidence({-1.6f,0,-2},{2.4f,20,2}),planeEvidence({-1.6f,-20,-3},{2.4f,0,3}),planeEvidence({5,1,0},{6,2,1})};
 assert(std::abs(geometryPlane(planes)-.4f)<.0001f);
 near(point(compose(right,inverse(right)),{1,2,3}),{1,2,3});Pose bad;bad.r[0]=-1;assert(!valid(bad));
 std::cout<<"PASS holding grip orientation, independent palm placement, scale and exact finger reflection\n";
}
