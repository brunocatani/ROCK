#pragma pack(push,1)
#include <ROCK/NativeInteractionV1_1.h>
struct CallerPacking {char byte;std::uint64_t value;};
static_assert(alignof(CallerPacking)==1);
#pragma pack(pop)

namespace ni=rock::api::native_interaction;
static_assert(static_cast<unsigned>(ni::kInterfaceId)==14);
static_assert(ni::ApiV1::majorVersion==1&&ni::ApiV1::minorVersion==0);
static_assert(sizeof(ni::ApiV1)==176&&alignof(ni::ApiV1)==8);
static_assert(offsetof(ni::ApiV1,registerExtension)==0);
static_assert(offsetof(ni::ApiV1,readBody)==16);
static_assert(offsetof(ni::ApiV1,createBody)==48);
static_assert(offsetof(ni::ApiV1,createSlider)==96);
static_assert(offsetof(ni::ApiV1,setWeaponContactExclusion)==120);
static_assert(offsetof(ni::ApiV1,publishVisuals)==168);
static_assert(sizeof(ni::Frame)==104&&offsetof(ni::Frame,world)==40);
static_assert(sizeof(ni::BodyIdentity)==40&&offsetof(ni::BodyIdentity,shape)==24);
static_assert(sizeof(ni::BodySnapshot)==128&&offsetof(ni::BodySnapshot,world)==48);
static_assert(sizeof(ni::BodyCreate)==160&&offsetof(ni::BodyCreate,pose)==56);
static_assert(sizeof(ni::SliderCreate)==152&&offsetof(ni::SliderCreate,bodyWorld)==16);
static_assert(sizeof(ni::WeaponSource)==80&&sizeof(ni::SourceGeometry)==120);
static_assert(sizeof(ni::WeaponFrame)==216&&sizeof(ni::WeaponCommand)==112);
static_assert(sizeof(ni::Contact)==136);
static_assert(sizeof(ni::SelectionQuery)==56&&sizeof(ni::SelectionReply)==120);
static_assert(offsetof(ni::SelectionReply,ticket)==24);
static_assert(sizeof(ni::ExtractedReference)==16);
static_assert(sizeof(ni::BodyVisual)==136&&offsetof(ni::BodyVisual,label)==68);
static_assert(sizeof(ni::ProximityQuery)==16&&sizeof(ni::ProximityResult)==20);
static_assert(sizeof(ni::ImpactQuery)==24&&sizeof(ni::ImpactResult)==12);
static_assert(sizeof(ni::Callbacks)==128&&offsetof(ni::Callbacks,update)==16);
static_assert(offsetof(ni::Callbacks,validateSelection)==96);
static_assert(offsetof(ni::Callbacks,classifyPush)==120);
static_assert(std::is_standard_layout_v<ni::Callbacks>&&std::is_trivially_copyable_v<ni::SelectionReply>);
int main() {}

static_assert(sizeof(rock::api::native_interaction::v1_1::Api)==184);
static_assert(offsetof(rock::api::native_interaction::v1_1::Api,readContactDetails)==176);
