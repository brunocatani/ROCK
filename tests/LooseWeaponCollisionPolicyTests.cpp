#include "physics-interaction/weapon/LooseWeaponCollisionPolicy.h"
#include "physics-interaction/collision/NativePlayerCollisionPolicy.h"
#include <cassert>
#include <limits>
namespace
{
    enum class WeaponType : std::uint8_t {
        kHandToHand, kOneHandSword, kOneHandDagger, kOneHandAxe, kOneHandMace,
        kTwoHandSword, kTwoHandAxe, kBow, kStaff, kGun, kGrenade, kMine, kNone=255
    };
    struct Pose
    {
        struct { float entry[3][3]{}; } rotate;
        struct { float x{}, y{}, z{}; } translate;
    };

    Pose pose(float xGame, float degrees)
    {
        Pose result{};
        const float angle=degrees*0.017453293f;
        result.rotate.entry[0][0]=result.rotate.entry[1][1]=std::cos(angle);
        result.rotate.entry[0][1]=std::sin(angle);
        result.rotate.entry[1][0]=-std::sin(angle);
        result.rotate.entry[2][2]=1;
        result.translate.x=xGame;
        return result;
    }
}

int main()
{
    using rock::loose_weapon_collision_policy::eligibleType;
    for (unsigned type=0;type<256;++type)
        assert(eligibleType(static_cast<WeaponType>(type))==(type<=6 || type==9));
    using rock::loose_weapon_collision_policy::Parking;
    using rock::loose_weapon_collision_policy::poseDelta;
    using rock::loose_weapon_collision_policy::admissions;
    using Slot=rock::loose_weapon_collision_policy::Slot;
    using Ids=std::array<std::uint32_t,rock::loose_weapon_collision_policy::kMaxWeapons>;
    // Sessions follow exact world references, independently of hand swaps.
    assert((admissions({}, {11,22})==Ids{11,22}));
    assert((admissions({}, {11,11})==Ids{11,0}));
    assert((admissions({}, {0,22})==Ids{22,0}));
    assert((admissions({Slot{11},Slot{}}, {22,11})==Ids{0,22}));
    std::array<Slot,rock::loose_weapon_collision_policy::kMaxWeapons> active{};
    for (std::size_t i=0;i<active.size();++i) active[i]={100u+static_cast<std::uint32_t>(i)};
    active[0]={11};active[1]={22};
    assert((admissions(active, {22,11})==Ids{}));
    // A dropped slot is not evicted while its gun is falling. Once parked,
    // it can serve a new gun while the other held gun remains untouched.
    assert((admissions(active, {22,33})==Ids{}));
    active[0]={}; // Completion frees one owner without evicting another drop.
    assert((admissions(active, {22,33})==Ids{33,0}));
    assert((admissions({}, {22,22})==Ids{22}));

    std::array<Parking,2> timers;
    assert(!timers[0].update(false,true,true,1.5f));
    assert(!timers[1].update(false,true,true,0.5f));
    assert(!timers[0].update(true,true,true,0.5f));
    assert(timers[0].supportedRestSeconds==0);
    assert(timers[1].update(false,true,true,1.5f));
    Parking dropped;
    // Time and zero motion alone never retire an unsupported airborne gun.
    for (int i=0;i<100;++i) assert(!dropped.update(false,false,true,0.125f));
    assert(dropped.supportedRestSeconds==0);
    assert(!dropped.update(false,true,false,1.0f));
    Parking nearby;
    for (int i=0;i<15;++i) assert(!nearby.update(false,true,true,0.125f));
    assert(nearby.update(false,true,true,0.125f));
    assert(nearby.supportedRestSeconds==2.0f);
    Parking regrabbed;
    assert(!regrabbed.update(false,true,true,1.5f));
    assert(!regrabbed.update(true,true,true,1.0f));
    assert(regrabbed.supportedRestSeconds==0);
    assert(!regrabbed.update(false,true,true,1.0f));
    Parking bouncing;
    assert(!bouncing.update(false,true,true,1.5f));
    assert(!bouncing.update(false,false,true,0.1f));
    assert(bouncing.supportedRestSeconds==0);
    assert(!bouncing.update(false,true,true,1.5f));
    assert(!bouncing.update(false,true,false,0.1f));
    assert(bouncing.supportedRestSeconds==0);
    assert(!bouncing.update(false,true,true,1.5f));
    assert(bouncing.update(false,true,true,0.5f));

    constexpr float gameToMeters=0.01f;
    const auto anchor=pose(0,30);
    Parking jitter;
    // Millimetre oscillations and sub-degree rotation keep the whole dwell,
    // even though their instantaneous velocity can exceed the old cutoff.
    for (int i=0;i<16;++i) {
        const float direction=i%2 ? 1.0f : -1.0f;
        const auto delta=poseDelta(pose(0.2f*direction,30+0.5f*direction),anchor,gameToMeters);
        assert(delta.withinRestTolerance());
        assert(jitter.update(false,true,delta.withinRestTolerance(),0.125f)==(i==15));
    }
    // Bounded per-frame movement must not conceal continued drift or rolling.
    assert(poseDelta(pose(0.9f,31.9f),anchor,gameToMeters).withinRestTolerance());
    assert(!poseDelta(pose(1.1f,30),anchor,gameToMeters).withinRestTolerance());
    assert(!poseDelta(pose(0,32.1f),anchor,gameToMeters).withinRestTolerance());
    Parking sliding;
    auto windowAnchor=anchor;
    for (int i=0;i<160;++i) {
        const auto current=pose(0.2f*i,30);
        if (sliding.supportedRestSeconds==0) windowAnchor=current;
        assert(!sliding.update(false,true,poseDelta(current,windowAnchor,gameToMeters).withinRestTolerance(),0.125f));
    }
    const float nan=std::numeric_limits<float>::quiet_NaN();
    assert(!poseDelta(pose(nan,30),anchor,gameToMeters).withinRestTolerance());
    assert(!poseDelta(pose(0,nan),anchor,gameToMeters).withinRestTolerance());
    assert(!poseDelta(anchor,anchor,nan).withinRestTolerance());
    assert(!jitter.update(false,true,true,nan) && jitter.supportedRestSeconds==0);
    assert(!jitter.update(false,true,true,0) && jitter.supportedRestSeconds==0);
    using namespace rock::native_player_collision;
    using namespace rock::collision_layer_policy;
    using Motion=rock::physics_body_classifier::BodyMotionType;
    for (auto hand : {ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY,ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY}) {
        assert(isRetiredWeaponHandPair(hand,FO4_LAYER_WEAPON));
        assert(isRetiredWeaponHandPair(FO4_LAYER_WEAPON,hand));
        assert(!isRetiredWeaponHandPair(hand,FO4_LAYER_CLUTTER));
    }
    assert(!isRetiredWeaponHandPair(ROCK_LAYER_DYNAMIC_WEAPON_PROXY,FO4_LAYER_WEAPON));
    assert(allowsRetiredWeaponHandContact(Motion::Static,true));
    assert(allowsRetiredWeaponHandContact(Motion::Keyframed,true));
    assert(!allowsRetiredWeaponHandContact(Motion::Dynamic,true));
    assert(!allowsRetiredWeaponHandContact(Motion::Keyframed,false));
    const auto original=buildRockDynamicHandProxyExpectedMask(false,false,false);
    const auto fixed=buildRockDynamicHandProxyExpectedMask(false,false,true);
    assert((original^fixed)==layerBitOrZero(FO4_LAYER_WEAPON));
}
