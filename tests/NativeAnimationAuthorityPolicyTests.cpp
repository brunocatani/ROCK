#include "physics-interaction/animation/NativeAnimationAuthorityPolicy.h"

#include <cassert>

int main()
{
    using namespace rock::native_animation_authority_policy;

    static_assert(classifyBone("RArm_Collarbone") == kArms);
    static_assert(classifyBone("LArm_ForeArm3") == kArms);
    static_assert(classifyBone("RArm_Hand") == (kArms | kHands));
    static_assert(classifyBone("LArm_Finger23") == kHands);
    static_assert(classifyBone("RArm_Thumb1") == kHands);
    static_assert(classifyBone("Weapon") == kWeapon);
    static_assert(classifyBone("weaponleft") == kWeapon);

    static_assert(classifyBone("Root") == 0);
    static_assert(classifyBone("COM") == 0);
    static_assert(classifyBone("SPINE1") == 0);
    static_assert(classifyBone("Chest") == 0);
    static_assert(classifyBone("Head") == 0);
    static_assert(classifyBone("LLeg_Thigh") == 0);
    static_assert(classifyBone("WeaponMagazine") == 0);

    assert(isRequested(classifyBone("LArm_Hand"), kArms));
    assert(isRequested(classifyBone("LArm_Hand"), kHands));
    assert(!isRequested(classifyBone("LArm_Finger11"), kArms));
    assert(!isRequested(classifyBone("RArm_UpperArm"), kHands));
    assert(isRequested(classifyBone("WeaponLeft"), kReloadPose));
    return 0;
}
