#include "RockConfig.h"
#include "physics-interaction/collision/ColliderTuning.h"
#include "physics-interaction/hand/SkeletonBoneNameIndex.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <utility>

namespace
{
    void near(float actual, float expected)
    {
        assert(std::abs(actual - expected) < 0.0001f);
    }

    void verifyLookup(rock::SkeletonBoneNameIndex& index, const rock::DirectSkeletonBoneSnapshot& snapshot)
    {
        const auto view = index.bind(snapshot);
        for (const auto& bone : snapshot.bones) {
            const auto first = std::find_if(snapshot.bones.begin(), snapshot.bones.end(),
                [&](const auto& candidate) { return candidate.name == bone.name; });
            assert(view.find(bone.name) == &*first);
        }
        assert(!view.find("absent"));
    }
}

int main()
{
    using namespace rock;
    int ownerA = 0, ownerB = 0;
    DirectSkeletonBoneSnapshot snapshot{};
    snapshot.valid = true;
    snapshot.topologyOwner = &ownerA;
    snapshot.topologyRevision = 1;
    for (const auto name : { "Zeta", "Palm", "", "Finger", "Palm" }) {
        DirectSkeletonBoneEntry bone{};
        bone.name = name;
        bone.world.translate.x = static_cast<float>(snapshot.bones.size());
        snapshot.bones.push_back(std::move(bone));
    }
    SkeletonBoneNameIndex index;
    verifyLookup(index, snapshot);
    auto view = index.bind(snapshot);
    near(view.find("Palm")->world.translate.x, 1.0f); // First duplicate wins.
    const auto* storage = view.indices.data();
    snapshot.bones[1].world.translate.x = 17.0f;
    const auto refreshedView = index.bind(snapshot);
    assert(refreshedView.indices.data() == storage);
    near(refreshedView.find("Palm")->world.translate.x, 17.0f); // Poses are never cached.

    // Copy/reallocation must not leave borrowed entry/name pointers in the index.
    auto copied = snapshot;
    copied.bones.reserve(100);
    copied.bones[1].world.translate.x = 23.0f;
    verifyLookup(index, copied);
    near(index.bind(copied).find("Palm")->world.translate.x, 23.0f);
    near(snapshot.bones[1].world.translate.x, 17.0f);

    // Same-sized topology replacement, then a distinct reader at the same revision.
    std::swap(copied.bones[0], copied.bones[3]);
    copied.bones[1].name = "Changed";
    ++copied.topologyRevision;
    verifyLookup(index, copied);
    copied.topologyOwner = &ownerB;
    std::swap(copied.bones[1], copied.bones[4]);
    verifyLookup(index, copied);
    copied.valid = false;
    assert(!index.bind(copied).find("Palm"));
    copied.valid = true;
    verifyLookup(index, copied);
    copied.bones.clear();
    assert(!index.bind(copied).find("Palm"));

    using hand_collider_semantics::HandColliderRole;
    constexpr auto palm = static_cast<std::size_t>(HandColliderRole::PalmAnchor);
    constexpr auto finger = static_cast<std::size_t>(HandColliderRole::IndexBase);
    RockConfigValues config{};
    const auto defaults = collider_tuning::prepareHand(config, false);
    near(defaults.roles[palm].palmDimensionScale.x, 0.8f);
    near(defaults.roles[palm].palmDimensionScale.y, 1.0f);
    near(defaults.roles[palm].palmDimensionScale.z, 2.0f);
    near(defaults.roles[finger].radiusScale, 1.0f);

    config.rockHandBoneColliderRadiusScaleOverrides =
        "IndexBase=0.8;PowerArmor.IndexBase=1.7;IndexBase=0.6;PowerArmor.IndexBase=1.9;PowerArmor.IndexBase=nan;IndexBase=bad";
    config.rockHandPalmColliderDimensionScaleOverrides =
        "PalmAnchor=0.8,1,2;PowerArmor.PalmAnchor=2,3,4;PowerArmor.PalmAnchor=5,6,7;PalmAnchor=1,1,1;PowerArmor.PalmAnchor=1,2,3,4";
    const auto handStandard = collider_tuning::prepareHand(config, false);
    const auto handArmor = collider_tuning::prepareHand(config, true);
    near(handStandard.roles[finger].radiusScale, 0.6f);
    near(handArmor.roles[finger].radiusScale, 1.9f);
    near(handStandard.roles[palm].palmDimensionScale.x, 1.0f);
    near(handArmor.roles[palm].palmDimensionScale.x, 5.0f);
    near(handArmor.roles[palm].palmDimensionScale.z, 7.0f);
    config.rockHandBoneColliderRadiusScaleOverrides = "IndexBase=-2;Standard.IndexBase=20";
    near(collider_tuning::prepareHand(config, false).roles[finger].radiusScale, 8.0f);
    near(collider_tuning::prepareHand(config, true).roles[finger].radiusScale, 0.05f);

    config = {};
    const auto standardDefaults = collider_tuning::prepareBody(config, false);
    const auto armorDefaults = collider_tuning::prepareBody(config, true);
    for (std::size_t i = 0; i < standardDefaults.descriptors.size(); ++i) {
        near(standardDefaults.descriptors[i].radius, skeleton_bone_debug_math::kStandardBodyColliderDescriptors[i].radiusGameUnits);
        near(armorDefaults.descriptors[i].radius, skeleton_bone_debug_math::kPowerArmorBodyColliderDescriptors[i].radiusGameUnits);
        near(standardDefaults.descriptors[i].lengthScale, 1.0f);
    }
    const auto chestIt = std::find_if(skeleton_bone_debug_math::kStandardBodyColliderDescriptors.begin(),
        skeleton_bone_debug_math::kStandardBodyColliderDescriptors.end(),
        [](const auto& descriptor) { return descriptor.zone == body_zone::BodyZoneKind::Chest; });
    assert(chestIt != skeleton_bone_debug_math::kStandardBodyColliderDescriptors.end());
    const auto chest = static_cast<std::size_t>(chestIt - skeleton_bone_debug_math::kStandardBodyColliderDescriptors.begin());
    config.rockBodyBoneColliderZoneScaleOverrides =
        "Chest=2,3,4;PowerArmor.Chest=4,5,6;PowerArmor.Chest=7,7,7;Chest=1,1,1";
    config.rockBodyBoneColliderRadiusScaleOverrides =
        "Chest=0.7;Chest>Neck=0.8;PowerArmor.Chest=0.9;PowerArmor.Chest->Neck=1.1;PowerArmor.Chest>Neck=1.2";
    const auto bodyStandard = collider_tuning::prepareBody(config, false);
    const auto bodyArmor = collider_tuning::prepareBody(config, true);
    near(bodyStandard.descriptors[chest].radius, standardDefaults.descriptors[chest].radius * 0.8f);
    near(bodyStandard.descriptors[chest].lengthScale, 1.0f);
    // Body zones retain FIRST profile match, unlike the hand/radius LAST match.
    near(bodyArmor.descriptors[chest].radius, armorDefaults.descriptors[chest].radius * 4.0f * 1.2f);
    near(bodyArmor.descriptors[chest].lengthScale, 5.0f);
    near(bodyArmor.descriptors[chest].convexRadius, armorDefaults.descriptors[chest].convexRadius * 6.0f);

    config.rockBodyBoneColliderZoneScaleOverrides = "Chest=1,1,1,2,3,4;PowerArmor.Chest=nan,1,1;PowerArmor.Chest=1,2";
    const auto offset = collider_tuning::prepareBody(config, true).descriptors[chest];
    assert(offset.hasLocalOffset);
    near(offset.localOffsetGame.x, 2.0f);
    near(offset.localOffsetGame.z, 4.0f);

    config = {};
    config.rockHighlightEnabled = false;
    assert(collider_tuning::prepareHand(config, false).signature == defaults.signature);
    assert(collider_tuning::prepareBody(config, false).signature == standardDefaults.signature);
    config.rockBodyBoneLegAndFootCollidersEnabled = true;
    assert(collider_tuning::prepareBody(config, false).signature != standardDefaults.signature);
    config.rockHandPalmColliderDimensionScaleOverrides = "PalmAnchor=1,1,1";
    assert(collider_tuning::prepareHand(config, false).signature != defaults.signature);
    config.rockHandPalmColliderDimensionScaleOverrides = RockConfigValues{}.rockHandPalmColliderDimensionScaleOverrides;
    assert(collider_tuning::prepareHand(config, false).signature == defaults.signature);
    return 0;
}
