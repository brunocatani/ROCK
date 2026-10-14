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
    // A narrowed capture can retain reader/revision and count but change output membership.
    copied.payload = SkeletonBoneCapturePayload::HandRootsWithSceneNodes;
    std::swap(copied.bones[0], copied.bones[4]);
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
    config.rockHighlightEnabled = false;
    assert(collider_tuning::prepareHand(config, false).signature == defaults.signature);
    config.rockHandPalmColliderDimensionScaleOverrides = "PalmAnchor=1,1,1";
    assert(collider_tuning::prepareHand(config, false).signature != defaults.signature);
    config.rockHandPalmColliderDimensionScaleOverrides = RockConfigValues{}.rockHandPalmColliderDimensionScaleOverrides;
    assert(collider_tuning::prepareHand(config, false).signature == defaults.signature);
    return 0;
}
