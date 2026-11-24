#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/authored/HoldingGripAdapter.h"
#include "physics-interaction/weapon/handling/FrikWeaponOffsetPolicy.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <nlohmann/json.hpp>

/*
 * The seat ROCK derives from FRIK's weapon offsets must equal the hand
 * relation FRIK itself produces with the engine's own transforms: FRIK writes
 * the Weapon local under RArm_Hand, then rotates the hand as
 * hand.local = R * hand.local (NiMatrix3 product) with the weapon unmoved.
 */
namespace
{
    using namespace rock;
    namespace policy = frik_weapon_offset_policy;

    // FRIK's own Minigun.json, values as FRIK stores them; getWeaponOffset
    // hands ROCK the same NiTransform.
    constexpr const char* kMinigun =
        R"({"Minigun":{"rotation":[-0.12200000137090683,0.9900000095367432,0.0689999982714653,0.0,
        0.9869999885559082,0.11400000005960464,0.10899999737739563,0.0,0.10000000149011612,0.08100000023841858,-0.9919999837875366,0.0],
        "scale":1.25,"x":9.201794624328613,"y":6.294676780700684,"z":6.744982719421387},
        "Minigun-primHand-PowerArmor":{"rotation":[0.9659258,0.0,-0.2588190,0.0,0.0,1.0,0.0,0.0,0.2588190,0.0,0.9659258,0.0],
        "scale":1.0,"x":0.0,"y":0.0,"z":0.0}})";

    RE::NiTransform engineTransform(const nlohmann::json& value)
    {
        RE::NiTransform t{};
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                t.rotate.entry[row][column] = value["rotation"][row * 4 + column].get<float>();
            }
        }
        t.translate = { value["x"].get<float>(), value["y"].get<float>(), value["z"].get<float>() };
        t.scale = value["scale"].get<float>();
        return transform_math::orthonormalizedTransform(t);
    }

    // Test input frames: elementary rotations combined with CommonLib's product.
    // (FromEulerAngles* relocates into the game and cannot run here.)
    RE::NiMatrix3 axisRotation(const int axis, const float angle)
    {
        RE::NiMatrix3 m{};
        m.MakeIdentity();
        const int a = (axis + 1) % 3;
        const int b = (axis + 2) % 3;
        m.entry[a][a] = std::cos(angle);
        m.entry[a][b] = -std::sin(angle);
        m.entry[b][a] = std::sin(angle);
        m.entry[b][b] = std::cos(angle);
        return m;
    }

    RE::NiTransform rotationAbout(float x, float y, float z, float t)
    {
        RE::NiTransform r{};
        r.rotate = axisRotation(0, x) * axisRotation(1, y) * axisRotation(2, z);
        r.translate = { t, -2.0f * t, 0.5f * t };
        r.scale = 1.0f;
        return r;
    }

    void near(const RE::NiTransform& a, const RE::NiTransform& b, float tolerance)
    {
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                assert(std::abs(a.rotate.entry[row][column] - b.rotate.entry[row][column]) < tolerance);
            }
        }
        assert(std::abs(a.translate.x - b.translate.x) < tolerance * 10.0f);
        assert(std::abs(a.translate.y - b.translate.y) < tolerance * 10.0f);
        assert(std::abs(a.translate.z - b.translate.z) < tolerance * 10.0f);
        assert(std::abs(a.scale - b.scale) < tolerance);
    }
}

int main()
{
    const auto json = nlohmann::json::parse(kMinigun);
    const RE::NiTransform weaponInHand = engineTransform(json["Minigun"]);
    const RE::NiTransform primaryHand = engineTransform(json["Minigun-primHand-PowerArmor"]);
    const auto weaponPose = policy::validated(holding_grip_math::fromNative(weaponInHand));
    const auto handPose = policy::validated(holding_grip_math::fromNative(primaryHand));
    assert(weaponPose && handPose);

    // Weapon offset alone: the seat is the engine inverse of FRIK's Weapon local.
    const auto plain = policy::resolve(weaponPose, std::nullopt);
    assert(plain && plain->hasWeaponOffset && !plain->hasPrimaryHandRotation);
    assert(std::abs(plain->weaponScale() - 1.25f) < 1e-6f);
    near(holding_grip_math::toNative(policy::seat(*plain, {})), transform_math::invertTransform(weaponInHand), 0.002f);

    // Weapon offset and primary-hand rotation: replay FRIK's frame with real
    // engine math.
    const auto both = policy::resolve(weaponPose, handPose);
    assert(both && both->hasWeaponOffset && both->hasPrimaryHandRotation);
    const RE::NiTransform forearm = rotationAbout(0.3f, -1.1f, 2.0f, 40.0f);
    RE::NiTransform handLocal = rotationAbout(-0.4f, 0.2f, 0.7f, 12.0f);
    const RE::NiTransform weaponWorld =
        transform_math::composeTransforms(transform_math::composeTransforms(forearm, handLocal), weaponInHand);
    handLocal.rotate = primaryHand.rotate * handLocal.rotate;
    const RE::NiTransform handWorld = transform_math::composeTransforms(forearm, handLocal);
    const RE::NiTransform frikHandInWeapon =
        transform_math::composeTransforms(transform_math::invertTransform(weaponWorld), handWorld);
    near(holding_grip_math::toNative(policy::seat(*both, {})), frikHandInWeapon, 0.002f);

    // A lone hand rotation keeps the native seat's wrist position.
    const auto rotated = policy::resolve(std::nullopt, handPose);
    assert(rotated && !rotated->replacesSeat() && rotated->weaponScale() == 1.0f);
    const RE::NiTransform nativeSeat = rotationAbout(1.0f, 0.1f, -0.6f, 3.0f);
    const auto seat = holding_grip_math::toNative(policy::seat(*rotated, holding_grip_math::fromNative(nativeSeat)));
    RE::NiTransform handRotation{};
    handRotation.rotate = primaryHand.rotate;
    near(seat, transform_math::composeTransforms(nativeSeat, handRotation), 0.001f);
    assert(!policy::resolve(std::nullopt, std::nullopt));

    // FRIK's pistol/rifle name variants; other weapons keep the display name.
    assert(policy::weaponKey("Pipe", "PipeStockFull") == "Pipe Rifle");
    assert(policy::weaponKey("Pipe", "PipeGripPistol") == "Pipe");
    assert(policy::weaponKey("Laser", "LaserRifleStock") == "Laser Rifle");
    assert(policy::weaponKey("10mm", "PipeStock") == "10mm");

    // A mirrored rotation or an implausible scale is not a hand placement.
    policy::Pose mirrored{};
    mirrored.r = { -1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    assert(!policy::validated(mirrored));
    policy::Pose tiny = *weaponPose;
    tiny.s = 0.01f;
    assert(!policy::validated(tiny));

    std::cout << "FRIK weapon offset policy tests passed\n";
    return 0;
}
