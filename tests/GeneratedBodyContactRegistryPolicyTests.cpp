#include "physics-interaction/contact/GeneratedBodyContactRegistry.h"

#include <cstdio>
#include <cstdint>

namespace
{
    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }

        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, bool value)
    {
        if (!value) {
            return true;
        }

        std::printf("%s expected false\n", label);
        return false;
    }

    template <class T>
    bool expectEqual(const char* label, T actual, T expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected %llu got %llu\n", label, static_cast<unsigned long long>(expected), static_cast<unsigned long long>(actual));
        return false;
    }
}

int main()
{
    using namespace rock::generated_body_contact_registry;

    bool ok = true;

    {
        Registry<8> registry;
        Entry entries[4]{};
        entries[0].bodyId = 300;
        entries[0].kind = GeneratedBodyKind::Weapon;
        entries[0].partKind = 7;
        entries[0].role = 2;
        entries[0].subRole = 5;
        entries[0].flags = kFlagSampledVelocity;
        entries[0].sampledVelocityHavokX = 1.0f;
        entries[0].sampledVelocityHavokY = 2.0f;
        entries[0].sampledVelocityHavokZ = 3.0f;

        entries[1].bodyId = 100;
        entries[1].kind = GeneratedBodyKind::RightHand;
        entries[1].role = 9;
        entries[1].flags = kFlagPrimaryAnchor;

        entries[2].bodyId = 200;
        entries[2].kind = GeneratedBodyKind::Body;
        entries[2].zone = 4;
        entries[2].side = 1;
        entries[2].flags = kFlagPowerArmor;

        entries[3].bodyId = kInvalidBodyId;
        entries[3].kind = GeneratedBodyKind::LeftHand;

        registry.publish(entries, 4);
        ok &= expectEqual("registry publishes valid unique entries", registry.count(), static_cast<std::uint32_t>(3));

        Classification hand{};
        ok &= expectTrue("right hand classified", registry.tryClassify(100, hand));
        ok &= expectEqual("right hand kind", hand.kind, GeneratedBodyKind::RightHand);
        ok &= expectEqual("right hand role", hand.role, static_cast<std::uint32_t>(9));
        ok &= expectTrue("right hand primary flag", hasFlag(hand.flags, kFlagPrimaryAnchor));

        Classification weapon{};
        ok &= expectTrue("weapon classified", registry.tryClassify(300, weapon));
        ok &= expectEqual("weapon kind", weapon.kind, GeneratedBodyKind::Weapon);
        ok &= expectEqual("weapon part", weapon.partKind, static_cast<std::uint32_t>(7));
        ok &= expectTrue("weapon sampled velocity valid", hasFiniteSampledVelocity(weapon));

        Classification body{};
        ok &= expectTrue("body classified", registry.tryClassify(200, body));
        ok &= expectEqual("body kind", body.kind, GeneratedBodyKind::Body);
        ok &= expectTrue("body power armor flag", hasFlag(body.flags, kFlagPowerArmor));

        Classification unknown{};
        ok &= expectFalse("unknown body rejected", registry.tryClassify(999, unknown));
        registry.clear();
        ok &= expectEqual("clear empties registry", registry.count(), static_cast<std::uint32_t>(0));
        ok &= expectFalse("cleared body rejected", registry.tryClassify(100, hand));
    }

    {
        Registry<4> registry;
        Entry entries[3]{};
        entries[0].bodyId = 42;
        entries[0].kind = GeneratedBodyKind::RightHand;
        entries[1].bodyId = 42;
        entries[1].kind = GeneratedBodyKind::Weapon;
        entries[2].bodyId = 43;
        entries[2].kind = GeneratedBodyKind::Body;

        registry.publish(entries, 3);
        Classification classification{};
        ok &= expectFalse("duplicate body IDs are rejected", registry.tryClassify(42, classification));
        ok &= expectTrue("non-duplicate survives duplicate rejection", registry.tryClassify(43, classification));
        ok &= expectEqual("non-duplicate body kind", classification.kind, GeneratedBodyKind::Body);
    }

    return ok ? 0 : 1;
}
