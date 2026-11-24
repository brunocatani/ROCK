// Looks up FRIK's weapon offsets for a weapon through FRIK's API.
#include "physics-interaction/weapon/handling/FrikWeaponOffsets.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/authored/HoldingGripAdapter.h"
#include "rock_support/Fo4VrRuntime.h"

#include "RE/Bethesda/TESForms.h"
#include "RE/NetImmerse/NiNode.h"

#include <array>
#include <string>

namespace rock::frik_weapon_offsets
{
    namespace
    {
        namespace policy = frik_weapon_offset_policy;
        using frik_visual_authority::WeaponOffsetMode;
        using frik_visual_authority::WeaponOffsetSource;

        // What find() last logged for a weapon, so a change is logged once.
        struct Logged
        {
            WeaponOffsetSource weapon{ WeaponOffsetSource::None };
            WeaponOffsetSource primaryHand{ WeaponOffsetSource::None };
            bool powerArmor{ false };
            bool refused{ false };

            bool operator==(const Logged&) const = default;
        };

        struct KeySlot
        {
            // Identity witnesses only; never dereferenced.
            const RE::TESObjectWEAP* weapon{ nullptr };
            const RE::NiAVObject* root{ nullptr };
            bool used{ false };
            std::string key;
            Logged logged{};
        };

        // Frame-thread state: FRIK's weapon name per weapon model, which costs
        // a node search to build.
        std::array<KeySlot, 4> s_keys{};
        std::size_t s_nextSlot{ 0 };

        [[nodiscard]] std::string_view gripStockName(const RE::NiAVObject* root)
        {
            auto* mount = root ? f4vr::findNode(const_cast<RE::NiAVObject*>(root), "P-Grip") : nullptr;
            auto* child = mount ? f4vr::getFirstChild(mount) : nullptr;
            return child && child->name.c_str() ? std::string_view{ child->name.c_str() } : std::string_view{};
        }

        [[nodiscard]] KeySlot& keySlot(const RE::TESObjectWEAP* weapon, const RE::NiAVObject* root)
        {
            for (auto& slot : s_keys) {
                if (slot.used && slot.weapon == weapon && slot.root == root) {
                    return slot;
                }
            }
            auto& slot = s_keys[s_nextSlot];
            s_nextSlot = (s_nextSlot + 1) % s_keys.size();
            slot = { .weapon = weapon,
                .root = root,
                .used = true,
                .key = policy::weaponKey(RE::TESFullName::GetFullName(*weapon, false), gripStockName(root)) };
            return slot;
        }

        /*
         * One of FRIK's offsets for the weapon, when it counts: a built-in one
         * only while FrikBuiltInWeaponOffsets is on. source reports what FRIK
         * had; refused is set when that offset is no hand placement.
         */
        [[nodiscard]] std::optional<policy::Pose> offset(const std::string& key, const WeaponOffsetMode mode,
            const bool inPowerArmor, WeaponOffsetSource& source, bool& refused)
        {
            RE::NiTransform transform{};
            source = frik_visual_authority::getWeaponOffset(key.c_str(), mode, inPowerArmor, transform);
            if (source == WeaponOffsetSource::None ||
                (source == WeaponOffsetSource::BuiltIn && !g_rockConfig.rockFrikBuiltInWeaponOffsets)) {
                source = WeaponOffsetSource::None;
                return std::nullopt;
            }
            auto pose = policy::validated(holding_grip_math::fromNative(transform));
            refused = refused || !pose;
            return pose;
        }

        [[nodiscard]] const char* sourceName(const WeaponOffsetSource source)
        {
            switch (source) {
            case WeaponOffsetSource::BuiltIn:
                return "built-in";
            case WeaponOffsetSource::Saved:
                return "saved";
            default:
                return "none";
            }
        }
    }

    std::optional<Override> find(const RE::TESObjectWEAP* weapon, const RE::NiAVObject* weaponRoot,
        const bool inPowerArmor)
    {
        // Opt-in: without the setting no offset affects any weapon.
        if (!g_rockConfig.rockFrikUserWeaponOffsets || !weapon) {
            return std::nullopt;
        }
        auto& slot = keySlot(weapon, weaponRoot);
        Logged now{ .powerArmor = inPowerArmor };
        const auto weaponInHand = offset(slot.key, WeaponOffsetMode::Weapon, inPowerArmor, now.weapon, now.refused);
        const auto primaryHand =
            offset(slot.key, WeaponOffsetMode::PrimaryHand, inPowerArmor, now.primaryHand, now.refused);
        auto result = policy::resolve(weaponInHand, primaryHand);
        if (now != slot.logged) {
            slot.logged = now;
            if (now.refused) {
                ROCK_LOG_WARN(Weapon,
                    "FRIK weapon offset for {:08X} key='{}' powerArmor={} is not a hand placement; ROCK keeps its "
                    "grip for it",
                    weapon->formID, slot.key, inPowerArmor);
            }
            if (result) {
                ROCK_LOG_INFO(Weapon,
                    "FRIK weapon offset applies to {:08X} key='{}' powerArmor={} weapon={} primaryHand={} "
                    "scale={:.4f}",
                    weapon->formID, slot.key, inPowerArmor, sourceName(now.weapon), sourceName(now.primaryHand),
                    result->weaponScale());
            }
        }
        return result;
    }

    RE::NiTransform seat(const Override& override, const RE::NiTransform& nativeHandInWeapon)
    {
        return holding_grip_math::toNative(policy::seat(override, holding_grip_math::fromNative(nativeHandInWeapon)));
    }
}
