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
            std::string key;
            WeaponOffsetSource weapon{ WeaponOffsetSource::None };
            WeaponOffsetSource primaryHand{ WeaponOffsetSource::None };
            bool powerArmor{ false };
            bool refused{ false };

            bool operator==(const Logged&) const = default;
        };

        struct LogSlot
        {
            // Identity witness only; never dereferenced.
            const RE::TESObjectWEAP* weapon{ nullptr };
            Logged logged{};
        };

        // Frame-thread state: the last log per weapon form, for the two
        // equipped weapons and the few lookups between them.
        std::array<LogSlot, 4> s_logs{};
        std::size_t s_nextLog{ 0 };

        [[nodiscard]] std::string_view gripStockName(const RE::NiAVObject* root)
        {
            auto* mount = root ? f4vr::findNode(const_cast<RE::NiAVObject*>(root), "P-Grip") : nullptr;
            auto* child = mount ? f4vr::getFirstChild(mount) : nullptr;
            return child && child->name.c_str() ? std::string_view{ child->name.c_str() } : std::string_view{};
        }

        [[nodiscard]] Logged& logSlot(const RE::TESObjectWEAP* weapon)
        {
            for (auto& slot : s_logs) {
                if (slot.weapon == weapon) {
                    return slot.logged;
                }
            }
            auto& slot = s_logs[s_nextLog];
            s_nextLog = (s_nextLog + 1) % s_logs.size();
            slot = { .weapon = weapon };
            return slot.logged;
        }

        /*
         * FRIK's name for the weapon, read from the model every call: the
         * Weapon node stays while each weapon's model is swapped under it,
         * and a draw attaches the stock after its first frames, so a name
         * kept from then is the pistol's.
         */
        [[nodiscard]] std::string weaponKey(const RE::TESObjectWEAP* weapon, const RE::NiAVObject* root)
        {
            const std::string_view name = RE::TESFullName::GetFullName(*weapon, false);
            return policy::weaponKey(name, policy::namedByStock(name) ? gripStockName(root) : std::string_view{});
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
        Logged now{ .key = weaponKey(weapon, weaponRoot), .powerArmor = inPowerArmor };
        const auto weaponInHand = offset(now.key, WeaponOffsetMode::Weapon, inPowerArmor, now.weapon, now.refused);
        const auto primaryHand =
            offset(now.key, WeaponOffsetMode::PrimaryHand, inPowerArmor, now.primaryHand, now.refused);
        auto result = policy::resolve(weaponInHand, primaryHand);
        if (auto& logged = logSlot(weapon); now != logged) {
            logged = now;
            if (now.refused) {
                ROCK_LOG_WARN(Weapon,
                    "FRIK weapon offset for {:08X} key='{}' powerArmor={} is not a hand placement; ROCK keeps its "
                    "grip for it",
                    weapon->formID, now.key, inPowerArmor);
            }
            if (result) {
                ROCK_LOG_INFO(Weapon,
                    "FRIK weapon offset applies to {:08X} key='{}' powerArmor={} weapon={} primaryHand={} "
                    "scale={:.4f}",
                    weapon->formID, now.key, inPowerArmor, sourceName(now.weapon), sourceName(now.primaryHand),
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
