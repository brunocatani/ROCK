#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <type_traits>

namespace rock::hand_world_authority_registry_policy
{
    inline constexpr std::size_t kTagCapacity = 64;

    enum class Role : std::uint8_t
    {
        Unknown = 0,
        GrabHeld,
        GrabReturn,
        EquipHandoff,
        DynamicContact,
        PrimaryGrip,
        SupportGrip,
        PrimaryDetach,
        Gunstock,
        WeaponCollision,
        WeaponReturn,
        Provider,
    };

    [[nodiscard]] inline constexpr bool weaponPresentationFollowsRole(
        const Role role) noexcept
    {
        switch (role) {
        case Role::EquipHandoff:
        case Role::DynamicContact:
        case Role::PrimaryGrip:
        case Role::SupportGrip:
        case Role::Gunstock:
        case Role::WeaponCollision:
        case Role::WeaponReturn:
            return true;
        case Role::Unknown:
        case Role::GrabHeld:
        case Role::GrabReturn:
        case Role::PrimaryDetach:
        case Role::Provider:
        default:
            return false;
        }
    }

    template <class Hand, class Target>
    struct Snapshot
    {
        std::array<char, kTagCapacity> tag{};
        std::size_t tagLength = 0;
        Target target{};
        Hand hand{};
        Role role = Role::Unknown;
        int priority = 0;
        std::uint64_t sequence = 0;
        bool reserved = false;
        bool valid = false;
    };

    template <std::size_t Capacity, class Hand, class Target>
    class Registry
    {
    public:
        static_assert(Capacity > 0);
        static_assert(std::is_nothrow_copy_assignable_v<Hand>);
        static_assert(std::is_nothrow_copy_assignable_v<Target>);
        using Entry = Snapshot<Hand, Target>;

        [[nodiscard]] Entry* find(
            const std::string_view tag,
            const Hand hand) noexcept
        {
            for (auto& entry : _entries) {
                if (entry.valid && entry.hand == hand &&
                    tagView(entry) == tag) {
                    return &entry;
                }
            }
            return nullptr;
        }

        [[nodiscard]] const Entry* find(
            const std::string_view tag,
            const Hand hand) const noexcept
        {
            for (const auto& entry : _entries) {
                if (entry.valid && entry.hand == hand &&
                    tagView(entry) == tag) {
                    return &entry;
                }
            }
            return nullptr;
        }

        /*
         * Reserve before the external registry call. A new claim is rejected
         * before hFRIK can accept it when the local fixed table is full.
         */
        [[nodiscard]] Entry* findOrReserve(
            const std::string_view tag,
            const Hand hand) noexcept
        {
            if (tag.empty() || tag.size() >= kTagCapacity) {
                return nullptr;
            }
            if (auto* existing = find(tag, hand)) {
                return existing;
            }
            for (auto& entry : _entries) {
                if (entry.reserved && entry.hand == hand &&
                    tagView(entry) == tag) {
                    return &entry;
                }
            }
            for (auto& entry : _entries) {
                if (!entry.valid && !entry.reserved) {
                    entry.tag.fill('\0');
                    std::copy(tag.begin(), tag.end(), entry.tag.begin());
                    entry.tagLength = tag.size();
                    entry.hand = hand;
                    entry.reserved = true;
                    return &entry;
                }
            }
            return nullptr;
        }

        [[nodiscard]] bool commit(
            Entry& entry,
            const std::string_view tag,
            const Hand hand,
            const Role role,
            const int priority,
            const Target& target) noexcept
        {
            if (tag.empty() || tag.size() >= kTagCapacity ||
                ((!entry.valid && !entry.reserved) ||
                    (entry.hand != hand || tagView(entry) != tag))) {
                return false;
            }

            ++_sequence;
            if (_sequence == 0) {
                _sequence = 1;
            }
            entry.tag.fill('\0');
            std::copy(tag.begin(), tag.end(), entry.tag.begin());
            entry.tagLength = tag.size();
            entry.target = target;
            entry.hand = hand;
            entry.role = role;
            entry.priority = priority;
            entry.sequence = _sequence;
            entry.reserved = false;
            entry.valid = true;
            return true;
        }

        void cancelReservation(Entry& entry) noexcept
        {
            if (entry.reserved && !entry.valid) {
                entry = {};
            }
        }

        [[nodiscard]] bool invalidate(
            const std::string_view tag,
            const Hand hand) noexcept
        {
            if (auto* entry = find(tag, hand)) {
                *entry = {};
                return true;
            }
            return false;
        }

        [[nodiscard]] bool hasAny(const Hand hand) const noexcept
        {
            for (const auto& entry : _entries) {
                if (entry.valid && entry.hand == hand) {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] bool winner(
            const Hand hand,
            Entry& out) const noexcept
        {
            out = {};
            const Entry* selected = nullptr;
            for (const auto& entry : _entries) {
                if (!entry.valid || entry.hand != hand) {
                    continue;
                }
                if (!selected || entry.priority > selected->priority ||
                    (entry.priority == selected->priority &&
                        entry.sequence > selected->sequence)) {
                    selected = &entry;
                }
            }
            if (!selected) {
                return false;
            }
            out = *selected;
            return true;
        }

        template <class Visitor>
        void forEachActive(Visitor&& visitor) const
        {
            for (const auto& entry : _entries) {
                if (entry.valid) {
                    visitor(entry);
                }
            }
        }

        [[nodiscard]] std::size_t activeCount() const noexcept
        {
            std::size_t count = 0;
            for (const auto& entry : _entries) {
                count += entry.valid ? 1u : 0u;
            }
            return count;
        }

        void reset() noexcept
        {
            _entries = {};
            _sequence = 0;
        }

    private:
        [[nodiscard]] static std::string_view tagView(
            const Entry& entry) noexcept
        {
            return std::string_view(entry.tag.data(), entry.tagLength);
        }

        std::array<Entry, Capacity> _entries{};
        std::uint64_t _sequence = 0;
    };
}
