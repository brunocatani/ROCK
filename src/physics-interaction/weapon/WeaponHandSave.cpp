#include "physics-interaction/weapon/WeaponHandSave.h"

#include "physics-interaction/weapon/KnifeReverseGrip.h"
#include "physics-interaction/weapon/NativeHandEquipment.h"
#include "physics-interaction/PhysicsLog.h"

#include <F4SE/API.h>
#include <F4SE/Interfaces.h>

#include <Windows.h>
#include <cstdint>

namespace rock::weapon_hand_save
{
    namespace
    {
        constexpr std::uint32_t kUniqueId = 0x524F434B;     // 'ROCK'
        constexpr std::uint32_t kRecordType = 0x484E4453;   // 'HNDS'
        constexpr std::uint32_t kRecordVersion = 1;

        enum Flags : std::uint32_t
        {
            kPrimaryLeft = 1u << 0,
            kPrimaryFiring = 1u << 1,
            kPrimarySupported = 1u << 2,
            kSecondaryLeft = 1u << 3,
            kRightGripReversed = 1u << 4,
            kLeftGripReversed = 1u << 5,
        };

        // Version 1 layout; a change needs a new version.
        struct Record
        {
            std::uint32_t primaryForm;
            std::uint32_t secondaryForm;
            std::uint32_t gripForm[2]; // right, left
            std::uint32_t flags;
        };
        static_assert(sizeof(Record) == 20);

        // No exception crosses the F4SE callback boundary.
        template <class Body>
        void guarded(const char* what, Body&& body) noexcept
        {
            try {
                body();
            } catch (...) {
                OutputDebugStringA(what);
            }
        }

        void saveRecord(const F4SE::SerializationInterface* serialization)
        {
            const auto hands = native_hand_equipment::savedHands();
            const auto right = knife_reverse_grip::savedGrip(false);
            const auto left = knife_reverse_grip::savedGrip(true);
            std::uint32_t flags = 0;
            if (hands.primaryLeft) flags |= kPrimaryLeft;
            if (hands.primaryFiring) flags |= kPrimaryFiring;
            if (hands.primarySupported) flags |= kPrimarySupported;
            if (hands.secondaryLeft) flags |= kSecondaryLeft;
            if (right.reversed) flags |= kRightGripReversed;
            if (left.reversed) flags |= kLeftGripReversed;
            const Record record{ hands.primaryForm, hands.secondaryForm, { right.formID, left.formID }, flags };
            if (!serialization->WriteRecord(kRecordType, kRecordVersion, &record, sizeof(record))) {
                ROCK_LOG_WARN(Weapon, "Saved weapon hands not written: the co-save refused the record");
            }
        }

        void clear()
        {
            native_hand_equipment::restoreHands({});
            knife_reverse_grip::restoreGrip(false, {});
            knife_reverse_grip::restoreGrip(true, {});
        }

        void onSave(const F4SE::SerializationInterface* serialization)
        {
            guarded("ROCK: saving weapon hands failed\n", [&] { saveRecord(serialization); });
        }

        void onRevert(const F4SE::SerializationInterface*)
        {
            guarded("ROCK: clearing saved weapon hands failed\n", [] { clear(); });
        }

        // A form of a plugin no longer loaded resolves to nothing.
        std::uint32_t resolve(const F4SE::SerializationInterface* serialization, const std::uint32_t formID)
        {
            if (!formID) return 0;
            if (const auto resolved = serialization->ResolveFormID(formID)) return *resolved;
            ROCK_LOG_INFO(Weapon, "Saved weapon hands: form {:08X} no longer resolves", formID);
            return 0;
        }

        void loadRecords(const F4SE::SerializationInterface* serialization)
        {
            std::uint32_t type = 0, version = 0, length = 0;
            while (serialization->GetNextRecordInfo(type, version, length)) {
                if (type != kRecordType) continue;
                Record record{};
                if (version != kRecordVersion || length != sizeof(record) ||
                    serialization->ReadRecordData(&record, sizeof(record)) != sizeof(record)) {
                    ROCK_LOG_WARN(Weapon, "Saved weapon hands unreadable version={} length={}", version, length);
                    continue;
                }
                native_hand_equipment::restoreHands({
                    .primaryForm = resolve(serialization, record.primaryForm),
                    .primaryLeft = (record.flags & kPrimaryLeft) != 0,
                    .primaryFiring = (record.flags & kPrimaryFiring) != 0,
                    .primarySupported = (record.flags & kPrimarySupported) != 0,
                    .secondaryForm = resolve(serialization, record.secondaryForm),
                    .secondaryLeft = (record.flags & kSecondaryLeft) != 0,
                });
                knife_reverse_grip::restoreGrip(false, { resolve(serialization, record.gripForm[0]), (record.flags & kRightGripReversed) != 0 });
                knife_reverse_grip::restoreGrip(true, { resolve(serialization, record.gripForm[1]), (record.flags & kLeftGripReversed) != 0 });
            }
        }
    }

    namespace
    {
        void onLoad(const F4SE::SerializationInterface* serialization)
        {
            guarded("ROCK: loading weapon hands failed\n", [&] { loadRecords(serialization); });
        }
    }

    bool install() noexcept
    {
        const auto* serialization = F4SE::GetSerializationInterface();
        if (!serialization) return false;
        serialization->SetUniqueID(kUniqueId);
        serialization->SetSaveCallback(onSave);
        serialization->SetLoadCallback(onLoad);
        serialization->SetRevertCallback(onRevert);
        return true;
    }
}
