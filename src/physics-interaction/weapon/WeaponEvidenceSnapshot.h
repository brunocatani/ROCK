#pragma once

#include "physics-interaction/weapon/WeaponTypes.h"

#include <memory>
#include <span>
#include <utility>
#include <vector>

namespace rock
{
    // Owns the immutable published records, including their strings and points.
    // A borrowed record is valid only while this snapshot remains alive. Native
    // addresses in a record still require the caller's thread/generation checks.
    class WeaponEvidenceSnapshot
    {
    public:
        using Records = std::vector<WeaponCollisionProfileEvidenceDescriptor>;

        WeaponEvidenceSnapshot() = default;
        explicit WeaponEvidenceSnapshot(std::shared_ptr<const Records> records) : _records(std::move(records)) {}

        [[nodiscard]] std::span<const WeaponCollisionProfileEvidenceDescriptor> records() const
        {
            return _records ? std::span<const WeaponCollisionProfileEvidenceDescriptor>{ *_records } :
                              std::span<const WeaponCollisionProfileEvidenceDescriptor>{};
        }
        [[nodiscard]] auto begin() const { return records().begin(); }
        [[nodiscard]] auto end() const { return records().end(); }
        [[nodiscard]] auto size() const { return records().size(); }
        [[nodiscard]] const auto& operator[](std::size_t index) const { return records()[index]; }
        [[nodiscard]] const WeaponCollisionProfileEvidenceDescriptor* find(std::uint32_t bodyId) const
        {
            if (bodyId == 0x7FFF'FFFF) return nullptr;
            for (const auto& record : records())
                if (record.valid && record.bodyId == bodyId) return &record;
            return nullptr;
        }

    private:
        std::shared_ptr<const Records> _records;
    };
}
