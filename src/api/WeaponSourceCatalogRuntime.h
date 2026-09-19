#pragma once
#include "WeaponSourceCatalog.h"
#include "RE/NetImmerse/NiAVObject.h"
#include <array>
#include <atomic>

namespace rock::provider {
    // Game-thread catalog. Pins prevent address reuse from inheriting an old key;
    // removed nodes are released on refresh, and shutdown releases the catalog.
    struct WeaponSourceCatalog {
        struct Entry { RE::NiPointer<RE::NiAVObject> node; WeaponSourceRecord value{}; };
        static constexpr std::size_t Capacity=4096;
        inline static std::atomic<std::uint64_t> nextKey{1};
        std::array<Entry,Capacity> entries{}; // Sorted by node address.
        std::uint32_t count{};
        std::uint64_t generation{};
        RE::NiAVObject* root{}; // Pinned by its catalog entry.
        bool overflow{};
        void clear() {
            for (std::uint32_t i=0;i<count;++i) entries[i]={};
            count=0; generation=0; root=nullptr; overflow=false;
        }
    };
}
